/*
 * fec.c -- forward error correction based on Vandermonde matrices
 * 980624
 * (C) 1997-98 Luigi Rizzo (luigi@iet.unipi.it)
 *
 * Portions derived from code by Phil Karn (karn@ka9q.ampr.org),
 * Robert Morelos-Zaragoza (robert@spectra.eng.hawaii.edu) and Hari
 * Thirumoorthy (harit@spectra.eng.hawaii.edu), Aug 1995
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

/*
 * The following parameter defines how many bits are used for
 * field elements. The code supports any value from 2 to 16
 * but fastest operation is achieved with 8 bit elements
 * This is the only parameter you may want to change.
 */
#include "common.h"
#include <stdalign.h>

#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__)) && (GF_BITS == 8)
#include <immintrin.h>
#define TETRYS_HAVE_X86_SIMD 1
#endif

#define DEB(x)
#define DDB(x)
#define TICK(x)
#define TOCK(x)

#define INVERT_STACK_LIMIT 64

/*
 * Primitive polynomials - see Lin & Costello, Appendix A,
 * and  Lee & Messerschmitt, p. 453.
 */
static const char *const allPp[] = {    /* GF_BITS	polynomial		*/
    NULL,		    /*  0	no code			*/
    NULL,		    /*  1	no code			*/
    "111",		    /*  2	1+x+x^2			*/
    "1101",		    /*  3	1+x+x^3			*/
    "11001",		    /*  4	1+x+x^4			*/
    "101001",		    /*  5	1+x^2+x^5		*/
    "1100001",		    /*  6	1+x+x^6			*/
    "10010001",		    /*  7	1 + x^3 + x^7		*/
    "101110001",	    /*  8	1+x^2+x^3+x^4+x^8	*/
    "1000100001",	    /*  9	1+x^4+x^9		*/
    "10010000001",	    /* 10	1+x^3+x^10		*/
    "101000000001",	    /* 11	1+x^2+x^11		*/
    "1100101000001",	    /* 12	1+x+x^4+x^6+x^12	*/
    "11011000000001",	    /* 13	1+x+x^3+x^4+x^13	*/
    "110000100010001",	    /* 14	1+x+x^6+x^10+x^14	*/
    "1100000000000001",	    /* 15	1+x+x^15		*/
    "11010000000010001"	    /* 16	1+x+x^3+x^12+x^16	*/
};


/*
 * To speed up computations, we have tables for logarithm, exponent
 * and inverse of a number. If GF_BITS <= 8, we use a table for
 * multiplication as well (and 4-bit split tables for SIMD pshufb),
 * otherwise we use a table of logarithms.
 */

static gf gf_exp[2 * GF_SIZE];	/* index->poly form conversion table	*/
static int gf_log[GF_SIZE + 1];	/* Poly->index form conversion table	*/
static gf inverse[GF_SIZE + 1];	/* inverse of field elem.		*/
				/* inv[\alpha**i]=\alpha**(GF_SIZE-i-1)	*/

/**
 * Reduce @p x modulo GF_SIZE (2^GF_BITS - 1) without a division.
 * @param x  Value to reduce.
 * @return Equivalent element in [0, GF_SIZE].
 */
static inline gf
modnn(int x)
{
    while (x >= GF_SIZE) {
	x -= GF_SIZE;
	x = (x >> GF_BITS) + (x & GF_SIZE);
    }
    return (gf) x;
}

#define SWAP(a, b, t) do { t tmp = (a); (a) = (b); (b) = tmp; } while (0)

#if (GF_BITS <= 8)
static gf gf_mul_table[GF_SIZE + 1][GF_SIZE + 1];
#if (GF_BITS == 8)
/* Split 4-bit multiplication tables (8 KiB total, fits in L1d) for pshufb SIMD. */
alignas(16) static gf gf_mul_lo[256][16];
alignas(16) static gf gf_mul_hi[256][16];
#endif

#define gf_mul(x, y) gf_mul_table[x][y]

#define USE_GF_MULC const gf *__gf_mulc_
#define GF_MULC0(c) __gf_mulc_ = gf_mul_table[c]
#define GF_ADDMULC(dst, x) dst ^= __gf_mulc_[x]

/**
 * Fill the multiplication tables of GF(2^GF_BITS) (GF_BITS <= 8), including
 * the 4-bit split tables used by the SSSE3/AVX2 pshufb kernels when GF_BITS == 8.
 */
static void
init_mul_table(void)
{
    int i, j;
    for (i = 0; i < GF_SIZE + 1; i++)
	for (j = 0; j < GF_SIZE + 1; j++)
	    gf_mul_table[i][j] = gf_exp[modnn(gf_log[i] + gf_log[j])];

    for (j = 0; j < GF_SIZE + 1; j++)
	gf_mul_table[0][j] = gf_mul_table[j][0] = 0;

#if (GF_BITS == 8)
    for (i = 0; i < 256; i++) {
	for (j = 0; j < 16; j++) {
	    gf_mul_lo[i][j] = gf_mul_table[i][j];
	    gf_mul_hi[i][j] = gf_mul_table[i][j << 4];
	}
    }
#endif
}
#else	/* GF_BITS > 8 */
/**
 * Multiply two field elements with the exp/log tables (GF_BITS > 8; for
 * GF_BITS <= 8 gf_mul() is a macro using the multiplication table).
 * @param x  First operand.
 * @param y  Second operand.
 * @return Product x * y in GF(2^GF_BITS).
 */
static inline gf
gf_mul(gf x, gf y)
{
    if (x == 0 || y == 0)
	return 0;

    return gf_exp[gf_log[x] + gf_log[y]];
}
#define init_mul_table()

#define USE_GF_MULC const gf *__gf_mulc_
#define GF_MULC0(c) __gf_mulc_ = &gf_exp[gf_log[c]]
#define GF_ADDMULC(dst, x) do { if (x) dst ^= __gf_mulc_[gf_log[x]]; } while (0)
#endif

#define NEW_GF_MATRIX(rows, cols) \
    ((gf *)my_malloc((size_t)(rows) * (size_t)(cols) * sizeof(gf), "NEW_GF_MATRIX"))

/**
 * Generate the exp, log and inverse tables of GF(2^GF_BITS).
 */
static void
generate_gf(void)
{
    int i;
    gf mask;
    const char *Pp = allPp[GF_BITS];

    mask = 1;	/* x ** 0 = 1 */
    gf_exp[GF_BITS] = 0; /* will be updated at the end of the 1st loop */
    for (i = 0; i < GF_BITS; i++, mask <<= 1) {
	gf_exp[i] = mask;
	gf_log[gf_exp[i]] = i;
	if (Pp[i] == '1')
	    gf_exp[GF_BITS] ^= mask;
    }
    gf_log[gf_exp[GF_BITS]] = GF_BITS;
    mask = (gf) (1U << (GF_BITS - 1));
    for (i = GF_BITS + 1; i < GF_SIZE; i++) {
	if (gf_exp[i - 1] >= mask)
	    gf_exp[i] = (gf) (gf_exp[GF_BITS] ^ ((gf_exp[i - 1] ^ mask) << 1));
	else
	    gf_exp[i] = (gf) (gf_exp[i - 1] << 1);
	gf_log[gf_exp[i]] = i;
    }

    gf_log[0] = GF_SIZE;
    for (i = 0; i < GF_SIZE; i++)
	gf_exp[i + GF_SIZE] = gf_exp[i];

    inverse[0] = 0;
    inverse[1] = 1;
    for (i = 2; i <= GF_SIZE; i++)
	inverse[i] = gf_exp[GF_SIZE - gf_log[i]];
}

/*
 * Portable scalar addmul1() kernel (also handles the c == 1 XOR fast-path
 * and any tail bytes left by SIMD kernels).
 */
#define UNROLL 16

/**
 * Scalar fallback kernel computing dst1[0..sz-1] ^= c * src1[0..sz-1] in GF(2^GF_BITS).
 * @param dst1  Destination vector (updated in place, non-overlapping with @p src1).
 * @param src1  Source vector.
 * @param c     Scalar field multiplier (non-zero).
 * @param sz    Number of field elements.
 */
static void
addmul1_scalar(gf *restrict dst1, const gf *restrict src1, gf c, int sz)
{
    if (sz <= 0)
	return;

    if (c == 1) {
	for (int i = 0; i < sz; i++)
	    dst1[i] ^= src1[i];
	return;
    }

    USE_GF_MULC;
    gf *dst = dst1;
    const gf *src = src1;
    const gf *end = dst + sz;
    const gf *lim = (sz >= UNROLL) ? (end - UNROLL + 1) : dst;
    GF_MULC0(c);
#if (UNROLL > 1)
    for (; dst < lim; dst += UNROLL, src += UNROLL) {
	GF_ADDMULC(dst[0], src[0]);
	GF_ADDMULC(dst[1], src[1]);
	GF_ADDMULC(dst[2], src[2]);
	GF_ADDMULC(dst[3], src[3]);
#if (UNROLL > 4)
	GF_ADDMULC(dst[4], src[4]);
	GF_ADDMULC(dst[5], src[5]);
	GF_ADDMULC(dst[6], src[6]);
	GF_ADDMULC(dst[7], src[7]);
#endif
#if (UNROLL > 8)
	GF_ADDMULC(dst[8], src[8]);
	GF_ADDMULC(dst[9], src[9]);
	GF_ADDMULC(dst[10], src[10]);
	GF_ADDMULC(dst[11], src[11]);
	GF_ADDMULC(dst[12], src[12]);
	GF_ADDMULC(dst[13], src[13]);
	GF_ADDMULC(dst[14], src[14]);
	GF_ADDMULC(dst[15], src[15]);
#endif
    }
#else
    (void) lim;
#endif
    for (; dst < end; dst++, src++)
	GF_ADDMULC(*dst, *src);
}

#ifdef TETRYS_HAVE_X86_SIMD
/**
 * SSSE3 pshufb kernel computing dst[0..sz-1] ^= c * src[0..sz-1] in GF(2^8),
 * 16 bytes per iteration using the 4-bit split tables gf_mul_lo / gf_mul_hi.
 */
__attribute__((target("ssse3")))
static void
addmul1_ssse3(gf *restrict dst, const gf *restrict src, gf c, int sz)
{
    if (sz <= 0)
	return;

    int i = 0;
    if (c == 1) {
	for (; i + 16 <= sz; i += 16) {
	    __m128i s = _mm_loadu_si128((const __m128i *) (src + i));
	    __m128i d = _mm_loadu_si128((const __m128i *) (dst + i));
	    _mm_storeu_si128((__m128i *) (dst + i), _mm_xor_si128(d, s));
	}
	for (; i < sz; i++)
	    dst[i] ^= src[i];
	return;
    }

    const __m128i lo_tbl = _mm_load_si128((const __m128i *) gf_mul_lo[c]);
    const __m128i hi_tbl = _mm_load_si128((const __m128i *) gf_mul_hi[c]);
    const __m128i mask = _mm_set1_epi8(0x0f);

    for (; i + 16 <= sz; i += 16) {
	__m128i v = _mm_loadu_si128((const __m128i *) (src + i));
	__m128i lo = _mm_and_si128(v, mask);
	__m128i hi = _mm_and_si128(_mm_srli_epi16(v, 4), mask);
	__m128i prod = _mm_xor_si128(_mm_shuffle_epi8(lo_tbl, lo),
				     _mm_shuffle_epi8(hi_tbl, hi));
	__m128i d = _mm_loadu_si128((const __m128i *) (dst + i));
	_mm_storeu_si128((__m128i *) (dst + i), _mm_xor_si128(d, prod));
    }
    if (i < sz)
	addmul1_scalar(dst + i, src + i, c, sz - i);
}

/**
 * AVX2 pshufb kernel computing dst[0..sz-1] ^= c * src[0..sz-1] in GF(2^8),
 * 32 bytes per iteration with SSSE3/scalar tail handling.
 */
__attribute__((target("avx2")))
static void
addmul1_avx2(gf *restrict dst, const gf *restrict src, gf c, int sz)
{
    if (sz <= 0)
	return;

    int i = 0;
    if (c == 1) {
	for (; i + 32 <= sz; i += 32) {
	    __m256i s = _mm256_loadu_si256((const __m256i *) (src + i));
	    __m256i d = _mm256_loadu_si256((const __m256i *) (dst + i));
	    _mm256_storeu_si256((__m256i *) (dst + i), _mm256_xor_si256(d, s));
	}
	if (i < sz)
	    addmul1_ssse3(dst + i, src + i, c, sz - i);
	return;
    }

    const __m256i lo_tbl =
	_mm256_broadcastsi128_si256(_mm_load_si128((const __m128i *) gf_mul_lo[c]));
    const __m256i hi_tbl =
	_mm256_broadcastsi128_si256(_mm_load_si128((const __m128i *) gf_mul_hi[c]));
    const __m256i mask = _mm256_set1_epi8(0x0f);

    for (; i + 32 <= sz; i += 32) {
	__m256i v = _mm256_loadu_si256((const __m256i *) (src + i));
	__m256i lo = _mm256_and_si256(v, mask);
	__m256i hi = _mm256_and_si256(_mm256_srli_epi16(v, 4), mask);
	__m256i prod = _mm256_xor_si256(_mm256_shuffle_epi8(lo_tbl, lo),
					_mm256_shuffle_epi8(hi_tbl, hi));
	__m256i d = _mm256_loadu_si256((const __m256i *) (dst + i));
	_mm256_storeu_si256((__m256i *) (dst + i), _mm256_xor_si256(d, prod));
    }
    if (i < sz)
	addmul1_ssse3(dst + i, src + i, c, sz - i);
}
#endif /* TETRYS_HAVE_X86_SIMD */

static void (*addmul1_impl)(gf *restrict, const gf *restrict, gf, int) = addmul1_scalar;

#define addmul(dst, src, c, sz) \
    do { if ((c) != 0) addmul1_impl((dst), (src), (c), (sz)); } while (0)

DEB(int pivloops = 0; int pivswaps = 0;)

void
init_fec(void)
{
    TICK(ticks[0]);
    generate_gf();
    TOCK(ticks[0]);
    DDB(fprintf(stderr, "generate_gf took %ldus\n", ticks[0]);)
    TICK(ticks[0]);
    init_mul_table();
    TOCK(ticks[0]);
    DDB(fprintf(stderr, "init_mul_table took %ldus\n", ticks[0]);)

#ifdef TETRYS_HAVE_X86_SIMD
    __builtin_cpu_init();
    if (__builtin_cpu_supports("avx2"))
	addmul1_impl = addmul1_avx2;
    else if (__builtin_cpu_supports("ssse3"))
	addmul1_impl = addmul1_ssse3;
    else
	addmul1_impl = addmul1_scalar;
#else
    addmul1_impl = addmul1_scalar;
#endif
}


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
/////////////// End of Galois field implementation (from Rizzo)
/////////////// Beginning of Tetrys operation
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


/**
 * Search the pivot used to zero column @p col of the k x k matrix @p src.
 * The diagonal element is tried first, then every unused row / column.
 * @param src    The matrix (k x k).
 * @param ipiv   Pivot usage markers, one per column.
 * @param k      Matrix size.
 * @param col    Column being zeroed.
 * @param[out] irow  Row of the pivot.
 * @param[out] icol  Column of the pivot.
 * @return 0 if a pivot was found, -1 if the matrix is singular, or a value
 *         > 0 (index + 1 of the first column linearly dependent on the
 *         others) when no pivot exists.
 */
static int
find_pivot(const gf *src, const int *ipiv, int k, int col, int *irow, int *icol)
{
    int row, ix, i;
#ifdef VERBOSE
    int j;
#endif

    *irow = *icol = -1;
    if (ipiv[col] != 1 && src[col * k + col] != 0) {
	*irow = col;
	*icol = col;
	return 0;
    }
    for (row = 0; row < k; row++) {
	if (ipiv[row] == 1)
	    continue;
	for (ix = 0; ix < k; ix++) {
	    DEB(pivloops++;)
	    if (ipiv[ix] == 0) {
		if (src[row * k + ix] != 0) {
		    *irow = row;
		    *icol = ix;
		    return 0;
		}
	    } else if (ipiv[ix] > 1) {
		fprintf(stderr, "fec: singular decoding matrix (pivot already used): "
			"these repair packets cannot rebuild the lost packets\n");
		return -1;
	    }
	}
    }
    /* Find the first column which is linearly dependent on the others. */
    i = 0;
    while ((i < k) && (ipiv[i] != 0))
	i++;
#ifdef VERBOSE
    fprintf(stderr, "fec: decoding not possible yet: repair packet #%d is linearly "
	    "dependent on the others (no pivot in column %d). It will be dropped "
	    "and decoding retried with the next repair packets. Pivot flags: ",
	    i + 1, col);
    for (j = 0; j < k; j++)
	fprintf(stderr, "%d ", ipiv[j]);
    fprintf(stderr, "\n");
#endif
    return i + 1; /* 0 corresponds to a successful decoding */
}

/**
 * From all rows of @p src, remove the multiples of @p pivot_row required to
 * zero the entry of column @p icol (skipped when the pivot row is already
 * the identity row held in @p id_row).
 * @param src        The matrix (k x k), modified in place.
 * @param pivot_row  Row used as pivot (a row of @p src).
 * @param id_row     Scratch row of k elements, all zero on entry and exit.
 * @param k          Matrix size.
 * @param icol       Pivot column.
 */
static void
eliminate_column(gf *src, const gf *pivot_row, gf *id_row, int k, int icol)
{
    id_row[icol] = 1;
    if (memcmp(pivot_row, id_row, (size_t) k * sizeof(gf)) != 0) {
	gf *p = src;
	for (int ix = 0; ix < k; ix++, p += k) {
	    if (ix != icol) {
		gf c = p[icol];
		p[icol] = 0;
		if (c != 0) {
		    /* pivot_row == &src[icol * k] is strictly disjoint from row ix != icol. */
		    addmul1_impl(p, pivot_row, c, k);
		}
	    }
	}
    }
    id_row[icol] = 0;
}

/**
 * Invert the k x k matrix @p src in place using Gauss-Jordan elimination over GF(2^8).
 * Uses stack scratch arrays for k <= INVERT_STACK_LIMIT (64) to avoid heap traffic.
 * @param src  Square matrix (k x k), inverted in place.
 * @param k    Matrix dimension.
 * @return 0 on success, -1 on error, or index + 1 of a linearly dependent column.
 */
static int
invert_mat(gf *src, int k)
{
    if (k <= 0)
	return -1;

    int indxc_s[INVERT_STACK_LIMIT];
    int indxr_s[INVERT_STACK_LIMIT];
    int ipiv_s[INVERT_STACK_LIMIT];
    gf id_row_s[INVERT_STACK_LIMIT];

    int *indxc = indxc_s;
    int *indxr = indxr_s;
    int *ipiv = ipiv_s;
    gf *id_row = id_row_s;

    if (k > INVERT_STACK_LIMIT) {
	indxc = my_malloc((size_t) k * sizeof(int), "indxc");
	indxr = my_malloc((size_t) k * sizeof(int), "indxr");
	ipiv = my_malloc((size_t) k * sizeof(int), "ipiv");
	id_row = NEW_GF_MATRIX(1, k);
    }

    memset(id_row, 0, (size_t) k * sizeof(gf));
    DEB(pivloops = 0; pivswaps = 0;)
    for (int i = 0; i < k; i++)
	ipiv[i] = 0;

    int error = 1;
    for (int col = 0; col < k; col++) {
	int irow, icol;
	int rc = find_pivot(src, ipiv, k, col, &irow, &icol);
	if (rc != 0) {
	    if (rc > 0)
		error = rc;
	    goto fail;
	}
	++(ipiv[icol]);
	if (irow != icol) {
	    for (int ix = 0; ix < k; ix++) {
		SWAP(src[irow * k + ix], src[icol * k + ix], gf);
	    }
	}
	indxr[col] = irow;
	indxc[col] = icol;
	gf *pivot_row = &src[icol * k];
	gf c = pivot_row[icol];
	if (c == 0) {
	    fprintf(stderr, "fec: null pivot after row exchange: decoding matrix is not invertible\n");
	    goto fail;
	}
	if (c != 1) {
	    DEB(pivswaps++;)
	    c = inverse[c];
	    pivot_row[icol] = 1;
	    for (int ix = 0; ix < k; ix++)
		pivot_row[ix] = gf_mul(c, pivot_row[ix]);
	}
	eliminate_column(src, pivot_row, id_row, k, icol);
    }
    for (int col = k - 1; col >= 0; col--) {
	if (indxr[col] < 0 || indxr[col] >= k)
	    fprintf(stderr, "fec: internal error: row permutation index %d out of range (matrix %dx%d)\n", indxr[col], k, k);
	else if (indxc[col] < 0 || indxc[col] >= k)
	    fprintf(stderr, "fec: internal error: column permutation index %d out of range (matrix %dx%d)\n", indxc[col], k, k);
	else if (indxr[col] != indxc[col]) {
	    for (int row = 0; row < k; row++) {
		SWAP(src[row * k + indxr[col]], src[row * k + indxc[col]], gf);
	    }
	}
    }
    error = 0;
fail:
    if (k > INVERT_STACK_LIMIT) {
	free(indxc);
	free(indxr);
	free(ipiv);
	free(id_row);
    }
    return error;
}

/**
 * Multiply the inverted decoding matrix @p coef_matrix (nb_row x nb_col) by
 * the repair symbols in @p repairQueue to reconstruct each lost source packet
 * into @p recover_vector (computing X = R * G^{-1}).
 * @param coef_matrix     Inverted decoding matrix (nb_row x nb_col, column-major per repair).
 * @param repairQueue     Queue of repair symbols (RRBuffer or tmpFecRep).
 * @param recover_vector  Output array of @p nb_row buffers (each PLS bytes).
 * @param nb_row          Number of lost packets to recover.
 * @param nb_col          Number of repair packets to combine.
 */
static void gf_mul_matrix_repairs(const gf *coef_matrix,
				  const struct circBUFFER *repairQueue,
				  gf **recover_vector, int nb_row, int nb_col)
{
    for (int row = 0; row < nb_row; row++) {
	memset(recover_vector[row], 0, PLS);
	const struct circBUFFERItem *item = repairQueue->firstItem;
	int col = 0;
	while (item != NULL && col < nb_col) {
	    const struct PDU *currentRepair = item->pkt;
	    gf tmp_coef = coef_matrix[col * nb_col + row];
	    if ((col * nb_col + row) >= (nb_row * nb_row)) {
		printf("\n void gf_mul_matrix_repairs : PROBLEM\n");
	    }
	    addmul(recover_vector[row], (const gf *) (&(currentRepair)->datav), tmp_coef, PLS);
	    col++;
	    item = item->next;
	}
    }
}

/**
 * Draw the next coding coefficient from the already-seeded rand31pmc PRNG.
 * Must match the per-step draw loop of the legacy get_coef() bit-for-bit.
 */
static inline gf next_coef(void)
{
    gf elt = (gf) (rand31pmc_next() % (GF_SIZE + 1));
#ifdef BINAIRE
    elt = (gf) (elt % 2);
#endif
#ifndef ALLOWZEROCOEF
    while (elt == 0) {
	elt = (gf) (rand31pmc_next() % (GF_SIZE + 1));
    }
#endif
    return elt;
}

int tet_encode(const struct circBUFFER *metaBuffer, gf *fec, int sz, int set_seed,
	       unsigned long int min, unsigned long int max)
{
    if (sz <= 0)
	return -1;
    memset(fec, 0, (size_t) sz);

    /*
     * Seed once and walk the sorted SEBuffer in parallel with i in [min, max]:
     * O(W) PRNG steps and O(W) pointer steps instead of O(W^2).
     */
    rand31pmc_seedi((unsigned long int) set_seed);
    const struct circBUFFERItem *item = metaBuffer->firstItem;
    while (item != NULL && item->seqnum < min)
	item = item->next;

    for (unsigned long int i = min; i <= max; i++) {
	gf elt = next_coef();
#ifndef TET_TESTGF
	while (item != NULL && item->seqnum < i)
	    item = item->next;
	if (item != NULL && item->seqnum == i) {
	    const struct PDU *src = item->pkt;
	    addmul(fec, (const gf *) (&src->datav), elt, sz);
	    if (MONITOR) {
		fprintf(tetrys->monitorFecSnd,
			"\nFEC_ENCODE min %lu max %lu ENCODE pkt %lu with coef %d repair_id %d - 3 bytes: %c %c %c - Data %c %c %c",
			min, max, i, elt, set_seed, fec[0], fec[1], fec[2],
			src->datav[0], src->datav[1], src->datav[2]);
		fflush(tetrys->monitorFecSnd);
	    }
	    item = item->next;
	}
#else
	(void) item;
	(void) elt;
#endif
    }

    return 0;
}

int mul_mat_by_repair(const struct circBUFFER *repairQueue, gf **lost_to_recover,
		      const gf *matrix, int nb_lost)
{
#ifndef TET_TESTGF
    gf_mul_matrix_repairs(matrix, repairQueue, lost_to_recover, nb_lost, nb_lost);
#else
    (void) repairQueue;
    (void) lost_to_recover;
    (void) matrix;
    (void) nb_lost;
#endif
    return 0;
}

/**
 * Subtract all received data packets in @p dataQueue from the repair symbol @p repair.
 * Each data packet is subtracted at most once, tracked via @c repair->subtracted_vect.
 * Walks the sorted @p dataQueue while advancing the PRNG monotonically in O(W).
 * @param repair     Repair item from RRBuffer or tmpFecRep.
 * @param dataQueue  Sorted buffer holding known data packets (RDBuffer or tmpFecDat).
 * @return 0 on success.
 */
static int subtract_dbuff_from_r(struct circBUFFERItem *repair,
				 const struct circBUFFER *dataQueue)
{
#ifdef TET_TESTGF
    const struct PDU *currentRepair = repair->pkt;
#else
    struct PDU *currentRepair = repair->pkt;
#endif
    if (PR_DB_2) {
	fprintf(stderr, "\nsubtract_dbuff_from_r: BEFORE Rep %lu min %lu max %lu",
		currentRepair->num, currentRepair->min_redundancy,
		currentRepair->max_redundancy);
	print_bitvector(repair);
    }
    unsigned long int repair_id = currentRepair->repair_id;
    unsigned long int mini = currentRepair->min_redundancy;
    unsigned long int maxi = currentRepair->max_redundancy;
    unsigned long int num = 0;
    int currentIndex = 0;

    /* Lazy PRNG seeding + monotonic cursor along sorted dataQueue: O(W). */
    int prng_seeded = 0;
    unsigned long int prng_seq = mini;
    gf prng_elt = 0;

    const struct circBUFFERItem *item = dataQueue->firstItem;
    while (item != NULL) {
	const struct PDU *currentData = item->pkt;
	num = currentData->num;
	if (num > maxi)
	    break;
	if (num >= mini) {
	    currentIndex = (int) (num - mini);
	    if (vect_getbit((const gf *) (&repair->subtracted_vect), currentIndex) == 0) {
		if (!prng_seeded) {
		    rand31pmc_seedi(repair_id);
		    prng_seq = mini;
		    prng_seeded = 1;
		}
		while (prng_seq <= num) {
		    prng_elt = next_coef();
		    prng_seq++;
		}
		if (PR_DB_2)
		    fprintf(stderr, "\n\tsubtract_dbuff_from_r: sub dat %lu from rep %lu index %d",
			    num, repair_id, currentIndex);
		vect_setbit((gf *) (&repair->subtracted_vect), currentIndex, 1);

		if (MONITOR) {
		    fprintf(tetrys->monitorSub,
			    "\nBEFORE SUBSTRATION min %lu max %lu SUB pkt %lu with coef %d repair_id %lu -- 3 first bytes of rep: %c %c %c dat: %c %c %c size %d \n",
			    mini, maxi, num, prng_elt, repair_id,
			    currentRepair->datav[0], currentRepair->datav[1],
			    currentRepair->datav[2], currentData->datav[0],
			    currentData->datav[1], currentData->datav[2],
			    dataQueue->pktSize);
		    fflush(tetrys->monitorSub);
		}
#ifndef TET_TESTGF
		addmul((gf *) (&(currentRepair)->datav),
		       (const gf *) (&(currentData)->datav), prng_elt,
		       dataQueue->pktSize);
#else
		(void) prng_elt;
#endif
		if (MONITOR) {
		    fprintf(tetrys->monitorSub,
			    "\nAFTER SUBSTRATION min %lu max %lu SUB pkt %lu with coef %d repair_id %lu -- 3 first bytes %c %c %c size %d \n",
			    mini, maxi, num, prng_elt, repair_id,
			    currentData->datav[0], currentData->datav[1],
			    currentData->datav[2], dataQueue->pktSize);
		    fflush(tetrys->monitorSub);
		}
	    }
	}
	item = item->next;
    }
    if (vect_get_nb_zero(repair) == 0) {
	repair->toDiscard = 1;
    }
    if (PR_DB_2) {
	fprintf(stderr, "\nsubtract_dbuff_from_r: AFTER  num %lu Rep %lu curI %d",
		num, repair_id, currentIndex);
	print_bitvector(repair);
    }
    return 0;
}

void subtract_all(void)
{
    if (PR_DB_2)
	fprintf(stderr, "\nsubtract_all:");
    struct circBUFFERItem *currentRepair = tetrys->RRBuffer->firstItem;
    while (currentRepair != NULL) {
	/** For each R pkt, check if it is still useful. If not, mark for deletion */
	if ((currentRepair->pkt->max_redundancy <= tetrys->nextExpected)
	    || (currentRepair->pkt->max_redundancy <= tetrys->p_seq_num_of_last_refresh)) {
	    currentRepair->toDiscard = 1;
	}
	/** For each R pkt, check if it can still be subtracted (discard if expired unrecovered data remain) */
	if (currentRepair->toDiscard == 0 && tetrys->RDBuffer->nbItemStored > 0
	    && tetrys->RDBuffer->firstItem != NULL) {
	    unsigned long int first_rd = tetrys->RDBuffer->firstItem->pkt->num;
	    unsigned long int check_end = currentRepair->pkt->max_redundancy;
	    if (first_rd < check_end)
		check_end = first_rd;
	    int nblostforever = 0;
	    for (unsigned long int seq = currentRepair->pkt->min_redundancy;
		 seq < check_end; seq++) {
		int bit_idx = (int) (seq - currentRepair->pkt->min_redundancy);
		if (vect_getbit((const gf *) (&currentRepair->subtracted_vect), bit_idx) == 0) {
		    int lost = 0;
		    for (int j = 0; j < tetrys->nb_pkt_lost; j++) {
			if (tetrys->lost_positions[j] == seq)
			    lost += 1;
		    }
		    if (lost == 0) {
			nblostforever += 1;
			break;
		    }
		}
	    }
	    if (nblostforever > 0)
		currentRepair->toDiscard = 1;
	}
	if (currentRepair->toDiscard == 0)
	    subtract_dbuff_from_r(currentRepair, tetrys->RDBuffer);
	currentRepair = currentRepair->next;
    }
}

void subtract_all_fec(void)
{
    if (PR_DB_2)
	fprintf(stdout, "\nsubtract_all_fec:");
    struct circBUFFERItem *currentRepair = tetrys->tmpFecRep->firstItem;
    while (currentRepair != NULL) {
	subtract_dbuff_from_r(currentRepair, tetrys->tmpFecDat);
	currentRepair = currentRepair->next;
    }
}

int get_lost_positions(void)
{
    unsigned long int followSeqNum = tetrys->p_seq_num_of_last_refresh;
    unsigned long int newPseq = followSeqNum;
    for (int i = 0; i < tetrys->lost_positions_cap; i++)
	tetrys->lost_positions[i] = ULONG_MAX;
    int nbLosses = 0;
    int initok = circ_init_iterator(tetrys->RDBuffer);
    int consecutive = 1;
    if (PR_DB_1)
	fprintf(stderr, "\nget_lost_positions: %lu ", tetrys->p_seq_num_of_last_refresh);
    while ((initok == 1) && (1 == circ_get_next(tetrys->RDBuffer))) {
	if (followSeqNum == tetrys->RDBuffer->iterator->pkt->num) {
	    followSeqNum++;
	    if (PR_DB_1)
		fprintf(stderr, " rcnum %lu ", tetrys->RDBuffer->iterator->pkt->num);
	    if (consecutive == 1)
		newPseq = tetrys->RDBuffer->iterator->pkt->num;
	} else if (followSeqNum < tetrys->RDBuffer->iterator->pkt->num) {
	    consecutive = 0;
	    for (; followSeqNum < tetrys->RDBuffer->iterator->pkt->num
		 && nbLosses < tetrys->lost_positions_cap; followSeqNum++) {
		tetrys->lost_positions[nbLosses] = followSeqNum;
		if (PR_DB_1)
		    fprintf(stderr, "- lnum %lu pos %d -", followSeqNum, nbLosses);
		nbLosses++;
	    }
	    followSeqNum = tetrys->RDBuffer->iterator->pkt->num + 1;
	}
    }
    if (PR_DB_1)
	circ_print_buffer_stat(tetrys->RDBuffer);
    if (tetrys->RRBuffer->nbItemStored > 0 && tetrys->RRBuffer->lastItem != NULL) {
	for (; followSeqNum <= tetrys->RRBuffer->lastItem->pkt->max_redundancy
	     && nbLosses < tetrys->lost_positions_cap; followSeqNum++) {
	    tetrys->lost_positions[nbLosses] = followSeqNum;
	    if (PR_DB_1)
		fprintf(stderr, "\nget_lost_positions: Red %lu lastrefr %lu lostnum %lu pos %d",
			tetrys->RRBuffer->lastItem->pkt->max_redundancy,
			tetrys->p_seq_num_of_last_refresh, followSeqNum, nbLosses);
	    nbLosses++;
	}
    }
    if (PR_DB_1)
	fprintf(stderr, "\nget_lost_positions: Final lastrefr %lu lostnum %lu pos %d  newPsqNum %lu ",
		tetrys->p_seq_num_of_last_refresh, followSeqNum, nbLosses, newPseq);
    tetrys->nb_pkt_lost = nbLosses;
    tetrys->p_seq_num_of_last_refresh = newPseq;
    return tetrys->nb_pkt_lost;
}

int get_fec_lost_positions(void)
{
    for (int i = 0; i < tetrys->lost_positions_cap; i++)
	tetrys->lost_positions[i] = ULONG_MAX;
    if (tetrys->tmpFecRep->firstItem == NULL) {
	tetrys->nb_pkt_lost = 0;
	return 0;
    }
    unsigned long int followSeqNum = tetrys->tmpFecRep->firstItem->pkt->min_redundancy;
    int nbLosses = 0;
    int initok = circ_init_iterator(tetrys->tmpFecDat);
    if (PR_DB_1)
	fprintf(stderr, "\nget_fec_lost_positions:  ");
    while ((initok == 1) && (1 == circ_get_next(tetrys->tmpFecDat))) {
	if (followSeqNum == tetrys->tmpFecDat->iterator->pkt->num) {
	    followSeqNum++;
	    if (PR_DB_1)
		fprintf(stderr, " rcnum %lu ", tetrys->tmpFecDat->iterator->pkt->num);
	} else if (followSeqNum < tetrys->tmpFecDat->iterator->pkt->num) {
	    for (; followSeqNum < tetrys->tmpFecDat->iterator->pkt->num
		 && nbLosses < tetrys->lost_positions_cap; followSeqNum++) {
		tetrys->lost_positions[nbLosses] = followSeqNum;
		if (PR_DB_1)
		    fprintf(stderr, "- lnum %lu pos %d -", followSeqNum, nbLosses);
		nbLosses++;
	    }
	    followSeqNum = tetrys->tmpFecDat->iterator->pkt->num + 1;
	}
    }
    if (PR_DB_1)
	circ_print_buffer_stat(tetrys->RDBuffer);
    if (tetrys->tmpFecRep->nbItemStored > 0 && tetrys->tmpFecRep->lastItem != NULL) {
	for (; followSeqNum <= tetrys->tmpFecRep->lastItem->pkt->max_redundancy
	     && nbLosses < tetrys->lost_positions_cap; followSeqNum++) {
	    tetrys->lost_positions[nbLosses] = followSeqNum;
	    if (PR_DB_1)
		fprintf(stderr, "- red lnum %lu pos %d -", followSeqNum, nbLosses);
	    nbLosses++;
	}
    }
    if (PR_DB_1)
	fprintf(stderr, "\nget_fec_lost_positions: Final lostnum %lu pos %d ",
		followSeqNum, nbLosses);
    tetrys->nb_pkt_lost = nbLosses;
    return tetrys->nb_pkt_lost;
}

/**
 * Fill column @p col of the k x k matrix @p matrix with the GF coefficients
 * connecting @p repair to tetrys->lost_positions[0..k-1] in O(W).
 */
static void fill_matrix_col_for_repair(gf *matrix, int k, int col,
				       const struct PDU *repair)
{
    unsigned long int mini = repair->min_redundancy;
    unsigned long int maxi = repair->max_redundancy;
    int seeded = 0;
    unsigned long int prng_seq = mini;
    gf elt = 0;

    for (int u = 0; u < k; u++) {
	unsigned long int pos = tetrys->lost_positions[u];
	if (pos >= mini && pos <= maxi) {
	    if (!seeded || pos < prng_seq) {
		rand31pmc_seedi(repair->repair_id);
		prng_seq = mini;
		seeded = 1;
	    }
	    while (prng_seq <= pos) {
		elt = next_coef();
		prng_seq++;
	    }
	    matrix[k * u + col] = elt;
	    if (MONITOR) {
		fprintf(tetrys->monitorSub,
			"\nDECODING_TRY ASSOCIATE coef %d  TO PKT %lu for REPAIR PKT min %lu max %lu repair_id %lu\n",
			elt, pos, mini, maxi, repair->repair_id);
		fflush(tetrys->monitorSub);
	    }
	}
    }
}

/*
 * Build and invert the coding matrix G such that X * G = R; on success
 * matrix holds G^{-1} so that X = R * G^{-1} (see common.h for contract).
 */
int build_matrix_and_invert(gf *matrix)
{
    int k = (int) tetrys->max_to_decode;
    if (k <= 0)
	return -1;
    memset(matrix, 0, (size_t) k * (size_t) k * sizeof(gf));

    int nb_rep = 0;
    const struct circBUFFERItem *item = tetrys->RRBuffer->firstItem;
    if (PR_DB_1)
	fprintf(stderr, "\nbuild_matrix_and_invert: max_to_dec %lu ", tetrys->max_to_decode);
    while (item != NULL && nb_rep < k) {
	if (PR_DB_1)
	    print_bitvector(item);
	fill_matrix_col_for_repair(matrix, k, nb_rep, item->pkt);
	nb_rep++;
	item = item->next;
    }
    if (PR_DB_2)
	pr_matrix(matrix, k, k, "matrix");
    int rc = invert_mat(matrix, k);
    if (PR_DB_2)
	pr_matrix(matrix, k, k, "inverse matrix");
    return rc;
}

int test_build_matrix_and_invert(void)
{
    int nb_rep = 0;
    const struct circBUFFERItem *item = tetrys->RRBuffer->firstItem;
    fprintf(stderr, "\nbuild_matrix_and_invert: max_to_dec %lu ", tetrys->max_to_decode);
    while (item != NULL && ((unsigned long) nb_rep < tetrys->max_to_decode)) {
	const struct PDU *repair = item->pkt;
	unsigned long int mini = repair->min_redundancy;
	unsigned long int maxi = repair->max_redundancy;
	int repair_min_max = (int) (maxi - mini);
	print_bitvector(item);
	fprintf(stdout, "\nRep %lu from min %lu to %lu len %d : ",
		repair->num, mini, maxi, repair_min_max);
	for (int bit_i = 0; bit_i <= repair_min_max; bit_i++)
	    fprintf(stdout, " %d",
		    vect_getbit((const gf *) &item->subtracted_vect, bit_i));
	printf("\n");

	rand31pmc_seedi(repair->repair_id);
	unsigned long int prng_seq = mini;
	gf elt = 0;
	for (int bit_i = 0; bit_i <= repair_min_max; bit_i++) {
	    unsigned long int target = mini + (unsigned long int) bit_i;
	    while (prng_seq <= target) {
		elt = next_coef();
		prng_seq++;
	    }
	    for (unsigned long u = 0; u < tetrys->max_to_decode; u++) {
		if (tetrys->lost_positions[u] == (unsigned long) bit_i) {
		    fprintf(stdout, "b%d c%d ",
			    vect_getbit((const gf *) &item->subtracted_vect, bit_i),
			    elt);
		}
	    }
	}
	nb_rep++;
	item = item->next;
    }
    printf("\n\n\n");
    return 0;
}

int build_fec_matrix_and_invert(gf *matrix, int nbLInBlock)
{
    int k = nbLInBlock;
    if (k <= 0)
	return -1;
    memset(matrix, 0, (size_t) k * (size_t) k * sizeof(gf));

    int nb_rep = 0;
    const struct circBUFFERItem *item = tetrys->tmpFecRep->firstItem;
    if (PR_DB_1)
	fprintf(stderr, "\nbuild_fec_matrix_and_invert: max_to_dec %d ", nbLInBlock);
    while (item != NULL && nb_rep < k) {
	fill_matrix_col_for_repair(matrix, k, nb_rep, item->pkt);
	nb_rep++;
	item = item->next;
    }
    if (PR_DB_2)
	pr_matrix(matrix, k, k, "build_fec_matrix_and_invert: matrix");
    int rc = invert_mat(matrix, k);
    if (PR_DB_2)
	pr_matrix(matrix, k, k, "build_fec_matrix_and_invert: inverse matrix");
    return rc;
}
