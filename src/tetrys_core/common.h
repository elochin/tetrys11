/* GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2011 Jerome Lacan, Emmanuel Lochin, Pierre-Ugo Tournoux, 2011 ISAE
 *
 * This program is free software: you can redistribute it and/or modify it under the terms 
 * of the GNU General Public License as published by the Free Software Foundation, either version 
 * 3 of the License, or any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * This code has been developped at the ISAE (Institut Superieur de l'Aeronautique et de l'Espace) 
 * a french space and aeronautic institute based in Toulouse, France in the context of a DMIA 
 * project (Department of Mathematics, Computer Sciences and Control Theory).
 * 
 * This code has been developped by:
 * 
 * Jerome Lacan - jerome.lacan@isae.fr
 * Emmanuel Lochin - emmanuel.lochin@isae.fr
 * Pierre-Ugo Tournoux - pierre-ugo.tournoux@isae.fr 
 *
 * Copyright ISAE 2011
 */

#ifndef _COMMON_H_
#define _COMMON_H_ 1

#include "common_struct_def.h"
#include "tetrys_main_lib.h"
#include "circularBuffer.h"
#include "rcv_automate.h"

/** Global sender statistics (allocated in main, shared read-only by library). */
extern struct senderStats *sndStats;
/** Global receiver statistics (allocated in main, shared read-only by library). */
extern struct receiverStats *rcvStats;
/** Set by the SIGALRM handler when a periodic timer expires. */
extern volatile sig_atomic_t tetrys_alarm_pending;
/** Set by SIGINT/SIGTERM/SIGQUIT handlers to request a clean shutdown. */
extern volatile sig_atomic_t tetrys_stop_requested;

/** Return the current wall-clock time as a double (seconds since epoch). */
double get_time(void);

/**
 * Create a UDP socket bound to @p port for receiving.
 * @param port  Local UDP port (host byte order).
 * @return The socket file descriptor, or exits on error.
 */
int server_socket_rcv(uint16_t port);

/**
 * Create a UDP socket bound to @p port + 1 for sending.
 * @param port  Base UDP port (host byte order).
 * @return The socket file descriptor, or exits on error.
 */
int server_socket_snd(uint16_t port);

/**
 * SIGALRM handler: flag that a periodic timer has expired (async-signal-safe).
 * Shared by modules that run the timer-driven Tetrys loop.
 * @param sig  Signal number (unused).
 */
void tetrys_handle_alarm(int sig);

/** Install signal handlers (SIGALRM, SIGINT, SIGFPE, etc.). */
void manage_signal(void);

/**
 * Graceful shutdown: flush stats and exit. Also usable on fatal errors.
 * @param sig Signal number (0 when called directly from library code).
 */
_Noreturn void quit(int sig);

/** SIGFPE handler — write an async-signal-safe message and terminate. */
_Noreturn void float_error(int sig);

/**
 * Parse a command-line integer with full validation (strtol based).
 * Exits with an error message when @p str is not a number in [@p min, @p max].
 * @param str  Text to parse (e.g. optarg).
 * @param min  Smallest accepted value.
 * @param max  Largest accepted value.
 * @param name Option name used in the error message.
 * @return The parsed value.
 */
long parse_long_arg(const char *str, long min, long max, const char *name);

/**
 * Wrapper around malloc() that aborts with @p message on allocation failure.
 * @param sz       Number of bytes to allocate.
 * @param message  Context string displayed on failure.
 * @return Pointer to the allocated memory (never NULL).
 */
void *my_malloc(size_t sz, const char *message);

/**
 * Print a GF matrix to stdout for debugging.
 * @param m1   Pointer to the matrix data (row-major, gf elements).
 * @param rows Number of rows.
 * @param cols Number of columns.
 * @param s    Label string printed before the matrix.
 */
int pr_matrix(const void *m1, int rows, int cols, const char *s);

/* -----------------------------------------------------------------------
 * FEC encoding / decoding helpers
 * ----------------------------------------------------------------------- */

/**
 * Populate tetrys->lost_positions[] with sequence numbers of missing packets
 * inferred from the RDBuffer and RRBuffer contents.
 * @return Number of lost packets found.
 */
int get_lost_positions(void);

/**
 * Multiply each repair symbol in @p repairQueue by the inverse matrix row
 * and accumulate the result into @p lost_to_recover.
 * @param repairQueue    Queue of repair symbols (RRBuffer).
 * @param lost_to_recover Matrix of recovery vectors (one row per lost packet).
 * @param matrix         The inverted decoding matrix (k × k, GF(2^8)).
 * @param nb_lost        Number of lost packets (number of rows to compute).
 * @return 0 on success.
 */
int mul_mat_by_repair(const struct circBUFFER *repairQueue,
                      gf **lost_to_recover, const gf *matrix, int nb_lost);

/**
 * Build the decoding matrix from the currently buffered repair symbols and
 * invert it in place using Gaussian elimination over GF(2^8).
 * @param matrix  Output buffer for the inverted matrix (k × k).
 * @return 0 on success, -1 if the matrix is singular (unrecoverable).
 */
int build_matrix_and_invert(gf *matrix);

/**
 * Subtract all buffered data packets from all buffered repair symbols.
 * After this call each repair symbol contains only the contribution of
 * the lost (not-yet-recovered) packets, ready for matrix inversion.
 */
void subtract_all(void);

/** Initialise the FEC Galois-field lookup tables (must be called once at startup). */
void init_fec(void);

/**
 * Build and invert the FEC decoding matrix for a block of @p nbLInBlock lines.
 * Used in FEC block-mode (fecMode=1) instead of build_matrix_and_invert().
 * @return 0 on success, -1 if singular.
 */
int build_fec_matrix_and_invert(gf *matrix, int nbLInBlock);

/**
 * FEC-mode variant of subtract_all(): subtract data from repair symbols
 * within the current FEC encoding window only.
 */
void subtract_all_fec(void);

/**
 * FEC-mode variant of get_lost_positions(): populate tetrys->lost_positions[]
 * for the current FEC block window.
 * @return Number of lost packets found within the window.
 */
int get_fec_lost_positions(void);

/**
 * Encode the data packets in @p metaBuffer into one FEC repair symbol @p fec.
 * Only packets with sequence numbers in [@p min, @p max] are included.
 * @param metaBuffer The sender encoding buffer (SEBuffer).
 * @param fec        Output buffer for the repair symbol (PLS bytes).
 * @param sz         Symbol size (should equal PLS).
 * @param set_seed   Repair sequence number (used as the Vandermonde row index).
 * @param min        Minimum data sequence number to cover.
 * @param max        Maximum data sequence number to cover.
 * @return 0 on success.
 */
int tet_encode(const struct circBUFFER *metaBuffer, gf *fec,
               int sz, int set_seed, unsigned long int min,
               unsigned long int max);

/** Unit test for build_matrix_and_invert(). @return 0 if correct. */
int test_build_matrix_and_invert(void);

#ifdef STATS
/* ---- Sender statistics ------------------------------------------------ */

/**
 * Reset the sender statistics.
 * @param stats  Statistics to initialise.
 * @param psize  Payload size used for the rate computations (bytes).
 */
void stat_init_sender_stats(struct senderStats *stats, int psize);

/** Account a repair packet about to be sent. */
void stat_repair_set_stats(const struct PDU *pkt, struct senderStats *stats);

/** Account a data packet about to be sent. */
void stat_data_set_stats(const struct PDU *pkt, struct senderStats *stats);

/** Account an acknowledgement received by the sender. */
void stat_ack_set_stats(const struct PDU *pkt, struct senderStats *stats);

/** Write the sender histograms and summary to the "sndStat_*.stats" files. */
void stat_save_sender_stats(struct senderStats *stats);

/** Print the sender summary on stdout. */
void stat_print_sender_stats(const struct senderStats *stats);

/* ---- Receiver statistics ---------------------------------------------- */

/**
 * Reset the receiver statistics.
 * @param stats  Statistics to initialise.
 * @param psize  Payload size used for the rate computations (bytes).
 */
void stat_init_rcv_stats(struct receiverStats *stats, int psize);

/** Account the losses detected on a packet reception (inter-loss histogram). */
void stat_rcv_loss(struct receiverStats *stats, const struct TETRYS *tet);

/** Open a loss event when the first loss is detected. */
void stat_rcv_start_loss_event(struct receiverStats *stats, const struct TETRYS *tet);

/** Close the current loss event once its packets are recovered. */
void stat_rcv_end_loss_event(struct receiverStats *stats, struct TETRYS *tet);

/** Account a repair packet received. */
void stat_rcv_repair_set_stats(const struct PDU *pkt, struct receiverStats *stats);

/** Account a data packet received. */
void stat_rcv_data_set_stats(struct receiverStats *stats);

/** Account an acknowledgement sent by the receiver. */
void stat_send_ack_set_stats(struct receiverStats *stats);

/** Account a matrix inversion attempt (indexed by the number of losses). */
void stat_decoding_try(void);

/** Account a successful matrix inversion. */
void stat_decoding_succ(void);

/** Account a failed matrix inversion. */
void stat_decoding_failure(void);

/** Write the receiver histograms and summaries to the "rcvStat_*.stats" files. */
void stat_save_rcv_stats(void);

/** Print the receiver summary on stdout. */
void stat_print_rcv_stats(const struct receiverStats *stats);

/**
 * Account the recovery of packet @p numrcvd (delay histogram).
 * @param numrcvd  Sequence number of the recovered packet.
 */
void stat_recover_pkt(unsigned long int numrcvd);
#endif

#endif				/* _COMMON_H_ 1 */
