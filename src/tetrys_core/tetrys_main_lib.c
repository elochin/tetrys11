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

#include "common.h"

/* 
 * NO one have to modify this file during the creation of a module.
 * Input/Output function are registered in the main file "tetrys_main.c" via the init_context() function
 */

static int tetrys_debug_nbTIAdded = 0;

#define NEW_GF_MATRIX(rows, cols) \
    ((gf *)my_malloc((size_t)(rows) * (size_t)(cols) * sizeof(gf), "NEW_GF_MATRIX"))

static int check_if_can_decode(void);
static int try_to_decode(void);
static void try_to_decode_fec_mode(void);
static void tet_try_flush_fo_buffer(void);
static void tet_actuate_loss_rate(int _pls);
static void send_repair(void);
static void scan_tetrys_ack(const struct PDU *stamp_to_parse);
static int check_nb_item_in_buffers(void);
static void non_blocking_input_check(void);
static void blocking_input_check(void);
static int tet_is_everything_rcvd(void);
static void send_ack_only(void);
static void send_redundancy(void);

/**
 * Clamp a decoded padding count to [0, TETRYS_MAX_PAYLOAD].
 * @param padding  Raw padding field read from datav.
 * @return Clamped padding value ensuring pls = TETRYS_MAX_PAYLOAD - padding is in range.
 */
static int clamp_padding(int padding)
{
    if (padding < 0)
	return 0;
    if (padding > TETRYS_MAX_PAYLOAD)
	return TETRYS_MAX_PAYLOAD;
    return padding;
}

/*
 *  Static function to calculate index and offset, given bit position
 */

/**
 * Compute the byte index @p idx and the bit offset @p ofs of bit @p posn in a bit-vector.
 * @param posn  Zero-based bit index.
 * @param[out] idx  Byte index within the vector.
 * @param[out] ofs  Bit offset within that byte.
 */
static void vect_idx_calc(size_t posn, size_t * idx, size_t * ofs)
{
    *idx = posn / CHAR_BIT;	/* Byte index [0-N] in the array    */
    *ofs = posn % CHAR_BIT;	/* Bit number [0-N] within the byte */
}

int vect_getbit(const gf * set, int number)
{
    size_t idx, ofs;

    if (number < 0 || number >= SIZE_SUBTRACT_VECTOR * CHAR_BIT)
	return 0;
    vect_idx_calc((size_t) number, &idx, &ofs);
    set += idx;
    return (*set & (1 << ofs)) != 0;	/* 0 or 1   */
}

void vect_setbit(gf * set, int number, int value)
{
    size_t idx, ofs;

    if (number < 0 || number >= SIZE_SUBTRACT_VECTOR * CHAR_BIT)
	return;
    vect_idx_calc((size_t) number, &idx, &ofs);
    set += idx;
    if (value)
	*set = (gf) (*set | (gf) (1U << ofs));	/* set bit  */
    else
	*set = (gf) (*set & (gf) ~(1U << ofs));	/* clear bit */
}

void print_bitvector(const struct circBUFFERItem *_tmp)
{
    if (_tmp->pkt->max_redundancy < _tmp->pkt->min_redundancy)
	return;
    unsigned long diff = _tmp->pkt->max_redundancy - _tmp->pkt->min_redundancy;
    int repair_min_max = (diff >= (unsigned long) (SIZE_SUBTRACT_VECTOR * CHAR_BIT))
	? (SIZE_SUBTRACT_VECTOR * CHAR_BIT - 1) : (int) diff;
    int bit_i;
    fprintf(stdout,
	    "\nprint_bitvector: Rep %lu from min %lu to %lu len %d : ",
	    _tmp->pkt->num, _tmp->pkt->min_redundancy,
	    _tmp->pkt->max_redundancy, repair_min_max);
    for (bit_i = 0; bit_i <= repair_min_max; bit_i++)
	fprintf(stdout, " %d ",
		vect_getbit((const gf *) & _tmp->subtracted_vect, bit_i));
}

int vect_get_nb_zero(const struct circBUFFERItem *_tmp)
{
    if (_tmp->pkt->max_redundancy < _tmp->pkt->min_redundancy)
	return 0;
    unsigned long diff = _tmp->pkt->max_redundancy - _tmp->pkt->min_redundancy;
    int repair_min_max = (diff >= (unsigned long) (SIZE_SUBTRACT_VECTOR * CHAR_BIT))
	? (SIZE_SUBTRACT_VECTOR * CHAR_BIT - 1) : (int) diff;
    int bit_i;
    int toRet = 0;
    for (bit_i = 0; bit_i <= repair_min_max; bit_i++)
	toRet +=
	    (0 == vect_getbit((const gf *) & _tmp->subtracted_vect, bit_i));
    if (PR_DB_1)
	fprintf(stdout,
		"\nvect_get_nb_zero=%d Rep %lu from min %lu to %lu len %d : ",
		toRet, _tmp->pkt->num, _tmp->pkt->min_redundancy,
		_tmp->pkt->max_redundancy, repair_min_max);
    return toRet;
}

/**
 * Create a circular buffer of @p nb_items packets and give it a short name
 * (at most 2 characters are kept) used in the debug traces.
 */
static struct circBUFFER *new_named_buffer(int nb_items, const char *name)
{
    struct circBUFFER *buf = circ_create_buffer(nb_items, PLS);

    snprintf(buf->buffName, sizeof(buf->buffName), "%.2s", name);
    return buf;
}

/** Allocate all the Tetrys packet buffers (only once, at init time). */
static void init_tetrys_buffers(void)
{
    tetrys->SEBuffer = new_named_buffer(4000, "SE");
    tetrys->RRBuffer = new_named_buffer(800, "RR");
    tetrys->RDBuffer = new_named_buffer(4000, "RD");
    tetrys->OBuffer = new_named_buffer(4000, "OB");
    tetrys->TIBuffer = new_named_buffer(4000, "TI");
    tetrys->DIBuffer = new_named_buffer(1000, "DI");
    tetrys->tmpFecRep = new_named_buffer(100, "FR");
    tetrys->tmpFecDat = new_named_buffer(100, "FD");
    tetrys->FOBuffer = new_named_buffer(100, "FD");
}

/** Allocate and clear the arrays used to store the lost / rebuilt packets. */
static void init_tetrys_recovery_arrays(void)
{
    int r;

    tetrys->lost_positions_cap = 1000;
    tetrys->lost_to_recover =
	my_malloc((size_t) tetrys->lost_positions_cap * sizeof(gf *),
		  "repair alloc for subtraction");
    tetrys->lost_positions =
	my_malloc((size_t) tetrys->lost_positions_cap * sizeof(unsigned long int),
		  "repair alloc for subtraction");
    for (r = 0; r < 100; r++)
	tetrys->lastRebuiltBlock[r] = 0;
    memset(&tetrys->blockId, '\0', sizeof(unsigned long int) * 600);
    tetrys->nbLastRebuiltBlock = 0;
    tetrys->nextExpected = 0;
    /* Single contiguous zeroed slab avoids 1000 separate heap allocations. */
    size_t slab_bytes = (size_t) tetrys->lost_positions_cap * (size_t) tetrys->pktSize;
    gf *slab = my_malloc(slab_bytes, "lost_to_recover slab");
    memset(slab, '\0', slab_bytes);
    for (r = 0; r < tetrys->lost_positions_cap; r++)
	tetrys->lost_to_recover[r] = slab + (size_t) r * (size_t) tetrys->pktSize;
}

/** Reset the receiver-side counters and refresh-window state. */
static void init_tetrys_rcv_state(void)
{
    tetrys->tot_d_pkt_rcv = 0;	      /**< the total number of data pkt rcvd */
    tetrys->tot_r_pkt_rcv = 0;	      /**< The total number of repair pkt rcvd */
    tetrys->tot_p_pkt_rcv = 0;	      /**< The total number of UDP pkt rcvd */

	/**< NOTE that A REFRESH EVENT refer indifferently to a RECOVERY EVENT or A REDUNDANCY RECEPTION WITHOUT LOSSES */
    tetrys->queue_num_of_first_wind_pkt = 0;	    /**< -The index of the p_seq_num_of_last_refresh in the queue windows */
						/**< =0 when highest_min_r_rcvd==max_seq_num_seen */
    tetrys->nb_d_since_last_refresh = 0;	   /**<  -The number of data pkt rcvd since last refresh */
    tetrys->nb_r_since_last_refresh = 0;	   /**<  -The number of repair pkt rcvd since last refresh*/
    tetrys->nb_p_since_last_refresh = 0;	   /**<  -The number of data pkt rcvd since last refresh*/
    tetrys->p_seq_num_of_last_refresh = 0;	/**< -The max_d_seq_num_seen+1 before at the last refresh. Refered as # "0" for the current repair windows */
    tetrys->queue_num_of_last_refresh = 0;	   /**<  - the queue index of the highest D pkt receive during last refresh */

    tetrys->nb_d_in_queue = 0;			    /**< -The number of stored data pkt. should be equal to "nb_d_since_last_refresh"*/
}

#if MONITOR || defined(SAVEPACKET) || defined(STATS)
/**
 * Open a trace or stats file in the working directory; exit if it cannot be created.
 * @param name  File path to open for writing.
 * @return Open FILE handle (never NULL).
 */
static FILE *monitor_fopen(const char *name)
{
    FILE *f = fopen(name, "w");

    if (f == NULL) {
	perror(name);
	exit(EXIT_FAILURE);
    }
    return f;
}
#endif

/*
 * Allocate global Tetrys structures, optionally statistics counters, then
 * initialize GF(2^8) tables and protocol state.
 */
void tetrys_alloc_globals(int role, int pkt_size)
{
    if ((tetrys = calloc(1, sizeof(struct TETRYS))) == NULL) {
	fprintf(stderr, "tetrys_alloc_globals: calloc TETRYS failed\n");
	exit(EXIT_FAILURE);
    }
    if ((sig_AckRed = calloc(1, sizeof(struct SIG_ACK_RED))) == NULL) {
	fprintf(stderr, "tetrys_alloc_globals: calloc SIG_ACK_RED failed\n");
	exit(EXIT_FAILURE);
    }
#ifdef STATS
    if ((sndStats = calloc(1, sizeof(struct senderStats))) == NULL) {
	fprintf(stderr, "tetrys_alloc_globals: calloc senderStats failed\n");
	exit(EXIT_FAILURE);
    }
    if ((rcvStats = calloc(1, sizeof(struct receiverStats))) == NULL) {
	fprintf(stderr, "tetrys_alloc_globals: calloc receiverStats failed\n");
	exit(EXIT_FAILURE);
    }
    if (role != ROLE_RECEIVER)
	stat_init_sender_stats(sndStats, pkt_size);
    if (role != ROLE_SENDER)
	stat_init_rcv_stats(rcvStats, pkt_size);
#else
    (void) role;
    (void) pkt_size;
#endif
    init_fec();
    init_tetrys();
}

/*
 * This function MUST be called before the reception of any packet
 * The init_fec() function MUST have been called previously ! 
 */

void init_tetrys(void)
{
    tetrys->closePending = 0;
    tetrys->fec_n = 0;
    tetrys->fec_k = 0;
    tetrys->monitorSnd = NULL;
    tetrys->monitorRcv = NULL;
    tetrys->monitorFecSnd = NULL;
    tetrys->monitorFecRcv = NULL;
    tetrys->monitorSub = NULL;
    tetrys->savePkt = NULL;
    tetrys->lossTimeRecovery = NULL;
#if MONITOR
    tetrys->monitorSnd = monitor_fopen("monitorCOEF.snd");
    tetrys->monitorRcv = monitor_fopen("monitorCOEF.rcv");
    tetrys->monitorFecSnd = monitor_fopen("monitorFEC.snd");
    tetrys->monitorFecRcv = monitor_fopen("monitorFEC.rcv");
    tetrys->monitorSub = monitor_fopen("monitorSub.rcv");
#endif
#ifdef SAVEPACKET
    tetrys->savePkt = monitor_fopen("pktSaved.tr");
#endif
    tetrys->snd_pkt_since_last_repair = 0;
    tetrys->repair_pending = 0;
    tetrys->nb_rep_sent = 4;
    tetrys->next_seqnumToAdd = 0;
    tetrys->nbOldDatOrRedRcvdSinceLastTry = 1;
    tetrys->min_seq_in_red = 0;	  /**< The minimum sequence number tb encoded in a red */
    tetrys->max_seq_in_red = 0;	  /**< The maximum sequence number tb encoded in a red */
    tetrys->currentPktInWindow = 0;
    tetrys->fecMode = 0;
    tetrys->deliverInOrder = 0;
    tetrys->nb_rebuilt = 0;
    tetrys->pktSize = PLS;
    tetrys->max_acked_minus_two = 0;
#ifdef STATS
    tetrys->lossTimeRecovery = monitor_fopen("lossTimeRecovery.txt");
#endif
    tetrys->nb_sent_to_upper_layer = 0;
    tetrys->start_time = get_time();
    tetrys->ts_last_ack_sent = 0.0;
    tetrys->nb_loss_delay = 0;
    tetrys->rtt = 0.0;
    tetrys->dst_rtt = 0.0;
    tetrys->previous_rtt = 0.0;
    tetrys->rcv_time = 0.0;
    tetrys->rtt_smooth_param = 0.5;
    tetrys->last_probe_rcvd = 0.0;
    tetrys->ts_of_last_probe_rcvd = 0.0;
    tetrys->lossRate = 0.0f;
    tetrys->nb_tot_fec_lost = 0;
    tetrys->fecMaxCurWind = 0;
    tetrys->p_size = tetrys->pktSize;		/**< Payload size+4 ( 4 for the int that describe the paload size :) )*/
    tetrys->max_d_seq_num_seen = 0;    /**< The highest seq_num seen for a data pkt */
    tetrys->max_r_seq_num_seen = 0;    /**< The highest redundancy seq_max seen */
    tetrys->max_seq_num_seen = 0;      /**< = MAX(max_r_seq_num_seen,max_d_seq_num_seen) */
    tetrys->min_data_pkt_to_retain = 0;	  /**< the min pkt that we may substract the repair pkt 
					       equal to the "highest_min_r_rcvd" seen a last refresh. */
    tetrys->highest_min_r_rcvd = 0;	  /**< The highest min redundancy received with a repair pkt. That means that after a recovery Data pkt with seq_num below this this value can be deleted. */
    //
    init_tetrys_rcv_state();
    //
    tetrys->nb_pkt_lost = 0;
    tetrys->padding_on_last_pkt = 0;
    init_tetrys_buffers();

    tetrys->fec_currentWindow = 0;	// for Fec Mode
    tetrys->fec_next_expected = 0;	// for Fec Mode
    tetrys->currentWindow = 0;
    init_tetrys_recovery_arrays();
    memset(&tetrys->lossHistory, '\0',
	   sizeof(unsigned char) * TETRYS_LOSS_HISTORY);
    memset(&tetrys->interPktIHistory, '\0',
	   sizeof(double) * TETRYS_INTERPKT_HISTORY);
    memset(&tetrys->interPktOHistory, '\0',
	   sizeof(double) * TETRYS_INTERPKT_HISTORY);
    tetrys->lastInterPktI_ts = 0.0;
    tetrys->lastInterPktO_ts = 0.0;
    tetrys->sensedPktIRate = 0.0;
    tetrys->sensedPktORate = 0.0;
    tetrys->last_rcv_time = 0;
    tetrys->begintime = get_time();
    tetrys->incoming_length = -1;
    /* Init the first number of redundancy packet */
    tetrys->nb_incoming_tet = 0;
    tetrys->nb_incoming_dat = 0;
    tetrys->nb_outgoing_item = 0;
    tetrys->tmp_timeval.tv_sec = 0;
    tetrys->tmp_timeval.tv_usec = 0;
    tetrys->nodeid = 0;
    tetrys->nbOfFOBtoOB = 0;
    tetrys->nbOBFailed = 0;
    sig_AckRed->endAt = -1.0;
    tetrys->freqTryToDecode = 1;
    atomic_init(&tetrys->end_transfer_requested, 0);
    tetrys->end_transfer_active = 0;
    atomic_init(&tetrys->transfer_acked, 0);
}

/** Close a FILE handle if non-NULL and clear the pointer. */
static void close_if_open(FILE **fp)
{
    if (fp != NULL && *fp != NULL) {
	fclose(*fp);
	*fp = NULL;
    }
}

void tetrys_cleanup(void)
{
    tetrys_disarm_alarm();

#ifdef STATS
    if (tetrys != NULL) {
	if (tetrys->currentRole != 2 && sndStats != NULL) {
	    stat_save_sender_stats(sndStats);
	    stat_print_sender_stats(sndStats);
	}
	if (tetrys->currentRole != 1 && rcvStats != NULL) {
	    stat_save_rcv_stats();
	    stat_print_rcv_stats(rcvStats);
	}
    }
#endif
    free(sndStats);
    sndStats = NULL;
    free(rcvStats);
    rcvStats = NULL;

    if (tetrys != NULL) {
	circ_free_queue(&tetrys->SEBuffer);
	circ_free_queue(&tetrys->RRBuffer);
	circ_free_queue(&tetrys->RDBuffer);
	circ_free_queue(&tetrys->OBuffer);
	circ_free_queue(&tetrys->TIBuffer);
	circ_free_queue(&tetrys->DIBuffer);
	circ_free_queue(&tetrys->tmpFecRep);
	circ_free_queue(&tetrys->tmpFecDat);
	circ_free_queue(&tetrys->FOBuffer);

	if (tetrys->lost_to_recover != NULL) {
	    free(tetrys->lost_to_recover[0]);
	    free(tetrys->lost_to_recover);
	    tetrys->lost_to_recover = NULL;
	}
	free(tetrys->lost_positions);
	tetrys->lost_positions = NULL;

	close_if_open(&tetrys->monitorSnd);
	close_if_open(&tetrys->monitorRcv);
	close_if_open(&tetrys->monitorSub);
	close_if_open(&tetrys->monitorFecSnd);
	close_if_open(&tetrys->monitorFecRcv);
	close_if_open(&tetrys->savePkt);
	close_if_open(&tetrys->lossTimeRecovery);

	free(tetrys);
	tetrys = NULL;
    }

    free(sig_AckRed);
    sig_AckRed = NULL;
}

void tetrys_disarm_alarm(void)
{
    struct itimerval itv;
    memset(&itv, 0, sizeof(itv));
    setitimer(ITIMER_REAL, &itv, NULL);
}

/**
 * Arm a one-shot ITIMER_REAL expiring at @p target_time (or in 100 us if overdue).
 * @param target_time  Absolute wall-clock timestamp (seconds since epoch).
 */
static void arm_itimer_at(double target_time)
{
    double delta = target_time - get_time();
    if (delta < 0.0001)
	delta = 0.0001;
    struct itimerval itv;
    memset(&itv, 0, sizeof(itv));
    itv.it_value.tv_sec = (time_t) delta;
    itv.it_value.tv_usec = (suseconds_t) ((delta - (double) itv.it_value.tv_sec) * 1000000.0);
    if (itv.it_value.tv_sec == 0 && itv.it_value.tv_usec < 100)
	itv.it_value.tv_usec = 100;
    setitimer(ITIMER_REAL, &itv, NULL);
}

void actuate_alarm(int action)
{
/**< action=1 if ack were sent, 2 if rep were sent, 3 if both, 4 send dat */
    if ((sig_AckRed->endAt > 0.0) && (sig_AckRed->endAt < get_time())) {
	tetrys_stop_requested = 1;
	return;
    }
    if (PR_DB_1)
	printf("\nactuateAlarm: BEGIN %d !", action);
    if (action == 1 || action == 3) {
	sig_AckRed->next_ack = get_time() + sig_AckRed->ack_freq;
    }
    if (action == 2 || action == 3) {
	if (tetrys->currentRole == 0)
	    sig_AckRed->next_ack = get_time() + sig_AckRed->ack_freq;				/**< If bidir ack is sent with red */
	sig_AckRed->next_rep = get_time() + sig_AckRed->rep_freq;
	if (PR_DB_1)
	    printf("\nactuateAlarm: next_rep %f -- get_time %f !",
		   sig_AckRed->next_rep, get_time());
    }
    if (action == 4) {
	sig_AckRed->next_datf = get_time() + sig_AckRed->datf_freq;
	if (PR_DB_1)
	    printf("\nactuateAlarm: next_datf %f -- get_time %f !",
		   sig_AckRed->next_datf, get_time());
    }

    double soonest = 0.0;
    int has_timer = 0;
    if (sig_AckRed->rep_enabled == 1) {
	soonest = sig_AckRed->next_rep;
	has_timer = 1;
    }
    if (sig_AckRed->ack_enabled == 1) {
	soonest = has_timer ? MIN(soonest, sig_AckRed->next_ack) : sig_AckRed->next_ack;
	has_timer = 1;
    }
    if (sig_AckRed->datf_enabled == 1) {
	soonest = has_timer ? MIN(soonest, sig_AckRed->next_datf) : sig_AckRed->next_datf;
	has_timer = 1;
    }
    if (has_timer)
	arm_itimer_at(soonest);

    if (PR_DB_1)
	fprintf(stdout,
		"\nactuateAlarm: @ %f action %d next ack %f us %f s ackFreq %f next rep %f ",
		get_time(), action, sig_AckRed->next_ack,
		(sig_AckRed->next_ack - get_time()), sig_AckRed->ack_freq,
		sig_AckRed->next_rep);

    return;
}

void tetrys_process_pending_alarm(void)
{
    if (!tetrys_alarm_pending)
	return;
    tetrys_alarm_pending = 0;
    if (sig_AckRed == NULL)
	return;
    double now = get_time();
    if (sig_AckRed->ack_enabled == 1 && sig_AckRed->next_ack <= now)
	send_ack_only();
    if (sig_AckRed->rep_enabled == 1 && sig_AckRed->next_rep <= now)
	send_redundancy();
}

void tetrys_check_end_transfer(void)
{
    if (tetrys == NULL || tetrys->currentRole != 1)
	return;
    if (!atomic_load(&tetrys->end_transfer_requested))
	return;
    if (!tetrys->end_transfer_active) {
	tetrys->end_transfer_active = 1;
	sig_AckRed->rep_enabled = 1;
	sig_AckRed->rep_freq = 0.01;
	sig_AckRed->ack_enabled = 0;
	sig_AckRed->ack_freq = 0.9;
	actuate_alarm(2);
    }
    if (tetrys->DIBuffer->nbItemStored == 0
	&& tetrys->OBuffer->nbItemStored == 0
	&& tet_is_everything_rcvd() == 1) {
	atomic_store(&tetrys->end_transfer_requested, 0);
	tetrys->end_transfer_active = 0;
	atomic_store(&tetrys->transfer_acked, 1);
    }
}

int tetrys_step_once(void)
{
    wait_for_input();

    tetrys_process_pending_alarm();
    if (tetrys_stop_requested)
	return 0;

    treat_ti_buffer();
    send_dat_in_order();
    treat_o_buffer();
    treat_di_buffer();
    treat_o_buffer();
    tetrys_check_end_transfer();
    return !tetrys_stop_requested;
}




/** Print the loss/repair counters when more losses than repairs are pending (debug). */
static void cid_print_pending_state(void)
{
    int i;

    if (tetrys->nb_pkt_lost >= tetrys->RRBuffer->nbItemStored)
	return;
    fprintf(stdout, "\ncheck_if_can_decode: %d loss %d red  psnolr %lu",
	    tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored,
	    tetrys->p_seq_num_of_last_refresh);
    for (i = 0; i < tetrys->nb_pkt_lost; i++) {
	printf(" lp %lu ", tetrys->lost_positions[i]);
    }
    if (PR_DB_1) {
	fflush(stdout);
	fflush(stderr);
    }
}

/**
 * Check that each of the first @p nb_decode lost packets is covered by at
 * least one of the first @p nb_decode repair packets.
 * @param nb_decode  Number of lost packets considered in the subset.
 * @return 1 if every lost packet is covered, 0 otherwise.
 */
static int cid_lost_all_included(int nb_decode)
{
    int lost_i = 0;
    int each_lost_included = 0;

    if (PR_DB_1)
	fprintf(stdout, "\ncheck_if_can_decode: nb_decode = %d", nb_decode);
    while (lost_i < nb_decode) {
	if (PR_DB_2)
	    fprintf(stdout, "\ncheck_if_can_decode: nb_decode = %d lost_i %d ",
		    nb_decode, lost_i);
	each_lost_included = 0;
	if (1 == circ_init_iterator(tetrys->RRBuffer)) {
	    int rep_i = 0;
	    while (each_lost_included == 0 && (rep_i < nb_decode)
		   && (1 == circ_get_next(tetrys->RRBuffer))) {
		/**< If it is included in one at least, stop */
		const struct PDU *rep = tetrys->RRBuffer->iterator->pkt;
		if (PR_DB_2)
		    fprintf(stdout,
			    "\ncheck_if_can_decode: TEST nb_decode = %d lost_i %d num=%lu inc in  rep_i=%d w min_red %lu max_red %lu ",
			    nb_decode, lost_i, tetrys->lost_positions[lost_i],
			    rep_i, rep->min_redundancy, rep->max_redundancy);
		if ((rep->min_redundancy <= tetrys->lost_positions[lost_i])
		    && (rep->max_redundancy >= tetrys->lost_positions[lost_i])) {
		    each_lost_included = 1;
		    if (PR_DB_2)
			fprintf(stdout, "y");
		} else if (PR_DB_2)
		    fprintf(stdout, "  NO ");
		rep_i++;
	    }
	}
	lost_i++;
	if (each_lost_included == 0)
	    lost_i = nb_decode;	//the current pkt is not included in any of the considered repair. e.g. it can not be reconstructed
    }
    if (tetrys->nb_pkt_lost < tetrys->RRBuffer->nbItemStored)
	printf("\n each_lost_included %d nb_decode %d lost_i %d ",
	       each_lost_included, nb_decode, lost_i);
    return each_lost_included;
}

/**
 * Check that each of the first @p nb_decode repair packets still contains
 * between 1 and @p nb_decode unknowns (not yet subtracted data packets).
 * Updates tetrys->max_red_useful.
 * @param nb_decode  Number of lost packets considered in the subset.
 * @return 1 if all the repairs are usable, 0 otherwise, -1 if the repair
 *         buffer could not be iterated.
 */
static int cid_repairs_usable(int nb_decode)
{
    int rep_i = 0;
    int bit_i;
    int each_repair_ok = 1;

    if (1 != circ_init_iterator(tetrys->RRBuffer))
	return -1;
    while ((each_repair_ok == 1) && (rep_i < nb_decode)
	   && (1 == circ_get_next(tetrys->RRBuffer))) {
	/**< Get the number of unknows in the repair*/
	const struct PDU *rep = tetrys->RRBuffer->iterator->pkt;
	if (rep->max_redundancy < rep->min_redundancy) {
	    each_repair_ok = 0;
	    break;
	}
	unsigned long diff = rep->max_redundancy - rep->min_redundancy;
	int repair_min_max =
	    (diff >= (unsigned long) (SIZE_SUBTRACT_VECTOR * CHAR_BIT))
	    ? (SIZE_SUBTRACT_VECTOR * CHAR_BIT - 1) : (int) diff;
	int repair_nb_included = 0;
	if (tetrys->nb_pkt_lost < tetrys->RRBuffer->nbItemStored)
	    print_bitvector(tetrys->RRBuffer->iterator);
	for (bit_i = 0; bit_i <= repair_min_max; bit_i++)
	    repair_nb_included +=
		(0 == vect_getbit((const gf *) &(tetrys->RRBuffer->iterator)->subtracted_vect,
				  bit_i));
	if (tetrys->nb_pkt_lost < tetrys->RRBuffer->nbItemStored)
	    fprintf(stdout,
		    "\ncheck_if_can_decode: nb_decode = %d rep_id %lu min %lu max %lu have %d pkt inc ",
		    nb_decode, rep->num, rep->min_redundancy,
		    rep->max_redundancy, repair_nb_included);
	if ((repair_nb_included < 1) || (repair_nb_included > nb_decode))
	    each_repair_ok = 0;	//At least one of the considered repair is  contain more than nb_decode variable
	tetrys->max_red_useful = tetrys->RRBuffer->iterator->seqnum;
	rep_i++;
    }
    if (PR_DB_1)
	fprintf(stdout,
		"\ncheck_if_can_decode: nb_decode = %d each_repair_ok %d max_red_useful %lu ",
		nb_decode, each_repair_ok, tetrys->max_red_useful);
    return each_repair_ok;
}

/**
 * Determine if some lost packets can be decoded and how many.
 * Updates tetrys->max_to_decode and tetrys->max_red_useful.
 * Assumes get_lost_positions() was already called.
 * @return Number of packets that can be decoded, 0 or -1 otherwise.
 */
static int check_if_can_decode(void)
{
	/**< The rule: X lost packet can be decoded by X repair if:
	*	-Each of the lost packet is included in at least one repair packet
	*	-There is nb lost considered=nb repair packet considered
	*	-Each repair packet include at least one lost packet and maximum X packet
	* Process iteratvely X=1 packet, check if can decode, X=2, X=3.. up to nb losses
	* Whitout reordering it will decode with X=nb_lost_pkt
	*/
    int rep_i = 0;
    int nb_decode;

    cid_print_pending_state();
    nb_decode = tetrys->nb_pkt_lost;
    tetrys->max_to_decode = (unsigned long) nb_decode;
    tetrys->max_red_useful = 0;
    /* Fast path: as many repairs as losses. */
    if (1 == circ_init_iterator(tetrys->RRBuffer)) {
	while ((rep_i < nb_decode) && (1 == circ_get_next(tetrys->RRBuffer))) {
	    rep_i++;
	    tetrys->max_red_useful = tetrys->RRBuffer->iterator->seqnum;
	}
    }
    if (rep_i >= nb_decode) {
	if (PR_DB_1)
	    printf("fastdecode %d", nb_decode);
	return nb_decode;
    }
    tetrys->max_to_decode = 0;
    tetrys->max_red_useful = 0;
#ifdef DECODEINORDER
    return 0;
#else
    /* Slow path: look for the largest X such that X losses are decodable by X repairs. */
    for (nb_decode = 1; nb_decode <= tetrys->nb_pkt_lost; nb_decode++) {
	/**< Chech if there are at least nb_decode repair packet */
	if (tetrys->RRBuffer->nbItemStored < nb_decode)
	    return -1;
	/**< For each lost "nb_decode" packet check if it is included in at least one of the "nb_decode" repair pkt */
	int each_lost_included = cid_lost_all_included(nb_decode);
	if (each_lost_included == 0) {
	    if (PR_DB_1)
		fprintf(stdout,
			"\ncheck_if_can_decode: nb_decode = %d !each_lost_included RETURN -1",
			nb_decode);
	    return -1;
	}
	/**< For each repair packet considered, check if there is at least one lost pkt included and maximum X lost packets */
	if (PR_DB_1)
	    fprintf(stdout,
		    "\ncheck_if_can_decode: nb_decode = %d each_lost_included CONTINUE",
		    nb_decode);
	int each_repair_ok = cid_repairs_usable(nb_decode);
	if (each_repair_ok < 0)
	    continue;
	if (each_repair_ok == 1) {
	    tetrys->max_to_decode = (unsigned long) nb_decode;
	    if (PR_DB_1)
		fprintf(stdout,
			"\ncheck_if_can_decode: EVERYTHING OK ret nb_decode = %d  ",
			nb_decode);
	    return nb_decode;
	}
	if (tetrys->nb_pkt_lost < tetrys->RRBuffer->nbItemStored)
	    printf("\n each_repair_ok %d each_lost_included %d nb_decode %d lost_i %d ",
		   each_repair_ok, each_lost_included, nb_decode, 0);
    }

    tetrys->max_to_decode = 0;
    tetrys->max_red_useful = 0;
    return 0;
#endif
}

/**
 * Hand the @p nb_to_decode packets rebuilt in tetrys->lost_to_recover to the
 * application (OBuffer when delivery is unordered) and to the RDBuffer.
 * @param nb_to_decode  Number of recovered packets stored in lost_to_recover.
 */
static void decode_store_recovered(int nb_to_decode)
{
    int r;
    int idx;
    int padding;

    for (r = 0; r < nb_to_decode; r++) {	/**< Add each of the lost and rebuilt packet in L to both the application and the tetrys datas buffer*/
	tetrys->nb_rebuilt++;
#ifdef STATS
	stat_recover_pkt(tetrys->lost_positions[r]);
#endif
	//OBuffer
	if (tetrys->deliverInOrder == 0) {
	    tetrys->tmp_circ_item =
		circ_add_not_sorted_item_in_buffer(tetrys->OBuffer);
	    if (tetrys->tmp_circ_item != NULL) {
		tetrys->tmp_circ_item->pkt->uint_pktType = PORT_DR;
		memcpy(&(tetrys->tmp_circ_item->pkt)->datav,
		       tetrys->lost_to_recover[r], PLS);
		padding = 0;
		memcpy(&padding, &(tetrys->tmp_circ_item->pkt)->datav, sizeof(int));
		padding = clamp_padding(padding);
		tetrys->tmp_circ_item->pkt->pls = TETRYS_MAX_PAYLOAD - padding;
	    }
	}
	//RDBuffer
	if (PR_DB_2 && tetrys->RDBuffer->lastItem != NULL) {
	    fprintf(stdout,
		    "\ntry_to_decode: recov %lu in RDB with lastref %lu nextExp %lu lastRD %lu ",
		    tetrys->lost_positions[r], tetrys->p_seq_num_of_last_refresh,
		    tetrys->nextExpected, tetrys->RDBuffer->lastItem->pkt->num);
	    fflush(stdout);
	}
	tetrys->tmp_circ_item =
	    circ_add_item_in_buffer(tetrys->lost_positions[r], tetrys->RDBuffer);
	if (tetrys->tmp_circ_item != NULL) {
	    tetrys->tmp_circ_item->pkt->pls = 0;	/**<used to determine wether the pkt were rebuilt or not */
	    memcpy(&(tetrys->tmp_circ_item->pkt)->datav,
		   tetrys->lost_to_recover[r], PLS);
	} else {
	    fprintf(stdout, "\ntry_to_decode: Failed to add");
	    for (idx = 0; idx < nb_to_decode; idx++)
		printf("- l %d p %lu -", idx, tetrys->lost_positions[idx]);
	    fflush(stdout);
	}
    }
}

/**
 * Build and invert the decoding matrix for @p nb_to_decode losses and, when
 * it succeeds, rebuild and deliver the lost packets. On failure the linearly
 * dependent repair is removed from the RRBuffer.
 * @param nb_to_decode  Number of lost packets to decode.
 */
static void decode_losses(int nb_to_decode)
{
    int r;
    int dependent_repair_idx;
    gf stack_mat[64 * 64];
    gf *matrix = (nb_to_decode <= 64) ? stack_mat
	: NEW_GF_MATRIX(nb_to_decode, nb_to_decode);
#ifdef STATS
    stat_decoding_try();
#endif
    if (PR_DB_2)
	test_build_matrix_and_invert();
    /**< If the matrix can be inverted, decode */
    if ((dependent_repair_idx = build_matrix_and_invert(matrix)) == 0) {
#ifdef STATS
	stat_decoding_succ();
#endif
	for (r = 0; r < nb_to_decode; r++)
	    memset(tetrys->lost_to_recover[r], '\0', PLS);
	mul_mat_by_repair(tetrys->RRBuffer, tetrys->lost_to_recover, matrix, nb_to_decode);	/**< Multiply the repair packets by G'. Now, lost_to_recover contains the data of lost packet */
	decode_store_recovered(nb_to_decode);
	auto_refresh_red_recov(tetrys->lost_positions[nb_to_decode - 1]);
	if (PR_DB_2)
	    fprintf(stdout,
		    "\ntry_to_decode: %d l %d r circ_find_from_buffer %lu",
		    tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored,
		    tetrys->max_red_useful);
    } else {
	if (1 != circ_del_item_x_from_buffer(dependent_repair_idx - 1,
					     tetrys->RRBuffer))
	    fprintf(stderr, "decoder: failed to remove the linearly dependent repair packet #%d from the RR buffer\n",
		    dependent_repair_idx - 1);
#ifdef STATS
	stat_decoding_failure();
#endif
    }
    if (nb_to_decode > 64)
	free(matrix);
    get_lost_positions();
}

/**
 * Attempt to recover lost packets using the buffered repair symbols.
 * @return Current number of pending lost packets (0 if none).
 */
static int try_to_decode(void)
{
    int nb_to_decode;

#ifdef STATS
    if (tetrys->nb_pkt_lost > 0)
	stat_rcv_start_loss_event(rcvStats, tetrys);
#endif
#ifdef DECODEINORDER
    if ((tetrys->nb_pkt_lost > 0) && (tetrys->RRBuffer->nbItemStored > 0)
	&& (tetrys->nb_pkt_lost > tetrys->RRBuffer->nbItemStored))
	return 0;
#endif
    /**< First, check both if there are some losses and if they are enought of redundancy to rebuilt the losses */
    if ((tetrys->nb_pkt_lost <= 0) || (tetrys->RRBuffer->nbItemStored <= 0))
	return 0;

    /**< Substract from all the repair in buffer the packet data packets that are in buffer and not substracted yet */
    subtract_all();
    circ_del_to_discard_from_buffer(tetrys->RRBuffer);
    if (PR_DB_1)
	fprintf(stdout, "\ntry_to_decode: %d loss %d red",
		tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored);
    /**< Check if there exists some packet that can be decoded */
    if (PR_DB_2 && tetrys->RRBuffer->firstItem != NULL)
	fprintf(stdout,
		"\ntry_to_decode: %d l %d r -- firstR %lu min %lu max %lu ",
		tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored,
		tetrys->RRBuffer->firstItem->pkt->num,
		tetrys->RRBuffer->firstItem->pkt->min_redundancy,
		tetrys->RRBuffer->firstItem->pkt->max_redundancy);
    if (PR_DB_2)
	circ_print_buffer_stat(tetrys->RRBuffer);
    if ((nb_to_decode = check_if_can_decode()) > 0) {	    /**< Some packet can be decoded up to lost packet tetrys->max_to_decode, tetrys->max_red_useful (the seqnum of the maximum redundancy packet to use). */
	if (PR_DB_2)
	    fprintf(stdout, "\ntry_to_decode: %d l %d r  decode %d",
		    tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored,
		    nb_to_decode);
	decode_losses(nb_to_decode);
    }
    return (int) tetrys->nb_pkt_lost;
}

void tet_end_transfer(void)
{
#ifdef VERBOSE
    printf("\ntet_end_transfer: BEGIN ");
    fflush(stdout);
#endif
    if (tetrys->currentRole != 1)
	return;

    atomic_store(&tetrys->transfer_acked, 0);
    atomic_store(&tetrys->end_transfer_requested, 1);
    kill(getpid(), SIGALRM);

    while (!tetrys_stop_requested && !atomic_load(&tetrys->transfer_acked)) {
	struct timespec ts = { 0, 10000000L };
	nanosleep(&ts, NULL);
    }
    return;
}

/**
 * Check whether all expected packets have been acknowledged.
 * @return 1 if everything is acknowledged, 0 if data remain unacked, -1 on receiver.
 */
static int tet_is_everything_rcvd(void)
{
    if (tetrys->currentRole == 2)
	return -1;
    if (tetrys->SEBuffer->nbItemStored < 1)
	return 1;		// If buffer is empty, al is acked then return true
    else if (tetrys->SEBuffer->lastItem->pkt->num <=
	     (tetrys->max_acked_minus_two + 2))
	return 1;
    else
	return 0;
}

/**
 * Update the sensed input packet rate with the current inter-arrival time.
 */
static void tet_actuate_ipkt_rate(void)
{
    if (tetrys->lastInterPktI_ts == 0.0) {
	tetrys->lastInterPktI_ts = get_time();
	return;
    }
    memmove(&tetrys->interPktIHistory[1], &tetrys->interPktIHistory[0],
	    sizeof(double) * (TETRYS_INTERPKT_HISTORY - 1));
    tetrys->interPktIHistory[0] = get_time() - tetrys->lastInterPktI_ts;
    tetrys->lastInterPktI_ts = get_time();
    double non_zero = 0.0;
    double tot_time = 0.0;
    int i;
    for (i = 0; i < TETRYS_INTERPKT_HISTORY; i++) {
	if (tetrys->interPktIHistory[i] > 0) {
	    non_zero++;
	    tot_time += tetrys->interPktIHistory[i];
	}
    }
    if (tot_time > 0.0 && non_zero > 0.0)
	tetrys->sensedPktIRate = 1.0 / (tot_time / non_zero);
    if (PR_DB_1)
	printf
	    ("\ntet_actuate_ipkt_rate: inter %f nz %f tt %f ainter %f pktRate %f ",
	     tetrys->interPktIHistory[0], non_zero, tot_time,
	     (non_zero > 0.0 ? tot_time / non_zero : 0.0), tetrys->sensedPktIRate);

}

/**
 * Remember that FEC block @p blockid has just been rebuilt (the last 100 are kept).
 * @param blockid  Identifier of the rebuilt FEC block.
 */
static void tet_actuate_last_rebuilt_block(unsigned long int blockid)
{
    memmove(&tetrys->lastRebuiltBlock[1], &tetrys->lastRebuiltBlock[0],
	    sizeof(unsigned long int) * 99);
    tetrys->lastRebuiltBlock[0] = blockid;
    if (tetrys->nbLastRebuiltBlock < 100)
	tetrys->nbLastRebuiltBlock++;
}

/**
 * Check whether FEC block @p blockid is among the recently rebuilt blocks.
 * @param blockid  Identifier of the FEC block to look up.
 * @return 1 if it was, 0 otherwise.
 */
static int tet_is_block_already_rebuilt(unsigned long int blockid)
{
    int i = 0;
    for (i = 0; i < tetrys->nbLastRebuiltBlock; i++) {
	if (tetrys->lastRebuiltBlock[i] == blockid)
	    return 1;
    }
    return 0;
}

/**
 * Update the sensed output packet rate with the current inter-arrival time.
 */
static void tet_actuate_opkt_rate(void)
{
    if (tetrys->lastInterPktO_ts == 0.0) {
	tetrys->lastInterPktO_ts = get_time();
	return;
    }
    memmove(&tetrys->interPktOHistory[1], &tetrys->interPktOHistory[0],
	    sizeof(double) * (TETRYS_INTERPKT_HISTORY - 1));
    tetrys->interPktOHistory[0] = get_time() - tetrys->lastInterPktO_ts;
    tetrys->lastInterPktO_ts = get_time();
    double non_zero = 0.0;
    double tot_time = 0.0;
    int i;
    for (i = 0; i < TETRYS_INTERPKT_HISTORY; i++) {
	if (tetrys->interPktOHistory[i] > 0) {
	    non_zero++;
	    tot_time += tetrys->interPktOHistory[i];
	}
    }
    if (tot_time > 0.0 && non_zero > 0.0)
	tetrys->sensedPktORate = 1.0 / (tot_time / non_zero);
}

/**
 * Update the local loss rate estimate after processing a packet of size _pls.
 * @param _pls  Positive if received data packet, 0 if lost and rebuilt.
 */
static void tet_actuate_loss_rate(int _pls)
{
    if (tetrys->currentRole == 1)
	return;
/**< First of all do the memmove of size of unsigned char */
    memmove(&tetrys->lossHistory[1], &tetrys->lossHistory[0],
	    sizeof(unsigned char) * (TETRYS_LOSS_HISTORY - 1));
    if (_pls > 0)
	tetrys->lossHistory[0] = 1;
    else
	tetrys->lossHistory[0] = 2;

/**< Then compute nbLost; nbTot; */
    float nbLost = 0;
    float nbTot = 0;
    int i = 0;
    for (i = 0; i < TETRYS_LOSS_HISTORY; i++) {
	if (tetrys->lossHistory[i] > 0)
	    nbTot++;
	if (tetrys->lossHistory[i] > 1)
	    nbLost++;
    }

    if (PR_DB_1)
	printf("\ntet_actuate_loss_rate: nbLost %f nbTot %f -- ", nbLost,
	       nbTot);
/**< this gives us the number of packets lost from our last stored history up to next_expted
 * now we need that lost pkt no yet rebuilts; i.e. the one return by int get_lost_positions()
 */
    get_lost_positions();
    unsigned long max_seq_num_seen = 0;
    if (tetrys->RDBuffer->nbItemStored > 0)
	max_seq_num_seen = tetrys->RDBuffer->lastItem->pkt->num;
    if ((tetrys->RRBuffer->nbItemStored > 0)
	&& (tetrys->RRBuffer->lastItem->pkt->max_redundancy >
	    max_seq_num_seen))
	max_seq_num_seen = tetrys->RRBuffer->lastItem->pkt->max_redundancy;

    if (tetrys->nextExpected < max_seq_num_seen)
	nbTot += (float) (max_seq_num_seen - tetrys->nextExpected);
    nbLost += (float) tetrys->nb_pkt_lost;
    if (PR_DB_1)
	printf(" nnbLost %f nnbTot %f nb_pkt_lost %d max %lu nEx %lu",
	       nbLost, nbTot, tetrys->nb_pkt_lost, max_seq_num_seen,
	       tetrys->nextExpected);
    if (nbTot > 0.0f)
	tetrys->lossRate = nbLost / nbTot;
}

/**
 * \fn void send_dat_in_order()
 * \brief Check in RDBuffer if there are some in-sequence packets to deliver.
 * Walks the sorted RDBuffer once with a local cursor so calling
 * tet_actuate_loss_rate() / get_lost_positions() does not reset iteration.
 */
void send_dat_in_order(void)
{
    if (tetrys->fecMode == 1)
	return;
    if (PR_DB_2) {
	printf("\nsend_dat_in_order: ");
	fflush(stdout);
    }
    const struct circBUFFERItem *cur = tetrys->RDBuffer->firstItem;
    while (cur != NULL) {
	if (cur->pkt->num == tetrys->nextExpected) {
	    if (tetrys->deliverInOrder == 1) {
		tetrys->tmp_circ_item =
		    circ_add_not_sorted_item_in_buffer(tetrys->OBuffer);
		if (tetrys->tmp_circ_item != NULL) {
		    tetrys->tmp_circ_item->pkt->uint_pktType = PORT_DR;
		    tetrys->tmp_circ_item->pkt->num = tetrys->nextExpected;
		    memcpy(&(tetrys->tmp_circ_item->pkt)->datav,
			   &(cur->pkt)->datav, PLS);
		    int padding = 0;
		    memcpy(&padding, &(tetrys->tmp_circ_item->pkt)->datav,
			   sizeof(int));
		    padding = clamp_padding(padding);
		    tetrys->tmp_circ_item->pkt->pls =
			TETRYS_MAX_PAYLOAD - padding;
		}
	    }
	    tet_actuate_loss_rate(cur->pkt->pls);
	    tetrys->nextExpected++;
	    cur = cur->next;
	} else if (cur->pkt->num > tetrys->nextExpected) {
	    if (PR_DB_2)
		printf("\nsend_dat_in_order: BREAK %lu > %lu ",
		       cur->pkt->num, tetrys->nextExpected);
	    break;		/* Sorted list */
	} else {
	    cur = cur->next;
	}
    }
    if ((tetrys->RDBuffer->nbItemStored > 1) && (PR_DB_2)) {
	printf
	    ("\n\nsend_dat_in_order: pseqnum %lu nextexp %lu firstNum %lu firstSeqnum %lu \n\n",
	     tetrys->p_seq_num_of_last_refresh, tetrys->nextExpected,
	     tetrys->RDBuffer->firstItem->pkt->num,
	     tetrys->RDBuffer->firstItem->seqnum);
	fflush(stdout);
    }
    if ((tetrys->nextExpected < tetrys->p_seq_num_of_last_refresh)
	&& (PR_DB_2))
	circ_print_buffer_stat(tetrys->RDBuffer);
    if ((tetrys->fecMode == 0)
	&& (tetrys->nextExpected + 50 <
	    tetrys->p_seq_num_of_last_refresh)) {
	fprintf(stderr,
		"\nNode %d send_dat_in_order:ERROR nextexp %lu < %lu ",
		tetrys->nodeid, tetrys->nextExpected + 50,
		tetrys->p_seq_num_of_last_refresh);
	fflush(stderr);
	fflush(stdout);
	circ_print_buffer_stat(tetrys->RDBuffer);
	fflush(stdout);
	fflush(stderr);
	quit(0);			/* exit(0); */
    }
    return;
}

/**
 * Flush in-order FEC packets from the FOBuffer to the OBuffer.
 */
static void tet_try_flush_fo_buffer(void)
{
    if (tetrys->FOBuffer->nbItemStored < 1)
	return;
    int try_again = 1;
    if (PR_DB_1) {
	printf
	    ("\ntet_try_flushFOBuffer: nbInBuff %d nextexpt %lu currWin %lu firsPkt num %lu curW %d minred %lu maxred %lu ",
	     tetrys->FOBuffer->nbItemStored, tetrys->fec_next_expected,
	     tetrys->fec_currentWindow,
	     tetrys->FOBuffer->firstItem->pkt->num,
	     tetrys->FOBuffer->firstItem->pkt->currentWindow,
	     tetrys->FOBuffer->firstItem->pkt->min_redundancy,
	     tetrys->FOBuffer->firstItem->pkt->max_redundancy);
	fflush(stdout);
    }
    while (try_again) {
	try_again = 0;
	unsigned long int curWind =
	    (unsigned long) tetrys->FOBuffer->firstItem->pkt->currentWindow;
	unsigned long int maxExp = tetrys->fec_next_expected;
	if (tetrys->FOBuffer->firstItem->pkt->num == tetrys->fec_next_expected) {	//2.2
	    if (PR_DB_1)
		printf("\ntet_try_flushFOBuffer: %lu av",
		       tetrys->OBuffer->nb_tot_added);
	    circ_add_not_sorted_pdu_in_buffer(tetrys->FOBuffer->firstItem->pkt,
					  tetrys->OBuffer);
	    if (PR_DB_1)
		printf("\ntet_try_flushFOBuffer: %lu ap",
		       tetrys->OBuffer->nb_tot_added);
	    tetrys->nbOfFOBtoOB++;
	    if (PR_DB_1) {
		printf
		    ("\ntet_try_flushFOBuffer: PKT SEND NOW RMV nbInBuff %d nextexpt %lu currWin %lu firsPkt num %lu curW %d minred %lu maxred %lu ",
		     tetrys->FOBuffer->nbItemStored,
		     tetrys->fec_next_expected, tetrys->fec_currentWindow,
		     tetrys->FOBuffer->firstItem->pkt->num,
		     tetrys->FOBuffer->firstItem->pkt->currentWindow,
		     tetrys->FOBuffer->firstItem->pkt->min_redundancy,
		     tetrys->FOBuffer->firstItem->pkt->max_redundancy);
		fflush(stdout);
	    }

	    tetrys->fec_currentWindow =
		(unsigned long) tetrys->FOBuffer->firstItem->pkt->currentWindow;
	    circ_del_item_from_buffer(tetrys->fec_next_expected,
				   tetrys->FOBuffer);
	    tetrys->fec_next_expected++;
	    try_again = 1;
	} else if ( /*(tetrys->FOBuffer->nbItemStored > 50) || */ ((tetrys->RRBuffer->nbItemStored > 0) && (tetrys->fecMaxCurWind > curWind))) {   /**< Si il y a une perte, on attend d'être sure de ne pouvoir reconstruire c.a.d de d'avoir reçu le paquet de redondance de seqnum >  */
	    int initok = circ_init_iterator(tetrys->FOBuffer);
	    while ((initok == 1) && (1 == circ_get_next(tetrys->FOBuffer))) {
		if ((unsigned long) tetrys->FOBuffer->iterator->pkt->currentWindow ==
		    curWind) {
		    circ_add_not_sorted_pdu_in_buffer(tetrys->FOBuffer->iterator->pkt, tetrys->OBuffer);	// Send all old the pkts in buffer
		    if (tetrys->FOBuffer->iterator->pkt->num > maxExp)
			tetrys->nb_tot_fec_lost +=
			    (int) (tetrys->FOBuffer->iterator->pkt->num -
				   maxExp);
		    if (PR_DB_1)
			printf("\ntet_try_flushFOBuffer: %lu af",
			       tetrys->OBuffer->nb_tot_added);
		    tetrys->nbOfFOBtoOB++;
		    maxExp = tetrys->FOBuffer->iterator->pkt->num + 1;
		    if (PR_DB_1) {
			printf
			    ("\ntet_try_flushFOBuffer: PKT SEND NOW toDiscrad nbInBuff %d nextexpt %lu currWin %lu  ",
			     tetrys->FOBuffer->nbItemStored,
			     tetrys->fec_next_expected,
			     tetrys->fec_currentWindow);
			fflush(stdout);
		    }
		    tetrys->FOBuffer->iterator->toDiscard = 1;	// Rmv them from FOBuffer
		}
	    }
	    if (PR_DB_1) {
		printf
		    ("\ntet_try_flushFOBuffer: DISCARD nbInBuff %d nextexpt %lu currWin %lu ",
		     tetrys->FOBuffer->nbItemStored,
		     tetrys->fec_next_expected, tetrys->fec_currentWindow);
		fflush(stdout);
	    }
	    circ_del_to_discard_from_buffer(tetrys->FOBuffer);	// Remove them actually
	    tetrys->fec_next_expected = maxExp;
	    tetrys->fec_currentWindow++;
	    try_again = 1;	// To send the current pkt if it is the first one of its block
	}
	if (tetrys->FOBuffer->nbItemStored < 1)
	    try_again = 0;
    }
    if (PR_DB_1) {
	printf
	    ("\ntet_try_flushFOBuffer: END nbInBuff %d nextexpt %lu currWin %lu ",
	     tetrys->FOBuffer->nbItemStored, tetrys->fec_next_expected,
	     tetrys->fec_currentWindow);
	fflush(stdout);
    }
    return;
}

/** Maximum number of data or repair packets tracked per FEC block. */
#define FEC_MAX_PKT_PER_BLOCK 100

/**
 * Discard packets older than 20 coding windows from the RDBuffer and mark
 * the old repairs of the RRBuffer as "to discard", then list the distinct
 * block ids found in the RRBuffer into tetrys->blockId.
 * @param[out] nb_block_id  Number of distinct block ids found.
 * @return 0 on success, -1 if a buffer could not be iterated.
 */
static int fec_list_blocks(int *nb_block_id)
{
    int i;

    *nb_block_id = 0;
    if (1 != circ_init_iterator(tetrys->RRBuffer)) {
	fprintf(stderr,
		"\ntry_to_decode_FecMode: circ_initIterator1  %d failed ! -- return",
		tetrys->RRBuffer->nbItemStored);
	return -1;
    }
    while (1 == circ_get_next(tetrys->RRBuffer)) {
	if ((tetrys->RRBuffer->iterator->pkt->currentWindow + 20) <
	    tetrys->RRBuffer->lastItem->pkt->currentWindow)
	    tetrys->RRBuffer->iterator->toDiscard = 1;
	int already_seen = 0;
	for (i = 0; i < *nb_block_id; i++)
	    if (tetrys->blockId[i] ==
		(unsigned long) tetrys->RRBuffer->iterator->pkt->currentWindow)
		already_seen = 1;
	if (already_seen == 0 && *nb_block_id < 600) {
	    tetrys->blockId[*nb_block_id] =
		(unsigned long) tetrys->RRBuffer->iterator->pkt->currentWindow;
	    (*nb_block_id)++;
	}
    }
    return 0;
}

/**
 * Collect the sequence numbers of the repair and data packets that belong to
 * the FEC block @p block_id.
 * @param block_id       Coding window identifying the block.
 * @param[out] rep_seq   Sequence numbers of the repairs of the block.
 * @param[out] nb_rep    Number of repairs found.
 * @param[out] nb_expec  Number of packets expected in the block.
 * @param[out] dat_seq   Sequence numbers of the data packets of the block.
 * @param[out] nb_dat    Number of data packets found.
 * @return 0 on success, -1 if a buffer could not be iterated.
 */
static int fec_collect_block(unsigned long block_id,
			     unsigned long rep_seq[], int *nb_rep,
			     int *nb_expec,
			     unsigned long dat_seq[], int *nb_dat)
{
    *nb_rep = 0;
    *nb_dat = 0;
    *nb_expec = 0;
    if (1 != circ_init_iterator(tetrys->RRBuffer)) {
	fprintf(stderr,
		"\ntry_to_decode_FecMode: circ_initIterator2 %d failed ! -- return",
		tetrys->RRBuffer->nbItemStored);
	return -1;
    }
    while (1 == circ_get_next(tetrys->RRBuffer)) {
	const struct PDU *pkt = tetrys->RRBuffer->iterator->pkt;
	if (block_id == (unsigned long) pkt->currentWindow
	    && *nb_rep < FEC_MAX_PKT_PER_BLOCK) {
	    rep_seq[(*nb_rep)++] = pkt->num;
	    *nb_expec = (int) (pkt->max_redundancy + 1 - pkt->min_redundancy);
	}
    }
    if (1 != circ_init_iterator(tetrys->RDBuffer)) {
	fprintf(stderr,
		"\ntry_to_decode_FecMode: circ_initIterator3  %d failed ! -- return",
		tetrys->RDBuffer->nbItemStored);
	return -1;
    }
    while (1 == circ_get_next(tetrys->RDBuffer)) {
	const struct PDU *pkt = tetrys->RDBuffer->iterator->pkt;
	if (block_id == (unsigned long) pkt->currentWindow
	    && *nb_dat < FEC_MAX_PKT_PER_BLOCK)
	    dat_seq[(*nb_dat)++] = pkt->num;
    }
    return 0;
}

/**
 * Move the packets @p seq[0..nb-1] from @p from to the temporary buffer @p to.
 * Stops silently (after a message) at the first packet that cannot be moved.
 * @param seq   Array of sequence numbers to move.
 * @param nb    Number of entries in @p seq.
 * @param from  Source queue.
 * @param to    Destination temporary queue.
 */
static void fec_move_to_tmp(const unsigned long seq[], int nb,
			    struct circBUFFER *from, struct circBUFFER *to)
{
    int j;
    const struct PDU *pdu;

    for (j = 0; j < nb; j++) {
	pdu = circ_find_from_buffer(seq[j], from);
	if (pdu == NULL) {
	    fprintf(stderr,
		    "\ntry_to_decode_FecMode:  circ_findFromBuffer return NULL ! -- break");
	    break;
	}
	if (NULL == circ_add_pdu_in_buffer(pdu, to)) {
	    // e.g. if failed to add pkt to the temporary buffer, abort for this block !
	    fprintf(stderr,
		    "\ntry_to_decode_FecMode: circ_add_pdu_in_buffer failed ! -- break");
	    break;
	}
	circ_del_item_from_buffer(seq[j], from);
    }
}

/**
 * Hand the @p nb_loss packets rebuilt in tetrys->lost_to_recover to the
 * application (OBuffer, or FOBuffer when in-order delivery is requested).
 * @param nb_loss   Number of rebuilt packets.
 * @param block_id  Coding window of the block just decoded.
 */
static void fec_deliver_recovered(int nb_loss, unsigned long block_id)
{
    int r;
    int padding;

    for (r = 0; r < nb_loss; r++) {
	/**< Add each of the lost and rebuilt packet in L to both the application and the tetrys datas buffer*/
	tetrys->nb_rebuilt++;
	if (PR_DB_1) {
	    fprintf(stdout,
		    "\ntry_to_decode_FecMode: recover pkt %lu pos %d",
		    tetrys->lost_positions[r], r);
	    fflush(stdout);
	}
	if (tetrys->deliverInOrder == 0)
	    tetrys->tmp_circ_item =
		circ_add_not_sorted_item_in_buffer(tetrys->OBuffer);
	else
	    tetrys->tmp_circ_item =
		circ_add_item_in_buffer(tetrys->lost_positions[r],
					tetrys->FOBuffer);
	if (tetrys->tmp_circ_item == NULL || tetrys->tmpFecRep->firstItem == NULL)
	    continue;
	struct PDU *pkt = tetrys->tmp_circ_item->pkt;
	pkt->num = tetrys->lost_positions[r];
	pkt->min_redundancy = tetrys->tmpFecRep->firstItem->pkt->min_redundancy;
	pkt->max_redundancy = tetrys->tmpFecRep->firstItem->pkt->max_redundancy;
	pkt->currentWindow = (int) block_id;
	pkt->uint_pktType = PORT_DR;
	pkt->local_rtt_probe = get_time();
	memcpy(&pkt->datav, tetrys->lost_to_recover[r], PLS);
	padding = 0;
	memcpy(&padding, &pkt->datav, sizeof(int));
	padding = clamp_padding(padding);
	pkt->pls = TETRYS_MAX_PAYLOAD - padding;

	if (tetrys->deliverInOrder == 1)
	    tet_try_flush_fo_buffer();
    }
}

/**
 * Try to decode one FEC block whose packets have been gathered.
 * @param block_id  Coding window identifier.
 * @param dat_seq   Sequence numbers of data packets in the block.
 * @param nb_dat    Number of data packets available.
 * @param rep_seq   Sequence numbers of repair packets in the block.
 * @param nb_rep    Number of repair packets available.
 * @param nb_expec  Expected number of data packets in the block.
 * @return 1 when the caller must stop scanning the remaining blocks
 *         (nothing to recover, or inversion failure), 0 otherwise.
 */
static int fec_decode_block(unsigned long block_id,
			    const unsigned long dat_seq[], int nb_dat,
			    const unsigned long rep_seq[], int nb_rep,
			    int nb_expec)
{
    int r;
    int nb_loss = nb_expec - nb_dat;
    gf stack_mat[64 * 64];
    gf *matrix;

    circ_empty_queue(tetrys->tmpFecDat);
    circ_empty_queue(tetrys->tmpFecRep);
    if (PR_DB_1)
	printf("\ntry_to_decode_FecMode: TREAT block %lu ", block_id);
    fec_move_to_tmp(dat_seq, nb_dat, tetrys->RDBuffer, tetrys->tmpFecDat);
    fec_move_to_tmp(rep_seq, nb_rep, tetrys->RRBuffer, tetrys->tmpFecRep);
    tet_actuate_last_rebuilt_block(block_id);
    if (nb_dat >= nb_expec || nb_loss <= 0)
	return 1;
    /* Subtract data from repair symbols (step 1.1). */
    subtract_all_fec();
    /* Build and invert the decoding matrix (step 1.2). */
    get_fec_lost_positions();
    matrix = (nb_loss <= 64) ? stack_mat : NEW_GF_MATRIX(nb_loss, nb_loss);
    if (0 != build_fec_matrix_and_invert(matrix, nb_loss)) {
	fprintf(stderr,
		"\ntry_to_decode_FecMode: build_fec_matrix_and_invert failed to invert ! -- break --> lost pkts from this block are definitly lost");
	if (nb_loss > 64)
	    free(matrix);
	return 1;
    }
    /* Recover lost packets from the repair symbols. */
    for (r = 0; r < nb_loss; r++)
	memset(tetrys->lost_to_recover[r], '\0', PLS);
    mul_mat_by_repair(tetrys->tmpFecRep, tetrys->lost_to_recover, matrix, nb_loss);
    /* lost_to_recover now holds the reconstructed data for each lost packet. */
    fec_deliver_recovered(nb_loss, block_id);
    if (PR_DB_1) {
	fprintf(stdout, "\ntry_to_decode_FecMode: %d loss recovered ", nb_loss);
	fflush(stdout);
    }
    if (nb_loss > 64)
	free(matrix);
    // then it's ok, flush tmp buffers
    circ_empty_queue(tetrys->tmpFecDat);
    circ_empty_queue(tetrys->tmpFecRep);
    if (PR_DB_1) {
	fprintf(stdout, "\ntry_to_decode_FecMode: tmpFecDat tmpFecRep emptyied ");
	fflush(stdout);
    }
    return 0;
}

/**
 * Remove packets from the RRBuffer and RDBuffer once they are almost full.
 * As the buffers are sorted, popping the first item removes the oldest one.
 */
static void fec_trim_buffers(void)
{
    while ((tetrys->RRBuffer->nbItemStored + 10) >
	   tetrys->RRBuffer->nbItemStorable) {
	circ_pop_first_pdu_from_buffer(&tetrys->tmp_pdu, tetrys->RRBuffer);
	printf("\ntry_to_decode_FecMode: END DELREPAIR pkt %lu  cur wind  %d RRBuffer ",
	       tetrys->tmp_pdu.num, tetrys->tmp_pdu.currentWindow);
    }
    while ((tetrys->RDBuffer->nbItemStored + 10) >
	   tetrys->RDBuffer->nbItemStorable) {
	circ_pop_first_pdu_from_buffer(&tetrys->tmp_pdu, tetrys->RDBuffer);
	printf("\ntry_to_decode_FecMode: END DELDATA pkt %lu  cur wind  %d RDBuffer ",
	       tetrys->tmp_pdu.num, tetrys->tmp_pdu.currentWindow);
    }
}

/**
 * FEC block-mode decoding: attempt to recover lost packets within the
 * current FEC encoding windows and trim old blocks.
 */
static void try_to_decode_fec_mode(void)
{
    int nb_block_id;
    int i;
    int nb_rep, nb_dat, nb_expec;
    unsigned long int dat_seq[FEC_MAX_PKT_PER_BLOCK];
    unsigned long int rep_seq[FEC_MAX_PKT_PER_BLOCK];

    memset(&tetrys->blockId, '\0', sizeof(unsigned long int) * 600);
    if (tetrys->RRBuffer->nbItemStored < 1 || tetrys->RDBuffer->nbItemStored < 1)
	return;

/**< Clean the RRBuffer -- pkt were marked as to be discarded by last call */
    circ_del_to_discard_from_buffer(tetrys->RRBuffer);
/**< Clean the RDBuffer */
    if (1 == circ_init_iterator(tetrys->RDBuffer)) {
	while (1 == circ_get_next(tetrys->RDBuffer)) {
	    if ((tetrys->RDBuffer->iterator->pkt->currentWindow + 20) <
		tetrys->RDBuffer->lastItem->pkt->currentWindow)
		tetrys->RDBuffer->iterator->toDiscard = 1;
	}
    }
    circ_del_to_discard_from_buffer(tetrys->RDBuffer);

    if (tetrys->RRBuffer->nbItemStored < 1 || tetrys->RDBuffer->nbItemStored < 1)
	return;

/**< Scan the different block */
    if (fec_list_blocks(&nb_block_id) != 0)
	return;
    if (PR_DB_1)
	printf("\ntry_to_decode_FecMode: will treat %d blockId", nb_block_id);

    /* For each block ID, attempt decoding. */
    for (i = 0; i < nb_block_id; i++) {
	if (fec_collect_block(tetrys->blockId[i], rep_seq, &nb_rep,
			      &nb_expec, dat_seq, &nb_dat) != 0)
	    return;
	if (PR_DB_1)
	    printf("\ntry_to_decode_FecMode: bid %lu got %d rep and %d dat for %d expected",
		   tetrys->blockId[i], nb_rep, nb_dat, nb_expec);
	if (nb_rep + nb_dat >= nb_expec &&
	    fec_decode_block(tetrys->blockId[i], dat_seq, nb_dat,
			     rep_seq, nb_rep, nb_expec))
	    break;
    }

    fec_trim_buffers();
    if (PR_DB_1) {
	fprintf(stdout, "\ntry_to_decode_FecMode: FIN");
	fflush(stdout);
    }
}

/**
 * Process a data packet (TD / TAD) popped from the TIBuffer: update the
 * receiver state, queue it for delivery and in the RDBuffer, then refresh
 * the state when no loss is pending.
 */
static void ti_treat_data(void)
{
    auto_rcv_dat(&tetrys->tmp_pdu);	/**< actuate the states variables for datas pkts */
    if (PR_DB_1) {
	printf("\ntreat_ti_buffer: DAT %d -> %d num %lu lastref %lu currentWindow %d nextExp %lu nbr %d nbrd %d",
	       tetrys->tmp_pdu.uint_pktType, PORT_DR, tetrys->tmp_pdu.num,
	       tetrys->p_seq_num_of_last_refresh,
	       tetrys->tmp_pdu.currentWindow, tetrys->nextExpected,
	       tetrys->RRBuffer->nbItemStored, tetrys->RDBuffer->nbItemStored);
	fflush(stdout);
    }
    tetrys->tmp_pdu.uint_pktType = PORT_DR;

    if ((tetrys->deliverInOrder == 0)
	&& (tetrys->tmp_pdu.num >= tetrys->nextExpected))
	circ_add_not_sorted_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->OBuffer);	/**< Send data to the app Buffer asap */
    else if ((tetrys->fecMode == 1)
	     && (0 == tet_is_block_already_rebuilt((unsigned long) tetrys->tmp_pdu.currentWindow))) {
	tetrys->tmp_circ_item =
	    circ_add_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->FOBuffer);
	if (tetrys->tmp_circ_item != NULL) {
	    tetrys->tmp_circ_item->pkt->local_rtt_probe = get_time();
	    tet_try_flush_fo_buffer();
	}
    }

    if (tetrys->tmp_pdu.num >= tetrys->nextExpected)
	circ_add_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->RDBuffer);	/**< and Add data to the dataQueue */
    else if (PR_DB_1)
	printf("\ntreatTIBuffer: RCVD OLD Dat %lu last ref %lu nb LOSS %d REP %d",
	       tetrys->tmp_pdu.num, tetrys->p_seq_num_of_last_refresh,
	       tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored);
    if (tetrys->fecMode == 0)
	get_lost_positions();	/**< Actuate tetrys->nb_pkt_lost */
    if (PR_DB_1)
	printf("\ntreatTIBuffer: Dat %lu nb LOSS %d REP %d",
	       tetrys->tmp_pdu.num, tetrys->nb_pkt_lost,
	       tetrys->RRBuffer->nbItemStored);
    if (tetrys->nb_pkt_lost == 0)
	auto_refresh_dat_nolosses();
}

/**
 * Compute the sequence number below which the RDBuffer can be purged after
 * the reception of the repair currently held in tetrys->tmp_pdu.
 * @return The purge threshold (0 if nothing should be removed).
 */
static int ti_repair_purge_threshold(void)
{
    int rmv_from;

    if (tetrys->tmp_pdu.min_redundancy >= tetrys->p_seq_num_of_last_refresh)
	rmv_from = (int) tetrys->p_seq_num_of_last_refresh;
    else
	rmv_from = (int) tetrys->tmp_pdu.min_redundancy;
    if (rmv_from > 2)
	rmv_from = rmv_from - 2;
    else
	rmv_from = 0;
    if (tetrys->RRBuffer->nbItemStored > 0
	&& tetrys->RRBuffer->firstItem->pkt->min_redundancy <
	(unsigned long) rmv_from)
	rmv_from = (int) tetrys->RRBuffer->firstItem->pkt->min_redundancy;
    return rmv_from;
}

/**
 * Process a repair packet (TR / TAR) popped from the TIBuffer.
 * @return 1 if the packet belongs to an already rebuilt FEC block and was
 *         dropped (the caller must stop processing), 0 otherwise.
 */
static int ti_treat_repair(void)
{
    int rmv_from;

    auto_rcv_red(&tetrys->tmp_pdu);	/**< actuate the states variables for datas pkts */

    if (PR_DB_1) {
	printf("\ntreat_ti_buffer: Repair %d seqnum %lu min %lu max %lu cw %d  nextExpected %lu nbr %d nbrd %d",
	       tetrys->tmp_pdu.uint_pktType, tetrys->tmp_pdu.num,
	       tetrys->tmp_pdu.min_redundancy, tetrys->tmp_pdu.max_redundancy,
	       tetrys->tmp_pdu.currentWindow, tetrys->nextExpected,
	       tetrys->RRBuffer->nbItemStored, tetrys->RDBuffer->nbItemStored);
	fflush(stdout);
    }

    if (tetrys->fecMode == 1) {
	if (tetrys->fecMaxCurWind < (unsigned long) tetrys->tmp_pdu.currentWindow)
	    tetrys->fecMaxCurWind = (unsigned long) tetrys->tmp_pdu.currentWindow;
	if (1 == tet_is_block_already_rebuilt((unsigned long) tetrys->tmp_pdu.currentWindow)) {
	    if (PR_DB_1)
		printf("\ntreatTIBuffer: DROP block %d -- pkt %d seqnum %lu min %lu max %lu ",
		       tetrys->tmp_pdu.currentWindow,
		       tetrys->tmp_pdu.uint_pktType, tetrys->tmp_pdu.num,
		       tetrys->tmp_pdu.min_redundancy,
		       tetrys->tmp_pdu.max_redundancy);
	    return 1;
	}
    }

    tetrys->tmp_circ_item = NULL;
    if ((tetrys->tmp_pdu.min_redundancy >= tetrys->min_data_pkt_to_retain)
	&& (tetrys->tmp_pdu.max_redundancy >= tetrys->p_seq_num_of_last_refresh)) {
	tetrys->tmp_circ_item =
	    circ_add_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->RRBuffer);	/**< and Add data to the repairQueue */
	if (tetrys->tmp_circ_item == NULL)
	    printf("\ntreatTIBuffer: ITEM NULL block %d -- pkt %d seqnum %lu min %lu max %lu ",
		   tetrys->tmp_pdu.currentWindow, tetrys->tmp_pdu.uint_pktType,
		   tetrys->tmp_pdu.num, tetrys->tmp_pdu.min_redundancy,
		   tetrys->tmp_pdu.max_redundancy);
    }
    if ((tetrys->tmp_circ_item != NULL) && PR_DB_2)
	print_bitvector(tetrys->tmp_circ_item);

    if (tetrys->fecMode == 0)
	get_lost_positions();	/**< Actuate tetrys->nb_pkt_lost */
    rmv_from = ti_repair_purge_threshold();
    if (PR_DB_1) {
	printf("\ntreat_ti_buffer: Rep %lu nb LOSS %d redQue %d minRed %lu maxRed %lu del pdu in RD from %lu RMVFROM %d psnolr %lu",
	       tetrys->tmp_pdu.num, tetrys->nb_pkt_lost,
	       tetrys->RRBuffer->nbItemStored, tetrys->tmp_pdu.min_redundancy,
	       tetrys->tmp_pdu.max_redundancy, tetrys->tmp_pdu.min_redundancy,
	       rmv_from, tetrys->p_seq_num_of_last_refresh);
	fflush(stdout);
    }
    if ((tetrys->fecMode == 1) && (tetrys->deliverInOrder == 1))
	tet_try_flush_fo_buffer();
    else
	send_dat_in_order();
    if ((tetrys->fecMode == 0) && (rmv_from > 0)) {
	/*
	 * Never purge a packet that was received but not delivered yet: with
	 * in-order delivery nextExpected only advances in send_dat_in_order(),
	 * so anything at or above it is still waiting for a hole to be filled
	 * and deleting it would lose it for good.
	 */
	if (tetrys->deliverInOrder == 1
	    && (unsigned long) rmv_from > tetrys->nextExpected) {
#ifdef VERBOSE
	    fprintf(stderr,
		    "decoder: purge limited to packet #%lu: packets #%lu..#%d are received but not delivered yet (waiting for a missing packet)\n",
		    tetrys->nextExpected, tetrys->nextExpected, rmv_from - 1);
#endif
	    rmv_from = (int) tetrys->nextExpected;
	}
	circ_del_item_below_value(tetrys->RDBuffer, (unsigned long) rmv_from);
    }

    if (tetrys->nb_pkt_lost == 0)
	auto_refresh_red_nolosses();
    if ((tetrys->fecMode == 0)
	&& ((tetrys->tmp_pdu.num % tetrys->freqTryToDecode) == 0))
	try_to_decode();
    return 0;
}

/**
 * \fn void treat_ti_buffer ()
 * \brief Check if the buffer is empty, if not, pop the oldest item and perform the adapted process..
 *	  i.e.: 1. (the acknowledgement part is already treated)
 *	  	2. If repair: Add it the set of repair and trigger the repair process
 *	  	3. If data: Add it the set of data and trigger the data process
 */
void treat_ti_buffer(void)
{
    int repcirc;

    if (PR_DB_1)
	printf("\ntreatTIBuffer: BEGIN ");
    if (tetrys->TIBuffer->nbItemStored == 0)
	return;
    memset(&tetrys->tmp_pdu, '\0', sizeof(struct PDU));
    repcirc = circ_pop_first_pdu_from_buffer(&tetrys->tmp_pdu, tetrys->TIBuffer);
    if (1 != repcirc)	// ret 1 if ok, 0 if failed
	return;

    auto_rcv_pkt(&tetrys->tmp_pdu);
    if ((tetrys->tmp_pdu.uint_pktType == PORT_TAD)
	|| (tetrys->tmp_pdu.uint_pktType == PORT_TD)) {
	ti_treat_data();
    } else if ((tetrys->tmp_pdu.uint_pktType == PORT_TAR)
	       || (tetrys->tmp_pdu.uint_pktType == PORT_TR)) {
	if (ti_treat_repair())
	    return;
    }
    if (tetrys->fecMode == 1)
	try_to_decode_fec_mode();
}

/** Set the ack field of tetrys->tmp_pdu from the last refresh point. */
static void di_set_ack_min_redundancy(void)
{
    if (tetrys->p_seq_num_of_last_refresh > 1)
	tetrys->tmp_pdu.ack_min_redundancy =
	    tetrys->p_seq_num_of_last_refresh - 2;
    else
	tetrys->tmp_pdu.ack_min_redundancy = 0;
}

/**
 * Fill the coding-window fields of tetrys->tmp_pdu (sender or both roles).
 * Also flags a repair as pending when enough data was sent since the last one.
 */
static void di_set_encoding_window(void)
{
    tetrys->max_seq_in_red = tetrys->SEBuffer->nb_tot_added;
    tetrys->tmp_pdu.min_redundancy = tetrys->min_seq_in_red;
    if (tetrys->fecMode == 1)
	tetrys->tmp_pdu.max_redundancy = tetrys->min_seq_in_red + (unsigned long) tetrys->fec_n;
    else
	tetrys->tmp_pdu.max_redundancy = tetrys->max_seq_in_red;
    tetrys->tmp_pdu.currentWindow = tetrys->currentWindow;	       /**< For FEC mode only, 0 otherwise */
    tetrys->tmp_pdu.currentPktInWindow = tetrys->currentPktInWindow;	 /**< For FEC mode only, 0 otherwise */
    if (tetrys->snd_pkt_since_last_repair >= tetrys->fec_n)
	tetrys->repair_pending = 1;
#ifdef STATS
    stat_data_set_stats(&tetrys->tmp_pdu, sndStats);
#endif
}

/**
 * Emit the repairs required after a data packet was queued: one repair in
 * Tetrys mode when pending, or the repairs of the current block in FEC mode.
 */
static void di_send_repairs(void)
{
    if (tetrys->fecMode == 0) {	/**< Act as a normal tetrys */
	if (PR_DB_2)
	    fprintf(stdout, "ActAsTetrys");
	if (tetrys->repair_pending)
	    send_repair();
	return;
    }
    /**< Act as FEC */
    while ((tetrys->currentPktInWindow >= tetrys->fec_n)
	   && (tetrys->currentPktInWindow < tetrys->fec_k)) {
	send_repair();
	tetrys->repair_pending = 1;
	tetrys->currentPktInWindow++;
    }
    if (tetrys->currentPktInWindow >= tetrys->fec_k) {
	tetrys->currentPktInWindow = (-1);	/**< Reset the mechanism **/
	tetrys->currentWindow++;
	tetrys->repair_pending = 0;
	tetrys->min_seq_in_red = tetrys->SEBuffer->nb_tot_added;
	circ_empty_queue(tetrys->SEBuffer);
    }
    tetrys->currentPktInWindow++;
    if (PR_DB_2)
	printf("ActAsFec currentWindow %d currentPktInWind %d fec_n %d fec_k %d",
	       tetrys->currentWindow, tetrys->currentPktInWindow,
	       tetrys->fec_n, tetrys->fec_k);
}

/**
 * Add the data packet held in tetrys->tmp_pdu to the encoding window
 * (SEBuffer) and to the output buffer, then send the repairs if needed.
 */
static void di_encode_and_queue(void)
{
    tetrys->tmp_pdu.num = tetrys->SEBuffer->nb_tot_added;
    tetrys->tmp_circ_item = circ_add_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->SEBuffer);
    tetrys->snd_pkt_since_last_repair++;
    /**< Then add it to the encoding windows, and to the Output buffer */
    if (PR_DB_2)
	printf("\ntreatDIBuffer: ---- add pkt type %d num %lu to OBuffer minred %lu maxred %lu cur %d wind %d tetcur %d ",
	       tetrys->tmp_pdu.uint_pktType, tetrys->tmp_pdu.num,
	       tetrys->tmp_pdu.min_redundancy, tetrys->tmp_pdu.max_redundancy,
	       tetrys->tmp_pdu.currentPktInWindow,
	       tetrys->tmp_pdu.currentWindow, tetrys->currentWindow);

    di_set_ack_min_redundancy();
    tetrys->tmp_pdu.lossRate = tetrys->lossRate;
    tetrys->tmp_circ_item =
	circ_add_not_sorted_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->OBuffer);
    di_send_repairs();
}

/**
 * Pop one application packet from the DIBuffer and process it according to
 * the role: data (sender / both) or acknowledgement (receiver).
 */
void treat_di_buffer(void)
{
    if (tetrys->DIBuffer->nbItemStored == 0)
	return;
    memset(&tetrys->tmp_pdu, '\0', sizeof(struct PDU));
    if (0 == circ_pop_first_pdu_from_buffer(&tetrys->tmp_pdu, tetrys->DIBuffer))
	return;			//ret 0 if failed

    tetrys->tmp_pdu.lossRate = tetrys->lossRate;
    if (tetrys->currentRole == 2 || tetrys->currentRole == 0) {	/**< TAP, TAD */
	di_set_ack_min_redundancy();
	if (PR_DB_1)
	    fprintf(stdout, "\ntreatDIBuffer: send ack %lu ",
		    tetrys->tmp_pdu.ack_min_redundancy);
    }
    if (tetrys->currentRole < 2)	/**< TD, TAD */
	di_set_encoding_window();
    if (tetrys->currentRole == 0) {	/**< Both sender and receiver DI packet are DATA that need to be added to the encoding windows and sent as data. The acknoledgement will be biggybacked in the header */
	tetrys->tmp_pdu.uint_pktType = PORT_TAD;
	di_set_ack_min_redundancy();
	if (PR_DB_1)
	    fprintf(stdout, "\ntreatDIBuffer: send ack %lu ",
		    tetrys->tmp_pdu.ack_min_redundancy);
	actuate_alarm(2);	/**< Reset the rep timer */
    } else if (tetrys->currentRole == 1) {	/**< Sender only.. DI packet are data only, they have to be included in the source buffer and sent as data. They wont contain acknowledgement datas (it would'nt make sense ) */
	tetrys->tmp_pdu.uint_pktType = PORT_TD;
    } else if (tetrys->currentRole == 2) {	/**< Receiver only DI packets are Ack only (  */
	// Produce ack packet
	tetrys->tmp_pdu.uint_pktType = PORT_TAP;
	di_set_ack_min_redundancy();
	if (PR_DB_1)
	    fprintf(stdout, "\ntreatDIBuffer: send ack %lu ",
		    tetrys->tmp_pdu.ack_min_redundancy);
	/**< Then add it to the encoding windows, and to the Output buffer */
	if (PR_DB_2)
	    fprintf(stdout, "\ntreatDIBuffer: add pkt type %d to OBuffer ",
		    tetrys->tmp_pdu.uint_pktType);
	tetrys->tmp_circ_item =
	    circ_add_not_sorted_pdu_in_buffer(&tetrys->tmp_pdu, tetrys->OBuffer);
	actuate_alarm(1);	/**< Reset the ack timer */
    }
    if (tetrys->currentRole < 2)	/**< TD, TAD */
	di_encode_and_queue();
}

/**
 * \fn void treat_o_buffer ()
 * \brief Check if the buffer is empty, if not, pop the oldest item and perform the adapted process..
 *	  For packet type in (PORT_TD, PORT_TA ....):
 *		if there is one fd in out_fun_tab that match packet type:
 *		match the packet source, if matched:
 *	  	Call the function with the packet to send
 */
void treat_o_buffer(void)
{
    if (PR_DB_1)
	fprintf(stdout, "\ntreatOBuffer: Begin");
    while (tetrys->OBuffer->nbItemStored > 0) {	/*todel */
	memset(&tetrys->tmp_pdu, '\0', sizeof(struct PDU));
	if (PR_DB_2)
	    fprintf(stdout, "\ntreatOBuffer: POP (among %d pkts) ",
		    tetrys->OBuffer->nbItemStored);
	if (0 ==
	    circ_pop_first_pdu_from_buffer(&tetrys->tmp_pdu, tetrys->OBuffer))
	    return;		//ret 0 if failed
	if ((tetrys->tmp_pdu.uint_pktType < 660)
	    || (tetrys->tmp_pdu.uint_pktType > 670)
	    || (tetrys->tmp_pdu.pls < 0) || (tetrys->tmp_pdu.pls > PLS)) {
	    fprintf(stderr,
		    "\ntreatOBuffer: Return cause inconcistant pkt_pdu %d pls %d num %lu min %lu max %lu cw %d ",
		    tetrys->tmp_pdu.uint_pktType, tetrys->tmp_pdu.pls,
		    tetrys->tmp_pdu.num, tetrys->tmp_pdu.min_redundancy,
		    tetrys->tmp_pdu.max_redundancy,
		    tetrys->tmp_pdu.currentWindow);
	    fflush(stderr);
	    tetrys->nbOBFailed++;
	    quit(0);		/*return; */
	}
	/**< Search for the right function that match the pkt type */
	int fd_max_tmp = 0;

	set_rtt_stuff(&tetrys->tmp_pdu);
#ifdef SAVEPACKET
	write_packet('s', &tetrys->tmp_pdu);
#endif
	if (sig_AckRed->endAt > 0.0) {
	    tetrys->tmp_pdu.local_rtt_probe = -1;
	    tetrys->tmp_pdu.dst_rtt_probe = -1;
	    tetrys->tmp_pdu.rtt_estimation = -1;
	    tetrys->tmp_pdu.lossRate = -1;
	}
	if (PR_DB_2)
	    fprintf(stdout, "\ntreatOBuffer: before while out_fun");
	tetrys->tmp_pdu.nodeid = tetrys->nodeid;
	while (fd_max_tmp < tetrys->nb_out_fun) {
	    if (PR_DB_2)
		fprintf(stdout,
			"\n\ntreatOBuffer: test fun %d type %d for pkt type %d of pls %d num %lu  min %lu max %lu cw %d ",
			fd_max_tmp,
			tetrys->out_fun_tab[fd_max_tmp].pkttype,
			tetrys->tmp_pdu.uint_pktType, tetrys->tmp_pdu.pls,
			tetrys->tmp_pdu.num,
			tetrys->tmp_pdu.min_redundancy,
			tetrys->tmp_pdu.max_redundancy,
			tetrys->tmp_pdu.currentWindow);
	    if (tetrys->out_fun_tab[fd_max_tmp].pkttype ==
		tetrys->tmp_pdu.uint_pktType) {
		tetrys->out_fun_tab[fd_max_tmp].out_fun(&tetrys->tmp_pdu);
		fd_max_tmp = 100;
	    }
	    fd_max_tmp++;
	}
	if (fd_max_tmp < 100)
	    fprintf(stdout,
		    "treat_o_buffer: pkt type %d will be dropped cause it have no output handler",
		    tetrys->tmp_pdu.uint_pktType);
	if (PR_DB_1) {
	    printf("\ntreatOBuffer: END nbStored %d nbTotAdded %lu ",
		   tetrys->OBuffer->nbItemStored,
		   tetrys->OBuffer->nb_tot_added);
	    fflush(stdout);
	}
    }				/*todel  */
    return;
}



/** Build and enqueue a repair packet covering the current encoding window. */
static void send_repair(void)
{
    if (PR_DB_1)
	fprintf(stdout, "\nsendRepair: SE %d minseq %lu maxseq %lu ",
		tetrys->SEBuffer->nbItemStored, tetrys->min_seq_in_red,
		tetrys->max_seq_in_red);
    if ((tetrys->SEBuffer->nbItemStored > 0)
	&& (tetrys->min_seq_in_red < tetrys->max_seq_in_red)) {
	memset(&tetrys->tmp_rep_pdu, '\0', sizeof(struct PDU));	//CLEAN ADDED Fri Dec 18 TODO ATTENTION
	tetrys->tmp_rep_pdu.min_redundancy = tetrys->min_seq_in_red;
	tetrys->tmp_rep_pdu.max_redundancy = tetrys->max_seq_in_red;
	tetrys->tmp_rep_pdu.repair_id = tetrys->nb_rep_sent;
	tetrys->snd_pkt_since_last_repair = 0;
	tetrys->repair_pending = 0;
	tetrys->tmp_rep_pdu.num = tetrys->tmp_rep_pdu.repair_id;
	if (tetrys->currentRole == 0) {
	    tetrys->tmp_rep_pdu.uint_pktType = PORT_TAR;
	    if (tetrys->p_seq_num_of_last_refresh > 1)
		tetrys->tmp_rep_pdu.ack_min_redundancy =
		    tetrys->p_seq_num_of_last_refresh - 2;
	    else
		tetrys->tmp_rep_pdu.ack_min_redundancy = 0;
	}
	if (tetrys->currentRole == 1)
	    tetrys->tmp_rep_pdu.uint_pktType = PORT_TR;
	tetrys->tmp_rep_pdu.currentWindow = tetrys->currentWindow;
	tet_encode(tetrys->SEBuffer, (gf *) & (tetrys->tmp_rep_pdu).datav, PLS, (int) tetrys->tmp_rep_pdu.repair_id, tetrys->tmp_rep_pdu.min_redundancy, tetrys->tmp_rep_pdu.max_redundancy);
																								      /**< Encode all the data pkt in buffer from min_red to max_red ( including the ones with seqnum=min_red and seqnum=max_red)**/
	tetrys->nb_rep_sent++;
	tetrys->tmp_rep_pdu.pls = PLS;	// always for Repair pkts
	if (PR_DB_1)
	    fprintf(stdout,
		    "\nsendRepair: num %lu minRed %lu maxRed %lu ack_min %lu plastref %lu ",
		    tetrys->tmp_rep_pdu.num,
		    tetrys->tmp_rep_pdu.min_redundancy,
		    tetrys->tmp_rep_pdu.max_redundancy,
		    tetrys->tmp_rep_pdu.ack_min_redundancy,
		    tetrys->p_seq_num_of_last_refresh);
#ifdef STATS
	stat_repair_set_stats(&(tetrys->tmp_rep_pdu), sndStats);
#endif
	tetrys->tmp_rep_pdu.lossRate = tetrys->lossRate;
	/**< Then add it to the encoding windows, and to the Output buffer */
	tetrys->tmp_circ_item =
	    circ_add_not_sorted_pdu_in_buffer(&tetrys->tmp_rep_pdu,
					  tetrys->OBuffer);
    }
    actuate_alarm(2);	 /**< Reset the rep timer */
    return;
}

/**
 * \fn void wait_for_input ()
 * \brief Wait for an input on the various part on both network, pipe/socket..
 *
 */
void wait_for_input(void)
{
    if (PR_DB_1)
	fprintf(stdout, "\nwait_for_input:");

    if (check_nb_item_in_buffers() == 0) {
	blocking_input_check();
    } else {
	non_blocking_input_check();
    }
    return;
}

/**
 * Return the number of items stored in the TIBuffer.
 */
static int check_nb_item_in_TIBuffer(void)
{
    return tetrys->TIBuffer->nbItemStored;
}

/**
 * Return the number of items stored in the DIBuffer.
 */
static int check_nb_item_in_DIBuffer(void)
{
    return tetrys->DIBuffer->nbItemStored;
}

/**
 * Return the number of items stored in the OBuffer.
 */
static int check_nb_item_in_OBuffer(void)
{
    return tetrys->OBuffer->nbItemStored;
}

/** Return the total number of items across TI, DI and O buffers. */
static int check_nb_item_in_buffers(void)
{
    return (check_nb_item_in_TIBuffer() + check_nb_item_in_DIBuffer() +
	    check_nb_item_in_OBuffer());
}

/**
 * Process ACK information carried in @p stamp_to_parse (TA, TAP, TAR or TAD)
 * and prune acknowledged packets from SEBuffer.
 * @param stamp_to_parse  Incoming PDU to inspect.
 */
static void scan_tetrys_ack(const struct PDU *stamp_to_parse)
{
	/**< Scan only if it is actually an Ack */
    if (PR_DB_2)
	fprintf(stdout,
		"\nscan_tetrys_ack: pkttype %d ack_min %lu tetmin %lu ",
		stamp_to_parse->uint_pktType,
		stamp_to_parse->ack_min_redundancy,
		tetrys->min_seq_in_red);
    if ((stamp_to_parse->uint_pktType != PORT_TAD)
	&& (stamp_to_parse->uint_pktType != PORT_TAR)
	&& (stamp_to_parse->uint_pktType != PORT_TA)
	&& (stamp_to_parse->uint_pktType != PORT_TAP))
	return;
	/**< return without processing otherwise */
#ifdef STATS
    stat_ack_set_stats(stamp_to_parse, sndStats);
#endif
    if (tetrys->fecMode == 0) {	/**< Act as a normal tetrys */
	tetrys->max_acked_minus_two = stamp_to_parse->ack_min_redundancy;
	if (stamp_to_parse->ack_min_redundancy >
	    (tetrys->min_seq_in_red + 3)) {
	    tetrys->min_seq_in_red = MAX(stamp_to_parse->ack_min_redundancy - 2, 0);	// min redundancy is the only parameter of interest
	    if (PR_DB_1)
		fprintf(stdout,
			"\n\n\n\nACTUATE\n\nSCAN_TETRTRYS_ACK: ACUTATE pkt type %d ack_min %lu tetmin %lu -> circ_del_item_below_value\n\nEND ACTUATE\n\n\n\n",
			stamp_to_parse->uint_pktType,
			stamp_to_parse->ack_min_redundancy,
			tetrys->min_seq_in_red);
	}
	if (PR_DB_1)
	    fprintf(stdout,
		    "\n\nscan_tetrys_ack: pkt type %d ack_min %lu tetmin %lu -> circ_del_item_below_value\n\n",
		    stamp_to_parse->uint_pktType,
		    stamp_to_parse->ack_min_redundancy,
		    tetrys->min_seq_in_red);
	if (tetrys->min_seq_in_red > 2) {
	    circ_del_item_below_value(tetrys->SEBuffer,
				   tetrys->min_seq_in_red);
	}
    }
    tetrys->nb_ack_rcv++;
    if ((tetrys->SEBuffer->nbItemStored == 0) && (sig_AckRed->endAt > 0.0))
	tetrys_stop_requested = 1;
    return;
}

/** Queue an application packet read from an input fd (action 1) in the DIBuffer. */
static void harvest_app_data(void)
{
    if (tetrys->currentRole < 2) {
	tet_actuate_ipkt_rate();
	circ_add_data_in_buffer((gf *) & tetrys->pkt_to_parse,
				tetrys->incoming_length, tetrys->DIBuffer,
				PORT_DS, tetrys->nb_incoming_dat);
	if (PR_DB_2)
	    printf("\nharvest_input: add DS in DI Buffer ");
    } else if (tetrys->currentRole == 2) {
	circ_add_data_in_buffer((gf *) & tetrys->pkt_to_parse,
				tetrys->incoming_length, tetrys->DIBuffer,
				PORT_AR, tetrys->nb_incoming_dat);
	if (PR_DB_2)
	    printf("\nharvest_input: add AR in DI Buffer ");
    }
#ifdef SAVEPACKET
    if (tetrys->DIBuffer->lastAdded != NULL)
	write_packet('s', tetrys->DIBuffer->lastAdded->pkt);
#endif
    tetrys->nb_incoming_dat++;
}

/**
 * Queue a Tetrys data / repair packet received from the network (action 2)
 * in the TIBuffer after processing its acknowledgement and RTT fields.
 */
static void harvest_tetrys_packet(void)
{
    const struct PDU *pdu = (const struct PDU *) &tetrys->pkt_to_parse;

    if (tetrys->incoming_length != (int) sizeof(struct PDU))
	return;
    if (tetrys->nodeid == pdu->nodeid)
	return;
    if ((pdu->uint_pktType == PORT_TD || pdu->uint_pktType == PORT_TAD)
	&& (pdu->pls < 0 || pdu->pls > PLS)) {
	fprintf(stderr, "harvest_tetrys_packet: invalid TD pls %d\n", pdu->pls);
	return;
    }
    if ((pdu->uint_pktType == PORT_TR || pdu->uint_pktType == PORT_TAR)
	&& (pdu->min_redundancy > pdu->max_redundancy
	    || (pdu->max_redundancy - pdu->min_redundancy) >=
	    (unsigned long) (SIZE_SUBTRACT_VECTOR * CHAR_BIT))) {
	fprintf(stderr,
		"harvest_tetrys_packet: invalid TR window [%lu, %lu]\n",
		pdu->min_redundancy, pdu->max_redundancy);
	return;
    }
    {
	unsigned long window_cap = (unsigned long) tetrys->lost_positions_cap;
	unsigned long max_allowed =
	    (ULONG_MAX - tetrys->p_seq_num_of_last_refresh >= window_cap)
	    ? (tetrys->p_seq_num_of_last_refresh + window_cap - 1UL)
	    : ULONG_MAX;
	if ((pdu->uint_pktType == PORT_TD || pdu->uint_pktType == PORT_TAD)
	    && pdu->num > max_allowed) {
	    fprintf(stderr,
		    "harvest_tetrys_packet: dropping out-of-window TD num %lu (max %lu)\n",
		    pdu->num, max_allowed);
	    return;
	}
	if ((pdu->uint_pktType == PORT_TR || pdu->uint_pktType == PORT_TAR)
	    && pdu->max_redundancy > max_allowed) {
	    fprintf(stderr,
		    "harvest_tetrys_packet: dropping out-of-window TR max_redundancy %lu (max %lu)\n",
		    pdu->max_redundancy, max_allowed);
	    return;
	}
    }

    scan_tetrys_ack(pdu);
    if ((pdu->uint_pktType == PORT_TAD) || (pdu->uint_pktType == PORT_TD))
	tet_actuate_opkt_rate();	/**< TI packet are */
#ifdef SAVEPACKET
    write_packet('r', pdu);
#endif
    get_rtt_stuff(pdu);
    if (PR_DB_1)
	printf("\n\t HARVEST nid %d tetrys->tmp_pdu.local_rtt_probe=%f && tetrys->tmp_pdu.dst_rtt_probe==%f && tetrys->tmp_pdu.rtt_estimation=%f && tetrys->tmp_pdu.lossRate=%f",
	       tetrys->nodeid, pdu->local_rtt_probe, pdu->dst_rtt_probe,
	       pdu->rtt_estimation, pdu->lossRate);
    if (pdu->local_rtt_probe == -1 && pdu->dst_rtt_probe == -1
	&& pdu->rtt_estimation == -1 && pdu->lossRate == -1
	&& tetrys->closePending == 0)
	tetrys->closePending = 1;
    circ_add_not_sorted_pdu_in_buffer(pdu, tetrys->TIBuffer);
    if (PR_DB_2)
	printf("\nharvest_input: add in TI Buffer %d pkt", tetrys_debug_nbTIAdded);
    tetrys_debug_nbTIAdded++;
    tetrys->nb_incoming_tet++;
}

/**
 * Process an acknowledgement packet (TA / TAP, action 3) received from the network.
 * @param input_action  Action code returned by the input handler.
 */
static void harvest_ack_packet(int input_action)
{
    struct PDU *pdu = (struct PDU *) &tetrys->pkt_to_parse;

    if (tetrys->incoming_length != (int) sizeof(struct PDU))
	return;
    if (tetrys->nodeid == pdu->nodeid)
	return;
    scan_tetrys_ack(pdu);
#ifdef SAVEPACKET
    write_packet('r', pdu);
#endif
    get_rtt_stuff(pdu);
    if (PR_DB_2)
	fprintf(stdout, "\nharvest_input: action %d pkt type %d ",
		input_action, pdu->uint_pktType);
    if (pdu->uint_pktType == PORT_TAP) {
	pdu->uint_pktType = PORT_AS;
	circ_add_not_sorted_pdu_in_buffer(pdu, tetrys->OBuffer);
	tetrys->nb_outgoing_item++;
    }
}

/**
 * Run the handler of input slot @p fd_tmp and route the packet it returns:
 * application data to the DIBuffer, Tetrys packets to the TIBuffer, ACKs processed at once.
 * @param fd_tmp  Index in tetrys->fd_fun_tab.
 */
static void harvest_input(int fd_tmp)
{
    int input_action;

    if (PR_DB_2)
	fprintf(stdout, "\nharvest_input: BEGIN fd %d", fd_tmp);
    tetrys->incoming_length = -1;
    if ((input_action =
	 tetrys->fd_fun_tab[fd_tmp].fd_fun(tetrys->fd_fun_tab[fd_tmp].fd,
					   (gf *) & tetrys->pkt_to_parse)) < 0)
	fprintf(stdout,
		"\nharvest_input: failed to define the coresponding action ( %d ) ; Must be 1, 2 or 3 ! ",
		input_action);
    else if (PR_DB_2)
	printf("\nharvest_input: Input Action = %d ", input_action);

    if (input_action == 1)	/**< Si DS, AR Add it to the DIBuffer  */
	harvest_app_data();
    else if (input_action == 2)	/**< Si TAD, TAR TR TD;  Add it to the TIBuffer  */
	harvest_tetrys_packet();
    else if (input_action == 3)	/**< Si TA ou TAP */
	harvest_ack_packet(input_action);
    else if (input_action != 5 && input_action >= 0)
	fprintf(stderr,
		"\nharvest_input: failed to define the coresponding action ( %d ); Must be 1, 2 or 3 !",
		input_action);
    if (PR_DB_2)
	fprintf(stdout, "\nharvest_input: END fd %d", fd_tmp);
}

/**
 * Poll all registered input file descriptors without blocking and harvest any ready data.
 */
static void non_blocking_input_check(void)
{
		/**< DO max select (set the value of the maximum descriptor) */
    tetrys->tmp_timeval.tv_sec = 0;
    tetrys->tmp_timeval.tv_usec = 0;
    tetrys->max_fd_set = 0;
    int fd_max_tmp = 0;
    FD_ZERO(&tetrys->tet_fd_rset);	       /**< Disable all the fd */
    for (fd_max_tmp = 0; fd_max_tmp < tetrys->nb_fd_fun; fd_max_tmp++) {
	int cur_fd = tetrys->fd_fun_tab[fd_max_tmp].fd;
	if (cur_fd >= 0 && cur_fd < FD_SETSIZE) {
	    FD_SET(cur_fd, &tetrys->tet_fd_rset);
	    if (cur_fd > tetrys->max_fd_set)
		tetrys->max_fd_set = cur_fd;
	}
    }
    if (PR_DB_2)
	fprintf(stderr, "\nnon_blocking_input_check:  before SELECT");
		/**< DONE max select */
		/**< Do the blocking select thing with a null timer (All of the fd might have been previously set) */
    select(tetrys->max_fd_set + 1, &tetrys->tet_fd_rset, NULL, NULL, &tetrys->tmp_timeval);	   /**< Select turn off all of the fd that were not ready for reading */
    if (PR_DB_2)
	fprintf(stderr, "\nnon_blocking_input_check: select RETURNED ");

		/**< Then manage the available pkt/data */
    for (fd_max_tmp = 0; fd_max_tmp < tetrys->nb_fd_fun; fd_max_tmp++) {
	int cur_fd = tetrys->fd_fun_tab[fd_max_tmp].fd;
	if (cur_fd >= 0 && cur_fd < FD_SETSIZE) {
	    if (FD_ISSET(cur_fd, &tetrys->tet_fd_rset)) {
		harvest_input(fd_max_tmp);			   /**< Then test and harvest all of the FD that are still on */
	    }
	}
    }
    FD_ZERO(&tetrys->tet_fd_rset);	       /**< Disable all the fd */
    return;
}

/**
 * Wait atomically (via pselect with SIGALRM unmasked) until an input fd is readable
 * or a signal arrives.
 */
static void blocking_input_check(void)
{
    if (PR_DB_1)
	fprintf(stderr, "\nblocking_input_check:");
    if (tetrys_alarm_pending || tetrys_stop_requested
	|| (atomic_load(&tetrys->end_transfer_requested) && !tetrys->end_transfer_active))
	return;

    tetrys->max_fd_set = 0;
    int fd_max_tmp = 0;
    FD_ZERO(&tetrys->tet_fd_rset);	       /**< Disable all the fd */
    for (fd_max_tmp = 0; fd_max_tmp < tetrys->nb_fd_fun; fd_max_tmp++) {
	int cur_fd = tetrys->fd_fun_tab[fd_max_tmp].fd;
	if (cur_fd >= 0 && cur_fd < FD_SETSIZE) {
	    FD_SET(cur_fd, &tetrys->tet_fd_rset);
	    if (cur_fd > tetrys->max_fd_set)
		tetrys->max_fd_set = cur_fd;
	}
    }
    sigset_t pselect_mask;
    pthread_sigmask(SIG_SETMASK, NULL, &pselect_mask);
    sigdelset(&pselect_mask, SIGALRM);
    int selectReturn =
	pselect(tetrys->max_fd_set + 1, &tetrys->tet_fd_rset, NULL, NULL,
		NULL, &pselect_mask);
    if (selectReturn == -1) {
	if (PR_DB_2)
	    fprintf(stderr, "\nblocking_input_check: pselect return -1.. maybe interruped by sig: return !");
    } else {
			/**< Then manage the available pkt/data */
	for (fd_max_tmp = 0; fd_max_tmp < tetrys->nb_fd_fun; fd_max_tmp++) {
	    int cur_fd = tetrys->fd_fun_tab[fd_max_tmp].fd;
	    if (cur_fd >= 0 && cur_fd < FD_SETSIZE) {
		if (FD_ISSET(cur_fd, &tetrys->tet_fd_rset)) {
		    harvest_input(fd_max_tmp);				   /**< Then test and harvest all of the FD that are still on */
		}
	    }
	}
    }
    FD_ZERO(&tetrys->tet_fd_rset);	       /**< Disable all the fd */
    return;
}

/** Send a standalone ACK packet (no piggybacked data or repair). */
static void send_ack_only(void)
{
    if (tetrys->RDBuffer->nb_tot_added > 0) {
	tetrys->tmp_circ_item =
	    circ_add_not_sorted_item_in_buffer(tetrys->OBuffer);
	if (tetrys->tmp_circ_item != NULL) {
	    if (tetrys->p_seq_num_of_last_refresh > 1)
		tetrys->tmp_circ_item->pkt->ack_min_redundancy =
		    tetrys->p_seq_num_of_last_refresh;
	    else
		tetrys->tmp_circ_item->pkt->ack_min_redundancy = 0;
	    tetrys->tmp_circ_item->pkt->uint_pktType = PORT_TA;
	    tetrys->tmp_circ_item->pkt->num = 0;
	    tetrys->tmp_circ_item->pkt->pls = 0;
	    tetrys->tmp_circ_item->pkt->currentWindow = 0;
	    tetrys->tmp_circ_item->pkt->currentPktInWindow = 0;
	    tetrys->tmp_circ_item->pkt->repair_id = 0;
	    tetrys->tmp_circ_item->pkt->lossRate = tetrys->lossRate;
	    if (tetrys->currentRole == 0) {
		tetrys->tmp_circ_item->pkt->min_redundancy =
		    tetrys->min_seq_in_red;
		tetrys->tmp_circ_item->pkt->max_redundancy =
		    tetrys->max_seq_in_red;
	    }
	    if (PR_DB_1)
		fprintf(stderr, "\nsendAckOnly: ack min %lu pkt Type %d",
			tetrys->tmp_circ_item->pkt->ack_min_redundancy,
			tetrys->tmp_circ_item->pkt->uint_pktType);
	}
    }
#ifdef STATS
    stat_send_ack_set_stats(rcvStats);
#endif
    actuate_alarm(1);
    return;
}

/** Send a repair packet as a standalone redundancy update. */
static void send_redundancy(void)
{
    if (PR_DB_1)
	printf("\nWill Send Timeout trigered Redundancy !!\n");
    if (tetrys->SEBuffer->nbItemStored > 0) {
	send_repair();

    }
}
