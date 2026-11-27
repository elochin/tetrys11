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


/* Actuate the state variable at the reception of a redundancy packet */
void auto_rcv_red(const struct PDU *pkt)
{
    tetrys->tot_r_pkt_rcv++;
    if (tetrys->max_r_seq_num_seen > pkt->max_redundancy)
	tetrys->nbOldDatOrRedRcvdSinceLastTry += 1;
    tetrys->max_r_seq_num_seen =
	MAX(tetrys->max_r_seq_num_seen, pkt->max_redundancy);
    if (pkt->min_redundancy > (unsigned long) tetrys->highest_min_r_rcvd)
	tetrys->highest_min_r_rcvd = (int) pkt->min_redundancy;
    tetrys->nb_r_since_last_refresh++;
    tetrys->max_seq_num_seen =
	MAX(tetrys->max_r_seq_num_seen, tetrys->max_d_seq_num_seen);
#ifdef STATS
    stat_rcv_repair_set_stats(pkt, rcvStats);
#endif
    return;
}

/* Actuate the state variable at the reception of a data packet */
void auto_rcv_dat(const struct PDU *pkt)
{
    tetrys->tot_d_pkt_rcv++;
    tetrys->nb_d_since_last_refresh++;
    if (tetrys->max_d_seq_num_seen > pkt->num)
	tetrys->nbOldDatOrRedRcvdSinceLastTry += 1;
    tetrys->max_d_seq_num_seen = MAX(tetrys->max_d_seq_num_seen, pkt->num);
    tetrys->max_seq_num_seen =
	MAX(tetrys->max_r_seq_num_seen, tetrys->max_d_seq_num_seen);
#ifdef STATS
    stat_rcv_data_set_stats(rcvStats);
#endif
    return;
}

/* Actuate the state variable at the reception of a redundancy packet when no losses were detected */
void auto_refresh_red_nolosses(void)
{
    tetrys->queue_num_of_first_wind_pkt = tetrys->nb_d_in_queue;	//  To refresh again when removing pkt from infQueue
    tetrys->nb_r_since_last_refresh = 0;
    tetrys->nb_d_since_last_refresh = 0;
    tetrys->nb_p_since_last_refresh = 0;	// The number of pkt rcvd since last refresh
    tetrys->p_seq_num_of_last_refresh = tetrys->max_seq_num_seen + 1;
    tetrys->min_data_pkt_to_retain =
	(tetrys->highest_min_r_rcvd > 0)
	? (unsigned long) tetrys->highest_min_r_rcvd : 0UL;
    if (tetrys->fecMode == 0)
	circ_empty_queue(tetrys->RRBuffer);
}

/* Actuate the states variables at the reception of a data packet when no losses were detected */
void auto_refresh_dat_nolosses(void)
{
    tetrys->queue_num_of_last_refresh = tetrys->nb_d_in_queue;
    tetrys->nb_r_since_last_refresh = 0;
    tetrys->nb_d_since_last_refresh = 0;
    tetrys->nb_p_since_last_refresh = 0;	// The number of pkt rcvd since last refresh
    tetrys->p_seq_num_of_last_refresh = tetrys->max_seq_num_seen + 1;
    if (tetrys->fecMode == 0)
	circ_empty_queue(tetrys->RRBuffer);
    return;
}

/* Refresh stats variables at the end of a loss event */
void auto_refresh_red_recov(unsigned long int max_recov)
{
#ifdef STATS
    stat_rcv_end_loss_event(rcvStats, tetrys);
#endif
    tetrys->nb_loss_delay = 0;
    tetrys->queue_num_of_first_wind_pkt = tetrys->nb_d_in_queue;
    tetrys->min_data_pkt_to_retain = (unsigned long) tetrys->highest_min_r_rcvd;
    tetrys->nb_r_since_last_refresh = 0;
    tetrys->nb_d_since_last_refresh = 0;
    tetrys->nb_p_since_last_refresh = 0;	// The number of data pkt rcvd since last refresh
    tetrys->queue_num_of_last_refresh = tetrys->nb_d_in_queue;
    tetrys->p_seq_num_of_last_refresh = max_recov;
    if (tetrys->fecMode == 0)
	circ_del_item_below_or_equal_value(tetrys->RRBuffer,
				      tetrys->max_red_useful);
    if (PR_DB_1) {
	printf
	    ("\nauto_refresh_red_recov: MaxRecov %lu maxseen %lu maxredu %lu nred %d psnolr %lu ",
	     max_recov, tetrys->max_seq_num_seen, tetrys->max_red_useful,
	     tetrys->RRBuffer->nbItemStored,
	     tetrys->p_seq_num_of_last_refresh);
	fflush(stdout);
    }
}

/** Actuate stat variable at the reception of a pkt, whether it is a Data pkt or a redundancy */
void auto_rcv_pkt(const struct PDU *pkt)
{
    if (tetrys->fecMode == 1) {	/**< Act as FEC */
	if (pkt->currentWindow > tetrys->currentWindow) {      /**< New fec block.. then flush the previous one */
	    tetrys->p_seq_num_of_last_refresh = pkt->min_redundancy;	/**< consider all previous pkts as received (may be wrong) */
	}
    }
#ifdef STATS
    stat_rcv_loss(rcvStats, tetrys);
#endif
    tetrys->nb_p_since_last_refresh++;
    tetrys->tot_p_pkt_rcv++;
    tetrys->padding_on_last_pkt = PLS - pkt->pls;
}


/**< get Rtt stuff from an incomming pkt  2010 */
void get_rtt_stuff(const struct PDU *tmp_pdu)
{
    tetrys->last_probe_rcvd = tmp_pdu->local_rtt_probe;
    tetrys->dst_rtt = tmp_pdu->rtt_estimation;
    tetrys->ts_of_last_probe_rcvd = get_time();
    if (tmp_pdu->dst_rtt_probe != 0.0) {
	double rtt =
	    (tetrys->ts_of_last_probe_rcvd - tmp_pdu->dst_rtt_probe);
	if (tetrys->previous_rtt == 0.0)
	    tetrys->previous_rtt = rtt;
	else
	    tetrys->previous_rtt = tetrys->rtt;
	tetrys->rtt =
	    (tetrys->rtt_smooth_param * tetrys->previous_rtt) + (1.0 -
								 tetrys->
								 rtt_smooth_param)
	    * rtt;
    }
}



/**< set Rtt stuff to an outgoing pkt 2010 */
void set_rtt_stuff(struct PDU *_tmp_pdu)
{
    _tmp_pdu->local_rtt_probe = get_time();
    _tmp_pdu->dst_rtt_probe = tetrys->last_probe_rcvd + (get_time() - tetrys->ts_of_last_probe_rcvd);	// last probe + processing time spent
    _tmp_pdu->rtt_estimation = tetrys->rtt;
}

/* 
 * Pkt format: nowTS nodeId send/receiv pktType num repairId min_red max_red ack_min srcprobe dstprobe estimatedRtt estimatedLossRate
 *< Write packet if SAVEPACKET options if on 
 */

void write_packet(char snd_rcv, const struct PDU *tmp_pdu)
{
    fprintf(tetrys->savePkt,
	    "ts1 %f nnid3 %d pnid5 %d snd7 %c typ9 %d num11 %lu rId13 %lu mr15 %lu Mr17 %lu am19 %lu sp21 %f dp23 %f rtt25 %f dlr27 %f llr29 %f rtt31 %f drtt33 %f ipr35 %f opr37 %f\n",
	    get_time(), tetrys->nodeid, tmp_pdu->nodeid, snd_rcv,
	    tmp_pdu->uint_pktType, tmp_pdu->num, tmp_pdu->repair_id,
	    tmp_pdu->min_redundancy, tmp_pdu->max_redundancy,
	    tmp_pdu->ack_min_redundancy, tmp_pdu->local_rtt_probe,
	    tmp_pdu->dst_rtt_probe, tmp_pdu->rtt_estimation,
	    tmp_pdu->lossRate, tetrys->lossRate, tetrys->rtt,
	    tetrys->dst_rtt, tetrys->sensedPktIRate,
	    tetrys->sensedPktORate);
}
