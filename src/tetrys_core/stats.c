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

/**
 * Increment histogram bin @p idx safely if it lies within [0, @p size).
 * @param hist  Histogram array.
 * @param size  Number of bins in @p hist.
 * @param idx   Bin index to increment.
 */
static void stat_inc_bin(int *hist, int size, long idx)
{
    if (idx >= 0 && idx < size)
	hist[idx]++;
}

/**
 * Safe division returning 0.0 when @p den is zero.
 * @param num  Numerator.
 * @param den  Denominator.
 * @return @p num / @p den, or 0.0 if @p den == 0.0.
 */
static double safe_div(double num, double den)
{
    return (den != 0.0) ? (num / den) : 0.0;
}

/**
 * Open a statistics output file, aborting with a clear message on failure.
 * @param name Path of the file to open.
 * @param mode fopen() mode string.
 * @return A valid FILE pointer (never NULL).
 */
static FILE *stat_fopen(const char *name, const char *mode)
{
    FILE *file = fopen(name, mode);
    if (file == NULL) {
	perror(name);
	exit(EXIT_FAILURE);
    }
    return file;
}

void stat_init_sender_stats(struct senderStats *stats, int psize)
{
    int i = 0;
    stats->pktSize = psize;
    stats->statSize = 2000;
    stats->ackStamp = 0;
    stats->avgEncodingWindowLength = 0.0;
    stats->avgPeriod = 0.0;
    stats->avgPeriodDuration = 0.0;
    stats->nbSinceLastAck = 0;
    stats->nbAckRcvd = 0;
    stats->nbDatSent = 0;
    stats->nbRepSent = 0;
    stats->startTime = 0.0;
    stats->endTime = 0.0;
    for (i = 0; i < stats->statSize; i++) {
	stats->encodingWindowLength[i] = 0;
	stats->ackPeriod[i] = 0;
	stats->ackPeriodDuration[i] = 0;
    }
    for (i = 0; i < 200; i++) {
	stats->rcvLossRate[i] = 0;
    }
}

void stat_repair_set_stats(const struct PDU *pkt, struct senderStats *stats)
{
    long idx = (pkt->max_redundancy >= pkt->min_redundancy)
	? (long)(pkt->max_redundancy - pkt->min_redundancy + 1) : -1;
    stat_inc_bin(stats->encodingWindowLength, stats->statSize, idx);
    stats->nbRepSent++;
    return;
}

void stat_data_set_stats(const struct PDU *pkt, struct senderStats *stats)
{
    (void) pkt;
    stats->nbSinceLastAck++;
    stats->nbDatSent++;
    return;
}

void stat_ack_set_stats(const struct PDU *pkt, struct senderStats *stats)
{
    int f = (int)((double)pkt->lossRate * 200.0);
    stat_inc_bin(stats->rcvLossRate, 200, f);
    stat_inc_bin(stats->ackPeriod, stats->statSize, stats->nbSinceLastAck);
    stats->nbAckRcvd++;
    stats->nbSinceLastAck = 0;
    f = (int)((get_time() - stats->ackStamp) * 100.0);
    if (stats->nbAckRcvd > 1)
	stat_inc_bin(stats->ackPeriodDuration, stats->statSize, f);
    stats->ackStamp = get_time();
    return;
}

void stat_save_sender_stats(struct senderStats *stats)
{
    FILE *file_wind;
    FILE *file_ack;
    stats->endTime = get_time();
    file_wind = stat_fopen("senderStat_encodingWind.stats", "wb");
    file_ack = stat_fopen("senderStat_ackPeriod.stats", "wb");
    int i;
    double j;
    int nbEncWind = 0;
    int nbPeriod = 0;
    for (i = 0; i < stats->statSize; i++) {
	fprintf(file_wind, "%d\t%d\n", i, stats->encodingWindowLength[i]);
	stats->avgEncodingWindowLength =
	    stats->avgEncodingWindowLength +
	    ((double) (i * stats->encodingWindowLength[i]));
	nbEncWind = nbEncWind + stats->encodingWindowLength[i];
	fprintf(file_ack, "%d\t%d\t", i, stats->ackPeriod[i]);
	stats->avgPeriod =
	    stats->avgPeriod + ((double) (i * stats->ackPeriod[i]));
	nbPeriod = nbPeriod + stats->ackPeriod[i];
	j = (double) i;
	fprintf(file_ack, "%f\t%d\n", j / 100, stats->ackPeriodDuration[i]);
	stats->avgPeriodDuration =
	    stats->avgPeriodDuration +
	    ((double) (i * stats->ackPeriodDuration[i]));
    }
    stats->avgEncodingWindowLength =
	safe_div(stats->avgEncodingWindowLength, (double) nbEncWind);
    stats->avgPeriod = safe_div(stats->avgPeriod, (double) nbPeriod);
    stats->avgPeriodDuration =
	safe_div(stats->avgPeriodDuration, (double) nbPeriod) / 100.0;
    fclose(file_wind);
    fclose(file_ack);
    FILE *file_loss;
    file_loss = stat_fopen("senderStat_lossRate.stats", "wb");
    int nbLossRate = 0;
    for (i = 0; i < 200; i++) {
	j = (double) i;
	fprintf(file_loss, "%f\t%d\n", j / 2.0, stats->rcvLossRate[i]);
	stats->avgLossRate =
	    stats->avgLossRate +
	    j / 2.0 * ((double) stats->rcvLossRate[i]);
	nbLossRate = nbLossRate + stats->rcvLossRate[i];
    }
    stats->avgLossRate = safe_div(stats->avgLossRate, (double) nbLossRate);
    fclose(file_loss);
    FILE *file_main;
    file_main = stat_fopen("senderStat_main.stats", "wb");
    double dur = stats->endTime - stats->startTime;
    fprintf(file_main, "nbAckRcvd\t%d\n", stats->nbAckRcvd);
    fprintf(file_main, "nbDatSent\t%d\n", stats->nbDatSent);
    fprintf(file_main, "nbRepSent\t%d\n", stats->nbRepSent);
    fprintf(file_main, "avgEncWinSize\t%f\n",
	    stats->avgEncodingWindowLength);
    fprintf(file_main, "avgAckPeriod\t%f\n", stats->avgPeriod);
    fprintf(file_main, "avgAckPeriodDuration\t%f\n", stats->avgPeriodDuration);
    fprintf(file_main, "avgLossRate\t%f\n", stats->avgLossRate);
    fprintf(file_main, "avgEncodingRatio=\t%f\n",
	    safe_div((double) stats->nbDatSent,
		     (double) (stats->nbRepSent + stats->nbDatSent)));
    fprintf(file_main, "avgRedRatio=\t%f\n",
	    safe_div((double) stats->nbRepSent, (double) stats->nbDatSent));
    fprintf(file_main, "avgDataSendingRate %f\n",
	    safe_div((double) (stats->nbDatSent * stats->pktSize), dur));
    fprintf(file_main, "avgRepSendingRate %f\n",
	    safe_div((double) (stats->nbRepSent * stats->pktSize), dur));
    fprintf(file_main, "avgSendingRate %f\n",
	    safe_div((double) ((stats->nbDatSent + stats->nbRepSent) * stats->pktSize), dur));
    fclose(file_main);
    return;
}

void stat_print_sender_stats(const struct senderStats *stats)
{
    double dur = stats->endTime - stats->startTime;
    fprintf(stdout, "snd: nbAckRcvd\t\t%d\n", stats->nbAckRcvd);
    fprintf(stdout, "snd: nbDatSent\t\t%d\n", stats->nbDatSent);
    fprintf(stdout, "snd: nbRepSent\t\t%d\n", stats->nbRepSent);
    fprintf(stdout, "snd: avgEncWinSize\t%f\n",
	    stats->avgEncodingWindowLength);
    fprintf(stdout, "snd: avgAckPeriod\t\t%f\n", stats->avgPeriod);
    fprintf(stdout, "snd: avgAckPeriodDuration\t%f\n",
	    stats->avgPeriodDuration);
    fprintf(stdout, "snd: avgLossRate\t\t%f\n", stats->avgLossRate);
    fprintf(stdout, "snd: avgEncodingRatio\t%f\n",
	    safe_div((double) stats->nbDatSent,
		     (double) (stats->nbRepSent + stats->nbDatSent)));
    fprintf(stdout, "snd: avgRedRatio=\t\t%f\n",
	    safe_div((double) stats->nbRepSent, (double) stats->nbDatSent));
    fprintf(stdout, "snd: session duration %f end %f start %f\n",
	    dur, stats->endTime, stats->startTime);
    fprintf(stdout, "snd: avgDataSendingRate\t%f Bytes/s\n",
	    safe_div((double) (stats->nbDatSent * stats->pktSize), dur));
    fprintf(stdout, "snd: avgRepSendingRate\t%f Bytes/s\n",
	    safe_div((double) (stats->nbRepSent * stats->pktSize), dur));
    fprintf(stdout, "snd: avgSendingRate\t\t%f Bytes/s\n",
	    safe_div((double) ((stats->nbDatSent + stats->nbRepSent) * stats->pktSize), dur));
}

///////////////////////////////////////////////////////////////////////////////////////////

void stat_init_rcv_stats(struct receiverStats *stats, int psize)
{
    stats->startTime = 0.0;
    stats->timeLossEvent = 0.0;
    stats->timeInterLossEvent = get_time();
    stats->avgEncodingWindowLength = 0.0;
    stats->avgPeriod = 0.0;
    stats->avgPeriodDuration = 0.0;
    stats->avgInterLoss = 0.0;
    stats->avgLossEventSize = 0.0;
    stats->avgLossEventDuration = 0.0;
    stats->avgInterLossEventSize = 0.0;
    stats->avgInterLossEventDuration = 0.0;
    stats->avgNbLossInEvent = 0.0;
    stats->avgInversionTries = 0.0;
    stats->avgInversionSuccess = 0.0;
    stats->avgInversionFailure = 0.0;
    stats->avgDelayLoss = 0.0;
    stats->nbDatSinceLastLoss = 0;
    stats->prev_nb_pkt_lost = 0;
    stats->pktSize = psize;
    stats->startLossEvent = 0;
    stats->nbDatSinceLastEvent = 0;
    stats->nbLossInEvent = 0;
    stats->nbRepRcvd = 0;
    stats->nbSinceLastAck = 0;
    stats->avgLossRate = 0;
    stats->nbAckSent = 0;
    stats->statSize = 2000;
    stats->nbDatRcvd = 0;
    stats->gilbert_goodgood = 0;
    stats->gilbert_badbad = 0;
    int i = 0;
    for (i = 0; i < stats->statSize; i++) {
	stats->encodingWindowLength[i] = 0;
	stats->ackPeriod[i] = 0;
	stats->interLossSize[i] = 0;
	stats->lossEventSize[i] = 0;
	stats->nbLossInEvents[i] = 0;
	stats->lossEventDuration[i] = 0;
	stats->interLossEventDuration[i] = 0;
	stats->interLossEventSize[i] = 0;
    }
    for (i = 0; i < 1000; i++) {
	stats->inversionTries[i] = 0;
	stats->inversionSuccess[i] = 0;
	stats->inversionFailure[i] = 0;
	stats->tetrysDelay[i] = 0;
    }

}

/* Called a each packet reception when at least one losses is detected */
void stat_rcv_loss(struct receiverStats *stats, const struct TETRYS *tet)
{
    if (tet->nb_pkt_lost != stats->prev_nb_pkt_lost) {
	int i = stats->prev_nb_pkt_lost;
	int lim = MIN(tet->nb_pkt_lost, 1000);
	for (; i < lim; i++) {
	    tetrys->delayByLoss[i] = get_time();
	}
	stat_inc_bin(stats->interLossSize, stats->statSize, stats->nbDatSinceLastLoss);
	stats->nbDatSinceLastLoss = 0;
	stats->prev_nb_pkt_lost = tet->nb_pkt_lost;
    }
    return;
}

/* Called a each packet reception when at least one losses is detected */
void stat_rcv_start_loss_event(struct receiverStats *stats,
			    const struct TETRYS *tet)
{
    if (stats->startLossEvent == 0) {
	stats->startLossEvent = (int) tet->p_seq_num_of_last_refresh;
	stats->timeLossEvent = get_time();
	stat_inc_bin(stats->interLossEventSize, stats->statSize, stats->nbDatSinceLastEvent);
    }
    stats->nbLossInEvent = tet->nb_pkt_lost;
    stat_inc_bin(stats->interLossEventDuration, stats->statSize,
		 (long) ((get_time() - stats->timeInterLossEvent) * 100.0));
}

void stat_rcv_end_loss_event(struct receiverStats *stats, struct TETRYS *tet)
{
    int i = 0;
    int lim = MIN(tet->nb_pkt_lost, 1000);
    for (i = 0; i < lim; i++) {
	fprintf(tet->lossTimeRecovery, "%f %d\n",
		(get_time() - tet->delayByLoss[i]), tet->nb_pkt_lost);
	tet->delayByLoss[i] = 0;
    }
    long ev_sz = (tetrys->max_seq_num_seen >= (unsigned long) stats->startLossEvent)
	? (long)(tetrys->max_seq_num_seen - (unsigned long) stats->startLossEvent) : -1;
    stat_inc_bin(stats->lossEventSize, stats->statSize, ev_sz);
    stat_inc_bin(stats->lossEventDuration, stats->statSize,
		 (long) ((get_time() - stats->timeLossEvent) * 100.0));
    stat_inc_bin(stats->nbLossInEvents, stats->statSize, stats->nbLossInEvent);
    stats->startLossEvent = 0;
    stats->nbLossInEvent = 0;
    stats->nbDatSinceLastLoss = 0;
    stats->nbDatSinceLastEvent = 0;
    stats->timeInterLossEvent = get_time();
}

void stat_rcv_repair_set_stats(const struct PDU *pkt, struct receiverStats *stats)
{
    long idx = (pkt->max_redundancy >= pkt->min_redundancy)
	? (long)(pkt->max_redundancy - pkt->min_redundancy + 1) : -1;
    stat_inc_bin(stats->encodingWindowLength, stats->statSize, idx);
    stats->nbRepRcvd++;
    return;
}

void stat_rcv_data_set_stats(struct receiverStats *stats)
{
    stats->nbSinceLastAck++;
    stats->nbDatSinceLastLoss++;
    stats->nbDatSinceLastEvent++;
    stats->nbDatRcvd++;
    return;
}

void stat_send_ack_set_stats(struct receiverStats *stats)
{
    stat_inc_bin(stats->ackPeriod, stats->statSize, stats->nbSinceLastAck);
    stats->nbSinceLastAck = 0;
    stats->nbAckSent++;
    return;
}

void stat_recover_pkt(unsigned long int numrcvd)
{
    long idx = (tetrys->max_seq_num_seen >= numrcvd)
	? (long)(tetrys->max_seq_num_seen - numrcvd) : -1;
    stat_inc_bin(rcvStats->tetrysDelay, 1000, idx);
    return;
}

void stat_decoding_try(void)
{
    stat_inc_bin(rcvStats->inversionTries, 1000, tetrys->nb_pkt_lost);
    return;
}

void stat_decoding_succ(void)
{
    stat_inc_bin(rcvStats->inversionSuccess, 1000, tetrys->nb_pkt_lost);
    return;
}

void stat_decoding_failure(void)
{
    stat_inc_bin(rcvStats->inversionFailure, 1000, tetrys->nb_pkt_lost);
    return;
}

/**
 * Write the loss-event histograms of the receiver (one file per histogram)
 * and compute the corresponding averages in rcvStats.
 */
static void stat_rcv_save_event_hists(void)
{
    int i;
    FILE *file_wind;
    FILE *file_ack;
    FILE *file_interLoss;
    FILE *file_lossEventSize;
    FILE *file_lossEventDuration;
    FILE *file_interLossEventSize;
    FILE *file_interLossEventDuration;
    FILE *file_nbLossInEvent;
    file_wind = stat_fopen("rcvStat_encodingWind.stats", "wb");
    file_ack = stat_fopen("rcvStat_ackPeriod.stats", "wb");
    file_interLoss = stat_fopen("rcvStat_interLoss.stats", "wb");
    file_lossEventSize = stat_fopen("rcvStat_lossEventSize.stats", "wb");
    file_lossEventDuration =
	stat_fopen("rcvStat_lossEventDuration.stats", "wb");
    file_interLossEventSize =
	stat_fopen("rcvStat_interLossEventSize.stats", "wb");
    file_interLossEventDuration =
	stat_fopen("rcvStat_interLossEventDuration.stats", "wb");
    file_nbLossInEvent = stat_fopen("rcvStat_nbLossInEvent.stats", "wb");
    int nbEncWind = 0;
    int nbPeriod = 0;
    int nbInterLoss = 0;
    int nbLossEventSize = 0;
    int nbLossEventDuration = 0;
    int nbInterLossEventSize = 0;
    int nbInterLossEventDuration = 0;
    int nbNbLossInEvent = 0;
    for (i = 0; i < rcvStats->statSize; i++) {
	fprintf(file_wind, "%d\t%d\n", i, rcvStats->encodingWindowLength[i]);	//
	rcvStats->avgEncodingWindowLength =
	    rcvStats->avgEncodingWindowLength +
	    ((double) (i * rcvStats->encodingWindowLength[i]));
	nbEncWind = nbEncWind + rcvStats->encodingWindowLength[i];
	fprintf(file_ack, "%d\t%d\n", i, rcvStats->ackPeriod[i]);	//
	rcvStats->avgPeriod =
	    rcvStats->avgPeriod + ((double) (i * rcvStats->ackPeriod[i]));
	nbPeriod = nbPeriod + rcvStats->ackPeriod[i];
	fprintf(file_interLoss, "%d\t%d\n", i, rcvStats->interLossSize[i]);	//+
	rcvStats->avgInterLoss =
	    rcvStats->avgInterLoss +
	    ((double) (i * rcvStats->interLossSize[i]));
	nbInterLoss = nbInterLoss + rcvStats->interLossSize[i];
	fprintf(file_lossEventSize, "%d\t%d\n", i, rcvStats->lossEventSize[i]);	//+
	rcvStats->avgLossEventSize =
	    rcvStats->avgLossEventSize +
	    ((double) (i * rcvStats->lossEventSize[i]));
	nbLossEventSize = nbLossEventSize + rcvStats->lossEventSize[i];
	fprintf(file_lossEventDuration, "%f\t%d\n", ((double) (i)) / 100.0, rcvStats->lossEventDuration[i]);	//+
	rcvStats->avgLossEventDuration =
	    rcvStats->avgLossEventDuration +
	    ((double) (i * rcvStats->lossEventDuration[i])) / 100.0;
	nbLossEventDuration =
	    nbLossEventDuration + rcvStats->lossEventDuration[i];
	fprintf(file_interLossEventSize, "%d\t%d\n", i, rcvStats->interLossEventSize[i]);	//+
	rcvStats->avgInterLossEventSize =
	    rcvStats->avgInterLossEventSize +
	    ((double) (i * rcvStats->interLossEventSize[i]));
	nbInterLossEventSize =
	    nbInterLossEventSize + rcvStats->interLossEventSize[i];
	fprintf(file_interLossEventDuration, "%f\t%d\n", ((double) (i)) / 100.0, rcvStats->interLossEventDuration[i]);	//+
	rcvStats->avgInterLossEventDuration =
	    rcvStats->avgInterLossEventDuration +
	    ((double) (i * rcvStats->interLossEventDuration[i])) / 100.0;
	nbInterLossEventDuration =
	    nbInterLossEventDuration + rcvStats->interLossEventDuration[i];
	fprintf(file_nbLossInEvent, "%d\t%d\n", i, rcvStats->nbLossInEvents[i]);	//+
	rcvStats->avgNbLossInEvent =
	    rcvStats->avgNbLossInEvent +
	    ((double) (i * rcvStats->nbLossInEvents[i]));
	nbNbLossInEvent = nbNbLossInEvent + rcvStats->nbLossInEvents[i];


    }
    rcvStats->avgEncodingWindowLength =
	safe_div(rcvStats->avgEncodingWindowLength, (double) nbEncWind);
    rcvStats->avgPeriod = safe_div(rcvStats->avgPeriod, (double) nbPeriod);
    rcvStats->avgInterLoss =
	safe_div(rcvStats->avgInterLoss, (double) nbInterLoss);
    rcvStats->avgLossEventSize =
	safe_div(rcvStats->avgLossEventSize, (double) nbLossEventSize);
    rcvStats->avgLossEventDuration =
	safe_div(rcvStats->avgLossEventDuration, (double) nbLossEventDuration);
    rcvStats->avgInterLossEventSize =
	safe_div(rcvStats->avgInterLossEventSize, (double) nbInterLossEventSize);
    rcvStats->avgInterLossEventDuration =
	safe_div(rcvStats->avgInterLossEventDuration,
		 (double) nbInterLossEventDuration);
    rcvStats->avgNbLossInEvent =
	safe_div(rcvStats->avgNbLossInEvent, (double) nbNbLossInEvent);
    fclose(file_wind);
    fclose(file_ack);
    fclose(file_interLoss);
    fclose(file_lossEventSize);
    fclose(file_lossEventDuration);
    fclose(file_interLossEventSize);
    fclose(file_interLossEventDuration);
    fclose(file_nbLossInEvent);
}

/**
 * Write the matrix-inversion and delay-loss histograms and compute their
 * averages in rcvStats. The totals are returned for the summary files.
 * @param[out] nb_try         Total matrix inversion attempts.
 * @param[out] nb_suc         Total successful matrix inversions.
 * @param[out] nb_fai         Total failed matrix inversions.
 * @param[out] nb_delay_loss  Total recovered packet delay samples.
 */
static void stat_rcv_save_inversion_hists(int *nb_try, int *nb_suc,
					  int *nb_fai, int *nb_delay_loss)
{
    FILE *file_inversion = stat_fopen("rcvStat_inversion.stats", "wb");
    FILE *file_delayLoss = stat_fopen("rcvStat_delayLoss.stats", "wb");
    int i;
    int nbTry = 0;
    int nbSuc = 0;
    int nbFai = 0;
    int nbDelayLoss = 0;

    rcvStats->avgDelayLoss = 0.0;
    for (i = 0; i < 1000; i++) {
	fprintf(file_inversion, "\n%d\ttry %d\tsuc %d\tfai %d", i,
		rcvStats->inversionTries[i], rcvStats->inversionSuccess[i],
		rcvStats->inversionFailure[i]);
	rcvStats->avgInversionTries =
	    rcvStats->avgInversionTries +
	    ((double) (i * rcvStats->inversionTries[i]));
	rcvStats->avgInversionSuccess =
	    rcvStats->avgInversionSuccess +
	    ((double) (i * rcvStats->inversionSuccess[i]));
	rcvStats->avgInversionFailure =
	    rcvStats->avgInversionFailure +
	    ((double) (i * rcvStats->inversionFailure[i]));
	nbTry = nbTry + rcvStats->inversionTries[i];
	nbFai = nbFai + rcvStats->inversionFailure[i];
	nbSuc = nbSuc + rcvStats->inversionSuccess[i];
	fprintf(file_delayLoss, "%d\t%d\n", i, rcvStats->tetrysDelay[i]);	//+
	rcvStats->avgDelayLoss =
	    rcvStats->avgDelayLoss + 1.0 * (i * rcvStats->tetrysDelay[i]);
	nbDelayLoss = nbDelayLoss + rcvStats->tetrysDelay[i];
    }
    fclose(file_inversion);
    rcvStats->avgInversionTries =
	safe_div(rcvStats->avgInversionTries, (double) nbTry);
    rcvStats->avgInversionSuccess =
	safe_div(rcvStats->avgInversionSuccess, (double) nbSuc);
    rcvStats->avgInversionFailure =
	safe_div(rcvStats->avgInversionFailure, (double) nbFai);
    printf("\navgDelayLoss %f nb %f %f", rcvStats->avgDelayLoss,
	   ((double) nbDelayLoss),
	   safe_div(rcvStats->avgDelayLoss, (double) nbDelayLoss));
    rcvStats->avgDelayLoss =
	safe_div(rcvStats->avgDelayLoss, (double) nbDelayLoss);
    fclose(file_delayLoss);
    *nb_try = nbTry;
    *nb_suc = nbSuc;
    *nb_fai = nbFai;
    *nb_delay_loss = nbDelayLoss;
}

/** Write the summary file "rcvStat_main.stats" of the receiver. */
static void stat_rcv_save_main(int nb_delay_loss)
{
    FILE *file_main = stat_fopen("rcvStat_main.stats", "wb");
    double dur = rcvStats->endTime - rcvStats->startTime;
    fprintf(file_main, "nbAckSent\t%d\n", rcvStats->nbAckSent);
    fprintf(file_main, "nbDatRcvd\t%d\n", rcvStats->nbDatRcvd);
    fprintf(file_main, "nbRepRcvd\t%d\n", rcvStats->nbRepRcvd);
    fprintf(file_main, "nbRebuilt\t%d\n", tetrys->nb_rebuilt);
    fprintf(file_main, "nbDatTot\t%lu\n", tetrys->RDBuffer->nb_tot_added);
    fprintf(file_main, "avgEncWinSize\t%f\n",
	    rcvStats->avgEncodingWindowLength);
    fprintf(file_main, "avgAckPeriod\t%f\n", rcvStats->avgPeriod);
    fprintf(file_main, "avgLossRate\t%f\n", rcvStats->avgLossRate);
    fprintf(file_main, "avgEncodingRatio=\t%f\n",
	    safe_div((double) rcvStats->nbDatRcvd,
		     (double) (rcvStats->nbRepRcvd + rcvStats->nbDatRcvd)));
    fprintf(file_main, "avgRedRatio=\t%f\n",
	    safe_div((double) rcvStats->nbRepRcvd, (double) rcvStats->nbDatRcvd));
    fprintf(file_main, "avgDataSendingRate %f\n",
	    safe_div((double) (rcvStats->nbDatRcvd * rcvStats->pktSize), dur));
    fprintf(file_main, "avgRepSendingRate %f\n",
	    safe_div((double) (rcvStats->nbRepRcvd * rcvStats->pktSize), dur));
    fprintf(file_main, "avgSendingRate %f\n",
	    safe_div((double) ((rcvStats->nbDatRcvd + rcvStats->nbRepRcvd) * rcvStats->pktSize), dur));
    fprintf(file_main, "avgInterLoss\t%f\n", rcvStats->avgInterLoss);
    fprintf(file_main, "avgLossEventSize\t%f\n",
	    rcvStats->avgLossEventSize);
    fprintf(file_main, "avgLossEventDuration\t%f\n",
	    rcvStats->avgLossEventDuration);
    fprintf(file_main, "avgInterLossEventSize\t%f\n",
	    rcvStats->avgInterLossEventSize);
    fprintf(file_main, "avgInterLossEventDuration\t%f\n",
	    rcvStats->avgInterLossEventDuration);
    fprintf(file_main, "avgNbLossInEvent\t%f\n",
	    rcvStats->avgNbLossInEvent);
    fprintf(file_main, "avgDelayLoss\t%f\tnb%d\n", rcvStats->avgDelayLoss,
	    nb_delay_loss);
    fclose(file_main);
}

/** Append one line of results to "rcvStat_resultsPaper.stats". */
static void stat_rcv_save_paper(int nb_try, int nb_suc, int nb_fai)
{
    int allowzerocoef = 1;
#ifndef ALLOWZEROCOEF
    allowzerocoef = 0;
#endif
    FILE *file_main = stat_fopen("rcvStat_resultsPaper.stats", "a");
    fprintf(file_main,
	    "plr %f fieldssize %d gfbits %d allowzerocoef %d redratio %f goodgood %d badbad %d nbDat %d avgDelayLoss %f avgRecurenceTime %f avgMatrix %f singularP %f nonSingularP %f\n",
	    safe_div((double) tetrys->nb_rebuilt,
		     (double) tetrys->RDBuffer->nb_tot_added), GF_SIZE, GF_BITS,
	    allowzerocoef,
	    safe_div((double) rcvStats->nbRepRcvd,
		     (double) rcvStats->nbDatRcvd + (double) rcvStats->nbRepRcvd),
	    rcvStats->gilbert_goodgood, rcvStats->gilbert_badbad,
	    rcvStats->nbDatRcvd, rcvStats->avgDelayLoss,
	    rcvStats->avgLossEventSize, rcvStats->avgNbLossInEvent,
	    safe_div((double) nb_suc, (double) nb_try),
	    safe_div((double) nb_fai, (double) nb_try));
    fclose(file_main);
}

void stat_save_rcv_stats(void)
{
    int nb_try, nb_suc, nb_fai, nb_delay_loss;

    rcvStats->endTime = get_time();
    rcvStats->avgLossRate =
	safe_div((double) (tetrys->p_seq_num_of_last_refresh -
			   tetrys->tot_d_pkt_rcv),
		 (double) (tetrys->tot_d_pkt_rcv));
    stat_rcv_save_event_hists();
    stat_rcv_save_inversion_hists(&nb_try, &nb_suc, &nb_fai, &nb_delay_loss);
    stat_rcv_save_main(nb_delay_loss);
    stat_rcv_save_paper(nb_try, nb_suc, nb_fai);
}

void stat_print_rcv_stats(const struct receiverStats *stats)
{
    double dur = stats->endTime - stats->startTime;
    fprintf(stdout, "rcv: nbAckSent\t\t\t%d\n", stats->nbAckSent);
    fprintf(stdout, "rcv: nbDatRcvd\t\t\t%d\n", stats->nbDatRcvd);
    fprintf(stdout, "rcv: nbRepRcvd\t\t\t%d\n", stats->nbRepRcvd);
    fprintf(stdout, "rcv: avgEncWinSize\t\t%f\n",
	    stats->avgEncodingWindowLength);
    fprintf(stdout, "rcv: avgAckPeriod\t\t%f\n", stats->avgPeriod);
    fprintf(stdout, "rcv: avgLossRate\t\t%f\n", stats->avgLossRate);
    fprintf(stdout, "rcv: avgEncodingRatio\t%f\n",
	    safe_div((double) stats->nbDatRcvd,
		     (double) (stats->nbRepRcvd + stats->nbDatRcvd)));
    fprintf(stdout, "rcv: avgRedRatio\t\t%f\n",
	    safe_div((double) stats->nbRepRcvd, (double) stats->nbDatRcvd));
    fprintf(stdout, "rcv: avgDataSendingRate\t%f Bytes/s\n",
	    safe_div((double) (stats->nbDatRcvd * stats->pktSize), dur));
    fprintf(stdout, "rcv: avgRepSendingRate\t%f Bytes/s\n",
	    safe_div((double) (stats->nbRepRcvd * stats->pktSize), dur));
    fprintf(stdout, "rcv: avgSendingRate\t%f Bytes/s\n",
	    safe_div((double) ((stats->nbDatRcvd + stats->nbRepRcvd) * stats->pktSize), dur));
    fprintf(stdout, "rcv: avgInterLoss\t\t%f\n", stats->avgInterLoss);
    fprintf(stdout, "rcv: avgLossEventSize\t%f\n",
	    stats->avgLossEventSize);
    fprintf(stdout, "rcv: avgLossEventDuration\t%f\n",
	    stats->avgLossEventDuration);
    fprintf(stdout, "rcv: avgInterLossEventSize\t%f\n",
	    stats->avgInterLossEventSize);
    fprintf(stdout, "rcv: avgInterLossEventDuration\t%f\n",
	    stats->avgInterLossEventDuration);
    fprintf(stdout, "rcv: avgNbLossInEvent\t%f\n",
	    stats->avgNbLossInEvent);
}
