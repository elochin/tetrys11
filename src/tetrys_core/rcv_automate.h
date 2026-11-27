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

#ifndef RCV_AUTOMATE_H
#define RCV_AUTOMATE_H

#include "common_struct_def.h"

/**
 * \file rcv_automate.h
 * \brief Receiver state-machine functions.
 *
 * These functions update the global Tetrys state in response to incoming
 * data or repair packets and trigger recovery/delivery actions accordingly.
 *
 * Functions marked "not used in circular-buffer version" are kept for
 * reference but are superseded by the linked-list buffer implementation.
 */

/**
 * Update receiver state upon reception of a repair (redundancy) packet.
 * Checks for loss, triggers subtraction, and attempts decoding if possible.
 * @param pkt  The received repair PDU.
 */
void auto_rcv_red(const struct PDU *pkt);

/**
 * Update receiver state upon reception of a data packet.
 * Detects gaps in the sequence space to infer losses.
 * @param pkt  The received data PDU.
 */
void auto_rcv_dat(const struct PDU *pkt);

/**
 * Update state after a repair arrives with no loss detected in the window.
 * Advances the refresh point and prunes old buffered data.
 */
void auto_refresh_red_nolosses(void);

/**
 * Update state after a data packet arrives with no loss detected.
 * Advances the refresh point when the window is complete.
 */
void auto_refresh_dat_nolosses(void);

/**
 * Finalise statistics at the end of a loss/recovery event.
 * @param max_recov  Sequence number of the last successfully recovered packet.
 */
void auto_refresh_red_recov(unsigned long int max_recov);

/**
 * Update per-packet statistics (loss history, inter-arrival timing, etc.)
 * for any received packet, regardless of type.
 * @param pkt  The received PDU (data or repair).
 */
void auto_rcv_pkt(const struct PDU *pkt);

/**
 * Extract RTT probe fields from an incoming packet and update tetrys->rtt.
 * @param tmp_pdu  The received PDU.
 */
void get_rtt_stuff(const struct PDU *tmp_pdu);

/**
 * Stamp RTT probe fields onto an outgoing packet before transmission.
 * @param _tmp_pdu  The PDU being prepared for sending.
 */
void set_rtt_stuff(struct PDU *_tmp_pdu);

/**
 * Write a packet trace record to the savePkt file (SAVEPACKET compile flag).
 * @param snd_rcv  'S' for sent, 'R' for received.
 * @param tmp_pdu  The PDU to log.
 */
void write_packet(char snd_rcv, const struct PDU *tmp_pdu);

#endif /* RCV_AUTOMATE_H */
