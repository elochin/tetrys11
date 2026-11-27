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

#ifndef TETRYS_MAIN
#define TETRYS_MAIN 1

#include "common_struct_def.h"

/**
 * Allocate and zero-initialise the global @c tetrys, @c sig_AckRed and stats
 * structures, then call init_fec() and init_tetrys().
 * Not reentrant: only a single Tetrys instance per process is supported.
 * @param role      Node role (ROLE_BIDIRECTIONAL, ROLE_SENDER or ROLE_RECEIVER).
 * @param pkt_size  Application packet size passed to stats initialisation (bytes).
 */
void tetrys_alloc_globals(int role, int pkt_size);

/** Initialise the global Tetrys state (allocates buffers, resets counters). */
void init_tetrys(void);

/**
 * Free all memory, buffers, recovery arrays and open files owned by the
 * global Tetrys state (plus stats and sig_AckRed). Safe to call once at exit.
 */
void tetrys_cleanup(void);

/**
 * Execute one iteration of the protocol event loop: wait for input, handle any
 * pending SIGALRM timer, process TIBuffer, deliver in-order data, flush
 * OBuffer, process DIBuffer, and flush OBuffer again.
 * @return 1 to continue running, 0 when shutdown was requested.
 */
int tetrys_step_once(void);

/**
 * Update the SIGALRM scheduler after an ACK or repair was sent.
 * @param action  1 = ACK sent, 2 = repair sent, 3 = both.
 */
void actuate_alarm(int action);

/** Disarm any pending SIGALRM interval timer. */
void tetrys_disarm_alarm(void);

/**
 * Execute any timer work signalled by SIGALRM (ACK / repair emission) outside
 * of signal handler context.
 */
void tetrys_process_pending_alarm(void);

/**
 * Check whether an end-of-transfer flush was requested by the application thread,
 * arm fast periodic repairs, and mark transfer_acked once every packet is ACKed.
 */
void tetrys_check_end_transfer(void);

/**
 * Process all packets waiting in the Tetrys Input Buffer (TIBuffer).
 * TIBuffer holds raw packets received from the network; this function
 * classifies each one (data / repair / ACK) and routes it accordingly.
 */
void treat_ti_buffer(void);

/**
 * Process all packets waiting in the Data Input Buffer (DIBuffer).
 * DIBuffer holds application-layer data ready to be encoded and sent.
 * Encodes data into the Sender Encoding buffer and triggers repair emission.
 */
void treat_di_buffer(void);

/**
 * Drain the Output Buffer (OBuffer), sending each pending packet on the wire
 * via the registered output handlers.
 */
void treat_o_buffer(void);

/**
 * Block until at least one registered input file descriptor is readable,
 * then read the incoming packet into TIBuffer.
 */
void wait_for_input(void);

/**
 * Count the number of zero bits in the subtracted-vector of a repair item.
 * A zero bit means the corresponding data packet has not yet been subtracted
 * from this repair symbol.
 * @param _tmp  Pointer to the repair buffer item to inspect.
 * @return Number of data packets not yet subtracted.
 */
int vect_get_nb_zero(const struct circBUFFERItem *_tmp);

/**
 * Set or clear a single bit in a bit-vector stored as a gf array.
 * @param set     Pointer to the first byte of the bit-vector.
 * @param number  Zero-based bit position to modify.
 * @param value   1 to set the bit, 0 to clear it.
 */
void vect_setbit(gf *set, int number, int value);

/**
 * Read a single bit from a bit-vector stored as a gf array.
 * @param set     Pointer to the first byte of the bit-vector.
 * @param number  Zero-based bit position to read.
 * @return 1 if the bit is set, 0 if it is clear.
 */
int vect_getbit(const gf *set, int number);

/**
 * Print the subtracted-vector of a repair buffer item to stdout.
 * Each bit represents whether the corresponding data packet has been
 * subtracted from this repair symbol (1 = subtracted, 0 = pending).
 * @param _tmp  Pointer to the repair buffer item to print.
 */
void print_bitvector(const struct circBUFFERItem *_tmp);

/**
 * Deliver buffered data packets to the upper layer in sequence order.
 * Packets are released only when all preceding sequence numbers are available.
 */
void send_dat_in_order(void);

/** Request end-of-transfer flushing and block until all queued data are ACKed. */
void tet_end_transfer(void);

#endif
