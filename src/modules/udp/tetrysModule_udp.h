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

#ifndef tetudpSTRUCT
#define tetudpSTRUCT 1

#include "common.h"
#include <pthread.h>
#include <semaphore.h>

#define	FILE_MODE	(00400 | 00200 | 00040 | 00004)

/**
 * State of the UDP module: sockets, addresses, pipes with the application
 * and per-type packet counters.
 */
struct tetrys_udp {
    ssize_t bytes;	/**< Byte count of the last I/O operation. */
    int fd_red;	/**< Descriptor of the Tetrys UDP socket. */
    char *server;	/**< Remote host name or IP (sender only). */
    struct sockaddr server_address;	/**< Generic address, used as size reference for recvfrom(). */
    char nom[255];	/**< Printable remote IP address (debug). */
    socklen_t lg;	/**< Length of the data/redundancy socket address. */
    uint32_t dport_dat;	/**< UDP port of the data socket. */
    uint32_t dport_red;	/**< UDP port of the Tetrys (redundancy) socket. */
    struct sockaddr_in dst_insock_red;	/**< Peer address of the Tetrys socket (updated by recvfrom() on the receiver). */
    struct in_addr dst_inaddr_red;	/**< Peer IP of the Tetrys socket. */

    struct hostent *host_red;	/**< Resolved remote host (sender only). */
    int iperf_packet_len;	/**< Size of the application packets (bytes). */
    sem_t accept_packet;	/**< Process-local semaphore throttling application writes into the pipe. */
    int accept_packet_init;	/**< Non-zero once @c accept_packet has been initialised. */
    int iperfToTetrys[2];	/**< Pipe application -> Tetrys. */
    int tetrysToIperf[2];	/**< Pipe Tetrys -> application. */

    int nbDR_sent;	/**< Number of decoded data packets delivered to the application sent. */
    int nbAS_sent;	/**< Number of packets delivered to the sender application sent. */
    int nbAR_sent;	/**< Number of receiver-application packets sent. */
    int nbDS_sent;	/**< Number of sender-application packets sent. */
    int nbTD_sent;	/**< Number of Tetrys data packets sent. */
    int nbTR_sent;	/**< Number of Tetrys repair packets sent. */
    int nbTAD_sent;	/**< Number of Tetrys data packets with a piggybacked ACK sent. */
    int nbTAR_sent;	/**< Number of Tetrys repair packets with a piggybacked ACK sent. */
    int nbTA_sent;	/**< Number of Tetrys ACK packets sent. */
    int nbTAP_sent;	/**< Number of ACK-carrying receiver-application packets (TAP) sent. */

    int nbDR_rcvd;	/**< Number of decoded data packets delivered to the application received. */
    int nbAS_rcvd;	/**< Number of packets delivered to the sender application received. */
    int nbAR_rcvd;	/**< Number of receiver-application packets received. */
    int nbDS_rcvd;	/**< Number of sender-application packets received. */

    int nbTD_rcvd;	/**< Number of Tetrys data packets received. */
    int nbTR_rcvd;	/**< Number of Tetrys repair packets received. */
    int nbTAD_rcvd;	/**< Number of Tetrys data packets with a piggybacked ACK received. */
    int nbTAR_rcvd;	/**< Number of Tetrys repair packets with a piggybacked ACK received. */
    int nbTA_rcvd;	/**< Number of Tetrys ACK packets received. */
    int nbTAP_rcvd;	/**< Number of ACK-carrying receiver-application packets (TAP) received. */

};
extern struct tetrys_udp *tetudp;



/**
 * Initialise the UDP module: pipes, semaphore, sockets and counters.
 * tetrys->currentRole must be set (1 = sender, 2 = receiver).
 * @param _server            Remote host (sender only).
 * @param _udp_port          UDP port of the Tetrys traffic.
 * @param _iperf_packet_len  Size of the application packets (bytes).
 */
void init_module_tetudp(char *_server, int _udp_port,
		       int _iperf_packet_len);

/** Close sockets/pipes, destroy semaphore and free the global @c tetudp state. */
void cleanup_module_tetudp(void);

/** Register the UDP and pipe handlers in the Tetrys dispatch tables. */
void init_context(void);

/**
 * Allocate and initialise the whole Tetrys engine and the UDP module.
 * @param iperf_packet_len  Size of the application packets (bytes).
 * @param _fec_n            FEC parameter n.
 * @param _fec_k            FEC parameter k.
 * @param _role             1 = sender, 2 = receiver.
 * @param _fecMode          0 for Tetrys, 1 for block FEC.
 * @param _server           Remote host (sender only).
 * @param _port             UDP port.
 * @return 0 on success.
 */
extern int tetrys_init_all(int iperf_packet_len, int _fec_n, int _fec_k,
			   int _role, int _fecMode, char *_server,
			   int _port);

/**
 * Tetrys protocol main loop (never returns); usable as a thread body.
 * @param arg  Unused.
 */
extern void *tetrys_main_run(void *arg);

#endif				/* tetudpSTRUCT */
