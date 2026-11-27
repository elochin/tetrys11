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

#include "tetrysModule_udp.h"

/* Global variable definition — declared extern in tetrysModule_udp.h. */
struct tetrys_udp *tetudp;

/*
 * Tetrys module which requires data and redundancy to be sent on different udp port number
 * Tetrys packet are sent on top of UDP
 * It receive the data from an UDP source and send the decoded ones to an UDP sing
 * If one want to send the datas to Tetrys trought AF_INET sock, pipe or SM, modify the input 
 * output function.
 */

/* For the sender
 * DS Input from UDP_DIn socket
 * TA TAP input from UDP_Tin socket X
 * TD from UDP_TDOut UDP socket on port X
 * TR from UDP_TROut UDP socket on port X+1
 */

/* For the receiver
 * AR Input from UDP_DIn socket
 * TA TAP output from UDP_Tout socket on port X
 * TD from UDP_TDin UDP socket on port X
 * TR from UDP_TRin UDP socket on port Y
 */


/* Return 1 as it only receive data pkts -- Read from iperfToTetrys[0] pipe */

/**
 * Read one application packet from the iperf pipe.
 * @param read_fd       Unused (the descriptor is kept in the module state).
 * @param pkt_to_parse  Buffer receiving the packet.
 * @return 1 (queue in DIBuffer), 5 (retry later) or -1 (error).
 */
static int input_handler_iperf(int read_fd, gf * pkt_to_parse)
{
    (void) read_fd;
    if (PR_DB_1) {
	printf("\ninput_handler_iperf: Begin");
	fflush(stdout);
    }
    tetudp->bytes =
	read(tetudp->iperfToTetrys[0], pkt_to_parse,
	     (size_t) tetudp->iperf_packet_len);
    if (tetudp->bytes == 0)
	return 5;
    if (tetudp->bytes < 0) {
	if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
	    // We assume Sig alarm triggered the event..
	    if (PR_DB_1)
		fprintf(stderr,
			"\ninputHandler_iperf: Return 5 for event trigered");
	    return 5;
	} else {
	    fprintf(stderr,
		    "\ninputHandler_iperf: read error tetudp->bytes < 0 ");
	    return -1;
	}
    }
    tetudp->nbDS_rcvd++;
    if (tetudp->bytes > TETRYS_MAX_PAYLOAD) {
	fprintf(stderr,
		"\ninputHandler_iperf: rcv packet TOO BIG tetudp->bytes==%d , truncate to %d ",
		(int) tetudp->bytes, TETRYS_MAX_PAYLOAD);
	tetudp->bytes = TETRYS_MAX_PAYLOAD;
    }
    if (PR_DB_1)
	printf("\ninputHandler_iperf: rcvd %d bytes pkt -- return 1 ",
	       (int) tetudp->bytes);
    tetrys->incoming_length = (int) tetudp->bytes;

    if (PR_DB)
	printf
	    ("\n\n sent: TD=%d TR=%d TAD=%d TAR=%d TA=%d TAP=%d DR=%d AS=%d",
	     tetudp->nbTD_sent, tetudp->nbTR_sent, tetudp->nbTAD_sent,
	     tetudp->nbTAR_sent, tetudp->nbTA_sent, tetudp->nbTAP_sent,
	     tetudp->nbDR_sent, tetudp->nbAS_sent);
    if (PR_DB)
	printf
	    ("\n rcvd: TD=%d TR=%d TAD=%d TAR=%d TA=%d TAP=%d DR=%d AS=%d\n",
	     tetudp->nbTD_rcvd, tetudp->nbTR_rcvd, tetudp->nbTAD_rcvd,
	     tetudp->nbTAR_rcvd, tetudp->nbTA_rcvd, tetudp->nbTAP_rcvd,
	     tetudp->nbDR_rcvd, tetudp->nbAS_rcvd);

    return 1;
}


/**
 * Receive one Tetrys packet from the UDP socket.
 * @param div_fd        Unused (the descriptor is kept in the module state).
 * @param pkt_to_parse  Buffer receiving the packet.
 * @return 2 (data/repair: TIBuffer), 3 (ACK), 5 (retry later) or -1 (error).
 */
static int input_handler_udp(int div_fd, gf * pkt_to_parse)
{
    (void) div_fd;
    tetudp->bytes = -1;
    tetudp->bytes =
	recvfrom(tetudp->fd_red, pkt_to_parse, BUFSIZE, 0,
		 (struct sockaddr *) &tetudp->dst_insock_red, &tetudp->lg);
    if (tetudp->bytes < 0) {
	if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
	    // We assume Sig alarm triggered the event..
	    if (PR_DB_1)
		fprintf(stderr,
			"\ninputHandler_udp: Return 5 for event trigered");
	    return 5;
	} else {
	    fprintf(stderr,
		    "\ninputHandler_udp: recvfrom error tetudp->bytes < 0 ");
	    return -1;
	}
    }
    if (tetudp->bytes != (ssize_t) sizeof(struct PDU)) {
	fprintf(stderr,
		"\ninputHandler_udp: ignoring malformed datagram (%zd bytes, expected %zu)\n",
		tetudp->bytes, sizeof(struct PDU));
	return -1;
    }
    tetrys->incoming_length = (int) tetudp->bytes;

    /* As the first two bytes are expected to be the pkt type, read it ! */
    int ptype = pkt_to_parse[1] * 256 + pkt_to_parse[0];
    if (ptype == PORT_TD)
	tetudp->nbTD_rcvd++;
    if (ptype == PORT_TR)
	tetudp->nbTR_rcvd++;
    if (ptype == PORT_TAD)
	tetudp->nbTAD_rcvd++;
    if (ptype == PORT_TAR)
	tetudp->nbTAR_rcvd++;
    if (ptype == PORT_TA)
	tetudp->nbTA_rcvd++;
    if (ptype == PORT_TAP)
	tetudp->nbTAP_rcvd++;

    if (ptype == PORT_TAD || ptype == PORT_TAR ||
	ptype == PORT_TD || ptype == PORT_TR) {
	return 2;
    } else if (ptype == PORT_TA || ptype == PORT_TAP) {
	return 3;
    }
    fprintf(stderr,
	    "\ninputHandler_udp: unable to determine the type %d of %d bytes pkt rcvd ",
	    ptype, (int) tetudp->bytes);
    return -1;
}

/*
 * 0,1: loc -> foreign = dat
 *      foreign -> loc = ack
 * 2:   foreign -> loc = dat
 *      loc -> foreign = ack
 * Convention:  local= src
 *              foreign=dst
 */

/**
 * Write a decoded packet to the iperf pipe.
 * @param tmp_pdu  Packet to deliver.
 * @return Number of bytes written, or -1 on error.
 */
static int output_handler_iperf(struct PDU *tmp_pdu)
{
    if (tmp_pdu->uint_pktType != PORT_DR
	&& tmp_pdu->uint_pktType != PORT_AS) {
	perror("\noutputHandler_iperf: Wrong pkt type: accept only DR AS");
	return -1;
    }
    memcpy(&tmp_pdu->datav, &tmp_pdu->pls, sizeof(int));
    if ((tetudp->bytes =
	 write(tetudp->tetrysToIperf[1], &tmp_pdu->datav, PLS)) != PLS) {
	perror("\noutputHandler_iperf: DR AS -- sendto");
    }
    if (tmp_pdu->uint_pktType == PORT_DR)
	tetudp->nbDR_sent++;

    return (int) tetudp->bytes;
}


/* Handle the Tetrys TR repair packet */
/**
 * Send a Tetrys protocol packet (TD/TR/TA/TAD/TAR/TAP) over UDP.
 * @param tmp_pdu  Packet to send.
 * @return Number of bytes sent, or -1 on error.
 */
static int output_handler_udp(struct PDU *tmp_pdu)
{
    if (tmp_pdu->uint_pktType != PORT_TD
	&& tmp_pdu->uint_pktType != PORT_TAD
	&& tmp_pdu->uint_pktType != PORT_TA
	&& tmp_pdu->uint_pktType != PORT_TAP
	&& tmp_pdu->uint_pktType != PORT_TR
	&& tmp_pdu->uint_pktType != PORT_TAR) {
	perror
	    ("\noutputHandler_iperf: Wrong pkt type: accept only TD TAD TA TAP TR TAR");
	return -1;
    }


    if (tmp_pdu->uint_pktType == PORT_TD) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTD_sent++;
    }
    if (tmp_pdu->uint_pktType == PORT_TR) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTR_sent++;
    }
    if (tmp_pdu->uint_pktType == PORT_TAD) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTAD_sent++;
    }
    if (tmp_pdu->uint_pktType == PORT_TAR) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTAR_sent++;
    }
    if (tmp_pdu->uint_pktType == PORT_TA) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTA_sent++;
    }
    if (tmp_pdu->uint_pktType == PORT_TAP) {
	if (PR_DB_1)
	    fprintf(stderr,
		    "\n\noutputHandler_div: SEND ACK %lu  type %d \n",
		    tmp_pdu->ack_min_redundancy, tmp_pdu->uint_pktType);
	tetudp->nbTAP_sent++;
    }

    /* if should use rep socket: */
    if ((tetudp->bytes =
	 sendto(tetudp->fd_red, tmp_pdu, sizeof(struct PDU), 0,
		(struct sockaddr *) &tetudp->dst_insock_red,
		tetudp->lg)) != sizeof(struct PDU)) {
	if (tetudp->bytes < 0)
	    fprintf(stderr,
		    "udp: UDP send failed (%s): packet of type %d (%zu bytes) lost, repair packets will compensate\n",
		    strerror(errno), tmp_pdu->uint_pktType, sizeof(struct PDU));
	else
	    fprintf(stderr, "udp: partial UDP send: %zd / %zu bytes\n",
		    tetudp->bytes, sizeof(struct PDU));
    }
    sem_post(&tetudp->accept_packet);
    return (int) tetudp->bytes;
}

/**
 * Put @p fd in non-blocking mode (select() drives the reads).
 * @param fd  File descriptor to configure.
 */
static void udp_set_nonblocking(int fd)
{
    int flags;

    if ((flags = fcntl(fd, F_GETFL, 0)) < 0)
	perror("F_GETFL error");
    flags |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) < 0)
	perror("F_SETFL error");
}

/** Create the pipes and the flow-control semaphore shared with the application. */
static void udp_create_pipes_and_sem(void)
{
    if (0 != pipe(tetudp->iperfToTetrys)) {
	fprintf(stderr,
		"\ninitModule_udp: Failed to create iperfToTetrys pipe -- EXIT FAILURE");
	exit(EXIT_FAILURE);
    }
    if (0 != pipe(tetudp->tetrysToIperf)) {
	fprintf(stderr,
		"\ninitModule_udp: Failed to create tetrysToIperf pipe -- EXIT FAILURE");
	exit(EXIT_FAILURE);
    }
    if (sem_init(&tetudp->accept_packet, 0, 4) != 0) {
	perror("sem_init(accept_packet)");
	exit(EXIT_FAILURE);
    }
    tetudp->accept_packet_init = 1;
#ifdef VERBOSE
    int sem_val = 0;
    sem_getvalue(&tetudp->accept_packet, &sem_val);
    fprintf(stderr, "\nTetrys: InitMod SEM %d  ", sem_val);
    fflush(stderr);
#endif
}

/**
 * Sender side: resolve the server address and open the UDP socket towards it.
 * @param udp_port  Destination UDP port.
 */
static void udp_open_sender_socket(int udp_port)
{
    tetudp->dport_dat = (uint32_t) udp_port;
    tetudp->dport_red = (uint32_t) udp_port;
    memset(&tetudp->dst_insock_red, 0, sizeof(struct sockaddr_in));
    if ((tetudp->host_red = gethostbyname2(tetudp->server, AF_INET)) == NULL) {
	fprintf(stderr, "udp_open_sender_socket: gethostbyname failed\n");
	exit(EXIT_FAILURE);
    }
    tetudp->fd_red = server_socket_snd((uint16_t) tetudp->dport_red);
    memcpy(&tetudp->dst_inaddr_red.s_addr, tetudp->host_red->h_addr_list[0],
	   sizeof(tetudp->dst_inaddr_red.s_addr));
    tetudp->dst_insock_red.sin_family = AF_INET;
    tetudp->dst_insock_red.sin_port = htons((uint16_t) tetudp->dport_red);
    memcpy(&tetudp->dst_insock_red.sin_addr, tetudp->host_red->h_addr_list[0],
	   sizeof(tetudp->dst_inaddr_red.s_addr));
    inet_ntop(AF_INET, &(tetudp->dst_insock_red.sin_addr), tetudp->nom, 255);
}

/** Reset every sent / received packet counter of the module. */
static void udp_reset_counters(void)
{
    tetudp->nbDR_sent = 0;
    tetudp->nbAS_sent = 0;
    tetudp->nbAR_sent = 0;
    tetudp->nbDS_sent = 0;
    tetudp->nbTD_sent = 0;
    tetudp->nbTR_sent = 0;
    tetudp->nbTAD_sent = 0;
    tetudp->nbTAR_sent = 0;
    tetudp->nbTA_sent = 0;
    tetudp->nbTAP_sent = 0;

    tetudp->nbDR_rcvd = 0;
    tetudp->nbAS_rcvd = 0;
    tetudp->nbAR_rcvd = 0;
    tetudp->nbDS_rcvd = 0;
    tetudp->nbTD_rcvd = 0;
    tetudp->nbTR_rcvd = 0;
    tetudp->nbTAD_rcvd = 0;
    tetudp->nbTAR_rcvd = 0;
    tetudp->nbTA_rcvd = 0;
    tetudp->nbTAP_rcvd = 0;
}

/*
 *	Usage: 
 *	need the udp sock initialised by iperf
 */
void init_module_tetudp(char *_server, int _udp_port, int _iperf_packet_len)
{
    if ((tetudp =
	 (struct tetrys_udp *) calloc(1, sizeof(struct tetrys_udp))) == NULL) {
	fprintf(stderr,
		"init_module_tetudp: Not enough memory for allocating struct tetrys_udp\n");
	exit(EXIT_FAILURE);
    }
    udp_create_pipes_and_sem();
    tetudp->iperf_packet_len = _iperf_packet_len;
    tetudp->server = _server;
    /* Then open the DAT client socket and related stuff */
    tetudp->fd_red = -1;
    if (tetrys->currentRole == 1) {	/* If sender DAT and RED sock are client, IO is server */
	udp_open_sender_socket(_udp_port);
    } else if (tetrys->currentRole == 2) {	/* if receiver, DAT sock is server, RED sock == NULL, IO is client */
	tetudp->dport_red = (uint32_t) _udp_port;
	tetudp->fd_red = server_socket_rcv((uint16_t) tetudp->dport_red);
#ifdef VERBOSE
	fprintf(stderr, "\ninitModule_safety: fd_red waiting on port %u ",
		tetudp->dport_red);
#endif
    } else {
	fprintf(stderr, "initModule_safety: Error, currenrRoler == %d",
		tetrys->currentRole);
	exit(EXIT_FAILURE);
    }
    /* DO put the RED socket in non blocking mode: Will blck with select, not with recvfrom */
    udp_set_nonblocking(tetudp->fd_red);

    tetudp->lg = sizeof(tetudp->server_address);
    if ((tetrys->currentRole == 1)
	&& ((tetudp->host_red == NULL) || (tetudp->fd_red < 0))) {
	perror("gethostbyname");
	exit(EXIT_FAILURE);
    }
    udp_set_nonblocking(tetudp->iperfToTetrys[0]);
    udp_reset_counters();
}

void cleanup_module_tetudp(void)
{
    if (tetudp == NULL)
	return;
    if (tetudp->fd_red >= 0) {
	close(tetudp->fd_red);
	tetudp->fd_red = -1;
    }
    for (int i = 0; i < 2; i++) {
	if (tetudp->iperfToTetrys[i] >= 0) {
	    close(tetudp->iperfToTetrys[i]);
	    tetudp->iperfToTetrys[i] = -1;
	}
	if (tetudp->tetrysToIperf[i] >= 0) {
	    close(tetudp->tetrysToIperf[i]);
	    tetudp->tetrysToIperf[i] = -1;
	}
    }
    if (tetudp->accept_packet_init) {
	sem_destroy(&tetudp->accept_packet);
	tetudp->accept_packet_init = 0;
    }
    free(tetudp);
    tetudp = NULL;
}

/*
 * \fn void init_context(void)
 * \brief Set the right function depending on the input/output events
 */

void init_context(void)
{
    FD_ZERO(&tetrys->tet_fd_rset);
    tetrys->fd_fun_tab[0].fd = tetudp->iperfToTetrys[0];
    tetrys->fd_fun_tab[0].fd_fun = input_handler_iperf;
    tetrys->fd_fun_tab[1].fd = tetudp->fd_red;
    tetrys->fd_fun_tab[1].fd_fun = input_handler_udp;
    tetrys->nb_fd_fun = 2;
    FD_SET(tetudp->iperfToTetrys[0], &tetrys->tet_fd_rset);
    FD_SET(tetudp->fd_red, &tetrys->tet_fd_rset);
    tetrys->out_fun_tab[0].pkttype = PORT_TD;
    tetrys->out_fun_tab[0].out_fun = output_handler_udp;
    tetrys->out_fun_tab[1].pkttype = PORT_TR;
    tetrys->out_fun_tab[1].out_fun = output_handler_udp;
    tetrys->out_fun_tab[2].pkttype = PORT_TA;
    tetrys->out_fun_tab[2].out_fun = output_handler_udp;
    tetrys->out_fun_tab[3].pkttype = PORT_DR;
    tetrys->out_fun_tab[3].out_fun = output_handler_iperf;
    tetrys->out_fun_tab[4].pkttype = PORT_AS;
    tetrys->out_fun_tab[4].out_fun = output_handler_iperf;
    tetrys->out_fun_tab[5].pkttype = PORT_TAR;
    tetrys->out_fun_tab[5].out_fun = output_handler_udp;
    tetrys->out_fun_tab[6].pkttype = PORT_TAD;
    tetrys->out_fun_tab[6].out_fun = output_handler_udp;
    tetrys->out_fun_tab[7].pkttype = PORT_TAP;
    tetrys->out_fun_tab[7].out_fun = output_handler_udp;
    tetrys->nb_out_fun = 8;
    return;
}
