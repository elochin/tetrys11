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
 * Tetrys TUN/TAP tunnel over UDP
 * --------------------------------
 * Wraps a TUN interface in a Tetrys FEC-protected UDP tunnel.
 * Modelled after src/modules/udp/ but replaces iperf pipes with a TUN fd:
 *   - input_handler_tun   reads raw IP packets from the TUN interface
 *   - output_handler_tun  writes decoded IP packets back to the TUN interface
 *   - input_handler_udp / output_handler_udp  handle the Tetrys UDP channel
 *
 * Usage:
 *   sender   (role 1): tetrystunnel_udp -i tun0 -c <server-IP> [-p port] [-n n] [-k k]
 *   receiver (role 2): tetrystunnel_udp -i tun0 -s              [-p port]
 */

/* Tetrys core — must come first (pulls in common_struct_def.h, circularBuffer, …) */
#include "common.h"

/* TUN/TAP */
#include <net/if.h>
#include <linux/if_tun.h>
#include <sys/ioctl.h>

/* ------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------ */
#define PERROR(x)    do { perror(x); exit(1); } while (0)
#define CLIENT       0
#define SERVER       1
#define TUN_PKT_LEN  1500   /* maximum IP packet size read from TUN */

/* ------------------------------------------------------------------
 * Module-private state: replaces tetrys_udp, using TUN fd instead
 * of iperf pipes.
 * ------------------------------------------------------------------ */
/** State of the TUN module: TUN device, UDP socket, peer address and counters. */
struct tetrys_tun {
    int         tun_fd;              /**< TUN interface file descriptor */
    int         fd_red;              /**< UDP socket for Tetrys packets */
    ssize_t     bytes;               /**< scratch: last I/O byte count */
    char       *server;              /**< remote host name / IP string (sender only) */
    struct sockaddr  server_address; /**< generic sockaddr (size ref for recvfrom) */
    char        nom[255];            /**< printable remote IP (debug) */
    socklen_t   lg;                  /**< length of dst_insock_red */
    uint32_t   dport_red;           /**< UDP port used for all Tetrys traffic */
    struct sockaddr_in dst_insock_red;/**< remote sockaddr (set by sender; updated by recvfrom on receiver) */
    struct in_addr     dst_inaddr_red;     /**< Remote IP address (sender only). */
    struct hostent    *host_red;           /**< Resolved remote host (sender only). */

    /* counters */
    int nbTD_sent;	/**< Tetrys data packets sent. */
    int nbTR_sent;	/**< Tetrys repair packets sent. */
    int nbTAD_sent;	/**< Tetrys data with ACK packets sent. */
    int nbTAR_sent;	/**< Tetrys repair with ACK packets sent. */
    int nbTA_sent;	/**< Tetrys ACK packets sent. */
    int nbTAP_sent;	/**< ACK-carrying receiver-application packets (TAP) sent. */
    int nbDR_sent;	/**< Decoded data packets written to the TUN interface. */
    int nbAS_sent;	/**< Return packets (payload of a received TAP) written to the sender TUN interface. */
    int nbTD_rcvd;	/**< Tetrys data packets received. */
    int nbTR_rcvd;	/**< Tetrys repair packets received. */
    int nbTAD_rcvd;	/**< Tetrys data with ACK packets received. */
    int nbTAR_rcvd;	/**< Tetrys repair with ACK packets received. */
    int nbTA_rcvd;	/**< Tetrys ACK packets received. */
    int nbTAP_rcvd;	/**< ACK-carrying receiver-application packets (TAP) received. */
};

static struct tetrys_tun *tettun;   /* single global instance */

static const char *progname;

/* ==================================================================
 * tun_alloc — allocate or reconnect a TUN/TAP device
 * ================================================================== */
/**
 * Allocate or reconnect a TUN/TAP device.
 * @param dev    Interface name (in), actual name (out); must hold IFNAMSIZ bytes.
 * @param flags  IFF_TUN / IFF_TAP and IFF_NO_PI flags.
 * @return The TUN file descriptor, or a negative value on error.
 */
static int tun_alloc(char *dev, int flags)
{
    struct ifreq ifr;
    int fd, err;
    const char *clonedev = "/dev/net/tun";

    if ((fd = open(clonedev, O_RDWR)) < 0) {
        perror("Opening /dev/net/tun");
        return fd;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = (short) flags;
    if (*dev)
        snprintf(ifr.ifr_name, IFNAMSIZ, "%s", dev);

    if ((err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return err;
    }

    snprintf(dev, IFNAMSIZ, "%s", ifr.ifr_name); /* dev holds IFNAMSIZ bytes (see main) */
    return fd;
}

/* ==================================================================
 * input_handler_tun
 * Read one raw IP packet from the TUN interface.
 * Returns 1  → Tetrys should queue the packet in DIBuffer (data input).
 * Returns 5  → EINTR; caller should retry.
 * Returns -1 → unrecoverable error.
 * ================================================================== */
/**
 * Read one raw IP packet from the TUN interface.
 * @param read_fd       Unused (the fd is kept in the module state).
 * @param pkt_to_parse  Buffer receiving the packet.
 * @return 1 (queue in DIBuffer), 5 (retry later) or -1 (error).
 */
static int input_handler_tun(int read_fd, gf *pkt_to_parse)
{
    (void)read_fd;   /* fd is tettun->tun_fd, already stored in struct */

    memset(pkt_to_parse, '\0', TUN_PKT_LEN);
    tettun->bytes = read(tettun->tun_fd, pkt_to_parse, TUN_PKT_LEN);
    if (tettun->bytes < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return 5;   /* no data available, let the main loop retry */
        fprintf(stderr, "\ninputHandler_tun: read error (%s)\n", strerror(errno));
        return -1;
    }
    if (tettun->bytes == 0)
        return 5;   /* nothing to process */
    if (tettun->bytes > TETRYS_MAX_PAYLOAD) {
        fprintf(stderr, "\ninputHandler_tun: packet too large (%d B), truncated to %d B\n",
                (int)tettun->bytes, TETRYS_MAX_PAYLOAD);
        tettun->bytes = TETRYS_MAX_PAYLOAD;
    }
    tetrys->incoming_length = (int)tettun->bytes;
    return 1;   /* → queue in DIBuffer */
}

/* ==================================================================
 * input_handler_udp
 * Receive one Tetrys packet from the UDP socket.
 * Returns 2  → data/repair packet (TD/TR/TAD/TAR): queue in TIBuffer.
 * Returns 3  → ACK packet (TA/TAP): process immediately.
 * Returns 5  → EINTR.
 * Returns -1 → error or unknown type.
 * ================================================================== */
/**
 * Receive one Tetrys packet from the UDP socket.
 * @param div_fd        Unused (the fd is kept in the module state).
 * @param pkt_to_parse  Buffer receiving the packet.
 * @return 2 (data/repair: TIBuffer), 3 (ACK), 5 (retry later) or -1 (error).
 */
static int input_handler_udp(int div_fd, gf *pkt_to_parse)
{
    (void)div_fd;

    tettun->bytes = recvfrom(tettun->fd_red, pkt_to_parse, BUFSIZE, 0,
                             (struct sockaddr *)&tettun->dst_insock_red,
                             &tettun->lg);
    if (tettun->bytes < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return 5;
        fprintf(stderr, "\ninputHandler_udp: recvfrom error (%s)\n", strerror(errno));
        return -1;
    }
    if (tettun->bytes != (ssize_t)sizeof(struct PDU)) {
        fprintf(stderr, "\ninputHandler_udp: ignoring malformed datagram (%zd bytes, expected %zu)\n",
                tettun->bytes, sizeof(struct PDU));
        return -1;
    }
    tetrys->incoming_length = (int)tettun->bytes;

    /* Decode packet type from first two bytes (little-endian PORT_T* constant) */
    int ptype = pkt_to_parse[1] * 256 + pkt_to_parse[0];

    if (ptype == PORT_TD)  tettun->nbTD_rcvd++;
    if (ptype == PORT_TR)  tettun->nbTR_rcvd++;
    if (ptype == PORT_TAD) tettun->nbTAD_rcvd++;
    if (ptype == PORT_TAR) tettun->nbTAR_rcvd++;
    if (ptype == PORT_TA)  tettun->nbTA_rcvd++;
    if (ptype == PORT_TAP) tettun->nbTAP_rcvd++;

    if (ptype == PORT_TAD || ptype == PORT_TAR ||
        ptype == PORT_TD  || ptype == PORT_TR)
        return 2;   /* → TIBuffer */

    if (ptype == PORT_TA || ptype == PORT_TAP)
        return 3;   /* → treat as ACK */

    fprintf(stderr, "\ninputHandler_udp: unknown packet type %d (%d bytes)\n",
            ptype, (int)tettun->bytes);
    return -1;
}

/* ==================================================================
 * output_handler_tun
 * Write a decoded IP packet (DR or AS) to the TUN interface.
 * ================================================================== */
/**
 * Write a decoded forward packet (DR) or piggybacked return packet (AS) to
 * the local TUN interface.
 * @param tmp_pdu  Packet to deliver.
 * @return Number of bytes written, 0 if empty/corrupt, -1 on a bad type.
 */
static int output_handler_tun(struct PDU *tmp_pdu)
{
    if (tmp_pdu->uint_pktType != PORT_DR && tmp_pdu->uint_pktType != PORT_AS) {
        fprintf(stderr, "\noutputHandler_tun: unexpected type %d (expected DR or AS)\n",
                tmp_pdu->uint_pktType);
        return -1;
    }
    if (tmp_pdu->pls <= 0 || tmp_pdu->pls > TETRYS_MAX_PAYLOAD)
        return 0;   /* empty or corrupt length: nothing to write to TUN */
    /* datav = [int padding][IP packet of pls bytes] (see circ_add_data_in_buffer) */
    tettun->bytes = write(tettun->tun_fd, tmp_pdu->datav + sizeof(int),
                          (size_t)tmp_pdu->pls);
    if (tettun->bytes < 0)
        perror("\noutputHandler_tun: write");
    if (tmp_pdu->uint_pktType == PORT_AS)
        tettun->nbAS_sent++;
    else
        tettun->nbDR_sent++;
    return (int)tettun->bytes;
}

/* ==================================================================
 * output_handler_udp
 * Send a Tetrys protocol packet (TD/TR/TA/TAD/TAR/TAP) over UDP.
 * ================================================================== */
/**
 * Send a Tetrys protocol packet (TD/TR/TA/TAD/TAR/TAP) over UDP.
 * @param tmp_pdu  Packet to send.
 * @return Number of bytes sent, or -1 on an unexpected packet type.
 */
static int output_handler_udp(struct PDU *tmp_pdu)
{
    /* Receiver: don't send ACKs before the sender's address is known */
    if (tettun->dst_insock_red.sin_port == 0)
        return 0;

    switch (tmp_pdu->uint_pktType) {
    case PORT_TD:  tettun->nbTD_sent++;  break;
    case PORT_TR:  tettun->nbTR_sent++;  break;
    case PORT_TAD: tettun->nbTAD_sent++; break;
    case PORT_TAR: tettun->nbTAR_sent++; break;
    case PORT_TA:  tettun->nbTA_sent++;  break;
    case PORT_TAP: tettun->nbTAP_sent++; break;
    default:
        fprintf(stderr, "\noutputHandler_udp: unexpected type %d\n",
                tmp_pdu->uint_pktType);
        return -1;
    }

    tettun->bytes = sendto(tettun->fd_red, tmp_pdu, sizeof(struct PDU), 0,
                           (struct sockaddr *)&tettun->dst_insock_red,
                           tettun->lg);
    if (tettun->bytes < 0)
        fprintf(stderr, "tunnel: UDP send failed (%s): packet of type %d (%zu bytes) lost, "
                "repair packets will compensate\n",
                strerror(errno), tmp_pdu->uint_pktType, sizeof(struct PDU));
    else if (tettun->bytes != (ssize_t)sizeof(struct PDU))
        fprintf(stderr, "tunnel: partial UDP send: %zd / %zu bytes\n",
                tettun->bytes, sizeof(struct PDU));

    return (int)tettun->bytes;
}

/* ==================================================================
 * init_module_tettun
 * Initialise the TUN/Tetrys module: open UDP socket, configure
 * addressing, and set file descriptors to non-blocking mode.
 * Mirrors init_module_tetudp() from tetrysModule_udp.c.
 * ================================================================== */
/**
 * Initialise the TUN module: UDP socket, addressing, non-blocking mode and counters.
 * @param _tun_fd      TUN file descriptor.
 * @param _server      Remote host (sender only, NULL on the receiver).
 * @param _udp_port    UDP port of the Tetrys traffic.
 * @param _packet_len  Unused, kept for symmetry with the UDP module.
 */
static void init_module_tettun(int _tun_fd, char *_server, int _udp_port,
                               int _packet_len)
{
    (void)_packet_len;   /* kept for API symmetry; TUN packets are variable */

    /* calloc: the receiver relies on dst_insock_red.sin_port == 0 until the first packet */
    if ((tettun = calloc(1, sizeof(struct tetrys_tun))) == NULL) {
        fprintf(stderr, "init_module_tettun: malloc failed\n");
        exit(EXIT_FAILURE);
    }

    tettun->tun_fd = _tun_fd;
    tettun->server = _server;

    tettun->fd_red = -1;
    int nonblocking_flag;

    if (tetrys->currentRole == ROLE_SENDER) {
        /* Client/sender: open a bound UDP socket and resolve the server address */
        tettun->dport_red = (uint32_t)_udp_port;
        if ((tettun->host_red =
             gethostbyname2(tettun->server, AF_INET)) == NULL) {
            fprintf(stderr, "init_module_tettun: gethostbyname2 failed\n");
            exit(EXIT_FAILURE);
        }
        tettun->fd_red = server_socket_snd((uint16_t)tettun->dport_red);
        memset(&tettun->dst_insock_red, 0, sizeof(tettun->dst_insock_red));
        memcpy(&tettun->dst_inaddr_red.s_addr,
               tettun->host_red->h_addr_list[0],
               sizeof(tettun->dst_inaddr_red.s_addr));
        tettun->dst_insock_red.sin_family = AF_INET;
        tettun->dst_insock_red.sin_port   = htons((uint16_t)tettun->dport_red);
        memcpy(&tettun->dst_insock_red.sin_addr,
               tettun->host_red->h_addr_list[0],
               sizeof(tettun->dst_inaddr_red.s_addr));
        inet_ntop(AF_INET, &tettun->dst_insock_red.sin_addr,
                  tettun->nom, sizeof(tettun->nom));
        fprintf(stderr, "init_module_tettun: sender → %s:%d\n",
                tettun->nom, _udp_port);

    } else {
        /* Server/receiver: bind and wait for incoming Tetrys packets */
        tettun->dport_red  = (uint32_t)_udp_port;
        tettun->host_red   = NULL;
        tettun->fd_red     = server_socket_rcv((uint16_t)tettun->dport_red);
        fprintf(stderr, "init_module_tettun: receiver listening on port %d\n",
                _udp_port);
    }

    if (tettun->fd_red < 0)
        PERROR("init_module_tettun: failed to open UDP socket");

    /* Put the UDP socket in non-blocking mode (select drives reading) */
    if ((nonblocking_flag = fcntl(tettun->fd_red, F_GETFL, 0)) < 0)
        perror("F_GETFL fd_red");
    nonblocking_flag |= O_NONBLOCK;
    if (fcntl(tettun->fd_red, F_SETFL, nonblocking_flag) < 0)
        perror("F_SETFL fd_red");

    /* Put the TUN fd in non-blocking mode as well */
    if ((nonblocking_flag = fcntl(tettun->tun_fd, F_GETFL, 0)) < 0)
        perror("F_GETFL tun_fd");
    nonblocking_flag |= O_NONBLOCK;
    if (fcntl(tettun->tun_fd, F_SETFL, nonblocking_flag) < 0)
        perror("F_SETFL tun_fd");

    tettun->lg = sizeof(tettun->server_address);

    /* Reset all packet counters */
    tettun->nbTD_sent = tettun->nbTR_sent  = 0;
    tettun->nbTAD_sent = tettun->nbTAR_sent = 0;
    tettun->nbTA_sent  = tettun->nbTAP_sent = 0;
    tettun->nbDR_sent  = tettun->nbAS_sent  = 0;
    tettun->nbTD_rcvd  = tettun->nbTR_rcvd  = 0;
    tettun->nbTAD_rcvd = tettun->nbTAR_rcvd = 0;
    tettun->nbTA_rcvd  = tettun->nbTAP_rcvd = 0;
}

/* ==================================================================
 * init_context
 * Register I/O handlers with the Tetrys dispatch tables.
 * Mirrors init_context() in tetrysModule_udp.c.
 * ================================================================== */
/**
 * Register the I/O handlers in the Tetrys dispatch tables (TUN and UDP).
 */
static void init_context(void)
{
    FD_ZERO(&tetrys->tet_fd_rset);

    /* --- Input handlers ------------------------------------------ */
    /* fd_fun_tab[0]: TUN fd → application data (→ DIBuffer, return 1) */
    tetrys->fd_fun_tab[0].fd     = tettun->tun_fd;
    tetrys->fd_fun_tab[0].fd_fun = input_handler_tun;

    /* fd_fun_tab[1]: UDP socket → Tetrys packets (→ TIBuffer, return 2/3) */
    tetrys->fd_fun_tab[1].fd     = tettun->fd_red;
    tetrys->fd_fun_tab[1].fd_fun = input_handler_udp;

    tetrys->nb_fd_fun = 2;
    FD_SET(tettun->tun_fd, &tetrys->tet_fd_rset);
    FD_SET(tettun->fd_red, &tetrys->tet_fd_rset);

    /* --- Output handlers ----------------------------------------- */
    /* Tetrys protocol packets go over UDP */
    tetrys->out_fun_tab[0].pkttype = PORT_TD;  tetrys->out_fun_tab[0].out_fun = output_handler_udp;
    tetrys->out_fun_tab[1].pkttype = PORT_TR;  tetrys->out_fun_tab[1].out_fun = output_handler_udp;
    tetrys->out_fun_tab[2].pkttype = PORT_TA;  tetrys->out_fun_tab[2].out_fun = output_handler_udp;
    tetrys->out_fun_tab[3].pkttype = PORT_TAR; tetrys->out_fun_tab[3].out_fun = output_handler_udp;
    tetrys->out_fun_tab[4].pkttype = PORT_TAD; tetrys->out_fun_tab[4].out_fun = output_handler_udp;
    tetrys->out_fun_tab[5].pkttype = PORT_TAP; tetrys->out_fun_tab[5].out_fun = output_handler_udp;

    /* Decoded data goes to the TUN interface */
    tetrys->out_fun_tab[6].pkttype = PORT_DR;  tetrys->out_fun_tab[6].out_fun = output_handler_tun;
    tetrys->out_fun_tab[7].pkttype = PORT_AS;  tetrys->out_fun_tab[7].out_fun = output_handler_tun;

    tetrys->nb_out_fun = 8;
}

/* ==================================================================
 * Signal handlers and helpers (mirrors tetrys_udp_main.c)
 * ================================================================== */

/**
 * Shutdown signal handler: flag that termination was requested (async-signal-safe).
 * @param sig  Signal number received.
 */
static void handle_stop_signal(int sig)
{
    tetrys_stop_requested = (sig_atomic_t)sig;
}

/** Close module descriptors and free @c tettun. */
static void cleanup_module_tettun(void)
{
    if (tettun == NULL)
        return;
    if (tettun->fd_red >= 0) {
        close(tettun->fd_red);
        tettun->fd_red = -1;
    }
    if (tettun->tun_fd >= 0) {
        close(tettun->tun_fd);
        tettun->tun_fd = -1;
    }
    free(tettun);
    tettun = NULL;
}

/** Print sent/received tunnel packet counters at shutdown. */
static void tun_print_summary(int sig)
{
    fprintf(stderr, "\nTetrys TUN tunnel: signal %d received, shutting down.\n", sig);
    if (tettun) {
        fprintf(stderr,
                "Sent  — TD:%d TR:%d TA:%d TAD:%d TAR:%d TAP:%d DR:%d AS:%d\n",
                tettun->nbTD_sent, tettun->nbTR_sent, tettun->nbTA_sent,
                tettun->nbTAD_sent, tettun->nbTAR_sent, tettun->nbTAP_sent,
                tettun->nbDR_sent, tettun->nbAS_sent);
        fprintf(stderr,
                "Rcvd  — TD:%d TR:%d TA:%d TAD:%d TAR:%d TAP:%d\n",
                tettun->nbTD_rcvd, tettun->nbTR_rcvd, tettun->nbTA_rcvd,
                tettun->nbTAD_rcvd, tettun->nbTAR_rcvd, tettun->nbTAP_rcvd);
    }
}

void manage_signal(void)
{
    struct sigaction sa_alarm;
    struct sigaction sa_stop;

    signal(SIGHUP, SIG_IGN);
    signal(SIGFPE, float_error);

    memset(&sa_alarm, 0, sizeof(sa_alarm));
    sa_alarm.sa_handler = tetrys_handle_alarm;
    sigemptyset(&sa_alarm.sa_mask);
    sigaction(SIGALRM, &sa_alarm, NULL);

    memset(&sa_stop, 0, sizeof(sa_stop));
    sa_stop.sa_handler = handle_stop_signal;
    sigemptyset(&sa_stop.sa_mask);
    sigaction(SIGINT,  &sa_stop, NULL);
    sigaction(SIGTERM, &sa_stop, NULL);
    sigaction(SIGQUIT, &sa_stop, NULL);

    sigemptyset(&tetrys->block_alarm);
    sigaddset(&tetrys->block_alarm, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &tetrys->block_alarm, NULL);
}

_Noreturn void quit(int sig)
{
    tun_print_summary(sig);
    cleanup_module_tettun();
    tetrys_cleanup();
    exit(EXIT_FAILURE);
}

/**
 * Abort when the role or the FEC parameters are inconsistent.
 */
static void check_tetrys_config(void)
{
    if (tetrys->currentRole == ROLE_SENDER &&
        (tetrys->fec_n == 0 || tetrys->fec_k == 0)) {
        fprintf(stderr, "check_tetrys_config: fec_n or fec_k is 0\n");
        exit(EXIT_FAILURE);
    }
    if (tetrys->currentRole < ROLE_SENDER ||
        tetrys->currentRole > ROLE_RECEIVER) {
        fprintf(stderr, "check_tetrys_config: role must be 1 (sender) or 2 (receiver)\n");
        exit(EXIT_FAILURE);
    }
}

/* ==================================================================
 * tetrys_init_all
 * Full Tetrys + TUN module initialisation.
 * Mirrors tetrys_init_all() in tetrys_udp_main.c but calls
 * init_module_tettun() instead of init_module_tetudp().
 * ================================================================== */
/**
 * Allocate and initialise the whole Tetrys engine and the TUN module.
 * @param _tun_fd   TUN file descriptor.
 * @param _fec_n    Core repair cadence parameter (k - 1; 1 repair every _fec_n + 1 data pkts in Tetrys mode).
 * @param _fec_k    Core redundancy parameter (n - k).
 * @param _role     ROLE_SENDER or ROLE_RECEIVER.
 * @param _fecMode  0 for Tetrys, 1 for block FEC.
 * @param _server   Remote host (sender only).
 * @param _port     UDP port.
 * @return 0.
 */
static int tetrys_init_all(int _tun_fd, int _fec_n, int _fec_k,
                            int _role, int _fecMode,
                            char *_server, int _port)
{
    tetrys_alloc_globals(_role, TUN_PKT_LEN);

    tetrys->fecMode     = 0;
    tetrys->currentRole = 0;

    sig_AckRed->rep_freq     = 0.1;
    sig_AckRed->ack_freq     = 0.05;
    sig_AckRed->next_rep     = 0;
    sig_AckRed->next_ack     = 0;
    sig_AckRed->rep_enabled  = 0;
    sig_AckRed->ack_enabled  = 0;
    sig_AckRed->datf_enabled = 0;

    tetrys->fec_n          = _fec_n;
    tetrys->fec_k          = _fec_k;
    tetrys->currentRole    = _role;
    tetrys->fecMode        = _fecMode;
    tetrys->nodeid         = (uint16_t)tetrys->currentRole;
    tetrys->deliverInOrder = 1;

    sig_AckRed->ack_enabled = (tetrys->currentRole == ROLE_RECEIVER) ? 1 : 0;
    /*
     * The sender also emits a repair every rep_freq seconds while some packets
     * are not acknowledged yet (send_redundancy() sends nothing otherwise). A
     * repair is otherwise sent only every fec_n data packets, so the last
     * packets of a flow, or a lost repair, would never be recovered.
     */
    sig_AckRed->rep_enabled = (tetrys->currentRole == ROLE_SENDER) ? 1 : 0;

    check_tetrys_config();
    init_module_tettun(_tun_fd, _server, _port, TUN_PKT_LEN);
    init_context();

#ifdef VERBOSE
    fprintf(stderr, "tetrys_init_all: done (role=%d fec_n=%d fec_k=%d port=%d)\n",
            _role, _fec_n, _fec_k, _port);
#endif
    return 0;
}

/* ==================================================================
 * tetrys_main_run
 * Tetrys protocol main loop.
 * Mirrors tetrys_main_run() in tetrys_udp_main.c.
 * ================================================================== */
/**
 * Tetrys protocol main loop; exits cleanly on SIGINT/SIGTERM/SIGQUIT.
 * @param arg  Unused.
 * @return Always NULL.
 */
static void *tetrys_main_run(void *arg)
{
    (void)arg;

    manage_signal();

    /* The receiver re-calls init_context after signal setup */
    if (tetrys->currentRole == ROLE_RECEIVER)
        init_context();

#ifdef STATS
    if (tetrys->currentRole != ROLE_RECEIVER)
        sndStats->startTime = get_time();
    if (tetrys->currentRole != ROLE_SENDER)
        rcvStats->startTime = get_time();
#endif

    if (tetrys->currentRole == ROLE_RECEIVER && sig_AckRed->ack_enabled)
        actuate_alarm(1);
    if (tetrys->currentRole == ROLE_SENDER   && sig_AckRed->rep_enabled)
        actuate_alarm(2);

    srand(4);

    while (!tetrys_stop_requested) {
        if (!tetrys_step_once())
            break;
    }

    tetrys_disarm_alarm();
    tun_print_summary((int)tetrys_stop_requested);
    cleanup_module_tettun();
    tetrys_cleanup();
    return NULL;
}

/* ==================================================================
 * usage
 * ================================================================== */
/**
 * Print the command-line help and exit.
 */
static void usage(void)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr,
            "  %s -i <ifacename> -c <serverIP> [-p <port>] [-n <fec_n>] [-k <fec_k>]\n",
            progname);
    fprintf(stderr, "  %s -i <ifacename> -s [-p <port>]\n", progname);
    fprintf(stderr, "  %s -h\n\n", progname);
    fprintf(stderr, "  -i <ifacename>  TUN interface name (mandatory)\n");
    fprintf(stderr, "  -s              receiver/server mode (role 2)\n");
    fprintf(stderr, "  -c <serverIP>   sender/client mode (role 1), server IP\n");
    fprintf(stderr, "  -p <port>       UDP port (default 30001)\n");
    fprintf(stderr, "  -n <fec_n>      FEC block length n (default 10, sender only; requires 2 <= k < n)\n");
    fprintf(stderr, "  -k <fec_k>      FEC data symbols k (default 8, sender only; 1 repair every k data pkts)\n");
    fprintf(stderr, "  -h              this help text\n");
    exit(1);
}

/* ==================================================================
 * main
 * ================================================================== */
/**
 * Entry point: parse the options, open the TUN device and run the tunnel.
 */
int main(int argc, char *argv[])
{
    int  option;
    int  flags = IFF_TUN;
    char if_name[IFNAMSIZ] = "";
    char remote_ip[16]     = "";
    unsigned short port    = 30001;
    int  cliserv           = -1;
    int  fec_n             = 10;
    int  fec_k             = 8;
    int  tun_fd;

    progname = argv[0];

    while ((option = getopt(argc, argv, "i:sc:p:n:k:h")) > 0) {
        switch (option) {
        case 'h': usage();                                    break;
        case 'i': snprintf(if_name, sizeof(if_name), "%s", optarg); break;
        case 's': cliserv = SERVER;                           break;
        case 'c': cliserv = CLIENT;
                  snprintf(remote_ip, sizeof(remote_ip), "%s", optarg); break;
        case 'p': port  = (unsigned short) parse_long_arg(optarg, 1, 65535, "-p"); break;
        case 'n': fec_n = (int) parse_long_arg(optarg, 1, 255, "-n"); break;
        case 'k': fec_k = (int) parse_long_arg(optarg, 1, 255, "-k"); break;
        default:
            fprintf(stderr, "Unknown option %c\n", option);
            usage();
        }
    }

    argc -= optind;

    if (argc > 0)
        { fprintf(stderr, "Too many options!\n");           usage(); }
    if (*if_name == '\0')
        { fprintf(stderr, "Must specify interface name!\n"); usage(); }
    if (cliserv < 0)
        { fprintf(stderr, "Must specify -s or -c!\n");      usage(); }
    if (cliserv == CLIENT && *remote_ip == '\0')
        { fprintf(stderr, "Must specify server address!\n"); usage(); }
    if (cliserv == CLIENT && (fec_k < 2 || fec_n <= fec_k)) {
        fprintf(stderr, "Error: sender requires 2 <= k < n (got n=%d, k=%d)\n",
                fec_n, fec_k);
        usage();
    }

    /* Open TUN interface */
    if ((tun_fd = tun_alloc(if_name, flags | IFF_NO_PI)) < 0) {
        fprintf(stderr, "Error connecting to TUN interface %s!\n", if_name);
        exit(1);
    }
    printf("Successfully connected to TUN interface %s\n", if_name);

    /* Map client/server to Tetrys roles and convert (-n, -k) like tetrys_generate_udp */
    int   role   = (cliserv == CLIENT) ? ROLE_SENDER : ROLE_RECEIVER;
    char *server = (cliserv == CLIENT) ? remote_ip   : NULL;
    int   core_fec_n = (role == ROLE_SENDER) ? (fec_k - 1) : 0;
    int   core_fec_k = (role == ROLE_SENDER) ? (fec_n - fec_k) : 0;

    /* Initialise Tetrys engine + TUN module */
    tetrys_init_all(tun_fd, core_fec_n, core_fec_k, role, 0, server, port);

    printf("Tetrys TUN tunnel ready — %s, port %d",
           (role == ROLE_SENDER) ? "sender" : "receiver", port);
    if (role == ROLE_SENDER)
        printf(", FEC n=%d k=%d (1 repair / %d data pkts), server=%s",
               fec_n, fec_k, fec_k, remote_ip);
    printf("\n");

    /* Enter the Tetrys main loop (blocks indefinitely) */
    tetrys_main_run(NULL);

    return 0;   /* unreachable */
}
