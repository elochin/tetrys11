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

#ifndef COMMON_STRUCT_DEF_H
#define COMMON_STRUCT_DEF_H

#include "rand31.h"

#include <fcntl.h>
#include <netdb.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <limits.h>
#include <semaphore.h>
#include <stdatomic.h>

#ifndef PR_DB
#define PR_DB 0
#endif
#ifndef PR_DB_1
#define PR_DB_1 0
#endif
#ifndef PR_DB_2
#define PR_DB_2 0
#endif
/**
 * Build with -DMONITOR to write the coding traces monitorFEC.snd and monitorSub.rcv
 * (and the three other monitor*.snd/.rcv files, which stay empty) in the working
 * directory. It is 0, so nothing is created, by default. Test it with `#if MONITOR`,
 * never `#ifdef MONITOR`.
 */
#ifndef MONITOR
#define MONITOR 0
#endif
#ifndef GF_BITS
#define GF_BITS  8		/* code over GF(2**GF_BITS) - change to suit */
#define	GF_SIZE ((1 << GF_BITS) - 1)	/* powers of \alpha */
#if (GF_BITS <= 8)
typedef unsigned char gf;
#else
typedef unsigned short gf;
#endif
#endif


#define MIN(a,b)  ((a)<(b)?(a):(b))
#define MAX(a,b)  ((a)>(b)?(a):(b))

/** Maximum size of a raw UDP receive buffer (bytes). */
#define BUFSIZE 65535

/////////////////////////////////////

/////////////////////////////////////
#ifndef PDUSTRUCT
# define PDUSTRUCT 1

/*
 * UDP port assignments for each Tetrys packet type.
 * The packet type is encoded in the destination port to distinguish
 * data (TD), repair (TR), and acknowledgement (TA/TAD/TAR/TAP) flows.
 * See the "Tetrys buffers and procedures" page for the figures.
 */
# define PORT_TD  660  /**< Tetrys data packet, sender -> receiver */
# define PORT_TR  661  /**< Tetrys repair packet, sender -> receiver */
# define PORT_TAD 662  /**< Data packet with a piggybacked ACK (both-ways role 0) */
# define PORT_TAR 663  /**< Repair packet with a piggybacked ACK (both-ways role 0) */
# define PORT_TA  664  /**< ACK-only packet, receiver -> sender (ack_freq timer) */
# define PORT_TAS 665  /**< Alias value of PORT_DS (legacy) */
# define PORT_DS  665  /**< Application packet, SenderApp -> Tetrys sender */
# define PORT_TAP 666  /**< Application packet of the ReceiverApp (AR) wrapped with a piggybacked Tetrys ACK, receiver -> sender */
# define PORT_AR  667  /**< Application packet, ReceiverApp -> Tetrys receiver */
# define PORT_DR  668  /**< Data delivered by the Tetrys receiver to the ReceiverApp */
# define PORT_AS  669  /**< Packet delivered by the Tetrys sender to the SenderApp (payload of a received TAP) */

/** Size of the data area of a Tetrys PDU (bytes): a 4-byte padding header followed by up to PLS - 4 bytes of application payload. */
# define PLS 1304
/** Maximum application payload size inside a Tetrys PDU (bytes). */
# define TETRYS_MAX_PAYLOAD (PLS - (int)sizeof(int))

/** Maximum number of data packets that can be subtracted from a repair symbol. */
# define SIZE_SUBTRACT_VECTOR 200

/** Number of past packets tracked for loss-rate estimation. */
#define TETRYS_LOSS_HISTORY 100

/** Number of past inter-packet intervals tracked for rate estimation. */
#define TETRYS_INTERPKT_HISTORY 25

/**
 * \enum TetrysRole
 * \brief Operating mode of a Tetrys node.
 */
typedef enum {
    ROLE_BIDIRECTIONAL = 0, /**< Node acts as both sender and receiver */
    ROLE_SENDER        = 1, /**< Node acts as sender only */
    ROLE_RECEIVER      = 2  /**< Node acts as receiver only */
} TetrysRole;

/**
 * \struct PDU common_struct_def.h
 * \brief Tetrys protocol data unit. Used for all packet types:
 *        data (TD/TAD), repair (TR/TAR), and acknowledgement (TA/TAS/TAP).
 *
 * For data packets, @c datav starts with a host-endian @c int padding length
 * (@c TETRYS_MAX_PAYLOAD - @c pls) followed immediately by @c pls payload bytes.
 * The remaining bytes up to @c PLS are zero-padded so FEC encoding treats every
 * symbol uniformly as @c PLS bytes.
 */
#ifndef NS2
struct PDU {
    /** Packet type identifier — one of the PORT_T* constants.
     *  If the value matches a known PORT_T* constant, it is a Tetrys packet. */
    uint16_t uint_pktType;
    /** Reserved/padding — used internally by circularBuffer to mark empty slots. */
    uint16_t empty;
    /** Node identifier of the originating node. */
    uint16_t nodeid;
    /** Payload size in bytes. Non-zero for TD/TR/TAD/TAR/TAP; 0 for pure ACKs. */
    int pls;
    /** FEC mode only: encoding window index the packet belongs to (0 otherwise). */
    int currentWindow;
    /** FEC mode only: position of this packet within its encoding window. */
    int currentPktInWindow;
    /** Data sequence number (TD/TAD packets); 0 for repair and ACK packets. */
    unsigned long num;
    /** Repair symbol identifier (TR/TAR packets); 0 for data and ACK packets. */
    unsigned long repair_id;
    /** Timestamp of this packet at the sending node (used for RTT probing). */
    double local_rtt_probe;
    /** Most recent timestamp echoed from the remote node (used for RTT probing). */
    double dst_rtt_probe;
    /** Smoothed RTT estimate at the sending node (seconds). */
    double rtt_estimation;
    /** Loss rate estimate at the sending node (fraction in [0,1]). */
    float lossRate;
    /** Lower bound of the redundancy window:
     *  TD/TAD/TAR/TR: current min sequence number covered by repair;
     *  TA/TAP/FD/FR: 0. The receiver buffer (RRBuffer) is pruned up to this value. */
    unsigned long int min_redundancy;
    /** Upper bound of the redundancy window:
     *  TR/TAR: highest data sequence number covered by this repair symbol;
     *  TD/TAD/TA/TAS/FD/FR: 0. */
    unsigned long int max_redundancy;
    /** Cumulative ACK: highest data sequence number below which all packets were
     *  received (TAD/TAR/TA/TAS packets); 0 for TR/TD/FD/FR. */
    unsigned long int ack_min_redundancy;
    /** Payload data (or FEC symbol) — up to PLS bytes. */
    gf datav[PLS];
};
_Static_assert(sizeof(struct PDU) == 1400, "struct PDU wire layout must stay 1400 bytes");
#endif				/*NS2 */

#endif				/* PDUSTRUCT */


#ifndef TETRYS_STRUCT
# define TETRYS_STRUCT 1
#define	FILE_MODE	(00400 | 00200 | 00040 | 00004)
	/*!
	 * \brief Generic input handler type.
	 *  \return the action to perform. if incoming:
	 * - -1 if failed to parse the packet
	 * - 1 to add the packet in DIBuffer
	 * - 2 to add the packet in TIBuffer
	 * - 3 to treat the packet as an Ack
	 *  \return the action to perform. if outgoing:
	 *  int fd depict the type of packet
	 *  If fd math the types of packet, call the function fd_function fd_fun. This function assum that the init things  (files descriptor, socket ...) were previously done and are ready to use.
	 * \return the action to perform. if incoming:( 1= dat; 2=repair; 3=
	 */
typedef int (*fd_function) (int cur_fd, gf * pkt_to_parse);
/** Association between an input file descriptor and its handler. */
struct FD_FUN {
    int fd;                 /**< Input file descriptor (socket, pipe or TUN device). */
    fd_function fd_fun;     /**< Handler called when @c fd is readable. */
};
typedef int (*out_function) (struct PDU * pdu_to_send);
/** Association between a packet type and the handler that emits it. */
struct OUT_FUN {
    uint16_t pkttype;       /**< Packet type (PORT_TD, PORT_TR, PORT_DR, ...). */
    out_function out_fun;   /**< Handler called to send a packet of this type. */
};

/**
 * \struct TETRYS common_struct_def.h
 * \brief Master state structure for a Tetrys protocol instance (singleton,
 *        not reentrant — accessed through global pointer ::tetrys).
 *
 * Holds all state needed to encode, decode, buffer and forward packets.
 * Owned by the caller via tetrys_alloc_globals() / init_tetrys() and freed
 * with all buffers and recovery arrays by tetrys_cleanup().
 * Parameters can be tuned at runtime to adapt to network conditions
 * (loss rate, RTT, encoding ratio, etc.).
 *
 * Fields are grouped into logical sections:
 *   - General configuration
 *   - RTT and rate estimation
 *   - Loss history and inter-packet timing
 *   - Goodput and recovery statistics
 *   - Sequence number tracking
 *   - Refresh-window bookkeeping
 *   - FEC decoding state
 *   - Packet buffers (circular queues)
 *   - FEC-mode specific state
 *   - Output / pipe / file handles
 *   - Encoding parameters
 *   - I/O multiplexing
 *   - Thread synchronisation (optional)
 */
struct TETRYS {
    /* ------------------------------------------------------------------
     * General configuration
     * ------------------------------------------------------------------ */
    int closePending;         /**< Non-zero when a graceful shutdown has been requested. */
    int currentRole;          /**< Node role: ROLE_BIDIRECTIONAL=0, ROLE_SENDER=1, ROLE_RECEIVER=2. */
    int deliverInOrder;       /**< 1: deliver packets to upper layer in sequence order;
                                   0: deliver as received or decoded. */
    int fecMode;              /**< 1: receiver operates as a pure FEC block decoder
                                   (flushes after each encoding window); 0: streaming mode. */
    int currentWindow;        /**< FEC mode: index of the encoding window currently being processed. */
    unsigned long int nextExpected; /**< Next data sequence number expected by the receiver. */
    int p_size;               /**< Packet payload size configured at init. */
    int pktSize;              /**< Full packet size (header + payload). */

    /* ------------------------------------------------------------------
     * RTT and sending-rate estimation
     * ------------------------------------------------------------------ */
    double ts_last_ack_sent;  /**< Timestamp of the last ACK sent (seconds since epoch). */
    double rtt;               /**< Current smoothed RTT estimate (seconds). */
    double previous_rtt;      /**< RTT estimate from the previous update cycle. */
    double rtt_rate;          /**< Rate of RTT change (derivative). */
    double last_rcv_time;     /**< Timestamp of the last received packet. */
    double rtt_smooth_param;  /**< Smoothing coefficient for the EWMA RTT filter (0–1). */
    /* RTT probing fields (added 2010) */
    double last_probe_rcvd;       /**< Value of the last RTT probe received from the remote. */
    double ts_of_last_probe_rcvd; /**< Timestamp at which the last RTT probe arrived. */
    double dst_rtt;               /**< RTT estimate reported by the remote node. */

    /* ------------------------------------------------------------------
     * Loss history and inter-packet timing
     * ------------------------------------------------------------------ */
    /** Per-loss-event recovery delay history (up to 1000 entries, seconds). */
    double delayByLoss[1000];
    /** Sliding window of the last TETRYS_LOSS_HISTORY packet outcomes:
     *  0 = no information, 1 = received, 2 = lost and rebuilt.
     *  Loss rate ≈ count(2) / count(!=0). */
    unsigned char lossHistory[TETRYS_LOSS_HISTORY];
    /** History of inter-packet arrival intervals at the input (seconds). */
    double interPktIHistory[TETRYS_INTERPKT_HISTORY];
    /** History of inter-packet departure intervals at the output (seconds). */
    double interPktOHistory[TETRYS_INTERPKT_HISTORY];
    double sensedPktORate;    /**< Estimated output packet rate (packets/second). */
    double sensedPktIRate;    /**< Estimated input packet rate (packets/second). */
    double lastInterPktI_ts;  /**< Timestamp of the previous input packet (for interval calc). */
    double lastInterPktO_ts;  /**< Timestamp of the previous output packet (for interval calc). */
    FILE *lossTimeRecovery;   /**< File handle for logging loss-recovery delay traces. */
    int nb_loss_delay;        /**< Number of loss-delay samples recorded so far. */

    /* ------------------------------------------------------------------
     * Goodput and delivery statistics
     * ------------------------------------------------------------------ */
    int nb_sent_to_upper_layer;      /**< Total packets forwarded to the application. */
    int nb_rebuilt;                  /**< Total packets recovered via FEC decoding. */
    int nbOldDatOrRedRcvdSinceLastTry; /**< Count of duplicate/old packets received since
                                            the last decoding attempt. */

    /* ------------------------------------------------------------------
     * Sequence number tracking
     * ------------------------------------------------------------------ */
    unsigned long int max_d_seq_num_seen; /**< Highest data sequence number observed. */
    unsigned long int max_r_seq_num_seen; /**< Highest repair sequence number observed. */
    unsigned long int max_seq_num_seen;   /**< MAX(max_d_seq_num_seen, max_r_seq_num_seen). */
    /** Minimum data sequence number still needed for repair subtraction;
     *  equals the highest min_redundancy value seen at the last refresh. */
    unsigned long int min_data_pkt_to_retain;
    /** Highest min_redundancy value received with any repair packet.
     *  Data packets with sequence number below this value can be discarded
     *  after recovery. */
    int highest_min_r_rcvd;
    unsigned long int tot_d_pkt_rcv; /**< Cumulative count of data packets received. */
    unsigned long int tot_r_pkt_rcv; /**< Cumulative count of repair packets received. */
    unsigned long int tot_p_pkt_rcv; /**< Cumulative count of all UDP packets received. */
    unsigned long int nb_rep_sent;   /**< Total repair packets sent. */
    unsigned long int nb_ack_rcv;    /**< Total ACK packets received. */
    unsigned long int max_acked_minus_two; /**< max_acked - 2; used for ACK hysteresis. */

    /* ------------------------------------------------------------------
     * Refresh-window bookkeeping
     * A "refresh event" is either a recovery event or a repair reception
     * with no detected loss.
     * ------------------------------------------------------------------ */
    /** Queue index of p_seq_num_of_last_refresh; 0 when the window is fully ACKed. */
    int queue_num_of_first_wind_pkt;
    /** Number of data packets received since the last refresh event. */
    int nb_d_since_last_refresh;
    /** Number of repair packets received since the last refresh event. */
    int nb_r_since_last_refresh;
    /** Number of UDP packets received since the last refresh event. */
    int nb_p_since_last_refresh;
    /** max_d_seq_num_seen + 1 at the time of the last refresh; serves as
     *  sequence number "0" for the current repair window. */
    unsigned long int p_seq_num_of_last_refresh;
    /** Queue index of the highest data packet received at the last refresh. */
    int queue_num_of_last_refresh;
    int snd_pkt_since_last_repair;  /**< Sender: packets sent since last repair was emitted. */
    int currentPktInWindow;         /**< Position of the current packet within its window. */
    /** Number of data packets currently stored in the receive queue;
     *  should equal nb_d_since_last_refresh. */
    int nb_d_in_queue;
    int repair_pending;             /**< Non-zero when a repair packet is ready to be sent. */

    /* ------------------------------------------------------------------
     * FEC decoding state
     * ------------------------------------------------------------------ */
    unsigned long int *lost_positions; /**< Array of sequence numbers of lost packets. */
    int nb_pkt_lost;                   /**< Number of packets currently missing. */
    unsigned long int max_to_decode;   /**< Highest sequence number involved in decoding. */
    unsigned long int max_red_useful;  /**< Highest repair sequence number still useful. */
    int padding_on_last_pkt;           /**< Padding bytes appended to the last packet in window. */
    int lost_positions_cap;            /**< Allocated length of the lost_positions array. */
    gf **lost_to_recover;             /**< Contiguous-slab row pointers (nb_lostArray entries of @c pktSize bytes). */

    /* ------------------------------------------------------------------
     * Packet buffers (circular queues)
     *
     * Naming convention:
     *   SE  = Sender Encoding buffer   (data queued for encoding)
     *   RR  = Receiver Repair buffer   (repair symbols waiting for use)
     *   RD  = Receiver Data buffer     (data packets waiting for delivery)
     *   O   = Output buffer            (packets ready to send on the wire)
     *   TI  = Tetrys Input buffer      (packets received from the network)
     *   DI  = Data Input buffer        (data from the application layer)
     *   FO  = FEC Output buffer        (decoded blocks ready for delivery)
     * ------------------------------------------------------------------ */
    struct circBUFFER *SEBuffer;   /**< Sender encoding window — data to be covered by repairs. */
    struct circBUFFER *RRBuffer;   /**< Receiver repair queue — incoming repair symbols. */
    struct circBUFFER *RDBuffer;   /**< Receiver data queue — received data packets. */
    struct circBUFFER *OBuffer;    /**< Output queue — packets pending transmission. */
    struct circBUFFER *TIBuffer;   /**< Network input queue — packets from the wire. */
    struct circBUFFER *DIBuffer;   /**< Application input queue — data from upper layer. */
    struct circBUFFER *tmpFecRep;  /**< Temporary repair buffer used during FEC decoding. */
    struct circBUFFER *tmpFecDat;  /**< Temporary data buffer used during FEC decoding. */
    struct circBUFFER *FOBuffer;   /**< FEC output buffer — decoded blocks awaiting delivery. */

    /* ------------------------------------------------------------------
     * FEC-mode specific state
     * ------------------------------------------------------------------ */
    unsigned long int lastRebuiltBlock[100]; /**< Sequence numbers of the most recently rebuilt blocks. */
    unsigned long int blockId[600];          /**< Block identifiers for FEC window management. */
    unsigned long int fec_currentWindow;     /**< Index of the FEC encoding window being processed. */
    unsigned long int fec_next_expected;     /**< Next expected block index in FEC mode. */
    int nbOfFOBtoOB;          /**< Number of FEC output blocks moved to the output buffer. */
    int nb_tot_fec_lost;      /**< Total FEC symbols lost across all windows. */
    unsigned long int fecMaxCurWind; /**< Highest sequence number in the current FEC window. */
    int nbLastRebuiltBlock;   /**< Number of entries in lastRebuiltBlock[]. */

    /* ------------------------------------------------------------------
     * Timestamps
     * ------------------------------------------------------------------ */
    double rcv_time;      /**< Timestamp of the most recently received packet. */
    double start_time;    /**< Timestamp when the session started. */
    double begintime;     /**< Timestamp of the first packet processed. */

    /* ------------------------------------------------------------------
     * Encoding parameters
     * ------------------------------------------------------------------ */
    float lossRate;           /**< Current loss rate estimate used to set redundancy (0–1). */
    int fec_n;                /**< Streaming mode: send one repair every @c fec_n + 1 data packets; FEC block mode: n - k check symbols per block. */
    int fec_k;                /**< FEC block mode: k data symbols per block (unused in streaming mode). */
    /** Sequence number to assign to the next outgoing packet. */
    unsigned long int next_seqnumToAdd;
    unsigned long int min_seq_in_red; /**< Lowest sequence number still in the active encoding window (driven by peer ACKs). */
    unsigned long int max_seq_in_red; /**< Highest sequence number currently covered by a repair symbol. */

    /* ------------------------------------------------------------------
     * Monitor / trace file handles (enabled by compile-time flags)
     * ------------------------------------------------------------------ */
    FILE *monitorSnd;     /**< Sender encoding trace (MONITOR flag). */
    FILE *monitorRcv;     /**< Receiver decoding trace (MONITOR flag). */
    FILE *monitorSub;     /**< Subtraction trace (MONITOR flag). */
    FILE *monitorFecSnd;  /**< FEC sender trace (MONITOR flag). */
    FILE *monitorFecRcv;  /**< FEC receiver trace (MONITOR flag). */
    FILE *savePkt;        /**< Raw packet dump file (SAVEPACKET flag). */

    /* ------------------------------------------------------------------
     * I/O multiplexing and temporary buffers
     * ------------------------------------------------------------------ */
    gf pkt_to_parse[BUFSIZE];  /**< Scratch buffer for the most recently received raw packet. */
    struct PDU tmp_rep_pdu;    /**< Scratch PDU used when building outgoing repair packets. */
    int incoming_length;       /**< Byte length of the last packet read into pkt_to_parse. */
    unsigned long nb_incoming_tet;  /**< Count of Tetrys packets received. */
    unsigned long nb_incoming_dat;  /**< Count of data packets received from application. */
    unsigned long nb_outgoing_item; /**< Count of items sent to the output buffer. */
    struct timeval tmp_timeval;     /**< Scratch timeval for gettimeofday calls. */
    struct PDU tmp_pdu;             /**< Scratch PDU for transient packet parsing. */
    uint16_t nodeid;                /**< Local node identifier. */
    int nbOBFailed;                 /**< Number of times the output buffer was full. */
    unsigned long int freqTryToDecode; /**< How often try_to_decode() is called (packets). */

    /* ------------------------------------------------------------------
     * End-of-transfer thread synchronisation
     * ------------------------------------------------------------------ */
    _Atomic int end_transfer_requested; /**< Set by app thread to trigger end-of-transfer flushing. */
    int end_transfer_active;            /**< Set by protocol thread once 10 ms repair mode is armed. */
    _Atomic int transfer_acked;         /**< Set by protocol thread once all queued data are ACKed. */

    /* ------------------------------------------------------------------
     * Versatile I/O dispatch tables
     * Each entry maps a file descriptor (or packet type) to a handler.
     * Up to 10 handlers are supported per direction.
     * ------------------------------------------------------------------ */
    fd_set tet_fd_rset;               /**< fd_set used by select() for input multiplexing. */
    struct FD_FUN fd_fun_tab[10];     /**< Input handler dispatch table (fd → function). */
    int nb_fd_fun;                    /**< Number of registered input handlers. */
    int max_fd_set;                   /**< Highest fd number in tet_fd_rset (for select). */
    struct OUT_FUN out_fun_tab[10];   /**< Output handler dispatch table (pkttype → function). */
    int nb_out_fun;                   /**< Number of registered output handlers. */
    sigset_t block_alarm;             /**< Signal mask used to defer SIGALRM during critical sections. */
    struct circBUFFERItem *tmp_circ_item; /**< Scratch pointer for circular buffer traversal. */
};
extern struct TETRYS *tetrys;

/**
 * \struct SIG_ACK_RED common_struct_def.h
 * \brief Timer state for SIGALRM-driven periodic ACK and repair emission.
 *
 * The SIGALRM handler reads this structure to decide whether to send
 * an ACK, a repair packet, or a data flush at the current time.
 * All timestamps are in seconds (same epoch as get_time()).
 */
struct SIG_ACK_RED {
    double endAt;         /**< Session end time; when now > endAt the session terminates. */
    /* ACK scheduling */
    double next_ack;      /**< Timestamp at which the next ACK should be sent. */
    int ack_enabled;      /**< Non-zero when periodic ACK sending is active. */
    double ack_freq;      /**< Interval between ACKs (seconds). */
    /* Repair scheduling */
    double next_rep;      /**< Timestamp at which the next repair packet should be sent. */
    int rep_enabled;      /**< Non-zero when periodic repair sending is active. */
    double rep_freq;      /**< Interval between repair packets (seconds). */
    /* Data flush scheduling */
    double next_datf;     /**< Timestamp at which the next data flush should occur. */
    int datf_enabled;     /**< Non-zero when periodic data flushing is active. */
    double datf_freq;     /**< Interval between data flushes (seconds). */
};
extern struct SIG_ACK_RED *sig_AckRed;

/**
 * \struct senderStats common_struct_def.h
 * \brief Accumulated statistics for the sender side of a Tetrys session.
 */
struct senderStats {
    double startTime;   /**< Session start timestamp (seconds). */
    double endTime;     /**< Session end timestamp (seconds). */
    double ackStamp;    /**< Timestamp of the most recently processed ACK. */
    int pktSize;        /**< Configured packet payload size (bytes). */
    int statSize;       /**< Number of valid samples in the statistics arrays. */
    /** Encoding window size at each repair emission: encodingWindowLength[i] is the
     *  number of data packets covered by the i-th repair packet sent. */
    int encodingWindowLength[2000];
    /** Inter-ACK data count: ackPeriod[i] is the number of data packets sent between
     *  the (i-1)-th and i-th received ACK. */
    int ackPeriod[2000];
    /** Time duration (ms) between consecutive received ACKs. */
    int ackPeriodDuration[2000];
    /** Loss rate histogram reported by the receiver, at 0.5% granularity.
     *  rcvLossRate[i] = number of times a loss rate of i*0.5% was reported.
     *  E.g. rcvLossRate[12]=23 means "6% loss rate was reported 23 times". */
    int rcvLossRate[200];
    int nbAckRcvd;              /**< Total ACKs received during the session. */
    int nbDatSent;              /**< Total data packets sent. */
    int nbRepSent;              /**< Total repair packets sent. */
    int nbSinceLastAck;         /**< Data packets sent since the last ACK was received. */
    double avgPeriodDuration;   /**< Average time between ACKs (seconds). */
    double avgPeriod;           /**< Average inter-ACK packet count. */
    double avgEncodingWindowLength; /**< Average encoding window size (packets). */
    double avgLossRate;         /**< Average loss rate reported by the receiver. */
};
/** Receiver-side statistics (histograms, averages and counters). */
struct receiverStats {
    int statSize;	/**< Number of valid histogram bins. */
    int pktSize;	/**< Payload size used for the rate computations (bytes). */
    double startTime;	/**< Session start (seconds since epoch). */
    double endTime;	/**< Session end (seconds since epoch). */
    double timeLossEvent;     /**< The time stamp of a loss event */
    double timeInterLossEvent;	   /**< The time stamp of a loss event */
    int encodingWindowLength[2000];	/**< The number of data packets encoded in a repair packet received */
    int ackPeriod[2000];     /**< The number of data packets sent between the reception of two acknoledgement */
    int interLossSize[2000];	 /**< The number of packet received between two losses, e.g interLoss[120]=45, mean that during the session, an the number of time that 120 was succeffully consecutly received was 45  */
    int lossEventSize[2000];	 /**< The number of packet received between the first losses and the time at which packet where recovered, e.g lossEventSize[120]=45, mean that during the session, the number of time that the loss event last 120 packets was 45  */
    int nbLossInEvents[2000];	/**< Histogram of the number of packets lost per loss event. */
    int lossEventDuration[2000];     /**< The time between the end of the last lost event and another losses with a granularity of 0.01 sec. e.g interLossEventDuration[120]=45, mean that during the session, the number of time that the loss event last between 1.20 sec and 1.21 sec was 45  */
    int interLossEventDuration[2000];	  /**< The time between the end of the last lost event and another losses with a granularity of 0.01 sec. e.g interLossEventDuration[120]=45, mean that during the session, the number of time that the loss event last between 1.20 sec and 1.21 sec was 45  */
    int interLossEventSize[2000];     /**< The number of packet received between the end of the last lost event and another losses, e.g interLossEventSize[120]=45, mean that during the session, the number of time that the inter loss event was 120 packets long was 45  */
    int tetrysDelay[1000];     /**< for each received packet, the delay between it's reception from udp and it's transmission to the application. Between 0 and 1000*10 ms = 10s. */
    int inversionTries[1000];	  /**< the number of time that a decoding of size X should have been inversed.*/
    int inversionSuccess[1000];	   /**< the number of time that a decoding of size X succeed.*/
    int inversionFailure[1000];	   /**< the number of time that a decoding of size X failed.*/
    int nbreceived;	/**< Total packets received. */
    int nbLosses;	/**< Total packets declared lost. */
    int nbOldLoss;	/**< Losses already counted before the current event. */
    //
    double avgEncodingWindowLength;	/**< Average encoding window length (packets). */
    double avgPeriod;	/**< Average number of data packets between two ACKs. */
    double avgPeriodDuration;	/**< Average time between two ACKs (s). */
    double avgInterLoss;	/**< Average number of packets received between two losses. */
    double avgLossEventSize;	/**< Average loss event size (packets). */
    double avgLossEventDuration;	/**< Average loss event duration (s). */
    double avgInterLossEventSize;	/**< Average number of packets between two loss events. */
    double avgInterLossEventDuration;	/**< Average time between two loss events (s). */
    double avgNbLossInEvent;	/**< Average number of losses per loss event. */
    double avgInversionTries;	/**< Average size of the attempted matrix inversions. */
    double avgInversionSuccess;	/**< Average size of the successful matrix inversions. */
    double avgInversionFailure;	/**< Average size of the failed matrix inversions. */
    double avgDelayLoss;	/**< Average delivery delay of the recovered packets. */
    int nbDatSinceLastLoss;	/**< Data packets received since the last loss. */
    int prev_nb_pkt_lost;	/**< Number of pending losses at the previous packet. */
    int startLossEvent;	/**< 1 while a loss event is in progress, 0 otherwise. */
    int nbDatSinceLastEvent;	/**< Data packets received since the last loss event ended. */
    int nbLossInEvent;	/**< Losses counted in the current loss event. */
    int nbRepRcvd;	/**< Repair packets received. */
    int nbDatRcvd;	/**< Data packets received. */
    int nbSinceLastAck;	/**< Packets received since the last ACK was sent. */
    double avgLossRate;	/**< Average loss rate observed. */
    int nbAckSent;	/**< ACK packets sent. */
    int gilbert_goodgood;	/**< Gilbert model: good -> good transitions. */
    int gilbert_badbad;	/**< Gilbert model: bad -> bad transitions. */
};

#endif				/* TETRYS_STRUCT */
#endif				/* COMMON_STRUCT_DEF_H */
