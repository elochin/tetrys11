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

/* 
 *	This file must:
 *		1. open initialize Tetrys,
 *		2. fork to new process or thread
 *		3. Generate traffic on the fly or/file reading -- send it to pipe if sem is ok
 *		4. Reconstruct traffic
 */

static gf tetrys_generator_buff[2000];
static int tetrys_generator_from_file = 0;
static FILE *tetrys_generator_file;
static char *tetrys_generator_filename = NULL;
static long int tetrys_generator_fsize = 2147483640;
static int tetrys_generator_bread;
static int tetrys_generator_pls = 1000;
static int tetrys_generator_next_pls;
static unsigned long int tetrys_generator_nb_gen = 0;
static unsigned long int tetrys_generator_nb_bytes_gen = 0;
static unsigned long int tetrys_generator_nb_rcv = 0;
static unsigned long int tetrys_generator_nb_bytes_rcv = 0;
static int tetrys_generator_wb = 0;
static double tetrys_generator_start_time;
static double tetrys_generator_end_time;
static int tetrys_generator_continue_rcv = 1;
static unsigned long int tetrys_generator_nb_to_send = 2147483640;
static unsigned long tetrys_generator_inter_packet = 0;
static int tetrys_generator_role;
static sigset_t tetrys_generator_block_alarm;


/**
 * Signal handler for SIGINT/SIGTERM/SIGQUIT: sets the shutdown flag cleanly
 * (async-signal-safe).
 * @param sig  Signal number received.
 */
static void tetrys_generator_handle_stop(int sig)
{
    tetrys_stop_requested = (sig_atomic_t) sig;
}

/**
 * Print the transfer statistics on termination.
 */
static void tetrys_generator_end(int sig)
{
    (void) sig;
    tetrys_generator_end_time = get_time();
    double tetrys_generator_transfert_duration =
	(tetrys_generator_start_time > 0.0)
	? (tetrys_generator_end_time - tetrys_generator_start_time) : 0.0;
    if (tetrys_generator_transfert_duration <= 0.0)
	tetrys_generator_transfert_duration = 1e-6;
    double dtetrysGenerator_nbBytesGen = (double) tetrys_generator_nb_bytes_gen;
    printf
	("\n\ntetrys_trafficGenerator::main Generated %lu Bytes in %lu packets in %f second = %f B/s ",
	 tetrys_generator_nb_bytes_gen, tetrys_generator_nb_gen,
	 tetrys_generator_transfert_duration,
	 (dtetrysGenerator_nbBytesGen /
	  tetrys_generator_transfert_duration));
    double dtetrysGenerator_nbBytesRcv = (double) tetrys_generator_nb_bytes_rcv;
    printf
	("\ntetrys_trafficGenerator::main Received %lu Bytes in %lu packets in %f second = %f B/s ",
	 tetrys_generator_nb_bytes_rcv, tetrys_generator_nb_rcv,
	 tetrys_generator_transfert_duration,
	 (dtetrysGenerator_nbBytesRcv /
	  tetrys_generator_transfert_duration));
#ifdef VERBOSE
    if (tetrys != NULL) {
	fprintf(stdout,
		"\n\tTetrys RES \n\tTot fwd in buffers:\n\t RRB %lu RDB %lu SEB %lu TIB %lu DIB %lu OB %lu \n\t Tot stored in buffers:\n\t RRB %d RDB %d SEB %d TIB %d DIB %d OB %d nbloss %d nb red %d rebuilt %d",
		tetrys->RRBuffer->nb_tot_added, tetrys->RDBuffer->nb_tot_added,
		tetrys->SEBuffer->nb_tot_added, tetrys->TIBuffer->nb_tot_added,
		tetrys->DIBuffer->nb_tot_added, tetrys->OBuffer->nb_tot_added,
		tetrys->RRBuffer->nbItemStored, tetrys->RDBuffer->nbItemStored,
		tetrys->SEBuffer->nbItemStored, tetrys->TIBuffer->nbItemStored,
		tetrys->DIBuffer->nbItemStored, tetrys->OBuffer->nbItemStored,
		tetrys->nb_pkt_lost, tetrys->RRBuffer->nbItemStored,
		tetrys->nb_rebuilt);
    }
#endif
    fflush(stdout);
}

/**
 * Install the signal handlers of the generator.
 */
static void tetrys_generator_manage_signal(void)
{
    struct sigaction sa;

    signal(SIGHUP, SIG_IGN);
    signal(SIGFPE, float_error);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = tetrys_generator_handle_stop;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);

    sigemptyset(&tetrys_generator_block_alarm);
    sigaddset(&tetrys_generator_block_alarm, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &tetrys_generator_block_alarm, NULL);
}

/*
 * A micro-second delay function.
 */

/**
 * Sleep during the inter-packet delay.
 * @param usec  Delay in microseconds.
 */
static void tetrys_generator_delay_loop(unsigned long usec)
{
    struct timespec ts;

    ts.tv_sec = (time_t) (usec / 1000000UL);
    ts.tv_nsec = (long) (usec % 1000000UL) * 1000L;

    if (nanosleep(&ts, NULL) < 0)
	perror("tetrys_generator_delay_loop: nanosleep");

}

/**
 * Fill the generator buffer with the next packet (file content or pattern).
 */
static void tetrys_generator_read_next_packet(void)
{
    tetrys_generator_next_pls = -1;
    if (tetrys_generator_from_file == 1) {
	if ((unsigned long) tetrys_generator_fsize <= tetrys_generator_nb_bytes_gen) {
	    tetrys_generator_fsize = 0;
	    return;
	}			// Terminate condition for file reading
	memset(tetrys_generator_buff, 0, sizeof(tetrys_generator_buff));
	tetrys_generator_bread = 0;

	/* Read up to tetrys_generator_pls bytes in one block (bounded by remaining file size). */
	long remaining = tetrys_generator_fsize - ftell(tetrys_generator_file);
	if (remaining > 0) {
	    size_t to_read = (remaining < (long) tetrys_generator_pls)
		? (size_t) remaining : (size_t) tetrys_generator_pls;
	    size_t got = fread(tetrys_generator_buff, 1, to_read,
			       tetrys_generator_file);
	    tetrys_generator_bread = (int) got;
	}
	tetrys_generator_next_pls = tetrys_generator_bread;
    } else {
	memset(tetrys_generator_buff, 0, sizeof(tetrys_generator_buff));
	for (tetrys_generator_bread = 0; (tetrys_generator_bread < 2000);
	     tetrys_generator_bread++) {
	    tetrys_generator_buff[tetrys_generator_bread] =
		(gf) (tetrys_generator_nb_gen % 256);
	}
	tetrys_generator_next_pls = tetrys_generator_pls;
    }
    if (tetrys_generator_next_pls > 0)
	tetrys_generator_nb_bytes_gen += (unsigned long) tetrys_generator_next_pls;
    return;
}

/**
 * Write the packet received from Tetrys to the output file, if any.
 */
static void tetrys_generator_write_next_packet(void)
{
    if (tetrys_generator_from_file == 1) {
	int tetrys_generator_to_write = 0;
	memcpy(&tetrys_generator_to_write, tetrys_generator_buff,
	       sizeof(int));
	if (tetrys_generator_to_write <= 0 ||
	    tetrys_generator_to_write > TETRYS_MAX_PAYLOAD) {
	    return;
	}
	tetrys_generator_wb =
	    (int) fwrite(&tetrys_generator_buff[sizeof(int)],
			 (size_t) tetrys_generator_to_write, 1,
			 tetrys_generator_file);
	tetrys_generator_nb_bytes_rcv += (unsigned long) tetrys_generator_to_write;
    }

}

/**
 * Open the input file (sender) or the output file (receiver).
 */
static void tetrys_generator_init_file(void)
{
    if (tetrys_generator_from_file == 1) {
	if (tetrys_generator_role == 1) {
	    tetrys_generator_file = fopen(tetrys_generator_filename, "rb");
	    if (tetrys_generator_file == NULL) {
		perror(tetrys_generator_filename);
		exit(EXIT_FAILURE);
	    }
	    fseek(tetrys_generator_file, 0, SEEK_END);
	    tetrys_generator_fsize = ftell(tetrys_generator_file);
	    fseek(tetrys_generator_file, 0, SEEK_SET);
	    fflush(tetrys_generator_file);
	}
	if (tetrys_generator_role == 2) {
	    tetrys_generator_file = fopen(tetrys_generator_filename, "wb");
	    if (tetrys_generator_file == NULL) {
		perror(tetrys_generator_filename);
		exit(EXIT_FAILURE);
	    }
	}
    }
    return;
}

/**
 * Receiver loop: read the packets delivered by Tetrys until the end of the transfer.
 */
static void tetrys_generator_rcv_loop(void)
{
    fd_set tetrys_generator_rset;
    tetrys_generator_init_file();
    int tetrys_generator_pdu_pls = 0;
    int tetrys_generator_terminate_sig = 0;
    while (tetrys_generator_continue_rcv == 1 && !tetrys_stop_requested) {
	FD_ZERO(&tetrys_generator_rset);
	FD_SET(tetudp->tetrysToIperf[0], &tetrys_generator_rset);
	int tetrys_generator_sel =
	    select(tetudp->tetrysToIperf[0] + 1, &tetrys_generator_rset,
		   NULL, NULL, NULL);
	if (tetrys_generator_sel == -1) {
	    if (errno == EINTR)
		continue;
	    perror(" -- select");
	    break;
	}
	size_t total = 0;
	while (total < PLS) {
	    ssize_t r = read(tetudp->tetrysToIperf[0],
			     &tetrys_generator_buff[total], PLS - total);
	    if (r < 0) {
		if (errno == EINTR)
		    continue;
		break;
	    }
	    if (r == 0)
		break;
	    total += (size_t) r;
	}
	if (total < PLS) {
	    tetrys_generator_continue_rcv = 0;
	    break;
	}
	tetrys_generator_next_pls = (int) total;

	if (tetrys_generator_start_time == 0.0)
	    tetrys_generator_start_time = get_time();
	memcpy(&tetrys_generator_pdu_pls, tetrys_generator_buff,
	       sizeof(int));
	memcpy(&tetrys_generator_terminate_sig,
	       &tetrys_generator_buff[sizeof(int)], sizeof(int));
	if ((tetrys_generator_pdu_pls == sizeof(int))
	    && (tetrys_generator_terminate_sig == 6666)) {
	    tetrys_generator_continue_rcv = 0;
	} else
	    tetrys_generator_write_next_packet();
	tetrys_generator_nb_rcv++;
    }

}

/**
 * Sender loop: generate the packets and hand them to Tetrys.
 */
static void tetrys_generator_snd_loop(void)
{
    tetrys_generator_start_time = get_time();
    tetrys_generator_init_file();	//set the buffer
    tetrys_generator_read_next_packet();
    printf("BEGIN\n");
    while (!tetrys_stop_requested
	   && (tetrys_generator_nb_gen <= tetrys_generator_nb_to_send)
	   && (tetrys_generator_nb_bytes_gen <= (unsigned long) tetrys_generator_fsize)
	   && (tetrys_generator_next_pls > 0)) {

	do {
	    if (0 == sem_wait(&tetudp->accept_packet)) {
		tetrys_generator_wb =
		    (int) write(tetudp->iperfToTetrys[1], tetrys_generator_buff,
				(size_t) tetrys_generator_next_pls);
	    }
	} while (!tetrys_stop_requested
		 && (tetrys_generator_wb != tetrys_generator_next_pls)
		 && (errno == EINTR));
	tetrys_generator_nb_gen++;
	printf(".");
	fflush(stdout);
	//Wait
	if (tetrys_generator_inter_packet > 0)
	    tetrys_generator_delay_loop(tetrys_generator_inter_packet);
	tetrys_generator_read_next_packet();
    }
    if (tetrys_stop_requested)
	return;
    tetrys_generator_delay_loop(50000);	// Wait to be shure that both packet wont be concatenated in pipe
    tet_end_transfer();		// Return when all the data sent are recvd and acked

    tetrys_generator_delay_loop(50000);	// Wait to be shure that both packet wont be concatenated in pipe
    //Send an ultimate pkt without 1 as pls (will turn rcv to terminate)
    int tetrys_generator_terminate_flag = 6666;
    for (int term_i = 0; term_i < 5 && !tetrys_stop_requested; term_i++) {
	if (sem_wait(&tetudp->accept_packet) == 0) {
	    if (write(tetudp->iperfToTetrys[1], &tetrys_generator_terminate_flag,
		      sizeof(int)) < 0) {
		break;
	    }
	}
	tetrys_generator_delay_loop(50000);
    }
}

/**
 * Print the command-line help.
 * @param progname  Program name shown in the usage message.
 */
static void tetrys_generator_usage(const char *progname)
{
    printf("USAGE: \nsender example: \n\t%s -f ./test1Mo.tr -i 100000 -s 1000 -m 1 192.168.0.19 12345\n"
	   "receiver example: \n\t%s -f rebuilt1Mo.tr -s 1000 -m 2 192.168.0.19 12345\n",
	   progname, progname);
    exit(EXIT_FAILURE);
}



/**
 * Entry point of the UDP traffic generator (sender or receiver).
 */
int main(int argc, char *argv[])
{

    extern char *optarg;
    extern int optind;
    char *tetrys_generator_server = NULL;
    int tetrys_generator_port;
    int c;


    /* 
     * sender: tetrys_generate_udp -f ./test1Mo.tr -i 100000 -s 1000 -m 1 192.168.0.19 12345
     * receiver: tetrys_generate_udp -f rebuilt1Mo.tr -s 1000 -m 2 192.168.0.19 12345
     */
    tetrys_generator_role = 0;	// 1 sender ; 2 receiver
    int tetrys_generator_n = 0;	// 1 sender ; 2 receiver
    int tetrys_generator_k = 0;	// 1 sender ; 2 receiver
    while ((c = getopt(argc, argv, "f:i:s:m:n:k:q:")) != -1) {
	switch (c) {
	case 'f':
	    tetrys_generator_from_file = 1;
	    tetrys_generator_filename = optarg;
	    break;
	case 'i':
	    tetrys_generator_inter_packet = (unsigned long) parse_long_arg(optarg, 0, 1000000000L, "-i");	//us 0=asap
	    break;
	case 's':
	    tetrys_generator_pls = (int) parse_long_arg(optarg, 1, TETRYS_MAX_PAYLOAD, "-s");	// Bytes
	    break;
	case 'm':
	    tetrys_generator_role = (int) parse_long_arg(optarg, 1, 2, "-m");	// 1 sender ; 2 receiver
	    break;
	case 'n':
	    tetrys_generator_n = (int) parse_long_arg(optarg, 1, 255, "-n") - 1;	// 1 sender ; 2 receiver
	    break;
	case 'k':
	    tetrys_generator_k = (int) parse_long_arg(optarg, 1, 255, "-k") - 1;	// 1 sender ; 2 receiver
	    break;
	case 'q':
	    tetrys_generator_nb_to_send = (unsigned long) parse_long_arg(optarg, 1, 2147483640L, "-q");	// 1 sender ; 2 receiver
	    break;
	default:
	    tetrys_generator_usage(argv[0]);
	    break;
	}
    }
    if (argc < 3)
	tetrys_generator_usage(argv[0]);
    tetrys_generator_server = argv[argc - 2];
    tetrys_generator_port = (int) parse_long_arg(argv[argc - 1], 1, 65535, "port");

    pthread_t tetrys_thread;
    tetrys_init_all(tetrys_generator_pls, tetrys_generator_k,
		    (tetrys_generator_n - tetrys_generator_k),
		    tetrys_generator_role, 0,
		    tetrys_generator_server /*mSettings->mHost */ ,
		    tetrys_generator_port);
#ifdef VERBOSE
    fprintf(stderr,
	    "\ntetrys_trafficGenerator::main: Tetrys Init all is done");
    fflush(stderr);
#endif
    // 3 for fec_k; 1 for fec_n-fec_k ; 1 for sender mode; 0 to act as Tetrys (Vs Fec); 
    // NULL for the udp dest server
    tetrys_generator_manage_signal();
    if (pthread_create(&tetrys_thread, NULL, &tetrys_main_run, NULL) != 0) {
	fprintf(stderr,
		"tetrys_trafficGenerator::main: pthread_create returned an error EXIT_FAILURE");
	exit(EXIT_FAILURE);
    }
    if (tetrys_generator_role == 1)
	tetrys_generator_snd_loop();
    if (tetrys_generator_role == 2)
	tetrys_generator_rcv_loop();
    tetrys_stop_requested = 1;
    if (tetudp != NULL && tetudp->iperfToTetrys[1] >= 0) {
	close(tetudp->iperfToTetrys[1]);
	tetudp->iperfToTetrys[1] = -1;
    }
    pthread_kill(tetrys_thread, SIGALRM);
    pthread_join(tetrys_thread, NULL);
    tetrys_generator_end(0);
    if (tetrys_generator_file != NULL) {
	fclose(tetrys_generator_file);
	tetrys_generator_file = NULL;
    }
    cleanup_module_tetudp();
    tetrys_cleanup();
    return 0;
}
