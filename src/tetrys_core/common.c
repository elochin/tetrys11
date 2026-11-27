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

/* Global variable definitions — declared extern in the headers. */
struct TETRYS       *tetrys;
struct SIG_ACK_RED  *sig_AckRed;
struct senderStats  *sndStats;
struct receiverStats *rcvStats;
volatile sig_atomic_t tetrys_alarm_pending = 0;
volatile sig_atomic_t tetrys_stop_requested = 0;

/*
 * Mem alloc & Queue managment functions
 */

void *my_malloc(size_t sz, const char *message)
{
    void *p = malloc(sz);
    if (p != NULL)
	return p;
    fprintf(stderr, "my_malloc: failure for %zu bytes in <%s>\n", sz, message);
    exit(1);
}

long parse_long_arg(const char *str, long min, long max, const char *name)
{
    char *end = NULL;
    errno = 0;
    long value = strtol(str, &end, 10);
    if (errno != 0 || end == str || *end != '\0' || value < min || value > max) {
	fprintf(stderr, "Invalid value '%s' for %s (expected %ld..%ld)\n",
		str, name, min, max);
	exit(EXIT_FAILURE);
    }
    return value;
}

/* 
 * get_time function 
 */

double get_time(void)
{
    struct timeval time_tv;
    double time;
    gettimeofday(&time_tv, NULL);
    time = (double) time_tv.tv_sec + (double) (time_tv.tv_usec) * 0.000001;
    return (time);
}

/* 
 * open a server socket in reception and Ack emission 
 */

int server_socket_rcv(uint16_t port)
{
    int server_socket;
    struct sockaddr_in server_sockaddr;
    server_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_socket == -1) {
	perror("socket");
	exit(EXIT_FAILURE);
    }
    memset(&server_sockaddr, 0, sizeof(server_sockaddr));
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(port);

    if (bind(server_socket, (struct sockaddr *) &server_sockaddr,
	     sizeof(server_sockaddr)) == -1) {
	perror("bind");
	exit(EXIT_FAILURE);
    }
    return (server_socket);
}

/* 
 * open a client socket to send and receive ACK 
 */

int server_socket_snd(uint16_t port)
{
    int client_socket;
    struct sockaddr_in server_sockaddr;

    client_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (client_socket == -1) {
	perror("socket");
	exit(EXIT_FAILURE);
    }
    memset(&server_sockaddr, 0, sizeof(server_sockaddr));
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons((uint16_t)(port + 1));

    /* Socket Bind */
    if (bind(client_socket, (struct sockaddr *) &server_sockaddr,
	     sizeof(server_sockaddr)) == -1) {
	perror("bind");
	exit(EXIT_FAILURE);
    }
    return (client_socket);
}

void tetrys_handle_alarm(int sig)
{
    (void) sig;
    tetrys_alarm_pending = 1;
}

_Noreturn void float_error(int sig)
{
    (void) sig;
    static const char msg[] = "\nFloat error\n";
    ssize_t unused = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void) unused;
    _exit(EXIT_FAILURE);
}

/* 
 * Debug function and fun stuff to inspect the size of data of the inverted matrix 
 */

int pr_matrix(const void *m1, int rows, int cols, const char *s)
{
    int r, c;
#if GF_BITS <= 8
    const uint8_t *m = m1;
#else
    const uint16_t *m = m1;
#endif
    fprintf(stdout, "%s\n", s);
    for (r = 0; r < rows; r++) {
	for (c = 0; c < cols; c++)
	    fprintf(stdout, "%3d  ", m[r * cols + c]);
	fprintf(stdout, "\n");
    }
    fprintf(stdout, "\n");
    return 0;
}
