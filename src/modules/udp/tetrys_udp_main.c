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

#include <stdio.h>
#include "tetrysModule_udp.h"
#include <semaphore.h>

/* \file divert_send.c
\Fichier principale emmeteur/recepteur
  *  \brief Fonction de base, reception et envoi des divers type de packet
  *  \author Pierre Ugo Tournoux
  *  \author Emmanuel Lochin
  *  \author Jerome Lacan
  *  \version 1.0
  *  \date    2009
  *  \bug A voir 
  *  \warning A voir
  *
  *  \section Fonctionalite
  *  Tetrys doit pouvoir effectuer une fiabilite bidirectionelle ou dans un sens.
  *  SenderApp est l'application emetrice a fiabiliser. ReceveirApp est l'aplpciation receptrice du flux fiabilise. TetrysSender (resp. TetrysSender) est le tetrys qui encode (resp. decode) dans le cas du directionel.  TD est un packet tetrys contenenat des data en clair. TR est un packet Tetrys contenant une combianaison lineaire. TA est un Ack Tetrys. TAP est une info de ReceiverApp a SenderApp dans lequel un Ack Tetrys a ete Piggybacke. DS est un packet de donnee de SenderApp a TetrysSender. DR est un packet de donnee de TetrysReceiver a ReceiverApp. Inversement pour AR et AS.   Il doit y avoir une option permettant de selectionner les roles. Les fonctionalite de base vers lequel ce fichier doit rediriger sont:
  *	- Emmeteur uniquement (TetrysSender): Dans ce cas Tetrys doit:
  *		-# recevoir les paquet envoyé par SenderApp quel quelle soit.
  *		-# les envoyer a TetrysReceiver apres ajout a la fenetre d'encodage et de l'entete Tetrys
  *		-# envoyer les paquets de redondance a TetrysReceiver
  *		-# recevoir les Ack de TetrysReceiver
  *		-# faire suivre les Ack reçu de TetrysReicever  a SenderApp dans le cas ou ils seraient piggybacke
  *	- Recepteur uniquement (TetrysReceiver):
  *		-# Recevoir les paquet de data venant de
  *		-# Recevoir les paquet de redundance
  *		-# Envoyer les paquet de data, recu ou reconstruit a l'application receptrice
  *		-# Recevoir les paquet de donné/Ack de l'appli receptriceEnvoyer et ajouter l'ack Tetrys.
  *		-# Envoyer des Ack de maniere spontanee en fonction du RTT
  * 	- Emmeteur et recepteur. Dans ce cas on note A1 et A2 les deux application. T1 et T2 les deux Tetrys qui sont ala fois emmeteur et recepteur.
  *		-# ..
  *  \section Ordonancement
  *  Pour TetrysSender/Receiver, traiter en priorite les donne issue/vers des applications par rapport aux donnee reçu par le reseau peu mener a un depassement de buffer UDP/RAW, avec le paquet acummule. Il vaut mieux vider en priorite les buffer reseau, puis emmettre et effectuer les operation lourdes qd on peut.
  *  On doit etre capacble d'envoyer des Ack ou des repair spontanement, donc via alarm. Il faut recevoir des signaux mais uniquement lorsque l'application est en attente bloquante de donnee du resau. Ces signaux sont suspendus lorsque Tetrys effectue des operation internes ou une lecture non blquante du reseau.
  *  Pour permettre une ce traitement decale des donnee recu via le reseau, il faut les stocker dans les buffer. Seul la lecture des Ack est effectue de suite de par leur rapidite de traitement. Chaque type de donnee DS,DR,TD,TR,TAP est stocke dans un buffer specifique en attendant d'etre traite.
  * 
  *  \section Buffers
  *	- DIBuffer: Buffer pour les DS: Ces paquet sont transmis par SenderApp et recu soit par un pipe, soit via Tetrys en lecture de fichier, soit par le reseau comme dans le cas des divert socket. Quel que soit leur moyen de reception les donnée sont decoupees en segment et stockedans le DIBuffer.
  *  	- TOBuffer: Une fois traite et ajoute a la fenetre d'encodage, les DS devienne des TD et sont mis en attente dans TOBuffer et seront emis sur le reseau. Le traitement d'un DS peut engendrer la generation d'un TR, lequel est lui aussi mis en queue dans TOBuffer. Les TR generes via les signaux sont aussi mis en queue dans TOBuffer. La generation d'Ack via signaux doit est lui aussi mis en queue dans TOBuffer..
  *	- TIBuffer: A la reception d'un TD/TR, ce dernier mis en queue dans TIBuffer est sera traite ulterieurement
  *	- DOBuffer: TA et TAP ci est traite directement. Si TAP, l'information est mis en queue dans DOBuffer. Les DR, decode via TR ou recu par TD, sont eux aussi mis en queue dans DOBuffer en attendant d'etre transmis a ReceiverApp.
  *	- SEBuffer: Sender Encoding Buffer buffer qui contient l'ensemble de la fenetre d'encodage. Un TR transmit contient une combinaison lineaire de chacun des pacuet contenu dans DEBuffer. DEBuffer est vide via les TA ou TAP.
  *	- RDBuffer: Receiver Decoding/Data Buffer. Le buffer des TD reçu et gardee pour une future substraction aux TR
  *	- RRBuffer: Receiver RepairBuffer. Il contient les repair buffer auxquels ont ete soustrait le contenu de RDBuffer.
  *
  *
  *  \section Fonction
  *	Following, the figure X, we describe here the operation related to each steps (1,2,2.1,2.2,3..).
  *	- 1. Wait for Data from the Network: The received message can be TA,TAP,TR,TP. In case of UDP bridge or DIvert socket, it can also be AR and DS. The message are waited thanks to a select or Pselect function. This function is then context dependant. Need one for Diver, one for UDP and one for AppLayerTetrys. Depending on message types:, they are sent to step 2 (if 
  *		-# TA or TAP: goto step 2.
  *		-# TD or TR:  goto step 3.
  *		-# AR or DS:  goto step 8.
  *	- 2. As it is not time consuming, treat the acknoledgement e.g. 
  *		-# 2.1 remove from SEBuffer the packet which have a seqNum higher than (TA/TAP).ack_min_redundancy. Dans le cas du multicast,
  *		-# 2.2 extract the piggybacked Ack AS from TAP and queue it in DOBuffer
  *	- 3. Enqueue TR and TD in TIBuffer for future processing when 1 wont be busy. Use circ_add_item_in_buffer to  add it in buffer.
  *	- 4. When 1 and 8 are not busy; Pop the oldest item (TR/TD) in TIBuffer. Goto to 5 if TD, to 6 if TR
  *	- 5. If TD, Add it to RDBuffer.	5.1, 5.2 and goto 7.
  *		-# 5.1. add the data to RDBuffer via circ_add_pdu_in_buffer(stamp,tetrys->dataQueue); goto 5.2
  *		-# 5.2. launch delivery (if in-order will deliver only if no losses). If Out of Order, add to OBuffer.
  *	- 6. If TR and losses detected, add it in the TR buffer using circ_add_pdu_in_buffer(stamp,tetrys->repairQueue). goto 7.
  *	- 7. If nbLost <= nbR, try to decode. If decoded, goto 5.1-5.2
  *	- 8. Wait for (AR,DS) info. Can be through pipe, UDP/TCP socket or div socket. This function is context dependent. but must goto 8.1. If Tetrys is BIDIR, then AR=DS and we add both the TD header and TAP header.
  *		-# 8.1. add data to DIBuffer for futures process
  * 	- 9. When 8 and 1 are not busy: Pop packet from buffer and switch to 10 if AR, 11 if DS
  *	- 10. Add the Tetrys Ack Header to AR and enqueue TAP in OBuffer
  *	- 11. Treat D:
  *		-# 11.1 Add the Tetrys header to D and enqueue TD in OBuffer; refresh the freqAck timer;
  *		-# 11.2 Add D to the encoding window
  *	- 12. Catch the freqAck timer when expire. Build and Ack header and enqueue TA to OBuffer.
  *	- 13. Build a repair packet with func setRepair; then enqueue to OBuffer; refresh redundancyFreq timer/
  *	- 14. Catch the redundancyFreq timer when expire. Goto 13
  *	- 15. Pop item in buffer.
  *		-# if DR/AS transmit to app with goto 16.
  *		-# if TR,TD,TA,TAP send througt the network with goto 17.
  *	- 16. This function is context dependant, UDP, pipe or div
  *	- 17. This function is context dependant UDP or DIV.
  *
  *  \section Entete Tetrys, TA/TAP/TR/TD
  *	See
  *	\struct PDU common_struct_def.h common_struct_def.h
  *
  *   @arg \f$x_1\f$
  *   @arg \f$x_2\f$
  *   @arg \f$x_3\f$
  */


/* With one sig should handle both Ack timer and red timer */

void manage_signal(void)
{
    struct sigaction sa;

    signal(SIGHUP, SIG_IGN);
    signal(SIGFPE, float_error);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = tetrys_handle_alarm;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, NULL);

    sigset_t thread_mask;
    sigemptyset(&thread_mask);
    sigaddset(&thread_mask, SIGALRM);
    sigaddset(&thread_mask, SIGINT);
    sigaddset(&thread_mask, SIGTERM);
    sigaddset(&thread_mask, SIGQUIT);
    pthread_sigmask(SIG_BLOCK, &thread_mask, NULL);

    if (tetrys != NULL) {
	sigemptyset(&tetrys->block_alarm);
	sigaddset(&tetrys->block_alarm, SIGALRM);
    }
}


_Noreturn void quit(int sig)
{
    (void) sig;
    fprintf(stderr, "\nTetrys fatal error, shutting down.\n");
    cleanup_module_tetudp();
    tetrys_cleanup();
    exit(EXIT_FAILURE);
}


/**
 * Abort when the role or the FEC parameters are inconsistent.
 */
static void check_tetrys_config(void)
{
    if ((tetrys->currentRole == 1)
	&& (tetrys->fec_n == 0 || tetrys->fec_k == 0)) {
	fprintf(stderr, "check_tetrys_config: fec_n or fec_k == 0\n");
	exit(EXIT_FAILURE);
    }
    if (tetrys->currentRole > 2) {
	fprintf(stderr,
		"check_tetrys_config:  1 =< tetrys->currentRole < 2 !\n");
	exit(EXIT_FAILURE);
    }
    if (tetrys->currentRole < 1) {
	fprintf(stderr,
		"check_tetrys_config:  1 =< tetrys->currentRole < 2 !\n");
	exit(EXIT_FAILURE);
    }
    return;
}


int tetrys_init_all(int iperf_packet_len, int _fec_n, int _fec_k,
		    int _role, int _fecMode, char *_server, int _port)
{
    tetrys_alloc_globals(_role, iperf_packet_len);

    tetrys->fecMode = 0;
    tetrys->currentRole = 0;
    sig_AckRed->rep_freq = 0.1;
    sig_AckRed->ack_freq = 0.05;
    sig_AckRed->next_rep = 0;
    sig_AckRed->next_ack = 0;
    sig_AckRed->rep_enabled = 0;
    sig_AckRed->ack_enabled = 0;
    sig_AckRed->datf_enabled = 0;

    tetrys->fec_n = _fec_n;
    tetrys->fec_k = _fec_k;
    tetrys->currentRole = _role;
    if (tetrys->currentRole == 2)
	sig_AckRed->ack_enabled = 1;
    else
	sig_AckRed->ack_enabled = 0;
    sig_AckRed->rep_enabled = 0;

    tetrys->fecMode = _fecMode;
    tetrys->nodeid = (uint16_t) tetrys->currentRole;
    tetrys->deliverInOrder = 1;
    check_tetrys_config();
    init_module_tetudp(_server, _port, iperf_packet_len);
    init_context();
#ifdef VERBOSE
    fprintf(stderr, "\n\ttetrys_main: INIT CONTEXT -- [ Done ]");
#endif

    return 0;
}

/** One-time start-up of the main loop: signals, role-specific context and first alarms. */
static void tetrys_run_init(void)
{
#ifdef STATS
    if (tetrys->currentRole != 2)
	stat_init_sender_stats(sndStats, tetrys->pktSize);
#endif
#ifdef VERBOSE
    fprintf(stderr, "\n\ttetrys_main: INIT -- [ OK ]");
#endif
	/**< Init the divert socket related stuff */
    manage_signal();
#ifdef VERBOSE
    fprintf(stderr, "\n\ttetrys_main: MANAGESIGNAL -- [ OK ]");
#endif
	/* Start the main loop -- send data/redundancy packet and process Ack */
	/* As long as the buffer of datas to be sent is non empty, try to send a packet */
    srand(4);
    if (tetrys->currentRole == 2) {
	init_context();
    }
#ifdef STATS
    if (tetrys->currentRole != 2)
	sndStats->startTime = get_time();
    if (tetrys->currentRole != 1)
	rcvStats->startTime = get_time();
#endif
    if ((tetrys->currentRole == 2) && (sig_AckRed->ack_enabled == 1))
	actuate_alarm(1);							       /* action=1 if ack were sent, 2 if rep were sent, 3 if both */
    if ((tetrys->currentRole == 1) && (sig_AckRed->rep_enabled == 1))
	actuate_alarm(2);
}

/** Print the periodic progress / statistics traces of the main loop. */
static void tetrys_report_progress(void)
{
    if (PR_DB_1) {
	int sem_val = 0;
	sem_getvalue(&tetudp->accept_packet, &sem_val);
	printf("\nTetrys: SEM %d \n", sem_val);
	fflush(stdout);
    }
#ifdef VERBOSE
    if (((tetrys->currentRole == 1)
	 && (tetrys->SEBuffer->nb_tot_added > 1000
	     && (tetrys->SEBuffer->nb_tot_added % 2000) == 0))
	|| ((tetrys->currentRole == 2)
	    && (tetrys->RDBuffer->nb_tot_added > 1000
		&& (tetrys->RDBuffer->nb_tot_added % 2000) == 0))) {
	fprintf(stdout,
		"\n\tWHILE  %d -- [ OK ] \n\tTot fwd in buffers:\n\t RRB %lu RDB %lu SEB %lu TIB %lu DIB %lu OB %lu \n\t Tot stored in buffers:\n\t RRB %d RDB %d SEB %d TIB %d DIB %d OB %d nbloss %d nb red %d rebuilt %d ",
		tetrys->currentRole, tetrys->RRBuffer->nb_tot_added,
		tetrys->RDBuffer->nb_tot_added,
		tetrys->SEBuffer->nb_tot_added,
		tetrys->TIBuffer->nb_tot_added,
		tetrys->DIBuffer->nb_tot_added,
		tetrys->OBuffer->nb_tot_added,
		tetrys->RRBuffer->nbItemStored,
		tetrys->RDBuffer->nbItemStored,
		tetrys->SEBuffer->nbItemStored,
		tetrys->TIBuffer->nbItemStored,
		tetrys->DIBuffer->nbItemStored,
		tetrys->OBuffer->nbItemStored, tetrys->nb_pkt_lost,
		tetrys->RRBuffer->nbItemStored, tetrys->nb_rebuilt);
	fflush(stdout);
    }
#endif
#ifdef STATS
    if ((tetrys->currentRole == 1)
	&& ((tetrys->SEBuffer->nb_tot_added % 2000) == 0))
	printf("\n\t Nb ACK rcvd %d ", sndStats->nbAckRcvd);
    if ((tetrys->currentRole == 2)
	&& ((tetrys->RDBuffer->nb_tot_added % 2000) == 0))
	printf("\n\t Nb ACK sent %d ", rcvStats->nbAckSent);
#endif
}

void *tetrys_main_run(void *arg)
{
    (void) arg;
    tetrys_run_init();
    while (!tetrys_stop_requested) {
	tetrys_report_progress();
	if (!tetrys_step_once())
	    break;
    }
    tetrys_disarm_alarm();
    return NULL;
}
