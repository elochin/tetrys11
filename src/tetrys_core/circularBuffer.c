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


#include "circularBuffer.h"
#include "common.h"

/*
 * Sorted packet queue backed by contiguous pre-allocated PDU and item pools.
 * Steady-state insertions and deletions reuse pooled slots with zero malloc/free.
 */

static int circ_find_free_slot(const struct circBUFFER *metaBuffer);
static void circ_link_item(struct circBUFFER *metaBuffer,
			   struct circBUFFERItem *addedItem,
			   struct circBUFFERItem *before_item);

/**
 * Allocate a pooled item for sequence number @p seqnum and link it sorted into
 * @p metaBuffer. When @p zero_pdu is non-zero the underlying struct PDU is zeroed;
 * callers that immediately memcpy a full PDU pass 0 to avoid a redundant memset.
 */
static struct circBUFFERItem *circ_alloc_item(unsigned long seqnum,
					      struct circBUFFER *metaBuffer,
					      int zero_pdu)
{
    if (PR_DB_2)
	fprintf(stdout,
		"\ncirc_add_item_in_buffer: item %lu in buff: %c%c%c ",
		seqnum, metaBuffer->buffName[0], metaBuffer->buffName[1],
		metaBuffer->buffName[2]);
    if (metaBuffer->nbItemStorable <= metaBuffer->nbItemStored) {
	fprintf(stderr,
		"circularBuffer.c: Node %u circ_alloc_item %c%c%c -- ERROR: Buffer full %d / %d\n",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2],
		metaBuffer->nbItemStored, metaBuffer->nbItemStorable);
	circ_print_buffer_stat(metaBuffer);
	quit(0);
	return NULL;
    }

    /* Combined duplicate check and insertion-point search (O(1) tail fast-path). */
    struct circBUFFERItem *insert_before = NULL;
    if (metaBuffer->lastItem == NULL
	|| seqnum <= (metaBuffer->lastItem)->seqnum) {
	insert_before = metaBuffer->firstItem;
	while (insert_before != NULL && insert_before->seqnum < seqnum)
	    insert_before = insert_before->next;
	if (insert_before != NULL && insert_before->seqnum == seqnum) {
	    fprintf(stderr,
		    "circularBuffer.c: Node %u circ_alloc_item %c%c%c -- ERROR: Seqnum %lu already exists in buffer\n",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], metaBuffer->buffName[2], seqnum);
	    circ_print_buffer_stat(metaBuffer);
	    return NULL;
	}
    }

    int slot = circ_find_free_slot(metaBuffer);
    metaBuffer->indexOfLastAdded = (unsigned long) slot;
    struct circBUFFERItem *addedItem = &metaBuffer->item_pool[slot];
    addedItem->next = NULL;
    addedItem->previous = NULL;
    addedItem->seqnum = seqnum;
    addedItem->toDiscard = 0;
    memset(addedItem->subtracted_vect, 0, SIZE_SUBTRACT_VECTOR);

    addedItem->pkt = metaBuffer->buffer[slot];
    if (zero_pdu)
	memset(addedItem->pkt, 0, sizeof(struct PDU));
    (addedItem->pkt)->empty = 0;
    (addedItem->pkt)->lossRate = 0.0f;
    (addedItem->pkt)->num = seqnum;

    metaBuffer->nbItemStored++;
    circ_link_item(metaBuffer, addedItem, insert_before);
    metaBuffer->lastAdded = addedItem;
    metaBuffer->nb_tot_added++;
    return addedItem;
}

void *circ_add_data_in_buffer(const gf * datas, int sizeofDatas,
			   struct circBUFFER *queue, uint16_t setPktType,
			   unsigned long seqnum)
{
    if (sizeofDatas < 0 || sizeofDatas > TETRYS_MAX_PAYLOAD) {
	fprintf(stderr,
		"circ_add_data_in_buffer: invalid payload size %d (max %d)\n",
		sizeofDatas, TETRYS_MAX_PAYLOAD);
	return NULL;
    }
    struct circBUFFERItem *addedItem = circ_alloc_item(seqnum, queue, 1);
    if (addedItem == NULL)
	return NULL;
    int padding_built = TETRYS_MAX_PAYLOAD - sizeofDatas;
    memcpy(&(addedItem->pkt)->datav, &padding_built, sizeof(int));
    memcpy(&(addedItem->pkt)->datav[sizeof(int)], datas, (size_t) sizeofDatas);
    (addedItem->pkt)->pls = sizeofDatas;
    if (PR_DB_2)
	fprintf(stderr,
		"\ncirc_add_data_in_buffer: nid %u - %d pls for created PDU in %c%c%c  ",
		tetrys->nodeid, (addedItem->pkt)->pls, queue->buffName[0],
		queue->buffName[1], queue->buffName[2]);
    (addedItem->pkt)->uint_pktType = setPktType;
    return addedItem;
}

void *circ_add_not_sorted_pdu_in_buffer(const struct PDU *pdu,
				    struct circBUFFER *queue)
{
    struct circBUFFERItem *addedItem =
	circ_alloc_item(queue->nb_tot_added, queue, 0);
    if (addedItem != NULL) {
	memcpy(addedItem->pkt, pdu, sizeof(struct PDU));
	(addedItem->pkt)->empty = 0;
	if (PR_DB_2)
	    fprintf(stderr,
		    "\ncirc_add_not_sorted_pdu_in_buffer:  %d pls for added PDU in %c%c%c  ",
		    (addedItem->pkt)->pls, queue->buffName[0],
		    queue->buffName[1], queue->buffName[2]);
	return addedItem;
    }
    if (PR_DB_1)
	fprintf(stderr,
		" circ_add_not_sorted_pdu_in_buffer %c%c%c failed\n",
		queue->buffName[0], queue->buffName[1], queue->buffName[2]);
    return NULL;
}

void *circ_add_pdu_in_buffer(const struct PDU *pdu, struct circBUFFER *queue)
{
    struct circBUFFERItem *addedItem = circ_alloc_item(pdu->num, queue, 0);
    if (addedItem != NULL) {
	memcpy(addedItem->pkt, pdu, sizeof(struct PDU));
	(addedItem->pkt)->empty = 0;
	return addedItem;
    }
    if (PR_DB_1)
	fprintf(stderr,
		" circ_add_pdu_in_buffer %c%c%c failed\n",
		queue->buffName[0], queue->buffName[1], queue->buffName[2]);
    return NULL;
}

void *circ_create_buffer(int nbpkt, int sizeofpkt)
{
    if (nbpkt <= 0)
	return NULL;
    struct circBUFFER *metaBuffer =
	(struct circBUFFER *) calloc(1, sizeof(struct circBUFFER));
    if (metaBuffer == NULL) {
	fprintf(stderr,
		"Node %u Not enough memory for allocating information queue -- EXIT FAILURE\n",
		tetrys->nodeid);
	quit(0);
	return NULL;
    }
    metaBuffer->nbItemStorable = nbpkt;
    metaBuffer->sizeOfBuffer = (unsigned long) nbpkt * (unsigned long) sizeofpkt;
    metaBuffer->pktSize = sizeofpkt;

    metaBuffer->pdu_pool =
	(struct PDU *) calloc((size_t) nbpkt, sizeof(struct PDU));
    metaBuffer->item_pool =
	(struct circBUFFERItem *) calloc((size_t) nbpkt,
					  sizeof(struct circBUFFERItem));
    metaBuffer->buffer =
	(struct PDU **) malloc((size_t) nbpkt * sizeof(struct PDU *));
    if (metaBuffer->pdu_pool == NULL || metaBuffer->item_pool == NULL
	|| metaBuffer->buffer == NULL) {
	free(metaBuffer->buffer);
	free(metaBuffer->item_pool);
	free(metaBuffer->pdu_pool);
	free(metaBuffer);
	fprintf(stderr,
		"Node %u Not enough memory for allocating buffer pools EXIT FAILURE\n",
		tetrys->nodeid);
	quit(0);
	return NULL;
    }
    for (int i = 0; i < nbpkt; i++) {
	metaBuffer->buffer[i] = &metaBuffer->pdu_pool[i];
	metaBuffer->pdu_pool[i].empty = 1;
    }
    return metaBuffer;
}

/**
 * Find the first free memory slot after the last added item (wrapping around).
 * The caller must have checked that the buffer is not full.
 * @param metaBuffer  Circular buffer to search for an empty PDU slot.
 * @return Index of the free slot (the caller stores it in indexOfLastAdded).
 */
static int circ_find_free_slot(const struct circBUFFER *metaBuffer)
{
    int tmp_index = (metaBuffer->indexOfLastAdded == 0)
	? -1 : (int) metaBuffer->indexOfLastAdded - 1;
    int empty = 0;

    while (empty == 0) {
	tmp_index++;
	if (tmp_index >= metaBuffer->nbItemStorable)
	    tmp_index = 0;
	if ((metaBuffer->buffer[tmp_index])->empty == 1)
	    empty = 1;
    }
    return tmp_index;
}

/**
 * Insert @p addedItem in the sorted list of @p metaBuffer, before
 * @p before_item (NULL means "at the end").
 */
static void circ_link_item(struct circBUFFER *metaBuffer,
			   struct circBUFFERItem *addedItem,
			   struct circBUFFERItem *before_item)
{
    unsigned long seqnum = addedItem->seqnum;

    if (metaBuffer->firstItem == NULL) {
	metaBuffer->firstItem = addedItem;
	metaBuffer->lastItem = metaBuffer->firstItem;
    } else if (before_item != NULL && before_item->previous != NULL) {
	addedItem->previous = before_item->previous;
	(before_item->previous)->next = addedItem;
	addedItem->next = before_item;
	before_item->previous = addedItem;
    } else if (before_item != NULL && before_item->previous == NULL
	       && (before_item->next != NULL || seqnum < before_item->seqnum)) {
	addedItem->previous = NULL;
	addedItem->next = before_item;
	before_item->previous = addedItem;
	metaBuffer->firstItem = addedItem;
    } else if (before_item == NULL) {
	(metaBuffer->lastItem)->next = addedItem;
	addedItem->previous = metaBuffer->lastItem;
	metaBuffer->lastItem = addedItem;
	addedItem->next = NULL;
    } else {
	fprintf(stderr,
		"Node %u circularBuffer: circ_link_item %c%c%c -- ERROR, exit failure",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2]);
	quit(0);
    }
}

void *circ_add_not_sorted_item_in_buffer(struct circBUFFER *metaBuffer)
{
    return circ_alloc_item(metaBuffer->nb_tot_added, metaBuffer, 1);
}

void *circ_add_item_in_buffer(unsigned long seqnum,
			   struct circBUFFER *metaBuffer)
{
    return circ_alloc_item(seqnum, metaBuffer, 1);
}

void *circ_find_from_buffer(unsigned long seqnum,
			  struct circBUFFER *metaBuffer)
{
    struct circBUFFERItem *tmp_item = metaBuffer->firstItem;
    if ((metaBuffer->firstItem == NULL) || (metaBuffer->lastItem == NULL))
	return NULL;
    if (seqnum > (metaBuffer->lastItem)->seqnum)
	return NULL;
    while (tmp_item != NULL && tmp_item->seqnum < seqnum)
	tmp_item = tmp_item->next;
    if (tmp_item != NULL && tmp_item->seqnum == seqnum)
	return tmp_item->pkt;
    return NULL;
}

/**
 * Look up the item @p seqnum in the sorted list of @p metaBuffer without
 * unlinking it. An error is printed when @p seqnum is above the last item.
 * @return The item, or NULL if it is not in the buffer.
 */
static struct circBUFFERItem *circ_find_del(unsigned long int seqnum,
					    const struct circBUFFER *metaBuffer)
{
    if (metaBuffer->firstItem == NULL || metaBuffer->lastItem == NULL)
	return NULL;
    if (seqnum > (metaBuffer->lastItem)->seqnum) {
	fprintf(stderr,
		"Node %u circ_find_del %c%c%c, seqnum %lu > last %lu and %d item stored\n",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2], seqnum,
		(metaBuffer->lastItem)->seqnum, metaBuffer->nbItemStored);
	return NULL;
    }
    struct circBUFFERItem *tmp_item = metaBuffer->firstItem;
    while (tmp_item != NULL && tmp_item->seqnum < seqnum)
	tmp_item = tmp_item->next;
    if (tmp_item != NULL && tmp_item->seqnum == seqnum)
	return tmp_item;
    return NULL;
}

int circ_del_item_x_from_buffer(int indice, struct circBUFFER *metaBuffer)
{
    int has_more = circ_init_iterator(metaBuffer);
    unsigned long int toDel = 0;
    int count = 0;
#ifdef VERBOSE
    fprintf(stderr, "node %u: buffer %c%c: looking for item #%d to remove\n",
	    tetrys->nodeid, metaBuffer->buffName[0],
	    metaBuffer->buffName[1], indice);
#endif
    while (has_more == 1) {
	has_more = circ_get_next(metaBuffer);
	if (has_more == 1) {
	    toDel = metaBuffer->iterator->seqnum;
	} else {
	    fprintf(stderr,
		    "node %u: buffer %c%c: cannot remove item #%d, the buffer holds fewer items\n",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], indice);
	    return 0;
	}
	if (count == indice)
	    has_more = 0;
	count++;
    }
#ifdef VERBOSE
    fprintf(stderr, "node %u: buffer %c%c: repair packet #%lu removed "
	    "(linearly dependent, it brings no new information)\n",
	    tetrys->nodeid, metaBuffer->buffName[0],
	    metaBuffer->buffName[1], toDel);
#endif
    return circ_del_item_from_buffer(toDel, metaBuffer);
}

int circ_pop_first_pdu_from_buffer(struct PDU *_pkt,
			       struct circBUFFER *metaBuffer)
{
    int succ = 0;
    if (metaBuffer->nbItemStored > 0) {
	if (metaBuffer->firstItem == NULL) {
	    fprintf(stderr,
		    "\nNode %u circ_pop_first_pdu_from_buffer: %c%c%c firstItem==NULL",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], metaBuffer->buffName[2]);
	    return 0;
	}
	if (metaBuffer->firstItem->pkt == NULL) {
	    fprintf(stderr,
		    "\nNode %u circ_pop_first_pdu_from_buffer: %c%c%c firstItem->pkt==NULL",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], metaBuffer->buffName[2]);
	    return 0;
	}
	if (PR_DB_2)
	    fprintf(stderr,
		    "\nNode %u circ_pop_first_pdu_from_buffer: %c%c%c pop pkt type %d ",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], metaBuffer->buffName[2],
		    metaBuffer->firstItem->pkt->uint_pktType);
	memcpy(_pkt, (metaBuffer->firstItem)->pkt, sizeof(struct PDU));

	succ =
	    circ_del_item_from_buffer(metaBuffer->firstItem->seqnum,
				   metaBuffer);
    }
    return succ;
}

int circ_del_item_from_buffer(unsigned long int seqnum,
			   struct circBUFFER *metaBuffer)
{
    struct circBUFFERItem *toDel = circ_find_del(seqnum, metaBuffer);
    if (toDel == NULL) {
	fprintf(stderr,
		"\nNode %u circularBuffer: %c%c%c delItemFromInBuffer %lu NOT FOUND %d item in buff!\n",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2], seqnum,
		metaBuffer->nbItemStored);
	return 0;
    }
    toDel->pkt->empty = 1;
    if (metaBuffer->firstItem == metaBuffer->lastItem
	|| metaBuffer->nbItemStored == 1) {
	metaBuffer->firstItem = NULL;
	metaBuffer->lastItem = NULL;
    } else if (toDel->next == NULL && toDel == metaBuffer->lastItem) {
	metaBuffer->lastItem = toDel->previous;
	metaBuffer->lastItem->next = NULL;
    } else if (toDel->previous == NULL && toDel == metaBuffer->firstItem) {
	metaBuffer->firstItem = toDel->next;
	metaBuffer->firstItem->previous = NULL;
    } else if ((toDel->next != NULL) && (toDel->previous != NULL)) {
	(toDel->next)->previous = toDel->previous;
	(toDel->previous)->next = toDel->next;
    } else {
	fprintf(stderr,
		"\nNode %u circularBuffer: %c%c%c circ_del_item_from_buffer -- ERROR invalid list links\n",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2]);
	quit(0);
    }
    toDel->next = NULL;
    toDel->previous = NULL;
    toDel->toDiscard = 0;
    metaBuffer->nbItemStored--;
    metaBuffer->lastAdded = NULL;
    return 1;
}

int circ_init_iterator(struct circBUFFER *metaBuffer)
{
    if (metaBuffer->firstItem != NULL) {
	metaBuffer->firstCall = 1;
	metaBuffer->iterator = metaBuffer->firstItem;
	return 1;
    }
    return 0;
}

int circ_get_next(struct circBUFFER *metaBuffer)
{
    if (metaBuffer->firstCall != 1) {
	if ((metaBuffer->iterator->next) != NULL) {
	    (metaBuffer->iterator) = (metaBuffer->iterator)->next;
	    return 1;
	}
	return 0;
    }
    metaBuffer->firstCall = 0;
    if (metaBuffer->iterator != NULL)
	return 1;
    return 0;
}

void circ_empty_queue(struct circBUFFER *metaBuffer)
{
    while (metaBuffer->firstItem != NULL) {
	unsigned long int seq = metaBuffer->firstItem->seqnum;
	if (!circ_del_item_from_buffer(seq, metaBuffer))
	    break;
    }
    metaBuffer->lastAdded = NULL;
    metaBuffer->firstItem = NULL;
    metaBuffer->lastItem = NULL;
    metaBuffer->nbItemStored = 0;
    metaBuffer->iterator = NULL;
    metaBuffer->indexOfLastAdded = 0;
}

void circ_free_queue(struct circBUFFER **queue)
{
    if (queue == NULL || *queue == NULL)
	return;
    struct circBUFFER *metaBuffer = *queue;
    circ_empty_queue(metaBuffer);
    free(metaBuffer->buffer);
    free(metaBuffer->item_pool);
    free(metaBuffer->pdu_pool);
    free(metaBuffer);
    *queue = NULL;
}

void circ_print_buffer_stat(const struct circBUFFER *metaBuffer)
{
    fprintf(stderr,
	    "\n\tNode %u -- %c%c%c Number of storable item   \t| %d",
	    tetrys->nodeid, metaBuffer->buffName[0],
	    metaBuffer->buffName[1], metaBuffer->buffName[2],
	    metaBuffer->nbItemStorable);
    fprintf(stderr,
	    "\n\tNode %u -- %c%c%c Number of stored item     \t| %d",
	    tetrys->nodeid, metaBuffer->buffName[0],
	    metaBuffer->buffName[1], metaBuffer->buffName[2],
	    metaBuffer->nbItemStored);
    fprintf(stderr, "\n\tNode %u -- %c%c%c Number of item added \t| %lu",
	    tetrys->nodeid, metaBuffer->buffName[0],
	    metaBuffer->buffName[1], metaBuffer->buffName[2],
	    metaBuffer->nb_tot_added);
    if (metaBuffer->firstItem != NULL) {
	fprintf(stderr,
		"\n\tNode %u -- %c%c%c First item  %lu \t| Last item %lu",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2],
		(metaBuffer->firstItem)->seqnum,
		(metaBuffer->lastItem)->seqnum);
    } else
	fprintf(stderr, "\n\tNode %u -- %c%c%c Queue Empty ",
		tetrys->nodeid, metaBuffer->buffName[0],
		metaBuffer->buffName[1], metaBuffer->buffName[2]);
}

/**
 * Find the first item at the head of @p metaBuffer that is marked @c toDiscard = 1.
 * @return Sequence number of that item, or 0 if none.
 */
static unsigned long int circ_find_first_to_discard_from_buffer(const struct
								circBUFFER
								*metaBuffer)
{
    const struct circBUFFERItem *tmp_item = metaBuffer->firstItem;
    while ((tmp_item != NULL) && (tmp_item->toDiscard == 0))
	tmp_item = tmp_item->next;
    if ((tmp_item != NULL) && (tmp_item->toDiscard == 1))
	return tmp_item->pkt->num;
    return 0;
}

void circ_del_to_discard_from_buffer(struct circBUFFER *metaBuffer)
{
    unsigned long int seqNumToDiscard;
    if (metaBuffer->nbItemStored == 0)
	return;
    if ((metaBuffer->firstItem == NULL) || (metaBuffer->lastItem == NULL))
	return;
    seqNumToDiscard = circ_find_first_to_discard_from_buffer(metaBuffer);
    while ((seqNumToDiscard > 0)
	   && (seqNumToDiscard <= metaBuffer->lastItem->pkt->num)) {
	int succ = circ_del_item_from_buffer(seqNumToDiscard, metaBuffer);
	if (succ == 0)
	    fprintf(stderr,
		    "\nNode %u circ_del_to_discard_from_buffer: %c%c%c Failed to discard num %lu\n",
		    tetrys->nodeid, metaBuffer->buffName[0],
		    metaBuffer->buffName[1], metaBuffer->buffName[2],
		    seqNumToDiscard);
	seqNumToDiscard = circ_find_first_to_discard_from_buffer(metaBuffer);
    }
    if ((metaBuffer->nbItemStored > 0)
	&& (metaBuffer->lastItem->pkt->num > 4000)
	&& (seqNumToDiscard > metaBuffer->lastItem->pkt->num - 4000)
	&& (seqNumToDiscard < metaBuffer->lastItem->pkt->num)) {
	printf
	    ("\n circ_del_to_discard_from_buffer Buffer looks corrupted\n");
    }
}

int circ_del_item_below_value(struct circBUFFER *metaBuffer,
			      unsigned long int val)
{
    int nbDel = 0;
    if ((metaBuffer->nbItemStored > 0) && (metaBuffer->firstItem != NULL)
	&& (metaBuffer->firstItem->seqnum < val)
	&& ((metaBuffer->lastItem->seqnum + 100) >= val)) {
	while ((metaBuffer->nbItemStored > 0)
	       && (metaBuffer->firstItem->seqnum < val)) {
	    if (PR_DB_1)
		printf("- %lu val %lu seq ", val,
		       metaBuffer->firstItem->seqnum);
	    int succ =
		circ_del_item_from_buffer(metaBuffer->firstItem->seqnum,
				       metaBuffer);
	    if (succ == 0)
		printf
		    ("\ncirc_del_item_below_value seqnum %lu not found",
		     metaBuffer->firstItem->seqnum);
	    nbDel++;
	}
    }
    return nbDel;
}

int circ_del_item_below_or_equal_value(struct circBUFFER *metaBuffer,
				  unsigned long int val)
{
    int nbDel = 0;
    if ((metaBuffer->nbItemStored > 0) && (metaBuffer->firstItem != NULL)
	&& (metaBuffer->firstItem->seqnum <= val)
	&& ((metaBuffer->lastItem->seqnum + 100) >= val)) {
	while ((metaBuffer->nbItemStored > 0)
	       && (metaBuffer->firstItem->seqnum <= val)) {
	    if (PR_DB_1)
		printf("- %lu val %lu seq ", val,
		       metaBuffer->firstItem->seqnum);
	    int succ =
		circ_del_item_from_buffer(metaBuffer->firstItem->seqnum,
				       metaBuffer);
	    if (succ == 0)
		printf
		    ("\ncirc_del_item_below_or_equal_value seqnum %lu not found",
		     metaBuffer->firstItem->seqnum);
	    nbDel++;
	}
    }
    return nbDel;
}
