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

#ifndef CIRC_STRUCT
# define CIRC_STRUCT 1

#include "common_struct_def.h"

/**
 * \struct circBUFFERItem circularBuffer.h
 * \brief A single node in the doubly-linked list that backs a circBUFFER.
 *
 * Each item is drawn from its queue's pre-allocated pool (@c item_pool) and
 * holds a pointer to a pooled PDU (@c pdu_pool) plus a bit-vector
 * (@c subtracted_vect) tracking which data packets have been subtracted
 * from the repair symbol stored in this item during FEC decoding.
 */
struct circBUFFERItem {
    struct circBUFFERItem *previous; /**< Link to the preceding item (NULL for the first). */
    struct circBUFFERItem *next;     /**< Link to the following item (NULL for the last). */
    unsigned long seqnum;            /**< Sequence number of the packet stored in this item. */
    int toDiscard;                   /**< Non-zero if this item should be recycled at the next cleanup pass. */
    /** Bit-vector tracking subtraction state for FEC decoding.
     *  Bit i = 1 means data packet at offset i from min_redundancy has been
     *  subtracted from this repair symbol. */
    gf subtracted_vect[SIZE_SUBTRACT_VECTOR];
    struct PDU *pkt;                 /**< Pointer to the packet payload in @c pdu_pool. */
};

/**
 * \struct circBUFFER circularBuffer.h
 * \brief A doubly-linked sorted packet queue backed by contiguous PDU/item pools.
 *
 * Despite its name this is not a ring buffer — it is an ordered linked list
 * backed by pre-allocated arrays (@c pdu_pool and @c item_pool) so steady-state
 * insertions and deletions perform zero heap allocations or frees.
 */
struct circBUFFER {
    struct PDU *pdu_pool;            /**< Contiguous pool of @c nbItemStorable PDU slots. */
    struct circBUFFERItem *item_pool; /**< Contiguous pool of @c nbItemStorable list nodes (1-to-1 with @c pdu_pool). */
    struct PDU **buffer;             /**< Slot pointer table indexing @c pdu_pool[i]. */
    char buffName[3];                /**< Short human-readable label (e.g. "SE", "RR"). */
    int nbItemStorable;              /**< Maximum number of items the queue is allowed to hold. */
    int nbItemStored;                /**< Current number of items in the queue. */
    unsigned long int nb_tot_added;  /**< Cumulative count of all items ever added. */
    int pktSize;                     /**< Payload size of packets stored in this queue (bytes). */
    unsigned long sizeOfBuffer;      /**< Allocated size of the legacy buffer array (bytes). */
    struct circBUFFERItem *firstItem; /**< Head of the linked list (lowest sequence number). */
    struct circBUFFERItem *lastItem;  /**< Tail of the linked list (highest sequence number). */
    unsigned long indexOfLastAdded;   /**< Slot index of the most recently added item. */
    struct circBUFFERItem *iterator;  /**< Cursor for sequential traversal via circ_get_next(). */
    struct circBUFFERItem *lastAdded; /**< Pointer to the most recently inserted item. */
    int firstCall;                    /**< Non-zero before the first circ_get_next() call on this iterator. */
};

/* -----------------------------------------------------------------------
 * Buffer creation and destruction
 * ----------------------------------------------------------------------- */

/**
 * Allocate and initialise a new pooled packet queue.
 * Ownership passes to the caller, who must release it with circ_free_queue().
 * @param nbpkt     Maximum number of packets the buffer should hold.
 * @param sizeofpkt Size of each packet payload (bytes).
 * @return Pointer to the new circBUFFER (exits via quit() on allocation failure).
 */
void *circ_create_buffer(int nbpkt, int sizeofpkt);

/** Release all memory associated with the queue and its pools, setting @p *queue to NULL. */
void circ_free_queue(struct circBUFFER **queue);

/** Remove all items from the queue without freeing the queue itself. */
void circ_empty_queue(struct circBUFFER *metaBuffer);

/* -----------------------------------------------------------------------
 * Insertion — three variants for different use cases
 * ----------------------------------------------------------------------- */

/**
 * Insert a PDU into the queue, sorted by pdu->num (sequence number).
 * The PDU is copied by value into a pooled slot owned by @p queue.
 * @return Pointer to the new item (owned by @p queue), or NULL on duplicate.
 */
void *circ_add_pdu_in_buffer(const struct PDU *pdu, struct circBUFFER *queue);

/**
 * Insert a PDU at the tail of the queue WITHOUT sorting.
 * Use this when packets are known to arrive already in order, or when
 * ordering is not required and speed matters (O(1) tail append).
 * @return Pointer to the new item (owned by @p queue), or NULL on duplicate.
 */
void *circ_add_not_sorted_pdu_in_buffer(const struct PDU *pdu, struct circBUFFER *queue);

/**
 * Insert a raw byte array as a new data packet at the tail (unsorted).
 * Wraps the raw data in a PDU with type @p setPktType and sequence @p seqnum.
 * @return Pointer to the new item (owned by @p queue), or NULL on error/duplicate.
 */
void *circ_add_data_in_buffer(const gf *datas, int sizeofDatas,
                           struct circBUFFER *queue, uint16_t setPktType,
                           unsigned long seqnum);

/**
 * Allocate a new zeroed item and append it at the tail (unsorted).
 * The caller is responsible for filling in the item's fields afterwards.
 * @return Pointer to the new item (owned by @p metaBuffer), or NULL on duplicate.
 */
void *circ_add_not_sorted_item_in_buffer(struct circBUFFER *metaBuffer);

/**
 * Allocate a new zeroed item with the given sequence number, inserted sorted.
 * @return Pointer to the new item (owned by @p metaBuffer), or NULL on duplicate.
 */
void *circ_add_item_in_buffer(unsigned long seqnum, struct circBUFFER *metaBuffer);

/* -----------------------------------------------------------------------
 * Lookup and removal
 * ----------------------------------------------------------------------- */

/**
 * Find the PDU with the given sequence number.
 * @return Pointer to the matching struct PDU (owned by @p metaBuffer), or NULL if not found.
 */
void *circ_find_from_buffer(unsigned long seqnum, struct circBUFFER *metaBuffer);

/**
 * Remove the item with the given sequence number and free it.
 * @return 1 on success, 0 if not found.
 */
int circ_del_item_from_buffer(unsigned long seqnum, struct circBUFFER *metaBuffer);

/**
 * Remove the item at position @p indice (0-based from the head) and free it.
 * @return 1 on success, 0 if out of range.
 */
int circ_del_item_x_from_buffer(int indice, struct circBUFFER *metaBuffer);

/**
 * Remove all items whose sequence number is strictly less than @p val.
 * @return Number of items removed.
 */
int circ_del_item_below_value(struct circBUFFER *metaBuffer, unsigned long int val);

/**
 * Remove all items whose sequence number is less than or equal to @p val.
 * @return Number of items removed.
 */
int circ_del_item_below_or_equal_value(struct circBUFFER *metaBuffer, unsigned long int val);

/** Remove all items that have been marked with toDiscard = 1. */
void circ_del_to_discard_from_buffer(struct circBUFFER *metaBuffer);

/* -----------------------------------------------------------------------
 * Pop (remove and return) operations
 * ----------------------------------------------------------------------- */

/**
 * Copy the first PDU into @p _pkt and remove it from the queue.
 * @return 1 on success, 0 if the queue is empty.
 */
int circ_pop_first_pdu_from_buffer(struct PDU *_pkt, struct circBUFFER *metaBuffer);

/* -----------------------------------------------------------------------
 * Sequential iteration
 * ----------------------------------------------------------------------- */

/**
 * Reset the iterator to the head of the queue.
 * Must be called before the first circ_get_next() on a new traversal.
 * @return 1 on success, 0 if the queue is empty.
 */
int circ_init_iterator(struct circBUFFER *metaBuffer);

/**
 * Advance the iterator to the next item.
 * @return 1 if the iterator moved to a valid item, 0 at end-of-queue.
 */
int circ_get_next(struct circBUFFER *metaBuffer);

/* -----------------------------------------------------------------------
 * Miscellaneous
 * ----------------------------------------------------------------------- */

/** Print queue statistics (name, items stored, total added) to stdout. */
void circ_print_buffer_stat(const struct circBUFFER *metaBuffer);

#endif
