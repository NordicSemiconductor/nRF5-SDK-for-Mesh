/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QUEUE_H__
#define QUEUE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "utils.h"

/**
 * @defgroup QUEUE Linked list queue implementation
 * @ingroup MESH_CORE
 * @{
 */

/** Queue element instance, representing a single queue element. */
typedef struct queue_elem
{
    void* p_data;               /**< Pointer to data representing the queue element. Never changed by the queue module. */
    struct queue_elem* p_next;  /**< Pointer to the next element in the queue. Set and used by the queue module. */
} queue_elem_t;

/** Queue instance. All manipulation of struct internals should be left to the queue module. */
typedef struct
{
    queue_elem_t* p_front; /**< Pointer to the front of the queue, where the queue elements are popped. */
    queue_elem_t* p_back;  /**< Pointer to the back of the queue, where the queue elements are pushed. */
} queue_t;

/**
 * Initializes a queue instance.
 *
 * @param[in,out] p_queue Pointer to an uninitialized queue instance.
 */
void queue_init(queue_t* p_queue);

/**
 * Pushes a single queue element to the back of the given queue instance.
 *
 * @warning: The user has to set the @c p_data pointer to reference the desired
 * data of the element before pushing it to the queue. This function will
 * assert if @c p_data is NULL. The @c p_data member will never be changed by
 * the queue module.
 *
 * @param[in,out] p_queue The queue instance to push to.
 * @param[in] p_elem Pointer to a statically allocated queue element to push to the back of the queue.
 */
void queue_push(queue_t* p_queue, queue_elem_t* p_elem);

/**
 * Pops the element at the front of the queue, and removes it from the queue.
 *
 * @param[in] p_queue Queue instance to pop from.
 *
 * @returns The @c p_data pointer of the element at the front of the queue, or NULL if the queue is empty.
 */
queue_elem_t* queue_pop(queue_t* p_queue);

/**
 * Peeks at the element at the front of the queue, but does not remove it.
 *
 * @param[in] p_queue Queue instance to peek from.
 *
 * @returns The @c p_data pointer of the element at the front of the queue, or NULL if the queue is empty.
 */
queue_elem_t* queue_peek(const queue_t* p_queue);

/**
 * Pops all elements of @p p_src and adds them at the end of @p p_dst.
 *
 * After this operation, @p p_dst will contain all of its own elements (in their original order), then
 * all the elements of @p p_src (in their original order). @p p_src will be empty.
 *
 * @param[in,out] p_dst Destination queue for all the elements.
 * @param[in,out] p_src Source queue for all the elements.
 */
void queue_merge(queue_t * p_dst, queue_t * p_src);

/**
 * @defgroup QUEUE_MANIPULATION Queue manipulation API
 *
 * API for manipulating the contents of the queue other than the head and tail. Allows for
 * iterating, adding, and removing items from anywhere in the queue.
 * @{
 */

/** Queue iterator structure */
typedef struct
{
    queue_t * p_queue; /**< Queue this iterator is iterating over. */
    queue_elem_t ** pp_elem; /**< Double pointer to the current iterator element. */
    bool repeat; /**< Whether to repeat the same element again. */
} queue_elem_iterator_t;

/**
 * Iterator init value for the beginning of the queue.
 *
 * @param[in] p_QUEUE Queue to iterate over.
 */
#define QUEUE_ITERATOR_BEGIN(p_QUEUE)   \
    {                                   \
        .p_queue = (p_QUEUE),           \
        .pp_elem = &(p_QUEUE)->p_front, \
        .repeat = false                 \
    }
/**
 * Iterator init value for the end of the queue.
 *
 * @param[in] p_QUEUE Queue to iterate over.
 */
#define QUEUE_ITERATOR_END(p_QUEUE)   \
    {                                   \
        .p_queue = (p_QUEUE),           \
        .pp_elem = &(p_QUEUE)->p_back, \
        .repeat = false                 \
    }

/**
 * Iterates over a queue like for loop.
 *
 * @warning This function is not thread-safe. It should only be used on queues that operate in a
 * single IRQ level.
 *
 * @param[in] p_QUEUE Queue to iterate over.
 * @param[in] ITERATOR Iterator name.
 */
#define QUEUE_FOREACH(p_QUEUE, ITERATOR)                                 \
    for (queue_elem_iterator_t ITERATOR = QUEUE_ITERATOR_BEGIN(p_QUEUE); \
         (*((ITERATOR).pp_elem) != NULL);                                \
         queue_iterator_iterate(&ITERATOR))

/**
 * Removes the element the iterator is at.
 *
 * The iterator will be moved to the next element immediately, instead of being moved at the next
 * iteration call. This allows the user to remove items in the middle of an iteration without
 * skipping any elements:
 *
 * @code{.c}
 * QUEUE_FOREACH(p_queue, it)
 * {
 *     if (should_delete_elem(&it))
 *     {
 *         queue_iterator_elem_remove(&it);
 *     }
 * }
 * @endcode
 *
 * In this code snippet, the @p should_delete_elem call will be executed on every queue element,
 * even though the @c queue_iterator_elem_remove call removes elements.
 *
 * Example of removing node B (with the iterator location):
 * @code
 * Before:
 *  A -> B -> C -> D
 *       ^it
 * After:
 *  A -> C -> D
 *       ^it
 * After next iterate:
 *  A -> C -> D
 *       ^it
 * @endcode
 *
 * @warning This function is not thread-safe. It should only be used on queues that operate in a
 * single IRQ level.
 *
 * @param[in,out] p_it Iterator to remove element at.
 */
void queue_iterator_elem_remove(queue_elem_iterator_t * p_it);

/**
 * Inserts an element at the current iterator location.
 *
 * Inserts the new element at the position of the iterator, before the element the iterator is
 * currently pointing at. The iterator will stay at the new entry, and will move to the same entry
 * as it was on prior to the insertion on the next iteration.
 *
 * Example of inserting node D (with the iterator location):
 * @code
 * Before:
 *  A -> B -> C
 *       ^it
 * After:
 *  A -> D -> B -> C
 *       ^it
 * After next iterate:
 *  A -> D -> B -> C
 *            ^it
 * @endcode
 *
 * @warning This function is not thread-safe. It should only be used on queues that operate in a
 * single IRQ level.
 *
 * @param[in,out] p_it Iterator to insert an element at.
 * @param[in,out] p_elem Element to insert.
 */
void queue_iterator_elem_insert(queue_elem_iterator_t * p_it, queue_elem_t * p_elem);

/**
 * Iterates to the next element.
 *
 * @warning This function is not thread-safe. It should only be used on queues that operate in a
 * single IRQ level.
 *
 * @param[in,out] p_it Iterator to iterate.
 */
void queue_iterator_iterate(queue_elem_iterator_t * p_it);

/** @} */

/** @} */

#endif /* QUEUE_H__ */

