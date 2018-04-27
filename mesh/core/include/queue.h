/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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
 * Initialize a queue instance.
 *
 * @param[in,out] p_queue Pointer to an uninitialized queue instance.
 */
void queue_init(queue_t* p_queue);

/**
 * Push a single queue element to the back of the given queue instance.
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
 * Pop the element at the front of the queue, removing it from the queue.
 *
 * @param[in] p_queue Queue instance to pop from.
 *
 * @returns The @c p_data pointer of the element at the front of the queue, or NULL if the queue is empty.
 */
queue_elem_t* queue_pop(queue_t* p_queue);

/**
 * Peek at the element at the front of the queue, but don't remove it.
 *
 * @param[in] p_queue Queue instance to peek from.
 *
 * @returns The @c p_data pointer of the element at the front of the queue, or NULL if the queue is empty.
 */
queue_elem_t* queue_peek(const queue_t* p_queue);

/** @} */

#endif /* QUEUE_H__ */

