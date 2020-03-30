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

#ifndef MSQUEUE_H__
#define MSQUEUE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup MULTI_STAGE_QUEUE Multi stage queue module
 * @ingroup MESH_CORE
 * Generic multi-stage FIFO queue for fixed-size objects.
 *
 * The multi-stage queue operates very similarly to the @c fifo_t queue, with three distictions:
 * - It does not do any IRQ locking, and assumes that each "stage" in the queue is operated from a
 *   single context.
 * - It does not do any copying, but instead provides pointers directly into the queue buffer.
 * - The multistage queue operates in N stages, where each stage is an index in the queue. This
 *   allows the user to process elements in a pipeline-like manner, where each stage processes an
 *   element, before making it available to the next. Stage 0 is the head of the pipeline, while
 *   the last stage is the tail.
 *
 * Example usage:
 * @code{c}
    void set_value(void)
    {
        uint32_t * p_integer = msq_get(&m_queue, 0);
        *p_integer = 42;
        msq_move(&queue, 0);
    }

    uint32_t get_value(void)
    {
        uint32_t * p_integer = msq_get(&m_queue, 1);
        uint32_t retval = *p_integer;
        msq_move(&queue, 1);
        return retval;
    }
 * @endcode
 * @{
 */


/**
 * Single queue instance.
 */
typedef struct
{
   uint8_t stage_count; /**< The number of stages in the @c p_stages array. */
   uint8_t elem_size;   /**< Size of a single element in bytes. */
   uint8_t elem_count;  /**< Number of elements in the elem_array. Must be a power of two. */
   uint8_t * p_stages;  /**< Array used for keeping track of the different stages of the queue. */
   void * p_elem_array; /**< Element array of the elements operated on. */
} msq_t;

/**
 * Initializes a message queue instance.
 *
 * All member variables will be checked, and the queue will be flushed.
 *
 * @param[in,out] p_queue The queue to initialize.
 */
void msq_init(msq_t * p_queue);

/**
 * Gets the element in a specified stage.
 *
 * @param[in] p_queue Queue to get from.
 * @param[in] stage   Stage in the queue to get from.
 *
 * @returns A pointer to the element at the specified stage, or NULL if no element has reached
 *          this stage.
 */
void * msq_get(const msq_t * p_queue, uint8_t stage);

/**
 * Moves a stage one element, marking the completion of this stage for the current element.
 *
 * @param[in,out] p_queue Queue to move in.
 * @param[in]     stage   Stage to move.
 */
void msq_move(msq_t * p_queue, uint8_t stage);

/**
 * Flushes all stages in the queue.
 *
 * @param[in,out] p_queue Queue to flush.
 */
void msq_reset(msq_t * p_queue);

/**
 * Gets the number of available elements at the given stage.
 *
 * @param[in] p_queue Queue to check.
 * @param[in] stage   Stage in the queue to check.
 *
 * @returns The number of elements available in the given stage.
 */
uint8_t msq_available(const msq_t * p_queue, uint8_t stage);

/** @} */

#endif /* MSQUEUE_H__ */
