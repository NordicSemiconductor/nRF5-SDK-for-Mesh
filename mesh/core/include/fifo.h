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
#ifndef MESH_FIFO_H__
#define MESH_FIFO_H__

/**
 * @defgroup FIFO Generic FIFO-implementation
 * @ingroup MESH_CORE
 * Allows instances of any type to be queued in a FIFO.
 * @{
 */


#include <stdint.h>
#include <stdbool.h>

#include "nrf_mesh_config_core.h"

/**
 * @brief Specialized function pointer type for copying memory between two elements in the FIFO.
 *        Used to replace memcpy in case the elements require more complex copy mechanisms
 *        (e.g. deep copy).
 */
typedef void (*fifo_memcpy_t)(void* dest, const void* src);

/**
 * FIFO-structure, containing metadata about the elements and buffer for a single
 *        FIFO-instance.
 */
typedef struct
{
    /**
     * Pointer to an array of elements, which will be used as a buffer for the
     * FIFO. The number of elements must be in the power of two.
     */
    void*           elem_array;
    uint32_t        elem_size;          /**< Size of a single element in bytes. */
    uint32_t        array_len;          /**< Number of elements in the elem_array. Must be in the power of two. */
    uint32_t        head;               /**< Head index. Only for internal housekeeping. */
    uint32_t        tail;               /**< Tail index. Only for internal housekeeping. */
#if FIFO_STATS
    uint32_t        pushes;             /**< Number of successful pushes. */
    uint32_t        drops;              /**< Number of dropped elements because of full FIFO. */
    uint32_t        max_len;            /**< Max number of enqueued elements. */
#endif
} fifo_t;

/**
 * Initialize the given FIFO-instance.
 *
 * @note       The FIFO structure fields elem_array, elem_size, and array_len must be filled before
 *             init is called on the structure.
 *
 * @param[in]  p_fifo                    Pointer to a FIFO-structure
 */
void fifo_init(fifo_t* p_fifo);

/**
 * Copy the given element to the head of the FIFO.
 *
 * @param[in,out] p_fifo FIFO queue to enqueue element to.
 * @param[in] p_elem Element to enqueue.
 *
 * @retval NRF_SUCCESS The element was successfully enqueued.
 * @retval NRF_ERROR_INVALID_LENGTH The FIFO structure has been initialized
 *         with invalid length, and cannot be used.
 * @retval NRF_ERROR_NO_MEM The FIFO is full, and the element can't be
 *         enqueued.
 */
uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem);

/**
 * Copy the element at the tail to the given element. Frees the element from
 *  the given queue.
 *
 * @param[in,out] p_fifo FIFO queue to pop element from.
 * @param[out] p_elem Memory to copy into.
 *
 * @retval NRF_SUCCESS The element was successfully popped.
 * @retval NRF_ERROR_INVALID_LENGTH The FIFO structure has been initialized
 *         with invalid length, and cannot be used.
 * @retval NRF_ERROR_NOT_FOUND The FIFO is empty, and the element can't be
 *         popped.
 */
uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem);

/**
 * Peek with an offset from the tail of the given FIFO, but don't remove any
 *  elements.
 *
 * @param[in,out] p_fifo FIFO queue to peek at.
 * @param[out] p_elem Memory to copy into.
 * @param[in] elem The offset from the tail of the queue to peek at.
 *
 * @retval NRF_SUCCESS The element was successfully peeked at, and copied to
 *         the given memory.
 * @retval NRF_ERROR_INVALID_LENGTH The FIFO structure has been initialized
 *         with invalid length, and cannot be used.
 * @retval NRF_ERROR_NOT_FOUND The given offset is larger than the number of
 *         enqueued elements, and the element can't be peeked at.
 */
uint32_t fifo_peek_at(const fifo_t* p_fifo, void* p_elem, uint32_t elem);

/**
 * Peek at the tail of the given FIFO, but don't remove any elements.
 *
 * @param[in,out] p_fifo FIFO queue to peek at.
 * @param[out] p_elem Memory to copy into.
 *
 * @retval NRF_SUCCESS The element was successfully peeked at, and copied to
 *         the given memory.
 * @retval NRF_ERROR_INVALID_LENGTH The FIFO structure has been initialized
 *         with invalid length, and cannot be used.
 * @retval NRF_ERROR_NOT_FOUND The FIFO is empty, and the element can't be
 *         peeked at.
 */
uint32_t fifo_peek(const fifo_t* p_fifo, void* p_elem);

/**
 * Remove all elements from the given FIFO.
 *
 * @param[in,out] p_fifo The FIFO structure to flush.
 */
void fifo_flush(fifo_t* p_fifo);

/**
 * Get the number of elements queued up in the given FIFO.
 *
 * @param[in] p_fifo The FIFO structure to get the length of.
 *
 * @return The number of enqueued elements in the given FIFO.
 */
uint32_t fifo_get_len(const fifo_t* p_fifo);

/**
 * Return whether the given FIFO is full.
 *
 * @param[in] p_fifo The FIFO structure to check.
 *
 * @return Whether or not the given FIFO is full.
 */
bool fifo_is_full(const fifo_t* p_fifo);

/**
 * Return whether the given FIFO is empty.
 *
 * @param[in] p_fifo The FIFO structure to check.
 *
 * @return Whether or not the given FIFO is empty.
 */
bool fifo_is_empty(const fifo_t* p_fifo);

/** @} */

#endif /* FIFO_H__ */
