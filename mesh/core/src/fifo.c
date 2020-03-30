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

#include <string.h>

#include "fifo.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define FIFO_ELEM_AT(p_fifo, index) ((uint8_t*) (p_fifo->elem_array) + (p_fifo->elem_size) * (index))
#define FIFO_IS_FULL(p_fifo)    (p_fifo->tail + p_fifo->array_len == p_fifo->head)
#define FIFO_IS_EMPTY(p_fifo)   (p_fifo->tail                     == p_fifo->head)
/*****************************************************************************
* Interface functions
*****************************************************************************/
void fifo_init(fifo_t* p_fifo)
{
    NRF_MESH_ASSERT(p_fifo != NULL);
    NRF_MESH_ASSERT(p_fifo->elem_size > 0 && p_fifo->elem_array != NULL && p_fifo->array_len > 0);
    NRF_MESH_ASSERT((p_fifo->array_len & (p_fifo->array_len - 1)) == 0);

    p_fifo->head = 0;
    p_fifo->tail = 0;

#if FIFO_STATS
    p_fifo->pushes = 0;
    p_fifo->drops = 0;
    p_fifo->max_len = 0;
#endif
}

uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem)
{
    if (p_fifo == NULL || p_elem == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (p_fifo->array_len == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (FIFO_IS_FULL(p_fifo))
    {
#if FIFO_STATS
        p_fifo->drops++;
#endif
        _ENABLE_IRQS(was_masked);
        return NRF_ERROR_NO_MEM;
    }

    void* p_dest = FIFO_ELEM_AT(p_fifo, (p_fifo->head) & (p_fifo->array_len - 1));
    memcpy(p_dest, p_elem, p_fifo->elem_size);
    ++p_fifo->head;

#if FIFO_STATS
    uint32_t len = fifo_get_len(p_fifo);
    if (len > p_fifo->max_len)
    {
        p_fifo->max_len = len;
    }
    p_fifo->pushes++;
#endif

    _ENABLE_IRQS(was_masked);

    return NRF_SUCCESS;
}

uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem)
{
    if (p_fifo == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (p_fifo->array_len == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (FIFO_IS_EMPTY(p_fifo))
    {
        _ENABLE_IRQS(was_masked);
        return NRF_ERROR_NOT_FOUND;
    }

    void* p_src = FIFO_ELEM_AT(p_fifo, (p_fifo->tail) & (p_fifo->array_len - 1));

    if (p_elem != NULL)
    {
        memcpy(p_elem, p_src, p_fifo->elem_size);
    }

    ++p_fifo->tail;
    _ENABLE_IRQS(was_masked);

    return NRF_SUCCESS;
}

uint32_t fifo_peek_at(const fifo_t* p_fifo, void* p_elem, uint32_t elem)
{
    if (p_fifo == NULL || p_elem == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (p_fifo->array_len == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (fifo_get_len(p_fifo) <= elem)
    {
        _ENABLE_IRQS(was_masked);
        return NRF_ERROR_NOT_FOUND;
    }

    void* p_src = FIFO_ELEM_AT(p_fifo, (p_fifo->tail + elem) & (p_fifo->array_len - 1));
    memcpy(p_elem, p_src, p_fifo->elem_size);
    _ENABLE_IRQS(was_masked);

    return NRF_SUCCESS;
}

uint32_t fifo_peek(const fifo_t* p_fifo, void* p_elem)
{
    return fifo_peek_at(p_fifo, p_elem, 0);
}

void fifo_flush(fifo_t* p_fifo)
{
    p_fifo->tail = p_fifo->head;
}

uint32_t fifo_get_len(const fifo_t* p_fifo)
{
    return (p_fifo->head - p_fifo->tail);
}

bool fifo_is_full(const fifo_t* p_fifo)
{
    return FIFO_IS_FULL(p_fifo);
}

bool fifo_is_empty(const fifo_t* p_fifo)
{
    return FIFO_IS_EMPTY(p_fifo);
}
