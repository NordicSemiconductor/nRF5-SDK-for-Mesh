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
#include <stdbool.h>
#include <string.h>

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_core.h"
#include "packet_buffer.h"
#include "nrf_error.h"
#include "utils.h"
#include "toolchain.h"
#include "nordic_common.h"

/*******************************                  *******************************
******************************** Local functions ********************************
********************************                  *******************************/

static inline uint16_t m_get_packet_buffer_index(const packet_buffer_t * p_buffer, const packet_buffer_packet_t * p_packet)
{
    return (uint16_t)((const uint8_t *)p_packet - p_buffer->buffer);
}


static inline packet_buffer_packet_t * m_get_packet(const packet_buffer_t * p_buffer, uint16_t index)
{
    return (packet_buffer_packet_t *) &p_buffer->buffer[index];
}

static void m_index_increment(const packet_buffer_t * p_buffer, uint16_t * p_index)
{
    uint16_t packet_size = sizeof(packet_buffer_packet_t) + m_get_packet(p_buffer, *p_index)->size;
    uint16_t next = *p_index + ALIGN_VAL(packet_size, WORD_SIZE);

    if (next > p_buffer->size - sizeof(packet_buffer_packet_t))
    {
        /* can't fit a header, roll over. */
        next = 0;
    }

    *p_index = next;
}

static packet_buffer_packet_t * m_get_next_packet(const packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    uint16_t index = m_get_packet_buffer_index(p_buffer, p_packet);
    m_index_increment(p_buffer, &index);
    return m_get_packet(p_buffer, index);
}

static void m_free_popped_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    NRF_MESH_ASSERT((uint8_t *) p_packet == &p_buffer->buffer[p_buffer->tail]);

    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;

    m_index_increment(p_buffer, &p_buffer->tail);
    if (m_get_packet(p_buffer, p_buffer->tail)->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_buffer->tail = 0;
    }
}

static void m_free_reserved_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    /* Only the head can be reserved */
    NRF_MESH_ASSERT((uint8_t *) p_packet == &p_buffer->buffer[p_buffer->head]);

    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
}

static uint16_t m_max_packet_len_get(const packet_buffer_t *  const p_buffer)
{
    return (p_buffer->size - sizeof(packet_buffer_packet_t));
}

static packet_buffer_packet_t * m_reserve_packet(packet_buffer_t * p_buffer, uint16_t length)
{
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->head);
    p_packet->size = length;
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_RESERVED;

#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif
    return p_packet;
}

static void m_reset_buffer(packet_buffer_t * p_buffer)
{
    m_get_packet(p_buffer, 0)->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    p_buffer->head                          = 0;
    p_buffer->tail                          = 0;
}

/* Checks if there is sufficient space in the packet buffer for the given packet length,
 * moves the packet_buffer head and tail indexes as necessary. */
static uint32_t m_prepare_for_reserve(packet_buffer_t * p_buffer, uint16_t length)
{
    uint16_t packet_len_with_header = ALIGN_VAL(length + sizeof(packet_buffer_packet_t), WORD_SIZE);
    uint32_t status;

    if (p_buffer->head < p_buffer->tail)
    {
        if (packet_len_with_header <= (p_buffer->tail - p_buffer->head))
        {
            status = NRF_SUCCESS;
        }
        else
        {
            /* There's just no space */
            status = NRF_ERROR_NO_MEM;
        }
    }
    else if (p_buffer->head > p_buffer->tail)
    {
        uint32_t space_before_end = (p_buffer->size - p_buffer->head);

        if (packet_len_with_header <= space_before_end)
        {
            status = NRF_SUCCESS;
        }
        else if (packet_len_with_header <= p_buffer->tail)
        {
            /* There's space at the beginning, pad the rest of the buffer */
            if (sizeof(packet_buffer_packet_t) <= space_before_end)
            {
                m_get_packet(p_buffer, p_buffer->head)->packet_state = PACKET_BUFFER_MEM_STATE_PADDING;
            }
            p_buffer->head = 0;
            status = NRF_SUCCESS;
        }
        else
        {
            status = NRF_ERROR_NO_MEM;
        }
    }
    else /* head == tail */
    {
        if (m_get_packet(p_buffer, p_buffer->tail)->packet_state == PACKET_BUFFER_MEM_STATE_FREE)
        {
            /* Buffer is empty */
            m_reset_buffer(p_buffer);
            status = NRF_SUCCESS;
        }
        else
        {
            /* Completely full */
            status = NRF_ERROR_NO_MEM;
        }
    }
    return status;
}

/*******************************                  *******************************
******************************** Public functions *******************************
********************************                  *******************************/
uint16_t packet_buffer_max_packet_len_get(const packet_buffer_t * const p_buffer)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(p_buffer->size > sizeof(packet_buffer_packet_t));

    return m_max_packet_len_get(p_buffer);
}


void packet_buffer_init(packet_buffer_t * p_buffer, void * const p_pool, const uint16_t pool_size)
{
    NRF_MESH_ASSERT(NULL != p_pool);
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(IS_VALID_RAM_ADDR(p_pool));
    NRF_MESH_ASSERT(IS_VALID_RAM_ADDR( (uint8_t *) p_pool + pool_size - 1));
    NRF_MESH_ASSERT(pool_size > sizeof(packet_buffer_packet_t) );

    p_buffer->size = pool_size;
    p_buffer->head = 0;
    p_buffer->tail = 0;
    p_buffer->buffer = (uint8_t*) p_pool;
    packet_buffer_packet_t * p_first_packet = m_get_packet(p_buffer, 0);
    p_first_packet->size = p_buffer->size - sizeof(packet_buffer_packet_t);
    p_first_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
}

void packet_buffer_flush(packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    /* Can't flush while a packet is reserved: */
    NRF_MESH_ASSERT(m_get_packet(p_buffer, p_buffer->head)->packet_state !=
                    PACKET_BUFFER_MEM_STATE_RESERVED);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    bool has_popped_packet = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED);

    if (has_popped_packet)
    {
        /* Move head to the next packet after the popped packet.  */
        p_packet = m_get_next_packet(p_buffer, p_packet);
        p_buffer->head         = m_get_packet_buffer_index(p_buffer, p_packet);
        p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }
    else
    {
        m_reset_buffer(p_buffer);
    }

    _ENABLE_IRQS(was_masked);
}

uint32_t packet_buffer_reserve(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet, uint16_t length)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != pp_packet);
    NRF_MESH_ASSERT(m_get_packet(p_buffer, p_buffer->head)->packet_state !=
                    PACKET_BUFFER_MEM_STATE_RESERVED);

    if (length == 0 || length > m_max_packet_len_get(p_buffer))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    /* Check if the packet buffer has enough space for the requested packet. */
    uint32_t status = m_prepare_for_reserve(p_buffer, length);
    if (NRF_SUCCESS == status)
    {
        *pp_packet = m_reserve_packet(p_buffer, length);
    }

    _ENABLE_IRQS(was_masked);
    return status;
}

void packet_buffer_commit(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet, uint16_t length)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != p_packet);
    NRF_MESH_ASSERT(PACKET_BUFFER_MEM_STATE_RESERVED == p_packet->packet_state);
    NRF_MESH_ASSERT(p_packet->size >= length);
    NRF_MESH_ASSERT(0 < length);

    /* Can end up asserting if a higher priority interrupt comes in and tries to pop twice after
     * we've marked the current packet as committed and before we mark the next as free. */
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    p_packet->size         = length;
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_COMMITTED;

    /* Move the head to the next available slot */
    m_index_increment(p_buffer, &p_buffer->head);

    if (p_buffer->head != p_buffer->tail)
    {
        /* Mark the current head as available */
        packet_buffer_packet_t * p_next_packet = m_get_packet(p_buffer, p_buffer->head);
        p_next_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }

#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif

    _ENABLE_IRQS(was_masked);
}

uint32_t packet_buffer_pop(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != pp_packet);

    uint32_t status = NRF_SUCCESS;

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);

    if (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        m_index_increment(p_buffer, &p_buffer->tail);
        p_packet = m_get_packet(p_buffer, p_buffer->tail);
    }

    switch (p_packet->packet_state)
    {
        case PACKET_BUFFER_MEM_STATE_COMMITTED:
            p_packet->packet_state = PACKET_BUFFER_MEM_STATE_POPPED;
            *pp_packet             = p_packet;
            break;
        case PACKET_BUFFER_MEM_STATE_POPPED:
            NRF_MESH_ASSERT(false);
            break;
        default:
            status = NRF_ERROR_NOT_FOUND;
            break;
    }

    _ENABLE_IRQS(was_masked);
    return status;
}

bool packet_buffer_can_pop(packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(NULL != p_buffer);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    if (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_packet = m_get_next_packet(p_buffer, p_packet);
    }
    bool can_pop = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);

    _ENABLE_IRQS(was_masked);
    return can_pop;
}

bool packet_buffer_packets_ready_to_pop(packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(NULL != p_buffer);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    /* get first non-popped packet */
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    while (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED ||
           p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_packet = m_get_next_packet(p_buffer, p_packet);
    }
    bool ready_to_pop = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);

    _ENABLE_IRQS(was_masked);
    return ready_to_pop;
}

void packet_buffer_free(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != p_packet);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    switch (p_packet->packet_state)
    {
        case PACKET_BUFFER_MEM_STATE_POPPED:
            m_free_popped_packet(p_buffer, p_packet);
            break;
        case PACKET_BUFFER_MEM_STATE_RESERVED:
            m_free_reserved_packet(p_buffer, p_packet);
            break;
        default:
            /* Only POPPED packets and RESERVED packets can be freed. */
            NRF_MESH_ASSERT(false);
    }
#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif

    _ENABLE_IRQS(was_masked);
}

bool packet_buffer_is_empty(const packet_buffer_t * p_buffer)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    if (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_packet = m_get_next_packet(p_buffer, p_packet);
    }

    bool is_empty = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_FREE);

    _ENABLE_IRQS(was_masked);
    return is_empty;
}
