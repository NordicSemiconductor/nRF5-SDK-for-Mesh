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
#include <stdbool.h>
#include <string.h>

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_core.h"
#include "packet_buffer.h"
#include "nrf_error.h"
#include "utils.h"
#include "toolchain.h"

#if PACKET_BUFFER_DEBUG_MODE
/* Seal is used as a buffer between memory blocks for overflow detection. */
#define PACKET_BUFFER_MEM_SEAL 0x5EA15EA1
#endif

/*******************************                  *******************************
******************************** Local functions ********************************
********************************                  *******************************/
static void m_index_increment(uint16_t * index, uint16_t arr_size, uint16_t length)
{
    *index += length;
    if (*index == arr_size)
    {
        *index = 0;
    }
    NRF_MESH_ASSERT(arr_size > *index);
}

static void m_free_popped_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
#if PACKET_BUFFER_DEBUG_MODE
        DEBUG_ATOMIC_FUNCTION_ENTER(p_buffer->free_lock);
#endif
    NRF_MESH_ASSERT((uint8_t*)p_packet == &p_buffer->buffer[p_buffer->tail]);
    m_index_increment(&p_buffer->tail, p_buffer->size, p_packet->size + sizeof(packet_buffer_packet_t));
    NRF_MESH_ASSERT(sizeof(packet_buffer_packet_t) < (uint16_t) (p_buffer->size - p_buffer->tail) );
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_EXIT(p_buffer->free_lock);
#endif
}

static inline uint16_t m_get_packet_buffer_index(const packet_buffer_t * p_buffer, const packet_buffer_packet_t * p_packet)
{
    return (uint16_t)((const uint8_t *)p_packet - p_buffer->buffer);
}


static inline packet_buffer_packet_t * m_get_packet(const packet_buffer_t * p_buffer, uint16_t index)
{
    return (packet_buffer_packet_t *) &p_buffer->buffer[index];
}

static uint8_t * m_get_next_packet(const packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    uint8_t* proceeding_packet_ref = &p_packet->packet[p_packet->size];
    /* Packet proceeding this may have wrapped around */
    if (proceeding_packet_ref == &p_buffer->buffer[p_buffer->size])
    {
        proceeding_packet_ref = p_buffer->buffer;
    }

    return proceeding_packet_ref;
}

static void m_free_reserved_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    /* Check if there are any other packets reserved after the given block*/
    uint8_t* proceeding_packet_ref = m_get_next_packet(p_buffer, p_packet);
    /* Locking IRQs before accesing head :(*/
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (proceeding_packet_ref == &p_buffer->buffer[p_buffer->head])
    {
        /* This was the last packet reserved: now it's the new head */
        p_buffer->head = m_get_packet_buffer_index(p_buffer, p_packet);
         _ENABLE_IRQS(was_masked);
        p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }
    else
    {
        _ENABLE_IRQS(was_masked);
        /* Creating a hole in the packet buffer */
        p_packet->packet_state = PACKET_BUFFER_MEM_STATE_SKIPPED;
    }
}

static packet_buffer_mem_state_t m_get_packet_state(packet_buffer_t * p_buffer, uint16_t index)
{
    packet_buffer_packet_t * packet = m_get_packet(p_buffer, index);
    return packet->packet_state;
}

static uint16_t m_max_packet_len_get(const packet_buffer_t *  const p_buffer)
{
    return (p_buffer->size - sizeof(packet_buffer_packet_t));
}

static void m_adjust_reserved_packet_len(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet, uint16_t length)
{
    if (p_packet->size > length)
    {
        /* The packet size needs to be shrunk: */
        uint16_t reduction_in_packet = p_packet->size - length;
        if (0 == p_buffer->head)
        {   /* Reverse wrap-around */
            p_buffer->head = p_buffer->size - reduction_in_packet;
        }
        else
        {
            p_buffer->head -= reduction_in_packet;
        }
        p_packet->size = length;
        /* Mark the current head as available */
        packet_buffer_packet_t * p_next_packet =   m_get_packet(p_buffer, p_buffer->head);
        p_next_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
#if PACKET_BUFFER_DEBUG_MODE
        p_next_packet->seal = PACKET_BUFFER_MEM_SEAL;
#endif
    }
    else
    {
        /* An already reserved packet can only be shrunk not enlarged */
        NRF_MESH_ASSERT(p_packet->size == length);
    }
}

static packet_buffer_packet_t * m_reserve_packet(packet_buffer_t * p_buffer, uint16_t length)
{
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->head);
    p_packet->size = length;
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_RESERVED;
    /* Move the head to the next available slot and mark it so. */
    m_index_increment(&p_buffer->head, p_buffer->size, length + sizeof(packet_buffer_packet_t));

    if (p_buffer->head != p_buffer->tail)
    {
        /* Mark the current head as available */
        packet_buffer_packet_t * p_next_packet =   m_get_packet(p_buffer, p_buffer->head);
        p_next_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }

#if PACKET_BUFFER_DEBUG_MODE
    NRF_MESH_ASSERT(PACKET_BUFFER_MEM_SEAL == p_packet->seal);
    _GET_LR(p_packet->last_caller);
    packet_buffer_packet_t * p_next_packet =   m_get_packet(p_buffer, p_buffer->head);
    uint32_t seal_value = PACKET_BUFFER_MEM_SEAL;
    p_next_packet->seal = seal_value;
#endif
    return p_packet;
}

/* Can only be called if there is sufficient space for packet_length + header
 * between packet_start and available_space_end
 */
static void m_adjust_desired_packet_length(uint16_t packet_start, uint16_t available_space_end, uint16_t* p_packet_length)
{
    /* If the buffer will not have enough space for another packet to be initialized,
     * then use all available memory for the current allocation.
     *
     * So instead of:
     * start|....|packet_length|Not enough space for another packet|end ->
     * do this:
     * start|....|packet_length|end
     */
    uint16_t available_space  = available_space_end - packet_start;
    uint16_t packet_size_with_header = *p_packet_length + sizeof(packet_buffer_packet_t);
    NRF_MESH_ASSERT(packet_size_with_header <=  available_space);

    if ( (uint16_t) (available_space - packet_size_with_header) <= sizeof(packet_buffer_packet_t) )
    {
        /* Fill out the rest of the available space. */
        *p_packet_length = available_space - sizeof(packet_buffer_packet_t);
    }
}

static uint32_t m_wrap_head_to_start(packet_buffer_t * p_buffer, uint16_t length)
{
    if (p_buffer->tail >= length)
    {
        /* Need to wrap around */
        /* Creating a hole at the end of the packet buffer */
        packet_buffer_packet_t * p_packet_skipped = m_get_packet(p_buffer, p_buffer->head);
        p_packet_skipped->size = p_buffer->size - p_buffer->head - sizeof(packet_buffer_packet_t);
        p_packet_skipped->packet_state = PACKET_BUFFER_MEM_STATE_SKIPPED;
#if PACKET_BUFFER_DEBUG_MODE
        _GET_LR(p_packet_skipped->last_caller);
#endif
        p_buffer->head = 0;
        return NRF_SUCCESS;
    }
    else
    {
        /* Not enough memory. */
        return NRF_ERROR_NO_MEM;
    }
}

static uint32_t m_head_is_tail_prepare_reserve(packet_buffer_t * p_buffer, uint16_t length)
{
    uint32_t status = NRF_SUCCESS;
    if (m_get_packet_state(p_buffer, p_buffer->tail) == PACKET_BUFFER_MEM_STATE_FREE)
    {
        if ((p_buffer->size - p_buffer->head) < length)
        {
            status = m_wrap_head_to_start(p_buffer, length);
        }
    }
    else
    {
        /* Out of memory. */
        status = NRF_ERROR_NO_MEM;
    }
    return status;
}

static uint32_t m_head_ahead_prepare_reserve(packet_buffer_t * p_buffer, uint16_t length)
{
    uint32_t status = NRF_SUCCESS;
    if ((p_buffer->size - p_buffer->head) < length)
    {
        status = m_wrap_head_to_start(p_buffer, length);
    }
    return status;
}


static uint32_t m_tail_ahead_prepare_reserve(packet_buffer_t * p_buffer, uint16_t length)
{
    uint32_t status = NRF_SUCCESS;
    if ((p_buffer->tail - p_buffer->head) < length)
    {
        /* Not enough memory. */
        status = NRF_ERROR_NO_MEM;
    }
    return status;
}

/* Checks if there is sufficient space in the packet buffer for the given packet length,
 * moves the packet_buffer head and tail indexes as necessary, and adjusts the p_length
 * for optimal buffer use .*/
static uint32_t m_prepare_for_reserve(packet_buffer_t * p_buffer, uint16_t* p_length)
{
    uint32_t status;
    uint16_t packet_len_with_header = *p_length + sizeof(packet_buffer_packet_t);

    /* Check the amount of mem available depending on the head/tail locations */
    if (p_buffer->head == p_buffer->tail)
    {
       status = m_head_is_tail_prepare_reserve(p_buffer, packet_len_with_header);
    }
    else if (p_buffer->head > p_buffer->tail)
    {/* ...|tail|....|head|...|end|*/
        status = m_head_ahead_prepare_reserve(p_buffer, packet_len_with_header);
    }
    else
    {/* head has wrapped around, i.e: ...|head|....|tail|...*/
        status = m_tail_ahead_prepare_reserve(p_buffer, packet_len_with_header);
    }


    if (status == NRF_SUCCESS)
    {
        uint16_t available_space_end;
        if (p_buffer->head < p_buffer->tail)
        {
            available_space_end = p_buffer->tail;
        }
        else
        {
            available_space_end = p_buffer->size;
        }
        m_adjust_desired_packet_length(p_buffer->head, available_space_end, p_length);
    }
    return status;
}

static packet_buffer_packet_t * free_skipped_packets(packet_buffer_t * p_buffer)
{
    packet_buffer_packet_t * p_next_packet = m_get_packet(p_buffer, p_buffer->tail);
    while (PACKET_BUFFER_MEM_STATE_SKIPPED == p_next_packet->packet_state)
    {
#if PACKET_BUFFER_DEBUG_MODE
        NRF_MESH_ASSERT(PACKET_BUFFER_MEM_SEAL == p_next_packet->seal);
#endif
        m_free_popped_packet(p_buffer, p_next_packet);
        p_next_packet = m_get_packet(p_buffer, p_buffer->tail);
    }
    return p_next_packet;
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

#if PACKET_BUFFER_DEBUG_MODE
    p_buffer->pop_lock = 0xFFFFFFFF;
    p_buffer->free_lock = 0xFFFFFFFF;
    p_first_packet->seal = PACKET_BUFFER_MEM_SEAL;
#endif
}

void packet_buffer_flush(packet_buffer_t * p_buffer)
{
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    /* We're altering the popping here, and risk asserting if someone comes in an pops a packet
     * while we're flushing. :( */
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    bool has_popped_packet = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED);

    if (has_popped_packet)
    {
        /* Skip the popped packet */
        p_packet = (packet_buffer_packet_t *) m_get_next_packet(p_buffer, p_packet);
    }

    /* Mark all committed packets skipped. */
    while (p_packet->packet_state != PACKET_BUFFER_MEM_STATE_RESERVED &&
           m_get_packet_buffer_index(p_buffer, p_packet) != p_buffer->head)
    {
        p_packet->packet_state = PACKET_BUFFER_MEM_STATE_SKIPPED;
        p_packet = (packet_buffer_packet_t *) m_get_next_packet(p_buffer, p_packet);
    }

    if (!has_popped_packet)
    {
        /* Have to skip the packets now, as no one is coming after us to do it. */
        (void) free_skipped_packets(p_buffer);
    }
    _ENABLE_IRQS(was_masked);
}

uint32_t packet_buffer_reserve(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet, uint16_t length)
{
#if PACKET_BUFFER_DEBUG_MODE
    /* TODO: Uncomment the line below when the IRQ locking is removed. */
    //DEBUG_ATOMIC_FUNCTION_ENTER();
#endif
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != pp_packet);

    uint32_t status = NRF_SUCCESS;
    if (length == 0 || length > m_max_packet_len_get(p_buffer))
    {
        status = NRF_ERROR_INVALID_LENGTH;
    }
    else
    {
        /* Word-align the length */
        length = ALIGN_VAL(length, WORD_SIZE);
        /* The two calls below must not be interrupted by another reserve or commit call*/
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        /* Check if the packet buffer has enough space for the requested packet. */
        status = m_prepare_for_reserve(p_buffer, &length);
        if (NRF_SUCCESS == status)
        {
            *pp_packet = m_reserve_packet(p_buffer, length);
        }
        _ENABLE_IRQS(was_masked);
    }
#if PACKET_BUFFER_DEBUG_MODE
    /* TODO: Uncomment the line below when the IRQ locking is removed. */
    //DEBUG_ATOMIC_FUNCTION_EXIT();
#endif
    return status;
}

void packet_buffer_commit(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet, uint16_t length)
{
#if PACKET_BUFFER_DEBUG_MODE
    /* TODO: Uncomment the line below when the IRQ locking is removed. */
    //DEBUG_ATOMIC_FUNCTION_ENTER();
#endif
    /* A fair list of requirements:*/
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != p_packet);
    NRF_MESH_ASSERT(PACKET_BUFFER_MEM_STATE_RESERVED == p_packet->packet_state);
    NRF_MESH_ASSERT(p_packet->size >= length);
    NRF_MESH_ASSERT(0 < length);

    /* Word-align the length */
    length = ALIGN_VAL(length, WORD_SIZE);

    /* Check if there are any other packets reserved after the given block*/
    if (length != p_packet->size)
    {
        uint8_t* proceeding_packet_ref = m_get_next_packet(p_buffer, p_packet);
        /* Locking IRQs before accesing head :(*/
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        if (proceeding_packet_ref == &p_buffer->buffer[p_buffer->head])
        {
            /* This is the last reserved packet...*/
            uint16_t head_before_reserve = m_get_packet_buffer_index(p_buffer, p_packet);
            uint16_t available_space_end = head_before_reserve < p_buffer->tail ? p_buffer->tail : p_buffer->size;
            m_adjust_desired_packet_length(head_before_reserve, available_space_end, &length);
            m_adjust_reserved_packet_len(p_buffer, p_packet, length);
        }
        _ENABLE_IRQS(was_masked);
    }
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_COMMITTED;

#if PACKET_BUFFER_DEBUG_MODE
    NRF_MESH_ASSERT(PACKET_BUFFER_MEM_SEAL == p_packet->seal);
    _GET_LR(p_packet->last_caller);
    /* TODO: Uncomment the line below when the IRQ locking is removed. */
    //DEBUG_ATOMIC_FUNCTION_EXIT();
#endif
}

/* No IRQ locks required in this code since calling it twice is not
 * an acceptable scenario.
 */
uint32_t packet_buffer_pop(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != pp_packet);
#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_ENTER(p_buffer->pop_lock);
#endif

    uint32_t status = NRF_SUCCESS;
    packet_buffer_packet_t * p_next_packet = free_skipped_packets(p_buffer);

    switch (p_next_packet->packet_state)
    {
        case PACKET_BUFFER_MEM_STATE_COMMITTED:
            p_next_packet->packet_state = PACKET_BUFFER_MEM_STATE_POPPED;
            *pp_packet = p_next_packet;
            break;
        case PACKET_BUFFER_MEM_STATE_POPPED:
            NRF_MESH_ASSERT(false);
            break;
        default:
            status = NRF_ERROR_NOT_FOUND;
            break;
    }

#if PACKET_BUFFER_DEBUG_MODE
        NRF_MESH_ASSERT(PACKET_BUFFER_MEM_SEAL == p_next_packet->seal);
        _GET_LR(p_next_packet->last_caller);
        DEBUG_ATOMIC_FUNCTION_EXIT(p_buffer->pop_lock);
#endif

    return status;
}

bool packet_buffer_can_pop(packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_ENTER(p_buffer->pop_lock);
#endif

    packet_buffer_packet_t * p_next_packet = free_skipped_packets(p_buffer);

    bool retval = false;
    if (p_next_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED)
    {
        retval = true;
    }

#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_EXIT(p_buffer->pop_lock);
#endif

    return retval;
}

bool packet_buffer_packets_ready_to_pop(packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_ENTER(p_buffer->pop_lock);
#endif

    packet_buffer_packet_t * p_next_packet = free_skipped_packets(p_buffer);
    packet_buffer_packet_t * p_start_packet = p_next_packet;
    bool retval = (p_next_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);
    while (p_next_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED || PACKET_BUFFER_MEM_STATE_SKIPPED == p_next_packet->packet_state)
    {
        p_next_packet = (packet_buffer_packet_t *) m_get_next_packet(p_buffer, p_next_packet);
        retval = (p_next_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);
        if (p_next_packet == p_start_packet) /* Wrapped around */
        {
            break;
        }
    }

#if PACKET_BUFFER_DEBUG_MODE
    DEBUG_ATOMIC_FUNCTION_EXIT(p_buffer->pop_lock);
#endif

    return retval;
}

/* m_free_reserved_packet will disable IRQs since it accesses the HEAD.
 * No IRQ locks required in the rest of the code since calling this twice
 * is not an acceptable scenario.
 */
void packet_buffer_free(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet)
{
    NRF_MESH_ASSERT(NULL != p_buffer);
    NRF_MESH_ASSERT(NULL != p_packet);

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
        NRF_MESH_ASSERT(PACKET_BUFFER_MEM_SEAL == p_packet->seal);
        _GET_LR(p_packet->last_caller);
#endif
}
