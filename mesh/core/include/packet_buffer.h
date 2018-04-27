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
#ifndef PACKET_BUFFER_H__
#define PACKET_BUFFER_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "nrf_mesh.h"
#include "nrf_mesh_config_core.h"
#include "utils.h"
/**
 * @defgroup PACKET_BUFFER Packet Buffer
 * @ingroup MESH_CORE
 * Provides functionality for managing packet buffers.
 *
 * # Packet manager design
 * The packet manager provides functions for initializing packet manager instances and allocating
 * and deallocating packet buffers.
 *
 * The packet manager design is based on three critical assumptions:
 *
 * - One consumer and multiple producers.
 * - The packets are consumed in the same order they are produced.
 * - The consumer frees a popped packet before popping the next.
 *
 * There are 5 states a packet buffer may be in:
 *
 * - `FREE` : Available to producers to acquire and use.
 * - `RESERVED`: Acquired by and in use by the producer. This state is entered by the
 *               packet_buffer_reserve() function.
 * - `COMMITED`: Available to the consumer to acquire and read. This state is entered by the
 *               packet_buffer_commit() function.
 * - `POPPED`: Acquired by and in use of the consumer. This state is entered by calling the
 *             packet_buffer_pop() function. The consumer has to make a packet_buffer_free()
 *             call on this packet before it can acquire another packet.
 * - `SKIPPED`: A packet that was dropped by the producer (not commited). It has no meaningful
 *              information but have to be freed by a call to packet_buffer_pop() made by the consumer.
 *              There are two scenarios that can cause a packet to be in this state:
 *   1. The available space before the buffer wrap-around is not sufficient to satisfy a reserve
 *      call, but the space starting from buffer 0 and up is sufficient. In this case the buffer
 *      is skipped, as illustrated by the image below.
 *   2. The producer decides to free a buffer after reserving it, but there have already been
 *      other reservations of packets made that follow the freed buffer. In this case, the freed
 *      buffer cannot be treated as a free buffer in order to avoid holes in the packet manager.
 *   
 * ![Producer/Consumer](packet_mgr_producer_consumer.svg)
 *
 * # Initializing
 *
 * Due to the limitation that only one consumer can use a packet buffer at a time, the problem
 * of having multiple consumers (such as when using multiple advertisers) is solved by using multiple packet
 * buffer instances, where the buffers are managed independently. Each packet buffer instance is
 * created by a call to packet_buffer_init(), which accepts a pool of memory, the size of the pool
 * and creates an initialized packet manager instance.
 *
 * @code{.c}
 * static uint8_t buffer[128];
 * packet_buffer_t packet_buffer;
 * uint32_t status = packet_buffer_init(&packet_buffer, buffer, sizeof(buffer));
 * @endcode
 *
 * ![Initial packet buffer state](packet_mgr_initial_state.svg)
 *
 * # Reserving packets
 *
 * Packet buffers are reserved using the packet_buffer_reserve() function.
 *
 * @code{.c}
 * // Reserve a 10-byte packet
 * packet_buffer_packet_t * p_packet;
 * uint32_t status = packet_buffer_reserve(&packet_buffer, &p_packet, 10);
 * @endcode
 *
 * ![Packet buffer state after reserving a buffer](packet_mgr_reserve_1.svg)
 *
 * After allocating a second packet, the packet buffer contains two reserved buffers:
 *
 * ![Packet buffer state after reserving two buffers](packet_mgr_reserve_2.svg)
 *
 * # Committing packets
 *
 * Packet buffers are committed using the packet_mgr_commit() function.
 *
 * @code{.c}
 * // Commit the first reserved packet:
 * packet_buffer_commit(&packet_buffer, p_packet, 10);
 * @endcode
 *
 * ![Packet buffer state after committing the first reserved buffer](packet_mgr_commit.svg)
 *
 * # Popping packets
 *
 * Packet buffers are popped using the packet_buffer_pop() function.
 *
 * @code{.c}
 * // Pop the first packet:
 * packet_mgr_packet_t * p_popped_packet;
 * uint32_t status = packet_buffer_pop(&packet_buffer, &p_popped_packet);
 * @endcode
 *
 * ![Packet buffer state after popping the first packet buffer](packet_mgr_pop.svg)
 *
 * # Freeing packets
 *
 * Freeing packet buffers is done using the packet_buffer_free() function.
 *
 * @code{.c}
 * // Free the popped packet buffer:
 * uint32_t status = packet_mgr_free(&packet_buffer, &p_popped_packet);
 * @endcode
 *
 * ![Packet buffer state after freeing the previously popped buffer](packet_mgr_free_1.svg)
 *
 * After freeing the remaining reserved buffer, the state of the packet buffer becomes:
 *
 * ![Packet buffer state after freeing all reserved buffers](packet_mgr_free_2.svg)
 *
 * @{
 */

/**
 * Packet buffer memory states.
 */
typedef enum
{
    PACKET_BUFFER_MEM_STATE_FREE,      /**< The packet is available for reserving. */
    PACKET_BUFFER_MEM_STATE_RESERVED,  /**< The packet is reserved for allocation. */
    PACKET_BUFFER_MEM_STATE_COMMITTED, /**< The packet is committed to the buffer, and may be popped. */
    PACKET_BUFFER_MEM_STATE_POPPED,    /**< The packet has been popped from the buffer, but not yet freed. */
    PACKET_BUFFER_MEM_STATE_SKIPPED,   /**< The packet is not used but there are proceeding packets in use. */
} packet_buffer_mem_state_t;

/**
 * Structure of the header for each allocated packet buffer.
 */
typedef struct
{
#if PACKET_BUFFER_DEBUG_MODE
    uint32_t seal;        /**< Packet seal, used for detecting memory corruptions. */
    uint32_t last_caller; /**< Address of last caller that modified refcount on the packet. */
#endif
    uint16_t size;                       /**< Size of the packet in bytes */
    packet_buffer_mem_state_t packet_state; /**< State the given packet is in. */
    uint8_t packet[] __attribute((aligned(WORD_SIZE))); /**< The packet data buffer of length @ref size. */
} packet_buffer_packet_t;


/**
 * Packet buffer structure used for managing a pool of memory for packet allocations.
 *
 * @note Although an instance of this structure is owned by the user of the packet buffer,
 * it's contents can only be modified by the packet buffer functions, and must not
 * be touched by the user.
 */
typedef struct
{
#if PACKET_BUFFER_DEBUG_MODE
    uint32_t pop_lock;      /**< Variable used with DEBUG_ATOMIC_FUNCTION_ENTER/EXIT macros. */
    uint32_t free_lock;     /**< Variable used with DEBUG_ATOMIC_FUNCTION_ENTER/EXIT macros. */
#endif
    uint16_t size;      /**< Pool size */
    uint16_t head;      /**< Header index for tracking available memory */
    uint16_t tail;      /**< Tail index for tracking used memory */
    uint8_t * buffer;   /**< Pool of memory */
} packet_buffer_t;



/**
 * Returns the maximum possible packet size that can be reserved with the given packet buffer.
 *
 * @warning    This function requires that:
 *               - Pointer supplied is not NULL
 *               - The packet buffer is initialized.
 *
 * @param[in]  p_buffer  Pointer to the packet buffer instance.
 *
 * @return     Max possible packet length.
 */

uint16_t packet_buffer_max_packet_len_get(const packet_buffer_t * const p_buffer);

/**
 * Initializes a packet buffer in the specified memory pool.
 *
 * @warning This function requires that:
 *               - p_buffer is a reference to an valid packet_buffer_t instance.
 *               - p_pool is not NULL and a valid pointer in the DATA or CODE ram space
 *               - pool_size is larger than the minimum over-head of sizeof(packet_buffer_packet_t)
 *
 * @param[in, out] p_buffer Reference to a @ref packet_buffer_t instance, it will be initialized
 *                          with the given memory pool.
 * @param[in] p_pool Pointer to the start of the available memory pool.
 * @param[in] pool_size Size (in bytes) of the memory pool.
 */
void packet_buffer_init(packet_buffer_t * p_buffer, void * const p_pool, const uint16_t pool_size);

/**
 * Flushes a packet buffer, dropping all committed packets.
 *
 * @param[in,out] p_buffer Buffer to flush.
 */
void packet_buffer_flush(packet_buffer_t * p_buffer);

/**
 * @defgroup PRODUCER Packet producer functions.
 * @{
 */

/**
 * Reserves a packet in the given packet buffer.
 *
 * @param[in, out] p_buffer A packet buffer instance to reserve a packet on.
 * @param[out] pp_packet Reference to the reserved packet.
 * @param[in] length Number of bytes of payload to reserve, not including the
 * static header fields of the packet.
 *
 * @warning This function requires that:
 *               - Pointers supplied are not NULL
 *
 * @retval NRF_SUCCESS The packet is reserved successfully, and pp_packet points to
 * a valid packet pointer instance.
 * @retval NRF_ERROR_NO_MEM The packet buffer does not have enough available memory.
 * @retval NRF_ERROR_INVALID_LENGTH The length of the packet requested cannot be 0 or greater
 * than the maximum available packet size in the given packet buffer. Use @ref
 * packet_buffer_max_packet_len_get to determine the max packet length possible.
 */
uint32_t packet_buffer_reserve(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet, uint16_t length);

/**
 * Commits a reserved packet to the given packet buffer.
 *
 * @warning This function requires that:
 *               - Pointers supplied are not NULL
 *               - The given packet (p_packet) is a reserved packet
 *               - length is larger than 0
 *               - length is less than or equal to the reserved size (p_packet->size)
 *
 * @param[in, out] p_buffer A packet buffer instance to commit a packet to.
 * @param[in, out] p_packet Pointer to the packet buffer to commit.
 * @param[in] length The new length of the given packet. Must be equal to or
 *                   smaller than the size of the reserved memory.
 */
void packet_buffer_commit(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet, uint16_t length);

/** @} */

/**
 * @defgroup CONSUMER Packet consumer functions.
 * @{
 */

/**
 * Pops a packet from the given packet buffer instance.
 *
 * The packet must be freed before the next packet may be popped.
 *
 * @warning This function is not thread safe.
 *
 * @warning This function requires that:
 *               - Pointers supplied are not NULL
 *               - There is no other packet in popped state
 *
 * @param[in, out] p_buffer A packet buffer instance to pop a packet from.
 * @param[out] pp_packet The popped packet pointer.
 *
 * @retval NRF_SUCCESS A committed packet was found and popped successfully.
 * pp_packet points to a valid packet pointer instance.
 * @retval NRF_ERROR_NOT_FOUND The packet buffer has no packets in @ref
 * PACKET_BUFFER_MEM_STATE_COMMITTED state.
 */
 uint32_t packet_buffer_pop(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet);

/**
 * Checks if a packet can be popped right away.
 *
 * @warning This function is not thread safe.
 *
 * @warning This function requires that:
 *               - Pointer supplied is not NULL.
 *
 * @param[in, out] p_buffer The packet buffer instance to check for poppable packets on.
 *
 * @retval true  A packet can be popped from this packet buffer.
 * @retval false Packet manager is not ready to allow a packet to be popped.
 */
bool packet_buffer_can_pop(packet_buffer_t * p_buffer);

/**
 * Checks if there are any packets ready to be popped.
 *
 * @warning        Even though there are packets ready to be popped, the packet manager might not be
 *                 in a right state to allow popping of a packet, use @ref packet_buffer_can_pop to
 *                 find out if a packet is available to be popped and if the packet manager is
 *                 ready.
 * @warning        This function is not thread safe.
 * @warning        This function requires that:
 *                  - Pointer supplied is not NULL.
 *
 * @param[in, out] p_buffer  The packet buffer instance to check for poppable packets on.
 *
 * @retval         true      One or more packets are available to be popped from this packet buffer.
 * @retval         false     No packets are available for popping in this packet buffer.
 */
bool packet_buffer_packets_ready_to_pop(packet_buffer_t * p_buffer);

/**
 * Frees the given packet to the buffer.
 *
 * The packet must be in either the @ref PACKET_BUFFER_MEM_STATE_POPPED state or be the
 * last packet of the buffer to be in the @ref PACKET_BUFFER_MEM_STATE_RESERVED state.
 *
 * @warning This function is not thread safe.
 *
 * @warning This function requires that:
 *               - Pointers supplied are not NULL
 *               - The supplied packet is either in Popped state or Reserved state
 *
 * @param[in, out] p_buffer The packet buffer instance the @p p_packet belongs to.
 * @param[in] p_packet The packet to free.
 *
 */
void packet_buffer_free(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet);

/** @} */

/** @} */

#endif

