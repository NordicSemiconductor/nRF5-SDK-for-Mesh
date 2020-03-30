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
#ifndef PACKET_MGR_H__
#define PACKET_MGR_H__

#include <stdint.h>

#include "packet.h"
#include "nrf_mesh.h"
#include "nrf_mesh_config_core.h"
#include "utils.h"
/**
 * @defgroup PACKET_MGR Packet Manager
 * @ingroup MESH_CORE
 * Manages packet buffers.
 * @warning This module is to be deprecated and replaced with @ref PACKET_BUFFER.
 * @{
 */

/** Maximum length of the packets that can be allocated. */
#define PACKET_MGR_PACKET_MAXLEN    (NRF_MESH_SEG_PAYLOAD_SIZE_MAX)
/** Default length of allocated packets. */
#define PACKET_MGR_DEFAULT_PACKET_LEN 40

/** Sets the packet manager alignment to the native alignment. */
#define PACKET_MGR_ALIGNMENT  (WORD_SIZE)


#if __linux__
#   if PACKET_MGR_BLAME_MODE > 0
#       warning "Packet manager blame mode is not supported on Linux."
#       undef PACKET_MGR_BLAME_MODE
#   endif
#   define PACKET_MGR_BLAME_MODE 0
#endif

/**
 * Initializes the packet manager.
 *
 * @param[in] p_init_params Pointer to the mesh initialization structure.
 */
void packet_mgr_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Allocates a new packet buffer.
 *
 * The size of the allocated buffer is always equal to or greater than the specified size.
 *
 * @param[out] pp_buffer Pointer to a variable where a pointer to the allocated memory
 *                       is stored.
 * @param[in]  size      Size of the requested packet buffer.
 *
 * @retval NRF_SUCCESS      Memory was successfully allocated.
 * @retval NRF_ERROR_NO_MEM No memory was available for allocation.
 */
uint32_t packet_mgr_alloc(packet_generic_t ** pp_buffer, uint16_t size);

/**
 * Frees a previously allocated packet buffer.
 *
 * @param[in] p_buffer Pointer to the start of the buffer.
 */
void packet_mgr_free(packet_generic_t * p_buffer);

/**
 * Gets the amount of free space left in the memory pool.
 * @return the amount of free space left in the memory pool.
 */
uint32_t packet_mgr_get_free_space(void);

/**
 * Gets the reference count for a packet buffer.
 *
 * @param[in] p_packet Pointer to a packet buffer.
 *
 * @return The current reference count of the buffer.
 */
uint8_t packet_mgr_refcount_get(packet_generic_t * p_packet);

/**
 * Gets the size of a given packet buffer.
 *
 * @param[in] p_packet Pointer to a packet buffer.
 *
 * @return The size of the buffer.
 */
uint16_t packet_mgr_size_get(packet_generic_t * p_packet);

/** @} */

#endif

