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
#include <stdlib.h>
#include <string.h>

#include <nrf_error.h>

#include "packet_mgr.h"
#include "nrf_mesh_assert.h"

/*lint -e429 Custodial pointer has not been freed or returned */

/**
 * Structure of the header for each allocated packet buffer.
 */
typedef struct
{
    uint16_t size;               /**< Size of the block in bytes. */
    bool     in_use;             /**< Whether the block is in use. */
} buffer_header_t;

/********************
 * Static functions *
 ********************/

/**
 * Gets the buffer header from a packet buffer.
 * @param p_buffer Packet buffer.
 * @return Returns a pointer to the buffer header of the packet buffer.
 */
static inline buffer_header_t * buffer_header_get(packet_generic_t * p_buffer)
{
    return (buffer_header_t *) (((uint8_t *) p_buffer) - sizeof(buffer_header_t));
}

/**
 * Gets the start of the memory region of a buffer.
 * @param p_header Pointer to the buffer header.
 * @return Returns a pointer to the start of the memory for a buffer.
 */
static inline packet_generic_t * buffer_get_mem(buffer_header_t * p_header)
{
    return (packet_generic_t *) (((uint8_t *) p_header) + sizeof(buffer_header_t));
}

/******************************
 * Public interface functions *
 ******************************/

void packet_mgr_init(const nrf_mesh_init_params_t * p_init_params)
{
}

uint32_t packet_mgr_alloc(packet_generic_t ** pp_buffer, uint16_t size)
{
    buffer_header_t * current;
    uint32_t retval = NRF_SUCCESS;

    if (size > PACKET_MGR_PACKET_MAXLEN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    current = malloc(size + sizeof(buffer_header_t));
    if (current == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    current->size = size;
    current->in_use = true;
    *pp_buffer = buffer_get_mem(current);

    return retval;
}

void packet_mgr_free(packet_generic_t * p_buffer)
{
    buffer_header_t * header = buffer_header_get(p_buffer);

    NRF_MESH_ASSERT(header->in_use);

    header->in_use = false;
    free(header);
}

