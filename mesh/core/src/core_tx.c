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
#include "core_tx.h"
#include "nrf_mesh_assert.h"
#include "utils.h"
#include "nordic_common.h"

/** Number of supported concurrent bearers. Should match the core_tx_bearer_t structure. */
#define BEARER_COUNT (1)

/*****************************************************************************
* Static globals
*****************************************************************************/

static core_tx_complete_cb_t m_tx_complete_callback;

static const core_tx_bearer_interface_t * m_bearers[BEARER_COUNT];
/*****************************************************************************
* Static functions
*****************************************************************************/
static inline const core_tx_bearer_interface_t * bearer_if_get(core_tx_bearer_t bearer)
{
    /** Can only get a single bearer at a time. */
    NRF_MESH_ASSERT(is_power_of_two(bearer));
    uint32_t index = log2_get((uint32_t) bearer);
    NRF_MESH_ASSERT(index < BEARER_COUNT);
    return m_bearers[index];
}
/*****************************************************************************
* Interface functions
*****************************************************************************/

void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback)
{
    m_tx_complete_callback = tx_complete_callback;
}

core_tx_bearer_t core_tx_packet_alloc(uint32_t net_packet_len,
                                      const core_tx_metadata_t * p_metadata,
                                      uint8_t ** pp_packet,
                                      nrf_mesh_tx_token_t token)
{
    NRF_MESH_ASSERT(p_metadata != NULL);
    NRF_MESH_ASSERT(pp_packet != NULL);
    NRF_MESH_ASSERT((uint8_t) p_metadata->bearer < (1 << BEARER_COUNT));
    NRF_MESH_ASSERT(is_power_of_two(p_metadata->bearer));
    NRF_MESH_ASSERT(p_metadata->role < CORE_TX_ROLE_COUNT);
    core_tx_bearer_t bearers = (core_tx_bearer_t) 0;

    *pp_packet = bearer_if_get(p_metadata->bearer)->packet_alloc(net_packet_len, p_metadata, token);

    if (*pp_packet != NULL)
    {
        bearers = p_metadata->bearer;
    }
    return bearers;
}

void core_tx_packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    NRF_MESH_ASSERT(p_metadata != NULL);
    NRF_MESH_ASSERT(p_packet != NULL);
    NRF_MESH_ASSERT((uint8_t) p_metadata->bearer < (1 << BEARER_COUNT));
    NRF_MESH_ASSERT(is_power_of_two(p_metadata->bearer));
    NRF_MESH_ASSERT(p_metadata->role < CORE_TX_ROLE_COUNT);

    bearer_if_get(p_metadata->bearer)->packet_send(p_metadata, p_packet);
}

void core_tx_packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    NRF_MESH_ASSERT(p_metadata != NULL);
    NRF_MESH_ASSERT(p_packet != NULL);
    NRF_MESH_ASSERT((uint8_t) p_metadata->bearer < (1 << BEARER_COUNT));
    NRF_MESH_ASSERT(is_power_of_two(p_metadata->bearer));
    NRF_MESH_ASSERT(p_metadata->role < CORE_TX_ROLE_COUNT);

    bearer_if_get(p_metadata->bearer)->packet_discard(p_metadata, p_packet);
}

void core_tx_complete(const core_tx_metadata_t * p_metadata,
                      uint32_t timestamp,
                      nrf_mesh_tx_token_t token)
{
    if (m_tx_complete_callback)
    {
        m_tx_complete_callback(p_metadata, timestamp, token);
    }
}

void core_tx_bearer_register(core_tx_bearer_t bearer, const core_tx_bearer_interface_t * p_interface)
{
    NRF_MESH_ASSERT(bearer_if_get(bearer) == NULL);
    NRF_MESH_ASSERT(p_interface != NULL);
    NRF_MESH_ASSERT(p_interface->packet_alloc != NULL);
    NRF_MESH_ASSERT(p_interface->packet_send != NULL);
    NRF_MESH_ASSERT(p_interface->packet_discard != NULL);
    m_bearers[log2_get((uint32_t) bearer)] = p_interface;
}
