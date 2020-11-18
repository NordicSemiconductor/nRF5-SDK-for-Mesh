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
#include "core_tx.h"
#include "nrf_mesh_assert.h"
#include "utils.h"
#include "nordic_common.h"
#include "list.h"
#include "log.h"

NRF_MESH_STATIC_ASSERT(sizeof(core_tx_bearer_bitmap_t) * 8 >= CORE_TX_BEARER_COUNT_MAX);
/*****************************************************************************
* Static globals
*****************************************************************************/

static core_tx_complete_cb_t m_tx_complete_callback;

/** Linked list of all registered bearers. */
static list_node_t * mp_bearers;

/** Staging-packet state */
struct
{
    uint8_t length;
    packet_mesh_net_packet_t buffer;
    core_tx_bearer_bitmap_t bearer_bitmap;
} m_packet;

/** Number of active bearers */
static uint8_t m_bearer_count;
/*****************************************************************************
* Static functions
*****************************************************************************/
static inline void alloc_result_log(core_tx_bearer_t * p_bearer, core_tx_alloc_result_t result)
{
#ifdef CORE_TX_DEBUG
    switch (result)
    {
        case CORE_TX_ALLOC_SUCCESS:
            p_bearer->debug.alloc_count++;
            break;
        case CORE_TX_ALLOC_FAIL_REJECTED:
            p_bearer->debug.reject_count++;
            break;
        case CORE_TX_ALLOC_FAIL_NO_MEM:
            p_bearer->debug.no_mem_count++;
            break;
        default:
            __LOG(LOG_SRC_NETWORK, LOG_LEVEL_WARN, "Unknown result %u\n", result);
            return;
    }
    static const char * p_result_names[] = {[CORE_TX_ALLOC_SUCCESS]             = "Success",
                                            [CORE_TX_ALLOC_FAIL_REJECTED]       = "Rejected",
                                            [CORE_TX_ALLOC_FAIL_NO_MEM]         = "No memory"};
    __LOG(LOG_SRC_NETWORK, LOG_LEVEL_INFO, "Bearer 0x%p alloc: %s\n", p_bearer, p_result_names[result]);
#endif
}
/*****************************************************************************
* Interface functions
*****************************************************************************/

void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback)
{
    m_tx_complete_callback = tx_complete_callback;
}

core_tx_bearer_bitmap_t core_tx_packet_alloc(const core_tx_alloc_params_t * p_params, uint8_t ** pp_packet)
{
    NRF_MESH_ASSERT(p_params != NULL);
    NRF_MESH_ASSERT(pp_packet != NULL);
    NRF_MESH_ASSERT(p_params->role < CORE_TX_ROLE_COUNT);
    NRF_MESH_ASSERT(m_packet.bearer_bitmap == 0);
    NRF_MESH_ASSERT(p_params->net_packet_len <= sizeof(m_packet.buffer));

#if MESH_FEATURE_RELAY_ENABLED || MESH_FEATURE_FRIEND_ENABLED
    if (p_params->role == CORE_TX_ROLE_RELAY)
    {
        NRF_MESH_ASSERT(p_params->token == NRF_MESH_RELAY_TOKEN ||
                        IS_IN_RANGE(p_params->token,
                                    NRF_MESH_FRIEND_TOKEN_BEGIN,
                                    NRF_MESH_FRIEND_TOKEN_BEGIN + MESH_FRIEND_FRIENDSHIP_COUNT - 1));
    }
#endif

    LIST_FOREACH(p_iterator, mp_bearers)
    {
        core_tx_bearer_t * p_bearer = PARENT_BY_FIELD_GET(core_tx_bearer_t, list_node, p_iterator);

        if ((p_params->bearer_selector & p_bearer->type) > 0)
        {
            core_tx_alloc_result_t result = p_bearer->p_interface->packet_alloc(p_bearer, p_params);

            alloc_result_log(p_bearer, result);

            if (result == CORE_TX_ALLOC_SUCCESS)
            {
                m_packet.bearer_bitmap |= (1ULL << p_bearer->bearer_index);
            }
        }
    }

    if (m_packet.bearer_bitmap != 0)
    {
        m_packet.length = p_params->net_packet_len;
        *pp_packet = m_packet.buffer.pdu;
    }
    return m_packet.bearer_bitmap;
}

void core_tx_packet_send(void)
{
    NRF_MESH_ASSERT(m_packet.bearer_bitmap != 0);

    LIST_FOREACH(p_iterator, mp_bearers)
    {
        core_tx_bearer_t * p_bearer = PARENT_BY_FIELD_GET(core_tx_bearer_t, list_node, p_iterator);

        if ((m_packet.bearer_bitmap & (1ULL << p_bearer->bearer_index)) > 0)
        {
            p_bearer->p_interface->packet_send(p_bearer, m_packet.buffer.pdu, m_packet.length);
        }
    }
    m_packet.bearer_bitmap = 0;
}

void core_tx_packet_discard(void)
{
    NRF_MESH_ASSERT(m_packet.bearer_bitmap != 0);

    LIST_FOREACH(p_iterator, mp_bearers)
    {
        core_tx_bearer_t * p_bearer = PARENT_BY_FIELD_GET(core_tx_bearer_t, list_node, p_iterator);

        if ((m_packet.bearer_bitmap & (1ULL << p_bearer->bearer_index)) > 0)
        {
            p_bearer->p_interface->packet_discard(p_bearer);
        }
    }
    m_packet.bearer_bitmap = 0;
}

core_tx_bearer_type_t core_tx_bearer_type_get(uint32_t bearer_index)
{
    LIST_FOREACH(p_iterator, mp_bearers)
    {
        core_tx_bearer_t * p_bearer = PARENT_BY_FIELD_GET(core_tx_bearer_t, list_node, p_iterator);

        if (bearer_index == p_bearer->bearer_index)
        {
            return p_bearer->type;
        }
    }
    return CORE_TX_BEARER_TYPE_INVALID;
}


uint32_t core_tx_bearer_count_get(void)
{
    return m_bearer_count;
}

void core_tx_complete(core_tx_bearer_t * p_bearer,
                      core_tx_role_t role,
                      uint32_t timestamp,
                      nrf_mesh_tx_token_t token)
{
    NRF_MESH_ASSERT(p_bearer);

    if (m_tx_complete_callback)
    {
        m_tx_complete_callback(role, p_bearer->bearer_index, timestamp, token);
    }
}

void core_tx_bearer_add(core_tx_bearer_t * p_bearer,
                        const core_tx_bearer_interface_t * p_interface,
                        core_tx_bearer_type_t type)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    NRF_MESH_ASSERT(p_interface != NULL);
    NRF_MESH_ASSERT(type != CORE_TX_BEARER_TYPE_INVALID);
    NRF_MESH_ASSERT(type != CORE_TX_BEARER_TYPE_ALLOW_ALL);
    NRF_MESH_ASSERT(p_interface->packet_alloc != NULL);
    NRF_MESH_ASSERT(p_interface->packet_send != NULL);
    NRF_MESH_ASSERT(p_interface->packet_discard != NULL);
    NRF_MESH_ASSERT(m_bearer_count < CORE_TX_BEARER_COUNT_MAX);

    p_bearer->p_interface  = p_interface;
    p_bearer->bearer_index = m_bearer_count++;
    p_bearer->type         = type;

    list_add(&mp_bearers, &p_bearer->list_node);
}

#ifdef UNIT_TEST
void core_tx_reset(void)
{
    mp_bearers = NULL;
    m_bearer_count = 0;
    m_packet.bearer_bitmap = 0;
}
#endif
