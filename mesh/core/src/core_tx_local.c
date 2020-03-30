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
#include "core_tx_local.h"

#include <stdint.h>
#include <stdbool.h>

#include "core_tx.h"
#include "bearer_event.h"
#include "timer.h"
#include "mesh_mem.h"
#include "network.h"
#include "queue.h"

typedef struct
{
    queue_elem_t        node;
    core_tx_role_t      role;
    nrf_mesh_tx_token_t token;
    uint32_t            length;
    uint8_t             data[];
} loopback_data_item_t;

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params);
static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length);
static void packet_discard(core_tx_bearer_t * p_bearer);

/*****************************************************************************
* Static globals
*****************************************************************************/
static const core_tx_bearer_interface_t m_bearer_if =
{
    .packet_alloc   = packet_alloc,
    .packet_send    = packet_send,
    .packet_discard = packet_discard,
};

static core_tx_bearer_t m_bearer;
static queue_t m_loopback_data_queue;
static loopback_data_item_t * mp_item;
static bearer_event_flag_t m_loopback_flag;

/*****************************************************************************
* Static functions
*****************************************************************************/
static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);

    if (!(p_params->bearer_selector & CORE_TX_BEARER_TYPE_LOCAL) || (mp_item != NULL))
    {
        return CORE_TX_ALLOC_FAIL_REJECTED;
    }

    mp_item = mesh_mem_alloc(sizeof(loopback_data_item_t) + p_params->net_packet_len);

    if (mp_item == NULL)
    {
        return CORE_TX_ALLOC_FAIL_NO_MEM;
    }

    mp_item->role = p_params->role;
    mp_item->token = p_params->token;
    mp_item->length = p_params->net_packet_len;

    return CORE_TX_ALLOC_SUCCESS;
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(mp_item->length >= packet_length);

    memcpy(mp_item->data, p_packet, packet_length);
    queue_push(&m_loopback_data_queue, &mp_item->node);
    mp_item = NULL;
    bearer_event_flag_set(m_loopback_flag);
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(mp_item != NULL);
    mesh_mem_free(mp_item); /*lint !e424 Inappropriate deallocation (free) for 'modified' data */
    mp_item = NULL;
}

static bool loopback_process(void)
{
    nrf_mesh_rx_metadata_t metadata =
    {
        .source = NRF_MESH_RX_SOURCE_LOOPBACK,
    };

    queue_elem_t * p_node = queue_pop(&m_loopback_data_queue);
    while (p_node != NULL)
    {
        loopback_data_item_t * p_item = PARENT_BY_FIELD_GET(loopback_data_item_t, node, p_node);

        metadata.params.loopback.tx_token = p_item->token;
        core_tx_complete(&m_bearer, p_item->role, timer_now(), p_item->token);
        (void)network_packet_in(p_item->data, p_item->length, &metadata);

        p_node = queue_pop(&m_loopback_data_queue);
        mesh_mem_free(p_item); /*lint !e424 Inappropriate deallocation (free) for 'modified' data */
    }

    return true;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_local_init(void)
{
    m_loopback_flag = bearer_event_flag_add(loopback_process);
    core_tx_bearer_add(&m_bearer, &m_bearer_if, CORE_TX_BEARER_TYPE_LOCAL);
    queue_init(&m_loopback_data_queue);
}
