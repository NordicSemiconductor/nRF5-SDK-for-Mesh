/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#include <stdint.h>
#include "core_tx_adv.h"
#include "core_tx.h"
#include "advertiser.h"
#include "nrf_mesh_assert.h"


static struct
{
    uint32_t adv_tx_count;
    advertiser_t advertiser;
} m_bearer_roles[CORE_TX_ROLE_COUNT];

static struct
{
    core_tx_role_t role;
    adv_packet_t * p_packet;
} m_current_alloc;


static uint8_t m_originator_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR];
static uint8_t m_relay_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_RELAY];

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params);
static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length);
static void packet_discard(core_tx_bearer_t * p_bearer);

static const core_tx_bearer_interface_t m_interface = {packet_alloc,
                                                       packet_send,
                                                       packet_discard};
static core_tx_bearer_t m_bearer;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void adv_tx_complete_callback(advertiser_t * p_adv,
                                     nrf_mesh_tx_token_t token,
                                     timestamp_t timestamp)
{
    core_tx_role_t role = (core_tx_role_t) (p_adv - &m_bearer_roles[0].advertiser);
    core_tx_complete(&m_bearer, role, timestamp, token);
}

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet == NULL);

    m_current_alloc.p_packet =
        advertiser_packet_alloc(&m_bearer_roles[p_params->role].advertiser,
                                sizeof(ble_ad_header_t) + p_params->net_packet_len);

    if (m_current_alloc.p_packet == NULL)
    {
        return CORE_TX_ALLOC_FAIL_NO_MEM;
    }
    else
    {
        m_current_alloc.p_packet->token          = p_params->token;
        m_current_alloc.p_packet->config.repeats = m_bearer_roles[p_params->role].adv_tx_count;
        m_current_alloc.role                     = p_params->role;

        return CORE_TX_ALLOC_SUCCESS;
    }
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    /* Build the packet data */
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) &m_current_alloc.p_packet->packet.payload[0];
    p_ad_data->type           = AD_TYPE_MESH;
    p_ad_data->length         = BLE_AD_DATA_OVERHEAD + packet_length;
    memcpy(p_ad_data->data, p_packet, packet_length);

    advertiser_packet_send(&m_bearer_roles[m_current_alloc.role].advertiser,
                           m_current_alloc.p_packet);
    m_current_alloc.p_packet = NULL;
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    advertiser_packet_discard(&m_bearer_roles[m_current_alloc.role].advertiser,
                              m_current_alloc.p_packet);
    m_current_alloc.p_packet = NULL;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_adv_init(void)
{
    m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].adv_tx_count = CORE_TX_REPEAT_ORIGINATOR_DEFAULT;
    advertiser_instance_init(&m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].advertiser,
                             adv_tx_complete_callback,
                             m_originator_adv_packet_buffer,
                             sizeof(m_originator_adv_packet_buffer));
    advertiser_enable(&m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].advertiser);

    m_bearer_roles[CORE_TX_ROLE_RELAY].adv_tx_count = CORE_TX_REPEAT_RELAY_DEFAULT;
    advertiser_instance_init(&m_bearer_roles[CORE_TX_ROLE_RELAY].advertiser,
                             NULL,
                             m_relay_adv_packet_buffer,
                             sizeof(m_relay_adv_packet_buffer));
    advertiser_enable(&m_bearer_roles[CORE_TX_ROLE_RELAY].advertiser);

    core_tx_bearer_add(&m_bearer, &m_interface, CORE_TX_BEARER_TYPE_ADV);
}

void core_tx_adv_count_set(core_tx_role_t role, uint8_t tx_count)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    NRF_MESH_ASSERT(tx_count > 0);
    m_bearer_roles[role].adv_tx_count = tx_count;
}

uint8_t core_tx_adv_count_get(core_tx_role_t role)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    return m_bearer_roles[role].adv_tx_count;
}

void core_tx_adv_interval_set(core_tx_role_t role, uint32_t interval_ms)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    advertiser_interval_set(&m_bearer_roles[role].advertiser, interval_ms);
}

uint32_t core_tx_adv_interval_get(core_tx_role_t role)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    return US_TO_MS(m_bearer_roles[role].advertiser.config.advertisement_interval_us);
}

void core_tx_adv_address_set(core_tx_role_t role, const ble_gap_addr_t * p_addr)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    advertiser_address_set(&m_bearer_roles[role].advertiser, p_addr);
}
