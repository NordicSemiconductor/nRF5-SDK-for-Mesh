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
#include <stdint.h>
#include "core_tx_adv.h"
#include "core_tx.h"
#include "advertiser.h"
#include "nrf_mesh_assert.h"

static uint32_t m_adv_tx_count[CORE_TX_ROLE_COUNT];
static advertiser_t m_advertisers[CORE_TX_ROLE_COUNT];

static uint8_t m_originator_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR];
static uint8_t m_relay_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_RELAY];

static uint8_t * packet_alloc(uint32_t net_packet_len,
                              const core_tx_metadata_t * p_metadata,
                              nrf_mesh_tx_token_t token);
static void packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);
static void packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

static const core_tx_bearer_interface_t m_interface = {packet_alloc,
                                                       packet_send,
                                                       packet_discard};
/*****************************************************************************
* Static functions
*****************************************************************************/
static inline adv_packet_t * get_adv_packet_from_net_packet(void * p_net_packet)
{
    return PARENT_BY_FIELD_GET(adv_packet_t, packet.payload[sizeof(ble_ad_header_t)], p_net_packet);
}

static void adv_tx_complete_callback(advertiser_t * p_adv,
                                     nrf_mesh_tx_token_t token,
                                     timestamp_t timestamp)
{
    core_tx_metadata_t metadata;
    metadata.bearer = CORE_TX_BEARER_ADV;
    metadata.role = (core_tx_role_t)(p_adv - m_advertisers);
    core_tx_complete(&metadata, timestamp, token);
}

static uint8_t * packet_alloc(uint32_t net_packet_len,
                              const core_tx_metadata_t * p_metadata,
                              nrf_mesh_tx_token_t token)
{
    adv_packet_t * p_packet = advertiser_packet_alloc(&m_advertisers[p_metadata->role],
                                                      sizeof(ble_ad_header_t) + net_packet_len);
    if (p_packet == NULL)
    {
        return NULL;
    }
    else
    {
        p_packet->token = token;
        p_packet->config.repeats = m_adv_tx_count[p_metadata->role];
        ble_ad_data_t * p_ad_data = (ble_ad_data_t *) &p_packet->packet.payload[0];
        p_ad_data->type = AD_TYPE_MESH;
        p_ad_data->length = BLE_AD_DATA_OVERHEAD + net_packet_len;
        return p_ad_data->data;
    }
}

static void packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    advertiser_packet_send(&m_advertisers[p_metadata->role],
                           get_adv_packet_from_net_packet(p_packet));
}

static void packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    advertiser_packet_discard(&m_advertisers[p_metadata->role],
                              get_adv_packet_from_net_packet(p_packet));
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_adv_init(void)
{
    m_adv_tx_count[CORE_TX_ROLE_ORIGINATOR] = CORE_TX_REPEAT_ORIGINATOR_DEFAULT;
    advertiser_instance_init(&m_advertisers[CORE_TX_ROLE_ORIGINATOR],
                             adv_tx_complete_callback,
                             m_originator_adv_packet_buffer,
                             sizeof(m_originator_adv_packet_buffer));
    advertiser_enable(&m_advertisers[CORE_TX_ROLE_ORIGINATOR]);

    m_adv_tx_count[CORE_TX_ROLE_RELAY] = CORE_TX_REPEAT_RELAY_DEFAULT;
    advertiser_instance_init(&m_advertisers[CORE_TX_ROLE_RELAY],
                             NULL,
                             m_relay_adv_packet_buffer,
                             sizeof(m_relay_adv_packet_buffer));
    advertiser_enable(&m_advertisers[CORE_TX_ROLE_RELAY]);

    core_tx_bearer_register(CORE_TX_BEARER_ADV, &m_interface);
}

void core_tx_adv_count_set(core_tx_role_t role, uint8_t tx_count)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    NRF_MESH_ASSERT(tx_count > 0);
    m_adv_tx_count[role] = tx_count;
}

uint8_t core_tx_adv_count_get(core_tx_role_t role)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    return m_adv_tx_count[role];
}

void core_tx_adv_interval_set(core_tx_role_t role, uint32_t interval_ms)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    advertiser_interval_set(&m_advertisers[role], interval_ms);
}

uint32_t core_tx_adv_interval_get(core_tx_role_t role)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    return US_TO_MS(m_advertisers[role].config.advertisement_interval_us);
}

void core_tx_adv_address_set(core_tx_role_t role, const ble_gap_addr_t * p_addr)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    advertiser_address_set(&m_advertisers[role], p_addr);
}
