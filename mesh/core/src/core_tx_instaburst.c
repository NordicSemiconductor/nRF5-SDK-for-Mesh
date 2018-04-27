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
#include "core_tx_instaburst.h"
#include "core_tx.h"
#include "instaburst_tx.h"
#include "nrf_mesh_assert.h"
#include "nordic_common.h"
#include "nrf_mesh_config_bearer.h"


static instaburst_tx_t m_instaburst[CORE_TX_ROLE_COUNT];

static uint8_t m_originator_instaburst_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR];
static uint8_t m_relay_instaburst_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY];

static const uint8_t m_instaburst_channels[] = CORE_TX_INSTABURST_CHANNELS;


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

static void instaburst_tx_complete_callback(instaburst_tx_t * p_instaburst,
                                            nrf_mesh_tx_token_t token,
                                            uint32_t timestamp)
{
    core_tx_metadata_t metadata;
    metadata.bearer = CORE_TX_BEARER_ADV;
    metadata.role = (core_tx_role_t)(p_instaburst - m_instaburst);
    core_tx_complete(&metadata, timestamp, token);
}

static uint8_t * packet_alloc(uint32_t net_packet_len,
                              const core_tx_metadata_t * p_metadata,
                              nrf_mesh_tx_token_t token)
{
    uint8_t * p_buffer =
        instaburst_tx_buffer_alloc(&m_instaburst[p_metadata->role], sizeof(ble_ad_header_t) + net_packet_len, token);
    if (p_buffer)
    {
        ble_ad_data_t * p_ad_data = (ble_ad_data_t *) p_buffer;
        p_ad_data->type = AD_TYPE_MESH;
        p_ad_data->length = BLE_AD_DATA_OVERHEAD + net_packet_len;
        return p_ad_data->data;
    }
    else
    {
        return NULL;
    }
}

static void packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    instaburst_tx_buffer_commit(&m_instaburst[p_metadata->role],
                                (uint8_t *) PARENT_BY_FIELD_GET(ble_ad_data_t, data, p_packet));
}

static void packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    instaburst_tx_buffer_discard(&m_instaburst[p_metadata->role],
                                 (uint8_t *) PARENT_BY_FIELD_GET(ble_ad_data_t, data, p_packet));
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_instaburst_init(void)
{
    instaburst_tx_config_t instaburst_config;
    instaburst_config.channel_count  = ARRAY_SIZE(m_instaburst_channels);
    instaburst_config.p_channels     = m_instaburst_channels;
    instaburst_config.radio_mode     = RADIO_MODE_BLE_2MBIT;
    instaburst_config.set_id         = 0;
    instaburst_config.tx_power       = RADIO_POWER_NRF_0DBM;
    instaburst_config.callback       = instaburst_tx_complete_callback;
    instaburst_config.interval_ms    = BEARER_ADV_INT_DEFAULT_MS;

    instaburst_tx_instance_init(&m_instaburst[CORE_TX_ROLE_ORIGINATOR],
                             &instaburst_config,
                             m_originator_instaburst_packet_buffer,
                             sizeof(m_originator_instaburst_packet_buffer));
    instaburst_tx_enable(&m_instaburst[CORE_TX_ROLE_ORIGINATOR]);

    /* For consistency, disable the relay callback for instaburst too */
    instaburst_config.callback = NULL;

    instaburst_tx_instance_init(&m_instaburst[CORE_TX_ROLE_RELAY],
                             &instaburst_config,
                             m_relay_instaburst_packet_buffer,
                             sizeof(m_relay_instaburst_packet_buffer));
    instaburst_tx_enable(&m_instaburst[CORE_TX_ROLE_RELAY]);

    core_tx_bearer_register(CORE_TX_BEARER_ADV, &m_interface);
}


void core_tx_instaburst_interval_set(core_tx_role_t role, uint32_t interval_ms)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    instaburst_tx_interval_set(&m_instaburst[role], interval_ms);
}

uint32_t core_tx_instaburst_interval_get(core_tx_role_t role)
{
    NRF_MESH_ASSERT(role < CORE_TX_ROLE_COUNT);
    return m_instaburst[role].config.interval_ms;
}
