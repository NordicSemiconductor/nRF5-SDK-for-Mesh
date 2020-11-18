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
#include "core_tx_lpn.h"
#include "core_tx.h"
#include "broadcast.h"
#include "packet.h"
#include "bearer_event.h"
#include "advertiser.h"
#include "mesh_opt_core.h"
#include "mesh_config_listener.h"

#if MESH_FEATURE_LPN_ENABLED

typedef enum
{
    LPN_BEARER_STATE_FREE,
    LPN_BEARER_STATE_ALLOCATED,
    LPN_BEARER_STATE_SENDING,
} lpn_bearer_state_t;

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params);
static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length);
static void packet_discard(core_tx_bearer_t * p_bearer);
/*****************************************************************************
* Static globals
*****************************************************************************/
static const core_tx_bearer_interface_t m_bearer_if = {
    .packet_alloc   = packet_alloc,
    .packet_send    = packet_send,
    .packet_discard = packet_discard,
};

static core_tx_bearer_t m_bearer;

static struct
{
    lpn_bearer_state_t state;
    broadcast_t broadcast;
    bearer_event_sequential_t tx_complete_event;

    struct
    {
        timestamp_t timestamp;
        core_tx_role_t role;
        nrf_mesh_tx_token_t token;
        packet_t buffer;
    } packet;
} m_lpn_bearer;

static uint8_t m_adv_channels[BEARER_ADV_CHANNELS_MAX] = NRF_MESH_ADV_CHAN_DEFAULT;
/*****************************************************************************
* Static functions
*****************************************************************************/
static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    if (p_params->bearer_selector != CORE_TX_BEARER_TYPE_LOW_POWER)
    {
        /* This bearer should only send packets that are explicitly sent on LPN. */
        return CORE_TX_ALLOC_FAIL_REJECTED;
    }

    if (m_lpn_bearer.state != LPN_BEARER_STATE_FREE)
    {
        return CORE_TX_ALLOC_FAIL_NO_MEM;
    }

    m_lpn_bearer.packet.role = p_params->role;
    m_lpn_bearer.packet.token = p_params->token;
    m_lpn_bearer.state = LPN_BEARER_STATE_ALLOCATED;

    return CORE_TX_ALLOC_SUCCESS;
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(m_lpn_bearer.state == LPN_BEARER_STATE_ALLOCATED);
    NRF_MESH_ASSERT(packet_length + sizeof(ble_ad_header_t) <= BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);

    m_lpn_bearer.packet.buffer.header.length = BLE_ADV_PACKET_OVERHEAD + sizeof(ble_ad_header_t) + packet_length;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) m_lpn_bearer.packet.buffer.payload;
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = BLE_AD_DATA_OVERHEAD + packet_length;
    memcpy(p_ad_data->data, p_packet, packet_length);

    m_lpn_bearer.state = LPN_BEARER_STATE_SENDING;

    NRF_MESH_ERROR_CHECK(broadcast_send(&m_lpn_bearer.broadcast));
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(m_lpn_bearer.state == LPN_BEARER_STATE_ALLOCATED);
    m_lpn_bearer.state = LPN_BEARER_STATE_FREE;
}

static void tx_complete_process(void * p_context)
{
    NRF_MESH_ASSERT(m_lpn_bearer.state == LPN_BEARER_STATE_SENDING);

    m_lpn_bearer.state = LPN_BEARER_STATE_FREE;

    core_tx_complete(&m_bearer,
                     m_lpn_bearer.packet.role,
                     m_lpn_bearer.packet.timestamp,
                     m_lpn_bearer.packet.token);
}

static void tx_complete_cb(broadcast_params_t * p_params, timestamp_t timestamp)
{
    m_lpn_bearer.packet.timestamp = timestamp;
    NRF_MESH_ERROR_CHECK(bearer_event_sequential_post(&m_lpn_bearer.tx_complete_event));
}

static void tx_power_config_listener_cb(mesh_config_change_reason_t reason, mesh_config_entry_id_t id, const void * p_entry)
{
    if (id.file == MESH_OPT_CORE_FILE_ID &&
        id.record == (MESH_OPT_CORE_TX_POWER_RECORD_START + CORE_TX_ROLE_ORIGINATOR))
    {
        const radio_tx_power_t * p_tx_power = p_entry;
        switch (reason)
        {
            case MESH_CONFIG_CHANGE_REASON_SET:
                m_lpn_bearer.broadcast.params.radio_config.tx_power = *p_tx_power;
                break;
            case MESH_CONFIG_CHANGE_REASON_DELETE:
                m_lpn_bearer.broadcast.params.radio_config.tx_power = RADIO_POWER_NRF_0DBM;
                break;
            default:
                break;
        }
    }
}

MESH_CONFIG_LISTENER(m_lpn_tx_power_config_listener, MESH_OPT_CORE_TX_POWER_EID, tx_power_config_listener_cb);

/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_lpn_init(radio_tx_power_t tx_power)
{
    m_lpn_bearer.broadcast.params.access_address = BEARER_ACCESS_ADDR_DEFAULT;
    m_lpn_bearer.broadcast.params.radio_config.radio_mode = RADIO_MODE_BLE_1MBIT;
    m_lpn_bearer.broadcast.params.radio_config.tx_power = tx_power;
    m_lpn_bearer.broadcast.params.radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE;
    m_lpn_bearer.broadcast.params.p_channels = m_adv_channels;
    m_lpn_bearer.broadcast.params.channel_count = ARRAY_SIZE((uint8_t[]) NRF_MESH_ADV_CHAN_DEFAULT);
    m_lpn_bearer.broadcast.params.tx_complete_cb = tx_complete_cb;
    m_lpn_bearer.broadcast.params.p_packet = &m_lpn_bearer.packet.buffer;

    ble_gap_addr_t addr;
    advertiser_address_default_get(&addr);
    memcpy(m_lpn_bearer.packet.buffer.addr, addr.addr, BLE_GAP_ADDR_LEN);
    m_lpn_bearer.packet.buffer.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    m_lpn_bearer.packet.buffer.header.addr_type = (addr.addr_type != BLE_GAP_ADDR_TYPE_PUBLIC);

    m_lpn_bearer.state = LPN_BEARER_STATE_FREE;

    bearer_event_sequential_add(&m_lpn_bearer.tx_complete_event, tx_complete_process, &m_lpn_bearer);

    core_tx_bearer_add(&m_bearer, &m_bearer_if, CORE_TX_BEARER_TYPE_LOW_POWER);
}

#endif /* MESH_FEATURE_LPN_ENABLED */
