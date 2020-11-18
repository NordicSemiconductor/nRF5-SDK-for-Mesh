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
#include "core_tx_friend.h"
#include "core_tx.h"
#include "broadcast.h"
#include "packet.h"
#include "bearer_event.h"
#include "advertiser.h"
#include "radio_config.h"
#include "timer.h"
#include "log.h"

/** Time spent preprocessing the packet sending before it goes out. */
#define TX_PREPROCESSING_US           (RADIO_RAMPUP_TIME)

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

static uint8_t m_adv_channels[BEARER_ADV_CHANNELS_MAX] = NRF_MESH_ADV_CHAN_DEFAULT;
/*****************************************************************************
* Static functions
*****************************************************************************/
static core_tx_friend_t * bearer_to_friend(core_tx_bearer_t * p_bearer)
{
    return PARENT_BY_FIELD_GET(core_tx_friend_t, bearer, p_bearer);
}

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    core_tx_friend_t * p_friend = bearer_to_friend(p_bearer);

    /* Friend bearers should only accept packets that matches its assigned token and that use the FRIEND bearer type. */
    if (p_params->bearer_selector != CORE_TX_BEARER_TYPE_FRIEND ||
        p_friend->token != p_params->token || !p_friend->enabled)
    {
        return CORE_TX_ALLOC_FAIL_REJECTED;
    }

    if (p_friend->state != CORE_TX_FRIEND_STATE_READY)
    {
        return CORE_TX_ALLOC_FAIL_NO_MEM;
    }

    p_friend->packet.role = p_params->role;
    p_friend->state = CORE_TX_FRIEND_STATE_ALLOCATED;

    return CORE_TX_ALLOC_SUCCESS;
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    core_tx_friend_t * p_friend = bearer_to_friend(p_bearer);
    NRF_MESH_ASSERT_DEBUG(p_friend->state == CORE_TX_FRIEND_STATE_ALLOCATED);
    NRF_MESH_ASSERT_DEBUG(packet_length + sizeof(ble_ad_header_t) <= BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);

    p_friend->packet.buffer.header.length = BLE_ADV_PACKET_OVERHEAD + sizeof(ble_ad_header_t) + packet_length;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) p_friend->packet.buffer.payload;
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = BLE_AD_DATA_OVERHEAD + packet_length;
    memcpy(p_ad_data->data, p_packet, packet_length);

    p_friend->state = CORE_TX_FRIEND_STATE_WAITING;
}

static void tx_timeout(timestamp_t timestamp, void * p_context)
{
    core_tx_friend_t * p_friend = (core_tx_friend_t *) p_context;
    NRF_MESH_ASSERT_DEBUG(p_friend->state == CORE_TX_FRIEND_STATE_WAITING);
    p_friend->state = CORE_TX_FRIEND_STATE_SENDING;
    NRF_MESH_ERROR_CHECK(broadcast_send(&p_friend->broadcast));
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    core_tx_friend_t * p_friend = bearer_to_friend(p_bearer);
    NRF_MESH_ASSERT_DEBUG(p_friend->state == CORE_TX_FRIEND_STATE_ALLOCATED);
    p_friend->state = CORE_TX_FRIEND_STATE_READY;
}

static void tx_complete_cb(broadcast_params_t * p_params, timestamp_t timestamp)
{
    core_tx_friend_t * p_friend = PARENT_BY_FIELD_GET(core_tx_friend_t, broadcast.params, p_params);
    p_friend->packet.timestamp = timestamp;
#if FRIEND_DEBUG
    p_friend->stats.tx_timeout_delay_cur_us = TIMER_OLDER_THAN(p_friend->stats.last_request_timestamp, timestamp) ?
                                               TIMER_DIFF(timestamp, p_friend->stats.last_request_timestamp) :
                                                -TIMER_DIFF(timestamp, p_friend->stats.last_request_timestamp);
    if (p_friend->stats.tx_timeout_delay_cur_us > p_friend->stats.tx_timeout_delay_max_us)
    {
        p_friend->stats.tx_timeout_delay_max_us = p_friend->stats.tx_timeout_delay_cur_us;
    }
#endif

    NRF_MESH_ERROR_CHECK(bearer_event_sequential_post(&p_friend->tx_complete_event));
}

static void tx_complete_process(void * p_context)
{
    core_tx_friend_t * p_friend = (core_tx_friend_t *) p_context;
    NRF_MESH_ASSERT_DEBUG(p_friend->state == CORE_TX_FRIEND_STATE_SENDING);

    p_friend->state = CORE_TX_FRIEND_STATE_READY;

    core_tx_complete(&p_friend->bearer,
                    p_friend->packet.role,
                    p_friend->packet.timestamp,
                    p_friend->token);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void core_tx_friend_init(core_tx_friend_t * p_bearer, nrf_mesh_tx_token_t token, radio_tx_power_t tx_power)
{
    p_bearer->broadcast.params.access_address = BEARER_ACCESS_ADDR_DEFAULT;
    p_bearer->broadcast.params.radio_config.radio_mode = RADIO_MODE_BLE_1MBIT;
    p_bearer->broadcast.params.radio_config.tx_power = tx_power;
    p_bearer->broadcast.params.radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE;
    p_bearer->broadcast.params.p_channels = m_adv_channels;
    p_bearer->broadcast.params.channel_count = ARRAY_SIZE((uint8_t[]) NRF_MESH_ADV_CHAN_DEFAULT);
    p_bearer->broadcast.params.tx_complete_cb = tx_complete_cb;
    p_bearer->broadcast.params.p_packet = &p_bearer->packet.buffer;

    ble_gap_addr_t addr;
    advertiser_address_default_get(&addr);
    memcpy(p_bearer->packet.buffer.addr, addr.addr, BLE_GAP_ADDR_LEN);
    p_bearer->packet.buffer.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    p_bearer->packet.buffer.header.addr_type = (addr.addr_type != BLE_GAP_ADDR_TYPE_PUBLIC);

    p_bearer->tx_timer.cb = tx_timeout;
    p_bearer->tx_timer.p_context = p_bearer;
    p_bearer->tx_timer.interval = 0;

    p_bearer->token = token;

    p_bearer->state = CORE_TX_FRIEND_STATE_READY;
    p_bearer->enabled = false;

    bearer_event_sequential_add(&p_bearer->tx_complete_event, tx_complete_process, p_bearer);

    core_tx_bearer_add(&p_bearer->bearer, &m_bearer_if, CORE_TX_BEARER_TYPE_FRIEND);
}

void core_tx_friend_enable(core_tx_friend_t * p_bearer)
{
    p_bearer->enabled = true;
}

void core_tx_friend_disable(core_tx_friend_t * p_bearer)
{
    /* As the core is expected to allocate and send (or discard) a packet atomically, it shouldn't
     * be possible to disable while in allocated state. We'll assert here to ensure that this
     * assumption is correct, as handling this state could get complex. */
    NRF_MESH_ASSERT_DEBUG(p_bearer->state != CORE_TX_FRIEND_STATE_ALLOCATED);

    if (p_bearer->state == CORE_TX_FRIEND_STATE_WAITING)
    {
        timer_sch_abort(&p_bearer->tx_timer);
        p_bearer->state = CORE_TX_FRIEND_STATE_READY;
    }
    p_bearer->enabled = false;
}

void core_tx_friend_schedule(core_tx_friend_t * p_bearer, timestamp_t tx_time)
{
    timer_sch_reschedule(&p_bearer->tx_timer, tx_time - (TX_PREPROCESSING_US - TIMER_NOW_MAX_ERROR_US));

#if FRIEND_DEBUG
    p_bearer->stats.last_request_timestamp = tx_time;
#endif
}
