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
#include "advertiser.h"
#include "broadcast.h"
#include "radio_config.h"
#include "packet_buffer.h"
#include "timer_scheduler.h"
#include "rand.h"
#include "toolchain.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_assert.h"
#include "nrf.h"
#include "debug_pins.h"

#define GAP_ADDR_TYPE_INDEX 5
#define GAP_ADDR_TYPE_VALUE_RANDOM_STATIC                 0xC0

static const uint8_t m_ble_adv_channels[] = NRF_MESH_ADV_CHAN_DEFAULT;
static prng_t m_adv_prng;

static inline bool is_active(const advertiser_t * p_adv)
{
    /* Any state that will lead to the timer firing is considered active. */
    return (p_adv->timer.state != TIMER_EVENT_STATE_UNUSED);
}

static inline packet_buffer_packet_t * get_packet_buffer_from_adv_packet(adv_packet_t * p_packet)
{
    return PARENT_BY_FIELD_GET(packet_buffer_packet_t, packet, p_packet);
}

static inline bool is_tx_complete_event_pending(advertiser_t * p_adv)
{
    return ((p_adv->tx_complete_callback != NULL) &&
            bearer_event_sequential_pending(&p_adv->tx_complete_event));
}

/* This is in highest priority: Called by radio_irq_handler in broadcast.c */
static void broadcast_complete_cb(broadcast_params_t * p_broadcast, timestamp_t timestamp)
{
    /* Find the owner of this broadcast */
    advertiser_t * p_adv = PARENT_BY_FIELD_GET(advertiser_t, broadcast.params, p_broadcast);

    if ((p_adv->p_packet->config.repeats == 0 ||
         p_adv->p_packet->config.repeats == ADVERTISER_REPEAT_INFINITE) &&
        p_adv->tx_complete_callback != NULL)
    {
        p_adv->tx_complete_params.token = p_adv->p_packet->token;
        p_adv->tx_complete_params.timestamp = timestamp;

        NRF_MESH_ASSERT(NRF_SUCCESS == bearer_event_sequential_post(&p_adv->tx_complete_event));
    }

    if (p_adv->p_packet->config.repeats == 0)
    {
        packet_buffer_packet_t * p_buf_packet = get_packet_buffer_from_adv_packet(p_adv->p_packet);
        p_adv->p_packet = NULL;
        packet_buffer_free(&p_adv->buf, p_buf_packet);
    }
}

static inline void randomize_channels(advertiser_channels_t * p_channels)
{
    if (p_channels->randomize_order)
    {
        for (uint8_t i = 0; i < p_channels->count; ++i)
        {
            /* Swap a channel with another random channel. */
            uint8_t rand_index = rand_prng_get(&m_adv_prng) % p_channels->count;
            if (rand_index != i)
            {
                uint8_t temp = p_channels->channel_map[i];
                p_channels->channel_map[i] = p_channels->channel_map[rand_index];
                p_channels->channel_map[rand_index] = temp;
            }
        }
    }
}

static inline void schedule_first_time(timer_event_t * p_timer_evt, uint32_t max_delay)
{
    NRF_MESH_ASSERT(max_delay >= MS_TO_US(BEARER_ADV_INT_MIN_MS) + ADVERTISER_INTERVAL_RANDOMIZATION_US);
    uint32_t rand_offset = rand_prng_get(&m_adv_prng) % (max_delay - MS_TO_US(BEARER_ADV_INT_MIN_MS));
    timer_sch_reschedule(p_timer_evt, timer_now() + MS_TO_US(BEARER_ADV_INT_MIN_MS) + rand_offset);
}

static inline void setup_next_timeout(timer_event_t * p_timer_evt, uint32_t base_interval)
{
    /* For the next timeout, it's enough to set the interval of the timer, as it will make it go
     * back into the scheduler queue after finishing the callback. */
    NRF_MESH_ASSERT(p_timer_evt->state == TIMER_EVENT_STATE_IN_CALLBACK);
    uint32_t rand_offset = rand_prng_get(&m_adv_prng) % ADVERTISER_INTERVAL_RANDOMIZATION_US;
    p_timer_evt->interval = base_interval + rand_offset;
}

static inline void update_repeat_count(advertiser_t * p_adv, adv_packet_t * p_adv_packet)
{
    NRF_MESH_ASSERT(p_adv_packet->config.repeats > 0);
    if (p_adv_packet->config.repeats != ADVERTISER_REPEAT_INFINITE)
    {
        p_adv_packet->config.repeats--;
    }
}

/**
 * Check whether the advertiser's current packet should be freed.
 *
 * @param[in,out] p_adv Advertiser to check.
 *
 * @returns Whether the advertiser's current packet should be freed.
 */
static inline bool should_free_current_packet(advertiser_t * p_adv)
{
    NRF_MESH_ASSERT(p_adv->p_packet != NULL);

    /* Infinite-repeat packets are replaced when new packets are added. */
    bool should_replace_infinite_packet =
        (p_adv->p_packet->config.repeats == ADVERTISER_REPEAT_INFINITE &&
         packet_buffer_packets_ready_to_pop(&p_adv->buf));

    /* Any packets with 0 repeats should be replaced */
    bool packet_has_no_repeats = (p_adv->p_packet->config.repeats == 0);

    return (should_replace_infinite_packet || packet_has_no_repeats);
}

static bool next_packet_fetch(advertiser_t * p_adv)
{
    while (p_adv->p_packet == NULL || should_free_current_packet(p_adv))
    {
        /* Free current packet */
        if (p_adv->p_packet != NULL)
        {
            packet_buffer_free(&p_adv->buf, get_packet_buffer_from_adv_packet(p_adv->p_packet));
            p_adv->p_packet = NULL;
        }

        packet_buffer_packet_t * p_packet_buf;
        if (packet_buffer_pop(&p_adv->buf, &p_packet_buf) == NRF_SUCCESS)
        {
            p_adv->p_packet = (adv_packet_t *) p_packet_buf->packet;
            p_adv->broadcast.params.p_packet = &p_adv->p_packet->packet;
        }
        else
        {
            /* No more packets left. */
            return false;
        }
    }
    return true;
}

/**
 * Schedule a single advertisement event.
 *
 * @param[in,out] p_adv Advertiser to schedule for.
 */
static void schedule_broadcast(advertiser_t * p_adv)
{
    NRF_MESH_ASSERT(p_adv->p_packet != NULL);

    randomize_channels(&p_adv->config.channels);
    update_repeat_count(p_adv, p_adv->p_packet);

    NRF_MESH_ASSERT(NRF_SUCCESS == broadcast_send(&p_adv->broadcast));
}

static void timeout_event(timestamp_t timestamp, void * p_context)
{
    advertiser_t * p_adv = (advertiser_t *) p_context;
    p_adv->timer.interval = 0;

    if (p_adv->enabled)
    {
        bool has_packet;

        /* Only attempt to transmit if the broadcast context is inactive, and no TX_COMPLETE event
         * is pending. Skip this event if the broadcast is still waiting to fire from the previous
         * advertisement event. */
        if (p_adv->broadcast.active || is_tx_complete_event_pending(p_adv))
        {
            has_packet = true;
        }
        else
        {
            has_packet = next_packet_fetch(p_adv);
            if (has_packet)
            {
                schedule_broadcast(p_adv);
            }
        }

        if (has_packet)
        {
            setup_next_timeout(&p_adv->timer, p_adv->config.advertisement_interval_us);
        }
    }
}

static void set_gap_addr_type(ble_gap_addr_t* p_addr)
{
    NRF_MESH_ASSERT(p_addr->addr_type == BLE_GAP_ADDR_TYPE_PUBLIC ||
                    p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC);
    if (p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
    {
        p_addr->addr[GAP_ADDR_TYPE_INDEX] |= GAP_ADDR_TYPE_VALUE_RANDOM_STATIC;
    }
}

static inline void set_default_advertiser_configuration(advertiser_config_t * p_config)
{
    advertiser_address_default_get(&p_config->adv_addr);

    p_config->advertisement_interval_us = MS_TO_US(BEARER_ADV_INT_DEFAULT_MS);
    uint8_t channel_count;
    for (channel_count = 0;
         channel_count < BEARER_ADV_CHANNELS_MAX && channel_count < sizeof(m_ble_adv_channels) / sizeof(m_ble_adv_channels[0]);
         ++channel_count)
    {
        p_config->channels.channel_map[channel_count] = m_ble_adv_channels[channel_count];
    }
    p_config->channels.count = channel_count;
    p_config->channels.randomize_order = false;
}

static inline void set_default_broadcast_configuration(broadcast_t * p_broadcast)
{
    p_broadcast->params.access_address = BEARER_ACCESS_ADDR_DEFAULT;
    p_broadcast->params.radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE;
    p_broadcast->params.radio_config.radio_mode = RADIO_MODE_BLE_1MBIT;
    p_broadcast->params.radio_config.tx_power = RADIO_POWER_NRF_0DBM;
}

static inline void set_adv_address(advertiser_t * p_adv, packet_t * p_packet)
{
    p_packet->header.addr_type = p_adv->config.adv_addr.addr_type;
    memcpy(p_packet->addr, p_adv->config.adv_addr.addr, BLE_GAP_ADDR_LEN);
}

static void tx_complete_event_callback(void * p_context)
{
    advertiser_t * p_adv = (advertiser_t *)p_context;

    NRF_MESH_ASSERT(p_adv != NULL);
    NRF_MESH_ASSERT(p_adv->tx_complete_callback != NULL);
    p_adv->tx_complete_callback(p_adv, p_adv->tx_complete_params.token, p_adv->tx_complete_params.timestamp);
}
/**************************************************************************************************
************************************* Public APIs *************************************************
***************************************************************************************************/

void advertiser_init(void)
{
    rand_prng_seed(&m_adv_prng);
}

void advertiser_instance_init(advertiser_t * p_adv,
                              advertiser_tx_complete_cb_t tx_complete_cb,
                              uint8_t * p_buffer,
                              uint32_t buffer_size)
{
    NRF_MESH_ASSERT(p_adv != NULL && p_buffer != NULL);
    NRF_MESH_ASSERT(buffer_size > (BLE_ADV_PACKET_MIN_LENGTH + sizeof(packet_buffer_packet_t)));
    packet_buffer_init(&p_adv->buf, p_buffer, buffer_size);
    set_default_advertiser_configuration(&p_adv->config);
    set_default_broadcast_configuration(&p_adv->broadcast);
    p_adv->broadcast.params.p_channels = p_adv->config.channels.channel_map;
    p_adv->broadcast.params.channel_count = p_adv->config.channels.count;
    p_adv->broadcast.params.tx_complete_cb = broadcast_complete_cb;
    p_adv->broadcast.params.p_packet = NULL;
    p_adv->tx_complete_callback = tx_complete_cb;
    p_adv->timer.cb = timeout_event;
    p_adv->timer.p_context = p_adv;
    p_adv->enabled = false;

    if (tx_complete_cb != NULL)
    {
        bearer_event_sequential_add(&p_adv->tx_complete_event, tx_complete_event_callback, p_adv);
    }
}

void advertiser_enable(advertiser_t * p_adv)
{
    if (!p_adv->enabled)
    {
        p_adv->enabled = true;
        if (p_adv->p_packet != NULL || packet_buffer_can_pop(&p_adv->buf))
        {
            schedule_first_time(&p_adv->timer, p_adv->config.advertisement_interval_us + ADVERTISER_INTERVAL_RANDOMIZATION_US);
        }
    }
}

void advertiser_disable(advertiser_t * p_adv)
{
    p_adv->enabled = false;
    timer_sch_abort(&p_adv->timer);
}

adv_packet_t * advertiser_packet_alloc(advertiser_t * p_adv, uint32_t adv_payload_size)
{
    NRF_MESH_ASSERT(p_adv != NULL);
    NRF_MESH_ASSERT(adv_payload_size <= BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);

    packet_buffer_packet_t * p_buf_packet;
    uint32_t status = packet_buffer_reserve(&p_adv->buf, &p_buf_packet, sizeof(adv_packet_t) - BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH + adv_payload_size);
    if (NRF_SUCCESS == status)
    {
        adv_packet_t * p_adv_packet  = (adv_packet_t *) p_buf_packet->packet;
        packet_payload_size_set(&p_adv_packet->packet, adv_payload_size);
        p_adv_packet->packet.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
        set_adv_address(p_adv, &p_adv_packet->packet);
        p_adv_packet->token = NRF_MESH_INITIAL_TOKEN;
        return p_adv_packet;
    }
    else
    {
        NRF_MESH_ASSERT(status == NRF_ERROR_NO_MEM);
        return NULL;
    }
}

void advertiser_packet_send(advertiser_t * p_adv, adv_packet_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL && NULL != p_adv);
    NRF_MESH_ASSERT(p_packet->config.repeats > 0);

    packet_buffer_packet_t * p_buf_packet = get_packet_buffer_from_adv_packet(p_packet);
    /* Force set fields with mandatory values: */
    p_packet->packet.header._rfu1 = 0;
    p_packet->packet.header._rfu2 = 0;
    p_packet->packet.header._rfu3 = 0;

    packet_buffer_commit(&p_adv->buf, p_buf_packet, p_buf_packet->size);
    if (p_adv->enabled && !is_active(p_adv))
    {
        schedule_first_time(&p_adv->timer, p_adv->config.advertisement_interval_us + ADVERTISER_INTERVAL_RANDOMIZATION_US);
    }
}

void advertiser_packet_discard(advertiser_t * p_adv, adv_packet_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL && NULL != p_adv);
    packet_buffer_packet_t * p_buf_packet = get_packet_buffer_from_adv_packet(p_packet);
    packet_buffer_free(&p_adv->buf, p_buf_packet);
}

void advertiser_config_set(advertiser_t * p_adv, const advertiser_config_t * p_config)
{
    NRF_MESH_ASSERT(p_adv != NULL && p_config != NULL);
    advertiser_channels_set(p_adv, &p_config->channels);
    advertiser_address_set(p_adv, &p_config->adv_addr);
    advertiser_interval_set(p_adv, US_TO_MS(p_config->advertisement_interval_us));
}

void advertiser_channels_set(advertiser_t * p_adv, const advertiser_channels_t * p_channels)
{
    NRF_MESH_ASSERT(p_adv != NULL && p_channels != NULL);
    NRF_MESH_ASSERT(p_channels->count > 0);
    NRF_MESH_ASSERT(p_channels->count <= BEARER_ADV_CHANNELS_MAX);
    for (uint32_t i = 0; i < p_channels->count; i++)
    {
        NRF_MESH_ASSERT(p_channels->channel_map[i] < RADIO_NO_RF_CHANNELS);
    }
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    memcpy(&p_adv->config.channels, p_channels, sizeof(advertiser_channels_t));
    p_adv->broadcast.params.channel_count = p_adv->config.channels.count;
    _ENABLE_IRQS(was_masked);
}

void advertiser_address_set(advertiser_t * p_adv, const ble_gap_addr_t * p_addr)
{
    NRF_MESH_ASSERT(p_adv != NULL && p_addr != NULL);
    NRF_MESH_ASSERT(p_addr->addr_type == BLE_GAP_ADDR_TYPE_PUBLIC ||
                    p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    memcpy(&p_adv->config.adv_addr, p_addr, sizeof(ble_gap_addr_t));
    /* Enforce Bluetooth Core Specification v4.0 GAP address rules */
    set_gap_addr_type(&p_adv->config.adv_addr);
    _ENABLE_IRQS(was_masked);
}

void advertiser_interval_set(advertiser_t * p_adv, uint32_t interval_ms)
{
    NRF_MESH_ASSERT(NULL != p_adv);
    NRF_MESH_ASSERT(interval_ms >= BEARER_ADV_INT_MIN_MS);
    NRF_MESH_ASSERT(interval_ms <= BEARER_ADV_INT_MAX_MS);
    p_adv->config.advertisement_interval_us = MS_TO_US(interval_ms);
    if (is_active(p_adv) && p_adv->enabled)
    {
        /* Instead of waiting for the slow advertisement interval to fire the next advertisement,
         * reschedule it to let the interval take immediate effect. */
        schedule_first_time(&p_adv->timer, p_adv->config.advertisement_interval_us + ADVERTISER_INTERVAL_RANDOMIZATION_US);
    }
}

void advertiser_config_get(const advertiser_t * p_adv, advertiser_config_t * p_config)
{
    NRF_MESH_ASSERT(p_config != NULL && NULL != p_adv);
    memcpy(p_config, &p_adv->config, sizeof(advertiser_config_t));
}

void advertiser_tx_power_set(advertiser_t * p_adv, radio_tx_power_t tx_power)
{
    NRF_MESH_ASSERT(NULL != p_adv);
    p_adv->broadcast.params.radio_config.tx_power = tx_power;
}

void advertiser_flush(advertiser_t * p_adv)
{
    packet_buffer_flush(&p_adv->buf);

    /* Stop the sending of the current packet: */
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (p_adv->p_packet != NULL)
    {
        p_adv->p_packet->config.repeats = 0;
    }
    _ENABLE_IRQS(was_masked);
}

void advertiser_address_default_get(ble_gap_addr_t * p_addr)
{
    uint32_t hw_addr_type = (NRF_FICR->DEVICEADDRTYPE & FICR_DEVICEADDRTYPE_DEVICEADDRTYPE_Msk);
    switch (hw_addr_type)
    {
        case FICR_DEVICEADDRTYPE_DEVICEADDRTYPE_Public:
            p_addr->addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
            break;
        case FICR_DEVICEADDRTYPE_DEVICEADDRTYPE_Random:
            p_addr->addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
            break;
        default:
            /* All other values should be impossible: */
            NRF_MESH_ASSERT(false);
    }

    p_addr->addr_id_peer = 0;
    memcpy(p_addr->addr, (uint8_t*) NRF_FICR->DEVICEADDR, BLE_GAP_ADDR_LEN);
    set_gap_addr_type(p_addr);
}
