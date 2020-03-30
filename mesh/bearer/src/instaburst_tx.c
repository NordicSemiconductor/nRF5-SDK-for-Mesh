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

#include "instaburst_tx.h"
#include "instaburst_internal.h"
#include "adv_ext_packet.h"
#include "timeslot.h"
#include "debug_pins.h"
#include "nrf_mesh_assert.h"
#include "rand.h"
#include "adv_ext_tx.h"
#include "packet.h"
#include "advertiser.h"
#include "nordic_common.h"
/*****************************************************************************
 * Local defines
 *****************************************************************************/
#define INSTABURST_ADV_INT_RANDOMIZATION_MAX    MS_TO_US(10)

/** The minimum number of tokens to allocate space for. */
#define TOKEN_COUNT_MIN (16)

/* In order to reuse the aux buffer for regular advertisement packets, reserved buffer space for
 * headers must be able to fit the adv addr, as we're using that space to place it. */
NRF_MESH_STATIC_ASSERT(BLE_ADV_PACKET_OVERHEAD <= AUX_BUFFER_OVERHEAD_MAX);
/*****************************************************************************
* Static globals
*****************************************************************************/
static prng_t m_rand;
static ble_gap_addr_t m_ble_gap_addr;
static bool m_buffer_lock; /**< Flag allowing users to prevent us from committing any unfinished buffers. */
static const uint8_t m_adv_channels[] = NRF_MESH_ADV_CHAN_ALL;
/*****************************************************************************
* Static functions
*****************************************************************************/
/**
 * nRF5 SDK's 16 bit CRC function. Inlined to avoid all the unnecessary dependencies.
 */
static uint16_t crc16_compute(uint8_t const * p_data, uint32_t size, uint16_t const * p_crc)
{
    uint16_t crc = (p_crc == NULL) ? 0xFFFF : *p_crc;

    for (uint32_t i = 0; i < size; i++)
    {
        crc  = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xFF) << 4) << 1;
    }

    return crc;
}

static inline uint32_t randomized_tx_interval_get(instaburst_tx_t * p_instaburst)
{
    return MS_TO_US(p_instaburst->config.interval_ms) +
           (rand_prng_get(&m_rand) % INSTABURST_ADV_INT_RANDOMIZATION_MAX);
}

static inline nrf_mesh_tx_token_t * tx_buffer_token_get(packet_buffer_packet_t * p_buffer, uint32_t index)
{
    /* Tokens are stored at the end of the buffer in reverse order: */
    return (nrf_mesh_tx_token_t *) &p_buffer->packet[p_buffer->size - (1 + index) * sizeof(nrf_mesh_tx_token_t)];
}

static void event_id_generate(uint8_t set_id, adv_ext_tx_event_t * p_tx_event)
{
    const adv_ext_tx_packet_t * p_packet = (adv_ext_tx_packet_t *) &p_tx_event->packet_data[0];
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < p_tx_event->params.packet_count; ++i)
    {
        crc = crc16_compute(p_packet->data, p_packet->data_len, &crc);
        p_packet = adv_ext_tx_packet_next_get(p_packet);
    }
    p_tx_event->params.id.data_id = crc & INSTABURST_DATA_ID_MASK;
    p_tx_event->params.id.set_id  = set_id;

    instaburst_event_id_cache_put(&p_tx_event->params.id);
}

static inline uint8_t * buffer_reserve_pointer_get(instaburst_tx_t * p_instaburst)
{
    return &p_instaburst->p_alloc_packet->data[p_instaburst->p_alloc_packet->data_len];
}

static inline bool should_transmit_as_regular_packet(const adv_ext_tx_event_t * p_event)
{
    return (p_event->params.packet_count == 1 &&
            ((adv_ext_tx_packet_t *) &p_event->packet_data[0])->data_len <= BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH);
}

static packet_t * regular_packet_create(adv_ext_tx_packet_t * p_tx_packet)
{
    packet_t * p_packet = (packet_t *) (p_tx_packet->data - BLE_ADV_PACKET_BUFFER_OVERHEAD);

    p_packet->header.type       = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    p_packet->header.addr_type  = (BLE_GAP_ADDR_TYPE_PUBLIC != m_ble_gap_addr.addr_type);
    p_packet->header._rfu1      = 0;
    p_packet->header.length     = p_tx_packet->data_len + BLE_ADV_PACKET_OVERHEAD;
    p_packet->header._rfu2      = 0;
    memcpy(p_packet->addr, m_ble_gap_addr.addr, BLE_GAP_ADDR_LEN);
    return p_packet;
}

static inline uint8_t channel_get_and_iterate(instaburst_tx_t * p_instaburst)
{
    uint8_t channel = p_instaburst->config.p_channels[p_instaburst->channel_index++];
    if (p_instaburst->channel_index >= p_instaburst->config.channel_count)
    {
        p_instaburst->channel_index = 0;
    }
    return channel;
}

static bool reserve_buffer(instaburst_tx_t * p_instaburst)
{
    uint32_t adv_ext_buffer_maxlen =
        sizeof(adv_ext_tx_event_t) +
        ADV_EXT_TX_CHAIN_MAX_COUNT * (sizeof(adv_ext_tx_packet_t) + ADV_EXT_PACKET_LEN_MAX);

    uint32_t status = packet_buffer_reserve(&p_instaburst->packet_buffer,
                                            &p_instaburst->p_alloc_buf,
                                            adv_ext_buffer_maxlen + sizeof(nrf_mesh_tx_token_t) * TOKEN_COUNT_MIN);
    if (status == NRF_SUCCESS)
    {
        adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) &p_instaburst->p_alloc_buf->packet[0];
        p_event->params.packet_count = 1;
        p_event->params.token_count  = 0;
        p_event->params.channel      = channel_get_and_iterate(p_instaburst);
        p_instaburst->p_alloc_packet = (adv_ext_tx_packet_t *) &p_event->packet_data[0];
        p_instaburst->p_alloc_packet->data_len = 0;

        p_instaburst->p_next_alloc = &p_instaburst->p_alloc_packet->data[0];
        return true;
    }
    else
    {
        p_instaburst->p_alloc_buf    = NULL;
        p_instaburst->p_alloc_packet = NULL;
        p_instaburst->p_next_alloc   = NULL;
        return false;
    }
}

static bool order_tx(instaburst_tx_t * p_instaburst)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_ORDER_TX);

    bool did_order = false;

    NRF_MESH_ASSERT(p_instaburst->p_tx_buf != NULL);

    adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) p_instaburst->p_tx_buf->packet;

    /* Send packets that are short enough on the regular advertisement channels. */
    if (should_transmit_as_regular_packet(p_event))
    {
        if (!p_instaburst->broadcast.active)
        {
            p_instaburst->broadcast.params.p_packet = regular_packet_create((adv_ext_tx_packet_t *) p_event->packet_data);
            NRF_MESH_ERROR_CHECK(broadcast_send(&p_instaburst->broadcast));
            did_order = true;

#ifdef INSTABURST_TX_DEBUG
            p_instaburst->debug.tx_regular_packet++;
#endif
        }
    }
    else
    {
        DEBUG_PIN_TOGGLE_NTIMES(DEBUG_PIN_INSTABURST_TX_BUFFER, p_event->params.packet_count);
        if (adv_ext_tx(&p_instaburst->adv_ext_tx, p_event) == NRF_SUCCESS)
        {
            did_order = true;

#ifdef INSTABURST_TX_DEBUG
            p_instaburst->debug.tx_adv_ext++;
#endif
        }
    }

    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_ORDER_TX);
    return did_order;
}

static void tx_timeout(uint32_t timestamp, void * p_context)
{
    instaburst_tx_t * p_instaburst = p_context;
    instaburst_tx_finalize(p_instaburst);

    bool did_order = false;
    UNUSED_VARIABLE(did_order);

    if (p_instaburst->p_tx_buf == NULL &&
        packet_buffer_pop(&p_instaburst->packet_buffer, &p_instaburst->p_tx_buf) == NRF_SUCCESS)
    {
        did_order = order_tx(p_instaburst);
    }

#ifdef INSTABURST_TX_DEBUG
    if (!did_order)
    {
        p_instaburst->debug.tx_skipped++;
    }
#endif

    p_instaburst->timer_event.interval = randomized_tx_interval_get(p_instaburst);
}

static void tx_complete_event(void * p_context)
{
    instaburst_tx_t * p_instaburst = p_context;
    adv_ext_tx_event_t * p_event   = (adv_ext_tx_event_t *) p_instaburst->p_tx_buf->packet;

    if (p_instaburst->config.callback)
    {
        for (uint32_t i = 0; i < p_event->params.token_count; ++i)
        {
            nrf_mesh_tx_token_t * p_token = tx_buffer_token_get(p_instaburst->p_tx_buf, i);
            p_instaburst->config.callback(p_instaburst, *p_token, p_instaburst->prev_tx_timestamp);
        }
    }

    packet_buffer_free(&p_instaburst->packet_buffer, p_instaburst->p_tx_buf);
    p_instaburst->p_tx_buf = NULL;
}

static void adv_ext_tx_callback(adv_ext_tx_t * p_tx, const adv_ext_tx_event_t * p_tx_event, timestamp_t timestamp)
{
    instaburst_tx_t * p_instaburst = PARENT_BY_FIELD_GET(instaburst_tx_t, adv_ext_tx, p_tx);
    p_instaburst->prev_tx_timestamp = timestamp;
    NRF_MESH_ASSERT(p_instaburst->p_tx_buf->packet == (const uint8_t *) p_tx_event);
    NRF_MESH_ERROR_CHECK(bearer_event_sequential_post(&p_instaburst->tx_complete_event));
}

static void broadcast_tx_callback(broadcast_params_t * p_broadcast, timestamp_t timestamp)
{
    instaburst_tx_t * p_instaburst = PARENT_BY_FIELD_GET(instaburst_tx_t, broadcast.params, p_broadcast);
    p_instaburst->prev_tx_timestamp = timestamp;
    NRF_MESH_ASSERT(p_instaburst->p_tx_buf->packet != NULL);
    NRF_MESH_ERROR_CHECK(bearer_event_sequential_post(&p_instaburst->tx_complete_event));
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void instaburst_tx_init(uint32_t lfclk_ppm)
{
    advertiser_address_default_get(&m_ble_gap_addr);
    adv_ext_tx_init(lfclk_ppm);
    rand_prng_seed(&m_rand);
}

void instaburst_tx_instance_init(instaburst_tx_t * p_instaburst,
                                 const instaburst_tx_config_t * p_config,
                                 uint8_t * p_packet_buffer,
                                 uint32_t packet_buffer_size)
{
    NRF_MESH_ASSERT(p_instaburst != NULL);
    NRF_MESH_ASSERT(p_config != NULL);
    NRF_MESH_ASSERT(p_packet_buffer != NULL);
    NRF_MESH_ASSERT(p_config->p_channels != NULL);
    NRF_MESH_ASSERT(p_config->set_id <= INSTABURST_SET_ID_MAX);
    NRF_MESH_ASSERT(packet_buffer_size >= INSTABURST_TX_BUFFER_MIN_SIZE);

    for (uint32_t i = 0; i < p_config->channel_count; ++i)
    {
        NRF_MESH_ASSERT(p_config->p_channels[i] <= INSTABURST_CHANNEL_INDEX_MAX);
    }

    packet_buffer_init(&p_instaburst->packet_buffer, p_packet_buffer, packet_buffer_size);

    p_instaburst->config = *p_config;

    p_instaburst->timer_event.cb = tx_timeout;
    p_instaburst->timer_event.p_context = p_instaburst;
    p_instaburst->timer_event.interval = 0;

    p_instaburst->broadcast.params.access_address = BEARER_ACCESS_ADDR_DEFAULT;
    p_instaburst->broadcast.params.channel_count  = ARRAY_SIZE(m_adv_channels);
    p_instaburst->broadcast.params.p_channels     = m_adv_channels;
    p_instaburst->broadcast.params.tx_complete_cb = broadcast_tx_callback;
    p_instaburst->broadcast.params.radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE;
    p_instaburst->broadcast.params.radio_config.radio_mode     = RADIO_MODE_BLE_1MBIT;
    p_instaburst->broadcast.params.radio_config.tx_power       = p_config->tx_power;

    bearer_event_sequential_add(&p_instaburst->tx_complete_event, tx_complete_event, p_instaburst);

    adv_ext_tx_config_t tx_config;
    tx_config.radio_config.payload_maxlen = ADV_EXT_PACKET_LEN_MAX;
    tx_config.radio_config.radio_mode     = p_config->radio_mode;
    tx_config.radio_config.tx_power       = p_config->tx_power;
    tx_config.callback                    = adv_ext_tx_callback;
    adv_ext_tx_instance_init(&p_instaburst->adv_ext_tx, &tx_config);
}

void instaburst_tx_enable(instaburst_tx_t * p_instaburst)
{
    NRF_MESH_ASSERT(p_instaburst != NULL);
    NRF_MESH_ASSERT(!timer_sch_is_scheduled(&p_instaburst->timer_event));
    /* Randomize the entire interval, as if we're at some arbitrary place in the middle of two
     * transmissions. Minimizes the chance of collisions in case of synchronized enabling. */
    p_instaburst->timer_event.timestamp =
        timer_now() + rand_prng_get(&m_rand) % (MS_TO_US(p_instaburst->config.interval_ms) +
                                                INSTABURST_ADV_INT_RANDOMIZATION_MAX);
    timer_sch_schedule(&p_instaburst->timer_event);
}

void instaburst_tx_disable(instaburst_tx_t * p_instaburst)
{
    timer_sch_abort(&p_instaburst->timer_event);
}

bool instaburst_tx_is_enabled(const instaburst_tx_t * p_instaburst)
{
    return timer_sch_is_scheduled(&p_instaburst->timer_event);
}

uint8_t * instaburst_tx_buffer_alloc(instaburst_tx_t * p_instaburst, uint32_t data_len, nrf_mesh_tx_token_t tx_token)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_TX_BUFFER);
    NRF_MESH_ASSERT(p_instaburst != NULL);
    NRF_MESH_ASSERT(data_len > 0 && data_len <= ADV_EXT_TX_PAYLOAD_MAXLEN);

    bool has_buffer = true;

    if (p_instaburst->p_alloc_buf == NULL)
    {
        has_buffer = false;
    }
    else
    {
        NRF_MESH_ASSERT(p_instaburst->p_next_alloc != NULL);
        NRF_MESH_ASSERT(p_instaburst->p_alloc_packet != NULL);

        /* Only one alloc->commit at a time. */
        NRF_MESH_ASSERT(p_instaburst->p_next_alloc == buffer_reserve_pointer_get(p_instaburst));

        adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) p_instaburst->p_alloc_buf->packet;

        uint32_t new_len = p_instaburst->p_alloc_packet->data_len + data_len;
        nrf_mesh_tx_token_t * p_token =
            tx_buffer_token_get(p_instaburst->p_alloc_buf, p_event->params.token_count);

        /* As tokens are stored at the end of the buffer in reverse order, we'll need to abort once
         * the packet data and the tokens make contact. */
        bool data_and_token_overlap =
            (&p_instaburst->p_alloc_packet->data[sizeof(adv_ext_tx_packet_t) + new_len] >=
             (uint8_t *) p_token);

        if (new_len > ADV_EXT_TX_PAYLOAD_MAXLEN || data_and_token_overlap)
        {
            /* Can't fit the data in this packet, need to allocate a new one.  */
            if (p_event->params.packet_count == ADV_EXT_TX_CHAIN_MAX_COUNT || data_and_token_overlap)
            {
                /* Can't fit another packet in this buffer. */
                if (!instaburst_tx_finalize(p_instaburst))
                {
#ifdef INSTABURST_TX_DEBUG
                    p_instaburst->debug.failed_allocs++;
#endif
                    return NULL;
                }
                has_buffer = false;
            }
            else
            {
                /* Allocate a new packet in the buffer, right after the previous. */
                p_instaburst->p_alloc_packet =
                    (adv_ext_tx_packet_t *) adv_ext_tx_packet_next_get(p_instaburst->p_alloc_packet);
                p_instaburst->p_alloc_packet->data_len = 0;
                p_instaburst->p_next_alloc = p_instaburst->p_alloc_packet->data;
                p_event->params.packet_count++;
            }
        }
    }
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_TX_BUFFER);

    if (!has_buffer)
    {
        if (!reserve_buffer(p_instaburst))
        {
#ifdef INSTABURST_TX_DEBUG
            p_instaburst->debug.failed_allocs++;
#endif
            return NULL;
        }
    }

    /* Set token */
    adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) p_instaburst->p_alloc_buf->packet;
    nrf_mesh_tx_token_t * p_token =
        tx_buffer_token_get(p_instaburst->p_alloc_buf, p_event->params.token_count++);
    *p_token = tx_token;

    p_instaburst->p_next_alloc += data_len;
    return buffer_reserve_pointer_get(p_instaburst);
}

void instaburst_tx_buffer_commit(instaburst_tx_t * p_instaburst, const uint8_t * p_buffer)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_TX_BUFFER);

    NRF_MESH_ASSERT(p_buffer);
    NRF_MESH_ASSERT(p_instaburst);
    NRF_MESH_ASSERT(p_instaburst->p_alloc_packet);

    NRF_MESH_ASSERT(p_buffer == buffer_reserve_pointer_get(p_instaburst));

    p_instaburst->p_alloc_packet->data_len = p_instaburst->p_next_alloc - &p_instaburst->p_alloc_packet->data[0];

    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_TX_BUFFER);
}

void instaburst_tx_buffer_discard(instaburst_tx_t * p_instaburst, const uint8_t * p_buffer)
{

    NRF_MESH_ASSERT(p_buffer);
    NRF_MESH_ASSERT(p_instaburst);
    NRF_MESH_ASSERT(p_instaburst->p_alloc_packet);
    NRF_MESH_ASSERT(p_instaburst->p_alloc_buf);

    NRF_MESH_ASSERT(p_buffer == buffer_reserve_pointer_get(p_instaburst));

    adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) p_instaburst->p_alloc_buf->packet;
    p_event->params.token_count--;

    p_instaburst->p_next_alloc = (uint8_t *) p_buffer;
}

bool instaburst_tx_finalize(instaburst_tx_t * p_instaburst)
{
    NRF_MESH_ASSERT(p_instaburst != NULL);
    adv_ext_tx_event_t * p_event = (adv_ext_tx_event_t *) &p_instaburst->p_alloc_buf->packet[0];

    if (!m_buffer_lock &&
        p_instaburst->p_alloc_buf &&
        p_instaburst->p_alloc_packet &&
        p_instaburst->p_next_alloc &&
        p_instaburst->p_next_alloc != &((adv_ext_tx_packet_t *) p_event->packet_data)->data[0] &&
        p_instaburst->p_next_alloc == buffer_reserve_pointer_get(p_instaburst))
    {
        event_id_generate(p_instaburst->config.set_id, p_event);

        /* Shouldn't have a packet with no data. */
        NRF_MESH_ASSERT(&p_instaburst->p_alloc_packet->data_len > 0);

        /* The guard at the top should have prevented 0-packet events. */
        NRF_MESH_ASSERT(p_event->params.packet_count > 0);

        packet_buffer_commit(&p_instaburst->packet_buffer,
                             p_instaburst->p_alloc_buf,
                             p_instaburst->p_alloc_buf->size);
        p_instaburst->p_alloc_buf = NULL;
        p_instaburst->p_next_alloc = NULL;
        p_instaburst->p_alloc_packet = NULL;
        return true;
    }
    else
    {
        return false;
    }
}

void instaburst_tx_buffer_lock(bool lock)
{
    /* Keep a count to allow multiple locks */
    static uint32_t s_count = 0;
    if (lock)
    {
        s_count++;
        m_buffer_lock = true;
    }
    else
    {
        NRF_MESH_ASSERT(s_count > 0);
        if (--s_count == 0)
        {
            m_buffer_lock = false;
        }
    }
}

void instaburst_tx_interval_set(instaburst_tx_t * p_instaburst, uint32_t interval_ms)
{
    NRF_MESH_ASSERT(p_instaburst != NULL);
    NRF_MESH_ASSERT(interval_ms >= BEARER_ADV_INT_MIN_MS);
    NRF_MESH_ASSERT(interval_ms <= BEARER_ADV_INT_MAX_MS);

    p_instaburst->config.interval_ms = interval_ms;
    if (timer_sch_is_scheduled(&p_instaburst->timer_event))
    {
        timer_sch_reschedule(&p_instaburst->timer_event,
                             timer_now() + randomized_tx_interval_get(p_instaburst));
    }
}

void instaburst_tx_tx_power_set(instaburst_tx_t * p_instaburst, radio_tx_power_t tx_power)
{
    NRF_MESH_ASSERT(p_instaburst != NULL);

    p_instaburst->config.tx_power = tx_power;
    p_instaburst->broadcast.params.radio_config.tx_power = tx_power;
    p_instaburst->adv_ext_tx.config.radio_config.tx_power = tx_power;
}

