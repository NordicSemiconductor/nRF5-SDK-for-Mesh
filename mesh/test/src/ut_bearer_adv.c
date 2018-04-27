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

#include <stdio.h>
#include <stdbool.h>

#include "unity.h"

#include "bearer_adv.h"
#include "event.h"
#include "rand.h"
#include "radio.h"
#include "timer.h"
#include "packet.h"
#include "packet_mgr.h"
#include "bearer_event.h"
#include "radio_config.h"
#include "nrf_mesh_events.h"

#include "nrf_mesh_assert.h"

static uint32_t                 m_rand;
static timestamp_t              m_time_now;
static radio_rx_cb_t            m_rx_cb;
static radio_tx_cb_t            m_tx_cb;
static radio_idle_cb_t          m_idle_cb;
static timestamp_t              m_expected_timeout;
static bool                     m_timer_order_expected;
static packet_t                 m_packet;
static packet_t*                mp_packet_expected;
static uint32_t                 m_timer_orders;
static uint32_t                 m_radio_tx_orders;
static uint32_t                 m_radio_rx_orders;
static uint32_t                 m_radio_rx_calls;
static bool                     m_radio_init_called;
static uint8_t                  m_expected_scan_ch;
static uint8_t                  m_expected_tx_ch;
static timer_sch_callback_t     m_timer_callback;
static void*                    mp_timer_context;
static advertiser_t             m_advertiser;
nrf_mesh_assertion_handler_t    m_assertion_handler;

static void rx_cb(packet_t * p_packet, bearer_t bearer, const packet_meta_t * p_meta);
void nrf_mesh_assertion_handler(uint32_t pc);

void setUp(void)
{
    m_assertion_handler = nrf_mesh_assertion_handler;
    memset(&m_advertiser, 0, sizeof(advertiser_t));
    m_advertiser.adv_int_min_ms = 100;
    m_advertiser.adv_int_max_ms = 110;
    m_advertiser.adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    m_advertiser.adv_channel_map = 0x07;
    m_time_now = 0;
    m_rand = 0;
    m_timer_orders = 0;
    m_timer_order_expected = true;
    m_radio_tx_orders = 0;
    m_radio_rx_orders = 0;
    m_radio_rx_calls = 0;
    m_radio_init_called = false;
    m_expected_tx_ch = 37;
    m_expected_scan_ch = 37;
    mp_packet_expected = &m_packet;
    packet_net_t* p_net = (packet_net_t*) &m_packet.payload[0];
    p_net->length = BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH - BLE_AD_DATA_OVERHEAD;
    p_net->ad_type = AD_TYPE_MESH;
    m_packet.header.length = BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH;
    packet_net_payload_size_set(p_net, 8);
    bearer_scan_config_t scan_config;
    scan_config.scan_channel_map = 0x07;
    scan_config.scan_int_ms = 100;
    scan_config.scan_window_ms = 100;
    scan_config.rx_cb = rx_cb;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_init(&scan_config));

    bearer_adv_advertiser_init(&m_advertiser);
}

void tearDown(void)
{

}


/********************/

/* TODO: Use CMock generated mock functions in this unit test. */
/*lint -e522 Mock functions lack side-effects. */

uint32_t rand_prng_get(prng_t* p_prng)
{
    return m_rand;
}

void rand_prng_seed(prng_t* p_prng)
{
}

void radio_mode_set(uint8_t radio_mode)
{
}

uint8_t radio_mode_get(void)
{
    return 3;
}

void radio_preemptable_cancel(void)
{
}

uint32_t radio_access_addr_get(void)
{
    return 0x8E89BED6;
}

radio_tx_power_t radio_tx_power_get(void)
{
    return RADIO_POWER_NRF_0DBM;
}

void radio_access_addr_set(uint32_t access_address)
{
}

void radio_tx_power_set(radio_tx_power_t tx_power)
{
}

void event_handle(nrf_mesh_evt_t * p_evt)
{
}

uint32_t radio_order(radio_event_t* p_evts, uint32_t* p_count)
{
    TEST_ASSERT_NOT_NULL(p_count);
    if (p_evts[0].event_type == RADIO_EVENT_TYPE_TX)
    {
        TEST_ASSERT_EQUAL(3, *p_count);
    }
    else
    {
        TEST_ASSERT_EQUAL(1, *p_count);
    }
    for (uint32_t i = 0; i < *p_count; ++i)
    {
        radio_event_t* p_evt = &p_evts[i];
        if (p_evt->event_type == RADIO_EVENT_TYPE_TX)
        {
            TEST_ASSERT_EQUAL_PTR(mp_packet_expected, p_evt->p_packet);
            TEST_ASSERT_EQUAL(m_expected_tx_ch, p_evt->channel);
            m_radio_tx_orders++;
            if (++m_expected_tx_ch == 40)
            {
                m_expected_tx_ch = 37;
            }

            m_tx_cb(p_evt->p_packet, (i == (*p_count - 1)) && p_evt->free_on_end);
        }
        else
        {
            TEST_ASSERT_EQUAL_PTR(&m_packet, p_evt->p_packet);
            TEST_ASSERT_EQUAL(m_expected_scan_ch, p_evt->channel);
            m_radio_rx_orders++;
            if (++m_expected_scan_ch == 40)
            {
                m_expected_scan_ch = 37;
            }
        }
    }
    return NRF_SUCCESS;
}

void timer_sch_schedule(timer_event_t* p_timer_evt)
{
    TEST_ASSERT(m_timer_order_expected);
    TEST_ASSERT_EQUAL(m_expected_timeout, p_timer_evt->timestamp);
    m_timer_callback = p_timer_evt->cb;
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    mp_timer_context = p_timer_evt->p_context;
    m_timer_orders++;
}

void timer_sch_abort(timer_event_t* p_timer_evt)
{
}

void timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timestamp)
{
    TEST_ASSERT(m_timer_order_expected);
    TEST_ASSERT_EQUAL(m_expected_timeout, new_timestamp);
    m_timer_callback = p_timer_evt->cb;
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    mp_timer_context = p_timer_evt->p_context;
    p_timer_evt->timestamp = new_timestamp;
    m_timer_orders++;
}

void radio_init(const radio_init_params_t * p_init_params)
{
    TEST_ASSERT_EQUAL(0x8E89BED6, p_init_params->access_address);
    m_rx_cb = p_init_params->rx_cb;
    m_tx_cb = p_init_params->tx_cb;
    m_idle_cb = p_init_params->idle_cb;

    TEST_ASSERT_FALSE(m_radio_init_called);
    m_radio_init_called = true;
}

void radio_invoke(void)
{
}

timestamp_t timer_now(void)
{
    return m_time_now;
}

uint32_t packet_mgr_alloc(packet_generic_t** pp_packet, uint16_t size)
{
    *pp_packet = &m_packet;
    return NRF_SUCCESS;
}

void packet_mgr_free(packet_generic_t* p_packet)
{
}

uint32_t sd_ble_gap_address_get(ble_gap_addr_t* p_addr)
{
    for (uint32_t i = 0; i < 6; ++i)
    {
        p_addr->addr[i] = i;
    }
    p_addr->addr[5] = 0xC0;
    p_addr->addr_type = 1;
    return NRF_SUCCESS;
}


void app_error_handler(uint32_t error, uint32_t line, const uint8_t* file)
{
    printf("APP error: %d, @%s:L%d\n", error, file, line);
    TEST_FAIL();
}

static void rx_cb(packet_t * p_packet, bearer_t bearer, const packet_meta_t * p_meta)
{
    TEST_ASSERT_EQUAL(&m_packet, p_packet);
    TEST_ASSERT_EQUAL(BEARER_ADV_RADIO, bearer);
    m_radio_rx_calls++;
}

uint32_t bearer_event_generic_post(bearer_event_callback_t cb, void* p_context)
{
    TEST_ASSERT_NOT_NULL(cb);
    cb(p_context);
    return NRF_SUCCESS;
}

void nrf_mesh_assertion_handler(uint32_t pc)
{
    char str[256];
    sprintf(str, "ASSERT AT PC %u", pc);
    TEST_FAIL_MESSAGE(str);
}

void bearer_adtype_filtering_set(bool filter){}

void bearer_adtype_add(uint8_t type){}
void bearer_adtype_remove(uint8_t type) {}

void bearer_event_critical_section_begin() {}
void bearer_event_critical_section_end() {}
/************************/
void test_bearer_adv_time_offsets(void)
{
    m_expected_timeout = 1000;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 1));
    TEST_ASSERT_TRUE(m_radio_init_called);
    TEST_ASSERT_EQUAL(0, m_radio_tx_orders);
    TEST_ASSERT_EQUAL(1, m_timer_orders);
    m_time_now = m_expected_timeout + 1;

    m_timer_order_expected = false;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;

    TEST_ASSERT_EQUAL(1, m_timer_orders);
    TEST_ASSERT_EQUAL(3, m_radio_tx_orders);

    m_expected_timeout = 101000;
    m_timer_order_expected = true;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 2));
    TEST_ASSERT_EQUAL(2, m_timer_orders);

    m_time_now = m_expected_timeout + 1;
    m_rand = 500;
    m_expected_timeout = 201500;

    m_timer_order_expected = false;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(2, m_timer_orders);
    TEST_ASSERT_EQUAL(100500, m_advertiser.internal.adv_evt.interval);
    TEST_ASSERT_EQUAL(6, m_radio_tx_orders);

    m_time_now = m_expected_timeout + 1;
    m_timer_order_expected = false;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(2, m_timer_orders);
    TEST_ASSERT_EQUAL(9, m_radio_tx_orders);

    /* time now is 201500, expecting it will schedule 102ms in the future */
    m_expected_timeout = 303500;
    m_rand = 10000 + 2000; /* rand rollover */
    m_timer_order_expected = true;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 1));
    TEST_ASSERT_EQUAL(3, m_timer_orders);

    m_time_now = m_expected_timeout;
    m_timer_order_expected = false;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(3, m_timer_orders);
    TEST_ASSERT_EQUAL(12, m_radio_tx_orders);
}

void test_bearer_adv_scanning(void)
{
    m_expected_timeout = m_time_now + 1000;
    TEST_ASSERT_TRUE(m_radio_init_called);
    TEST_ASSERT_EQUAL(0, m_radio_rx_orders);
    bearer_adv_scan_start();
    TEST_ASSERT_EQUAL(1, m_timer_orders);
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;

    TEST_ASSERT_EQUAL(1, m_radio_rx_orders);
    TEST_ASSERT_EQUAL(0, m_radio_tx_orders);
    m_expected_timeout = 1000;
    m_timer_order_expected = true;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 1));
    TEST_ASSERT_EQUAL(2, m_timer_orders);
    TEST_ASSERT_EQUAL(1, m_radio_rx_orders);
    TEST_ASSERT_EQUAL(0, m_radio_tx_orders);

    m_time_now = m_expected_timeout;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(1, m_radio_rx_orders);
    TEST_ASSERT_EQUAL(3, m_radio_tx_orders);

    /* RX propagation */
    TEST_ASSERT_EQUAL(0, m_radio_rx_calls);
    m_rx_cb((uint8_t*) &m_packet, true, 0x12345678, 0);
    TEST_ASSERT_EQUAL(1, m_radio_rx_calls);

    m_expected_scan_ch = 37;
    m_idle_cb();
    TEST_ASSERT_EQUAL(2, m_radio_rx_orders);
}

void test_bearer_adv_available(void)
{
    /* FIFO overflow and whatnot */
    TEST_ASSERT_TRUE(m_radio_init_called);

    m_timer_order_expected = true;
    m_expected_timeout = 1000;
    for (uint32_t i = 0; i < BEARER_TX_QUEUE_LENGTH; ++i)
    {
        TEST_ASSERT_TRUE(bearer_adv_available(&m_advertiser));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 1));
    }
    TEST_ASSERT_FALSE(bearer_adv_available(&m_advertiser));

    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, bearer_adv_tx(&m_advertiser, &m_packet, 1));
    TEST_ASSERT_FALSE(bearer_adv_available(&m_advertiser));
    m_time_now = m_expected_timeout;
    m_expected_timeout = 101000;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(3, m_radio_tx_orders);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_tx(&m_advertiser, &m_packet, 1));

    for (uint32_t i = 0; i < BEARER_TX_QUEUE_LENGTH; ++i)
    {
        m_time_now = m_expected_timeout;
        m_expected_timeout += 100000;
        m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
        TEST_ASSERT_EQUAL((i + 2) * 3, m_radio_tx_orders);
    }
    m_timer_order_expected = false;
    uint32_t tx_order_count = m_radio_tx_orders;
    m_timer_callback(m_time_now, mp_timer_context);
    m_advertiser.internal.adv_evt.timestamp += m_advertiser.internal.adv_evt.interval;
    TEST_ASSERT_EQUAL(m_radio_tx_orders, tx_order_count);
}

void test_address_set(void)
{
    ble_gap_addr_t addr = { .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC };

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, bearer_adv_addr_default_set(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, bearer_adv_addr_default_set(&addr));

    addr.addr[0] = 0x33;
    /* Invalid address type. */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, bearer_adv_addr_default_set(&addr));

    bearer_adv_gap_type_set(&addr);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_adv_addr_default_set(&addr));

    bearer_adv_advertiser_init(&m_advertiser);
    TEST_ASSERT_EQUAL_MEMORY(&addr, &m_advertiser.internal.ble_adv_addr, sizeof(ble_gap_addr_t));
}
