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

#include "scanner.h"

#include <unity.h>
#include <cmock.h>

#include "utils.h"
#include "test_assert.h"

#include "bearer_event_mock.h"
#include "filter_engine_mock.h"
#include "nrf_mesh_cmsis_mock_mock.h"
#include "packet_buffer_mock.h"
#include "radio_config_mock.h"
#include "timeslot_timer_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "timeslot_mock.h"
#include "mesh_pa_lna_internal_mock.h"
#include "bearer_handler_mock.h"

/* Include module to be tested (to get access to internal state) */
#include "../../bearer/src/scanner.c"

/* Initialize the RADIO peripheral, it will be externed by the headers. */
NRF_RADIO_Type * NRF_RADIO;
static NRF_RADIO_Type m_radio;
NRF_PPI_Type * NRF_PPI;
static NRF_PPI_Type m_ppi;
NRF_TIMER_Type * NRF_TIMER0;
static NRF_TIMER_Type m_timer0;
NRF_TIMER_Type * NRF_TIMER2;
static NRF_TIMER_Type m_timer2;

#define TIME_NOW                (123)
#define RSSISAMPLE_VALUE        (50)
#define RXMATCH_VALUE           (5)
#define DATAWHITEIV_VALUE       (37)
#define TIME_UNTIL_END_EVENT    (100)
#define BEARER_EVENT_FLAG       (7)
#define LNA_TIMER_INDEX         (1)

static int packet_buffer_free_callback_cnt = 0;

void test_init(void);
void test_enable_CONTINUOUS(void);
void test_disable(void);
void test_radio_start_IN_WINDOW(void);
void test_timer_scan_window_start_callback_NO_CHANNEL_INDEX_WRAP(void);
void test_radio_irq_handler_AFTER_WINDOW_START(void);
void test_timer_scan_window_end_callback(void);

/******** Helper functions ********/
static void scanner_init_helper(void)
{
    test_init();

    m_scanner.packet_buffer.size = SCANNER_BUFFER_SIZE;
    m_scanner.packet_buffer.head = 0;
    m_scanner.packet_buffer.tail = 0;
    m_scanner.packet_buffer.buffer = (uint8_t *) m_scanner.packet_buffer_data;
}

static void scanner_enable_helper(void)
{
    test_enable_CONTINUOUS();
}

static void scanner_disable_helper(void)
{
    test_disable();
}

static void scanner_start_helper(void)
{
    test_radio_start_IN_WINDOW();
    NRF_RADIO->STATE = RADIO_STATE_STATE_RxIdle;
}

static void scanner_window_start_helper(void)
{
    test_timer_scan_window_start_callback_NO_CHANNEL_INDEX_WRAP();
}

static void scanner_window_started_helper(void)
{
    test_radio_irq_handler_AFTER_WINDOW_START();
    NRF_RADIO->STATE = RADIO_STATE_STATE_Rx;
}

static void scanner_window_end_helper(void)
{
    test_timer_scan_window_end_callback();
}

static void scanner_reset(void)
{
    memset(&m_scanner, 0, sizeof(m_scanner));
    m_scanner.window_state = SCAN_WINDOW_STATE_ON;
}

static bool scanner_packet_process_callback(void)
{
    return true;
}

static void expect_lna_setup(void)
{
    /* Setup LNA with IRQ to turn it off */
    NRF_TIMER2->CC[SCANNER_TIMER_INDEX_LNA_SETUP] = 10;
    mesh_lna_setup_start_Expect(NRF_TIMER2->CC[SCANNER_TIMER_INDEX_LNA_SETUP] + LNA_SETUP_OVERHEAD_US, SCANNER_TIMER_INDEX_LNA_SETUP);
    mesh_pa_lna_setup_stop_Expect();
    NVIC_ClearPendingIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
}
/******** CUnit callbacks ********/
void setUp(void)
{
    packet_buffer_mock_Init();
    timeslot_timer_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    radio_config_mock_Init();
    timeslot_mock_Init();
    nrf_mesh_cmsis_mock_mock_Init();
    filter_engine_mock_Init();
    mesh_pa_lna_internal_mock_Init();
    bearer_handler_mock_Init();

    memset(&m_radio,  0, sizeof(NRF_RADIO_Type));
    memset(&m_ppi,    0, sizeof(NRF_PPI_Type));
    memset(&m_timer2, 0, sizeof(NRF_TIMER_Type));

    NRF_RADIO           = (NRF_RADIO_Type*) &m_radio;
    NRF_PPI             = (NRF_PPI_Type*) &m_ppi;
    NRF_TIMER0          = (NRF_TIMER_Type*) &m_timer0;
    NRF_TIMER2          = (NRF_TIMER_Type*) &m_timer2;

    packet_buffer_free_callback_cnt = 0;
}

void tearDown(void)
{
    packet_buffer_mock_Verify();
    packet_buffer_mock_Destroy();
    timeslot_timer_mock_Verify();
    timeslot_timer_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    radio_config_mock_Verify();
    radio_config_mock_Destroy();
    timeslot_mock_Verify();
    timeslot_mock_Destroy();
    nrf_mesh_cmsis_mock_mock_Verify();
    nrf_mesh_cmsis_mock_mock_Destroy();
    filter_engine_mock_Verify();
    filter_engine_mock_Destroy();
    mesh_pa_lna_internal_mock_Verify();
    mesh_pa_lna_internal_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
    scanner_reset();
}

/******** Tests ********/
void test_init(void)
{
    packet_buffer_init_Expect(&m_scanner.packet_buffer,
                              m_scanner.packet_buffer_data,
                              SCANNER_BUFFER_SIZE);
    bearer_event_flag_add_ExpectAndReturn(scanner_packet_process_callback, BEARER_EVENT_FLAG);
    scanner_init(scanner_packet_process_callback);
    TEST_ASSERT_EQUAL(SCANNER_STATE_IDLE, m_scanner.state);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL(BEARER_EVENT_FLAG, m_scanner.nrf_mesh_process_flag);
}

void test_enable_CONTINUOUS(void)
{
    scanner_init_helper();
    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    bearer_handler_wake_up_Expect();
    scanner_enable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_RUNNING, m_scanner.state);
}

void test_enable_NOT_CONTINUOUS(void)
{
    /* Test behavior on successful enable operation when not in continuous mode */
    scanner_init_helper();
    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_end,
                                TIME_NOW + MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS));
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    scanner_config_scan_time_set(MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS) * 2,
                                 MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS));
    bearer_handler_wake_up_Expect();
    scanner_enable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_RUNNING, m_scanner.state);

    /* Test enabling when already enabled */
    scanner_enable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_RUNNING, m_scanner.state);
}

void test_disable(void)
{
    /* Test when enabled, without radio context */
    scanner_enable_helper();
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_abort_Expect(&m_scanner.timer_window_start);
    scanner_disable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_IDLE, m_scanner.state);

    /* Test after being disabled */
    scanner_disable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_IDLE, m_scanner.state);

    /* Test when enabled, with radio context */
    scanner_reset();
    scanner_enable_helper();
    m_scanner.has_radio_context = true;
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_abort_Expect(&m_scanner.timer_window_start);
    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);
    scanner_disable();
    TEST_ASSERT_EQUAL(SCANNER_STATE_IDLE, m_scanner.state);
}

void test_is_enabled(void)
{
    /* Check return value when not initialized */
    TEST_ASSERT_EQUAL(false, scanner_is_enabled());

    /* Check return value when not enabled */
    scanner_init_helper();
    TEST_ASSERT_EQUAL(false, scanner_is_enabled());

    /* Check return value when enabled */
    scanner_reset();
    scanner_enable_helper();
    TEST_ASSERT_EQUAL(true, scanner_is_enabled());
}

void test_rx(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_init_helper();

    /* Check behavior when packet buffer does not return a packet */
    packet_buffer_pop_ExpectAndReturn(&m_scanner.packet_buffer, NULL, NRF_ERROR_NOT_FOUND);
    packet_buffer_pop_IgnoreArg_pp_packet();
    TEST_ASSERT_NULL(scanner_rx());

    /* Check behavior when packet buffer returns a packet */
    packet_buffer_pop_ExpectAndReturn(&m_scanner.packet_buffer, NULL, NRF_SUCCESS);
    packet_buffer_pop_IgnoreArg_pp_packet();
    packet_buffer_pop_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);
    fen_filters_apply_ExpectAndReturn(FILTER_TYPE_POST_PROC, (scanner_packet_t *)p_packet_buffer_packet->packet, false);
    TEST_ASSERT_EQUAL_PTR(p_packet_buffer_packet->packet, scanner_rx());
}

void test_packet_release(void)
{
    packet_buffer_packet_t packet_buffer_packet;

    scanner_init_helper();

    /* Test when not waiting for memory */
    packet_buffer_free_Expect(&m_scanner.packet_buffer, &packet_buffer_packet);
    scanner_packet_release((const scanner_packet_t *)&packet_buffer_packet.packet);

    /* Test when waiting for memory (and have radio context) */
    m_scanner.waiting_for_memory = true;
    m_scanner.has_radio_context = true;
    packet_buffer_free_Expect(&m_scanner.packet_buffer, &packet_buffer_packet);
    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);
    scanner_packet_release((const scanner_packet_t *)&packet_buffer_packet.packet);
}

void test_config_radio_config_set(void)
{
    scanner_config_radio_mode_set(RADIO_MODE_BLE_1MBIT);
    TEST_ASSERT_EQUAL_UINT8(RADIO_MODE_BLE_1MBIT, m_scanner.config.radio_config.radio_mode);
}

void test_config_scan_time_set(void)
{
    uint32_t scan_interval_us        = MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS);
    uint32_t scan_interval_us_double = MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS) * 2;
    uint32_t scan_window_us          = MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS);

    /* No timers should be scheduled when not enabled */
    scanner_init_helper();
    scanner_config_scan_time_set(scan_interval_us_double, scan_window_us);

    /* When enabled in continuous mode, window start timer shall be scheduled */
    scanner_reset();
    scanner_enable_helper();
    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_end,
                                TIME_NOW + MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS));
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    scanner_config_scan_time_set(scan_interval_us_double, scan_window_us);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us_double, m_scanner.config.scan_interval_us);
    TEST_ASSERT_EQUAL_UINT32(scan_window_us, m_scanner.config.scan_window_us);
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us_double, m_scanner.timer_window_end.interval);
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us_double, m_scanner.timer_window_start.interval);

    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    scanner_config_scan_time_set(scan_interval_us, scan_window_us);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us, m_scanner.config.scan_interval_us);
    TEST_ASSERT_EQUAL_UINT32(scan_window_us, m_scanner.config.scan_window_us);
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us, m_scanner.timer_window_end.interval);
    TEST_ASSERT_EQUAL_UINT32(scan_interval_us, m_scanner.timer_window_start.interval);

    /* Test invalid values */
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_scan_time_set(scan_interval_us, scan_interval_us + 1));
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_scan_time_set(MS_TO_US(BEARER_SCAN_INT_MAX_MS) + 1, scan_window_us));
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_scan_time_set(scan_interval_us, MS_TO_US(BEARER_SCAN_WIN_MIN_MS) - 1));
}

void test_config_channels_set(void)
{
    uint8_t valid_channels[] = {37, 38, 39};
    uint8_t invalid_channels[] = {36, 38, 39};
    uint8_t too_many_channels[] = {37, 38, 39, 39};

    scanner_config_channels_set(valid_channels, ARRAY_SIZE(valid_channels));

    /* Verify resulting state */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(valid_channels,
                                  m_scanner.config.channels,
                                  ARRAY_SIZE(valid_channels));
    TEST_ASSERT_EQUAL_UINT8(ARRAY_SIZE(valid_channels), m_scanner.config.channel_count);
    TEST_ASSERT_EQUAL_UINT8(0, m_scanner.channel_index);

    /* Test invalid values */
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_channels_set(invalid_channels, ARRAY_SIZE(invalid_channels)));
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_channels_set(too_many_channels, ARRAY_SIZE(too_many_channels)));
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_channels_set(valid_channels, 0));
}

void test_config_access_addresses_set(void)
{
    uint32_t access_addresses[] = {1, 2, 3, 4, 5, 6, 7, 8};

    scanner_config_access_addresses_set(access_addresses, ARRAY_SIZE(access_addresses));

    /* Verify resulting state */
    TEST_ASSERT_EQUAL_UINT32_ARRAY(access_addresses,
                                   m_scanner.config.access_addresses,
                                   ARRAY_SIZE(access_addresses));

    /* Test invalid values */
    TEST_NRF_MESH_ASSERT_EXPECT(
        scanner_config_access_addresses_set(access_addresses, ARRAY_SIZE(access_addresses) + 1));
    TEST_NRF_MESH_ASSERT_EXPECT(scanner_config_access_addresses_set(access_addresses, 0));
}

void test_config_reset(void)
{
    uint8_t channels[] = SCANNER_CHANNELS_DEFAULT;
    uint32_t access_addresses[] =
    {
        BEARER_ACCESS_ADDR_DEFAULT,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID,
        SCANNER_ACCESS_ADDR_INVALID
    };

    /* When resetting when enabled in continuous mode, window start timer shall be scheduled */
    scanner_enable_helper();
    scanner_config_radio_mode_set(RADIO_MODE_NRF_2MBIT);
    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    scanner_config_reset();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(channels,
                                  m_scanner.config.channels,
                                  ARRAY_SIZE(channels));
    TEST_ASSERT_EQUAL_UINT8(ARRAY_SIZE(channels),
                            m_scanner.config.channel_count);
    TEST_ASSERT_EQUAL_UINT8(0, m_scanner.channel_index);
    TEST_ASSERT_EQUAL_UINT32_ARRAY(access_addresses,
                                   m_scanner.config.access_addresses,
                                   ARRAY_SIZE(access_addresses));
    TEST_ASSERT_EQUAL(RADIO_MODE_BLE_1MBIT, m_scanner.config.radio_config.radio_mode);
    TEST_ASSERT_EQUAL_UINT8(RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE,
                            m_scanner.config.radio_config.payload_maxlen);
    TEST_ASSERT_EQUAL_UINT32(MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS),
                             m_scanner.config.scan_interval_us);
    TEST_ASSERT_EQUAL_UINT32(MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS),
                             m_scanner.config.scan_window_us);
    TEST_ASSERT_EQUAL_UINT32(MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS),
                             m_scanner.timer_window_end.interval);
    TEST_ASSERT_EQUAL_UINT32(MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS),
                             m_scanner.timer_window_start.interval);
}

void test_radio_start_NOT_IN_WINDOW(void)
{
    scanner_enable_helper();
    m_radio.EVENTS_ADDRESS = 1;
    m_scanner.window_state = SCAN_WINDOW_STATE_OFF;
    radio_config_reset_Expect();
    radio_config_config_Expect(&m_scanner.config.radio_config);
    radio_config_access_addr_set_Expect(BEARER_ACCESS_ADDR_DEFAULT, 0);

    scanner_radio_start(TIME_NOW);

    /* Verify resulting state */
    TEST_ASSERT_NOT_EQUAL(0, m_radio.RXADDRESSES);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_radio.INTENSET);
    TEST_ASSERT_EQUAL(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
                      m_radio.SHORTS);
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_ADDRESS);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_RADIO->EVENTS_ADDRESS,
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].EEP);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_TIMER0->TASKS_CAPTURE[SCANNER_TIMER_INDEX_TIMESTAMP],
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].TEP);
    TEST_ASSERT_EQUAL(1 << (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO), m_ppi.CHENSET);
    TEST_ASSERT_EQUAL(0, m_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(0, m_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(false, m_scanner.waiting_for_memory);
    TEST_ASSERT_NULL(m_scanner.p_buffer_packet);
}

void test_radio_start_IN_WINDOW(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_enable_helper();
    m_radio.EVENTS_ADDRESS = 1;
    radio_config_reset_Expect();
    radio_config_config_Expect(&m_scanner.config.radio_config);
    radio_config_access_addr_set_Expect(BEARER_ACCESS_ADDR_DEFAULT, 0);
    radio_config_channel_set_Expect(m_scanner.config.channels[0]);
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t), NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);

    expect_lna_setup();

    scanner_radio_start(TIME_NOW);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(true, m_scanner.has_radio_context);
    TEST_ASSERT_NOT_EQUAL(0, m_radio.RXADDRESSES);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_radio.INTENSET);
    TEST_ASSERT_EQUAL(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
                      m_radio.SHORTS);
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_ADDRESS);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_RADIO->EVENTS_ADDRESS,
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].EEP);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_TIMER0->TASKS_CAPTURE[SCANNER_TIMER_INDEX_TIMESTAMP],
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].TEP);
    TEST_ASSERT_EQUAL(1 << (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO), m_ppi.CHENSET);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_RXEN);
    TEST_ASSERT_NOT_EQUAL(0, m_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(false, m_scanner.waiting_for_memory);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL_HEX32((1 << (TIMER_INTENSET_COMPARE0_Pos + LNA_TIMER_INDEX)), NRF_TIMER2->INTENSET);
}

void test_radio_start_IN_WINDOW_BUFFER_RESERVE_FAILED(void)
{
    scanner_enable_helper();
    m_radio.EVENTS_ADDRESS = 1;
    radio_config_reset_Expect();
    radio_config_config_Expect(&m_scanner.config.radio_config);
    radio_config_access_addr_set_Expect(BEARER_ACCESS_ADDR_DEFAULT, 0);
    radio_config_channel_set_Expect(m_scanner.config.channels[0]);
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t), NRF_ERROR_NO_MEM);

    scanner_radio_start(TIME_NOW);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(true, m_scanner.has_radio_context);
    TEST_ASSERT_NOT_EQUAL(0, m_radio.RXADDRESSES);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_radio.INTENSET);
    TEST_ASSERT_EQUAL(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
                      m_radio.SHORTS);
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_ADDRESS);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_RADIO->EVENTS_ADDRESS,
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].EEP);
    TEST_ASSERT_EQUAL((uint32_t) &NRF_TIMER0->TASKS_CAPTURE[SCANNER_TIMER_INDEX_TIMESTAMP],
                      m_ppi.CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].TEP);
    TEST_ASSERT_EQUAL(1 << (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO), m_ppi.CHENSET);
    TEST_ASSERT_EQUAL(0, m_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(0, m_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(true, m_scanner.waiting_for_memory);
}

void test_radio_stop(void)
{
    scanner_start_helper();

    NRF_RADIO->EVENTS_END = 1;      /* Set END event flag to test that it is cleared */
    packet_buffer_free_Expect(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);

    mesh_pa_lna_cleanup_Expect();

    scanner_radio_stop();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(false, m_scanner.has_radio_context);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_END);
    TEST_ASSERT_NULL(m_scanner.p_buffer_packet);
}

void test_radio_irq_handler_END_EVENT_CRC_ERROR(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_start_helper();

    /* Set up radio */
    m_radio.EVENTS_END = 1;
    NRF_RADIO->STATE   = RADIO_STATE_STATE_RxIdle;

    /* Set up radio_handle_end_event() */
    packet_buffer_free_Expect(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);

    /* Set up radio_setup_next_operation() */
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t),
                                          NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_END);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(1, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
}

void test_radio_irq_handler_END_EVENT_LENGTH_ERROR(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;
    scanner_packet_t * p_scanner_packet = (scanner_packet_t *)p_packet_buffer_packet->packet;

    scanner_start_helper();

    /* Set up radio */
    m_radio.EVENTS_END = 1;
    NRF_RADIO->STATE   = RADIO_STATE_STATE_RxIdle;
    m_radio.CRCSTATUS  = RADIO_CRCSTATUS_CRCSTATUS_CRCOk;

    p_scanner_packet->packet.header.length = m_scanner.config.radio_config.payload_maxlen + 1;

    /* Set up radio_handle_end_event() */
    packet_buffer_free_Expect(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);

    /* Set up radio_setup_next_operation() */
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t),
                                          NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_END);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(1, scanner_stats_get()->length_out_of_bounds);
}

void test_radio_irq_handler_END_EVENT_SUCCESSFUL(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;
    scanner_packet_t * p_scanner_packet = (scanner_packet_t *)p_packet_buffer_packet->packet;

    scanner_window_started_helper();

    /* Set up radio */
    m_radio.EVENTS_END  = 1;
    m_radio.CRCSTATUS   = RADIO_CRCSTATUS_CRCSTATUS_CRCOk;
    m_radio.RSSISAMPLE  = RSSISAMPLE_VALUE;
    m_radio.RXMATCH     = RXMATCH_VALUE;
    m_radio.DATAWHITEIV = DATAWHITEIV_VALUE;
    m_radio.STATE       = RADIO_STATE_STATE_RxIdle;
    m_timer0.CC[TS_TIMER_INDEX_RADIO] = TIME_NOW;

    p_scanner_packet->packet.header.length = 2;

    /* Set up radio_handle_end_event() */
    fen_filters_apply_ExpectAndReturn(FILTER_TYPE_PRE_PROC, (scanner_packet_t *)m_scanner.p_buffer_packet->packet, false);
    ts_timer_to_device_time_ExpectAndReturn(TIME_NOW, TIME_NOW + 100);
    packet_buffer_commit_Expect(&m_scanner.packet_buffer,
                                m_scanner.p_buffer_packet,
                                SCANNER_PACKET_OVERHEAD +
                                    ((scanner_packet_t *)m_scanner.p_buffer_packet->packet)->
                                        packet.header.length);
    packet_buffer_commit_IgnoreArg_length();
    bearer_event_flag_set_Expect(BEARER_EVENT_FLAG);

    /* Set up radio_setup_next_operation() */
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet, sizeof(scanner_packet_t),
                                          NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_END);
    TEST_ASSERT_EQUAL_UINT32(TIME_NOW+100, p_scanner_packet->metadata.timestamp);
    TEST_ASSERT_EQUAL_UINT32(m_scanner.config.access_addresses[NRF_RADIO->RXMATCH], p_scanner_packet->metadata.access_addr);
    TEST_ASSERT_EQUAL_UINT8(DATAWHITEIV_VALUE & 0x3F, p_scanner_packet->metadata.channel);
    TEST_ASSERT_EQUAL_INT8(-RSSISAMPLE_VALUE, p_scanner_packet->metadata.rssi);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL_UINT32(1, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
}

static void packet_buffer_free_callback_AFTER_WINDOW_START(packet_buffer_t* const p_buffer, packet_buffer_packet_t* const p_packet, int cmock_num_calls)
{
    packet_buffer_free_callback_cnt++;

    TEST_ASSERT_EQUAL_PTR(&m_scanner.packet_buffer, p_buffer);
    TEST_ASSERT_EQUAL_PTR(m_scanner.p_buffer_packet, p_packet);
    TEST_ASSERT_EQUAL(0, cmock_num_calls);

    /* Simulate radio behavior when disabling the radio */
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    m_radio.STATE = RADIO_STATE_STATE_Disabled;
}

void test_radio_irq_handler_AFTER_WINDOW_START(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_window_start_helper();

    packet_buffer_free_StubWithCallback(packet_buffer_free_callback_AFTER_WINDOW_START);
    radio_config_channel_set_Expect(m_scanner.config.channels[1]);
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t), NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);
    m_radio.TASKS_RXEN = 0;

    mesh_pa_lna_cleanup_Expect();
    expect_lna_setup();

    scanner_radio_irq_handler();

    packet_buffer_free_StubWithCallback(NULL);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, packet_buffer_free_callback_cnt);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
}

void test_radio_irq_handler_AFTER_WINDOW_END(void)
{
    scanner_window_end_helper();

    packet_buffer_free_Expect(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);
    m_radio.TASKS_RXEN = 0;

    mesh_pa_lna_cleanup_Expect();

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    TEST_ASSERT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_OFF, m_scanner.window_state);
    TEST_ASSERT_EQUAL(0, m_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
}

void test_radio_irq_handler_SCANNER_DISABLE(void)
{
    scanner_disable_helper();

    mesh_pa_lna_cleanup_Expect();

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    TEST_ASSERT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
}

void test_radio_irq_handler_RADIO_INTERRUPT_INSIDE_WINDOW(void)
{
    scanner_window_started_helper();
    scanner_radio_irq_handler();
}

void test_radio_irq_handler_WINDOW_END_AFTER_RX(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;
    scanner_packet_t * p_scanner_packet = (scanner_packet_t *)p_packet_buffer_packet->packet;

    scanner_window_started_helper();

    /* End scan window */
    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);
    m_scanner.timer_window_end.cb(TIME_NOW, NULL);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_OFF, m_scanner.window_state);

    /* Set up radio */
    m_radio.EVENTS_END  = 1;
    m_radio.CRCSTATUS   = RADIO_CRCSTATUS_CRCSTATUS_CRCOk;
    m_radio.RSSISAMPLE  = RSSISAMPLE_VALUE;
    m_radio.RXMATCH     = RXMATCH_VALUE;
    m_radio.DATAWHITEIV = DATAWHITEIV_VALUE;
    m_radio.STATE       = RADIO_STATE_STATE_RxIdle;
    m_timer0.CC[TS_TIMER_INDEX_RADIO] = TIME_NOW;

    p_scanner_packet->packet.header.length = 2;

    /* Set up radio_handle_end_event() */
    fen_filters_apply_ExpectAndReturn(FILTER_TYPE_PRE_PROC, (scanner_packet_t *)m_scanner.p_buffer_packet->packet, false);
    ts_timer_to_device_time_ExpectAndReturn(TIME_NOW, TIME_NOW + 100);
    packet_buffer_commit_Expect(&m_scanner.packet_buffer,
                                m_scanner.p_buffer_packet,
                                SCANNER_PACKET_OVERHEAD +
                                    ((scanner_packet_t *)m_scanner.p_buffer_packet->packet)->
                                        packet.header.length);
    packet_buffer_commit_IgnoreArg_length();
    bearer_event_flag_set_Expect(BEARER_EVENT_FLAG);

    mesh_pa_lna_cleanup_Expect();

    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(0, m_radio.EVENTS_END);
    TEST_ASSERT_EQUAL_UINT32(TIME_NOW+100, p_scanner_packet->metadata.timestamp);
    TEST_ASSERT_EQUAL_UINT32(m_scanner.config.access_addresses[NRF_RADIO->RXMATCH], p_scanner_packet->metadata.access_addr);
    TEST_ASSERT_EQUAL_UINT8(DATAWHITEIV_VALUE & 0x3F, p_scanner_packet->metadata.channel);
    TEST_ASSERT_EQUAL_INT8(-RSSISAMPLE_VALUE, p_scanner_packet->metadata.rssi);
    TEST_ASSERT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_OFF, m_scanner.window_state);
    TEST_ASSERT_EQUAL_UINT32(1, scanner_stats_get()->successful_receives);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->crc_failures);
    TEST_ASSERT_EQUAL_UINT32(0, scanner_stats_get()->length_out_of_bounds);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
}

static void packet_buffer_free_callback_WINDOW_START_DURING_RADIO_RX(packet_buffer_t* const p_buffer, packet_buffer_packet_t* const p_packet, int cmock_num_calls)
{
    packet_buffer_free_callback_cnt++;

    TEST_ASSERT_EQUAL_PTR(&m_scanner.packet_buffer, p_buffer);
    TEST_ASSERT_EQUAL_PTR(m_scanner.p_buffer_packet, p_packet);
    TEST_ASSERT_EQUAL(1, cmock_num_calls);      /*packet_buffer_free has already been called once from scanner_window_started_helper */

    /* Simulate radio behavior when disabling the radio */
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    m_radio.STATE = RADIO_STATE_STATE_Disabled;
}

void test_radio_irq_handler_CTX_ENABLE_WINDOW_START(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_init_helper();

    radio_config_reset_Expect();
    radio_config_config_Expect(&m_scanner.config.radio_config);
    radio_config_access_addr_set_Expect(BEARER_ACCESS_ADDR_DEFAULT, 0);

    scanner_radio_start(TIME_NOW);

    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_abort_Expect(&m_scanner.timer_window_end);
    timer_sch_reschedule_Expect(&m_scanner.timer_window_start, TIME_NOW);
    bearer_handler_wake_up_Expect();
    scanner_enable();

    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);
    m_scanner.timer_window_start.cb(TIME_NOW, NULL);

    /* Set up radio_setup_next_operation() */
    radio_config_channel_set_Expect(m_scanner.config.channels[1]);
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t), NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);
    m_radio.PACKETPTR = 0;
    m_radio.TASKS_RXEN = 0;

    expect_lna_setup();
    scanner_radio_irq_handler();

    /* Verify resulting state */
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_NOT_EQUAL(0, m_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_RXEN);
}

void test_radio_irq_handler_WINDOW_START_DURING_RADIO_RX(void)
{
    packet_buffer_packet_t * p_packet_buffer_packet =
        (packet_buffer_packet_t *)m_scanner.packet_buffer_data;

    scanner_window_started_helper();

    /* Start new scan window (causing a channel change) */
    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);
    m_scanner.timer_window_start.cb(TIME_NOW, NULL);
    TEST_ASSERT_EQUAL(2, m_scanner.channel_index);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_NEXT_CHANNEL, m_scanner.window_state);

    /* Set up radio_handle_end_event() */
    packet_buffer_free_StubWithCallback(packet_buffer_free_callback_WINDOW_START_DURING_RADIO_RX);
    radio_config_channel_set_Expect(m_scanner.config.channels[2]);
    packet_buffer_reserve_ExpectAndReturn(&m_scanner.packet_buffer,
                                          &m_scanner.p_buffer_packet,
                                          sizeof(scanner_packet_t), NRF_SUCCESS);
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buffer_packet);
    packet_buffer_free_callback_cnt = 0;
    m_radio.PACKETPTR = 0;
    m_radio.TASKS_RXEN = 0;

    mesh_pa_lna_cleanup_Expect();
    expect_lna_setup();

    scanner_radio_irq_handler();

    packet_buffer_free_StubWithCallback(NULL);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, packet_buffer_free_callback_cnt);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_DISABLE);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_ON, m_scanner.window_state);
    TEST_ASSERT_NOT_NULL(m_scanner.p_buffer_packet);
    TEST_ASSERT_NOT_EQUAL(0, m_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(1, m_radio.TASKS_RXEN);
}

void test_timer_scan_window_start_callback_NO_CHANNEL_INDEX_WRAP(void)
{
    scanner_start_helper();

    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);

    m_scanner.timer_window_start.cb(TIME_NOW, NULL);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, m_scanner.channel_index);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_NEXT_CHANNEL, m_scanner.window_state);
}

void test_timer_scan_window_start_callback_CHANNEL_INDEX_WRAP(void)
{
    scanner_start_helper();

    m_scanner.channel_index = m_scanner.config.channel_count - 1;
    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);

    m_scanner.timer_window_start.cb(TIME_NOW, NULL);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(0, m_scanner.channel_index);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_NEXT_CHANNEL, m_scanner.window_state);
}

void test_timer_scan_window_end_callback(void)
{
    scanner_window_started_helper();

    NVIC_SetPendingIRQ_Expect(RADIO_IRQn);

    m_scanner.timer_window_end.cb(TIME_NOW, NULL);

    /* Verify resulting state */
    TEST_ASSERT_EQUAL(1, m_scanner.channel_index);
    TEST_ASSERT_EQUAL(SCAN_WINDOW_STATE_OFF, m_scanner.window_state);
}
