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

#include "timer.h"

#include <string.h>
#include <stdbool.h>
#include "cmock.h"
#include "unity.h"

#include "app_timer_mock.h"

#include "hal.h"
#include "nrf_error.h"

#define TIMER_US_TO_TICKS(US)                              \
            ((uint32_t)ROUNDED_DIV(                        \
            (US) * (uint64_t)APP_TIMER_CLOCK_FREQ,         \
            1000000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))

#define TIMER_TICKS_TO_US(TICKS)                           \
            ((uint32_t)ROUNDED_DIV(                        \
            (TICKS) * 1000000ull * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1),         \
            APP_TIMER_CLOCK_FREQ))

#define RTC_RESOLUTION        24
#define TIMER_FIRED_TAIL      7
#define TIMER_FIRED_TIMESTAMP TIMER_TICKS_TO_US((1ul << 24) | TIMER_FIRED_TAIL)
#define OVFW_TIMESTAMP        512000000ul
#define OVFW_TICKS            0x1000000ul

extern void nrf_mesh_timer_tail_handle(void);
extern void nrf_mesh_timer_ovfw_handle(void);

static NRF_RTC_Type m_dummy_timer;
NRF_RTC_Type * NRF_RTC1 = &m_dummy_timer;

static bool m_is_timer_fired;
static bool m_is_timer_init;

// function does not handle more then 1 overflow
static void timer_now_smart_mock(timestamp_t timestamp)
{
    uint32_t tikcs = TIMER_US_TO_TICKS(timestamp);

    NRF_RTC1->EVENTS_OVRFLW = tikcs >> RTC_RESOLUTION ? 1 : 0;
    NRF_RTC1->COUNTER = tikcs & APP_TIMER_MAX_CNT_VAL;
}

static void timer_normal_cb(timestamp_t timestamp)
{
    m_is_timer_fired = true;
    TEST_ASSERT_EQUAL(TIMER_FIRED_TIMESTAMP, timestamp);
}

static void timer_ovfw_and_delay_cb(timestamp_t timestamp)
{
    m_is_timer_fired = true;
    TEST_ASSERT_EQUAL(TIMER_TICKS_TO_US(OVFW_TICKS + 2), timestamp);
}

static void timer_ovfw_cb(timestamp_t timestamp)
{
    m_is_timer_fired = true;
    TEST_ASSERT_EQUAL(TIMER_TICKS_TO_US(OVFW_TICKS), timestamp);
}

bool is_app_timer_init(void)
{
    bool retuned_value = m_is_timer_init;
    m_is_timer_init = true;
    return retuned_value;
}

void setUp(void)
{
    app_timer_mock_Init();
}

void tearDown(void)
{
    app_timer_mock_Verify();
    app_timer_mock_Destroy();
}

void test_timer_init(void)
{
    app_timer_init_ExpectAndReturn(NRF_SUCCESS);
    timer_init();
    app_timer_mock_Verify();

    timer_init();
    app_timer_mock_Verify();
}

void test_timer_stop(void)
{
    NRF_RTC1->EVTENCLR = 0ul;
    NRF_RTC1->INTENCLR = 0ul;
    timer_stop();
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENCLR);
}

void test_timer_now(void)
{
    timestamp_t expected1;
    timestamp_t expected2;

    // normal case
    NRF_RTC1->EVENTS_OVRFLW = 0;
    NRF_RTC1->COUNTER = 0xFFFFFFul;

    expected1 = timer_now();

    TEST_ASSERT_EQUAL((timestamp_t)TIMER_TICKS_TO_US(0xFFFFFFul), expected1);

    // rtc has been overflowed during reading of the current timestamp
    NRF_RTC1->EVENTS_OVRFLW = 1;
    NRF_RTC1->COUNTER = 0ul;

    expected2 = timer_now();

    TEST_ASSERT_EQUAL((timestamp_t)TIMER_TICKS_TO_US((1ul << 24) | 0), expected2);
    TEST_ASSERT_TRUE(expected1 < expected2);
}

void test_timer_start(void)
{
    // time fits in timer range without overflow
    m_is_timer_fired = false;
    timer_stop();
    memset(NRF_RTC1, 0, sizeof(NRF_RTC_Type));
    timer_now_smart_mock(32);

    // RTC margin for timer_start is 3 which is equal to 92 us.
    // Set something bigger than 32 us + 92 us.
    timer_start(130, timer_normal_cb);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);
    TEST_ASSERT_EQUAL(TIMER_US_TO_TICKS(130 - 32) + NRF_RTC1->COUNTER, NRF_RTC1->CC[1]);

    timer_now_smart_mock(TIMER_FIRED_TIMESTAMP);

    nrf_mesh_timer_tail_handle();

    TEST_ASSERT_TRUE(m_is_timer_fired);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);

    m_is_timer_fired = false;
    timer_now_smart_mock(0);
    for (uint8_t i = 0; i < 10; i++)
    {
        nrf_mesh_timer_ovfw_handle();
    }
    TEST_ASSERT_FALSE(m_is_timer_fired);

    // run expired timestamp, time fits in timer range without overflow
    m_is_timer_fired = false;
    timer_stop();
    memset(NRF_RTC1, 0, sizeof(NRF_RTC_Type));
    timer_now_smart_mock(130);

    timer_start(32, timer_normal_cb);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);
    TEST_ASSERT_EQUAL(NRF_RTC1->COUNTER + 3, NRF_RTC1->CC[1]);

    timer_now_smart_mock(TIMER_FIRED_TIMESTAMP);

    nrf_mesh_timer_tail_handle();

    TEST_ASSERT_TRUE(m_is_timer_fired);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);

    m_is_timer_fired = false;
    timer_now_smart_mock(0);
    for (uint8_t i = 0; i < 10; i++)
    {
        nrf_mesh_timer_ovfw_handle();
    }
    TEST_ASSERT_FALSE(m_is_timer_fired);

    // time cause rtc overflow + delayed overflow interrupt (NRF_RTC1->COUNTER goes over tail)
    m_is_timer_fired = false;
    timer_stop();
    memset(NRF_RTC1, 0, sizeof(NRF_RTC_Type));
    timer_now_smart_mock(0);

    timer_start(OVFW_TIMESTAMP, timer_ovfw_and_delay_cb);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);

    NRF_RTC1->COUNTER = 2;
    nrf_mesh_timer_ovfw_handle();

    TEST_ASSERT_TRUE(m_is_timer_fired);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);

    m_is_timer_fired = false;
    timer_now_smart_mock(0x55);
    for (uint8_t i = 0; i < 10; i++)
    {
        nrf_mesh_timer_ovfw_handle();
    }
    TEST_ASSERT_FALSE(m_is_timer_fired);

    // time cause rtc overflow
    m_is_timer_fired = false;
    timer_stop();
    memset(NRF_RTC1, 0, sizeof(NRF_RTC_Type));
    timer_now_smart_mock(0);

    timer_start(OVFW_TIMESTAMP, timer_ovfw_cb);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_NOT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);

    nrf_mesh_timer_ovfw_handle();

    TEST_ASSERT_FALSE(m_is_timer_fired);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENSET);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->INTENSET);
    TEST_ASSERT_EQUAL(3ul, NRF_RTC1->CC[1]);

    nrf_mesh_timer_tail_handle();

    TEST_ASSERT_TRUE(m_is_timer_fired);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);
    TEST_ASSERT_EQUAL(RTC_EVTEN_COMPARE1_Msk, NRF_RTC1->EVTENCLR);

    m_is_timer_fired = false;
    timer_now_smart_mock(0x55);
    for (uint8_t i = 0; i < 10; i++)
    {
        nrf_mesh_timer_ovfw_handle();
    }
    TEST_ASSERT_FALSE(m_is_timer_fired);
}
