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

#include "app_timer.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"

#include "nordic_common.h"
#include "nrf.h"

#define RTC_RESOLUTION        24
#define IS_OVERFLOW_PENDING() (NRF_RTC1->EVENTS_OVRFLW == 1)
/* Margin is required to prevent situation when written CC value is equal to COUNTER.
 * Situation with equality will cause losing interrupt for the tail counting until next overflow. */
#define PROTECTION_MARGIN_FOR_TIMER_START   3ul
#define PROTECTION_MARGIN_FOR_OVFW_HANDLER  2ul

#define TIMER_US_TO_TICKS(US)                              \
            ((uint32_t)ROUNDED_DIV(                        \
            (US) * (uint64_t)APP_TIMER_CLOCK_FREQ,         \
            1000000ull * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))

#define TIMER_TICKS_TO_US(TICKS)                                         \
            ((uint32_t)ROUNDED_DIV(                                      \
            (TICKS) * 1000000ull * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1), \
            APP_TIMER_CLOCK_FREQ))

static volatile uint32_t m_ovfw_counter;

static volatile uint32_t m_ovfw_timer_counter;
static volatile uint32_t m_tail_timer_counter;
static timer_callback_t volatile mp_cb;

extern bool is_app_timer_init(void);

void nrf_mesh_timer_tail_handle(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE1_Msk;
    NRF_RTC1->INTENCLR = RTC_EVTEN_COMPARE1_Msk;

    NRF_MESH_ASSERT(mp_cb != NULL);
    mp_cb(timer_now());
}

void nrf_mesh_timer_ovfw_handle(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    NRF_RTC1->EVENTS_OVRFLW = 0;
    m_ovfw_counter++;
    _ENABLE_IRQS(was_masked);

    if (m_ovfw_timer_counter != 0)
    {
        m_ovfw_timer_counter--;

        if (m_ovfw_timer_counter == 0)
        {
            if (m_tail_timer_counter > NRF_RTC1->COUNTER)
            {
                _DISABLE_IRQS(was_masked);
                uint32_t cnt = NRF_RTC1->COUNTER;
                NRF_RTC1->CC[1] = m_tail_timer_counter > cnt + PROTECTION_MARGIN_FOR_OVFW_HANDLER ?
                        m_tail_timer_counter : cnt + PROTECTION_MARGIN_FOR_TIMER_START;
                _ENABLE_IRQS(was_masked);
                NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE1_Msk;
                NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE1_Msk;
            }
            else
            {
                NRF_MESH_ASSERT(mp_cb != NULL);
                mp_cb(timer_now());
            }
        }
    }
}

void timer_init(void)
{
    if (!is_app_timer_init())
    {
        NRF_MESH_ERROR_CHECK(app_timer_init());
    }
}

timestamp_t timer_now(void)
{
    uint32_t was_masked;
    uint32_t ovfw_counter;
    uint32_t sample;

    _DISABLE_IRQS(was_masked);
    ovfw_counter = m_ovfw_counter;
    sample = NRF_RTC1->COUNTER;
    if (IS_OVERFLOW_PENDING())
    {
        sample = NRF_RTC1->COUNTER;
        ovfw_counter++;
    }
    _ENABLE_IRQS(was_masked);

    ovfw_counter <<= RTC_RESOLUTION;

    return TIMER_TICKS_TO_US(ovfw_counter | sample);
}

void timer_start(timestamp_t timestamp, timer_callback_t cb)
{
    volatile uint32_t was_masked = 0ul;
    NRF_MESH_ASSERT(cb != NULL);
    mp_cb = cb;

    timestamp_t time_now = timer_now();
    timestamp_t time_diff = 0ul;
    if (TIMER_OLDER_THAN(time_now, timestamp))
    {
        time_diff = timestamp - time_now;
    }

    uint32_t timeout_ticks = TIMER_US_TO_TICKS(time_diff);

    _DISABLE_IRQS(was_masked);
    uint32_t ticks_now = NRF_RTC1->COUNTER;

    timeout_ticks = MAX(PROTECTION_MARGIN_FOR_TIMER_START, timeout_ticks);
    if (timeout_ticks <= APP_TIMER_MAX_CNT_VAL - ticks_now)
    {
        NRF_RTC1->CC[1] = ticks_now + timeout_ticks;
        NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE1_Msk;
        NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE1_Msk;
        m_ovfw_timer_counter = 0;
        m_tail_timer_counter = 0;
    }
    else
    {
        timeout_ticks -= (APP_TIMER_MAX_CNT_VAL - ticks_now);
        m_ovfw_timer_counter = (timeout_ticks >> RTC_RESOLUTION) + 1;
        m_tail_timer_counter = timeout_ticks & APP_TIMER_MAX_CNT_VAL;
    }
    _ENABLE_IRQS(was_masked);
}

void timer_stop(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE1_Msk;
    NRF_RTC1->INTENCLR = RTC_EVTEN_COMPARE1_Msk;

    m_ovfw_timer_counter = 0;
    m_tail_timer_counter = 0;
    mp_cb = NULL;

#if defined(UNIT_TEST)
    m_ovfw_counter = 0;
#endif
}
