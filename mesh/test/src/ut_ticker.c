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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <unity.h>
#include <cmock.h>

#include "timer_scheduler_mock.h"
#include "timer_mock.h"

#include "ticker.h"

timer_event_t * mp_timer;

static void timer_sch_schedule_cb(timer_event_t * p_timer, int num_calls)
{
    mp_timer = p_timer;
}

void setUp(void)
{
    timer_scheduler_mock_Init();
    timer_mock_Init();
    mp_timer = NULL;

    timer_sch_schedule_StubWithCallback(timer_sch_schedule_cb);
    timer_now_IgnoreAndReturn(0);
    ticker_init();
}

void tearDown(void)
{
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

void test_ticker_get(void)
{
    TEST_ASSERT_EQUAL(0, ticker_get());

    mp_timer->cb(100, NULL);
    TEST_ASSERT_EQUAL(1, ticker_get());

    mp_timer->cb(0xFFFFFFFF, NULL);
    TEST_ASSERT_EQUAL(2, ticker_get());

    mp_timer->cb(0, NULL);
    TEST_ASSERT_EQUAL(3, ticker_get());
}
