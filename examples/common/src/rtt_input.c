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

#include "rtt_input.h"
#include "nrf_mesh_config_examples.h"

#include <stdlib.h>
#include "SEGGER_RTT.h"
#include "nrf_mesh_defines.h"
#include "app_timer.h"
#include "mesh_app_utils.h"
#include "hal.h"

#if RTT_INPUT_ENABLED

static rtt_input_handler_t m_rtt_input_handler;

APP_TIMER_DEF(m_rtt_timer);

void timeout_handler(void * p_unused)
{
    if (m_rtt_input_handler != NULL)
    {
        for (;;)
        {
            int key = SEGGER_RTT_GetKey();
            if (key < 0)
            {
                break;
            }
            m_rtt_input_handler(key);
        }
    }
}

void rtt_input_enable(rtt_input_handler_t rtt_input_handler, uint32_t poll_period_ms)
{
    m_rtt_input_handler = rtt_input_handler;

    ERROR_CHECK(app_timer_create(&m_rtt_timer, APP_TIMER_MODE_REPEATED, timeout_handler));
    ERROR_CHECK(app_timer_start(m_rtt_timer, MAX(APP_TIMER_MIN_TIMEOUT_TICKS, HAL_MS_TO_RTC_TICKS(poll_period_ms)), NULL));
}

void rtt_input_disable(void)
{
    ERROR_CHECK(app_timer_stop(m_rtt_timer));
    m_rtt_input_handler = NULL;
}

#endif /* RTT_INPUT_ENABLED */
