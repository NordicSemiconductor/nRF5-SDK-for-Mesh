/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#include <stdlib.h>
#include "SEGGER_RTT.h"
#include "nrf.h"
#include "nrf_mesh_defines.h"


#define RTT_TIMER             NRF_TIMER2
#define RTT_TIMER_IRQ_HANDLER TIMER2_IRQHandler
#define RTT_TIMER_IRQn        TIMER2_IRQn
#define RTT_TIMER_PRESCALER   9                                             /**< 31250 Hz */
#define RTT_TIMER_FREQ        (16000000ULL / (1UL << RTT_TIMER_PRESCALER))

static rtt_input_handler_t m_rtt_input_handler;


void RTT_TIMER_IRQ_HANDLER(void)
{
    if (RTT_TIMER->EVENTS_COMPARE[0])
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
        RTT_TIMER->EVENTS_COMPARE[0] = 0;
    }
}

void rtt_input_enable(rtt_input_handler_t rtt_input_handler, uint32_t poll_period_ms)
{
    m_rtt_input_handler = rtt_input_handler;

    RTT_TIMER->MODE        = TIMER_MODE_MODE_Timer;
    RTT_TIMER->BITMODE     = TIMER_BITMODE_BITMODE_16Bit;
    RTT_TIMER->PRESCALER   = RTT_TIMER_PRESCALER;
    RTT_TIMER->CC[0]       = (uint32_t)((poll_period_ms * RTT_TIMER_FREQ) / 1000);
    RTT_TIMER->SHORTS      = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    RTT_TIMER->INTENSET    = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    RTT_TIMER->TASKS_START = 1;

    NVIC_SetPriority(RTT_TIMER_IRQn, NRF_MESH_IRQ_PRIORITY_LOWEST);
    NVIC_EnableIRQ(RTT_TIMER_IRQn);
}

void rtt_input_disable(void)
{
    NVIC_DisableIRQ(RTT_TIMER_IRQn);
    RTT_TIMER->TASKS_STOP = 1;

    m_rtt_input_handler = NULL;
}
