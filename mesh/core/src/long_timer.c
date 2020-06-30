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
#include "long_timer.h"
#include "utils.h"

#define TIMER_STEP  ((UINT32_MAX / 2) - 1)

static void timer_handler(timestamp_t timestamp, void * p_context)
{
    long_timer_t * p_timer = p_context;
    p_timer->remaining_time_us -= MIN(TIMER_STEP, p_timer->remaining_time_us);

    if (p_timer->remaining_time_us == 0)
    {
        p_timer->event.interval = 0;
        p_timer->callback(p_timer->p_context);
    }
    else
    {
        p_timer->event.interval = MIN(TIMER_STEP, p_timer->remaining_time_us);
    }
}

void lt_schedule(long_timer_t * p_timer, lt_callback_t callback, void * p_context, uint64_t delay_us)
{
    p_timer->callback = callback;
    p_timer->p_context = p_context;
    p_timer->remaining_time_us = delay_us;
    p_timer->event.p_context = p_timer;
    p_timer->event.cb = timer_handler;
    timer_sch_reschedule(&p_timer->event, timer_now() + MIN(TIMER_STEP, p_timer->remaining_time_us));
}

void lt_abort(long_timer_t * p_timer)
{
    timer_sch_abort(&p_timer->event);
}

uint64_t lt_remaining_time_get(const long_timer_t * p_timer)
{
    timestamp_t diff = timer_diff(timer_now(), p_timer->event.timestamp);

    return p_timer->remaining_time_us > diff ? p_timer->remaining_time_us - diff : 0;
}
