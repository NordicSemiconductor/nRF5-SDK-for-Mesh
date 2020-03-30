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
#include <stddef.h>

#include "timeslot_timer.h"

#include "bearer_event.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"

#include "nrf_soc.h"
#include "nrf.h"

#define TIMER_COMPARE_COUNT     (3)

/** Time from timeslot API starts the TIMER0 until we are sure we have had time to set all timeouts. */
#define TIMER_TS_BEGIN_MARGIN_US    (120)
/*****************************************************************************
* Static globals
*****************************************************************************/
/** Array of function pointers for callbacks for each timer. */
static timer_callback_t m_callbacks[TIMER_COMPARE_COUNT];
/** Array of PPI tasks to trigger on timeout. */
static uint32_t*        mp_ppi_tasks[TIMER_COMPARE_COUNT];
/** Time captured at the beginning of the current timeslot. */
static timestamp_t   m_ts_start_time;
/** Time captured at the end of the previous timeslot. */
static ts_timestamp_t   m_ts_end_time;
/** Timeslot currently in progress. */
static bool             m_is_in_ts;
/** Timer mutex. */
static uint32_t         m_timer_mut;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void timer_set(uint8_t timer, ts_timestamp_t timeout)
{
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET  = (1u << (TIMER_INTENSET_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = timeout;
}

/** Implement mutex lock, the SD-mut is hidden behind SVC, and cannot be used in IRQ level <= 1.
    While the mutex is locked, the timer is unable to receive timer interrupts, and the
    timers may safely be changed */
static inline void timer_mut_lock(void)
{
    _DISABLE_IRQS(m_timer_mut);
}

/** Implement mutex unlock, the SD-mut is hidden behind SVC, and cannot be used in IRQ level <= 1 */
static inline void timer_mut_unlock(void)
{
    _ENABLE_IRQS(m_timer_mut);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void ts_timer_event_handler(void)
{
    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        if (NRF_TIMER0->EVENTS_COMPARE[i] && (NRF_TIMER0->INTENSET & (1u << (TIMER_INTENCLR_COMPARE0_Pos + i))))
        {
            ts_timer_callback_t cb = m_callbacks[i];
            NRF_MESH_ASSERT(cb != NULL);
            m_callbacks[i] = NULL;
            NRF_TIMER0->INTENCLR = (1u << (TIMER_INTENCLR_COMPARE0_Pos + i));
            ts_timestamp_t time_now = ts_timer_now();
            cb(time_now);
            NRF_TIMER0->EVENTS_COMPARE[i] = 0;
        }
    }
}

uint32_t ts_timer_order_cb(uint8_t timer,
                           ts_timestamp_t time,
                           ts_timer_callback_t callback)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    timer_mut_lock();

    m_callbacks[timer] = callback;
    mp_ppi_tasks[timer] = NULL;

    if (m_is_in_ts)
    {
        timer_set(timer, time);
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t ts_timer_order_cb_ppi(uint8_t timer,
                               ts_timestamp_t time,
                               ts_timer_callback_t callback,
                               uint32_t* p_task)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    timer_mut_lock();

    m_callbacks[timer] = callback;
    mp_ppi_tasks[timer] = p_task;

    if (m_is_in_ts)
    {
        timer_set(timer, time);
        /* Setup PPI */
        NRF_PPI->CH[TS_TIMER_PPI_CH_START + timer].EEP = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
        NRF_PPI->CH[TS_TIMER_PPI_CH_START + timer].TEP = (uint32_t) p_task;
        NRF_PPI->CHENSET = (1 << (TS_TIMER_PPI_CH_START + timer));
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t ts_timer_order_ppi(uint8_t timer,
                            ts_timestamp_t time,
                            uint32_t* p_task)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    timer_mut_lock();

    m_callbacks[timer] = NULL;
    mp_ppi_tasks[timer] = p_task;

    if (m_is_in_ts)
    {
        NRF_TIMER0->INTENCLR = (1u << (TIMER_INTENSET_COMPARE0_Pos + timer));
        NRF_TIMER0->CC[timer] = time;
        NRF_TIMER0->EVENTS_COMPARE[timer] = 0;

        /* Setup PPI */
        NRF_PPI->CH[TS_TIMER_PPI_CH_START + timer].EEP = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
        NRF_PPI->CH[TS_TIMER_PPI_CH_START + timer].TEP = (uint32_t) p_task;
        NRF_PPI->CHENSET = (1 << (TS_TIMER_PPI_CH_START + timer));
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t ts_timer_abort(uint8_t timer)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (mp_ppi_tasks[timer] == NULL && m_callbacks[timer] == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    timer_mut_lock();
    if (timer < TIMER_COMPARE_COUNT)
    {
        m_callbacks[timer] = NULL;
        if (m_is_in_ts)
        {
            NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
            NRF_PPI->CHENCLR = (1 << (TS_TIMER_PPI_CH_START + timer));
        }
    }
    timer_mut_unlock();

    return NRF_SUCCESS;
}

ts_timestamp_t ts_timer_now(void)
{
    timer_mut_lock();
    ts_timestamp_t time = 0;
    if (m_is_in_ts)
    {
        NRF_TIMER0->EVENTS_COMPARE[TS_TIMER_INDEX_TIMESTAMP] = 0;
        NRF_TIMER0->TASKS_CAPTURE[TS_TIMER_INDEX_TIMESTAMP] = 1;
        time = NRF_TIMER0->CC[TS_TIMER_INDEX_TIMESTAMP];
    }
    else
    {
        /* return the end of the previous TS */
        time = m_ts_end_time;
    }
    timer_mut_unlock();
    return time;
}

timestamp_t ts_timer_start_get(void)
{
    return m_ts_start_time;
}

timestamp_t ts_timer_to_device_time(ts_timestamp_t timestamp)
{
    return m_ts_start_time + timestamp;
}

void ts_timer_on_ts_begin(void)
{
    /* executed in STACK_LOW */
#if !defined(HOST)
    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        NRF_TIMER0->CC[i] = 0;
        NRF_TIMER0->EVENTS_COMPARE[i] = 0;
        (void) NRF_TIMER0->EVENTS_COMPARE[i];
    }

    /* only enable interrupts if we're not locked. */
    if (!m_timer_mut)
    {
        (void) NVIC_EnableIRQ(TIMER0_IRQn);
    }
#endif

    m_is_in_ts = true;
    m_ts_start_time = timer_now() - ts_timer_now();
}

void ts_timer_on_ts_end(ts_timestamp_t timeslot_end_time)
{
    /* executed in STACK_LOW */
    /* purge ts-local timers */
    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        m_callbacks[i] = NULL;
        mp_ppi_tasks[i] = NULL;
    }
    m_ts_end_time = timeslot_end_time;
    m_is_in_ts = false;

#if !defined(HOST)
    /* Kill any pending interrupts, to avoid softdevice triggering. */
    NRF_TIMER0->INTENCLR = 0xFFFFFFFF;
    (void) NVIC_ClearPendingIRQ(TIMER0_IRQn);
#endif
}

