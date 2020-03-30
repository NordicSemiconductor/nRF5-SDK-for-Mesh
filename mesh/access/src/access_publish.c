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

#include "access_publish.h"

#include <stdint.h>
#include <string.h>

#include "access.h"
#include "access_config.h"

#include "bearer_event.h"
#include "nrf_mesh_assert.h"
#include "utils.h"
#include "timer_scheduler.h"

NRF_MESH_STATIC_ASSERT(ACCESS_PUBLISH_RESOLUTION_MAX <= UINT8_MAX);

/** Margin for when to round the elapsed time between timer ticks up to the next 100 ms when rescheduling publication events, in us. */
#define ACCESS_PUBLISH_ROUNDING_MARGIN MS_TO_US(50)
/* Converts seconds into the corresponding number of 100 ms intervals. */
#define SEC_TO_100MS(s) ((s) * 10U)

/** Publish timer scheduler instance. */
static timer_event_t m_publish_timer;

/** Whether the publish timer is running. */
static bool m_publish_timer_running;

/** Current publish timer resolution. */
static access_publish_resolution_t m_timer_resolution;

/** Counter for the publish timer, counts in multiples of 100 ms. */
static volatile uint32_t m_publish_timer_counter;

/** List of scheduled publications. */
static access_model_publication_state_t * mp_publication_list;

/********************* Internal Functions ********************/

static inline uint32_t calculate_publish_period(const access_publish_period_t * p_period)
{
    /* Calculate the publish period in steps of 100 ms: */
    switch (p_period->step_res)
    {
        case ACCESS_PUBLISH_RESOLUTION_100MS:
            return p_period->step_num;
        case ACCESS_PUBLISH_RESOLUTION_1S:
            return p_period->step_num * SEC_TO_100MS(1);   /* 1s = 10 * 100 ms */
        case ACCESS_PUBLISH_RESOLUTION_10S:
            return p_period->step_num * SEC_TO_100MS(10);  /* 10 s = 100 * 100 ms */
        case ACCESS_PUBLISH_RESOLUTION_10MIN:
            return p_period->step_num * SEC_TO_100MS(600); /* 10 m = 600 s = 6000 * 100 ms */
        default:
            NRF_MESH_ASSERT(false);
            return 0;
    }
}

static inline uint32_t step_resolution_to_us(access_publish_resolution_t resolution)
{
    switch (resolution)
    {
        case ACCESS_PUBLISH_RESOLUTION_100MS:
            return MS_TO_US(100);  /* 100 ms = 100000 us */
        case ACCESS_PUBLISH_RESOLUTION_1S:
            return SEC_TO_US(1);   /* 1 s = 1000000 us */
        case ACCESS_PUBLISH_RESOLUTION_10S:
            return SEC_TO_US(10);  /* 10 s = 10000000 us */
        case ACCESS_PUBLISH_RESOLUTION_10MIN:
            return SEC_TO_US(600); /* 10 m = 600 s = 600000000 us */
        default:
            NRF_MESH_ASSERT(false);
            return 0;
    }
}

static inline uint32_t calculate_publish_target(access_model_publication_state_t * p_state)
{
    return m_publish_timer_counter + calculate_publish_period(&p_state->period);
}

static void add_to_publication_list(access_model_publication_state_t * p_pubstate)
{
    p_pubstate->target = calculate_publish_target(p_pubstate);

    access_model_publication_state_t * p_current = mp_publication_list;
    access_model_publication_state_t * p_previous = NULL;

    while (p_current != NULL && !TIMER_OLDER_THAN(p_pubstate->target, p_current->target))
    {
        p_previous = p_current;
        p_current = p_current->p_next;
    }

    if (p_previous == NULL)
    {
        p_pubstate->p_next = p_current;
        mp_publication_list = p_pubstate;
    }
    else if (p_current != NULL)
    {
        p_previous->p_next = p_pubstate;
        p_pubstate->p_next = p_current;
    }
    else
    {
        p_previous->p_next = p_pubstate;
        p_pubstate->p_next = NULL;
    }
}

static timestamp_t calculate_next_timestamp(void)
{
    timestamp_t new_timestamp = 0;

    /*
     * If the next publication event has a lower resolution than the current one, we may need to wait
     * for a number of steps at the current resolution before being able to switch, in order to
     * maintain correct timing. The following switch only changes resolution if the timing is right.
     */
    if (mp_publication_list->period.step_res > m_timer_resolution)
    {
        /* Calculate remaining steps at the current timer resolution: */
        const uint32_t remaining_steps = (mp_publication_list->target - m_publish_timer_counter) * MS_TO_US(100) / step_resolution_to_us(m_timer_resolution);

        bool resolution_changed = false;
        switch (mp_publication_list->period.step_res)
        {
            case ACCESS_PUBLISH_RESOLUTION_1S:
                /* If the number of remaining steps at current resolution are divisible by 1 second, switch to counting 1 second intervals. */
                if (remaining_steps % 10 == 0)
                {
                    new_timestamp = timer_now() + step_resolution_to_us(ACCESS_PUBLISH_RESOLUTION_1S);
                    m_timer_resolution = ACCESS_PUBLISH_RESOLUTION_1S;
                    resolution_changed = true;
                }
                break;
            case ACCESS_PUBLISH_RESOLUTION_10S:
                /* If the number of remaining steps at current resolution are divisible by 10 seconds, switch to counting 10 second intervals. */
                if ((m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_100MS && remaining_steps % 100 == 0)
                        || (m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_1S && remaining_steps % 10 == 0))
                {
                    new_timestamp = timer_now() + step_resolution_to_us(ACCESS_PUBLISH_RESOLUTION_10S);
                    m_timer_resolution = ACCESS_PUBLISH_RESOLUTION_10S;
                    resolution_changed = true;
                }
                break;
            case ACCESS_PUBLISH_RESOLUTION_10MIN:
                /* If the number of remaining steps at current resolution are divisible by 10 minutes, switch to counting 10 minute intervals. */
                if ((m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_100MS && remaining_steps % 6000 == 0)
                        || (m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_1S && remaining_steps % 600 == 0)
                        || (m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_10S && remaining_steps % 60 == 0))
                {
                    new_timestamp = timer_now() + step_resolution_to_us(ACCESS_PUBLISH_RESOLUTION_10MIN);
                    m_timer_resolution = ACCESS_PUBLISH_RESOLUTION_10MIN;
                    resolution_changed = true;
                }
                break;
        }

        if (!resolution_changed)
        {
            new_timestamp = timer_now() + step_resolution_to_us(m_timer_resolution);
        }
    }
    else
    {
        new_timestamp = timer_now() + step_resolution_to_us((access_publish_resolution_t)mp_publication_list->period.step_res);
        m_timer_resolution = (access_publish_resolution_t) mp_publication_list->period.step_res;
    }

    return new_timestamp;
}

static void schedule_publication_timer(void)
{
    if (m_publish_timer_running)
    {
        /* Get time elapsed since the current timer was started: */
        const timestamp_t time_remaining = m_publish_timer.timestamp - timer_now();
        const timestamp_t time_elapsed = step_resolution_to_us(m_timer_resolution) - time_remaining;

        if (m_timer_resolution == ACCESS_PUBLISH_RESOLUTION_100MS && time_elapsed > ACCESS_PUBLISH_ROUNDING_MARGIN)
        {
            m_publish_timer_counter += 1; /* If above the rounding margin, add 100 ms to the timer counter */
        }
        else
        {
            m_publish_timer_counter += time_elapsed / MS_TO_US(100);
        }
    }

    if (mp_publication_list != NULL)
    {
        timestamp_t new_timestamp = calculate_next_timestamp();

        if (m_publish_timer_running)
        {
            timer_sch_reschedule(&m_publish_timer, new_timestamp);
        }
        else
        {
            m_publish_timer_running = true;
            m_publish_timer.timestamp = new_timestamp;
            timer_sch_schedule(&m_publish_timer);
        }
    }
    else if (m_publish_timer_running)
    {
        timer_sch_abort(&m_publish_timer);
    }
}

static void trigger_publication_timers(void)
{
    while (mp_publication_list != NULL && (mp_publication_list->target == m_publish_timer_counter || TIMER_OLDER_THAN(mp_publication_list->target, m_publish_timer_counter)))
    {
        access_model_handle_t handle = mp_publication_list->model_handle;
        access_model_publication_state_t * p_pubstate = mp_publication_list;

        /* Remove the publication event from the list so it can be re-added at the correct spot;
         * the event can be re-added at the beginning of the list, but if it is, it will have a
         * target time in the future, causing the loop to terminate. */
        mp_publication_list = mp_publication_list->p_next;

        void * p_args = NULL;
        NRF_MESH_ERROR_CHECK(access_model_p_args_get(handle, &p_args));
        p_pubstate->publish_timeout_cb(handle, p_args);
        add_to_publication_list(p_pubstate);
    }

    schedule_publication_timer();
}

static void publish_timer_tick(timestamp_t now, void * p_context)
{
    switch (m_timer_resolution)
    {
        case ACCESS_PUBLISH_RESOLUTION_100MS:
            m_publish_timer_counter += 1;                 /* Add 100 ms */
            break;
        case ACCESS_PUBLISH_RESOLUTION_1S:
            m_publish_timer_counter += SEC_TO_100MS(1);   /* Add 10 * 100 ms = 1 s */
            break;
        case ACCESS_PUBLISH_RESOLUTION_10S:
            m_publish_timer_counter += SEC_TO_100MS(10);  /* Add 100 * 100 ms = 10 s */
            break;
        case ACCESS_PUBLISH_RESOLUTION_10MIN:
            m_publish_timer_counter += SEC_TO_100MS(600); /* Add 6000 * 100 ms = 10 m */
            break;
        default:
            NRF_MESH_ASSERT(false);
            break;
    }

    /* Trigger pending publication events: */
    m_publish_timer_running = false;
    trigger_publication_timers();
}

static void schedule_publication_event(access_model_publication_state_t * p_pubstate)
{
    add_to_publication_list(p_pubstate);

    /* Check if the publication state was inserted at the front of the list */
    if (mp_publication_list == p_pubstate)
    {
        schedule_publication_timer();
    }
}

/********************* Interface functions *********************/

void access_publish_init(void)
{
    memset(&m_publish_timer, 0, sizeof(m_publish_timer));
    m_publish_timer.cb = publish_timer_tick;
    m_publish_timer_counter = 0;
    mp_publication_list = NULL;
    m_publish_timer_running = false;
}

void access_publish_clear(void)
{
    if (m_publish_timer_running)
    {
        timer_sch_abort(&m_publish_timer);
    }
    access_publish_init();
}

void access_publish_period_set(access_model_publication_state_t * p_pubstate, access_publish_resolution_t resolution, uint8_t step_number)
{
    NRF_MESH_ASSERT(p_pubstate != NULL);

    bearer_event_critical_section_begin();

    /* Find the publication state in the publication list: */
    access_model_publication_state_t * p_previous = NULL;
    access_model_publication_state_t * p_current = mp_publication_list;
    while (p_current != NULL && p_current != p_pubstate)
    {
        p_previous = p_current;
        p_current = p_current->p_next;
    }

    /* Update publication period: */
    p_pubstate->period.step_res = resolution;
    p_pubstate->period.step_num = step_number;

    if (p_current != NULL) /* Remove the event from the list so it can be re-inserted at the correct spot: */
    {
        if (p_previous == NULL) /* The publication state was found at the start of the list */
        {
            mp_publication_list = p_current->p_next;
        }
        else                   /* The publication state was found elsewhere in the list */
        {
            p_previous->p_next = p_current->p_next;
        }
    }

    if (step_number != 0) /* Add publication event to the list; */
    {
        schedule_publication_event(p_pubstate);
    }
    else if (p_current != NULL && p_previous == NULL) /* The first event in the list was removed, reschedule timer */
    {
        schedule_publication_timer();
    }

    bearer_event_critical_section_end();
}

void access_publish_period_get(const access_model_publication_state_t * p_pubstate, access_publish_resolution_t * p_resolution, uint8_t * p_step_number)
{
    NRF_MESH_ASSERT(p_pubstate != NULL);

    *p_resolution = (access_publish_resolution_t) p_pubstate->period.step_res;
    *p_step_number = p_pubstate->period.step_num;
}


