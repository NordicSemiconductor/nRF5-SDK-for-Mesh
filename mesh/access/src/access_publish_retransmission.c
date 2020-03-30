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

#include "access_publish_retransmission.h"

#include "access_internal.h"

#include <string.h>

#include "utils.h"
#include "timer.h"
#include "timer_scheduler.h"
#include "nrf_mesh_assert.h"
#include "mesh_mem.h"

/**
 * Evaluates retransmission interval according to the following formula
 * (@tagMeshSp section 4.2.2.7):
 *
 * retransmission interval [ms] = (Publish Retransmit Interval Steps + 1) * 50ms
 */
#define GET_RETRANSMISSION_INTERVAL_US(step) (MS_TO_US(((step) + 1) * 50))

/** Invalid pool index. */
#define POOL_INDEX_INVALID (ACCESS_HANDLE_INVALID)

/** Margin in microseconds for which two timeout events are fired "at the same time". */
#define TIMER_TIMEOUT_MARGIN_US (MS_TO_US(1))

typedef struct
{
    access_message_tx_t tx_message;
    const uint8_t *p_access_payload;
    uint16_t access_payload_length;
    timestamp_t next_timeout_us;
    timestamp_t interval_us;
    uint8_t retransmits_left;
    bool in_use;
} publication_t;

static struct
{
    timer_event_t timer;
    publication_t pool[ACCESS_MODEL_COUNT];
    uint16_t active_count;
    access_model_handle_t next_timeout_index;
} m_retr;

static uint16_t get_next_timeout_index(void);
static void message_remove(access_model_handle_t index);

static void retransmissions_timer_cb(timestamp_t timestamp, void * p_context)
{
    UNUSED_VARIABLE(p_context);

    NRF_MESH_ASSERT(0 < m_retr.active_count);

    timestamp_t future = timestamp + TIMER_TIMEOUT_MARGIN_US;

    for (uint16_t i = 0; i < ARRAY_SIZE(m_retr.pool); ++i)
    {
        if (!m_retr.pool[i].in_use)
        {
            continue;
        }
        else if (TIMER_OLDER_THAN(m_retr.pool[i].next_timeout_us, future))
        {
            uint32_t status = access_packet_tx(i, &m_retr.pool[i].tx_message,
                                               m_retr.pool[i].p_access_payload,
                                               m_retr.pool[i].access_payload_length);
            switch (status)
            {
                case NRF_SUCCESS:
                case NRF_ERROR_NO_MEM:
                case NRF_ERROR_FORBIDDEN:
                case NRF_ERROR_INVALID_STATE:
                    /* No-op */
                    break;

                case NRF_ERROR_NOT_FOUND:
                case NRF_ERROR_INVALID_LENGTH:
                case NRF_ERROR_INVALID_ADDR:
                case NRF_ERROR_INVALID_PARAM:
                    message_remove(i);
                    continue;

                case NRF_ERROR_NULL:
                default:
                    NRF_MESH_ASSERT(status == NRF_SUCCESS);
                    break;
            }

            m_retr.pool[i].retransmits_left--;
            if (m_retr.pool[i].retransmits_left == 0)
            {
                message_remove(i);
                continue;
            }

            m_retr.pool[i].next_timeout_us += m_retr.pool[i].interval_us;
        }
    }

    m_retr.next_timeout_index = get_next_timeout_index();

    /* Setting the interval > 0 will reschedule the timer. */
    if (m_retr.active_count > 0)
    {
        NRF_MESH_ASSERT(m_retr.next_timeout_index < ACCESS_MODEL_COUNT);
        m_retr.timer.interval = TIMER_DIFF(m_retr.pool[m_retr.next_timeout_index].next_timeout_us, timestamp);
    }
    else
    {
        m_retr.timer.interval = 0;
    }
}

static uint16_t get_next_timeout_index(void)
{
    uint16_t next_index = POOL_INDEX_INVALID;
    for (uint16_t i = 0; i < ARRAY_SIZE(m_retr.pool); ++i)
    {
        if (m_retr.pool[i].in_use)
        {
            if (next_index == POOL_INDEX_INVALID ||
                TIMER_OLDER_THAN(m_retr.pool[i].next_timeout_us,
                                 m_retr.pool[next_index].next_timeout_us))
            {
                next_index = i;
            }
        }
    }
    return next_index;
}

static void message_add(access_model_handle_t model_handle,
                        const access_publish_retransmit_t *p_publication_retransmit,
                        const access_message_tx_t *p_tx_message,
                        const uint8_t *p_access_payload,
                        uint16_t access_payload_len)
{
    timestamp_t time_now = timer_now();

    m_retr.pool[model_handle].tx_message = *p_tx_message;
    m_retr.pool[model_handle].p_access_payload = p_access_payload;
    m_retr.pool[model_handle].access_payload_length = access_payload_len;
    m_retr.pool[model_handle].retransmits_left = p_publication_retransmit->count;
    m_retr.pool[model_handle].interval_us = GET_RETRANSMISSION_INTERVAL_US(p_publication_retransmit->interval_steps);
    m_retr.pool[model_handle].next_timeout_us = time_now + m_retr.pool[model_handle].interval_us;
    m_retr.pool[model_handle].in_use = true;

    m_retr.active_count++;
}

static void message_remove(access_model_handle_t index)
{
    NRF_MESH_ASSERT(m_retr.pool[index].in_use);
    NRF_MESH_ASSERT(m_retr.active_count > 0);

    m_retr.pool[index].in_use = false;
    m_retr.active_count--;
    if (m_retr.active_count == 0)
    {
        m_retr.next_timeout_index = POOL_INDEX_INVALID;
    }

    mesh_mem_free((uint8_t*) m_retr.pool[index].p_access_payload);
    m_retr.pool[index].p_access_payload = NULL;
}

static void reschedule_next(access_model_handle_t new_index)
{
    uint16_t next_index = get_next_timeout_index();

    if (m_retr.next_timeout_index != next_index ||
        m_retr.next_timeout_index == new_index)
    {
        m_retr.next_timeout_index = next_index;
        NRF_MESH_ASSERT(m_retr.next_timeout_index != POOL_INDEX_INVALID);
        timer_sch_reschedule(&m_retr.timer,
                             m_retr.pool[m_retr.next_timeout_index].next_timeout_us);
    }
}

/********************************* Public API *********************************/

void access_publish_retransmission_init(void)
{
    memset(&m_retr, 0, sizeof(m_retr));
    m_retr.timer.cb = retransmissions_timer_cb;
    m_retr.next_timeout_index = POOL_INDEX_INVALID;
}

void access_publish_retransmission_message_add(access_model_handle_t model_handle,
                                               const access_publish_retransmit_t *p_publish_retransmit,
                                               const access_message_tx_t *p_tx_message,
                                               const uint8_t *p_access_payload,
                                               uint16_t access_payload_len)
{
    NRF_MESH_ASSERT(ACCESS_MODEL_COUNT > model_handle);
    NRF_MESH_ASSERT(p_publish_retransmit != NULL);
    NRF_MESH_ASSERT(p_tx_message != NULL);
    NRF_MESH_ASSERT(p_publish_retransmit->count != 0);
    NRF_MESH_ASSERT(p_access_payload != NULL);
    NRF_MESH_ASSERT(access_payload_len != 0);

    if (m_retr.pool[model_handle].in_use)
    {
        message_remove(model_handle);
    }

    message_add(model_handle, p_publish_retransmit, p_tx_message,
                p_access_payload, access_payload_len);

    reschedule_next(model_handle);
}
