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

#include "access_reliable.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "access.h"
#include "access_config.h"
#include "access_utils.h"

#include "device_state_manager.h"
#include "nrf_mesh_config_app.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_defines.h"
#include "packet_mesh.h"

#include "timer.h"
#include "timer_scheduler.h"
#include "bearer_event.h"
#include "bearer_defines.h"

#include "log.h"

/* ******************* Definitions ******************* */

/** Invalid pool index. */
#define ACCESS_RELIABLE_INDEX_INVALID (0xFFFF)

typedef struct
{
    access_reliable_t params;
    timestamp_t next_timeout;
    uint32_t interval;
    bool in_use;
} access_reliable_ctx_t;

static bool m_is_in_reliable_timer_cb;

static uint32_t calculate_interval(const access_reliable_t * p_message);

/* ******************* Static asserts ******************* */

NRF_MESH_STATIC_ASSERT(ACCESS_RELIABLE_BACK_OFF_FACTOR > 0);
NRF_MESH_STATIC_ASSERT(ACCESS_RELIABLE_INTERVAL_DEFAULT >= MS_TO_US(BEARER_ADV_INT_MIN_MS));
NRF_MESH_STATIC_ASSERT(ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY >= MS_TO_US(BEARER_ADV_INT_MIN_MS));

/* ******************* Static variables ******************* */

static struct
{
    timer_event_t timer;
    access_reliable_ctx_t pool[ACCESS_RELIABLE_TRANSFER_COUNT];
    uint16_t active_count;
    uint16_t next_timeout_index;
} m_reliable;

/* ******************* Static functions ******************* */

static void reliable_timer_cb(timestamp_t timestamp, void * p_context)
{
    NRF_MESH_ASSERT(0 < m_reliable.active_count);

    m_is_in_reliable_timer_cb = true;
    timestamp += ACCESS_RELIABLE_TIMEOUT_MARGIN; /* TODO: Divide by two? */
    m_reliable.next_timeout_index = ACCESS_RELIABLE_INDEX_INVALID;

    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (!m_reliable.pool[i].in_use)
        {
            continue;
        }
        else if (TIMER_OLDER_THAN(m_reliable.pool[i].params.timeout, timestamp))
        {
            /* Remove first, in case a crazy user tries to reschedule it in the callback. */
            m_reliable.pool[i].in_use = false;
            m_reliable.active_count--;

            void * p_args;
            NRF_MESH_ERROR_CHECK(access_model_p_args_get(m_reliable.pool[i].params.model_handle, &p_args));
            m_reliable.pool[i].params.status_cb(m_reliable.pool[i].params.model_handle, p_args, ACCESS_RELIABLE_TRANSFER_TIMEOUT);
        }
        else if (TIMER_OLDER_THAN(m_reliable.pool[i].next_timeout, timestamp))
        {
            m_reliable.next_timeout_index = i;

            uint32_t status = access_model_publish(m_reliable.pool[i].params.model_handle, &m_reliable.pool[i].params.message);

            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "[er%d] <= access_model_publish()\n", status);
            }

            if (NRF_SUCCESS == status ||
                NRF_ERROR_INVALID_STATE == status)
            {
                m_reliable.pool[i].next_timeout += m_reliable.pool[i].interval;
                m_reliable.pool[i].interval *= ACCESS_RELIABLE_BACK_OFF_FACTOR;
            }
            else if (NRF_ERROR_NO_MEM == status ||
                     NRF_ERROR_FORBIDDEN == status)
            {
                /* If there is no more memory available (NRF_ERROR_NO_MEM) or we cannot allocate
                 * sequence numbers right now (NRF_ERROR_FORBIDDEN), we might as well cancel the
                 * rest and set the timer to fire in ACCESS_RELIABLE_RETRY_DELAY. */
                m_reliable.pool[i].next_timeout += ACCESS_RELIABLE_RETRY_DELAY;
                break;
            }
            else
            {
                /* This should have been caught by the first publish() call. */
                NRF_MESH_ASSERT(false);
            }

            if (TIMER_OLDER_THAN(m_reliable.pool[i].params.timeout, m_reliable.pool[i].next_timeout))
            {
                /* Shift timeout forward. */
                m_reliable.pool[i].next_timeout = m_reliable.pool[i].params.timeout;
            }
        }
        else if (ACCESS_RELIABLE_INDEX_INVALID == m_reliable.next_timeout_index ||
                 TIMER_OLDER_THAN(m_reliable.pool[i].next_timeout,
                                  m_reliable.pool[m_reliable.next_timeout_index].next_timeout))
        {
            /* Keep track of the next firing timeout. */
            m_reliable.next_timeout_index = i;
        }
    }

    /* Setting the interval > 0 will reschedule the timer. */
    if (m_reliable.active_count > 0)
    {
        NRF_MESH_ASSERT(m_reliable.next_timeout_index < ACCESS_RELIABLE_TRANSFER_COUNT);
        timestamp -= ACCESS_RELIABLE_TIMEOUT_MARGIN;
        m_reliable.timer.interval = TIMER_DIFF(m_reliable.pool[m_reliable.next_timeout_index].next_timeout, timestamp);
    }
    else
    {
        m_reliable.timer.interval = 0;
    }
    m_is_in_reliable_timer_cb = false;
}

/**
 * Searches for a context index given a model handle.
 * Returns true if it is found, false otherwise.
 */
static bool find_index(access_model_handle_t model_handle, uint16_t * p_index)
{
    *p_index = ACCESS_RELIABLE_INDEX_INVALID;

    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (m_reliable.pool[i].in_use &&
            m_reliable.pool[i].params.model_handle == model_handle)
        {
            *p_index = i;
            break;
        }
    }
    return (*p_index != ACCESS_RELIABLE_INDEX_INVALID);
}

/**
 * Gets the first available context.
 * Returns false if there are no available contexts or if the context already exists.
 */
static bool available_context_get(const access_reliable_t * p_message, uint16_t * p_index, uint32_t * p_status)
{
    *p_index = ACCESS_RELIABLE_INDEX_INVALID;
    *p_status = NRF_ERROR_NO_MEM;
    bearer_event_critical_section_begin();
    if (ACCESS_RELIABLE_TRANSFER_COUNT >= m_reliable.active_count)
    {
        for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
        {
            if (!m_reliable.pool[i].in_use)
            {
                if (ACCESS_RELIABLE_INDEX_INVALID == *p_index)
                {
                    *p_status = NRF_SUCCESS;
                    *p_index = i;
                }
            }
            else if (m_reliable.pool[i].params.model_handle == p_message->model_handle)
            {
                *p_status = NRF_ERROR_INVALID_STATE;
                break;
            }
        }
    }
    bearer_event_critical_section_end();
    return (NRF_SUCCESS == *p_status);
}

static bool is_earliest_timeout(uint16_t index)
{
    return !(m_reliable.active_count > 0 &&
             TIMER_OLDER_THAN(m_reliable.pool[m_reliable.next_timeout_index].next_timeout,
                              m_reliable.pool[index].next_timeout));
}

static uint16_t get_next_timeout_index(uint16_t old_index)
{
    uint32_t next_timeout = m_reliable.pool[old_index].next_timeout + ACCESS_RELIABLE_TIMEOUT_MAX;
    uint16_t next_index = ACCESS_RELIABLE_INDEX_INVALID;
    for (uint16_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (m_reliable.pool[i].in_use && TIMER_OLDER_THAN(m_reliable.pool[i].next_timeout, next_timeout))
        {
            next_timeout = m_reliable.pool[i].next_timeout;
            next_index = i;
        }
    }
    return next_index;
}

static uint32_t calculate_interval(const access_reliable_t * p_message)
{
    uint8_t ttl;
    /* The model handle should already been checked by the TX attempt. */
    NRF_MESH_ERROR_CHECK(access_model_publish_ttl_get(p_message->model_handle, &ttl));

    if (ACCESS_TTL_USE_DEFAULT == ttl)
    {
        ttl = access_default_ttl_get();
    }

    uint16_t length = access_utils_opcode_size_get(p_message->message.opcode) + p_message->message.length;
    uint32_t interval = (ttl * ACCESS_RELIABLE_HOP_PENALTY) + ACCESS_RELIABLE_INTERVAL_DEFAULT;

    if (NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX < length)
    {
        interval += ((length + (PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - 1)) /
                PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE) * ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY;
    }
    return MIN(p_message->timeout, interval);
}

static void add_reliable_message(uint16_t index, const access_reliable_t * p_message)
{
    NRF_MESH_ASSERT(!m_reliable.pool[index].in_use);
    uint32_t time_now = timer_now();
    memcpy(&(m_reliable.pool[index].params), p_message, sizeof(access_reliable_t));
    m_reliable.pool[index].interval = calculate_interval(p_message);
    m_reliable.pool[index].params.timeout += time_now;
    m_reliable.pool[index].next_timeout = time_now + m_reliable.pool[index].interval;

    bearer_event_critical_section_begin();
    if (m_is_in_reliable_timer_cb && m_reliable.next_timeout_index == ACCESS_RELIABLE_INDEX_INVALID)
    {
        m_reliable.next_timeout_index = index;
    }
    else if (is_earliest_timeout(index))
    {
        timer_sch_reschedule(&m_reliable.timer, m_reliable.pool[index].next_timeout);
        m_reliable.next_timeout_index = index;
    }
    m_reliable.pool[index].in_use = true;
    m_reliable.active_count++;
    bearer_event_critical_section_end();
}

static void remove_and_reschedule(uint16_t index)
{
    NRF_MESH_ASSERT(m_reliable.pool[index].in_use);
    NRF_MESH_ASSERT(m_reliable.active_count > 0);
    m_reliable.pool[index].in_use = false;
    m_reliable.active_count--;
    if (m_reliable.active_count > 0)
    {
        if (m_reliable.next_timeout_index == index)
        {
            m_reliable.next_timeout_index = get_next_timeout_index(index);
            NRF_MESH_ASSERT(m_reliable.next_timeout_index != ACCESS_RELIABLE_INDEX_INVALID);
            timer_sch_reschedule(&m_reliable.timer,
                                 m_reliable.pool[m_reliable.next_timeout_index].next_timeout);
        }
    }
    else
    {
        timer_sch_abort(&m_reliable.timer);
    }
}

/* ******************* Semi-Public API ******************* */

void access_reliable_message_rx_cb(access_model_handle_t model_handle, const access_message_rx_t * p_message, void * p_args)
{
    NRF_MESH_ASSERT(ACCESS_MODEL_COUNT > model_handle);
    bearer_event_critical_section_begin();
    uint16_t index;
    if (find_index(model_handle, &index) &&
        m_reliable.pool[index].params.reply_opcode.opcode == p_message->opcode.opcode &&
        m_reliable.pool[index].params.reply_opcode.company_id == p_message->opcode.company_id)
    {
        /* Remove first, in case a crazy user tries to reschedule it in the callback. */
        remove_and_reschedule(index);
        m_reliable.pool[index].params.status_cb(model_handle, p_args, ACCESS_RELIABLE_TRANSFER_SUCCESS);
    }

    bearer_event_critical_section_end();
}

bool access_reliable_model_is_free(access_model_handle_t model_handle)
{
    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (m_reliable.pool[i].in_use && m_reliable.pool[i].params.model_handle == model_handle)
        {
            return false;
        }
    }
    return true;
}

/* ******************* Public API ******************* */

void access_reliable_init(void)
{
    memset(&m_reliable, 0, sizeof(m_reliable));
    m_reliable.timer.cb = reliable_timer_cb;
    m_is_in_reliable_timer_cb = false;
}

void access_reliable_cancel_all(void)
{
    bearer_event_critical_section_begin();
    if (m_reliable.active_count > 0)
    {
        m_reliable.active_count = 0;
        timer_sch_abort(&m_reliable.timer);
    }

    /* Cancel all active transfers */
    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (m_reliable.pool[i].in_use)
        {
            access_model_handle_t model_handle = m_reliable.pool[i].params.model_handle;
            access_reliable_cb_t status_cb = m_reliable.pool[i].params.status_cb;

            memset(&m_reliable.pool[i], 0, sizeof(access_reliable_ctx_t));

            /* Notify model */
            void * p_args;
            NRF_MESH_ERROR_CHECK(access_model_p_args_get(model_handle, &p_args));
            status_cb(model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
        }
    }

    bearer_event_critical_section_end();
}

uint32_t access_model_reliable_cancel(access_model_handle_t model_handle)
{
    if (ACCESS_MODEL_COUNT <= model_handle)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        uint32_t status;
        uint16_t index;
        void * p_args;
        bearer_event_critical_section_begin();
        if (find_index(model_handle, &index))
        {
            status = NRF_SUCCESS;
            remove_and_reschedule(index);

            /* Notify model */
            NRF_MESH_ERROR_CHECK(access_model_p_args_get(model_handle, &p_args));
            m_reliable.pool[index].params.status_cb(model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
        }
        else
        {
            status = NRF_ERROR_NOT_FOUND;
        }

        bearer_event_critical_section_end();
        return status;
    }
}

uint32_t access_model_reliable_publish(const access_reliable_t * p_reliable)
{
    uint32_t status;
    uint16_t index;

    if (NULL == p_reliable || NULL == p_reliable->status_cb)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_MODEL_COUNT <= p_reliable->model_handle)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (ACCESS_RELIABLE_TIMEOUT_MIN > p_reliable->timeout  ||
             ACCESS_RELIABLE_TIMEOUT_MAX < p_reliable->timeout)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (!available_context_get(p_reliable, &index, &status))
    {
        return status;
    }
    else
    {
        status = access_model_publish(p_reliable->model_handle, &p_reliable->message);
        if (NRF_SUCCESS == status || NRF_ERROR_NO_MEM == status || NRF_ERROR_INVALID_STATE == status)
        {
            /** @todo If we get @c NRF_ERROR_NO_MEM, we could be even "smarter" and retry in @ref
             * ACCESS_RELIABLE_RETRY_DELAY scaled based on advertising intervals or something.
             * Ref.: MBTLE-1542. */
            add_reliable_message(index, p_reliable);
            return NRF_SUCCESS;
        }
        else
        {
            return status;
        }
    }
}
