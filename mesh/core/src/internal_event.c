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
#include "fifo.h"
#include "internal_event.h"

/* The internal_event_type_t must fit inside a single byte, to make sure it can go into a packet. */
NRF_MESH_STATIC_ASSERT(INTERNAL_EVENT__LAST <= 0xFF);

#if INTERNAL_EVT_ENABLE

static internal_event_report_cb_t m_report_cb;
static fifo_t           m_internal_event_fifo;
static internal_event_t m_internal_event_fifo_buffer[INTERNAL_EVENT_BUFFER_SIZE];
static bool m_internal_event_initialized = false;

void internal_event_init(internal_event_report_cb_t report_cb)
{
    m_report_cb = report_cb;
    if (NULL == report_cb)
    {
        memset(m_internal_event_fifo_buffer, 0, sizeof(m_internal_event_fifo_buffer));

        m_internal_event_fifo.elem_array  = m_internal_event_fifo_buffer;
        m_internal_event_fifo.elem_size   = sizeof(internal_event_t);
        m_internal_event_fifo.array_len   = INTERNAL_EVENT_BUFFER_SIZE;

        fifo_init(&m_internal_event_fifo);
    }
    m_internal_event_initialized = true;
}

uint32_t internal_event_push(internal_event_t * p_event)
{
    uint32_t status;
    if (m_internal_event_initialized == false)
    {
        status = NRF_ERROR_NOT_SUPPORTED;
    }
    else if (NULL == m_report_cb)
    {
        status = fifo_push(&m_internal_event_fifo, p_event);
    }
    else
    {
        status = m_report_cb(p_event);
    }
    return status;
}

uint32_t internal_event_pop(internal_event_t * p_event)
{
    uint32_t status = NRF_ERROR_INVALID_STATE;
    if (NULL == m_report_cb)
    {
        status = fifo_pop(&m_internal_event_fifo, p_event);
    }
    return status;
}

#endif  /* INTERNAL_EVT_ENABLE */
