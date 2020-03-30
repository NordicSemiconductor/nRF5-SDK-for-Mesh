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
#include "instaburst.h"

#include "instaburst_rx.h"
#include "instaburst_tx.h"
#include "cache.h"
#include "nrf_mesh_assert.h"

NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(INSTABURST_EVENT_ID_CACHE_SIZE));

#if INSTABURST_CACHE_ENABLED
static cache_t m_id_cache;
static nrf_mesh_instaburst_event_id_t m_id_cache_buffer[INSTABURST_EVENT_ID_CACHE_SIZE];
#endif

void instaburst_init(uint32_t lfclk_ppm, bearer_event_flag_callback_t packet_process_cb)
{
#if INSTABURST_CACHE_ENABLED
    m_id_cache.array_len = INSTABURST_EVENT_ID_CACHE_SIZE;
    m_id_cache.elem_array = m_id_cache_buffer;
    m_id_cache.elem_size  = sizeof(nrf_mesh_instaburst_event_id_t);
    cache_init(&m_id_cache);
#endif

    instaburst_rx_init(packet_process_cb);
    instaburst_tx_init(lfclk_ppm);
}

bool instaburst_event_id_cache_has_value(const nrf_mesh_instaburst_event_id_t * p_id)
{
#if INSTABURST_CACHE_ENABLED
    return cache_has_elem(&m_id_cache, p_id);
#else
    return false;
#endif
}

void instaburst_event_id_cache_put(const nrf_mesh_instaburst_event_id_t * p_id)
{
#if INSTABURST_CACHE_ENABLED
    cache_put(&m_id_cache, p_id);
#endif
}
