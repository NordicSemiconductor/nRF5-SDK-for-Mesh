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

#include "nrf_mesh_assert.h"
#include "filter_engine.h"
#include "utils.h"

typedef struct
{
    list_node_t *   p_list_head;
    uint32_t        accepted_amount;
} fen_filter_set_t;

static fen_filter_set_t m_filter_sets[FILTER_TYPE_END];

void fen_filter_start(filter_t * p_filter)
{
    NRF_MESH_ASSERT(p_filter != NULL);
    NRF_MESH_ASSERT(p_filter->handler != NULL);
    NRF_MESH_ASSERT((p_filter->type == FILTER_TYPE_PRE_PROC) || (p_filter->type == FILTER_TYPE_POST_PROC));

    list_add(&m_filter_sets[p_filter->type].p_list_head, &p_filter->node);
}

void fen_filter_stop(filter_t * p_filter)
{
    NRF_MESH_ASSERT(p_filter != NULL);
    NRF_MESH_ASSERT((p_filter->type == FILTER_TYPE_PRE_PROC) || (p_filter->type == FILTER_TYPE_POST_PROC));

    (void)list_remove(&m_filter_sets[p_filter->type].p_list_head, &p_filter->node);
}

bool fen_filters_apply(filter_type_t type, scanner_packet_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL);
    NRF_MESH_ASSERT((type == FILTER_TYPE_PRE_PROC) || (type == FILTER_TYPE_POST_PROC));

    LIST_FOREACH(p_node, m_filter_sets[type].p_list_head)
    {
        filter_t * p_filter = PARENT_BY_FIELD_GET(filter_t, node, p_node);

        if (p_filter->handler(p_packet, p_filter->p_data))
        {
            return true;
        }
    }

    m_filter_sets[type].accepted_amount++;

    return false;
}

uint32_t fen_accepted_amount_get(filter_type_t type)
{
    NRF_MESH_ASSERT((type == FILTER_TYPE_PRE_PROC) || (type == FILTER_TYPE_POST_PROC));

    return m_filter_sets[type].accepted_amount;
}

#if defined(UNIT_TEST)
list_node_t * fen_pre_filter_list_head_get(void)
{
    return m_filter_sets[FILTER_TYPE_PRE_PROC].p_list_head;
}

list_node_t * fen_post_filter_list_head_get(void)
{
    return m_filter_sets[FILTER_TYPE_POST_PROC].p_list_head;
}
#endif
