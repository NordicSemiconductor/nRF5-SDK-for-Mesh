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
#include "msqueue.h"
#include "utils.h"
#include "nrf_mesh_assert.h"
/******************************************************************************
* Static functions
******************************************************************************/
static inline void * get_stage(const msq_t * p_queue, uint8_t stage)
{
    /* The index of the stage is the stage value modulo the number of elements in the array.
     * This way, the stage can be a running counter, and we only truncate it when using it for
     * array access. With this mechanism, having the first and last stage point to the same
     * value isn't ambigiuos, as they'll still be different numbers. */
    uint32_t index = (p_queue->p_stages[stage] & (p_queue->elem_count - 1));

    return (void *) ((uint8_t *) p_queue->p_elem_array + (index * p_queue->elem_size));
}

static inline uint8_t stage_get_available(const msq_t * p_queue, uint8_t stage)
{
    if (stage == 0)
    {
        /* The first stage will be out of space when it's `elem_count` indexes ahead of the last */
        return ((p_queue->p_stages[p_queue->stage_count - 1] + p_queue->elem_count) - p_queue->p_stages[0]);
    }
    else
    {
        return (p_queue->p_stages[stage - 1] - p_queue->p_stages[stage]);
    }
}
/******************************************************************************
* Interface functions
******************************************************************************/
void msq_init(msq_t * p_queue)
{
    NRF_MESH_ASSERT(p_queue->elem_size != 0);
    NRF_MESH_ASSERT(p_queue->stage_count > 1);
    NRF_MESH_ASSERT(p_queue->p_elem_array != NULL);
    NRF_MESH_ASSERT(p_queue->p_stages != NULL);
    NRF_MESH_ASSERT(is_power_of_two(p_queue->elem_count));
    msq_reset(p_queue);
}

void * msq_get(const msq_t * p_queue, uint8_t stage)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    NRF_MESH_ASSERT(stage < p_queue->stage_count);
    if (stage_get_available(p_queue, stage) != 0)
    {
        return get_stage(p_queue, stage);
    }
    else
    {
        return NULL;
    }
}

void msq_move(msq_t * p_queue, uint8_t stage)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    NRF_MESH_ASSERT(stage < p_queue->stage_count);
    if (stage_get_available(p_queue, stage) != 0)
    {
        p_queue->p_stages[stage]++;
    }
}

void msq_reset(msq_t * p_queue)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    for (uint32_t i = 0; i < p_queue->stage_count; i++)
    {
        p_queue->p_stages[i] = 0;
    }
}

uint8_t msq_available(const msq_t * p_queue, uint8_t stage)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    NRF_MESH_ASSERT(stage < p_queue->stage_count);
    return stage_get_available(p_queue, stage);
}
