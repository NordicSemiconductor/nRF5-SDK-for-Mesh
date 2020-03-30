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
#include <string.h>
#include "queue.h"

#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "utils.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
/*****************************************************************************
* Interface functions
*****************************************************************************/
void queue_init(queue_t* p_queue)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    p_queue->p_front = NULL;
    p_queue->p_back  = NULL;
}

void queue_push(queue_t* p_queue, queue_elem_t* p_elem)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    NRF_MESH_ASSERT(p_elem != NULL);

    p_elem->p_next = NULL;

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (p_queue->p_front == NULL)
    {
        p_queue->p_front = p_elem;
    }
    else
    {
        p_queue->p_back->p_next = p_elem;
    }
    p_queue->p_back = p_elem;
    _ENABLE_IRQS(was_masked);
}

queue_elem_t* queue_pop(queue_t* p_queue)
{
    NRF_MESH_ASSERT(p_queue != NULL);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    queue_elem_t* p_elem = p_queue->p_front;

    if (p_elem != NULL)
    {
        p_queue->p_front = p_elem->p_next;
    }

    if (p_queue->p_front == NULL)
    {
        p_queue->p_back = NULL;
    }

    _ENABLE_IRQS(was_masked);
    return p_elem;
}

queue_elem_t* queue_peek(const queue_t* p_queue)
{
    NRF_MESH_ASSERT(p_queue != NULL);
    return p_queue->p_front;
}

void queue_merge(queue_t * p_dst, queue_t * p_src)
{
    NRF_MESH_ASSERT(p_dst != NULL);
    NRF_MESH_ASSERT(p_src != NULL);

    if (queue_peek(p_src) == NULL)
    {
        // p_src is empty, do nothing.
    }
    else if (queue_peek(p_dst) == NULL)
    {
        // p_dst is empty
        *p_dst = *p_src;
        queue_init(p_src);
    }
    else
    {
        // both queues have elements
        p_dst->p_back->p_next = p_src->p_front;
        p_dst->p_back = p_src->p_back;
        queue_init(p_src);
    }
}

void queue_iterator_elem_remove(queue_elem_iterator_t * p_it)
{
    /* If we're explicitly removing p_queue->p_back, we need to translate that iterator to a normal
     * p_front iterator and remove that, as p_back doesn't provide any information about the
     * second-to-last element, making it impossible to cleanly terminate the back of the queue: */
    if (p_it->pp_elem == &p_it->p_queue->p_back)
    {
        QUEUE_FOREACH(p_it->p_queue, it)
        {
            if (*it.pp_elem == *p_it->pp_elem)
            {
                /* The local iterator now points to the same element as the requested element, but it
                 * also contains information about the second-to-last entry, which is required to
                 * cleanly remove the element. Use this iterator instead. */
                queue_iterator_elem_remove(&it);
                return;
            }
        }
    }

    if (*p_it->pp_elem == p_it->p_queue->p_back)
    {
        if (p_it->pp_elem == &p_it->p_queue->p_front)
        {
            p_it->p_queue->p_back = NULL;
        }
        else
        {
            p_it->p_queue->p_back = PARENT_BY_FIELD_GET(queue_elem_t, p_next, p_it->pp_elem);
        }
    }
    *p_it->pp_elem = (*p_it->pp_elem)->p_next;
    p_it->repeat = true;
}

void queue_iterator_elem_insert(queue_elem_iterator_t * p_it, queue_elem_t * p_elem)
{
    p_elem->p_next = *p_it->pp_elem;
    *p_it->pp_elem = p_elem;
}

void queue_iterator_iterate(queue_elem_iterator_t * p_it)
{
    if (p_it->repeat)
    {
        p_it->repeat = false;
    }
    else
    {
        p_it->pp_elem = &(*p_it->pp_elem)->p_next;
    }
}
