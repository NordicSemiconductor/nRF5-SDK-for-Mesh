/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#include <nrf_error.h>

#include "event.h"
#include "list.h"
#include "utils.h"

/** Linked list of event handlers */
static list_node_t * mp_evt_handlers_head;

void event_handle(const nrf_mesh_evt_t * p_evt)
{
    NRF_MESH_ASSERT(p_evt != NULL);

    if (mp_evt_handlers_head != NULL)
    {
        LIST_FOREACH(p_node, mp_evt_handlers_head)
        {
            nrf_mesh_evt_handler_t * p_handler = PARENT_BY_FIELD_GET(nrf_mesh_evt_handler_t,
                                                                     p_next,
                                                                     p_node);

            p_handler->evt_cb(p_evt);
        }
    }
}

void event_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    NRF_MESH_ASSERT(p_handler_params != NULL);
    list_add(&mp_evt_handlers_head, (list_node_t*) &p_handler_params->p_next);
}

void event_handler_remove(nrf_mesh_evt_handler_t * p_handler_params)
{
    NRF_MESH_ASSERT(p_handler_params != NULL);
    /* This function ignores attempts to remove event handlers that are not in the list. */
    (void) list_remove(&mp_evt_handlers_head, (list_node_t*) &p_handler_params->p_next);
}
