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

#include "app_dtt.h"

#include "utils.h"
#include "mesh_app_utils.h"
#include "sdk_config.h"
#include "example_common.h"
#include "generic_dtt_server.h"

#include "log.h"
#include "generic_dtt_mc.h"

/** This sample implementation shows how the model behavior requirements of Generic OnOff server can
 * be implemented.
 */

/* Forward declaration */
static void generic_dtt_state_get_cb(const generic_dtt_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_dtt_status_params_t * p_out);
static void generic_dtt_state_set_cb(const generic_dtt_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_dtt_set_params_t * p_in,
                                       generic_dtt_status_params_t * p_out);

const generic_dtt_server_callbacks_t m_gen_dtt_cbs =
{
    .dtt_cbs.set_cb = generic_dtt_state_set_cb,
    .dtt_cbs.get_cb = generic_dtt_state_get_cb
};


void generic_dtt_state_get_cb(const generic_dtt_server_t * p_self,
                              const access_message_rx_meta_t * p_meta,
                              generic_dtt_status_params_t * p_out)
{
    app_dtt_server_t * p_app = PARENT_BY_FIELD_GET(app_dtt_server_t, server, p_self);

    ERROR_CHECK(generic_dtt_mc_dtt_state_get(p_app->server.state_handle, &p_out->transition_time_ms));
}

void generic_dtt_state_set_cb(const generic_dtt_server_t * p_self,
                              const access_message_rx_meta_t * p_meta,
                              const generic_dtt_set_params_t * p_in,
                              generic_dtt_status_params_t * p_out)
{
    app_dtt_server_t * p_app = PARENT_BY_FIELD_GET(app_dtt_server_t, server, p_self);

    p_out->transition_time_ms = p_in->transition_time_ms;
    ERROR_CHECK(generic_dtt_mc_dtt_state_set(p_app->server.state_handle, p_out->transition_time_ms));
}

/***** Interface functions *****/
uint32_t app_dtt_init(app_dtt_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->server.settings.p_callbacks = &m_gen_dtt_cbs;

    status = generic_dtt_server_init(&p_app->server, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Set the default state.
     */
    return generic_dtt_mc_open(&p_app->server.state_handle);
}

