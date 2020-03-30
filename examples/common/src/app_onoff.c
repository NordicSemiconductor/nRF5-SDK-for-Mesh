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

#include "app_onoff.h"

#include <stdint.h>

#include "utils.h"
#include "sdk_config.h"
#include "example_common.h"
#include "generic_onoff_server.h"
#include "app_transition.h"

#include "log.h"
#include "app_timer.h"

/** This sample implementation shows how the model behavior requirements of Generic OnOff server can
 * be implemented.
 */

/* Forward declaration */
static void generic_onoff_state_get_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_onoff_status_params_t * p_out);
static void generic_onoff_state_set_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_onoff_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_onoff_status_params_t * p_out);

const generic_onoff_server_callbacks_t onoff_srv_cbs =
{
    .onoff_cbs.set_cb = generic_onoff_state_set_cb,
    .onoff_cbs.get_cb = generic_onoff_state_get_cb
};

static void transition_parameters_set(app_onoff_server_t * p_app,
                                      const model_transition_t * p_in_transition)
{
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_params->transition_type = APP_TRANSITION_TYPE_SET;
    p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

    if (p_in_transition == NULL)
    {
        p_app->state.transition.delay_ms = 0;
        p_params->transition_time_ms = 0;
    }
    else
    {
        p_app->state.transition.delay_ms = p_in_transition->delay_ms;
        p_params->transition_time_ms = p_in_transition->transition_time_ms;
    }
}

static void onoff_current_value_update(app_onoff_server_t * p_app)
{
    if (p_app->onoff_set_cb != NULL)
    {
        p_app->onoff_set_cb(p_app, p_app->state.present_onoff);
    }
}

/***** Generic OnOff model interface callbacks *****/

static void generic_onoff_state_get_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_onoff_status_params_t * p_out)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: GET \n");

    app_onoff_server_t   * p_app = PARENT_BY_FIELD_GET(app_onoff_server_t, server, p_self);

    /* Requirement: Provide the current value of the OnOff state */
    p_app->onoff_get_cb(p_app, &p_app->state.present_onoff);
    p_out->present_on_off = p_app->state.present_onoff;
    p_out->target_on_off = p_app->state.target_onoff;

    /* Requirement: Always report remaining time */
    p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
}

static void generic_onoff_state_set_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_onoff_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_onoff_status_params_t * p_out)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: SET: %d\n", p_in->on_off);

    app_onoff_server_t   * p_app = PARENT_BY_FIELD_GET(app_onoff_server_t, server, p_self);

    bool present_on_off;
    p_app->onoff_get_cb(p_app, &present_on_off);
    /* Update internal representation of OnOff value, process timing */
    p_app->value_updated = false;
    p_app->state.target_onoff = p_in->on_off;

    uint32_t transition_time_ms = 0;
    if (present_on_off != p_in->on_off)
    {
        transition_parameters_set(p_app, p_in_transition);
        transition_time_ms = app_transition_requested_get(&p_app->state.transition)->transition_time_ms;
        app_transition_trigger(&p_app->state.transition);
    }

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_on_off = p_app->state.present_onoff;
        p_out->target_on_off = p_app->state.target_onoff;
        p_out->remaining_time_ms = transition_time_ms;
    }
}

static inline app_onoff_server_t * transition_to_app(const app_transition_t * p_transition)
{
    app_onoff_state_t * p_state = PARENT_BY_FIELD_GET(app_onoff_state_t,
                                  transition,
                                  p_transition);
    return PARENT_BY_FIELD_GET(app_onoff_server_t,
                               state,
                               p_state);
}

static void transition_start_cb(const app_transition_t * p_transition)
{
    app_onoff_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    if (p_app->state.initial_present_onoff == 0 && p_app->state.target_onoff == 1)
    {
        p_app->state.present_onoff = p_app->state.target_onoff;
        onoff_current_value_update(p_app);
        generic_onoff_status_params_t status =
        {
            .present_on_off = p_app->state.present_onoff,
            .target_on_off = p_app->state.target_onoff,
        };
        status.remaining_time_ms = app_transition_ongoing_get(&p_app->state.transition)->transition_time_ms;
        (void) generic_onoff_server_status_publish(&p_app->server, &status);
    }

    /* Inform the application that transition have started. */
    if (p_app->onoff_transition_cb != NULL)
    {
        p_app->onoff_transition_cb(p_app, p_params->transition_time_ms, p_app->state.target_onoff);
    }
}

static void transition_complete_cb(const app_transition_t * p_transition)
{
    app_onoff_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    if (p_app->state.present_onoff != p_app->state.target_onoff)
    {
        p_app->state.present_onoff = p_app->state.target_onoff;

        onoff_current_value_update(p_app);

        generic_onoff_status_params_t status = {
                    .present_on_off = p_app->state.present_onoff,
                    .target_on_off = p_app->state.target_onoff,
                    .remaining_time_ms = 0
                };
        (void) generic_onoff_server_status_publish(&p_app->server, &status);
    }

    /* Inform the application that the transition is complete */
    if (p_app->onoff_transition_cb != NULL)
    {
        p_app->onoff_transition_cb(p_app, p_params->transition_time_ms, p_app->state.target_onoff);
    }
}


/***** Interface functions *****/

void app_onoff_status_publish(app_onoff_server_t * p_app)
{
    app_transition_abort(&p_app->state.transition);
    p_app->onoff_get_cb(p_app, &p_app->state.present_onoff);
    p_app->state.target_onoff = p_app->state.present_onoff;

    generic_onoff_status_params_t status = {
                .present_on_off = p_app->state.present_onoff,
                .target_on_off = p_app->state.target_onoff,
                .remaining_time_ms = 0
            };
    (void) generic_onoff_server_status_publish(&p_app->server, &status);
}

uint32_t app_onoff_init(app_onoff_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->server.settings.p_callbacks = &onoff_srv_cbs;
    if ( (p_app->onoff_get_cb == NULL) ||
         ( (p_app->onoff_set_cb == NULL) &&
           (p_app->onoff_transition_cb == NULL) ) )
    {
        return NRF_ERROR_NULL;
    }

    status = generic_onoff_server_init(&p_app->server, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_app->state.transition.delay_start_cb = NULL;
    p_app->state.transition.transition_start_cb = transition_start_cb;
    p_app->state.transition.transition_tick_cb = NULL;
    p_app->state.transition.transition_complete_cb = transition_complete_cb;
    p_app->state.transition.p_context = &p_app->state.transition;

    /* For any onoff transitions, required_delta is always 1 */
    app_transition_requested_get(&p_app->state.transition)->required_delta = 1;

    return app_transition_init(&p_app->state.transition);
}
