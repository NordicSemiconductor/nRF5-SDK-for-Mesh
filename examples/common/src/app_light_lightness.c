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

#include "app_light_lightness.h"

#include <stdlib.h>

#include "nrf_mesh_config_examples.h"
#include "utils.h"
#include "mesh_app_utils.h"
#include "log.h"
#include "nrf_assert.h"
#include "nrf_mesh_assert.h"
#include "app_timer.h"
#include "app_transition.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"

#include "light_lightness_mc.h"
#include "light_lightness_setup_server.h"
#include "light_lightness_utils.h"
#include "model_common.h"
#include "list.h"
#include "bearer_event.h"

/** This sample implementation shows how the model behavior requirements
 * of Light Lightness Setup server can be implemented.
 */

static generic_ponoff_setup_server_callbacks_t m_ponoff_srv_cbs;
static generic_dtt_server_callbacks_t m_dtt_srv_cbs;
static list_node_t * mp_app_light_lightness_head;
static bearer_event_flag_t m_transition_abort_flag;

/* Forward declarations */
static void transition_parameters_set(app_light_lightness_setup_server_t * p_app,
                                      int32_t delta,
                                      const model_transition_t * p_in_transition,
                                      app_transition_type_t transition_type);

static void light_lightness_state_set_cb(const light_lightness_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         const light_lightness_set_params_t * p_in,
                                         const model_transition_t * p_in_transition,
                                         light_lightness_status_params_t * p_out);

static void light_lightness_state_get_cb(const light_lightness_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         light_lightness_status_params_t * p_out);

static void light_lightness_state_last_get_cb(const light_lightness_setup_server_t * p_self,
                                              const access_message_rx_meta_t * p_meta,
                                              light_lightness_last_status_params_t * p_out);

static void light_lightness_state_default_set_cb(const light_lightness_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const light_lightness_default_set_params_t * p_in,
                                                 light_lightness_default_status_params_t * p_out);

static void light_lightness_state_default_get_cb(const light_lightness_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 light_lightness_default_status_params_t * p_out);

static void light_lightness_state_range_set_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_lightness_range_set_params_t * p_in,
                                               light_lightness_range_status_params_t * p_out);

static void light_lightness_state_range_get_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               light_lightness_range_status_params_t * p_out);

static void light_lightness_state_move_set_cb(const light_lightness_setup_server_t * p_self,
                                              const access_message_rx_meta_t * p_meta,
                                              const light_lightness_move_set_params_t * p_in,
                                              const model_transition_t * p_in_transition,
                                              light_lightness_status_params_t * p_out);


static void light_lightness_state_delta_set_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_lightness_delta_set_params_t * p_in,
                                               const model_transition_t * p_in_transition,
                                               light_lightness_status_params_t * p_out);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_light_lightness_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                            uint8_t scene_index);
static void app_light_lightness_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                             uint8_t scene_index,
                                             uint32_t delay_ms,
                                             uint32_t transition_time_ms);
static void app_light_lightness_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                             uint8_t scene_index);

const app_scene_callbacks_t m_scene_light_lightness_cbs =
{
    .scene_store_cb = app_light_lightness_scene_store,
    .scene_recall_cb = app_light_lightness_scene_recall,
    .scene_delete_cb = app_light_lightness_scene_delete
};
#endif

/**** end of callback declarations ****/

static const light_lightness_setup_server_callbacks_t light_lightness_setup_srv_cbs =
{
    .light_lightness_cbs.set_cb         = light_lightness_state_set_cb,
    .light_lightness_cbs.get_cb         = light_lightness_state_get_cb,
    .light_lightness_cbs.last_get_cb    = light_lightness_state_last_get_cb,
    .light_lightness_cbs.default_set_cb = light_lightness_state_default_set_cb,
    .light_lightness_cbs.default_get_cb = light_lightness_state_default_get_cb,
    .light_lightness_cbs.range_set_cb   = light_lightness_state_range_set_cb,
    .light_lightness_cbs.range_get_cb   = light_lightness_state_range_get_cb,
    .light_lightness_cbs.move_set_cb    = light_lightness_state_move_set_cb,
    .light_lightness_cbs.delta_set_cb   = light_lightness_state_delta_set_cb
};

static void range_get(uint16_t state_handle, light_lightness_range_status_params_t * p_range)
{
    ERROR_CHECK(light_lightness_mc_range_min_state_get(state_handle, &p_range->range_min));
    ERROR_CHECK(light_lightness_mc_range_max_state_get(state_handle, &p_range->range_max));
    ERROR_CHECK(light_lightness_mc_range_status_state_get(state_handle, &p_range->status));
}

static uint16_t target_snapshot_get(uint16_t state_handle, uint16_t lightness)
{
    light_lightness_range_status_params_t range;
    uint16_t target_snapshot;

    /* If delta is too large, target value should get clipped to range limits */
    range_get(state_handle, &range);

    /* There is a special case for lightness value of `0`. If requested lightness is zero, and
     * present value is also zero, target should be zero */
    target_snapshot = lightness > range.range_max ? range.range_max :
                      lightness < range.range_min && lightness != 0 ? range.range_min :
                      lightness;

    return target_snapshot;
}
/***** Light Lightness model interface callbacks ****/

static void light_lightness_state_get_cb(const light_lightness_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         light_lightness_status_params_t * p_out)
{
    app_light_lightness_setup_server_t * p_app;
    uint16_t present_lightness;
    light_lightness_range_status_params_t range;

    p_app = PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                                light_lightness_setup_server,
                                p_self);
    NRF_MESH_ASSERT(p_app && p_out);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Requirement: Provide the current value of the light_lightness state */
    p_app->app_light_lightness_get_cb(p_app, &present_lightness);
    p_out->present_lightness = present_lightness;

    /* Requirement: Report remaining time during processing of SET or DELTA SET,
     *              Report zero/unknown transition time during processing of MOVE. */
    if (p_params->transition_type == APP_TRANSITION_TYPE_MOVE_SET)
    {
        range_get(p_self->state.handle, &range);
        p_out->target_lightness = p_app->state.target_lightness == range.range_max ? UINT16_MAX : 0;
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0 || p_params->required_delta == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
    else
    {
        p_out->target_lightness = p_app->state.target_lightness;
        p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "GET Lightness: pr: %d tgt: %d tt: %d\n",
          p_out->present_lightness, p_out->target_lightness, p_out->remaining_time_ms);
}

static void light_lightness_state_set_cb(const light_lightness_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         const light_lightness_set_params_t * p_in,
                                         const model_transition_t * p_in_transition,
                                         light_lightness_status_params_t * p_out)
{
    app_light_lightness_setup_server_t * p_app;
    uint16_t present_lightness;
    uint32_t transition_time_ms;

    p_app = PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                                light_lightness_setup_server,
                                p_self);
    NRF_MESH_ASSERT(p_app && p_in);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Notify anyone interested that we just got a command to set the lightness */
    if (p_app->app_add_notify.app_notify_set_cb != NULL)
    {
        p_app->app_add_notify.app_notify_set_cb(p_app->app_add_notify.p_app_notify_v, p_in->lightness);
    }

    p_app->app_light_lightness_get_cb(p_app, &present_lightness);

    p_app->state.target_snapshot = target_snapshot_get(p_self->state.handle, p_in->lightness);
    p_app->state.init_present_snapshot = p_app->state.present_lightness;

    ERROR_CHECK(light_lightness_mc_actual_state_set(p_self->state.handle, p_app->state.target_snapshot));
    if (p_app->state.target_snapshot >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_self->state.handle, p_app->state.target_snapshot,
                                                      LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_FLASH_ONLY));
    }

    if (!p_self->state.initialized || (present_lightness != p_in->lightness))
    {
        (void) transition_parameters_set(p_app,
                                         (int32_t)p_app->state.target_snapshot - (int32_t)present_lightness,
                                         p_in_transition,
                                         APP_TRANSITION_TYPE_SET);

        transition_time_ms = p_params->transition_time_ms;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "SET: target lightness: %d  delay: %d  tt: %d  req-delta: %d \n",
              p_app->state.target_snapshot,
              p_app->state.transition.delay_ms,
              p_params->transition_time_ms,
              p_params->required_delta);

        app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
        app_scene_model_scene_changed(p_app->p_app_scene);
#endif
    }
    else
    {
        transition_time_ms = 0;
    }

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_lightness = p_app->state.present_lightness;
        p_out->target_lightness  = p_app->state.target_snapshot;
        p_out->remaining_time_ms = transition_time_ms;
    }
}

static void light_lightness_state_delta_set_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_lightness_delta_set_params_t * p_in,
                                               const model_transition_t * p_in_transition,
                                               light_lightness_status_params_t * p_out)
{
    app_light_lightness_setup_server_t * p_app;
    int32_t target_lightness;
    light_lightness_range_status_params_t range;

    p_app = PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                                light_lightness_setup_server,
                                p_self);
    NRF_MESH_ASSERT(p_app && p_in);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Notify anyone interested that we just got a command to set the lightness */
    if (p_app->app_add_notify.app_notify_set_cb != NULL)
    {
        p_app->app_add_notify.app_notify_set_cb(p_app->app_add_notify.p_app_notify_v, p_in->delta_lightness);
    }

    p_app->state.new_tid = model_transaction_is_new(&p_app->light_lightness_setup_server.light_lightness_srv.tid_tracker);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tid: %d %s\n", p_in->tid, p_app->state.new_tid ?
          "New TID" : "Same TID, cumulative delta set");

    p_app->app_light_lightness_get_cb(p_app, &p_app->state.present_lightness);

    /* If delta is too large, target value should get clipped to range limits */
    range_get(p_self->state.handle, &range);

    if (p_app->state.new_tid)
    {
        p_app->state.init_present_snapshot = p_app->state.present_lightness;
    }

    target_lightness = p_app->state.new_tid ?
                        (int32_t)p_app->state.present_lightness + p_in->delta_lightness :
                        (int32_t)p_app->state.init_present_snapshot + p_in->delta_lightness;

    /* There is a special case for target lightness values of less than or equal to `0`.
     * If calculated target is less than or equal to zero, target should be zero. */
    p_app->state.target_snapshot = target_lightness <= 0 ? 0 :
                                   target_lightness > range.range_max ? range.range_max :
                                   target_lightness < range.range_min ? range.range_min :
                                   target_lightness;

    (void) transition_parameters_set(p_app,
                                     p_in->delta_lightness,
                                     p_in_transition,
                                     APP_TRANSITION_TYPE_DELTA_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Delta SET: delta: %d  delay: %d  tt: %d target: %d\n",
          p_in->delta_lightness,
          p_app->state.transition.delay_ms,
          p_params->transition_time_ms,
          p_app->state.target_snapshot);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (p_in->delta_lightness != 0)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_lightness = p_app->state.present_lightness;
        p_out->target_lightness  = p_app->state.target_snapshot;
        p_out->remaining_time_ms = p_params->transition_time_ms;
    }
}

static void light_lightness_state_move_set_cb(const light_lightness_setup_server_t * p_self,
                                              const access_message_rx_meta_t * p_meta,
                                              const light_lightness_move_set_params_t * p_in,
                                              const model_transition_t * p_in_transition,
                                              light_lightness_status_params_t * p_out)
{
    app_light_lightness_setup_server_t * p_app;
    light_lightness_range_status_params_t range;

    p_app = PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                                light_lightness_setup_server,
                                p_self);
    NRF_MESH_ASSERT(p_app && p_in);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Notify anyone interested that we just got a command to set the lightness */
    if (p_app->app_add_notify.app_notify_set_cb != NULL)
    {
        p_app->app_add_notify.app_notify_set_cb(p_app->app_add_notify.p_app_notify_v, p_in->delta);
    }

    transition_parameters_set(p_app, (int32_t)p_in->delta, p_in_transition,
                              APP_TRANSITION_TYPE_MOVE_SET);

    /* Requirement: For the status message: The target Generic Lightness state is the upper limit of
       the Generic Lightness state when the transition speed is positive, or the lower limit of the
       Generic Lightness state when the transition speed is negative. */
    p_app->app_light_lightness_get_cb(p_app, &p_app->state.present_lightness);
    range_get(p_self->state.handle, &range);

    if (p_in->delta > 0 &&
        p_app->state.transition.requested_params.transition_time_ms > 0)
    {
        p_app->state.target_snapshot = range.range_max;
    }
    else if (p_in->delta < 0 &&
             p_app->state.transition.requested_params.transition_time_ms > 0)
    {
        p_app->state.target_snapshot = range.range_min;
    }
    else
    {
        p_app->state.target_snapshot = p_app->state.present_lightness;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "MOVE SET: move-lightness: %d  delay: %d  tt: %d \n",
          p_in->delta,
          p_app->state.transition.delay_ms,
          p_params->transition_time_ms);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if ((p_in->delta != 0) && (p_app->state.transition.requested_params.transition_time_ms > 0))
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_lightness = p_app->state.present_lightness;
        /* Insert values as expected in level status message for Move Set */
        p_out->target_lightness = p_app->state.target_lightness == range.range_max ? UINT16_MAX : 0;
        /* Response to Move Set message is sent with unknown transition time value, if
        given transition time is non-zero. */
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0 || p_in->delta == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
}

/* Callback for updating the OnPowerUp state */
static void app_onpowerup_server_set_cb(const generic_ponoff_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        const generic_ponoff_set_params_t * p_in,
                                        generic_ponoff_status_params_t * p_out)
{
    light_lightness_setup_server_t * p_s_server;

    NRF_MESH_ASSERT(p_self && p_in);


    p_s_server = PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                                     generic_ponoff_setup_srv,
                                     p_self);

    ERROR_CHECK(light_lightness_mc_onpowerup_state_set(p_s_server->state.handle, p_in->on_powerup));

    if (p_out)
    {
        p_out->on_powerup = p_in->on_powerup;
    }
}

static void app_onpowerup_server_get_cb(const generic_ponoff_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        generic_ponoff_status_params_t * p_out)
{
    light_lightness_setup_server_t * p_s_server;

    NRF_MESH_ASSERT(p_self && p_out);

    p_s_server = PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                                     generic_ponoff_setup_srv,
                                     p_self);
    /* get the value of the state back from the flash */
    ERROR_CHECK(light_lightness_mc_onpowerup_state_get(p_s_server->state.handle, &p_out->on_powerup));
}

/* Callback for reading the DTT state */
void app_dtt_server_get_cb(const generic_dtt_server_t * p_self,
                           const access_message_rx_meta_t * p_meta,
                           generic_dtt_status_params_t * p_out)
{
    generic_ponoff_setup_server_t * p_ponoff_s_server;
    light_lightness_setup_server_t * p_s_server;

    NRF_MESH_ASSERT(p_self && p_out);

    p_ponoff_s_server =
        PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t,
                            generic_dtt_srv,
                            p_self);

    p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            generic_ponoff_setup_srv,
                            p_ponoff_s_server);

    /* get the value of the state back from the flash */
    ERROR_CHECK(light_lightness_mc_dtt_state_get(p_s_server->state.handle, &p_out->transition_time_ms));
}


/* Callback for updating the dtt state */
void app_dtt_server_set_cb(const generic_dtt_server_t * p_self,
                           const access_message_rx_meta_t * p_meta,
                           const generic_dtt_set_params_t * p_in,
                           generic_dtt_status_params_t * p_out)
{
    generic_dtt_status_params_t status_params;
    generic_ponoff_setup_server_t * p_ponoff_s_server;
    light_lightness_setup_server_t * p_s_server;

    NRF_MESH_ASSERT(p_self && p_in);

    p_ponoff_s_server = PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t,
                                            generic_dtt_srv,
                                            p_self);

    p_s_server = PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                                     generic_ponoff_setup_srv,
                                     p_ponoff_s_server);

    ERROR_CHECK(light_lightness_mc_dtt_state_set(p_s_server->state.handle, p_in->transition_time_ms));
    status_params.transition_time_ms = p_in->transition_time_ms;

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) generic_dtt_server_status_publish((generic_dtt_server_t *)p_self,
                                             (const generic_dtt_status_params_t *)&status_params);

    if (p_out)
    {
        p_out->transition_time_ms = p_in->transition_time_ms;
    }
}

static void light_lightness_state_last_get_cb(const light_lightness_setup_server_t * p_self,
                                              const access_message_rx_meta_t * p_meta,
                                              light_lightness_last_status_params_t * p_out)
{
    NRF_MESH_ASSERT(p_out);

    /* Requirement: Provide the current value of the light_lightness last state */
    /* Last is stored state. No callback to the app is required.*/
    ERROR_CHECK(light_lightness_mc_last_state_get(p_self->state.handle, &p_out->lightness));
}

static void light_lightness_state_default_get_cb(const light_lightness_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 light_lightness_default_status_params_t * p_out)
{
    NRF_MESH_ASSERT(p_out);

    /* Requirement: Provide the current value of the light_lightness default state */
    /* default is stored state. No callback to the top app is required.*/
    ERROR_CHECK(light_lightness_mc_default_state_get(p_self->state.handle, &p_out->lightness));
}

static void publish_default(const light_lightness_setup_server_t *p_setup_server,
                            uint16_t value)
{
    light_lightness_default_status_params_t default_status_params = {0};

    NRF_MESH_ASSERT(p_setup_server);

    default_status_params.lightness = value;

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) light_lightness_server_default_status_publish(&p_setup_server->light_lightness_srv,
                                                         &default_status_params);
}

static void light_lightness_state_default_set_cb(const light_lightness_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const light_lightness_default_set_params_t * p_in,
                                                 light_lightness_default_status_params_t * p_out)
{
    NRF_MESH_ASSERT(p_in);

    /* Update internal representation of default value. */
    ERROR_CHECK(light_lightness_mc_default_state_set(p_self->state.handle, p_in->lightness));
    publish_default(p_self, p_in->lightness);

    if (p_out)
    {
        /* Prepare response */
        p_out->lightness = p_in->lightness;
    }
}

static void light_lightness_state_range_get_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               light_lightness_range_status_params_t * p_out)
{
    NRF_MESH_ASSERT(p_out);

    light_lightness_range_status_params_t range;
    range_get(p_self->state.handle, &range);
    p_out->range_min = range.range_min;
    p_out->range_max = range.range_max;
    p_out->status = range.status;
}

static void publish_range(const light_lightness_setup_server_t * p_setup_server,
                          light_lightness_range_status_params_t * p_value)
{
    NRF_MESH_ASSERT(p_setup_server);

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) light_lightness_server_range_status_publish(&p_setup_server->light_lightness_srv,
                                                       (const light_lightness_range_status_params_t *)p_value);
}

static void light_lightness_state_range_set_cb(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_lightness_range_set_params_t * p_in,
                                               light_lightness_range_status_params_t * p_out)
{
    light_lightness_range_status_params_t value;

    NRF_MESH_ASSERT(p_in);

    range_get(p_self->state.handle, &value);

    value.status = LIGHT_LIGHTNESS_RANGE_STATUS_SUCCESS;
    /* value.min and value.max give
     * the Light Lightness Range state of the Light Lightness Server */

    /* Determine if the message requires a Range state update. */
    if (p_in->range_min < LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN)
    {
        value.status = LIGHT_LIGHTNESS_RANGE_STATUS_CANNOT_SET_RANGE_MIN;
        /* The message does not require a Range state update */
    }
    else if (p_in->range_max > LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX)
    {
        value.status = LIGHT_LIGHTNESS_RANGE_STATUS_CANNOT_SET_RANGE_MAX;
        /* The message does not require a Range state update */
    }
    else
    {
        /* The range values are valid.
            * The message requires a Range state update. */
        value.range_min = p_in->range_min;
        value.range_max = p_in->range_max;
        ERROR_CHECK(light_lightness_mc_range_min_state_set(p_self->state.handle, value.range_min));
        ERROR_CHECK(light_lightness_mc_range_max_state_set(p_self->state.handle, value.range_max));
        ERROR_CHECK(light_lightness_mc_range_status_state_set(p_self->state.handle, value.status));

        publish_range(p_self, &value);
    }

    if (p_out)
    {
        /* Prepare for a response.
         */
        p_out->range_min = value.range_min;
        p_out->range_max = value.range_max;
        p_out->status = value.status;
    }
}

/***/
static uint32_t app_light_lightness_setup_server_init(light_lightness_setup_server_t * p_setup_server,
                                                      uint8_t element_index)
{
    uint32_t status;

    if (!p_setup_server)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize the Light Lightnes Setup server callback structure.
     */
    p_setup_server->settings.p_callbacks = &light_lightness_setup_srv_cbs;

    /* The setup server will initialize a generic PonOff instance.
    *  Initialize the PonOff callback structure.
    */
    m_ponoff_srv_cbs.ponoff_cbs.set_cb = app_onpowerup_server_set_cb;
    m_ponoff_srv_cbs.ponoff_cbs.get_cb = app_onpowerup_server_get_cb;
    p_setup_server->generic_ponoff_setup_srv.settings.p_callbacks = &m_ponoff_srv_cbs;

    /* The generic PonOff instance will initialize a DTT instance.
     * Initialize the DTT callback structure.
     */
    m_dtt_srv_cbs.dtt_cbs.set_cb = app_dtt_server_set_cb;
    m_dtt_srv_cbs.dtt_cbs.get_cb = app_dtt_server_get_cb;
    p_setup_server->generic_ponoff_setup_srv.generic_dtt_srv.settings.p_callbacks = &m_dtt_srv_cbs;

    /* Set the default state.
     */
    status = light_lightness_mc_open(&p_setup_server->state.handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    p_setup_server->state.initialized = false;

    return light_lightness_setup_server_init(p_setup_server, element_index);
}

static uint32_t publish_lightness(app_light_lightness_setup_server_t * p_app, uint32_t remaining_time_ms)
{
    light_lightness_status_params_t publication_data;
    light_lightness_server_t * p_light_lightness_srv;

    publication_data.present_lightness = p_app->state.present_lightness;
    publication_data.target_lightness  = p_app->state.target_lightness;
    publication_data.remaining_time_ms = remaining_time_ms;

    /* Notify anyone interested that we are publishing lightness */
    if (p_app->app_add_notify.app_add_publish_cb != NULL)
    {
        p_app->app_add_notify.app_add_publish_cb(p_app->app_add_notify.p_app_publish_v, &publication_data);
    }

    p_light_lightness_srv = &p_app->light_lightness_setup_server.light_lightness_srv;
    return light_lightness_server_status_publish((const light_lightness_server_t * )p_light_lightness_srv,
                                                 (const light_lightness_status_params_t *)&publication_data);
}


/***** Transition module callback function implementation *****/
/* Note: The task of the transition module is to implement gradual changes of `present_lightness` value
 * for all possible time intervals and step size combinations allowed by the lightness model, and the
 * callbacks from this module are used to implement actual lightness value changes.
 *
 * If you need to implement transitions using external hardware or other mechanisms you can use
 * individual callbacks to initiate specific actions on external hardware.
 */

static bool m_transition_abort_flag_cb(void)
{
    LIST_FOREACH(p_iter, mp_app_light_lightness_head)
    {
        app_light_lightness_setup_server_t * p_app = PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                                                                         node,
                                                                         p_iter);
        if (p_app->abort_move)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Move stopped\n");
            p_app->abort_move = false;
            app_transition_abort(&p_app->state.transition);
        }
    }
    return true;
}

static inline app_light_lightness_setup_server_t * transition_to_app(const app_transition_t * p_transition)
{
    app_light_lightness_state_t   * p_state;

    p_state = PARENT_BY_FIELD_GET(app_light_lightness_state_t,
                                  transition,
                                  p_transition);

    return PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t,
                               state,
                               p_state);
}

static void transition_delay_start_cb(const app_transition_t * p_transition)
{
#if NRF_MESH_LOG_ENABLE
    app_light_lightness_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Element %d: starting delay, present-lightness: %d\n",
          p_app->light_lightness_setup_server.settings.element_index,
          p_app->state.present_lightness);
#endif
}

static void transition_start_cb(const app_transition_t * p_transition)
{
    app_light_lightness_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    if((p_app->state.transition.requested_params.transition_type == APP_TRANSITION_TYPE_DELTA_SET &&
       p_app->state.new_tid) ||
       (p_app->state.transition.requested_params.transition_type == APP_TRANSITION_TYPE_SET) ||
       (p_app->state.transition.requested_params.transition_type == APP_TRANSITION_TYPE_MOVE_SET))
    {
       p_app->state.initial_present_lightness = p_app->state.init_present_snapshot;
    }
    p_app->state.target_lightness = p_app->state.target_snapshot;

    p_app->state.published_ms = 0;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Element %d: starting transition: initial-l: %d delta: %d tt: %d\n",
          p_app->light_lightness_setup_server.settings.element_index,
          p_app->state.initial_present_lightness,
          p_params->required_delta,
          p_params->transition_time_ms);

    if (p_app->app_light_lightness_transition_cb != NULL)
    {
        p_app->app_light_lightness_transition_cb(p_app,
                    p_params->transition_time_ms,
                    p_app->state.target_lightness);
    }
}


/* 6.1.2.2.5 Binding with the Light Lightness Range state
 * Light Lightness Actual = Light Lightness Range Min
 *   for non-zero values of the Light Lightness Actual state that are less than
 *   the value of the Light Lightness Range Min state
 * Light Lightness Actual = Light Lightness Range Max
 *   for non-zero values of the Light Lightness Actual state that are greater than
 *   the value of the Light Lightness Range Max state
 */
static void present_lightness_set(app_light_lightness_setup_server_t * p_app)
{
    if (p_app->app_light_lightness_set_cb != NULL)
    {
        p_app->app_light_lightness_set_cb(p_app, p_app->state.present_lightness);
    }
}

static void present_lightness_set_with_range_clip(app_light_lightness_setup_server_t * p_app)
{
    light_lightness_range_status_params_t range;

    range_get(p_app->light_lightness_setup_server.state.handle, &range);
    p_app->state.present_lightness =
            light_lightness_utils_actual_to_range_restrict(p_app->state.present_lightness,
                                                           range.range_min, range.range_max);
    present_lightness_set(p_app);
}

static inline bool lightness_limit_check(uint16_t present_lightness,
                                         light_lightness_range_status_params_t * p_range,
                                         int32_t required_delta)
{
    /* Return true, if the present value is at the range limits */
    return ((required_delta > 0 && present_lightness == p_range->range_max) ||
            (required_delta < 0 && present_lightness == p_range->range_min));
}

static void transition_tick_cb(const app_transition_t * p_transition)
{
    int32_t present_lightness;
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    light_lightness_range_status_params_t range;
    app_light_lightness_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);
    elapsed_ms = app_transition_elapsed_time_get((app_transition_t *)p_transition);
    remaining_ms = app_transition_remaining_time_get((app_transition_t *)p_transition);

    /* Calculate new value using linear interpolation and provide to the application. */
    if (p_params->transition_type != APP_TRANSITION_TYPE_MOVE_SET)
    {
        int32_t delta = (p_app->state.target_lightness - p_app->state.initial_present_lightness);
        present_lightness = p_app->state.initial_present_lightness +
            (delta * (int64_t)elapsed_ms /
             (int32_t)p_params->transition_time_ms);
    }
    else
    {
        present_lightness = p_app->state.initial_present_lightness +
            (((int64_t)elapsed_ms * (int64_t)p_params->required_delta) /
             (int64_t)p_params->transition_time_ms);
    }

    range_get(p_app->light_lightness_setup_server.state.handle, &range);

    p_app->state.present_lightness = present_lightness == 0 ? 0 :
                                     MIN((int32_t)MAX(present_lightness, range.range_min), range.range_max);
    present_lightness_set(p_app);
    if (p_params->transition_type == APP_TRANSITION_TYPE_MOVE_SET &&
        lightness_limit_check(p_app->state.present_lightness, &range, p_params->required_delta))
    {
        /* Stop transition for lightness */
        p_app->abort_move = true;
        bearer_event_flag_set(m_transition_abort_flag);
    }

    /* While the transition has at least one half second remaining, */
    /* publish intermediate lightness at one second intervals. */
    if ((remaining_ms > 500) && (elapsed_ms - p_app->state.published_ms > 1000))
    {
        p_app->state.published_ms = elapsed_ms;
        /* Ignore status, as publish may fail due to several reasons and it is ok. */
        (void) publish_lightness(p_app, remaining_ms);
    }
}

static void transition_complete_cb(const app_transition_t * p_transition)
{
    app_light_lightness_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* This handles a case, when a new transition has a non-zero delay, but zero transition time.
     * This also handles a case, when such request is received in the middle of another transition.
     * In usual cases when transition time is non-zero, this assignment will have no effect since
     * this copying is already done at start of the transition. */
    p_app->state.target_lightness = p_app->state.target_snapshot;

    if (p_params->transition_type != APP_TRANSITION_TYPE_MOVE_SET)
    {
        p_app->state.present_lightness = p_app->state.target_lightness;
    }

    present_lightness_set_with_range_clip(p_app);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
      "Element %d: transition completed, present-L: %d\n",
      p_app->light_lightness_setup_server.settings.element_index,
      p_app->state.present_lightness);

    /* The transition is complete */
    if (p_app->app_light_lightness_transition_cb != NULL)
    {
        p_app->app_light_lightness_transition_cb(p_app,
                    p_params->transition_time_ms,
                    p_app->state.target_lightness);
    }

    if (p_app->state.present_lightness >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_app->light_lightness_setup_server.state.handle,
                                                      p_app->state.present_lightness,
                                                      LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL));
    }
    ERROR_CHECK(light_lightness_mc_actual_state_set(p_app->light_lightness_setup_server.state.handle,
                                                    p_app->state.present_lightness));

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) publish_lightness(p_app, 0);
}

static void transition_parameters_set(app_light_lightness_setup_server_t * p_app,
                                      int32_t delta,
                                      const model_transition_t * p_in_transition,
                                      app_transition_type_t transition_type)
{
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_params->transition_type = transition_type;
    p_params->required_delta  = delta;
    p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

    /* Requirement: If transition time parameters are unavailable and default transition time state
    is not available, transition shall be instantaneous. */
    if (p_in_transition == NULL)
    {
        p_app->state.transition.delay_ms = 0;

        ERROR_CHECK(light_lightness_mc_dtt_state_get(p_app->light_lightness_setup_server.state.handle,
                                                     &p_params->transition_time_ms));
    }
    else
    {
        p_app->state.transition.delay_ms = p_in_transition->delay_ms;
        p_params->transition_time_ms = p_in_transition->transition_time_ms;
    }
}


/***** Scene Interface functions *****/

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_light_lightness_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                            uint8_t scene_index)
{
    app_light_lightness_setup_server_t * p_app = 
                PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t, scene_if, p_app_scene_if);
    ERROR_CHECK(light_lightness_mc_scene_actual_state_store(
                    p_app->light_lightness_setup_server.state.handle, 
                    scene_index, 
                    p_app->state.present_lightness));

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
            "SCENE STORE: handle: %d  scene index: %d  present lightness: %d\n",
            p_app->light_lightness_setup_server.state.handle,
            scene_index,
            p_app->state.present_lightness);
}

static void app_light_lightness_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                             uint8_t scene_index,
                                             uint32_t delay_ms,
                                             uint32_t transition_time_ms)
{
    uint16_t present_lightness;
    uint16_t recall_lightness;

    app_light_lightness_setup_server_t * p_app = 
                PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t, scene_if, p_app_scene_if);

    p_app->app_light_lightness_get_cb(p_app, &present_lightness);

    ERROR_CHECK(light_lightness_mc_scene_actual_state_recall(
                    p_app->light_lightness_setup_server.state.handle, 
                    scene_index, 
                    &recall_lightness));

    p_app->state.target_snapshot = 
        target_snapshot_get(p_app->light_lightness_setup_server.state.handle, recall_lightness);
    p_app->state.init_present_snapshot = p_app->state.present_lightness;

    ERROR_CHECK(light_lightness_mc_actual_state_set(p_app->light_lightness_setup_server.state.handle, 
                                                    p_app->state.target_snapshot));

    if (p_app->state.target_snapshot >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_app->light_lightness_setup_server.state.handle,
                                                      p_app->state.target_snapshot,
                                                      LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_FLASH_ONLY));
    }

    model_transition_t in_transition = {.delay_ms = delay_ms, 
                                        .transition_time_ms = transition_time_ms};

    if (!p_app->light_lightness_setup_server.state.initialized || (present_lightness != recall_lightness))
    {
        app_transition_abort(&p_app->state.transition);
        
        transition_parameters_set(p_app, 
                                  (int32_t)p_app->state.target_snapshot - (int32_t)present_lightness,
                                  &in_transition, 
                                  APP_TRANSITION_TYPE_SET);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "SCENE RECALL: target lightness: %d  delay: %d  tt: %d \n",
            p_app->state.target_snapshot,
            in_transition.delay_ms,
            in_transition.transition_time_ms);

        app_transition_trigger(&p_app->state.transition);
    }
}

static void app_light_lightness_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                             uint8_t scene_index)
{
    app_light_lightness_setup_server_t * p_app = 
                PARENT_BY_FIELD_GET(app_light_lightness_setup_server_t, scene_if, p_app_scene_if);
    app_transition_abort(&p_app->state.transition);
    /* No need to do anything else */
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0 */

/***** Interface functions *****/

uint32_t app_light_lightness_model_init(app_light_lightness_setup_server_t * p_app, uint8_t element)
{
    uint32_t status = NRF_ERROR_NULL;
    if (p_app == NULL)
    {
        return status;
    }

    if ( (p_app->app_light_lightness_get_cb == NULL) ||
         ( (p_app->app_light_lightness_set_cb == NULL) &&
           (p_app->app_light_lightness_transition_cb == NULL) ) )
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize a light lightness setup server.
     * This will also initialize all the models that the light lightness setup server extends.
     */
    status = app_light_lightness_setup_server_init(&p_app->light_lightness_setup_server, element);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    p_app->scene_if.p_callbacks = &m_scene_light_lightness_cbs;
#endif

    p_app->state.transition.delay_start_cb = transition_delay_start_cb;
    p_app->state.transition.transition_start_cb = transition_start_cb;
    p_app->state.transition.transition_tick_cb = transition_tick_cb;
    p_app->state.transition.transition_complete_cb = transition_complete_cb;
    p_app->state.transition.p_context = &p_app->state.transition;
    p_app->abort_move = false;

    status = app_transition_init(&p_app->state.transition);

    if (status == NRF_SUCCESS)
    {
        list_add(&mp_app_light_lightness_head, &p_app->node);
        m_transition_abort_flag = bearer_event_flag_add(m_transition_abort_flag_cb);
    }

    return status;
}

uint32_t app_light_lightness_binding_setup(app_light_lightness_setup_server_t * p_app)
{
    light_lightness_saved_values_t state;
    light_lightness_setup_server_t * p_s_server;
    uint16_t index;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_s_server = &p_app->light_lightness_setup_server;
    index    = p_s_server->state.handle;

    ERROR_CHECK(light_lightness_mc_range_status_state_get(index, &state.range.status));
    ERROR_CHECK(light_lightness_mc_range_min_state_get(index, &state.range.range_min));
    ERROR_CHECK(light_lightness_mc_range_max_state_get(index, &state.range.range_max));
    ERROR_CHECK(light_lightness_mc_onpowerup_state_get(index, &state.onpowerup));
    ERROR_CHECK(light_lightness_mc_actual_state_get(index, &state.actual_lightness));
    ERROR_CHECK(light_lightness_mc_last_state_get(index, &state.last_lightness));
    ERROR_CHECK(light_lightness_mc_default_state_get(index, &state.default_lightness));

    return light_lightness_ponoff_binding_setup(p_s_server, &state);
}

uint32_t app_light_lightness_current_value_publish(app_light_lightness_setup_server_t * p_app)
{
    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    app_transition_abort(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    uint16_t old_present_lightness = p_app->state.present_lightness;
#endif

    p_app->app_light_lightness_get_cb(p_app, &p_app->state.present_lightness);

    if (p_app->state.present_lightness >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_app->light_lightness_setup_server.state.handle,
                                                      p_app->state.present_lightness,
                                                      LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL));
    }
    ERROR_CHECK(light_lightness_mc_actual_state_set(p_app->light_lightness_setup_server.state.handle,
                                                    p_app->state.present_lightness));

    /* Notify anyone interested that we just got a command to set the lightness */
    if (p_app->app_add_notify.app_notify_set_cb != NULL)
    {
        p_app->app_add_notify.app_notify_set_cb(p_app->app_add_notify.p_app_notify_v, p_app->state.present_lightness);
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (old_present_lightness != p_app->state.present_lightness)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    return publish_lightness(p_app, 0);
}

uint32_t app_light_lightness_direct_actual_set(app_light_lightness_setup_server_t * p_app, 
                                               uint16_t lightness_value)
{
    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    uint16_t old_present_lightness = p_app->state.present_lightness;
#endif

    p_app->state.present_lightness = lightness_value;

    present_lightness_set_with_range_clip(p_app);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* The transition is complete */
    app_transition_abort(&p_app->state.transition);
    if (p_app->app_light_lightness_transition_cb != NULL)
    {
        p_app->app_light_lightness_transition_cb(p_app,
                    p_params->transition_time_ms,
                    lightness_value);
    }

    if (p_app->state.present_lightness >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_app->light_lightness_setup_server.state.handle,
                                                      p_app->state.present_lightness,
                                                      LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL));
    }
    ERROR_CHECK(light_lightness_mc_actual_state_set(p_app->light_lightness_setup_server.state.handle,
                                                    p_app->state.present_lightness));

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (old_present_lightness != p_app->state.present_lightness)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    return NRF_SUCCESS;
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t app_light_lightness_scene_context_set(app_light_lightness_setup_server_t * p_app, 
                                               app_scene_setup_server_t  * p_app_scene)
{
    if (p_app == NULL || p_app_scene == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->p_app_scene = p_app_scene;
    return NRF_SUCCESS;
}
#endif
