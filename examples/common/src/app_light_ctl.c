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
#include "app_light_ctl.h"

#include <stdlib.h>

/* Application */
#include "log.h"
#include "nrf_mesh_config_examples.h"
#include "light_ctl_mc.h"
#include "light_lightness_mc.h"
#include "light_ctl_utils.h"
#include "list.h"
#include "bearer_event.h"

#include "mesh_app_utils.h"

/** This sample implementation shows how the model behavior requirements of the CTL Setup server can
 * be implemented.
 */

#define CTL_ADDITIONAL_PUBLISH_START_MARGIN_MS      (1500)
#define CTL_ADDITIONAL_PUBLISH_INTERVAL_MS          (1000)

#define APP_CTL_TRANSITION_SET APP_TRANSITION_TYPE_SET
#define APP_CTL_TRANSITION_DELTA_SET APP_TRANSITION_TYPE_DELTA_SET
#define APP_CTL_TRANSITION_MOVE_SET APP_TRANSITION_TYPE_MOVE_SET

#define CHECK_INVALID_TEMPERATURE_RANGE(__range)                        \
    if ((__range.temperature32_range_min == light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_UNKNOWN)) || \
        (__range.temperature32_range_max == light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_UNKNOWN))) \
    {                                                                   \
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Unknown Temperature in CTL range\n"); \
    }

static list_node_t * mp_app_light_ctl_head;
static bearer_event_flag_t m_transition_abort_flag;

/* Forward declarations */
static void transition_parameters_set(app_light_ctl_setup_server_t * p_app,
                                      int32_t delta,
                                      const model_transition_t * p_in_transition,
                                      app_transition_type_t transition_type);

static void ctl_state_set_cb(const light_ctl_setup_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const light_ctl_set_params_t * p_in,
                             const model_transition_t * p_in_transition,
                             light_ctl_status_params_t * p_out);

static void ctl_state_get_cb(const light_ctl_setup_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             light_ctl_status_params_t * p_out);

static void ctl_state_temperature32_set_cb(const light_ctl_setup_server_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           const light_ctl_temperature_set_params_t * p_in,
                                           const model_transition_t * p_in_transition,
                                           light_ctl_temperature_status_params_t * p_out);

static void ctl_state_temperature32_get_cb(const light_ctl_setup_server_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           light_ctl_temperature_status_params_t * p_out);

static void ctl_state_default_set_cb(const light_ctl_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     const light_ctl_default_set_params_t * p_in,
                                     light_ctl_default_status_params_t * p_out);

static void ctl_state_default_get_cb(const light_ctl_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     light_ctl_default_status_params_t * p_out);

static void ctl_state_range_set_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const light_ctl_temperature_range_set_params_t * p_in,
                                   light_ctl_temperature_range_status_params_t * p_out);

static void ctl_state_range_get_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   light_ctl_temperature_range_status_params_t * p_out);

static void ctl_state_move_set_cb(const light_ctl_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const light_ctl_temperature_move_set_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  light_ctl_temperature_status_params_t * p_out);

static void ctl_state_delta_set_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const light_ctl_temperature_delta_set_params_t * p_in,
                                   const model_transition_t * p_in_transition,
                                   light_ctl_temperature_status_params_t * p_out);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_light_ctl_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                      uint8_t scene_index);
static void app_light_ctl_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                       uint8_t scene_index,
                                       uint32_t delay_ms,
                                       uint32_t transition_time_ms);
static void app_light_ctl_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                       uint8_t scene_index);

const app_scene_callbacks_t m_scene_light_ctl_cbs =
{
    .scene_store_cb = app_light_ctl_scene_store,
    .scene_recall_cb = app_light_ctl_scene_recall,
    .scene_delete_cb = app_light_ctl_scene_delete
};
#endif

/**** end of callback declarations ****/

static const light_ctl_setup_server_callbacks_t ctl_setup_srv_cbs =
{
    .light_ctl_cbs.set_cb = ctl_state_set_cb,
    .light_ctl_cbs.get_cb = ctl_state_get_cb,
    .light_ctl_cbs.temperature32_set_cb = ctl_state_temperature32_set_cb,
    .light_ctl_cbs.temperature32_get_cb = ctl_state_temperature32_get_cb,
    .light_ctl_cbs.temperature32_range_set_cb = ctl_state_range_set_cb,
    .light_ctl_cbs.temperature32_range_get_cb = ctl_state_range_get_cb,
    .light_ctl_cbs.default_set_cb = ctl_state_default_set_cb,
    .light_ctl_cbs.default_get_cb = ctl_state_default_get_cb,
    .light_ctl_cbs.move_set_cb = ctl_state_move_set_cb,
    .light_ctl_cbs.delta_set_cb = ctl_state_delta_set_cb
};

static int32_t ctl_state_delta_max_compute(app_light_ctl_setup_server_t * p_app,
                                           uint32_t initial_present_temperature32,
                                           uint32_t target_temperature32,
                                           int16_t initial_present_delta_uv,
                                           int16_t target_delta_uv)
{
    int32_t temp_delta;
    int32_t duv_delta;

    /* Compute which is the bigest delta, and return it */
    temp_delta = (int32_t)light_ctl_utils_temperature32_to_temperature(target_temperature32) -
                 (int32_t)light_ctl_utils_temperature32_to_temperature(initial_present_temperature32);
    duv_delta = target_delta_uv - initial_present_delta_uv;

    return (abs(temp_delta) > abs(duv_delta) ? temp_delta : duv_delta);
}

/***** CTL model interface callbacks ****/

static void ctl_state_get_cb(const light_ctl_setup_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             light_ctl_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint16_t lightness;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Requirement: Provide the current value of some of the CTL composite state */
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_app->p_app_ll->app_light_lightness_get_cb(p_app->p_app_ll, &lightness);
    p_out->present_lightness = lightness;
    p_out->target_lightness = p_app->p_app_ll->state.target_lightness;
    p_out->present_temperature32 = present_ctl_state.temperature32;
    p_out->target_temperature32 = p_app->state.target_temperature32;
    /* the ctl_status_params doesn't do delta_uv, so it is unused from the present_ctl_state */

    /* Requirement: Report remaining time during processing of SET or DELTA SET,
     *              Report zero/unknown transition time during processing of MOVE. */
    if (p_params->transition_type == APP_CTL_TRANSITION_MOVE_SET)
    {
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0) ? 0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
    else
    {
        p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
      "GET CTL: pr L: %d T: %d   tgt L: %d T: %d  rem-tt: %d ms req-delta: %d \n",
      p_out->present_lightness,
      light_ctl_utils_temperature32_to_temperature(p_out->present_temperature32),
      p_out->target_lightness,
      light_ctl_utils_temperature32_to_temperature(p_out->target_temperature32),
      p_out->remaining_time_ms,
      p_params->required_delta);
}

static void ctl_state_set_cb(const light_ctl_setup_server_t * p_self,
                             const access_message_rx_meta_t * p_meta,
                             const light_ctl_set_params_t * p_in,
                             const model_transition_t * p_in_transition,
                             light_ctl_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint32_t transition_time_ms;
    light_lightness_set_params_t ll_in;
    light_lightness_status_params_t ll_out;
    light_lightness_setup_server_state_cbs_t * p_ll_srv_cbs;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Set a module flag to let the code know there is a CTL state set occurring */
    p_app->ctl_state_set_active = true;

    /* First, call the light lightness mid app to do the lightness portion of the CTL state */
    ll_in.lightness = p_in->lightness;
    ll_in.tid = p_in->tid;

    p_ll_srv_cbs = (light_lightness_setup_server_state_cbs_t *)
        &(p_app->p_app_ll->light_lightness_setup_server.settings.p_callbacks->light_lightness_cbs);
    p_ll_srv_cbs->set_cb(&p_app->p_app_ll->light_lightness_setup_server, p_meta, &ll_in,
                         p_in_transition, &ll_out);

    /* Process the temperature and delta UV states (CTL specific - no lightness involved) */
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_app->state.target_temp32_snapshot = p_in->temperature32;
    p_app->state.target_duv_snapshot = p_in->delta_uv;
    p_app->state.init_present_temp32_snapshot = p_app->state.present_temperature32;
    p_app->state.init_present_duv_snapshot = p_app->state.present_delta_uv;

    ERROR_CHECK(light_ctl_mc_temperature32_state_set(p_self->state.handle, p_app->state.target_temp32_snapshot));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_set(p_self->state.handle, p_app->state.target_duv_snapshot));

    /* If this is first time since boot, set all of the values, otherwise, check which values to set */
    if (!p_self->state.initialized || (present_ctl_state.temperature32 != p_in->temperature32) ||
        (present_ctl_state.delta_uv != p_in->delta_uv))
    {
        int32_t delta = ctl_state_delta_max_compute(p_app, present_ctl_state.temperature32,
                                                    p_app->state.target_temp32_snapshot,
                                                    present_ctl_state.delta_uv,
                                                    p_app->state.target_duv_snapshot);
        transition_parameters_set(p_app, delta, p_in_transition, APP_CTL_TRANSITION_SET);

        transition_time_ms = p_params->transition_time_ms;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "CTL SET: target lightness: %d temperature: %d duv: %d delay: %d  tt: %d  req-delta: %d \n",
              p_in->lightness,
              light_ctl_utils_temperature32_to_temperature(p_in->temperature32),
              p_in->delta_uv,
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
        p_out->present_lightness = ll_out.present_lightness;
        p_out->target_lightness  = ll_out.target_lightness;
        p_out->present_temperature32 = p_app->state.present_temperature32;
        p_out->target_temperature32 = p_app->state.target_temp32_snapshot;
        p_out->remaining_time_ms = transition_time_ms;
    }
}

static void ctl_state_temperature32_get_cb(const light_ctl_setup_server_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           light_ctl_temperature_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    light_ctl_temperature_range_set_params_t range_set;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Requirement: Provide the current value of some of the CTL composite state */
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_out->present_temperature32 = present_ctl_state.temperature32;
    p_out->present_delta_uv = present_ctl_state.delta_uv;
    p_out->target_delta_uv = p_app->state.target_delta_uv;
    /* the ctl_status_params doesn't do lightness, so it is unused from the present_ctl_state */

    /* Requirement: Report remaining time during processing of SET or DELTA SET,
     *              Report zero/unknown transition time during processing of MOVE. */
    if (p_params->transition_type == APP_CTL_TRANSITION_MOVE_SET)
    {
        ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_self->state.handle, &range_set));
        /* Insert values as expected in level status message */
        p_out->target_temperature32 =
            p_app->state.target_temperature32 == range_set.temperature32_range_max ? INT16_MAX :
                                                 INT16_MIN;
        /* Response to Move Set message is sent with unknown transition time value, if given
         * transition time is non-zero. */
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0 || p_params->required_delta == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
    else
    {
        p_out->target_temperature32 = p_app->state.target_temperature32;
        p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "GET Temp: pr: %d tgt: %d tt: %d\n",
          light_ctl_utils_temperature32_to_temperature(p_out->present_temperature32),
          light_ctl_utils_temperature32_to_temperature(p_out->target_temperature32),
          p_out->remaining_time_ms);
}

static void ctl_state_temperature32_set_cb(const light_ctl_setup_server_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           const light_ctl_temperature_set_params_t * p_in,
                                           const model_transition_t * p_in_transition,
                                           light_ctl_temperature_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint32_t transition_time_ms;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Set a module flag to indicate that CTL Temperature state set is in progress. */
    p_app->ctl_temperature_state_set_active = true;

    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);

    p_app->state.target_temp32_snapshot = p_in->temperature32;
    p_app->state.target_duv_snapshot = p_in->delta_uv;
    p_app->state.init_present_temp32_snapshot = p_app->state.present_temperature32;
    p_app->state.init_present_duv_snapshot = p_app->state.present_delta_uv;


    ERROR_CHECK(light_ctl_mc_temperature32_state_set(p_self->state.handle, p_app->state.target_temp32_snapshot));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_set(p_self->state.handle, p_app->state.target_duv_snapshot));

    /* If this is first time since boot, set all of the values, otherwise, check which values to set */
    if (!p_self->state.initialized || (present_ctl_state.temperature32 != p_in->temperature32) ||
        (present_ctl_state.delta_uv != p_in->delta_uv))
    {
        int32_t delta = ctl_state_delta_max_compute(p_app, present_ctl_state.temperature32,
                                                    p_app->state.target_temp32_snapshot,
                                                    present_ctl_state.delta_uv,
                                                    p_app->state.target_duv_snapshot);
        transition_parameters_set(p_app, delta, p_in_transition, APP_CTL_TRANSITION_SET);

        transition_time_ms = p_params->transition_time_ms;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Temp SET: target temperature: %d  duv: %d,  delay: %d  tt: %d  req-delta: %d \n",
              light_ctl_utils_temperature32_to_temperature(p_in->temperature32),
              p_in->delta_uv,
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
        p_out->present_temperature32 = p_app->state.present_temperature32;
        p_out->target_temperature32 = p_app->state.target_temp32_snapshot;
        p_out->present_delta_uv = p_app->state.present_delta_uv;
        p_out->target_delta_uv = p_app->state.target_duv_snapshot;
        p_out->remaining_time_ms = transition_time_ms;
    }
}

static void ctl_state_default_get_cb(const light_ctl_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     light_ctl_default_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);

    /* Requirement: Provide the current value of the CTL default composite state */
    /* The default is a stored composite state. No callback to the top app is required.*/
    ERROR_CHECK(light_lightness_mc_default_state_get(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                     &p_out->lightness));
    ERROR_CHECK(light_ctl_mc_default_temperature32_state_get(p_self->state.handle, &p_out->temperature32));
    ERROR_CHECK(light_ctl_mc_default_delta_uv_state_get(p_self->state.handle, (int16_t *) &p_out->delta_uv));
}

static void ctl_state_default_set_cb(const light_ctl_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     const light_ctl_default_set_params_t * p_in,
                                     light_ctl_default_status_params_t * p_out)
{
    light_ctl_default_status_params_t default_status;
    app_light_ctl_setup_server_t * p_app;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);

    /* Update internal representation of default value. */
    ERROR_CHECK(light_lightness_mc_default_state_set(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                     p_in->lightness));
    ERROR_CHECK(light_ctl_mc_default_temperature32_state_set(p_self->state.handle, p_in->temperature32));
    ERROR_CHECK(light_ctl_mc_default_delta_uv_state_set(p_self->state.handle, p_in->delta_uv));

    ERROR_CHECK(light_lightness_mc_default_state_get(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                     &default_status.lightness));
    ERROR_CHECK(light_ctl_mc_default_temperature32_state_get(p_self->state.handle, &default_status.temperature32));
    ERROR_CHECK(light_ctl_mc_default_delta_uv_state_get(p_self->state.handle, (int16_t *) &default_status.delta_uv));

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) light_ctl_server_default_status_publish(&p_self->ctl_srv, &default_status);

    if (p_out != NULL)
    {
        /* Prepare response */
        *p_out = default_status;
    }
}

static void ctl_state_range_get_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   light_ctl_temperature_range_status_params_t * p_out)
{
    light_ctl_temperature_range_set_params_t range_set;

    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_self->state.handle, &range_set));
    p_out->status_code = LIGHT_CTL_RANGE_STATUS_GOOD;
    p_out->temperature32_range_min = range_set.temperature32_range_min;
    p_out->temperature32_range_max = range_set.temperature32_range_max;
    /* The CTL server can't function if the temperature is unknown - check for unknown value */
    CHECK_INVALID_TEMPERATURE_RANGE(range_set);
}

static void ctl_state_range_set_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const light_ctl_temperature_range_set_params_t * p_in,
                                   light_ctl_temperature_range_status_params_t * p_out)
{
    light_ctl_temperature_range_status_params_t range;
    light_ctl_temperature_range_set_params_t range_set;

    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_self->state.handle, &range_set));

    /* The CTL server can't function if the temperature is unknown - check for unknown value */
    CHECK_INVALID_TEMPERATURE_RANGE(range_set);

    /* Range min and range max give the CTL Range state of the CTL Server */
    range.status_code = LIGHT_CTL_RANGE_STATUS_GOOD;
    range.temperature32_range_min = range_set.temperature32_range_min;
    range.temperature32_range_max = range_set.temperature32_range_max;

    /* Determine if the message requires a Range state update. */
    if (p_in->temperature32_range_min < light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN))
    {
        range.status_code = LIGHT_CTL_RANGE_STATUS_CANNOT_SET_MIN_RANGE;
        /* The message does not require a Range state update */
    }

    if (p_in->temperature32_range_max > light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX))
    {
        range.status_code = LIGHT_CTL_RANGE_STATUS_CANNOT_SET_MAX_RANGE;
        /* The message does not require a Range state update */
    }

    if (range.status_code == LIGHT_CTL_RANGE_STATUS_GOOD)
    {
        /* The range values are valid.
         * The message requires a Range state update. */
        range.temperature32_range_min = p_in->temperature32_range_min;
        range.temperature32_range_max = p_in->temperature32_range_max;

        range_set.temperature32_range_min = p_in->temperature32_range_min;
        range_set.temperature32_range_max = p_in->temperature32_range_max;
        ERROR_CHECK(light_ctl_mc_temperature32_range_state_set(p_self->state.handle, &range_set));

        /* Ignore status, as publish may fail due to several reasons and it is ok. */
        (void) light_ctl_server_temperature_range_status_publish(&p_self->ctl_srv, &range);
    }

    if (p_out)
    {
        /* Prepare response */
        *p_out = range;
    }
}

/* The delta and move functions are called when a level client sends the associated message to
 * change the temperature state of the CTL state (The lightness and delta UV parts of the CTL state
 * are untouched) */
static void ctl_state_delta_set_cb(const light_ctl_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const light_ctl_temperature_delta_set_params_t * p_in,
                                   const model_transition_t * p_in_transition,
                                   light_ctl_temperature_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    int64_t delta32;
    int64_t target_temperature32;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    NRF_MESH_ASSERT(p_app != NULL && p_in != NULL);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_app->ctl_temperature_state_set_active = true;

    p_app->state.new_tid = model_transaction_is_new(&p_app->light_ctl_setup_srv.ctl_srv.tid_tracker);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tid: %d %s\n", p_in->tid, p_app->state.new_tid ?
          "New TID" : "Same TID, cumulative delta set");

    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);

    delta32 = SCALE_UP((int64_t)p_in->delta_temperature, T32_SCALE_FACTOR);

    /* If delta is too large, target value should get clipped to range limits */
    light_ctl_temperature_range_set_params_t range_set;
    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_self->state.handle, &range_set));

    if (p_app->state.new_tid)
    {
        p_app->state.init_present_temp32_snapshot = p_app->state.present_temperature32;
        p_app->state.init_present_duv_snapshot = p_app->state.present_delta_uv;
    }
    target_temperature32 = p_app->state.new_tid ?
                           present_ctl_state.temperature32 + delta32 :
                           p_app->state.initial_present_temperature32 + delta32;

    p_app->state.target_temp32_snapshot =
        target_temperature32 > (int64_t)range_set.temperature32_range_max ? range_set.temperature32_range_max :
        target_temperature32 < (int64_t)range_set.temperature32_range_min ? range_set.temperature32_range_min :
        target_temperature32;

    transition_parameters_set(p_app, p_in->delta_temperature, p_in_transition, APP_CTL_TRANSITION_DELTA_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Delta SET: delta: %d  delay: %d  tt: %d\n",
          p_in->delta_temperature, p_app->state.transition.delay_ms,
          p_params->transition_time_ms);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (p_in->delta_temperature != 0)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_temperature32 = p_app->state.present_temperature32;
        p_out->target_temperature32 = p_app->state.target_temp32_snapshot;
        p_out->remaining_time_ms = p_params->transition_time_ms;
    }
}

static void ctl_state_move_set_cb(const light_ctl_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const light_ctl_temperature_move_set_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  light_ctl_temperature_status_params_t * p_out)
{
    app_light_ctl_setup_server_t * p_app;
    light_ctl_temperature_range_set_params_t range_set;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, light_ctl_setup_srv, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_app->ctl_temperature_state_set_active = true;
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_app->state.present_temperature32 = present_ctl_state.temperature32;
    p_app->state.present_delta_uv = present_ctl_state.delta_uv;

    /* Requirement: For the status message: The target temperature state is the upper limit of
     * the temperature state when the transition speed is positive, or the lower limit of the
     * temperature state when the transition speed is negative. */
    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_self->state.handle, &range_set));
    transition_parameters_set(p_app, (int32_t)p_in->delta_temperature, p_in_transition,
                              APP_CTL_TRANSITION_MOVE_SET);
    if (p_in->delta_temperature > 0 &&
        p_app->state.transition.requested_params.transition_time_ms > 0)
    {
        p_app->state.target_temp32_snapshot = range_set.temperature32_range_max;
    }
    else if (p_in->delta_temperature < 0 &&
             p_app->state.transition.requested_params.transition_time_ms > 0)
    {
        p_app->state.target_temp32_snapshot = range_set.temperature32_range_min;
    }
    else
    {
        p_app->state.target_temp32_snapshot = p_app->state.present_temperature32;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "MOVE SET: delta: %d  delay: %d  tt: %d\n",
          p_in->delta_temperature, p_app->state.transition.delay_ms,
          p_params->transition_time_ms);
    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if ((p_in->delta_temperature != 0) && 
        (p_app->state.transition.requested_params.transition_time_ms > 0))
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_temperature32 = p_app->state.present_temperature32;
        /* Insert values as expected in level status message for Move Set */
        p_out->target_temperature32 =
            p_app->state.target_temp32_snapshot == range_set.temperature32_range_max ? UINT32_MAX : 0;
        /* Response to Move Set message is sent with unknown transition time value, if given
         * transition time is non-zero. */
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0 || p_in->delta_temperature == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
}

static uint32_t app_ctl_setup_server_init(app_light_ctl_setup_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;
    light_ctl_setup_server_t * p_s_server;

    p_s_server = &p_app->light_ctl_setup_srv;

    /* Initialize the CTL Setup server callback structure. */
    p_s_server->settings.p_callbacks = &ctl_setup_srv_cbs;

    /* Set the default state (initialize flash). */
    status = light_ctl_mc_open(&p_s_server->state.handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    p_s_server->state.initialized = false;

    return light_ctl_setup_server_init(p_s_server, &p_app->p_app_ll->light_lightness_setup_server, element_index);
}

static uint32_t ctl_publish(app_light_ctl_setup_server_t * p_app, uint32_t remaining_time_ms)
{
    light_ctl_status_params_t publication_data;
    light_ctl_temperature_status_params_t temp_publication_data;
    light_ctl_server_t * p_ctl_srv;
    light_ctl_temperature_server_t * p_ctl_temperature_srv;
    uint32_t status1 = NRF_SUCCESS;
    uint32_t status2;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint32_t ll_remaining_time = 0;
    uint16_t lightness;

    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_app->p_app_ll->app_light_lightness_get_cb(p_app->p_app_ll, &lightness);

    if (!p_app->ctl_temperature_state_set_active)
    {
        /* Calculate how much time is remaining for light lightness (if any) */
        if (app_transition_ongoing_get(&p_app->p_app_ll->state.transition)->transition_time_ms > 0)
        {
            ll_remaining_time = app_transition_remaining_time_get(&p_app->p_app_ll->state.transition);
        }

        if (ll_remaining_time == 0)
        {
            /* Lightness is not transitioning, so use the provided time */
            publication_data.remaining_time_ms = remaining_time_ms;
        }
        else
        {
            /* Lightness isn't done transitioning, so use the longer of the 2 transition times */
            publication_data.remaining_time_ms = MAX(remaining_time_ms, ll_remaining_time);
        }
        /* Either a lightness or a CTL command came in - do a ctl status.  Temperature-specific
         * commands don't need a ctl status publish */
        publication_data.present_lightness = lightness;
        publication_data.present_temperature32 = present_ctl_state.temperature32;
        publication_data.target_lightness = p_app->p_app_ll->state.target_lightness;
        publication_data.target_temperature32 = p_app->state.target_temperature32;

        p_ctl_srv = &p_app->light_ctl_setup_srv.ctl_srv;
        status1 = light_ctl_server_status_publish(p_ctl_srv, &publication_data);
    }

    temp_publication_data.present_temperature32 = present_ctl_state.temperature32;
    temp_publication_data.present_delta_uv = present_ctl_state.delta_uv;
    temp_publication_data.target_temperature32 = p_app->state.target_temperature32;
    temp_publication_data.target_delta_uv = p_app->state.target_delta_uv;
    temp_publication_data.remaining_time_ms = remaining_time_ms;

    p_ctl_temperature_srv = &p_app->light_ctl_setup_srv.ctl_temperature_srv;
    status2 = light_ctl_server_temperature_status_publish(p_ctl_temperature_srv, &temp_publication_data);
    if (status1 != NRF_SUCCESS)
    {
        return status1;
    }
    return status2;
}

/* Callback used by light lightness module to inform this module when it has done a lightness
 * publish. This is to cover the case where a Light Lightness message was received meaning
 * that CTL should publish since the lightness component of the composite state changed. */
static void add_publish(const void * p_app_v, light_lightness_status_params_t * p_pub_data)
{
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    light_ctl_status_params_t publication_data;
    const app_light_ctl_setup_server_t * p_app = (app_light_ctl_setup_server_t *) p_app_v;

    if (p_app->light_ctl_setup_srv.state.initialized == false)
    {
        /* Ignore any publish before we have finished setting up the CTL server */
        return;
    }

    if (p_app->ctl_state_set_active == false)
    {
        p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
        /* Keeping info from the light lightness mid app for lightness and time - leaving temperature as
         * it is. */
        publication_data.present_lightness = p_pub_data->present_lightness;
        publication_data.target_lightness =  p_pub_data->target_lightness;
        publication_data.remaining_time_ms = p_pub_data->remaining_time_ms;
        publication_data.present_temperature32 = present_ctl_state.temperature32;
        publication_data.target_temperature32 = p_app->state.target_temperature32;

        /* Ignore status, as publish may fail due to several reasons and it is ok. */
        (void) light_ctl_server_status_publish(&p_app->light_ctl_setup_srv.ctl_srv, &publication_data);
    }
    /* else - don't publish anything - if there is a ctl state set command in progress, it will do
     * publish commands that will run in parallel to this */
}

/***** Transition module callback function implementation *****/
/* Note: The task of the transition module is to implement gradual changes of CTL's Temperature and
 * Delta UV states (the lightness mid app handles the lightness gradual changes) for all possible
 * time intervals and step size combinations allowed by the CGL model, and the callbacks from
 * this module are used to implement actual temperature and delta UV changes.
 *
 * If you need to implement transitions using external hardware or other mechanisms you can use
 * individual callbacks to initiate specific actions on external hardware.
 */

static app_light_ctl_setup_server_t * transition_to_app(const app_transition_t * p_transition)
{
    app_light_ctl_state_t * p_state;

    p_state = PARENT_BY_FIELD_GET(app_light_ctl_state_t, transition, p_transition);

    return PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, state, p_state);
}

static void transition_delay_start_cb(const app_transition_t * p_transition)
{
#if NRF_MESH_LOG_ENABLE
    app_light_ctl_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Element %d: starting delay\n",
          p_app->light_ctl_setup_srv.settings.element_index);
#endif
}

static void transition_start_cb(const app_transition_t * p_transition)
{
    app_light_ctl_setup_server_t * p_app;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);

    if ((p_app->state.transition.requested_params.transition_type == APP_CTL_TRANSITION_DELTA_SET &&
        p_app->state.new_tid) ||
        (p_app->state.transition.requested_params.transition_type == APP_CTL_TRANSITION_SET) ||
        (p_app->state.transition.requested_params.transition_type == APP_CTL_TRANSITION_MOVE_SET))
    {
        p_app->state.initial_present_temperature32 = p_app->state.init_present_temp32_snapshot;
        p_app->state.initial_present_delta_uv = p_app->state.init_present_duv_snapshot;
    }
    p_app->state.target_temperature32 = p_app->state.target_temp32_snapshot;
    p_app->state.target_delta_uv = p_app->state.target_duv_snapshot;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Element %d: starting transition: initial-t32: %d delta: %d tt: %d\n",
          p_app->light_ctl_setup_srv.settings.element_index,
          light_ctl_utils_temperature32_to_temperature(p_app->state.initial_present_temperature32),
          p_params->required_delta,
          p_params->transition_time_ms);

    p_app->state.published_ms = 0;

    if (p_app->app_light_ctl_transition_cb != NULL)
    {
        app_light_ctl_temperature_duv_hw_state_t target_ctl_state;
        target_ctl_state.delta_uv = p_app->state.target_delta_uv;
        target_ctl_state.temperature32 = p_app->state.target_temperature32;
        p_app->app_light_ctl_transition_cb(p_app,
                    p_params->transition_time_ms,
                    target_ctl_state);
    }
}

/* 6.1.3.1.3 Binding with the Temperature Range state
 * Light CTL Temperature = Light CTL Range Min for values of the Light CTL Temperature state that
 *   are less than the value of the Light CTL Temperature Range Min state.
 *
 * Light CTL Temperature = Light CTL Temperature Range Max for values of the Light CTL Temperature
 *   state that are greater than the value of the Light CTL Temperature Range Max state
 */
static void present_ctl_values_set(app_light_ctl_setup_server_t * p_app, light_ctl_temperature_range_set_params_t * p_range)
{
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;

    present_ctl_state.temperature32 = p_app->state.present_temperature32;
    present_ctl_state.temperature32 = light_ctl_utils_temperature32_range_restrict(present_ctl_state.temperature32,
                                                                                   p_range->temperature32_range_min,
                                                                                   p_range->temperature32_range_max);

    present_ctl_state.delta_uv = p_app->state.present_delta_uv;

    if (p_app->app_light_ctl_set_cb != NULL)
    {
        p_app->app_light_ctl_set_cb(p_app, &present_ctl_state);
    }

    p_app->state.present_temperature32 = present_ctl_state.temperature32;
}

static inline bool temperature32_limit_check(uint32_t present_temperature32,
                                             light_ctl_temperature_range_set_params_t * p_range,
                                             int32_t required_delta)
{
    /* Return true, if the present value is at the range limits */
    return ((required_delta > 0 && present_temperature32 == p_range->temperature32_range_max) ||
            (required_delta < 0 && present_temperature32 == p_range->temperature32_range_min));
}

static bool m_transition_abort_flag_cb(void)
{
    LIST_FOREACH(p_iter, mp_app_light_ctl_head)
    {
        app_light_ctl_setup_server_t * p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t,
                                                                   node,
                                                                   p_iter);
        if (p_app->abort_move)
        {
            p_app->abort_move = false;
            app_transition_abort(&p_app->state.transition);
        }
    }
    return true;
}

static void transition_tick_cb(const app_transition_t * p_transition)
{
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    app_light_ctl_setup_server_t * p_app;
    int64_t delta_temperature32;
    int32_t delta_duv;
    int32_t total_time_ms;
    light_ctl_temperature_range_set_params_t range_set;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Putting the uint32_t into an int32 for signed integer division.  No data is lost since in
     * @tagMeshMdlSp section 3.1.3 gives the maximum transition time of 10.5 hours (that is < 38
     * million ms, which is well below 31 bits) */
    total_time_ms = p_params->transition_time_ms;
    elapsed_ms = app_transition_elapsed_time_get((app_transition_t *)p_transition);
    remaining_ms = total_time_ms - elapsed_ms;

    if (p_params->transition_type != APP_CTL_TRANSITION_MOVE_SET)
    {
        /* Lightness is handled in the other state machine */
        /* Calculate new values using linear interpolation and provide to the application. */
        delta_temperature32 =
            ((int64_t)p_app->state.target_temperature32 - (int64_t)p_app->state.initial_present_temperature32);
        p_app->state.present_temperature32 = p_app->state.initial_present_temperature32 +
            ((int64_t)delta_temperature32 * (int64_t)elapsed_ms / total_time_ms);

        delta_duv = (p_app->state.target_delta_uv - p_app->state.initial_present_delta_uv);
        p_app->state.present_delta_uv = p_app->state.initial_present_delta_uv +
            ((int64_t)delta_duv * (int64_t)elapsed_ms / total_time_ms);
    }
    else
    {
        /* A move command only comes from a level client, which is only bound to the temperature
         * state, not the delta UV state */
        delta_temperature32 = SCALE_UP((int64_t)p_params->required_delta, T32_SCALE_FACTOR);
        p_app->state.present_temperature32 = p_app->state.initial_present_temperature32 +
            (((int64_t)elapsed_ms * delta_temperature32) / (int64_t)p_params->transition_time_ms);
    }

    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_app->light_ctl_setup_srv.state.handle, &range_set));
    /* The CTL server can't function if the temperature is unknown - check for unknown value */
    CHECK_INVALID_TEMPERATURE_RANGE(range_set);
    present_ctl_values_set(p_app, &range_set);

    if (p_params->transition_type == APP_CTL_TRANSITION_MOVE_SET &&
        temperature32_limit_check(p_app->state.present_temperature32, &range_set, p_params->required_delta))
    {
        /* Stop transition on the temperature/delta_uv FSM */
        p_app->abort_move = true;
        bearer_event_flag_set(m_transition_abort_flag);
    }

    /* While the transition has at least one half second remaining, publish intermediate CTL state
     * at one second intervals. */
    if ((remaining_ms > CTL_ADDITIONAL_PUBLISH_START_MARGIN_MS) &&
        (elapsed_ms - p_app->state.published_ms > CTL_ADDITIONAL_PUBLISH_INTERVAL_MS))
    {
        p_app->state.published_ms = elapsed_ms;
        /* Ignore status, as publish may fail due to several reasons and it is ok. */
        (void) ctl_publish(p_app, remaining_ms);
    }
}

static void transition_complete_cb(const app_transition_t * p_transition)
{
    app_light_ctl_setup_server_t * p_app;
    light_ctl_temperature_range_set_params_t range_set;

    p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    p_app->state.target_temperature32 = p_app->state.target_temp32_snapshot;
    p_app->state.target_delta_uv = p_app->state.target_duv_snapshot;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Element %d: transition completed\n",
          p_app->light_ctl_setup_srv.settings.element_index);

    if (p_params->transition_type != APP_CTL_TRANSITION_MOVE_SET)
    {
        p_app->state.present_temperature32 = p_app->state.target_temperature32;
        p_app->state.present_delta_uv = p_app->state.target_delta_uv;
    }

    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(p_app->light_ctl_setup_srv.state.handle, &range_set));
    CHECK_INVALID_TEMPERATURE_RANGE(range_set);
    present_ctl_values_set(p_app, &range_set);

    /* The transition is complete */
    if (p_app->app_light_ctl_transition_cb != NULL)
    {
        app_light_ctl_temperature_duv_hw_state_t target_ctl_state;
        target_ctl_state.delta_uv = p_app->state.target_delta_uv;
        target_ctl_state.temperature32 = p_app->state.target_temperature32;
        p_app->app_light_ctl_transition_cb(p_app,
                    p_params->transition_time_ms,
                    target_ctl_state);
    }

    ERROR_CHECK(light_ctl_mc_temperature32_state_set(p_app->light_ctl_setup_srv.state.handle, p_app->state.present_temperature32));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_set(p_app->light_ctl_setup_srv.state.handle, p_app->state.present_delta_uv));

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) ctl_publish(p_app, 0);

    /* If this was a CTL state set, we have completed this CTL state or temperature state set
     * command.  If it wasn't a CTL state or temperature state set, they are already false, so it's
     * perfectly acceptable to set to false */
    p_app->ctl_state_set_active = false;
    p_app->ctl_temperature_state_set_active = false;
}

/* Returns true if the identified transition time is a valid non-zero value;
 * false otherwise.
 */
static void transition_parameters_set(app_light_ctl_setup_server_t * p_app,
                                      int32_t delta,
                                      const model_transition_t * p_in_transition,
                                      app_transition_type_t transition_type)
{
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_params->transition_type = transition_type;
    p_params->required_delta  = delta;
    p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

    /* Requirement: If transition time parameters are unavailable and default transition time state
     * is not available, transition shall be instantaneous. */
    if (p_in_transition == NULL)
    {
        p_app->state.transition.delay_ms = 0;

        ERROR_CHECK(light_lightness_mc_dtt_state_get(p_app->p_app_ll->light_lightness_setup_server.state.handle,
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
static void app_light_ctl_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                      uint8_t scene_index)
{
    app_light_ctl_setup_server_t * p_app = 
                PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, scene_if, p_app_scene_if);
    ERROR_CHECK(light_ctl_mc_scene_temperature32_state_store (
                    p_app->light_ctl_setup_srv.state.handle, 
                    scene_index, 
                    p_app->state.present_temperature32));
    ERROR_CHECK(light_ctl_mc_scene_delta_uv_state_store (
                    p_app->light_ctl_setup_srv.state.handle, 
                    scene_index, 
                    p_app->state.present_delta_uv));

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
            "SCENE STORE: handel: %d  scene index: %d  present temperature: %d  present duv: %d\n",
            p_app->light_ctl_setup_srv.state.handle,
            scene_index,
            light_ctl_utils_temperature32_to_temperature(p_app->state.present_temperature32),
            p_app->state.present_delta_uv);
}

static void app_light_ctl_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                       uint8_t scene_index,
                                       uint32_t delay_ms,
                                       uint32_t transition_time_ms)
{
    app_light_ctl_setup_server_t * p_app;
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint32_t recall_temperature32;
    int16_t recall_delta_uv;

    p_app = PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, scene_if, p_app_scene_if);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Set a module flag to let the code know there is a CTL state set occurring */
    p_app->ctl_state_set_active = true;

    /* Process the temperature and delta UV states (CTL specific - no lightness involved) */
    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);

    ERROR_CHECK(light_ctl_mc_scene_temperature32_state_recall(
                    p_app->light_ctl_setup_srv.state.handle, 
                    scene_index, 
                    &recall_temperature32));
    ERROR_CHECK(light_ctl_mc_scene_delta_uv_state_recall(
                    p_app->light_ctl_setup_srv.state.handle, 
                    scene_index, 
                    &recall_delta_uv));

    p_app->state.target_temp32_snapshot = recall_temperature32;
    p_app->state.target_duv_snapshot = recall_delta_uv;
    p_app->state.init_present_temp32_snapshot = p_app->state.present_temperature32;
    p_app->state.init_present_duv_snapshot = p_app->state.present_delta_uv;

    ERROR_CHECK(light_ctl_mc_temperature32_state_set(p_app->light_ctl_setup_srv.state.handle, 
                                                     p_app->state.target_temp32_snapshot));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_set(p_app->light_ctl_setup_srv.state.handle, 
                                                p_app->state.target_duv_snapshot));

    model_transition_t in_transition = {.delay_ms = delay_ms, 
                                        .transition_time_ms = transition_time_ms};
    
    if ((!p_app->light_ctl_setup_srv.state.initialized) || 
        (present_ctl_state.temperature32 != recall_temperature32) ||
        (present_ctl_state.delta_uv != recall_delta_uv))
    {
        app_transition_abort(&p_app->state.transition);

        int32_t delta = ctl_state_delta_max_compute(p_app, present_ctl_state.temperature32,
                                                    p_app->state.target_temp32_snapshot,
                                                    present_ctl_state.delta_uv,
                                                    p_app->state.target_duv_snapshot);

        transition_parameters_set(p_app, delta, &in_transition, APP_CTL_TRANSITION_SET);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "SCENE RECALL: temperature: %d duv: %d delay: %d  tt: %d  req-delta: %d \n",
              light_ctl_utils_temperature32_to_temperature(recall_temperature32),
              recall_delta_uv,
              p_app->state.transition.delay_ms,
              p_params->transition_time_ms,
              p_params->required_delta);

        app_transition_trigger(&p_app->state.transition);
    }
}

static void app_light_ctl_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                       uint8_t scene_index)
{
    app_light_ctl_setup_server_t * p_app = 
                PARENT_BY_FIELD_GET(app_light_ctl_setup_server_t, scene_if, p_app_scene_if);
    app_transition_abort(&p_app->state.transition);
    /* No need to do anything else */
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0 */


/***** Interface functions *****/

uint32_t app_light_ctl_model_init(app_light_ctl_setup_server_t * p_app, uint8_t element_index,
                                  app_light_lightness_setup_server_t * p_app_ll)
{
    uint32_t status;

    if (p_app == NULL || p_app_ll == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ( (p_app->app_light_ctl_get_cb == NULL) ||
         ( (p_app->app_light_ctl_set_cb == NULL) &&
           (p_app->app_light_ctl_transition_cb == NULL) ) )
    {
        return NRF_ERROR_NULL;
    }

    p_app->p_app_ll = p_app_ll;

    /* Set up the added callback from light lightness to CTL */
    p_app->p_app_ll->app_add_notify.p_app_publish_v = p_app;
    p_app->p_app_ll->app_add_notify.app_add_publish_cb = add_publish;

    /* Initialize a CTL setup server.  The main.c code already initialized our light lightness setup
     * server, and the CTL main element is shared with that light lightness server.  The init code
     * uses the p_app to get the model handle for the light lightness server and all its extended
     * servers.
     */
    status = app_ctl_setup_server_init(p_app, element_index);

    if (status != NRF_SUCCESS)
    {
        return status;
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    p_app->scene_if.p_callbacks = &m_scene_light_ctl_cbs;
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
        list_add(&mp_app_light_ctl_head, &p_app->node);
        m_transition_abort_flag = bearer_event_flag_add(m_transition_abort_flag_cb);
    }

    return status;
}

uint32_t app_light_ctl_binding_setup(app_light_ctl_setup_server_t * p_app)
{
    light_ctl_saved_values_t state;
    uint16_t index;
    uint16_t ll_index;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    index = p_app->light_ctl_setup_srv.state.handle;
    ll_index = p_app->p_app_ll->light_lightness_setup_server.state.handle;

    ERROR_CHECK(light_lightness_mc_onpowerup_state_get(ll_index, &state.onpowerup));
    ERROR_CHECK(light_ctl_mc_temperature32_state_get(index, &state.temperature32));
    ERROR_CHECK(light_ctl_mc_default_temperature32_state_get(index, &state.default_temperature32));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_get(index, (int16_t *) &state.delta_uv));
    ERROR_CHECK(light_ctl_mc_default_delta_uv_state_get(index, (int16_t *) &state.default_delta_uv));
    ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(index, &state.temperature32_range));

    /* The CTL server can't function if the temperature is unknown - check for unknown value */
    CHECK_INVALID_TEMPERATURE_RANGE(state.temperature32_range);

    return light_ctl_ponoff_binding_setup(&p_app->light_ctl_setup_srv, &state);
}

uint32_t app_light_ctl_current_value_publish(app_light_ctl_setup_server_t * p_app)
{
    app_light_ctl_temperature_duv_hw_state_t present_ctl_state;
    uint16_t lightness;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Stop any transitions on the temperature/delta_uv FSM and the lightness FSM */
    app_transition_abort(&p_app->state.transition);
    app_transition_abort(&p_app->p_app_ll->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    uint32_t old_present_temperature32 = p_app->state.present_temperature32;
    int16_t old_present_delta_uv = p_app->state.present_delta_uv;
#endif

    p_app->app_light_ctl_get_cb(p_app, &present_ctl_state);
    p_app->p_app_ll->app_light_lightness_get_cb(p_app->p_app_ll, &lightness);

    p_app->state.present_temperature32 = present_ctl_state.temperature32;
    p_app->state.present_delta_uv = present_ctl_state.delta_uv;
    p_app->p_app_ll->state.present_lightness = lightness;

    /* Set persistent memory for ctl and light lightness */
    ERROR_CHECK(light_ctl_mc_temperature32_state_set(p_app->light_ctl_setup_srv.state.handle, p_app->state.present_temperature32));
    ERROR_CHECK(light_ctl_mc_delta_uv_state_set(p_app->light_ctl_setup_srv.state.handle, p_app->state.present_delta_uv));

    if (lightness >= LIGHT_LIGHTNESS_LAST_MIN)
    {
        ERROR_CHECK(light_lightness_mc_last_state_set(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                    lightness,
                                                    LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL));
    }

    ERROR_CHECK(light_lightness_mc_actual_state_set(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                  lightness));

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if ((old_present_temperature32 != p_app->state.present_temperature32) || 
        (old_present_delta_uv != p_app->state.present_delta_uv))
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    return ctl_publish(p_app, 0);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t app_light_ctl_scene_context_set(app_light_ctl_setup_server_t * p_app, 
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
