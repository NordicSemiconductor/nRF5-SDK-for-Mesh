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

#include "app_level.h"

#include <stdint.h>
#include <stdlib.h>

#include "mesh_app_utils.h"
#include "sdk_config.h"
#include "example_common.h"
#include "generic_level_server.h"
#include "generic_level_mc.h"

#include "log.h"
#include "nrf_mesh_assert.h"

/* Forward declaration */
static void generic_level_state_get_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_level_status_params_t * p_out);
static void generic_level_state_set_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_level_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_level_status_params_t * p_out);

static void generic_level_state_delta_set_cb(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_level_delta_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_level_status_params_t * p_out);

static void generic_level_state_move_set_cb(const generic_level_server_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const generic_level_move_set_params_t * p_in,
                                            const model_transition_t * p_in_transition,
                                            generic_level_status_params_t * p_out);

static const generic_level_server_callbacks_t m_level_srv_cbs =
{
    .level_cbs.get_cb = generic_level_state_get_cb,
    .level_cbs.set_cb = generic_level_state_set_cb,
    .level_cbs.delta_set_cb = generic_level_state_delta_set_cb,
    .level_cbs.move_set_cb = generic_level_state_move_set_cb
};

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_level_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                  uint8_t scene_index);
static void app_level_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                   uint8_t scene_index,
                                   uint32_t delay_ms,
                                   uint32_t transition_time_ms);
static void app_level_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                   uint8_t scene_index);

const app_scene_callbacks_t m_scene_level_cbs =
{
    .scene_store_cb = app_level_scene_store,
    .scene_recall_cb = app_level_scene_recall,
    .scene_delete_cb = app_level_scene_delete
};
#endif

static void transition_params_set(app_level_server_t * p_app,
                                  int32_t delta,
                                  const model_transition_t * p_in_transition,
                                  app_transition_type_t transition_type)
{
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_params->transition_type = transition_type;
    p_params->required_delta = delta;
    p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

    /* Requirement: If transition time parameters are unavailable and default transition time state
    is not available, transition shall be instantaneous. */
    if (p_in_transition == NULL)
    {
        p_app->state.transition.delay_ms = 0;
#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
        generic_dtt_status_params_t dtt_params = {0};
        /* For this implementation, the app_level uses the DTT instance available with Scene
         * Setup server */
        p_app->p_app_scene->scene_setup_server.p_gen_dtt_server->settings.p_callbacks->dtt_cbs.get_cb(p_app->p_app_scene->scene_setup_server.p_gen_dtt_server,
                                                                                                      NULL, &dtt_params);
        p_params->transition_time_ms = dtt_params.transition_time_ms;
#else
        p_params->transition_time_ms = 0;
#endif
    }
    else
    {
        p_app->state.transition.delay_ms = p_in_transition->delay_ms;
        p_params->transition_time_ms = p_in_transition->transition_time_ms;
    }
}

/**************************************************************************************************/
/***** Transition module callback function implementation *****/
/* Note: The task of the transition module is to implement gradual changes of `present_level` value
 * for all possible time intervals and step size combinations allowed by the level model, and the
 * callbacks from this module are used to implement actual level value changes.
 *
 * If you need to implement transitions using external hardware or other mechanisms you can use
 * individual callbacks to initiate specific actions on external hardware.
 */

static inline app_level_server_t * transition_to_app(const app_transition_t * p_transition)
{
    app_level_state_t * p_state = PARENT_BY_FIELD_GET(app_level_state_t,
                                  transition,
                                  p_transition);
    return PARENT_BY_FIELD_GET(app_level_server_t,
                               state,
                               p_state);
}

static void app_level_delay_start_cb(const app_transition_t * p_transition)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Starting delay\n");
}

static void app_level_transition_start_cb(const app_transition_t * p_transition)
{
    app_level_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    p_app->state.delta = p_params->required_delta;
    p_app->state.initial_present_level = p_app->state.init_present_snapshot;
    p_app->state.target_level = p_app->state.target_snapshot;

    if (p_app->level_transition_cb != NULL)
    {
        p_app->level_transition_cb(p_app, p_params->transition_time_ms,
                                          p_app->state.target_level,
                                          p_params->transition_type);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Starting transition\n");
}

static void app_level_transition_tick_cb(const app_transition_t * p_transition)
{
    app_level_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    if (p_params->transition_type != APP_TRANSITION_TYPE_MOVE_SET)
    {
        /* Calculate new value using linear interpolation and provide to the application. */
        int32_t delta = (p_app->state.target_level - p_app->state.initial_present_level);
        p_app->state.present_level = p_app->state.initial_present_level +
                                        (delta * (int64_t)app_transition_elapsed_time_get(&p_app->state.transition) /
                                          (int64_t)p_params->transition_time_ms);
    }
    else
    {
        p_app->state.present_level = p_app->state.initial_present_level +
                                        (((int64_t)app_transition_elapsed_time_get(&p_app->state.transition) * (int64_t)p_app->state.delta) /
                                          (int64_t)p_params->transition_time_ms);
    }

    if (p_app->level_set_cb != NULL)
    {
        p_app->level_set_cb(p_app, p_app->state.present_level);
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "Elapsed time: %d\n", (uint32_t) app_transition_elapsed_time_get(&p_app->state.transition));
}

static void app_level_transition_complete_cb(const app_transition_t * p_transition)
{
    app_level_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* This handles a case, when a new transition has a non-zero delay, but zero transition time.
     * This also handles a case, when such request is received in the middle of another transition.
     * In usual cases when transition time is non-zero, this assignment will have no effect since
     * this copying is already done at start of the transition. */
    p_app->state.target_level = p_app->state.target_snapshot;

    if (p_params->transition_type != APP_TRANSITION_TYPE_MOVE_SET)
    {
        p_app->state.present_level = p_app->state.target_level;
    }

    generic_level_status_params_t status_params;
    status_params.present_level = p_app->state.present_level;
    status_params.target_level = p_app->state.target_level;
    status_params.remaining_time_ms = 0;
    (void) generic_level_server_status_publish(&p_app->server, &status_params);

    if (p_app->level_set_cb != NULL)
    {
        p_app->level_set_cb(p_app, p_app->state.present_level);
    }

    if (p_app->level_transition_cb != NULL)
    {
        p_app->level_transition_cb(p_app, p_params->transition_time_ms,
                                          p_app->state.target_level,
                                          p_params->transition_type);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Transition completed: Present-L: %d  Target-L: %d\n",
          p_app->state.present_level, p_app->state.target_level);
}

/**************************************************************************************************/
/***** Generic Level model interface callbacks *****/

static void generic_level_state_get_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_level_status_params_t * p_out)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Requirement: Provide the current value of the Level state */
    p_app->level_get_cb(p_app, &p_app->state.present_level);
    p_out->present_level = p_app->state.present_level;
    p_out->target_level = p_app->state.target_level;

    /* Requirement: Report remaining time during processing of SET or DELTA SET,
     *              Report zero/unknown transition time during processing of MOVE. */
    if (p_params->transition_type == APP_TRANSITION_TYPE_MOVE_SET)
    {
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
    else
    {
        p_out->remaining_time_ms = app_transition_remaining_time_get(&p_app->state.transition);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "GET: Present-L: %d  Target-L: %d  Remaining_t: %d \n",
          p_out->present_level, p_out->target_level, p_out->remaining_time_ms);
}

static void generic_level_state_set_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_level_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_level_status_params_t * p_out)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Update internal representation of Level value, process timing. */
    p_app->state.target_snapshot = p_in->level;
    p_app->state.init_present_snapshot = p_app->state.present_level;

    ERROR_CHECK(generic_level_mc_level_state_set(p_app->server.state_handle,
                                                 p_app->state.target_snapshot));

    transition_params_set(p_app,
                          (int32_t)p_app->state.target_snapshot - (int32_t)p_app->state.init_present_snapshot,
                          p_in_transition,
                          APP_TRANSITION_TYPE_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: Level: %d  delay: %d  tt: %d  req-delta: %d  trans-type: %d\n",
          p_app->state.target_snapshot,  p_app->state.transition.delay_ms, p_params->transition_time_ms,
          p_params->required_delta,
          p_params->transition_type);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (p_app->state.present_level != p_in->level)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_app->state.present_level;
        p_out->target_level = p_app->state.target_snapshot;
        p_out->remaining_time_ms = p_params->transition_time_ms;
    }
}

static void generic_level_state_delta_set_cb(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_level_delta_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_level_status_params_t * p_out)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Update internal representation of Level value, process timing. */
    /* Requirement: If TID is same as previous TID for the same message, delta value is cumulative. */
    int32_t delta = p_in->delta_level % ((int32_t)UINT16_MAX + 1);
    if (!model_transaction_is_new(&p_app->server.tid_tracker))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tid: %d Same TID, assuming cumulative delta set.\n", p_in->tid);
    }
    else
    {
        p_app->state.init_present_snapshot = p_app->state.present_level;
    }

    p_app->state.target_snapshot = p_app->state.init_present_snapshot + delta;

    transition_params_set(p_app, delta, p_in_transition, APP_TRANSITION_TYPE_DELTA_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Delta SET: delta: %d  delay: %d  tt: %d\n",
          p_in->delta_level, p_app->state.transition.delay_ms, p_params->transition_time_ms);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Delta SET: initial-level: %d  present-level: %d  target-level: %d\n",
          p_app->state.init_present_snapshot, p_app->state.present_level, p_app->state.target_snapshot);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (p_in->delta_level != 0)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_app->state.present_level;
        p_out->target_level = p_app->state.target_snapshot;
        p_out->remaining_time_ms = p_params->transition_time_ms;
    }
}

static void generic_level_state_move_set_cb(const generic_level_server_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const generic_level_move_set_params_t * p_in,
                                            const model_transition_t * p_in_transition,
                                            generic_level_status_params_t * p_out)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    /* Requirement: For the status message: The target Generic Level state is the upper limit of
       the Generic Level state when the transition speed is positive, or the lower limit of the
       Generic Level state when the transition speed is negative. */
    if (p_in->move_level > 0)
    {
        p_app->state.target_snapshot = INT16_MAX;
    }
    else
    {
        p_app->state.target_snapshot = INT16_MIN;
    }

    p_app->state.init_present_snapshot = p_app->state.present_level;

    transition_params_set(p_app,
                          (int32_t) p_in->move_level,
                          p_in_transition,
                          APP_TRANSITION_TYPE_MOVE_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "MOVE SET: move-level: %d  delay: %d  tt: %d \n",
          p_in->move_level, p_app->state.transition.delay_ms, p_params->transition_time_ms);

    app_transition_trigger(&p_app->state.transition);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (p_in->move_level != 0)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_app->state.present_level;
        p_out->target_level = p_app->state.target_snapshot;
        /* Response to Move Set message is sent with unknown transition time value, if
        given transition time is non-zero. */
        p_out->remaining_time_ms = (p_params->transition_time_ms == 0) ?
                                    0 : MODEL_TRANSITION_TIME_UNKNOWN;
    }
}


/***** Scene Interface functions *****/

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_level_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                  uint8_t scene_index)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, scene_if, p_app_scene_if);
    ERROR_CHECK(generic_level_mc_scene_level_store(p_app->server.model_handle, scene_index,
                                                   p_app->state.present_level));
}

static void app_level_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                   uint8_t scene_index,
                                   uint32_t delay_ms,
                                   uint32_t transition_time_ms)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, scene_if, p_app_scene_if);

    int16_t recall_level;
    ERROR_CHECK(generic_level_mc_scene_level_recall(p_app->server.model_handle, scene_index,
                                                    &recall_level));

    p_app->state.target_snapshot = recall_level;
    p_app->state.init_present_snapshot = p_app->state.present_level;

    ERROR_CHECK(generic_level_mc_level_state_set(p_app->server.state_handle,
                                                 p_app->state.target_snapshot));

    model_transition_t in_transition = {.delay_ms = delay_ms,
                                        .transition_time_ms = transition_time_ms};

    app_transition_abort(&p_app->state.transition);

    transition_params_set(p_app,
                          (int32_t)p_app->state.target_snapshot - (int32_t)p_app->state.init_present_snapshot,
                          &in_transition,
                          APP_TRANSITION_TYPE_SET);

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "SCENE RECALL: target level: %d  delay: %d  tt: %d \n",
            p_app->state.target_snapshot, in_transition.delay_ms, in_transition.transition_time_ms);

    app_transition_trigger(&p_app->state.transition);
}

static void app_level_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                   uint8_t scene_index)
{
    app_level_server_t * p_app = PARENT_BY_FIELD_GET(app_level_server_t, scene_if, p_app_scene_if);
    app_transition_abort(&p_app->state.transition);
    /* No need to do anything else */
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0 */


/***** Interface functions *****/

uint32_t app_level_current_value_publish(app_level_server_t * p_app)
{
#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    int16_t old_present_level = p_app->state.present_level;
#endif

    p_app->level_get_cb(p_app, &p_app->state.present_level);
    app_transition_abort(&p_app->state.transition);

    p_app->state.target_level = p_app->state.present_level;
    p_app->state.transition.delay_ms = 0;
    memset(app_transition_ongoing_get(&p_app->state.transition), 0, sizeof(app_transition_params_t));
    memset(app_transition_requested_get(&p_app->state.transition), 0, sizeof(app_transition_params_t));

    ERROR_CHECK(generic_level_mc_level_state_set(p_app->server.state_handle,
                                                 p_app->state.present_level));

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    if (old_present_level != p_app->state.present_level)
    {
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif

    generic_level_status_params_t status = {
                .present_level = p_app->state.present_level,
                .target_level = p_app->state.target_level,
                .remaining_time_ms = 0
            };
    return generic_level_server_status_publish(&p_app->server, &status);
}


uint32_t app_level_init(app_level_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ( (p_app->level_get_cb == NULL) ||
         ( (p_app->level_set_cb == NULL) &&
           (p_app->level_transition_cb == NULL) ) )
    {
        return NRF_ERROR_NULL;
    }

    p_app->server.settings.p_callbacks = &m_level_srv_cbs;
    status = generic_level_server_init(&p_app->server, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Set the default state.
     */
    status = generic_level_mc_open(&p_app->server.state_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    p_app->scene_if.p_callbacks = &m_scene_level_cbs;
#endif

    p_app->state.transition.delay_start_cb = app_level_delay_start_cb;
    p_app->state.transition.transition_start_cb = app_level_transition_start_cb;
    p_app->state.transition.transition_tick_cb = app_level_transition_tick_cb;
    p_app->state.transition.transition_complete_cb = app_level_transition_complete_cb;
    p_app->state.transition.p_context = p_app;

    return app_transition_init(&p_app->state.transition);
}

uint32_t app_level_value_restore(app_level_server_t * p_app)
{
    int16_t level;

    ERROR_CHECK(generic_level_mc_level_state_get(p_app->server.state_handle, &level));

    return (generic_level_server_state_set(&p_app->server, level));
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t app_level_scene_context_set(app_level_server_t * p_app,
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
