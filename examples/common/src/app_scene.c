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

#include "app_scene.h"

#include <stdlib.h>

#include "nrf_mesh_config_examples.h"
#include "log.h"
#include "nrf_mesh_assert.h"
#include "mesh_app_utils.h"
#include "scene_mc.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0

/** This sample implementation shows how the model behavior requirements of Scene Setup server can
 * be implemented.
 */

/* Forward declarations */
static void scene_state_store_cb(const scene_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const scene_store_params_t * p_in,
                                 scene_register_status_params_t * p_out);

static void scene_state_delete_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_delete_params_t * p_in,
                                  scene_register_status_params_t * p_out);

static void scene_state_get_cb(const scene_setup_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               scene_status_params_t * p_out);

static void scene_state_register_get_cb(const scene_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        scene_register_status_params_t * p_out);

static void scene_state_recall_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_recall_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  scene_status_params_t * p_out);

static uint32_t publish_status(app_scene_setup_server_t * p_app,
                               uint8_t status_code,
                               uint32_t remaining_time_ms);

/**** end of callback declarations ****/

static const scene_setup_server_callbacks_t scene_setup_srv_cbs =
{
    .scene_cbs.store_cb = scene_state_store_cb,
    .scene_cbs.delete_cb = scene_state_delete_cb,
    .scene_cbs.get_cb = scene_state_get_cb,
    .scene_cbs.register_get_cb = scene_state_register_get_cb,
    .scene_cbs.recall_cb = scene_state_recall_cb
};

static void scene_register_copy(const scene_setup_server_t * p_self, uint16_t * p_scenes_array)
{
    for (uint32_t i = 0; i < SCENE_REGISTER_ARRAY_SIZE; i++)
    {
       ERROR_CHECK(scene_mc_scene_number_get(p_self->state_handle, i, p_scenes_array));
       p_scenes_array++;
    };
}

/***** Scene model interface callbacks ****/

static void transition_parameters_set(app_scene_setup_server_t * p_app,
                                      const model_transition_t * p_in_transition,
                                      app_transition_type_t transition_type)
{
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    p_params->transition_type = transition_type;
    p_params->required_delta  = 1;
    p_params->minimum_step_ms = TRANSITION_STEP_MIN_MS;

    NRF_MESH_ASSERT(p_in_transition != NULL);

    p_app->state.transition.delay_ms = p_in_transition->delay_ms;
    p_params->transition_time_ms = p_in_transition->transition_time_ms;
}

static void scene_state_store_cb(const scene_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const scene_store_params_t * p_in,
                                 scene_register_status_params_t * p_out)
{
    app_scene_setup_server_t * p_app;

    NRF_MESH_ASSERT(p_self && p_in);
    p_app = PARENT_BY_FIELD_GET(app_scene_setup_server_t, scene_setup_server, p_self);
    uint8_t status_code = SCENE_STATUS_REGISTER_FULL;

    uint8_t scene_index;
    uint32_t status = scene_mc_store(p_self->state_handle, p_in->scene_number, &scene_index);
    if (status == NRF_SUCCESS)
    {
        p_app->state.current_scene_number = p_in->scene_number;
        for (uint32_t i = 0; i < p_app->next_model_interface; i++)
        {
            p_app->scene_models[i]->p_callbacks->scene_store_cb(p_app->scene_models[i],
                                                                scene_index);
        }
        status_code = SCENE_STATUS_SUCCESS;
    }

    if (p_out != NULL)
    {
        p_out->status_code = status_code;
        p_out->current_scene = p_app->state.current_scene_number;
        scene_register_copy(p_self, &p_out->scenes[0]);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "STORE SCENE: status: %d, target: %d, current: %d\n",
          status_code, p_app->state.target_scene_number, p_app->state.current_scene_number);
}

static void scene_state_delete_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_delete_params_t * p_in,
                                  scene_register_status_params_t * p_out)
{
    app_scene_setup_server_t * p_app;

    NRF_MESH_ASSERT(p_self && p_in);
    p_app = PARENT_BY_FIELD_GET(app_scene_setup_server_t, scene_setup_server, p_self);

    uint8_t scene_index;
    uint32_t status = scene_mc_delete(p_self->state_handle, p_in->scene_number, &scene_index);

    if (status == NRF_SUCCESS)
    {
        if ( (p_app->state.current_scene_number == p_in->scene_number) ||
             (p_app->state.target_scene_number == p_in->scene_number) )
        {
            app_transition_abort(&p_app->state.transition);
            p_app->state.target_scene_number = SCENE_NUMBER_NO_SCENE;
            p_app->state.current_scene_number = SCENE_NUMBER_NO_SCENE;
        }

        for (uint32_t i = 0; i < p_app->next_model_interface; i++)
        {
            p_app->scene_models[i]->p_callbacks->scene_delete_cb(p_app->scene_models[i],
                                                                 scene_index);
        }
    }

    if (p_out != NULL)
    {
        /* Delete operation is successful regardless if it was found or not. See Mesh Model
           Specification section 5.3.4.2.2. */
        p_out->status_code = SCENE_STATUS_SUCCESS;
        p_out->current_scene = p_app->state.current_scene_number;
        scene_register_copy(p_self, &p_out->scenes[0]);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "DELETE SCENE: target: %d, current: %d\n",
          p_app->state.target_scene_number, p_app->state.current_scene_number);
}

static void scene_state_get_cb(const scene_setup_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               scene_status_params_t * p_out)
{
    app_scene_setup_server_t * p_app;

    NRF_MESH_ASSERT(p_self && p_out);
    p_app = PARENT_BY_FIELD_GET(app_scene_setup_server_t, scene_setup_server, p_self);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    /* Requirement: Provide the current value of some of the Scene composite state */
    p_out->status_code = SCENE_STATUS_SUCCESS;
    p_out->current_scene = p_app->state.current_scene_number;
    p_out->target_scene = p_app->state.target_scene_number;

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

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
        "GET SCENE: current s: %d  target s: %d  rem-tt: %d ms req-delta: %d \n",
        p_out->current_scene,
        p_out->target_scene,
        p_out->remaining_time_ms,
        p_params->required_delta);
}

static void scene_state_register_get_cb(const scene_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        scene_register_status_params_t * p_out)
{
    app_scene_setup_server_t * p_app;

    NRF_MESH_ASSERT(p_self && p_out);
    p_app = PARENT_BY_FIELD_GET(app_scene_setup_server_t, scene_setup_server, p_self);

    /* Requirement: Provide the current value of some of the Scene composite state */
    p_out->status_code = SCENE_STATUS_SUCCESS;
    p_out->current_scene = p_app->state.current_scene_number;
    scene_register_copy(p_self, &p_out->scenes[0]);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "REGISTER GET SCENE: current s: %d\n", p_out->current_scene);
}

static void scene_state_recall_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_recall_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  scene_status_params_t * p_out)
{
    app_scene_setup_server_t * p_app;
    uint8_t status_code = SCENE_STATUS_SUCCESS;

    NRF_MESH_ASSERT(p_self && p_in);
    p_app = PARENT_BY_FIELD_GET(app_scene_setup_server_t, scene_setup_server, p_self);
    app_transition_params_t * p_params = app_transition_requested_get(&p_app->state.transition);

    if (p_app->state.current_scene_number != p_in->scene_number)
    {
        uint8_t scene_index;
        uint32_t status = scene_mc_recall(p_self->state_handle, p_in->scene_number, &scene_index);
        status_code = status == NRF_SUCCESS ? SCENE_STATUS_SUCCESS : SCENE_STATUS_NOT_FOUND;
        if (status_code == SCENE_STATUS_SUCCESS)
        {
            model_transition_t transition = {0};
            generic_dtt_status_params_t dtt_params;
            const model_transition_t * p_transition;

            if (p_in_transition == NULL)
            {
                NRF_MESH_ASSERT(p_self->p_gen_dtt_server->settings.p_callbacks->dtt_cbs.get_cb != NULL);
                p_self->p_gen_dtt_server->settings.p_callbacks->dtt_cbs.get_cb(p_self->p_gen_dtt_server,
                                                                               NULL, &dtt_params);
                transition.transition_time_ms = dtt_params.transition_time_ms;
                p_transition = &transition;
            }
            else
            {
                p_transition = p_in_transition;
            }

            transition_parameters_set(p_app, p_transition, APP_TRANSITION_TYPE_SET);
            p_app->state.target_scene_number = p_in->scene_number;

            for (uint32_t i = 0; i < p_app->next_model_interface; i++)
            {
                p_app->scene_models[i]->p_callbacks->scene_recall_cb(p_app->scene_models[i],
                                                                     scene_index,
                                                                     p_app->state.transition.delay_ms,
                                                                     p_app->state.transition.requested_params.transition_time_ms);
            }

            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
                "RECALL SCENE: scene: %d  index: %d  delay: %d  tt: %d \n",
                p_in->scene_number,
                scene_index,
                p_app->state.transition.delay_ms,
                p_params->transition_time_ms);

            app_transition_trigger(&p_app->state.transition);
        }
    }

    if (p_out != NULL)
    {
        p_out->status_code = status_code;
        p_out->current_scene = p_app->state.current_scene_number;
        p_out->target_scene = p_app->state.target_scene_number;
        p_out->remaining_time_ms = p_params->transition_time_ms;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "RECALL SCENE: status: %d, current: %d, target: %d\n",
          status_code,
          p_app->state.current_scene_number,
          p_app->state.target_scene_number);
}

/***/
static uint32_t app_scene_setup_server_init(scene_setup_server_t * p_setup_server,
                                            uint8_t element_index)
{
    uint32_t status;

    if (p_setup_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Set the default state
     */
    status = scene_mc_open(&p_setup_server->state_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize the Scene Setup server callback structure. */
   p_setup_server->settings.p_callbacks = &scene_setup_srv_cbs;

    status = scene_setup_server_init(p_setup_server, element_index);
    return status;
}

static uint32_t publish_status(app_scene_setup_server_t * p_app, uint8_t status_code,
                               uint32_t remaining_time_ms)
{
    scene_status_params_t publication_data;
    scene_server_t * p_scene_srv;

    publication_data.status_code = status_code;
    publication_data.current_scene = p_app->state.current_scene_number;
    publication_data.target_scene = p_app->state.target_scene_number;
    publication_data.remaining_time_ms = remaining_time_ms;

    p_scene_srv = &p_app->scene_setup_server.scene_srv;

    return scene_server_status_publish((const scene_server_t *)p_scene_srv,
                                       (const scene_status_params_t *)&publication_data);
}

/***** Transition module callback function implementation *****/
/* Note: The task of the transition module is to implement gradual changes of `current_scene_number`
 * value for all possible time intervals and step size combinations allowed by the scene model,
 * and the callbacks from this module are used to implement actual scene value changes.
 *
 * If you need to implement transitions using external hardware or other mechanisms you can use
 * individual callbacks to initiate specific actions on external hardware.
 */
static app_scene_setup_server_t * transition_to_app(const app_transition_t * p_transition)
{
    app_scene_state_t * p_state = PARENT_BY_FIELD_GET(app_scene_state_t, transition, p_transition);

    return PARENT_BY_FIELD_GET(app_scene_setup_server_t, state, p_state);
}

static void transition_delay_start_cb(const app_transition_t * p_transition)
{
    app_scene_setup_server_t * p_app = transition_to_app(p_transition);

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Element %d: starting delay\n",
          p_app->scene_setup_server.settings.element_index);
}

static void transition_start_cb(const app_transition_t * p_transition)
{
    app_scene_setup_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
          "Element %d: starting transition: target_scene: %d delta: %d tt: %d\n",
          p_app->scene_setup_server.settings.element_index,
          p_app->state.target_scene_number,
          p_params->required_delta,
          p_params->transition_time_ms);

    p_app->state.current_scene_number = SCENE_NUMBER_NO_SCENE;

    if (p_app->app_scene_transition_cb != NULL)
    {
        p_app->app_scene_transition_cb(p_app, p_params->transition_time_ms,
                                       p_app->state.target_scene_number);
    }
}

static void transition_complete_cb(const app_transition_t * p_transition)
{
    app_scene_setup_server_t * p_app = transition_to_app(p_transition);
    app_transition_params_t * p_params = app_transition_ongoing_get(&p_app->state.transition);

    p_app->state.current_scene_number = p_app->state.target_scene_number;
    p_app->state.target_scene_number = SCENE_NUMBER_NO_SCENE;

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
          "Element %d: transition completed, current_scene: %d\n",
          p_app->scene_setup_server.settings.element_index,
          p_app->state.current_scene_number);

    /* The transition is complete */
    if (p_app->app_scene_transition_cb != NULL)
    {
        p_app->app_scene_transition_cb(p_app, p_params->transition_time_ms,
                                       p_app->state.target_scene_number);
    }

    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) publish_status(p_app, SCENE_STATUS_SUCCESS, p_params->transition_time_ms);
}


/***** Interface functions *****/

uint32_t app_scene_model_init(app_scene_setup_server_t * p_app, uint8_t element)
{
    uint32_t status;

    if ((p_app == NULL) ||
        (p_app->app_scene_transition_cb == NULL))
    {
        return NRF_ERROR_NULL;
    }

    status = app_scene_setup_server_init(&p_app->scene_setup_server, element);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_app->state.current_scene_number = SCENE_NUMBER_NO_SCENE;

    p_app->state.transition.delay_start_cb = transition_delay_start_cb;
    p_app->state.transition.transition_start_cb = transition_start_cb;
    p_app->state.transition.transition_tick_cb = NULL;
    p_app->state.transition.transition_complete_cb = transition_complete_cb;
    p_app->state.transition.p_context = &p_app->state.transition;

    status = app_transition_init(&p_app->state.transition);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_app->next_model_interface = 0;

    return status;
}

uint32_t app_scene_model_add(app_scene_setup_server_t * p_app,
                             app_scene_model_interface_t * p_app_scene_model_interface)
{
    if ((p_app == NULL) || (p_app_scene_model_interface == NULL))
    {
        return NRF_ERROR_NULL;
    }

    if (p_app->next_model_interface >= ARRAY_SIZE(p_app->scene_models))
    {
        return NRF_ERROR_NO_MEM;
    }

    p_app->scene_models[p_app->next_model_interface] = p_app_scene_model_interface;
    p_app->next_model_interface++;

    return NRF_SUCCESS;
}

void app_scene_model_scene_changed(app_scene_setup_server_t * p_app)
{
    p_app->state.current_scene_number = SCENE_NUMBER_NO_SCENE;
}

#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0*/
