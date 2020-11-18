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

#include "app_light_lc.h"

/* Application */
#include "light_lightness_mc.h"
#include "light_lc_state_utils.h"
#include "light_lc_mc.h"
#include "light_lc_fsm.h"

#include "mesh_app_utils.h"

/** This sample implementation shows how the model behavior requirements of LC Setup server can be
 * implemented.
 */

/* Forward declarations */

/* LC callbacks */
static void app_persist_set_cb(const light_lc_setup_server_t * p_s_server,
                               const light_lc_state_t lc_state,
                               const void * value_to_set);

static void app_persist_get_cb(const light_lc_setup_server_t * p_s_server,
                               const light_lc_state_t lc_state,
                               void * p_retval);

static void app_actual_set_cb(const light_lc_setup_server_t * p_s_server,
                              const uint16_t actual_value);

static void app_actual_get_cb(const light_lc_setup_server_t * p_s_server,
                              uint16_t * p_actual_value);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    const struct 
    {
        const light_lc_state_t lc_state;
    } lc_state_vector[] = {
        { LIGHT_LC_STATE_LIGHT_LC_OCC_MODE },
        { LIGHT_LC_STATE_LIGHT_LC_MODE },
        { LIGHT_LC_STATE_LIGHT_LC_LIGHT_ONOFF },
        { LIGHT_LC_STATE_AMBIENT_LUXLEVEL_ON },
        { LIGHT_LC_STATE_AMBIENT_LUXLEVEL_PROLONG },
        { LIGHT_LC_STATE_AMBIENT_LUXLEVEL_STANDBY },
        { LIGHT_LC_STATE_LIGHTNESS_ON },
        { LIGHT_LC_STATE_LIGHTNESS_PROLONG },
        { LIGHT_LC_STATE_LIGHTNESS_STANDBY },
        { LIGHT_LC_STATE_REGULATOR_ACCURACY },
        { LIGHT_LC_STATE_REGULATOR_KID },
        { LIGHT_LC_STATE_REGULATOR_KIU },
        { LIGHT_LC_STATE_REGULATOR_KPD },
        { LIGHT_LC_STATE_REGULATOR_KPU },
        { LIGHT_LC_STATE_TIME_FADE },
        { LIGHT_LC_STATE_TIME_FADE_ON },
        { LIGHT_LC_STATE_TIME_FADE_STANDBY_AUTO },
        { LIGHT_LC_STATE_TIME_FADE_STANDBY_MANUAL },
        { LIGHT_LC_STATE_TIME_OCCUPANCY_DELAY },
        { LIGHT_LC_STATE_TIME_PROLONG },
        { LIGHT_LC_STATE_TIME_RUN_ON },
    };

static void app_light_lc_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                     uint8_t scene_index);
static void app_light_lc_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                      uint8_t scene_index,
                                      uint32_t delay_ms,
                                      uint32_t transition_time_ms);
static void app_light_lc_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                      uint8_t scene_index);

const app_scene_callbacks_t m_scene_light_lc_cbs =
{
    .scene_store_cb = app_light_lc_scene_store,
    .scene_recall_cb = app_light_lc_scene_recall,
    .scene_delete_cb = app_light_lc_scene_delete
};
#endif

/**** end of callback declarations ****/

static const light_lc_setup_server_callbacks_t m_lc_setup_srv_cbs =
{
    .light_lc_cbs.light_lc_persist_set_cb = app_persist_set_cb,
    .light_lc_cbs.light_lc_persist_get_cb = app_persist_get_cb,
    .light_lc_cbs.light_lc_actual_set_cb = app_actual_set_cb,
    .light_lc_cbs.light_lc_actual_get_cb = app_actual_get_cb,
};

/* Callback used by light lightness to inform us (LC) when it has done a lightness set.  This is
 * to cover the case where a Light Lightness command was sent down (not an LC command) meaning that
 * LC should make sure it is not controlling the lightness (@tagMeshMdlSp secton 6.2.3.1 Upon an
 * unsolicted change of the Light Lightness Linear state ... Light LC Mode = 0b0) */
static void add_lightness_set_cb(const void * p_app_v, uint16_t lightness)
{
    app_light_lc_setup_server_t * p_app = (app_light_lc_setup_server_t *) p_app_v;
    uint8_t mode;

    ERROR_CHECK(light_lc_mc_state_get(p_app->light_lc_setup_srv.state.handle, LIGHT_LC_STATE_LIGHT_LC_MODE, &mode));
    if (mode)
    {
        /* The LC controller is controlling the light, but now lightness has stated it has changed
         * the state.  Turn off the LC mode */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Turning off LC control (lightness was set)\n")
        ERROR_CHECK(light_lc_fsm_mode_on_off_event_generate(&p_app->light_lc_setup_srv, false));
    }
}


/* LC model interface callbacks */

static void app_persist_set_cb(const light_lc_setup_server_t * p_s_server,
                               const light_lc_state_t lc_state,
                               const void * p_value)
{
#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    uint32_t old_value = 0;
    ERROR_CHECK(light_lc_mc_state_get(p_s_server->state.handle, lc_state, &old_value));
#endif

    if (light_lc_mc_state_set(p_s_server->state.handle, lc_state, p_value) != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot set lc_state %d\n", lc_state);
    }

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    uint32_t new_value = 0;
    ERROR_CHECK(light_lc_mc_state_get(p_s_server->state.handle, lc_state, &new_value));
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, 
            "Persist set: lc_state[%d]  old_value: %lu  new_value: %lu\n", 
            lc_state, old_value, new_value);
    if (old_value != new_value)
    {
        app_light_lc_setup_server_t * p_app;
        p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, light_lc_setup_srv, p_s_server);
        app_scene_model_scene_changed(p_app->p_app_scene);
    }
#endif
}

static void app_persist_get_cb(const light_lc_setup_server_t * p_s_server,
                               const light_lc_state_t lc_state,
                               void *p_retval)
{
    ERROR_CHECK(light_lc_mc_state_get(p_s_server->state.handle, lc_state, p_retval));
}


/* Call the light lightness mid app code to set the lightness */
static void app_actual_set_cb(const light_lc_setup_server_t * p_s_server,
                              uint16_t actual_lightness)
{
    app_light_lc_setup_server_t * p_app;
    app_light_lightness_setup_server_t * p_app_ll;

    p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, light_lc_setup_srv, p_s_server);

    NRF_MESH_ASSERT(p_app != NULL);

    p_app_ll = p_app->p_app_ll;

    ERROR_CHECK(app_light_lightness_direct_actual_set(p_app_ll, actual_lightness));
}

/* Call the light lightness mid app code to get the lightness */
static void app_actual_get_cb(const light_lc_setup_server_t * p_s_server,
                              uint16_t * p_actual_value)
{
    app_light_lc_setup_server_t * p_app;
    app_light_lightness_setup_server_t * p_app_ll;
    light_lightness_status_params_t out_data;
    light_lightness_setup_server_state_cbs_t * p_ll_cbs;

    p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, light_lc_setup_srv, p_s_server);
    NRF_MESH_ASSERT(p_app != NULL);

    p_app_ll = p_app->p_app_ll;

    p_ll_cbs = (light_lightness_setup_server_state_cbs_t *)
        &(p_app_ll->light_lightness_setup_server.settings.p_callbacks->light_lightness_cbs);

    p_ll_cbs->get_cb(&p_app_ll->light_lightness_setup_server, NULL, &out_data);
    *p_actual_value = out_data.present_lightness;
}


/* end of callback definitions */

static uint32_t app_lc_setup_server_init(light_lc_setup_server_t * p_s_server, uint8_t element_index)
{
    if (!p_s_server)
    {
        return NRF_ERROR_NULL;
    }
    p_s_server->settings.p_callbacks = &m_lc_setup_srv_cbs;

    return light_lc_setup_server_init(p_s_server, element_index);
}

/***** Scene Interface functions *****/

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_light_lc_scene_store(const app_scene_model_interface_t * p_app_scene_if,
                                     uint8_t scene_index)
{
    app_light_lc_setup_server_t * p_app;

    p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, scene_if, p_app_scene_if);

    /* The light_lightness storeage is not added to the scene, but is handled throught the light lc
       setup server. */
    p_app->p_app_ll->scene_if.p_callbacks->scene_store_cb(&p_app->p_app_ll->scene_if, scene_index);

    for(uint32_t i = 0; i < ARRAY_SIZE(lc_state_vector); i++)
    {
        uint32_t value = 0;
        ERROR_CHECK(light_lc_mc_state_get(p_app->light_lc_setup_srv.state.handle,
                                            lc_state_vector[i].lc_state, 
                                            &value));
        ERROR_CHECK(light_lc_mc_scene_state_store(p_app->light_lc_setup_srv.state.handle,
                                                    scene_index, 
                                                    lc_state_vector[i].lc_state, 
                                                    &value));
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG2, "SCENE STORE: lc_state[%d]: value: %lu\n",
                lc_state_vector[i].lc_state,
                value);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "SCENE STORE: handel: %d  scene index: %d\n",
            p_app->light_lc_setup_srv.state.handle,
            scene_index);
}

static void app_light_lc_scene_recall(const app_scene_model_interface_t * p_app_scene_if,
                                       uint8_t scene_index,
                                       uint32_t delay_ms,
                                       uint32_t transition_time_ms)
{
    app_light_lc_setup_server_t * p_app;

    p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, scene_if, p_app_scene_if);

    /* The Scene Recall behavior for the Light LC Server model has different handling of the recall
       operation based on the Light LC Mode. See @tagMeshMdlSp secton 6.5.1.3.2.*/
    uint8_t mode;
    ERROR_CHECK(light_lc_mc_scene_state_recall(p_app->light_lc_setup_srv.state.handle,
                                               scene_index, 
                                               LIGHT_LC_STATE_LIGHT_LC_MODE, 
                                               &mode));

    /* The Recall operation shall start with the Light LC State Machine in Off state. */
    ERROR_CHECK(light_lc_fsm_mode_on_off_event_generate(&p_app->light_lc_setup_srv, false));

    if (!mode) /* Light LC Mode is equal to 0b0. */
    {
        /* The light_lightness recall is only excecuted when the LC Mode is off. */
        p_app->p_app_ll->scene_if.p_callbacks->scene_recall_cb(&p_app->p_app_ll->scene_if,
                                                               scene_index,
                                                               delay_ms,
                                                               transition_time_ms);
    }

    /* Start recall operations. */
    for (uint32_t i = 0; i < ARRAY_SIZE(lc_state_vector); i++)
    {
        uint32_t old_value = 0;
        uint32_t value = 0;
        ERROR_CHECK(light_lc_mc_state_get(p_app->light_lc_setup_srv.state.handle,
                                            lc_state_vector[i].lc_state, 
                                            &old_value));
        ERROR_CHECK(light_lc_mc_scene_state_recall(p_app->light_lc_setup_srv.state.handle,
                                                scene_index, 
                                                lc_state_vector[i].lc_state, 
                                                &value));
        ERROR_CHECK(light_lc_mc_state_set(p_app->light_lc_setup_srv.state.handle,
                                        lc_state_vector[i].lc_state, 
                                        &value));

        if (old_value != value)
        {
            uint16_t property_id;
            uint8_t property_value_size;
            uint32_t property_value = 0;
            light_lc_property_status_params_t out_data = {0};
            ERROR_CHECK(light_lc_state_utils_property_id_from_lc_state(lc_state_vector[i].lc_state, &property_id));
            ERROR_CHECK(light_lc_state_utils_property_data_size_get(property_id, &property_value_size));
            property_value = light_lc_state_utils_property_get(&p_app->light_lc_setup_srv, property_id);
            out_data.property_id = property_id;
            memcpy(out_data.property_buffer, &property_value, property_value_size);
            (void) light_lc_setup_server_property_status_publish(&p_app->light_lc_setup_srv, &out_data);           
        }
    }

    /* Set Light LC Mode state to current value*/
    ERROR_CHECK(light_lc_fsm_mode_on_off_event_generate(&p_app->light_lc_setup_srv, 
                                                        (bool) mode));

    /* Recall Light LC Occupancy Mode state */
    light_lc_occupancy_mode_status_params_t out_occ_data;
    out_occ_data.occupancy_mode = light_lc_state_utils_occ_mode_get(&p_app->light_lc_setup_srv);
    uint32_t status = light_lc_server_occ_mode_status_publish(&p_app->light_lc_setup_srv.lc_srv, &out_occ_data);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "light_lc_server_occ_mode_status_publish ignore publish_status: status(%d).\n", status);
    }


    /* Recall Light LC Light OnOff state */
    light_lc_light_onoff_status_params_t out_onoff_data =
                        light_lc_state_utils_light_onoff_get(&p_app->light_lc_setup_srv);
    ERROR_CHECK(light_lc_fsm_light_on_off_event_generate(&p_app->light_lc_setup_srv, 
                                                        (bool) out_onoff_data.present_light_onoff, 
                                                        NULL));

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "SCENE RECALL:  handle %d  scene index %d  mode: %d\n",
              p_app->light_lc_setup_srv.state.handle,
              scene_index,
              mode);
}

static void app_light_lc_scene_delete(const app_scene_model_interface_t * p_app_scene_if,
                                      uint8_t scene_index)
{
    app_light_lc_setup_server_t * p_app;
    p_app = PARENT_BY_FIELD_GET(app_light_lc_setup_server_t, scene_if, p_app_scene_if);

    /* The light_lightness recall is only excecuted when the LC Mode is off. */
    p_app->p_app_ll->scene_if.p_callbacks->scene_delete_cb(&p_app->p_app_ll->scene_if, scene_index);

    /* No need to do anything else */
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0 */


/***** Interface functions *****/

uint32_t app_light_lc_model_init(app_light_lc_setup_server_t * p_app,
                                 uint8_t element_index,
                                 app_light_lightness_setup_server_t * p_app_ll)
{
    uint32_t status;
    if (p_app == NULL || p_app_ll == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->p_app_ll = p_app_ll;

    /* Set up the added callback from light lightness to LC */
    p_app->p_app_ll->app_add_notify.p_app_notify_v = p_app;
    p_app->p_app_ll->app_add_notify.app_notify_set_cb = add_lightness_set_cb;

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    p_app->scene_if.p_callbacks = &m_scene_light_lc_cbs;
#endif

    /* Setup the flash for the LC server */
    status = light_lc_mc_open(&p_app->light_lc_setup_srv.state.handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize the LC setup server */
    return app_lc_setup_server_init(&p_app->light_lc_setup_srv, element_index);
}

uint32_t app_light_lc_ponoff_binding(app_light_lc_setup_server_t * p_app, bool * p_lc_control)
{
    uint8_t onpowerup_value;

    if ((p_app == NULL) || (p_lc_control == NULL))
    {
        return NRF_ERROR_NULL;
    }

    ERROR_CHECK(light_lightness_mc_onpowerup_state_get(
                    p_app->p_app_ll->light_lightness_setup_server.state.handle,
                    &onpowerup_value));

    return light_lc_setup_server_ponoff_binding_setup(&p_app->light_lc_setup_srv, 
                                                      onpowerup_value, 
                                                      p_lc_control);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t app_light_lc_scene_context_set(app_light_lc_setup_server_t * p_app, 
                                        app_scene_setup_server_t  * p_app_scene)
{
    if (p_app == NULL || p_app_scene == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->p_app_scene = p_app_scene;
    return (app_light_lightness_scene_context_set(p_app->p_app_ll, p_app_scene));
}
#endif
