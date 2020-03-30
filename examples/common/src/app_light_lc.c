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
    if (light_lc_mc_state_set(p_s_server->state.handle,lc_state, p_value) != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot set lc_state %d\n", lc_state);
    }
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

    ERROR_CHECK(light_lightness_mc_onpowerup_state_get(p_app->p_app_ll->light_lightness_setup_server.state.handle,
                                                     &onpowerup_value));

    return light_lc_setup_server_ponoff_binding_setup(&p_app->light_lc_setup_srv, onpowerup_value, p_lc_control);
}
