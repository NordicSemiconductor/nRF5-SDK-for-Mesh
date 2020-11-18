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

#ifndef APP_LIGHT_LC_H__
#define APP_LIGHT_LC_H__

#include <stdint.h>

#include "app_light_lightness.h"
#include "light_lc_setup_server.h"
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "app_scene.h"
#endif

/**
 * @defgroup APP_LIGHT_LC Light LC Setup Server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application Light LC Setup Server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Light LC Setup Server model.
 *
 * The application should use the set callback provided by this module to set the hardware state.
 * The hardware state could be changed by reflecting the value provided by the set callback on the
 * GPIO or by sending this value to the connected lighting peripheral using some other interface
 * (e.g.  serial interface). Similarly, the application should use the get callback provided by this
 * module to read the hardware state.
 *
 * This module triggers the set callback only when it determines that it is time to inform the user
 * application. It is possible that the client can send multiple overlapping set commands.  In such
 * case any transition in progress will be abandoned and fresh transition will be started if
 * required.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to
 * the requirements defined in the following sections: @tagMeshMdlSp section 6.2 (Lighting control) and section
 * 6.5 (Lighting control models).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_light_lc_setup_server_t context.
 *
 * Individual timer instances are created for each model instance.
 * An app light lightness structure needs to be created separately for each model instance.
 *
 * @param[in] _name                 Name of the [app_light_lc_setup_server_t](@ref __app_light_lc_setup_server_t) instance.
 * @param[in] _force_segmented      If the Light LC Setup Server shall use force segmentation of messages.
 * @param[in] _mic_size             MIC size to be used by the Light LC Setup Server.
 */

#define APP_LIGHT_LC_SETUP_SERVER_DEF(_name, _force_segmented, _mic_size)                     \
    APP_TIMER_DEF(_name ## _fsm_timer);                                                       \
    APP_TIMER_DEF(_name ## _light_pi_timer);                                                  \
    APP_TIMER_DEF(_name ## _sensor_delay_timer);                                              \
    static app_light_lc_setup_server_t _name =                                                \
    {                                                                                         \
        .light_lc_setup_srv.settings.force_segmented = _force_segmented,                      \
        .light_lc_setup_srv.settings.transmic_size = _mic_size,                               \
        .light_lc_setup_srv.fsm_timer.p_timer_id = &_name ## _fsm_timer,                      \
        .light_lc_setup_srv.light_pi_timer.p_timer_id = &_name ## _light_pi_timer,            \
        .light_lc_setup_srv.sensor_delay_timer.p_timer_id = &_name ## _sensor_delay_timer,    \
    };

/* Forward declaration */
typedef struct __app_light_lc_setup_server_t app_light_lc_setup_server_t;

/** Application level structure holding the LC Setup server model context */
struct __app_light_lc_setup_server_t
{
    /** LC setup server context */
    light_lc_setup_server_t light_lc_setup_srv;

    /** Pointer to the light lightness app structure */
    app_light_lightness_setup_server_t * p_app_ll;
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
    /** Internal variable. Scene callback interface. 
     * @note Available only if  @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_model_interface_t scene_if;
    /** Internal variable. Pointer to app_scene context. 
     * @note Available only if  @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_setup_server_t  * p_app_scene;
#endif
};

/** Initializes the behavioral module for the Light LC Setup Server model
 *
 * @param[in] p_app                  Pointer to [app_light_lc_setup_server_t](@ref
 *                                   __app_light_lc_setup_server_t) context.
 * @param[in] element_index          Element index on which this server will be instantiated.
 * @param[in] p_app_ll               Pointer to [app_light_lightness_setup_server_t](@ref
 *                                   __app_light_lightness_setup_server_t) context.
 *
 * @retval NRF_SUCCESS               If initialization is successful
 * @retval NRF_ERROR_NULL            NULL pointer is supplied to the function
 * @retval NRF_ERROR_RESOURCES       No more instances can be created. In that case, increase value
 *                                   of @ref LIGHT_LC_SETUP_SERVER_INSTANCES_MAX.
 * @retval NRF_ERROR_INVALID_PARAM   If the application timer module has not been initialized.
 * @retval NRF_ERROR_INVALID_STATE   If the application timer is running.
 * @retval NRF_ERROR_NO_MEM          No memory available to send the message at this point.
 * @retval NRF_ERROR_FORBIDDEN       Device has been provisioned and changes to model subscription
 *                                   list are not allowed.
 * @retval NRF_ERROR_NOT_FOUND       Access handle invalid.
 */
uint32_t app_light_lc_model_init(app_light_lc_setup_server_t * p_app,
                                 uint8_t element_index,
                                 app_light_lightness_setup_server_t * p_app_ll);

/** Informs the model that the system is ready to have the powerup onoff bindings
 *
 * This is called by main.c when the mesh is initialized and stable.  Note that this function must
 * be called from the same IRQ level that mesh_init() is set at.
 *
 * @param[in]  p_app                Pointer to [app_light_lc_setup_server_t](@ref
 *                                  __app_light_lc_setup_server_t) context.
 * @param[out] p_lc_control         Returns true if Light LC is controlling the system, false if it
 *                                  is not in control.
 *
 * @retval NRF_SUCCESS              Bindings are setup successfully
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied, or specified timeout is too
 *                                  short.
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 */
uint32_t app_light_lc_ponoff_binding(app_light_lc_setup_server_t * p_app, bool * p_lc_control);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
/** Sets the scene context
 *
 * This is needed for app light lc to inform app scene when the state change occurs.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] p_app                 Pointer to [app_light_lc_setup_server_t](@ref
 *                                  __app_light_lc_setup_server_t) context.
 * @param[in] p_app_scene           Pointer to scene behavioral moduel context.
 *
 * @retval NRF_SUCCESS              Value is restored successfully
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 */
uint32_t app_light_lc_scene_context_set(app_light_lc_setup_server_t * p_app, 
                                         app_scene_setup_server_t  * p_app_scene);
#endif

/** @} end of APP_LIGHT_LC */
#endif /* APP_LIGHT_LC_H__*/
