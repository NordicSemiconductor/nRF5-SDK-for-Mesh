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

#ifndef APP_DTT_H__
#define APP_DTT_H__

#include <stdint.h>

#include "generic_dtt_server.h"

/**
 * @defgroup APP_DTT Generic Default Transition Time server app support file
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level Default Transition Time server behavioral structures, functions, and callbacks.
 *
 * This module implements a thin layer to store and restore Defaut Transition time states for the
 * applications which want to use Default Transition Time model independently (unlike app modules
 * for some other models that encapsulate DTT state storage). This can be utilized to add Default
 * Transition Time support to simple applications based on Generic OnOff or Level servers.
 *
 * @{
 */

/**
 * Macro to create application level app_dtt_server_t context.
 *
 * @param[in] _name                 Name of the app_dtt_server_t instance
 * @param[in] _force_segmented      If the Generic DTT server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic DTT server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 *                                  If this callback is not used by the appplication, this can be
 *                                  set to NULL.
 */
#define APP_DTT_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb)  \
    static app_dtt_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .dtt_set_cb = _set_cb,  \
    };

/* Forward declaration */
typedef struct __app_dtt_server_t app_dtt_server_t;

/** Application state set callback prototype.
 *
 * This callback is called by the this module whenever application is required to
 * be informed to reflect the desired DTT value. Usually it is not necessary to implement this
 * callback since default transition time values are usually consumed by the models internally.
 *
 * @param[in]   p_app           Pointer to [app_dtt_server_t](@ref __app_dtt_server_t) context.
 * @param[in]   default_tt      New default transition time value to be used by the application.
 */
typedef void (*app_dtt_set_cb_t)(const app_dtt_server_t * p_app, uint32_t default_tt);

/** Application level structure holding the DTT server model context and DTT state representation */
struct __app_dtt_server_t
{
    /** DTT server model interface context structure */
    generic_dtt_server_t server;

    /** Callaback to be called for informing the user application to update the value*/
    app_dtt_set_cb_t  dtt_set_cb;

    /** Internal variable. Representation of the DTT state required for transition behavioral
     * implementation of lighting models. */
    uint32_t default_transition_time;
};


/** Initializes the behavioral module for the generic DTT model
 *
 * @param[in] p_app                 Pointer to [app_dtt_server_t](@ref __app_dtt_server_t)
 *                                  context.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              If initialization is successful.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated, or
 *                                  no more subscription lists available in memory pool.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index, or access handle invalid.
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval  NRF_ERROR_INVALID_PARAM Model not bound to appkey, publish address not set or wrong
 *                                  opcode format. The application timer module has not been
 *                                  initialized or timeout handler is not provided.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer is running.
*/
uint32_t app_dtt_init(app_dtt_server_t * p_app, uint8_t element_index);

/** @} end of APP_DTT */
#endif /* APP_DTT_H__ */
