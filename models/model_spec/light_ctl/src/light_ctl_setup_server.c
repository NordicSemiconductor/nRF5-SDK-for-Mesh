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

#include "light_ctl_setup_server.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "utils.h"

#include "light_ctl_messages.h"
#include "light_ctl_utils.h"

#include "mesh_config_entry.h"
#include "mesh_config.h"

#include "access.h"
#include "access_config.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "light_ctl_utils.h"


/* This factor must be 65535, for maintaining accuracy during integer calculations performed on
 * the temperature values by behavioral part of this model. */
STATIC_ASSERT(T32_SCALE_FACTOR == 65535);

static uint32_t m_total_ctl_ss_instances;

typedef enum
{
    CTL_SERVER,
    CTL_SETUP_SERVER,
} server_context_type_t;

/* Internal structure for communicating to bound publish function. */
typedef struct
{
    uint32_t present_temperature32;          /**< The present value of the Temperature32 state */
    uint32_t target_temperature32;           /**< The target value of the Temperature32 state (optional) */
    uint32_t remaining_time_ms;              /**< Remaining time value in milliseconds */
} ctl_temperature_params_t;

/** Forward declarations. */
static void ctl_temperature_bound_state_publish(const light_ctl_setup_server_t * p_s_server,
                                                const ctl_temperature_params_t * p_params);



static uint32_t status_send(const light_ctl_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const light_ctl_status_params_t * p_params)
{
    light_ctl_status_msg_pkt_t msg_pkt;

    msg_pkt.present_lightness = p_params->present_lightness;
    msg_pkt.present_temperature = light_ctl_utils_temperature32_to_temperature(p_params->present_temperature32);
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_lightness = p_params->target_lightness;
        msg_pkt.target_temperature = light_ctl_utils_temperature32_to_temperature(p_params->target_temperature32);
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   LIGHT_CTL_STATUS_MAXLEN :
                   LIGHT_CTL_STATUS_MINLEN),
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}


static uint32_t status_temperature_send(const light_ctl_temperature_server_t * p_server,
                                        const access_message_rx_t * p_message,
                                        const light_ctl_temperature_status_params_t * p_params)
{
    light_ctl_temperature_status_msg_pkt_t msg_pkt;

    msg_pkt.present_temperature = light_ctl_utils_temperature32_to_temperature(p_params->present_temperature32);
    msg_pkt.present_delta_uv = p_params->present_delta_uv;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_temperature = light_ctl_utils_temperature32_to_temperature(p_params->target_temperature32);
        msg_pkt.target_delta_uv = p_params->target_delta_uv;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   LIGHT_CTL_TEMPERATURE_STATUS_MAXLEN :
                   LIGHT_CTL_TEMPERATURE_STATUS_MINLEN),
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}

static uint32_t status_temperature_range_send(const void * p_ctx,
                                              server_context_type_t ctx_type,
                                              const access_message_rx_t * p_message,
                                              const light_ctl_temperature_range_status_params_t * p_params)
{
    light_ctl_temperature_range_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (ctx_type == CTL_SERVER)
    {
        light_ctl_server_t * p_server = (light_ctl_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        light_ctl_setup_server_t * p_s_server = (light_ctl_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    msg_pkt.status_code = p_params->status_code;
    msg_pkt.range_min = light_ctl_utils_temperature32_to_temperature(p_params->temperature32_range_min);
    msg_pkt.range_max = light_ctl_utils_temperature32_to_temperature(p_params->temperature32_range_max);

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_CTL_TEMPERATURE_RANGE_STATUS_LEN,
        .force_segmented = force_segmented,
        .transmic_size = transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(model_handle, &reply);
    }
    else
    {
        return access_model_reply(model_handle, p_message, &reply);
    }
}

static uint32_t status_default_send(const void * p_ctx,
                                    server_context_type_t ctx_type,
                                    const access_message_rx_t * p_message,
                                    const light_ctl_default_status_params_t * p_params)
{
    light_ctl_default_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (ctx_type == CTL_SERVER)
    {
        const light_ctl_server_t * p_server = (light_ctl_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        const light_ctl_setup_server_t * p_s_server = (light_ctl_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    msg_pkt.lightness = p_params->lightness;
    msg_pkt.temperature = light_ctl_utils_temperature32_to_temperature(p_params->temperature32);
    msg_pkt.delta_uv = p_params->delta_uv;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_DEFAULT_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_CTL_DEFAULT_STATUS_LEN,
        .force_segmented = force_segmented,
        .transmic_size = transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(model_handle, &reply);
    }
    else
    {
        return access_model_reply(model_handle, p_message, &reply);
    }
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    light_ctl_server_t * p_server = (light_ctl_server_t *)p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_args);
    light_ctl_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->light_ctl_cbs.get_cb(p_s_server, NULL, &out_data);

    (void) status_send(p_server, NULL, &out_data);
}

static void periodic_temperature_publish_cb(access_model_handle_t handle, void * p_args)
{
    light_ctl_temperature_server_t * p_server = (light_ctl_temperature_server_t *)p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv,
                                                          p_args);
    light_ctl_temperature_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_get_cb(p_s_server, NULL, &out_data);

    (void) status_temperature_send(p_server, NULL, &out_data);
}

/* Utility functions */
static uint32_t clip_temperature32_to_range(const light_ctl_setup_server_t * p_s_server,
                                            const access_message_rx_meta_t * p_meta,
                                            uint32_t temperature32)
{
    light_ctl_temperature_range_status_params_t range = {0};

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, p_meta, &range);
    temperature32 = light_ctl_utils_temperature32_range_restrict(temperature32,
                                                           range.temperature32_range_min,
                                                           range.temperature32_range_max);

    return temperature32;
}

static bool handle_set_message_validate(uint16_t short_message_bytes,
                                        uint16_t long_message_bytes,
                                        uint16_t message_bytes,
                                        uint8_t transition_time,
                                        uint8_t delay,
                                        model_transition_t ** pp_transition)
{
    model_transition_t * p_transition;

    p_transition = *pp_transition;

    if (short_message_bytes == message_bytes)
    {
        /* The message does not specify a transition. */
        *pp_transition = NULL;
        return true;
    }

    if (long_message_bytes == message_bytes)
    {
        /* The message specifies a transition. */
        if (!model_transition_time_is_valid(transition_time))
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid parameter (%d) = (transition_time)\n",
                  transition_time);
            return false;
        }
        p_transition->transition_time_ms = model_transition_time_decode(transition_time);
        p_transition->delay_ms = model_delay_decode(delay);
        return true;
    }

    /* The message length is invalid. */
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid parameter (%d) = (message_bytes)\n", message_bytes);

    return false;
}

/**************************************************************************************************/
/**** CTL server ****/
/** Opcode Handlers */
static void handle_set(access_model_handle_t model_handle,
                       const access_message_rx_t * p_rx_msg,
                       void * p_args)
{
    light_ctl_server_t * p_server;
    light_ctl_setup_server_t * p_s_server;
    light_ctl_set_msg_pkt_t * p_msg_pkt;
    light_ctl_status_params_t out_data = {0};
    light_ctl_status_params_t * p_out_data = NULL;
    light_ctl_set_params_t in_data = {0};
    model_transition_t transition = {0};
    model_transition_t * p_transition;

    p_server = (light_ctl_server_t *) p_args;
    p_msg_pkt = (light_ctl_set_msg_pkt_t *) p_rx_msg->p_data;
    if (!model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data,
                            LIGHT_CTL_OPCODE_SET, p_msg_pkt->tid))
    {
        return;
    }

    p_transition = &transition;
    if (!handle_set_message_validate(LIGHT_CTL_SET_MINLEN, LIGHT_CTL_SET_MAXLEN, p_rx_msg->length,
                                     p_msg_pkt->transition_time, p_msg_pkt->delay, &p_transition))
    {
        return;
    }

    /* Check for prohibited values @tagMeshMdlSp section 6.1.3.1.  For any prohibited values, don't process,
     * and don't respond */
    if ((p_msg_pkt->temperature < LIGHT_CTL_TEMPERATURE_MIN_LIMIT) || (p_msg_pkt->temperature > LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
    {
        return;
    }

    in_data.lightness = p_msg_pkt->lightness;
    in_data.temperature32 = light_ctl_utils_temperature_to_temperature32(p_msg_pkt->temperature);
    in_data.delta_uv = p_msg_pkt->delta_uv;
    in_data.tid = p_msg_pkt->tid;

    p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_server);

    in_data.temperature32 = clip_temperature32_to_range(p_s_server, &p_rx_msg->meta_data,
                                                        in_data.temperature32);

    if (LIGHT_CTL_OPCODE_SET == p_rx_msg->opcode.opcode)
    {
        p_out_data = &out_data;
    }

    p_s_server->settings.p_callbacks->light_ctl_cbs.set_cb(p_s_server, &p_rx_msg->meta_data, &in_data,
                                                     p_transition, p_out_data);

    if (p_out_data != NULL)
    {
        (void) status_send(p_server, p_rx_msg, p_out_data);
    }
}

static void handle_get(access_model_handle_t model_handle,
                       const access_message_rx_t * p_rx_msg,
                       void * p_args)
{
    light_ctl_server_t * p_server = (light_ctl_server_t *) p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_args);
    light_ctl_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_ctl_cbs.get_cb(p_s_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_temperature_range_get(access_model_handle_t model_handle,
                                         const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_ctl_server_t * p_server = (light_ctl_server_t *) p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_args);
    light_ctl_temperature_range_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server,
                                                                             &p_rx_msg->meta_data,
                                                                             &out_data);

        (void) status_temperature_range_send(p_server, CTL_SERVER, p_rx_msg, &out_data);
    }
}

static void handle_default_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_ctl_server_t * p_server = (light_ctl_server_t *) p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_args);
    light_ctl_default_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_ctl_cbs.default_get_cb(p_s_server, &p_rx_msg->meta_data,
                                                                 &out_data);

        (void) status_default_send(p_server, CTL_SERVER, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_SET_UNACKNOWLEDGED), handle_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_GET), handle_temperature_range_get},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_DEFAULT_GET), handle_default_get},
};

static uint32_t ctl_server_init(light_ctl_server_t * p_server, uint8_t element_index)
{
    uint32_t status;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Add this ctl server model to the mesh */
    access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(LIGHT_CTL_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers),
            .p_args = p_server,
            .publish_timeout_cb = periodic_publish_cb
        };

    status = access_model_add(&init_params, &p_server->model_handle);

    return status;
}

uint32_t light_ctl_server_status_publish(const light_ctl_server_t * p_server,
                                         const light_ctl_status_params_t * p_params)
{
    light_ctl_setup_server_t * p_s_server;
    ctl_temperature_params_t status_params;

    if ((p_server == NULL) || (p_params == NULL))
    {
        return NRF_ERROR_NULL;
    }

    p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_srv, p_server);

    /* Make sure to publish the bound state, then publish the CTL state */
    status_params.present_temperature32 = p_params->present_temperature32;
    status_params.target_temperature32 = p_params->target_temperature32;
    status_params.remaining_time_ms = p_params->remaining_time_ms;

    ctl_temperature_bound_state_publish(p_s_server, &status_params);

    return status_send(p_server, NULL, p_params);
}

uint32_t light_ctl_server_temperature_status_publish(const light_ctl_temperature_server_t * p_server,
                                                     const light_ctl_temperature_status_params_t * p_params)
{
    light_ctl_setup_server_t * p_s_server;
    ctl_temperature_params_t status_params;

    if ((p_server == NULL) || (p_params == NULL))
    {
        return NRF_ERROR_NULL;
    }

    p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv, p_server);

    status_params.present_temperature32 = p_params->present_temperature32;
    status_params.target_temperature32 = p_params->target_temperature32;
    status_params.remaining_time_ms = p_params->remaining_time_ms;

    ctl_temperature_bound_state_publish(p_s_server, &status_params);
    return status_temperature_send(p_server, NULL, p_params);
}

uint32_t light_ctl_server_temperature_range_status_publish(const light_ctl_server_t * p_server,
                                                           const light_ctl_temperature_range_status_params_t * p_params)
{
    if ((p_server == NULL) || (p_params == NULL))
    {
        return NRF_ERROR_NULL;
    }

    return status_temperature_range_send(p_server, CTL_SERVER, NULL, p_params);
}

uint32_t light_ctl_server_default_status_publish(const light_ctl_server_t * p_server,
                                                 const light_ctl_default_status_params_t * p_params)
{
    if ((p_server == NULL) || (p_params == NULL))
    {
        return NRF_ERROR_NULL;
    }

    return status_default_send(p_server, CTL_SERVER, NULL, p_params);
}


/**************************************************************************************************/
/**** CTL Temperature Server */
/** Opcode Handlers */

static void handle_temperature_set(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    light_ctl_temperature_server_t * p_server;
    light_ctl_setup_server_t * p_s_server;
    light_ctl_temperature_set_msg_pkt_t * p_msg_pkt;
    light_ctl_temperature_status_params_t out_data = {0};
    light_ctl_temperature_status_params_t * p_out_data = NULL;
    light_ctl_temperature_set_params_t in_data = {0};
    model_transition_t transition = {0};
    model_transition_t * p_transition;

    p_server = (light_ctl_temperature_server_t *) p_args;
    p_msg_pkt = (light_ctl_temperature_set_msg_pkt_t *) p_rx_msg->p_data;
    if (!model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data,
                            LIGHT_CTL_OPCODE_TEMPERATURE_SET, p_msg_pkt->tid))
    {
        return;
    }

    p_transition = &transition;
    if (!handle_set_message_validate(LIGHT_CTL_TEMPERATURE_SET_MINLEN, LIGHT_CTL_TEMPERATURE_SET_MAXLEN,
                                     p_rx_msg->length, p_msg_pkt->transition_time,
                                     p_msg_pkt->delay, &p_transition))
    {
        return;
    }

    /* Check for prohibited values @tagMeshMdlSp section 6.1.3.1.  For any prohibited values, don't process,
     * and don't respond */
    if ((p_msg_pkt->temperature < LIGHT_CTL_TEMPERATURE_MIN_LIMIT) ||
        (p_msg_pkt->temperature > LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
    {
        return;
    }

    in_data.temperature32 = light_ctl_utils_temperature_to_temperature32(p_msg_pkt->temperature);
    in_data.delta_uv = p_msg_pkt->delta_uv;
    in_data.tid = p_msg_pkt->tid;

    p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv, p_server);

    in_data.temperature32 = clip_temperature32_to_range(p_s_server, &p_rx_msg->meta_data,
                                                        in_data.temperature32);

    if (LIGHT_CTL_OPCODE_TEMPERATURE_SET == p_rx_msg->opcode.opcode)
    {
        p_out_data = &out_data;
    }

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_set_cb(p_s_server, &p_rx_msg->meta_data,
                                                                   &in_data, p_transition, p_out_data);

    if (p_out_data != NULL)
    {
        (void) status_temperature_send(p_server, p_rx_msg, p_out_data);
    }
}

static void handle_temperature_get(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_ctl_temperature_server_t * p_server = (light_ctl_temperature_server_t *) p_args;
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv,
                                                          p_args);
    light_ctl_temperature_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_get_cb(p_s_server,
                                                                       &p_rx_msg->meta_data,
                                                                       &out_data);
        (void) status_temperature_send(p_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers_temperature[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_GET), handle_temperature_get},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_SET), handle_temperature_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_SET_UNACKNOWLEDGED), handle_temperature_set},
};

/** Callbacks for the model that CTL Temperature instantiated (extended) **/

static void level_state_set_cb(const generic_level_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               const generic_level_set_params_t * p_in,
                               const model_transition_t * p_in_transition,
                               generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_ctl_temperature_server_t * p_server =
        PARENT_BY_FIELD_GET(light_ctl_temperature_server_t, generic_level_srv, p_level_server);
    light_ctl_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv, p_server);
    light_ctl_temperature_set_params_t in_data = {0};
    light_ctl_temperature_status_params_t out_data = {0};
    light_ctl_temperature_status_params_t * p_out_temperature_data;
    uint16_t temp_temperature;
    light_ctl_temperature_status_params_t temperature32_data;
    light_ctl_temperature_range_status_params_t range_data;

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, NULL,
                                                                         &range_data);
    /* Convert the level data to CTL temperature */
    in_data.temperature32 = light_ctl_utils_level_to_temperature32(p_in->level,
                                                             range_data.temperature32_range_min,
                                                             range_data.temperature32_range_max);

    temp_temperature = light_ctl_utils_temperature32_to_temperature(in_data.temperature32);
    /* Check for prohibited values @tagMeshMdlSp section 6.1.3.1.  For any prohibited values, don't process,
     * and don't respond */
    if ((temp_temperature < LIGHT_CTL_TEMPERATURE_MIN_LIMIT) || (temp_temperature > LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
    {
        return;
    }

    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_temperature_data = p_out ? &out_data : NULL;

    /* Clip actual value to temperature range (binding - 6.1.3.1.3) */
    in_data.temperature32 = clip_temperature32_to_range(p_s_server, p_meta, in_data.temperature32);

    /* Get the current delta_uv so we don't accidently change it */
    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_get_cb(p_s_server, NULL,
                                                                   &temperature32_data);

    in_data.delta_uv = temperature32_data.present_delta_uv;

    /* Set clipped temperature value */
    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_set_cb(p_s_server, p_meta, &in_data,
                                                                   p_in_transition,
                                                                   p_out_temperature_data);

    if ((p_out != NULL) && (p_out_temperature_data != NULL))
    {
        p_out->present_level =
            light_ctl_utils_temperature32_to_level(p_out_temperature_data->present_temperature32,
                                             range_data.temperature32_range_min,
                                             range_data.temperature32_range_max);
        p_out->target_level =
            light_ctl_utils_temperature32_to_level(p_out_temperature_data->target_temperature32,
                                             range_data.temperature32_range_min,
                                             range_data.temperature32_range_max);
        p_out->remaining_time_ms = p_out_temperature_data->remaining_time_ms;
    }
}

static void level_state_get_cb(const generic_level_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               generic_level_status_params_t * p_out)
{
    light_ctl_temperature_server_t * p_server = PARENT_BY_FIELD_GET(light_ctl_temperature_server_t,
                                                              generic_level_srv, p_self);
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv,
                                                          p_server);
    light_ctl_temperature_status_params_t out_data = {0};
    light_ctl_temperature_range_status_params_t range_data;

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, NULL,
                                                                         &range_data);

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_get_cb(p_s_server, p_meta, &out_data);

    p_out->present_level = light_ctl_utils_temperature32_to_level(out_data.present_temperature32,
                                                            range_data.temperature32_range_min,
                                                            range_data.temperature32_range_max);

    /* If level move transition is in progress, higher layer will report uint32 limiting values */
    p_out->target_level = out_data.target_temperature32 == UINT32_MAX ? INT16_MAX :
                          out_data.target_temperature32 == 0 ? INT16_MIN :
                          light_ctl_utils_temperature32_to_level(out_data.target_temperature32,
                                                                 range_data.temperature32_range_min,
                                                                 range_data.temperature32_range_max);
    p_out->remaining_time_ms = out_data.remaining_time_ms;
}

static void level_state_delta_set_cb(const generic_level_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     const generic_level_delta_set_params_t * p_in,
                                     const model_transition_t * p_in_transition,
                                     generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_ctl_temperature_server_t * p_server = PARENT_BY_FIELD_GET(light_ctl_temperature_server_t,
                                                              generic_level_srv, p_level_server);
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t,
                                                          ctl_temperature_srv, p_server);
    light_ctl_temperature_delta_set_params_t in_data = {0};
    light_ctl_temperature_status_params_t out_data = {0};
    light_ctl_temperature_status_params_t * p_out_temp_data;
    light_ctl_temperature_range_status_params_t range_data;

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, NULL,
                                                                         &range_data);

    /* The delta level data is just an 32 bit signed offset (to allow it to be positive or
     * negative). Convert it to temperature delta and pass it on to the app level */
    in_data.delta_temperature = light_ctl_utils_level_delta_to_temperature_delta(p_in->delta_level,
                    light_ctl_utils_temperature32_to_temperature(range_data.temperature32_range_min),
                    light_ctl_utils_temperature32_to_temperature(range_data.temperature32_range_max));


    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_temp_data = p_out ? &out_data : NULL;
    p_s_server->settings.p_callbacks->light_ctl_cbs.delta_set_cb(p_s_server, p_meta, &in_data,
                                                           p_in_transition, p_out_temp_data);
    if ((p_out != NULL) && (p_out_temp_data != NULL))
    {
        p_out->present_level =
            light_ctl_utils_temperature32_to_level(p_out_temp_data->present_temperature32,
                                             range_data.temperature32_range_min,
                                             range_data.temperature32_range_max);
        p_out->target_level =
            light_ctl_utils_temperature32_to_level(p_out_temp_data->target_temperature32,
                                             range_data.temperature32_range_min,
                                             range_data.temperature32_range_max);
        p_out->remaining_time_ms =
            p_out_temp_data->remaining_time_ms;
    }
}

static void level_state_move_set_cb(const generic_level_server_t * p_self,
                                    const access_message_rx_meta_t * p_meta,
                                    const generic_level_move_set_params_t * p_in,
                                    const model_transition_t * p_in_transition,
                                    generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_ctl_temperature_server_t * p_server = PARENT_BY_FIELD_GET(light_ctl_temperature_server_t,
                                                              generic_level_srv, p_level_server);
    light_ctl_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_ctl_setup_server_t, ctl_temperature_srv,
                                                          p_server);
    light_ctl_temperature_move_set_params_t in_data = {0};
    light_ctl_temperature_status_params_t out_data = {0};
    light_ctl_temperature_status_params_t * p_out_temp_data;
    light_ctl_temperature_range_status_params_t range_data;

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, NULL,
                                                                         &range_data);

    /* The move level data is just a 16 bit signed offset (to allow it to be positive or
     * negative).  Convert it to temperature delta and pass it on to the app level. */
    in_data.delta_temperature = light_ctl_utils_level_delta_to_temperature_delta(p_in->move_level,
                    light_ctl_utils_temperature32_to_temperature(range_data.temperature32_range_min),
                    light_ctl_utils_temperature32_to_temperature(range_data.temperature32_range_max));

    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_temp_data = p_out ? &out_data : NULL;
    p_s_server->settings.p_callbacks->light_ctl_cbs.move_set_cb(p_s_server, p_meta, &in_data,
                                                          p_in_transition, p_out_temp_data);
    if ((p_out != NULL) && (p_out_temp_data != NULL))
    {
        p_out->present_level =
            light_ctl_utils_temperature32_to_level(p_out_temp_data->present_temperature32,
                                                   range_data.temperature32_range_min,
                                                   range_data.temperature32_range_max);
        /* Set target level as expected in Move Set status, see @tagMeshMdlSp section 3.3.2.2.4 */
        p_out->target_level = p_out_temp_data->target_temperature32 == UINT32_MAX ?
                              INT16_MAX : INT16_MIN;
        p_out->remaining_time_ms = p_out_temp_data->remaining_time_ms;
    }
}

/** Interface functions */
static const generic_level_server_callbacks_t m_level_srv_cbs =
{
    .level_cbs.get_cb = level_state_get_cb,
    .level_cbs.set_cb = level_state_set_cb,
    .level_cbs.delta_set_cb = level_state_delta_set_cb,
    .level_cbs.move_set_cb = level_state_move_set_cb
};

/* When Temperature has changed in the Light CTL state, when we publish that, we also need to
 * publish level */
static void ctl_temperature_bound_state_publish(const light_ctl_setup_server_t * p_s_server,
                                                const ctl_temperature_params_t * p_params)
{
    generic_level_status_params_t pub_params;
    generic_level_server_t * p_level_server;
    light_ctl_temperature_range_status_params_t range_data;

    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb(p_s_server, NULL,
                                                                         &range_data);

    pub_params.present_level = light_ctl_utils_temperature32_to_level(p_params->present_temperature32,
                                                                range_data.temperature32_range_min,
                                                                range_data.temperature32_range_max);
    pub_params.target_level = light_ctl_utils_temperature32_to_level(p_params->target_temperature32,
                                                               range_data.temperature32_range_min,
                                                               range_data.temperature32_range_max);
    pub_params.remaining_time_ms = p_params->remaining_time_ms;

    p_level_server = (generic_level_server_t *) &p_s_server->ctl_temperature_srv.generic_level_srv;

    /* Send out a level status */
    /* Ignore status, as publish may fail due to several reasons and it is ok. */
    (void) generic_level_server_status_publish(p_level_server, &pub_params);
}

static uint32_t ctl_temperature_server_init(light_ctl_temperature_server_t * p_server,
                                            uint8_t element_index)
{
    uint32_t status;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize parent model instance - Generic Level */
    p_server->generic_level_srv.settings.p_callbacks = &m_level_srv_cbs;

    status = generic_level_server_init(&p_server->generic_level_srv, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Add this CTL Temperature server model to the mesh */
    access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers_temperature[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers_temperature),
            .p_args = p_server,
            .publish_timeout_cb = periodic_temperature_publish_cb
        };

    status = access_model_add(&init_params, &p_server->model_handle);

    /* The CTL Temperature Server sets up its subscription list, then shares it with its extended
     * level model */
    status = access_model_subscription_list_alloc(p_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Deallocate the level model's subscription list so we can share with it */
    status = access_model_subscription_list_dealloc(p_server->generic_level_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Share the subscription list with the level model */
    status = access_model_subscription_lists_share(p_server->model_handle,
                                                   p_server->generic_level_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    return status;
}

/**************************************************************************************************/
/**** CTL Setup Server */
/** Opcode Handlers */

static void handle_default_set(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    const light_ctl_setup_server_t * p_s_server = (light_ctl_setup_server_t *) p_args;
    light_ctl_default_set_msg_pkt_t * p_msg_pkt;
    light_ctl_default_set_params_t in_data = {0};
    light_ctl_default_status_params_t out_data = {0};
    light_ctl_default_status_params_t * p_out_data = NULL;

    if (sizeof(light_ctl_default_set_msg_pkt_t) == p_rx_msg->length)
    {
        p_msg_pkt = (light_ctl_default_set_msg_pkt_t *) p_rx_msg->p_data;

        /* Check for prohibited values @tagMeshMdlSp section 6.1.3.1.  For any prohibited values, don't process,
         * and don't respond */
        if ((p_msg_pkt->temperature < LIGHT_CTL_TEMPERATURE_MIN_LIMIT) ||
            (p_msg_pkt->temperature > LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
        {
            return;
        }

        in_data.lightness = p_msg_pkt->lightness;
        in_data.temperature32 = light_ctl_utils_temperature_to_temperature32(p_msg_pkt->temperature);
        in_data.delta_uv = p_msg_pkt->delta_uv;

        if (LIGHT_CTL_OPCODE_DEFAULT_SET == p_rx_msg->opcode.opcode)
        {
            p_out_data = &out_data;
        }

        p_s_server->settings.p_callbacks->light_ctl_cbs.default_set_cb(p_s_server, &p_rx_msg->meta_data,
                                                                 &in_data, p_out_data);
        if (p_out_data != NULL)
        {
            (void) status_default_send(p_s_server, CTL_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static void handle_temperature_range_set(access_model_handle_t model_handle,
                                         const access_message_rx_t * p_rx_msg,
                                         void * p_args)
{
    const light_ctl_setup_server_t * p_s_server = (light_ctl_setup_server_t *) p_args;
    light_ctl_temperature_range_set_msg_pkt_t * p_msg_pkt;
    light_ctl_temperature_range_set_params_t in_data = {0};
    light_ctl_temperature_range_status_params_t out_data = {0};
    light_ctl_temperature_range_status_params_t * p_out_data = NULL;

    if (sizeof(light_ctl_temperature_range_set_msg_pkt_t) == p_rx_msg->length)
    {
        p_msg_pkt = (light_ctl_temperature_range_set_msg_pkt_t *) p_rx_msg->p_data;

        /* Check for prohibited values @tagMeshMdlSp section 6.1.3.3.  For any prohibited values, don't
         * process, and don't respond */
        if ((p_msg_pkt->range_min < LIGHT_CTL_TEMPERATURE_MIN_LIMIT) ||
            (p_msg_pkt->range_max > LIGHT_CTL_TEMPERATURE_MAX_LIMIT) ||
            (p_msg_pkt->range_min > p_msg_pkt->range_max))
        {
            return;
        }

        in_data.temperature32_range_min = light_ctl_utils_temperature_to_temperature32(p_msg_pkt->range_min);
        in_data.temperature32_range_max = light_ctl_utils_temperature_to_temperature32(p_msg_pkt->range_max);


        if (LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET == p_rx_msg->opcode.opcode)
        {
            p_out_data = &out_data;
        }

        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_set_cb(p_s_server,
                                                                             &p_rx_msg->meta_data,
                                                                             &in_data, p_out_data);

        if (p_out_data != NULL)
        {
            (void) status_temperature_range_send(p_s_server, CTL_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_DEFAULT_SET), handle_default_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_DEFAULT_SET_UNACKNOWLEDGED), handle_default_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET), handle_temperature_range_set},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED), handle_temperature_range_set},
};

uint32_t light_ctl_setup_server_init(light_ctl_setup_server_t * p_s_server,
                                     light_lightness_setup_server_t * p_ll_s_server,
                                     uint8_t element_index)
{
    uint32_t status;

    if (m_total_ctl_ss_instances >= LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX)
    {
        return NRF_ERROR_NO_MEM;
    }

    if (p_s_server == NULL ||
        p_s_server->settings.p_callbacks == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.default_set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.get_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_get_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_range_get_cb == NULL ||
        p_s_server->settings.p_callbacks->light_ctl_cbs.default_get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_s_server->settings.element_index = element_index;

    /* Initialize parent model instance - CTL server */
    status = ctl_server_init(&p_s_server->ctl_srv, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize model instance - CTL temperature server on the next higher element */
    status = ctl_temperature_server_init(&p_s_server->ctl_temperature_srv, element_index + 1);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Add this CTL setup server model to the mesh */
    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(LIGHT_CTL_SETUP_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers_setup[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers_setup),
        .p_args = p_s_server,
    };

    status = access_model_add(&init_params, &p_s_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* The CTL Setup Server sets up its subscription list, then shares it with each of its extended
     * models */
    status = access_model_subscription_list_alloc(p_s_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Share the subscription list with the CTL Server (no deallocation needed, since the CTL server
     * doesn't do an alloc). */
    status = access_model_subscription_lists_share(p_s_server->model_handle,
                                                   p_s_server->ctl_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Note that we don't share the subscription list with the temperature model - it's on a
     * different element */

    /* The Light Lightness instance shares CTL setup server's subscription list */
    status = access_model_subscription_list_dealloc(p_ll_s_server->model_handle);
    if (NRF_SUCCESS == status)
    {
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->generic_ponoff_setup_srv.model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.generic_onoff_srv.model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->generic_ponoff_setup_srv.generic_dtt_srv.model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->light_lightness_srv.model_handle));
        NRF_MESH_ERROR_CHECK(access_model_subscription_lists_share(p_s_server->model_handle,
                                                                   p_ll_s_server->light_lightness_srv.generic_level_srv.model_handle));

        m_total_ctl_ss_instances++;
    }

    return status;
}

/* @tagMeshMdlSp sections 6.1.3.1.2, 6.1.3.4.1 - Binding with the Generic OnPowerUp state */
uint32_t light_ctl_ponoff_binding_setup(light_ctl_setup_server_t * p_s_server, light_ctl_saved_values_t * p_saved_states)
{
    if (p_s_server == NULL || p_saved_states == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Do the appropriate bindings, and set the states */
    light_ctl_temperature_set_params_t ctl_params = {0};

    /* 6.1.2.2.4 Binding with the Generic OnPowerUp state */
    if ((p_saved_states->onpowerup == GENERIC_ON_POWERUP_OFF) ||
        (p_saved_states->onpowerup == GENERIC_ON_POWERUP_DEFAULT))
    {
        ctl_params.temperature32 = p_saved_states->default_temperature32;
        ctl_params.delta_uv = p_saved_states->default_delta_uv;
    }
    else if (p_saved_states->onpowerup == GENERIC_ON_POWERUP_RESTORE)
    {
        /* Restore the last known values */
        ctl_params.temperature32 = p_saved_states->temperature32;
        ctl_params.delta_uv = p_saved_states->delta_uv;
    }
    else
    {
        return NRF_ERROR_INVALID_DATA;
    }

    /* Before setting the temperature value, make sure it conforms to the range. (binding -
     * 6.1.3.1.3) */
    ctl_params.temperature32 =
        clip_temperature32_to_range(p_s_server, NULL, ctl_params.temperature32);

    /* Set temperature (we can't use "set_cb" since we don't control lightness at powerup) */
    p_s_server->settings.p_callbacks->light_ctl_cbs.temperature32_set_cb(p_s_server, NULL, &ctl_params,
                                                                   NULL, NULL);

    p_s_server->state.initialized = true;

    return NRF_SUCCESS;
}
