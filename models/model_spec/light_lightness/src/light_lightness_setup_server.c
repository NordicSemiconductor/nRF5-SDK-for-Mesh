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

#include "light_lightness_setup_server.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "utils.h"

#include "light_lightness_messages.h"
#include "light_lightness_utils.h"

#include "mesh_config_entry.h"
#include "mesh_config.h"

#include "access.h"
#include "access_config.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"
#include "generic_dtt_server.h"

/* Each element can host only one light lightness instance.*/
STATIC_ASSERT(LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX <= ACCESS_ELEMENT_COUNT,
              "LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX is greater than allowed.");

typedef enum
{
    LIGHT_LIGHTNESS_SERVER,
    LIGHT_LIGHTNESS_SETUP_SERVER,
} server_context_type_t;

/** Total number of Light Lightness Setup server instances that can be created. */
static uint32_t m_total_ll_instances;

typedef void (*lightness_set_response_t)(light_lightness_server_t * p_server,
                                         const access_message_rx_t * p_rx_msg,
                                         light_lightness_status_params_t * p_out_data);

static uint32_t status_actual_send(const light_lightness_server_t * p_server,
                                   const access_message_rx_t * p_message,
                                   const light_lightness_status_params_t * p_params)
{
    light_lightness_status_msg_pkt_t msg_pkt;

    msg_pkt.present_lightness = p_params->present_lightness;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_lightness = p_params->target_lightness;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   LIGHT_LIGHTNESS_STATUS_MAXLEN :
                   LIGHT_LIGHTNESS_STATUS_MINLEN),
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


static uint32_t status_linear_send(light_lightness_server_t * p_server,
                                   const access_message_rx_t * p_message,
                                   const light_lightness_linear_status_params_t * p_params)
{
    light_lightness_linear_status_msg_pkt_t msg_pkt;

    msg_pkt.present_lightness = p_params->present_lightness;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_lightness = p_params->target_lightness;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LINEAR_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   LIGHT_LIGHTNESS_LINEAR_STATUS_MAXLEN :
                   LIGHT_LIGHTNESS_LINEAR_STATUS_MINLEN),
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

static uint32_t status_last_send(light_lightness_server_t * p_server,
                                 const access_message_rx_t * p_message,
                                 const light_lightness_last_status_params_t * p_params)
{
    light_lightness_last_status_msg_pkt_t msg_pkt;

    if (p_params->lightness < LIGHT_LIGHTNESS_LAST_MIN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    msg_pkt.lightness = p_params->lightness;
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LAST_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_LIGHTNESS_LAST_STATUS_LEN,
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

static uint32_t status_default_send(const void * p_ctx,
                                    server_context_type_t ctx_type,
                                    const access_message_rx_t * p_message,
                                    const light_lightness_default_status_params_t * p_params)
{
    light_lightness_default_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (ctx_type == LIGHT_LIGHTNESS_SERVER)
    {
        light_lightness_server_t * p_server = (light_lightness_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        light_lightness_setup_server_t * p_s_server = (light_lightness_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    msg_pkt.lightness = p_params->lightness;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_DEFAULT_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_LIGHTNESS_DEFAULT_STATUS_LEN,
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

static uint32_t status_range_send(const void * p_ctx,
                                  server_context_type_t ctx_type,
                                  const access_message_rx_t * p_message,
                                  const light_lightness_range_status_params_t * p_params)
{
    light_lightness_range_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (p_params->range_min < LIGHT_LIGHTNESS_RANGE_MIN ||
        p_params->range_max < LIGHT_LIGHTNESS_RANGE_MIN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (ctx_type == LIGHT_LIGHTNESS_SERVER)
    {
        light_lightness_server_t * p_server = (light_lightness_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        light_lightness_setup_server_t * p_s_server = (light_lightness_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    msg_pkt.status    = p_params->status;
    msg_pkt.range_min = p_params->range_min;
    msg_pkt.range_max = p_params->range_max;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_RANGE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_LIGHTNESS_RANGE_STATUS_LEN,
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
    light_lightness_server_t * p_server = (light_lightness_server_t *)p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb(p_s_server,
                                                                 NULL, &out_data);

    (void) status_actual_send(p_server, NULL, &out_data);
}

/* Utility functions */
static uint16_t clip_ll_actual_to_range(const light_lightness_setup_server_t * p_s_server,
                                        const access_message_rx_meta_t * p_meta,
                                        uint16_t ll_actual)
{
    light_lightness_range_status_params_t range = {.status = 0};

    p_s_server->settings.p_callbacks->light_lightness_cbs.range_get_cb(p_s_server,
                                                                       p_meta,
                                                                       &range);

    ll_actual = light_lightness_utils_actual_to_range_restrict(ll_actual,
                                                         range.range_min,
                                                         range.range_max);

    return ll_actual;
}

static void actual_set_response(light_lightness_server_t * p_server,
                               const access_message_rx_t * p_rx_msg,
                               light_lightness_status_params_t * p_out_data)
{
    if (p_out_data)
    {
        (void) status_actual_send(p_server, p_rx_msg, p_out_data);
    }
}

static void linear_set_response(light_lightness_server_t * p_server,
                               const access_message_rx_t * p_rx_msg,
                               light_lightness_status_params_t * p_out_data)
{
    light_lightness_linear_status_params_t out_linear_data = {0};

    if (p_out_data)
    {
        out_linear_data.present_lightness =
            light_lightness_utils_actual_to_linear(p_out_data->present_lightness);
        out_linear_data.target_lightness =
            light_lightness_utils_actual_to_linear(p_out_data->target_lightness);
        out_linear_data.remaining_time_ms = p_out_data->remaining_time_ms;
        (void) status_linear_send(p_server, p_rx_msg, &out_linear_data);
    }
}

static void handle_set(light_lightness_server_t * p_server,
                       const access_message_rx_t * p_rx_msg,
                       light_lightness_set_params_t * p_in_data,
                       light_lightness_status_params_t * p_out_data,
                       model_transition_t * p_transition,
                       lightness_set_response_t response)
{
    light_lightness_setup_server_t * p_s_server;

    p_s_server = PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                                     light_lightness_srv,
                                     p_server);

    /* Clip the actual value to range (binding - 6.1.2.2.5) */
    p_in_data->lightness = clip_ll_actual_to_range(p_s_server,
                                                   &p_rx_msg->meta_data,
                                                   p_in_data->lightness);

    /* Set clipped actual value */
    p_s_server->settings.p_callbacks->light_lightness_cbs.set_cb(p_s_server,
                                                                 &p_rx_msg->meta_data,
                                                                 p_in_data,
                                                                 p_transition,
                                                                 p_out_data);
    response(p_server, p_rx_msg, p_out_data);
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
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid parameter (%d) = (transition_time)\n", transition_time);
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
/**** Light Lightness server ****/
/** Opcode Handlers */
static void handle_actual_set(access_model_handle_t model_handle,
                              const access_message_rx_t * p_rx_msg,
                              void * p_args)
{
    light_lightness_server_t * p_server;
    light_lightness_set_msg_pkt_t * p_msg_pkt;
    light_lightness_status_params_t out_data = {0};
    light_lightness_set_params_t in_data = {0};
    model_transition_t transition = {0};
    model_transition_t * p_transition;

    p_server = (light_lightness_server_t *) p_args;
    p_msg_pkt = (light_lightness_set_msg_pkt_t *) p_rx_msg->p_data;
    if (!model_tid_validate(&p_server->tid_tracker,
                            &p_rx_msg->meta_data,
                            LIGHT_LIGHTNESS_OPCODE_SET,
                            p_msg_pkt->tid))
    {
        return;
    }

    p_transition = &transition;
    if (!handle_set_message_validate(LIGHT_LIGHTNESS_SET_MINLEN,
                                     LIGHT_LIGHTNESS_SET_MAXLEN,
                                     p_rx_msg->length,
                                     p_msg_pkt->transition_time,
                                     p_msg_pkt->delay,
                                     &p_transition))
    {
        return;
    }

    in_data.lightness = p_msg_pkt->lightness;
    in_data.tid = p_msg_pkt->tid;

    handle_set(p_server,
               p_rx_msg,
               &in_data,
               (LIGHT_LIGHTNESS_OPCODE_SET == p_rx_msg->opcode.opcode) ? &out_data : NULL,
               p_transition,
               actual_set_response);
}

static void handle_linear_set(access_model_handle_t model_handle,
                              const access_message_rx_t * p_rx_msg,
                              void * p_args)
{
    light_lightness_server_t * p_server;
    light_lightness_linear_set_msg_pkt_t * p_msg_pkt;
    light_lightness_status_params_t out_data = {0};
    light_lightness_set_params_t in_data = {0};
    model_transition_t transition = {0};
    model_transition_t * p_transition;

    p_server = (light_lightness_server_t *) p_args;
    p_msg_pkt = (light_lightness_linear_set_msg_pkt_t *) p_rx_msg->p_data;
    if (!model_tid_validate(&p_server->tid_tracker,
                            &p_rx_msg->meta_data,
                            LIGHT_LIGHTNESS_OPCODE_SET,
                            p_msg_pkt->tid))
    {
        return;
    }

    p_transition = &transition;
    if (!handle_set_message_validate(LIGHT_LIGHTNESS_LINEAR_SET_MINLEN,
                                     LIGHT_LIGHTNESS_LINEAR_SET_MAXLEN,
                                     p_rx_msg->length,
                                     p_msg_pkt->transition_time,
                                     p_msg_pkt->delay,
                                     &p_transition))
    {
        return;
    }

    in_data.lightness = light_lightness_utils_linear_to_actual(p_msg_pkt->lightness);
    in_data.tid = p_msg_pkt->tid;

    handle_set(p_server,
               p_rx_msg,
               &in_data,
               (LIGHT_LIGHTNESS_OPCODE_LINEAR_SET == p_rx_msg->opcode.opcode) ? &out_data : NULL,
               p_transition,
               linear_set_response);
}

static void handle_get(access_model_handle_t model_handle,
                       const access_message_rx_t * p_rx_msg,
                       void * p_args)
{
    light_lightness_server_t * p_server = (light_lightness_server_t *) p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb(p_s_server,
                                                                     &p_rx_msg->meta_data,
                                                                     &out_data);
        (void) status_actual_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_linear_get(access_model_handle_t model_handle,
                              const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_lightness_server_t * p_server = (light_lightness_server_t *) p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_status_params_t out_data = {0};
    light_lightness_linear_status_params_t out_linear_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb(p_s_server,
                                                                     &p_rx_msg->meta_data,
                                                                     &out_data);
        out_linear_data.present_lightness =
            light_lightness_utils_actual_to_linear(out_data.present_lightness);
        out_linear_data.target_lightness =
            light_lightness_utils_actual_to_linear(out_data.target_lightness);
        out_linear_data.remaining_time_ms = out_data.remaining_time_ms;
        (void) status_linear_send(p_server, p_rx_msg, &out_linear_data);
    }
}

static void handle_last_get(access_model_handle_t model_handle,
                            const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_lightness_server_t * p_server = (light_lightness_server_t *) p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_last_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_lightness_cbs.last_get_cb(p_s_server,
                                                                          &p_rx_msg->meta_data,
                                                                          &out_data);
        (void) status_last_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_default_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_lightness_server_t * p_server = (light_lightness_server_t *) p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_default_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_lightness_cbs.default_get_cb(p_s_server,
                                                                             &p_rx_msg->meta_data,
                                                                             &out_data);

        (void) status_default_send(p_server, LIGHT_LIGHTNESS_SERVER, p_rx_msg, &out_data);

    }
}

static void handle_range_get(access_model_handle_t model_handle,
                             const access_message_rx_t * p_rx_msg,
                             void * p_args)
{
    light_lightness_server_t * p_server = (light_lightness_server_t *) p_args;
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_args);
    light_lightness_range_status_params_t out_data = {.status = 0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->light_lightness_cbs.range_get_cb(p_s_server,
                                                                           &p_rx_msg->meta_data,
                                                                           &out_data);

        (void) status_range_send(p_server, LIGHT_LIGHTNESS_SERVER,
                                 p_rx_msg, &out_data);

    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_SET), handle_actual_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_SET_UNACKNOWLEDGED), handle_actual_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LINEAR_GET), handle_linear_get},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LINEAR_SET), handle_linear_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LINEAR_SET_UNACKNOWLEDGED), handle_linear_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_LAST_GET), handle_last_get},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_DEFAULT_GET), handle_default_get},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_RANGE_GET), handle_range_get},
};

/** Callbacks for models that Light Lightness instantiated (extended) **/

static void level_state_set_cb(const generic_level_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               const generic_level_set_params_t * p_in,
                               const model_transition_t * p_in_transition,
                               generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_lightness_server_t * p_server =
        PARENT_BY_FIELD_GET(light_lightness_server_t,
                            generic_level_srv, p_level_server);
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_server);
    light_lightness_set_params_t in_data = {0};
    light_lightness_status_params_t out_data = {0};
    light_lightness_status_params_t * p_out_ll_data;

    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_ll_data = p_out ? &out_data : NULL;

    /* Convert the level data to light lightness actual */
    in_data.lightness = light_lightness_utils_generic_level_to_actual(p_in->level);

    /* Clip actual value to range (binding - 6.1.2.2.5) */
    in_data.lightness = clip_ll_actual_to_range(p_s_server,
                                                p_meta,
                                                in_data.lightness);
    /* Set clipped actual value */
    p_s_server->settings.p_callbacks->light_lightness_cbs.set_cb(p_s_server,
                                                                 p_meta,
                                                                 &in_data,
                                                                 p_in_transition,
                                                                 p_out_ll_data);
    if (p_out != NULL && p_out_ll_data != NULL)
    {
        p_out->present_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->present_lightness);
        p_out->target_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->target_lightness);
        p_out->remaining_time_ms = p_out_ll_data->remaining_time_ms;
    }
}

static void level_state_get_cb(const generic_level_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               generic_level_status_params_t * p_out)
{
    light_lightness_server_t * p_server =
        PARENT_BY_FIELD_GET(light_lightness_server_t,
                            generic_level_srv, p_self);
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_server);
    light_lightness_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb(p_s_server,
                                                                 p_meta,
                                                                 &out_data);

    p_out->present_level = light_lightness_utils_actual_to_generic_level(out_data.present_lightness);
    p_out->target_level = light_lightness_utils_actual_to_generic_level(out_data.target_lightness);
    p_out->remaining_time_ms = out_data.remaining_time_ms;
}

static void level_state_delta_set_cb(const generic_level_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     const generic_level_delta_set_params_t * p_in,
                                     const model_transition_t * p_in_transition,
                                     generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_lightness_server_t * p_server =
        PARENT_BY_FIELD_GET(light_lightness_server_t,
                            generic_level_srv, p_level_server);
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_server);
    light_lightness_delta_set_params_t in_data = {0};
    light_lightness_status_params_t out_data = {0};
    light_lightness_status_params_t * p_out_ll_data;

    /* The delta level data is just an 32 bit signed offset (to allow
     * it to be positive or negative). Pass it on to the app level,
     * and it will wrap as needed */
    in_data.delta_lightness = p_in->delta_level;

    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_ll_data = p_out ? &out_data : NULL;
    p_s_server->settings.p_callbacks->light_lightness_cbs.delta_set_cb(p_s_server,
                                                                       p_meta,
                                                                       &in_data,
                                                                       p_in_transition,
                                                                       p_out_ll_data);
    if (p_out != NULL && p_out_ll_data != NULL)
    {
        p_out->present_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->present_lightness);
        p_out->target_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->target_lightness);
        p_out->remaining_time_ms =
            p_out_ll_data->remaining_time_ms;
    }
}

static void level_state_move_set_cb(const generic_level_server_t * p_self,
                                    const access_message_rx_meta_t * p_meta,
                                    const generic_level_move_set_params_t * p_in,
                                    const model_transition_t * p_in_transition,
                                    generic_level_status_params_t * p_out)
{
    generic_level_server_t * p_level_server  = (generic_level_server_t *) p_self;
    light_lightness_server_t * p_server =
        PARENT_BY_FIELD_GET(light_lightness_server_t,
                            generic_level_srv, p_level_server);
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            light_lightness_srv, p_server);
    light_lightness_move_set_params_t in_data = {0};
    light_lightness_status_params_t out_data = {0};
    light_lightness_status_params_t * p_out_ll_data;

    /* The move level data is just a 16 bit signed offset (to allow
     * it to be positive or negative). Pass it on to the app level,
     * and it will wrap as needed */
    in_data.delta = p_in->move_level;

    /* TID already validated by level model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_server->tid_tracker = p_level_server->tid_tracker;

    p_out_ll_data = p_out ? &out_data : NULL;
    p_s_server->settings.p_callbacks->light_lightness_cbs.move_set_cb(p_s_server,
                                                                      p_meta,
                                                                      &in_data,
                                                                      p_in_transition,
                                                                      p_out_ll_data);
    if (p_out != NULL && p_out_ll_data != NULL)
    {
        p_out->present_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->present_lightness);
        p_out->target_level =
            light_lightness_utils_actual_to_generic_level(p_out_ll_data->target_lightness);
        p_out->remaining_time_ms = p_out_ll_data->remaining_time_ms;
    }
}

static void onoff_set_cb(const generic_onoff_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         const generic_onoff_set_params_t * p_in,
                         const model_transition_t * p_in_transition,
                         generic_onoff_status_params_t * p_out)
{
    generic_onoff_server_t * p_onoff_server = (generic_onoff_server_t *) p_self;
    generic_ponoff_server_t * p_ponoff_server =
        PARENT_BY_FIELD_GET(generic_ponoff_server_t,
                            generic_onoff_srv, p_onoff_server);
    generic_ponoff_setup_server_t * p_ponoff_s_server =
        PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t,
                            generic_ponoff_srv, p_ponoff_server);
    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            generic_ponoff_setup_srv, p_ponoff_s_server);
    light_lightness_set_params_t in_data = {0};
    light_lightness_status_params_t out_data = {0};
    light_lightness_status_params_t * p_out_ll_data;
    light_lightness_last_status_params_t last_data = {0};
    light_lightness_default_status_params_t default_data = {0};
    model_transition_t * p_ll_transition = NULL;
    model_transition_t ll_transition;

    /* TID already validated by on/off model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_s_server->light_lightness_srv.tid_tracker = p_onoff_server->tid_tracker;

    p_out_ll_data = p_out ? &out_data : NULL;

    if (p_in->on_off == true)
    {
        /* 6.1.2.2.3 Bind Generic on/off to light lightness default
         * (if non-0) otherwise bind to light lightness actual. */

        /* First get the "default" state value */
        p_s_server->settings.p_callbacks->light_lightness_cbs.default_get_cb(p_s_server,
                                                                             p_meta,
                                                                             &default_data);
        if (default_data.lightness != 0)
        {
            in_data.lightness = default_data.lightness;
        }
        else
        {
            /* Since Default is 0, get the "last" state value */
            p_s_server->settings.p_callbacks->light_lightness_cbs.last_get_cb(p_s_server,
                                                                              p_meta,
                                                                              &last_data);
            in_data.lightness = last_data.lightness;
        }
    }
    else
    {
        /* The command is to turn "off", so set lightness to a 0 */
        in_data.lightness = 0;
    }

    if (p_in_transition != NULL)
    {
        p_ll_transition = &ll_transition;
        ll_transition.delay_ms = p_in_transition->delay_ms;
        ll_transition.transition_time_ms = p_in_transition->transition_time_ms;
    }

    /* Clip actual value to range (binding - 6.1.2.2.5) */

    in_data.lightness = clip_ll_actual_to_range(p_s_server,
                                                p_meta,
                                                in_data.lightness);
    /* Set clipped actual value */
    p_s_server->settings.p_callbacks->light_lightness_cbs.set_cb(p_s_server,
                                                                 p_meta,
                                                                 &in_data,
                                                                 p_ll_transition,
                                                                 p_out_ll_data);

    if (p_out != NULL && p_out_ll_data != NULL)
    {
        p_out->present_on_off =
            light_lightness_utils_actual_to_generic_onoff(p_out_ll_data->present_lightness);
        p_out->target_on_off =
            light_lightness_utils_actual_to_generic_onoff(p_out_ll_data->target_lightness);
        p_out->remaining_time_ms = p_out_ll_data->remaining_time_ms;
    }
}

static void onoff_get_cb(const generic_onoff_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         generic_onoff_status_params_t * p_out)
{
    generic_ponoff_server_t * p_ponoff_server =
        PARENT_BY_FIELD_GET(generic_ponoff_server_t,
                            generic_onoff_srv, p_self);

    generic_ponoff_setup_server_t * p_ponoff_s_server =
        PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t,
                            generic_ponoff_srv, p_ponoff_server);

    light_lightness_setup_server_t * p_s_server =
        PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                            generic_ponoff_setup_srv, p_ponoff_s_server);
    light_lightness_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb(p_s_server,
                                                                 p_meta,
                                                                 &out_data);
    p_out->present_on_off =
        light_lightness_utils_actual_to_generic_onoff(out_data.present_lightness);
    p_out->target_on_off =
        light_lightness_utils_actual_to_generic_onoff(out_data.target_lightness);
    p_out->remaining_time_ms = out_data.remaining_time_ms;
}

/** Interface functions */
static const generic_level_server_callbacks_t m_level_srv_cbs =
{
    .level_cbs.get_cb = level_state_get_cb,
    .level_cbs.set_cb = level_state_set_cb,
    .level_cbs.delta_set_cb = level_state_delta_set_cb,
    .level_cbs.move_set_cb = level_state_move_set_cb
};

/* When a linear or actual state has changed, and we're publishing
 * that, we also need to publish level and on/off */
static void publish_bound_states(light_lightness_server_t * p_ll_server,
                                 const generic_level_status_params_t * p_level_params)
{
    generic_onoff_server_t * p_onoff_server;
    generic_onoff_status_params_t onoff_params;
    light_lightness_setup_server_t * p_ll_s_server;
    int16_t level;

    generic_level_server_t * p_level_server;

    /* Send out a level status */
    p_level_server = &(p_ll_server->generic_level_srv);

    (void) generic_level_server_status_publish(p_level_server, p_level_params);

    p_ll_s_server = PARENT_BY_FIELD_GET(light_lightness_setup_server_t,
                                        light_lightness_srv, p_ll_server);

    /* Send out a generic on/off status */
    p_onoff_server =
        &(p_ll_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.generic_onoff_srv);

    level = p_level_params->present_level;
    onoff_params.present_on_off = (level > INT16_MIN)?1:0;
    level = p_level_params->target_level;
    onoff_params.target_on_off = (level > INT16_MIN)?1:0;
    onoff_params.remaining_time_ms = p_level_params->remaining_time_ms;

    (void) generic_onoff_server_status_publish(p_onoff_server, &onoff_params);
}

static void publish_bound_actual_states(const light_lightness_server_t * p_server,
                                        const light_lightness_status_params_t * p_params)
{
    generic_level_status_params_t pub_params;

    pub_params.present_level =
        light_lightness_utils_actual_to_generic_level(p_params->present_lightness);
    pub_params.target_level =
        light_lightness_utils_actual_to_generic_level(p_params->target_lightness);
    pub_params.remaining_time_ms = p_params->remaining_time_ms;

    publish_bound_states((light_lightness_server_t *) p_server, &pub_params);
}

static void publish_bound_linear_states(const light_lightness_server_t * p_server,
                                        const light_lightness_linear_status_params_t * p_params)
{
    generic_level_status_params_t pub_params;
    uint16_t actual_val;

    /* Convert eaach linear val to actual, then to level */
    actual_val               = light_lightness_utils_linear_to_actual(p_params->present_lightness);
    pub_params.present_level = light_lightness_utils_actual_to_generic_level(actual_val);
    actual_val               = light_lightness_utils_linear_to_actual(p_params->target_lightness);
    pub_params.target_level  = light_lightness_utils_actual_to_generic_level(actual_val);

    pub_params.remaining_time_ms = p_params->remaining_time_ms;

    publish_bound_states((light_lightness_server_t *) p_server, &pub_params);
}

static uint32_t light_lightness_server_init(light_lightness_server_t * p_server,
                                            uint8_t element_index)
{
    uint32_t status;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize parent model instances - Generic Level */
    p_server->generic_level_srv.settings.p_callbacks = &m_level_srv_cbs;

    status = generic_level_server_init(&p_server->generic_level_srv, element_index);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_dealloc(p_server->generic_level_srv.model_handle);
    }

    /* Add this light lightness server model to the mesh */
    if (status == NRF_SUCCESS)
    {

        access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(LIGHT_LIGHTNESS_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers),
            .p_args = p_server,
            .publish_timeout_cb = periodic_publish_cb
        };

        status = access_model_add(&init_params, &p_server->model_handle);
    }

    return status;
}

uint32_t light_lightness_server_status_publish(const light_lightness_server_t * p_server,
                                               const light_lightness_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    publish_bound_actual_states(p_server, p_params);

    return status_actual_send(p_server, NULL, p_params);
}

uint32_t light_lightness_server_linear_status_publish(light_lightness_server_t * p_server,
                                                      const light_lightness_linear_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    publish_bound_linear_states(p_server, p_params);

    return status_linear_send(p_server, NULL, p_params);
}

uint32_t light_lightness_server_last_status_publish(light_lightness_server_t * p_server,
                                                    const light_lightness_last_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_last_send(p_server, NULL, p_params);
}

uint32_t light_lightness_server_default_status_publish(const light_lightness_server_t * p_server,
                                                       const light_lightness_default_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_default_send(p_server, LIGHT_LIGHTNESS_SERVER, NULL, p_params);
}

uint32_t light_lightness_server_range_status_publish(const light_lightness_server_t * p_server,
                                                     const light_lightness_range_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_range_send(p_server, LIGHT_LIGHTNESS_SERVER, NULL, p_params);
}

/**************************************************************************************************/
/**** Setup Server */
/** Opcode Handlers */

static void handle_default_set(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    const light_lightness_setup_server_t * p_s_server =
        (light_lightness_setup_server_t *) p_args;
    light_lightness_default_set_params_t in_data = {0};
    light_lightness_default_status_params_t out_data = {0};
    light_lightness_default_status_params_t * p_out_data = NULL;

    if (sizeof(light_lightness_default_set_msg_pkt_t) == p_rx_msg->length)
    {
        light_lightness_default_set_msg_pkt_t * p_msg_params_packed =
            (light_lightness_default_set_msg_pkt_t *) p_rx_msg->p_data;
        in_data.lightness = p_msg_params_packed->lightness;

        if (LIGHT_LIGHTNESS_OPCODE_DEFAULT_SET == p_rx_msg->opcode.opcode)
        {
            p_out_data = &out_data;
        }

        p_s_server->settings.p_callbacks->light_lightness_cbs.default_set_cb(p_s_server,
                                                                             &p_rx_msg->meta_data,
                                                                             &in_data,
                                                                             p_out_data);
        if (p_out_data)
        {
            (void) status_default_send(p_s_server, LIGHT_LIGHTNESS_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static void handle_range_set(access_model_handle_t model_handle,
                             const access_message_rx_t * p_rx_msg,
                             void * p_args)
{
    const light_lightness_setup_server_t * p_s_server = (light_lightness_setup_server_t *) p_args;
    light_lightness_range_set_params_t in_data = {0};
    light_lightness_range_status_params_t out_data = {.status = 0};
    light_lightness_range_status_params_t * p_out_data = NULL;

    if (sizeof(light_lightness_range_set_msg_pkt_t) == p_rx_msg->length)
    {
        light_lightness_range_set_msg_pkt_t * p_msg_params_packed =
            (light_lightness_range_set_msg_pkt_t *) p_rx_msg->p_data;
        in_data.range_min = p_msg_params_packed->range_min;
        in_data.range_max = p_msg_params_packed->range_max;

        /* Check for prohibited values @tagMeshMdlSp section 6.1.2.5.  For any
         * prohibited values, don't process, and don't respond */
        if ((in_data.range_min == 0) ||
            (in_data.range_min > in_data.range_max))
        {
            return;
        }

        if (LIGHT_LIGHTNESS_OPCODE_RANGE_SET == p_rx_msg->opcode.opcode)
        {
            p_out_data = &out_data;
        }

        p_s_server->settings.p_callbacks->light_lightness_cbs.range_set_cb(p_s_server,
                                                                           &p_rx_msg->meta_data,
                                                                           &in_data,
                                                                           p_out_data);

        if (LIGHT_LIGHTNESS_OPCODE_RANGE_SET == p_rx_msg->opcode.opcode)
        {
            (void) status_range_send(p_s_server, LIGHT_LIGHTNESS_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_DEFAULT_SET), handle_default_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_DEFAULT_SET_UNACKNOWLEDGED), handle_default_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_RANGE_SET), handle_range_set},
    {ACCESS_OPCODE_SIG(LIGHT_LIGHTNESS_OPCODE_RANGE_SET_UNACKNOWLEDGED), handle_range_set},
};

static const generic_onoff_server_callbacks_t m_onoff_srv_cbs =
{
    .onoff_cbs.set_cb = onoff_set_cb,
    .onoff_cbs.get_cb = onoff_get_cb
};

uint32_t light_lightness_setup_server_init(light_lightness_setup_server_t * p_s_server,
                                           uint8_t element_index)
{
    uint32_t status;

    if (!(p_s_server
       && p_s_server->settings.p_callbacks
       && p_s_server->settings.p_callbacks->light_lightness_cbs.set_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.default_set_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.range_set_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.delta_set_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.move_set_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.get_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.last_get_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.default_get_cb
       && p_s_server->settings.p_callbacks->light_lightness_cbs.range_get_cb))
    {
        return NRF_ERROR_NULL;
    }

    if (m_total_ll_instances >= LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
    {
        return NRF_ERROR_RESOURCES;
    }

    p_s_server->settings.element_index = element_index;

    /* Initialize parent model instances - Generic Power OnOff Setup */
    p_s_server->generic_ponoff_setup_srv.settings.element_index = element_index;

    /* Generic PonOff will init an OnOff instance. Set it up. */
    p_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.generic_onoff_srv.settings.p_callbacks =
        &m_onoff_srv_cbs;

    do
    {
        if (NRF_SUCCESS != (status = generic_ponoff_setup_server_init(
                                         &p_s_server->generic_ponoff_setup_srv,
                                         element_index)))
        {
            break;
        }

        /* Initialize parent model instances - Light Lightness */
        if (NRF_SUCCESS != (status = access_model_subscription_list_dealloc(
                                         p_s_server->generic_ponoff_setup_srv.model_handle)))
        {
            break;
        }


        if (NRF_SUCCESS != (status = access_model_subscription_list_dealloc(
                                         p_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_list_dealloc(
                                         p_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.generic_onoff_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_list_dealloc(
                                         p_s_server->generic_ponoff_setup_srv.generic_dtt_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = light_lightness_server_init(
                                         &p_s_server->light_lightness_srv,
                                         element_index)))
        {
            break;
        }

        /* Add this light lightness setup server model to the mesh */
        access_model_add_params_t init_params =
            {
                .model_id = ACCESS_MODEL_SIG(LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID),
                .element_index =  element_index,
                .p_opcode_handlers = &m_opcode_handlers_setup[0],
                .opcode_count = ARRAY_SIZE(m_opcode_handlers_setup),
                .p_args = p_s_server,
            };

        if (NRF_SUCCESS != (status = access_model_add(
                                         &init_params,
                                         &p_s_server->model_handle)))
        {
            break;
        }

        /* Light Lightness Server sets up its subscription list, then
         * shares it with each of its extended models */
        if (NRF_SUCCESS != (status = access_model_subscription_list_alloc(
                                         p_s_server->model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->generic_ponoff_setup_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->generic_ponoff_setup_srv.generic_ponoff_srv.generic_onoff_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->generic_ponoff_setup_srv.generic_dtt_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->light_lightness_srv.model_handle)))
        {
            break;
        }

        if (NRF_SUCCESS != (status = access_model_subscription_lists_share(
                                         p_s_server->model_handle,
                                         p_s_server->light_lightness_srv.generic_level_srv.model_handle)))
        {
            break;
        }
    } while (0);

    if (NRF_SUCCESS == status)
    {
        m_total_ll_instances++;
    }

    return status;
}

uint32_t light_lightness_ponoff_binding_setup(light_lightness_setup_server_t * p_s_server,
                                              light_lightness_saved_values_t * p_saved_states)
{
    if (p_s_server == NULL ||
        p_saved_states == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Do the appropriate bindings, and set the lightness. */
    light_lightness_set_params_t ll_actual_set = {0};
    uint16_t led_value = 0;

    /* 6.1.2.2.4 Binding with the Generic OnPowerUp state */
    if (p_saved_states->onpowerup == GENERIC_ON_POWERUP_OFF)
    {
        led_value = 0;
    }
    else if ((p_saved_states->onpowerup == GENERIC_ON_POWERUP_DEFAULT) &&
             (p_saved_states->default_lightness != 0))
    {
        led_value = p_saved_states->default_lightness;
    }
    else if ((p_saved_states->onpowerup == GENERIC_ON_POWERUP_DEFAULT) &&
             (p_saved_states->default_lightness == 0))
    {
        led_value = p_saved_states->last_lightness;
    }
    else if (p_saved_states->onpowerup == GENERIC_ON_POWERUP_RESTORE)
    {
        /* For Light Lightness, restore the actual value */
        led_value = p_saved_states->actual_lightness;
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Restore saved values - none matched\n");
    }

    /* Before setting the lightness value, make sure it conforms to
     * the range. (binding - 6.1.2.2.5) */
    led_value = light_lightness_utils_actual_to_range_restrict(led_value,
                                                         p_saved_states->range.range_min,
                                                         p_saved_states->range.range_max);
    ll_actual_set.lightness = led_value;

    /* Set clipped value */
    p_s_server->settings.p_callbacks->light_lightness_cbs.set_cb(p_s_server,
                                                                 NULL,
                                                                 &ll_actual_set,
                                                                 NULL,
                                                                 NULL);
    p_s_server->state.initialized = true;

    return NRF_SUCCESS;
}
