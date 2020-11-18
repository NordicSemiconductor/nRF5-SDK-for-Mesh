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

#include "light_lc_setup_server.h"

#include "access_config.h"
#include "nrf_mesh_assert.h"

#include "light_lc_messages.h"
#include "sensor_messages.h"
#include "light_lc_sensor_utils.h"
#include "light_lc_state_utils.h"
#include "light_lc_fsm.h"
#include "light_lc_occupancy_sensor.h"
#include "light_lc_ambient_light_sensor.h"
#include "generic_ponoff_common.h"

/* Each element can host only one LC Setup server instance.*/
STATIC_ASSERT(LIGHT_LC_SETUP_SERVER_INSTANCES_MAX <= ACCESS_ELEMENT_COUNT,\
              "LIGHT_LC_SETUP_SERVER_INSTANCES_MAX is out of range");

/** Total number of Light LC Setup server instances that can be created. */
static uint32_t m_total_lc_ss_instances;

static uint32_t status_mode_send(const light_lc_server_t * p_server,
                                 const access_message_rx_t * p_message,
                                 const light_lc_mode_status_params_t * p_params)
{
    light_lc_mode_status_msg_pkt_t msg_pkt;

    msg_pkt.mode = p_params->mode;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LC_MODE_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = sizeof(light_lc_mode_status_msg_pkt_t),
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

static uint32_t status_occupancy_mode_send(light_lc_server_t * p_server,
                                           const access_message_rx_t * p_message,
                                           const light_lc_occupancy_mode_status_params_t * p_params)
{
    light_lc_occupancy_mode_status_msg_pkt_t msg_pkt;

    msg_pkt.occupancy_mode = p_params->occupancy_mode;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LC_OCCUPANCY_MODE_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = sizeof(light_lc_occupancy_mode_status_msg_pkt_t),
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

static uint32_t status_light_onoff_send(light_lc_server_t * p_server,
                                        const access_message_rx_t * p_message,
                                        const light_lc_light_onoff_status_params_t * p_params)
{
    light_lc_light_onoff_status_msg_pkt_t msg_pkt;

    msg_pkt.present_light_onoff = p_params->present_light_onoff;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_light_onoff = p_params->target_light_onoff;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LC_LIGHT_ONOFF_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   LIGHT_LC_LIGHT_ONOFF_STATUS_MAXLEN :
                   LIGHT_LC_LIGHT_ONOFF_STATUS_MINLEN),
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

static uint32_t status_property_send(const light_lc_setup_server_t * p_s_server,
                                     const access_message_rx_t * p_message,
                                     const light_lc_property_status_params_t * p_params)
{
    light_lc_property_status_msg_pkt_t msg_pkt;
    uint8_t property_value_size;
    uint32_t status;

    msg_pkt.property_id = p_params->property_id;
    status = light_lc_state_utils_property_data_size_get(msg_pkt.property_id, &property_value_size);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (property_value_size > LIGHT_LC_PROPERTY_BUF_SIZE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Invalid buffer length %d, property %d length s.b. %d\n",
              LIGHT_LC_PROPERTY_BUF_SIZE, p_params->property_id, property_value_size);
        return NRF_ERROR_INVALID_LENGTH;
    }

    memcpy(msg_pkt.property_buffer, p_params->property_buffer, property_value_size);

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(LIGHT_LC_PROPERTY_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = LIGHT_LC_PROPERTY_SET_STATUS_FIXED_LEN + property_value_size,
        .force_segmented = p_s_server->settings.force_segmented,
        .transmic_size = p_s_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        status = access_model_publish(p_s_server->model_handle, &reply);
    }
    else
    {
        status = access_model_reply(p_s_server->model_handle, p_message, &reply);
    }

    return status;
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *)p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);

    light_lc_light_onoff_status_params_t out_data;

    out_data = light_lc_state_utils_light_onoff_get(p_s_server);

    (void) status_light_onoff_send(p_server, NULL, &out_data);
}

/**************************************************************************************************/
/**** LC server ****/
/** Opcode Handlers */

static void handle_lc_mode_set(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);
    light_lc_mode_set_params_t in_data = {0};
    light_lc_mode_status_params_t out_data = {0};

    if (p_rx_msg->length == sizeof(light_lc_mode_set_msg_pkt_t))
    {
        light_lc_mode_set_msg_pkt_t * p_msg_params =
            (light_lc_mode_set_msg_pkt_t *) p_rx_msg->p_data;

        in_data.mode = p_msg_params->mode;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "generate lc mode %d\n", in_data.mode);
        uint32_t status = light_lc_fsm_mode_on_off_event_generate(p_s_server, in_data.mode);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Set the LC mode state to off, and generate a mode "\
                                                "off event failed %d\n", status);
        }

        if (p_rx_msg->opcode.opcode == LIGHT_LC_MODE_OPCODE_SET)
        {
            out_data.mode = light_lc_state_utils_mode_get(p_s_server);

            (void) status_mode_send(p_server, p_rx_msg, &out_data);
        }
    }
}

static void handle_lc_occ_mode_set(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_server);
    light_lc_occupancy_mode_set_params_t in_data = {0};
    light_lc_occupancy_mode_status_params_t out_data = {0};

    if (p_rx_msg->length == sizeof(light_lc_occupancy_mode_set_msg_pkt_t))
    {
        light_lc_occupancy_mode_set_msg_pkt_t * p_msg_params =
            (light_lc_occupancy_mode_set_msg_pkt_t *) p_rx_msg->p_data;

        in_data.occupancy_mode = p_msg_params->occupancy_mode;
        light_lc_state_utils_occ_mode_set(p_s_server, in_data.occupancy_mode);

        /* occupancy mode doesn't generate an event, just set the state var */
        out_data.occupancy_mode = light_lc_state_utils_occ_mode_get(p_s_server);
        if (p_rx_msg->opcode.opcode == LIGHT_LC_OCCUPANCY_MODE_OPCODE_SET)
        {
            (void) status_occupancy_mode_send(p_server, p_rx_msg, &out_data);
        }

        /* Publish the occupancy mode status change */
        (void) light_lc_server_occ_mode_status_publish(p_server, &out_data);
    }
}

static void handle_lc_light_onoff_set(access_model_handle_t model_handle,
                                      const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);
    light_lc_light_onoff_set_params_t in_data = {0};
    model_transition_t in_data_tr = {0};
    model_transition_t * p_in_data_tr = NULL;
    light_lc_light_onoff_status_params_t out_data = {0};

    if (p_rx_msg->length == LIGHT_LC_LIGHT_ONOFF_SET_MINLEN ||
        p_rx_msg->length == LIGHT_LC_LIGHT_ONOFF_SET_MAXLEN)
    {
        light_lc_light_onoff_set_msg_pkt_t * p_msg_params =
            (light_lc_light_onoff_set_msg_pkt_t *) p_rx_msg->p_data;

        in_data.light_on_off = p_msg_params->light_on_off;
        in_data.tid = p_msg_params->tid;
        if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data,
                               LIGHT_LC_LIGHT_ONOFF_OPCODE_SET, in_data.tid))
        {
            if (p_rx_msg->length == LIGHT_LC_LIGHT_ONOFF_SET_MAXLEN)
            {
                if (!model_transition_time_is_valid(p_msg_params->transition_time))
                {
                    return;
                }

                in_data_tr.transition_time_ms =
                    model_transition_time_decode(p_msg_params->transition_time);
                in_data_tr.delay_ms = model_delay_decode(p_msg_params->delay);
                p_in_data_tr = &in_data_tr;
            }

            uint32_t status = light_lc_fsm_light_on_off_event_generate(p_s_server, in_data.light_on_off, p_in_data_tr);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Set the LC mode state to off, and generate a mode "\
                      "off event failed %d\n", status);
            }
            if (p_rx_msg->opcode.opcode == LIGHT_LC_LIGHT_ONOFF_OPCODE_SET)
            {
                /* find out what the FSM did here */
                out_data = light_lc_state_utils_light_onoff_get(p_s_server);
                status = status_light_onoff_send(p_server, p_rx_msg, &out_data);
                if (status != NRF_SUCCESS)
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Status light onoff send failed %d\n",
                          status);
                }
            }
        }
    }
}

static void handle_lc_mode_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);
    light_lc_mode_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        out_data.mode = light_lc_state_utils_mode_get(p_s_server);

        (void) status_mode_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_lc_occ_mode_get(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);
    light_lc_occupancy_mode_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        out_data.occupancy_mode = light_lc_state_utils_occ_mode_get(p_s_server);

        (void) status_occupancy_mode_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_lc_light_onoff_get(access_model_handle_t model_handle,
                                      const access_message_rx_t * p_rx_msg,
                                      void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_args);
    light_lc_light_onoff_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        out_data = light_lc_state_utils_light_onoff_get(p_s_server);

        (void) status_light_onoff_send(p_server, p_rx_msg, &out_data);
    }
}

static void handle_sensor_status(access_model_handle_t handle,
                                 const access_message_rx_t * p_rx_msg,
                                 void * p_args)
{
    light_lc_server_t * p_server = (light_lc_server_t *) p_args;
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_server);
    uint8_t * p_msg_data;
    uint16_t length;

    p_msg_data = (uint8_t *) p_rx_msg->p_data;
    length = p_rx_msg->length;
    if (length < SENSOR_STATUS_MINLEN || length > SENSOR_STATUS_MAXLEN)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid sensor data length %d\n", length);
        return;
    }

    /* There may be multiple messages in the sensor data - send to both sensor modules and they can
     * determine if there is anything they care about
     */
    (void) light_lc_occupancy_sensor_data_received(p_s_server, p_msg_data, length);
    (void) light_lc_ambient_light_sensor_data_received(p_s_server, p_msg_data, length);
    /* This is an unsolicited status broadcast from a sensor - no reply is expected.  The handler
     * has completed the expected functionality. */
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_LC_MODE_OPCODE_GET), handle_lc_mode_get},
    {ACCESS_OPCODE_SIG(LIGHT_LC_MODE_OPCODE_SET), handle_lc_mode_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_MODE_OPCODE_SET_UNACKNOWLEDGED), handle_lc_mode_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_OCCUPANCY_MODE_OPCODE_GET), handle_lc_occ_mode_get},
    {ACCESS_OPCODE_SIG(LIGHT_LC_OCCUPANCY_MODE_OPCODE_SET), handle_lc_occ_mode_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_OCCUPANCY_MODE_OPCODE_SET_UNACKNOWLEDGED), handle_lc_occ_mode_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_LIGHT_ONOFF_OPCODE_GET), handle_lc_light_onoff_get},
    {ACCESS_OPCODE_SIG(LIGHT_LC_LIGHT_ONOFF_OPCODE_SET), handle_lc_light_onoff_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_LIGHT_ONOFF_OPCODE_SET_UNACKNOWLEDGED), handle_lc_light_onoff_set},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_STATUS), handle_sensor_status},
};

/**************************************************************************************************/
/**** Setup Server */
/** Opcode Handlers */
static void handle_lc_property_set(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_args;
    light_lc_property_status_params_t out_data = {0};
    light_lc_property_set_msg_pkt_t * p_msg_params;
    uint8_t property_value_size;
    uint16_t property_id;
    uint32_t property_value = 0;

    p_msg_params = (light_lc_property_set_msg_pkt_t *) p_rx_msg->p_data;

    property_id = p_msg_params->property_id;
    if (property_id == 0)
    {
        /* MeshDeviceProperties_v1.1 4.1.3 - property id of 0 is prohibited */
        return;
    }
    if (light_lc_state_utils_property_data_size_get(p_msg_params->property_id, &property_value_size) != NRF_SUCCESS)
    {
        return;
    }

    if (p_rx_msg->length == LIGHT_LC_PROPERTY_SET_STATUS_FIXED_LEN + property_value_size)
    {
        memcpy(&property_value, p_msg_params->property_buffer, property_value_size);

        light_lc_state_utils_property_set(p_s_server, property_value, property_id);

        out_data.property_id = property_id;
        property_value = light_lc_state_utils_property_get(p_s_server, property_id);

        memcpy(out_data.property_buffer, &property_value, property_value_size);

        if (p_rx_msg->opcode.opcode == LIGHT_LC_PROPERTY_OPCODE_SET)
        {
            (void) status_property_send(p_s_server, p_rx_msg, &out_data);
        }

        (void) light_lc_setup_server_property_status_publish(p_s_server, &out_data);

    }
}

static void handle_lc_property_get(access_model_handle_t model_handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_args;
    light_lc_property_status_params_t out_data = {0};
    light_lc_property_get_msg_pkt_t in_data;
    light_lc_property_get_msg_pkt_t * p_data;
    uint8_t property_value_size;
    uint32_t property_value = 0;

    p_data = (light_lc_property_get_msg_pkt_t *) p_rx_msg->p_data;

    in_data.property_id = p_data->property_id;

    if (in_data.property_id == 0)
    {
        /* MeshDeviceProperties_v1.1 4.1.3 - property id of 0 is prohibited */
        return;
    }

    /* This is a get message that has data attached to it  (the property id) */
    if (p_rx_msg->length == LIGHT_LC_PROPERTY_GET_LEN)
    {
        out_data.property_id = in_data.property_id;
        property_value = light_lc_state_utils_property_get(p_s_server, in_data.property_id);

        if (light_lc_state_utils_property_data_size_get(in_data.property_id, &property_value_size) != NRF_SUCCESS)
        {
            return;
        }

        memcpy(out_data.property_buffer, &property_value, property_value_size);

        (void) status_property_send(p_s_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_LC_PROPERTY_OPCODE_GET), handle_lc_property_get},
    {ACCESS_OPCODE_SIG(LIGHT_LC_PROPERTY_OPCODE_SET), handle_lc_property_set},
    {ACCESS_OPCODE_SIG(LIGHT_LC_PROPERTY_OPCODE_SET_UNACKNOWLEDGED), handle_lc_property_set},
};

/** Callbacks from the extended on/off server (bound to Light LC Light
 * OnOff state) */
static void onoff_set_cb(const generic_onoff_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         const generic_onoff_set_params_t * p_in,
                         const model_transition_t * p_in_transition,
                         generic_onoff_status_params_t * p_out)
{
    const generic_onoff_server_t * p_onoff_server = p_self;
    light_lc_server_t * p_server = PARENT_BY_FIELD_GET(light_lc_server_t, generic_onoff_srv, p_self);
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_server);
    light_lc_light_onoff_set_params_t in_data = {0};
    light_lc_light_onoff_status_params_t out_data = {0};

    in_data.light_on_off = p_in->on_off;
    /* TID already validated by on/off model, so copy results into our tid_tracker */
    in_data.tid = p_in->tid;
    p_s_server->lc_srv.tid_tracker = p_onoff_server->tid_tracker;

    uint32_t status = light_lc_fsm_light_on_off_event_generate(p_s_server, in_data.light_on_off,
                                                         (model_transition_t *) p_in_transition);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Status light onoff event generate failed %d\n", status);
    }

    /* Report what the FSM did here */
    out_data = light_lc_state_utils_light_onoff_get(p_s_server);
    p_out->present_on_off = out_data.present_light_onoff;
    p_out->target_on_off = out_data.target_light_onoff;
    p_out->remaining_time_ms = out_data.remaining_time_ms;
}

static void onoff_get_cb(const generic_onoff_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         generic_onoff_status_params_t * p_out)
{
    const generic_onoff_server_t * p_onoff_server = p_self;
    light_lc_server_t * p_server = PARENT_BY_FIELD_GET(light_lc_server_t, generic_onoff_srv, p_onoff_server);
    light_lc_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(light_lc_setup_server_t, lc_srv, p_server);
    light_lc_light_onoff_status_params_t out_data = {0};

    out_data = light_lc_state_utils_light_onoff_get(p_s_server);

    p_out->present_on_off = out_data.present_light_onoff;
    p_out->target_on_off = out_data.target_light_onoff;
    p_out->remaining_time_ms = out_data.remaining_time_ms;
}

static const generic_onoff_server_callbacks_t m_onoff_srv_cbs =
{
    .onoff_cbs.set_cb  = onoff_set_cb,
    .onoff_cbs.get_cb  = onoff_get_cb,
};

static uint32_t lc_server_init(light_lc_setup_server_t * p_s_server,
                               uint8_t element_index)
{
    uint32_t status;
    light_lc_server_t * p_server;

    p_server = &p_s_server->lc_srv;

    /* Initialize parent model instances - Generic OnOff */
    p_server->generic_onoff_srv.settings.p_callbacks = &m_onoff_srv_cbs;

    status = generic_onoff_server_init(&p_server->generic_onoff_srv, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_subscription_list_dealloc(p_server->generic_onoff_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Add this LC server model to the mesh */
    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(LIGHT_LC_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    status = access_model_add(&init_params, &p_server->model_handle);
    return status;
}

uint32_t light_lc_server_mode_status_publish(const light_lc_server_t * p_server,
                                             const light_lc_mode_status_params_t * p_params)
{
    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_mode_send(p_server, NULL, p_params);
}

uint32_t light_lc_server_occ_mode_status_publish(light_lc_server_t * p_server,
                                                 const light_lc_occupancy_mode_status_params_t * p_params)
{
    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_occupancy_mode_send(p_server, NULL, p_params);
}

uint32_t light_lc_server_light_onoff_status_publish(light_lc_server_t * p_server,
                                                    const light_lc_light_onoff_status_params_t * p_params)
{
    generic_onoff_status_params_t onoff_params;

    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Since we are publishing light onoff status (done on change of state variable), and we are
     * bound to Generic OnOff (@tagMeshMdlSp section 6.2.3.3.1), publish a generic onoff server status,
     * too */
    onoff_params.present_on_off = p_params->present_light_onoff;
    onoff_params.target_on_off = p_params->target_light_onoff;
    onoff_params.remaining_time_ms = p_params->remaining_time_ms;
    (void) generic_onoff_server_status_publish(&p_server->generic_onoff_srv, &onoff_params);

    return status_light_onoff_send(p_server, NULL, p_params);
}

uint32_t light_lc_setup_server_property_status_publish(const light_lc_setup_server_t * p_s_server,
                                                       const light_lc_property_status_params_t * p_params)
{
    if (p_s_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_property_send(p_s_server, NULL, p_params);
}


uint32_t light_lc_setup_server_init(light_lc_setup_server_t * p_s_server,
                                    uint8_t element_index)
{
    uint32_t status;

    if (p_s_server == NULL ||
        p_s_server->settings.p_callbacks == NULL ||
        p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb == NULL ||
        p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_actual_set_cb == NULL ||
        p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_actual_get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (m_total_lc_ss_instances >= LIGHT_LC_SETUP_SERVER_INSTANCES_MAX)
    {
        return NRF_ERROR_RESOURCES;
    }

    p_s_server->state.ambient_luxlevel = 0;
    p_s_server->state.ambient_luxlevel_valid = false;
    p_s_server->state.luxlevel_out = 0;
    p_s_server->state.lightness_out = 0;

    p_s_server->transition_info.transition_time_ms = 0;
    p_s_server->transition_info.present_light_onoff = 0;
    p_s_server->transition_info.target_light_onoff = 0;
    p_s_server->transition_info.initial_present_lightness = 0;
    p_s_server->transition_info.target_lightness = 0;
    p_s_server->transition_info.initial_present_luxlevel = 0;
    p_s_server->transition_info.target_luxlevel = 0;

    status = light_lc_occupancy_sensor_init(p_s_server);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize parent model instance - LC Server */
    p_s_server->lc_srv.settings.element_index = element_index;
    status = lc_server_init(p_s_server, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Add this LC setup server model to the mesh */
    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(LIGHT_LC_SETUP_SERVER_MODEL_ID),
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

    /* LC Setup Server sets up its subscription list, then
     * shares it with its extended model */
    status = access_model_subscription_list_alloc(p_s_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_subscription_lists_share(p_s_server->model_handle,
                                                    p_s_server->lc_srv.generic_onoff_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_subscription_lists_share(p_s_server->model_handle,
                                                   p_s_server->lc_srv.model_handle);
    if (NRF_SUCCESS == status)
    {
        m_total_lc_ss_instances++;
    }
    return status;
}

uint32_t light_lc_setup_server_ponoff_binding_setup(light_lc_setup_server_t * p_s_server,
                                                    uint8_t onpowerup,
                                                    bool * p_lc_control)
{
    /* now that the models are all set up, read values out of flash, do the appropriate bindings,
     * and set the lightness for the case that LC is controlling the device.  For the cases where LC
     * mode is off, the mid app will call light_lightness_ponoff_binding to set the lightness */

    /* @tagMeshMdlSp section 6.5.1.2 PowerUp sequence behavior */

    uint8_t lc_mode;
    uint8_t lc_occ_mode;
    light_lc_light_onoff_status_params_t lc_light_onoff;
    uint16_t actual_lightness;
    uint32_t status;

    if ((p_s_server == NULL) || (p_lc_control == NULL))
    {
        return NRF_ERROR_NULL;
    }

    lc_mode = light_lc_state_utils_mode_get(p_s_server);
    lc_occ_mode = light_lc_state_utils_occ_mode_get(p_s_server);
    lc_light_onoff = light_lc_state_utils_light_onoff_get(p_s_server);

    if ((onpowerup == GENERIC_ON_POWERUP_OFF) || (onpowerup == GENERIC_ON_POWERUP_DEFAULT))
    {
        /* Start FSM as Off */
        status = light_lc_fsm_init(p_s_server, false);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
        /* Set the LC mode state to off, and generate a mode off event */
        status = light_lc_fsm_mode_on_off_event_generate(p_s_server, false);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
        /* Set the LC occ mode to its last known state */
        light_lc_state_utils_occ_mode_set(p_s_server, lc_occ_mode);
        /* The FSM will set the LC Light OnOff state to 0 with this event. */
        status = light_lc_fsm_light_on_off_event_generate(p_s_server, false, NULL);
        if (status != NRF_SUCCESS)
        {
            return status;
        }

        *p_lc_control = false; /* LC is not controlling the light */
    }
    else if ((onpowerup == GENERIC_ON_POWERUP_RESTORE) && (lc_mode == 0))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ONPOWERUP_START_FSM and lc_mode=0 \n");
        /* Start FSM as Off */
        status = light_lc_fsm_init(p_s_server, false);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Start FSM off failed %d\n", status);
            return status;
        }
        /* Set the LC mode state to off, and generate a mode off event */
        status = light_lc_fsm_mode_on_off_event_generate(p_s_server, false);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Set the LC mode state to off, and generate a mode "\
                  "off event failed %d\n", status);
            return status;
        }
        /* Set the LC occ mode to its last known state */
        light_lc_state_utils_occ_mode_set(p_s_server, lc_occ_mode);

        /* We do not set LC Light OnOff at all */
        *p_lc_control = false; /* LC is not controlling the light */
    }
    else if ((onpowerup == GENERIC_ON_POWERUP_RESTORE) && (lc_mode == 1))
    {
        /* Just booting up - set the light to a known value of 0, and
         * let the LC server take it from there */
        actual_lightness = 0;
        p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_actual_set_cb(p_s_server, actual_lightness);
        /* start FSM as On */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LC FSM ON \n");
        status = light_lc_fsm_init(p_s_server, true);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
        /* The following are to be generated in order */
        status = light_lc_fsm_mode_on_off_event_generate(p_s_server, true);
        if (status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Set the LC mode state to on, and generate a mode "\
                  "on event failed %d\n", status);
            return status;
        }
        /* Set the LC occ mode to its last known state */
        light_lc_state_utils_occ_mode_set(p_s_server, lc_occ_mode);
        /* generate a Light ONOff state */
        status = light_lc_fsm_light_on_off_event_generate(p_s_server, lc_light_onoff.target_light_onoff, NULL);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
        *p_lc_control = true; /* LC is controlling the light */
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Flash restore values - none matched\n");
        status = NRF_ERROR_INVALID_PARAM;
    }
    return status;
}
