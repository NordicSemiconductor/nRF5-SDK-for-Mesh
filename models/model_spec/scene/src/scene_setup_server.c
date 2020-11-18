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

#include "scene_setup_server.h"

#include "access_config.h"

#include "log.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0

/* Each element can host only one light lightness instance.*/
STATIC_ASSERT(SCENE_SETUP_SERVER_INSTANCES_MAX <= ACCESS_ELEMENT_COUNT,
              "SCENE_SETUP_SERVER_INSTANCES_MAX is greater than allowed.");

typedef enum
{
    SCENE_SERVER,
    SCENE_SETUP_SERVER,
} server_context_type_t;

/** Total number of Scene Setup server instances that can be created. */
static uint32_t m_total_scene_instances;

static uint32_t status_send(const void * p_ctx,
                            server_context_type_t ctx_type,
                            const access_message_rx_t * p_message,
                            const scene_status_params_t * p_params)
{
    scene_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (p_params->remaining_time_ms > TRANSITION_TIME_STEP_10M_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (ctx_type == SCENE_SERVER)
    {
        scene_server_t * p_server = (scene_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        scene_setup_server_t * p_s_server = (scene_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    msg_pkt.status_code = p_params->status_code;
    msg_pkt.current_scene = p_params->current_scene;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_scene = p_params->target_scene;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(SCENE_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = (p_params->remaining_time_ms > 0 ?
                   SCENE_STATUS_MAXLEN :
                   SCENE_STATUS_MINLEN),
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

static uint32_t register_status_send(const void * p_ctx,
                                     server_context_type_t ctx_type,
                                     const access_message_rx_t * p_message,
                                     const scene_register_status_params_t * p_params)
{
    uint8_t msg_pkt[SCENE_REGISTER_STATUS_MAXLEN] = {0};
    scene_register_status_msg_pkt_t * p_msg_pkt = (scene_register_status_msg_pkt_t *)msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (ctx_type == SCENE_SERVER)
    {
        scene_server_t * p_server = (scene_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        scene_setup_server_t * p_s_server = (scene_setup_server_t *) p_ctx;
        model_handle = p_s_server->model_handle;
        force_segmented = p_s_server->settings.force_segmented;
        transmic_size = p_s_server->settings.transmic_size;
    }

    p_msg_pkt->status_code = p_params->status_code;
    p_msg_pkt->current_scene = p_params->current_scene;
    uint16_t len = SCENE_REGISTER_STATUS_MINLEN;
    uint32_t j = 0;
    for (uint32_t k = 0; k < ARRAY_SIZE(p_params->scenes); k++)
    {
         if (p_params->scenes[k] != SCENE_NUMBER_NO_SCENE)
         {
             p_msg_pkt->scenes[j++] = p_params->scenes[k];
             len += 2;
         }
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(SCENE_OPCODE_REGISTER_STATUS),
        .p_buffer = (const uint8_t *) p_msg_pkt,
        .length = len,
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

/**************************************************************************************************/
/**** Scene server ****/
static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    scene_server_t * p_server = (scene_server_t *) p_args;
    scene_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(scene_setup_server_t,
                                                            scene_srv, p_args);
    scene_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->scene_cbs.get_cb(p_s_server, NULL, &out_data);

    (void) status_send(p_server, SCENE_SERVER, NULL, &out_data);
}


/** Opcode Handlers */
static void handle_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    scene_server_t * p_server = (scene_server_t *) p_args;
    scene_setup_server_t * p_s_server =
            PARENT_BY_FIELD_GET(scene_setup_server_t, scene_srv, p_args);
    scene_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->scene_cbs.get_cb(p_s_server,
                                                           &p_rx_msg->meta_data,
                                                           &out_data);
        (void) status_send(p_server, SCENE_SERVER, p_rx_msg, &out_data);
    }
}

static void handle_register_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    scene_server_t * p_server = (scene_server_t *) p_args;
    scene_setup_server_t * p_s_server =
            PARENT_BY_FIELD_GET(scene_setup_server_t, scene_srv, p_args);
    scene_register_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->scene_cbs.register_get_cb(p_s_server,
                                                                    &p_rx_msg->meta_data,
                                                                    &out_data);
        (void) register_status_send(p_server, SCENE_SERVER, p_rx_msg, &out_data);
    }
}

static void handle_recall(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    scene_server_t * p_server = (scene_server_t *) p_args;
    scene_setup_server_t * p_s_server =
            PARENT_BY_FIELD_GET(scene_setup_server_t, scene_srv, p_args);
    scene_recall_params_t in_data = {0};
    model_transition_t in_data_tr = {0};
    scene_status_params_t out_data = {0};

    if ((p_rx_msg->length == SCENE_RECALL_MINLEN) || (p_rx_msg->length == SCENE_RECALL_MAXLEN))
    {
        scene_recall_msg_pkt_t * p_msg_params_packed = (scene_recall_msg_pkt_t *) p_rx_msg->p_data;
        in_data.scene_number = p_msg_params_packed->scene_number;
        in_data.tid = p_msg_params_packed->tid;

        if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, SCENE_OPCODE_RECALL, in_data.tid))
        {
            if (p_rx_msg->length == SCENE_RECALL_MAXLEN)
            {
                if (!model_transition_time_is_valid(p_msg_params_packed->transition_time))
                {
                    return;
                }

                in_data_tr.transition_time_ms = model_transition_time_decode(p_msg_params_packed->transition_time);
                in_data_tr.delay_ms = model_delay_decode(p_msg_params_packed->delay);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: transition_time_ms = %X\n", in_data_tr.transition_time_ms);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Server: delay_ms = %X\n", in_data_tr.delay_ms);
            }

            p_s_server->settings.p_callbacks->scene_cbs.recall_cb(p_s_server,
                                                                  &p_rx_msg->meta_data,
                                                                  &in_data,
                                                                  (p_rx_msg->length == SCENE_RECALL_MINLEN) ? NULL : &in_data_tr,
                                                                  (p_rx_msg->opcode.opcode == SCENE_OPCODE_RECALL) ? &out_data : NULL);

            if (p_rx_msg->opcode.opcode == SCENE_OPCODE_RECALL)
            {
                (void) status_send(p_server, SCENE_SERVER, p_rx_msg, &out_data);
            }
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_REGISTER_GET), handle_register_get},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_RECALL), handle_recall},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_RECALL_UNACKNOWLEDGED), handle_recall},
};

static uint32_t scene_server_init(scene_server_t * p_server,
                                  uint8_t element_index)
{
    uint32_t status;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(SCENE_SERVER_MODEL_ID),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    status = access_model_add(&init_params, &p_server->model_handle);

    return status;
}

/** Interface functions */
uint32_t scene_server_status_publish(const scene_server_t * p_server,
                                     const scene_status_params_t * p_params)
{
    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, SCENE_SERVER, NULL, p_params);
}


/**************************************************************************************************/
/**** Scene Setup Server */
static void periodic_publish_ss_cb(access_model_handle_t handle, void * p_args)
{
    scene_setup_server_t * p_s_server = (scene_setup_server_t *) p_args;
    scene_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->scene_cbs.get_cb(p_s_server, NULL, &out_data);
    (void) status_send(p_s_server, SCENE_SETUP_SERVER, NULL, &out_data);
}

/** Opcode Handlers */
static void handle_store(access_model_handle_t model_handle,
                         const access_message_rx_t * p_rx_msg,
                         void * p_args)
{
    const scene_setup_server_t * p_s_server = (scene_setup_server_t *) p_args;
    scene_store_params_t in_data = {0};
    scene_register_status_params_t out_data = {0};

    if (sizeof(scene_store_msg_pkt_t) == p_rx_msg->length)
    {
        scene_store_msg_pkt_t * p_msg_params_packed = (scene_store_msg_pkt_t *) p_rx_msg->p_data;
        in_data.scene_number = p_msg_params_packed->scene_number;

        p_s_server->settings.p_callbacks->scene_cbs.store_cb(p_s_server,
                                                             &p_rx_msg->meta_data,
                                                             &in_data,
                                                             (p_rx_msg->opcode.opcode == SCENE_OPCODE_STORE) ? &out_data : NULL);

        if (p_rx_msg->opcode.opcode == SCENE_OPCODE_STORE)
        {
            (void) register_status_send(p_s_server, SCENE_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static void handle_delete(access_model_handle_t model_handle,
                          const access_message_rx_t * p_rx_msg,
                          void * p_args)
{
    const scene_setup_server_t * p_s_server = (scene_setup_server_t *) p_args;
    scene_delete_params_t in_data = {0};
    scene_register_status_params_t out_data = {0};

    if (sizeof(scene_delete_msg_pkt_t) == p_rx_msg->length)
    {
        scene_delete_msg_pkt_t * p_msg_params_packed = (scene_delete_msg_pkt_t *) p_rx_msg->p_data;
        in_data.scene_number = p_msg_params_packed->scene_number;

        p_s_server->settings.p_callbacks->scene_cbs.delete_cb(p_s_server,
                                                              &p_rx_msg->meta_data,
                                                              &in_data,
                                                              (p_rx_msg->opcode.opcode == SCENE_OPCODE_DELETE) ? &out_data : NULL);

        if (p_rx_msg->opcode.opcode == SCENE_OPCODE_DELETE)
        {
            (void) register_status_send(p_s_server, SCENE_SETUP_SERVER, p_rx_msg, &out_data);
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_STORE), handle_store},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_STORE_UNACKNOWLEDGED), handle_store},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_DELETE), handle_delete},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_DELETE_UNACKNOWLEDGED), handle_delete},
};

/** Interface functions */
uint32_t scene_setup_server_init(scene_setup_server_t * p_s_server, uint8_t element_index)
{
    if (p_s_server == NULL ||
        p_s_server->settings.p_callbacks == NULL ||
        p_s_server->settings.p_callbacks->scene_cbs.store_cb == NULL        ||
        p_s_server->settings.p_callbacks->scene_cbs.delete_cb == NULL       ||
        p_s_server->settings.p_callbacks->scene_cbs.get_cb == NULL          ||
        p_s_server->settings.p_callbacks->scene_cbs.register_get_cb == NULL ||
        p_s_server->settings.p_callbacks->scene_cbs.recall_cb == NULL ||
        p_s_server->p_gen_dtt_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (m_total_scene_instances >= SCENE_SETUP_SERVER_INSTANCES_MAX)
    {
        return NRF_ERROR_RESOURCES;
    }

    /* Ensure that default transition time server is already initialized and it is on the same
     * element as that of the scene setup server. */
    uint16_t dtt_server_element;
    uint32_t status = access_model_element_index_get(p_s_server->p_gen_dtt_server->model_handle,
                                                     &dtt_server_element);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (dtt_server_element != element_index)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    p_s_server->settings.element_index = element_index;

    p_s_server->scene_srv.settings.force_segmented = p_s_server->settings.force_segmented;
    p_s_server->scene_srv.settings.transmic_size = p_s_server->settings.transmic_size;
    status = scene_server_init(&p_s_server->scene_srv, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Add this scene setup server model to the mesh */
    access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(SCENE_SETUP_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers_setup[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers_setup),
            .p_args = p_s_server,
            .publish_timeout_cb = periodic_publish_ss_cb
        };

    status = access_model_add(&init_params, &p_s_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* The subscription list of scene models is shared with the default transition time model on the
     * same element. */
    status = access_model_subscription_lists_share(p_s_server->p_gen_dtt_server->model_handle,
                                                   p_s_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_subscription_lists_share(p_s_server->model_handle,
                                                   p_s_server->scene_srv.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    m_total_scene_instances++;
    return status;
}

#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0*/
