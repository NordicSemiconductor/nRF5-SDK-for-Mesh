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

#include "generic_ponoff_setup_server.h"
#include "generic_ponoff_common.h"
#include "generic_ponoff_messages.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "utils.h"
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

typedef enum
{
    PONOFF_SERVER,
    PONOFF_SETUP_SERVER
} server_context_type_t;

static uint32_t status_send(const void * p_ctx,
                            server_context_type_t ctx_type,
                            const access_message_rx_t * p_message,
                            const generic_ponoff_status_params_t * p_params)
{
    generic_ponoff_status_msg_pkt_t msg_pkt;
    uint16_t model_handle;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;

    if (ctx_type == PONOFF_SERVER)
    {
        const generic_ponoff_server_t * p_server = (generic_ponoff_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }
    else
    {
        NRF_MESH_ASSERT(p_message != NULL);
        const generic_ponoff_setup_server_t * p_server = (generic_ponoff_setup_server_t *) p_ctx;
        model_handle = p_server->model_handle;
        force_segmented = p_server->settings.force_segmented;
        transmic_size = p_server->settings.transmic_size;
    }

    if (p_params->on_powerup > GENERIC_ON_POWERUP_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    msg_pkt.on_powerup = p_params->on_powerup;

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_PONOFF_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = sizeof(generic_ponoff_status_msg_pkt_t),
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
    generic_ponoff_server_t * p_server = (generic_ponoff_server_t *) p_args;
    generic_ponoff_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t, generic_ponoff_srv, p_args);
    generic_ponoff_status_params_t out_data = {0};

    p_s_server->settings.p_callbacks->ponoff_cbs.get_cb(p_s_server, NULL, &out_data);
    (void) status_send(p_server, PONOFF_SERVER, NULL, &out_data);
}


/**************************************************************************************************/
/**** Generic ponoff server ****/
/** Opcode Handlers */

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_ponoff_server_t * p_server = (generic_ponoff_server_t *) p_args;
    generic_ponoff_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(generic_ponoff_setup_server_t, generic_ponoff_srv, p_args);
    generic_ponoff_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_s_server->settings.p_callbacks->ponoff_cbs.get_cb(p_s_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, PONOFF_SERVER, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_PONOFF_OPCODE_GET), handle_get},
};

/** Interface functions */

static uint32_t generic_ponoff_server_init(generic_ponoff_server_t * p_server, uint8_t element_index)
{
   uint32_t status;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize parent model instsances - Generic OnOff */
    status = generic_onoff_server_init(&p_server->generic_onoff_srv, element_index);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_dealloc(p_server->generic_onoff_srv.model_handle);
    }

    if (status == NRF_SUCCESS)
    {

        access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(GENERIC_PONOFF_SERVER_MODEL_ID),
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

uint32_t generic_ponoff_server_status_publish(generic_ponoff_server_t * p_server, const generic_ponoff_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, PONOFF_SERVER, NULL, p_params);
}

/**************************************************************************************************/
/**** Setup Server */
/** Opcode Handlers */

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_ponoff_setup_server_t * p_server = p_args;
    generic_ponoff_set_params_t in_param = {0};
    generic_ponoff_status_params_t out_param = {0};

    if (p_rx_msg->length == sizeof(generic_ponoff_set_msg_pkt_t))
    {
        generic_ponoff_set_msg_pkt_t * p_msg_params_packed = (generic_ponoff_set_msg_pkt_t *) p_rx_msg->p_data;

        if (p_msg_params_packed->on_powerup > GENERIC_ON_POWERUP_MAX)
        {
            return;
        }

        in_param.on_powerup = p_msg_params_packed->on_powerup;
        p_server->settings.p_callbacks->ponoff_cbs.set_cb(p_server, &p_rx_msg->meta_data, &in_param,
                                   (p_rx_msg->opcode.opcode == GENERIC_PONOFF_OPCODE_SET) ? &out_param : NULL);

        if (p_rx_msg->opcode.opcode == GENERIC_PONOFF_OPCODE_SET)
        {
            (void) status_send(p_server, PONOFF_SETUP_SERVER, p_rx_msg, &out_param);
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_PONOFF_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_PONOFF_OPCODE_SET_UNACKNOWLEDGED), handle_set},
};

/** Interface functions */

uint32_t generic_ponoff_setup_server_init(generic_ponoff_setup_server_t * p_server, uint8_t element_index)
{
    uint32_t status;

    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->ponoff_cbs.set_cb == NULL ||
        p_server->settings.p_callbacks->ponoff_cbs.get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize parent model instsances - Generic Default Transition Time */
    status = generic_dtt_server_init(&p_server->generic_dtt_srv, element_index);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_dealloc(p_server->generic_dtt_srv.model_handle);
    }

    /* Initialize parent model instsances - Generic Power OnOff */
    if (status == NRF_SUCCESS)
    {
        p_server->generic_ponoff_srv.settings.force_segmented = p_server->settings.force_segmented;
        p_server->generic_ponoff_srv.settings.transmic_size = p_server->settings.transmic_size;
        status = generic_ponoff_server_init(&p_server->generic_ponoff_srv, element_index);
    }

    if (status == NRF_SUCCESS)
    {
        /* Initialize this model instance */
        access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(GENERIC_PONOFF_SETUP_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers_setup[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers_setup),
            .p_args = p_server,
            .publish_timeout_cb = NULL
        };

        status = access_model_add(&init_params, &p_server->model_handle);
    }

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_lists_share(p_server->model_handle, p_server->generic_ponoff_srv.model_handle);
    }

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_lists_share(p_server->model_handle, p_server->generic_ponoff_srv.generic_onoff_srv.model_handle);
    }

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_lists_share(p_server->model_handle, p_server->generic_dtt_srv.model_handle);
    }

    return status;
}
