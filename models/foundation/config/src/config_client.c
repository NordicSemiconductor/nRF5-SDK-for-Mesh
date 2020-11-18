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

#include "config_client.h"
#include "config_opcodes.h"
#include "config_messages.h"

#include <stdint.h>

#include "composition_data.h"

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "device_state_manager.h"

#include "nrf_mesh_assert.h"
#include "mesh_mem.h"
#include "log.h"

#define MAKE_BYTE_MASK(bits) (0xff >> (8u - bits))
#define CONFIG_PUBLISH_PERIOD(steps, resolution) \
    ((resolution & MAKE_BYTE_MASK(ACCESS_PUBLISH_STEP_RES_BITS)) << ACCESS_PUBLISH_STEP_NUM_BITS \
        | (steps & MAKE_BYTE_MASK(ACCESS_PUBLISH_STEP_NUM_BITS)))

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define CONFIG_CLIENT_MODEL_ID (0x0001)

#define PAYLOAD_LENGTH_MAX (NRF_MESH_SEG_PAYLOAD_SIZE_MAX)

#define IS_SIG_MODEL(model_id)                          \
    (model_id.company_id == ACCESS_COMPANY_ID_NONE)

/**
 * Calculates the size of a packet depending on whether it is a SIG model or not.
 */
#define PACKET_LENGTH_WITH_ID(packet_type, sig_model)                   \
    (sizeof(packet_type) - ((sig_model) ? sizeof(uint16_t) : 0))

typedef enum
{
    CONFIG_CLIENT_STATE_NONE,
    CONFIG_CLIENT_STATE_IDLE,
    CONFIG_CLIENT_STATE_WAIT_REPLY
} config_client_state_t;

typedef struct
{
    access_model_handle_t model_handle;
    config_client_state_t state;
    config_client_event_cb_t event_cb;
} config_client_t;

typedef struct
{
    config_opcode_t opcode;
    uint16_t payload_length_min;
    uint16_t payload_length_max;
} config_message_lut_t;

/*****************************************************************************
 * Static data
 *****************************************************************************/

static config_client_t m_client;
static uint8_t * mp_packet_buffer;

/*****************************************************************************
 * Static functions
 *****************************************************************************/
static void config_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args);

static void reliable_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    /* Some messages does not carry data, s.t. no packet is allocated for it. E.g., netkey_get(). */
    if (mp_packet_buffer != NULL)
    {
        mesh_mem_free(mp_packet_buffer);
        mp_packet_buffer = NULL;
    }

    m_client.state = CONFIG_CLIENT_STATE_IDLE;

    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            m_client.event_cb(CONFIG_CLIENT_EVENT_TYPE_TIMEOUT, NULL, 0);
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            m_client.event_cb(CONFIG_CLIENT_EVENT_TYPE_CANCELLED, NULL, 0);
            break;
        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static uint32_t send_reliable(config_opcode_t opcode, uint16_t length, config_opcode_t reply_opcode)
{
    access_reliable_t reliable;
    reliable.model_handle = m_client.model_handle;
    reliable.message.p_buffer = mp_packet_buffer;
    reliable.message.length = length;
    reliable.message.opcode.opcode = opcode;
    reliable.message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = reply_opcode;
    reliable.reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reliable.timeout = CONFIG_CLIENT_ACKED_TRANSACTION_TIMEOUT;
    reliable.status_cb = reliable_status_cb;

    uint32_t status = access_model_reliable_publish(&reliable);
    if (status != NRF_SUCCESS)
    {
        mesh_mem_free(mp_packet_buffer);
        mp_packet_buffer = NULL;
    }
    else
    {
        m_client.state = CONFIG_CLIENT_STATE_WAIT_REPLY;
    }
    return status;
}

/*****************************************************************************
 * Opcode handlers
 *****************************************************************************/

static const config_message_lut_t m_config_message_lut[] =
{
    {CONFIG_OPCODE_COMPOSITION_DATA_STATUS,             COMPOSITION_DATA_LENGTH_MIN,                                    PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_APPKEY_STATUS,                       sizeof(config_msg_appkey_status_t),                             sizeof(config_msg_appkey_status_t)},
    {CONFIG_OPCODE_APPKEY_LIST,                         sizeof(config_msg_appkey_list_t),                               PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_BEACON_STATUS,                       sizeof(config_msg_net_beacon_status_t),                         sizeof(config_msg_net_beacon_status_t)},
    {CONFIG_OPCODE_DEFAULT_TTL_STATUS,                  sizeof(config_msg_default_ttl_status_t),                        sizeof(config_msg_default_ttl_status_t)},
    {CONFIG_OPCODE_MODEL_APP_STATUS,                    PACKET_LENGTH_WITH_ID(config_msg_app_status_t, true),           PACKET_LENGTH_WITH_ID(config_msg_app_status_t, false)},
    {CONFIG_OPCODE_MODEL_PUBLICATION_STATUS,            PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, true),   PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, false)},
    {CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS,           PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, true),  PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, false)},
    {CONFIG_OPCODE_NETKEY_STATUS,                       sizeof(config_msg_netkey_status_t),                             sizeof(config_msg_netkey_status_t)},
    {CONFIG_OPCODE_NETKEY_LIST,                         0,                                                              PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_NODE_RESET_STATUS,                   0,                                                              0},
    {CONFIG_OPCODE_RELAY_STATUS,                        sizeof(config_msg_relay_status_t),                              sizeof(config_msg_relay_status_t)},
    {CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS,             sizeof(config_msg_network_transmit_status_t),                   sizeof(config_msg_network_transmit_status_t)},
    {CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS,            sizeof(config_msg_key_refresh_phase_status_t),                  sizeof(config_msg_key_refresh_phase_status_t)},
    {CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS,        sizeof(config_msg_heartbeat_publication_status_t),              sizeof(config_msg_heartbeat_publication_status_t)},
    {CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS,       sizeof(config_msg_heartbeat_subscription_status_t),             sizeof(config_msg_heartbeat_subscription_status_t)},
    {CONFIG_OPCODE_FRIEND_STATUS,                       sizeof(config_msg_friend_status_t),                             sizeof(config_msg_friend_status_t)},
    {CONFIG_OPCODE_GATT_PROXY_STATUS,                   sizeof(config_msg_proxy_status_t),                              sizeof(config_msg_proxy_status_t)},
    {CONFIG_OPCODE_NODE_IDENTITY_STATUS,                sizeof(config_msg_identity_status_t),                           sizeof(config_msg_identity_status_t)},
    {CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS,   sizeof(config_msg_low_power_node_polltimeout_status_t),         sizeof(config_msg_low_power_node_polltimeout_status_t)},
    {CONFIG_OPCODE_SIG_MODEL_APP_LIST,                  sizeof(config_msg_sig_model_app_list_t),                        PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_VENDOR_MODEL_APP_LIST,               sizeof(config_msg_vendor_model_app_list_t),                     PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST,         sizeof(config_msg_sig_model_subscription_list_t),               PAYLOAD_LENGTH_MAX},
    {CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST,      sizeof(config_msg_vendor_model_subscription_list_t),            PAYLOAD_LENGTH_MAX},
};

static const access_opcode_handler_t m_opcode_handlers[] =
{
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_COMPOSITION_DATA_STATUS),             config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_STATUS),                       config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_LIST),                         config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_BEACON_STATUS),                       config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_DEFAULT_TTL_STATUS),                  config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_APP_STATUS),                    config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS),            config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS),           config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_STATUS),                       config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_LIST),                         config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_RESET_STATUS),                   config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_RELAY_STATUS),                        config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS),             config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS),            config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS),        config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS),       config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_FRIEND_STATUS),                       config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_GATT_PROXY_STATUS),                   config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_IDENTITY_STATUS),                config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS),   config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_APP_LIST),                  config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_VENDOR_MODEL_APP_LIST),               config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST),         config_opcode_handler },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST),      config_opcode_handler },
};

/* Ensure that the two tables are the same size. */
NRF_MESH_STATIC_ASSERT((sizeof(m_opcode_handlers)/sizeof(m_opcode_handlers[0]) ==
                        sizeof(m_config_message_lut)/sizeof(m_config_message_lut[0])));


static bool get_opcode_index(config_opcode_t opcode, uint32_t * p_index)
{
    /* TODO: ~ Optimization candidate ~ */
    for (uint32_t i = 0; i < sizeof(m_config_message_lut)/sizeof(m_config_message_lut[0]); ++i)
    {
        if (m_config_message_lut[i].opcode == opcode)
        {
            *p_index = i;
            return true;
        }
    }
    return false;
}

static void config_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    uint32_t i = 0;
    NRF_MESH_ASSERT(get_opcode_index((config_opcode_t) p_message->opcode.opcode, &i));
    if (p_message->length < m_config_message_lut[i].payload_length_min ||
        p_message->length > m_config_message_lut[i].payload_length_max)
    {
        return;
    }

    config_client_event_t evt = {(config_opcode_t) p_message->opcode.opcode, (const config_msg_t *) p_message->p_data};
    m_client.event_cb(CONFIG_CLIENT_EVENT_TYPE_MSG, &evt, p_message->length);
}

static bool client_in_wrong_state(uint32_t * p_status)
{
    if (m_client.state == CONFIG_CLIENT_STATE_NONE)
    {
        *p_status = NRF_ERROR_INVALID_STATE;
        return true;
    }
    else if (m_client.state == CONFIG_CLIENT_STATE_WAIT_REPLY)
    {
        *p_status = NRF_ERROR_BUSY;
        return true;
    }
    return false;
}

static uint32_t subscription_virtual_add_del_owr_send(uint16_t element_address,
                                                      nrf_mesh_address_t address,
                                                      access_model_id_t model_id,
                                                      config_opcode_t opcode)
{
    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_subscription_virtual_add_del_owr_t * p_msg = (config_msg_subscription_virtual_add_del_owr_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    memcpy(p_msg->virtual_uuid, address.p_virtual_uuid, NRF_MESH_UUID_SIZE);
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    return send_reliable(opcode, length, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
}

static uint32_t subscription_add_del_owr_send(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id, config_opcode_t opcode)
{

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_subscription_add_del_owr_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_subscription_add_del_owr_t * p_msg = (config_msg_subscription_add_del_owr_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    p_msg->address = address.value;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    return send_reliable(opcode, length, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
}


static uint32_t publication_set(const config_publication_state_t * p_publication_state)
{
    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(p_publication_state->model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_publication_set_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_publication_set_t * p_msg = (config_msg_publication_set_t *) mp_packet_buffer;
    p_msg->element_address = p_publication_state->element_address;
    p_msg->publish_address = p_publication_state->publish_address.value;
    p_msg->state.appkey_index = p_publication_state->appkey_index;
    p_msg->state.credential_flag = 0; /* TODO: Not supported. */
    p_msg->state.rfu = 0;
    p_msg->state.publish_ttl = p_publication_state->publish_ttl;
    p_msg->state.publish_period = CONFIG_PUBLISH_PERIOD(p_publication_state->publish_period.step_num,
                                                        p_publication_state->publish_period.step_res);
    p_msg->state.retransmit_count = p_publication_state->retransmit_count;
    p_msg->state.retransmit_interval = p_publication_state->retransmit_interval;
    config_msg_model_id_set(&p_msg->state.model_id, &p_publication_state->model_id, sig_model);

    return send_reliable(CONFIG_OPCODE_MODEL_PUBLICATION_SET, length, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS);
}

static uint32_t publication_virtual_set(const config_publication_state_t * p_publication_state)
{
    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(p_publication_state->model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_publication_virtual_set_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_publication_virtual_set_t * p_msg = (config_msg_publication_virtual_set_t *) mp_packet_buffer;
    p_msg->element_address = p_publication_state->element_address;
    memcpy(p_msg->publish_uuid, p_publication_state->publish_address.p_virtual_uuid, NRF_MESH_UUID_SIZE);
    p_msg->state.appkey_index = p_publication_state->appkey_index;
    p_msg->state.credential_flag = 0; /* TODO: Not supported. */
    p_msg->state.rfu = 0;
    p_msg->state.publish_ttl = p_publication_state->publish_ttl;
    p_msg->state.publish_period = CONFIG_PUBLISH_PERIOD(p_publication_state->publish_period.step_num,
                                                        p_publication_state->publish_period.step_res);
    p_msg->state.retransmit_count = p_publication_state->retransmit_count;
    p_msg->state.retransmit_interval = p_publication_state->retransmit_interval;
    config_msg_model_id_set(&p_msg->state.model_id, &p_publication_state->model_id, sig_model);

    return send_reliable(CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET, length, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS);
}

static uint32_t app_bind_unbind_send(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id, config_opcode_t opcode)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_app_bind_unbind_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_app_bind_unbind_t * p_msg = (config_msg_app_bind_unbind_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    p_msg->appkey_index = appkey_index;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    return send_reliable(opcode, length, CONFIG_OPCODE_MODEL_APP_STATUS);
}

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t config_client_init(config_client_event_cb_t event_cb)
{
    if (event_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    const access_model_add_params_t alloc_params =
        {
            .model_id = {
                .model_id = CONFIG_CLIENT_MODEL_ID,
                .company_id = ACCESS_COMPANY_ID_NONE
            },
            .element_index = 0,
            .p_opcode_handlers = m_opcode_handlers,
            .opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0])
        };

    /* Ensure that the model has not already been initialized. */
    NRF_MESH_ASSERT(m_client.state == CONFIG_CLIENT_STATE_NONE);

    uint32_t status = access_model_add(&alloc_params, &m_client.model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    m_client.state = CONFIG_CLIENT_STATE_IDLE;
    m_client.event_cb = event_cb;
    return NRF_SUCCESS;
}

uint32_t config_client_server_set(dsm_handle_t server_devkey_handle, dsm_handle_t server_address_handle)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (server_address_handle == DSM_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    else if (server_devkey_handle == DSM_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    NRF_MESH_ASSERT(m_client.model_handle != ACCESS_HANDLE_INVALID);
    dsm_handle_t old_address = DSM_HANDLE_INVALID;
    status = access_model_publish_address_get(m_client.model_handle, &old_address);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_publish_address_set(m_client.model_handle, server_address_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = access_model_publish_application_set(m_client.model_handle, server_devkey_handle);
    if (status != NRF_SUCCESS)
    {
        if (old_address != DSM_HANDLE_INVALID)
        {
            /* Attempt to recover. */
            (void) access_model_publish_address_set(m_client.model_handle, old_address);
        }
        return status;
    }
    else
    {
        return status;
    }

}

uint32_t config_client_server_bind(dsm_handle_t server_devkey_handle)
{
    uint32_t status = NRF_SUCCESS;
    if (!client_in_wrong_state(&status))
    {
        status = access_model_application_bind(m_client.model_handle, server_devkey_handle);
    }

    return status;
}

uint32_t config_client_composition_data_get(uint8_t page_number)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_composition_data_get_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_composition_data_get_t * p_msg = (config_msg_composition_data_get_t *) mp_packet_buffer;
    p_msg->page_number = page_number;

    return send_reliable(CONFIG_OPCODE_COMPOSITION_DATA_GET, length, CONFIG_OPCODE_COMPOSITION_DATA_STATUS);
}

uint32_t config_client_appkey_add(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(p_appkey != NULL);
    NRF_MESH_ASSERT(mp_packet_buffer == NULL);

    uint16_t length = sizeof(config_msg_appkey_add_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_appkey_add_t * p_msg = (config_msg_appkey_add_t *) mp_packet_buffer;
    config_msg_key_index_24_set(&p_msg->key_indexes, netkey_index, appkey_index);
    memcpy(&p_msg->appkey[0], &p_appkey[0], NRF_MESH_KEY_SIZE);

    return send_reliable(CONFIG_OPCODE_APPKEY_ADD, length, CONFIG_OPCODE_APPKEY_STATUS);
}

uint32_t config_client_appkey_delete(uint16_t netkey_index, uint16_t appkey_index)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_appkey_delete_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_appkey_delete_t * p_msg = (config_msg_appkey_delete_t *) mp_packet_buffer;
    config_msg_key_index_24_set(&p_msg->key_indexes, netkey_index, appkey_index);

    return send_reliable(CONFIG_OPCODE_APPKEY_DELETE, length, CONFIG_OPCODE_APPKEY_STATUS);
}

uint32_t config_client_appkey_get(uint16_t netkey_index)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_appkey_get_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_appkey_get_t * p_msg = (config_msg_appkey_get_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    return send_reliable(CONFIG_OPCODE_APPKEY_GET, length, CONFIG_OPCODE_APPKEY_LIST);
}

uint32_t config_client_appkey_update(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_appkey_update_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_appkey_update_t * p_msg = (config_msg_appkey_update_t *) mp_packet_buffer;
    config_msg_key_index_24_set(&p_msg->key_indexes, netkey_index, appkey_index);
    memcpy(&p_msg->appkey[0], &p_appkey[0], NRF_MESH_KEY_SIZE);

    return send_reliable(CONFIG_OPCODE_APPKEY_UPDATE, length, CONFIG_OPCODE_APPKEY_STATUS);
}

uint32_t config_client_model_publication_get(uint16_t element_address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_publication_get_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_publication_get_t * p_msg = (config_msg_publication_get_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    return send_reliable(CONFIG_OPCODE_MODEL_PUBLICATION_GET, length, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS);
}

uint32_t config_client_model_publication_set(const config_publication_state_t * p_publication_state)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (p_publication_state == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_publication_state->publish_address.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        return publication_virtual_set(p_publication_state);
    }
    else
    {
        return publication_set(p_publication_state);
    }
}

uint32_t config_client_net_beacon_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_BEACON_GET, 0, CONFIG_OPCODE_BEACON_STATUS);
}

uint32_t config_client_net_beacon_set(config_net_beacon_state_t state)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_net_beacon_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_net_beacon_set_t * p_msg = (config_msg_net_beacon_set_t *) mp_packet_buffer;
    p_msg->beacon_state = state;

    return send_reliable(CONFIG_OPCODE_BEACON_SET, length, CONFIG_OPCODE_BEACON_STATUS);
}

uint32_t config_client_default_ttl_get(void)
{
    return send_reliable(CONFIG_OPCODE_DEFAULT_TTL_GET, 0, CONFIG_OPCODE_DEFAULT_TTL_STATUS);
}

uint32_t config_client_default_ttl_set(uint8_t ttl)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (ttl > NRF_MESH_TTL_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);

    uint16_t length = sizeof(config_msg_default_ttl_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_default_ttl_set_t * p_msg = (config_msg_default_ttl_set_t *) mp_packet_buffer;
    p_msg->ttl = ttl;

    return send_reliable(CONFIG_OPCODE_DEFAULT_TTL_SET, length, CONFIG_OPCODE_DEFAULT_TTL_STATUS);
}

uint32_t config_client_model_subscription_add(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (address.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        return subscription_virtual_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD);
    }
    else
    {
        return subscription_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD);
    }
}

uint32_t config_client_model_subscription_delete(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (address.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        return subscription_virtual_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE);
    }
    else
    {
        return subscription_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE);
    }
}

uint32_t config_client_model_subscription_delete_all(uint16_t element_address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_subscription_delete_all_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_subscription_delete_all_t * p_msg = (config_msg_subscription_delete_all_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    return send_reliable(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL, length, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
}

uint32_t config_client_model_subscription_get(uint16_t element_address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_model_subscription_get_t * p_msg = (config_msg_model_subscription_get_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    config_opcode_t opcode = sig_model ? CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET : CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET;
    config_opcode_t reply_opcode = sig_model ? CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST : CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST;
    return send_reliable(opcode, length, reply_opcode);
}

uint32_t config_client_model_subscription_overwrite(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (address.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        return subscription_virtual_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE);
    }
    else
    {
        return subscription_add_del_owr_send(element_address, address, model_id, CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE);
    }
}

uint32_t config_client_relay_get(void)
{
    return send_reliable(CONFIG_OPCODE_RELAY_GET, 0, CONFIG_OPCODE_RELAY_STATUS);
}

uint32_t config_client_relay_set(config_relay_state_t relay_state, uint8_t retransmit_count, uint8_t retransmit_interval_steps)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (retransmit_count > CONFIG_RETRANSMIT_COUNT_MAX ||
        retransmit_interval_steps > CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);

    uint16_t length = sizeof(config_msg_relay_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_relay_set_t * p_msg = (config_msg_relay_set_t *) mp_packet_buffer;
    p_msg->relay_state = relay_state;
    p_msg->relay_retransmit_count = retransmit_count;
    p_msg->relay_retransmit_interval_steps = retransmit_interval_steps;

    return send_reliable(CONFIG_OPCODE_RELAY_SET, length, CONFIG_OPCODE_RELAY_STATUS);
}

uint32_t config_client_network_transmit_get(void)
{
    return send_reliable(CONFIG_OPCODE_NETWORK_TRANSMIT_GET, 0, CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS);
}

uint32_t config_client_network_transmit_set(uint8_t transmit_count, uint8_t transmit_interval_steps)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }
    else if (transmit_count > CONFIG_RETRANSMIT_COUNT_MAX ||
             transmit_interval_steps > CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);

    uint16_t length = sizeof(config_msg_network_transmit_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_network_transmit_set_t * p_msg = (config_msg_network_transmit_set_t *) mp_packet_buffer;
    p_msg->network_transmit_count = transmit_count;
    p_msg->network_transmit_interval_steps = transmit_interval_steps;

    return send_reliable(CONFIG_OPCODE_NETWORK_TRANSMIT_SET, length, CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS);
}

uint32_t config_client_model_app_bind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id)
{
    return app_bind_unbind_send(element_address, appkey_index, model_id, CONFIG_OPCODE_MODEL_APP_BIND);
}

uint32_t config_client_model_app_get(uint16_t element_address, access_model_id_t model_id)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    bool sig_model = IS_SIG_MODEL(model_id);
    uint16_t length = PACKET_LENGTH_WITH_ID(config_msg_model_app_get_t, sig_model);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_model_app_get_t * p_msg = (config_msg_model_app_get_t *) mp_packet_buffer;
    p_msg->element_address = element_address;
    config_msg_model_id_set(&p_msg->model_id, &model_id, sig_model);

    config_opcode_t opcode = sig_model ? CONFIG_OPCODE_SIG_MODEL_APP_GET : CONFIG_OPCODE_VENDOR_MODEL_APP_GET;
    config_opcode_t reply_opcode = sig_model ? CONFIG_OPCODE_SIG_MODEL_APP_LIST : CONFIG_OPCODE_VENDOR_MODEL_APP_LIST;
    return send_reliable(opcode, length, reply_opcode);
}


uint32_t config_client_model_app_unbind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id)
{
    return app_bind_unbind_send(element_address, appkey_index, model_id, CONFIG_OPCODE_MODEL_APP_UNBIND);
}

uint32_t config_client_netkey_add(uint16_t netkey_index, const uint8_t * p_netkey)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_netkey_add_update_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_netkey_add_update_t * p_msg = (config_msg_netkey_add_update_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    memcpy(p_msg->netkey, p_netkey, NRF_MESH_KEY_SIZE);

    return send_reliable(CONFIG_OPCODE_NETKEY_ADD, length, CONFIG_OPCODE_NETKEY_STATUS);
}

uint32_t config_client_netkey_delete(uint16_t netkey_index)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_netkey_delete_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_netkey_delete_t * p_msg = (config_msg_netkey_delete_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    return send_reliable(CONFIG_OPCODE_NETKEY_DELETE, length, CONFIG_OPCODE_NETKEY_STATUS);
}

uint32_t config_client_netkey_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_NETKEY_GET, 0, CONFIG_OPCODE_NETKEY_LIST);
}

uint32_t config_client_netkey_update(uint16_t netkey_index, const uint8_t * p_netkey)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_netkey_add_update_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_netkey_add_update_t * p_msg = (config_msg_netkey_add_update_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    memcpy(p_msg->netkey, p_netkey, NRF_MESH_KEY_SIZE);

    return send_reliable(CONFIG_OPCODE_NETKEY_UPDATE, length, CONFIG_OPCODE_NETKEY_STATUS);
}

uint32_t config_client_node_reset(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_NODE_RESET, 0, CONFIG_OPCODE_NODE_RESET_STATUS);
}

uint32_t config_client_key_refresh_phase_get(uint16_t netkey_index)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_key_refresh_phase_get_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_key_refresh_phase_get_t * p_msg = (config_msg_key_refresh_phase_get_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    return send_reliable(CONFIG_OPCODE_KEY_REFRESH_PHASE_GET, length, CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS);
}

uint32_t config_client_key_refresh_phase_set(uint16_t netkey_index, nrf_mesh_key_refresh_phase_t phase)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_key_refresh_phase_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_key_refresh_phase_set_t * p_msg = (config_msg_key_refresh_phase_set_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    p_msg->transition = (uint8_t) phase;
    return send_reliable(CONFIG_OPCODE_KEY_REFRESH_PHASE_SET, length, CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS);
}

uint32_t config_client_friend_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_FRIEND_GET, 0, CONFIG_OPCODE_FRIEND_STATUS);
}

uint32_t config_client_friend_set(config_friend_state_t state)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_friend_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_friend_set_t * p_msg = (config_msg_friend_set_t *) mp_packet_buffer;
    p_msg->friend_state = state;
    return send_reliable(CONFIG_OPCODE_FRIEND_SET, length, CONFIG_OPCODE_FRIEND_STATUS);
}

uint32_t config_client_gatt_proxy_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_GATT_PROXY_GET, 0, CONFIG_OPCODE_GATT_PROXY_STATUS);
}

uint32_t config_client_gatt_proxy_set(config_gatt_proxy_state_t state)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_proxy_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_proxy_set_t * p_msg = (config_msg_proxy_set_t *) mp_packet_buffer;
    p_msg->proxy_state = state;
    return send_reliable(CONFIG_OPCODE_GATT_PROXY_SET, length, CONFIG_OPCODE_GATT_PROXY_STATUS);
}

uint32_t config_client_node_identity_get(uint16_t netkey_index)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_identity_get_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_identity_get_t * p_msg = (config_msg_identity_get_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    return send_reliable(CONFIG_OPCODE_NODE_IDENTITY_GET, length, CONFIG_OPCODE_NODE_IDENTITY_STATUS);
}

uint32_t config_client_node_identity_set(uint16_t netkey_index, config_identity_state_t state)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_identity_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_identity_set_t * p_msg = (config_msg_identity_set_t *) mp_packet_buffer;
    p_msg->netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    p_msg->identity_state = state;
    return send_reliable(CONFIG_OPCODE_NODE_IDENTITY_SET, length, CONFIG_OPCODE_NODE_IDENTITY_STATUS);
}

uint32_t config_client_heartbeat_publication_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET, 0, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS);
}

uint32_t config_client_heartbeat_publication_set(const config_msg_heartbeat_publication_set_t * p_publication)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_heartbeat_publication_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_heartbeat_publication_set_t * p_msg = (config_msg_heartbeat_publication_set_t *) mp_packet_buffer;
    *p_msg = *p_publication;
    return send_reliable(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET, length, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS);
}

uint32_t config_client_heartbeat_subscription_get(void)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    return send_reliable(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET, 0, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS);
}

uint32_t config_client_heartbeat_subscription_set(const config_msg_heartbeat_subscription_set_t * p_subscription)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_heartbeat_subscription_set_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_heartbeat_subscription_set_t * p_msg = (config_msg_heartbeat_subscription_set_t *) mp_packet_buffer;
    *p_msg = *p_subscription;

    return send_reliable(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET, length, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS);
}

uint32_t config_client_low_power_node_polltimeout_get(uint16_t lpn_address)
{
    uint32_t status = NRF_SUCCESS;
    if (client_in_wrong_state(&status))
    {
        return status;
    }

    NRF_MESH_ASSERT(mp_packet_buffer == NULL);
    uint16_t length = sizeof(config_msg_low_power_node_polltimeout_get_t);
    mp_packet_buffer = mesh_mem_alloc(length);
    if (mp_packet_buffer == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    config_msg_low_power_node_polltimeout_get_t * p_msg = (config_msg_low_power_node_polltimeout_get_t *) mp_packet_buffer;
    p_msg->lpn_address = lpn_address;
    return send_reliable(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_GET, length, CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS);
}

void config_client_pending_msg_cancel(void)
{
    (void)access_model_reliable_cancel(m_client.model_handle);
}

#if defined(UNIT_TEST)
void config_client_reset(void)
{
    memset(&m_client, 0, sizeof(m_client));
}
#endif
