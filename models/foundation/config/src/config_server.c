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

#include <stdint.h>
#include <string.h>

#include "access.h"
#include "access_config.h"
#include "access_status.h"
#include "access_utils.h"
#include "event.h"

#include "config_messages.h"
#include "config_opcodes.h"
#include "config_server_events.h"
#include "config_server.h"
#include "composition_data.h"
#include "device_state_manager.h"
#include "packed_index_list.h"
#include "bearer_defines.h"

#include "flash_manager.h"
#include "net_state.h"
#include "net_beacon.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_keygen.h"
#include "mesh_opt_core.h"
#include "mesh_opt_gatt.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_utils.h"
#include "heartbeat.h"
#include "mesh_stack.h"
#include "log.h"

#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "proxy.h"
#endif

#if MESH_FEATURE_FRIEND_ENABLED
#include "mesh_friend.h"
#include "friend_internal.h"
#endif

/** Configuration server model ID. */
#define CONFIG_SERVER_MODEL_ID  0x0000

/*
 * Checks if the length of a packet is valid. This macro is used to check the sizes
 * of packets that have a config_model_id_t member, which means the packets can vary in
 * size by sizeof(uint16_t).
 */
#define IS_PACKET_LENGTH_VALID_WITH_ID(packet_type, p_packet) \
    ((p_packet)->length == sizeof(packet_type) || (p_packet)->length == sizeof(packet_type) - sizeof(uint16_t))

/**
 * Checks if the message is a SIG or vendor model message.
 * For some messages, both variants are possible, and the difference is determined by
 * checking the length of the message; SIG models contain a 16-bit model ID, while
 * vendor models contain a 32-bit model ID, making SIG messages 2 bytes shorter than
 * vendor messages.
 */
#define IS_SIG_MODEL(p_message, message_type) \
    ((p_message)->length == sizeof(message_type) - sizeof(uint16_t))

/**
 * Calculates the size of a packet depending on whether it is a SIG model or not.
 * See the description of IS_SIG_MODEL() for an explanation of the difference.
 */
#define PACKET_LENGTH_WITH_ID(packet_type, sig_model) \
    (sizeof(packet_type) - ((sig_model) ? sizeof(uint16_t) : 0))

#define KEY_REFRESH_TRANSITION_TO_PHASE_2      2
#define KEY_REFRESH_TRANSITION_TO_PHASE_3      3


/********** Static asserts for limits imposed by configuration message sizes **********/

/** The Application keys must fit inside the Vendor Model App List packet. */
NRF_MESH_STATIC_ASSERT(DSM_APP_MAX_LIMIT <=
    (2 * (ACCESS_MESSAGE_LENGTH_MAX - ACCESS_UTILS_SIG_OPCODE_SIZE(CONFIG_OPCODE_VENDOR_MODEL_APP_LIST) - offsetof(config_msg_vendor_model_app_list_t, key_indexes)) / 3));

/** The network keys must fit inside the NetKey List packet. */
NRF_MESH_STATIC_ASSERT(DSM_SUBNET_MAX_LIMIT <=
    (2 * (ACCESS_MESSAGE_LENGTH_MAX - ACCESS_UTILS_SIG_OPCODE_SIZE(CONFIG_OPCODE_NETKEY_LIST)) / 3));

/** The subscription addresses must fit inside the Vendor Model Subscription List packet. */
NRF_MESH_STATIC_ASSERT(DSM_ADDR_MAX_LIMIT <=
    ((ACCESS_MESSAGE_LENGTH_MAX - ACCESS_UTILS_SIG_OPCODE_SIZE(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST) - offsetof(config_msg_vendor_model_subscription_list_t, subscriptions)) / 2));

/** The composition data must fit within the Composition Data Status packet. If this assert fails,
 * there are too many models or too many elements defined. Note that the macro assumes all models to be vendor models. */
NRF_MESH_STATIC_ASSERT(CONFIG_COMPOSITION_DATA_SIZE <=
    (ACCESS_MESSAGE_LENGTH_MAX - ACCESS_UTILS_SIG_OPCODE_SIZE(CONFIG_OPCODE_COMPOSITION_DATA_STATUS) - sizeof(config_msg_composition_data_status_t)));


typedef enum
{
    NODE_RESET_IDLE,
    NODE_RESET_PENDING,
#if MESH_FEATURE_GATT_PROXY_ENABLED
    NODE_RESET_PENDING_PROXY,
#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */
    NODE_RESET_FLASHING,
} node_reset_state_t;

static access_model_handle_t m_config_server_handle = ACCESS_HANDLE_INVALID;
static config_server_evt_cb_t m_evt_cb;

/** Mesh event handler. */
static void mesh_event_cb(const nrf_mesh_evt_t * p_evt);
static nrf_mesh_evt_handler_t m_mesh_evt_handler = { .evt_cb = mesh_event_cb };

static nrf_mesh_tx_token_t m_reset_token;
static node_reset_state_t m_node_reset_pending = NODE_RESET_IDLE;

/********** Helper functions **********/

static inline void app_evt_send(const config_server_evt_t * p_evt)
{
    if (m_evt_cb)
    {
        m_evt_cb(p_evt);
    }
}

static bool model_id_extract(access_model_id_t * p_dest,
                             const config_model_id_t * p_id,
                             const access_message_rx_t * p_incoming,
                             uint16_t msg_size)
{
    bool sig_model = (p_incoming->length == ((msg_size) - sizeof(uint16_t)));

    if (sig_model)
    {
        p_dest->model_id = p_id->sig.model_id;
        p_dest->company_id = ACCESS_COMPANY_ID_NONE;
    }
    else
    {
        p_dest->model_id = p_id->vendor.model_id;
        p_dest->company_id = p_id->vendor.company_id;
    }

    return sig_model;
}

/*lint -ecall(569, send_reply) Conversion of size_t (returned by sizeof()) to uint16_t causes loss of precision. */
/*
 * Sends a message as a reply to an incoming message.
 */
static void send_reply(access_model_handle_t handle, const access_message_rx_t * p_message, uint16_t opcode,
        const uint8_t * p_reply, uint16_t reply_length, uint32_t tx_token)
{
    const access_message_tx_t reply =
    {
        .opcode =
        {
            .opcode = opcode,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .p_buffer = p_reply,
        .length = reply_length,
        .access_token = (nrf_mesh_tx_token_t)tx_token,
        .force_segmented = false,
        .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT
    };

    (void) access_model_reply(handle, p_message, &reply);
}

/* Sends the network beacon state in response to a network beacon state set/get message: */
static void send_net_beacon_state(access_model_handle_t handle, const access_message_rx_t * p_incoming)
{
    config_msg_net_beacon_status_t beacon_status =
    {
        .beacon_state = net_beacon_state_get() ? CONFIG_NET_BEACON_STATE_ENABLED : CONFIG_NET_BEACON_STATE_DISABLED
    };
    send_reply(handle, p_incoming, CONFIG_OPCODE_BEACON_STATUS,
               (const uint8_t *) &beacon_status, sizeof(beacon_status), nrf_mesh_unique_token_get());
}

/* Sends the appkey status in response to appkey manipulation functions: */
static void send_appkey_status(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               access_status_t status,
                               config_msg_key_index_24_t key_indexes)
{
    const config_msg_appkey_status_t response =
    {
        .status = status,
        .key_indexes = key_indexes
    }; /*lint !e64 Lint incorrectly complains about a type mismach in this initializer. */
    send_reply(handle, p_message, CONFIG_OPCODE_APPKEY_STATUS,
               (const uint8_t *) &response, sizeof(response), nrf_mesh_unique_token_get());
}

/* Sends the publication status in response to publication messages. */
static void send_publication_status(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
        uint16_t element_address, access_model_handle_t model_handle)
{
    /* Build the publication status packet: */
    config_msg_publication_status_t response;
    uint32_t status;
    bool credential_flag;

    memset(&response.state, 0, sizeof(response.state));
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = element_address;
    NRF_MESH_ASSERT(access_model_publish_friendship_credential_flag_get(model_handle, &credential_flag) == NRF_SUCCESS);
    response.state.credential_flag = credential_flag;
    response.state.rfu = 0;

    /* Get the model ID: */
    access_model_id_t model_id;
    NRF_MESH_ASSERT(access_model_id_get(model_handle, &model_id) == NRF_SUCCESS);

    bool sig_model = model_id.company_id == ACCESS_COMPANY_ID_NONE;
    config_msg_model_id_set(&response.state.model_id, &model_id, sig_model);

    dsm_handle_t publish_address_handle;
    NRF_MESH_ASSERT(access_model_publish_address_get(model_handle, &publish_address_handle) == NRF_SUCCESS);

    if (publish_address_handle ==  DSM_HANDLE_INVALID)
    {
        /* If no publish address is set, the rest of the packet is set to 0: */
        response.publish_address = NRF_MESH_ADDR_UNASSIGNED;
    }
    else
    {
        /* Obtain the publish address: */
        nrf_mesh_address_t publish_address;
        NRF_MESH_ASSERT(dsm_address_get(publish_address_handle, &publish_address) == NRF_SUCCESS);
        response.publish_address = publish_address.value;

        /* Obtain the application key handle: */
        dsm_handle_t appkey_handle;
        status = access_model_publish_application_get(model_handle, &appkey_handle);

        if (status == NRF_ERROR_NOT_FOUND)
        {
            /* If model is not found, the rest of the packet is set to 0: */
            response.publish_address = NRF_MESH_ADDR_UNASSIGNED;
        }
        else if (status == NRF_SUCCESS)
        {
            /* Get the appkey index: */
            mesh_key_index_t appkey_index;
            status = dsm_appkey_handle_to_appkey_index(appkey_handle, &appkey_index);

            if (status == NRF_ERROR_NOT_FOUND)
            {
                /* If appkey is not found (possibly deleted), the rest of the packet is set to 0: */
                response.publish_address = NRF_MESH_ADDR_UNASSIGNED;
            }
            else if (status == NRF_SUCCESS)
            {
                response.state.appkey_index = appkey_index;

                /* Obtain the publish TTL: */
                (void ) access_model_publish_ttl_get(model_handle, &response.state.publish_ttl);

                /* Obtain the publish period: */
                access_publish_resolution_t publish_resolution;
                uint8_t publish_steps;

                memset(&publish_resolution, 0, sizeof(access_publish_resolution_t));
                (void) access_model_publish_period_get(model_handle, &publish_resolution, &publish_steps);

                /* Set the publish period in the message: */
                response.state.publish_period  = publish_resolution << ACCESS_PUBLISH_STEP_NUM_BITS | publish_steps;

                /* Set publish retransmit count and interval: */
                access_publish_retransmit_t publish_retransmit;
                (void) access_model_publish_retransmit_get(model_handle, &publish_retransmit);
                response.state.retransmit_count = publish_retransmit.count;             /* TODO: MBTLE-2388 */
                response.state.retransmit_interval = publish_retransmit.interval_steps; /* TODO: MBTLE-2388 */
            }
        }
        else
        {
            /* If this asserts, something is wrong */
            NRF_MESH_ASSERT(false);
        }
    }

    /* Finally send the message: */
    send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model), nrf_mesh_unique_token_get());
}

/* Sends the publication error codes in response to publication messages. */
static void status_error_pub_send(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
                                      bool sig_model, uint8_t status_opcode)
{
    switch (p_incoming->opcode.opcode)
    {
        case CONFIG_OPCODE_MODEL_PUBLICATION_GET:
        case CONFIG_OPCODE_MODEL_PUBLICATION_SET:
        case CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET:
        {
            /* Build the publication status packet: */
            config_msg_publication_status_t response;

            memset(&response, 0, sizeof(config_msg_publication_status_t));
            response.status = status_opcode;

            switch(p_incoming->opcode.opcode)
            {
                case CONFIG_OPCODE_MODEL_PUBLICATION_GET:
                {
                    const config_msg_publication_get_t * p_pdu = (const config_msg_publication_get_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_PUBLICATION_SET:
                {
                    const config_msg_publication_set_t * p_pdu = (const config_msg_publication_set_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->state.model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET:
                {
                    const config_msg_publication_virtual_set_t * p_pdu = (const config_msg_publication_virtual_set_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->state.model_id;
                    break;
                }

                default:
                    NRF_MESH_ASSERT(false);
                    break;
            }

            /* Finally send the message: */
            send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &response,
                       PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model), nrf_mesh_unique_token_get());
            break;
        }
    }
}

/* Sends the subscription error codes in response to subscription messages. */
static void status_error_sub_send(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
                                      bool sig_model, uint8_t status_opcode)
{
    switch (p_incoming->opcode.opcode)
    {
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL:
        {
            /* Build the subscription status packet: */
            config_msg_subscription_status_t response;

            memset(&response, 0, sizeof(config_msg_subscription_status_t));
            response.status = status_opcode;

            switch(p_incoming->opcode.opcode)
            {
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE:
                {
                    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.address  = p_pdu->address;
                    response.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE:
                {
                    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.address = 0; /* Since incoming message does not have address field */
                    response.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL:
                {
                    const config_msg_subscription_delete_all_t * p_pdu = (const config_msg_subscription_delete_all_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.model_id = p_pdu->model_id;
                    break;
                }

                default:
                    NRF_MESH_ASSERT(false);
                    break;
            }

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &response,
                       PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model), nrf_mesh_unique_token_get());
            break;
        }

        case CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET:
        {
            /* Build the subscription status packet: */
            config_msg_sig_model_subscription_list_t response;

            memset(&response, 0, sizeof(config_msg_sig_model_subscription_list_t));
            response.status = status_opcode;

            const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_incoming->p_data;
            response.element_address = p_pdu->element_address;
            response.sig_model_id    = p_pdu->model_id.sig.model_id;

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &response,
                       sizeof(config_msg_sig_model_subscription_list_t), nrf_mesh_unique_token_get());
            break;
        }

        case CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET:
        {
            /* Build the subscription status packet: */
            config_msg_vendor_model_subscription_list_t response;

            memset(&response, 0, sizeof(config_msg_vendor_model_subscription_list_t));
            response.status = status_opcode;

            const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_incoming->p_data;
            response.element_address = p_pdu->element_address;
            response.vendor_model_id = p_pdu->model_id.vendor.model_id;
            response.vendor_company_id = p_pdu->model_id.vendor.company_id;

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &response,
                       sizeof(config_msg_vendor_model_subscription_list_t), nrf_mesh_unique_token_get());
            break;
        }

    }
}

static access_status_t delete_all_subscriptions(access_model_handle_t model_handle)
{
    /* Get a list of all the models: */
    dsm_handle_t subscribed_addresses[ACCESS_SUBSCRIPTION_LIST_COUNT];
    uint16_t subscription_count = ACCESS_SUBSCRIPTION_LIST_COUNT;
    uint32_t status = access_model_subscriptions_get(model_handle, subscribed_addresses, &subscription_count);
    if (status == NRF_ERROR_NOT_SUPPORTED)
    {
        return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
    }
    NRF_MESH_ASSERT(status == NRF_SUCCESS);

    for (uint16_t i = 0; i < subscription_count; ++i)
    {
        NRF_MESH_ASSERT(access_model_subscription_remove(model_handle, subscribed_addresses[i]) == NRF_SUCCESS);
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscribed_addresses[i]) == NRF_SUCCESS);
    }

    return ACCESS_STATUS_SUCCESS;
}

/* Sends the subscription status in response to subscription messages. */
static void send_subscription_status(access_model_handle_t this_handle, const access_message_rx_t * p_message,
        uint16_t element_address, uint16_t subscription_address, config_model_id_t model_id, bool sig_model)
{
    config_msg_subscription_status_t packet;

    packet.status = ACCESS_STATUS_SUCCESS;
    packet.element_address = element_address;
    packet.address = subscription_address;
    packet.model_id = model_id;

    send_reply(this_handle, p_message, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &packet,
            PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model), nrf_mesh_unique_token_get());
}

static void send_netkey_status(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               access_status_t status,
                               config_msg_key_index_12_t key_index)
{
    const config_msg_netkey_status_t response = {status, key_index};
    send_reply(handle, p_message, CONFIG_OPCODE_NETKEY_STATUS,
               (const uint8_t *) &response, sizeof(response), nrf_mesh_unique_token_get());
}

static uint16_t get_element_index(uint16_t element_address)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if (element_address < node_address.address_start)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }

    uint16_t retval = element_address - node_address.address_start;
    if (retval >= (uint16_t) ACCESS_ELEMENT_COUNT)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }
    else
    {
        return retval;
    }
}

static inline access_status_t get_subscription_list(access_model_handle_t model_handle, uint16_t * p_sublist, uint16_t * p_subcount)
{
    uint32_t status = access_model_subscriptions_get(model_handle, p_sublist, p_subcount);
    switch (status)
    {
        case NRF_SUCCESS:
        {
            /* Resolve address handles to actual addresses */
            nrf_mesh_address_t addr;
            for (uint32_t i = 0; i < *p_subcount; i++)
            {
                if (dsm_address_get(*p_sublist, &addr) != NRF_SUCCESS)
                {
                    return ACCESS_STATUS_UNSPECIFIED_ERROR;
                }
                else
                {
                    if (addr.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL ||
                        addr.type == NRF_MESH_ADDRESS_TYPE_GROUP)
                    {
                        *(p_sublist++) = addr.value;
                    }
                    else
                    {
                        return ACCESS_STATUS_UNSPECIFIED_ERROR;
                    }
                }
            }
            return ACCESS_STATUS_SUCCESS;
        }
        case NRF_ERROR_NOT_SUPPORTED:
            return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
        case NRF_ERROR_NOT_FOUND:
            return ACCESS_STATUS_INVALID_MODEL;
        default:
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
    }
}

static uint32_t config_server_heartbeat_publication_params_get(heartbeat_publication_information_t * p_pub_info)
{

    const heartbeat_publication_state_t * p_hb_pub = heartbeat_publication_get();
    const nrf_mesh_network_secmat_t *p_net_secmat = NULL;
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if (dsm_net_secmat_from_keyindex_get(p_hb_pub->netkey_index, &p_net_secmat) == NRF_SUCCESS)
    {
        p_pub_info->p_net_secmat  = p_net_secmat;
        p_pub_info->local_address = node_address.address_start;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

/********** Opcode handler functions **********/
static void handle_appkey_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_add_t))
    {
        return;
    }

    const config_msg_appkey_add_t * p_pdu = (const config_msg_appkey_add_t *) p_message->p_data;
    config_msg_key_index_24_t key_indexes = p_pdu->key_indexes;

    uint16_t netkey_index, appkey_index;
    config_msg_key_index_24_get(&key_indexes, &netkey_index, &appkey_index);

    uint32_t status;
    access_status_t status_code;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));

    dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
    evt.type = CONFIG_SERVER_EVT_APPKEY_ADD;
    status = dsm_appkey_add(appkey_index, network_handle, p_pdu->appkey, &evt.params.appkey_add.appkey_handle);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "dsm_appkey_add(appkey_handle:%X appkey_index:%X)\n", evt.params.appkey_add.appkey_handle, appkey_index );

    switch (status)
    {
        case NRF_SUCCESS:
            status_code = ACCESS_STATUS_SUCCESS;
            break;
        case NRF_ERROR_INTERNAL:
        {
            dsm_handle_t related_subnet;
            NRF_MESH_ERROR_CHECK(dsm_appkey_handle_to_subnet_handle(evt.params.appkey_add.appkey_handle,
                                                                    &related_subnet));
            status_code = related_subnet == network_handle ? ACCESS_STATUS_SUCCESS :
                                                             ACCESS_STATUS_INVALID_NETKEY;
            break;
        }
        case NRF_ERROR_INVALID_PARAM:
            status_code = ACCESS_STATUS_INVALID_APPKEY;
            break;
        case NRF_ERROR_FORBIDDEN:
            status_code = ACCESS_STATUS_KEY_INDEX_ALREADY_STORED;
            break;
        case NRF_ERROR_NO_MEM:
            status_code = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
            break;
        case NRF_ERROR_NOT_FOUND:
            status_code = ACCESS_STATUS_INVALID_NETKEY;
            break;
        default:
            status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_appkey_status(handle, p_message, status_code, key_indexes);

    if (status == NRF_SUCCESS)
    {
        app_evt_send(&evt);
    }
}


static void handle_appkey_update(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_update_t))
    {
        return;
    }

    const config_msg_appkey_update_t * p_pdu = (const config_msg_appkey_update_t *) p_message->p_data;
    config_msg_key_index_24_t key_indexes = p_pdu->key_indexes;

    uint16_t netkey_index, appkey_index;
    config_msg_key_index_24_get(&key_indexes, &netkey_index, &appkey_index);

    access_status_t status_code = ACCESS_STATUS_SUCCESS;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_APPKEY_UPDATE;


    dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
    if (DSM_HANDLE_INVALID == network_handle)
    {
        status_code = ACCESS_STATUS_INVALID_NETKEY;
    }
    else
    {
        evt.params.appkey_update.appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);
        if (DSM_HANDLE_INVALID == evt.params.appkey_update.appkey_handle)
        {
            status_code = ACCESS_STATUS_INVALID_APPKEY;
        }
    }

    if (ACCESS_STATUS_SUCCESS == status_code)
    {
        dsm_handle_t related_subnet;
        NRF_MESH_ERROR_CHECK(dsm_appkey_handle_to_subnet_handle(evt.params.appkey_update.appkey_handle,
                                                                &related_subnet));
        if (network_handle != related_subnet)
        {
            status_code = ACCESS_STATUS_INVALID_BINDING;
        }
    }

    if (ACCESS_STATUS_SUCCESS == status_code)
    {

        uint32_t status = dsm_appkey_update(evt.params.appkey_update.appkey_handle, p_pdu->appkey);

        switch (status)
        {
            case NRF_SUCCESS:
                break;
            case NRF_ERROR_INVALID_STATE:
                status_code = ACCESS_STATUS_CANNOT_UPDATE;
                break;
            case NRF_ERROR_NOT_FOUND:
                status_code = ACCESS_STATUS_INVALID_APPKEY;
                break;
            default:
                status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
    }

    send_appkey_status(handle, p_message, status_code, key_indexes);

    if (ACCESS_STATUS_SUCCESS == status_code)
    {
        app_evt_send(&evt);
    }
}

static void handle_appkey_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_delete_t))
    {
        return;
    }

    const config_msg_appkey_delete_t * p_pdu = (const config_msg_appkey_delete_t *) p_message->p_data;
    config_msg_key_index_24_t key_indexes = p_pdu->key_indexes;

    access_status_t status_code = ACCESS_STATUS_SUCCESS;
    uint16_t netkey_index, appkey_index;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_APPKEY_DELETE;
    config_msg_key_index_24_get(&key_indexes, &netkey_index, &appkey_index);
    evt.params.appkey_delete.appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);

    if (DSM_HANDLE_INVALID != evt.params.appkey_delete.appkey_handle)
    {
        dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
        if (DSM_HANDLE_INVALID == network_handle)
        {
            status_code = ACCESS_STATUS_INVALID_NETKEY;
        }

        if (ACCESS_STATUS_SUCCESS == status_code)
        {
            dsm_handle_t related_subnet;
            NRF_MESH_ERROR_CHECK(dsm_appkey_handle_to_subnet_handle(evt.params.appkey_delete.appkey_handle,
                                                                    &related_subnet));
            if (network_handle == related_subnet)
            {
                NRF_MESH_ERROR_CHECK(access_model_publication_by_appkey_stop(evt.params.appkey_delete.appkey_handle));
                NRF_MESH_ERROR_CHECK(dsm_appkey_delete(evt.params.appkey_delete.appkey_handle));
            }
            else
            {
                status_code = ACCESS_STATUS_INVALID_BINDING;
            }
        }
    }

    send_appkey_status(handle, p_message, status_code, key_indexes);

    if (ACCESS_STATUS_SUCCESS == status_code &&
        DSM_HANDLE_INVALID != evt.params.appkey_delete.appkey_handle)
    {
        app_evt_send(&evt);
    }
}

static void handle_appkey_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_get_t))
    {
        return;
    }

    const config_msg_appkey_get_t * p_pdu = (const config_msg_appkey_get_t *) p_message->p_data;
    uint16_t netkey_index = p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    mesh_key_index_t appkeys[DSM_APP_MAX];
    uint32_t appkey_count = DSM_APP_MAX;
    uint32_t status = dsm_appkey_get_all(dsm_net_key_index_to_subnet_handle(netkey_index),
        appkeys, &appkey_count);

    uint8_t packet_buffer[sizeof(config_msg_appkey_list_t) + PACKED_INDEX_LIST_SIZE(DSM_APP_MAX)];
    uint16_t reply_length = sizeof(config_msg_appkey_list_t);
    config_msg_appkey_list_t * p_reply = (config_msg_appkey_list_t *) packet_buffer;
    p_reply->netkey_index = p_pdu->netkey_index;

    switch (status)
    {
        case NRF_SUCCESS:
            p_reply->status = ACCESS_STATUS_SUCCESS;
            reply_length += PACKED_INDEX_LIST_SIZE(appkey_count);
            packed_index_list_create(appkeys, (uint8_t *) p_reply->packed_appkey_indexes, appkey_count);
            break;
        case NRF_ERROR_NOT_FOUND:
            p_reply->status = ACCESS_STATUS_INVALID_NETKEY;
            break;
        default:
            p_reply->status = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }
    send_reply(handle, p_message, CONFIG_OPCODE_APPKEY_LIST, packet_buffer, reply_length, nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_APPKEY_GET;
    evt.params.appkey_get.netkey_index = p_pdu->netkey_index;

    app_evt_send(&evt);
}

static void handle_config_beacon_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_net_beacon_set_t))
    {
        return;
    }

    const config_msg_net_beacon_set_t * p_pdu = (const config_msg_net_beacon_set_t *) p_message->p_data;
    if (p_pdu->beacon_state == CONFIG_NET_BEACON_STATE_ENABLED ||
        p_pdu->beacon_state == CONFIG_NET_BEACON_STATE_DISABLED)
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_BEACON_SET;
        evt.params.beacon_set.beacon_state = (config_net_beacon_state_t)p_pdu->beacon_state;

        net_beacon_state_set(evt.params.beacon_set.beacon_state);
        send_net_beacon_state(handle, p_message);
        app_evt_send(&evt);
    }
}

static void handle_config_beacon_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length == 0)
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_BEACON_GET;

        send_net_beacon_state(handle, p_message);
        app_evt_send(&evt);
    }
}

static void handle_composition_data_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_composition_data_get_t))
    {
        return;
    }

    const config_msg_composition_data_get_t * p_pdu = (const config_msg_composition_data_get_t *) p_message->p_data;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_COMPOSITION_DATA_GET;
    evt.params.composition_data_get.page_number = p_pdu->page_number;

    uint8_t buffer[sizeof(config_msg_composition_data_status_t) + CONFIG_COMPOSITION_DATA_SIZE];
    config_msg_composition_data_status_t * p_response = (config_msg_composition_data_status_t *) buffer;
    p_response->page_number = 0;
    uint16_t size = CONFIG_COMPOSITION_DATA_SIZE;
    config_composition_data_get(p_response->data, &size);

    send_reply(handle, p_message, CONFIG_OPCODE_COMPOSITION_DATA_STATUS, buffer,
               sizeof(config_msg_composition_data_status_t) + size, nrf_mesh_unique_token_get());

    app_evt_send(&evt);
}

static void handle_config_default_ttl_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

    uint8_t ttl = access_default_ttl_get();

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_DEFAULT_TTL_GET;
    app_evt_send(&evt);
    send_reply(handle, p_message, CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t *) &ttl, sizeof(ttl), nrf_mesh_unique_token_get());
}

static void handle_config_default_ttl_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_default_ttl_set_t * p_pdu = (const config_msg_default_ttl_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_default_ttl_set_t))
    {
        return;
    }

    uint8_t ttl = p_pdu->ttl;
    if (access_default_ttl_set(ttl) == NRF_SUCCESS)
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_DEFAULT_TTL_SET;
        evt.params.default_ttl_set.default_ttl = ttl;
        app_evt_send(&evt);

        send_reply(handle, p_message, CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t *) &ttl, sizeof(ttl), nrf_mesh_unique_token_get());
    }
}

static void handle_config_friend_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }
    config_msg_friend_status_t status_message = { .friend_state = CONFIG_FRIEND_STATE_UNSUPPORTED };

#if MESH_FEATURE_FRIEND_ENABLED
    status_message.friend_state = (mesh_friend_is_enabled() ?
                                   CONFIG_FRIEND_STATE_SUPPORTED_ENABLED :
                                   CONFIG_FRIEND_STATE_SUPPORTED_DISABLED);
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_FRIEND_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_FRIEND_GET;
    app_evt_send(&evt);
}

static void handle_config_friend_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_friend_set_t * p_pdu = (const config_msg_friend_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_friend_set_t) ||
        p_pdu->friend_state >= CONFIG_FRIEND_STATE_UNSUPPORTED)
    {
        return;
    }

    config_msg_friend_status_t status_message = { .friend_state = CONFIG_FRIEND_STATE_UNSUPPORTED };

#if MESH_FEATURE_FRIEND_ENABLED
    switch (p_pdu->friend_state)
    {
        case CONFIG_FRIEND_STATE_SUPPORTED_DISABLED:
            mesh_friend_disable();
            break;

        case CONFIG_FRIEND_STATE_SUPPORTED_ENABLED:
            mesh_friend_enable();
            break;

        default:
            NRF_MESH_ASSERT(false); /* Input checked above. */
            break;
    }
    status_message.friend_state = (mesh_friend_is_enabled() ?
                                   CONFIG_FRIEND_STATE_SUPPORTED_ENABLED :
                                   CONFIG_FRIEND_STATE_SUPPORTED_DISABLED);
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_FRIEND_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_FRIEND_SET;
    evt.params.friend_set.friend_state = (config_friend_state_t) status_message.friend_state;
    app_evt_send(&evt);
}

static void handle_config_gatt_proxy_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

#if MESH_FEATURE_GATT_PROXY_ENABLED
    const config_msg_proxy_status_t status_message = {
        .proxy_state = (proxy_is_enabled() ? CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED
                        : CONFIG_GATT_PROXY_STATE_RUNNING_DISABLED)};
#else
    const config_msg_proxy_status_t status_message = { .proxy_state = CONFIG_GATT_PROXY_STATE_UNSUPPORTED };
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_GATT_PROXY_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_GATT_PROXY_GET;
    app_evt_send(&evt);
}

static void handle_config_gatt_proxy_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_proxy_set_t * p_pdu = (const config_msg_proxy_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_proxy_set_t) ||
        p_pdu->proxy_state >= CONFIG_GATT_PROXY_STATE_UNSUPPORTED)
    {
        return;
    }

#if MESH_FEATURE_GATT_PROXY_ENABLED
    bool enabled = p_pdu->proxy_state == CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED;
    uint32_t err_code = mesh_opt_gatt_proxy_set(enabled);
    NRF_MESH_ASSERT_DEBUG(err_code == NRF_SUCCESS || err_code == NRF_ERROR_INVALID_STATE);

    (void) proxy_start();

    const config_msg_proxy_status_t status_message = {
        .proxy_state = (proxy_is_enabled() ? CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED
                        : CONFIG_GATT_PROXY_STATE_RUNNING_DISABLED)};
#else
    const config_msg_proxy_status_t status_message = { .proxy_state = CONFIG_GATT_PROXY_STATE_UNSUPPORTED };
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_GATT_PROXY_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_GATT_PROXY_SET;
    evt.params.proxy_set.proxy_state = (config_gatt_proxy_state_t)p_pdu->proxy_state;
    app_evt_send(&evt);
}

static void handle_config_key_refresh_phase_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_key_refresh_phase_get_t))
    {
        return;
    }

    const config_msg_key_refresh_phase_get_t * p_pdu = (const config_msg_key_refresh_phase_get_t *) p_message->p_data;

    nrf_mesh_key_refresh_phase_t kr_phase;
    dsm_handle_t subnet_handle = dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK);
    uint32_t status = dsm_subnet_kr_phase_get(subnet_handle, &kr_phase);
    config_msg_key_refresh_phase_status_t status_message = { .netkey_index = p_pdu->netkey_index };
    if (status == NRF_SUCCESS)
    {
        status_message.status = ACCESS_STATUS_SUCCESS;
        status_message.phase = (uint8_t) kr_phase;
    }
    else
    {
        status_message.status = ACCESS_STATUS_INVALID_NETKEY;
        status_message.phase = 0x00;
    }
    send_reply(handle, p_message, CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_KEY_REFRESH_PHASE_GET;
    evt.params.key_refresh_phase_get.subnet_handle = subnet_handle;
    app_evt_send(&evt);
}

static void handle_config_key_refresh_phase_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_key_refresh_phase_set_t))
    {
        return;
    }

    const config_msg_key_refresh_phase_set_t * p_pdu = (const config_msg_key_refresh_phase_set_t *) p_message->p_data;
    config_msg_key_refresh_phase_status_t status_message =
    {
        .netkey_index = p_pdu->netkey_index,
        .phase = 0x00,
        .status = ACCESS_STATUS_INVALID_NETKEY, /* According to the specification, this is the only possible error status code for the KR state. */
    };

    /* Get the current key refresh phase: */
    nrf_mesh_key_refresh_phase_t current_kr_phase;
    dsm_handle_t subnet_handle = dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK);
    uint32_t status = dsm_subnet_kr_phase_get(subnet_handle, &current_kr_phase);
    if (status == NRF_SUCCESS)
    {
        switch (p_pdu->transition)
        {
            case KEY_REFRESH_TRANSITION_TO_PHASE_2:
                if (current_kr_phase == NRF_MESH_KEY_REFRESH_PHASE_2 || current_kr_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
                {
                    if (current_kr_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
                    {
                        NRF_MESH_ERROR_CHECK(dsm_subnet_update_swap_keys(subnet_handle));
                    }
                    status_message.status = ACCESS_STATUS_SUCCESS;
                    status_message.phase = NRF_MESH_KEY_REFRESH_PHASE_2;
                    break;
                }
                return;
            case KEY_REFRESH_TRANSITION_TO_PHASE_3:
                if (current_kr_phase != NRF_MESH_KEY_REFRESH_PHASE_3)
                {
                    if (current_kr_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
                    {
                        NRF_MESH_ERROR_CHECK(dsm_subnet_update_commit(subnet_handle));
                    }

                    status_message.status = ACCESS_STATUS_SUCCESS;
                    status_message.phase = NRF_MESH_KEY_REFRESH_PHASE_0;
                    break;
                }
                return;
            default:
                return;
        }
    }

    send_reply(handle, p_message, CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    if (status_message.status == ACCESS_STATUS_SUCCESS)
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_KEY_REFRESH_PHASE_SET;
        evt.params.key_refresh_phase_set.kr_phase = (nrf_mesh_key_refresh_phase_t) status_message.phase;
        evt.params.key_refresh_phase_set.subnet_handle = subnet_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_publication_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_get_t, p_message))
    {
        return;
    }

    const config_msg_publication_get_t * p_pdu = (const config_msg_publication_get_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_publication_get_t));

    /* Get element index from element address */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
    }
    else
    {
        send_publication_status(handle, p_message, p_pdu->element_address, model_handle);

        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_MODEL_PUBLICATION_GET;
        evt.params.publication_get.model_handle = model_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_set_t, p_message))
            ||
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_virtual_set_t, p_message))
       )
    {
        return;
    }

    bool sig_model;
    access_model_id_t model_id;

    uint16_t element_address, publish_address = NRF_MESH_ADDR_UNASSIGNED;
    const config_publication_params_t * p_pubstate;

    /* Extract fields that are different (or placed at different offsets) based on the incoming opcode: */
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        const config_msg_publication_set_t * p_pdu = (const config_msg_publication_set_t *) p_message->p_data;
        sig_model = model_id_extract(&model_id, &p_pdu->state.model_id, p_message, sizeof(config_msg_publication_set_t));
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
        publish_address = p_pdu->publish_address;
    }
    else
    {
        const config_msg_publication_virtual_set_t * p_pdu = (const config_msg_publication_virtual_set_t *) p_message->p_data;
        sig_model = model_id_extract(&model_id, &p_pdu->state.model_id, p_message, sizeof(config_msg_publication_virtual_set_t));
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
    }

    uint16_t element_index = get_element_index(element_address);

    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Get the application key handle for the application key to publish on: */
    dsm_handle_t publish_appkey_handle = dsm_appkey_index_to_appkey_handle(p_pubstate->appkey_index);
    if (publish_appkey_handle == DSM_HANDLE_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_APPKEY);
        return;
    }

    /* Validate and add the publish address to the DSM: */
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t publish_address_stored;
    nrf_mesh_address_type_t publish_addr_type = nrf_mesh_address_type_get(publish_address);
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        if (publish_addr_type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
        {
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
            return;
        }
        else if (publish_addr_type == NRF_MESH_ADDRESS_TYPE_UNICAST || publish_addr_type == NRF_MESH_ADDRESS_TYPE_GROUP)
        {
            /* Check if given publish address is different than the currently assigned address */
            if (access_model_publish_address_get(model_handle, &publish_address_handle) != NRF_SUCCESS)
            {
                status = dsm_address_publish_add(publish_address, &publish_address_handle);
            }
            else
            {
                if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
                {

                    if ((publish_address_stored.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL) ||
                        (publish_address_stored.type != NRF_MESH_ADDRESS_TYPE_VIRTUAL  &&
                         publish_address_stored.value != publish_address))
                    {
                        /* This should never assert */
                        NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
                        status = dsm_address_publish_add(publish_address, &publish_address_handle);
                    }
                    else
                    {
                        /* Use the retrieved publish_address_handle */
                    }
                }
                else
                {
                    status = dsm_address_publish_add(publish_address, &publish_address_handle);
                }
            }
        }
    }
    else
    {
        const uint8_t * publish_address_uuid = ((const config_msg_publication_virtual_set_t *) p_message->p_data)->publish_uuid;

        /* Check if given publish address is different than the currently assigned address */
        if (access_model_publish_address_get(model_handle, &publish_address_handle) != NRF_SUCCESS)
        {
            status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
        }
        else
        {
            if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
            {

                if ((publish_address_stored.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
                     memcmp(publish_address_stored.p_virtual_uuid, publish_address_uuid, NRF_MESH_UUID_SIZE) != 0) ||
                    (publish_address_stored.type != NRF_MESH_ADDRESS_TYPE_VIRTUAL))
                {
                    /* This should never assert */
                    NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
                    status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
                }
                else
                {
                    /* Use the retrieved publish_address_handle */
                }
            }
            else
            {
                status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
            }
        }
    }

    switch (status)
    {
        case NRF_ERROR_NO_MEM:
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INSUFFICIENT_RESOURCES);
            return;

        case NRF_SUCCESS:
            break;

        default:
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            return;
    }

    /* If publish address is unassigned for non virtual set, ignore all incoming parameters */
    if (publish_address != NRF_MESH_ADDR_UNASSIGNED || p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET)
    {
        access_publish_period_t publish_period;
        access_publish_retransmit_t publish_retransmit;
        publish_period.step_res = p_pubstate->publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS;
        publish_period.step_num = p_pubstate->publish_period & ~(0xff << ACCESS_PUBLISH_STEP_NUM_BITS);
        publish_retransmit.count = p_pubstate->retransmit_count;
        publish_retransmit.interval_steps = p_pubstate->retransmit_interval;


        /* Disable publishing for the model while updating the publication parameters: */
        status = access_model_publish_period_set(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0);
        switch (status)
        {
            case NRF_SUCCESS:
                break;

            case NRF_ERROR_NOT_SUPPORTED:
                /* Setting publish period when not supported, is an error */
                if (publish_period.step_num != 0)
                {
                    status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_PUBLISH_MODEL);
                    return;
                }
                break;

            default:
                /* No other error should be possible. */
                NRF_MESH_ASSERT(false);
                return;
        }

#if MESH_FEATURE_LPN_ENABLED
        status = access_model_publish_friendship_credential_flag_set(model_handle, p_pubstate->credential_flag);
#else
        if (!!p_pubstate->credential_flag)
        {
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_FEATURE_NOT_SUPPORTED);
            return;
        }
        status = NRF_SUCCESS;
#endif

        status = status == NRF_SUCCESS ? access_model_publish_retransmit_set(model_handle, publish_retransmit) : status;
        status = status == NRF_SUCCESS ? access_model_publish_address_set(model_handle, publish_address_handle) : status;
        status = status == NRF_SUCCESS ? access_model_publish_application_set(model_handle, publish_appkey_handle) : status;
        status = status == NRF_SUCCESS ? access_model_publish_ttl_set(model_handle, p_pubstate->publish_ttl) : status;

        if (status != NRF_SUCCESS)
        {
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            return;
        }

        if (publish_period.step_num != 0)
        {
            /* Set publishing parameters for the model: */
            NRF_MESH_ASSERT(access_model_publish_period_set(model_handle, (access_publish_resolution_t) publish_period.step_res,
                            publish_period.step_num) == NRF_SUCCESS);
        }
    }
    else
    {
        NRF_MESH_ASSERT(access_model_publication_stop(model_handle) == NRF_SUCCESS);
    }

    send_publication_status(handle, p_message, element_address, model_handle);

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET) ? 
                 CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET : CONFIG_SERVER_EVT_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET;
    evt.params.model_publication_set.model_handle = model_handle;
    app_evt_send(&evt);  
}

static void handle_config_model_subscription_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if ((address_type != NRF_MESH_ADDRESS_TYPE_GROUP) || (p_pdu->address == NRF_MESH_ALL_NODES_ADDR))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Add the address to the DSM as a subscription address: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(p_pdu->address, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_INVALID_ADDR:
                access_status = ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        status_error_sub_send(handle, p_message, sig_model, access_status);
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);

        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
        }
        else
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        }
    }
    else
    {
        send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
                p_pdu->model_id, sig_model);
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD;
        evt.params.model_subscription_add.model_handle = model_handle;
        evt.params.model_subscription_add.address_handle = subscription_address_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_subscription_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if (address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Get the handle for the subscription address: */
    dsm_handle_t subscription_address_handle;
    nrf_mesh_address_t group_address = { NRF_MESH_ADDRESS_TYPE_GROUP, p_pdu->address, NULL };
    status = dsm_address_handle_get(&group_address, &subscription_address_handle);

    if (status == NRF_SUCCESS)
    {
         /* Remove the subscription from the model: */
        status = access_model_subscription_remove(model_handle, subscription_address_handle);
        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
            return;
        }
        else if (status != NRF_SUCCESS)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            return;
        }

        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
    }
    else if (status != NRF_ERROR_NOT_FOUND)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        return;
    }

    send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
            p_pdu->model_id, sig_model);
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE;
    evt.params.model_subscription_delete.model_handle = model_handle;
    evt.params.model_subscription_delete.address_handle = subscription_address_handle;
    app_evt_send(&evt);
}

static void handle_config_model_subscription_delete_all(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_delete_all_t, p_message))
    {
        return;
    }

    const config_msg_subscription_delete_all_t * p_pdu = (const config_msg_subscription_delete_all_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_delete_all_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, reply_status);
    }
    else
    {
        send_subscription_status(handle, p_message, p_pdu->element_address, NRF_MESH_ADDR_UNASSIGNED,
                p_pdu->model_id, sig_model);

        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE_ALL;
        evt.params.model_subscription_delete_all.model_handle = model_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_subscription_overwrite(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Check that the subscription address is valid before continuing: */
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(p_pdu->address);
    if (address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Delete the old subscription list: */
    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, reply_status);
        return;
    }

    /* Add the new address to the DSM: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(p_pdu->address, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_INVALID_ADDR:
                access_status = ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        status_error_sub_send(handle, p_message, sig_model, access_status);
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
        }
        else
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        }
    }
    else
    {
        send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
                p_pdu->model_id, sig_model);

        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_OVERWRITE;
        evt.params.model_subscription_overwrite.model_handle = model_handle;
        evt.params.model_subscription_overwrite.address_handle = subscription_address_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_subscription_virtual_address_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_virtual_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Check if the address exists in the DSM, and if not, add it: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_virtual_add(p_pdu->virtual_uuid, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        status_error_sub_send(handle, p_message, sig_model, access_status);
        return;
    }

    /* Add the subscription to the model: */
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
        if (status == NRF_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
        }
        else
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        }
    }
    else
    {
        nrf_mesh_address_t target_address;
        NRF_MESH_ASSERT(dsm_address_get(subscription_address_handle, &target_address) == NRF_SUCCESS);
        send_subscription_status(handle, p_message, p_pdu->element_address,
                target_address.value, p_pdu->model_id, sig_model);

        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD;
        evt.params.model_subscription_add.model_handle = model_handle;
        evt.params.model_subscription_add.address_handle = subscription_address_handle;
        app_evt_send(&evt);
    }
}

static void handle_config_model_subscription_virtual_address_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_virtual_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Get the 16-bit address from the virtual UUID: */
    dsm_handle_t subscription_address_handle;
    nrf_mesh_address_t virtual_address = { NRF_MESH_ADDRESS_TYPE_VIRTUAL, NRF_MESH_ADDR_UNASSIGNED, p_pdu->virtual_uuid };
    NRF_MESH_ASSERT(nrf_mesh_keygen_virtual_address(p_pdu->virtual_uuid, &virtual_address.value) == NRF_SUCCESS);

    /* Get the address handle from the DSM: */
    status = dsm_address_handle_get(&virtual_address, &subscription_address_handle);
    if (status == NRF_ERROR_NOT_FOUND)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }
    else if (status != NRF_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        return;
    }

    /* Remove the subscription: */
    status = access_model_subscription_remove(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_CANNOT_REMOVE);
        return;
    }

    NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
    send_subscription_status(handle, p_message, p_pdu->element_address, virtual_address.value,
            p_pdu->model_id, sig_model);

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE;
    evt.params.model_subscription_delete.model_handle = model_handle;
    evt.params.model_subscription_delete.address_handle = subscription_address_handle;
    app_evt_send(&evt);
}

static void handle_config_model_subscription_virtual_address_overwrite(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_virtual_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_virtual_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }


    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Delete the old subscription list: */
    access_status_t reply_status = delete_all_subscriptions(model_handle);
    if (reply_status != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, reply_status);
        return;
    }

    /* Get the 16-bit address from the virtual UUID: */
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_virtual_add(p_pdu->virtual_uuid, &subscription_address_handle);
    if (status != NRF_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case NRF_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        status_error_sub_send(handle, p_message, sig_model, access_status);
    }
    else
    {
        /* Add the subscription to the model: */
        status = access_model_subscription_add(model_handle, subscription_address_handle);
        if (status != NRF_SUCCESS)
        {
            NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);
            if (status == NRF_ERROR_NOT_SUPPORTED)
            {
                status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
            }
            else
            {
                status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            }
        }
        else
        {
            nrf_mesh_address_t target_address;
            NRF_MESH_ASSERT(dsm_address_get(subscription_address_handle, &target_address) == NRF_SUCCESS);
            send_subscription_status(handle, p_message, p_pdu->element_address,
                    target_address.value, p_pdu->model_id, sig_model);

            config_server_evt_t evt;
            memset(&evt, 0, sizeof(config_server_evt_t));
            evt.type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE;
            evt.params.model_subscription_overwrite.model_handle = model_handle;
            evt.params.model_subscription_overwrite.address_handle = subscription_address_handle;
            app_evt_send(&evt);
        }
    }
}

static void send_relay_status(access_model_handle_t handle, const access_message_rx_t * p_message)
{
    config_msg_relay_status_t status_message = { 0 };

#if MESH_FEATURE_RELAY_ENABLED
    mesh_opt_core_adv_t relay;
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &relay));
    status_message.relay_state = (relay.enabled ?
                                  CONFIG_RELAY_STATE_SUPPORTED_ENABLED :
                                  CONFIG_RELAY_STATE_SUPPORTED_DISABLED);

    status_message.relay_retransmit_count = relay.tx_count - 1;
    if ((relay.tx_interval_ms == BEARER_ADV_INT_MIN_MS) &&
        (status_message.relay_retransmit_count == 0))
    {
        status_message.relay_retransmit_interval_steps = 0;
    }
    else
    {
        status_message.relay_retransmit_interval_steps =
            CONFIG_RETRANSMIT_INTERVAL_MS_TO_STEP(relay.tx_interval_ms) - 1;
    }
#else
    status_message.relay_state = CONFIG_RELAY_STATE_UNSUPPORTED;
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_RELAY_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());
}

static void handle_config_relay_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0) /* Ignore messages with invalid length */
    {
        return;
    }

    send_relay_status(handle, p_message);

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_RELAY_GET;
    app_evt_send(&evt);
}

static void handle_config_relay_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_relay_set_t * p_pdu = (const config_msg_relay_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_relay_set_t) ||
        p_pdu->relay_state >= CONFIG_RELAY_STATE_UNSUPPORTED)
    {
        return;
    }

#if MESH_FEATURE_RELAY_ENABLED
    mesh_opt_core_adv_t relay_state;

    relay_state.enabled = (p_pdu->relay_state == CONFIG_RELAY_STATE_SUPPORTED_ENABLED);

    /**
     * According to @tagMeshSp : "The Relay Retransmit Count + 1 is the number of times that
     * packet is transmitted for each packet that is relayed."
     */
    relay_state.tx_count = p_pdu->relay_retransmit_count + 1;

    if ((p_pdu->relay_retransmit_count == 0) && (p_pdu->relay_retransmit_interval_steps == 0))
    {
        /* No retransmits. Setting advertising interval to minimum to send first relay packet
         * (if any) as soon as possible */
        relay_state.tx_interval_ms = BEARER_ADV_INT_MIN_MS;
    }
    else
    {
        relay_state.tx_interval_ms = CONFIG_RETRANSMIT_INTERVAL_STEP_TO_MS(p_pdu->relay_retransmit_interval_steps + 1);
    }

    relay_state.tx_interval_ms = MAX(BEARER_ADV_INT_MIN_MS, relay_state.tx_interval_ms);
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY,
                                               &relay_state));
#endif /* MESH_FEATURE_RELAY_ENABLED */

    send_relay_status(handle, p_message);

#if MESH_FEATURE_RELAY_ENABLED
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_RELAY_SET;
    evt.params.relay_set.relay_state = (config_relay_state_t)p_pdu->relay_state;
    evt.params.relay_set.retransmit_count = p_pdu->relay_retransmit_count;
    evt.params.relay_set.interval_steps = p_pdu->relay_retransmit_interval_steps;
    app_evt_send(&evt);
#endif /* MESH_FEATURE_RELAY_ENABLED */
}

static config_msg_network_transmit_status_t net_transmit_status_get(void)
{
    config_msg_network_transmit_status_t net_status = { 0 };

    mesh_opt_core_adv_t net;
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR,
                                               &net));
    net_status.network_transmit_count = net.tx_count - 1;
    net_status.network_transmit_interval_steps =
        CONFIG_RETRANSMIT_INTERVAL_MS_TO_STEP(net.tx_interval_ms) - 1;

    return net_status;
}

static void handle_config_network_transmit_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0) /* Ignore messages with invalid length */
    {
        return;
    }

    config_msg_network_transmit_status_t status_message = net_transmit_status_get();
    send_reply(handle, p_message, CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS,
            (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_NETWORK_TRANSMIT_GET;
    app_evt_send(&evt);
}


static void handle_config_network_transmit_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_network_transmit_set_t * p_pdu = (const config_msg_network_transmit_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_network_transmit_set_t))
    {
        return;
    }

    mesh_opt_core_adv_t net;

    /* The main advertiser should always be enabled. */
    net.enabled = true;

    /**
     * According to @tagMeshSp section 4.2.19: "The number of transmissions is the
     * Transmit Count + 1."
     */
    net.tx_count = p_pdu->network_transmit_count + 1;
    net.tx_interval_ms = MAX(BEARER_ADV_INT_MIN_MS,
                             CONFIG_RETRANSMIT_INTERVAL_STEP_TO_MS(p_pdu->network_transmit_interval_steps + 1));
    NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &net));

    config_msg_network_transmit_status_t status_message = net_transmit_status_get();
    send_reply(handle, p_message, CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS,
            (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());


    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_NETWORK_TRANSMIT_SET;
    evt.params.network_transmit_set.retransmit_count = status_message.network_transmit_count;
    evt.params.network_transmit_set.interval_steps = status_message.network_transmit_interval_steps;
    app_evt_send(&evt);
}

static void handle_config_sig_model_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_message->p_data;

    if (p_message->length != PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, true))
    {
        return;
    }

    /* Get the model handle: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_model_subscription_get_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    uint16_t subscription_count = DSM_ADDR_MAX;
    uint16_t subscription_list[DSM_ADDR_MAX];
    access_status_t error_code = get_subscription_list(model_handle, subscription_list, &subscription_count);
    if (error_code != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, error_code);
        return;
    }

    uint8_t response_buffer[sizeof(config_msg_sig_model_subscription_list_t) + DSM_ADDR_MAX * sizeof(uint16_t)];
    config_msg_sig_model_subscription_list_t * p_response = (config_msg_sig_model_subscription_list_t *) response_buffer;

    p_response->status = ACCESS_STATUS_SUCCESS;
    p_response->element_address = p_pdu->element_address;
    p_response->sig_model_id = model_id.model_id;
    memcpy((void *) p_response->subscriptions, subscription_list, subscription_count * sizeof(uint16_t));

    send_reply(handle, p_message, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, response_buffer,
            sizeof(config_msg_sig_model_subscription_list_t) + subscription_count * sizeof(uint16_t), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_SIG_MODEL_SUBSCRIPTION_GET;
    evt.params.model_subscription_get.model_handle = model_handle;
    app_evt_send(&evt);
}

static void handle_config_vendor_model_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_message->p_data;
    if (p_message->length != PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, false))
    {
        return;
    }

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    access_model_id_t model_id;
    access_model_handle_t model_handle;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_model_subscription_get_t));
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    uint16_t subscription_count = DSM_ADDR_MAX;
    uint16_t subscription_list[DSM_ADDR_MAX];
    access_status_t error_code = get_subscription_list(model_handle, subscription_list, &subscription_count);
    if (error_code != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, error_code);
        return;
    }

    const uint16_t subscription_list_size = subscription_count * sizeof(uint16_t);
    uint8_t response_buffer[sizeof(config_msg_vendor_model_subscription_list_t) + DSM_ADDR_MAX * sizeof(uint16_t)];
    config_msg_vendor_model_subscription_list_t * p_response = (config_msg_vendor_model_subscription_list_t *) response_buffer;

    p_response->status = ACCESS_STATUS_SUCCESS;
    p_response->element_address = p_pdu->element_address;
    p_response->vendor_model_id = p_pdu->model_id.vendor.model_id;
    p_response->vendor_company_id = p_pdu->model_id.vendor.company_id;
    memcpy((void *) p_response->subscriptions, subscription_list, subscription_list_size);

    send_reply(handle, p_message, CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, response_buffer,
            sizeof(config_msg_vendor_model_subscription_list_t) + subscription_list_size, nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_VENDOR_MODEL_SUBSCRIPTION_GET;
    evt.params.model_subscription_get.model_handle = model_handle;
    app_evt_send(&evt);
}

static inline uint32_t heartbeat_publication_count_decode(uint8_t count_log)
{
    if (count_log == 0x00)
    {
        return 0x00;
    }
    else if (count_log <= HEARTBEAT_MAX_COUNT_LOG)
    {
        return (1 << (count_log - 1));
    }
    else if (count_log == HEARTBEAT_INF_COUNT_LOG)
    {
        return HEARTBEAT_INF_COUNT;
    }
    else
    {
        return HEARTBEAT_INVALID_COUNT;
    }
}

static inline uint32_t heartbeat_pubsub_period_decode(uint8_t period_log)
{
    if (period_log == 0x00)
    {
        return 0x00;
    }
    else if (period_log <= HEARTBEAT_MAX_PERIOD_LOG)
    {
        return (1 << (period_log - 1));
    }
    else
    {
        return HEARTBEAT_INVALID_PERIOD;
    }
}

static inline uint8_t heartbeat_pubsub_period_encode(uint32_t period)
{
    if (period == 0)
    {
        return 0x00;
    }
    else if (period <= HEARTBEAT_MAX_PERIOD)
    {
        return (log2_get(period) + 1);
    }
    else
    {
        NRF_MESH_ASSERT(false);
        return HEARTBEAT_MAX_PERIOD_LOG;
    }
}

static inline uint8_t heartbeat_publication_count_encode(uint32_t count)
{
    if (count <= 1)
    {
        return count;
    }
    else if (count <= HEARTBEAT_MAX_COUNT)
    {
        /* Finding smallest n where 2^(n-1) is greater than or equal to the count value */
        return (log2_get(count - 1) + 1 + 1);
    }
    else if (count == HEARTBEAT_INF_COUNT)
    {
        return HEARTBEAT_INF_COUNT_LOG;
    }
    else
    {
        NRF_MESH_ASSERT(false);
        return HEARTBEAT_MAX_COUNT_LOG;
    }
}

static inline uint8_t heartbeat_subscription_count_encode(uint32_t count)
{
    if (count == 0)
    {
        return 0x00;
    }
    else if (count <= HEARTBEAT_MAX_COUNT)
    {
        /* Finding largest n where 2^(n-1) is less than or equal to the count value */
        return (log2_get(count) + 1);
    }
    else if (count == HEARTBEAT_INF_COUNT)
    {
        return HEARTBEAT_INF_COUNT_LOG;
    }
    else
    {
        NRF_MESH_ASSERT(false);
        return HEARTBEAT_MAX_COUNT_LOG;
    }
}

static void handle_heartbeat_publication_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_publication_status_t status_message;
    const heartbeat_publication_state_t * p_hb_pub = heartbeat_publication_get();

    /* Ignore messages with invalid length */
    if (p_message->length != 0)
    {
        return;
    }

    status_message.status       = ACCESS_STATUS_SUCCESS;
    status_message.destination  = p_hb_pub->dst;

    /* As per @tagMeshSp section 4.2.17.2 log(n) should be
    larger than or equal to n */
    status_message.count_log    = heartbeat_publication_count_encode(p_hb_pub->count);
    status_message.period_log   = heartbeat_pubsub_period_encode(p_hb_pub->period);
    status_message.ttl          = p_hb_pub->ttl;
    status_message.features     = p_hb_pub->features;
    status_message.netkey_index = p_hb_pub->netkey_index;

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS,
              (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_HEARTBEAT_PUBLICATION_GET;
    app_evt_send(&evt);
}

static void handle_heartbeat_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_publication_status_t status_message;
    const config_msg_heartbeat_publication_set_t *p_pdu = (config_msg_heartbeat_publication_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_heartbeat_publication_set_t))
    {
        return;
    }

    const heartbeat_publication_state_t hb_pub = {
        .dst          = p_pdu->destination,
        .count        = heartbeat_publication_count_decode(p_pdu->count_log),
        .period       = heartbeat_pubsub_period_decode(p_pdu->period_log),
        .ttl          = p_pdu->ttl,
        .features     = p_pdu->features & HEARTBEAT_TRIGGER_TYPE_RFU_MASK,
        .netkey_index = p_pdu->netkey_index
    };

    /* This is specifically required for INVALID_NETKEY status code */
    if (dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index) == DSM_HANDLE_INVALID)
    {
        status_message.status  = ACCESS_STATUS_INVALID_NETKEY;
    }
    else
    {
        status_message.status = (heartbeat_publication_set(&hb_pub) == NRF_SUCCESS)
                                    ? ACCESS_STATUS_SUCCESS
                                    : ACCESS_STATUS_CANNOT_SET;
    }

    status_message.destination  =  p_pdu->destination;
    status_message.period_log   =  p_pdu->period_log;
    status_message.count_log    =  p_pdu->count_log;
    status_message.ttl          =  p_pdu->ttl;
    status_message.features     =  p_pdu->features & HEARTBEAT_TRIGGER_TYPE_RFU_MASK;
    status_message.netkey_index =  p_pdu->netkey_index;

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS,
              (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    if (status_message.status == ACCESS_STATUS_SUCCESS)
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_HEARTBEAT_PUBLICATION_SET;
        evt.params.heartbeat_publication_set.p_publication_state = &hb_pub;
        app_evt_send(&evt);
    }
}

static void handle_heartbeat_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_subscription_status_t status_message;
    const heartbeat_subscription_state_t * p_hb_sub;

    /* Ignore messages with invalid length */
    if (p_message->length != 0)
    {
        return;
    }

    p_hb_sub = heartbeat_subscription_get();

    /* When the Heartbeat Subscription Source or Destination state is set to the unassigned address,
     the value of - the Source and Destination fields of the Status message shall be set to the
     unassigned address and the values of the CountLog, PeriodLog, MinHops, and MaxHops fields shall
     be set to 0x00. Refer to @tagMeshSp section 4.4.1.2.16 */
    status_message.status = ACCESS_STATUS_SUCCESS;
    if (p_hb_sub->src == NRF_MESH_ADDR_UNASSIGNED ||
        p_hb_sub->dst == NRF_MESH_ADDR_UNASSIGNED)
    {
        status_message.source      = NRF_MESH_ADDR_UNASSIGNED;
        status_message.destination = NRF_MESH_ADDR_UNASSIGNED;
        status_message.period_log  = 0x00;
        status_message.count_log   = 0x00;
        status_message.min_hops    = 0x00;
        status_message.max_hops    = 0x00;
    }
    else
    {
        status_message.source      = p_hb_sub->src;
        status_message.destination = p_hb_sub->dst;
        status_message.count_log   = heartbeat_subscription_count_encode(p_hb_sub->count);
        status_message.period_log  = heartbeat_pubsub_period_encode(p_hb_sub->period);
        status_message.min_hops    = p_hb_sub->min_hops;
        status_message.max_hops    = p_hb_sub->max_hops;
    }

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_HEARTBEAT_SUBSCRIPTION_GET;
    app_evt_send(&evt);
}

static void handle_heartbeat_subscription_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_subscription_status_t status_message;
    const config_msg_heartbeat_subscription_set_t *p_pdu = (config_msg_heartbeat_subscription_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_heartbeat_subscription_set_t))
    {
        return;
    }

    heartbeat_subscription_state_t new_subscription_state = {
        .src    = p_pdu->source,
        .dst    = p_pdu->destination,
        .period = heartbeat_pubsub_period_decode(p_pdu->period_log)
        /* other state values shall remain unchanged, see @tagMeshSp section 4.4.1.2.16 */
    };

    if (heartbeat_subscription_set(&new_subscription_state) != NRF_SUCCESS)
    {
        return;
    }

    const heartbeat_subscription_state_t * p_hb_sub = heartbeat_subscription_get();

    status_message.status = ACCESS_STATUS_SUCCESS;
    status_message.source = p_hb_sub->src;
    status_message.destination = p_hb_sub->dst;
    status_message.period_log = heartbeat_pubsub_period_encode(p_hb_sub->period);
    status_message.count_log = heartbeat_subscription_count_encode(p_hb_sub->count);
    status_message.min_hops = p_hb_sub->min_hops;
    status_message.max_hops = p_hb_sub->max_hops;

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_HEARTBEAT_SUBSCRIPTION_SET;
    evt.params.heartbeat_subscription_set.p_subscription_state = p_hb_sub;
    app_evt_send(&evt);
}

static void handle_model_app_bind_unbind(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_app_bind_unbind_t, p_message))
    {
        return;
    }

    const config_msg_app_bind_unbind_t * p_pdu = (const config_msg_app_bind_unbind_t *) p_message->p_data;
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_app_bind_unbind_t));

    config_msg_app_status_t response;
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = p_pdu->element_address;
    response.appkey_index = p_pdu->appkey_index;
    response.model_id = p_pdu->model_id;

    access_model_handle_t model_handle;

    if (sig_model && model_id.model_id == CONFIG_SERVER_MODEL_ID)
    {
        response.status = ACCESS_STATUS_CANNOT_BIND;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), nrf_mesh_unique_token_get());
        return;
    }

    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (ACCESS_ELEMENT_INDEX_INVALID == element_index)
    {
        response.status = ACCESS_STATUS_INVALID_ADDRESS;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), nrf_mesh_unique_token_get());
        return;
    }

    uint32_t status = access_handle_get(element_index, model_id, &model_handle);

    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Access  Info:\n\t\telement_index=%X\t\tmodel_id = %X-%X\t\tmodel_handle=%d\n",
                                           element_index, model_id.model_id, model_id.company_id, model_handle);

    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        response.status = ACCESS_STATUS_INVALID_MODEL;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), nrf_mesh_unique_token_get());
        return;
    }

    uint16_t appkey_index = p_pdu->appkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    dsm_handle_t appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));

    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_APP_BIND)
    {
        evt.type = CONFIG_SERVER_EVT_MODEL_APP_BIND;
        evt.params.model_app_bind.model_handle = model_handle;
        evt.params.model_app_bind.appkey_handle = appkey_handle;
        status = access_model_application_bind(model_handle, appkey_handle);
    }
    else
    {
        evt.type = CONFIG_SERVER_EVT_MODEL_APP_UNBIND;
        evt.params.model_app_unbind.model_handle = model_handle;
        evt.params.model_app_unbind.appkey_handle = appkey_handle;
        status = access_model_application_unbind(model_handle, appkey_handle);

        if (status == NRF_SUCCESS)
        {
            dsm_handle_t active_appkey_handle;
            NRF_MESH_ERROR_CHECK(access_model_publish_application_get(model_handle, &active_appkey_handle));
            if (active_appkey_handle == appkey_handle)
            {
                NRF_MESH_ERROR_CHECK(access_model_publication_stop(model_handle));
            }
        }
    }

    switch (status)
    {
        case NRF_SUCCESS:
            response.status = ACCESS_STATUS_SUCCESS;
            break;
        case NRF_ERROR_NOT_FOUND:
            response.status = ACCESS_STATUS_INVALID_MODEL;
            break;
        case NRF_ERROR_INVALID_PARAM:
            response.status = ACCESS_STATUS_INVALID_APPKEY;
            break;
        default:
            response.status = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), nrf_mesh_unique_token_get());

    if (response.status == ACCESS_STATUS_SUCCESS)
    {
        app_evt_send(&evt);
    }
}

static void handle_netkey_add_update(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_netkey_add_update_t * p_pdu = (const config_msg_netkey_add_update_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_netkey_add_update_t))
    {
        return;
    }

    uint16_t netkey_index = p_pdu->netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    uint32_t status;
    dsm_handle_t network_handle;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    if (p_message->opcode.opcode == CONFIG_OPCODE_NETKEY_ADD)
    {
        status = dsm_subnet_add(netkey_index, p_pdu->netkey, &network_handle);
        evt.type = CONFIG_SERVER_EVT_NETKEY_ADD;
        evt.params.netkey_add.netkey_handle = network_handle;
    }
    else
    {
        network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
        status = dsm_subnet_update(network_handle, p_pdu->netkey);
        evt.type = CONFIG_SERVER_EVT_NETKEY_UPDATE;
        evt.params.netkey_update.netkey_handle = network_handle;
    }

    access_status_t status_code;
    switch (status)
    {
        case NRF_SUCCESS:
        /* fall through */
        case NRF_ERROR_INTERNAL:
            status_code = ACCESS_STATUS_SUCCESS;
            break;
        case NRF_ERROR_INVALID_PARAM:
            status_code = ACCESS_STATUS_INVALID_NETKEY;
            break;
        case NRF_ERROR_FORBIDDEN:
            status_code = ACCESS_STATUS_KEY_INDEX_ALREADY_STORED;
            break;
        case NRF_ERROR_NO_MEM:
            status_code = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
            break;
        case NRF_ERROR_NOT_FOUND:
            status_code = ACCESS_STATUS_INVALID_NETKEY;
            break;
        case NRF_ERROR_INVALID_STATE:
            status_code = ACCESS_STATUS_CANNOT_UPDATE;
            break;
        default:
            status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_netkey_status(handle, p_message, status_code, p_pdu->netkey_index);

    if (NRF_SUCCESS == status)
    {
        app_evt_send(&evt);
    }
}

static void handle_netkey_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_netkey_delete_t * p_pdu = (const config_msg_netkey_delete_t *) p_message->p_data;
    config_msg_key_index_12_t netkey_index = p_pdu->netkey_index;

    if (p_message->length != sizeof(config_msg_netkey_delete_t))
    {
        return;
    }

    uint32_t status = NRF_SUCCESS;
    dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK);

    if (network_handle == p_message->meta_data.subnet_handle)
    {
        status = NRF_ERROR_FORBIDDEN;
    }

    if (DSM_HANDLE_INVALID != network_handle && NRF_SUCCESS == status)
    {
        uint16_t appkey_instances[DSM_APP_MAX];
        uint32_t counter = DSM_APP_MAX;

        if (NRF_SUCCESS == dsm_appkey_get_all(network_handle, appkey_instances, &counter))
        {
            for (uint32_t i = 0; i < counter; i++)
            {
                appkey_instances[i] = dsm_appkey_index_to_appkey_handle(appkey_instances[i]);
            }
        }

        status = dsm_subnet_delete(network_handle);

        if (NRF_SUCCESS == status)
        {
            for (uint32_t i = 0; i < counter; i++)
            {
                NRF_MESH_ERROR_CHECK(access_model_publication_by_appkey_stop(appkey_instances[i]));
            }
        }
    }

    if (NRF_SUCCESS == status)
    {
        send_netkey_status(handle, p_message, ACCESS_STATUS_SUCCESS, netkey_index);
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_NETKEY_DELETE;
        evt.params.netkey_delete.netkey_handle = network_handle;
        app_evt_send(&evt);
    }
    else if (NRF_ERROR_FORBIDDEN == status)
    {
        send_netkey_status(handle, p_message, ACCESS_STATUS_CANNOT_REMOVE, netkey_index);
    }
    else
    {
        send_netkey_status(handle, p_message, ACCESS_STATUS_INVALID_NETKEY, netkey_index);
    }
}

static void handle_netkey_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }

    uint32_t num_netkeys = DSM_SUBNET_MAX;
    mesh_key_index_t netkey_indexes[DSM_SUBNET_MAX];
    (void) dsm_subnet_get_all(netkey_indexes, &num_netkeys);

    /* The 12-bit netkey indexes needs to be packed into an array before being sent to the client: */
    uint8_t buffer[PACKED_INDEX_LIST_SIZE(DSM_SUBNET_MAX)];
    uint16_t reply_length = PACKED_INDEX_LIST_SIZE(num_netkeys);
    packed_index_list_create(netkey_indexes, buffer, num_netkeys);

    send_reply(handle, p_message, CONFIG_OPCODE_NETKEY_LIST, buffer, reply_length, nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_NETKEY_GET;
    app_evt_send(&evt);
}

static void handle_node_identity_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_identity_get_t))
    {
        return;
    }

    const config_msg_identity_get_t * p_pdu = (const config_msg_identity_get_t *) p_message->p_data;
    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_NODE_IDENTITY_GET;
    evt.params.identity_get.netkey_index = p_pdu->netkey_index;

#if MESH_FEATURE_GATT_PROXY_ENABLED
    access_status_t access_status = ACCESS_STATUS_SUCCESS;
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    dsm_handle_t subnet = dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index);
    if (dsm_beacon_info_get(subnet, &p_beacon_info) != NRF_SUCCESS)
    {
        access_status = ACCESS_STATUS_INVALID_NETKEY;
    }

    const config_msg_identity_status_t reply =
    {
        .status = access_status,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = ((access_status == ACCESS_STATUS_SUCCESS && proxy_node_id_is_enabled(p_beacon_info))
                            ? CONFIG_IDENTITY_STATE_RUNNING
                            : CONFIG_IDENTITY_STATE_STOPPED)
    };
#else
    const config_msg_identity_status_t reply =
    {
        .status = ACCESS_STATUS_SUCCESS,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = CONFIG_IDENTITY_STATE_UNSUPPORTED
    };
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_NODE_IDENTITY_STATUS,
               (const uint8_t *) &reply, sizeof(config_msg_identity_status_t), nrf_mesh_unique_token_get());

    app_evt_send(&evt);
}

static void handle_node_identity_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_identity_set_t))
    {
        return;
    }

    const config_msg_identity_set_t * p_pdu = (const config_msg_identity_set_t *) p_message->p_data;

    if (p_pdu->identity_state >= CONFIG_IDENTITY_STATE_UNSUPPORTED)
    {
        /* Invalid value */
        return;
    }

#if MESH_FEATURE_GATT_PROXY_ENABLED
    access_status_t access_status = ACCESS_STATUS_SUCCESS;
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;

    dsm_handle_t subnet = dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index);
    nrf_mesh_key_refresh_phase_t kr_phase;
    if (dsm_beacon_info_get(subnet, &p_beacon_info) == NRF_SUCCESS &&
        dsm_subnet_kr_phase_get(subnet, &kr_phase)  == NRF_SUCCESS)
    {
        if (p_pdu->identity_state == CONFIG_IDENTITY_STATE_RUNNING)
        {
            if (!proxy_is_enabled() || proxy_node_id_enable(p_beacon_info, kr_phase) != NRF_SUCCESS)
            {
                access_status = ACCESS_STATUS_TEMPORARILY_UNABLE_TO_CHANGE_STATE;
            }
        }
        else /* p_pdu->identity_state == CONFIG_IDENTITY_STATE_STOPPED */
        {
            uint32_t err_code = proxy_node_id_disable();
            NRF_MESH_ASSERT_DEBUG(err_code == NRF_SUCCESS || NRF_ERROR_INVALID_STATE);
        }
    }
    else
    {
        access_status = ACCESS_STATUS_INVALID_NETKEY;
    }

    const config_msg_identity_status_t reply =
    {
        .status = access_status,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = ((access_status == ACCESS_STATUS_SUCCESS &&
                            proxy_node_id_is_enabled(p_beacon_info))
                            ? CONFIG_IDENTITY_STATE_RUNNING
                            : CONFIG_IDENTITY_STATE_STOPPED)
    };
#else
    const config_msg_identity_status_t reply =
    {
        .status = ACCESS_STATUS_SUCCESS,
        .netkey_index = p_pdu->netkey_index,
        .identity_state = CONFIG_IDENTITY_STATE_UNSUPPORTED
    };
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_NODE_IDENTITY_STATUS,
               (const uint8_t *) &reply, sizeof(config_msg_identity_status_t), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_NODE_IDENTITY_SET;
    evt.params.identity_set.netkey_index = p_pdu->netkey_index;
    app_evt_send(&evt);
}

static void handle_node_reset(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    /* Send reply, and then wait until reply has been clocked out of the radio (i.e. wait for the
     * TX_COMPLETE event) before clearing the state */
    m_reset_token = nrf_mesh_unique_token_get();
    m_node_reset_pending = NODE_RESET_PENDING;
    send_reply(handle, p_message, CONFIG_OPCODE_NODE_RESET_STATUS, NULL, 0, m_reset_token);
}

static uint8_t model_app_response_create(uint16_t opcode,
                                         uint8_t * p_buffer,
                                         access_status_t status,
                                         uint16_t element_address,
                                         access_model_id_t * p_model_id,
                                         mesh_key_index_t * p_appkey_list,
                                         uint16_t appkey_count)
{
    uint8_t size;

    if (opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET)
    {
        config_msg_sig_model_app_list_t * p_response = (config_msg_sig_model_app_list_t *)p_buffer;
        p_response->status = status;
        p_response->element_address = element_address;
        p_response->sig_model_id = p_model_id->model_id;

        if (NULL != p_appkey_list)
        {
            packed_index_list_create(p_appkey_list, p_response->key_indexes, appkey_count);
        }

        size = sizeof(config_msg_sig_model_app_list_t);
    }
    else
    {
        config_msg_vendor_model_app_list_t * p_response = (config_msg_vendor_model_app_list_t *)p_buffer;
        p_response->status = status;
        p_response->element_address = element_address;
        p_response->vendor_company_id = p_model_id->company_id;
        p_response->vendor_model_id = p_model_id->model_id;

        if (NULL != p_appkey_list)
        {
            packed_index_list_create(p_appkey_list, p_response->key_indexes, appkey_count);
        }

        size = sizeof(config_msg_vendor_model_app_list_t);
    }

    return size + PACKED_INDEX_LIST_SIZE(appkey_count);
}

static void handle_model_app_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_model_app_get_t, p_message))
    {
        return;
    }

    const config_msg_model_app_get_t * p_pdu = (const config_msg_model_app_get_t *) p_message->p_data;

    /* Get the model handle: */
    access_model_handle_t model_handle;
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_model_app_get_t));
    uint16_t element_index = get_element_index(p_pdu->element_address);
    uint16_t response_opcode = p_message->opcode.opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET ?
            CONFIG_OPCODE_SIG_MODEL_APP_LIST : CONFIG_OPCODE_VENDOR_MODEL_APP_LIST;
    uint8_t response_buffer[sizeof(config_msg_vendor_model_app_list_t) + PACKED_INDEX_LIST_SIZE(DSM_APP_MAX)];


    if (ACCESS_ELEMENT_INDEX_INVALID == element_index)
    {
        uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                          response_buffer,
                                                          ACCESS_STATUS_INVALID_ADDRESS,
                                                          p_pdu->element_address,
                                                          &model_id,
                                                          NULL,
                                                          0);
        send_reply(handle, p_message, response_opcode, response_buffer, response_size, nrf_mesh_unique_token_get());
        return;
    }

    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                          response_buffer,
                                                          ACCESS_STATUS_INVALID_MODEL,
                                                          p_pdu->element_address,
                                                          &model_id,
                                                          NULL,
                                                          0);
        send_reply(handle, p_message, response_opcode, response_buffer, response_size, nrf_mesh_unique_token_get());
        return;
    }

    /* Get the application list: */
    uint16_t appkey_instances[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    NRF_MESH_ASSERT(access_model_applications_get(model_handle, appkey_instances, &appkey_count) == NRF_SUCCESS);

    /* Retrieve the appkey indexes from the DSM: */
    for (uint16_t i = 0; i < appkey_count; ++i)
    {
        NRF_MESH_ASSERT(dsm_appkey_handle_to_appkey_index(appkey_instances[i], &appkey_instances[i]) == NRF_SUCCESS);
    }

    uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                      response_buffer,
                                                      ACCESS_STATUS_SUCCESS,
                                                      p_pdu->element_address,
                                                      &model_id,
                                                      appkey_instances,
                                                      appkey_count);
    send_reply(handle, p_message, response_opcode, response_buffer, response_size, nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = p_message->opcode.opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET ?
        CONFIG_SERVER_EVT_SIG_MODEL_APP_GET : CONFIG_SERVER_EVT_VENDOR_MODEL_APP_GET;
    evt.params.model_app_get.model_handle = model_handle;
    app_evt_send(&evt);
}

static void handle_config_low_power_node_polltimeout_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_low_power_node_polltimeout_get_t, p_message))
    {
        return;
    }

    const config_msg_low_power_node_polltimeout_get_t * p_pdu = (config_msg_low_power_node_polltimeout_get_t *) p_message->p_data;
    if (nrf_mesh_address_type_get(p_pdu->lpn_address) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return;
    }

    config_msg_low_power_node_polltimeout_status_t response = {p_pdu->lpn_address, {0, 0, 0}};

#if MESH_FEATURE_FRIEND_ENABLED
    uint32_t remaining_time = friend_remaining_poll_timeout_time_get(p_pdu->lpn_address);
    response.polltimeout[0] = 0xFF & remaining_time;
    response.polltimeout[1] = 0xFF & (remaining_time >> 8);
    response.polltimeout[2] = 0xFF & (remaining_time >> 16);
#endif

    send_reply(handle, p_message, CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS,
               (const uint8_t *) &response, sizeof(response), nrf_mesh_unique_token_get());

    config_server_evt_t evt;
    memset(&evt, 0, sizeof(config_server_evt_t));
    evt.type = CONFIG_SERVER_EVT_LOW_POWER_NODE_POLLTIMEOUT_GET;
    evt.params.lpn_polltimeout_get.lpn_address = p_pdu->lpn_address;
    app_evt_send(&evt);
}

static void apply_reset(void)
{
    /* Clear all the state. */
    mesh_stack_config_clear();
#if PERSISTENT_STORAGE
    if (!flash_manager_is_stable())
    {
        m_node_reset_pending = NODE_RESET_FLASHING;
    }
    else
#endif
    {
        config_server_evt_t evt;
        memset(&evt, 0, sizeof(config_server_evt_t));
        evt.type = CONFIG_SERVER_EVT_NODE_RESET;
        app_evt_send(&evt);
        m_node_reset_pending = NODE_RESET_IDLE;
    }
}

static void mesh_event_cb(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
#if MESH_FEATURE_GATT_PROXY_ENABLED
        case NRF_MESH_EVT_PROXY_STOPPED:
            if (m_node_reset_pending == NODE_RESET_PENDING_PROXY)
            {
                apply_reset();
            }
            break;
#endif
        case NRF_MESH_EVT_TX_COMPLETE:
            if (p_evt->params.tx_complete.token == m_reset_token)
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                if (NODE_RESET_PENDING == m_node_reset_pending)
                {
                    /* Disconnect from the GATT client if we're in a connection.
                     * As a side effect, this ensures that flashing will complete,
                     * even if we have a low connection interval.
                     */
                    if (proxy_is_connected())
                    {
                        m_node_reset_pending = NODE_RESET_PENDING_PROXY;
                        (void) proxy_stop();
                    }
                    else
                    {
                        apply_reset();
                    }
                }
#else
                if (NODE_RESET_PENDING == m_node_reset_pending)
                {
                    apply_reset();
                }
#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */
            }
            break;

        case NRF_MESH_EVT_FLASH_STABLE:
            if (NODE_RESET_FLASHING == m_node_reset_pending && flash_manager_is_stable())
            {
                config_server_evt_t evt;
                memset(&evt, 0, sizeof(config_server_evt_t));
                evt.type = CONFIG_SERVER_EVT_NODE_RESET;
                app_evt_send(&evt);
                m_node_reset_pending = NODE_RESET_IDLE;
            }
            break;

        default:
            break;
    }
}

/********** Opcode handler list *********/
static const access_opcode_handler_t opcode_handlers[] =
{
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_ADD)                                   , handle_appkey_add },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_UPDATE)                                , handle_appkey_update },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_SET)                        , handle_config_model_publication_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_DELETE)                                , handle_appkey_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_GET)                                   , handle_appkey_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_COMPOSITION_DATA_GET)                         , handle_composition_data_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_BEACON_GET)                                   , handle_config_beacon_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_BEACON_SET)                                   , handle_config_beacon_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_DEFAULT_TTL_GET)                              , handle_config_default_ttl_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_DEFAULT_TTL_SET)                              , handle_config_default_ttl_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_FRIEND_GET)                                   , handle_config_friend_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_FRIEND_SET)                                   , handle_config_friend_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_GATT_PROXY_GET)                               , handle_config_gatt_proxy_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_GATT_PROXY_SET)                               , handle_config_gatt_proxy_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_KEY_REFRESH_PHASE_GET)                        , handle_config_key_refresh_phase_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_KEY_REFRESH_PHASE_SET)                        , handle_config_key_refresh_phase_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_GET)                        , handle_config_model_publication_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET)        , handle_config_model_publication_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD)                       , handle_config_model_subscription_add },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE)                    , handle_config_model_subscription_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL)                , handle_config_model_subscription_delete_all },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE)                 , handle_config_model_subscription_overwrite },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD)       , handle_config_model_subscription_virtual_address_add },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE)    , handle_config_model_subscription_virtual_address_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE) , handle_config_model_subscription_virtual_address_overwrite },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_RELAY_GET)                                    , handle_config_relay_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_RELAY_SET)                                    , handle_config_relay_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET)                   , handle_config_sig_model_subscription_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET)                , handle_config_vendor_model_subscription_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET)                    , handle_heartbeat_publication_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET)                    , handle_heartbeat_publication_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET)                   , handle_heartbeat_subscription_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET)                   , handle_heartbeat_subscription_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_APP_BIND)                               , handle_model_app_bind_unbind },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_APP_UNBIND)                             , handle_model_app_bind_unbind },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_ADD)                                   , handle_netkey_add_update },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_UPDATE)                                , handle_netkey_add_update },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_DELETE)                                , handle_netkey_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETKEY_GET)                                   , handle_netkey_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_IDENTITY_GET)                            , handle_node_identity_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_IDENTITY_SET)                            , handle_node_identity_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_RESET)                                   , handle_node_reset },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_APP_GET)                            , handle_model_app_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_VENDOR_MODEL_APP_GET)                         , handle_model_app_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETWORK_TRANSMIT_GET)                         , handle_config_network_transmit_get},
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_GET)               , handle_config_low_power_node_polltimeout_get},
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETWORK_TRANSMIT_SET)                         , handle_config_network_transmit_set}
};

uint32_t config_server_init(config_server_evt_cb_t evt_cb)
{
    const access_model_add_params_t init_params =
    {
        .model_id = {
            .model_id = CONFIG_SERVER_MODEL_ID,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .element_index = 0,
        .p_opcode_handlers = opcode_handlers,
        .opcode_count = sizeof(opcode_handlers) / sizeof(access_opcode_handler_t)
    };

    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

    /* Register function for the publication information getting for the heartbeat */
    heartbeat_public_info_getter_register(config_server_heartbeat_publication_params_get);

#if !defined(UNIT_TEST)
    /* Ensure the model has not already been initialized. */
    NRF_MESH_ASSERT(m_config_server_handle == ACCESS_HANDLE_INVALID);
#endif

    m_evt_cb = evt_cb;
    return access_model_add(&init_params, &m_config_server_handle);
}

uint32_t config_server_bind(dsm_handle_t devkey_handle)
{
    return access_model_application_bind(m_config_server_handle, devkey_handle);
}
