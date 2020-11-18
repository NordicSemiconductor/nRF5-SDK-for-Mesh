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

#ifndef CONFIG_CLIENT_H__
#define CONFIG_CLIENT_H__

#include <stdint.h>
#include "config_messages.h"
#include "config_opcodes.h"

#include "access.h"
#include "device_state_manager.h"

/**
 * @defgroup CONFIG_CLIENT Configuration client
 * @ingroup CONFIG_MODEL
 * Remotely configure a mesh device by communicating with the remote device's Config server model.
 *
 * @{
 */

/** Acknowledged message transaction timeout */
#ifndef CONFIG_CLIENT_ACKED_TRANSACTION_TIMEOUT
#define CONFIG_CLIENT_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(60))
#endif

/** Publication state parameter structure. */
typedef struct
{
    /** Element address of the model to set the publication state. */
    uint16_t element_address;
    /**
     * Publish address.
     * Set type to @ref NRF_MESH_ADDRESS_TYPE_VIRTUAL to set it to a virtual address.
     */
    nrf_mesh_address_t publish_address;
    /** Application key index. */
    uint16_t appkey_index;
    /**
     * Set @c true to use friendship credentials for publishing.
     * @warning Not supported.
     */
    bool frendship_credential_flag;
    /**
     * Publish TTL value.
     * Set to @ref ACCESS_TTL_USE_DEFAULT to use the default TTL configuration for the node.
     */
    uint8_t publish_ttl;
    /** Publish period. */
    access_publish_period_t publish_period;
    /** Retransmit count. */
    uint8_t retransmit_count;
    /** Retransmit interval (in multiples of 50 ms). */
    uint8_t retransmit_interval;
    /** Model identifier. */
    access_model_id_t model_id;
} config_publication_state_t;

/** Configuration client event types. */
typedef enum
{
    CONFIG_CLIENT_EVENT_TYPE_TIMEOUT,
    CONFIG_CLIENT_EVENT_TYPE_CANCELLED,
    CONFIG_CLIENT_EVENT_TYPE_MSG
} config_client_event_type_t;

/** Union of possible status message responses. */
typedef union
{
    config_msg_appkey_status_t appkey_status;
    config_msg_net_beacon_status_t net_beacon_status;
    config_msg_publication_status_t publication_status;
    config_msg_subscription_status_t subscription_status;
    config_msg_netkey_status_t netkey_status;
    config_msg_proxy_status_t proxy_status;
    config_msg_key_refresh_phase_status_t key_refresh_phase_status;
    config_msg_friend_status_t friend_status;
    config_msg_heartbeat_publication_status_t heartbeat_publication_status;
    config_msg_heartbeat_subscription_status_t heartbeat_subscription_status;
    config_msg_default_ttl_status_t default_ttl_status;
    config_msg_app_status_t app_status;
    config_msg_identity_status_t identity_status;
    config_msg_composition_data_status_t composition_data_status;
    config_msg_relay_status_t relay_status;
    config_msg_appkey_list_t appkey_list;
    config_msg_sig_model_app_list_t sig_model_app_list;
    config_msg_vendor_model_app_list_t vendor_model_app_list;
    config_msg_sig_model_subscription_list_t sig_model_subscription_list;
    config_msg_vendor_model_subscription_list_t vendor_model_subscription_list;
} config_msg_t;

/** Configuration client event structure.  */
typedef struct
{
    /** Opcode of the status reply. */
    config_opcode_t opcode;
    /** Pointer to message structure. */
    const config_msg_t * p_msg;
} config_client_event_t;

/**
 * Configuration client event callback type.
 *
 * @param[in] event_type Event type.
 * @param[in] p_event    Pointer to event data, may be @c NULL.
 */
typedef void (*config_client_event_cb_t)(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length);


/**
 * Initializes the configuration client.
 *
 * @warning This function can only be called _once_.
 *
 * @param[in] event_cb Event callback pointer.
 *
 * @retval NRF_SUCCESS      Successfully initialized client and added to the access layer.
 * @retval NRF_ERROR_NULL   @c event_cb was @c NULL.
 * @retval NRF_ERROR_NO_MEM @ref ACCESS_MODEL_COUNT number of models already allocated.
 */
uint32_t config_client_init(config_client_event_cb_t event_cb);

/**
 * Sets the configuration server to configure.
 *
 * @note The address should be the address of the root element of the node (element 0).
 * @note The configuration client will handle the switching of device key implicitly.
 * @note The device key must be bound the client.
 *
 * @param[in] server_devkey_handle  Device key handle for the remote server.
 * @param[in] server_address_handle Handle for the address of the remote server.
 *
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_server_set(dsm_handle_t server_devkey_handle, dsm_handle_t server_address_handle);

/**
 * Binds the configuration client to a server.
 *
 * @note This function should be called for each new device that is to be configured.
 *
 * @param[in] server_devkey_handle Device key handle for the remote server.
 *
 * @retval NRF_SUCCESS             Successfully bound the server to the client.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 * @retval NRF_ERROR_INVALID_PARAM Invalid application key handle.
 */
uint32_t config_client_server_bind(dsm_handle_t server_devkey_handle);

/**
 * Sends a composition data GET request.
 *
 * @note Response: @ref CONFIG_OPCODE_COMPOSITION_DATA_STATUS
 *
 * @param[in] page_number          Device composition page number to be requested from the server
 *
 * @note Page 0x00 is the only mandatory page in Mesh 1.0.
 *       It is possible to read all supported Composition Data Pages by reading 0xFF first,
 *       and then reading one less than the returned page number until the page number is 0x00.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_composition_data_get(uint8_t page_number);

/**
 * Sends an application key add request.
 *
 * @note Response: @ref CONFIG_OPCODE_APPKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 * @param[in] p_appkey     Pointer to @ref NRF_MESH_KEY_SIZE byte application key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_add(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey);

/**
 * Sends an application key delete request.
 *
 * @note Response: @ref CONFIG_OPCODE_APPKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_delete(uint16_t netkey_index, uint16_t appkey_index);

/**
 * Sends an application key(s) get request.
 *
 * @note Response: @ref CONFIG_OPCODE_APPKEY_LIST
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_get(uint16_t netkey_index);

/**
 * Sends an application key update request.
 *
 * @note Response: @ref CONFIG_OPCODE_APPKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] appkey_index Application key index.
 * @param[in] p_appkey     Pointer to @ref NRF_MESH_KEY_SIZE byte application key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_appkey_update(uint16_t netkey_index, uint16_t appkey_index, const uint8_t * p_appkey);

/**
 * Sends a network key add request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] p_netkey     Pointer to @ref NRF_MESH_KEY_SIZE byte network key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_add(uint16_t netkey_index, const uint8_t * p_netkey);

/**
 * Sends a network key delete request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_delete(uint16_t netkey_index);

/**
 * Sends a network key(s) get request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETKEY_LIST
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_get(void);

/**
 * Sends a network key update request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETKEY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] p_netkey     Pointer to @ref NRF_MESH_KEY_SIZE byte network key.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_netkey_update(uint16_t netkey_index, const uint8_t * p_netkey);

/**
 * Sends a publication get request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_PUBLICATION_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model identifier.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_publication_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a model publication set request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_PUBLICATION_STATUS
 *
 * @param[in] p_publication_state Publication state parameter struct pointer.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_publication_set(const config_publication_state_t * p_publication_state);

/**
 * Sends a subscription add request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_add(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a subscription delete request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_delete(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a subscription delete all request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_delete_all(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a subscription get request.
 *
 * @note Response for SIG models: @ref CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST
 * @note Response for vendor models: @ref CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a subscription overwrite request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS
 *
 * @warning This will clear the subscription list of the model.
 *
 * @param[in] element_address Element address of the model.
 * @param[in] address         Address to add to the subscription list.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_subscription_overwrite(uint16_t element_address, nrf_mesh_address_t address, access_model_id_t model_id);

/**
 * Sends a application bind request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_APP_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] appkey_index    Application key index to bind/unbind.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_bind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id);

/**
 * Sends an application get request.
 *
 * @note Response for SIG models: @ref CONFIG_OPCODE_SIG_MODEL_APP_LIST
 * @note Response for vendor models: @ref CONFIG_OPCODE_VENDOR_MODEL_APP_LIST
 *
 * @param[in] element_address Element address of the model.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_get(uint16_t element_address, access_model_id_t model_id);

/**
 * Sends a application unbind request.
 *
 * @note Response: @ref CONFIG_OPCODE_MODEL_APP_STATUS
 *
 * @param[in] element_address Element address of the model.
 * @param[in] appkey_index    Application key index to bind/unbind.
 * @param[in] model_id        Model ID of the model.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_model_app_unbind(uint16_t element_address, uint16_t appkey_index, access_model_id_t model_id);

/**
 * Sends a default TTL get request.
 *
 * @note Response: @ref CONFIG_OPCODE_DEFAULT_TTL_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_default_ttl_get(void);

/**
 * Sends a default TTL set request.
 *
 * @note Response: @ref CONFIG_OPCODE_DEFAULT_TTL_STATUS
 *
 * @param[in] ttl Default TTL value. Must be less than @ref NRF_MESH_TTL_MAX.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_default_ttl_set(uint8_t ttl);

/**
 * Sends a relay state get request.
 *
 * @note Response: @ref CONFIG_OPCODE_RELAY_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_relay_get(void);

/**
 * Sends a relay state set request.
 *
 * @note Response: @ref CONFIG_OPCODE_RELAY_STATUS
 *
 * @param[in] relay_state               Relay state.
 * @param[in] retransmit_count          Number of times to re-transmit relayed packets.
 * @param[in] retransmit_interval_steps Number of 10 ms steps between each re-transmission.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 * @retval NRF_ERROR_INVALID_PARAM Invalid parameter values. See @ref CONFIG_RETRANSMIT_COUNT_MAX
 *                                 and @ref CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX.
 */
uint32_t config_client_relay_set(config_relay_state_t relay_state, uint8_t retransmit_count, uint8_t retransmit_interval_steps);

/**
 * Sends a network transmit get request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_network_transmit_get(void);

/**
 * Sends a network transmit set request.
 *
 * @note Response: @ref CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS
 *
 * @param[in] transmit_count          Number of times to re-transmit originated packets.
 * @param[in] transmit_interval_steps Number of 10 ms steps between each re-transmission.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 * @retval NRF_ERROR_INVALID_PARAM Invalid parameter values. See @ref CONFIG_RETRANSMIT_COUNT_MAX
 *                                 and @ref CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX.
 */
uint32_t config_client_network_transmit_set(uint8_t transmit_count, uint8_t transmit_interval_steps);

/**
 * Sends a secure network beacon state get request.
 *
 * @note Response: @ref CONFIG_OPCODE_BEACON_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_net_beacon_get(void);

/**
 * Sends a secure network beacon state set request.
 *
 * @note Response: @ref CONFIG_OPCODE_BEACON_STATUS
 *
 * @param[in] state New secure network beacon state.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_net_beacon_set(config_net_beacon_state_t state);

/**
 * Sends a node reset request.
 *
 * @note Response: @ref CONFIG_OPCODE_NODE_RESET_STATUS
 *
 * @warning This will "un-provision" the node and remove it from the network.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_node_reset(void);

/**
 * Gets the current key refresh phase of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_key_refresh_phase_get(uint16_t netkey_index);

/**
 * Sets the current key refresh phase of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] phase        Key refresh phase to set for the node.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_key_refresh_phase_set(uint16_t netkey_index, nrf_mesh_key_refresh_phase_t phase);

/**
 * Gets the current Friend state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_FRIEND_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_friend_get(void);

/**
 * Sets the Friend state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_FRIEND_STATUS
 *
 * @param[in] state New Friend state.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_friend_set(config_friend_state_t state);

/**
 * Gets the current GATT Proxy state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_GATT_PROXY_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_gatt_proxy_get(void);

/**
 * Sets the GATT Proxy state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_GATT_PROXY_STATUS
 *
 * @param[in] state New Friend state.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_gatt_proxy_set(config_gatt_proxy_state_t state);

/**
 * Gets the current Node Identity state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_NODE_IDENTITY_STATUS
 *
 * @param[in] netkey_index Network key index.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_node_identity_get(uint16_t netkey_index);

/**
 * Sets the current Node Identity state of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_NODE_IDENTITY_STATUS
 *
 * @param[in] netkey_index Network key index.
 * @param[in] state        Node Identity state to set for the node.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_node_identity_set(uint16_t netkey_index, config_identity_state_t state);

/**
 * Gets the heartbeat publication state value of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_heartbeat_publication_get(void);

/**
 * Sets the heartbeat publication state value of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS
 *
 * @param[in]  p_publication    Pointer to the @ref config_msg_heartbeat_publication_set_t structure
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_heartbeat_publication_set(const config_msg_heartbeat_publication_set_t * p_publication);

/**
 * Gets the heartbeat subscription state value of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_heartbeat_subscription_get(void);

/**
 * Sets the heartbeat subscription state value of a node.
 *
 * @note Response: @ref CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS
 *
 * @param[in]  p_subscription    Pointer to the @ref config_msg_heartbeat_subscription_set_t structure
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_heartbeat_subscription_set(const config_msg_heartbeat_subscription_set_t * p_subscription);

/**
 * Gets the current value of the PollTimeout timer of the Low Power node.
 *
 * @note Response: @ref CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS
 *
 * @param[in] lpn_address Address of the Low Power node.
 *
 * @retval NRF_SUCCESS             Successfully sent request.
 * @retval NRF_ERROR_BUSY          The client is in a transaction. Try again later.
 * @retval NRF_ERROR_NO_MEM        Not enough memory available for sending request.
 * @retval NRF_ERROR_INVALID_STATE Client not initialized.
 */
uint32_t config_client_low_power_node_polltimeout_get(uint16_t lpn_address);

/**
 * Cancel any ongoing reliable message transfer.
 */
void config_client_pending_msg_cancel(void);

/** @} end of CONFIG_CLIENT */

#endif  /* CONFIG_CLIENT_H__ */

