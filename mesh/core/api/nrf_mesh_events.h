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

#ifndef MESH_EVT_H__
#define MESH_EVT_H__

#include <stdbool.h>
#include "nrf_mesh.h"
#include "nrf_mesh_dfu_types.h"
#include "list.h"
#include "mesh_config.h"
#include "heartbeat.h"
#include "mesh_friendship_types.h"

/**
 * @defgroup NRF_MESH_EVENTS Mesh events
 * @ingroup NRF_MESH
 * Runtime events in the core mesh.
 * @{
 */

/**
 * Mesh event types.
 */
typedef enum
{
    /** A message is received. */
    NRF_MESH_EVT_MESSAGE_RECEIVED,
    /** Transmission completed. */
    NRF_MESH_EVT_TX_COMPLETE,
    /** An IV update event occurred. */
    NRF_MESH_EVT_IV_UPDATE_NOTIFICATION,
    /** A key refresh event occurred. */
    NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION,
    /** An authenticated network beacon is received. */
    NRF_MESH_EVT_NET_BEACON_RECEIVED,
    /** A heartbeat message is received. */
    NRF_MESH_EVT_HB_MESSAGE_RECEIVED,
    /** The heartbeat subscription parameters changed. */
    NRF_MESH_EVT_HB_SUBSCRIPTION_CHANGE,
    /** DFU request for this node to be the relay of a transfer. */
    NRF_MESH_EVT_DFU_REQ_RELAY,
    /** DFU request for this node to be the source of a transfer. */
    NRF_MESH_EVT_DFU_REQ_SOURCE,
    /** DFU transfer starts. */
    NRF_MESH_EVT_DFU_START,
    /** DFU transfer ended. */
    NRF_MESH_EVT_DFU_END,
    /** DFU bank available. */
    NRF_MESH_EVT_DFU_BANK_AVAILABLE,
    /** The device firmware is outdated, according to a trusted source. */
    NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED,
    /** The device firmware is outdated, according to an un-authenticated source. */
    NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH,
    /** Flash operations queue is empty, and flash is stable. There are no event parameters for this
     * message. */
    NRF_MESH_EVT_FLASH_STABLE,
    /** RX failed. */
    NRF_MESH_EVT_RX_FAILED,
    /** SAR session failed. */
    NRF_MESH_EVT_SAR_FAILED,
    /** Flash malfunctioned. */
    NRF_MESH_EVT_FLASH_FAILED,
    /** Mesh configuration persistent storage is stable. */
    NRF_MESH_EVT_CONFIG_STABLE,
    /** Mesh configuration persistent storage failed while storing. */
    NRF_MESH_EVT_CONFIG_STORAGE_FAILURE,
    /** Mesh configuration persistent storage failed while loading an entry. */
    NRF_MESH_EVT_CONFIG_LOAD_FAILURE,
    /** The node received a Friend Offer. */
    NRF_MESH_EVT_LPN_FRIEND_OFFER,
    /** The node received a Friend Update. */
    NRF_MESH_EVT_LPN_FRIEND_UPDATE,
    /** The Friend Request timed out. */
    NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT,
    /** The LPN successfully polled all data from the Friend node. */
    NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE,
    /** The friendship is successfully established. */
    NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED,
    /** The friendship is successfully terminated. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED,
    /** The mesh proxy server is stopped. */
    NRF_MESH_EVT_PROXY_STOPPED,
    /** The mesh has been disabled. */
    NRF_MESH_EVT_DISABLED,
    /** The mesh has been enabled. This event provides a suitable context and a starting point
     * to begin calling the Mesh stack APIs after the stack is fully initialized. */
    NRF_MESH_EVT_ENABLED,
    /** The node has received a Friend Request from an LPN. Unless the friendship is actively
     * terminated, the request is accepted automatically. */
    NRF_MESH_EVT_FRIEND_REQUEST,
    /** The mesh stack completed and stopped all activities and ready to power off. */
    NRF_MESH_EVT_READY_TO_POWER_OFF,
} nrf_mesh_evt_type_t;

/**
 * Message received event structure.
 */
typedef struct
{
    /** Buffer containing the message data. */
    const uint8_t * p_buffer;
    /** Message length. */
    uint16_t length;
    /** Source address of the message. */
    nrf_mesh_address_t src;
    /** Destination address of the message. */
    nrf_mesh_address_t dst;
    /** Security material used in the decryption of the payload. */
    nrf_mesh_secmat_t secmat;
    /** Time-to-live for the message. This is a 7-bit value. */
    uint8_t ttl;
    /** Metadata for the received packet. In case of a segmented message, the metadata represents the last packet. */
    const nrf_mesh_rx_metadata_t * p_metadata;
} nrf_mesh_evt_message_t;

/**
 * IV update event structure.
 */
typedef struct
{
    /** Current IV update state. */
    net_state_iv_update_t state;
    /** Network ID of the beacon that triggered the notification. NULL if unknown. */
    const uint8_t * p_network_id;
    /** IV index currently used for sending messages. */
    uint32_t iv_index;
} nrf_mesh_evt_iv_update_notification_t;

/**
 * Key refresh notification structure.
 */
typedef struct
{
    uint16_t subnet_index;
    const uint8_t * p_network_id;
    nrf_mesh_key_refresh_phase_t phase;
} nrf_mesh_evt_key_refresh_notification_t;

/**
 * Network beacon received structure.
 */
typedef struct
{
    const nrf_mesh_beacon_info_t * p_beacon_info; /**< Pointer to the associated beacon info used to authenticate the incoming beacon. */
    const nrf_mesh_beacon_secmat_t * p_beacon_secmat; /**< The security material within the @p p_beacon_info that authenticated the incoming beacon. */
    const nrf_mesh_rx_metadata_t * p_rx_metadata; /**< RX metadata for the packet that produced the beacon. */
    const uint8_t * p_auth_value; /**< Authentication value in the beacon. */
    uint32_t iv_index; /**< IV index in the beacon. */
    struct
    {
        net_state_iv_update_t iv_update; /**< IV update flag in the beacon. */
        bool key_refresh; /**< Key refresh flag in the beacon. */
    } flags;
} nrf_mesh_evt_net_beacon_received_t;

/**
 * Heartbeat received event structure.
 *
 * This event structure is formatted as recommended by @tagMeshSp section 3.6.7.3.
 */
typedef struct
{
    /** Initial TTL value used for sending the heartbeat message. */
    uint8_t  init_ttl;
    /** Calculated hop value. Number of hops = Initial TTL - Received message TTL + 1. */
    uint8_t  hops;
    /** Features' state bitmap. See @ref MESH_DEFINES_HEARTBEAT to interpret bit fields. */
    uint16_t features;
    /** Metadata for the received packet. In case of a segmented message,
     * the metadata represents the last packet. */
    uint16_t src;
} nrf_mesh_evt_hb_message_t;

/** Parameters for changes to the heartbeat subscription state. */
typedef struct
{
    /** Old subscription state, or NULL if heartbeat subscriptions were disabled before the change. */
    const heartbeat_subscription_state_t * p_old;
    /** New subscription state, or NULL if heartbeat subscriptions are disabled. */
    const heartbeat_subscription_state_t * p_new;
} nrf_mesh_evt_hb_subscription_change_t;

/**
 * Transmission complete event structure.
 */
typedef struct
{
    /** Packet identifier. */
    nrf_mesh_tx_token_t token;
	/** Timestamp of the TX complete event. */
    timestamp_t timestamp;
} nrf_mesh_evt_tx_complete_t;

/** DFU event parameters. */
typedef union
{
    /** Firmware outdated event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        nrf_mesh_fwid_t current;                /**< FWID union containing the current firmware ID of the given type. */
    } fw_outdated;
    /** DFU Relay request event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        uint8_t authority;                      /**< Authority level of the transfer. */
    } req_relay;
    /** DFU Source request event parameters. */
    struct
    {
        nrf_mesh_dfu_type_t dfu_type;           /**< DFU type and firmware ID of the transfer. */
    } req_source;
    /** DFU Start event parameters. */
    struct
    {
        nrf_mesh_dfu_role_t     role;           /**< The role of the device in the transfer. */
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
    } start;
    /** DFU end event parameters. */
    struct
    {
        nrf_mesh_dfu_role_t     role;           /**< The role of the device in the transfer. */
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the transfer. */
        nrf_mesh_dfu_end_t      end_reason;     /**< Reason for the end event. */
    } end;
    /** Bank available event parameters. */
    struct
    {
        nrf_mesh_dfu_transfer_t transfer;       /**< DFU type and firmware ID of the bank. */
        const uint32_t * p_start_addr;          /**< Start address of the bank. */
        uint32_t length;                        /**< Length of the firmware in the bank. */
        bool is_signed;                         /**< Flag indicating whether the bank is signed with an encryption key. */
    } bank;
} nrf_mesh_evt_dfu_t;

/**
 * RX failure reason codes.
 */
typedef enum
{
    /** The replay protection cache is full. */
    NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL
} nrf_mesh_rx_failed_reason_t;

/**
 * RX failed event structure.
 */
typedef struct
{
    /** Unicast address of the sender. */
    uint16_t src;
    /**IV index bit of the RX packet. */
    uint8_t ivi : 1;
    /** Reason for the RX failure. */
    nrf_mesh_rx_failed_reason_t reason;
} nrf_mesh_evt_rx_failed_t;

/**
 * SAR session cancelled reason codes.
 */
typedef enum
{
    /** The transport SAR session timed out. */
    NRF_MESH_SAR_CANCEL_REASON_TIMEOUT,
    /** The transport SAR session TX retry limit is exceeded. */
    NRF_MESH_SAR_CANCEL_REASON_RETRY_OVER,
    /** There were not enough resources to process the transport SAR session. */
    NRF_MESH_SAR_CANCEL_REASON_NO_MEM,
    /** The peer cancelled the SAR session. */
    NRF_MESH_SAR_CANCEL_BY_PEER,
    /** The packet is malformed. */
    NRF_MESH_SAR_CANCEL_REASON_INVALID_FORMAT,
    /** The peer started another SAR session. */
    NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION,
    /** The friendship was terminated and the ongoing RX could not continue. */
    NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_TERMINATED,
    /** The friendship was established and the ongoing RX could not continue. */
    NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_ESTABLISHED,
    /** The LPN did not receive all segments before @ref NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE. */
    NRF_MESH_SAR_CANCEL_REASON_LPN_RX_NOT_COMPLETE
} nrf_mesh_sar_session_cancel_reason_t;

/**
 * SAR failed event structure.
 */
typedef struct
{
    /** Packet ID of the SAR session. */
    nrf_mesh_tx_token_t token;
    /** Reason for closing the session. */
    nrf_mesh_sar_session_cancel_reason_t reason;
} nrf_mesh_evt_sar_failed_t;

/**
 * User tokens for the flash manager.
 */
typedef enum
{
    /** Mesh core flash user. */
    NRF_MESH_FLASH_USER_CORE,
    /** Device state manager flash user. */
    NRF_MESH_FLASH_USER_DEVICE_STATE_MANAGER,
    /** Access layer flash user. */
    NRF_MESH_FLASH_USER_ACCESS
} nrf_mesh_flash_user_module_t;

typedef struct
{
    /** The module the event is reported from. */
    nrf_mesh_flash_user_module_t user;
    /** The flash entry that failed. */
    const void * p_flash_entry;
    /** The address of the flash page the attempted operation failed. */
    void * p_flash_page;
    /** The start of the area owned by the flash manager of the module reporting the event. */
    const void * p_area;
    /** The number of pages provided to the flash manager of the module reporting the event. */
    uint32_t page_count;
} nrf_mesh_evt_flash_failed_t;

typedef struct
{
    mesh_config_entry_id_t id; /**< ID being stored when the storage failure occured. */
} nrf_mesh_evt_config_storage_failure_t;

typedef struct
{
    mesh_config_load_failure_t reason; /**< Reason for the load failure. */
    mesh_config_entry_id_t id;         /**< ID being loaded when the load failure occured. */
    const void * p_data;               /**< Failing data. */
    uint32_t data_len;                 /**< Length of the failing data. */
} nrf_mesh_evt_config_load_failure_t;

typedef struct
{
    /** Source (element) address of the Friend node. */
    uint16_t src;
    /** Friend Offer parameters. */
    struct
    {
        /** Number of Friend Offer messages the Friend node has sent in its lifetime. */
        uint16_t friend_counter;
        /** Receive window supported by the Friend node.  */
        uint8_t receive_window_ms;
        /** Size of the message queue the Friend has reserved for messages to the LPN. */
        uint8_t friend_queue_size;
        /** Number of subscriptions the Friend can store for the LPN. */
        uint8_t subscription_list_size;
        /** The RSSI of the Friend Request as measured by the Friend. */
        int8_t measured_rssi;
    } offer;
    /** Network layer security material used in the decryption of the payload. */
    const nrf_mesh_network_secmat_t * p_net;
    /** Metadata for the received packet. */
    const nrf_mesh_rx_metadata_t * p_metadata;
} nrf_mesh_evt_lpn_friend_offer_t;

typedef struct
{
    /** Network layer security material used in the decryption of the payload. */
    const nrf_mesh_network_secmat_t * p_secmat_net;
    /** The Key Refresh Flag indicates whether the Key Refresh procedure is in progress (see Section 3.10.4). */
    bool key_refresh_in_phase2;
    /** The IV Update Flag indicates whether the IV Update procedure is in progress (see Section 3.10.5). */
    bool iv_update_active;
    /** The field is set to indicate whether the Friend Queue is empty or not. */
    bool is_friend_queue_empty;
    /** The IV Index field contains the current IV Index. */
    uint32_t iv_index;
} nrf_mesh_evt_lpn_friend_update_t;

/**
 * Device role in the friendship.
 */
typedef enum
{
    /** Friend role. */
    NRF_MESH_FRIENDSHIP_ROLE_FRIEND,
    /** Low Power node role. */
    NRF_MESH_FRIENDSHIP_ROLE_LPN
} nrf_mesh_friendship_role_t;

typedef struct
{
    /** Source device role for the event. */
    nrf_mesh_friendship_role_t role;
    /** Source (element) address of the Low Power node. */
    uint16_t lpn_src;
    /** Source (element) address of the Friend node. */
    uint16_t friend_src;
} nrf_mesh_evt_friendship_established_t;

typedef enum
{
    /** The Low Power node actively terminated the friendship. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_LPN,
    /** There was no response from the LPN within the Poll Timeout. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_TIMEOUT,
    /** The Friend node did not reply to the (repeated) Friend Poll. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NO_REPLY,
    /** The Low Power node was not able to send transport command due to internal fault. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_INTERNAL_TX_FAILED,
    /** A new Friend Request was received from an LPN. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NEW_FRIEND_REQUEST,
    /** The friendship was terminated through the API. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_USER,
    /** The LPN established a friendship with a different Friend. */
    NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NEW_FRIEND
} nrf_mesh_evt_friendship_terminated_reason_t;

typedef struct
{
    /** Source role of the event. */
    nrf_mesh_friendship_role_t role;
    /** Source (element) address of the Low Power node. */
    uint16_t lpn_src;
    /** Source (element) address of the Friend node. */
    uint16_t friend_src;
    /** Reason for friendship termination. */
    nrf_mesh_evt_friendship_terminated_reason_t reason;
} nrf_mesh_evt_friendship_terminated_t;

typedef struct
{
    /** Pointer to the friendship parameter structure. */
    const mesh_friendship_t * p_friendship;
    /** Network layer security material used in the decryption of the payload. */
    const nrf_mesh_network_secmat_t * p_net;
    /** Metadata for the received packet. */
    const nrf_mesh_rx_metadata_t * p_metadata;
} nrf_mesh_evt_friend_request_t;

/**
 * Mesh event structure.
 */
typedef struct
{
    /** Type of event. */
    nrf_mesh_evt_type_t type;

    /** Event parameters. */
    union {
        /** Incoming message. */
        nrf_mesh_evt_message_t                  message;
        /** Transmission complete. */
        nrf_mesh_evt_tx_complete_t              tx_complete;
        /** IV update notification event. */
        nrf_mesh_evt_iv_update_notification_t   iv_update;
        /** Key refresh notification event. */
        nrf_mesh_evt_key_refresh_notification_t key_refresh;
        /** A network beacon was received. */
        nrf_mesh_evt_net_beacon_received_t      net_beacon;
        /** HB message received/sent event. */
        nrf_mesh_evt_hb_message_t               hb_message;
        /** HB subscription state changed. */
        nrf_mesh_evt_hb_subscription_change_t   hb_subscription_change;

        /** DFU event. */
        nrf_mesh_evt_dfu_t                      dfu;
        /** RX failed. */
        nrf_mesh_evt_rx_failed_t                rx_failed;
        /** SAR failed event. */
        nrf_mesh_evt_sar_failed_t               sar_failed;
        /** Flash failed event. */
        nrf_mesh_evt_flash_failed_t             flash_failed;
        /** Configuration storage failure event. */
        nrf_mesh_evt_config_storage_failure_t   config_storage_failure;
        /** Configuration load failure event. */
        nrf_mesh_evt_config_load_failure_t      config_load_failure;
        /** Friend Offer event. */
        nrf_mesh_evt_lpn_friend_offer_t         friend_offer;
        /** Friend Update event. */
        nrf_mesh_evt_lpn_friend_update_t        friend_update;
        /** Friendship established event. */
        nrf_mesh_evt_friendship_established_t   friendship_established;
        /** Friendship terminated event. */
        nrf_mesh_evt_friendship_terminated_t    friendship_terminated;
        /** Friend Request event. */
        nrf_mesh_evt_friend_request_t           friend_request;
    } params;
} nrf_mesh_evt_t;

/**
 * Mesh event handler callback type.
 *
 * To forward mesh events to the application, register a callback of this type using
 * the function @ref nrf_mesh_evt_handler_add().
 *
 * @param[in] p_evt Mesh event pointer.
 *
 * @see nrf_mesh_evt_handler_add()
 */
typedef void (*nrf_mesh_evt_handler_cb_t)(const nrf_mesh_evt_t * p_evt);

/**
 * Mesh event handler context structure.
 *
 * @note This structure must be statically allocated.
 */
typedef struct
{
    /** Callback function pointer. */
    nrf_mesh_evt_handler_cb_t evt_cb;
    /** Node for the keeping in linked list. Set and used internally. */
    list_node_t node;
    /** Boolean to ensure list integrity. Set and used internally. */
    bool is_removed;
} nrf_mesh_evt_handler_t;

/**
 * Register an event handler to get events from the core stack.
 *
 * @todo Allow masking out certain events.
 *
 * @param[in,out] p_handler_params Event handler parameters.
 */
void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params);

/**
 * Remove an event handler.
 *
 * @param[in,out] p_handler_params Event handler parameters.
 */
void nrf_mesh_evt_handler_remove(nrf_mesh_evt_handler_t * p_handler_params);

/** @} */

#endif /* MESH_EVT_H__ */

