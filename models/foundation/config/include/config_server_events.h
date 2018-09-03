/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef CONFIG_SERVER_EVENTS_H__
#define CONFIG_SERVER_EVENTS_H__

#include <stdint.h>
#include "device_state_manager.h"
#include "access.h"
#include "heartbeat.h"

/**
 * @defgroup CONFIG_SERVER_EVENTS Configuration Server application events
 * @ingroup CONFIG_SERVER
 * @{
 */

typedef enum
{
    /** A new application key was added. */
    CONFIG_SERVER_EVT_APPKEY_ADD,
    /** An existing application key was updated. */
    CONFIG_SERVER_EVT_APPKEY_UPDATE,
    /** The publication paremeters for a given model was set. */
    CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET,
    /** The given application key was deleted. */
    CONFIG_SERVER_EVT_APPKEY_DELETE,
    /** Secure network beacon parameters was set. */
    CONFIG_SERVER_EVT_BEACON_SET,
    /** A new default TTL value was set. */
    CONFIG_SERVER_EVT_DEFAULT_TTL_SET,
    /** Friendship parameters was set (not supported). */
    CONFIG_SERVER_EVT_FRIEND_SET,
    /** GATT proxy parameters was set (not supported). */
    CONFIG_SERVER_EVT_GATT_PROXY_SET,
    /** Key refresh phase was set. */
    CONFIG_SERVER_EVT_KEY_REFRESH_PHASE_SET,
    /** Publication to a virtual address for a given model was set. */
    CONFIG_SERVER_EVT_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET,
    /** A subscription was added to the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD,
    /** A subscription was deleted from the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE,
    /** All subscriptions was deleted for the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE_ALL,
    /** All subscriptions was overwritten by a new subscription for the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_OVERWRITE,
    /** A subscription to a virtual address was added to the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD,
    /** A subscription to a virtual address was removed from the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE,
    /** All subscriptions was overwritten by a new subscription to a virtual address for the given model. */
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE,
    /** Core network transmission parameters was set. */
    CONFIG_SERVER_EVT_NETWORK_TRANSMIT_SET,
    /** Core relay parameters was set. */
    CONFIG_SERVER_EVT_RELAY_SET,
    /** Low power node poll timeout was set (not supported). */
    CONFIG_SERVER_EVT_LOW_POWER_NODE_POLLTIMEOUT_SET,
    /** Heartbeat publication parameters was set. */
    CONFIG_SERVER_EVT_HEARTBEAT_PUBLICATION_SET,
    /** Heartbeat subscription parameters was set. */
    CONFIG_SERVER_EVT_HEARTBEAT_SUBSCRIPTION_SET,
    /** The given model was bound to a new application key. */
    CONFIG_SERVER_EVT_MODEL_APP_BIND,
    /** The given model was unbound from an application key. */
    CONFIG_SERVER_EVT_MODEL_APP_UNBIND,
    /** A new network key was added. */
    CONFIG_SERVER_EVT_NETKEY_ADD,
    /** A network key was deleted. */
    CONFIG_SERVER_EVT_NETKEY_DELETE,
    /** A network key was updated. */
    CONFIG_SERVER_EVT_NETKEY_UPDATE,
    /** The Node Identity was set (not supported). */
    CONFIG_SERVER_EVT_NODE_IDENTITY_SET,
    /** The node was reset, i.e., all mesh state cleared. */
    CONFIG_SERVER_EVT_NODE_RESET
} config_server_evt_type_t;

/** Config server Appkey Add event parameter structure. */
typedef struct
{
    /** Application key handle. */
    dsm_handle_t appkey_handle;
} config_server_evt_appkey_add_t;

/** Config server Appkey Update event parameter structure. */
typedef struct
{
    /** Application key handle. */
    dsm_handle_t appkey_handle;
} config_server_evt_appkey_update_t;

/** Config server Model Publication Set event parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
} config_server_evt_model_publication_set_t;

/** Config server Appkey Delete event parameter structure. */
typedef struct
{
    /** Application key handle. */
    dsm_handle_t appkey_handle;
} config_server_evt_appkey_delete_t;

/** Config server Beacon State event parameter structure. */
typedef struct
{
    /** Current state of the secure network broadcast beacon. @c true if active, @c false
     * otherwise. */
    bool beacon_state;
} config_server_evt_beacon_set_t;

/** Config server Default TTL Set event parameter structure. */
typedef struct
{
    /** Default TTL value used for publishing messages. */
    uint8_t default_ttl;
} config_server_evt_default_ttl_set_t;

/** Config server Key Refresh Phase Set event parameter structure. */
typedef struct
{
    /** Current key refresh phase. */
    nrf_mesh_key_refresh_phase_t kr_phase;
} config_server_evt_key_refresh_phase_set_t;

/** Config server event Subscription Add parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
    /** DSM address handle. */
    dsm_handle_t address_handle;
} config_server_evt_model_subscription_add_t;

/** Config server Subscription Delete event parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
    /** DSM address handle. */
    dsm_handle_t address_handle;
} config_server_evt_model_subscription_delete_t;

/** Config server Subscription Delete All event parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
} config_server_evt_model_subscription_delete_all_t;

/** Config server Subscription Overwrite event parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
    /** DSM address handle. */
    dsm_handle_t address_handle;
} config_server_evt_model_subscription_overwrite_t;

/** Config server Network Transmit Set event parameter structure. */
typedef struct
{
    /** Number of retransmissions for each network PDU. */
    uint8_t retransmit_count;
    /** Number of interval steps between each re-transmission (10 ms/step). */
    uint8_t interval_steps;
} config_server_evt_network_transmit_set_t;

/** Config server Relay Set event parameter structure. */
typedef struct
{
    /** Relay enabled if @c true, otherwise disabled. */
    bool enabled;
    /** Number of re-transmissions per relayed message. */
    uint8_t retransmit_count;
    /** Number of interval steps between each re-transmission (10 ms/step). */
    uint8_t interval_steps;
} config_server_evt_relay_set_t;

/** Config server Heartbeat Publication Set event parameter structure. */
typedef struct
{
    const heartbeat_publication_state_t * p_publication_state;
} config_server_evt_heartbeat_publication_set_t;

/** Config server Heartbeat Subscription Set event parameter structure. */
typedef struct
{
    const heartbeat_subscription_state_t * p_subscription_state;
} config_server_evt_heartbeat_subscription_set_t;

/** Config server Model Application Bind event parameter structure. */
typedef struct
{
    /** Access model handle. */
    access_model_handle_t model_handle;
    /** Application key handle. */
    dsm_handle_t appkey_handle;
} config_server_evt_model_app_bind_t;

/** Config server Model Application Unbind event parameter structure. */
typedef struct
{
    /* Access model handle. */
    access_model_handle_t model_handle;
    /** Application key handle. */
    dsm_handle_t appkey_handle;
} config_server_evt_model_app_unbind_t;

/** Config server Network Key Add event parameter structure. */
typedef struct
{
    /** New network key's handle. */
    dsm_handle_t netkey_handle;
} config_server_evt_netkey_add_t;

/** Config server Network Key delete event parameter structure. */
typedef struct
{
    /** Deleted network key's handle.*/
    dsm_handle_t netkey_handle;
} config_server_evt_netkey_delete_t;

/** Config server Network Key Update event parameter structure. */
typedef struct
{
    /** Updated network key's handle. */
    dsm_handle_t netkey_handle;
} config_server_evt_netkey_update_t;

/** Configuration server event structure. */
typedef struct
{
    /** Type of event. */
    config_server_evt_type_t type;
    /** Union of event parameters. */
    union
    {
        config_server_evt_appkey_add_t appkey_add;
        config_server_evt_appkey_update_t appkey_update;
        config_server_evt_model_publication_set_t model_publication_set;
        config_server_evt_appkey_delete_t appkey_delete;
        config_server_evt_beacon_set_t beacon_set;
        config_server_evt_default_ttl_set_t default_ttl_set;
        config_server_evt_key_refresh_phase_set_t key_refresh_phase_set;
        config_server_evt_model_subscription_add_t model_subscription_add;
        config_server_evt_model_subscription_delete_t model_subscription_delete;
        config_server_evt_model_subscription_delete_all_t model_subscription_delete_all;
        config_server_evt_model_subscription_overwrite_t model_subscription_overwrite;
        config_server_evt_network_transmit_set_t network_transmit_set;
        config_server_evt_relay_set_t relay_set;
        config_server_evt_heartbeat_publication_set_t heartbeat_publication_set;
        config_server_evt_heartbeat_subscription_set_t heartbeat_subscription_set;
        config_server_evt_model_app_bind_t model_app_bind;
        config_server_evt_model_app_unbind_t model_app_unbind;
        config_server_evt_netkey_add_t netkey_add;
        config_server_evt_netkey_delete_t netkey_delete;
        config_server_evt_netkey_update_t netkey_update;
    } params;
} config_server_evt_t;

/**
 * Config server event callback type.
 * @param[in] p_evt Event pointer from the configuration server.
 */
typedef void (*config_server_evt_cb_t)(const config_server_evt_t * p_evt);

/** @} end of CONFIG_SERVER_EVENTS */

#endif /* CONFIG_SERVER_EVENTS_H__ */
