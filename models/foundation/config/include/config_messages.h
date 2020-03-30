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

#ifndef CONFIG_MESSAGES_H__
#define CONFIG_MESSAGES_H__

#include <stddef.h>
#include <stdint.h>

#include "nrf_mesh_assert.h"
#include "access.h"

/**
 * @defgroup CONFIG_MESSAGES Message formats
 * @ingroup CONFIG_MODEL
 * Message format definitions for the Config models.
 * @{
 */

/** Maximum re-transmit count. */
#define CONFIG_RETRANSMIT_COUNT_MAX          ((1 << 3) - 1)
/** Maximum number of re-transmit interval steps. */
#define CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX ((1 << 5) - 1)
/** Gets the retransmit interval in milliseconds from the number of steps. */
#define CONFIG_RETRANSMIT_INTERVAL_STEP_TO_MS(STEP) ((STEP) * 10)
/** Gets the retransmit interval steps from the interval in milliseconds. */
#define CONFIG_RETRANSMIT_INTERVAL_MS_TO_STEP(MS)   ((MS) / 10)

/*lint -align_max(push) -align_max(1) */

/**
 * 24-bit key index type.
 * Provides two 12-bit key indexes in three bytes. Use the dedicated functions
 * to manipulate this structure.
 * @see config_msg_key_index_24_set(), config_msg_key_index_24_get()
 */
typedef struct __attribute((packed))
{
    uint8_t key_id_1_lsb;     /**< 8 LSB of the first key index. */
    uint8_t key_id_1_msb : 4; /**< 4 MSB of the first key index. */
    uint8_t key_id_2_lsb : 4; /**< 4 LSB of the second key index. */
    uint8_t key_id_2_msb;     /**< 8 MSB of the second key index. */
} config_msg_key_index_24_t;

/* Ensure the size of the above type is correct: */
NRF_MESH_STATIC_ASSERT(sizeof(config_msg_key_index_24_t) == 3);

/**
 * Sets the value of the keys in a 24-bit index struct.
 * @param[out] p_idx24  Pointer to a 24-bit key index structure.
 * @param[in]  key_id_1 Value to assign to the first key index in the struct.
 * @param[in]  key_id_2 Value to assign th the second key index in the struct.
 */
static inline void config_msg_key_index_24_set(config_msg_key_index_24_t * p_idx24, uint16_t key_id_1, uint16_t key_id_2)
{
    p_idx24->key_id_1_lsb = key_id_1 & 0xff;
    p_idx24->key_id_1_msb = (key_id_1 >> 8) & 0xf;

    p_idx24->key_id_2_lsb = key_id_2 & 0xf;
    p_idx24->key_id_2_msb = (key_id_2 >> 4) & 0xff;
}

/**
 * Gets the value of one or both of the keys in a 24-bit index struct.
 * @param[in]  p_idx24    Pointer to a 24-bit key index structure.
 * @param[out] p_key_id_1 Pointer to where the first key index in the struct should be returned.
 *                        This parameter can be set to @c NULL to skip retrieving the first key ID.
 * @param[out] p_key_id_2 Pointer to where the second key index in the struct should be returned.
 *                        This parameter can be set to @c NULL to skip retrieveing the second key ID.
 */
static inline void config_msg_key_index_24_get(const config_msg_key_index_24_t * p_idx24, uint16_t * p_key_id_1, uint16_t * p_key_id_2)
{
    if (p_key_id_1 != NULL)
    {
        *p_key_id_1 = p_idx24->key_id_1_lsb | (p_idx24->key_id_1_msb << 8);
    }

    if (p_key_id_2 != NULL)
    {
        *p_key_id_2 = p_idx24->key_id_2_lsb | (p_idx24->key_id_2_msb << 4);
    }
}

/** Mask used for setting or extracting 12-bit key indexes. */
#define CONFIG_MSG_KEY_INDEX_12_MASK    0x0fff

/**
 * 12-bit key index type.
 * This provides room to store one 12-bit key in two bytes. Use the dedicated functions
 * to manipulate this structure.
 */
typedef uint16_t config_msg_key_index_12_t;

/**
 * Model ID type for configuration messages.
 * This is used in messages that can either contain a 16-bit SIG identifier or
 * a 32-bit vendor-specific model ID. For SIG identifiers the company_id is ignored.
 */
typedef union __attribute((packed))
{
    struct __attribute((packed))
    {
        uint16_t model_id;     /**< Model ID. */
    } sig;
    struct __attribute((packed))
    {
        uint16_t company_id;   /**< Vendor-specific company ID. */
        uint16_t model_id;     /**< Model ID. */
    } vendor;
} config_model_id_t;
NRF_MESH_STATIC_ASSERT(sizeof(config_model_id_t) == sizeof(uint32_t));

/**
 * Sets the value of the model id for different kind of models (SIG or vendor).
 * @param[out] p_dst  Pointer to the packed payload.
 * @param[in]  p_src  Pointer to the access model memory .
 * @param[in]  is_sig Type of the model. True for SIG, false for vendor models.
 */
static inline void config_msg_model_id_set(config_model_id_t * p_dst, const access_model_id_t * p_src, bool is_sig)
{
    if (is_sig)
    {
        p_dst->sig.model_id = p_src->model_id;
    }
    else
    {
        p_dst->vendor.model_id = p_src->model_id;
        p_dst->vendor.company_id = p_src->company_id;
    }
}

/** Message format for the AppKey Add message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; /**< Pair containing a netkey and an appkey index. */
    uint8_t appkey[NRF_MESH_KEY_SIZE];     /**< Application key data. */
} config_msg_appkey_add_t;

/** Message format for the AppKey Update message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; /**< Pair containing a netkey and an appkey index. */
    uint8_t appkey[NRF_MESH_KEY_SIZE];     /**< Application key data. */
} config_msg_appkey_update_t;

/** Message format for the AppKey Delete message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; /**< Pair containing a netkey and an appkey index. */
} config_msg_appkey_delete_t;

/** Message format for the AppKey Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                        /**< Status code. */
    config_msg_key_index_24_t key_indexes; /**< Pair containing a netkey and an appkey index. */
} config_msg_appkey_status_t;

/** Message format for the AppKey Get message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Network key to report application keys for. */
} config_msg_appkey_get_t;

/** Message format for the AppKey List message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Network key index. */
    uint8_t packed_appkey_indexes[];        /**< Packed list of application key indexes. */
} config_msg_appkey_list_t;

/** Message format for the Default TTL Set message. */
typedef struct __attribute((packed))
{
    uint8_t ttl;            /**< Default TTL value. */
} config_msg_default_ttl_set_t;

/** Message format for the Default TTL Status message. */
typedef struct __attribute((packed))
{
    uint8_t ttl;            /**< Default TTL value. */
} config_msg_default_ttl_status_t;

/** Possible values for the network beacon state. */
typedef enum
{
    CONFIG_NET_BEACON_STATE_DISABLED = 0,   /**< The network beacon is disabled. */
    CONFIG_NET_BEACON_STATE_ENABLED = 1     /**< The network beacon is enabled. */
} config_net_beacon_state_t;

/** Message format for the Config Beacon Set message. */
typedef struct __attribute((packed))
{
    uint8_t beacon_state;      /**< Beacon state. */
} config_msg_net_beacon_set_t;

/** Message format for the Config Beacon Status message. */
typedef struct __attribute((packed))
{
    uint8_t beacon_state;      /**< Beacon state. */
} config_msg_net_beacon_status_t;

/** Publication parameters. */
typedef struct __attribute((packed))
{
    uint16_t appkey_index : 12;                   /**< Application key index. */
    uint16_t credential_flag : 1;                 /**< Friendship credentials flag. */
    uint16_t rfu : 3;                             /**< Reserved for future use, set to 0. */
    uint8_t  publish_ttl;                         /**< TTL for outgoing messages. */
    uint8_t  publish_period;                      /**< Period for periodic publishing. */
    uint8_t  retransmit_count : 3;                /**< Number of retransmissions of each message. */
    uint8_t  retransmit_interval : 5;             /**< Number of 50 ms steps between each retransmission. */
    config_model_id_t  model_id;                  /**< Model identifier. */
} config_publication_params_t;

/** Message format for the Model Publication Get message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;   /**< Address of the element. */
    config_model_id_t model_id; /**< Identifier of the model. */
} config_msg_publication_get_t;

/** Message format for the Model Publication Set message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;          /**< Unicast address of the element. */
    uint16_t publish_address;          /**< Publish address. */
    config_publication_params_t state; /**< The publication parameters to set. */
} config_msg_publication_set_t;

/** Message format for the Model Publication Virtual Set message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;                  /**< Unicast address of the element. */
    uint8_t  publish_uuid[NRF_MESH_UUID_SIZE]; /**< Virtual address label UUID. */
    config_publication_params_t state;         /**< The publication parameters to set. */
} config_msg_publication_virtual_set_t;

/** Message format for the Model Publication Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                    /**< Status code. */
    uint16_t element_address;          /**< Unicast address of the element. */
    uint16_t publish_address;          /**< Publish address. */
    config_publication_params_t state; /**< Current publication parameters. */
} config_msg_publication_status_t;

/** Message format for the Model Subscription Add/Delete/Overwrite messages. */
typedef struct __attribute((packed))
{
    uint16_t          element_address; /**< Address of the element. */
    uint16_t          address;         /**< Address to subscribe to. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_add_del_owr_t;

/** Message format for the Model Subscription Virtual Add/Delete/Overwrite messages. */
typedef struct __attribute((packed))
{
    uint16_t          element_address;                  /**< Address of the element. */
    uint8_t           virtual_uuid[NRF_MESH_UUID_SIZE]; /**< Label UUID for the virtual address to subscribe to. */
    config_model_id_t model_id;                         /**< ID of the model. */
} config_msg_subscription_virtual_add_del_owr_t;

/** Message format for the Model Subscription Delete All message. */
typedef struct __attribute((packed))
{
    uint16_t          element_address; /**< Address of the element. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_delete_all_t;

/** Message format for the Model Subscription Status message. */
typedef struct __attribute((packed))
{
    uint8_t           status;          /**< Status code. */
    uint16_t          element_address; /**< Address of the element. */
    uint16_t          address;         /**< Address that the model was subscribed to. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_status_t;

/** Message format for the Network Key Add/Update messages. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index;              /**< Network key index. */
    uint8_t                   netkey[NRF_MESH_KEY_SIZE]; /**< Network key contents. */
} config_msg_netkey_add_update_t;

/** Message format for the Network Key Delete message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Network key index. */
} config_msg_netkey_delete_t;

/** Message format for the Network Key Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Network key index.*/
} config_msg_netkey_status_t;

/** Possible values for the GATT Proxy state. */
typedef enum
{
    CONFIG_GATT_PROXY_STATE_RUNNING_DISABLED = 0x00, /**< The GATT proxy is running, but disabled. */
    CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED  = 0x01, /**< The GATT proxy is running and enabled. */
    CONFIG_GATT_PROXY_STATE_UNSUPPORTED      = 0x02  /**< The GATT proxy feature is not supported. */
} config_gatt_proxy_state_t;

/** Message format for the GATT Proxy Status message. */
typedef struct __attribute((packed))
{
    uint8_t proxy_state; /**< The state of the GATT proxy service. */
} config_msg_proxy_status_t;

/** Message format for the GATT Proxy Set message. */
typedef struct __attribute((packed))
{
    uint8_t proxy_state; /**< The desired state of the GATT proxy service. */
} config_msg_proxy_set_t;

/** Possible values for the Friend state. */
typedef enum
{
    CONFIG_FRIEND_STATE_SUPPORTED_DISABLED = 0x00, /**< Friendship is supported, but disabled. */
    CONFIG_FRIEND_STATE_SUPPORTED_ENABLED  = 0x01, /**< Friendship is supported and enabled. */
    CONFIG_FRIEND_STATE_UNSUPPORTED        = 0x02  /**< Friendship is not supported. */
} config_friend_state_t;

/** Message format for the Friend Set message. */
typedef struct __attribute((packed))
{
    uint8_t friend_state; /**< The desired state of the friendship feature. */
} config_msg_friend_set_t;

/** Message format for the Friend Status message. */
typedef struct __attribute((packed))
{
    uint8_t friend_state; /**< The state of the friendship feature. */
} config_msg_friend_status_t;

/** Message format for the Key Refresh Phase Get message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Index of the network to get the key refresh phase for. */
} config_msg_key_refresh_phase_get_t;

/** Message format for the Key Refresh Phase Set message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Index of the network to set the key refresh phase for. */
    uint8_t transition;                     /**< ID of the phase to transition to. */
} config_msg_key_refresh_phase_set_t;

/** Message format for the Key Refresh Phase Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Index of the network the key refresh phase is reported for. */
    uint8_t phase;                          /**< Current key refresh phase for the subnet. */
} config_msg_key_refresh_phase_status_t;

/** Message format for the Heartbeat Publication Set message*/
typedef struct __attribute((packed))
{
    uint16_t destination;                   /**< Heartbeat publication destination. */
    uint8_t count_log;                      /**< Number of heartbeat messages to be sent. */
    uint8_t period_log;                     /**< Period of transmitted heartbeat messages. */
    uint8_t ttl;                            /**< TTL for heartbeat message. */
    uint16_t features;                      /**< Features triggering heartbeat messages. */
    config_msg_key_index_12_t netkey_index; /**< Index for the network key used to send heartbeats. */
} config_msg_heartbeat_publication_set_t;

/** Message format for the Heartbeat Publication Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t destination;                   /**< Heartbeat publication destination. */
    uint8_t count_log;                      /**< Number of heartbeat messages to be sent. */
    uint8_t period_log;                     /**< Period of transmitted heartbeat messages. */
    uint8_t ttl;                            /**< TTL for heartbeat message. */
    uint16_t features;                      /**< Features triggering heartbeat messages. */
    config_msg_key_index_12_t netkey_index; /**< Index for the network key used to send heartbeats. */
} config_msg_heartbeat_publication_status_t;

/** Message format for the Heartbeat Subscription Set message. */
typedef struct __attribute((packed))
{
    uint16_t source;                        /**< Source of heartbeat messages. */
    uint16_t destination;                   /**< Destination of heartbeat messages. */
    uint8_t period_log;                     /**< Period of heartbeat messages. */
} config_msg_heartbeat_subscription_set_t;

/** Message format for the Heartbeat Subscription Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t source;                        /**< Source of heartbeat messages. */
    uint16_t destination;                   /**< Destination of heartbeat messages. */
    uint8_t period_log;                     /**< Period of heartbeat messages. */
    uint8_t count_log;                      /**< Number of heartbeat messages received. */
    uint8_t min_hops;                       /**< Least number of hops in received heartbeat messages. */
    uint8_t max_hops;                       /**< Largest number of hops in received heartbeat messages. */
} config_msg_heartbeat_subscription_status_t;

/** Message format for the Model App Bind/Unbind message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;               /**< Unicast address of the element. */
    config_msg_key_index_12_t appkey_index; /**< Application key index. */
    config_model_id_t model_id;             /**< Model ID. */
} config_msg_app_bind_unbind_t;

/** Message format for the Model App Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t element_address;               /**< Unicast address of the element. */
    config_msg_key_index_12_t appkey_index; /**< Application key index. */
    config_model_id_t model_id;             /**< Model ID. */
} config_msg_app_status_t;

/** Possible values for the identity state. */
typedef enum
{
    CONFIG_IDENTITY_STATE_STOPPED     = 0x00, /**< The node identity advertisement is stopped. */
    CONFIG_IDENTITY_STATE_RUNNING     = 0x01, /**< The node identity advertisement is running. */
    CONFIG_IDENTITY_STATE_UNSUPPORTED = 0x02  /**< Node identity advertising is not supported. */
} config_identity_state_t;

/** Message format for the Node Identity Get message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
} config_msg_identity_get_t;

/** Message format for the Node Identity Set message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
    uint8_t identity_state;                 /**< Current state of the node identity advertisement. */
} config_msg_identity_set_t;

/** Message format for the Node Identity Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
    uint8_t identity_state;                 /**< Identity advertisement state. */
} config_msg_identity_status_t;

/** Message format for the Composition Data Get message. */
typedef struct __attribute((packed))
{
    uint8_t page_number; /**< Page-number for the composition data page to retrieve. */
} config_msg_composition_data_get_t;

/** Message format for the Composition Data Status message. */
typedef struct __attribute((packed))
{
    uint8_t page_number; /**< Page-number for the composition data page contained in this response. */
    uint8_t data[];      /**< Composition data. */
} config_msg_composition_data_status_t;

/** Values for the relay state. */
typedef enum
{
    CONFIG_RELAY_STATE_SUPPORTED_DISABLED = 0x00, /**< Relaying is supported, but disabled. */
    CONFIG_RELAY_STATE_SUPPORTED_ENABLED  = 0x01, /**< Relaying is supported and enabled. */
    CONFIG_RELAY_STATE_UNSUPPORTED        = 0x02  /**< Relaying is not supported. */
} config_relay_state_t;

/** Message format for the Relay Status message. */
typedef struct __attribute((packed))
{
    uint8_t relay_state;                         /**< Current state of the relaying feature. */
    uint8_t relay_retransmit_count          : 3; /**< Number of retransmissions per relayed packet. */
    uint8_t relay_retransmit_interval_steps : 5; /**< Number of 10 ms steps between retransmissions. */
} config_msg_relay_status_t;

/** Message format for the Relay Set message. */
typedef struct __attribute((packed))
{
    uint8_t relay_state;                         /**< Desired state of the relaying feature. */
    uint8_t relay_retransmit_count          : 3; /**< Desired number of retransmissions per relayed packed. */
    uint8_t relay_retransmit_interval_steps : 5; /**< Desired number of 10 ms steps between retransmissions. */
} config_msg_relay_set_t;

/** Message format for the Network Transmit Set message. */
typedef struct __attribute((packed))
{
    uint8_t network_transmit_count          : 3; /**< Desired number of retransmissions per packed. */
    uint8_t network_transmit_interval_steps : 5; /**< Desired number of 10 ms steps between retransmissions. */
} config_msg_network_transmit_set_t;

/** Message format for the Network Transmit Status message. */
typedef struct __attribute((packed))
{
    uint8_t network_transmit_count          : 3; /**< Number of retransmissions per transmited packet. */
    uint8_t network_transmit_interval_steps : 5; /**< Number of 10 ms steps between retransmissions. */
} config_msg_network_transmit_status_t;

/** Message format for the SIG/Vendor Model App Get message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;   /**< Unicast address of the element. */
    config_model_id_t model_id; /**< Model ID. */
} config_msg_model_app_get_t;

/** Message format for the SIG Model App List message. */
typedef struct __attribute((packed))
{
    uint8_t status;           /**< Status code. */
    uint16_t element_address; /**< Unicast address of the element. */
    uint16_t sig_model_id;    /**< SIG model ID. */
    uint8_t key_indexes[];    /**< Key indexes. */
} config_msg_sig_model_app_list_t;

/** Message format for the Vendor Model App List message. */
typedef struct __attribute((packed))
{
    uint8_t status;             /**< Status code. */
    uint16_t element_address;   /**< Unicast address of the element. */
    uint16_t vendor_company_id; /**< Vendor company ID. */
    uint16_t vendor_model_id;   /**< Vendor model ID. */
    uint8_t key_indexes[];      /**< List of application key indexes. */
} config_msg_vendor_model_app_list_t;

/** Message format for the SIG/Vendor Model Subscription Get message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;   /**< Unicast address of the element. */
    config_model_id_t model_id; /**< Model ID. */
} config_msg_model_subscription_get_t;

/** Message format for the SIG Model Subscription List message. */
typedef struct __attribute((packed))
{
    uint8_t status;           /**< Status code. */
    uint16_t element_address; /**< Unicast address of the element. */
    uint16_t sig_model_id;    /**< SIG model ID. */
    uint16_t subscriptions[]; /**< Subscription list. */
} config_msg_sig_model_subscription_list_t;

/** Message format for the Vendor Model Subscription List message. */
typedef struct __attribute((packed))
{
    uint8_t status;             /**< Status code. */
    uint16_t element_address;   /**< Unicast address of the element. */
    uint16_t vendor_company_id; /**< Vendor company ID. */
    uint16_t vendor_model_id;   /**< Vendor model ID. */
    uint16_t subscriptions[];   /**< Subscription list. */
} config_msg_vendor_model_subscription_list_t;

/** Message format for the Low Power node PollTimeout Get message. */
typedef struct __attribute((packed))
{
    uint16_t lpn_address;       /**< The unicast address of the Low Power node. */
} config_msg_low_power_node_polltimeout_get_t;

/** Message format for the Low Power node PollTimeout Status message. */
typedef struct __attribute((packed))
{
    uint16_t lpn_address;       /**< The unicast address of the Low Power node. */
    uint8_t  polltimeout[3];    /**< Poll Timeout. */
} config_msg_low_power_node_polltimeout_status_t;

/*lint -align_max(pop) */
/** @} */

#endif

