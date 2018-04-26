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
#ifndef MESH_HEARTBEAT_H__
#define MESH_HEARTBEAT_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "timer.h"
#include "utils.h"

#include "nrf_mesh.h"
#include "transport.h"

/**
 * @defgroup HEARTBEAT Heartbeat module
 * @ingroup MESH_CORE
 *
 * The Heartbeat module provides functionality for sending and receiving heartbeat messages.
 * Configuration of this module is controlled by two states: the Heartbeat
 * Publication state and the Heartbeat Subscription state, defined in the Mesh Profile
 * Specification v1.0.
 *
 * The Heartbeat Publication state is contained within the Config Server module, and Heartbeat Subscription state
 * is part of the Heartbeat module.
 *
 * The Config server uses @ref heartbeat_publication_state_updated() to inform the heartbeat module about
 * publication state changes and the @ref heartbeat_subscription_set() function to set the
 * subscription state. This module is also tightly integrated with the @c transport module. The module
 * registers its internal opcode handler function with the transport layer during initialization.
 *
 * @warning Currently, heartbeat publication and subscription states are not stored in flash.
 *
 * # Receiving heartbeat messages
 * The transport layer calls the registered heartbeat opcode handler  after receiving a
 * @ref TRANSPORT_CONTROL_OPCODE_HEARTBEAT message. The opcode handler then parses
 * the message to derive the initial TTL value used to send this message, calculates the hop count between
 * the current and the originating nodes, and generates
 * an @ref NRF_MESH_EVT_HB_MESSAGE_RECEIVED event for the user application.
 *
 * # Sending heartbeat messages
 * Heartbeat messages can be sent periodically by configuring the Heartbeat Publication state.
 * Heartbeat messages can also be triggered if the state of the node's
 * features (Relay, Proxy, Friendship, and LPN features) change as a result of feature
 * configuration messages or local action. To trigger heartbeat messages on feature
 * state changes, @ref heartbeat_on_feature_change_trigger() is called with the appropriate
 * parameters.
 *
 * @{
 */

#define HEARTBEAT_MAX_PERIOD_LOG (0x11)
#define HEARTBEAT_MAX_PERIOD     (1u << (HEARTBEAT_MAX_PERIOD_LOG - 1))
#define HEARTBEAT_INVALID_PERIOD (0xFFFFFFFF)

#define HEARTBEAT_MAX_COUNT_LOG  (0x11)
#define HEARTBEAT_MAX_COUNT      (1u << (HEARTBEAT_MAX_COUNT_LOG - 1))
#define HEARTBEAT_INF_COUNT      (0xFFFF)
#define HEARTBEAT_INF_COUNT_LOG  (0xFF)
#define HEARTBEAT_INVALID_COUNT  (0xFFFFFFFF)

/** Heartbeat publication state */
typedef struct
{
    /** The destination address for the heartbeat message */
    uint16_t            dst;
    /** Heartbeat publication counter value */
    uint32_t            count;
    /** Heartbeat publication period in seconds */
    uint32_t            period;
    /** Heartbeat publication ttl */
    uint8_t             ttl;

    /** The Heartbeat Publication Features state determines the features that trigger sending
     * Heartbeat messages when changed
     */
    uint16_t           features;

    /** The global NetKey Index of the NetKey used to send Heartbeat messages */
    uint16_t    netkey_index;
} heartbeat_publication_state_t;

/** Heartbeat publication information state */
typedef struct
{
    /** Heartbeat publication state data */
    const heartbeat_publication_state_t * p_publication;
    /** Local address */
    uint16_t    local_address;
    /** Pointer to valid network security material */
    const nrf_mesh_network_secmat_t * p_net_secmat;
} heartbeat_publication_information_t;

/** Heartbeat subscription state structure which controls the receiving of heartbeat messages */
typedef struct
{
    /** The unicast source address for Heartbeat messages a node shall process*/
    uint16_t  src;
    /** The destination address for Heartbeat messages */
    uint16_t  dst;
    /** The number of periodical Heartbeat transport control messages received */
    uint32_t            count;
    /** The number of seconds left for processing periodical Heartbeat transport control messages*/
    uint32_t            period;
    /** The minimum hops value registered when receiving Heartbeat messages*/
    uint16_t            min_hops;
    /** The maximum hops value registered when receiving Heartbeat messages*/
    uint16_t            max_hops;

} heartbeat_subscription_state_t;

/** Function type to fetch publication parameters for the heartbeat. */
typedef uint32_t (*heartbeat_publication_params_get_cb_t)(heartbeat_publication_information_t * p_pub_info);

/** Function type to decrement the heartbeat publication counter value. */
typedef void (*heartbeat_publication_count_decrement_cb_t)(void);

/**
 * Provides the unicast address of the current node to the heartbeat module.
 *
 * @param[in]   address     Unicast address of this node.
 */
void heartbeat_local_address_set(uint16_t address);

/**
 * Initializes the heartbeat module.
 *
 * This function should be called only once after powerup. Multiple calls to this function will
 * result in a mesh assertion.
 */
void heartbeat_init(void);

/**
 * Sets the value of internal heartbeat publication state.
 *
 * @param[in]   p_hb_pub    Pointer to strucutre holding heartbeat publication state.
 *
 * @retval      NRF_SUCCESS             If the internal state has been updated successfully
 * @retval      NRF_ERROR_INVALID_PARAM If an invalid netkey index is provided
 */
uint32_t heartbeat_publication_set(const heartbeat_publication_state_t * p_hb_pub);

/**
 * Gets the value of the internal heartbeat publication state.
 * @returns Returns a pointer to the heartbeat publication state.
 */
const heartbeat_publication_state_t * heartbeat_publication_get(void);

/**
 * Sets the value of internal heartbeat subscription state.
 *
 * @param[in]   p_hb_sub  Pointer to structure holding the new heartbeat subscription state.
 *
 * @retval      NRF_SUCCESS             If the internal state has been successfully updated.
 * @retval      NRF_ERROR_INVALID_PARAM If incorrect input values are used to set the heartbeat
 *                                      subscription state.
 */
uint32_t heartbeat_subscription_set(const heartbeat_subscription_state_t * p_hb_sub);

/**
 * Gets the value of internal heartbeat subscription state.
 * @returns Returns a pointer to the heartbeat publication state.
 */
const heartbeat_subscription_state_t * heartbeat_subscription_get(void);


/**
 * Triggers the heartbeat message when a particular feature is turned on or off.
 *
 * This function should always be called when the state of a feature is changed.
 * The Heartbeat module will decide whether to trigger the heartbeat based on the configuration of the
 * heartbeat_publication state.
 *
 * @note The heartbeat message will not be sent out immediately when this function is called.
 *       This delay is to prevent problems when the function is called from config server
 *       message handlers. The message will be sent following the next TX_COMPLETE event, at which
 *       point the current state information will be sent.
 *
 *  @param[in]  hb_trigger      Bitmask of the features that caused the heartbeat to trigger.
 *
 */
void heartbeat_on_feature_change_trigger(uint16_t hb_trigger);

/**
 * Sets the callback functions used to communicate with the config server.
 *
 * @param[in]  p_cb          Pointer to the function for retrieving the publication information.
 * @param[in]  p_pub_cnt_cb  Pointer to the function that decrements heartbeat publication count.
 */
void heartbeat_config_server_cb_set(heartbeat_publication_params_get_cb_t p_cb,
                                    heartbeat_publication_count_decrement_cb_t p_pub_cnt_cb);

/**
 * Notifies the heartbeat module about changes in the heartbeat publication state.
 */
void heartbeat_publication_state_updated(void);

/** @} */

#endif
