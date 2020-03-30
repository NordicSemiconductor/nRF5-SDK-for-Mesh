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

#ifndef NODE_SETUP_H__
#define NODE_SETUP_H__

#include "network_setup_types.h"
#include "nrf_mesh_prov_events.h"


/**
 * @defgroup NODE_SETUP Helper module for configuring the newly provisioned devices.
 *
 * This module serves as an example for executing various configuration steps for the newly
 * provisioned device. If required, you can extend the configurator state machine to carry out
 * custom configuration of the nodes.
 *
 * @{
 */

/** Maximum number of status message that can be checked for a given config model message */
#define MAX_COMBINED_EXPECTED_STATUS_MSGS   5

/** Number of times config client messages will be re-sent if client model is busy */
#define CLIENT_BUSY_SEND_RETRY_LIMIT     (3)

/** Delay after which config client message will be re-sent if client model is busy */
#define CLIENT_BUSY_SEND_RETRY_DELAY_MS  (5000)

/**
 * @defgroup NODE_SETUP_CALLBACKS User application callback prototypes for node setup module
 * @{
 */


/** Callback to user indicating that all steps in the node setup were completed. */
typedef void (*node_setup_successful_cb_t)(void);

/** Callback to user indicating that the node setup failed. */
typedef void (*node_setup_failed_cb_t)(void);

/** @} end of NODE_SETUP_CALLBACKS */

/**
 * Starts state machine for configuring the newly provisioned node. If this function is called
 * when previous setup process is underway it will trigger error condition.
 *
 * @param[in]  address        Unicast address of the node to be configured.
 * @param[in]  retry_cnt      Number of times a message can be resent if failed
 * @param[in]  p_appkey       Pointer to the appkey that will be used for configuring nodes
 * @param[in]  appkey_idx     Desired appkey index.
 * @param[in]  netkey_idx     Desired netkey index for the appkey.
 * @param[in]  p_uri          Pointer to the URI string
 */
void node_setup_start(uint16_t address, uint8_t  retry_cnt, const uint8_t * p_appkey,
                      uint16_t appkey_idx, uint16_t netkey_idx, const char * p_uri);

/**
 * Sets the application callbacks to be called when node setup succeeds or fails.
 *
 * @param[in]  config_success_cb    Application callback called when all configuration steps complete
 *                                  successfully.
 * @param[in]  config_failed_cb     Application callback called when configuration fails.
 */
void node_setup_cb_set(node_setup_successful_cb_t config_success_cb,
                       node_setup_failed_cb_t config_failed_cb);

/**
 * Process the config client events.
 *
 * This function must be called by the user application from config client callback,
 * and all callback params should be passed to this function without modifications.
 *
 * @param[in] event_type Event type.
 * @param[in] p_event    Pointer to event data, may be @c NULL.
 * @param[in] length     Length of the event
 */
void node_setup_config_client_event_process(config_client_event_type_t event_type,
                                            const config_client_event_t * p_event,
                                            uint16_t length);

/**
 * Get URI string by URI hash from unprovisioned beacon.
 * If provisioner is able to find URI in internal URI base that means it has configuration for device
 *
 * @param[in] p_in_uri_hash pointer to URI hash from unprovisioned beacon.
 *
 * @returns pointer to URI string if it is present, NULL otherwise.
 */
const char * node_setup_uri_get(const uint8_t * p_in_uri_hash);

/**
 *  Checks hash collision for predefined URIs of examples.
 *  If URIs have the same URI hash value then the static provisioner will not work as expected.
 *  Therefore, if this function asserts, customers should change at least one indicated URI to avoid collision of hashes.
 *  After changing URI hash, the corresponding example and provisioner example should be rebuilt.
 */
void node_setup_uri_check(void);

/** @} end of NODE_SETUP */

#endif /* NODE_SETUP_H__ */
