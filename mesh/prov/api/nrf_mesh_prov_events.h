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

#ifndef NRF_MESH_PROV_EVENTS_H__
#define NRF_MESH_PROV_EVENTS_H__

#include "nrf_mesh_prov_types.h"
#include "nrf_mesh_prov_bearer.h"

/**
 * @defgroup NRF_MESH_PROV_EVENTS Events
 * @ingroup  NRF_MESH_PROV
 * Provisioning event definitions.
 * @{
 */

typedef enum
{
    /** Received an unprovisioned node beacon.
     * Event parameters: @ref nrf_mesh_prov_evt_t::unprov */
    NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED,
    /** Provisioning link established.
     * Event parameters: @ref nrf_mesh_prov_evt_t::link_established */
    NRF_MESH_PROV_EVT_LINK_ESTABLISHED,
    /** Provisioning link lost.
     * Event parameters: @ref nrf_mesh_prov_evt_t::link_closed */
    NRF_MESH_PROV_EVT_LINK_CLOSED,
    /** Provisioning invite received.
     * Event parameters: @ref nrf_mesh_prov_evt_t::invite_received */
    NRF_MESH_PROV_EVT_INVITE_RECEIVED,
    /** Provisioning start received.
     * Event parameters: @ref nrf_mesh_prov_evt_t::start_received */
    NRF_MESH_PROV_EVT_START_RECEIVED,
    /** Provisioning output request.
     * Event parameters: @ref nrf_mesh_prov_evt_t::output_request */
    NRF_MESH_PROV_EVT_OUTPUT_REQUEST,
    /** Provisioning input request. Reply to this event with
     * @ref nrf_mesh_prov_auth_data_provide().
     * Event parameters: @ref nrf_mesh_prov_evt_t::input_request */
    NRF_MESH_PROV_EVT_INPUT_REQUEST,
    /** Provisioning static data request. Reply to this event with
     * @ref nrf_mesh_prov_auth_data_provide().
     * Event parameters: @ref nrf_mesh_prov_evt_t::static_request */
    NRF_MESH_PROV_EVT_STATIC_REQUEST,
    /** OOB public key requested. Reply to this event with
     * @ref nrf_mesh_prov_pubkey_provide().
     * Event parameters: @ref nrf_mesh_prov_evt_t::oob_pubkey_request */
    NRF_MESH_PROV_EVT_OOB_PUBKEY_REQUEST,
    /** Provisionee capabilities received.
     * Event parameters: @ref nrf_mesh_prov_evt_t::oob_caps_received */
    NRF_MESH_PROV_EVT_CAPS_RECEIVED,
    /** Provisioning completed.
     * Event parameters: @ref nrf_mesh_prov_evt_t::complete */
    NRF_MESH_PROV_EVT_COMPLETE,
    /** ECDH calculation requested. Reply to this event with
     * @ref nrf_mesh_prov_shared_secret_provide().
     * Event parameters: @ref nrf_mesh_prov_evt_t::ecdh_request */
    NRF_MESH_PROV_EVT_ECDH_REQUEST,
    /** Provisioning failed message received.
     * Event parameters: @ref nrf_mesh_prov_evt_t::failed */
    NRF_MESH_PROV_EVT_FAILED
} nrf_mesh_prov_evt_type_t;

/**
 * Unprovisioned node beacon received event structure.
 */
typedef struct
{
    /** Device UUID of the unprovisioned node. */
    uint8_t device_uuid[NRF_MESH_UUID_SIZE];
    /** Provisioning over GATT supported by the unprovisioned node.  */
    bool gatt_supported;
    /** Whether the beacon has a URI hash or not. */
    bool uri_hash_present;
    /** Hash of the URI in the beacon, or 0 if no URI is present. */
    uint8_t uri_hash[NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE];
    /** Metadata for the received packet. */
    const nrf_mesh_rx_metadata_t * p_metadata;
} nrf_mesh_prov_evt_unprov_t;

/**
 * Provisioning link established event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_prov_evt_link_established_t;

/**
 * Provisioning link closed event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Reason for closing the link. */
    nrf_mesh_prov_link_close_reason_t close_reason;
} nrf_mesh_prov_evt_link_closed_t;

/**
 * Provisioning invite event
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Time in seconds during which the device will identify itself using any means it can. */
    uint8_t attention_duration_s;
} nrf_mesh_prov_evt_invite_received_t;

/**
 * Provisioning start event
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_prov_evt_start_received_t;

/**
 * Provisioning input requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Requested action. When the device is acting as **provisionee**, the action field
     * contains value from @ref nrf_mesh_prov_input_action_t. When the device is acting as
     * **provisioner**, the action field contains value from @ref nrf_mesh_prov_output_action_t. */
    uint8_t action;
    /** Size of the input data requested. */
    uint8_t size;
} nrf_mesh_prov_evt_input_request_t;

/**
 * Provisioning output requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Requested action. When the device is acting as **provisionee**, the action field
     * contains value from @ref nrf_mesh_prov_output_action_t. When the device is acting as
     * **provisioner**, the action field contains value from @ref nrf_mesh_prov_input_action_t. */
    uint8_t action;
    /** Size of the output data provided. */
    uint8_t size;
    /** Pointer to the data to output. */
    const uint8_t * p_data;
} nrf_mesh_prov_evt_output_request_t;

/**
 * Static provisioning data requested event.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_prov_evt_static_request_t;

/**
 * OOB authentication capabilities received from the provisionee.
 *
 * When this event is received, the application must provide the provisioning
 * stack with the OOB authentication mechanism to use via the
 * @ref nrf_mesh_prov_oob_use() function.
 *
 * @note Only _provisioner_ devices will receive this event. To set the OOB
 * capabilities for a _provisionee_ device, specify the capabilities when
 * initializing the provisioning stack.
 *
 * @see nrf_mesh_prov_oob_use()
 */
typedef struct
{
    /** Provisioning context where the capabilities were received. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Capabilities reported by the provisionee node. */
    nrf_mesh_prov_oob_caps_t oob_caps;
} nrf_mesh_prov_evt_caps_received_t;

/**
 * Provisioning complete event. Signals the completion of a provisioning session.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Device key of the provisioned device. This must be copied into static memory before use. */
    const uint8_t * p_devkey;
    /** Pointer to provisioning data structure. */
    const nrf_mesh_prov_provisioning_data_t * p_prov_data;
} nrf_mesh_prov_evt_complete_t;

/**
 * Provisioning failed event. Signals that a Provisioning Failed message has been
 * sent or received because of a provisioning protocol error.
 */
typedef struct
{
    /** Provisioning context pointer. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Failure code indicating which error occured. */
    nrf_mesh_prov_failure_code_t failure_code;
} nrf_mesh_prov_evt_failed_t;

/**
 * Out-of-band public key requested event. Upon receiving this event, a provisioner
 * should read the public key from a node and provide it to the provisioning module
 * through the nrf_mesh_prov_pubkey_provide() function.
 */
typedef struct
{
    /** Provisioning context. */
    nrf_mesh_prov_ctx_t * p_context;
} nrf_mesh_prov_evt_oob_pubkey_request_t;

/**
 * Request for the application to perform the ECDH calculation. This event will only
 * be emitted if ECDH offloading is enabled by the appropriate option with
 * @ref nrf_mesh_opt_set().
 */
typedef struct
{
    /** Provisioning context. */
    nrf_mesh_prov_ctx_t * p_context;
    /** Pointer to the public key of the peer node. */
    const uint8_t * p_peer_public;
    /** Private key of this node. */
    const uint8_t * p_node_private;
} nrf_mesh_prov_evt_ecdh_request_t;

/** Provisioning event structure. */
typedef struct
{
    nrf_mesh_prov_evt_type_t type;
    /* Event paramenters. */
    union
    {
        /** Unprovisioned beacon received event (@ref NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED). */
        nrf_mesh_prov_evt_unprov_t              unprov;
        /** Provisioning link established event (@ref NRF_MESH_PROV_EVT_LINK_ESTABLISHED). */
        nrf_mesh_prov_evt_link_established_t    link_established;
        /** Provisioning link lost event (@ref NRF_MESH_PROV_EVT_LINK_CLOSED). */
        nrf_mesh_prov_evt_link_closed_t         link_closed;
        /** Provisioning invite event (@ref NRF_MESH_PROV_EVT_INVITE_RECEIVED). */
        nrf_mesh_prov_evt_invite_received_t     invite_received;
        /** Provisioning start event (@ref NRF_MESH_PROV_EVT_START_RECEIVED). */
        nrf_mesh_prov_evt_start_received_t      start_received;
        /** Provisioning input requested (@ref NRF_MESH_PROV_EVT_INPUT_REQUEST). */
        nrf_mesh_prov_evt_input_request_t       input_request;
        /** Provisioning output requested (@ref NRF_MESH_PROV_EVT_OUTPUT_REQUEST). */
        nrf_mesh_prov_evt_output_request_t      output_request;
        /** Static provisioning data requested (@ref NRF_MESH_PROV_EVT_STATIC_REQUEST). */
        nrf_mesh_prov_evt_static_request_t      static_request;
        /** OOB public key requested (@ref NRF_MESH_PROV_EVT_OOB_PUBKEY_REQUEST). */
        nrf_mesh_prov_evt_oob_pubkey_request_t  oob_pubkey_request;
        /** Provisioning capabilities received (@ref NRF_MESH_PROV_EVT_CAPS_RECEIVED). */
        nrf_mesh_prov_evt_caps_received_t       oob_caps_received;
        /** Provisioning complete (@ref NRF_MESH_PROV_EVT_COMPLETE). */
        nrf_mesh_prov_evt_complete_t            complete;
        /** ECDH request (@ref NRF_MESH_PROV_EVT_ECDH_REQUEST). */
        nrf_mesh_prov_evt_ecdh_request_t        ecdh_request;
        /** Provisioning failed (@ref NRF_MESH_PROV_EVT_FAILED). */
        nrf_mesh_prov_evt_failed_t              failed;
    } params;
} nrf_mesh_prov_evt_t;

/**
 * Provisioning event handler callback type.
 *
 * @param[in] p_evt Provisioning event pointer.
 */
typedef void (*nrf_mesh_prov_evt_handler_cb_t)(const nrf_mesh_prov_evt_t * p_evt);

/** @} end of NRF_MESH_PROV_EVENTS */

#endif /* NRF_MESH_PROV_EVENTS_H__ */
