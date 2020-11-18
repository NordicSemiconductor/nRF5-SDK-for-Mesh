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

#include "prov_provisionee.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_error.h"

#include "enc.h"
#include "event.h"
#include "utils.h"
#include "log.h"

#include "provisioning.h"
#include "nrf_mesh_prov_bearer.h"
#include "prov_pdu.h"
#include "prov_utils.h"

#ifndef PROV_DEBUG_MODE
#define PROV_DEBUG_MODE 0
#endif

#define SECURE_PROVISIONING_MIN_AUTH_SIZE    (6)

/****************** Call-back function declarations ******************/
static void prov_provisionee_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length);
static void prov_provisionee_cb_ack_received(prov_bearer_t * p_bearer);
static void prov_provisionee_cb_link_established(prov_bearer_t * p_bearer);
static void prov_provisionee_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);
static const prov_bearer_callbacks_t m_prov_callbacks =
             {
                .rx = prov_provisionee_pkt_in,
                .ack = prov_provisionee_cb_ack_received,
                .opened = prov_provisionee_cb_link_established,
                .closed = prov_provisionee_cb_link_closed
             };

/****************** Local functions ******************/
static void send_failed(nrf_mesh_prov_ctx_t * p_ctx, nrf_mesh_prov_failure_code_t failure_code)
{
    if (NRF_SUCCESS != prov_tx_failed(p_ctx->p_active_bearer, failure_code))
    {
        p_ctx->p_active_bearer->p_interface->link_close(
            p_ctx->p_active_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
    }

    /* Don't post the same event multiple times. */
    if (p_ctx->state != NRF_MESH_PROV_STATE_FAILED)
    {
        nrf_mesh_prov_evt_t app_event;
        app_event.type = NRF_MESH_PROV_EVT_FAILED;
        app_event.params.failed.p_context =  p_ctx;
        app_event.params.failed.failure_code = failure_code;

        /* Set the original failure code. Every PDU received from this
         * point will result in an NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU.
         */
        p_ctx->failure_code = failure_code;
        p_ctx->state = NRF_MESH_PROV_STATE_FAILED;

        p_ctx->event_handler(&app_event);
    }
}

static uint32_t send_capabilities(nrf_mesh_prov_ctx_t * p_ctx)
{
    prov_pdu_caps_t pdu;
    pdu.pdu_type = PROV_PDU_TYPE_CAPABILITIES;

    pdu.num_elements = p_ctx->capabilities.num_elements;
    pdu.algorithms = LE2BE16(p_ctx->capabilities.algorithms);
    pdu.pubkey_type = p_ctx->capabilities.pubkey_type;
    pdu.oob_static_types = p_ctx->capabilities.oob_static_types;
    pdu.oob_output_size = p_ctx->capabilities.oob_output_size;
    pdu.oob_output_actions = LE2BE16(p_ctx->capabilities.oob_output_actions);
    pdu.oob_input_size = p_ctx->capabilities.oob_input_size;
    pdu.oob_input_actions = LE2BE16(p_ctx->capabilities.oob_input_actions);

#if PROV_DEBUG_MODE
    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: sending capabilities\n");
#endif

    return prov_tx_capabilities(p_ctx->p_active_bearer, &pdu, p_ctx->confirmation_inputs);
}

#if NRF_MESH_PROV_FORCE_SECURE_PROVISIONING
/* Secure Provisioning requires any of the following methods: */
/* 1) FIPS P-256 Elliptic Curve Algorithm (validated elsewhere), a Public Key Type that is not transferred in
 * band (i.e., ”OOB Public Key is used” is selected), and a Static OOB of any size.
 */
static inline bool secure_provisioning_with_pubkeyoob_and_staticoob(const prov_pdu_prov_start_t * p_pdu)
{
    return (p_pdu->public_key == NRF_MESH_PROV_PUBLIC_KEY_OOB && p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_STATIC);
}

/* or,
 * 2) FIPS P-256 Elliptic Curve Algorithm (validated elsewhere); OOB Action of Input Numeric, Input Alphanumeric,
 * Output Numeric, or Output Alphanumeric; and OOB Size of at least 6 octets.
 */
static inline bool secure_provisioning_with_oob_input_or_ouput(const prov_pdu_prov_start_t * p_pdu)
{
    return (
              (
               (
                (
                 p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_OUTPUT
                )
                &&
                (
                 p_pdu->auth_action == NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC ||
                 p_pdu->auth_action == NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC
                )
               )
               ||
               (
                (
                 p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_INPUT
                )
                &&
                (
                 p_pdu->auth_action == NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER ||
                 p_pdu->auth_action == NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING
                )
               )
              )

              &&

              (
               p_pdu->auth_size >= SECURE_PROVISIONING_MIN_AUTH_SIZE
              )
           );
}

static inline bool secure_provisioning_request_check(const prov_pdu_prov_start_t * p_pdu)
{
    if (secure_provisioning_with_pubkeyoob_and_staticoob(p_pdu) ||
        secure_provisioning_with_oob_input_or_ouput(p_pdu))
    {
        return true;
    }

    /* Requested authentication method is not secure. */
    return false;
}
#endif

static uint32_t handle_prov_start(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_buffer)
{
    const prov_pdu_prov_start_t * p_pdu = (const prov_pdu_prov_start_t *) p_buffer;

    if (p_pdu->algorithm >= NRF_MESH_PROV_ALGORITHM_RFU ||
        p_pdu->public_key >= NRF_MESH_PROV_PUBLIC_KEY_PROHIBITED ||
        p_pdu->auth_method >= NRF_MESH_PROV_OOB_METHOD_PROHIBITED)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if (p_pdu->public_key == NRF_MESH_PROV_PUBLIC_KEY_OOB &&
        p_ctx->capabilities.pubkey_type != NRF_MESH_PROV_OOB_PUBKEY_TYPE_OOB)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if ((p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_NONE ||
         p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_STATIC) &&
        (p_pdu->auth_action > 0 || p_pdu->auth_size > 0))
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if (p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_OUTPUT &&
       (p_pdu->auth_action >= NRF_MESH_PROV_OUTPUT_ACTION_RFU ||
        p_pdu->auth_size < 1 || p_pdu->auth_size > 8))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_OUTPUT &&
        !IS_SET(p_ctx->capabilities.oob_output_actions, p_pdu->auth_action))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_INPUT &&
       (p_pdu->auth_action >= NRF_MESH_PROV_INPUT_ACTION_RFU ||
        p_pdu->auth_size < 1 || p_pdu->auth_size > 8))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (p_pdu->auth_method == NRF_MESH_PROV_OOB_METHOD_INPUT &&
        !IS_SET(p_ctx->capabilities.oob_input_actions, p_pdu->auth_action))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

#if NRF_MESH_PROV_FORCE_SECURE_PROVISIONING
    /* Check if the secure provisioning is requested by the provisioner.
     * Note: Check for FIPS P-256 Elliptic Curve Algorithm is already done earlier in this function.
     */
    if (!secure_provisioning_request_check(p_pdu))
    {
        return NRF_ERROR_FORBIDDEN;
    }
#endif

    /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
    memcpy(p_ctx->confirmation_inputs + PROV_CONFIRM_INPUTS_START_OFFSET, p_buffer + 1, sizeof(prov_pdu_prov_start_t) - 1);

    p_ctx->pubkey_oob = (bool) p_pdu->public_key;
    p_ctx->oob_method = (nrf_mesh_prov_oob_method_t) p_pdu->auth_method;
    p_ctx->oob_size = p_pdu->auth_method != NRF_MESH_PROV_OOB_METHOD_STATIC ? p_pdu->auth_size : 16;
    p_ctx->oob_action = p_pdu->auth_action;

    return NRF_SUCCESS;
}

/* Handles an incoming provisioning data message. */
static uint32_t handle_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_buffer)
{
    const prov_pdu_data_t * p_pdu = (const prov_pdu_data_t *) p_buffer;

    prov_pdu_data_t unencrypted_pdu;

    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_ctx->session_key;
    ccm_data.p_nonce = p_ctx->data_nonce;
    ccm_data.p_m = (uint8_t *) &p_pdu->data;
    ccm_data.m_len = sizeof(prov_pdu_data_block_t);
    ccm_data.p_out = (uint8_t *) &unencrypted_pdu.data;
    ccm_data.p_mic = (uint8_t *) p_pdu->mic;
    ccm_data.p_a = NULL;
    ccm_data.a_len = 0;
    ccm_data.mic_len = PROV_PDU_DATA_MIC_LENGTH;

    bool mic_passed = false;
    enc_aes_ccm_decrypt(&ccm_data, &mic_passed);
    if (!mic_passed)
    {
#if PROV_DEBUG_MODE
        __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisionee: provisioning data could not be authenticated!\n");
#endif
        return NRF_ERROR_INVALID_DATA;
    }

    memcpy(p_ctx->data.netkey, unencrypted_pdu.data.netkey, NRF_MESH_KEY_SIZE);
    p_ctx->data.iv_index = BE2LE32(unencrypted_pdu.data.iv_index);
    p_ctx->data.address = BE2LE16(unencrypted_pdu.data.address);
    p_ctx->data.netkey_index = BE2LE16(unencrypted_pdu.data.netkey_index);
    p_ctx->data.flags.iv_update = unencrypted_pdu.data.flags.iv_update;
    p_ctx->data.flags.key_refresh = unencrypted_pdu.data.flags.key_refresh;
    return NRF_SUCCESS;
}

static uint32_t request_authentication(nrf_mesh_prov_ctx_t * p_ctx)
{
    uint32_t retval = NRF_SUCCESS;

    /* Request OOB data/action for authentication. */
    switch (p_ctx->oob_method)
    {
        case NRF_MESH_PROV_OOB_METHOD_INPUT:
        {
            nrf_mesh_prov_evt_t event;
            event.type = NRF_MESH_PROV_EVT_INPUT_REQUEST;
            event.params.input_request.p_context =  p_ctx;
            event.params.input_request.size = p_ctx->oob_size;
            event.params.input_request.action = p_ctx->oob_action;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_INPUT;
            p_ctx->event_handler(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested OOB input from application, size = %d\n", p_ctx->oob_size);
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_OUTPUT:
        {
            prov_utils_generate_oob_data(p_ctx, p_ctx->auth_value);

            nrf_mesh_prov_evt_t event;
            event.type = NRF_MESH_PROV_EVT_OUTPUT_REQUEST;
            event.params.output_request.p_context =  p_ctx;
            event.params.output_request.size = p_ctx->oob_size;
            event.params.output_request.action = p_ctx->oob_action;
            event.params.output_request.p_data = p_ctx->auth_value;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            p_ctx->event_handler(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested OOB output from application, size = %d\n", p_ctx->oob_size);
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_STATIC:
        {
            /* Request static provisioning data from the application. */
            nrf_mesh_prov_evt_t event;
            event.type = NRF_MESH_PROV_EVT_STATIC_REQUEST;
            event.params.static_request.p_context =  p_ctx;
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_STATIC;
            p_ctx->event_handler(&event);

#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: requested static OOB data from application\n");
#endif

            break;
        }
        case NRF_MESH_PROV_OOB_METHOD_NONE:
            memset(p_ctx->auth_value, 0, sizeof(p_ctx->auth_value));
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            break;
        default:
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisioning: unrecognized OOB mode!\n");
#endif
            retval = NRF_ERROR_INTERNAL;
            break;
    }

    return retval;
}

static void start_authentication(nrf_mesh_prov_ctx_t * p_ctx)
{
    if (prov_utils_use_ecdh_offloading())
    {
        nrf_mesh_prov_evt_t app_event;
        app_event.type = NRF_MESH_PROV_EVT_ECDH_REQUEST;
        app_event.params.ecdh_request.p_context =  p_ctx;
        app_event.params.ecdh_request.p_node_private = p_ctx->p_private_key;
        app_event.params.ecdh_request.p_peer_public = p_ctx->peer_public_key;
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH;
        p_ctx->event_handler(&app_event);
    }
    else if (NRF_SUCCESS == prov_utils_calculate_shared_secret(p_ctx, p_ctx->shared_secret))
    {
        if (NRF_SUCCESS != request_authentication(p_ctx))
        {
            send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
        }
    }
    else
    {
        /* The ECDH fails if the public key is not valid: */
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
    }
}

/****************** Callback functions ******************/
static void prov_provisionee_pkt_in(prov_bearer_t * p_bearer, const uint8_t * p_buffer, uint16_t length)
{
    nrf_mesh_prov_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    const prov_pdu_type_t pdu_type = (prov_pdu_type_t) p_buffer[0];

    if (!prov_packet_length_valid(p_buffer, length) || pdu_type >= PROV_PDU_TYPE_INVALID)
    {
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU);
        return;
    }
    else if (!prov_utils_is_valid_pdu(p_ctx->role, p_ctx->state, pdu_type))
    {
        __LOG(LOG_SRC_PROV, LOG_LEVEL_DBG1, "Got unexpected PDU %u in state %u\n", pdu_type, p_ctx->state);
        send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU);
        return;
    }

    switch (p_buffer[0])
    {
        case PROV_PDU_TYPE_INVITE:
        {
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: invite received!\n");
#endif
            /* Copy PDU contents (excluding PDU type) into the confirmation inputs: */
            memcpy(p_ctx->confirmation_inputs + PROV_CONFIRM_INPUTS_INVITE_OFFSET, p_buffer + 1, sizeof(prov_pdu_invite_t) - 1);

            const prov_pdu_invite_t * p_invite = (const prov_pdu_invite_t *) p_buffer;

            nrf_mesh_prov_evt_t event;
            event.type = NRF_MESH_PROV_EVT_INVITE_RECEIVED;
            event.params.invite_received.p_context = p_ctx;
            event.params.invite_received.attention_duration_s = p_invite->attention_duration_s;
            p_ctx->event_handler(&event);

            if (NRF_SUCCESS != send_capabilities(p_ctx))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
            }
            break;
        }
        case PROV_PDU_TYPE_START:
        {
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: provisioning start message received!\n");
#endif
            if (NRF_SUCCESS != handle_prov_start(p_ctx, p_buffer))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
                break;
            }

            nrf_mesh_prov_evt_t event;
            event.type = NRF_MESH_PROV_EVT_START_RECEIVED;
            event.params.start_received.p_context = p_ctx;
            p_ctx->event_handler(&event);

            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_PUB_KEY;
            if (p_ctx->pubkey_oob)
            {
                memset(&event, 0, sizeof(nrf_mesh_prov_evt_t));

                event.type = NRF_MESH_PROV_EVT_OOB_PUBKEY_REQUEST;
                event.params.oob_pubkey_request.p_context = p_ctx;
                p_ctx->event_handler(&event);
            }
            break;
        }
        case PROV_PDU_TYPE_PUBLIC_KEY:
        {
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: public key message received!\n");
#endif
            const prov_pdu_pubkey_t * p_pdu = (const prov_pdu_pubkey_t *) p_buffer;
            if (!prov_utils_is_valid_public_key(p_pdu->public_key))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
                break;
            }

            memcpy(p_ctx->peer_public_key, p_pdu->public_key, NRF_MESH_PROV_PUBKEY_SIZE);

            if (!p_ctx->pubkey_oob)
            {
                if (NRF_SUCCESS != prov_tx_public_key(p_ctx->p_active_bearer, p_ctx->p_public_key))
                {
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                }
            }
            else
            {
                start_authentication(p_ctx);
            }
            break;
        }
        case PROV_PDU_TYPE_CONFIRMATION:
            if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_CONFIRMATION)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioning: provisioning confirmation received!\n");
#endif
                const prov_pdu_confirm_t * p_pdu = (const prov_pdu_confirm_t *) p_buffer;
                memcpy(p_ctx->peer_confirmation, p_pdu->confirmation, sizeof(p_ctx->peer_confirmation));

                uint8_t confirmation_value[PROV_CONFIRMATION_LEN];
                prov_utils_authentication_values_derive(p_ctx, p_ctx->confirmation_salt, confirmation_value, p_ctx->node_random);
                uint32_t err_code = prov_tx_confirmation(p_ctx->p_active_bearer, confirmation_value);
                if (NRF_SUCCESS != err_code)
                {
                    /* NRF_ERROR_NO_MEM is the only expected error code */
                    NRF_MESH_ASSERT(NRF_ERROR_NO_MEM == err_code);
                    send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
                }
            }
            else if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_STATIC)
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisioning: provisioning confirmation received!\n");
#endif
                const prov_pdu_confirm_t * p_pdu = (const prov_pdu_confirm_t *) p_buffer;
                memcpy(p_ctx->peer_confirmation, p_pdu->confirmation, sizeof(p_ctx->peer_confirmation));
                p_ctx->state = NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD;
            }
            break;
        case PROV_PDU_TYPE_RANDOM:
        {
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: provisioner's random number received!\n");
#endif
            const prov_pdu_random_t * p_pdu = (const prov_pdu_random_t *) p_buffer;
            memcpy(p_ctx->peer_random, p_pdu->random, sizeof(p_pdu->random));

            if (!prov_utils_confirmation_check(p_ctx))
            {
#if PROV_DEBUG_MODE
                __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Provisionee: could not authenticate provisioner!\n");
#endif
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED);
            }
            else if (NRF_SUCCESS == prov_tx_random(p_ctx->p_active_bearer, p_ctx->node_random))
            {
                prov_utils_derive_keys(p_ctx, p_ctx->session_key, p_ctx->data_nonce, p_ctx->device_key);
            }
            else
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
            }
            break;
        }
        case PROV_PDU_TYPE_DATA:
#if PROV_DEBUG_MODE
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Provisionee: received provisioning data!\n");
#endif
            if (NRF_SUCCESS != handle_data(p_ctx, p_buffer))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED);
            }
            else if (!prov_data_is_valid(&p_ctx->data))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT);
            }
            else if (!prov_address_is_valid(&p_ctx->data, p_ctx->capabilities.num_elements))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_CANNOT_ASSIGN_ADDR);
            }
            else if (NRF_SUCCESS != prov_tx_complete(p_ctx->p_active_bearer))
            {
                send_failed(p_ctx, NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES);
            }
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void complete_provisioning(nrf_mesh_prov_ctx_t * p_ctx)
{
    nrf_mesh_prov_evt_t app_event;
    app_event.type = NRF_MESH_PROV_EVT_COMPLETE;
    app_event.params.complete.p_context = p_ctx;
    app_event.params.complete.p_devkey = p_ctx->device_key;
    app_event.params.complete.p_prov_data = &p_ctx->data;
    p_ctx->event_handler(&app_event);
}

static void prov_provisionee_cb_ack_received(prov_bearer_t * p_bearer)
{
    nrf_mesh_prov_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    switch (p_ctx->state)
    {
        case NRF_MESH_PROV_STATE_INVITE:
            /* This is a response to send_capabilities */
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_START;
            break;
        case NRF_MESH_PROV_STATE_WAIT_START:
            break;
        case NRF_MESH_PROV_STATE_WAIT_PUB_KEY:
            start_authentication(p_ctx);
            break;
        case NRF_MESH_PROV_STATE_WAIT_OOB_INPUT:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
            break;
        case NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD:
        case NRF_MESH_PROV_STATE_WAIT_CONFIRMATION:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_RANDOM;
            break;
        case NRF_MESH_PROV_STATE_WAIT_RANDOM:
            p_ctx->state = NRF_MESH_PROV_STATE_WAIT_DATA;
            break;
        case NRF_MESH_PROV_STATE_WAIT_DATA:
            complete_provisioning(p_ctx);
            p_ctx->state = NRF_MESH_PROV_STATE_COMPLETE;
            break;
        case NRF_MESH_PROV_STATE_FAILED:
            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Got ACK for failed PDU. Waiting for Link Close\n");
            break;
        default:
            __LOG(LOG_SRC_PROV, LOG_LEVEL_ERROR, "Provisionee: unexpected ack while in state %u\n", p_ctx->state);
    }
}

static void prov_provisionee_cb_link_established(prov_bearer_t * p_bearer)
{
    nrf_mesh_prov_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    p_ctx->p_active_bearer = p_bearer;
    /* Stop listening on all other bearers */
    prov_bearer_t * p_tmp_bearer;
    LIST_FOREACH(p_item, p_ctx->p_bearers)
    {
        p_tmp_bearer = PARENT_BY_FIELD_GET(prov_bearer_t, node, p_item);
        if (p_tmp_bearer != p_bearer)
        {
            (void) p_tmp_bearer->p_interface->listen_stop(p_tmp_bearer);
        }
    }

    p_ctx->state = NRF_MESH_PROV_STATE_INVITE;
    nrf_mesh_prov_evt_t app_event;
    app_event.type = NRF_MESH_PROV_EVT_LINK_ESTABLISHED;
    app_event.params.link_established.p_context =  p_ctx;
    p_ctx->event_handler(&app_event);
}
static void prov_provisionee_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    nrf_mesh_prov_ctx_t * p_ctx = prov_bearer_ctx_get(p_bearer);
    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_DATA && reason == NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS)
    {
        complete_provisioning(p_ctx);
    }
    p_ctx->state = NRF_MESH_PROV_STATE_IDLE;
    nrf_mesh_prov_evt_t app_event;
    app_event.type = NRF_MESH_PROV_EVT_LINK_CLOSED;
    app_event.params.link_closed.p_context =  p_ctx;
    app_event.params.link_closed.close_reason = reason;
    p_ctx->event_handler(&app_event);
}

/****************** Interface functions ******************/

uint32_t prov_provisionee_listen(nrf_mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_IDLE &&
        p_ctx->state != NRF_MESH_PROV_STATE_WAIT_LINK)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        p_bearer->p_callbacks = &m_prov_callbacks;
        return p_bearer->p_interface->listen_start(
            p_bearer, URI, oob_info_sources, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US);
    }
}

uint32_t prov_provisionee_auth_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size)
{
    //TODO: This interaction could be done in init. We don't provide any new information in the event triggering this.
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_INPUT
            && p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_STATIC
            && p_ctx->state != NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t retval = NRF_SUCCESS;
    if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_INPUT)
    {
        retval = prov_tx_input_complete(p_ctx->p_active_bearer);
        if (NRF_SUCCESS != retval)
        {
            return retval;
        }
    }
    else if (p_ctx->state == NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD)
    {
        uint8_t confirmation_value[PROV_CONFIRMATION_LEN];
        prov_utils_authentication_values_derive(p_ctx, p_ctx->confirmation_salt, confirmation_value, p_ctx->node_random);
        retval = prov_tx_confirmation(p_ctx->p_active_bearer, confirmation_value);
    }
    else
    {
        p_ctx->state = NRF_MESH_PROV_STATE_WAIT_CONFIRMATION;
    }

    return retval;
}

uint32_t prov_provisionee_shared_secret(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared)
{
    if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_shared == NULL)
    {
        return NRF_ERROR_NULL;
    }

    memcpy(p_ctx->shared_secret, p_shared, sizeof(p_ctx->shared_secret));
    return request_authentication(p_ctx);
}
