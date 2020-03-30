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

#ifndef NRF_MESH_PROV_H__
#define NRF_MESH_PROV_H__

#include <ble.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_prov_types.h"
#include "nrf_mesh_prov_events.h"
#include "nrf_mesh_prov_bearer.h"
#include "nrf_mesh_assert.h"
#include "bitfield.h"

/* Ensure that the supported bearers only fills one uint32_t. */
NRF_MESH_STATIC_ASSERT(BITFIELD_BLOCK_COUNT(NRF_MESH_PROV_BEARER_COUNT) == 1);

/**
 * @defgroup NRF_MESH_PROV Provisioning API
 * @ingroup MESH_API_GROUP_PROV
 * Functionality for supporting provisioning of a node.
 * @{
 */
/**
 * @defgroup NRF_MESH_PROV_DEFINES Defines
 * Provisioning defines
 * @{
 */

/**
 * Sets the default authentication capabilities.
 * @param[in] NUM_ELEMENTS Number of elements present in the node (@ref ACCESS_ELEMENT_COUNT).
 */
#define NRF_MESH_PROV_OOB_CAPS_DEFAULT(NUM_ELEMENTS)                    \
    {NUM_ELEMENTS, NRF_MESH_PROV_ALGORITHM_FIPS_P256EC, 0, NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED, 0, 0, 0, 0}

/** @} */

/**
 * @addtogroup NRF_MESH_PROV_TYPES
 * @{
 * Provisioning context structure.
 */
struct nrf_mesh_prov_ctx
{
    list_node_t * p_bearers;    /**< Bearer linked list head pointer. */
    uint32_t supported_bearers; /**< Supported bearer types bitfield, @ref nrf_mesh_prov_bearer_type_t. */
    prov_bearer_t * p_active_bearer; /**< Pointer to the currently active bearer (valid when ) */
    nrf_mesh_prov_evt_handler_cb_t event_handler; /**< Application event handler callback function. */

    const uint8_t * p_public_key;  /**< Public key of this node. */
    const uint8_t * p_private_key; /**< Private key of this node. */

    uint8_t peer_public_key[NRF_MESH_PROV_PUBKEY_SIZE];   /**< Public key of the peer node. */
    uint8_t shared_secret[NRF_MESH_PROV_ECDHSECRET_SIZE]; /**< ECDH shared secret: P-256(private key, peer public key). */

    uint8_t device_key[NRF_MESH_KEY_SIZE];  /**< Node device key. */
    uint8_t session_key[NRF_MESH_KEY_SIZE]; /**< Provisioning session key. */
    uint8_t data_nonce[PROV_NONCE_LEN];  /**< Provisioning data nonce. Only 13 bytes are used. */

    uint8_t node_random[PROV_RANDOM_LEN]; /**< Random number for the current node. */
    uint8_t peer_random[PROV_RANDOM_LEN]; /**< Random number for the peer node. */
    uint8_t auth_value[PROV_AUTH_LEN];    /**< Authentication value. */

    uint8_t confirmation_salt[PROV_SALT_LEN];   /**< Confirmation salt value. */
    uint8_t peer_confirmation[PROV_CONFIRMATION_LEN];   /**< Confirmation value for the peer node. */
    uint8_t confirmation_inputs[PROV_CONFIRMATION_INPUT_LEN]; /**< Confirmation inputs, used to calculate the confirmation key. */

    uint8_t oob_size;   /**< Size of the chosen OOB authentication data. */
    uint8_t oob_action; /**< Chosen OOB action. */
    bool pubkey_oob;    /**< Uses out-of-band public key. */

    nrf_mesh_prov_role_t role;                 /**< Provisioning role, provisioner or provisionee. */
    nrf_mesh_prov_failure_code_t failure_code; /**< Error code sent with the previous provisioning failed packet. */
    nrf_mesh_prov_state_t state;               /**< Provisioning state machine state. */
    nrf_mesh_prov_oob_method_t oob_method;     /**< Chosen OOB authentication method. */
    nrf_mesh_prov_oob_caps_t capabilities;     /**< Node OOB and authentication capabilities. */
    nrf_mesh_prov_provisioning_data_t data;    /**< Provisioning data to send to the provisionee or received from the provisioner. */

    uint8_t attention_duration_s;  /**< Time in seconds during which the device will identify itself using any means it can. */
};
/** @} */

/**
 * Initializes the provisioning context structure.
 *
 * @warning If calling this function the first time, it is required that the @c p_ctx is zero
 * initialized. Any further calls require that @c p_ctx is left untouched.
 *
 * @param[in,out]  p_ctx            Pointer to the provisioning context structure to initialize.
 * @param[in]      p_public_key     Pointer to the node's public key. The public key is 64 bytes long.
 * @param[in]      p_private_key    Pointer to the node's private key. The private key is 32 bytes long.
 * @param[in]      p_caps           Pointer to a structure containing the node's out-of-band
 *                                  authentication capabilities.
 * @param[in]      event_handler    Event handler callback function.
 *
 * @retval NRF_SUCCESS             The library was successfully initialized.
 * @retval NRF_ERROR_NULL          One or more parameters were NULL.
 * @retval NRF_ERROR_INVALID_STATE Initialization was attempted when the provisioning was already working.
 */
uint32_t nrf_mesh_prov_init(nrf_mesh_prov_ctx_t *            p_ctx,
                            const uint8_t *                  p_public_key,
                            const uint8_t *                  p_private_key,
                            const nrf_mesh_prov_oob_caps_t * p_caps,
                            nrf_mesh_prov_evt_handler_cb_t   event_handler);

/**
 * Adds a new bearer to the provisioning context structure.
 *
 * This function is intented to be used in conjunction with a bearer specific
 * `bearer_<type>_interface_get()` function. E.g., nrf_mesh_prov_bearer_adv_interface_get().
 *
 * @param[in,out] p_ctx         Provisioning context structure.
 * @param[in,out] p_prov_bearer Generic provisioning context structure.
 *
 * @retval NRF_SUCCESS         Successfully added bearer.
 * @retval NRF_ERROR_NULL      One or more parameters were NULL.
 * @retval NRF_ERROR_FORBIDDEN A bearer of the given type already exists in the provisioning context.
 */
uint32_t nrf_mesh_prov_bearer_add(nrf_mesh_prov_ctx_t * p_ctx,
                                  prov_bearer_t * p_prov_bearer);

/**
 * Listens for an incoming provisioning link.
 *
 * @param[in, out] p_ctx       Pointer to a statically allocated provisioning context structure.
 * @param[in] URI              Optional device URI string used as identifier in some other context.
 *                             May be NULL.
 * @param[in] oob_info_sources Known OOB information sources, see @ref
 *                             NRF_MESH_PROV_OOB_INFO_SOURCES.
 * @param[in] bearer_types     Bitfield of @ref nrf_mesh_prov_bearer_type_t bearers to listen on.
 *
 * @retval NRF_SUCCESS             The provisioning bearer was successfully put into listening mode.
 * @retval NRF_ERROR_INVALID_STATE The provisioning context is not in an idle state.
 * @retval NRF_ERROR_INVALID_PARAM (One of) the given bearer type(s) is/are not supported.
 */
uint32_t nrf_mesh_prov_listen(nrf_mesh_prov_ctx_t * p_ctx,
                              const char *          URI,
                              uint16_t              oob_info_sources,
                              uint32_t              bearer_types);

/**
 * Stops listening for an incoming provisioning link.
 *
 * @param[in, out] p_ctx Pointer to a statically allocated provisioning context structure.
 *
 * @retval NRF_SUCCESS             The provisioning bearer was successfully put into listening mode.
 * @retval NRF_ERROR_INVALID_STATE The provisioning context is not listening.
 */
uint32_t nrf_mesh_prov_listen_stop(nrf_mesh_prov_ctx_t * p_ctx);

/**
 * Generates a valid keypair for use with the provisioning cryptography.
 *
 * @param[out] p_public  Pointer to where the generated public key is stored.
 * @param[out] p_private Pointer to where the generated private key is stored.
 *
 * @retval NRF_SUCCESS The keypair was successfully generated.
 */
uint32_t nrf_mesh_prov_generate_keys(uint8_t * p_public, uint8_t * p_private);

/**
 * Provisions a device.
 *
 * @param[in,out] p_ctx                 Pointer to a statically allocated provisioning context structure.
 * @param[in]     p_target_uuid         Device UUID of the device that is to be provisioned.
 * @param[in]     attention_duration_s  Time in seconds during which the device will identify itself using any means it can.
 * @param[in]     p_data                Pointer to a structure containing the provisioning data for the
 * device.
 * @param[in]     bearer                Which bearer to establish the provisioning link on.
 *
 * @retval NRF_SUCCESS             The provisioning process was started.
 * @retval NRF_ERROR_NULL          One or more parameters were NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer type is not supported.
 * @retval NRF_ERROR_INVALID_DATA  The provisioning data failed some boundary conditions.
 * @retval NRF_ERROR_INVALID_STATE The given context is in use.
 */
uint32_t nrf_mesh_prov_provision(nrf_mesh_prov_ctx_t *                     p_ctx,
                                 const uint8_t *                           p_target_uuid,
                                 uint8_t                                   attention_duration_s,
                                 const nrf_mesh_prov_provisioning_data_t * p_data,
                                 nrf_mesh_prov_bearer_type_t               bearer);

/**
 * Selects which out-of-band authentication method to use.
 *
 * This function is used in response to the reception of a @ref NRF_MESH_PROV_EVT_CAPS_RECEIVED
 * event.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] method    Specifies the authentication method to use.
 * @param[in] action    The action that must be taken for the specified method is dependent on
 *                      the provisionee device, and can be read from
 *                      the @ref NRF_MESH_PROV_EVT_CAPS_RECEIVED event.
 * @param[in] size      Size of the out-of-band authentication data. Must be between 1 and 8
 *                      inclusive or 0 when @c NRF_MESH_PROV_OOB_METHOD_NONE is used.
 *
 * @retval NRF_SUCCESS              The out-of-band method was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_LENGTH The size of the authentication data was invalid.
 */
uint32_t nrf_mesh_prov_oob_use(nrf_mesh_prov_ctx_t *      p_ctx,
                               nrf_mesh_prov_oob_method_t method,
                               uint8_t                    action,
                               uint8_t                    size);

/**
 * Provides out-of-band authentication data input to the provisioning stack.
 *
 * When replying to an @ref NRF_MESH_PROV_EVT_INPUT_REQUEST
 * and acting as a **provisionee**, the @ref nrf_mesh_prov_input_action_t
 * determines how @p p_data will be formatted.
 *
 * @p p_data must be a pointer to a `uint32_t` number that contains the authentication
 * data and @p size must be identical to @ref nrf_mesh_prov_evt_input_request_t::size
 * when the input is one of the following:
 *
 * - @ref nrf_mesh_prov_input_action_t::NRF_MESH_PROV_INPUT_ACTION_PUSH,
 * - @ref nrf_mesh_prov_input_action_t::NRF_MESH_PROV_INPUT_ACTION_TWIST, or
 * - @ref nrf_mesh_prov_input_action_t::NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER.
 *
 * When the input action is
 * @ref nrf_mesh_prov_input_action_t::NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING,
 * @p p_data must be an array of alphanumeric uppercase
 * ASCII values of @p size . That is, with values in the ranges
 * 'A'-'Z' or '0'-'9'.
 *
 * When replying to an @ref NRF_MESH_PROV_EVT_INPUT_REQUEST
 * and acting as a **provisioner**, the @ref nrf_mesh_prov_output_action_t
 * determines how @p p_data will be formatted.
 *
 * @p p_data must be a pointer to a `uint32_t` number that contains the authentication
 * data and @p size must be identical to @ref nrf_mesh_prov_evt_input_request_t::size
 * when the input is one of the following:
 *
 * - @ref nrf_mesh_prov_output_action_t::NRF_MESH_PROV_OUTPUT_ACTION_BLINK,
 * - @ref nrf_mesh_prov_output_action_t::NRF_MESH_PROV_OUTPUT_ACTION_BEEP,
 * - @ref nrf_mesh_prov_output_action_t::NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE, or
 * - @ref nrf_mesh_prov_output_action_t::NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC.
 *
 * When the output action is
 * @ref nrf_mesh_prov_output_action_t::NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC
 * @p p_data must be an array of alphanumeric uppercase
 * ASCII values of @p size . That is, with values in the ranges
 * 'A'-'Z' or '0'-'9'.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_data    Pointer to an array of authentication data. The size of this array should
 *                      match the size of the data requested in the request event for @ref
 *                      NRF_MESH_PROV_EVT_INPUT_REQUEST, or be 16 bytes for a
 *                      @ref NRF_MESH_PROV_EVT_STATIC_REQUEST event. The maximum size of the data is
 *                      16 bytes.
 * @param[in] size      Size according to @ref nrf_mesh_prov_evt_input_request_t::size.
 *
 * @retval NRF_SUCCESS              The authentication data was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE  Authentication data was provided even though it was not
 *                                  requested by the current provisioning context.
 * @retval NRF_ERROR_INVALID_LENGTH The size of the authentication data was invalid.
 * @retval NRF_ERROR_INVALID_DATA   The provided data did not meet the requirements
 *                                  corresponding to the requested data.
 */
uint32_t nrf_mesh_prov_auth_data_provide(nrf_mesh_prov_ctx_t * p_ctx,
                                         const uint8_t *       p_data,
                                         uint8_t               size);
/**
 * Provides out-of-band authentication number to the provisioning stack.
 *
 * This function is a simple wrapper for the @ref nrf_mesh_prov_auth_data_provide()
 * API.
 *
 * @param[in,out] p_ctx  Provisioning context pointer.
 * @param[in]     number Number displayed by the peer device.
 *
 * @return Inherits the returns from @ref nrf_mesh_prov_auth_data_provide().
 */
static inline uint32_t nrf_mesh_prov_oob_number_provide(nrf_mesh_prov_ctx_t * p_ctx,
                                                        uint32_t number)
{
    /* Input sanitation is done by nrf_mesh_prov_auth_data_provide() */
    return nrf_mesh_prov_auth_data_provide(p_ctx, (const uint8_t *) &number, p_ctx->oob_size);
}

/**
 * Provides the shared secret to the provisioning stack after running a requested ECDH calculation.
 * This function is used only if ECDH offloading is enabled in the options API.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_shared  Pointer to the shared secret calculated by the external ECDH code.
 *
 * @retval NRF_SUCCESS             The shared secret was accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE A shared secret was not requested by the current provisioning
 *                                 context.
 * @retval NRF_ERROR_NULL          The pointer provided to the shared secret was NULL.
 */
uint32_t nrf_mesh_prov_shared_secret_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);

/**
 * Provides a public key to the provisioner if the provisionee has exposed it out-of-band.
 *
 * @param[in,out] p_ctx Pointer to a statically allocated provisioning context structure.
 * @param[in] p_key     Pointer to the start of an array containing the public key for the
 * provisionee.
 *
 * @retval NRF_SUCCESS             The public key was valid and accepted by the provisioning system.
 * @retval NRF_ERROR_INVALID_STATE The public key was provided even tough it was not requested by
 *                                 the specified provisioning context.
 * @retval NRF_ERROR_INVALID_PARAM The public key was invalid.
 * @retval NRF_ERROR_NULL          The @c p_key argument was NULL.
 */
uint32_t nrf_mesh_prov_pubkey_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key);

/**
 * Starts the scanning for unprovisioned devices.
 *
 * @param[in] event_handler Event handler callback function for reporting unprovisioned device
 *                          events.
 *
 * @retval NRF_SUCCESS    Successfully started scanning for unprovisioned devices.
 * @retval NRF_ERROR_NULL Callback function pointer was NULL.
 */
uint32_t nrf_mesh_prov_scan_start(nrf_mesh_prov_evt_handler_cb_t event_handler);

/**
 * Stops the scanning for unprovisioned devices (if started).
 */
void nrf_mesh_prov_scan_stop(void);

/** @} */
#endif
