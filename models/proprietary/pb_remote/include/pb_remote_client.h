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
#ifndef PB_REMOTE_CLIENT_H__
#define PB_REMOTE_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "access_reliable.h"
#include "nrf_mesh_prov.h"

/**
 * @defgroup PB_REMOTE_CLIENT Remote Provisioning Client
 * @ingroup  PB_REMOTE
 *
 * This module implements the remote provisioning client (PB-Remote client) model.
 *
 * For conceptual information, see @ref md_doc_user_guide_modules_provisioning_pb_remote.
 *
 * @mscfile pb_remote_client_init.msc Initializing the Remote Provisioning Client
 * @dotfile pb_remote_client.dot Remote Provisioning Client state diagram
 *
 * @{
 */

/** Remote client model ID. */
#define PB_REMOTE_CLIENT_MODEL_ID (0x8004)

/** Acknowledged message transaction timeout */
#ifndef PB_REMOTE_CLIENT_ACKED_TRANSACTION_TIMEOUT
#define PB_REMOTE_CLIENT_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(60))
#endif

/** Remote provisioning client event types. */
typedef enum
{
     /** The remote has started scanning. */
    PB_REMOTE_EVENT_SCAN_STARTED,
    /** The remote has cancelled the scanning. */
    PB_REMOTE_EVENT_SCAN_CANCELED,
    /** The remote could not start the scanning procedure. */
    PB_REMOTE_EVENT_CANNOT_START_SCANNING,
    /** The remote could not cancel the scanning procedure. */
    PB_REMOTE_EVENT_CANNOT_CANCEL_SCANNING,
    /** The remote link was closed. */
    PB_REMOTE_EVENT_LINK_CLOSED,
    /** The remote got a link close command for an inactive link. */
    PB_REMOTE_EVENT_LINK_NOT_ACTIVE,
    /** Link is already active */
    PB_REMOTE_EVENT_LINK_ALREADY_OPEN,
    /** The remote could not open link. */
    PB_REMOTE_EVENT_CANNOT_OPEN_LINK,
    /** The remote could not close the link. */
    PB_REMOTE_EVENT_CANNOT_CLOSE_LINK,
    /** The unprovisioned device ID was not recognized by the server. */
    PB_REMOTE_EVENT_INVALID_UNPROV_DEVICE_ID,
    /** The session failed because of no response from the remote. */
    PB_REMOTE_EVENT_TX_FAILED,
    /** Received a remote UUID from the server. */
    PB_REMOTE_EVENT_REMOTE_UUID
} pb_remote_event_type_t;

/** Remote UUID event data. */
typedef struct
{
    /** The @ref NRF_MESH_UUID_SIZE byte long UUID of the unprovisioned device. */
    const uint8_t * p_uuid;
    /** ID of the unprovisioned device given by the server. */
    uint8_t device_id;
} pb_remote_event_remote_uuid_t;

/** Remote provisioning event structure. */
typedef struct
{
    /** Event type. */
    pb_remote_event_type_t type;
    /** Remote UUID parameters for the @ref PB_REMOTE_EVENT_REMOTE_UUID event. */
    pb_remote_event_remote_uuid_t remote_uuid;
} pb_remote_event_t;

/**
 * Remote provisioning client event handler callback function.
 *
 * @param[in] p_evt Remote provisioning event pointer.
 */
typedef void (*pb_remote_client_event_cb_t)(const pb_remote_event_t * p_evt);

/**
 * Remote Provisioning Client states.
 */
typedef enum
{
    PB_REMOTE_CLIENT_STATE_NONE,             /**< Uninitialized state. */
    PB_REMOTE_CLIENT_STATE_IDLE,             /**< Initialized and idle state. */
    PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED, /**< Remote provisioning link is established. */
    PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK,
    PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER,
    PB_REMOTE_CLIENT_STATE_WAIT_ACK_SCAN,
} pb_remote_client_state_t;

/**
 * Remote Provisioning Client context structure.
 * @note This struct is intended to be stored statically by the user. All fields are internal.
 * @todo This internal structure should be stored internally in the module.
 */
typedef struct
{
    /** Model handle for the client. */
    access_model_handle_t model_handle;
    /** Provisioning context structure pointer. */
    prov_bearer_t prov_bearer;
    /** Remote provisioning Client state. */
    pb_remote_client_state_t state;
    /** Access layer reliable message parameter structure. @todo Keep this state in a better way. */
    access_reliable_t reliable;
    /** User event callback. */
    pb_remote_client_event_cb_t event_cb;
    /** Current Provisioning PDU Type. */
    uint8_t current_prov_pdu_type;
} pb_remote_client_t;

/**
 * Initializes the Remote provisioning client model.
 *
 * @param[in,out] p_client       Client context structure pointer.
 * @param[in]     element_index  Element index to bind the remote client model.
 * @param[in]     event_cb       Event callback function.
 *
 * @retval NRF_SUCCESS             Successfully initialized the model.
 * @retval NRF_ERROR_NULL          NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM        No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN     Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND     Invalid element index.
 * @retval NRF_ERROR_INVALID_STATE Model was already initialized.
 */
uint32_t pb_remote_client_init(pb_remote_client_t * p_client,
                               uint16_t element_index,
                               pb_remote_client_event_cb_t event_cb);

/**
 * Gets the remote Remote provisioning client's bearer interface.
 *
 * This function is inteded to be used with the nrf_mesh_prov_bearer_add() API.
 *
 * @param[in,out] p_client Client context structure pointer
 *
 * @returns a generic bearer context structure pointer.
 */
prov_bearer_t * pb_remote_client_bearer_interface_get(pb_remote_client_t * p_client);

/**
 * Initiates the remote scanning procedure.
 *
 * @note It is assumed that the client's publish address is set to the server's address and vice
 *       versa.
 *
 * @param[in,out] p_client Client model context pointer.
 *
 * @retval NRF_SUCCESS             Successfully started the scanning procedure.
 * @retval NRF_ERROR_NULL          Null pointer supplied to function.
 * @retval NRF_ERROR_INVALID_ADDR  The destination address was not a valid
 *                                 unicast address.
 * @retval NRF_ERROR_INVALID_STATE The Client was not in a valid state to
 *                                 process this request.
 */
uint32_t pb_remote_client_remote_scan_start(pb_remote_client_t * p_client);

/**
 * Cancels the remote scanning procedure.
 *
 * @note It is assumed that the client's publish address is set to the server's address and vice
 *       versa.
 *
 * @param[in,out] p_client Client model context pointer.
 *
 * @retval NRF_SUCCESS             Successfully canceled the scanning procedure.
 * @retval NRF_ERROR_NULL          Null pointer supplied to function.
 * @retval NRF_ERROR_INVALID_ADDR  The destination address was not a valid
 *                                 unicast address.
 * @retval NRF_ERROR_INVALID_STATE The Client was not in a valid state to
 *                                 process this request.
 */
uint32_t pb_remote_client_remote_scan_cancel(pb_remote_client_t * p_client);

/** @} end of PB_REMOTE_CLIENT */

#endif  /* PB_REMOTE_CLIENT_H__ */
