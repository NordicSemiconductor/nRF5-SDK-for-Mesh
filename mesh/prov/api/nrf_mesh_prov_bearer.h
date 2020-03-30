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

#ifndef NRF_MESH_PROV_BEARER_H__
#define NRF_MESH_PROV_BEARER_H__

#include <stdint.h>
#include "list.h"

/**
 * @defgroup NRF_MESH_PROV_BEARER Bearers
 * @ingroup NRF_MESH_PROV
 * Bearer abstraction layer for provisioning.
 *
 * The provisioning bearer layer enables the provisioning procedures to be independent on which
 * bearer is actually used.
 * @{
 */

/**
 * Reason for why a provisioning link was closed.
 */
typedef enum
{
    NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS = 0, /**< The link was closed because provisioning was completed successfully. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT = 1, /**< The link timed out and was closed. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR   = 2, /**< The link was closed beacuse of an error. */
    NRF_MESH_PROV_LINK_CLOSE_REASON_LAST    = NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR /**< The highest known link close reason, values higher are unrecognized */
} nrf_mesh_prov_link_close_reason_t;

/** Forward declaration of the bearer context structure. */
typedef struct prov_bearer prov_bearer_t;

/**
 * @defgroup PROV_BEARER_INTERFACE Provisioning bearer interface
 * Defines the generic (downstream) interface any provisioning bearer should expose.
 * @{
 */

/**
 * Function used for transmitting a provisioning packet.
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 * @param[in]     p_data   Data to be transmitted.
 * @param[in]     length   Length of the data to be transmitted.
 * @return Returns @c NRF_SUCCESS on success or an error code otherwise. Any error will cause the
 *         provisioning procedure to fail.
 */
typedef uint32_t (*prov_bearer_if_tx_t)(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

/**
 * Function used to start listening for incoming provisioning links.
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 * @param[in]     p_data   An optional URI used for the unprovisioned beacon. Set to @c NULL to not
 *                         include a URI in the beacon.
 * @param[in]     oob_info OOB authentication information.
 * @param[in]     link_timeout_us Duration of time the link can be idle before timing out.
 * @return Returns @c NRF_SUCCESS on success or an error code otherwise.
 */
typedef uint32_t (*prov_bearer_if_listen_start_t)(prov_bearer_t *       p_bearer,
                                                  const char * p_uri,
                                                  uint16_t     oob_info,
                                                  uint32_t     link_timeout_us);

/**
 * Function used to stop listening for incoming provisioning links.
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 * @return Returns @c NRF_SUCCESS on success or an error code otherwise.
 */
typedef uint32_t (*prov_bearer_if_listen_stop_t)(prov_bearer_t * p_bearer);

/**
 * Function used for opening a new provisioning link.
 *
 * @param[in,out] p_bearer        Pointer to the bearer context structure.
 * @param[in]     p_uuid          UUID of the device to establish a link to.
 * @param[in]     link_timeout_us Duration of time the link can be idle before timing out.
 *
 * @return Returns @c NRF_SUCCESS on success or an error code otherwise. Any error will cause the
 *         provisioning procedure to fail.
 */
typedef uint32_t (*prov_bearer_if_link_open_t)(prov_bearer_t * p_bearer,
                                               const uint8_t * p_uuid,
                                               uint32_t        link_timeout_us);

/**
 * Function used for closing a provisioning link.
 *
 * @param[in,out] p_bearer     Pointer to the bearer context structure.
 * @param[in]     close_reason Reason for why the link is being closed. This is included in the
 *                             close PDU.
 */
typedef void (*prov_bearer_if_link_close_t)(prov_bearer_t *                   p_bearer,
                                            nrf_mesh_prov_link_close_reason_t close_reason);

/**
 * Struct used to provide an interface to provisioning bearers.
 */
typedef struct
{
    /** Function for transmitting packets. */
    prov_bearer_if_tx_t tx;
    /** Function to start listening for incoming provisioning links. */
    prov_bearer_if_listen_start_t listen_start;
    /** Function to stop listening for incoming provisioning links. */
    prov_bearer_if_listen_stop_t listen_stop;
    /** Function for opening a provisioning link. */
    prov_bearer_if_link_open_t link_open;
    /** Function for closing a provisioning link. */
    prov_bearer_if_link_close_t link_close;
} prov_bearer_interface_t;

/** @} end of PROV_BEARER_INTERFACE */

/**
 * @defgroup PROV_BEARER_CALLBACKS Provisioning bearer callbacks
 * Defines the (upstream) interface a higher layer provisioning structure should implement. A bearer
 * uses this interface to notify its parent about events and incoming data.
 * @{
 */

/**
 * Callback function type used when a provisioning packet has been received.
 *
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 * @param[in]     p_data   Pointer to the received packet data.
 * @param[in]     length   Length of the received packet data.
 */
typedef void (*prov_bearer_cb_rx_t)(prov_bearer_t * p_bearer,
                                    const uint8_t * p_data,
                                    uint16_t        length);

/**
 * Callback function type used when the previous provisioning packet has been acknowledged.
 *
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 */
typedef void (*prov_bearer_cb_ack_t)(prov_bearer_t * p_bearer);

/**
 * Callback function type used when a provisioning link has been opened.
 *
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 */
typedef void (*prov_bearer_cb_link_opened_t)(prov_bearer_t * p_bearer);

/**
 * Callback function type used when a provisioning link has been closed.
 *
 * @param[in,out] p_bearer Pointer to the bearer context structure.
 * @param[in]     reason   Reason for why the provisioning link was closed.
 */
typedef void (*prov_bearer_cb_link_closed_t)(prov_bearer_t *                   p_bearer,
                                             nrf_mesh_prov_link_close_reason_t reason);

/**
 * Struct defining the bearer callback functions.
 */
typedef struct
{
    /** The callback function for passing up all the incoming packets except the control packets. */
    prov_bearer_cb_rx_t rx;
    /** The callback function for passing up all the incoming acknowledgments. */
    prov_bearer_cb_ack_t ack;
    /** The callback function for notifying the users that a link has been opened. */
    prov_bearer_cb_link_opened_t opened;
    /** The callback function for notifying the users that a link has been closed. */
    prov_bearer_cb_link_closed_t closed;
} prov_bearer_callbacks_t;

/* @} end of PROV_BEARER_CALLBACKS */

/** Number of supported bearers. */
#define NRF_MESH_PROV_BEARER_COUNT (3)

/**
 * Provisioning bearer types.
 */
typedef enum
{
    NRF_MESH_PROV_BEARER_ADV  = 0x01, /**< Advertising-based provisioning bearer, PB-ADV. */
    NRF_MESH_PROV_BEARER_GATT = 0x02, /**< GATT-based provisioning bearer, PB-GATT. */
    NRF_MESH_PROV_BEARER_MESH = 0x04  /**< Mesh-based provisioning bearer. */
} nrf_mesh_prov_bearer_type_t;

/**
 * Provisioning bearer context.
 *
 * Every provisioning bearer should instantiate this structure.
 */
struct prov_bearer
{
    list_node_t node;                            /**< Linked-list node parameter. */
    nrf_mesh_prov_bearer_type_t     bearer_type; /**< Bearer type. */
    const prov_bearer_interface_t * p_interface; /**< Provisioning bearer interface. */
    const prov_bearer_callbacks_t * p_callbacks; /**< Provisioning bearer callbacks. */
    void * p_parent;                             /**< Pointer to the parent context structure. */
};

/** @} */
#endif
