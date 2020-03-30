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

#ifndef SERIAL_PB_REMOTE_CLIENT_H__
#define SERIAL_PB_REMOTE_CLIENT_H__

#include <stdint.h>
#include "nrf_mesh_prov.h"
#include "serial_cmd.h"

/**
 * @defgroup SERIAL_PB_REMOTE_CLIENT PB-remote Client serial interface
 * @ingroup PB_REMOTE
 * @{
 */

/**
 * @defgroup MODEL_SPECIFIC_COMMAND_TYPES PB-MESH Client model specific serial commands
 * @see serial_cmd_model_specific_command_header_t
 * @{
 */
/** Send a scan_start request to the pb_remote server */
#define SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_START  1
/** Send a scan_stop request to the pb_remote server */
#define SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_CANCEL 2
/** Start the provisioning of a reported unprovisioned device. */
#define SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION   3
/** @} */

/** Provisioning data required by the remote provisioning client for command type @ref SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION. */
typedef struct __attribute__((packed))
{
    nrf_mesh_prov_provisioning_data_t prov_data; /**< Data needed to provision a device */
    nrf_mesh_prov_oob_caps_t capabilities;       /**< Provisioining capabilities of this device */
    uint8_t unprov_id;                           /**< Id of the unprovisioned device reported by the
                                                  * @ref PB_REMOTE_EVENT_REMOTE_UUID event @see pb_remote_event_type_t */
} serial_pbr_client_provision_t;

/** Format used by the pbr_client for the data field of the serial command
  * @ref model-specific-command. */
typedef struct __attribute__((packed))
{
    /** Command type indicating one of the @ref MODEL_SPECIFIC_COMMAND_TYPES */
    uint8_t cmd_type;
    /** Only relevant for @ref SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION type command. */
    serial_pbr_client_provision_t provisioning_data;
} serial_pbr_client_command_data_t;

/** Additional data format used for reporting the @ref PB_REMOTE_EVENT_REMOTE_UUID event. All other
 * events do not provide additional data.*/
typedef struct __attribute__((packed))
{
    uint8_t device_id;                /**< ID of the unprovisioned device given by the server. */
    uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< The UUID of the unprovisioned device. */
} serial_pbr_client_remote_uuid_event_t;

/**
 * Adds the pb_remote_client model to the serial_handler_models module and initializes the handlers
 * serial_pb_remote_client specific states.
 *
 * @param[in]  p_prov_ctxs          The pointer to the list of available provisioning instances.
 * @param[in]  prov_ctx_array_size  The number of available provisioning instances.
 * @param[in]  p_keypair            The pointer to the provisioning keypair (public and private
 *                                  keys)
 */
void serial_pb_remote_client_init(nrf_mesh_prov_ctx_t * p_prov_ctxs, uint8_t prov_ctx_array_size, serial_cmd_prov_keypair_t * p_keypair);

/** @} */

#endif
