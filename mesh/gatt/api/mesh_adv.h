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

#ifndef MESH_ADV_H__
#define MESH_ADV_H__

#include <stdint.h>
#include "ble_gap.h"
#include "app_util.h"

/**
 * @defgroup MESH_ADV Mesh application advertisement interface
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 *
 * This interface is required to support GATT proxy and PB-GATT.
 *
 * The implementation of the interface is left to the *user* in order to give the application as much
 * flexibility as possible. The mesh stack handles starting and stopping of the advertisements and
 * the required UUIDs and service data as defined by @tagMeshSp.
 *
 * @note A sample implementation of this interface can be found in `examples/common/src/mesh_adv.c`.
 * No changes are needed unless additional UUIDs or service data is desired.
 *
 * @{
 */

/** Tag used to set up Mesh GATT parameters. */
#define MESH_SOFTDEVICE_CONN_CFG_TAG (1)

/**
 * Default advertisement interval used by the mesh stack.
 */
#define MESH_ADV_INTERVAL_DEFAULT (MSEC_TO_UNITS(200, UNIT_0_625_MS))
/** Advertise indefinitely. */
#define MESH_ADV_TIMEOUT_INFINITE  (BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED)

/**
 * Sets the given service UUID and data in the advertisement data.
 *
 * @param[in] service_uuid   A 16-bit Bluetooth SIG assigned UUID.
 * @param[in] p_service_data Pointer to variable length service data associated with the UUID.
 * @param[in] length         Length of the service data.
 */
void mesh_adv_data_set(uint16_t service_uuid, const uint8_t * p_service_data, uint8_t length);

/**
 * Sets the advertisement parameters (timeout and interval).
 *
 * @param[in] timeout_ms     Advertisement timeout (in milliseconds).
 * @param[in] interval_units Advertisement interval (in 0.625 ms units).
 */
void mesh_adv_params_set(uint32_t timeout_ms, uint32_t interval_units);

/**
 * Starts advertising connectable advertisements.
 *
 * @note This call will assert if the advertisement data has not been set.
 */
void mesh_adv_start(void);

/**
 * Stops the connectable advertisements.
 */
void mesh_adv_stop(void);

/** @} end of MESH_ADV */
#endif /* MESH_ADV_H__ */
