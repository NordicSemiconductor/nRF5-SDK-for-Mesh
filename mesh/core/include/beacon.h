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
#ifndef MESH_BEACON_H__
#define MESH_BEACON_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_mesh.h"
#include "packet.h"
#include "advertiser.h"
#include "scanner.h"

#include "nrf_mesh_defines.h"
#include "nrf_mesh_opt.h"

/**
 * @defgroup BEACON Bluetooth Mesh Beacons
 * @ingroup MESH_CORE
 * This module manages Bluetooth Mesh beacons.
 * @{
 */

/**
 * @defgroup BEACON_TYPE Beacon types
 *
 * Beacon types used to identify the beacon type on air.
 * @{
 */
#define BEACON_TYPE_UNPROV        (0x00)  /**< Beacon type for unprovisioned beacons. */
#define BEACON_TYPE_SEC_NET_BCAST (0x01)  /**< Beacon type for secure network broadcast beacons. */
#define BEACON_TYPE_INVALID       (0xFF)  /**< Invalid beacon type. */
/** @} */

#define BEACON_PACKET_OVERHEAD    (1)     /**< Overhead of beacon packet. */
#define BEACON_PACKET_AD_LEN_OVERHEAD (BEACON_PACKET_OVERHEAD + BLE_AD_DATA_OVERHEAD) /**< Overhead of beacon packet for the AD length field. */
#define BEACON_DATA_MAXLEN            (BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH - (sizeof(ble_ad_header_t) + BEACON_PACKET_OVERHEAD)) /**< Maximum length of beacon data. */

/** Generic beacon packet */
typedef struct
{
    uint8_t beacon_type;  /**< Beacon type, see @ref BEACON_TYPE */
    uint8_t payload[];    /**< Beacon payload. */
} beacon_packet_t;

/**
 * Create a mesh beacon with the given contents.
 *
 * @param[in,out] p_adv Advertiser to allocate the beacon packet from.
 * @param[in] beacon_type The beacon type to run. Must be a value in @ref
 * BEACON_TYPE, that's not @ref BEACON_TYPE_INVALID.
 * @param[in] p_payload Pointer to payload data that will be copied into the
 * beacon packet. Cannot be NULL.
 * @param[in] payload_len Length of the payload given in the p_payload
 * parameter. Must be between 1 and @ref BEACON_DATA_MAXLEN, inclusive.
 *
 * @return Packet pointer with a single reference, or NULL if creation failed.
 */
adv_packet_t * beacon_create(advertiser_t * p_adv, uint8_t beacon_type, const void* p_payload, uint8_t payload_len);
/**
 * @}
 */
#endif
