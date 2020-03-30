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

#ifndef NRF_MESH_CONFIG_PROV_H__
#define NRF_MESH_CONFIG_PROV_H__

#ifdef CONFIG_APP_IN_CORE
#include "nrf_mesh_config_app.h"
#endif

/**
 * @defgroup NRF_MESH_CONFIG_PROV Provisioning configuration
 * @ingroup MESH_API_GROUP_PROV
 * Compile time configuration of the provisioning module.
 * @{
 */

/** @tagMeshSp section 5.4.4: "The provisioning protocol shall have a minimum
 * timeout of 60 seconds that is reset each time a provisioning protocol PDU is sent or received."
 */
#ifndef NRF_MESH_PROV_LINK_TIMEOUT_MIN_US
#define NRF_MESH_PROV_LINK_TIMEOUT_MIN_US 60000000
#endif

#if NRF_MESH_PROV_LINK_TIMEOUT_MIN_US < 60000000
#warning Provisioning link timeout shall have a minimum timeout of 60 seconds according to the @tagMeshSp.
#endif

/**
 * @defgroup NRF_MESH_CONFIG_PROV_BEARER Provisioning Bearer Configuration
 * Compile time configuration of the provisioning bearers.
 * @{
 */

/**
 * PB-ADV feature.
 *
 * This feature should normally always be enabled. Without it, a node cannot be
 * provisioned over the advertising bearer. You can disable this feature to save
 * power when the device is acting as a Low Power node.
 */
#ifndef MESH_FEATURE_PB_ADV_ENABLED
#define MESH_FEATURE_PB_ADV_ENABLED 1
#endif

/** PB-GATT feature. To be enabled only in combination with linking GATT files. */
#ifndef MESH_FEATURE_PB_GATT_ENABLED
#define MESH_FEATURE_PB_GATT_ENABLED 0
#endif

/** The default advertisement interval of the unprovisioned beacon. Meant for PB-ADV. */
#ifndef NRF_MESH_PROV_BEARER_ADV_UNPROV_BEACON_INTERVAL_MS
#define NRF_MESH_PROV_BEARER_ADV_UNPROV_BEACON_INTERVAL_MS 2000
#endif

/**
 * The default advertisement interval of the unprovisioned beacon. Meant for PB-GATT.
 */
#ifndef NRF_MESH_PROV_BEARER_GATT_UNPROV_BEACON_INTERVAL_MS
#define NRF_MESH_PROV_BEARER_GATT_UNPROV_BEACON_INTERVAL_MS 200
#endif

/** Size of the buffer for the outgoing packet buffer for PB-ADV. */
#ifndef NRF_MESH_PROV_BEARER_ADV_TX_BUFFER_SIZE
#define NRF_MESH_PROV_BEARER_ADV_TX_BUFFER_SIZE  128
#endif

/** @} end of NRF_MESH_CONFIG_PROV_BEARER */

/**
 * @defgroup MESH_CONFIG_PROVISIONEE Provisionee configuration
 * @{
 */

/** Set to 1 to force the mesh stack to use secure provisioning. */
#ifndef NRF_MESH_PROV_FORCE_SECURE_PROVISIONING
#define NRF_MESH_PROV_FORCE_SECURE_PROVISIONING (0)
#endif

/** @} end of MESH_CONFIG_PROVISIONEE */

/** @} end of NRF_MESH_CONFIG_PROV */

#endif
