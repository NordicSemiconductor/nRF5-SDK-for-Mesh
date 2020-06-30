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

#ifndef NRF_MESH_CONFIG_APP_H__
#define NRF_MESH_CONFIG_APP_H__

/**
 * @defgroup NRF_MESH_CONFIG_APP nRF Mesh application configuration
 *
 * Application configuration file. Copy it into every
 * application, and customize to fit the application requirements.
 * @{
 */

/**
 * @defgroup MODEL_CONFIG Model layer configuration parameters
 */

/** Define for acknowledging message transaction timeout.
 * @note @tagMeshSp recommends this to be minimum 60s. However, using
 * recommendation can result in client getting blocked for a significant amount of time (60s), if
 * acknowledged transaction does not receive a response.
 */
#define MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT  (SEC_TO_US(10))

/** @} end of MODEL_CONFIG */

/**
 * @defgroup DEVICE_CONFIG Device configuration
 *
 * @{
 */

/** Device company identifier. */
#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)

/** Device product identifier. */
#define DEVICE_PRODUCT_ID (0x0000)

/** Device version identifier. */
#define DEVICE_VERSION_ID (0x0000)

/** @} end of DEVICE_CONFIG */

/**
 * @defgroup ACCESS_CONFIG Access layer configuration
 * @{
 */

/**
 * The default TTL value for the node.
 */
#define ACCESS_DEFAULT_TTL (8)

/**
 * The number of models in the application.
 *
 * @note To fit the configuration and health models, this value must equal at least
 * the number of models needed by the application plus two.
 */
#define ACCESS_MODEL_COUNT (1 + /* Configuration server */  \
                            1 + /* Health server */  \
                            1   /* Generic OnOff client */)

/**
 * The number of elements in the application.
 *
 * @warning If the application is to support _multiple instances_ of the _same_ model, these instances
 * cannot be in the same element and a separate element is needed for each new instance of the same model.
 */
#define ACCESS_ELEMENT_COUNT (2)

/**
 * The number of allocated subscription lists for the application.
 *
 * @note This value must equal @ref ACCESS_MODEL_COUNT minus the number of
 * models operating on shared states.
 */
#define ACCESS_SUBSCRIPTION_LIST_COUNT (ACCESS_MODEL_COUNT)

/**
 * @defgroup ACCESS_RELIABLE_CONFIG Configuration of access layer reliable transfer
 * @{
 */

/** Number of the allowed parallel transfers (size of internal context pool). */
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)

/** @} end of ACCESS_RELIABLE_CONFIG */


/** @} end of ACCESS_CONFIG */


/**
 * @ingroup HEALTH_MODEL
 * @{
 */

/** The number of instances of the health server model. */
#define HEALTH_SERVER_ELEMENT_COUNT (1)

/** @} end of HEALTH_MODEL */


/**
 * @defgroup DSM_CONFIG Device State Manager configuration
 * Sizes for the internal storage of the Device State Manager.
 * @{
 */
/** Maximum number of subnetworks. */
#define DSM_SUBNET_MAX                                  (1)
/** Maximum number of applications. */
#define DSM_APP_MAX                                     (1)
/** Maximum number of device keys. */
#define DSM_DEVICE_MAX                                  (1)
/** Maximum number of virtual addresses. */
#define DSM_VIRTUAL_ADDR_MAX                            (1)
/** Maximum number of non-virtual addresses. */
#define DSM_NONVIRTUAL_ADDR_MAX                         (8)
/** @} end of DSM_CONFIG */

/** @} */

/**
 * @defgroup NRF_MESH_CONFIG_CORE Compile time configuration
 * Configuration of the compilation of the core mesh modules.
 * @ingroup CORE_CONFIG
 * @{
 */

/**
 * @defgroup MESH_CONFIG_GATT GATT configuration defines
 * @{
 */

/** PB-GATT feature. To be enabled only in combination with linking GATT files. */
#define MESH_FEATURE_PB_GATT_ENABLED                    (1)

/** GATT proxy feature. To be enabled only in combination with linking GATT proxy files. */
#define MESH_FEATURE_GATT_PROXY_ENABLED                 (1)

/** @} end of MESH_CONFIG_GATT */

/**
 * @defgroup MESH_CONFIG_FRIENDSHIP Friendship configuration defines
 * @{
 */

/** LPN feature */
#define MESH_FEATURE_LPN_ENABLED 1

/** @} end of MESH_CONFIG_FRIENDSHIP */

/** @} end of NRF_MESH_CONFIG_CORE */

/**
 * @defgroup NRF_MESH_CONFIG_EXAMPLES nRF Mesh example configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration common to all Mesh examples.
 * @{
 */

/**
 * @defgroup NRF_MESH_PROVISIONEE_CONFIG Provisionee support module configuration (Mesh examples)
 * Compile time configuration of the provisioning bearers for Mesh examples.
 * @{
 */

/** Enable Advertising-based provisioning bearer, PB-ADV. */
#define MESH_FEATURE_PB_ADV_ENABLED 0

/** @} end of NRF_MESH_PROVISIONEE_CONFIG */

/**
 * @defgroup RTT_INPUT_CONFIG RTT Input module configuration (mesh examples)
 * Configuration for compile time. Part of the RTT Input module for mesh examples.
 * @{
 */

/** Enable RTT Input support. */
#define RTT_INPUT_ENABLED 1

/** @} end of RTT_INPUT_CONFIG */

/**
 * @defgroup SIMPLE_HAL_CONFIG Simple Hardware Abstraction Layer configuration (mesh examples)
 * Configuration for compile time. Part of the Simple Hardware Abstraction Layer for mesh examples.
 * @{
 */

/** Enable support for LEDs. */
#define SIMPLE_HAL_LEDS_ENABLED 1

/** @} end of SIMPLE_HAL_CONFIG */

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE SoftDevice support module.
 *
 * @{
 */

/** GAP device name. */
#define GAP_DEVICE_NAME                 "nRF5x Mesh LPN Switch"

/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

/**
 * @defgroup NRF_MESH_CONFIG_PROV_BEARER Provisioning Bearer Configuration
 * Compile time configuration of the provisioning bearers.
 * @{
 */

/**
 * The default advertisement interval of the unprovisioned beacon. Meant for PB-GATT.
 *
 * @warning If the advertisement interval is set to below 200 ms, the mesh will not be able to
 * allocate sufficiently large timeslots for its persistent backend from the SoftDevice.
 */
#define NRF_MESH_PROV_BEARER_GATT_UNPROV_BEACON_INTERVAL_MS 2000


/**
 * Duration of the Mesh GATT Proxy Node Identity advertisements.
 *
 * @note Override the default duration (60 seconds) to save power.
 */
#define MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS 5000

/**
 * Default Proxy state.
 *
 * @note This define overrides the default proxy state. If set to false, the device will advertise the Node
 * Identity, but will not fall back to advertising the Network ID after provisioning.
 */
#define PROXY_ENABLED_DEFAULT (true)

/** @} end of NRF_MESH_CONFIG_PROV_BEARER */


/**
 * @defgroup DFU_SUPPORT_CONFIG BLE DFU support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE DFU support module.
 *
 * @{
 */

/** Enable BLE DFU support module. */
#define BLE_DFU_SUPPORT_ENABLED 0

/** @} end of DFU_SUPPORT_CONFIG */

/** @} end of NRF_MESH_CONFIG_EXAMPLES */

#endif /* NRF_MESH_CONFIG_APP_H__ */
