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
 * @defgroup NRF_MESH_CONFIG_APP nRF Mesh app config
 *
 * Application side configuration file. Should be copied into every
 * application, and customized to fit its requirements.
 * @{
 */

/**
 * @defgroup MODEL_CONFIG Model layer configuration parameters
 */

/**
 * The maximum number of available flash instances for the light lightness models to store persistent data.
 */
#define LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX (1)

/**
 * The maximum number of CTL server instances used by the application.
 */
#define LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX (1)

/**
 * Minimum value of the color temperature (in Kelvin) supported by the hardware.
 */
#define LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN (0x0320)

/**
 * Maximum value of the color temperature (in Kelvin) supported by the hardware.
 */
#define LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX (0x4E20)

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
#define ACCESS_DEFAULT_TTL (20)

/**
 * The number of models in the application.
 *
 * @note To fit the configuration and health models, this value must equal at least
 * the number of models needed by the application plus two.
 */
#define ACCESS_MODEL_COUNT (/* Element 0:                                                                       */ \
                            1 + /* Config Server                                                                */ \
                            1 + /* Health Server                                                                */ \
                            1 + /* Generic DTT Server (extended by Generic Power OnOff Setup Sever)             */ \
                            1 + /* Generic OnOff Server (extended by Generic Power OnOff Server)                */ \
                            1 + /* Generic Power OnOff Server (extended by Generic Power OnOff Setup Server)    */ \
                            1 + /* Generic Power OnOff Setup Server (extended by Light Lightness Server)        */ \
                            1 + /* Generic Level Server (extended by Light Lightness Server)                    */ \
                            1 + /* Light Lightness Server (extended by Light Lightness Setup Server)            */ \
                            1 + /* Light Lightness Setup Server (required by Light CTL Server)                  */ \
                            1 + /* Light CTL Server (extended by Light CTL Setup Server)                        */ \
                            1 + /* Light CTL Setup Server                                                       */ \
                            1 + /* Scene Server (extends Scene Setup Server)                                    */ \
                            1 + /* Scene Setup Server                                                           */ \
                            /* Element 1:                                                                       */ \
                            1 + /* Generic Level Server (extended by Light CTL Temperature Server)              */ \
                            1   /* Light CTL Temperature Server (required by Light CTL Server)                  */)

/**
 * The number of elements in the application.
 *
 * @warning If the application is to support _multiple instances_ of the _same_ model, these instances
 * cannot be in the same element and a separate element is needed for each new instance of the same model.
 */
#define ACCESS_ELEMENT_COUNT (2)

/**
 * The number of scene setup server instances used by the application.
 */
#ifndef SCENE_SETUP_SERVER_INSTANCES_MAX
#define SCENE_SETUP_SERVER_INSTANCES_MAX (1)
#endif

/** Number of Scene Models to support storage.
 */
#define APP_SCENE_MODEL_COUNT (2)

/**
 * The number of allocated subscription lists for the application.
 *
 * @note This value must equal @ref ACCESS_MODEL_COUNT minus the number of
 * models operating on shared states.
 */
#define ACCESS_SUBSCRIPTION_LIST_COUNT (4)

/**
 * @defgroup ACCESS_RELIABLE_CONFIG Configuration of access layer reliable transfer
 * @{
 */

/** Number of the allowed parallel transfers (size of the internal context pool). */
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
#define DSM_SUBNET_MAX                                  (4)
/** Maximum number of applications. */
#define DSM_APP_MAX                                     (8)
/** Maximum number of device keys. */
#define DSM_DEVICE_MAX                                  (1)
/** Maximum number of virtual addresses. */
#define DSM_VIRTUAL_ADDR_MAX                            (1)
/** Maximum number of non-virtual addresses. One for each of the servers and a group address. */
#define DSM_NONVIRTUAL_ADDR_MAX                         (ACCESS_MODEL_COUNT + 1)
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
/** @} end of NRF_MESH_CONFIG_CORE */

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration.
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of BLE SoftDevice support module.
 *
 * @{
 */
#define GAP_DEVICE_NAME                 "nRF5x Mesh Light CTL Setup Server"
/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

#endif /* NRF_MESH_CONFIG_APP_H__ */
