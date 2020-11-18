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

#ifndef MESH_STACK_H__
#define MESH_STACK_H__

#include "nrf_mesh.h"
#include "config_server_events.h"
#include "health_server.h"
#include "nrf_mesh_prov_events.h"

/**
 * @defgroup MESH_STACK Mesh stack
 *
 * High level management API for the mesh stack
 *
 * Functions for initializing and managing all mesh stack modules, including foundation models.
 * @{
 */

/**
 * Models initialization callback.
 *
 * This function is called to allow an application to initialize any models it needs before
 * configuration starts.
 */
typedef void (*mesh_stack_models_init_cb_t)(void);

/**
 * Mesh stack configuration parameters.
 *
 * Some fields are optional; the description of the specific fields notes if the
 * value of a field can be omitted. In this case, the value of the field should be
 * set to 0 (@c NULL for pointer values).
 */
typedef struct
{
    /**
     * Core initialization parameters structure.
     */
    nrf_mesh_init_params_t core;

    struct
    {
        /**
         * Pointer to a function used to inform about events from the configuration server.
         * Can be set to @c NULL if not used.
         *
         * @warning If the device receives a @ref CONFIG_SERVER_EVT_NODE_RESET event, it will erase all
         * mesh data from the persistent storage and reset the device after forwarding the event to the
         * application. The application developer should take care to be in a defined state after reset.
         * This is considered a "factory reset" of the mesh device.
         */
        config_server_evt_cb_t config_server_cb;

        /**
         * Attention callback function.
         * This callback is called when the device should enable or disable the attention state.
         * When the attention state is enabled, the device should make itself noticeable, by,
         * for example, blinking an LED, vibrating or making sounds. If set to @c NULL, the
         * attention state will be considered to be unsupported by the device.
         */
        health_server_attention_cb_t health_server_attention_cb;

        /**
         * Self-test function array.
         * Devices can provide a list of self-test functions that can be run by applications
         * providing a health client. This is useful if the device has functionality that provides
         * support for testing and status reporting. If set to @c NULL, the self-test functionality
         * will be considered to be unsupported by the device.
         *
         * @note If provided, the self-test array must be statically allocated and available for
         *       the entire life-time of the application.
         */
        const health_server_selftest_t * p_health_server_selftest_array;

        /**
         * Number of self-tests provided by @c p_health_server_selftest_array.
         */
        uint8_t health_server_num_selftests;

        /**
         * Pointer to a function used to allow initialization of application-specific models.
         * Any models used by the application should be initialized in this callback function,
         * which ensure that these models are available immediately after the device has been
         * provisioned. Can be set to @c NULL if not used.
         */
        mesh_stack_models_init_cb_t models_init_cb;
    } models;

} mesh_stack_init_params_t;

/**
 * Initialize the mesh stack.
 *
 * This function initializes all mesh stack modules, including the foundation models.
 *
 * @param[in]  p_init_params        Pointer to initialization parameter structure.
 * @param[out] p_device_provisioned Returns the device's provisioning state. Set to NULL if not required.
 *
 * @retval NRF_ERROR_NULL          The @c p_params parameter was @c NULL.
 * @retval NRF_ERROR_INVALID_STATE The device has already been configured.
 * @retval NRF_ERROR_INVALID_DATA  Data in the persistent memory was corrupted.
 *                                 Stack is reset to default settings, all persistent data is lost.
 *                                 Device requires reset to start as unprovisioned one.
 *                                 @warning After this status, no mesh API functions can be
 *                                 called since it might cause unpredictable behavior.
 * @retval NRF_ERROR_INVALID_PARAM One or more of the parameters in the @c p_params structure
 *                                 were invalid.
 * @retval NRF_SUCCESS             Initialization was successful.
 */
uint32_t mesh_stack_init(const mesh_stack_init_params_t * p_init_params,
                         bool * p_device_provisioned);

/**
 * Start dynamic behavior on the mesh stack.
 *
 * @warning After calling this function, no mesh API functions can be
 *          called from an IRQ priority other than the one specified
 *          in @ref nrf_mesh_init_params_t.irq_priority.
 *
 * @retval NRF_ERROR_INVALID_STATE The mesh stack has not been initialized,
 *                                 or it has already been started.
 * @retval NRF_SUCCESS             The mesh stack was successfully started.
 */
uint32_t mesh_stack_start(void);

/**
 * Start the power down procedure.
 * The function stops timer scheduler (timeslot system still works
 * to store @ref MESH_CONFIG_STRATEGY_ON_POWER_DOWN files, app_timer works as well).
 * The function stops and disables scanner, advertiser, bearer handler and GATT functionality.
 * When the power down procedure has been completed, the event
 * @ref NRF_MESH_EVT_READY_TO_POWER_OFF is generated and the stack is ready for power off.
 *
 * @warning After calling this function, no mesh API functions can be
 *          called since it might cause unpredictable behavior.
 *
 */
void mesh_stack_power_down(void);

/**
 * Store received provisioning data in flash.
 *
 * This function also binds the config server to the device key, and propagates the IV index to
 * the network state module.
 *
 * @param[in]  p_prov_data  Provisioning data to be stored.
 * @param[in]  p_devkey     Device key to be stored.
 *
 * @retval NRF_SUCCESS             Storing was successful.
 * @retval NRF_ERROR_NULL          Unexpected NULL pointer is given.
 * @retval NRF_ERROR_FORBIDDEN     Some of the data has been set before, and the device state must
 *                                 be reset before they can be changed again.
 * @retval NRF_ERROR_INVALID_DATA  The given address range is invalid or it overlaps with
 *                                 non-unicast type addresses.
 * @retval NRF_ERROR_INVALID_PARAM One or more of the parameters in the @c p_evt structure
 *                                 were invalid.
 * @retval NRF_ERROR_NO_MEM        The subnetwork or device key storage is out of space,
 *                                 @see DSM_SUBNET_MAX or DSM_DEVICE_MAX.
 * @retval NRF_ERROR_NOT_FOUND     Config server is not initialized.
 */
uint32_t mesh_stack_provisioning_data_store(const nrf_mesh_prov_provisioning_data_t * p_prov_data,
                                            const uint8_t * p_devkey);

/**
 * Clear the saved configuration and network state of the mesh node.
 *
 * This is a factory reset of the mesh stack.
 */
void mesh_stack_config_clear(void);

/**
 * Check if the device has been provisioned.
 *
 * @retval true   The device has been provisioned.
 * @retval false  The device has not been provisioned.
 */
bool mesh_stack_is_device_provisioned(void);

/**
 * Resets the device.
 *
 * @warning This function will return if there are any pending flash operations.
 *          In that case, the application should return from any function blocking the
 *          mesh from processing. When the flash operations are complete. The device will
 *          be reset.
 */
void mesh_stack_device_reset(void);

/**
 * Gets which flash areas used by the mesh stack for storing persistent data.
 *
 * @param[in,out] pp_start Returns a pointer to the first word used by the mesh stack for storing
 * persistent data, or NULL if no flash space is used for persistent data.
 * @param[in,out] p_length Returns the length of the mesh stack persistent data.
 *
 * @retval NRF_SUCCESS The parameters have successfully been populated.
 * @retval NRF_ERROR_NULL One or more of the parameters were NULL.
 */
uint32_t mesh_stack_persistence_flash_usage(const uint32_t ** pp_start, uint32_t * p_length);

/**
 * Gets a pointer to the Health Server instance in the primary element.
 *
 * @note The Health Server is initialized and added by the mesh stack module, and the pointer should
 * only be used for interacting with the @ref HEALTH_SERVER API.
 *
 * @returns A pointer to the Health Server instance in the mesh stack.
 */
health_server_t * mesh_stack_health_server_get(void);

/**
 * @}
 */

#endif /* MESH_STACK_H__ */
