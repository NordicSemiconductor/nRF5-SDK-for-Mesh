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

#ifndef MESH_PROVISIONEE_H__
#define MESH_PROVISIONEE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup MESH_PROVISIONEE Mesh examples provisionee support module
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Mesh examples support module for starting the provisioning process for a device in the
 * provisionee role using PB-ADV and static OOB authentication.
 * @{
 */

/**
 * Set BLE stack option callback type.
 *
 * This callback is called to indicate that BLE stack was restarted and any
 * BLE option can be set at this point through @link_sd_ble_opt_set function.
 */
typedef void (*mesh_provisionee_prov_sd_ble_opt_set_cb_t)(void);

/**
 * Provisioning complete callback type.
 *
 * This callback is called to indicate that the device has been successfully provisioned.
 */
typedef void (*mesh_provisionee_prov_complete_cb_t)(void);

/**
 * Start device identification callback type.
 *
 * This callback is called to indicate that the device can start identifying itself
 * using any means it can to attract attention.
 *
 * @note This callback is called every time when the device is asked to start identifying
 * itself.
 *
 * @param[in]   attention_duration_s    Time in seconds during which the device identifies
 *                                      itself using any means it can.
 */
typedef void (*mesh_provisionee_prov_device_identification_start)(uint8_t attention_duration_s);

/**
 * Stop device identification callback type.
 *
 * This callback is called to indicate that the device should stop identifying itself.
 *
 * @note This callback is called only once per provisioning process.
 */
typedef void (*mesh_provisionee_prov_device_identification_stop)(void);

/**
 * Provisioning abort callback type.
 *
 * This callback is called if the provisioning process is aborted.
 *
 * @note This callback can be used to stop the device from identifying itself if the provisioning process is aborted
 *       before @ref mesh_provisionee_prov_device_identification_stop is called.
 */
typedef void (*mesh_provisionee_prov_abort)(void);

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
     * Pointer to a function used to provide ability to set any BLE option
     * after the BLE stack was restarted during the provisioning. Use it if you need
     * to set any BLE option through @link_sd_ble_opt_set function.
     * Can be set to @c NULL if not used.
     */
    mesh_provisionee_prov_sd_ble_opt_set_cb_t prov_sd_ble_opt_set_cb;

    /**
     * Pointer to a function used to signal the completion of the device provisioning
     * procedure. Can be set to @c NULL if not used.
     *
     * @note Getting this callback means that a device is at minimum _provisioned_, however,
     *       it does not imply anthing about model configuration, added keys, etc. That may
     *       be altered by a Configuration Client at any point in time.
     *       See @ref CONFIG_SERVER_EVENTS.
     */
    mesh_provisionee_prov_complete_cb_t prov_complete_cb;

    /**
     * Pointer to a function used to signal the device to start identifying itself.
     * Can be set to @c NULL if not used.
     */
    mesh_provisionee_prov_device_identification_start prov_device_identification_start_cb;

    /**
     * Pointer to a function used to signal the device to stop identifying itself.
     * Can be set to @c NULL if not used.
     */
    mesh_provisionee_prov_device_identification_stop prov_device_identification_stop_cb;

    /**
     * Pointer to a function used to signal the abort of the device provisioning
     * procedure. Can be set to @c NULL if not used.
     */
    mesh_provisionee_prov_abort prov_abort_cb;

    /**
     * NULL-terminated device URI string.
     * This is an optional field that can be used to add additional data to the unprovisioned
     * node broadcast beacon.
     */
    const char * p_device_uri;

    /**
     * Data used for static OOB authentication.
     * This should be a pointer to a 16 byte long data array.
     */
    const uint8_t * p_static_data;

} mesh_provisionee_start_params_t;

/**
 * Start the provisioning process for a device in the provisionee role using static OOB
 * authentication.
 *
 * @param[in] p_start_params  Pointer to structure containing parameters related to the provisioning
 *                            procedure.
 *
 * @retval NRF_ERROR_INVALID_STATE  The provisioning module is not in idle state.
 * @retval NRF_ERROR_INVALID_PARAM  The p_static_data parameter is NULL.
 * @retval NRF_SUCCESS              The provisioning was started successfully.
 */
uint32_t mesh_provisionee_prov_start(const mesh_provisionee_start_params_t * p_start_params);

/**
 * Stops listening for incoming provisioning links.
 *
 * @retval NRF_ERROR_INVALID_STATE The provisionee is not currently listening.
 * @retval NRF_SUCCESS             Successfully stopped listening.
 */
uint32_t mesh_provisionee_prov_listen_stop(void);

/**
 * @}
 */

#endif /* MESH_PROVISIONEE_H__ */
