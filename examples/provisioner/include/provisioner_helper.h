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

#ifndef PROVISIONER_HELPER_H__
#define PROVISIONER_HELPER_H__


#include "nrf_mesh_prov_events.h"
#include "config_client.h"
#include "health_client.h"

#include "network_setup_types.h"
#include "example_network_config.h"

/**
 * @defgroup PROVISIONER_HELPER Helper module encapsulating the provisioner role
 *
 * This library provides simple APIs for the provisioner role. This library uses
 * provisioning APIs of the mesh stack, and provides a layer above it. It defines a set of events
 * that will be forwarded to the user application during the provisioning process.
 *
 * User application can initiate the provisioning process and repond with appropriate data when
 * events are generated. This module handles the process failures, timeouts and initiates retry.
 *
 * After provisioning is completed, it adds the given AppKey, at the give index, to the node
 * and binds all the models to this App Key, and configures the major node features as specified.
 * This modules takes care of adding node's address and keys to the DSM after it is provisioned.
 *
 * @{
 */

/**
 * @defgroup PROVISIONER_USER_CALLBACKS User callbacks
 * @{
 */

/**
 * Provisioning helper application data store trigger callback type.
 *
 * After provisioning is completed successfully, application receives this callback to trigger
 * data storage operation.
 */
typedef void (*prov_helper_data_store_cb_t) (void);

/** Callback to user indicating that the provisioning failed. */
typedef void (*prov_helper_failed_cb_t)(void);

/** Callback to user indicating that the provisioning succeeded. */
typedef void (*prov_helper_success_cb_t)(void);

/** @} end of PROVISIONER_USER_CALLBACKS */

/** UUID filter to be used while looking for devices to be provisioned. This filter will
 * be a sub-array matching filter if length of the given array is less than @ref NRF_MESH_UUID_SIZE.
 */
typedef struct
{
    /** Expected UUID byte array filter */
    const uint8_t * p_uuid;
    /** Length of the `p_uuid` filter array */
    uint32_t length;
} prov_helper_uuid_filter_t;

/** Parameter structure used during the initialization of provisioner helper module. */
typedef struct
{
    /** Application-specific structure storing dsm handles */
    network_dsm_handles_data_volatile_t * p_dev_data;
    /** Pointer to the network device count data */
    network_stats_data_stored_t * p_nw_data;
    /** key index for default netkey */
    uint16_t netkey_idx;
    /** Time in seconds during which the device to be provisioned will identify itself using any means it can */
    uint8_t attention_duration_s;
    /** Application should perform the required application specific data storage in this callback */
    prov_helper_data_store_cb_t p_data_store_cb;
    /** Application callback to be called when provisioning succeeds */
    prov_helper_success_cb_t p_prov_success_cb;
    /** Application callback to be called when provisioning fails */
    prov_helper_failed_cb_t p_prov_failed_cb;
} mesh_provisioner_init_params_t;

/** Initializes the provisioner, but does not start the scan.
 * @param[in] p_prov_init_info   Pointer to the @ref mesh_provisioner_init_params_t structure.
 */
void prov_helper_init(mesh_provisioner_init_params_t * p_prov_init_info);

/** Starts the scanning for unprovisioned nodes, and initiates provisioning process if given
 * conditions are met.
 */
void prov_helper_scan_start(void);

/** Stops the scanning for unprovisioned nodes */
void prov_helper_scan_stop(void);

/**
 * This API sets the address and UUID that will be used by the provisioner library.
 * The address will be chosen by the provisioner based on URI information.
 */
void prov_helper_provision_next_device(void);

/** Initializes the local node by adding self address, self device key, and by binding models. */
void prov_helper_provision_self(void);

/**
 *  Retrives the number of elements for the recently provisioned device.
 *
 * @returns value of the number of elements based on the capabilities messages received from the
 * provisioned node during provisioning process. Returned value will be undefined if no device has
 * been provisioned before calling this function.
 */
uint32_t prov_helper_element_count_get(void);

/**
 * Retrieves stored handles for the netkey, appkey and self device key.
 */
void prov_helper_device_handles_load(void);

/** @} end of PROVISIONER_HELPER */

#endif /* PROVISIONER_HELPER_H__ */
