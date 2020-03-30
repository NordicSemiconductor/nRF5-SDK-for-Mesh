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

#include <stdint.h>

#ifndef BLE_SOFTDEVICE_SUPPORT__
#define BLE_SOFTDEVICE_SUPPORT__

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT BLE SoftDevice support module
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application support module for initializing the @link_SD_Handler, @link_BLE_stack,
 * @link_GAP, and the @link_BLE_CPN module.
 *
 * The GAP and the Connection Parameters Negotiation module are required
 * for @ref md_doc_user_guide_modules_provisioning_gatt_proxy features.
 *
 * @{
 */

/**
 * Initializes the SoftDevice Handler and the BLE stack, registers Mesh
 * handler for SoC events.
 */
void ble_stack_init(void);

/**
 * Initializes the Generic Attribute Profile (GAP).
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 * parameters of the device, including the device name and the preferred
 * connection parameters.
 */
void gap_params_init(void);

/**
 * Initializes the Connection Parameters Negotiation module.
 *
 * @details Registers the handler for the Connection Parameters Negotiation module events that
 * initializes disconnection if the Connection Parameters Negotiation procedure
 * is failed.
 */
void conn_params_init(void);

/**
 * @}
 */

#endif /* BLE_SOFTDEVICE_SUPPORT__ */
