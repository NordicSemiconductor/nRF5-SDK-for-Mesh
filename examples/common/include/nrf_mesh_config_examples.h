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

#ifndef NRF_MESH_CONFIG_EXAMPLES_H__
#define NRF_MESH_CONFIG_EXAMPLES_H__

#include "nrf_mesh_config_app.h"

/**
 * @defgroup NRF_MESH_CONFIG_EXAMPLES Application support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration of the application support modules.
 * @{
 */

/**
 * @defgroup RTT_INPUT_CONFIG RTT Input module configuration (mesh examples)
 * Configuration for compile time. Part of the RTT Input module for mesh examples.
 * @{
 */

/** Enable RTT Input support. */
#ifndef RTT_INPUT_ENABLED
#define RTT_INPUT_ENABLED 1
#endif

/** @} end of RTT_INPUT_CONFIG */

/**
 * @defgroup SIMPLE_HAL_CONFIG Simple Hardware Abstraction Layer configuration (mesh examples)
 * Configuration for compile time. Part of the Simple Hardware Abstraction Layer for mesh examples.
 * @{
 */

/** Enable support for LEDs. */
#ifndef SIMPLE_HAL_LEDS_ENABLED
#define SIMPLE_HAL_LEDS_ENABLED 1
#endif

/** @} end of SIMPLE_HAL_CONFIG */

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE SoftDevice support module.
 *
 * @{
 */

/** GAP device name. */
#ifndef GAP_DEVICE_NAME
#define GAP_DEVICE_NAME                 "nRF5x Mesh Node"
#endif

/** Minimum acceptable connection interval. */
#ifndef MIN_CONN_INTERVAL
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)
#endif

/** Maximum acceptable connection interval. */
#ifndef MAX_CONN_INTERVAL
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)
#endif

/** Slave latency. */
#ifndef SLAVE_LATENCY
#define SLAVE_LATENCY                   0
#endif

/** Connection supervisory timeout (4 seconds). */
#ifndef CONN_SUP_TIMEOUT
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
#endif

/** Time from initiating the event (connect or start of notification) to the first
 * time sd_ble_gap_conn_param_update is called. */
#ifndef FIRST_CONN_PARAMS_UPDATE_DELAY
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)
#endif

/** Time between each call to sd_ble_gap_conn_param_update after the first call. */
#ifndef NEXT_CONN_PARAMS_UPDATE_DELAY
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)
#endif

/** Number of attempts before giving up the connection parameter negotiation. */
#ifndef MAX_CONN_PARAMS_UPDATE_COUNT
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#endif

/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

/**
 * @defgroup DFU_SUPPORT_CONFIG BLE DFU support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE DFU support module.
 *
 * @{
 */

/** Enable BLE DFU support module. */
#ifndef BLE_DFU_SUPPORT_ENABLED
#define BLE_DFU_SUPPORT_ENABLED 0
#endif

/** @} end of DFU_SUPPORT_CONFIG */

/** @} end of NRF_MESH_CONFIG_EXAMPLES */

#endif /* NRF_MESH_CONFIG_EXAMPLES_H__ */
