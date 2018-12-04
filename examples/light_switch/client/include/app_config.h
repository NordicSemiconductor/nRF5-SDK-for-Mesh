/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

#include <stdbool.h>

/**
 * @defgroup APP_SPECIFIC_DEFINES Application-specific macro definitions
 *
 * @{
 */

/** Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_CONFIG_FORCE_SEGMENTATION  (false)

/** Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_CONFIG_MIC_SIZE            (NRF_MESH_TRANSMIC_SIZE_SMALL)

/** Delay value used by the OnOff client for sending OnOff Set messages. */
#define APP_CONFIG_ONOFF_DELAY_MS           (50)

/** Transition time value used by the OnOff client for sending OnOff Set messages. */
#define APP_CONFIG_ONOFF_TRANSITION_TIME_MS (100)

/** @} end of APP_SPECIFIC_DEFINES */

/**
 * @defgroup APP_SDK_CONFIG SDK configuration
 *
 * Application-specific SDK configuration settings are provided here.
 *
 * @{
 */

/** Override default sdk_config.h values. */

/** Configuration for the BLE SoftDevice support module to be enabled. */
#define NRF_SDH_ENABLED 1
#define NRF_SDH_BLE_ENABLED 1
#define NRF_SDH_SOC_ENABLED 1
#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE 69
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 1
#define NRF_SDH_BLE_SERVICE_CHANGED 1
#define NRF_BLE_CONN_PARAMS_ENABLED 1

#define APP_TIMER_ENABLED 1
#define APP_TIMER_KEEPS_RTC_ACTIVE 1

/** @} end of APP_SDK_CONFIG */

#endif /* APP_CONFIG_H__ */
