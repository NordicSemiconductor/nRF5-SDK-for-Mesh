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

#ifndef NRF_MESH_CONFIG_BEARER_H__
#define NRF_MESH_CONFIG_BEARER_H__

#include "nrf_mesh_defines.h"
#include "bearer_defines.h"

/**
 * @defgroup NRF_MESH_CONFIG_BEARER Bearer configuration
 * @ingroup MESH_API_GROUP_BEARER
 * Compile time configuration of the bearer layer.
 * @{
 */

/**
 * @defgroup MESH_CONFIG_BEARER General configuration
 * General compile time configuration of the bearer layer.
 * @{
 */

/**
 * External power amplifier (PA) setup time in microseconds.
 *
 * Must be lower than the radio rampup time (as specified in the Electrical specification of each chip's Product Specification).
 */
#ifndef MESH_PA_SETUP_TIME_US
#define MESH_PA_SETUP_TIME_US 18
#endif

/**
 * External low noise amplifier (LNA) setup time in microseconds.
 *
 * Must be lower than the radio rampup time (as specified in the Electrical specification of each chip's Product Specification).
 */
#ifndef MESH_LNA_SETUP_TIME_US
#define MESH_LNA_SETUP_TIME_US 18
#endif

/** HF timer peripheral index to allocate for bearer handler. E.g. if set to 2, NRF_TIMER2 will be used. Must be a literal number. */
#ifndef BEARER_ACTION_TIMER_INDEX
#define BEARER_ACTION_TIMER_INDEX 2
#endif

/** Maximum overhead of application TX Complete call, */
#ifndef APPLICATION_TX_COMPLETE_OVERHEAD_US
#define APPLICATION_TX_COMPLETE_OVERHEAD_US 80
#endif

/** Default access address. */
#ifndef BEARER_ACCESS_ADDR_DEFAULT
#define BEARER_ACCESS_ADDR_DEFAULT BEARER_ACCESS_ADDR_NONCONN
#endif

/** Maximum number of channels an advertiser can transmit on. */
#ifndef BEARER_ADV_CHANNELS_MAX
#define BEARER_ADV_CHANNELS_MAX 3
#endif

/** Default advertisement interval. */
#ifndef BEARER_ADV_INT_DEFAULT_MS
#define BEARER_ADV_INT_DEFAULT_MS 20
#endif

/** Default scan interval */
#ifndef BEARER_SCAN_INT_DEFAULT_MS
#define BEARER_SCAN_INT_DEFAULT_MS 2000
#endif

/** Default scan window */
#ifndef BEARER_SCAN_WINDOW_DEFAULT_MS
#define BEARER_SCAN_WINDOW_DEFAULT_MS 2000
#endif

/** Default scanner buffer size */
#ifndef SCANNER_BUFFER_SIZE
#define SCANNER_BUFFER_SIZE 512
#endif

/** Buffer size for the Instaburst RX module. */
#ifndef INSTABURST_RX_BUFFER_SIZE
#define INSTABURST_RX_BUFFER_SIZE   (1024)
#endif

/** Instaburst feature flag. Normally enabled through CMake option. */
#ifndef EXPERIMENTAL_INSTABURST_ENABLED
#define EXPERIMENTAL_INSTABURST_ENABLED 0
#endif
/** @} end of MESH_CONFIG_BEARER */

/**
 * @defgroup MESH_CONFIG_BEARER_EVENT Bearer event configuration
 * Compile time configuration of the bearer event module.
 * @{
 */

/** Length of the asynchronous processing queue. */
#ifndef BEARER_EVENT_FIFO_SIZE
#define BEARER_EVENT_FIFO_SIZE 16
#endif

/** Number of flags available for allocation. */
#ifndef BEARER_EVENT_FLAG_COUNT
#define BEARER_EVENT_FLAG_COUNT     17
#endif

/**
 * Configure the bearer event module to use the SWI0 IRQ handler for processing the events.
 *
 * By default, the bearer event module uses the QDEC IRQ handler.
 * See @ref md_doc_user_guide_mesh_hw_resources for details.
 */
#ifndef BEARER_EVENT_USE_SWI0
#define BEARER_EVENT_USE_SWI0 0
#endif

/** @} end of MESH_CONFIG_BEARER_EVENT */


/** @} end of NRF_MESH_CONFIG_BEARER */

#endif
