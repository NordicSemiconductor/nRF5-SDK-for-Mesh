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
#ifndef INSTABURST_H__
#define INSTABURST_H__

/**
 * @defgroup INSTABURST Instaburst feature
 * @ingroup MESH_API_GROUP_NORDIC
 *
 * The Instaburst feature implements a subset of the Bluetooth 5.0 feature "Advertising
 * extensions" to achieve increased throughput for Mesh traffic. Instaburst is a feature
 * specific to Nordic's nRF5 SDK for Mesh, and the implementation does not aim to provide
 * generic, spec-compliant Advertising Extension functionality, but rather provide means
 * to increase Mesh throughput between Nordic devices.
 *
 * @warning Instaburst is a Nordic-specific feature that does not adhere to the Bluetooth Mesh
 * specification. It does not have the same requirements for test coverage, API stability
 * or specification compliance as the rest of Nordic's nRF5 SDK for Mesh.
 *
 * @{
 */

#include <stdint.h>
#include <stdbool.h>

#include "bearer_event.h"

/**
 * @defgroup INSTABURST_DEFINES Defines
 * Instaburst definitions
 * @{
 */

/** Maximum Set ID. */
#define INSTABURST_SET_ID_MAX 0x0F
/** Mask for the advertising data id field. */
#define INSTABURST_DATA_ID_MASK 0x0FFF
/** Highest channel permitted with Instaburst. */
#define INSTABURST_CHANNEL_INDEX_MAX 36
/** Number of entries in the event ID cache. */
#define INSTABURST_EVENT_ID_CACHE_SIZE  32

/** @} */

/**
 * Initializes the instaburst module and all its submodules.
 *
 * @param[in] lfclk_ppm Low frequency clock accuracy in parts per million.
 * @param[in] packet_process_cb Callback to signal on every RX.
 */
void instaburst_init(uint32_t lfclk_ppm, bearer_event_flag_callback_t packet_process_cb);

/** @} */

#endif /* INSTABURST_H__ */
