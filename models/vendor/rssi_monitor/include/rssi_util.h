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

#ifndef RSSI_UTIL_H__
#define RSSI_UTIL_H__

#include <stdbool.h>
#include <stdint.h>
#include "access.h"

/**
 * @defgroup RSSI_UTIL RSSI monitor util model interface
 * @ingroup RSSI_COMMON
 * @{
 */

/** Model ID for the RSSI Util model. */
#define RSSI_UTIL_MODEL_ID 0x0007

/** Object type for rssi util instances. */
typedef struct __rssi_util_t rssi_util_t;

/** Rssi util instance structure */
struct __rssi_util_t
{
    access_model_handle_t model_handle;
};

/** Rssi model opcodes. */
typedef enum
{
    RSSI_OPCODE_REQUEST_DATABASE_BEACON = 0xC7,
    RSSI_OPCODE_SEND_DATABASE_BEACON = 0xC8,
} rssi_util_opcode_t;

/** Initializes the rssi util model.
 *
 * @param[in,out] p_util Pointer to the rssi util instance structure.
 *
 * @retval NRF_SUCCESS The model was successfully initialized.
 *
 * @see access_model_add()
 */
uint32_t rssi_util_init(rssi_util_t * p_util);

/** Searches for a corresponding element address to a ingoing mac address. Starts the process of gathering 
 * mac/element address pairs from nearby nodes if the corresponding element address is not found 
 *
 * @param[in] p_util Pointer to the rssi util instance structure.
 * @param[in] p_mac_addr pointer to the ingoing mac address.
 * @param[in,out] p_element_addr pointer to where the corresponding element address is to be stored.
 *
 * @retval NRF_SUCCESS A corresponding element address was found.
 * @retval NRF_ERROR_NULL NULL pointer in function arguments
 * @retval NRF_ERROR_NOT_FOUND A corresponding element address was not found.
 *
 */
uint32_t rssi_util_mac_to_element_addr_find(rssi_util_t * p_util, const uint8_t* p_mac_addr, dsm_local_unicast_address_t* p_element_addr);

/**@} end of RSSI_UTIL */

#endif

