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

#ifndef RSSI_SERVER_H__
#define RSSI_SERVER_H__

#include <stdbool.h>
#include <stdint.h> 
#include "access.h"

/**
 * @defgroup RSSI_SERVER RSSI monitor server model interface
 * @ingroup RSSI_COMMON
 * @{
 */

/** Structure for holding the raw rssi data from each node before it is handled and sent to the rssi client */
typedef struct
{
    uint16_t src_addr;
    int      rssi_sum;
    uint8_t  msg_count;
} raw_rssi_data_entry_t;

/** Object type for rssi server instances. */
typedef struct __rssi_server_t rssi_server_t;

/** Rssi server instance structure */
struct __rssi_server_t
{
    access_model_handle_t model_handle;     
};

/** Initializes the RSSI server model.
 *
 * @param[in,out] p_server Pointer to the server instance structure.
 * @param[in] element_index Element index to use when registering the RSSI server.
 *
 * @retval NRF_SUCCESS The model was successfully initialized.
 * @retval NRF_ERROR_NULL NULL pointer in function arguments
 *
 * @see access_model_add()
 */
uint32_t rssi_server_init(rssi_server_t * p_server, uint16_t element_index);

/** Adds RSSI data samples to the models databuffer.
 *
 * @param[in] test_address The address of the device this RSSI sample was recieved from.
 * @param[in] rssi The RSSI sample value.
 *
 * @retval NRF_SUCCESS The data was added successfully
 *         NRF_ERROR_NO_MEM No room for new entries in the data buffer
 *         NRF_ERROR_INVALID_PARAM Invalid address parameter
 */
uint32_t rssi_server_add_rssi_data(uint16_t test_address, int8_t rssi);

/**@} end of RSSI_SERVER */

#endif

