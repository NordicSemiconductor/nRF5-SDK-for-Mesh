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

#ifndef RSSI_CLIENT_H__
#define RSSI_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>
#include "access.h"
#include "rssi_common.h"

/**
 * @defgroup RSSI_CLIENT RSSI monitor client model interface 
 * @ingroup RSSI_COMMON
 * @{
 */

/** Object type for rssi client instances */
typedef struct rssi_client_t rssi_client_t;

/** Event callback function type */ 
typedef void (*rssi_evt_cb_t)(const access_message_rx_t* p_message);

/** Rssi client instance structure */
struct rssi_client_t
{
    access_model_handle_t  model_handle;
    rssi_evt_cb_t rssi_handler;
};

/** Initializes a rssi client instance.
 *
 * @param[in,out] p_client      Pointer to the client instance structure.
 * @param[in]     element_index Index of the element to register the model with.
 * @param[in]     rssi_handler  Handler used to process incoming messages.
 *
 * @retval NRF_SUCCESS      The rssi client was successfully initialized.
 * @retval NRF_ERROR_NULL   Passed rssi_handler not valid
 * 
 * @see access_model_add() for other return values
 */
uint32_t rssi_client_init(rssi_client_t * p_client, uint16_t element_index, rssi_evt_cb_t rssi_handler);

/**@} end of RSSI_CLIENT */
#endif

