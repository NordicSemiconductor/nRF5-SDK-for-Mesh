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
#include <string.h>
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_serial.h"

#include "serial.h"
#include "serial_evt.h"

#include "serial_handler_app.h"
#include "serial_handler_device.h"
#include "serial_handler_dfu.h"
#include "serial_handler_mesh.h"
#include "serial_handler_openmesh.h"
#include "serial_handler_prov.h"


/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t nrf_mesh_serial_init(nrf_mesh_serial_app_rx_cb_t app_rx_cb)
{
    if (serial_state_get() != NRF_MESH_SERIAL_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t error_code = serial_handler_prov_init();
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
    error_code = serial_init();
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    /* Initialize the serial handlers that can't fail last */
    serial_handler_device_init();
    serial_handler_mesh_init();
    serial_handler_dfu_init();

    if (app_rx_cb != NULL)
    {
        serial_handler_app_cb_set(app_rx_cb);
    }
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_serial_enable(void)
{
    return serial_start();
}

nrf_mesh_serial_state_t nrf_mesh_serial_state_get(void)
{
    return serial_state_get();
}

uint32_t nrf_mesh_serial_tx(uint8_t* p_data, uint32_t length)
{
    if (serial_state_get() != NRF_MESH_SERIAL_STATE_RUNNING)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (length == 0 || length > NRF_MESH_SERIAL_PAYLOAD_MAXLEN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    serial_packet_t * p_evt;
    uint32_t status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + length, &p_evt);
    if (status == NRF_SUCCESS)
    {
        p_evt->opcode = SERIAL_OPCODE_EVT_APPLICATION;
        memcpy(p_evt->payload.evt.application.data, p_data, length);
        serial_tx(p_evt);
    }

    return status;
}

