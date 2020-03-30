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

#ifndef SERIAL_H__
#define SERIAL_H__

#include <stdint.h>

#include "nrf_mesh.h"
#include "nrf_mesh_serial.h"
#include "serial_packet.h"

/**
 * @defgroup MESH_SERIAL Serial components
 * Internal components of the Serial subsystem.
 * @ingroup MESH_API_GROUP_SERIAL
 */

/**
 * @defgroup SERIAL_INTERFACE Serial interface
 * @ingroup MESH_SERIAL
 * @{
 */

/** Serial API version. */
#define SERIAL_API_VERSION 10

/**
 * Initializes the serial interface abstraction layer.
 * @retval NRF_SUCCESS   The serial interface was successfully initialized.
 * @retval NRF_ERROR_INVALID_STATE The serial module has already been
 * initialized.
 */
uint32_t serial_init(void);

/**
 * Starts the serial interface by sending a device started event to the host.
 *
 * @retval NRF_SUCCESS The serial interface was successfully started.
 * @retval NRF_ERROR_INVALID_STATE The serial interface has not been
 * initialized, or has been started before.
 * @retval NRF_ERROR_NO_MEM The packet queue is full.
 */
uint32_t serial_start(void);

/**
 * Returns a serial packet buffer of the requested length.
 *
 * @param[in]  packet_len                The serial packet length (excludes the length field of the
 *                                       packet).
 * @param[out] pp_packet                 The serial packet instance pointer, returns the allocated
 *                                       buffer.
 *
 * @retval     NRF_SUCCESS               The packet is reserved successfully, and pp_packet points
 *                                       to a valid packet pointer.
 * @retval     NRF_ERROR_INVALID_STATE   The serial module has not been started.
 * @retval     NRF_ERROR_INVALID_LENGTH  The length of the packet requested cannot be 0 or greater
 *                                       than the maximum available packet size
 *
 */
uint32_t serial_packet_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet);

/**
 * Queues a packet for transmission on the serial interface.
 *
 * @param[in] p_packet Pointer to the packet to transmit.
 */
void serial_tx(const serial_packet_t * p_packet);

/**
 * Schedule processing for serial RX/TX.
 */
void serial_process(void);

/**
 * Translates an NRF_* error code into an ACI error code.
 * @param status the status code to translate.
 * @return       The ACI error code corresponding to the specified NRF error code.
 */
uint8_t serial_translate_error(uint32_t status);

/**
 * Get current state of the serial module.
 *
 * @return Current state of the serial module.
 */
nrf_mesh_serial_state_t serial_state_get(void);

/**
 * Send a simple command response to the given opcode, with the given serial status.
 *
 * @param[in] opcode Opcode to reply to.
 * @param[in] status Reply status code.
 * @param[in] p_data Additional data to be added to the reply (optional).
 * @param[in] length Length of additional data.
 */
void serial_cmd_rsp_send(uint8_t opcode, uint8_t status, const uint8_t * p_data, uint16_t length);

/** @} end of SERIAL_INTERFACE */

#endif
