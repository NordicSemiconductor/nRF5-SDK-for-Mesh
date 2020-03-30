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

#ifndef SERIAL_HANDLER_COMMON_H__
#define SERIAL_HANDLER_COMMON_H__

#include <stdint.h>
#include "serial_packet.h"
#include "nrf_mesh.h"

/**
 * @defgroup MESH_SERIAL_HANDLER Serial handlers
 * @ingroup MESH_SERIAL
 * Modules for handling incoming serial commands.
 */

/**
 * @defgroup SERIAL_HANDLER_COMMON Common serial handler functions
 * @ingroup MESH_SERIAL_HANDLER
 * @{
 */

/**
 * Command processing function call back dedicated to a specific command.
 *
 * @param[in]  p_cmd  The serial command received
 */
typedef void (*serial_handler_common_cmd_cb_t)(const serial_packet_t * p_cmd);

/** Structure for mapping a command opcode to the command processing function. */
typedef struct
{
    uint8_t opcode; /**< Serial command opcode @see SERIAL_CMD */
    uint8_t payload_minlen; /**< Expected minimum payload for this serial command (can be 0). */
    uint8_t payload_optional_extra_bytes; /**< Additonal payload bytes: can also be 0. */
    serial_handler_common_cmd_cb_t callback; /**< Command processing function */
} serial_handler_common_opcode_to_fp_map_t;

/**
 * Send command response with the given data if the status is @c NRF_SUCCESS, otherwise sends NULL.
 *
 * @param[in]  opcode  The opcode of the command.
 * @param[in]  status  Status as defined by @c NRF_ERROR_H__
 * @param[in]  p_data  The pointer to the data to be sent. Can be @c NULL if @c length is 0.
 * @param[in]  length  The length of the @c p_data in bytes.
 */
void serial_handler_common_cmd_rsp_nodata_on_error(uint8_t opcode, uint32_t status, const uint8_t * p_data, uint16_t length);

/**
 * RX handler pattern for serial packets.
 *
 * @param[in]  p_cmd           The serial command received
 * @param[in]  p_cmd_handlers  A populated array of command handlers.
 * @param[in]  no_handlers     Number of handler in @c p_cmd_handlers.
 */
void serial_handler_common_rx(const serial_packet_t * p_cmd, const serial_handler_common_opcode_to_fp_map_t * p_cmd_handlers, uint32_t no_handlers);

/** @} */

#endif /* SERIAL_HANDLER_COMMON_H__ */
