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

#ifndef SERIAL_PACKET_H__
#define SERIAL_PACKET_H__

#include <stdint.h>
#include <limits.h>
#include "serial_cmd.h"
#include "serial_evt.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_assert.h"

/**
 * @defgroup SERIAL_PACKET Serial packet
 * @ingroup MESH_SERIAL
 * @{
 */

/** Packet length in addition to parameter. */
#define SERIAL_PACKET_LENGTH_OVERHEAD (1)
/** Overhead for each packet: length and opcode. */
#define SERIAL_PACKET_OVERHEAD (2)

/*lint -align_max(push) -align_max(1) */

/**
 * Serial packet structure.
 */
typedef struct __attribute((packed))
{
    uint8_t length; /**< Length of the packet in bytes. */
    uint8_t opcode; /**< Opcode of the packet. */

    /** Union of the various payload structures for all serial packets. */
    union __attribute((packed))
    {
        serial_cmd_t cmd; /**< Command packet parameters. */
        serial_evt_t evt; /**< Event packet parameters. */
    } payload;
} serial_packet_t;


/* Check that the length of the packet fits in a uint8_t. */
NRF_MESH_STATIC_ASSERT(sizeof(serial_packet_t) <= (UINT8_MAX + 1));

/*lint -align_max(pop) */

/** @} */

#endif
