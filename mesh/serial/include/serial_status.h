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

#ifndef SERIAL_STATUS_H__
#define SERIAL_STATUS_H__

/**
 * @defgroup SERIAL_STATUS_CODES Serial status codes
 * @ingroup MESH_SERIAL
 * This file contains serial status codes. They match the status codes defined
 * in the ACI interface for the nRF8001 chip.
 * @{
 */

#define SERIAL_STATUS_SUCCESS                   0x00
#define SERIAL_STATUS_ERROR_UNKNOWN             0x80
#define SERIAL_STATUS_ERROR_INTERNAL            0x81
#define SERIAL_STATUS_ERROR_CMD_UNKNOWN         0x82
#define SERIAL_STATUS_ERROR_INVALID_STATE       0x83
#define SERIAL_STATUS_ERROR_INVALID_LENGTH      0x84
#define SERIAL_STATUS_ERROR_INVALID_PARAMETER   0x85
#define SERIAL_STATUS_ERROR_BUSY                0x86
#define SERIAL_STATUS_ERROR_INVALID_DATA        0x87
#define SERIAL_STATUS_ERROR_REJECTED            0x8e
#define SERIAL_STATUS_ERROR_TIMEOUT             0x93
#define SERIAL_STATUS_ERROR_INVALID_KEY_DATA    0x98

/** @} end of SERIAL_STATUS_CODES */

#endif
