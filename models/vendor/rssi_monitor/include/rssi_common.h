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

#ifndef RSSI_COMMON_H__
#define RSSI_COMMON_H__

/**
 * @defgroup RSSI_COMMON RSSI monitor common 
 * @ingroup MESH_API_GROUP_VENDOR_MODELS
 * @{
 */

/** Model ID for the RSSI Server model. */
#define RSSI_SERVER_MODEL_ID 0x0005
/** Model ID for the RSSI Client model. */
#define RSSI_CLIENT_MODEL_ID 0x0006

#ifndef RSSI_DATA_BUFFER_SIZE
#define RSSI_DATA_BUFFER_SIZE 20 /**< This number should reflect the maximum number of nodes that possibly are in direct radio contact with the device */
#endif

/** Packet format of the RSSI data entry sent from the server to the client*/
typedef struct __attribute((packed))
{
    uint16_t src_addr;
    int8_t   mean_rssi;
    uint8_t  msg_count;
} rssi_data_entry_t;

/** RSSI model common opcode */
typedef enum
{
    RSSI_OPCODE_RSSI_ACK = 0xC5,
    RSSI_OPCODE_SEND_RSSI_DATA = 0xC6,
} rssi_common_opcode_t;

/**@} end of RSSI_COMMON */
#endif
