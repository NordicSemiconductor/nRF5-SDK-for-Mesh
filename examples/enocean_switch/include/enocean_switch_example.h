/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef ENOCEAN_SWITCH_EXAMPLE_H__
#define ENOCEAN_SWITCH_EXAMPLE_H__

#include "ble_gap.h"
#include "enocean.h"

/**
 * @defgroup ENOCEAN_SWITCH_EAMPLE_H Enocean switch example header
 * @{
 */

/**
 * @defgroup ENOCEAN_SWITCH_EXAMPLE_DEFS Example specific defines
 * @{
 */

#define APP_FLASH_PAGE_COUNT           (1)
#define APP_DATA_ENTRY_HANDLE          (0x00001)

/** Static authentication data */
#define STATIC_AUTH_DATA {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

/** client UUID */
#define CLIENT_NODE_UUID {0x00, 0x59, 0xAB, 0xCD, 0xEF, 0xAB, 0xCD, 0xEF, 0xAC, 0xCD, 0xEF, 0xAB, 0xCD, 0xEF, 0xAB, 0xCD}

/** @} end of ENOCEAN_SWITCH_EXAMPLE_DEFS */


/**
 * @defgroup ENOCEAN_SWITCH_EXAMPLE_TYPE_DEFS Example specific structures
 * @{
 */
typedef struct
{
    uint8_t ble_gap_addr[BLE_GAP_ADDR_LEN];
    uint8_t key[PTM215B_COMM_PACKET_KEY_SIZE];
    uint32_t seq;
} app_secmat_flash_t;

/** @} end of ENOCEAN_SWITCH_EXAMPLE_TYPE_DEFS */



/** @} end of ENOCEAN_SWITCH_EXAMPLE_H */

#endif /* ENOCEAN_SWITCH_EXAMPLE_H__ */
