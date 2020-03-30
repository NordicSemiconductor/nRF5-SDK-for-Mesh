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

#include <string.h>
#include "nrf_mesh_defines.h"
#include "nrf.h"
#include "nrf_error.h"
#include "log.h"
#include "utils.h"

/** Structure for pasring UUID */
typedef struct __attribute((packed))
{
    uint32_t time_low;
    uint16_t time_mid;
    uint16_t time_hi_version;
    uint8_t clock_seq_hi_reserved;
    uint8_t clock_seq_low;
    uint8_t node[6];
} uuid_generic_format_t;

void mesh_app_uuid_print(const uint8_t * p_uuid)
{
    if (p_uuid == NULL)
    {
        return;
    }

    const uuid_generic_format_t * p_format = (const uuid_generic_format_t *) p_uuid;

    UNUSED_VARIABLE(p_format);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID (raw)", p_uuid, NRF_MESH_UUID_SIZE);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID : %08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
          LE2BE32(p_format->time_low), LE2BE16(p_format->time_mid), LE2BE16(p_format->time_hi_version),
          p_format->clock_seq_hi_reserved, p_format->clock_seq_low,
          p_format->node[0], p_format->node[1], p_format->node[2], p_format->node[3],
          p_format->node[4], p_format->node[5]);
}

