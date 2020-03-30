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
#ifndef PROXY_CONFIG_PACKET_H__
#define PROXY_CONFIG_PACKET_H__

#include <stdint.h>
#include "nrf_mesh_config_core.h"

#define PROXY_CONFIG_PARAM_OVERHEAD (offsetof(proxy_config_msg_t, params))

typedef enum
{
    PROXY_CONFIG_OPCODE_FILTER_TYPE_SET = 0x00,
    PROXY_CONFIG_OPCODE_FILTER_ADDR_ADD = 0x01,
    PROXY_CONFIG_OPCODE_FILTER_ADDR_REMOVE = 0x02,
    PROXY_CONFIG_OPCODE_FILTER_STATUS = 0x03,
} proxy_config_opcode_t;


typedef struct __attribute((packed))
{
    uint8_t filter_type;
} proxy_config_params_filter_type_set_t;

typedef struct __attribute((packed))
{
    uint16_t addrs[MESH_GATT_PROXY_FILTER_ADDR_COUNT];
} proxy_config_params_filter_addr_add_t;

typedef struct __attribute((packed))
{
    uint16_t addrs[MESH_GATT_PROXY_FILTER_ADDR_COUNT];
} proxy_config_params_filter_addr_remove_t;

typedef struct __attribute((packed))
{
    uint8_t filter_type;
    uint16_t list_size;
} proxy_config_params_filter_status_t;

typedef struct __attribute((packed))
{
    uint8_t opcode;
    union __attribute((packed))
    {
        proxy_config_params_filter_type_set_t filter_type_set;
        proxy_config_params_filter_addr_add_t filter_addr_add;
        proxy_config_params_filter_addr_remove_t filter_addr_remove;
        proxy_config_params_filter_status_t filter_status;
    } params;
} proxy_config_msg_t;

#endif /* PROXY_CONFIG_PACKET_H__ */
