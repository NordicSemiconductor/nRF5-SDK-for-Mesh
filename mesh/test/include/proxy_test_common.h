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
#ifndef PROXY_TEST_COMMON_H__
#define PROXY_TEST_COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "mesh_gatt.h"
#include "core_tx.h"

void init(void);
void establish_connection(uint16_t conn_index);
void disconnect(uint16_t conn_index);
void beacon_info_set(const nrf_mesh_beacon_info_t * p_beacon_info, uint32_t count);
void net_secmat_set(const nrf_mesh_network_secmat_t * p_secmat);
void gatt_evt_post(const mesh_gatt_evt_t * p_evt);
void mesh_evt_post(const nrf_mesh_evt_t * p_evt);
uint8_t * gatt_tx_packet_get(uint16_t * p_size);
bool gatt_tx_packet_is_allocated(void);
void gatt_tx_packet_availability_set(bool available);
uint32_t gatt_tx_packet_alloc_count(void);
const core_tx_bearer_interface_t * core_tx_if_get(void);
core_tx_bearer_t * core_tx_bearer_get(void);

#endif /* PROXY_TEST_COMMON_H__ */
