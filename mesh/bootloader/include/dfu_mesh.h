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
#ifndef DFU_MESH_H__
#define DFU_MESH_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu_packet.h"
#include "dfu_types_mesh.h"
#include "mesh_packet.h"
#include "bl_if.h"

#define SD_VERSION_INVALID                  (0x0000)
#define APP_VERSION_INVALID                 (0x00000000)

void dfu_mesh_init(uint8_t tx_slots);
void dfu_mesh_start(void);
uint32_t dfu_mesh_rx(dfu_packet_t* p_packet, uint16_t length, bool from_serial);
void dfu_mesh_timeout(void);
void dfu_mesh_packet_set_local_fields(mesh_packet_t* p_packet, uint8_t dfu_packet_len);
uint32_t dfu_mesh_req(dfu_type_t type, fwid_union_t* p_fwid, uint32_t* p_bank_addr);
uint32_t dfu_mesh_relay(dfu_type_t type, fwid_union_t* p_fwid, uint32_t transaction_id);
dfu_type_t dfu_mesh_missing_type_get(void);
bool dfu_mesh_app_is_valid(void);
uint32_t dfu_mesh_finalize(void);
void dfu_mesh_restart(void);
void dfu_mesh_on_flash_idle(void);

#endif /* DFU_MESH_H__ */
