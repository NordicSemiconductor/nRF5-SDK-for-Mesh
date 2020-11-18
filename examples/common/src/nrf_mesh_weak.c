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
#include <stdint.h>
#include "nrf_error.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_dfu.h"
#include "nrf_mesh_opt.h"
#include "packet.h"
#include "internal_event.h"

#include "nrf_mesh_prov.h"

#include "provisioning.h"

/*lint -e762 Redundantly declared symbol; these are all declared elsewhere. */

uint32_t _weak_handler()
{
    return NRF_ERROR_NOT_SUPPORTED;
}

void * _weak_handler_ptr()
{
    return NULL;
}

/** PROVISIONING  */

/** prov_provisioner.c  */

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisioner_provision(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_uuid,
                                                                                   uint8_t attention_duration_s, const nrf_mesh_prov_provisioning_data_t * p_data);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisioner_oob_use(nrf_mesh_prov_ctx_t * p_ctx, nrf_mesh_prov_oob_method_t method, uint8_t action, uint8_t size);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisioner_auth_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisioner_shared_secret(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisioner_oob_pubkey(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key);

/** prov_provisionee.c  */
uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisionee_init(nrf_mesh_prov_ctx_t * p_ctx, const char * URI, uint16_t oob_info_sources);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisionee_listen(nrf_mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisionee_auth_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size);

uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_provisionee_shared_secret(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);


/** prov_beacon.c */
void __attribute__((weak, alias ("_weak_handler"))) prov_beacon_unprov_packet_in(const uint8_t * p_data, uint8_t length, const nrf_mesh_rx_metadata_t * p_meta);

/** prov_bearer_adv.c  */
void __attribute__((weak, alias ("_weak_handler"))) prov_bearer_adv_packet_in(const uint8_t * p_data, uint8_t length, const nrf_mesh_rx_metadata_t * p_meta);

/** prov_utils.c */
uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_utils_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt);
uint32_t __attribute__((weak, alias ("_weak_handler"))) prov_utils_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt);

/** DFU  */
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_init(void);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_enable(void);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_abort(void);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_bank_flash(nrf_mesh_dfu_type_t bank_type);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_bank_info_get(nrf_mesh_dfu_type_t type, nrf_mesh_dfu_bank_info_t* p_bank_info);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_jump_to_bootloader(void);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_relay(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_request(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid,
        const uint32_t* p_bank_addr);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_rx(const uint8_t * p_packet, uint32_t length, const nrf_mesh_rx_metadata_t * p_metadata);
uint32_t __attribute__((weak, alias ("_weak_handler"))) nrf_mesh_dfu_state_get(nrf_mesh_dfu_transfer_state_t* p_dfu_transfer_state);
void __attribute__((weak, alias ("_weak_handler"))) mesh_flash_op_execute(timestamp_t available_time);
void __attribute__((weak, alias ("_weak_handler"))) mesh_flash_init(void);
