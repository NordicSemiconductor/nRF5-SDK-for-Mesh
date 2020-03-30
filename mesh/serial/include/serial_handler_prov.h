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

#ifndef SERIAL_HANDLER_PROV_H
#define SERIAL_HANDLER_PROV_H

#include <stdint.h>

#include "nrf_mesh.h"
#include "serial.h"

/**
 * @defgroup SERIAL_HANDLER_PROV Provisioning serial handler (PB-ADV)
 * @ingroup MESH_SERIAL_HANDLER
 * @{
 */

/**
 * Initializes the serial provisioning interface.
 *
 * @retval NRF_SUCCESS  The serial interface for provisioning was successfully initialized.
 * @retval NRF_ERROR_INTERNAL The ECDH module failed to generate keys.
 */
uint32_t serial_handler_prov_init(void);

/**
 * Callback function for when a serial packet has been received.
 * @param p_packet Pointer to the incoming serial packet.
 */
void serial_handler_prov_pkt_in(const serial_packet_t * p_packet);

/**
 * Gets the provisioning context with the given index.
 *
 * @param[in] index           Index of provisioning context.
 * @param[in,out] pp_prov_ctx Pointer to provisioning context pointer.
 *
 * @retval NRF_SUCCESS         Successfully returned provisioning context.
 * @retval NRF_ERROR_NOT_FOUND Invalid provisioning context index.
 */
uint32_t serial_handler_prov_context_get(uint8_t index, nrf_mesh_prov_ctx_t ** pp_prov_ctx);

/**
 * Gets the pointers to the stored public and private keys.
 *
 * @param[in,out] pp_public_key  Pointer to store the public key pointer.
 * @param[in,out] pp_private_key Pointer to store the private key pointer.
 *
 * @retval NRF_SUCCESS Successfully returned the key pointers.
 */
uint32_t serial_handler_prov_keys_get(const uint8_t ** pp_public_key, const uint8_t ** pp_private_key);

/**
 * Gets the Out-Of-Band capabilities.
 *
 * @param[in,out] pp_caps Pointer to OOB capabilities context pointer.
 *
 * @retval NRF_SUCCESS Successfully returned the OOB capabilities pointer.
 */
uint32_t serial_handler_prov_oob_caps_get(const nrf_mesh_prov_oob_caps_t ** pp_caps);

/** @} end of SERIAL_HANDLER_PROV */
#endif
