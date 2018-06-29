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

#ifndef ACCESS_LOOPBACK_H_
#define ACCESS_LOOPBACK_H_

#include <stdint.h>
#include <stdbool.h>

#include "access.h"
#include "device_state_manager.h"
#include "nrf_mesh.h"

/**
 * @internal
 * @defgroup ACCESS_LOOPBACK Access Layer internal definitions
 * @ingroup MESH_API_LOOPBACK_ACCESS
 * Provides structures and functions for internal management of the access loopback.
 * @{
 */

typedef struct
{
    /** Identifier of the outgoing message. */
    nrf_mesh_tx_token_t     token;
    /** Model opcode of the outgoing message. */
    access_opcode_t         opcode;
    /** Pointer to the message payload. */
    const uint8_t *         p_data;
    /** Length to the message payload. */
    uint16_t                length;
    /** Source address. */
    uint16_t                src_value;
    /** Destination address of the message. */
    nrf_mesh_address_t      dst;
    /** TTL value. */
    uint8_t                 ttl;
    /** Application key handle. */
    dsm_handle_t            appkey_handle;
    /** Network key handle. */
    dsm_handle_t            subnet_handle;
} access_loopback_request_t;

/**
 * Checks whether the outgoing message is suitable for the access internal loop.
 *
 * @param[in] p_addr Publish address
 *
 * @retval true  The outgoing message is suitable for the access internal loop.
 * @retval false Otherwise.
 */
bool is_access_loopback(const nrf_mesh_address_t * p_addr);

/**
 * Initialize the access internal loop functionality.
 */
void access_loopback_init(void);

/**
 * Proceeds handling of the outgoing message.
 *
 * @param[in]  p_req          Pointer to the message parameters
 *
 * @retval NRF_SUCCESS      Message is proceeded successfully.
 * @retval NRF_ERROR_NO_MEM There is no memory for the message proceeding.
 */
uint32_t access_loopback_handle(access_loopback_request_t * p_req);

/** @} */
#endif /* ACCESS_LOOPBACK_H_ */
