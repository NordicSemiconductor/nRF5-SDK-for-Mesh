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

#ifndef NRF_MESH_PROV_BEARER_ADV_H__
#define NRF_MESH_PROV_BEARER_ADV_H__

#include "prov_bearer_adv.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_prov_bearer.h"
#include "advertiser.h"
#include "timer_scheduler.h"

/**
 * @defgroup NRF_MESH_PROV_BEARER_ADV Provisioning over Advertising Bearer (PB-ADV)
 * @ingroup NRF_MESH_PROV_BEARER
 * This module provides support for the provisioning over advertising (PB-ADV)
 * provisioning bearer. PB-ADV is used for provisioning devices within radio
 * range of the provisioner.
 * @{
 */

/**
 * PB-ADV context structure.
 * All fields are set and maintained internally. The user should simply allocate the structure such
 * that it is retained for the duration of the provisioning procedure.
 */
typedef struct prov_bearer_adv
{
    advertiser_t               advertiser;          /**< Advertiser instance. */
    uint32_t                   link_id;             /**< PB-ADV link identifier. */
    prov_bearer_adv_instance_t instance_state;      /**< State identifier for the struct instance. */
    prov_bearer_adv_state_t    state;               /**< State identifier for the PB-ADV state machine. */
    uint8_t                    transaction_in;      /**< Transaction number for the incoming messages. */
    uint8_t                    transaction_out;     /**< Transaction number for the outgoing messages. */
    nrf_mesh_prov_link_close_reason_t close_reason; /**< Reason for closing the link. */
    timer_event_t              timeout_event;       /**< State structure for the timeout timer. */
    timer_event_t              link_timeout_event;  /**< State structure for the link timeout timer. */
    uint32_t                   sar_timeout;         /**< SAR timeout value. */
    uint32_t                   link_timeout;        /**< Link timeout value. */
    prov_bearer_adv_buffer_t   buffer;              /**< Buffer structure for ongoing transfer. */
    uint8_t                    tx_buffer[NRF_MESH_PROV_BEARER_ADV_TX_BUFFER_SIZE] __attribute__((aligned(WORD_SIZE))); /**< Buffer for outgoing packets. */
    nrf_mesh_tx_token_t        last_token;          /**< Token of the last packet sent to the advertiser. */
    bool                       queue_empty_pending; /**< Flag indicating whether a queue empty event is pending. */
    prov_bearer_t              prov_bearer;
    struct prov_bearer_adv *   p_next;              /**< Pointer to the next active PB-ADV link. */
} nrf_mesh_prov_bearer_adv_t;

/**
 * Gets the provisioning bearer interface for the PB-ADV bearer.
 *
 * @note This function is to be used in conjunction with nrf_mesh_prov_bearer_add().
 *
 * @param[in,out] p_bearer_adv Pointer to a PB-ADV bearer context structure.
 *
 * @returns A pointer to the bearer interface for the PB-ADV bearer.
 */
prov_bearer_t * nrf_mesh_prov_bearer_adv_interface_get(nrf_mesh_prov_bearer_adv_t * p_bearer_adv);

/** @} end of NRF_MESH_PROV_BEARER_ADV */
#endif /* NRF_MESH_PROV_BEARER_ADV_H__ */
