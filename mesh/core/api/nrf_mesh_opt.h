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

#ifndef NRF_MESH_OPT_H_
#define NRF_MESH_OPT_H_

#include <stdint.h>
#include "toolchain.h"

/**
 * @defgroup NRF_MESH_OPT Mesh options
 * @ingroup NRF_MESH
 * Configuration options for the core mesh.
 * @{
 */
/** Start of provisioning parameters. */
#define NRF_MESH_OPT_PROV_START     200
/** Start of transport layer parameters. */
#define NRF_MESH_OPT_TRS_START      300
/** Start of network layer parameters. */
#define NRF_MESH_OPT_NET_START      400

/**
 * Option ID type.
 *
 * @note All corresponding option values are assumed unsigned 32-bit integers
 *       unless stated otherwise.
 */
typedef enum
{
    /** Enable (1) / disable (0) ECDH offloading in provisioning. */
    NRF_MESH_OPT_PROV_ECDH_OFFLOADING = NRF_MESH_OPT_PROV_START,
    /** Transport SAR RX timeout. */
    NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT = NRF_MESH_OPT_TRS_START,
    /** Base transport SAR RX ack timeout. */
    NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_BASE,
    /** SAR RX ack timeout addition per hop. */
    NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION,
    /** Base TX SAR retry timeout. */
    NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_BASE,
    /** TX retry timeout addition per hop. */
    NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION,
    /** Number of retries before cancelling a SAR session. */
    NRF_MESH_OPT_TRS_SAR_TX_RETRIES,
    /** Default TTL value for segment acknowledgement messages. */
    NRF_MESH_OPT_TRS_SAR_SEGACK_TTL,
    /** 32-bit (@ref NRF_MESH_TRANSMIC_SIZE_SMALL) or 64-bit (@ref NRF_MESH_TRANSMIC_SIZE_LARGE) MIC size for transport layer. */
    NRF_MESH_OPT_TRS_SZMIC,
    /** Packet relaying enabled (1) or disabled (0). */
    NRF_MESH_OPT_NET_RELAY_ENABLE = NRF_MESH_OPT_NET_START,
    /** Number of retransmits per relayed packet. */
    NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT,
    /** Relay retransmit interval in milliseconds. */
    NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS,
    /** Relay TX power. */
    NRF_MESH_OPT_NET_RELAY_TX_POWER,
    /** Number of retransmits per network packet originating from this device. */
    NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT,
    /** Interval between retransmitted packets originating from this device in milliseconds. */
    NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS,
    /** TX power for packets originating from this device. */
    NRF_MESH_OPT_NET_NETWORK_TX_POWER
} nrf_mesh_opt_id_t;


/**
 * Options structure.
 */
typedef struct
{
    /** Length of opt field (for future compatibility). */
    uint32_t len;
    /** Option to set/get. */
    union
    {
        /** Unsigned 32-bit value. */
        uint32_t val;
        /** Byte array. */
        uint8_t * p_array;
    } opt;
} nrf_mesh_opt_t;

/**
 * Function for setting various nRF Mesh options.
 *
 * @deprecated This function has been deprecated. Use the appropriate mesh_opt-function instead.
 *
 * @param[in] id    Identifier for option to set. See @c nrf_mesh_opt_id_t.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS Successfully set option.
 */
_DEPRECATED uint32_t nrf_mesh_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Function for getting various nRF Mesh options.
 *
 * @deprecated This function has been deprecated. Use the appropriate mesh_opt-function instead.
 *
 * @param[in]  id    Identifier for option to get. See @c nrf_mesh_opt_id_t.
 * @param[out] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS Successfully retrieved option.
 */
_DEPRECATED uint32_t nrf_mesh_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/** @} end of NRF_MESH_OPT */
#endif
