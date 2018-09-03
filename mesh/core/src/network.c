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
#include "network.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nrf_error.h>

#include "enc.h"
#include "msg_cache.h"
#include "transport.h"
#include "nrf_mesh_assert.h"
#include "net_beacon.h"
#include "net_state.h"
#include "utils.h"
#include "log.h"
#include "internal_event.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "packet_mesh.h"
#include "core_tx.h"
#include "core_tx_adv.h"
#include "core_tx_instaburst.h"
#include "heartbeat.h"
#include "nrf_mesh_config_bearer.h"
#include "mesh_opt_core.h"
#if GATT_PROXY
#include "proxy.h"
#endif
/********************
 * Static variables *
 ********************/
static nrf_mesh_relay_check_cb_t m_relay_check_cb;
/********************
 * Static functions *
 ********************/

static inline uint32_t m_core_tx_buffer_size_get(const network_packet_metadata_t * p_net_metadata, uint32_t payload_len)
{
    return PACKET_MESH_NET_PDU_OFFSET + payload_len +
           net_packet_mic_size_get(p_net_metadata->control_packet);
}

static uint32_t allocate_packet(network_tx_packet_buffer_t * p_buffer)
{
    packet_mesh_net_packet_t * p_net_packet;

    const core_tx_alloc_params_t alloc_params =
    {
        .role           = p_buffer->role,
        .net_packet_len = m_core_tx_buffer_size_get(p_buffer->user_data.p_metadata,
                                                    p_buffer->user_data.payload_len), /*lint !e446 Side effect in initializer */
        .p_metadata     = p_buffer->user_data.p_metadata,
        .token          = p_buffer->user_data.token
    };

    if (core_tx_packet_alloc(&alloc_params,
                             (uint8_t **) &p_net_packet) != 0)
    {
        if (p_buffer->role != CORE_TX_ROLE_RELAY)
        {
            p_buffer->user_data.p_metadata->internal.iv_index = net_state_tx_iv_index_get();
            uint32_t status = net_state_seqnum_alloc(&p_buffer->user_data.p_metadata->internal.sequence_number);

            if (status != NRF_SUCCESS)
            {
                /* Failed to allocate a sequence number, discard the packet. */
                core_tx_packet_discard();
                return status;
            }
        }
        net_packet_header_set(p_net_packet, p_buffer->user_data.p_metadata);
        p_buffer->p_payload = (uint8_t *) packet_mesh_net_payload_get(p_net_packet);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

/**
 * Relay the network packet, if memory is available.
 *
 * @param[in] p_net_metadata Network metadata of packet to relay.
 * @param[in,out] p_net_payload Payload of the network packet to relay.
 * @param[in] payload_len Length of the network payload.
 */
static void packet_relay(network_packet_metadata_t * p_net_metadata,
                         const uint8_t * p_net_payload,
                         uint8_t payload_len)
{
    p_net_metadata->ttl--; /* Subtract this hop */

    network_tx_packet_buffer_t buffer;
    buffer.user_data.p_metadata = p_net_metadata;
    buffer.user_data.payload_len = payload_len;
    buffer.user_data.token = NRF_MESH_RELAY_TOKEN;
    buffer.role = CORE_TX_ROLE_RELAY;

    if (allocate_packet(&buffer) == NRF_SUCCESS)
    {
        memcpy(buffer.p_payload, p_net_payload, payload_len);
        network_packet_send(&buffer);
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_RELAYED, 0, payload_len, p_net_payload);
    }
    else
    {
        __LOG(LOG_SRC_NETWORK, LOG_LEVEL_WARN, "Unable to allocate memory for relay packet.\n");
    }

    p_net_metadata->ttl++; /* Revert the change (cannot affect the allocated packet) */
}

static bool metadata_is_valid(const network_packet_metadata_t * p_net_metadata)
{
    NRF_MESH_ASSERT(p_net_metadata != NULL);

    return (p_net_metadata->dst.type != NRF_MESH_ADDRESS_TYPE_INVALID &&
            p_net_metadata->internal.sequence_number <= NETWORK_SEQNUM_MAX &&
            p_net_metadata->ttl <= NRF_MESH_TTL_MAX);
}

/**
 * Decide whether to relay based on rules in Mesh Profile Specification v1.0, section 3.4.6.3.
 *
 * @note Assumes that the message has been accepted for transport processing.
 *
 * @param[in] p_metadata Metadata to evaluate
 *
 * @returns Whether or not the packet represented by the metadata should be relayed.
 */
static bool should_relay(const network_packet_metadata_t * p_metadata)
{
    /* Relay feature must be enabled */
#if EXPERIMENTAL_INSTABURST_ENABLED
    if (!core_tx_instaburst_is_enabled(CORE_TX_ROLE_RELAY))
#else
    if (!core_tx_adv_is_enabled(CORE_TX_ROLE_RELAY))
#endif
    {
        return false;
    }
    /* TTL must be 2 or greater */
    if (p_metadata->ttl < 2)
    {
        return false;
    }
    /* Should not be directed to a unicast address on this device */
    if (p_metadata->dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        nrf_mesh_address_t dummy_addr;
        if (nrf_mesh_rx_address_get(p_metadata->dst.value, &dummy_addr))
        {
            return false;
        }
    }
    /* Relay check callback function approves of the relay */
    if (m_relay_check_cb != NULL)
    {
        if (!m_relay_check_cb(p_metadata->src, p_metadata->dst.value, p_metadata->ttl))
        {
            return false;
        }
    }
    return true;
}
/******************************
 * Public interface functions *
 ******************************/

void network_init(const nrf_mesh_init_params_t * p_init_params)
{
    if (p_init_params == NULL)
    {
        m_relay_check_cb = NULL;
    }
    else
    {
        m_relay_check_cb = p_init_params->relay_cb;
    }

    net_state_init();
    net_state_recover_from_flash();
    net_beacon_init();
}

void network_enable(void)
{
    net_state_enable();
    net_beacon_enable();
}

uint32_t network_packet_alloc(network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);
    p_buffer->role = CORE_TX_ROLE_ORIGINATOR;
    return allocate_packet(p_buffer);
}

void network_packet_send(const network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->p_payload != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);

    packet_mesh_net_packet_t * p_net_packet = net_packet_from_payload(p_buffer->p_payload);

    net_packet_encrypt(p_buffer->user_data.p_metadata,
                       p_buffer->user_data.payload_len,
                       p_net_packet,
                       NET_PACKET_KIND_TRANSPORT);

    core_tx_packet_send();

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_PACKET_QUEUED_TX, 0, p_buffer->user_data.payload_len, p_buffer->p_payload);
}

void network_packet_discard(const network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->p_payload != NULL);

    core_tx_packet_discard();
}

uint32_t network_packet_in(const uint8_t * p_packet, uint32_t net_packet_len, const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    if (p_packet == NULL)
    {
        return NRF_ERROR_NULL;
    }

    const packet_mesh_net_packet_t * p_net_packet = (const packet_mesh_net_packet_t *) p_packet;
    uint32_t status = NRF_SUCCESS;

    /* Create a target buffer to decrypt into, don't have to allocate a new packet. */
    packet_mesh_net_packet_t net_decrypted_packet;

    /* Packet fields not touched by the encryption. */
    memcpy(&net_decrypted_packet,
           p_net_packet,
           net_packet_obfuscation_start_get(p_net_packet) - (uint8_t *) p_net_packet);

    network_packet_metadata_t net_metadata;
    status = net_packet_decrypt(&net_metadata,
                                net_packet_len,
                                p_net_packet,
                                &net_decrypted_packet,
                                NET_PACKET_KIND_TRANSPORT);
    if ((status == NRF_SUCCESS) && metadata_is_valid(&net_metadata))
    {
        NRF_MESH_ASSERT(net_metadata.p_security_material != NULL);

#if GATT_PROXY
        proxy_net_packet_processed(&net_metadata, p_rx_metadata);
#endif

        const uint8_t * p_net_payload = packet_mesh_net_payload_get(&net_decrypted_packet);

        uint8_t payload_len = net_packet_payload_len_get(&net_metadata, net_packet_len);

        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_PACKET_RECEIVED, 0, net_packet_len, &net_decrypted_packet);

        status = transport_packet_in((const packet_mesh_trs_packet_t *) p_net_payload,
                                     payload_len,
                                     &net_metadata,
                                     p_rx_metadata);

        if (should_relay(&net_metadata))
        {
            packet_relay(&net_metadata, p_net_payload, payload_len);
        }
        msg_cache_entry_add(net_metadata.src, net_metadata.internal.sequence_number);
    }
    return status;
}

uint32_t network_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt)
{
    if (p_opt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            cfg.tx_interval_ms = p_opt->opt.val;
            return mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &cfg);
        }
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            cfg.enabled = (bool) p_opt->opt.val;
            return mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &cfg);
        }
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            cfg.tx_count = p_opt->opt.val;
            return mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &cfg);
        }
        case NRF_MESH_OPT_NET_RELAY_TX_POWER:
            return mesh_opt_core_tx_power_set(CORE_TX_ROLE_RELAY, (radio_tx_power_t) p_opt->opt.val);
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            cfg.tx_interval_ms = p_opt->opt.val;
            return mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &cfg);
        }
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &cfg));
            cfg.tx_count = p_opt->opt.val;
            return mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &cfg);
        }
        case NRF_MESH_OPT_NET_NETWORK_TX_POWER:
            return mesh_opt_core_tx_power_set(CORE_TX_ROLE_ORIGINATOR, (radio_tx_power_t) p_opt->opt.val);
        default:
            return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t network_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt)
{
    if (p_opt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            p_opt->opt.val = cfg.tx_interval_ms;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            p_opt->opt.val = cfg.enabled;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &cfg));
            p_opt->opt.val = cfg.tx_count;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_RELAY_TX_POWER:
        {
            radio_tx_power_t tx_power;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_tx_power_get(CORE_TX_ROLE_RELAY, &tx_power));
            p_opt->opt.val = tx_power;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &cfg));
            p_opt->opt.val = cfg.tx_interval_ms;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &cfg));
            p_opt->opt.val = cfg.tx_count;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        case NRF_MESH_OPT_NET_NETWORK_TX_POWER:
        {
            radio_tx_power_t tx_power;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_tx_power_get(CORE_TX_ROLE_ORIGINATOR, &tx_power));
            p_opt->opt.val = tx_power;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        }
        default:
            break;
    }

    return NRF_SUCCESS;
}
