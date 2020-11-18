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
#include "network.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nrf_error.h>

#include "nrf_mesh_config_core.h"
#include "nrf_mesh_config_bearer.h"
#include "mesh_opt_core.h"
#include "msg_cache.h"
#include "transport.h"
#include "nrf_mesh_assert.h"
#include "net_beacon.h"
#include "net_state.h"
#include "utils.h"
#include "internal_event.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "packet_mesh.h"
#include "core_tx.h"
#include "core_tx_adv.h"
#include "core_tx_instaburst.h"
#include "heartbeat.h"
#include "enc.h"
#include "log.h"
#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "proxy.h"
#endif
#if MESH_FEATURE_FRIEND_ENABLED
#include "friend_internal.h"
#endif
#if MESH_FEATURE_LPN_ENABLED
#include "mesh_lpn.h"
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
        .role            = p_buffer->user_data.role,
        .net_packet_len  = m_core_tx_buffer_size_get(p_buffer->user_data.p_metadata,
                                                     p_buffer->user_data.payload_len), /*lint !e446 Side effect in initializer */
        .p_metadata      = p_buffer->user_data.p_metadata,
        .token           = p_buffer->user_data.token,
        .bearer_selector = p_buffer->user_data.bearer_selector,
    };

    if (core_tx_packet_alloc(&alloc_params,
                             (uint8_t **) &p_net_packet) != 0)
    {
#if MESH_FEATURE_RELAY_ENABLED || MESH_FEATURE_FRIEND_ENABLED
        if (p_buffer->user_data.role != CORE_TX_ROLE_RELAY)
#endif
        {
            uint32_t status = net_state_iv_index_and_seqnum_alloc(
                    &p_buffer->user_data.p_metadata->internal.iv_index,
                    &p_buffer->user_data.p_metadata->internal.sequence_number);

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
 * @param[in] p_rx_metadata RX metadata tied to the packet
 */
#if MESH_FEATURE_RELAY_ENABLED
static uint32_t packet_relay(network_packet_metadata_t *    p_net_metadata,
                             const uint8_t *                p_net_payload,
                             uint8_t                        payload_len,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    p_net_metadata->ttl--; /* Subtract this hop */

    network_tx_packet_buffer_t buffer;
    buffer.user_data.p_metadata = p_net_metadata;
    buffer.user_data.payload_len = payload_len;
    buffer.user_data.token = NRF_MESH_RELAY_TOKEN;
    // exclude local bearer from relay mechanism
    buffer.user_data.bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;
    buffer.user_data.role = CORE_TX_ROLE_RELAY;

#if MESH_FEATURE_GATT_PROXY_ENABLED
    if (p_rx_metadata->source != NRF_MESH_RX_SOURCE_GATT && !proxy_is_enabled())
    {
        buffer.user_data.bearer_selector ^= CORE_TX_BEARER_TYPE_GATT_SERVER;
    }
#else
    UNUSED_PARAMETER(p_rx_metadata);
#endif

    uint32_t status = allocate_packet(&buffer);

    if (status == NRF_SUCCESS)
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
    return status;
}
#endif /* MESH_FEATURE_RELAY_ENABLED */

static bool metadata_is_valid(const network_packet_metadata_t * p_net_metadata)
{
    NRF_MESH_ASSERT(p_net_metadata != NULL);

    return (p_net_metadata->dst.type != NRF_MESH_ADDRESS_TYPE_INVALID &&
            p_net_metadata->internal.sequence_number <= NETWORK_SEQNUM_MAX &&
            !(p_net_metadata->control_packet && p_net_metadata->dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL) &&
            p_net_metadata->ttl <= NRF_MESH_TTL_MAX);
}

/**
 * Decide whether to relay based on rules in @tagMeshSp section 3.4.6.3.
 *
 * @note Assumes that the message has been accepted for transport processing.
 *
 * @param[in] p_metadata Metadata to evaluate
 * @param[in] p_rx_metadata RX metadata tied to the packet
 *
 * @returns Whether or not the packet represented by the metadata should be relayed.
 */
#if MESH_FEATURE_RELAY_ENABLED
static bool should_relay(const network_packet_metadata_t * p_metadata,
                         const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    /* If the source address is one of our unicast rx addresses, we sent it ourselves, and shouldn't
     * process it to prevent relay loop */
    nrf_mesh_address_t dummy_addr;
    if (nrf_mesh_rx_address_get(p_metadata->src, &dummy_addr))
    {
        return false;
    }

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
        if (nrf_mesh_rx_address_get(p_metadata->dst.value, &dummy_addr))
        {
            return false;
        }
    }

#if MESH_FEATURE_GATT_PROXY_ENABLED
    if (p_rx_metadata->source == NRF_MESH_RX_SOURCE_GATT && !proxy_is_enabled())
    {
        return false;
    }
#endif

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
#endif /* MESH_FEATURE_RELAY_ENABLED */

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
    net_beacon_init();
}

void network_enable(void)
{
    net_state_enable();
#if !MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
    net_beacon_enable();
#endif
}

uint32_t network_packet_alloc(network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.role < CORE_TX_ROLE_COUNT);
    NRF_MESH_ASSERT(p_buffer->user_data.bearer_selector != CORE_TX_BEARER_TYPE_INVALID);
    return allocate_packet(p_buffer);
}

void network_packet_send(const network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->p_payload != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);

    packet_mesh_net_packet_t * p_net_packet = net_packet_from_payload(p_buffer->p_payload);
    __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_DBG1, "Net TX", (const uint8_t *) p_net_packet, sizeof(packet_mesh_net_packet_t));
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

    __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_DBG1, "  Net RX (enc)", p_packet, net_packet_len);
    network_packet_metadata_t net_metadata;
    status = net_packet_decrypt(&net_metadata,
                                net_packet_len,
                                p_net_packet,
                                &net_decrypted_packet,
                                NET_PACKET_KIND_TRANSPORT);
    if ((status == NRF_SUCCESS) && metadata_is_valid(&net_metadata))
    {
        __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_DBG1, "Net RX (unenc)", &net_decrypted_packet.pdu[0], net_packet_len);
        NRF_MESH_ASSERT(net_metadata.p_security_material != NULL);

#if MESH_FEATURE_GATT_PROXY_ENABLED
        proxy_net_packet_processed(&net_metadata, p_rx_metadata);
#endif

        const uint8_t * p_net_payload = packet_mesh_net_payload_get(&net_decrypted_packet);

        uint8_t payload_len = net_packet_payload_len_get(&net_metadata, net_packet_len);

        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_PACKET_RECEIVED, 0, net_packet_len, &net_decrypted_packet);

        status = transport_packet_in((const packet_mesh_trs_packet_t *) p_net_payload,
                                     payload_len,
                                     &net_metadata,
                                     p_rx_metadata);

#if MESH_FEATURE_RELAY_ENABLED
#if MESH_FEATURE_FRIEND_ENABLED
        /* Perform security material translation irrespective whether the received packet was on the
         * friendship security credentials or regular credentials
         */
        if (friend_friendship_established(net_metadata.src))
        {
            nrf_mesh_network_secmat_t * p_tx_secmat = nrf_mesh_net_master_secmat_get(net_metadata.p_security_material);
            if (p_tx_secmat != NULL)
            {
                net_metadata.p_security_material = p_tx_secmat;
            }

        }
#endif
        if (should_relay(&net_metadata, p_rx_metadata)
#if MESH_FEATURE_LPN_ENABLED
            && !mesh_lpn_is_in_friendship()
#endif
            )
        {
            if (packet_relay(&net_metadata, p_net_payload, payload_len, p_rx_metadata) == NRF_SUCCESS)
            {
                msg_cache_entry_add(net_metadata.src, net_metadata.internal.sequence_number);
            }
        }
        else
#endif
        {
            msg_cache_entry_add(net_metadata.src, net_metadata.internal.sequence_number);
        }

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
#if MESH_FEATURE_RELAY_ENABLED
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
#else
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS:
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
        case NRF_MESH_OPT_NET_RELAY_TX_POWER:
            return NRF_ERROR_NOT_FOUND;
#endif
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS:
        {
            mesh_opt_core_adv_t cfg;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR, &cfg));
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
#if MESH_FEATURE_RELAY_ENABLED
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
#else
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS:
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
        case NRF_MESH_OPT_NET_RELAY_TX_POWER:
            return NRF_ERROR_NOT_FOUND;
#endif
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
