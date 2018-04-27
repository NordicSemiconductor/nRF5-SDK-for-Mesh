/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#define PRIVACY_RANDOM_SIZE 7
#define PECB_SIZE           6

/** Offset of the start of the encrypted part of the network packet. */
#define NET_PACKET_ENCRYPTION_START_OFFSET PACKET_MESH_NET_DST0_OFFSET
/** Offset of the start of the obfuscated part of the network packet. */
#define NET_PACKET_OBFUSCATION_START_OFFSET PACKET_MESH_NET_CTL_OFFSET
/** Number of bytes before the payload that are being encrypted. */
#define NET_PACKET_ENCRYPTION_START_PAYLOAD_OVERHEAD (PACKET_MESH_NET_PDU_OFFSET - NET_PACKET_ENCRYPTION_START_OFFSET)
/******************
 * Local Typedefs *
 ******************/

/*lint -align_max(push) -align_max(1) */

/**
 * PECB data according to the Mesh Profile Specification v1.0 section 3.8.7.3
 */
typedef struct __attribute((packed))
{
    uint8_t          zero_padding[5];                     /**< Zero padding. */
    uint32_t         iv_index_be;                         /**< Current IV index (big endian). */
    uint8_t          privacy_random[PRIVACY_RANDOM_SIZE]; /**< The 7 LSBs of the privacy random value. */
} pecb_data_t;
/*lint -align_max(pop) */

/********************
 * Static variables *
 ********************/
static bool m_relay_enable;
static nrf_mesh_relay_check_cb_t m_relay_check_cb;
/********************
 * Static functions *
 ********************/
static inline uint32_t net_mic_size_get(bool is_control_packet)
{
    /* According to Mesh Profile Specification v1.0, section 3.4.4.3, control messages have an 8 byte mic, access
     * messages have a 4 byte mic. */
    return (is_control_packet ? 8 : 4);
}

static inline uint32_t m_core_tx_buffer_size_get(const network_packet_metadata_t * p_net_metadata, uint32_t payload_len)
{
    return PACKET_MESH_NET_PDU_OFFSET + payload_len +
           net_mic_size_get(p_net_metadata->control_packet);
}

static inline uint32_t payload_len_get(const network_packet_metadata_t * p_net_metadata,
                                       uint32_t net_packet_len)
{
    return net_packet_len - PACKET_MESH_NET_PDU_OFFSET -
           net_mic_size_get(p_net_metadata->control_packet);
}

static inline packet_mesh_net_packet_t * net_packet_from_payload(uint8_t * p_net_payload)
{
    return (packet_mesh_net_packet_t *) (p_net_payload - PACKET_MESH_NET_PDU_OFFSET);
}

static inline uint8_t * net_packet_enc_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_ENCRYPTION_START_OFFSET;
}

static inline uint8_t * net_packet_obfuscation_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_OBFUSCATION_START_OFFSET;
}

static void set_net_packet_header(packet_mesh_net_packet_t * p_net_packet,
                                  const network_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(nrf_mesh_address_type_get(p_metadata->src) == NRF_MESH_ADDRESS_TYPE_UNICAST);
    NRF_MESH_ASSERT(p_metadata->ttl <= NRF_MESH_TTL_MAX);
    NRF_MESH_ASSERT(p_metadata->internal.sequence_number <= NETWORK_SEQNUM_MAX);
    NRF_MESH_ASSERT(p_metadata->dst.type != NRF_MESH_ADDRESS_TYPE_INVALID);
    NRF_MESH_ASSERT(p_metadata->p_security_material != NULL);

    /* Header is in big endian */
    packet_mesh_net_ivi_set(p_net_packet, p_metadata->internal.iv_index & NETWORK_IVI_MASK);
    packet_mesh_net_nid_set(p_net_packet, p_metadata->p_security_material->nid);
    packet_mesh_net_ctl_set(p_net_packet, p_metadata->control_packet);
    packet_mesh_net_ttl_set(p_net_packet, p_metadata->ttl);
    packet_mesh_net_seq_set(p_net_packet, p_metadata->internal.sequence_number);
    packet_mesh_net_src_set(p_net_packet, p_metadata->src);
    packet_mesh_net_dst_set(p_net_packet, p_metadata->dst.value);
}

/**
 * Put all fields that are covered by obfuscation into the metadata.
 *
 * @param[in,out] p_net_metadata Metadata to fill
 * @param[in] p_net_deobfuscated_packet Deobfuscated packet to pick values from.
 */
static void deobfuscated_header_fields_get(network_packet_metadata_t * p_net_metadata,
                                           const packet_mesh_net_packet_t * p_net_deobfuscated_packet)
{
    p_net_metadata->control_packet           = packet_mesh_net_ctl_get(p_net_deobfuscated_packet);
    p_net_metadata->ttl                      = packet_mesh_net_ttl_get(p_net_deobfuscated_packet);
    p_net_metadata->internal.sequence_number = packet_mesh_net_seq_get(p_net_deobfuscated_packet);
    p_net_metadata->src                      = packet_mesh_net_src_get(p_net_deobfuscated_packet);
}

static inline core_tx_bearer_t get_bearer(const network_packet_metadata_t * p_net_metadata)
{
    /* There's only one bearer available so far. */
    return CORE_TX_BEARER_ADV;
}

static uint32_t allocate_packet(network_tx_packet_buffer_t * p_buffer)
{
    packet_mesh_net_packet_t * p_net_packet = NULL;
    if (core_tx_packet_alloc(m_core_tx_buffer_size_get(p_buffer->user_data.p_metadata, p_buffer->user_data.payload_len),
                             &p_buffer->core_tx,
                             (uint8_t **) &p_net_packet,
                             p_buffer->user_data.token) != 0)
    {
        if (p_buffer->core_tx.role != CORE_TX_ROLE_RELAY)
        {
            /* Populate internal metadata */
            net_state_iv_index_lock(true);
            p_buffer->user_data.p_metadata->internal.iv_index = net_state_tx_iv_index_get();
            uint32_t status = net_state_seqnum_alloc(&p_buffer->user_data.p_metadata->internal.sequence_number);
            net_state_iv_index_lock(false);

            if (status != NRF_SUCCESS)
            {
                /* Failed to allocate a sequence number, discard the packet. */
                core_tx_packet_discard(&p_buffer->core_tx, (uint8_t *) p_net_packet);
                return status;
            }
        }
        set_net_packet_header(p_net_packet, p_buffer->user_data.p_metadata);
        p_buffer->p_payload = (uint8_t *) packet_mesh_net_payload_get(p_net_packet);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

/** Redefinition of header_transfuscate() for clarity. */
#define header_obfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)    \
    header_transfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)
/** Redefinition of header_transfuscate() for clarity. */
#define header_deobfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)  \
    header_transfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)

/**
 * (De-)obfuscates a network header.
 *
 * The whole network header, except NID+IVI and DST fields, is obfuscated.
 *
 * @param[in]      p_net_metadata   Network metadata structure.
 * @param[in]      p_net_packet_in  Network packet pointer.
 * @param[out]     p_net_packet_out Network packet pointer. May be the
 *                                  same as @c p_net_packet_in.
 */
static void header_transfuscate(const network_packet_metadata_t * p_net_metadata,
                                const packet_mesh_net_packet_t * p_net_packet_in,
                                packet_mesh_net_packet_t * p_net_packet_out)
{
    uint8_t pecb[NRF_MESH_KEY_SIZE];

    /* Calculate the PECB: */
    pecb_data_t pecb_data;
    memset(&pecb_data.zero_padding[0], 0, sizeof(pecb_data.zero_padding));
    pecb_data.iv_index_be = LE2BE32(p_net_metadata->internal.iv_index);
    memcpy(pecb_data.privacy_random, net_packet_enc_start_get(p_net_packet_in), PRIVACY_RANDOM_SIZE);
    enc_aes_encrypt(p_net_metadata->p_security_material->privacy_key, (const uint8_t *) &pecb_data, pecb);

    utils_xor(net_packet_obfuscation_start_get(p_net_packet_out),
              net_packet_obfuscation_start_get(p_net_packet_in),
              pecb,
              NET_PACKET_ENCRYPTION_START_OFFSET - NET_PACKET_OBFUSCATION_START_OFFSET);
}

/**
 * Encrypts a network packet.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 * | ... ---------|------------- AES-CCM encrypted -------------|----------|
 * | ... | SRC:16 | DST:16 | Trans. payload [0-12] | Trans. MIC | Net. MIC |
 * -------------------------------------------------------------------------
 * @endverbatim
 *
 * @param[in,out] p_buffer Buffer to encrypt and obfuscate.
 */
static void network_pkt_encrypt(const network_tx_packet_buffer_t * p_buffer)
{
    uint8_t nonce[CCM_NONCE_LENGTH];

    enc_nonce_generate(p_buffer->user_data.p_metadata, ENC_NONCE_NET, 0, nonce);

    packet_mesh_net_packet_t * p_net_packet = net_packet_from_payload(p_buffer->p_payload);
    ccm_soft_data_t ccm_params;
    ccm_params.mic_len = net_mic_size_get(p_buffer->user_data.p_metadata->control_packet);
    ccm_params.p_key   = p_buffer->user_data.p_metadata->p_security_material->encryption_key;
    ccm_params.p_nonce = nonce;
    /* Include destination field in the encrypted payload. */
    ccm_params.p_m     = net_packet_enc_start_get(p_net_packet);
    ccm_params.p_out   = net_packet_enc_start_get(p_net_packet);
    ccm_params.m_len   = (NET_PACKET_ENCRYPTION_START_PAYLOAD_OVERHEAD + p_buffer->user_data.payload_len);
    ccm_params.a_len   = 0;
    ccm_params.p_a     = NULL;
    ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;

    enc_aes_ccm_encrypt(&ccm_params);

    header_obfuscate(p_buffer->user_data.p_metadata, p_net_packet, p_net_packet);
}

/**
 * Check whether the fields in the metadata that come from the obfuscated part of the packet can
 * represent a valid header.
 *
 * @param[in] p_net_metadata Metadata to check for.
 * @param[in] net_packet_len Length of the network packet.
 * @param[in] p_net_packet Pointer to the network packet, used for logging.
 *
 * @returns Whether the metadata represents a potentially valid header.
 */
static inline bool deobfuscated_header_is_valid(const network_packet_metadata_t * p_net_metadata,
                                                uint32_t net_packet_len,
                                                const packet_mesh_net_packet_t * p_net_packet)
{
    /* If the source address isn't a unicast address, this packet won't be valid, and we can skip it
     * without decrypting, saving us an average of 50% of all failing decryptions.  */
    if (nrf_mesh_address_type_get(p_net_metadata->src) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return false;
    }
    /* If the payload is too short to hold the mic, we'll skip it. */
    if (net_packet_len - PACKET_MESH_NET_PDU_OFFSET < net_mic_size_get(p_net_metadata->control_packet))
    {
        return false;
    }

    /* We check the message cache now, as we'll either have the right deobfuscation, and the
     * src+seq won't change after decryption, or we'll have the wrong deobfuscation, and most likely
     * pass the cache check, but abandon it after decryption. In the unlikely event of a wrongly
     * deobfuscated src+seq matching an existing src+seq in the message cache, we'll wrongly abandon
     * the packet here, but since the decryption would have failed anyway, it doesn't matter. */
    if (msg_cache_entry_exists(p_net_metadata->src, p_net_metadata->internal.sequence_number))
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED, PACKET_DROPPED_NETWORK_CACHE, net_packet_len, p_net_packet);
        return false;
    }
    /* If the source address is one of our unicast rx addresses, we sent it ourselves, and shouldn't
     * process it: */
    nrf_mesh_address_t dummy;
    if (nrf_mesh_rx_address_get(p_net_metadata->src, &dummy))
    {
        return false;
    }

    return true;
}

/**
 * Decrypts a network packet.
 *
 * @param[in,out] p_net_metadata Metadata structure to fill during decryption.
 * @param[in] net_packet_len Length of the entire network packet.
 * @param[in] p_net_encrypted_packet Encrypted network packet.
 * @param[out] p_net_decrypted_packet Pointer to buffer in which the encrypted
 *                                    packet is decrypted into.
 *
 * @returns Whether the decryption was successful and authenticated.
 */
static bool network_pkt_decrypt(network_packet_metadata_t * p_net_metadata,
                                uint32_t net_packet_len,
                                const packet_mesh_net_packet_t * p_net_encrypted_packet,
                                packet_mesh_net_packet_t * p_net_decrypted_packet)
{
    NRF_MESH_ASSERT(p_net_metadata != NULL && p_net_encrypted_packet != NULL &&
                    p_net_decrypted_packet != NULL &&
                    p_net_decrypted_packet != p_net_encrypted_packet);

    uint8_t nonce[CCM_NONCE_LENGTH];

    /* Configure CCM. */
    ccm_soft_data_t ccm_params;
    ccm_params.a_len   = 0;
    ccm_params.p_a     = NULL;
    ccm_params.p_nonce = nonce;
    ccm_params.p_m     = net_packet_enc_start_get(p_net_encrypted_packet);
    ccm_params.p_out   = net_packet_enc_start_get(p_net_decrypted_packet);

    p_net_metadata->internal.iv_index = net_state_rx_iv_index_get(packet_mesh_net_ivi_get(p_net_encrypted_packet));

    p_net_metadata->p_security_material = NULL;
    uint8_t nid = packet_mesh_net_nid_get(p_net_encrypted_packet);

    const nrf_mesh_network_secmat_t * p_secmat[2] = { NULL, NULL };
    do {
        nrf_mesh_net_secmat_next_get(nid, &p_secmat[0], &p_secmat[1]);

        for(uint8_t i = 0; p_secmat[i] != NULL; i++)
        {
            p_net_metadata->p_security_material = p_secmat[i];

            header_deobfuscate(p_net_metadata, p_net_encrypted_packet, p_net_decrypted_packet);
            deobfuscated_header_fields_get(p_net_metadata, p_net_decrypted_packet);

            if (deobfuscated_header_is_valid(p_net_metadata, net_packet_len, p_net_decrypted_packet))
            {
                ccm_params.mic_len = net_mic_size_get(p_net_metadata->control_packet);
                ccm_params.m_len   = (net_packet_len
                                      - NET_PACKET_ENCRYPTION_START_OFFSET
                                      - ccm_params.mic_len);
                ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;

                /* Create a nonce for use when authenticating the packet from the de-obfuscated header: */
                enc_nonce_generate(p_net_metadata, ENC_NONCE_NET, 0, nonce);

                ccm_params.p_key = p_net_metadata->p_security_material->encryption_key;
                bool authenticated = false;
                enc_aes_ccm_decrypt(&ccm_params, &authenticated);

                if (authenticated)
                {
                    p_net_metadata->dst.value = packet_mesh_net_dst_get(p_net_decrypted_packet);
                    p_net_metadata->dst.type = nrf_mesh_address_type_get(p_net_metadata->dst.value);
                    __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_INFO, "Unencrypted data: ", ccm_params.p_out, ccm_params.m_len);
                    return true;
                }
            }
        }
    } while (p_secmat[0] != NULL);

    return false;
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
    buffer.user_data.token = 0;
    buffer.core_tx.bearer = get_bearer(p_net_metadata);
    buffer.core_tx.role = CORE_TX_ROLE_RELAY;

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
    if (!m_relay_enable)
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

    m_relay_enable = true;

    net_state_init();
    net_state_recover_from_flash();
    net_beacon_init();
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
#if EXPERIMENTAL_INSTABURST_ENABLED
            core_tx_instaburst_interval_set(CORE_TX_ROLE_RELAY, p_opt->opt.val);
#else
            core_tx_adv_interval_set(CORE_TX_ROLE_RELAY, p_opt->opt.val);
#endif
            break;
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
            m_relay_enable = p_opt->opt.val;
            heartbeat_on_feature_change_trigger(HEARTBEAT_TRIGGER_TYPE_RELAY);
            break;
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
            if (p_opt->opt.val > NETWORK_RELAY_RETRANSMITS_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            else
            {
#if EXPERIMENTAL_INSTABURST_ENABLED
                return NRF_ERROR_NOT_SUPPORTED;
#else
                core_tx_adv_count_set(CORE_TX_ROLE_RELAY, p_opt->opt.val);
                break;
#endif
            }
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS:
#if EXPERIMENTAL_INSTABURST_ENABLED
            core_tx_instaburst_interval_set(CORE_TX_ROLE_ORIGINATOR, p_opt->opt.val);
#else
            core_tx_adv_interval_set(CORE_TX_ROLE_ORIGINATOR, p_opt->opt.val);
#endif
            break;
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT:
            if (p_opt->opt.val > NETWORK_RELAY_RETRANSMITS_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            else
            {
#if EXPERIMENTAL_INSTABURST_ENABLED
                return NRF_ERROR_NOT_SUPPORTED;
#else
                core_tx_adv_count_set(CORE_TX_ROLE_ORIGINATOR, p_opt->opt.val);
                break;
#endif
            }
        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
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
#if EXPERIMENTAL_INSTABURST_ENABLED
            p_opt->opt.val = core_tx_instaburst_interval_get(CORE_TX_ROLE_RELAY);
#else
            p_opt->opt.val = core_tx_adv_interval_get(CORE_TX_ROLE_RELAY);
#endif
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        case NRF_MESH_OPT_NET_RELAY_ENABLE:
            p_opt->opt.val = m_relay_enable;
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        case NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT:
#if EXPERIMENTAL_INSTABURST_ENABLED
            p_opt->opt.val = 1;
#else
            p_opt->opt.val = core_tx_adv_count_get(CORE_TX_ROLE_RELAY);
#endif
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS:
#if EXPERIMENTAL_INSTABURST_ENABLED
            p_opt->opt.val = core_tx_instaburst_interval_get(CORE_TX_ROLE_ORIGINATOR);
#else
            p_opt->opt.val = core_tx_adv_interval_get(CORE_TX_ROLE_ORIGINATOR);
#endif
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        case NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT:
#if EXPERIMENTAL_INSTABURST_ENABLED
            p_opt->opt.val = 1;
#else
            p_opt->opt.val = core_tx_adv_count_get(CORE_TX_ROLE_ORIGINATOR);
#endif
            p_opt->len = sizeof(p_opt->opt.val);
            break;
        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t network_packet_alloc(network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);
    p_buffer->core_tx.bearer = get_bearer(p_buffer->user_data.p_metadata);
    p_buffer->core_tx.role = CORE_TX_ROLE_ORIGINATOR;
    return allocate_packet(p_buffer);
}

void network_packet_send(const network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->p_payload != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata != NULL);
    NRF_MESH_ASSERT(p_buffer->user_data.p_metadata->p_security_material != NULL);

    network_pkt_encrypt(p_buffer);

    core_tx_packet_send(&p_buffer->core_tx, (uint8_t *) net_packet_from_payload(p_buffer->p_payload));

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_PACKET_QUEUED_TX, 0, p_buffer->user_data.payload_len, p_buffer->p_payload);
}

void network_packet_discard(const network_tx_packet_buffer_t * p_buffer)
{
    NRF_MESH_ASSERT(p_buffer != NULL);
    NRF_MESH_ASSERT(p_buffer->p_payload != NULL);

    core_tx_packet_discard(&p_buffer->core_tx, (uint8_t *) net_packet_from_payload(p_buffer->p_payload));
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
    uint8_t decrypt_buf[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
    packet_mesh_net_packet_t * p_net_decrypted_packet = (packet_mesh_net_packet_t *) &decrypt_buf[0];

    /* Packet fields not touched by the encryption. */
    memcpy(p_net_decrypted_packet, p_net_packet, NET_PACKET_OBFUSCATION_START_OFFSET);

    network_packet_metadata_t net_metadata;
    bool authenticated = network_pkt_decrypt(&net_metadata, net_packet_len, p_net_packet, p_net_decrypted_packet);
    if (authenticated && metadata_is_valid(&net_metadata))
    {
        NRF_MESH_ASSERT(net_metadata.p_security_material != NULL);

        const uint8_t * p_net_payload = packet_mesh_net_payload_get(p_net_decrypted_packet);

        uint8_t payload_len = payload_len_get(&net_metadata, net_packet_len);

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
