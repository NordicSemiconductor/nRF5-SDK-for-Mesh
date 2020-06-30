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
#include "net_packet.h"
#include "internal_event.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "msg_cache.h"
#include "net_state.h"
#include "enc.h"
#include "nordic_common.h"

#include "nrf_mesh_gatt.h"
#if MESH_FEATURE_GATT_ENABLED
#include "mesh_gatt.h"
#endif

/** Offset of the start of the encrypted part of the network packet. */
#define NET_PACKET_ENCRYPTION_START_OFFSET PACKET_MESH_NET_DST0_OFFSET
/** Offset of the start of the obfuscated part of the network packet. */
#define NET_PACKET_OBFUSCATION_START_OFFSET PACKET_MESH_NET_CTL_OFFSET
/** Number of bytes before the payload that are being encrypted. */
#define NET_PACKET_ENCRYPTION_START_PAYLOAD_OVERHEAD (PACKET_MESH_NET_PDU_OFFSET - NET_PACKET_ENCRYPTION_START_OFFSET)

/** Redefinition of header_transfuscate() for clarity. */
#define header_obfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)    \
    header_transfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)
/** Redefinition of header_transfuscate() for clarity. */
#define header_deobfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)  \
    header_transfuscate(p_net_metadata, p_net_packet_in, p_net_packet_out)

#define PRIVACY_RANDOM_SIZE 7
#define PECB_SIZE           6
/******************
 * Local Typedefs *
 ******************/

/*lint -align_max(push) -align_max(1) */

/**
 * PECB data according to @tagMeshSp section 3.8.7.3
 */
typedef struct __attribute((packed))
{
    uint8_t          zero_padding[5];                     /**< Zero padding. */
    uint32_t         iv_index_be;                         /**< Current IV index (big endian). */
    uint8_t          privacy_random[PRIVACY_RANDOM_SIZE]; /**< The 7 LSBs of the privacy random value. */
} pecb_data_t;
/*lint -align_max(pop) */


/*****************************************************************************
* Static functions
*****************************************************************************/
static inline enc_nonce_t nonce_type_get(net_packet_kind_t packet_kind)
{
    switch (packet_kind)
    {
        case NET_PACKET_KIND_TRANSPORT:
            return ENC_NONCE_NET;
        case NET_PACKET_KIND_PROXY_CONFIG:
            return ENC_NONCE_PROXY;
        default:
            NRF_MESH_ASSERT(false);
            return ENC_NONCE_NET;
    }
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
    if (net_packet_len - PACKET_MESH_NET_PDU_OFFSET < net_packet_mic_size_get(p_net_metadata->control_packet))
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

    return true;
}

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

static bool try_decrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t net_packet_len,
                        const packet_mesh_net_packet_t * p_net_encrypted_packet,
                        packet_mesh_net_packet_t * p_net_decrypted_packet,
                        const nrf_mesh_network_secmat_t * p_secmat,
                        net_packet_kind_t packet_kind)
{
    bool authenticated = false;
    uint8_t nonce[CCM_NONCE_LENGTH];

    /* Configure CCM. */
    ccm_soft_data_t ccm_params;
    ccm_params.a_len   = 0;
    ccm_params.p_a     = NULL;
    ccm_params.p_nonce = nonce;
    ccm_params.p_m     = net_packet_enc_start_get(p_net_encrypted_packet);
    ccm_params.p_out   = net_packet_enc_start_get(p_net_decrypted_packet);

    p_net_metadata->p_security_material = p_secmat;

    header_deobfuscate(p_net_metadata, p_net_encrypted_packet, p_net_decrypted_packet);

    deobfuscated_header_fields_get(p_net_metadata, p_net_decrypted_packet);

    if (deobfuscated_header_is_valid(p_net_metadata, net_packet_len, p_net_decrypted_packet))
    {
        ccm_params.mic_len = net_packet_mic_size_get(p_net_metadata->control_packet);
        ccm_params.m_len   = (net_packet_len
                                - NET_PACKET_ENCRYPTION_START_OFFSET
                                - ccm_params.mic_len);
        ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;

        /* Create a nonce for use when authenticating the packet from the de-obfuscated header: */
        enc_nonce_generate(p_net_metadata, nonce_type_get(packet_kind), 0, nonce);

        ccm_params.p_key = p_net_metadata->p_security_material->encryption_key;
        enc_aes_ccm_decrypt(&ccm_params, &authenticated);

        if (authenticated)
        {
            p_net_metadata->dst.value = packet_mesh_net_dst_get(p_net_decrypted_packet);
            p_net_metadata->dst.type = nrf_mesh_address_type_get(p_net_metadata->dst.value);
            __LOG_XB(LOG_SRC_NETWORK, LOG_LEVEL_INFO, "Unencrypted data: ", ccm_params.p_out, ccm_params.m_len);
        }
    }
    return authenticated;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t net_packet_decrypt(network_packet_metadata_t * p_net_metadata,
                            uint32_t net_packet_len,
                            const packet_mesh_net_packet_t * p_net_encrypted_packet,
                            packet_mesh_net_packet_t * p_net_decrypted_packet,
                            net_packet_kind_t packet_kind)
{
    NRF_MESH_ASSERT(p_net_metadata != NULL && p_net_encrypted_packet != NULL &&
                    p_net_decrypted_packet != NULL &&
                    p_net_decrypted_packet != p_net_encrypted_packet);

    static const uint32_t net_packet_max_len[] = {
        [NET_PACKET_KIND_TRANSPORT]    = PACKET_MESH_NET_MAX_SIZE,
#if MESH_FEATURE_GATT_ENABLED
        [NET_PACKET_KIND_PROXY_CONFIG] = MESH_GATT_PROXY_PDU_MAX_SIZE,
#endif
    };

    if (net_packet_len < PACKET_MESH_NET_PDU_OFFSET || net_packet_len > net_packet_max_len[packet_kind])
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_net_metadata->internal.iv_index = net_state_rx_iv_index_get(packet_mesh_net_ivi_get(p_net_encrypted_packet));
    p_net_metadata->p_security_material = NULL;
    uint8_t nid = packet_mesh_net_nid_get(p_net_encrypted_packet);

    const nrf_mesh_network_secmat_t * p_secmat[2] = { NULL, NULL };
    do {
        nrf_mesh_net_secmat_next_get(nid, &p_secmat[0], &p_secmat[1]);

        for (uint32_t i = 0; i < ARRAY_SIZE(p_secmat) && p_secmat[i] != NULL; i++)
        {
            if (try_decrypt(p_net_metadata,
                            net_packet_len,
                            p_net_encrypted_packet,
                            p_net_decrypted_packet,
                            p_secmat[i],
                            packet_kind))
            {
                return NRF_SUCCESS;
            }
        }
    } while (p_secmat[0] != NULL);

    return NRF_ERROR_NOT_FOUND;
}

void net_packet_encrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t payload_len,
                        packet_mesh_net_packet_t * p_net_packet,
                        net_packet_kind_t packet_kind)
{
    NRF_MESH_ASSERT(p_net_metadata);
    NRF_MESH_ASSERT(p_net_packet);
    NRF_MESH_ASSERT(packet_kind == NET_PACKET_KIND_PROXY_CONFIG || packet_kind == NET_PACKET_KIND_TRANSPORT);

    uint8_t nonce[CCM_NONCE_LENGTH];

    enc_nonce_generate(p_net_metadata, nonce_type_get(packet_kind), 0, nonce);

    ccm_soft_data_t ccm_params;
    ccm_params.mic_len = net_packet_mic_size_get(p_net_metadata->control_packet);
    ccm_params.p_key   = p_net_metadata->p_security_material->encryption_key;
    ccm_params.p_nonce = nonce;
    /* Include destination field in the encrypted payload. */
    ccm_params.p_m     = net_packet_enc_start_get(p_net_packet);
    ccm_params.p_out   = net_packet_enc_start_get(p_net_packet);
    ccm_params.m_len   = (NET_PACKET_ENCRYPTION_START_PAYLOAD_OVERHEAD + payload_len);
    ccm_params.a_len   = 0;
    ccm_params.p_a     = NULL;
    ccm_params.p_mic   = (uint8_t *) ccm_params.p_m + ccm_params.m_len;

    enc_aes_ccm_encrypt(&ccm_params);

    header_obfuscate(p_net_metadata, p_net_packet, p_net_packet);
}

void net_packet_header_set(packet_mesh_net_packet_t * p_net_packet,
                           const network_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_net_packet);
    NRF_MESH_ASSERT(p_metadata);
    NRF_MESH_ASSERT(nrf_mesh_address_type_get(p_metadata->src) == NRF_MESH_ADDRESS_TYPE_UNICAST);
    NRF_MESH_ASSERT(p_metadata->ttl <= NRF_MESH_TTL_MAX);
    NRF_MESH_ASSERT(p_metadata->internal.sequence_number <= NETWORK_SEQNUM_MAX);
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

uint32_t net_packet_payload_len_get(const network_packet_metadata_t * p_net_metadata,
                                    uint32_t net_packet_len)
{
    return net_packet_len - PACKET_MESH_NET_PDU_OFFSET -
           net_packet_mic_size_get(p_net_metadata->control_packet);
}

packet_mesh_net_packet_t * net_packet_from_payload(const uint8_t * p_net_payload)
{
    return (packet_mesh_net_packet_t *) (p_net_payload - PACKET_MESH_NET_PDU_OFFSET);
}

uint8_t * net_packet_enc_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_ENCRYPTION_START_OFFSET;
}

uint8_t * net_packet_payload_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return (uint8_t *) &p_net_packet->pdu[PACKET_MESH_NET_PDU_OFFSET];
}

uint8_t * net_packet_obfuscation_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_OBFUSCATION_START_OFFSET;
}
