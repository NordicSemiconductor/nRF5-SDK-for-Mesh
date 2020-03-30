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
#ifndef NET_PACKET_H__
#define NET_PACKET_H__

#include <stdint.h>
#include "nrf_mesh.h"
#include "packet_mesh.h"

/**
 * @defgroup NET_PACKET Network packet
 * Common operations performed on network packets.
 * @{
 */

/**
 * Kind of network packet.
 */
typedef enum
{
    NET_PACKET_KIND_TRANSPORT, /**< Network packet containing Transport layer data */
    NET_PACKET_KIND_PROXY_CONFIG, /**< Network packet containing proxy configuration data. */
} net_packet_kind_t;

typedef struct
{
    /** Packet destination address. */
    nrf_mesh_address_t dst;
    /** Address of the element the packet originates from (must be a unicast address). */
    uint16_t src;
    /** Time to live value for the packet, this is a 7 bit value. */
    uint8_t ttl;
    /** Flag indicating whether the packet is a control packet. */
    bool control_packet;
    /** Parameters generated internally. */
    struct
    {
        /** Sequence number of the message. */
        uint32_t sequence_number;
        /** IV index of the message. */
        uint32_t iv_index;
    } internal;
    /** Network security material. */
    const nrf_mesh_network_secmat_t * p_security_material;
} network_packet_metadata_t;


/**
 * Decrypt and verify a network packet.
 *
 * @param[out] p_net_metadata Metadata structure to fill during decryption.
 * @param[in] net_packet_len Length of the entire network packet.
 * @param[in] p_net_encrypted_packet Encrypted network packet.
 * @param[out] p_net_decrypted_packet Pointer to buffer in which the encrypted
 *                                    packet is decrypted into.
 *
 * @retval NRF_SUCCESS The packet was successfully decrypted.
 * @retval NRF_ERROR_NOT_FOUND Couldn't find a network key to decrypt the packet.
 */
uint32_t net_packet_decrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t net_packet_len,
                        const packet_mesh_net_packet_t * p_net_encrypted_packet,
                        packet_mesh_net_packet_t * p_net_decrypted_packet,
                        net_packet_kind_t packet_kind);

/**
 * Encrypt a network packet.
 *
 * @param[in,out] p_net_metadata Metadata of the packet to encrypt. Will get its internal variables set in the process.
 * @param[in] payload_len Length of the packet payload.
 * @param[in,out] p_net_packet Network packet to encrypt.
 * @param[in] packet_kind Kind of network packet.
 */
void net_packet_encrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t payload_len,
                        packet_mesh_net_packet_t * p_net_packet,
                        net_packet_kind_t packet_kind);

/**
 * Populate the header of the given network packet with the given metadata.
 *
 * @param[in,out] p_net_packet Network packet to populate header of.
 * @param[in] p_metadata Metadata to use when populating the header.
 */
void net_packet_header_set(packet_mesh_net_packet_t * p_net_packet,
                           const network_packet_metadata_t * p_metadata);
/**
 * Get length of network payload.
 *
 * @param[in] p_net_metadata Metadata for the packet to check for.
 * @param[in] net_packet_len Full length of the network packet.
 *
 * @returns The length of the network payload in bytes, not including the MIC.
 */
uint32_t net_packet_payload_len_get(const network_packet_metadata_t * p_net_metadata,
                                    uint32_t net_packet_len);

/**
 * Get network packet from the payload pointer.
 *
 * @param[in] p_net_payload Pointer to some network payload.
 *
 * @returns A pointer to the full network packet containing the given payload.
 */
packet_mesh_net_packet_t * net_packet_from_payload(const uint8_t * p_net_payload);

/**
 * Get the start of the encrypted part of the given network packet.
 *
 * @param[in] p_net_packet Network packet to get from.
 *
 * @returns A pointer to the first encrypted byte in the network packet.
 */
uint8_t * net_packet_enc_start_get(const packet_mesh_net_packet_t * p_net_packet);

/**
 * Get the start of the obfuscated part of the given network packet.
 *
 * @param[in] p_net_packet Network packet to get the obfuscated part of
 *
 * @returns A pointer to the first obfuscated byte in the network packet.
 */
uint8_t * net_packet_obfuscation_start_get(const packet_mesh_net_packet_t * p_net_packet);


/**
 * Get the start of the payload of the given network packet.
 *
 * @param[in] p_net_packet Network packet to get the payload of.
 *
 * @returns A pointer to the first payload byte in the network packet.
 */
uint8_t * net_packet_payload_get(const packet_mesh_net_packet_t * p_net_packet);

/**
 * Get size of the Network packet MIC field.
 *
 * @param[in] is_control_packet Flag indicating whether the packet is a control packet or not.
 *
 * @returns The size of the network MIC field in bytes.
 */
static inline uint32_t net_packet_mic_size_get(bool is_control_packet)
{
    /* According to @tagMeshSp section 3.4.4.3, control messages have an 8 byte mic, access
     * messages have a 4 byte mic. */
    return (is_control_packet ? 8 : 4);
}
/** @} */

#endif /* NET_PACKET_H__ */

