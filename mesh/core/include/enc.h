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
#ifndef MESH_ENC_H__
#define MESH_ENC_H__

#include <stdbool.h>
#include <stdint.h>

#include "ccm_soft.h"
#include "nrf_mesh.h"
#include "net_packet.h"

/**
 * @defgroup ENCRYPTION Encryption Module
 * @ingroup MESH_CORE
 * This module provides security toolbox functions for Bluetooth Mesh. It acts as a
 * wrapper for several cryptographic algorithms, such as, AES, AES-CMAC and
 * AES-CCM.
 *
 * @{
 */


/** Shortest allowed P value in the K2 key derivation procedure. */
#define ENC_K2_P_VALUE_MINLEN   1
/** Longest allowed P value in the K2 key derivation procedure. */
#define ENC_K2_P_VALUE_MAXLEN   16

/**
 * Nonce types.
 */
typedef enum
{
    ENC_NONCE_NET   = 0x00,              /**< Nonce for network data. */
    ENC_NONCE_APP   = 0x01,              /**< Nonce for application data. */
    ENC_NONCE_DEV   = 0x02,              /**< Nonce for device key data. */
    ENC_NONCE_PROXY = 0x03               /**< Nonce for proxy configuration data. */
} enc_nonce_t;

/*lint -align_max(push) -align_max(1) */

/**
 * Network nonce structure.
 */
typedef struct __attribute((packed))
{
    uint8_t  type;              /**< Nonce type. */
    uint32_t ttl   :  7;        /**< Time to live. */
    uint32_t ctl   :  1;        /**< Control message bit. */
    uint32_t seq   : 24;        /**< Sequence number. */
    uint16_t src;               /**< Source address. */
    uint16_t padding;           /**< Padding zero bytes. */
    uint32_t iv_index;          /**< IV index. */
} enc_nonce_net_t;

/**
 * Application nonce structure.
 */
typedef struct __attribute((packed))
{
    uint8_t  type;              /**< Nonce type. */
    uint32_t padding : 7;       /**< Padding bits. */
    uint32_t aszmic  : 1;       /**< Application MIC size bit if segmented message, 0 otherwise. */
    uint32_t seq     : 24;      /**< Sequence number. */
    uint16_t src;               /**< Source address. */
    uint16_t dst;               /**< Destination address. */
    uint32_t iv_index;          /**< IV index. */
} enc_nonce_app_t;

/**
 * Device key nonce structure.
 */
typedef enc_nonce_app_t enc_nonce_dev_t;


/**
 * Proxy nonce structure.
 */
typedef struct __attribute((packed))
{
    uint8_t type;      /**< Nonce type. */
    uint8_t pad;       /**< Padding. */
    uint32_t seq : 24; /**< Sequence number. */
    uint16_t src;      /**< Source address. */
    uint16_t pad2;     /**< Padding zero bytes. */
    uint32_t iv_index; /**< IV index. */
} enc_nonce_proxy_t;

/*lint -align_max(pop) */

/**
 * Generates a cryptographic key.
 * @param p_key         Pointer to a 128-bit buffer where the key should be stored.
 */
void enc_key_generate(uint8_t * p_key);

/**
 * Performs an AES encryption operation.
 * @param p_key         Pointer to a 128-bit encryption key.
 * @param p_plaintext   Pointer to the 128-bit data to be encrypted.
 * @param p_result      Pointer to a buffer where the encrypted data is stored.
 */
void enc_aes_encrypt(const uint8_t * p_key, const uint8_t * p_plaintext, uint8_t * p_result);

/**
 * Performs an AES-CMAC operation.
 * @param p_key         Pointer to a 128-bit encryption key.
 * @param p_data        Pointer to the data that should be hashed.
 * @param data_len      Length of the input data.
 * @param p_result      Pointer to where the 128-bit result should be stored.
 */
void enc_aes_cmac(const uint8_t * p_key, const uint8_t * p_data, uint16_t data_len, uint8_t * p_result);

/**
 * Performs an AES-CCM encryption and authentication operation.
 *
 * @param p_ccm_data      Pointer to structure with encryption data. See
 *                        @ref ccm_soft_data_t.
 */
void enc_aes_ccm_encrypt(ccm_soft_data_t * const p_ccm_data);

/**
 * Performs an AES-CCM decryption and authentication operation.
 *
 * @param p_ccm_data      Pointer to structure with encryption data. See
 *                        @ref ccm_soft_data_t.
 * @param p_mic_passed    Pointer to bool for storing result of MIC check.
 */
void enc_aes_ccm_decrypt(ccm_soft_data_t * const p_ccm_data, bool * const p_mic_passed);


/**
 * Utility function for generating nonce vector.
 *
 * @param[in]  p_net_metadata Network metadata to use in the nonce.
 * @param[in]  type           Type of nonce.
 * @param[in]  aszmic         Application MIC size bit. Only used for application/device keys.
 * @param[out] p_nonce        Pointer to buffer of size @ref CCM_NONCE_LENGTH for storing nonce.
 */

void enc_nonce_generate(const network_packet_metadata_t * p_net_metadata,
                        enc_nonce_t type,
                        uint8_t aszmic,
                        uint8_t * p_nonce);

/**
 * Salt generation function `s1`.
 *
 * @param[in] p_in      Pointer to a buffer containing the variable-length input data.
 * @param[in] in_length Length of the input buffer.
 * @param[out] p_out    Pointer to a buffer where the resulting 128-bit salt is written.
 */
void enc_s1(const uint8_t * p_in, uint16_t in_length, uint8_t * p_out);

/**
 * Key derivation function `k1`.
 *
 * @param[in] p_ikm       Pointer to variable length input keying material.
 * @param[in] ikm_length  Length of keying material.
 * @param[in] p_salt      Pointer to 128-bit salt.
 * @param[in] p_info      Pointer to variable length public constant.
 * @param[in] info_length Length of public constant.
 * @param[out] p_out      Pointer to 128-bit output keying material.
 */
void enc_k1(const uint8_t * p_ikm, const uint8_t ikm_length, const uint8_t * p_salt,
            const uint8_t * p_info, const uint8_t info_length, uint8_t * const p_out);

/**
 * Network key material derivation function `k2`.
 *
 * @param[in]  p_netkey   Pointer to a buffer containing the 128 bit network key.
 * @param[in]  p_p        Pointer to a buffer containing the P value. The required minimum byte length
 * is @ref ENC_K2_P_VALUE_MINLEN, and the length cannot exceed @ref ENC_K2_P_VALUE_MAXLEN bytes.
 * @param[in]  length_p   Length of the P value.
 * @param[out] p_output   Pointer to a buffer where the derived key material is stored.
 */
void enc_k2(const uint8_t * p_netkey, const uint8_t * p_p, uint16_t length_p,
            nrf_mesh_network_secmat_t * p_output);

/**
 * Key derivation function `k3`.
 *
 * @param[in]  p_in       Pointer to a buffer containing the input data. The input
 *                        data to this function must be 128 bits long.
 * @param[out] p_out      Pointer to a buffer where the output data is stored. The
 *                        output data from this function is 64 bits long.
 */
void enc_k3(const uint8_t * p_in, uint8_t * p_out);

/**
 * Key derivation function `k4`.
 *
 * @param[in]  p_in       Pointer to a buffer containing the input data. The input
 *                        data to this function must be 128 bits long.
 * @param[out] p_out      Pointer to a buffer where the output data is stored. The
 *                        output data from this function is 5 bits (rounded up to 1 byte)
 *                        long.
 */
void enc_k4(const uint8_t * p_in, uint8_t * p_out);

/** @} */

#endif

