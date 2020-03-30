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

#include <stdlib.h>

#include "net_packet.h"

#include "unity.h"
#include "cmock.h"

#include "utils.h"
#include "test_assert.h"

#include "log.h"
#include "packet_mesh.h"

#include "enc_mock.h"
#include "msg_cache_mock.h"
#include "net_state_mock.h"
#include "nrf_mesh_externs_mock.h"

#define TOKEN 0x12345678U
#define IV_INDEX 0x87654321U
#define SEQNUM 0xabcdefU

#define ENC_NONCE_GENERATE_CALLS_MAX 2


static struct
{
    network_packet_metadata_t metadata[ENC_NONCE_GENERATE_CALLS_MAX];
    uint8_t nonce_return[CCM_NONCE_LENGTH];
    uint32_t calls;
    uint32_t executed;
    net_packet_kind_t kind;
} m_enc_nonce_generate_params;

void setUp(void)
{
    __LOG_INIT(LOG_SRC_NETWORK, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    enc_mock_Init();
    msg_cache_mock_Init();
    net_state_mock_Init();
    nrf_mesh_externs_mock_Init();
    m_enc_nonce_generate_params.calls = 0;
    m_enc_nonce_generate_params.executed = 0;
}

void tearDown(void)
{
    enc_mock_Verify();
    enc_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
}
/*****************************************************************************
* Helper functions
*****************************************************************************/

static ccm_soft_data_t m_expected_encrypt_ccm;

static void enc_aes_ccm_encrypt_callback(ccm_soft_data_t * const p_ccm, int calls)
{
    TEST_ASSERT_EQUAL_PTR(m_expected_encrypt_ccm.p_key, p_ccm->p_key);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_encrypt_ccm.p_nonce, p_ccm->p_nonce, CCM_NONCE_LENGTH);
    TEST_ASSERT_EQUAL_PTR(m_expected_encrypt_ccm.p_m, p_ccm->p_m);
    TEST_ASSERT_EQUAL(m_expected_encrypt_ccm.m_len, p_ccm->m_len);
    TEST_ASSERT_EQUAL_PTR(m_expected_encrypt_ccm.p_a, p_ccm->p_a);
    TEST_ASSERT_EQUAL(m_expected_encrypt_ccm.a_len, p_ccm->a_len);
    TEST_ASSERT_EQUAL_PTR(m_expected_encrypt_ccm.p_out, p_ccm->p_out);
    TEST_ASSERT_EQUAL_PTR(m_expected_encrypt_ccm.p_mic, p_ccm->p_mic);
    TEST_ASSERT_EQUAL(m_expected_encrypt_ccm.mic_len, p_ccm->mic_len);
}

static bool m_decrypt_successful;
static ccm_soft_data_t m_expected_decrypt_ccm;
static void enc_aes_ccm_decrypt_callback(ccm_soft_data_t * const p_ccm, bool * p_mic_passed, int calls)
{
    TEST_ASSERT_EQUAL_PTR(m_expected_decrypt_ccm.p_key, p_ccm->p_key);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_decrypt_ccm.p_nonce, p_ccm->p_nonce, CCM_NONCE_LENGTH);
    TEST_ASSERT_EQUAL_PTR(m_expected_decrypt_ccm.p_m, p_ccm->p_m);
    TEST_ASSERT_EQUAL(m_expected_decrypt_ccm.m_len, p_ccm->m_len);
    TEST_ASSERT_EQUAL_PTR(m_expected_decrypt_ccm.p_a, p_ccm->p_a);
    TEST_ASSERT_EQUAL(m_expected_decrypt_ccm.a_len, p_ccm->a_len);
    TEST_ASSERT_NOT_NULL(p_ccm->p_out);
    TEST_ASSERT_EQUAL_PTR(m_expected_decrypt_ccm.p_mic, p_ccm->p_mic);
    TEST_ASSERT_EQUAL(m_expected_decrypt_ccm.mic_len, p_ccm->mic_len);
    *p_mic_passed = m_decrypt_successful;
    /* Just copy the payload right over */
    memcpy(p_ccm->p_out, p_ccm->p_m, p_ccm->m_len + p_ccm->mic_len);
}

static const nrf_mesh_network_secmat_t ** mpp_secmats;
static uint32_t m_secmat_count;
static uint8_t m_expected_nid;
static uint32_t m_secmat_calls;
void nrf_mesh_net_secmat_next_get_callback(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat,
        const nrf_mesh_network_secmat_t ** pp_aux_secmat, int calls)
{
    TEST_ASSERT_EQUAL(m_expected_nid, nid);
    if (m_secmat_calls == 0)
    {
        TEST_ASSERT_NULL(*pp_secmat);
        TEST_ASSERT_NULL(*pp_aux_secmat);
    }
    TEST_ASSERT_TRUE(m_secmat_calls < m_secmat_count);
    *pp_secmat = mpp_secmats[m_secmat_calls++];
    *pp_aux_secmat = NULL;
}

void secmat_get_Expect(const nrf_mesh_network_secmat_t ** pp_secmats, uint32_t secmat_count, uint8_t nid)
{
    nrf_mesh_net_secmat_next_get_StubWithCallback(nrf_mesh_net_secmat_next_get_callback);
    mpp_secmats = pp_secmats;
    m_secmat_count = secmat_count;
    m_expected_nid = nid;
    m_secmat_calls = 0;
}

void pecb_data_build(uint8_t * p_pecb_data, network_packet_metadata_t * p_meta, void * p_payload)
{
    memset(p_pecb_data, 0, 5);
    p_pecb_data[5] = (uint8_t)(p_meta->internal.iv_index >> 24);
    p_pecb_data[6] = (uint8_t)(p_meta->internal.iv_index >> 16);
    p_pecb_data[7] = (uint8_t)(p_meta->internal.iv_index >> 8);
    p_pecb_data[8] = (uint8_t)(p_meta->internal.iv_index);
    p_pecb_data[9]  = p_meta->dst.value >> 8;
    p_pecb_data[10] = p_meta->dst.value & 0xFF;
    memcpy(&p_pecb_data[11], p_payload, 5);
}

static void enc_nonce_generate_callback(const network_packet_metadata_t * p_net_metadata,
                                        enc_nonce_t type,
                                        uint8_t aszmic,
                                        uint8_t * p_nonce,
                                        int calls)
{
    network_packet_metadata_t * p_expected_metadata =
        &m_enc_nonce_generate_params.metadata[m_enc_nonce_generate_params.executed];
    TEST_ASSERT_EQUAL(p_expected_metadata->src, p_net_metadata->src);
    TEST_ASSERT_EQUAL_HEX32(p_expected_metadata->internal.iv_index, p_net_metadata->internal.iv_index);
    TEST_ASSERT_EQUAL(p_expected_metadata->internal.sequence_number, p_net_metadata->internal.sequence_number);
    TEST_ASSERT_EQUAL(p_expected_metadata->ttl, p_net_metadata->ttl);
    TEST_ASSERT_EQUAL(p_expected_metadata->control_packet, p_net_metadata->control_packet);
    if (m_enc_nonce_generate_params.kind == NET_PACKET_KIND_PROXY_CONFIG)
    {
        TEST_ASSERT_EQUAL(ENC_NONCE_PROXY, type);
    }
    else
    {
        TEST_ASSERT_EQUAL(ENC_NONCE_NET, type);
    }
    TEST_ASSERT_EQUAL(0, aszmic);
    TEST_ASSERT_NOT_NULL(p_nonce);
    m_enc_nonce_generate_params.calls--;
    m_enc_nonce_generate_params.executed++;
    memcpy(p_nonce, m_enc_nonce_generate_params.nonce_return, CCM_NONCE_LENGTH);
}

static void m_enc_nonce_generate_Expect(const network_packet_metadata_t * p_net_metadata, net_packet_kind_t kind)
{
    TEST_ASSERT_TRUE(m_enc_nonce_generate_params.calls < ENC_NONCE_GENERATE_CALLS_MAX);
    memcpy(&m_enc_nonce_generate_params.metadata[m_enc_nonce_generate_params.calls],
        p_net_metadata,
        sizeof(network_packet_metadata_t));
    m_enc_nonce_generate_params.kind = kind;
    m_enc_nonce_generate_params.calls++;
}

static void transfuscate_Expect(network_packet_metadata_t * p_metadata, uint8_t * p_packet, uint8_t * p_pecb, uint8_t * p_pecb_data)
{
    pecb_data_build(p_pecb_data, p_metadata, &p_packet[9]);
    enc_aes_encrypt_Expect(p_metadata->p_security_material->privacy_key, p_pecb_data, NULL);
    enc_aes_encrypt_IgnoreArg_p_result();
    enc_aes_encrypt_ReturnMemThruPtr_p_result(p_pecb, NRF_MESH_KEY_SIZE);
}

static void packet_encrypt_Expect(network_packet_metadata_t * p_metadata, uint32_t payload_len, uint8_t * p_packet, uint8_t * p_pecb, uint8_t * p_pecb_data, net_packet_kind_t kind)
{
    memset(p_packet, 0xAB, 9 + payload_len);

    m_expected_encrypt_ccm.a_len         = 0;
    m_expected_encrypt_ccm.p_a           = NULL;
    m_expected_encrypt_ccm.p_key         = p_metadata->p_security_material->encryption_key;
    m_expected_encrypt_ccm.p_m           = &p_packet[7];
    m_expected_encrypt_ccm.p_out         = &p_packet[7];
    m_expected_encrypt_ccm.p_nonce       = m_enc_nonce_generate_params.nonce_return;
    m_expected_encrypt_ccm.p_mic         = &p_packet[9 + payload_len];
    m_expected_encrypt_ccm.m_len         = 2 + payload_len;
    m_expected_encrypt_ccm.mic_len       = p_metadata->control_packet ? 8 : 4;

    m_enc_nonce_generate_Expect(p_metadata, kind);

    enc_nonce_generate_StubWithCallback(enc_nonce_generate_callback);

    enc_aes_ccm_encrypt_StubWithCallback(&enc_aes_ccm_encrypt_callback);

    transfuscate_Expect(p_metadata, p_packet, p_pecb, p_pecb_data);
}

static void packet_decrypt_Expect(network_packet_metadata_t * p_metadata, uint32_t packet_len, uint8_t * p_packet, net_packet_kind_t kind, bool succeed)
{
    uint8_t mic_len = p_metadata->control_packet ? 8 : 4;
    m_enc_nonce_generate_Expect(p_metadata, kind);
    enc_nonce_generate_StubWithCallback(enc_nonce_generate_callback);

    m_expected_decrypt_ccm.a_len   = 0;
    m_expected_decrypt_ccm.mic_len = mic_len;
    m_expected_decrypt_ccm.m_len   = packet_len - 7 - mic_len;
    m_expected_decrypt_ccm.p_a     = NULL;
    m_expected_decrypt_ccm.p_key   = p_metadata->p_security_material->encryption_key;
    m_expected_decrypt_ccm.p_m     = &p_packet[7];
    m_expected_decrypt_ccm.p_mic   = &p_packet[9] + packet_len - 9 - mic_len;
    m_expected_decrypt_ccm.p_nonce = m_enc_nonce_generate_params.nonce_return;
    m_expected_decrypt_ccm.p_out   = NULL;
    m_decrypt_successful           = succeed;
    enc_aes_ccm_decrypt_StubWithCallback(enc_aes_ccm_decrypt_callback);
}

/** Copy of the implementation from nrf_mesh_utils.c. Pulled in to avoid build problems with the other functions in that module */
nrf_mesh_address_type_t nrf_mesh_address_type_get(uint16_t address)
{
    if (address == NRF_MESH_ADDR_UNASSIGNED)
    {
        return NRF_MESH_ADDRESS_TYPE_INVALID;
    }
    else
    {
        static const nrf_mesh_address_type_t types_lookup[] =
        {
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b00 */
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b01 */
            NRF_MESH_ADDRESS_TYPE_VIRTUAL, /* 0b10 */
            NRF_MESH_ADDRESS_TYPE_GROUP,   /* 0b11 */
        };
        return types_lookup[(address & NRF_MESH_ADDR_TYPE_BITS_MASK) >> NRF_MESH_ADDR_TYPE_BITS_OFFSET];
    }
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_encrypt(void)
{
    uint8_t pecb_data[NRF_MESH_KEY_SIZE];
    nrf_mesh_network_secmat_t secmat;
    network_packet_metadata_t metadata;
    packet_mesh_net_packet_t payload;
    uint8_t pecb[NRF_MESH_KEY_SIZE];
    uint16_t dst = 0x1234;
    uint8_t len  = 10;
    memset(secmat.encryption_key, 0xEC, NRF_MESH_KEY_SIZE);
    memset(secmat.privacy_key, 0x93, NRF_MESH_KEY_SIZE);

    struct
    {
        bool control;
        net_packet_kind_t kind;
    } test_vector[] = {
        {false, NET_PACKET_KIND_PROXY_CONFIG},
        {true, NET_PACKET_KIND_PROXY_CONFIG},
        {false, NET_PACKET_KIND_TRANSPORT},
        {true, NET_PACKET_KIND_TRANSPORT},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(test_vector); ++i)
    {
        m_enc_nonce_generate_params.executed = 0;
        m_enc_nonce_generate_params.calls = 0;

        metadata.dst.value                = dst;
        metadata.p_security_material      = &secmat;
        metadata.internal.iv_index        = IV_INDEX;
        metadata.internal.sequence_number = SEQNUM;
        metadata.control_packet           = test_vector[i].control;

        memset(&payload, 0xAB, sizeof(payload));
        for (uint32_t i = 0; i < NRF_MESH_KEY_SIZE; ++i)
        {
            pecb[i] = 137 * i; // arbitrary values
        }
        packet_encrypt_Expect(&metadata, len, (uint8_t *) &payload, pecb, pecb_data, test_vector[i].kind);

        net_packet_encrypt(&metadata, len, &payload, test_vector[i].kind);
        /* Obfuscation is header[0..5] XOR pecb[0..5] */
        for (uint32_t i = 0; i < 6; ++i)
        {
            TEST_ASSERT_EQUAL_HEX8(0xAB ^ pecb[i], payload.pdu[i + 1]);
        }
        /* Network shouldn't alter the payload or destination fields */
        for (uint32_t i = 0; i < 2U + len; ++i)
        {
            TEST_ASSERT_EQUAL_HEX8(0xAB, payload.pdu[7 + i]);
        }
    }
    /* Invalid params */
    TEST_NRF_MESH_ASSERT_EXPECT(net_packet_encrypt(NULL, len, &payload, NET_PACKET_KIND_TRANSPORT));
    TEST_NRF_MESH_ASSERT_EXPECT(net_packet_encrypt(&metadata, len, NULL, NET_PACKET_KIND_TRANSPORT));
    TEST_NRF_MESH_ASSERT_EXPECT(net_packet_encrypt(&metadata, len, &payload, 0x44));
}

/**
 * Test decrypt function, to make sure it parses the header and calls the encryption module
 * correctly, as well as discarding packets correctly.
 *
 * The decrypt procedure works like this:
 * 1: get a netkey
 * 2: Deobfuscate the header
 * 3: Check that the deobfuscated fields are valid
 * 4: Check the message cache
 * 5: Check that the packet isn't from this device
 * 6: Decrypt the packet
 *
 * Note that steps 1-6 must pass before 7-9 can execute. Any failure in step 1-6 will result in an early return.
 */
void test_packet_in(void)
{
    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;
    typedef enum {
        STEP_LENGTH_CHECK,
        STEP_DEOBFUSCATION_VERIFICATION,
        STEP_MSG_CACHE,
        STEP_DECRYPTION,
        STEP_SUCCESS,
    } step_t;
    struct
    {
        network_packet_metadata_t meta;
        uint32_t length;
        net_packet_kind_t kind;
        step_t fail_step; /**< The step where the packet processing stops, or STEP_SUCCESS if it goes through all the steps */
    } vector[] = {
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_MSG_CACHE}, /* in cache */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0xFFFF, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_DEOBFUSCATION_VERIFICATION}, /* src is group */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+3, NET_PACKET_KIND_TRANSPORT, STEP_DEOBFUSCATION_VERIFICATION}, /* Packet is too short for a mic */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+4, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+7, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS}, /* long enough for a data packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 9+7, NET_PACKET_KIND_TRANSPORT, STEP_DEOBFUSCATION_VERIFICATION}, /* Packet is too short for a control mic */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 9+8, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, NET_PACKET_KIND_PROXY_CONFIG, STEP_SUCCESS}, /* Packet kind shouldn't affect success */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 0, NET_PACKET_KIND_TRANSPORT, STEP_LENGTH_CHECK}, /* Too short */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 8, NET_PACKET_KIND_TRANSPORT, STEP_LENGTH_CHECK}, /* Too short */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 30, NET_PACKET_KIND_TRANSPORT, STEP_LENGTH_CHECK}, /* Too long */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9, NET_PACKET_KIND_TRANSPORT, STEP_DEOBFUSCATION_VERIFICATION}, /* min length to start decryption */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 29, NET_PACKET_KIND_TRANSPORT, STEP_SUCCESS}, /* max length */
    };
    uint8_t pecb_data[NRF_MESH_KEY_SIZE];
    uint8_t pecb[NRF_MESH_KEY_SIZE];

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        m_enc_nonce_generate_params.calls = 0;
        m_enc_nonce_generate_params.executed = 0;
        const uint8_t original_net_packet[PACKET_MESH_NET_MAX_SIZE] = {
            (secmat.nid & 0x7f) | ((vector[i].meta.internal.iv_index & 0x01) << 7),
            (vector[i].meta.control_packet << 7) | (vector[i].meta.ttl & 0x7F),
            (vector[i].meta.internal.sequence_number >> 16),
            (vector[i].meta.internal.sequence_number >> 8),
            (vector[i].meta.internal.sequence_number >> 0),
            (vector[i].meta.src >> 8),
            (vector[i].meta.src >> 0),
            (vector[i].meta.dst.value >> 8),
            (vector[i].meta.dst.value >> 0)
        };
        uint8_t net_packet[PACKET_MESH_NET_MAX_SIZE];
        uint8_t decrypted_packet[PACKET_MESH_NET_MAX_SIZE];

        memcpy(net_packet, original_net_packet, sizeof(net_packet));

        /* Obfuscate the packet header with the same pecb that we'll give back when the module
         * requests it, will result in a clean header, because (header XOR pecb XOR pecb) = header */
        for (uint32_t i = 0; i < 6; ++i)
        {
            net_packet[i + 1] ^= pecb[i];
        }

        const nrf_mesh_network_secmat_t * p_secmats[] = {&secmat, NULL};
        if (vector[i].fail_step >= STEP_DEOBFUSCATION_VERIFICATION)
        {
            net_state_rx_iv_index_get_ExpectAndReturn(IV_INDEX & 0x01, IV_INDEX);

            /* 1: Get key */
            secmat_get_Expect(p_secmats, 2, secmat.nid & 0x7f);

            /* 2: Deobfuscate */
            transfuscate_Expect(&vector[i].meta, net_packet, pecb, pecb_data);
        }

        /* 3: Verify deobfuscated header fields */
        if (vector[i].fail_step >= STEP_MSG_CACHE)
        {
            /* 4: Check message cache */
            msg_cache_entry_exists_ExpectAndReturn(vector[i].meta.src,
                                                   vector[i].meta.internal.sequence_number,
                                                   vector[i].fail_step == STEP_MSG_CACHE);
        }

        if (vector[i].fail_step >= STEP_DECRYPTION)
        {
            /* 6: decrypt */
            packet_decrypt_Expect(&vector[i].meta,
                                    vector[i].length,
                                    net_packet,
                                    vector[i].kind,
                                    (vector[i].fail_step != STEP_DECRYPTION));
        }

        uint32_t status = net_packet_decrypt(&vector[i].meta,
                                             vector[i].length,
                                             (packet_mesh_net_packet_t *) net_packet,
                                             (packet_mesh_net_packet_t *) decrypted_packet,
                                             vector[i].kind);

        static const uint32_t expected_statuses[] = {
            [STEP_LENGTH_CHECK] = NRF_ERROR_INVALID_LENGTH,
            [STEP_DEOBFUSCATION_VERIFICATION] = NRF_ERROR_NOT_FOUND,
            [STEP_MSG_CACHE] = NRF_ERROR_NOT_FOUND,
            [STEP_DECRYPTION] = NRF_ERROR_NOT_FOUND,
            [STEP_SUCCESS] = NRF_SUCCESS,
        };
        TEST_ASSERT_EQUAL(expected_statuses[vector[i].fail_step],
                          status);

        nrf_mesh_externs_mock_Verify();
        enc_mock_Verify();
    }
}

void test_util_functions(void)
{
    network_packet_metadata_t net_meta;
    uint32_t length = 18;

    /* Lengths according to @tagMeshSp section 3.4.4 */

    net_meta.control_packet = true;
    TEST_ASSERT_EQUAL(length - 9 - 8, net_packet_payload_len_get(&net_meta, length));
    net_meta.control_packet = false;
    TEST_ASSERT_EQUAL(length - 9 - 4, net_packet_payload_len_get(&net_meta, length));

    packet_mesh_net_packet_t packet;
    TEST_ASSERT_EQUAL_PTR(&packet, net_packet_from_payload(&packet.pdu[9]));
    TEST_ASSERT_EQUAL_PTR(&packet.pdu[7], net_packet_enc_start_get(&packet));
    TEST_ASSERT_EQUAL_PTR(&packet.pdu[1], net_packet_obfuscation_start_get(&packet));
}

void test_packet_header_set(void)
{
    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;
    const network_packet_metadata_t vector[] = {
        {{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 3, false, {SEQNUM, IV_INDEX}, &secmat},
        {{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0201, 5, true, {SEQNUM, IV_INDEX}, &secmat},
        {{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0BCD, 7, false, {SEQNUM, IV_INDEX}, &secmat},
        {{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0F01, 82, false, {SEQNUM, IV_INDEX}, &secmat},
        {{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x1234, 4, true, {SEQNUM, IV_INDEX}, &secmat},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        const uint8_t expected_net_packet[PACKET_MESH_NET_MAX_SIZE] = {
            (secmat.nid & 0x7f) | ((vector[i].internal.iv_index & 0x01) << 7),
            (vector[i].control_packet << 7) | (vector[i].ttl & 0x7F),
            (vector[i].internal.sequence_number >> 16),
            (vector[i].internal.sequence_number >> 8),
            (vector[i].internal.sequence_number >> 0),
            (vector[i].src >> 8),
            (vector[i].src >> 0),
            (vector[i].dst.value >> 8),
            (vector[i].dst.value >> 0)};

        packet_mesh_net_packet_t net_packet;
        net_packet_header_set(&net_packet, &vector[i]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_net_packet, &net_packet, 9);
    }
}
