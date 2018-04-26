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

#include <stdlib.h>

#include "network.h"

#include "unity.h"
#include "cmock.h"

#include "nordic_common.h"
#include "test_assert.h"

#include "log.h"
#include "packet_mesh.h"

#include "core_tx_mock.h"
#include "core_tx_adv_mock.h"
#include "enc_mock.h"
#include "heartbeat_mock.h"
#include "msg_cache_mock.h"
#include "transport_mock.h"
#include "net_beacon_mock.h"
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
} m_enc_nonce_generate_params;

void setUp(void)
{
    __LOG_INIT(LOG_SRC_NETWORK, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    core_tx_mock_Init();
    core_tx_adv_mock_Init();
    enc_mock_Init();
    heartbeat_mock_Init();
    msg_cache_mock_Init();
    transport_mock_Init();
    net_beacon_mock_Init();
    net_state_mock_Init();
    nrf_mesh_externs_mock_Init();
    m_enc_nonce_generate_params.calls = 0;
    m_enc_nonce_generate_params.executed = 0;
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    core_tx_adv_mock_Verify();
    core_tx_adv_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
}
/*****************************************************************************
* Helper functions
*****************************************************************************/
struct
{
    uint16_t src;
    uint16_t dst;
    uint8_t ttl;
    uint32_t calls;
    bool retval;
} m_relay_callback_expect;

static bool relay_callback(uint16_t src, uint16_t dst, uint8_t ttl)
{
    TEST_ASSERT_TRUE(m_relay_callback_expect.calls > 0);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.src, src);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.dst, dst);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.ttl, ttl);
    m_relay_callback_expect.calls--;

    return m_relay_callback_expect.retval;
}

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
    TEST_ASSERT_EQUAL(p_expected_metadata->internal.iv_index, p_net_metadata->internal.iv_index);
    TEST_ASSERT_EQUAL(p_expected_metadata->internal.sequence_number, p_net_metadata->internal.sequence_number);
    TEST_ASSERT_EQUAL(p_expected_metadata->ttl, p_net_metadata->ttl);
    TEST_ASSERT_EQUAL(p_expected_metadata->control_packet, p_net_metadata->control_packet);
    TEST_ASSERT_EQUAL(ENC_NONCE_NET, type);
    TEST_ASSERT_EQUAL(0, aszmic);
    TEST_ASSERT_NOT_NULL(p_nonce);
    m_enc_nonce_generate_params.calls--;
    m_enc_nonce_generate_params.executed++;
    memcpy(p_nonce, m_enc_nonce_generate_params.nonce_return, CCM_NONCE_LENGTH);
}

static void m_enc_nonce_generate_Expect(const network_packet_metadata_t * p_net_metadata)
{
    TEST_ASSERT_TRUE(m_enc_nonce_generate_params.calls < ENC_NONCE_GENERATE_CALLS_MAX);
    memcpy(&m_enc_nonce_generate_params.metadata[m_enc_nonce_generate_params.calls],
        p_net_metadata,
        sizeof(network_packet_metadata_t));
    m_enc_nonce_generate_params.calls++;
}

static void transfuscate_Expect(network_packet_metadata_t * p_metadata, uint8_t * p_packet, uint8_t * p_pecb, uint8_t * p_pecb_data)
{
    pecb_data_build(p_pecb_data, p_metadata, &p_packet[9]);
    enc_aes_encrypt_Expect(p_metadata->p_security_material->privacy_key, p_pecb_data, NULL);
    enc_aes_encrypt_IgnoreArg_p_result();
    enc_aes_encrypt_ReturnMemThruPtr_p_result(p_pecb, NRF_MESH_KEY_SIZE);
}

static void packet_encrypt_Expect(network_packet_metadata_t * p_metadata, uint32_t payload_len, uint8_t * p_packet, uint8_t * p_pecb, uint8_t * p_pecb_data)
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

    m_enc_nonce_generate_Expect(p_metadata);

    enc_nonce_generate_StubWithCallback(enc_nonce_generate_callback);

    enc_aes_ccm_encrypt_StubWithCallback(&enc_aes_ccm_encrypt_callback);

    transfuscate_Expect(p_metadata, p_packet, p_pecb, p_pecb_data);
}

static void packet_decrypt_Expect(network_packet_metadata_t * p_metadata, uint32_t packet_len, uint8_t * p_packet, bool succeed)
{
    uint8_t mic_len = p_metadata->control_packet ? 8 : 4;
    m_enc_nonce_generate_Expect(p_metadata);
    enc_nonce_generate_Expect(NULL, ENC_NONCE_NET, 0, NULL);

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

static uint8_t relay_pecb[NRF_MESH_KEY_SIZE];
static void relay_Expect(network_packet_metadata_t * p_metadata, uint32_t packet_len, uint8_t * p_relay_packet, uint8_t * p_pecb_data)
{
    m_relay_callback_expect.calls = 1;
    m_relay_callback_expect.dst   = p_metadata->dst.value;
    m_relay_callback_expect.retval = true;
    m_relay_callback_expect.src    = p_metadata->src;
    m_relay_callback_expect.ttl    = p_metadata->ttl;
    static uint8_t * p_packet;
    p_packet = p_relay_packet;
    static core_tx_metadata_t relay_core_tx_meta;
    relay_core_tx_meta.bearer = CORE_TX_BEARER_ADV;
    relay_core_tx_meta.role   = CORE_TX_ROLE_RELAY;
    core_tx_packet_alloc_ExpectAndReturn(
        packet_len, &relay_core_tx_meta, NULL, 0, CORE_TX_BEARER_ADV);
    core_tx_packet_alloc_IgnoreArg_pp_packet();
    core_tx_packet_alloc_ReturnThruPtr_pp_packet(&p_packet);

    network_packet_metadata_t relay_meta;
    memcpy(&relay_meta, p_metadata, sizeof(relay_meta));
    relay_meta.ttl--;
    packet_encrypt_Expect(&relay_meta,
                          packet_len - 9 - (p_metadata->control_packet ? 8 : 4),
                          p_relay_packet,
                          relay_pecb,
                          p_pecb_data);
    core_tx_packet_send_Expect(&relay_core_tx_meta, p_relay_packet);
}

struct
{
    const packet_mesh_trs_packet_t * p_packet;
    uint32_t trs_packet_len;
    network_packet_metadata_t * p_net_metadata;
    const nrf_mesh_rx_metadata_t * p_rx_metadata;
    uint32_t calls;
} m_transport_packet_in_expect;

static uint32_t transport_packet_in_callback(const packet_mesh_trs_packet_t * p_packet,
                                             uint32_t trs_packet_len,
                                             const network_packet_metadata_t * p_net_metadata,
                                             const nrf_mesh_rx_metadata_t * p_rx_metadata,
                                             int calls)
{
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.trs_packet_len, trs_packet_len);
    if (trs_packet_len > 0)
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(m_transport_packet_in_expect.p_packet, p_packet, trs_packet_len);
    }
    /* Metadata has padding, so we have to compare the fields individually */
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->control_packet, p_net_metadata->control_packet);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->dst.type, p_net_metadata->dst.type);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->dst.value, p_net_metadata->dst.value);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->internal.iv_index, p_net_metadata->internal.iv_index);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->internal.sequence_number, p_net_metadata->internal.sequence_number);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->p_security_material, p_net_metadata->p_security_material);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->src, p_net_metadata->src);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->ttl, p_net_metadata->ttl);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_rx_metadata, p_rx_metadata);
    TEST_ASSERT_TRUE(m_transport_packet_in_expect.calls > 0);
    m_transport_packet_in_expect.calls--;
    return 0x12345678;
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
void test_init(void)
{
    net_beacon_init_Expect();
    net_state_recover_from_flash_Expect();
    net_state_init_Expect();
    nrf_mesh_init_params_t init_params = {0};
    network_init(&init_params);

    net_beacon_init_Expect();
    net_state_recover_from_flash_Expect();
    net_state_init_Expect();
    init_params.relay_cb = relay_callback;
    network_init(&init_params);
}

void test_alloc(void)
{
    struct
    {
        uint16_t src;
        uint16_t dst;
        uint8_t ttl;
        bool control;
        uint8_t payload_len;
        bool valid_params;
        bool core_tx_alloc_ok;
        bool seqnum_alloc_ok;
    } vector[] = {
        {0x0001, 0x0002, 8, false, 10, true, true, true},
        {0x0001, 0xFFFF, 8, false, 10, true, true, true},
        {0x0001, 0x0002, 128, false, 10, false, true, true}, /* TTL too high */
        {0x8001, 0x0002, 8, false, 10, false, true, true}, /* non-unicast src */
        {0x0001, 0x0002, 8, false, 10, true, false, true}, /* No packets available */
        {0x0001, 0x0002, 8, false, 10, true, false, false}, /* No seqnum available */
    };
    nrf_mesh_network_secmat_t secmat;
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    uint8_t * p_packet = payload;
    memset(secmat.encryption_key, 0xEC, NRF_MESH_KEY_SIZE);
    memset(secmat.privacy_key, 0x93, NRF_MESH_KEY_SIZE);
    secmat.nid                   = 0xAA;
    buffer.user_data.p_metadata  = &metadata;
    buffer.user_data.token       = TOKEN;
    buffer.core_tx.bearer        = CORE_TX_BEARER_ADV;
    buffer.core_tx.role          = CORE_TX_ROLE_ORIGINATOR;
    metadata.p_security_material = &secmat;

    for (volatile uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        metadata.control_packet      = vector[i].control;
        metadata.dst.value           = vector[i].dst;
        metadata.dst.type =
            (vector[i].dst < 0x8000) ? NRF_MESH_ADDRESS_TYPE_UNICAST : NRF_MESH_ADDRESS_TYPE_GROUP;
        metadata.src                 = vector[i].src;
        metadata.ttl                 = vector[i].ttl;
        buffer.user_data.payload_len = vector[i].payload_len;

        core_tx_packet_alloc_ExpectAndReturn(vector[i].payload_len + 9 + (vector[i].control ? 8 : 4),
                                             &buffer.core_tx,
                                             NULL,
                                             TOKEN,
                                             vector[i].core_tx_alloc_ok ? CORE_TX_BEARER_ADV : 0);
        core_tx_packet_alloc_IgnoreArg_pp_packet();
        core_tx_packet_alloc_ReturnThruPtr_pp_packet(&p_packet);
        if (vector[i].core_tx_alloc_ok)
        {
            uint32_t seqnum = SEQNUM;
            net_state_iv_index_lock_Expect(true);
            net_state_tx_iv_index_get_ExpectAndReturn(IV_INDEX);
            net_state_seqnum_alloc_ExpectAndReturn(&metadata.internal.sequence_number,
                                                vector[i].seqnum_alloc_ok ? NRF_SUCCESS
                                                                            : NRF_ERROR_FORBIDDEN);
            net_state_seqnum_alloc_ReturnMemThruPtr_p_seqnum(&seqnum, sizeof(seqnum));
            net_state_iv_index_lock_Expect(false);

            if (vector[i].seqnum_alloc_ok)
            {
                if (vector[i].valid_params)
                {
                    TEST_ASSERT_EQUAL(NRF_SUCCESS, network_packet_alloc(&buffer));
                    TEST_ASSERT_EQUAL_PTR(&payload[9], buffer.p_payload);

                    const uint8_t expected_header[9] = {
                        0x2A | ((IV_INDEX & 0x01) << 7),
                        (vector[i].ttl & 0x7F) | (vector[i].control << 7),
                        seqnum >> 16,
                        seqnum >> 8,
                        seqnum & 0xFF,
                        vector[i].src >> 8,
                        vector[i].src & 0xFF,
                        vector[i].dst >> 8,
                        vector[i].dst & 0xFF,
                    };
                    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_header, payload, sizeof(expected_header));
                }
                else
                {
                    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));
                }
            }
            else
            {
                core_tx_packet_discard_Expect(&buffer.core_tx, payload);
                TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, network_packet_alloc(&buffer));
            }
        }
        else
        {
            TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, network_packet_alloc(&buffer));
        }
        net_state_mock_Verify();
        core_tx_mock_Verify();
    }
    /* Invalid params not covered by vectors */
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(NULL));
    metadata.p_security_material = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));
    buffer.user_data.p_metadata = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));

}

void test_send(void)
{
    uint8_t pecb_data[NRF_MESH_KEY_SIZE];
    nrf_mesh_network_secmat_t secmat;
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    uint8_t pecb[NRF_MESH_KEY_SIZE];
    uint16_t dst = 0x1234;
    uint8_t len  = 10;
    memset(secmat.encryption_key, 0xEC, NRF_MESH_KEY_SIZE);
    memset(secmat.privacy_key, 0x93, NRF_MESH_KEY_SIZE);

    for (uint32_t control = 0; control < 2U; ++control)
    {
        m_enc_nonce_generate_params.executed = 0;
        m_enc_nonce_generate_params.calls = 0;

        metadata.dst.value                = dst;
        metadata.p_security_material      = &secmat;
        metadata.internal.iv_index        = IV_INDEX;
        metadata.internal.sequence_number = SEQNUM;
        metadata.control_packet           = control;

        memset(payload, 0xAB, sizeof(payload));
        for (uint32_t i = 0; i < NRF_MESH_KEY_SIZE; ++i)
        {
            pecb[i] = 137 * i; // arbitrary values
        }
        packet_encrypt_Expect(&metadata, len, payload, pecb, pecb_data);

        buffer.user_data.p_metadata       = &metadata;
        buffer.user_data.token            = TOKEN;
        buffer.user_data.payload_len      = len;
        buffer.core_tx.bearer             = CORE_TX_BEARER_ADV;
        buffer.core_tx.role               = CORE_TX_ROLE_ORIGINATOR;
        buffer.p_payload                  = &payload[9];
        core_tx_packet_send_Expect(&buffer.core_tx, payload);

        network_packet_send(&buffer);
        /* Obfuscation is header[0..5] XOR pecb[0..5] */
        for (uint32_t i = 0; i < 6; ++i)
        {
            TEST_ASSERT_EQUAL_HEX8(0xAB ^ pecb[i], payload[i + 1]);
        }
        /* Network shouldn't alter the payload or destination fields */
        for (uint32_t i = 0; i < 2U + len; ++i)
        {
            TEST_ASSERT_EQUAL_HEX8(0xAB, payload[7 + i]);
        }
    }
    /* Invalid params */
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(NULL));
    buffer.p_payload = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
    buffer.p_payload = &payload[9];
    buffer.user_data.p_metadata->p_security_material = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
    buffer.user_data.p_metadata = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
}

void test_discard(void)
{
    uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    metadata.control_packet      = false;
    buffer.user_data.p_metadata  = &metadata;
    buffer.user_data.token       = TOKEN;
    buffer.user_data.payload_len = 10;
    buffer.core_tx.bearer        = CORE_TX_BEARER_ADV;
    buffer.core_tx.role          = CORE_TX_ROLE_ORIGINATOR;
    buffer.p_payload             = &payload[9];

    for (uint32_t control = 0; control < 2U; ++control)
    {
        metadata.control_packet    = control;
        core_tx_packet_discard_Expect(&buffer.core_tx, payload);
        network_packet_discard(&buffer);
    }
}

/**
 * Test packet in function, to make sure it parses the header and calls the encryption module
 * correctly, as well as discarding packets correctly.
 *
 * The packet in procedure works like this:
 * 1: get a netkey
 * 2: Deobfuscate the header
 * 3: Check that the deobfuscated fields are valid
 * 4: Check the message cache
 * 5: Check that the packet isn't from this device
 * 6: Decrypt the packet
 * 7: Send to transport
 * 8: Relay if possible
 * 9: add to message cache
 *
 * Note that steps 1-6 must pass before 7-9 can execute. Any failure in step 1-6 will result in an early return.
 */
void test_packet_in(void)
{
    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;
    typedef enum {
        STEP_DEOBFUSCATION_VERIFICATION,
        STEP_MSG_CACHE,
        STEP_SRC_ADDRESS_IS_ME,
        STEP_DECRYPTION,
        STEP_DO_RELAY,
        STEP_SUCCESS,
    } step_t;
    struct
    {
        network_packet_metadata_t meta;
        uint32_t length;
        step_t fail_step; /**< The step where the packet processing stops, or STEP_SUCCESS if it goes through all the steps */
    } vector[] = {
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_MSG_CACHE}, /* in cache */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SRC_ADDRESS_IS_ME}, /* this device sent the packet, should discard. */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0xFFFF, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DEOBFUSCATION_VERIFICATION}, /* src is group */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+3, STEP_DEOBFUSCATION_VERIFICATION}, /* Packet is too short for a mic */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+4, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+7, STEP_SUCCESS}, /* long enough for a data packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 9+7, STEP_DEOBFUSCATION_VERIFICATION}, /* Packet is too short for a control mic */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 9+8, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DO_RELAY}, /* Packet is for this device, shouldn't relay */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 1, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DO_RELAY}, /* TTL too low to relay */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},
    };
    nrf_mesh_rx_metadata_t rx_meta;
    uint8_t pecb_data[NRF_MESH_KEY_SIZE];
    uint8_t pecb[NRF_MESH_KEY_SIZE];

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        m_enc_nonce_generate_params.calls = 0;
        m_enc_nonce_generate_params.executed = 0;
        uint8_t net_packet[PACKET_MESH_NET_MAX_SIZE] = {
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
        uint8_t relay_packet[PACKET_MESH_NET_MAX_SIZE];

        /* Obfuscate the packet header with the same pecb that we'll give back when the module
         * requests it, will result in a clean header, because (header XOR pecb XOR pecb) = header */
        for (uint32_t i = 0; i < 6; ++i)
        {
            net_packet[i + 1] ^= pecb[i];
        }

        net_state_rx_iv_index_get_ExpectAndReturn(IV_INDEX & 0x01, IV_INDEX);

        /* 1: Get key */
        const nrf_mesh_network_secmat_t * p_secmats[] = {&secmat, NULL};
        secmat_get_Expect(p_secmats, 2, secmat.nid & 0x7f);

        /* 2: Deobfuscate */
        transfuscate_Expect(&vector[i].meta, net_packet, pecb, pecb_data);

        uint8_t mic_len = vector[i].meta.control_packet ? 8 : 4;

        /* 3: Verify deobfuscated header fields */
        if (vector[i].fail_step >= STEP_MSG_CACHE)
        {
            /* 4: Check message cache */
            msg_cache_entry_exists_ExpectAndReturn(vector[i].meta.src,
                                                   vector[i].meta.internal.sequence_number,
                                                   vector[i].fail_step == STEP_MSG_CACHE);
        }

        if (vector[i].fail_step >= STEP_SRC_ADDRESS_IS_ME)
        {
            /* 5: Check that it's not from ourself */
            nrf_mesh_rx_address_get_ExpectAndReturn(
                vector[i].meta.src, NULL, vector[i].fail_step == STEP_SRC_ADDRESS_IS_ME);
            nrf_mesh_rx_address_get_IgnoreArg_p_address();
        }

        if (vector[i].fail_step >= STEP_DECRYPTION)
        {
            /* 6: decrypt */
            packet_decrypt_Expect(&vector[i].meta,
                                    vector[i].length,
                                    net_packet,
                                    (vector[i].fail_step != STEP_DECRYPTION));
        }

        if (vector[i].fail_step > STEP_DECRYPTION)
        {
            /* 7: Send to transport */
            transport_packet_in_StubWithCallback(transport_packet_in_callback);
            m_transport_packet_in_expect.p_packet = (const packet_mesh_trs_packet_t *) &net_packet[9];
            m_transport_packet_in_expect.trs_packet_len =
                vector[i].length - 9 - mic_len;
            m_transport_packet_in_expect.p_net_metadata = &vector[i].meta;
            m_transport_packet_in_expect.p_rx_metadata  = &rx_meta;
            m_transport_packet_in_expect.calls          = 1;

            /* 8: Relay if needed: */
            if (vector[i].meta.ttl >= 2)
            {
                if (vector[i].meta.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
                {
                    nrf_mesh_rx_address_get_ExpectAndReturn(vector[i].meta.dst.value,
                                                            NULL,
                                                            (vector[i].fail_step ==
                                                                STEP_DO_RELAY));
                    nrf_mesh_rx_address_get_IgnoreArg_p_address();
                }
                if (vector[i].fail_step > STEP_DO_RELAY)
                {
                    relay_Expect(&vector[i].meta, vector[i].length, relay_packet, pecb_data);
                }
            }

            /* 9: Add to message cache */
            msg_cache_entry_add_Expect(vector[i].meta.src, vector[i].meta.internal.sequence_number);
        }

        network_packet_in(net_packet, vector[i].length, &rx_meta);

        TEST_ASSERT_EQUAL(0, m_transport_packet_in_expect.calls);
        TEST_ASSERT_EQUAL(0, m_relay_callback_expect.calls);
        core_tx_mock_Verify();
        transport_mock_Verify();
        nrf_mesh_externs_mock_Verify();
        enc_mock_Verify();
    }
}
