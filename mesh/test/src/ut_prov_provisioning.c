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

#include "nrf_mesh_prov.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <cmock.h>
#include <unity.h>

#include "provisioning_mock.h"
#include "prov_utils_mock.h"
#include "enc_mock.h"
#include "nrf_mesh_utils_mock.h"

#include "utils.h"
#include "nrf_mesh_prov_types.h"

#include "prov_provisionee.h"
#include "prov_provisioner.h"

/* Provisioner public key in little endian format: */
#define PROVISIONER_PUBKEY_LE \
    { \
        0x2c, 0x31, 0xa4, 0x7b, 0x57, 0x79, 0x80, 0x9e, 0xf4, 0x4c, 0xb5, 0xea, 0xaf, 0x5c, 0x3e, 0x43, \
        0xd5, 0xf8, 0xfa, 0xad, 0x4a, 0x87, 0x94, 0xcb, 0x98, 0x7e, 0x9b, 0x03, 0x74, 0x5c, 0x78, 0xdd, \
        0x91, 0x95, 0x12, 0x18, 0x38, 0x98, 0xdf, 0xbe, 0xcd, 0x52, 0xe2, 0x40, 0x8e, 0x43, 0x87, 0x1f, \
        0xd0, 0x21, 0x10, 0x91, 0x17, 0xbd, 0x3e, 0xd4, 0xea, 0xf8, 0x43, 0x77, 0x43, 0x71, 0x5d, 0x4f  \
    }
/* Provisioner private key in little endian format: */
#define PROVISIONER_PRIVKEY_LE \
    { \
        0x06, 0xa5, 0x16, 0x69, 0x3c, 0x9a, 0xa3, 0x1a, 0x60, 0x84, 0x54, 0x5d, 0x0c, 0x5d, 0xb6, 0x41, \
        0xb4, 0x85, 0x72, 0xb9, 0x72, 0x03, 0xdd, 0xff, 0xb7, 0xac, 0x73, 0xf7, 0xd0, 0x45, 0x76, 0x63  \
    }

/* Provisionee public key in little endian format: */
#define PROVISIONEE_PUBKEY_LE \
    { \
        0xf4, 0x65, 0xe4, 0x3f, 0xf2, 0x3d, 0x3f, 0x1b, 0x9d, 0xc7, 0xdf, 0xc0, 0x4d, 0xa8, 0x75, 0x81, \
        0x84, 0xdb, 0xc9, 0x66, 0x20, 0x47, 0x96, 0xec, 0xcf, 0x0d, 0x6c, 0xf5, 0xe1, 0x65, 0x00, 0xcc, \
        0x02, 0x01, 0xd0, 0x48, 0xbc, 0xbb, 0xd8, 0x99, 0xee, 0xef, 0xc4, 0x24, 0x16, 0x4e, 0x33, 0xc2, \
        0x01, 0xc2, 0xb0, 0x10, 0xca, 0x6b, 0x4d, 0x43, 0xa8, 0xa1, 0x55, 0xca, 0xd8, 0xec, 0xb2, 0x79  \
    }
/* Provisionee private key in little endian format: */
#define PROVISIONEE_PRIVKEY_LE \
    { \
        0x52, 0x9a, 0xa0, 0x67, 0x0d, 0x72, 0xcd, 0x64, 0x97, 0x50, 0x2e, 0xd4, 0x73, 0x50, 0x2b, 0x03, \
        0x7e, 0x88, 0x03, 0xb5, 0xc6, 0x08, 0x29, 0xa5, 0xa3, 0xca, 0xa2, 0x19, 0x50, 0x55, 0x30, 0xba  \
    }

/* Shared secret derived from the keys. */
#define SHARED_SECRET_LE \
    { \
        0xab, 0x85, 0x84, 0x3a, 0x2f, 0x6d, 0x88, 0x3f, 0x62, 0xe5, 0x68, 0x4b, 0x38, 0xe3, 0x07, 0x33, \
        0x5f, 0xe6, 0xe1, 0x94, 0x5e, 0xcd, 0x19, 0x60, 0x41, 0x05, 0xc6, 0xf2, 0x32, 0x21, 0xeb, 0x69  \
    }

/* Random numbers for test_provisioning(): */
#define TEST_PROVISIONING_RANDOM_NUMBERS \
    { \
        0x8b, 0x19, 0xac, 0x31, 0xd5, 0x8b, 0x12, 0x4c, 0x94, 0x62, 0x09, 0xb5, 0xdb, 0x10, 0x21, 0xb9, /* Provisioner random number */ \
        0x55, 0xa2, 0xa2, 0xbc, 0xa0, 0x4c, 0xd3, 0x2f, 0xf6, 0xf3, 0x46, 0xbd, 0x0a, 0x0c, 0x1a, 0x3a  /* Provisionee random number */ \
    }

/* Sample messages from the Mesh Profile Specification v1.0: */
#define TEST_INVITE_PDU { 0x00, 0x00 }
#define TEST_CAPABILITIES_PDU { 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define TEST_START_PDU { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define TEST_PUBKEY_PDU_PROVISIONER { 0x03, \
    0x2c, 0x31, 0xa4, 0x7b, 0x57, 0x79, 0x80, 0x9e, 0xf4, 0x4c, 0xb5, 0xea, 0xaf, 0x5c, 0x3e, 0x43, \
    0xd5, 0xf8, 0xfa, 0xad, 0x4a, 0x87, 0x94, 0xcb, 0x98, 0x7e, 0x9b, 0x03, 0x74, 0x5c, 0x78, 0xdd, \
    0x91, 0x95, 0x12, 0x18, 0x38, 0x98, 0xdf, 0xbe, 0xcd, 0x52, 0xe2, 0x40, 0x8e, 0x43, 0x87, 0x1f, \
    0xd0, 0x21, 0x10, 0x91, 0x17, 0xbd, 0x3e, 0xd4, 0xea, 0xf8, 0x43, 0x77, 0x43, 0x71, 0x5d, 0x4f }
#define TEST_PUBKEY_PDU_PROVISIONEE { 0x03, \
    0xf4, 0x65, 0xe4, 0x3f, 0xf2, 0x3d, 0x3f, 0x1b, 0x9d, 0xc7, 0xdf, 0xc0, 0x4d, 0xa8, 0x75, 0x81, \
    0x84, 0xdb, 0xc9, 0x66, 0x20, 0x47, 0x96, 0xec, 0xcf, 0x0d, 0x6c, 0xf5, 0xe1, 0x65, 0x00, 0xcc, \
    0x02, 0x01, 0xd0, 0x48, 0xbc, 0xbb, 0xd8, 0x99, 0xee, 0xef, 0xc4, 0x24, 0x16, 0x4e, 0x33, 0xc2, \
    0x01, 0xc2, 0xb0, 0x10, 0xca, 0x6b, 0x4d, 0x43, 0xa8, 0xa1, 0x55, 0xca, 0xd8, 0xec, 0xb2, 0x79 }
#define TEST_CONFIRMATION_PDU_PROVISIONER { 0x05, 0xb3, 0x8a, 0x11, 0x4d, 0xfd, 0xca, 0x1f, 0xe1, 0x53, 0xbd, 0x2c, 0x1e, 0x0d, 0xc4, 0x6a, 0xc2 }
#define TEST_CONFIRMATION_PDU_PROVISIONEE { 0x05, 0xee, 0xba, 0x52, 0x1c, 0x19, 0x6b, 0x52, 0xcc, 0x2e, 0x37, 0xaa, 0x40, 0x32, 0x9f, 0x55, 0x4e }
#define TEST_RANDOM_PDU_PROVISIONER { 0x06, 0x8b, 0x19, 0xac, 0x31, 0xd5, 0x8b, 0x12, 0x4c, 0x94, 0x62, 0x09, 0xb5, 0xdb, 0x10, 0x21, 0xb9 }
#define TEST_RANDOM_PDU_PROVISIONEE { 0x06, 0x55, 0xa2, 0xa2, 0xbc, 0xa0, 0x4c, 0xd3, 0x2f, 0xf6, 0xf3, 0x46, 0xbd, 0x0a, 0x0c, 0x1a, 0x3a }
#define TEST_DATA_PDU { \
    0x07, 0xd0, 0xbd, 0x7f, 0x4a, 0x89, 0xa2, 0xff, 0x62, 0x22, 0xaf, 0x59, 0xa9, 0x0a, 0x60, 0xad, 0x58, \
    0xac, 0xfe, 0x31, 0x23, 0x35, 0x6f, 0x5c, 0xec, 0x29, 0x73, 0xe0, 0xec, 0x50, 0x78, 0x3b, 0x10, 0xc7 }
#define TEST_COMPLETE_PDU { 0x08 }

/* Sample data from the Mesh Profile Specification v1.0: */
#define DATA_NETKEY  { 0xef, 0xb2, 0x25, 0x5e, 0x64, 0x22, 0xd3, 0x30, 0x08, 0x8e, 0x09, 0xbb, 0x01, 0x5e, 0xd7, 0x07 }
#define DATA_NETKEY_INDEX 0x0567
#define DATA_FLAGS_IV_UPDATE 0
#define DATA_FLAGS_KEY_REFRESH 0
#define DATA_IVINDEX 0x01020304
#define DATA_ADDRESS 0x0b0c

#define PROVISIONEE_NUM_ELEMENTS 1

#define TEST_ATTENTION_DURATION_S 60

typedef union
{
     prov_pdu_invite_t pdu_invite;
     prov_pdu_caps_t pdu_caps;
     prov_pdu_prov_start_t pdu_start;
} prov_pdu_t;

static uint32_t tx_dummy(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static uint32_t listen_start_dummy(prov_bearer_t * p_bearer, const char * p_uri, uint16_t oob_info, uint32_t link_timeout_us);
static uint32_t listen_stop_dummy(prov_bearer_t * p_bearer);
static uint32_t link_open_dummy(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us);
static void link_close_dummy(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);

static nrf_mesh_prov_ctx_t m_provisioner_ctx;
static nrf_mesh_prov_ctx_t m_provisionee_ctx;
static prov_pdu_t m_prov_pdu;

static prov_bearer_interface_t m_interface =
{
    .tx = tx_dummy,
    .listen_start = listen_start_dummy,
    .listen_stop = listen_stop_dummy,
    .link_open = link_open_dummy,
    .link_close = link_close_dummy
};

static prov_bearer_t m_bearer_provisionee;
static prov_bearer_t m_bearer_provisioner;
static bool provisioner_oob_key_req_happened;
static bool provisionee_oob_key_req_happened;
static bool provisionee_invite_rx_happened;
static bool provisionee_start_rx_happened;

static uint32_t tx_dummy(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    (void)p_bearer;
    (void)p_data;
    (void)length;
    return NRF_SUCCESS;
}

static uint32_t listen_start_dummy(prov_bearer_t * p_bearer, const char * p_uri, uint16_t oob_info, uint32_t link_timeout_us)
{
    (void)p_bearer;
    (void)p_uri;
    (void)oob_info;
    (void)link_timeout_us;
    return NRF_SUCCESS;
}

static uint32_t listen_stop_dummy(prov_bearer_t * p_bearer)
{
    (void)p_bearer;
    return NRF_SUCCESS;
}

static uint32_t link_open_dummy(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us)
{
    (void)p_bearer;
    (void)p_uuid;
    (void)link_timeout_us;
    return NRF_SUCCESS;
}

static void link_close_dummy(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    (void)p_bearer;
    (void)close_reason;
}

uint32_t tx_caps_cb(prov_bearer_t* p_bearer, const prov_pdu_caps_t* p_caps, uint8_t* p_confirmation_inputs, int num_calls)
{
    (void)num_calls;
    (void)p_confirmation_inputs;

    TEST_ASSERT_TRUE(p_bearer == &m_bearer_provisionee);
    TEST_ASSERT_NOT_NULL(p_caps);
    TEST_ASSERT_EQUAL_UINT8(PROV_PDU_TYPE_CAPABILITIES, p_caps->pdu_type);
    TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.num_elements, p_caps->num_elements);
    TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.algorithms, BE2LE16(p_caps->algorithms));
    TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.pubkey_type, p_caps->pubkey_type);
    TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_static_types, p_caps->oob_static_types);
    TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_input_size, p_caps->oob_input_size);
    TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_output_size, p_caps->oob_output_size);
    TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.oob_input_actions, BE2LE16(p_caps->oob_input_actions));
    TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.oob_output_actions, BE2LE16(p_caps->oob_output_actions));

    memcpy(&m_prov_pdu.pdu_caps, p_caps, sizeof(prov_pdu_caps_t));

    return NRF_SUCCESS;
}

uint32_t tx_start_cb(prov_bearer_t* p_bearer, const prov_pdu_prov_start_t* p_start, uint8_t* p_confirmation_inputs, int num_calls)
{
    (void)num_calls;
    (void)p_confirmation_inputs;

    TEST_ASSERT_TRUE(p_bearer == &m_bearer_provisioner);
    TEST_ASSERT_NOT_NULL(p_start);
    TEST_ASSERT_EQUAL_UINT8(PROV_PDU_TYPE_START, p_start->pdu_type);
    TEST_ASSERT_EQUAL_UINT8(PROV_PDU_START_ALGO_FIPS_P256, p_start->algorithm);
    TEST_ASSERT_EQUAL_UINT8(1, p_start->public_key);
    TEST_ASSERT_EQUAL_UINT8(NRF_MESH_PROV_OOB_METHOD_OUTPUT, p_start->auth_method);
    TEST_ASSERT_EQUAL_UINT8(NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC, p_start->auth_action);
    TEST_ASSERT_EQUAL_UINT8(5, p_start->auth_size);

    memcpy(&m_prov_pdu.pdu_start, p_start, sizeof(prov_pdu_prov_start_t));

    return NRF_SUCCESS;
}

static void provisionee_event_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(p_evt);

    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_INVITE_RECEIVED:
            {
                const nrf_mesh_prov_evt_invite_received_t * p_invite = &p_evt->params.invite_received;

                TEST_ASSERT_TRUE(&m_provisionee_ctx == p_invite->p_context);
                TEST_ASSERT_EQUAL_UINT8(TEST_ATTENTION_DURATION_S, p_invite->attention_duration_s);

                provisionee_invite_rx_happened = true;
            }
            break;
        case NRF_MESH_PROV_EVT_START_RECEIVED:
            {
                const nrf_mesh_prov_evt_start_received_t * p_start = &p_evt->params.start_received;

                TEST_ASSERT_TRUE(&m_provisionee_ctx == p_start->p_context);

                provisionee_start_rx_happened = true;
            }
            break;
        case NRF_MESH_PROV_EVT_OOB_PUBKEY_REQUEST:
            {
                const nrf_mesh_prov_evt_oob_pubkey_request_t * p_data = &p_evt->params.oob_pubkey_request;

                TEST_ASSERT_TRUE(&m_provisionee_ctx == p_data->p_context);

                provisionee_oob_key_req_happened = true;
            }
            break;
        default:
            break;
    }
}

static void provisioner_event_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(p_evt);

    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_CAPS_RECEIVED:
            {
                const nrf_mesh_prov_evt_caps_received_t * p_data = &p_evt->params.oob_caps_received;
                const nrf_mesh_prov_oob_caps_t * p_caps = &p_data->oob_caps;

                TEST_ASSERT_TRUE(&m_provisioner_ctx == p_data->p_context);
                TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.num_elements, p_caps->num_elements);
                TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.algorithms, p_caps->algorithms);
                TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.pubkey_type, p_caps->pubkey_type);
                TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_static_types, p_caps->oob_static_types);
                TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_input_size, p_caps->oob_input_size);
                TEST_ASSERT_EQUAL_UINT8(m_provisionee_ctx.capabilities.oob_output_size, p_caps->oob_output_size);
                TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.oob_input_actions, p_caps->oob_input_actions);
                TEST_ASSERT_EQUAL_UINT16(m_provisionee_ctx.capabilities.oob_output_actions, p_caps->oob_output_actions);
            }
            break;
        case NRF_MESH_PROV_EVT_OOB_PUBKEY_REQUEST:
            {
                const nrf_mesh_prov_evt_oob_pubkey_request_t * p_data = &p_evt->params.oob_pubkey_request;

                TEST_ASSERT_TRUE(&m_provisioner_ctx == p_data->p_context);

                provisioner_oob_key_req_happened = true;
            }
            break;
        default:
            break;
    }
}

/********** Test Initialization and Finalization ***********/

void setUp(void)
{
    provisioning_mock_Init();
    prov_utils_mock_Init();
    enc_mock_Init();
    nrf_mesh_utils_mock_Init();
}

void tearDown(void)
{
    provisioning_mock_Verify();
    provisioning_mock_Destroy();
    prov_utils_mock_Verify();
    prov_utils_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    nrf_mesh_utils_mock_Verify();
    nrf_mesh_utils_mock_Destroy();
}

/********** Test Cases **********/

void test_provisioning(void)
{
    TEST_IGNORE_MESSAGE("UNIMPLEMENTED TEST");
}

void test_oob_authentication(void)
{
    const char * URI = "hello";
    uint16_t oob_info_sources = NRF_MESH_PROV_OOB_INFO_SOURCE_NUMBER | NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL;
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    nrf_mesh_prov_provisioning_data_t prov_data_stub;

    memset(&m_bearer_provisionee, 0, sizeof(prov_bearer_t));
    m_bearer_provisionee.bearer_type = NRF_MESH_PROV_BEARER_MESH;
    m_bearer_provisionee.p_interface = &m_interface;

    memset(&m_provisionee_ctx, 0, sizeof(nrf_mesh_prov_ctx_t));
    m_provisionee_ctx.p_active_bearer = &m_bearer_provisionee;
    m_provisionee_ctx.event_handler = provisionee_event_handler;
    m_bearer_provisionee.p_parent = &m_provisionee_ctx;

    /* 1 Test provisionee invitation */
    /* 1.1 Initialization */
    TEST_ASSERT_EQUAL_UINT32(NRF_SUCCESS, prov_provisionee_listen(&m_provisionee_ctx,
                                                                  &m_bearer_provisionee,
                                                                  URI,
                                                                  oob_info_sources));
    /* 1.2 Start listening */
    m_bearer_provisionee.p_callbacks->opened(&m_bearer_provisionee);

    /* 1.3 Receiving invitation from provisioner. Sending back capabilities. */
    prov_tx_capabilities_StubWithCallback(tx_caps_cb);
    prov_packet_length_valid_IgnoreAndReturn(true);
    m_prov_pdu.pdu_invite.pdu_type =PROV_PDU_TYPE_INVITE;
    m_prov_pdu.pdu_invite.attention_duration_s = TEST_ATTENTION_DURATION_S;
    m_provisionee_ctx.capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    m_provisionee_ctx.capabilities.num_elements = 1;
    m_provisionee_ctx.capabilities.pubkey_type = NRF_MESH_PROV_OOB_PUBKEY_TYPE_OOB;
    m_provisionee_ctx.capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    m_provisionee_ctx.capabilities.oob_input_size = 1;
    m_provisionee_ctx.capabilities.oob_input_actions = NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER;
    m_provisionee_ctx.capabilities.oob_output_size = 1;
    m_provisionee_ctx.capabilities.oob_input_actions = NRF_MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC;
    m_bearer_provisionee.p_callbacks->rx(&m_bearer_provisionee, (const uint8_t *)&m_prov_pdu.pdu_invite, sizeof(prov_pdu_invite_t));

    TEST_ASSERT_TRUE(provisionee_invite_rx_happened);

    memset(&m_bearer_provisioner, 0, sizeof(prov_bearer_t));
    m_bearer_provisioner.bearer_type = NRF_MESH_PROV_BEARER_MESH;
    m_bearer_provisioner.p_interface = &m_interface;

    memset(&m_provisioner_ctx, 0, sizeof(nrf_mesh_prov_ctx_t));
    m_provisioner_ctx.p_active_bearer = &m_bearer_provisioner;
    m_provisioner_ctx.event_handler = provisioner_event_handler;
    m_bearer_provisioner.p_parent = &m_provisioner_ctx;

    /* 2 Test provisioner invitation */
    /* 2.1 Initialization */
    TEST_ASSERT_EQUAL_UINT32(NRF_SUCCESS, prov_provisioner_provision(&m_provisioner_ctx,
                                                                     uuid,
                                                                     0,
                                                                     &prov_data_stub));
    /* 2.2 Start listening */
    prov_tx_invite_IgnoreAndReturn(NRF_SUCCESS);
    m_bearer_provisioner.p_callbacks->opened(&m_bearer_provisioner);

    /* 2.3 Receiving invitation from provisioner. Sending back capabilities. */
    prov_packet_length_valid_IgnoreAndReturn(true);
    m_bearer_provisioner.p_callbacks->rx(&m_bearer_provisioner, (const uint8_t *)&m_prov_pdu.pdu_caps, sizeof(prov_pdu_caps_t));

    m_provisioner_ctx.capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    m_provisioner_ctx.capabilities.num_elements = 1;
    m_provisioner_ctx.capabilities.pubkey_type = NRF_MESH_PROV_OOB_PUBKEY_TYPE_OOB;
    m_provisioner_ctx.capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    m_provisioner_ctx.capabilities.oob_input_size = 1;
    m_provisioner_ctx.capabilities.oob_input_actions = NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER;
    m_provisioner_ctx.capabilities.oob_output_size = 1;
    m_provisioner_ctx.capabilities.oob_output_actions = NRF_MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC;

    /* 3. Test provisioner tx OOB usage */
    prov_tx_start_StubWithCallback(tx_start_cb);
    TEST_ASSERT_EQUAL_UINT32(NRF_SUCCESS, prov_provisioner_oob_use(&m_provisioner_ctx,
                                                                   NRF_MESH_PROV_OOB_METHOD_OUTPUT,
                                                                   NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC,
                                                                   5));

    /* 4. Test provisionee tx OOB usage with OOB public key event */
    prov_packet_length_valid_IgnoreAndReturn(true);
    m_bearer_provisionee.p_callbacks->ack(&m_bearer_provisionee);
    m_bearer_provisionee.p_callbacks->rx(&m_bearer_provisionee, (const uint8_t *)&m_prov_pdu.pdu_start, sizeof(prov_pdu_prov_start_t));

    TEST_ASSERT_TRUE(provisionee_start_rx_happened);

    /* 5. Test provisioner OOB public key event */
    m_bearer_provisioner.p_callbacks->ack(&m_bearer_provisioner);

    TEST_ASSERT_TRUE(provisionee_oob_key_req_happened);
    TEST_ASSERT_TRUE(provisioner_oob_key_req_happened);
}

void test_disallow_public_key_oob_when_not_supported(void)
{
    const char * URI = "hello";
    uint16_t oob_info_sources = 0;

    memset(&m_bearer_provisionee, 0, sizeof(prov_bearer_t));
    m_bearer_provisionee.bearer_type = NRF_MESH_PROV_BEARER_MESH;
    m_bearer_provisionee.p_interface = &m_interface;

    memset(&m_provisionee_ctx, 0, sizeof(nrf_mesh_prov_ctx_t));
    m_provisionee_ctx.p_active_bearer = &m_bearer_provisionee;
    m_provisionee_ctx.event_handler = provisionee_event_handler;
    m_bearer_provisionee.p_parent = &m_provisionee_ctx;

    /* 1 Test provisionee invitation */
    /* 1.1 Initialization */
    TEST_ASSERT_EQUAL_UINT32(NRF_SUCCESS, prov_provisionee_listen(&m_provisionee_ctx,
                                                                  &m_bearer_provisionee,
                                                                  URI,
                                                                  oob_info_sources));
    /* 1.2 Start listening */
    m_bearer_provisionee.p_callbacks->opened(&m_bearer_provisionee);

    /* 1.3 Receiving invitation from provisioner. Sending back capabilities. */
    prov_tx_capabilities_StubWithCallback(tx_caps_cb);
    prov_packet_length_valid_IgnoreAndReturn(true);
    m_prov_pdu.pdu_invite.pdu_type = PROV_PDU_TYPE_INVITE;
    m_prov_pdu.pdu_invite.attention_duration_s = TEST_ATTENTION_DURATION_S;

    m_provisionee_ctx.capabilities.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    m_provisionee_ctx.capabilities.num_elements = 1;
    m_provisionee_ctx.capabilities.pubkey_type = NRF_MESH_PROV_OOB_PUBKEY_TYPE_INBAND;
    m_provisionee_ctx.capabilities.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    m_provisionee_ctx.capabilities.oob_input_size = 1;
    m_provisionee_ctx.capabilities.oob_input_actions = NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER;
    m_provisionee_ctx.capabilities.oob_output_size = 1;
    m_provisionee_ctx.capabilities.oob_input_actions = NRF_MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC;

    m_bearer_provisionee.p_callbacks->rx(&m_bearer_provisionee, (const uint8_t *)&m_prov_pdu.pdu_invite, sizeof(prov_pdu_invite_t));

    TEST_ASSERT_TRUE(provisionee_invite_rx_happened);

    m_bearer_provisionee.p_callbacks->ack(&m_bearer_provisionee);
    /* Now for the real test... The provisionee has indicated PUBKEY inband, but we'll select OOB. */
    memset(&m_prov_pdu, 0, sizeof(m_prov_pdu));
    m_prov_pdu.pdu_start.pdu_type = PROV_PDU_TYPE_START;
    m_prov_pdu.pdu_start.algorithm = NRF_MESH_PROV_ALGORITHM_FIPS_P256;

    m_prov_pdu.pdu_start.public_key = NRF_MESH_PROV_PUBLIC_KEY_OOB;

    m_prov_pdu.pdu_start.auth_method = NRF_MESH_PROV_OOB_METHOD_OUTPUT;
    m_prov_pdu.pdu_start.auth_action = NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC;
    m_prov_pdu.pdu_start.auth_size = 3;

    prov_tx_failed_ExpectAndReturn(&m_bearer_provisionee, NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT, NRF_SUCCESS);
    m_bearer_provisionee.p_callbacks->rx(&m_bearer_provisionee, (const uint8_t *) &m_prov_pdu.pdu_start, sizeof(prov_pdu_prov_start_t));
}
