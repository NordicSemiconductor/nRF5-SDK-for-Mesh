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

#include <unity.h>
#include <cmock.h>

#include "utils.h"
#include "prov_beacon.h"
#include "uri.h"

#include "beacon_mock.h"
#include "nrf_mesh_configure_mock.h"
#include "event_mock.h"
#include "enc_mock.h"

/* Sample data from @tagMeshSp section 8.4.1 (unprovisioned device beacon without URI): */
#define UNPROV_SAMPLE_DATA_1                                                                                                                       \
{                                                                                                                                                  \
    .uri = NULL,                                                                                                                                   \
    .oob_info = (NRF_MESH_PROV_OOB_INFO_SOURCE_STRING |                                                                                            \
                 NRF_MESH_PROV_OOB_INFO_SOURCE_ON_PIECE_OF_PAPER |                                                                                 \
                 NRF_MESH_PROV_OOB_INFO_SOURCE_ON_DEVICE),                                                                                         \
    .uuid   = {0x70, 0xcf, 0x7c, 0x97, 0x32, 0xa3, 0x45, 0xb6, 0x91, 0x49, 0x48, 0x10, 0xd2, 0xe9, 0xcb, 0xf4},                                    \
    .beacon_len = 18, \
    .uri_hash = {0}, \
    .beacon = {0x70, 0xcf, 0x7c, 0x97, 0x32, 0xa3, 0x45, 0xb6, 0x91, 0x49, 0x48, 0x10, 0xd2, 0xe9, 0xcb, 0xf4, 0xa0, 0x40} \
}

/* Sample data from @tagMeshSp section 8.4.2 (unprovisioned device beacon with URI): */
#define UNPROV_SAMPLE_DATA_2                                                                                                                       \
{                                                                                                                                                  \
    .uri = (URI_SCHEME_HTTPS "//www.example.com/mesh/products/light-switch-v3"),                                                                   \
    .oob_info = (NRF_MESH_PROV_OOB_INFO_SOURCE_NUMBER |                                                                                            \
                 NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL),                                                                                     \
    .uuid   = {0x70, 0xcf, 0x7c, 0x97, 0x32, 0xa3, 0x45, 0xb6, 0x91, 0x49, 0x48, 0x10, 0xd2, 0xe9, 0xcb, 0xf4},                                    \
    .beacon_len = 22, \
    .uri_hash = {0xd9, 0x74, 0x78, 0xb3, 0x66, 0x7f, 0x48, 0x39, 0x48, 0x74, 0x69, 0xc7, 0x2b, 0x8e, 0x5e, 0x9e}, \
    .beacon = {0x70, 0xcf, 0x7c, 0x97, 0x32, 0xa3, 0x45, 0xb6, 0x91, 0x49, 0x48, 0x10, 0xd2, 0xe9, 0xcb, 0xf4, 0x40, 0x20, 0xd9, 0x74, 0x78, 0xb3} \
}

typedef struct
{
    const char* uri;
    uint16_t oob_info;
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    uint8_t beacon_len;
    uint8_t uri_hash[NRF_MESH_KEY_SIZE];
    uint8_t beacon[22]; /**< Without beacon type */
} unprov_sample_data_t;

static nrf_mesh_prov_evt_t  m_evt;
static uint32_t m_evt_handle_calls_expected;


void setUp(void)
{
    m_evt_handle_calls_expected = 0;
    beacon_mock_Init();
    nrf_mesh_configure_mock_Init();
    event_mock_Init();
    enc_mock_Init();
}

void tearDown(void)
{
    enc_mock_Verify();
    beacon_mock_Verify();
    beacon_mock_Destroy();
    nrf_mesh_configure_mock_Verify();
    nrf_mesh_configure_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
}

void evt_handle_cb(const nrf_mesh_prov_evt_t* p_evt)
{
    TEST_ASSERT_NOT_EQUAL(0, m_evt_handle_calls_expected--);
    /* There's padding in the event struct, so we have to check fields manually: */
    TEST_ASSERT_EQUAL(NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED, p_evt->type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_evt.params.unprov.device_uuid, p_evt->params.unprov.device_uuid, NRF_MESH_UUID_SIZE);
    TEST_ASSERT_EQUAL(m_evt.params.unprov.gatt_supported, p_evt->params.unprov.gatt_supported);
    TEST_ASSERT_EQUAL(m_evt.params.unprov.uri_hash_present, p_evt->params.unprov.uri_hash_present);
    TEST_ASSERT_EQUAL(m_evt.params.unprov.p_metadata, p_evt->params.unprov.p_metadata);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_evt.params.unprov.uri_hash, p_evt->params.unprov.uri_hash, 4);
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_tx(void)
{
    unprov_sample_data_t sample_datas[] = {UNPROV_SAMPLE_DATA_1, UNPROV_SAMPLE_DATA_2};

    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        unprov_sample_data_t sample_data = sample_datas[i];

        nrf_mesh_configure_device_uuid_get_IgnoreAndReturn(sample_data.uuid);
        if (sample_data.uri != NULL)
        {
            enc_s1_ExpectWithArray((const uint8_t*) sample_data.uri, strlen(sample_data.uri), strlen(sample_data.uri), NULL, 0);
            enc_s1_IgnoreArg_p_out();
            enc_s1_ReturnMemThruPtr_p_out(sample_data.uri_hash, NRF_MESH_KEY_SIZE);
        }
        adv_packet_t output;
        advertiser_t adv;
        beacon_create_ExpectWithArrayAndReturn(&adv, 1, BEACON_TYPE_UNPROV, sample_data.beacon, sample_data.beacon_len, sample_data.beacon_len, &output);
        TEST_ASSERT_EQUAL_PTR(&output, prov_beacon_unprov_build(&adv, sample_data.uri, sample_data.oob_info));

        /* Fail beacon_enable */
        nrf_mesh_configure_device_uuid_get_IgnoreAndReturn(sample_data.uuid);
        if (sample_data.uri != NULL)
        {
            enc_s1_ExpectWithArray((const uint8_t*) sample_data.uri, strlen(sample_data.uri), strlen(sample_data.uri), NULL, 0);
            enc_s1_IgnoreArg_p_out();
            enc_s1_ReturnMemThruPtr_p_out(sample_data.uri_hash, NRF_MESH_KEY_SIZE);
        }
        beacon_create_ExpectWithArrayAndReturn(&adv, 1, BEACON_TYPE_UNPROV, sample_data.beacon, sample_data.beacon_len, sample_data.beacon_len, NULL);

        TEST_ASSERT_EQUAL_PTR(NULL, prov_beacon_unprov_build(&adv, sample_data.uri, sample_data.oob_info));
    }

}

void test_rx(void)
{
    unprov_sample_data_t sample_datas[] = {UNPROV_SAMPLE_DATA_1, UNPROV_SAMPLE_DATA_2};
    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        printf("sample data %d\n", i);
        unprov_sample_data_t sample_data = sample_datas[i];
        nrf_mesh_rx_metadata_t meta;
        memset(&m_evt, 0, sizeof(m_evt));
        meta.source = NRF_MESH_RX_SOURCE_SCANNER;
        m_evt.type = NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED;
        m_evt.params.unprov.p_metadata = &meta;
        memcpy(m_evt.params.unprov.device_uuid, sample_data.uuid, NRF_MESH_UUID_SIZE);
        if (sample_data.uri == NULL)
        {
            memset(m_evt.params.unprov.uri_hash, 0, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
        }
        else
        {
            memcpy(m_evt.params.unprov.uri_hash, sample_data.uri_hash, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
        }
        m_evt.params.unprov.gatt_supported = false;
        m_evt.params.unprov.uri_hash_present = (sample_data.uri != NULL);

        m_evt_handle_calls_expected = 1;
        prov_beacon_scan_start(evt_handle_cb);

        prov_beacon_unprov_packet_in(sample_data.beacon, sample_data.beacon_len, &meta);
        TEST_ASSERT_EQUAL(0, m_evt_handle_calls_expected);

        /* Disable reporting: */
        prov_beacon_scan_stop();
        prov_beacon_unprov_packet_in(sample_data.beacon, sample_data.beacon_len, &meta);
        TEST_ASSERT_EQUAL(0, m_evt_handle_calls_expected);

        /* Enable reporting: */
        prov_beacon_scan_start(evt_handle_cb);
        m_evt_handle_calls_expected = 1;
        prov_beacon_unprov_packet_in(sample_data.beacon, sample_data.beacon_len, &meta);
        TEST_ASSERT_EQUAL(0, m_evt_handle_calls_expected);
    }
}

void test_rx_invalid_length(void)
{
    const uint8_t dummy_data[64];
    nrf_mesh_rx_metadata_t meta;
    meta.source = NRF_MESH_RX_SOURCE_SCANNER;
    for (int i = -1; i < NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE + 1; ++i)
    {
        switch (i)
        {
            case 0:
            case NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE:
                /* Don't call for valid lengths. */
                break;
            default:
                /* Nothing should happen here. */
                prov_beacon_unprov_packet_in(
                    dummy_data, sizeof(dummy_data) + i, &meta);
                break;
        }
    }

}
