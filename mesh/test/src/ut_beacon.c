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

#include <cmock.h>
#include <unity.h>

#include "beacon.h"
#include "net_beacon_mock.h"
#include "prov_beacon_mock.h"
#include "advertiser_mock.h"
#include "ad_listener.h"

#include "test_assert.h"

#define BEACON_PACKET_ALLOC_OVERHEAD (2 /* AD data */ + 1 /* Beacon overhead */)

// Extern defintion of the beacon.c ad_listener
extern const ad_listener_t m_beacon_listener;

void setUp(void)
{
    advertiser_mock_Init();
    net_beacon_mock_Init();
    prov_beacon_mock_Init();
}

void tearDown(void)
{
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    prov_beacon_mock_Verify();
    prov_beacon_mock_Destroy();
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_beacon_create()
{
    uint8_t dummy_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    adv_packet_t packet;
    adv_packet_t * p_packet;
    advertiser_t adv;
    advertiser_packet_alloc_ExpectAndReturn(&adv, BEACON_PACKET_ALLOC_OVERHEAD + sizeof(dummy_data), &packet);
    p_packet = beacon_create(&adv, 0x43, dummy_data, sizeof(dummy_data));
    TEST_ASSERT_EQUAL_PTR(&packet, p_packet);
    TEST_ASSERT_EQUAL_HEX8(sizeof(dummy_data) + 1 /* ad type */ + 1 /* beacon type */, packet.packet.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(AD_TYPE_BEACON, packet.packet.payload[1]);
    TEST_ASSERT_EQUAL_HEX8(0x43, packet.packet.payload[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(dummy_data, &packet.packet.payload[3], sizeof(dummy_data));

    /* Test illegal params */
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(NULL, 0x43, dummy_data, sizeof(dummy_data)));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(&adv, 0x43, NULL, sizeof(dummy_data)));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(&adv, 0x43, dummy_data, 0));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(&adv, BEACON_TYPE_INVALID, dummy_data, sizeof(dummy_data)));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(&adv, 0x43, dummy_data, 100));
    TEST_ASSERT_EQUAL_PTR(NULL, beacon_create(&adv, 0x43, dummy_data, BEACON_DATA_MAXLEN + 1));
}

void test_beacon_packet_in()
{
    uint8_t beacon_data[] = {BEACON_TYPE_UNPROV, 0x01, 0x02, 0x03, 0x04};
    nrf_mesh_rx_metadata_t meta; // don't really care about contents
    prov_beacon_unprov_packet_in_Expect(&beacon_data[1], 4, &meta);
    m_beacon_listener.handler(beacon_data, sizeof(beacon_data), &meta);

    beacon_data[0] = BEACON_TYPE_SEC_NET_BCAST;
    net_beacon_packet_in_Expect(&beacon_data[1], 4, &meta);
    m_beacon_listener.handler(beacon_data, sizeof(beacon_data), &meta);

    beacon_data[0] = 0x43;
    m_beacon_listener.handler(beacon_data, sizeof(beacon_data), &meta);
    beacon_data[0] = BEACON_TYPE_SEC_NET_BCAST;
    m_beacon_listener.handler(beacon_data, 0, &meta);
    /* NULL metadata */
    net_beacon_packet_in_Expect(&beacon_data[1], 4, NULL);
    m_beacon_listener.handler(beacon_data, sizeof(beacon_data), NULL);

    TEST_NRF_MESH_ASSERT_EXPECT(m_beacon_listener.handler(NULL, sizeof(beacon_data), &meta));
}

