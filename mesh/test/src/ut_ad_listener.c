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

#include "ad_listener.h"

#include "unity.h"
#include "cmock.h"
#include "test_assert.h"

#include "ad_type_filter_mock.h"
#include "list_mock.h"

#define INCORRECT_ADV         0xEBu
#define TEST_AD               AD_TYPE_MESH
#define TEST_AD_PAYLOAD       0x01, 0x02, 0x03, 0x04, 0x05
#define TEST_AD_PAYLOAD_LEN   5
#define PB_ADV_AD_PAYLOAD     0x06, 0x07, 0x08, 0x09
#define PB_ADV_AD_PAYLOAD_LEN 4
#define BEACON_AD_PAYLOAD     0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
#define BEACON_AD_PAYLOAD_LEN 6
#define ACCESS_ADDRESS        0x12345678
#define CHANNEL               39
#define RSSI                  -34
#define TIMESTAMP             0xabcdef
#define ADDRESS_TYPE          1
#define GAP_ADDRESS_PATTERN   0x12
#define SEND_CYCLE_AMOUNT     5

static uint8_t m_test_cnt;
static uint8_t m_pb_adv_cnt;
static uint8_t m_beacon_cnt;
static uint8_t m_wildcard_cnt;
static list_node_t ** mpp_head;

static void ll_insert(list_node_t * p_prev, list_node_t * p_next)
{
    p_next->p_next = p_prev->p_next;
    p_prev->p_next = p_next;
}

static void list_add_cb(list_node_t ** pp_head, list_node_t * p_new, int num_calls)
{
    (void)num_calls;
    mpp_head = pp_head;

    if (*pp_head == NULL)
    {
        *pp_head = p_new;
        p_new->p_next = NULL;
        return;
    }

    list_node_t * p_node = *pp_head;
    while (p_node->p_next != NULL)
    {
        p_node = p_node->p_next;
    }

    ll_insert(p_node, p_new);
}

static uint32_t list_remove_cb(list_node_t ** pp_head, list_node_t * p_node, int num_calls)
{
    (void)num_calls;

    if (p_node == *pp_head)
    {
        *pp_head = p_node->p_next;
        p_node->p_next = NULL;
        return NRF_SUCCESS;
    }

    list_node_t * p_item = *pp_head;
    while (p_item->p_next != NULL && p_item->p_next != p_node)
    {
        p_item = p_item->p_next;
    }

    p_item->p_next = p_node->p_next;
    p_node->p_next = NULL;

    return NRF_SUCCESS;
}

static void dummy(const uint8_t * p_packet,
                  uint32_t ad_packet_length,
                  const nrf_mesh_rx_metadata_t * p_metadata)
{
    (void)p_packet;
    (void)ad_packet_length;
    (void)p_metadata;
}

static void ad_test_cb(const uint8_t * p_packet,
                       uint32_t ad_packet_length,
                       const nrf_mesh_rx_metadata_t * p_metadata)
{
    uint8_t example[] = {TEST_AD_PAYLOAD};

    TEST_ASSERT_TRUE(TEST_AD_PAYLOAD_LEN == ad_packet_length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(example, p_packet, TEST_AD_PAYLOAD_LEN);

    TEST_ASSERT_EQUAL(NRF_MESH_RX_SOURCE_SCANNER, p_metadata->source);
    TEST_ASSERT_EQUAL(ACCESS_ADDRESS, p_metadata->params.scanner.access_addr);
    TEST_ASSERT_EQUAL(CHANNEL, p_metadata->params.scanner.channel);
    TEST_ASSERT_EQUAL(RSSI, p_metadata->params.scanner.rssi);
    TEST_ASSERT_EQUAL(TIMESTAMP, p_metadata->params.scanner.timestamp);
    TEST_ASSERT_EQUAL(ADDRESS_TYPE, p_metadata->params.scanner.adv_addr.addr_type);
    TEST_ASSERT_EACH_EQUAL_INT8(GAP_ADDRESS_PATTERN,
                                p_metadata->params.scanner.adv_addr.addr,
                                BLE_GAP_ADDR_LEN);
    m_test_cnt++;
}

static void pb_adv_cb(const uint8_t * p_packet,
                      uint32_t ad_packet_length,
                      const nrf_mesh_rx_metadata_t * p_metadata)
{
   (void)p_metadata;
   uint8_t example[] = {PB_ADV_AD_PAYLOAD};

   TEST_ASSERT_TRUE(PB_ADV_AD_PAYLOAD_LEN == ad_packet_length);
   TEST_ASSERT_EQUAL_HEX8_ARRAY(example, p_packet, 1);
   m_pb_adv_cnt++;
}

static void beacon_cb(const uint8_t * p_packet,
                      uint32_t ad_packet_length,
                      const nrf_mesh_rx_metadata_t * p_metadata)
{
   (void)p_metadata;
   uint8_t example[] = {BEACON_AD_PAYLOAD};

   TEST_ASSERT_TRUE(BEACON_AD_PAYLOAD_LEN == ad_packet_length);
   TEST_ASSERT_EQUAL_HEX8_ARRAY(example, p_packet, 1);
   m_beacon_cnt++;
}

static void wildcard_cb(const uint8_t * p_packet,
                      uint32_t ad_packet_length,
                      const nrf_mesh_rx_metadata_t * p_metadata)
{
   (void)p_metadata;
   static uint8_t stage = 0;
   uint8_t size[3] =
   {
       PB_ADV_AD_PAYLOAD_LEN,
       TEST_AD_PAYLOAD_LEN,
       BEACON_AD_PAYLOAD_LEN
   };
   uint8_t example[3][6] =
   {
       {PB_ADV_AD_PAYLOAD},
       {TEST_AD_PAYLOAD},
       {BEACON_AD_PAYLOAD}
   };

   TEST_ASSERT_TRUE(size[stage] == ad_packet_length);
   TEST_ASSERT_EQUAL_HEX8_ARRAY(&example[stage][0], p_packet, size[stage]);

   if (++stage == 3)
   {
       stage = 0;
   }
   m_wildcard_cnt++;
}

static void corrupt_cb(const uint8_t * p_packet,
                       uint32_t ad_packet_length,
                       const nrf_mesh_rx_metadata_t * p_metadata)
{
    (void)p_metadata;
    (void)ad_packet_length;

    uint8_t * tmp = (uint8_t *)p_packet;

    tmp[0] = !tmp[0];
}

static void scanner_packet_init(scanner_packet_t * p_pkt, uint8_t * payload, uint8_t length)
{
    p_pkt->packet.header.length = BLE_ADV_PACKET_OVERHEAD + length;
    p_pkt->packet.header.addr_type = 1;
    p_pkt->packet.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    memset(p_pkt->packet.addr, 0x12, BLE_GAP_ADDR_LEN);

    p_pkt->metadata.access_addr = ACCESS_ADDRESS;
    p_pkt->metadata.channel = CHANNEL;
    p_pkt->metadata.rssi = RSSI;
    p_pkt->metadata.timestamp = TIMESTAMP;
    p_pkt->metadata.adv_addr.addr_type = ADDRESS_TYPE;
    memset(p_pkt->metadata.adv_addr.addr, GAP_ADDRESS_PATTERN, BLE_GAP_ADDR_LEN);

    memcpy(p_pkt->packet.payload, payload, length);
}

void setUp(void)
{
    ad_type_filter_mock_Init();
    list_mock_Init();

    list_add_StubWithCallback(list_add_cb);
    list_remove_StubWithCallback(list_remove_cb);
}

void tearDown(void)
{
    ad_type_filter_mock_Verify();
    ad_type_filter_mock_Destroy();
    list_mock_Verify();
    list_mock_Destroy();
}

void test_incoming_param_checker(void)
{
    ad_listener_t * p_adl = NULL;
    ad_listener_t adl =
    {
        .handler = NULL,
        .adv_packet_type = INCORRECT_ADV
    };

    TEST_ASSERT_TRUE(NRF_ERROR_NULL == ad_listener_subscribe(p_adl));

    p_adl = &adl;

    TEST_ASSERT_TRUE(NRF_ERROR_NULL == ad_listener_subscribe(p_adl));

    p_adl->handler = dummy;

    TEST_ASSERT_TRUE(NRF_ERROR_INVALID_PARAM == ad_listener_subscribe(p_adl));

    p_adl->adv_packet_type = ADL_WILDCARD_ADV_TYPE;
    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_ExpectAnyArgs();
    bearer_adtype_filtering_set_ExpectAnyArgs();
    list_add_Ignore();

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(p_adl));
}

void test_ad_filter_set(void)
{
    ad_listener_t adl1 =
    {
        .handler = dummy,
        .ad_type = TEST_AD,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    ad_listener_t adl2 =
    {
        .handler = dummy,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    bearer_adtype_add_Expect(TEST_AD);
    bearer_adtype_mode_set_Expect(AD_FILTER_WHITELIST_MODE);
    bearer_adtype_filtering_set_Expect(true);
    list_add_Expect(mpp_head, &adl1.node);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl1));

    for (uint16_t i = 0; i <= UINT8_MAX; i++)
    {
        bearer_adtype_add_Expect(i);
    }
    list_add_Expect(mpp_head, &adl2.node);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl2));

    // clean up queue
    *mpp_head = NULL;
}

void test_simple_unsubscription(void)
{
    ad_listener_t adl =
    {
        .handler = dummy,
        .ad_type = TEST_AD,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();
    list_add_Expect(mpp_head, &adl.node);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    list_remove_ExpectAndReturn(mpp_head, &adl.node, NRF_SUCCESS);
    bearer_adtype_remove_Expect(TEST_AD);
    bearer_adtype_filtering_set_Expect(false);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_unsubscribe(&adl));
}

void test_wildcard_unsubscription(void)
{
    ad_listener_t adl =
    {
        .handler = dummy,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();
    list_add_Expect(mpp_head, &adl.node);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    list_remove_ExpectAndReturn(mpp_head, &adl.node, NRF_SUCCESS);
    bearer_adtype_clear_Expect();
    bearer_adtype_filtering_set_Expect(false);

    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_unsubscribe(&adl));
}

void test_mixed_unsubscription(void)
{
    ad_listener_t adl1 =
    {
        .handler = dummy,
        .ad_type = TEST_AD,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    ad_listener_t adl2 =
    {
        .handler = dummy,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    ad_listener_t adl3 =
    {
        .handler = dummy,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    list_add_Expect(mpp_head, &adl1.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl1));
    list_add_Expect(mpp_head, &adl2.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl2));
    list_add_Expect(mpp_head, &adl3.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl3));

    list_remove_ExpectAndReturn(mpp_head, &adl2.node, NRF_SUCCESS);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_unsubscribe(&adl2));

    bearer_adtype_clear_Expect();
    bearer_adtype_add_Expect(TEST_AD);

    list_remove_ExpectAndReturn(mpp_head, &adl3.node, NRF_SUCCESS);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_unsubscribe(&adl3));

    bearer_adtype_remove_Expect(TEST_AD);
    bearer_adtype_filtering_set_Expect(false);

    list_remove_ExpectAndReturn(mpp_head, &adl1.node, NRF_SUCCESS);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_unsubscribe(&adl1));
}

void test_simple_listener_many_frames_process(void)
{
    ad_listener_t adl =
    {
        .handler = ad_test_cb,
        .ad_type = TEST_AD,
        .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND
    };

    uint8_t payload[] =
    {
        2,
        AD_TYPE_PB_ADV,
        3,

        1 + TEST_AD_PAYLOAD_LEN,
        TEST_AD,
        TEST_AD_PAYLOAD,

        2,
        AD_TYPE_BEACON,
        5
    };

    scanner_packet_t pkt;
    scanner_packet_init(&pkt, payload, sizeof(payload));

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    list_add_Expect(mpp_head, &adl.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    m_test_cnt = 0;

    for (uint8_t i = 0; i < SEND_CYCLE_AMOUNT; i++)
    {
        nrf_mesh_rx_metadata_t metadata = {.source         = NRF_MESH_RX_SOURCE_SCANNER,
                                           .params.scanner = pkt.metadata};
        ad_listener_process(pkt.packet.header.type, pkt.packet.payload, sizeof(payload), &metadata);
    }

    TEST_ASSERT_EQUAL_INT8(SEND_CYCLE_AMOUNT, m_test_cnt);

    // clean up queue
    *mpp_head = NULL;
}

void test_wildcard_listener_many_frames_process(void)
{
    ad_listener_t adl =
    {
        .handler = wildcard_cb,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    uint8_t payload[] =
    {
        1 + PB_ADV_AD_PAYLOAD_LEN,
        AD_TYPE_PB_ADV,
        PB_ADV_AD_PAYLOAD,

        1 + TEST_AD_PAYLOAD_LEN,
        TEST_AD,
        TEST_AD_PAYLOAD,

        1 + BEACON_AD_PAYLOAD_LEN,
        AD_TYPE_BEACON,
        BEACON_AD_PAYLOAD
    };

    scanner_packet_t pkt;
    scanner_packet_init(&pkt, payload, sizeof(payload));

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    list_add_Expect(mpp_head, &adl.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    m_wildcard_cnt = 0;

    for (uint8_t i = 0; i < SEND_CYCLE_AMOUNT; i++)
    {
        nrf_mesh_rx_metadata_t metadata = {.source         = NRF_MESH_RX_SOURCE_SCANNER,
                                           .params.scanner = pkt.metadata};
        ad_listener_process(pkt.packet.header.type, pkt.packet.payload, sizeof(payload), &metadata);
    }

    TEST_ASSERT_EQUAL_INT8(3 * SEND_CYCLE_AMOUNT, m_wildcard_cnt);

    // clean up queue
    *mpp_head = NULL;
}

void test_many_listeners_many_frames_process(void)
{
    ad_listener_t adl[] =
    {
        {
            .handler = wildcard_cb,
            .ad_type = ADL_WILDCARD_AD_TYPE,
            .adv_packet_type = ADL_WILDCARD_ADV_TYPE
        },
        {
            .handler = ad_test_cb,
            .ad_type = TEST_AD,
            .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND
        },
        {
            .handler = pb_adv_cb,
            .ad_type = AD_TYPE_PB_ADV,
            .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND
        },
        {
            .handler = beacon_cb,
            .ad_type = AD_TYPE_BEACON,
            .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND
        }
    };

    uint8_t payload[] =
    {
        1 + PB_ADV_AD_PAYLOAD_LEN,
        AD_TYPE_PB_ADV,
        PB_ADV_AD_PAYLOAD,

        1 + TEST_AD_PAYLOAD_LEN,
        TEST_AD,
        TEST_AD_PAYLOAD,

        1 + BEACON_AD_PAYLOAD_LEN,
        AD_TYPE_BEACON,
        BEACON_AD_PAYLOAD
    };

    scanner_packet_t pkt;
    scanner_packet_init(&pkt, payload, sizeof(payload));

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    for (uint8_t i = 0; i < 4; i++)
    {
        list_add_Expect(mpp_head, &adl[i].node);
        TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl[i]));
    }

    m_test_cnt = 0;
    m_pb_adv_cnt = 0;
    m_beacon_cnt = 0;
    m_wildcard_cnt = 0;

    for (uint8_t i = 0; i < SEND_CYCLE_AMOUNT; i++)
    {
        nrf_mesh_rx_metadata_t metadata = {.source         = NRF_MESH_RX_SOURCE_SCANNER,
                                           .params.scanner = pkt.metadata};
        ad_listener_process(pkt.packet.header.type, pkt.packet.payload, sizeof(payload), &metadata);
    }

    TEST_ASSERT_EQUAL_INT8(SEND_CYCLE_AMOUNT, m_test_cnt);
    TEST_ASSERT_EQUAL_INT8(SEND_CYCLE_AMOUNT, m_pb_adv_cnt);
    TEST_ASSERT_EQUAL_INT8(SEND_CYCLE_AMOUNT, m_beacon_cnt);
    TEST_ASSERT_EQUAL_INT8(3 * SEND_CYCLE_AMOUNT, m_wildcard_cnt);

    // clean up queue
    *mpp_head = NULL;
}

void test_hash_check(void)
{
    ad_listener_t adl =
    {
        .handler = corrupt_cb,
        .ad_type = TEST_AD,
        .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND
    };

    uint8_t payload[] =
    {
        1 + TEST_AD_PAYLOAD_LEN,
        TEST_AD,
        TEST_AD_PAYLOAD,
    };

    scanner_packet_t pkt;
    scanner_packet_init(&pkt, payload, sizeof(payload));

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    list_add_Expect(mpp_head, &adl.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    nrf_mesh_rx_metadata_t metadata = {.source         = NRF_MESH_RX_SOURCE_SCANNER,
                                        .params.scanner = pkt.metadata};
    TEST_NRF_MESH_ASSERT_EXPECT(ad_listener_process(pkt.packet.header.type, pkt.packet.payload, sizeof(payload), &metadata));

    // clean up queue
    *mpp_head = NULL;
}

void test_empty_ad_skip(void)
{
    ad_listener_t adl =
    {
        .handler = wildcard_cb,
        .ad_type = ADL_WILDCARD_AD_TYPE,
        .adv_packet_type = ADL_WILDCARD_ADV_TYPE
    };

    uint8_t payload[] = {};

    scanner_packet_t pkt;
    scanner_packet_init(&pkt, payload, sizeof(payload));

    bearer_adtype_add_Ignore();
    bearer_adtype_mode_set_Ignore();
    bearer_adtype_filtering_set_Ignore();

    list_add_Expect(mpp_head, &adl.node);
    TEST_ASSERT_TRUE(NRF_SUCCESS == ad_listener_subscribe(&adl));

    m_wildcard_cnt = 0;

    for (uint8_t i = 0; i < SEND_CYCLE_AMOUNT; i++)
    {
        nrf_mesh_rx_metadata_t metadata = {.source         = NRF_MESH_RX_SOURCE_SCANNER,
                                            .params.scanner = pkt.metadata};
        ad_listener_process(pkt.packet.header.type, pkt.packet.payload, sizeof(payload), &metadata);
    }

    TEST_ASSERT_EQUAL_INT8(0, m_wildcard_cnt);

    // clean up queue
    *mpp_head = NULL;
}
