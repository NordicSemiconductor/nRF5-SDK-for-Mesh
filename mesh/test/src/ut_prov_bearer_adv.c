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

#include "nrf_mesh_prov_bearer_adv.h"
#include "prov_bearer_adv.h"
#include "nrf_mesh_prov.h"
#include "log.h"

#include <cmock.h>
#include <unity.h>

#include "advertiser_mock.h"
#include "provisioning_mock.h"
#include "timer_scheduler_mock.h"
#include "rand_mock.h"
#include "prov_beacon_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_configure_mock.h"
#include "timer_mock.h"
#include "bearer_event_mock.h"
#include "ad_listener.h"

#include "test_assert.h"
#include "utils.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Transaction start values for each role (from @tagMeshSp section 5.2.1) */
#define PROVISIONER_TRANSACTION_START_VALUE 0
#define PROVISIONEE_TRANSACTION_START_VALUE 0x80

#define BLE_ADV_OVERHEAD (BLE_GAP_ADDR_LEN + sizeof(ble_ad_data_t) /* length field and AD Type*/)
#define PROV_ADV_OVERHEAD (sizeof(ble_ad_data_t) + 4 /*link ID*/ + 1 /* transaction no*/)
/** The largest provisioning PDU length (copied from @tagMeshSp table 5.3: 64 byte payload + 1 for pdu type). */
#define PROV_PAYLOAD_MAX_LENGTH  65
#define PROV_LINK_OPEN_DATA_SIZE 17
#define PROV_LINK_ACK_DATA_SIZE 1
#define PROV_LINK_CLOSE_DATA_SIZE 2
#define PROV_TRANS_ACK_DATA_SIZE 1
/** The largest payload length in a single provisioning data packet, see @tagMeshSp table 5.2 */
#define GENERIC_PROV_PDU_MAX_LEN 24
/** Max, see @tagMeshSp figure 5.3 */
#define PROV_START_PDU_HEADER_SIZE (1 /*SegN | GPCF */ + 2 /*Total length */ + 1 /* FCS */)
#define PROV_START_PDU_PAYLOAD_MAX_LEN (GENERIC_PROV_PDU_MAX_LEN - PROV_START_PDU_HEADER_SIZE)
/** Max, see @tagMeshSp figure 5.5 */
#define PROV_CONTINUE_PDU_HEADER_SIZE (1 /*SegN | GPCF */)
#define PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN (GENERIC_PROV_PDU_MAX_LEN - PROV_CONTINUE_PDU_HEADER_SIZE)

#define PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD   (3)
#define PROV_LINK_ACK_OPCODE_WITH_GPCF_FIELD    ((1<<2) | 3)
#define PROV_LINK_CLOSE_OPCODE_WITH_GPCF_FIELD  ((2<<2) | 3)
#define PROV_TRANS_ACK_OPCODE_WITH_GPCF_FIELD   (1)
#define PROV_TRANS_START_OPCODE_WITH_GPCF_FIELD (0)
#define PROV_TRANS_CONTINUE_OPCODE_WITH_GPCF_FIELD  (2)

#define PROV_BEARER_ADV_UNACKED_REPEAT_COUNT    (6)
#define PROV_BEARER_ADV_LINK_OPEN_ADVERTISER_INTERVAL_MS  (3 * BEARER_ADV_INT_DEFAULT_MS)
#define PROV_BEARER_ADV_LINK_ESTABLISHMENT_LENGTH_US      SEC_TO_US(60)

/** Flag the module should use to trigger async callback. */
#define ASYNC_FLAG  0xF1A6F1A6

static adv_packet_t m_packet;

#define ALLOC_AND_TX(P_ADV, ALLOC_SIZE, SUCCESS)                                                \
do                                                                                              \
{                                                                                               \
    advertiser_packet_alloc_ExpectAndReturn(P_ADV, ALLOC_SIZE, (SUCCESS) ? &m_packet : NULL);   \
    if (SUCCESS)                                                                                \
    {                                                                                           \
        advertiser_packet_send_Expect(P_ADV, &m_packet);                                        \
    }                                                                                           \
}                                                                                               \
while (0)

static uint8_t data[PROV_PAYLOAD_MAX_LENGTH+1];
static uint8_t uuid1[NRF_MESH_UUID_SIZE] = {0,1,2,3};
static uint8_t uuid2[NRF_MESH_UUID_SIZE] = {9,10,11,12};
static uint8_t link_open_payload[PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE];
static uint8_t link_ack_payload[PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE];
static uint8_t link_close_payload[PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE];
static uint8_t trans_data_payload[PROV_ADV_OVERHEAD + GENERIC_PROV_PDU_MAX_LEN];
static uint8_t trans_ack_payload[PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE];
static uint8_t minimal_payload[PROV_ADV_OVERHEAD + 1];

static prov_bearer_if_tx_t           prov_bearer_adv_tx;
static prov_bearer_if_listen_start_t prov_bearer_adv_listen;
static prov_bearer_if_listen_stop_t  prov_bearer_adv_listen_stop;
static prov_bearer_if_link_open_t    prov_bearer_adv_link_open;
static prov_bearer_if_link_close_t   prov_bearer_adv_link_close;

static nrf_mesh_rx_metadata_t m_dummy_metadata;

static advertiser_tx_complete_cb_t m_tx_complete_cb;
static bearer_event_flag_callback_t m_async_cb;

static struct
{
    advertiser_t * p_advertiser;
    uint8_t * p_adv_buf;
    uint32_t adv_buf_size;
    uint32_t calls;
} m_expected_adv_init;

static void rx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static void ack_cb(prov_bearer_t * p_bearer);
static void opened_cb(prov_bearer_t * p_bearer);
static void closed_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);

static prov_bearer_callbacks_t m_callbacks =
{
    .rx = rx_cb,
    .ack = ack_cb,
    .opened = opened_cb,
    .closed = closed_cb
};

/********** Local Mock Functions **********/
static void advertiser_instance_init_cb(advertiser_t * p_adv, advertiser_tx_complete_cb_t tx_cb, uint8_t * p_buffer, uint32_t buffer_size, int calls)
{
    TEST_ASSERT_EQUAL(m_expected_adv_init.p_advertiser, p_adv);
    TEST_ASSERT_EQUAL(m_expected_adv_init.p_adv_buf, p_buffer);
    TEST_ASSERT_EQUAL(m_expected_adv_init.adv_buf_size, buffer_size);
    TEST_ASSERT_NOT_NULL(tx_cb);
    TEST_ASSERT_NOT_EQUAL(0, m_expected_adv_init.calls);
    m_expected_adv_init.calls--;
    m_tx_complete_cb = tx_cb;
}

static void advertiser_instance_init_ExpectStub(advertiser_t * p_adv, uint8_t * p_buffer, uint32_t buffer_size)
{
    advertiser_instance_init_StubWithCallback(advertiser_instance_init_cb);
    m_expected_adv_init.p_advertiser = p_adv;
    m_expected_adv_init.p_adv_buf = p_buffer;
    m_expected_adv_init.adv_buf_size = buffer_size;
    m_expected_adv_init.calls++;
}

static bearer_event_flag_t bearer_event_flag_add_cb(bearer_event_flag_callback_t cb, int calls)
{
    TEST_ASSERT_NOT_NULL(cb);
    m_async_cb = cb;
    return ASYNC_FLAG;
}

static void prov_bearer_adv_packet_in(const uint8_t * p_data, uint32_t data_len, const nrf_mesh_rx_metadata_t * p_metadata)
{
    /* Extern definition of the PB-ADV ad_listener */
    extern const ad_listener_t m_pb_adv_ad_listener;

    m_pb_adv_ad_listener.handler(p_data, data_len, p_metadata);

}

#define PARENT_POINTER ((void*) 0xCAFEBABE)

static prov_bearer_t * mp_prov_bearer;
static bool m_link_opened_expected;
static bool m_link_closed_expected;
static bool m_pkt_in_expected;
static bool m_ack_expected;
static nrf_mesh_prov_link_close_reason_t m_link_close_reason;
static const uint8_t * m_pkt_in_p_data;
static uint16_t m_pkt_in_data_length;

static void prov_cb_link_opened_Expect(prov_bearer_t * p_bearer)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    mp_prov_bearer = p_bearer;
    m_link_opened_expected = true;
}

static void prov_cb_link_closed_Expect(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    mp_prov_bearer = p_bearer;
    m_link_close_reason = close_reason;
    m_link_closed_expected = true;
}

static void prov_cb_pkt_in_Expect(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_data);
    mp_prov_bearer = p_bearer;
    m_pkt_in_p_data = p_data;
    m_pkt_in_data_length = length;
    m_pkt_in_expected = true;
}

static void prov_cb_ack_in_Expect(prov_bearer_t * p_bearer)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    mp_prov_bearer = p_bearer;
    m_ack_expected = true;
}

static void rx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    TEST_ASSERT(m_pkt_in_expected);
    TEST_ASSERT_EQUAL_PTR(mp_prov_bearer, p_bearer);
    TEST_ASSERT_EQUAL(m_pkt_in_data_length, length);
    TEST_ASSERT_EQUAL_MEMORY(m_pkt_in_p_data, p_data, length);
    m_pkt_in_expected = false;
    mp_prov_bearer = NULL;
    m_pkt_in_data_length = 0;
    m_pkt_in_p_data = NULL;
}

static void ack_cb(prov_bearer_t * p_bearer)
{
    TEST_ASSERT(m_ack_expected);
    TEST_ASSERT_EQUAL_PTR(mp_prov_bearer, p_bearer);
    m_ack_expected = false;
    mp_prov_bearer = NULL;
}

static void opened_cb(prov_bearer_t * p_bearer)
{
    TEST_ASSERT(m_link_opened_expected);
    TEST_ASSERT_EQUAL_PTR(mp_prov_bearer, p_bearer);
    m_link_opened_expected = false;
    mp_prov_bearer = NULL;
}

static void closed_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    TEST_ASSERT(m_link_closed_expected);
    TEST_ASSERT_EQUAL_PTR(mp_prov_bearer, p_bearer);
    TEST_ASSERT_EQUAL(m_link_close_reason, reason);
    m_link_closed_expected = false;
    mp_prov_bearer = NULL;
    m_link_close_reason = NRF_MESH_PROV_LINK_CLOSE_REASON_LAST + 1;
}

static void setup_interface(nrf_mesh_prov_bearer_adv_t * p_bearer_adv)
{
    prov_bearer_t * p_bearer = nrf_mesh_prov_bearer_adv_interface_get(p_bearer_adv);
    prov_bearer_adv_tx = p_bearer->p_interface->tx;
    prov_bearer_adv_listen = p_bearer->p_interface->listen_start;
    prov_bearer_adv_listen_stop = p_bearer->p_interface->listen_stop;
    prov_bearer_adv_link_open = p_bearer->p_interface->link_open;
    prov_bearer_adv_link_close = p_bearer->p_interface->link_close;

    p_bearer->p_parent = PARENT_POINTER;
    p_bearer->p_callbacks = &m_callbacks;
}


/********** Test Setup **********/

void setUp(void)
{
    __LOG_INIT((LOG_SRC_PROV | LOG_SRC_TEST), LOG_LEVEL_ERROR, LOG_CALLBACK_DEFAULT);

    advertiser_mock_Init();
    provisioning_mock_Init();
    timer_scheduler_mock_Init();
    rand_mock_Init();
    prov_beacon_mock_Init();
    nrf_mesh_mock_Init();
    timer_mock_Init();

    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_cb);

    for (int i=0;i<PROV_PAYLOAD_MAX_LENGTH+1;i++)
    {
        data[i] = i;
    }
}

void tearDown(void)
{
    TEST_ASSERT_EQUAL(0, m_expected_adv_init.calls);
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    provisioning_mock_Verify();
    provisioning_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    prov_beacon_mock_Verify();
    prov_beacon_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

/* In order to use the prov_bearer_adv function. */
extern uint8_t calculate_3GPP_CRC(const uint8_t * p_input, uint16_t size);

/********** Test functions **********/


static ble_ad_data_t * get_transaction_start_packet(uint8_t * p_data, uint16_t data_size, uint8_t transaction)
{
    uint8_t start_payload_size = data_size > PROV_START_PDU_PAYLOAD_MAX_LEN ? PROV_START_PDU_PAYLOAD_MAX_LEN : data_size;
    uint8_t segN = ((data_size - start_payload_size) > 0) * (data_size - start_payload_size)/PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN; /*lint !e514 Boolean used in arithmetic */
    trans_data_payload[PROV_ADV_OVERHEAD - 1] = transaction;
    trans_data_payload[PROV_ADV_OVERHEAD] = (segN << 2) | PROV_TRANS_START_OPCODE_WITH_GPCF_FIELD;
    /* Total Length*/
    trans_data_payload[PROV_ADV_OVERHEAD + 2] = data_size & 0xFF;
    trans_data_payload[PROV_ADV_OVERHEAD + 1] = data_size >> 8;
    /* FCS*/
    trans_data_payload[PROV_ADV_OVERHEAD + 3] = calculate_3GPP_CRC(p_data, data_size);
    memcpy(&trans_data_payload[PROV_ADV_OVERHEAD + PROV_START_PDU_HEADER_SIZE ], p_data, start_payload_size);
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_data_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_START_PDU_HEADER_SIZE + start_payload_size;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_transaction_continue_packet(uint8_t * p_data, uint16_t data_size, uint8_t transaction, uint8_t segment)
{
    uint8_t continue_payload_size = data_size > PROV_START_PDU_PAYLOAD_MAX_LEN ? (data_size - PROV_START_PDU_PAYLOAD_MAX_LEN) : 0;
    if (segment == 0 || continue_payload_size <= (segment - 1 ) * PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN)
    {
        return NULL;
    }

    continue_payload_size -= (segment - 1 ) * PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN;
    continue_payload_size = continue_payload_size > PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN ? PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN: continue_payload_size;
    trans_data_payload[PROV_ADV_OVERHEAD - 1] = transaction;
    trans_data_payload[PROV_ADV_OVERHEAD] = (segment << 2) | PROV_TRANS_CONTINUE_OPCODE_WITH_GPCF_FIELD;
    memcpy(&trans_data_payload[PROV_ADV_OVERHEAD + PROV_CONTINUE_PDU_HEADER_SIZE ], p_data + PROV_START_PDU_PAYLOAD_MAX_LEN + (segment - 1)* PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN, continue_payload_size);
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_data_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_CONTINUE_PDU_HEADER_SIZE + continue_payload_size;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_transaction_ack_packet(uint8_t transaction)
{
    trans_ack_payload[PROV_ADV_OVERHEAD - 1] = transaction;
    trans_ack_payload[PROV_ADV_OVERHEAD] = PROV_TRANS_ACK_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) trans_ack_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_TRANS_ACK_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_open_packet(uint8_t * p_uuid)
{
    link_open_payload[PROV_ADV_OVERHEAD] = PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD;
    memcpy(&link_open_payload[PROV_ADV_OVERHEAD+1], p_uuid, NRF_MESH_UUID_SIZE);

    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_open_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_LINK_OPEN_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_ack_packet(void)
{
    link_ack_payload[PROV_ADV_OVERHEAD] = PROV_LINK_ACK_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_ack_payload;
    p_ad_data->length =  1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_LINK_ACK_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static ble_ad_data_t * get_link_close_packet(nrf_mesh_prov_link_close_reason_t close_reason)
{
    link_close_payload[PROV_ADV_OVERHEAD] = PROV_LINK_CLOSE_OPCODE_WITH_GPCF_FIELD;
    link_close_payload[PROV_ADV_OVERHEAD+1] = (uint8_t)close_reason;

    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) link_close_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + PROV_LINK_CLOSE_DATA_SIZE;
    p_ad_data->type = AD_TYPE_PB_ADV;
    return p_ad_data;
}

static void rx_link_open(prov_bearer_t * p_bearer, uint8_t * p_uuid, uint32_t link_id, bool accept)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    ble_ad_data_t *  p_ad_data  = get_link_open_packet(p_uuid);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        advertiser_interval_set_Expect(&p_bearer_adv->advertiser, BEARER_ADV_INT_DEFAULT_MS);
        ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, true);
        prov_cb_link_opened_Expect(p_bearer);
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 1000 + p_bearer_adv->link_timeout);
    }
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    if (accept)
    {
        TEST_ASSERT_EQUAL(1, m_packet.config.repeats);
    }
}

static void rx_link_ack(prov_bearer_t * p_bearer, uint32_t link_id, bool accept)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    ble_ad_data_t * p_ad_data  = get_link_ack_packet();
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        advertiser_flush_Expect(&p_bearer_adv->advertiser);
        advertiser_interval_set_Expect(&p_bearer_adv->advertiser, BEARER_ADV_INT_DEFAULT_MS);
        prov_cb_link_opened_Expect(p_bearer);
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 1000 + p_bearer_adv->link_timeout);
    }
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
}

static void rx_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason, uint32_t link_id,  bool accept)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    ble_ad_data_t * p_ad_data  = get_link_close_packet(close_reason);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        timer_sch_abort_Expect(&p_bearer_adv->link_timeout_event);
        prov_cb_link_closed_Expect(p_bearer, close_reason);
        advertiser_flush_Expect(&p_bearer_adv->advertiser);
    }
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
}

static void tx_link_open(prov_bearer_t * p_bearer, uint8_t * p_uuid, uint32_t link_id)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    p_bearer_adv->link_id = link_id;
    p_bearer_adv->link_timeout = 1000;
    p_bearer_adv->p_next = NULL;

    ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE, true);

    if (p_bearer_adv->instance_state != PROV_BEARER_ADV_INSTANCE_INITIALIZED)
    {
        advertiser_instance_init_ExpectStub(&p_bearer_adv->advertiser, p_bearer_adv->tx_buffer, sizeof(p_bearer_adv->tx_buffer));
    }

    rand_hw_rng_get_Expect((uint8_t*) &p_bearer_adv->link_id, sizeof(p_bearer_adv->link_id));
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 1000 + PROV_BEARER_ADV_LINK_ESTABLISHMENT_LENGTH_US);
    advertiser_interval_set_Expect(&p_bearer_adv->advertiser, PROV_BEARER_ADV_LINK_OPEN_ADVERTISER_INTERVAL_MS);
    advertiser_enable_Expect(&p_bearer_adv->advertiser);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_link_open(p_bearer, p_uuid, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));
    TEST_ASSERT_EQUAL(ADVERTISER_REPEAT_INFINITE, m_packet.config.repeats);
}

static void tx_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, true);

    if (PROV_BEARER_ADV_STATE_LINK_OPENING == p_bearer_adv->state)
    { // Stop ongoing Link Opening process
        advertiser_flush_Expect(&p_bearer_adv->advertiser);
    }

    prov_bearer_adv_link_close(p_bearer, close_reason);
    TEST_ASSERT_EQUAL(PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, m_packet.config.repeats);

    /* Send queue empty in order to move on to link closed state */
    advertiser_flush_Expect(&p_bearer_adv->advertiser);
    prov_cb_link_closed_Expect(p_bearer, close_reason);
    timer_sch_abort_Expect(&p_bearer_adv->link_timeout_event);
    p_bearer_adv->queue_empty_pending = true;
    TEST_ASSERT_NOT_NULL(m_async_cb);
    m_async_cb();
}

static void listen_start(prov_bearer_t * p_bearer)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    if (p_bearer_adv->instance_state != PROV_BEARER_ADV_INSTANCE_INITIALIZED)
    {
        advertiser_instance_init_ExpectStub(&p_bearer_adv->advertiser, p_bearer_adv->tx_buffer, sizeof(p_bearer_adv->tx_buffer));
    }

    advertiser_enable_Expect(&p_bearer_adv->advertiser);
    prov_beacon_unprov_build_ExpectAndReturn(&p_bearer_adv->advertiser, NULL, 0, &m_packet);
    advertiser_interval_set_Expect(&p_bearer_adv->advertiser, NRF_MESH_PROV_BEARER_ADV_UNPROV_BEACON_INTERVAL_MS);
    advertiser_packet_send_Expect(&p_bearer_adv->advertiser, &m_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_listen(p_bearer, NULL, 0, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));
    TEST_ASSERT_EQUAL(ADVERTISER_REPEAT_INFINITE, m_packet.config.repeats);
}

static void listen_stop(prov_bearer_t * p_bearer)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    advertiser_flush_Expect(&p_bearer_adv->advertiser);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, prov_bearer_adv_listen_stop(p_bearer));
}

static void receive_data_ack(prov_bearer_t * p_bearer, uint32_t link_id, bool accept)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    ble_ad_data_t * p_ad_data  = get_transaction_ack_packet(p_bearer_adv->transaction_out);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;
    if (accept)
    {
        timer_now_ExpectAndReturn(1000);
        timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 1000 + p_bearer_adv->link_timeout);
        timer_sch_abort_Expect(&p_bearer_adv->timeout_event);
        advertiser_flush_Expect(&p_bearer_adv->advertiser);
        prov_cb_ack_in_Expect(p_bearer);
    }
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
}

static uint8_t send_data_packet_internals(prov_bearer_t * p_bearer, uint8_t * p_data, uint8_t data_length)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    uint8_t no_segments = 1;
    uint8_t remaninig_data_length = data_length > PROV_START_PDU_PAYLOAD_MAX_LEN ? PROV_START_PDU_PAYLOAD_MAX_LEN : data_length;
    /* Start packet:*/
    ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_START_PDU_HEADER_SIZE + remaninig_data_length, true);
    remaninig_data_length = data_length - remaninig_data_length;
    while (remaninig_data_length > 0)
    {
        no_segments++;
        uint8_t current_chunk_data_length = remaninig_data_length > PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN ? PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN: remaninig_data_length;
        ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_CONTINUE_PDU_HEADER_SIZE + current_chunk_data_length, true);
        remaninig_data_length -= current_chunk_data_length;
    }
    return no_segments;
}

static uint8_t send_data_packet(prov_bearer_t * p_bearer, uint8_t * p_data, uint8_t data_length)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    uint8_t no_segments = send_data_packet_internals(p_bearer, p_data, data_length);
    timer_now_ExpectAndReturn(0);
    timer_now_ExpectAndReturn(0);
    timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 0);
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    timer_sch_reschedule_Expect(&p_bearer_adv->timeout_event, 0);
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    (void) prov_bearer_adv_tx(p_bearer, p_data, data_length);
    return no_segments;
}

static void receive_data(prov_bearer_t * p_bearer, uint8_t * p_data, uint16_t data_length, uint8_t transcation, uint32_t link_id)
{
    nrf_mesh_prov_bearer_adv_t * p_bearer_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);
    /* Can't receive data until a link is established*/
    ble_ad_data_t * p_ad_data  = get_transaction_start_packet(p_data, data_length, transcation);
    p_ad_data->data[3] = link_id & 0xFF;
    p_ad_data->data[2] = (link_id >> 8) & 0xFF;
    p_ad_data->data[1] = (link_id >> 16) & 0xFF;
    p_ad_data->data[0] = (link_id >> 24) & 0xFF;

    ALLOC_AND_TX(&p_bearer_adv->advertiser, PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, true);
    prov_cb_pkt_in_Expect(p_bearer, p_data, data_length);
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&p_bearer_adv->link_timeout_event, 1000 + p_bearer_adv->link_timeout);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);

    uint8_t segment = 1;
    p_ad_data = get_transaction_continue_packet(p_data, data_length, transcation, segment);
    while (NULL != p_ad_data)
    {
        p_ad_data->data[3] = link_id & 0xFF;
        p_ad_data->data[2] = (link_id >> 8) & 0xFF;
        p_ad_data->data[1] = (link_id >> 16) & 0xFF;
        p_ad_data->data[0] = (link_id >> 24) & 0xFF;
        prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
        segment++;
        p_ad_data = get_transaction_continue_packet(p_data, data_length, transcation, segment);
    }

}

void test_link_establish_active(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    memset(&bearer_adv, 0, sizeof(bearer_adv));
    setup_interface(&bearer_adv);

    uint32_t link_id = 0xFEDCBA98;

    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);

    /* Invalid link open rx and tx*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer_adv.prov_bearer, uuid1, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    /* Close link */
    tx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS);
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* RX a packet with LINK_CLOSE command, expect a callback*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback. */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Try to open a link and we fail due to pacman, the state should not change */
    bearer_adv.state = PROV_BEARER_ADV_STATE_IDLE;
    bearer_adv.link_id = link_id;

    if (bearer_adv.instance_state != PROV_BEARER_ADV_INSTANCE_INITIALIZED)
    {
        advertiser_instance_init_ExpectStub(&bearer_adv.advertiser, bearer_adv.tx_buffer, sizeof(bearer_adv.tx_buffer));
    }

    rand_hw_rng_get_Expect((uint8_t*) &bearer_adv.link_id, sizeof(bearer_adv.link_id));
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_OPEN_DATA_SIZE, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, prov_bearer_adv_link_open(&bearer_adv.prov_bearer, uuid1, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));


    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer_adv.prov_bearer, link_id, true);
    /* Invalid link open tx*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer_adv.prov_bearer, uuid1, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));
    /* Invalid listen stop call*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen_stop(&bearer_adv.prov_bearer));
    /* Invalid Link ACK */
    rx_link_ack(&bearer_adv.prov_bearer, link_id, false);
    /* Valid Link Close packet but different link id*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT, link_id + 1, false);
    /* Link CLOSE */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT, link_id, true);
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    rx_link_ack(&bearer_adv.prov_bearer, link_id, true);
    /* A call to the prov_bearer_adv_link_close shall not fail, so if one of the external modules lets us down
       we close without sending and immediately do a callback to the module above.
       *** Technically, this is illegal behaviour, perhaps we need a timer to retry closing? ***/
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, false);
    advertiser_flush_Expect(&bearer_adv.advertiser);
    prov_cb_link_closed_Expect(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    timer_sch_abort_Expect(&bearer_adv.link_timeout_event);
    prov_bearer_adv_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);

    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Close link */
    tx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Close link */
    tx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* RX a packet with LINK_CLOSE command, expect a callback. */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR, link_id, true);
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* RX of unknown close command is acceptable, and this should be forwarded. */
    rx_link_close(&bearer_adv.prov_bearer, (nrf_mesh_prov_link_close_reason_t) 0xff, link_id, true);
    /* Close link with unknown close command is not acceptable, assert */
    TEST_NRF_MESH_ASSERT_EXPECT(prov_bearer_adv_link_close(&bearer_adv.prov_bearer, (nrf_mesh_prov_link_close_reason_t) 0xff));
}

void test_link_establish_passive(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    uint32_t link_id = 0xFFFFFFFF;

    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);
    /* Ignore if by chance we match a close link request to the dummy link_id of 0. */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, 0, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen(&bearer_adv.prov_bearer, NULL, 0, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));
    listen_stop(&bearer_adv.prov_bearer);
    listen_start(&bearer_adv.prov_bearer);

    /* RX a packet with LINK_CLOSE command, do not expect a link closed callback*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, false);

    /* Send a packet with the LINK_OPEN opcode but no uuid, should be ignored. */
    minimal_payload[PROV_ADV_OVERHEAD] = PROV_LINK_OPEN_OPCODE_WITH_GPCF_FIELD;
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) minimal_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + 1;
    p_ad_data->type = AD_TYPE_PB_ADV;
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    /* Send a control packet with the unknown opcode, should be ignored:*/
    minimal_payload[PROV_ADV_OVERHEAD] = 0xFF;
    p_ad_data = (ble_ad_data_t *) minimal_payload;
    p_ad_data->length = 1 /* AD Type */ + PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t) + 1;
    p_ad_data->type = AD_TYPE_PB_ADV;
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    /* RX a packet with LINK_OPEN command using wrong uuid*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid2, link_id, false);

    /* Fail RX of a packet with LINK_OPEN command due to PACKET MANAGER*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    advertiser_interval_set_Expect(&bearer_adv.advertiser, BEARER_ADV_INT_DEFAULT_MS);
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, false);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, false);

    /* State shouldn't have changed due to the failures, so a successful RX should be possible: */

    /* RX a packet with LINK_OPEN command*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, true);

    /* Sending another link open should produce an ack without the callback to higher layers */
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_ACK_DATA_SIZE, true);
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, false);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);


    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_listen(&bearer_adv.prov_bearer, NULL, 0, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_link_open(&bearer_adv.prov_bearer, uuid1, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    /* RX a packet with LINK_CLOSE command, expect a callback*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);

    /* RX a packet with LINK_OPEN command, do not expect a link opened callback*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, false);
}

void test_multiple_bearer(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer1;
    static nrf_mesh_prov_bearer_adv_t bearer2;
    static nrf_mesh_prov_bearer_adv_t bearer3;
    static nrf_mesh_prov_bearer_adv_t bearer4;

    setup_interface(&bearer1);
    setup_interface(&bearer2);
    setup_interface(&bearer3);
    setup_interface(&bearer4);

    uint32_t link_id1 = 0x1;
    uint32_t link_id2 = 0x10;
    uint32_t link_id3 = 0x100;
    uint32_t link_id4 = 0x1000;

    /** Test out of order connection establishment (both as provisioner and provisionee). */
    /* Start listenining on bearer 1. */
    listen_start(&bearer1.prov_bearer);
    /* Open link 2. */
    tx_link_open(&bearer2.prov_bearer, uuid2, link_id2);
    /* Open link 3. */
    tx_link_open(&bearer3.prov_bearer, uuid1, link_id3);
    /* Stop listening on bearer1 before listening on bearer4 (out of order listening is not allowed)*/
    /* Stopping bearer 1 after taking action with bearer 2 and 3 in order to test remove_active_bearer
       function in prov_bearer_adv.c */
    listen_stop(&bearer1.prov_bearer);
    /* Start listenining on bearer 4. */
    listen_start(&bearer4.prov_bearer);
    /* Send link open on bearer1: doing this after starting to listen on bearer 4 in order to test
       get_bearer_from_state  function in prov_bearer_adv.c */
    tx_link_open(&bearer1.prov_bearer, uuid2, link_id1);

    /* LINK_OPEN for bearer4*/
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer4.prov_bearer, uuid1, link_id4, true);
    /* Link ACK for bearer 1, 2 and 3*/
    rx_link_ack(&bearer3.prov_bearer, link_id3, true);
    /* Link ACK */
    rx_link_ack(&bearer1.prov_bearer, link_id1, true);
    /* Link ACK */
    rx_link_ack(&bearer2.prov_bearer, link_id2, true);

    rx_link_close(&bearer1.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id1, true);
    rx_link_close(&bearer2.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id2, true);
    rx_link_close(&bearer3.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id3, true);
    rx_link_close(&bearer4.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id4, true);
}

void test_packet_send(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    uint32_t link_id = 0;
    uint8_t curr_transcation = PROVISIONER_TRANSACTION_START_VALUE;
    /* Can't send data until a link is established*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_tx(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH));

    /** Establish a connection **/
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer_adv.prov_bearer, link_id, true);

    /* Can't send data larger than PROV_PAYLOAD_MAX_LENGTH */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, prov_bearer_adv_tx(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH+1));

    /* Send 1 byte long data */
    uint8_t no_segments = send_data_packet(&bearer_adv.prov_bearer, data, 1);
    /* transcation no starts with 0 when in provisioner role (@tagMeshSp section 5.2.1) */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Can't send data without receiving an ack or timeout for the previous one*/
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, prov_bearer_adv_tx(&bearer_adv.prov_bearer, data, 1));
    /* Send in an ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;

    /* Send the largest packet that can fit into a single segment. */
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;

    /* Send a large enough packet to split it in two with only 1 byte in the continue packet. */
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(2, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;

    /* Send a large packet that barely fits two packets*/
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(2, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;

    /* Send a large enough packet to split it in three with only 1 byte in the last continue packet. */
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN + 1);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* Send in the expected ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;

    /* Send packets until rollover + 1: */
    while (curr_transcation != 1)
    {
        no_segments = send_data_packet(&bearer_adv.prov_bearer, data, 1);
        receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
        curr_transcation = (curr_transcation + 1) & 0x7f; // rolls over at 0x7f
        TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    }

    /* Send the largest possible data packet */
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* No ack, send a timeout */
    (void) send_data_packet_internals(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH);
    bearer_adv.timeout_event.cb(0, &bearer_adv);
    /* Transcation no must remain the same */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* Send in an ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);

    /* Close link*/
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
}

void test_packet_send_abnormal(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    uint32_t link_id = 0x12345678;
    uint8_t curr_transcation = PROVISIONEE_TRANSACTION_START_VALUE;
    /* Can't send data until a link is established*/
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, prov_bearer_adv_tx(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH));

    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);
    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, true);

    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    uint8_t no_segments = send_data_packet(&bearer_adv.prov_bearer, data, 1);
    TEST_ASSERT_EQUAL(1, no_segments);

    /* Send in ack with wrong transcation value, it should be ignored */
    ble_ad_data_t * p_ack_packet = get_transaction_ack_packet(curr_transcation-1);
    prov_bearer_adv_packet_in(p_ack_packet->data, p_ack_packet->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    /* LINK_OPEN and LINK_ACK control messages should also be ignored */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, false);
    rx_link_ack(&bearer_adv.prov_bearer, link_id, false);
    /* Send in an ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);
    curr_transcation++;
    /* Ignore if not expecting an ack */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, false);

    /* Send the largest possible data packet */
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH);
    /* transcation no must be incremented for each new transaction */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_out);
    TEST_ASSERT_EQUAL(3, no_segments);
    /* No ACK */
    /* Time out cb when the link time out has passed: this should produce a LINK_CLOSE message*/
    /* First attempt to TX fails*/
    timestamp_t timeout_timestamp = bearer_adv.sar_timeout + 1;
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, false);
    bearer_adv.timeout_event.cb(bearer_adv.sar_timeout + 1, &bearer_adv);

    /* timeout interval should be larger than 0 */
    TEST_ASSERT(bearer_adv.timeout_event.interval > 0);
    /* Timeout value should be updated to NOW to ensure a timeout trigger for the next retry cb. */
    TEST_ASSERT_EQUAL(timeout_timestamp, bearer_adv.sar_timeout);
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_LINK_CLOSE_DATA_SIZE, true);
    bearer_adv.timeout_event.cb(bearer_adv.sar_timeout+1, &bearer_adv);
    TEST_ASSERT_EQUAL(PROV_BEARER_ADV_UNACKED_REPEAT_COUNT, m_packet.config.repeats);

    /* Send queue empty in order to move on to link closed state */
    timer_sch_abort_Expect(&bearer_adv.timeout_event);
    advertiser_flush_Expect(&bearer_adv.advertiser);
    prov_cb_link_closed_Expect(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
    timer_sch_abort_Expect(&bearer_adv.link_timeout_event);
    bearer_adv.queue_empty_pending = true;
    TEST_ASSERT_NOT_NULL(m_async_cb);
    m_async_cb();
    /* Receive a link close when in TX state */
    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);
    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, true);
    no_segments = send_data_packet(&bearer_adv.prov_bearer, data, 1);
    TEST_ASSERT_EQUAL(1, no_segments);
    /* Close link*/
    timer_sch_abort_Expect(&bearer_adv.timeout_event);
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
}

void test_packet_receive(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    uint32_t link_id = 0xDAFA;
    uint8_t curr_transcation = PROVISIONEE_TRANSACTION_START_VALUE;
    /** Establish a connection **/
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Link ACK */
    rx_link_ack(&bearer_adv.prov_bearer, link_id, true);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send the largest packet that can fit into a single segment. */
    receive_data(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send a large enough packet to split it in two with only 1 byte in the continue packet. */
    receive_data(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send a large packet that barely fits two packets*/
    receive_data(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send a large enough packet to split it in three with only 1 byte in the last continue packet. */
    receive_data(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + PROV_CONTINUE_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send the largest possible data packet */
    receive_data(&bearer_adv.prov_bearer, data, PROV_PAYLOAD_MAX_LENGTH, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send packets until the transaction number wraps around */
    while (curr_transcation != PROVISIONEE_TRANSACTION_START_VALUE)
    {
        receive_data(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN, curr_transcation++, link_id);
        if (curr_transcation == 0x00)
        {
            /* Should roll over to start value */
            curr_transcation = PROVISIONEE_TRANSACTION_START_VALUE;
        }
        TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    }

    /* Re send the old packet after the wrap around and expect a response but no callback!*/
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, true);
    prov_cb_pkt_in_Expect(&bearer_adv.prov_bearer, data, 1);
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&bearer_adv.link_timeout_event, 1000 + bearer_adv.link_timeout);
    ble_ad_data_t * p_ad_data = get_transaction_start_packet(data, 1, curr_transcation);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);
    TEST_ASSERT_EQUAL(curr_transcation+1, bearer_adv.transaction_in);

    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);

    /* Send unknown control packet. Nothing should happen*/
    p_ad_data = get_link_open_packet(uuid1);
    p_ad_data->data[PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t)] = 0xFF;
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);
    /* Send unknown control packet. Nothing should happen*/
    p_ad_data = get_link_open_packet(uuid1);
    p_ad_data->data[PROV_ADV_OVERHEAD - sizeof(ble_ad_data_t)] = 0xFF;
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    listen_stop(&bearer_adv.prov_bearer);
}

void test_packet_receive_abnormal(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    uint32_t link_id = 0xBACA;
    uint8_t curr_transcation = PROVISIONER_TRANSACTION_START_VALUE;
    /* Can't receive data until a link is established*/
    ble_ad_data_t * p_ad_data = get_transaction_start_packet(data,1,0);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);

    /* Can't receive data until a link is established*/
    p_ad_data = get_transaction_start_packet(data,1,0);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);

    /* LINK_OPEN */
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(uuid1);
    rx_link_open(&bearer_adv.prov_bearer, uuid1, link_id, true);

    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    /* Send 1 byte long data. */
    receive_data(&bearer_adv.prov_bearer, data, 1, curr_transcation++, link_id);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send the same data. */
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, true);
    p_ad_data = get_transaction_start_packet(data, 1, curr_transcation-1);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);

    /* Re-send different data with the same transcation value, it won't care and still send an ack and ignore contents. */
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, true);
    p_ad_data = get_transaction_start_packet(&data[10], 10, curr_transcation-1);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);


    /* Increment transaction number with more than 1. Should be ignored, and the next expected
     * transaction number should stay unchanged. */
    uint8_t invalid_transaction_number = curr_transcation + 1;
    p_ad_data = get_transaction_start_packet(&data[10], 10, invalid_transaction_number);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Send new data with wrong fcs (CRC)*/
    p_ad_data = get_transaction_start_packet(&data[10], 10, curr_transcation);
    trans_data_payload[PROV_ADV_OVERHEAD + 3]++;
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /***** Send a new segmented packet and in between send an old packet *****/
    /* First segment of the new packet: */
    p_ad_data = get_transaction_start_packet(data, PROV_START_PDU_PAYLOAD_MAX_LEN +1, curr_transcation);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    /* transaction has not been updated yet*/
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    /* Re send an old packet but do not expect a response*/
    p_ad_data = get_transaction_start_packet(data, 1, curr_transcation-1);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);
    /* Send the rest of the new packet and expect an ack */
    ALLOC_AND_TX(&bearer_adv.advertiser, PROV_ADV_OVERHEAD + PROV_TRANS_ACK_DATA_SIZE, true);
    prov_cb_pkt_in_Expect(&bearer_adv.prov_bearer, data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1);
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(&bearer_adv.link_timeout_event, 1000 + bearer_adv.link_timeout);
    p_ad_data = get_transaction_continue_packet(data, PROV_START_PDU_PAYLOAD_MAX_LEN + 1, curr_transcation++, 1);
    prov_bearer_adv_packet_in(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, &m_dummy_metadata);
    TEST_ASSERT_EQUAL(1, m_packet.config.repeats);

    /* transaction number is now updated */
    TEST_ASSERT_EQUAL(curr_transcation, bearer_adv.transaction_in);

    /* Close the link to remove the internal reference to the bearer. */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);
}

void test_interface_return(void)
{
    nrf_mesh_prov_bearer_adv_t bearer;
    memset(&bearer, 0, sizeof(bearer));
    const prov_bearer_t * p_bearer  = nrf_mesh_prov_bearer_adv_interface_get(&bearer);
    const prov_bearer_interface_t * p_interface = p_bearer->p_interface;
    TEST_ASSERT_NOT_NULL(p_interface);
    TEST_ASSERT_NOT_NULL(p_interface->tx);
    TEST_ASSERT_NOT_NULL(p_interface->listen_start);
    TEST_ASSERT_NOT_NULL(p_interface->listen_stop);
    TEST_ASSERT_NOT_NULL(p_interface->link_open);
    TEST_ASSERT_NOT_NULL(p_interface->link_close);
    TEST_ASSERT_EQUAL(NRF_MESH_PROV_BEARER_ADV, p_bearer->bearer_type);
    TEST_ASSERT_NOT_NULL(m_async_cb);
}

void test_packet_fcs_sample_check(void)
{
    const uint8_t sample_data_8_7_3[] = {0x00, 0x00};
    TEST_ASSERT_EQUAL(0x14, calculate_3GPP_CRC(sample_data_8_7_3, sizeof(sample_data_8_7_3)));

    const uint8_t sample_data_8_7_4[] = {0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(0xD6, calculate_3GPP_CRC(sample_data_8_7_4, sizeof(sample_data_8_7_4)));

    const uint8_t sample_data_8_7_5[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(0x64, calculate_3GPP_CRC(sample_data_8_7_5, sizeof(sample_data_8_7_5)));

    const uint8_t sample_data_8_7_6[] =
    {
        0x03, 0x2c, 0x31, 0xa4, 0x7b, 0x57, 0x79, 0x80, 0x9e, 0xf4, 0x4c, 0xb5, 0xea, 0xaf, 0x5c, 0x3e,
        0x43, 0xd5, 0xf8, 0xfa, 0xad, 0x4a, 0x87, 0x94, 0xcb, 0x98, 0x7e, 0x9b, 0x03, 0x74, 0x5c, 0x78,
        0xdd, 0x91, 0x95, 0x12, 0x18, 0x38, 0x98, 0xdf, 0xbe, 0xcd, 0x52, 0xe2, 0x40, 0x8e, 0x43, 0x87,
        0x1f, 0xd0, 0x21, 0x10, 0x91, 0x17, 0xbd, 0x3e, 0xd4, 0xea, 0xf8, 0x43, 0x77, 0x43, 0x71, 0x5d,
        0x4f
    };
    TEST_ASSERT_EQUAL(0xD1, calculate_3GPP_CRC(sample_data_8_7_6, sizeof(sample_data_8_7_6)));

    const uint8_t sample_data_8_7_7[] =
    {
        0x03, 0xf4, 0x65, 0xe4, 0x3f, 0xf2, 0x3d, 0x3f, 0x1b, 0x9d, 0xc7, 0xdf, 0xc0, 0x4d, 0xa8, 0x75,
        0x81, 0x84, 0xdb, 0xc9, 0x66, 0x20, 0x47, 0x96, 0xec, 0xcf, 0x0d, 0x6c, 0xf5, 0xe1, 0x65, 0x00,
        0xcc, 0x02, 0x01, 0xd0, 0x48, 0xbc, 0xbb, 0xd8, 0x99, 0xee, 0xef, 0xc4, 0x24, 0x16, 0x4e, 0x33,
        0xc2, 0x01, 0xc2, 0xb0, 0x10, 0xca, 0x6b, 0x4d, 0x43, 0xa8, 0xa1, 0x55, 0xca, 0xd8, 0xec, 0xb2,
        0x79
    };
    TEST_ASSERT_EQUAL(0x10, calculate_3GPP_CRC(sample_data_8_7_7, sizeof(sample_data_8_7_7)));

    const uint8_t sample_data_8_7_8[] = {0x05, 0xb3, 0x8a, 0x11, 0x4d, 0xfd, 0xca, 0x1f, 0xe1, 0x53, 0xbd, 0x2c, 0x1e, 0x0d, 0xc4, 0x6a, 0xc2};
     TEST_ASSERT_EQUAL(0xD1, calculate_3GPP_CRC(sample_data_8_7_8, sizeof(sample_data_8_7_8)));

    const uint8_t sample_data_8_7_9[] = {0x05, 0xee, 0xba, 0x52, 0x1c, 0x19, 0x6b, 0x52, 0xcc, 0x2e, 0x37, 0xaa, 0x40, 0x32, 0x9f, 0x55, 0x4e};
     TEST_ASSERT_EQUAL(0xEC, calculate_3GPP_CRC(sample_data_8_7_9, sizeof(sample_data_8_7_9)));

    const uint8_t sample_data_8_7_10[] = {0x06, 0x8b, 0x19, 0xac, 0x31, 0xd5, 0x8b, 0x12, 0x4c, 0x94, 0x62, 0x09, 0xb5, 0xdb, 0x10, 0x21, 0xb9};
    TEST_ASSERT_EQUAL(0xD3, calculate_3GPP_CRC(sample_data_8_7_10, sizeof(sample_data_8_7_10)));

    const uint8_t sample_data_8_7_11[] = {0x06, 0x55, 0xa2, 0xa2, 0xbc, 0xa0, 0x4c, 0xd3, 0x2f, 0xf6, 0xf3, 0x46, 0xbd, 0x0a, 0x0c, 0x1a, 0x3a};
    TEST_ASSERT_EQUAL(0x59, calculate_3GPP_CRC(sample_data_8_7_11, sizeof(sample_data_8_7_11)));

    const uint8_t sample_data_8_7_12[] =
    {
        0x07, 0xd0, 0xbd, 0x7f, 0x4a, 0x89, 0xa2, 0xff, 0x62, 0x22, 0xaf, 0x59, 0xa9, 0x0a, 0x60, 0xad,
        0x58, 0xac, 0xfe, 0x31, 0x23, 0x35, 0x6f, 0x5c, 0xec, 0x29, 0x73, 0xe0, 0xec, 0x50, 0x78, 0x3b,
        0x10, 0xc7
    };
    TEST_ASSERT_EQUAL(0x8B, calculate_3GPP_CRC(sample_data_8_7_12, sizeof(sample_data_8_7_12)));

    const uint8_t sample_data_8_7_13[] = {0x08};
    TEST_ASSERT_EQUAL(0x3E, calculate_3GPP_CRC(sample_data_8_7_13, sizeof(sample_data_8_7_13)));
}

void test_tx_complete(void)
{
    /* Make bearer static so linked list internally won't be corrupted across tests. */
    static nrf_mesh_prov_bearer_adv_t bearer_adv;
    setup_interface(&bearer_adv);

    prov_bearer_t * p_bearer = nrf_mesh_prov_bearer_adv_interface_get(&bearer_adv);
    prov_bearer_adv_tx = p_bearer->p_interface->tx;
    prov_bearer_adv_listen = p_bearer->p_interface->listen_start;
    prov_bearer_adv_listen_stop = p_bearer->p_interface->listen_stop;
    prov_bearer_adv_link_open = p_bearer->p_interface->link_open;
    prov_bearer_adv_link_close = p_bearer->p_interface->link_close;

    p_bearer->p_parent = PARENT_POINTER;
    p_bearer->p_callbacks = &m_callbacks;
    uint32_t link_id = 0;

    /** Establish a connection **/
    /* Open link. */
    tx_link_open(&bearer_adv.prov_bearer, uuid1, link_id);
    /* Sent one packet: */
    TEST_ASSERT_EQUAL(1, bearer_adv.last_token);

    TEST_ASSERT_NOT_NULL(m_tx_complete_cb);
    bearer_event_flag_set_Expect(ASYNC_FLAG);
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
    TEST_ASSERT_TRUE(bearer_adv.queue_empty_pending);

    TEST_ASSERT_NOT_NULL(m_async_cb);
    m_async_cb();
    TEST_ASSERT_FALSE(bearer_adv.queue_empty_pending);

    rx_link_ack(&bearer_adv.prov_bearer, link_id, true);

    /* Send 3 segment data */
    uint32_t old_last_token = bearer_adv.last_token;
    uint8_t no_segments = send_data_packet(&bearer_adv.prov_bearer, data, PROV_PDU_MAX_LENGTH);
    TEST_ASSERT_EQUAL(3, no_segments);
    TEST_ASSERT_EQUAL(old_last_token + 3, bearer_adv.last_token);
    for (uint32_t i = 0; i < 2; ++i)
    {
        /* The token increase once per TX, mirror these tokens to mimic tx complete on each packet. */
        m_packet.token = old_last_token + 1 + i;
        m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
        TEST_ASSERT_FALSE(bearer_adv.queue_empty_pending);
    }
    bearer_event_flag_set_Expect(ASYNC_FLAG);
    m_packet.token = bearer_adv.last_token;
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
    TEST_ASSERT_TRUE(bearer_adv.queue_empty_pending);

    /* Receive an ack on the data packet so we can proceed the test */
    receive_data_ack(&bearer_adv.prov_bearer, link_id, true);

    /* Close the link to remove the internal reference to the bearer. */
    rx_link_close(&bearer_adv.prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS, link_id, true);

    /* Start listenining. */
    listen_start(&bearer_adv.prov_bearer);
    /* Should not mark the beacon as a pending tx packet (token should stay at initial value). */
    TEST_ASSERT_EQUAL(0, bearer_adv.last_token);
    bearer_adv.queue_empty_pending = false;

    nrf_mesh_prov_bearer_adv_t bearer_copy = bearer_adv;
    setup_interface(&bearer_copy);
    /* We'll get a TX complete callback for every transmitted beacon, but should
     * ignore them all. */
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);
    m_tx_complete_cb(&bearer_adv.advertiser, m_packet.token, 0);

    TEST_ASSERT_EQUAL_MEMORY(&bearer_copy, &bearer_adv, sizeof(bearer_adv));

    /* Getting an unprompted async process call shouldn't do anything */
    m_async_cb();
    TEST_ASSERT_EQUAL_MEMORY(&bearer_copy, &bearer_adv, sizeof(bearer_adv));
}
