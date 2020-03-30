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

#include <stdint.h>
#include <stdbool.h>

#include "test_assert.h"
#include "mesh_gatt.h"

#include "ble_hci.h"
#include "ble.h"
#include "packet_buffer.h"
#include "ble_gatts_mock.h"
#include "ble_gap_mock.h"

#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "bearer_event_mock.h"
#include "utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TX_TOKEN 0xDEADCAFE

#define FIVE_HEX_BYTES 0x01, 0x02, 0x03, 0x04, 0x05

#define SAMPLE_DATA_SEGMENT_1 0x03, 0x2c, 0x31, 0xa4, 0x7b, 0x57, 0x79, 0x80, 0x9e, 0xf4, 0x4c, 0xb5, 0xea, 0xaf, 0x5c, 0x3e, 0x43, 0xd5, 0xf8
#define SAMPLE_DATA_SEGMENT_2 0xfa, 0xad, 0x4a, 0x87, 0x94, 0xcb, 0x98, 0x7e, 0x9b, 0x03, 0x74, 0x5c, 0x78, 0xdd, 0x91, 0x95, 0x12, 0x18, 0x38
#define SAMPLE_DATA_SEGMENT_3 0x98, 0xdf, 0xbe, 0xcd, 0x52, 0xe2, 0x40, 0x8e, 0x43, 0x87, 0x1f, 0xd0, 0x21, 0x10, 0x91, 0x17, 0xbd, 0x3e, 0xd4
#define SAMPLE_DATA_SEGMENT_4 0xea, 0xf8, 0x43, 0x77, 0x43, 0x71, 0x5d, 0x4f

typedef struct
{
    uint8_t length;
    uint8_t segment[];
} pdu_segment_t;

/*******************************************************************************
 * Static globals
 ******************************************************************************/

mesh_gatt_t m_gatt;
static mesh_gatt_uuids_t uuids;
static uint16_t handle = 23;

static packet_buffer_t m_pdu_buffer;
static uint8_t m_pdu_buffer_raw[256];
static bool m_tx_complete_evt_expect;
static bool m_connected_evt_expect;
static bool m_disconnected_evt_expect;
static bool m_tx_ready_evt_expect;
static uint32_t m_hvx_return;

static bool m_flag_added;
static bearer_event_flag_t m_flag;
static bearer_event_flag_callback_t mp_flag_callback;
static uint8_t m_flag_set_counter;
/*******************************************************************************
 * Helper functions / macros
 ******************************************************************************/

#define EXPECT_PDU(...)                                                 \
    do                                                                  \
    {                                                                   \
        uint8_t pdu[] = __VA_ARGS__;                                    \
        packet_buffer_packet_t * p_packet;                              \
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&m_pdu_buffer, &p_packet, sizeof(pdu) + 1)); \
        p_packet->packet[0] = sizeof(pdu);                              \
        memcpy(&p_packet->packet[1], pdu, sizeof(pdu));                 \
        packet_buffer_commit(&m_pdu_buffer, p_packet, sizeof(pdu) + 1); \
    } while (0)

#define RECEIVE_PDU(...)                                                \
    do                                                                  \
    {   uint8_t pdu[] = __VA_ARGS__;                                    \
        uint8_t evt_buf[sizeof(ble_evt_t) + sizeof(pdu)];               \
        ble_evt_t * p_evt = (ble_evt_t *) evt_buf;                      \
        memset(p_evt, 0, sizeof(evt_buf));                              \
        p_evt->header.evt_id = BLE_GATTS_EVT_WRITE;                     \
        p_evt->header.evt_len = sizeof(ble_gatts_evt_write_t) + sizeof(ble_evt_hdr_t) + sizeof(pdu) - 1; \
        p_evt->evt.gatts_evt.conn_handle = 0;                           \
        p_evt->evt.gatts_evt.params.write.handle = m_gatt.handles.rx.value_handle; \
        p_evt->evt.gatts_evt.params.write.len = sizeof(pdu);            \
        memcpy(p_evt->evt.gatts_evt.params.write.data, pdu, sizeof(pdu)); \
        /* Check validity of message before expecting timer behavior.*/ \
        if (is_valid_pdu_type(pdu[0]) && sizeof(pdu) > 1)               \
        {                                                               \
            if ((pdu[0] >> 6 == 1 || pdu[0] >> 6 == 2))                 \
            {                                                           \
                timer_now_ExpectAndReturn(0);                           \
                timer_sch_reschedule_Expect(NULL, MESH_GATT_RX_SAR_TIMEOUT_US); \
                timer_sch_reschedule_IgnoreArg_p_timer_evt();           \
            }                                                           \
            else if (pdu[0] >> 6 == 3)                                  \
            {                                                           \
                timer_sch_abort_Ignore();                               \
            }                                                           \
        }                                                               \
        mesh_gatt_on_ble_evt(p_evt, &m_gatt);                           \
    } while (0);


static bool is_valid_pdu_type(uint8_t header)
{
    /* See @tagMeshSp table 6.3, page 261. */
    return (header & 0x3f) < 5;
}

static void rx_timeout_event_trigger(uint16_t conn_index)
{
    m_gatt.connections[conn_index].rx.timeout_event.cb(0, &m_gatt.connections[conn_index]);
}

static void tx_complete_evt_send(void)
{
    ble_evt_t ble_evt;
    ble_evt.header.evt_id = BLE_GATTS_EVT_HVN_TX_COMPLETE;
    ble_evt.header.evt_len = sizeof(ble_gatts_evt_hvn_tx_complete_t);
    ble_evt.evt.gatts_evt.conn_handle = 0;
    ble_evt.evt.gatts_evt.params.hvn_tx_complete.count = 1;
    mesh_gatt_on_ble_evt(&ble_evt, &m_gatt);
}

static void expected_pdu_check(const uint8_t * p_pdu, uint16_t length)
{
    TEST_ASSERT_MESSAGE(packet_buffer_can_pop(&m_pdu_buffer), "Could not pop from expected PDU buffer");
    packet_buffer_packet_t * p_packet = NULL;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&m_pdu_buffer, &p_packet));
    const pdu_segment_t * p_segment = (const pdu_segment_t *) p_packet->packet;
    TEST_ASSERT_EQUAL(p_segment->length, length);
    TEST_ASSERT_EQUAL_MEMORY(p_segment->segment, p_pdu, length);
    packet_buffer_free(&m_pdu_buffer, p_packet);
}

static void tx_complete_evt_expect(void)
{
    TEST_ASSERT_MESSAGE(!m_tx_complete_evt_expect, "Previously expected TX_COMPLETE not handled");
    m_tx_complete_evt_expect = true;
}

static void change_effective_mtu(uint16_t conn_handle, uint16_t mtu)
{
    ble_evt_t ble_evt;
    memset(&ble_evt, 0, sizeof(ble_evt_t));

    mtu = MIN(mtu + 3, MESH_GATT_MTU_SIZE_MAX);
    ble_evt.header.evt_id = BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST;
    ble_evt.evt.gatts_evt.conn_handle = conn_handle;

    ble_evt.evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu = mtu;
    sd_ble_gatts_exchange_mtu_reply_ExpectAndReturn(conn_handle, mtu, NRF_SUCCESS);
    mesh_gatt_on_ble_evt(&ble_evt, m_gatt.p_context);
}

static void connected_evt_expect(void)
{
    TEST_ASSERT(!m_connected_evt_expect);
    m_connected_evt_expect = true;
}

static void disconnected_evt_expect(void)
{
    TEST_ASSERT(!m_disconnected_evt_expect);
    timer_sch_abort_Ignore();
    m_disconnected_evt_expect = true;
}

static void disconnect_expect(uint16_t conn_handle)
{
    timer_sch_abort_Ignore();
    sd_ble_gap_disconnect_ExpectAndReturn(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION, NRF_SUCCESS);
}

static void tx_ready_evt_expect(void)
{
    TEST_ASSERT(!m_tx_ready_evt_expect);
    m_tx_ready_evt_expect = true;
}

static void connect(uint16_t conn_handle)
{
    ble_evt_t ble_evt;
    memset(&ble_evt, 0, sizeof(ble_evt_t));
    ble_evt.header.evt_id = BLE_GAP_EVT_CONNECTED;
    ble_evt.evt.gap_evt.conn_handle = conn_handle;
    mesh_gatt_on_ble_evt(&ble_evt, m_gatt.p_context);
}

static void disconnect(uint16_t conn_handle)
{
    ble_evt_t ble_evt;
    memset(&ble_evt, 0, sizeof(ble_evt_t));
    ble_evt.header.evt_id = BLE_GAP_EVT_DISCONNECTED;
    ble_evt.evt.gap_evt.conn_handle = conn_handle;
    mesh_gatt_on_ble_evt(&ble_evt, m_gatt.p_context);
}

/*******************************************************************************
 * Callback handlers
 ******************************************************************************/

static uint32_t sd_ble_gatts_hvx_cb(uint16_t conn_handle, ble_gatts_hvx_params_t const * p_hvx_params, int num_calls)
{
    expected_pdu_check(p_hvx_params->p_data, *p_hvx_params->p_len);
    uint32_t status = m_hvx_return;
    // reset the status:
    m_hvx_return = NRF_SUCCESS;
    return status;
}

static void m_gatt_evt_handler(const mesh_gatt_evt_t * p_evt, void * p_context)
{
    switch (p_evt->type)
    {
        case MESH_GATT_EVT_TYPE_TX_COMPLETE:
            TEST_ASSERT_MESSAGE(m_tx_complete_evt_expect, "Got unexpected TX_COMPLETE");
            TEST_ASSERT_EQUAL(TX_TOKEN, p_evt->params.tx_complete.token);
            m_tx_complete_evt_expect = false;
            break;

        case MESH_GATT_EVT_TYPE_RX:
            /* TODO: Check the type of PDU as well.  */
            expected_pdu_check(p_evt->params.rx.p_data, p_evt->params.rx.length);
            break;

        case MESH_GATT_EVT_TYPE_ADV_TIMEOUT:
            TEST_FAIL_MESSAGE("Unhandled event ADV_TIMEOUT");
            break;

        case MESH_GATT_EVT_TYPE_CONNECTED:
            TEST_ASSERT_MESSAGE(m_connected_evt_expect, "Got unexpected CONNECTED");
            m_connected_evt_expect = false;
            break;

        case MESH_GATT_EVT_TYPE_DISCONNECTED:
            TEST_ASSERT_MESSAGE(m_disconnected_evt_expect, "Got unexpected DISCCONNECTED");
            m_disconnected_evt_expect = false;
            break;

        case MESH_GATT_EVT_TYPE_TX_READY:
            TEST_ASSERT_MESSAGE(m_tx_ready_evt_expect, "Got unexpected TX_READY");
            m_tx_ready_evt_expect = false;
            break;

        default:
        {
            char buf[64];
            sprintf(buf, "Unexpected event %d", p_evt->type);
            TEST_FAIL_MESSAGE(buf);
            break;
        }

    }
}

static bearer_event_flag_t bearer_event_flag_add_mock(bearer_event_flag_callback_t callback, int n)
{
    TEST_ASSERT_FALSE(m_flag_added);

    if (!m_flag_added)
    {
        mp_flag_callback = callback;
        m_flag_added = true;
    }

    return m_flag;
}

static void helper_bearer_event_trigger(void)
{
    TEST_ASSERT_TRUE(m_flag_set_counter > 0);
    TEST_ASSERT_NOT_NULL(mp_flag_callback);
    TEST_ASSERT_TRUE(mp_flag_callback());
    m_flag_set_counter--;
}

static void helper_bearer_event_flag_set_expect(bearer_event_flag_t flag)
{
    bearer_event_flag_set_Expect(flag);
    TEST_ASSERT_EQUAL(m_flag, flag);
    m_flag_set_counter++;
}
/*******************************************************************************
 * Test setup/teardown
 ******************************************************************************/

void setUp(void)
{
    packet_buffer_init(&m_pdu_buffer, m_pdu_buffer_raw, sizeof(m_pdu_buffer_raw));
    timer_mock_Init();
    timer_scheduler_mock_Init();
    ble_gatts_mock_Init();
    ble_gap_mock_Init();
    bearer_event_mock_Init();
    m_hvx_return = NRF_SUCCESS;
}

void tearDown(void)
{
    sd_ble_gatts_hvx_StubWithCallback(NULL);
    TEST_ASSERT_MESSAGE(!m_tx_complete_evt_expect, "Did not get expected TX_COMPLETE");
    TEST_ASSERT_MESSAGE(!m_connected_evt_expect, "Did not get expected CONNECTED");
    TEST_ASSERT_MESSAGE(!m_disconnected_evt_expect, "Did not get expected DISCONNECTED");
    TEST_ASSERT_MESSAGE(!m_tx_ready_evt_expect, "Did not get expected TX_READY");
    TEST_ASSERT_EQUAL_MESSAGE(m_pdu_buffer.head,
                              m_pdu_buffer.tail,
                              "Not all expected PDUs sent.");

    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    ble_gatts_mock_Verify();
    ble_gatts_mock_Destroy();
    ble_gap_mock_Verify();
    ble_gap_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
}

/*******************************************************************************
 * Test cases
 ******************************************************************************/

void test_gatt_init(void)
{
    const uint16_t SERVICE_UUID = 0x1617;
    const uint16_t CHAR_TX_UUID = 0x1827;
    const uint16_t CHAR_RX_UUID = 0x1983;

    uuids.service = SERVICE_UUID;
    uuids.tx_char = CHAR_TX_UUID;
    uuids.rx_char = CHAR_RX_UUID;


    sd_ble_gatts_service_add_ExpectAndReturn(BLE_GATTS_SRVC_TYPE_PRIMARY, NULL, NULL, NRF_SUCCESS);
    sd_ble_gatts_service_add_IgnoreArg_p_uuid();
    sd_ble_gatts_service_add_IgnoreArg_p_handle();
    sd_ble_gatts_service_add_ReturnThruPtr_p_handle(&handle);

    sd_ble_gatts_characteristic_add_ExpectAndReturn(handle, NULL, NULL, NULL, NRF_SUCCESS);
    sd_ble_gatts_characteristic_add_IgnoreArg_p_attr_char_value();
    sd_ble_gatts_characteristic_add_IgnoreArg_p_char_md();
    sd_ble_gatts_characteristic_add_IgnoreArg_p_handles();

    sd_ble_gatts_characteristic_add_ExpectAndReturn(handle, NULL, NULL, NULL, NRF_SUCCESS);
    sd_ble_gatts_characteristic_add_IgnoreArg_p_attr_char_value();
    sd_ble_gatts_characteristic_add_IgnoreArg_p_char_md();
    sd_ble_gatts_characteristic_add_IgnoreArg_p_handles();

    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_mock);

    mesh_gatt_init(&uuids, m_gatt_evt_handler, &m_gatt);
}

void test_connect_event(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);
}

void test_disconnect_event(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    disconnected_evt_expect();
    disconnect(0);
}

void test_alloc(void)
{
    test_gatt_init();
    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, MESH_GATT_PROXY_PDU_MAX_SIZE, TX_TOKEN);
    TEST_ASSERT_NULL(p_packet);

    connected_evt_expect();
    connect(0);

    p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, MESH_GATT_PROXY_PDU_MAX_SIZE, TX_TOKEN);
    TEST_ASSERT_NOT_NULL(p_packet);

    TEST_NRF_MESH_ASSERT_EXPECT(mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, MESH_GATT_PROXY_PDU_MAX_SIZE + 1, TX_TOKEN));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_gatt_packet_alloc(NRF_SDH_BLE_TOTAL_LINK_COUNT, MESH_GATT_PDU_TYPE_PROV_PDU, MESH_GATT_PROXY_PDU_MAX_SIZE, TX_TOKEN));
}

void test_discard(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, MESH_GATT_PROXY_PDU_MAX_SIZE, TX_TOKEN);
    TEST_ASSERT_NOT_NULL(p_packet);

    mesh_gatt_packet_discard(0, p_packet);

    TEST_NRF_MESH_ASSERT_EXPECT(mesh_gatt_packet_discard(0, NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_gatt_packet_discard(1, p_packet));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_gatt_packet_discard(NRF_SDH_BLE_TOTAL_LINK_COUNT, p_packet));
}

void test_send_segmented_sample_data(void)
{
    /* The default MTU = 23-3 = 20
     * => To send the maxmium payload of 65+1 bytes, we must send 4 GATT PDUs. */
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    /* @tagMeshSp section 8.8, PB-GATT sample data. */
    const uint8_t PROXY_PDU[65] = {SAMPLE_DATA_SEGMENT_1,
                                   SAMPLE_DATA_SEGMENT_2,
                                   SAMPLE_DATA_SEGMENT_3,
                                   SAMPLE_DATA_SEGMENT_4};

    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, sizeof(PROXY_PDU), TX_TOKEN);

    EXPECT_PDU({0x43, SAMPLE_DATA_SEGMENT_1});
    EXPECT_PDU({0x83, SAMPLE_DATA_SEGMENT_2});
    EXPECT_PDU({0x83, SAMPLE_DATA_SEGMENT_3});
    EXPECT_PDU({0xc3, SAMPLE_DATA_SEGMENT_4});

    /* Send segment 1 */
    memcpy(p_packet, PROXY_PDU, sizeof(PROXY_PDU));
    sd_ble_gatts_hvx_StubWithCallback(sd_ble_gatts_hvx_cb);
    helper_bearer_event_flag_set_expect(m_flag);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_gatt_packet_send(0, p_packet));
    TEST_ASSERT_TRUE(mesh_gatt_packet_is_pending(0));

    /* Send segment 2 */
    helper_bearer_event_flag_set_expect(m_flag);
    helper_bearer_event_trigger();
    TEST_ASSERT_TRUE(mesh_gatt_packet_is_pending(0));

    /* Send segment 3 */
    helper_bearer_event_flag_set_expect(m_flag);
    helper_bearer_event_trigger();
    TEST_ASSERT_TRUE(mesh_gatt_packet_is_pending(0));

    /* Send segment 4 */
    helper_bearer_event_flag_set_expect(m_flag);
    helper_bearer_event_trigger();
    TEST_ASSERT_TRUE(mesh_gatt_packet_is_pending(0));

    /* Expect TX complete to be raised*/
    tx_complete_evt_expect();
    helper_bearer_event_trigger();
    TEST_ASSERT_FALSE(mesh_gatt_packet_is_pending(0));

    /* Trigger TX Complete from softdevice, no new TX complete should be raised to the application. */
    tx_complete_evt_send();
}

void test_send_single_segment(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    const uint8_t PDU[] = {0xca, 0xfe, 0xba, 0xbe};
    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, sizeof(PDU), TX_TOKEN);
    memcpy(p_packet, PDU, sizeof(PDU));

    sd_ble_gatts_hvx_StubWithCallback(sd_ble_gatts_hvx_cb);
    helper_bearer_event_flag_set_expect(m_flag);

    EXPECT_PDU({MESH_GATT_PDU_TYPE_PROV_PDU, 0xca, 0xfe, 0xba, 0xbe});
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_gatt_packet_send(0, p_packet));

    TEST_ASSERT_TRUE(mesh_gatt_packet_is_pending(0));

    tx_complete_evt_expect();
    helper_bearer_event_trigger();
    tx_complete_evt_send();

    TEST_ASSERT_FALSE(mesh_gatt_packet_is_pending(0));
}

void test_hvx_sys_attr_missing(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    const uint8_t PDU[] = {0xca, 0xfe, 0xba, 0xbe};
    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, sizeof(PDU), TX_TOKEN);
    memcpy(p_packet, PDU, sizeof(PDU));

    m_hvx_return = BLE_ERROR_GATTS_SYS_ATTR_MISSING;
    sd_ble_gatts_hvx_StubWithCallback(sd_ble_gatts_hvx_cb);
    sd_ble_gatts_sys_attr_set_ExpectAndReturn(0, NULL, 0, 0, NRF_SUCCESS);
    helper_bearer_event_flag_set_expect(m_flag);

    EXPECT_PDU({MESH_GATT_PDU_TYPE_PROV_PDU, 0xca, 0xfe, 0xba, 0xbe}); // one for the error
    EXPECT_PDU({MESH_GATT_PDU_TYPE_PROV_PDU, 0xca, 0xfe, 0xba, 0xbe}); // one for the successful retry
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_gatt_packet_send(0, p_packet));

    tx_complete_evt_expect();
    helper_bearer_event_trigger();
    tx_complete_evt_send();
}

void test_receive_single_segment(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    EXPECT_PDU({FIVE_HEX_BYTES});
    RECEIVE_PDU({MESH_GATT_PDU_TYPE_MESH_BEACON & 0x3f, FIVE_HEX_BYTES});
}

void test_receive_segmented_sample_data(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    RECEIVE_PDU({0x43, SAMPLE_DATA_SEGMENT_1});
    RECEIVE_PDU({0x83, SAMPLE_DATA_SEGMENT_2});
    RECEIVE_PDU({0x83, SAMPLE_DATA_SEGMENT_3});

    EXPECT_PDU({SAMPLE_DATA_SEGMENT_1,
                SAMPLE_DATA_SEGMENT_2,
                SAMPLE_DATA_SEGMENT_3,
                SAMPLE_DATA_SEGMENT_4});
    RECEIVE_PDU({0xc3, SAMPLE_DATA_SEGMENT_4});
}

void test_mtu_exchange_larger_than_preferred(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    change_effective_mtu(0, 100);
}

void test_mtu_exchange_less_than_preferred(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    change_effective_mtu(0, 45);
}

void test_tx_ready_event(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    uint8_t ble_evt_buf[sizeof(ble_evt_t) + 2];
    memset(ble_evt_buf, 0, sizeof(ble_evt_buf));
    ble_evt_t * p_evt = (ble_evt_t*) ble_evt_buf;

    p_evt->header.evt_id = BLE_GATTS_EVT_WRITE;
    p_evt->header.evt_len = sizeof(ble_gatts_evt_write_t) + sizeof(ble_evt_hdr_t) + 1;
    p_evt->evt.gatts_evt.params.write.len = 2;
    p_evt->evt.gatts_evt.params.write.op = BLE_GATTS_OP_WRITE_CMD;
    /* Write notification big-Indian. */
    p_evt->evt.gatts_evt.params.write.data[0] = BLE_GATT_HVX_NOTIFICATION;
    p_evt->evt.gatts_evt.params.write.handle = m_gatt.handles.tx.cccd_handle;

    tx_ready_evt_expect();
    mesh_gatt_on_ble_evt(p_evt, m_gatt.p_context);
}

void test_sar_timeout(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    /* Lets receive a couple of segments.. */
    RECEIVE_PDU({0x43, SAMPLE_DATA_SEGMENT_1});
    RECEIVE_PDU({0x83, SAMPLE_DATA_SEGMENT_2});

    /* Pretend to wait.. */
    sd_ble_gap_disconnect_ExpectAndReturn(0, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION, NRF_SUCCESS);
    rx_timeout_event_trigger(0);
}

void test_send_big_packet(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    change_effective_mtu(0, 66);

    /* @tagMeshSp section 8.8, PB-GATT sample data. */
    const uint8_t PROXY_PDU[65] = {SAMPLE_DATA_SEGMENT_1,
                                   SAMPLE_DATA_SEGMENT_2,
                                   SAMPLE_DATA_SEGMENT_3,
                                   SAMPLE_DATA_SEGMENT_4};

    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, sizeof(PROXY_PDU), TX_TOKEN);

    EXPECT_PDU({0x03,
                SAMPLE_DATA_SEGMENT_1,
                SAMPLE_DATA_SEGMENT_2,
                SAMPLE_DATA_SEGMENT_3,
                SAMPLE_DATA_SEGMENT_4
                });

    memcpy(p_packet, PROXY_PDU, sizeof(PROXY_PDU));
    sd_ble_gatts_hvx_StubWithCallback(sd_ble_gatts_hvx_cb);
    helper_bearer_event_flag_set_expect(m_flag);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_gatt_packet_send(0, p_packet));

    tx_complete_evt_expect();
    helper_bearer_event_trigger();
    tx_complete_evt_send();


}

void test_more_than_expected_connections(void)
{

    test_gatt_init();

    connected_evt_expect();
    connect(1);

    connect(0);

    disconnect(0);

    disconnected_evt_expect();
    disconnect(1);
}

void test_disconnect_on_next_tx_when_notifications_are_disabled(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    uint8_t ble_evt_buf[sizeof(ble_evt_t) + 2];
    memset(ble_evt_buf, 0, sizeof(ble_evt_buf));
    ble_evt_t * p_evt = (ble_evt_t*) ble_evt_buf;

    p_evt->header.evt_id = BLE_GATTS_EVT_WRITE;
    p_evt->header.evt_len = sizeof(ble_gatts_evt_write_t) + sizeof(ble_evt_hdr_t) + 1;
    p_evt->evt.gatts_evt.params.write.len = 2;
    p_evt->evt.gatts_evt.params.write.op = BLE_GATTS_OP_WRITE_CMD;
    p_evt->evt.gatts_evt.params.write.data[0] = 0;
    p_evt->evt.gatts_evt.params.write.handle = m_gatt.handles.tx.cccd_handle;

    mesh_gatt_on_ble_evt(p_evt, m_gatt.p_context);

    const uint8_t PDU[] = {0xca, 0xfe, 0xba, 0xbe};
    uint8_t * p_packet = mesh_gatt_packet_alloc(0, MESH_GATT_PDU_TYPE_PROV_PDU, sizeof(PDU), TX_TOKEN);
    memcpy(p_packet, PDU, sizeof(PDU));

    sd_ble_gatts_hvx_IgnoreAndReturn(NRF_ERROR_INVALID_STATE);

    disconnect_expect(0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_gatt_packet_send(0, p_packet));
}

void test_disconnect_on_wrong_pdu_type(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    disconnect_expect(0);
    RECEIVE_PDU({0x45, SAMPLE_DATA_SEGMENT_1});
}

void test_disconnect_on_too_short_message(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    disconnect_expect(0);
    RECEIVE_PDU({0x00});
}


void test_disconnect_on_too_long_message(void)
{
    test_gatt_init();

    connected_evt_expect();
    connect(0);

    /* 66 bytes of payload is one too much... */
    disconnect_expect(0);
    RECEIVE_PDU({0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
}
