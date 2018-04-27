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

#include <string.h>
#include <stdbool.h>

#include "unity.h"
#include "bearer.h"
#include "bearer_adv.h"
#include "packet_mgr.h"

static packet_t test_packet = {.header = {2, 0, 1, 0, 37, 0},
                               .addr   = {1, 2, 3, 4, 5, 6},
                               .payload = {0x0}};

nrf_mesh_assertion_handler_t m_assertion_handler;

static bearer_rx_cb_t   m_rx_cb;

static uint32_t         m_adv_radio_tx_calls;

static uint32_t         m_adv_available_calls;

static uint32_t         m_callback_retval;
static bool             m_is_available;

static nrf_mesh_init_params_t * mp_mesh_init_params;

void setUp(void)
{
    m_adv_radio_tx_calls = 0;

    m_adv_available_calls = 0;

    m_is_available = true;
    m_rx_cb = NULL;
    m_callback_retval = NRF_SUCCESS;

    test_packet.payload[0] = 30;
    test_packet.payload[1] = AD_TYPE_MESH;
}

void tearDown(void)
{

}

/********* temporary mocks ***********/

/* TODO: Use CMock for mocking instead to prevent Lint warnings about missing side-effects in the functions below. */
/*lint -esym(522, bearer_event_critical_section_begin, bearer_event_critical_section_end) */
/*lint -esym(522, bearer_adv_scan_start, bearer_adv_scan_stop) */

uint32_t bearer_adv_init(bearer_scan_config_t * p_config)
{
    m_rx_cb = p_config->rx_cb;
    return m_callback_retval;
}

bool bearer_adv_available(advertiser_t* p_adv)
{
    m_adv_available_calls++;
    return m_is_available;
}

uint32_t bearer_adv_tx(advertiser_t* p_adv, packet_t* p_packet, uint8_t repeats)
{
    if (m_callback_retval == NRF_SUCCESS)
    {
        m_adv_radio_tx_calls++;
        TEST_ASSERT_EQUAL(1, repeats);
    }
    return m_callback_retval;
}

void bearer_adv_adv_start(advertiser_t* p_adv){}

uint32_t bearer_adv_adv_stop(advertiser_t* p_adv) { return NRF_SUCCESS; }

void bearer_adv_scan_start(void){}

void bearer_adv_scan_stop(void){}

void bearer_adv_advertiser_init(advertiser_t* p_adv){}

uint32_t bearer_adv_reconfig(bearer_scan_config_t * config)
{
    return NRF_SUCCESS;
}

void radio_mode_set(uint8_t radio_mode) {}
void radio_access_addr_set(uint32_t access_address) {}

void packet_mgr_free(packet_generic_t* p_packet) {}

void bearer_event_critical_section_begin(void) {}
void bearer_event_critical_section_end(void) {}


/********************************/

void test_bearer_init_submodule_fail(void)
{
    m_callback_retval = NRF_ERROR_INTERNAL;
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));
}

void test_bearer_tx(void)
{
    packet_t packet;
    m_callback_retval = NRF_SUCCESS;
    memset(&packet, 0, sizeof(packet_t));
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));

    TEST_ASSERT_EQUAL(0, m_adv_radio_tx_calls);
    TEST_ASSERT_EQUAL(0, m_adv_available_calls);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_tx(&packet, BEARER_ADV_RADIO, 1));
    TEST_ASSERT_EQUAL(0, m_adv_available_calls);
    TEST_ASSERT_EQUAL(1, m_adv_radio_tx_calls);

    m_adv_radio_tx_calls = 0;
    m_adv_available_calls = 0;
}

void test_bearer_propagate_return_codes(void)
{
    packet_t packet;
    memset(&packet, 0, sizeof(packet_t));

    m_callback_retval = NRF_ERROR_INTERNAL;
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));

    m_callback_retval = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));

    m_callback_retval = NRF_ERROR_INVALID_PARAM;
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_tx(&packet,
                                                   BEARER_ADV_RADIO, 1));
    TEST_ASSERT_EQUAL(0, m_adv_radio_tx_calls);
}

static void m_add_all_ad_types(void)
{
    for (uint16_t ad_type=0; ad_type <= UINT8_MAX; ad_type++)
    {
        bearer_adtype_add(ad_type);
    }
}

static void m_remove_all_ad_types(void)
{
    for (uint16_t ad_type=0; ad_type <= UINT8_MAX; ad_type++)
    {
        bearer_adtype_remove(ad_type);
    }
}

void test_bearer_rx_adtype_filtering(void)
{
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));
    packet_t* p_packet = NULL;
    uint32_t  pack_dropped_inv_length=0, pack_dropped_inv_adtype=0, pack_dropped_inv_addr=0, pack_accepted=0;
    uint32_t  invlen_expected=0, invad_expected=0, invaddr_expected=0, accepted_expected=0;
    bearer_t bearer;
    packet_t packet;

    memcpy(&packet, &test_packet, sizeof(packet_t));

    /* AD type filtering not enabled */
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected+=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* AD type filtering enabled with empty filters*/
    bearer_adtype_filtering_set(true);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    invad_expected+=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Disable Ad type filtering */
    bearer_adtype_filtering_set(false);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Enable Ad type filtering  and add the AD type in use*/
    bearer_adtype_filtering_set(true);
    bearer_adtype_add(AD_TYPE_MESH);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Use an AD type that's not been added */
    packet.payload[1] = AD_TYPE_BEACON;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    invad_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Add the new AD type */
    bearer_adtype_add(AD_TYPE_BEACON);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Remove the unused AD type */
    bearer_adtype_remove(AD_TYPE_MESH);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Use the removed AD type */
    packet.payload[1] = AD_TYPE_MESH;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    invad_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Disable AD type filtering */
    bearer_adtype_filtering_set(false);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Add all possible AD types to the filters and enable AD type filtering */
    m_add_all_ad_types();
    bearer_adtype_filtering_set(true);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Remove from the filters and use AD_TYPE_MESH. */
    packet.payload[1] = AD_TYPE_MESH;
    bearer_adtype_remove(AD_TYPE_MESH);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    invad_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Using any other type should work since only AD_TYPE_MESH is removed*/
    packet.payload[1] = AD_TYPE_DFU;
    bearer_adtype_filtering_set(true);
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    bearer_stats_get(&pack_dropped_inv_length, &pack_dropped_inv_adtype, &pack_dropped_inv_addr, &pack_accepted);
    accepted_expected +=1;
    TEST_ASSERT_EQUAL(invlen_expected, pack_dropped_inv_length);
    TEST_ASSERT_EQUAL(invad_expected, pack_dropped_inv_adtype);
    TEST_ASSERT_EQUAL(invaddr_expected, pack_dropped_inv_addr);
    TEST_ASSERT_EQUAL(accepted_expected, pack_accepted);

    /* Reset state (as much as possible) */
    m_remove_all_ad_types();
    bearer_adtype_filtering_set(false);
}

void test_bearer_rx(void)
{
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));
    packet_t* p_packet = NULL;
    bearer_t bearer;
    packet_t packet;
    memcpy(&packet, &test_packet, sizeof(packet_t));

    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(&packet, p_packet);
    TEST_ASSERT_EQUAL(BEARER_ADV_RADIO, bearer);

    /* invalid bearer config */
    p_packet = NULL;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(&packet, p_packet);
    TEST_ASSERT_EQUAL(BEARER_ADV_RADIO, bearer); /* shouldn't discriminate */
}

void test_bearer_rx_overflow(void)
{
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));
    TEST_ASSERT_NOT_EQUAL(NULL, m_rx_cb);
    packet_t* p_packet = NULL;
    bearer_t bearer;

    packet_t packets[BEARER_RX_QUEUE_LENGTH + 1] = {{{0}}};
    for (uint32_t i = 0; i < BEARER_RX_QUEUE_LENGTH + 1; ++i)
    {
        memcpy(&packets[i], &test_packet, sizeof(test_packet));
        m_rx_cb(&packets[i], BEARER_ADV_RADIO, NULL);
    }

    /* pop entire queue */
    for (uint32_t i = 0; i < BEARER_RX_QUEUE_LENGTH; ++i)
    {
        p_packet = NULL;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
        TEST_ASSERT_EQUAL(&packets[i], p_packet); /* should have discarded the last rx, not the first */
        TEST_ASSERT_EQUAL(BEARER_ADV_RADIO, bearer);
    }

    /* the last rx shouldn't have been able to follow */
    p_packet = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL((packet_t*) NULL, p_packet);
}

void test_bearer_addr_filtering(void)
{
    TEST_ASSERT_EQUAL(m_callback_retval, bearer_init(mp_mesh_init_params));
    packet_t* p_packet = NULL;
    bearer_t bearer;
    packet_t packet;
    memcpy(&packet, &test_packet, sizeof(packet_t));
    uint32_t dropped_invalid_length;
    uint32_t dropped_invalid_adtype;
    uint32_t dropped_invalid_addr;
    uint32_t accepted;
    uint32_t exp_dropped_invalid_length = 0;
    uint32_t exp_dropped_invalid_adtype = 0;
    uint32_t exp_dropped_invalid_addr = 0;
    uint32_t exp_accepted = 0;

#define CHECK_STATS() do{ \
    bearer_stats_get(&dropped_invalid_length, &dropped_invalid_adtype, &dropped_invalid_addr, &accepted); \
    TEST_ASSERT_EQUAL_MESSAGE(exp_dropped_invalid_length, dropped_invalid_length, "Length"); \
    TEST_ASSERT_EQUAL_MESSAGE(exp_dropped_invalid_adtype, dropped_invalid_adtype, "AD type"); \
    TEST_ASSERT_EQUAL_MESSAGE(exp_dropped_invalid_addr, dropped_invalid_addr, "Addr"); \
    TEST_ASSERT_EQUAL_MESSAGE(exp_accepted, accepted, "Accepted"); \
} while (0)

    bearer_stats_get(&exp_dropped_invalid_length, &exp_dropped_invalid_adtype, &exp_dropped_invalid_addr, &exp_accepted);

    ble_gap_addr_t filter_list[3];
    filter_list[0].addr_type = 1;
    memset(filter_list[0].addr, 0xAB, BLE_GAP_ADDR_LEN);
    filter_list[1].addr_type = 0;
    memset(filter_list[0].addr, 0xCD, BLE_GAP_ADDR_LEN);
    filter_list[2].addr_type = 2; /* invalid addr type */
    memset(filter_list[0].addr, 0xEF, BLE_GAP_ADDR_LEN);

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, bearer_filter_gap_addr_whitelist_set(NULL, 3));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, bearer_filter_gap_addr_whitelist_set(filter_list, 0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_whitelist_set(filter_list, 3));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));

    memcpy(packet.addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[0].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[1].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1; /* Doesn't match invalid AD type in filter */
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    /* Not in whitelist */
    memset(packet.addr, 0x01, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    /* Alter list to make previous packet pass */
    memset(filter_list[0].addr, 0x01, BLE_GAP_ADDR_LEN);
    filter_list[1].addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    /* Test clear */
    ble_gap_addr_t alt_filter = {};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_whitelist_set(&alt_filter, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_whitelist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_blacklist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_clear());

    /* Blacklist tests */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_blacklist_set(filter_list, 3));

    memcpy(packet.addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[0].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[1].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1; /* Doesn't match invalid AD type in filter */
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    /* Not in blacklist */
    memset(packet.addr, 0x02, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    /* Alter list to make previous packet fit */
    memset(filter_list[0].addr, 0x02, BLE_GAP_ADDR_LEN);
    filter_list[1].addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    /* Test clear */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_blacklist_set(&alt_filter, 1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_blacklist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_whitelist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_clear());

    /* Test that all packets pass when not running filters */
    memcpy(packet.addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[0].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = filter_list[1].addr_type;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    memcpy(packet.addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1; /* Doesn't match invalid AD type in filter */
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    /* Not in blacklist */
    memset(packet.addr, 0x02, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    /* test range filter */

    memset(filter_list[0].addr, 0xAB, BLE_GAP_ADDR_LEN);
    memset(filter_list[1].addr, 0xAB, BLE_GAP_ADDR_LEN);
    filter_list[0].addr[5] = 0;
    filter_list[1].addr[5] = 0;
    filter_list[0].addr_type = 0;
    filter_list[1].addr_type = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list)); /* Should be allowed to set it twice */
    filter_list[1].addr[5] = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));
    filter_list[0].addr[5] = 2; /* first higher than second */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, bearer_filter_gap_addr_range_set(filter_list));
    filter_list[0].addr[5] = 0;
    filter_list[0].addr_type = 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, bearer_filter_gap_addr_range_set(filter_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, bearer_filter_gap_addr_range_set(NULL));
    filter_list[0].addr_type = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));

    memcpy(packet.addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    packet.header.addr_type = 0;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();

    packet.header.addr_type = 1;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    packet.header.addr_type = 0;
    packet.addr[5] = 2;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    packet.addr[5] = 0;
    packet.addr[4] = 0;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    packet.addr[5] = 0;
    packet.addr[4] = 0xAC;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    packet.addr[5] = 1;
    packet.addr[4] = 0xAB;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, bearer_rx(&p_packet, &bearer, NULL));
    exp_dropped_invalid_addr++;
    CHECK_STATS();

    filter_list[1].addr[5] = 2;
    m_rx_cb(&packet, BEARER_ADV_RADIO, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_rx(&p_packet, &bearer, NULL));
    exp_accepted++;
    CHECK_STATS();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
#undef CHECK_STATS
}
