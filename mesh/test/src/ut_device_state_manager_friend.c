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

#include <unity.h>
#include <cmock.h>
#include <string.h>
#include <stdlib.h>

#include "utils.h"
#include "test_assert.h"

#include "bearer_event_mock.h"
#include "device_state_manager.h"
#include "device_state_manager_flash.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_keygen_mock.h"
#include "net_state_mock.h"
#include "flash_manager_mock.h"
#include "mesh_opt_core_mock.h"
#include "manual_mock_queue.h"
#include "mesh_lpn_mock.h"
#include "heartbeat_mock.h"
#include "mesh_opt_friend_mock.h"

/* Enable this to check that the handle ordering is as expected (0..N-1). */
#define TEST_EXPLICIT_ORDERING 1
#define VIRTUAL_ADDR           0x8080
#define VIRTUAL_ADDRESS_COUNT  3

#define SUBSCRIPTION_ADDRESS_COUNT_MAX  (DSM_NONVIRTUAL_ADDR_MAX + DSM_VIRTUAL_ADDR_MAX)

/** Non-VLA version of a dsm flash entry */
typedef struct
{
    fm_header_t       header;
    dsm_flash_entry_t entry;
} dsm_entry_t;

MOCK_QUEUE_DEF(fm_entries_read_mock_queue, dsm_entry_t, NULL);

static flash_manager_t * mp_flash_manager;
static uint32_t m_flash_expect_calls;
static flash_manager_page_t m_flash_area[2];
static uint32_t m_fm_mem_listener_register_expect;
static fm_state_t m_add_manager_result_state;
static struct
{
    bool verify_contents; /**< Verify contents of the flash data. Should be turned off if we're doing batch-operations */
    void * p_data;
    uint32_t data_length;
    uint16_t flash_group;
    uint16_t flash_handle; /**< Expected flash handle if flash group is 0 */
} m_expected_flash_data;


static nrf_mesh_evt_handler_t m_mesh_evt_handler;

typedef struct
{
    uint16_t key_index;
    uint8_t nid;
    uint8_t key[NRF_MESH_KEY_SIZE];
    dsm_handle_t handle;

    struct
    {
        nrf_mesh_network_secmat_t net;
        nrf_mesh_beacon_secmat_t beacon;
    } secmat;
} test_net_t;

static uint32_t flash_manager_add_cb(flash_manager_t * p_manager, const flash_manager_config_t * p_config, int calls)
{
    mp_flash_manager = p_manager;
    TEST_ASSERT_EQUAL_PTR(&m_flash_area[0], p_config->p_area);
    TEST_ASSERT_EQUAL(1, p_config->page_count);
    TEST_ASSERT_NOT_NULL(p_config->write_complete_cb);
    TEST_ASSERT_NOT_NULL(p_config->invalidate_complete_cb);
    memcpy(&p_manager->config, p_config, sizeof(flash_manager_config_t));
    p_manager->internal.state = m_add_manager_result_state;
    return NRF_SUCCESS;
}

static void nrf_mesh_evt_handler_add_stub_cb(nrf_mesh_evt_handler_t * p_handler_params, int cmock_num_calls)
{
    (void) cmock_num_calls;
    m_mesh_evt_handler = *p_handler_params;
}


void setUp(void)
{
    m_fm_mem_listener_register_expect = 0;
    m_flash_expect_calls = 0;
    nrf_mesh_mock_Init();
    nrf_mesh_keygen_mock_Init();
    flash_manager_mock_Init();
    net_state_mock_Init();
    nrf_mesh_events_mock_Init();
    mesh_opt_core_mock_Init();
    fm_entries_read_mock_queue_Init();
    heartbeat_mock_Init();
    mesh_opt_friend_mock_Init();
    mesh_lpn_mock_Init();

    mp_flash_manager = NULL;
    m_add_manager_result_state = FM_STATE_READY;
    m_expected_flash_data.verify_contents = true;
    m_expected_flash_data.p_data = NULL;

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    flash_manager_recovery_page_get_IgnoreAndReturn(&m_flash_area[1]);
    flash_manager_add_StubWithCallback(flash_manager_add_cb);

    net_state_flash_area_get_ExpectAndReturn((void *)(PAGE_SIZE + (uint32_t)m_flash_area));
    nrf_mesh_evt_handler_add_ExpectAnyArgs();

    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_evt_handler_add_stub_cb);
    dsm_init();

    /* Ignore the subnet added call by default, it's tested in test_net: */
    nrf_mesh_subnet_added_Ignore();
}

void tearDown(void)
{
    flash_manager_remove_IgnoreAndReturn(NRF_SUCCESS);
    dsm_clear();

    TEST_ASSERT_EQUAL_MESSAGE(0, m_flash_expect_calls, "Not all expected flash operations called");
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_keygen_mock_Verify();
    nrf_mesh_keygen_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    mesh_opt_core_mock_Verify();
    mesh_opt_core_mock_Destroy();
    fm_entries_read_mock_queue_Verify();
    fm_entries_read_mock_queue_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    mesh_opt_friend_mock_Verify();
    mesh_opt_friend_mock_Destroy();
    mesh_lpn_mock_Verify();
    mesh_lpn_mock_Destroy();
}

/* fail-version of the flash callback, so we can "fail" flashing */
fm_entry_t * flash_manager_entry_alloc_cb(flash_manager_t * p_manager,
                                       fm_handle_t       handle,
                                       uint32_t          data_length,
                                       int calls)
{
    TEST_ASSERT_NOT_NULL(p_manager);
    TEST_ASSERT_EQUAL_PTR(mp_flash_manager, p_manager);

    if (m_expected_flash_data.verify_contents)
    {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0, m_flash_expect_calls, "Flash entry alloc called when not expected");
        m_flash_expect_calls--;
        TEST_ASSERT_EQUAL(m_expected_flash_data.data_length, data_length);
        if (m_expected_flash_data.flash_group == 0)
        {
            TEST_ASSERT_EQUAL_HEX16(m_expected_flash_data.flash_handle, handle);
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX16(m_expected_flash_data.flash_group, handle & DSM_FLASH_HANDLE_FILTER_MASK);
        }
    }

    fm_entry_t * p_data   = malloc(sizeof(fm_header_t) + ALIGN_VAL(data_length, 4));
    TEST_ASSERT_NOT_NULL(p_data);
    p_data->header.len_words = ALIGN_VAL(sizeof(fm_header_t) + data_length, 4);
    p_data->header.handle = handle;
    return p_data;
}

static void flash_manager_entry_commit_cb(const fm_entry_t * p_entry, int call_count)
{
    if (m_expected_flash_data.verify_contents)
    {
        TEST_ASSERT_NOT_NULL(m_expected_flash_data.p_data);
        TEST_ASSERT_NOT_EQUAL(0, m_expected_flash_data.data_length);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_flash_data.p_data,
                                    p_entry->data,
                                    m_expected_flash_data.data_length);
    }
    if (m_expected_flash_data.p_data)
    {
        free(m_expected_flash_data.p_data);
    }
    free((fm_entry_t*) p_entry);
    m_expected_flash_data.p_data = NULL;
}


static void flash_expect(void * p_expected_data, uint32_t data_length)
{
    TEST_ASSERT_EQUAL_MESSAGE(NULL, m_expected_flash_data.p_data, "The previous flash operation has not been finished yet.");
    m_expected_flash_data.data_length = data_length;
    m_expected_flash_data.verify_contents = true;
    m_expected_flash_data.p_data = malloc(data_length);
    memcpy(m_expected_flash_data.p_data, p_expected_data, data_length);

    m_flash_expect_calls++;
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_cb);
    flash_manager_entry_commit_StubWithCallback(flash_manager_entry_commit_cb);
}

static void flash_expect_subnet(const uint8_t * p_key, uint16_t key_index)
{
    dsm_flash_entry_subnet_t data;
    memset(&data, 0, sizeof(data));

    memcpy(data.key, p_key, sizeof(data.key));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    m_expected_flash_data.flash_group = DSM_FLASH_GROUP_SUBNETS;
    flash_expect(&data, sizeof(data));
}

static void flash_expect_subnet_update(const uint8_t * p_old_key, const uint8_t * p_new_key,
        uint16_t key_index, nrf_mesh_key_refresh_phase_t key_refresh_phase, dsm_handle_t handle)
{
    dsm_flash_entry_subnet_t data;
    memset(&data, 0, sizeof(data));


    memcpy(data.key, p_old_key, sizeof(data.key));
    memcpy(data.key_updated, p_new_key, sizeof(data.key_updated));
    memcpy(&data.key_index, &key_index, sizeof(data.key_index));
    data.key_refresh_phase = key_refresh_phase;

    m_expected_flash_data.flash_group = 0;
    m_expected_flash_data.flash_handle = DSM_HANDLE_TO_FLASH_HANDLE(DSM_FLASH_GROUP_SUBNETS, handle);
    flash_expect(&data, sizeof(data));
}

static void friendship_network_add(test_net_t * p_net,
                                   nrf_mesh_keygen_friendship_secmat_params_t * p_frnd_secmat_params,
                                   nrf_mesh_network_secmat_t * p_frnd_secmat)
{
    const nrf_mesh_network_secmat_t * p_net_secmat_valid;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(p_net->key_index, &p_net_secmat_valid));

    nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(p_net->key, p_frnd_secmat_params, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(p_frnd_secmat, sizeof(nrf_mesh_network_secmat_t));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, p_frnd_secmat_params));
}

static void network_add(test_net_t * p_test_net)
{
    p_test_net->secmat.net.nid = p_test_net->nid;
    memset(p_test_net->secmat.net.privacy_key, p_test_net->key_index | 0xA0, NRF_MESH_KEY_SIZE);
    memset(p_test_net->secmat.net.encryption_key, p_test_net->key_index | 0xB0, NRF_MESH_KEY_SIZE);
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(p_test_net->key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&p_test_net->secmat.net, sizeof(p_test_net->secmat.net));

    memset(p_test_net->secmat.beacon.net_id, p_test_net->key_index | 0xC0, NRF_MESH_NETID_SIZE);
    memset(p_test_net->secmat.beacon.key, p_test_net->key_index | 0xD0, NRF_MESH_KEY_SIZE);
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(p_test_net->key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&p_test_net->secmat.beacon, sizeof(p_test_net->secmat.beacon));

#if MESH_FEATURE_GATT_PROXY_ENABLED
    memset(p_test_net->secmat.beacon.identity_key, p_test_net->key_index | 0xE0, NRF_MESH_KEY_SIZE);
    nrf_mesh_keygen_identitykey_ExpectAndReturn(p_test_net->key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
    nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(p_test_net->secmat.beacon.identity_key, NRF_MESH_KEY_SIZE);
#endif
    uint8_t net_key[NRF_MESH_KEY_SIZE];
    memset(net_key, 0xFF, NRF_MESH_KEY_SIZE);
    flash_expect_subnet(p_test_net->key, p_test_net->key_index);
    nrf_mesh_subnet_added_Expect(p_test_net->key_index, p_test_net->secmat.beacon.net_id);

    /* add the net */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(p_test_net->key_index, p_test_net->key, &p_test_net->handle));
    TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, p_test_net->handle);
    TEST_ASSERT_NOT_EQUAL(0xABCD, p_test_net->handle); /* The handle must have changed */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_key_get(p_test_net->handle, net_key));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_test_net->key, net_key, NRF_MESH_KEY_SIZE);
}
/*****************************************************************************
* Tests:
* @note This file only tests DSM APIs specific to friendship.
*****************************************************************************/

void test_nrf_mesh_net_master_secmat_get(void)
{
    struct
    {
        dsm_handle_t network_handle;
        uint8_t old_nid;
        uint8_t new_nid;
        mesh_key_index_t key_index;
        bool is_in_friendship;
    } net[] =
    {
        {DSM_HANDLE_INVALID, 2, 18, 0, true},  /* Old and new NIDs are differenet, friendship */
        {DSM_HANDLE_INVALID, 3, 19, 1, false}, /* Old and new NIDs are differenet, no friendship */
        {DSM_HANDLE_INVALID, 4, 20, 2, true}  /* Old and new NIDs are differenet, friendship */
    };

    nrf_mesh_network_secmat_t old_secmat, new_secmat;
    nrf_mesh_beacon_secmat_t old_beacon_secmat, new_beacon_secmat;
    nrf_mesh_network_secmat_t old_friendship_secmat, new_friendship_secmat;
    const nrf_mesh_network_secmat_t * p_rx_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_rx_secmat_secondary = NULL;
    const nrf_mesh_network_secmat_t * p_test_old_friendship_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_test_new_friendship_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_test_old_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_test_new_secmat = NULL;

    const uint8_t old_key[NRF_MESH_KEY_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    const uint8_t new_key[NRF_MESH_KEY_SIZE] = { 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };

    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params = {0x0001, 0x0002, 1, 2};

    /* Test: NULL input, reutrns NULL */
    TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(NULL));

    /* Add the original network key, which will be refreshed: */
    memset(old_secmat.privacy_key,    1, NRF_MESH_KEY_SIZE);
    memset(old_secmat.encryption_key, 2, NRF_MESH_KEY_SIZE);
    memset(new_secmat.privacy_key,    11, NRF_MESH_KEY_SIZE);
    memset(new_secmat.encryption_key, 22, NRF_MESH_KEY_SIZE);
    memset(old_beacon_secmat.key,     3, NRF_MESH_KEY_SIZE);
    memset(old_beacon_secmat.net_id,  4, NRF_MESH_NETID_SIZE);
    memset(new_beacon_secmat.key,    33, NRF_MESH_KEY_SIZE);
    memset(new_beacon_secmat.net_id, 44, NRF_MESH_NETID_SIZE);
    memset(old_friendship_secmat.privacy_key,    5, NRF_MESH_KEY_SIZE);
    memset(old_friendship_secmat.encryption_key, 6, NRF_MESH_KEY_SIZE);
    memset(new_friendship_secmat.privacy_key,    55, NRF_MESH_KEY_SIZE);
    memset(new_friendship_secmat.encryption_key, 66, NRF_MESH_KEY_SIZE);

    /* Add networks and calculate friendship secmats */
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        friendship_secmat_params.lpn_address = i + 1;
        old_secmat.nid = net[i].old_nid;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&old_secmat, sizeof(old_secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&old_beacon_secmat, sizeof(old_beacon_secmat));
        flash_expect_subnet(old_key, net[i].key_index);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(net[i].key_index, old_key, &net[i].network_handle));

        if (net[i].is_in_friendship)
        {
            const nrf_mesh_network_secmat_t *p_net_secmat_valid;
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[i].key_index, &p_net_secmat_valid));

            old_friendship_secmat.nid = net[i].old_nid;
            nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(old_key, &friendship_secmat_params, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&old_friendship_secmat, sizeof(nrf_mesh_network_secmat_t));

            TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, &friendship_secmat_params));
        }


        /* Test: When RX secmat is same as network secmat, API returns NULL. */
        dsm_net_secmat_from_keyindex_get(net[i].key_index, &p_rx_secmat);
        p_test_old_secmat = p_rx_secmat;
        TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(p_rx_secmat));

        /* Test: When RX secmat is friendship secmat, API returns primary secmat if in friendship.
         *       When RX secmat is friendship secmat, API returns NULL if NOT in friendship.
         */
        nrf_mesh_friendship_secmat_get(friendship_secmat_params.lpn_address, &p_rx_secmat);
        p_test_old_friendship_secmat = p_rx_secmat;  /* save for later use */
        if (net[i].is_in_friendship)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_secmat, nrf_mesh_net_master_secmat_get(p_rx_secmat), sizeof(nrf_mesh_network_secmat_t));
        }
        else
        {
            TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(&old_friendship_secmat));
        }

        nrf_mesh_key_refresh_phase_t current_phase;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(net[i].network_handle, &current_phase));
        TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_0, current_phase);

        /* Start the key update procedure (go to KR phase 1): */
        new_secmat.nid = net[i].new_nid;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(new_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&new_secmat, sizeof(new_secmat));
        net_state_key_refresh_phase_changed_Expect(net[i].key_index, new_beacon_secmat.net_id, NRF_MESH_KEY_REFRESH_PHASE_1);
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(new_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&new_beacon_secmat, sizeof(new_beacon_secmat));
        if (net[i].is_in_friendship)
        {
            new_friendship_secmat.nid = net[i].new_nid;
            nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(new_key, &friendship_secmat_params, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&new_friendship_secmat, sizeof(nrf_mesh_network_secmat_t));
        }
        flash_expect_subnet_update(old_key, new_key, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_1, net[i].network_handle);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(net[i].network_handle, new_key));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(net[i].network_handle, &current_phase));
        TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_1, current_phase);


        /* Test: When RX secmat is same as OLD or NEW secmat, API returns NULL */
        /*       RX secmat is old */
        TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(p_test_old_secmat));
        /*       RX secmat is new */
        p_rx_secmat = p_test_old_secmat;
        p_rx_secmat_secondary = p_test_old_secmat;      /* This is actually unused, but API needs it */
        nrf_mesh_net_secmat_next_get(new_secmat.nid, &p_rx_secmat, &p_rx_secmat_secondary);
        p_test_new_secmat = p_rx_secmat;
        TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(p_test_new_secmat));

        /* Note: The test case when RX secmat is friendship secmat, but mater
         * secmat could not be located is an invalid scenario. */
        /* Test: When RX secmat is same as OLD friendship secmat, API returns corresponding
         * OLD master secmat. */
        if (net[i].is_in_friendship)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_secmat, nrf_mesh_net_master_secmat_get(p_test_old_friendship_secmat), sizeof(nrf_mesh_network_secmat_t));
        }

        /* Test: When RX secmat is same as NEW friendship secmat, API returns corresponding
         * NEW master secmat. */
        if (net[i].is_in_friendship)
        {
            p_test_new_friendship_secmat = p_test_old_friendship_secmat + 1; /* Hack to get the `secmat_updated` */
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_secmat, nrf_mesh_net_master_secmat_get(p_test_new_friendship_secmat), sizeof(nrf_mesh_network_secmat_t));
        }
    }
}

/* This test checks the retrieval of friendship secmat with same NID as that of other network secmats
 * when friend node has friendship credentials added to dsm. */
void test_secmat_lookup_with_same_nid(void)
{
    const uint8_t NID = 0x42;
    test_net_t nets[2];
    memset(nets, 0, sizeof(nets));

    nrf_mesh_network_secmat_t friendship_secmat = {
        .nid = NID,
        .privacy_key = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F},
        .encryption_key = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F},
    };

    /* Dummy parameters, not important in this test. */
    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params = {
        .lpn_address = 0x0001,
        .friend_address = 0x0002,
        .lpn_counter = 1,
        .friend_counter = 2
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(nets); ++i)
    {
        nets[i].key_index = i;
        nets[i].nid = NID;
        memset(nets[i].key, i, NRF_MESH_KEY_SIZE);
        nets[i].handle = 0xBEEF;

        network_add(&nets[i]);
    }

    friendship_network_add(&nets[0], &friendship_secmat_params, &friendship_secmat);

    const nrf_mesh_network_secmat_t * p_test_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_test_aux_secmat = NULL;

    /* We should get the friendship secmat first. */
    nrf_mesh_net_secmat_next_get(NID, &p_test_secmat, &p_test_aux_secmat);
    TEST_ASSERT_NOT_NULL(p_test_secmat);
    TEST_ASSERT_NULL(p_test_aux_secmat);

    TEST_ASSERT_EQUAL_MEMORY(&friendship_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));

    for (uint32_t i = 0; i < ARRAY_SIZE(nets); ++i)
    {
        /* Assume the decryption failed with the NID, let's try the next key. */
        //mesh_lpn_is_in_friendship_IgnoreAndReturn(false);
        nrf_mesh_net_secmat_next_get(NID, &p_test_secmat, &p_test_aux_secmat);
        TEST_ASSERT_NOT_NULL(p_test_secmat);
        TEST_ASSERT_NULL(p_test_aux_secmat);

        /* Check that we're getting the correct key. This test makes an assumption about the order
         * of the keys. */
        TEST_ASSERT_EQUAL_MEMORY(&nets[i].secmat.net, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));
    }

    /* All available networks with the given NID is exhausted. Last attempt should not result in any
     * secmat. */
    //mesh_lpn_is_in_friendship_IgnoreAndReturn(false);
    nrf_mesh_net_secmat_next_get(NID, &p_test_secmat, &p_test_aux_secmat);
    TEST_ASSERT_NULL(p_test_secmat);
    TEST_ASSERT_NULL(p_test_aux_secmat);
}
