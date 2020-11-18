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
#include <string.h>
#include <stdlib.h>

#include "utils.h"
#include "test_assert.h"

#include "bearer_event_mock.h"
#include "device_state_manager.h"
#include "mesh_opt_dsm.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_keygen_mock.h"
#include "net_state_mock.h"
#include "proxy_mock.h"
#include "mesh_opt_core_mock.h"
#include "manual_mock_queue.h"
#include "mesh_lpn_internal_mock.h"
#include "mesh_lpn_mock.h"
#include "heartbeat_mock.h"
#include "mesh_opt_friend_mock.h"
#include "mesh_config_entry_mock.h"
#include "mesh_config_mock.h"

/* Enable this to check that the handle ordering is as expected (0..N-1). */
#define TEST_EXPLICIT_ORDERING 1
#define VIRTUAL_ADDR           0x8080
#define VIRTUAL_ADDRESS_COUNT  3

typedef struct
{
    mesh_key_index_t             key_index;
    nrf_mesh_key_refresh_phase_t key_refresh_phase;
    uint8_t                      key[NRF_MESH_KEY_SIZE];
} dsm_legacy_entry_subnet_t;

typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    uint8_t          key[NRF_MESH_KEY_SIZE];
} dsm_reduced_legacy_entry_appkey_t;

typedef struct
{
    uint16_t id;
    union
    {
        dsm_entry_metainfo_t metainfo_entry;
        dsm_entry_addr_unicast_t unicats_entry;
        dsm_entry_subnet_t subnet_entry;
        dsm_entry_appkey_t appkey_entry;
        dsm_entry_devkey_t devkey_entry;
        dsm_entry_addr_nonvirtual_t nonvirtual_entry;
        dsm_entry_addr_virtual_t virtual_entry;
        dsm_legacy_entry_subnet_t legacy_subnet_entry;
        dsm_legacy_entry_appkey_t legacy_appkey_entry;
        dsm_reduced_legacy_entry_appkey_t reduced_legacy_appkey_entry;
        uint8_t common_stub;
    };
} snapshot_entry_t;

typedef struct
{
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    uint16_t addr;
    dsm_handle_t handle;
} subman_test_vaddr_info_t;

typedef struct
{
    uint16_t addr;
    dsm_handle_t handle;
} subman_test_addr_info_t;


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

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_addr_unicast_t unicast_info;
} test_unicast_entry_t;

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_addr_nonvirtual_t nonvirtual_info;
} test_nonvirtual_entry_t;

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_addr_virtual_t virtual_info;
} test_virtual_entry_t;

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_subnet_t subnet_info;
} test_subnet_entry_t;

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_appkey_t appkey_info;
} test_appkey_entry_t;

typedef struct
{
    int trigger_cnt;
    bool is_dummy;
    dsm_entry_devkey_t devkey_info;
} test_devkey_entry_t;

typedef struct
{
    int trigger_cnt;
} test_metadata_t;

static nrf_mesh_evt_handler_t m_mesh_evt_handler;
static bool m_test_in_friendship;

static test_metadata_t m_expected_metadata;
static test_unicast_entry_t m_expected_unicast_entry;
static test_nonvirtual_entry_t m_expected_nonvirtual_entry;
static test_virtual_entry_t m_expected_virtual_entry;
static test_subnet_entry_t m_expected_subnet_entry;
static test_appkey_entry_t m_expected_appkey_entry;
static test_devkey_entry_t m_expected_devkey_entry;

/*****************************************************************************
* Extern stubs
*****************************************************************************/
extern const mesh_config_entry_params_t m_dsm_metadata_params;
extern const mesh_config_entry_params_t m_dsm_unicast_addr_params;
extern const mesh_config_entry_params_t m_dsm_nonvirtual_addr_params;
extern const mesh_config_entry_params_t m_dsm_virtual_addr_params;
extern const mesh_config_entry_params_t m_dsm_subnet_params;
extern const mesh_config_entry_params_t m_dsm_appkey_params;
extern const mesh_config_entry_params_t m_dsm_devkey_params;
extern const mesh_config_entry_params_t m_dsm_legacy_subnet_params;
extern const mesh_config_entry_params_t m_dsm_reduced_legacy_appkey_params;
extern const mesh_config_entry_params_t m_dsm_full_legacy_appkey_params;

extern void dsm_legacy_pretreatment_do(mesh_config_entry_id_t * p_id, uint32_t entry_len);

static uint32_t entry_set_cb(mesh_config_entry_id_t id, const void* p_entry, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_EQUAL(MESH_OPT_DSM_FILE_ID, id.file);

    if (id.record == MESH_OPT_DSM_UNICAST_ADDR_RECORD)
    {
        dsm_entry_addr_unicast_t * p_data = (dsm_entry_addr_unicast_t *)p_entry;

        TEST_ASSERT_GREATER_THAN(0, m_expected_unicast_entry.trigger_cnt);
        m_expected_unicast_entry.trigger_cnt--;

        if (!m_expected_unicast_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_UINT16(m_expected_unicast_entry.unicast_info.addr.address_start, p_data->addr.address_start);
            TEST_ASSERT_EQUAL_UINT16(m_expected_unicast_entry.unicast_info.addr.count, p_data->addr.count);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_unicast_addr_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD,
                    MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX - 1))
    {
        dsm_entry_addr_nonvirtual_t * p_data = (dsm_entry_addr_nonvirtual_t *)p_entry;
        TEST_ASSERT_GREATER_THAN(0, m_expected_nonvirtual_entry.trigger_cnt);
        m_expected_nonvirtual_entry.trigger_cnt--;

        if (!m_expected_nonvirtual_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_UINT16(m_expected_nonvirtual_entry.nonvirtual_info.addr, p_data->addr);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_nonvirtual_addr_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD,
                    MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_VIRTUAL_ADDR_MAX - 1))
    {
        dsm_entry_addr_virtual_t * p_data = (dsm_entry_addr_virtual_t *)p_entry;
        TEST_ASSERT_GREATER_THAN(0, m_expected_virtual_entry.trigger_cnt);
        m_expected_virtual_entry.trigger_cnt--;

        if (!m_expected_virtual_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_virtual_entry.virtual_info.uuid, p_data->uuid, NRF_MESH_UUID_SIZE);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_virtual_addr_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                    MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1))
    {
        dsm_entry_subnet_t * p_data = (dsm_entry_subnet_t *)p_entry;
        TEST_ASSERT_GREATER_THAN(0, m_expected_subnet_entry.trigger_cnt);
        m_expected_subnet_entry.trigger_cnt--;

        if (!m_expected_subnet_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_subnet_entry.subnet_info.key, p_data->key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_UINT16(m_expected_subnet_entry.subnet_info.key_index, p_data->key_index);

            if (m_expected_subnet_entry.subnet_info.key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                TEST_ASSERT_EQUAL(m_expected_subnet_entry.subnet_info.key_refresh_phase, p_data->key_refresh_phase);
                TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_subnet_entry.subnet_info.key_updated, p_data->key_updated, NRF_MESH_KEY_SIZE);
            }
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_subnet_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                    MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
    {
        dsm_entry_appkey_t * p_data = (dsm_entry_appkey_t *)p_entry;
        TEST_ASSERT_GREATER_THAN(0, m_expected_appkey_entry.trigger_cnt);
        m_expected_appkey_entry.trigger_cnt--;

        if (!m_expected_appkey_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_appkey_entry.appkey_info.key, p_data->key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_UINT16(m_expected_appkey_entry.appkey_info.key_index, p_data->key_index);
            TEST_ASSERT_EQUAL_UINT16(m_expected_appkey_entry.appkey_info.subnet_handle, p_data->subnet_handle);

            if (m_expected_appkey_entry.appkey_info.is_key_updated)
            {
                TEST_ASSERT_EQUAL(m_expected_appkey_entry.appkey_info.is_key_updated, p_data->is_key_updated);
                TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_appkey_entry.appkey_info.key_updated, p_data->key_updated, NRF_MESH_KEY_SIZE);
            }
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_appkey_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_DEVKEYS_RECORD,
                    MESH_OPT_DSM_DEVKEYS_RECORD + DSM_DEVICE_MAX - 1))
    {
        dsm_entry_devkey_t * p_data = (dsm_entry_devkey_t *)p_entry;
        TEST_ASSERT_GREATER_THAN(0, m_expected_devkey_entry.trigger_cnt);
        m_expected_devkey_entry.trigger_cnt--;

        if (!m_expected_devkey_entry.is_dummy)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_devkey_entry.devkey_info.key, p_data->key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_UINT16(m_expected_devkey_entry.devkey_info.key_owner, p_data->key_owner);
            TEST_ASSERT_EQUAL_UINT16(m_expected_devkey_entry.devkey_info.subnet_handle, p_data->subnet_handle);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_devkey_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (id.record == MESH_OPT_DSM_METADATA_RECORD)
    {
        dsm_entry_metainfo_t * p_metadata = (dsm_entry_metainfo_t *)p_entry;

        TEST_ASSERT_GREATER_THAN(0, m_expected_metadata.trigger_cnt);
        m_expected_metadata.trigger_cnt--;

        TEST_ASSERT_EQUAL_UINT16(DSM_SUBNET_MAX, p_metadata->max_subnets);
        TEST_ASSERT_EQUAL_UINT16(DSM_DEVICE_MAX, p_metadata->max_devkeys);
        TEST_ASSERT_EQUAL_UINT16(DSM_APP_MAX, p_metadata->max_appkeys);
        TEST_ASSERT_EQUAL_UINT16(DSM_VIRTUAL_ADDR_MAX, p_metadata->max_addrs_virtual);
        TEST_ASSERT_EQUAL_UINT16(DSM_NONVIRTUAL_ADDR_MAX, p_metadata->max_addrs_nonvirtual);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_dsm_metadata_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

static void nrf_mesh_evt_handler_add_stub_cb(nrf_mesh_evt_handler_t * p_handler_params, int cmock_num_calls)
{
    (void) cmock_num_calls;
    m_mesh_evt_handler = *p_handler_params;
}

#if MESH_FEATURE_LPN_ENABLED
static void helper_trigger_event(nrf_mesh_evt_type_t event_type)
{
    nrf_mesh_evt_t evt;

    switch (event_type)
    {
        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            m_test_in_friendship = false;
            evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
            break;
        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            m_test_in_friendship = true;
            evt.params.friendship_established.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
            break;
        case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
            break;
        default:
            TEST_ASSERT_TRUE(false);
            break;
    }

    evt.type = event_type;
    m_mesh_evt_handler.evt_cb(&evt);
}
#endif

void setUp(void)
{
    nrf_mesh_mock_Init();
    nrf_mesh_keygen_mock_Init();
    net_state_mock_Init();
    nrf_mesh_events_mock_Init();
    proxy_mock_Init();
    mesh_opt_core_mock_Init();
    mesh_lpn_mock_Init();
    mesh_lpn_internal_mock_Init();
    heartbeat_mock_Init();
    mesh_opt_friend_mock_Init();
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    nrf_mesh_evt_handler_add_ExpectAnyArgs();

    mesh_config_entry_set_StubWithCallback(entry_set_cb);
    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_evt_handler_add_stub_cb);
    dsm_init();

    /* Ignore the subnet added call by default, it's tested in test_net: */
    nrf_mesh_subnet_added_Ignore();
}

void tearDown(void)
{
    mesh_config_file_clear_Expect(MESH_OPT_DSM_FILE_ID);
    dsm_clear();

    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_keygen_mock_Verify();
    nrf_mesh_keygen_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    mesh_opt_core_mock_Verify();
    mesh_opt_core_mock_Destroy();
    proxy_mock_Verify();
    proxy_mock_Destroy();
    mesh_lpn_mock_Verify();
    mesh_lpn_mock_Destroy();
    mesh_lpn_internal_mock_Verify();
    mesh_lpn_internal_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    mesh_opt_friend_mock_Verify();
    mesh_opt_friend_mock_Destroy();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
}

static void persist_expected_metadata(void)
{
    m_expected_metadata.trigger_cnt++;
}

static void check_metainfo(void)
{
    dsm_entry_metainfo_t data;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_METADATA_RECORD
    };

    m_dsm_metadata_params.callbacks.getter(id, &data);
    TEST_ASSERT_EQUAL_UINT16(DSM_SUBNET_MAX, data.max_subnets);
    TEST_ASSERT_EQUAL_UINT16(DSM_APP_MAX, data.max_appkeys);
    TEST_ASSERT_EQUAL_UINT16(DSM_DEVICE_MAX, data.max_devkeys);
    TEST_ASSERT_EQUAL_UINT16(DSM_NONVIRTUAL_ADDR_MAX, data.max_addrs_nonvirtual);
    TEST_ASSERT_EQUAL_UINT16(DSM_VIRTUAL_ADDR_MAX, data.max_addrs_virtual);
}

static void persist_expect_unicast(dsm_local_unicast_address_t * p_addr, bool is_call_from_dsm)
{
    m_expected_unicast_entry.is_dummy = is_call_from_dsm;
    m_expected_unicast_entry.trigger_cnt++;
    m_expected_unicast_entry.unicast_info.addr = *p_addr;
}

static void check_stored_unicat(dsm_local_unicast_address_t * p_addr)
{
    dsm_local_unicast_address_t stored_addr;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_UNICAST_ADDR_RECORD
    };

    m_dsm_unicast_addr_params.callbacks.getter(id, &stored_addr);
    TEST_ASSERT_EQUAL_UINT16(p_addr->address_start, stored_addr.address_start);
    TEST_ASSERT_EQUAL_UINT16(p_addr->count, stored_addr.count);
    TEST_ASSERT_EQUAL_INT(0, m_expected_unicast_entry.trigger_cnt);
}

static void persist_expect_addr_nonvirtual(uint16_t addr, bool is_call_from_dsm)
{
    m_expected_nonvirtual_entry.is_dummy = is_call_from_dsm;
    m_expected_nonvirtual_entry.trigger_cnt++;
    m_expected_nonvirtual_entry.nonvirtual_info.addr = addr;
}

static void check_stored_addr_nonvirtual(uint16_t addr, dsm_handle_t handle)
{
    uint16_t stored_addr;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + handle
    };

    m_dsm_nonvirtual_addr_params.callbacks.getter(id, &stored_addr);
    TEST_ASSERT_EQUAL_UINT16(addr, stored_addr);
    TEST_ASSERT_EQUAL_INT(0, m_expected_nonvirtual_entry.trigger_cnt);
}

static void persist_expect_addr_virtual(const uint8_t * p_uuid, bool is_call_from_dsm)
{
    m_expected_virtual_entry.trigger_cnt++;
    m_expected_virtual_entry.is_dummy = is_call_from_dsm;

    memcpy(m_expected_virtual_entry.virtual_info.uuid, p_uuid, NRF_MESH_UUID_SIZE);
}

static void check_stored_addr_virtual(const uint8_t * p_uuid, dsm_handle_t handle)
{
    uint8_t stored_addr[NRF_MESH_UUID_SIZE];
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + handle - DSM_NONVIRTUAL_ADDR_MAX
    };

    m_dsm_virtual_addr_params.callbacks.getter(id, stored_addr);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_uuid, stored_addr, NRF_MESH_UUID_SIZE);
    TEST_ASSERT_EQUAL_INT(0, m_expected_virtual_entry.trigger_cnt);
}

static void persist_expect_subnet(const uint8_t * p_key, uint16_t key_index, bool is_call_from_dsm)
{
    m_expected_subnet_entry.trigger_cnt++;
    m_expected_subnet_entry.is_dummy = is_call_from_dsm;

    memset(&m_expected_subnet_entry.subnet_info, 0, sizeof(dsm_entry_subnet_t));
    memcpy(m_expected_subnet_entry.subnet_info.key, p_key, NRF_MESH_KEY_SIZE);
    m_expected_subnet_entry.subnet_info.key_index = key_index;
    m_expected_subnet_entry.subnet_info.key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
}

static void persist_expect_subnet_update(const uint8_t * p_old_key,
                                         const uint8_t * p_new_key,
                                         uint16_t key_index,
                                         nrf_mesh_key_refresh_phase_t key_refresh_phase,
                                         dsm_handle_t handle,
                                         bool is_call_from_dsm)
{
    m_expected_subnet_entry.trigger_cnt++;
    m_expected_subnet_entry.is_dummy = is_call_from_dsm;

    memset(&m_expected_subnet_entry.subnet_info, 0, sizeof(dsm_entry_subnet_t));
    memcpy(m_expected_subnet_entry.subnet_info.key, p_old_key, NRF_MESH_KEY_SIZE);
    memcpy(m_expected_subnet_entry.subnet_info.key_updated, p_new_key, NRF_MESH_KEY_SIZE);
    m_expected_subnet_entry.subnet_info.key_index = key_index;
    m_expected_subnet_entry.subnet_info.key_refresh_phase = key_refresh_phase;
}

static void check_stored_subnet(const uint8_t * p_key,
                                const uint8_t * p_new_key,
                                uint16_t key_index,
                                nrf_mesh_key_refresh_phase_t key_refresh_phase,
                                dsm_handle_t handle)
{
    dsm_entry_subnet_t stored_subnet;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_SUBNETS_RECORD + handle
    };

    m_dsm_subnet_params.callbacks.getter(id, &stored_subnet);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_key, stored_subnet.key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_UINT16(key_index, stored_subnet.key_index);

    if (key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        TEST_ASSERT_EQUAL(key_refresh_phase, stored_subnet.key_refresh_phase);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(p_new_key, stored_subnet.key_updated, NRF_MESH_KEY_SIZE);
    }

    TEST_ASSERT_EQUAL_INT(0, m_expected_subnet_entry.trigger_cnt);
}

static void persist_expect_appkey(const uint8_t * p_key,
                                  uint16_t key_index,
                                  dsm_handle_t subnet_handle,
                                  bool is_call_from_dsm)
{
    m_expected_appkey_entry.trigger_cnt++;
    m_expected_appkey_entry.is_dummy = is_call_from_dsm;

    memset(&m_expected_appkey_entry.appkey_info, 0, sizeof(dsm_entry_appkey_t));
    memcpy(m_expected_appkey_entry.appkey_info.key, p_key, NRF_MESH_KEY_SIZE);
    m_expected_appkey_entry.appkey_info.key_index = key_index;
    m_expected_appkey_entry.appkey_info.subnet_handle = subnet_handle;
    m_expected_appkey_entry.appkey_info.is_key_updated = false;
}

static void persist_expect_appkey_update(const uint8_t * p_old_key,
                                         const uint8_t * p_new_key,
                                         uint16_t key_index,
                                         dsm_handle_t subnet_handle,
                                         dsm_handle_t app_handle,
                                         bool is_call_from_dsm)
{
    m_expected_appkey_entry.trigger_cnt++;
    m_expected_appkey_entry.is_dummy = is_call_from_dsm;

    memset(&m_expected_appkey_entry.appkey_info, 0, sizeof(dsm_entry_appkey_t));
    memcpy(m_expected_appkey_entry.appkey_info.key, p_old_key, NRF_MESH_KEY_SIZE);
    m_expected_appkey_entry.appkey_info.key_index = key_index;
    m_expected_appkey_entry.appkey_info.subnet_handle = subnet_handle;
    m_expected_appkey_entry.appkey_info.is_key_updated = true;
    memcpy(m_expected_appkey_entry.appkey_info.key_updated, p_new_key, NRF_MESH_KEY_SIZE);
}

static void check_stored_appkey(const uint8_t * p_key,
                                const uint8_t * p_new_key,
                                uint16_t key_index,
                                dsm_handle_t subnet_handle,
                                dsm_handle_t app_handle,
                                bool is_key_updated)
{
    dsm_entry_appkey_t stored_appkey;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_APPKEYS_RECORD + app_handle
    };

    m_dsm_appkey_params.callbacks.getter(id, &stored_appkey);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_key, stored_appkey.key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_UINT16(key_index, stored_appkey.key_index);
    TEST_ASSERT_EQUAL_UINT16(subnet_handle, stored_appkey.subnet_handle);

    if (is_key_updated)
    {
        TEST_ASSERT_EQUAL(is_key_updated, stored_appkey.is_key_updated);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(p_new_key, stored_appkey.key_updated, NRF_MESH_KEY_SIZE);
    }

    TEST_ASSERT_EQUAL_INT(0, m_expected_appkey_entry.trigger_cnt);
}

static void persist_expect_devkey(const uint8_t * p_key,
                                  uint16_t key_owner,
                                  dsm_handle_t subnet_handle,
                                  bool is_call_from_dsm)
{
    m_expected_devkey_entry.trigger_cnt++;
    m_expected_devkey_entry.is_dummy = is_call_from_dsm;

    memset(&m_expected_devkey_entry.devkey_info, 0, sizeof(dsm_entry_devkey_t));
    memcpy(m_expected_devkey_entry.devkey_info.key, p_key, NRF_MESH_KEY_SIZE);
    m_expected_devkey_entry.devkey_info.key_owner = key_owner;
    m_expected_devkey_entry.devkey_info.subnet_handle = subnet_handle;
}

static void check_stored_devkey(const uint8_t * p_key,
                                uint16_t key_owner,
                                dsm_handle_t subnet_handle,
                                dsm_handle_t dev_handle)
{
    dsm_entry_devkey_t stored_devkey;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = MESH_OPT_DSM_DEVKEYS_RECORD + dev_handle - DSM_APP_MAX
    };

    m_dsm_devkey_params.callbacks.getter(id, &stored_devkey);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_key, stored_devkey.key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_UINT16(key_owner, stored_devkey.key_owner);
    TEST_ASSERT_EQUAL_UINT16(subnet_handle, stored_devkey.subnet_handle);
    TEST_ASSERT_EQUAL_INT(0, m_expected_devkey_entry.trigger_cnt);
}

static void persist_invalidate_expect(uint16_t entry_handle)
{
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
        .record = entry_handle
    };

    mesh_config_entry_delete_ExpectAndReturn(id, NRF_SUCCESS);
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
    nrf_mesh_subnet_added_Expect(p_test_net->key_index, p_test_net->secmat.beacon.net_id);

    /* add the net */
    persist_expect_subnet(p_test_net->key, p_test_net->key_index, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(p_test_net->key_index, p_test_net->key, &p_test_net->handle));
    check_stored_subnet(p_test_net->key, NULL, p_test_net->key_index, NRF_MESH_KEY_REFRESH_PHASE_0, p_test_net->handle);
    TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, p_test_net->handle);
    TEST_ASSERT_NOT_EQUAL(0xABCD, p_test_net->handle); /* The handle must have changed */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_key_get(p_test_net->handle, net_key));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(p_test_net->key, net_key, NRF_MESH_KEY_SIZE);
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

static void subnet_key_refresh_phase_Verify(dsm_handle_t subnet_handle,
                                            nrf_mesh_key_refresh_phase_t expected_phase)
{
        /* Verify the changed key: */
    nrf_mesh_key_refresh_phase_t kr_phase;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(subnet_handle, &kr_phase));
    TEST_ASSERT_EQUAL(expected_phase, kr_phase);
}

static void nid_key_refresh_phase_Verify(uint8_t nid,
                                         nrf_mesh_key_refresh_phase_t expected_phase)
{
    /* Fetch secmat to get the key refresh phase.  */
    const nrf_mesh_network_secmat_t * p_primary_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_secondary_secmat = NULL;
    nrf_mesh_net_secmat_next_get(nid, &p_primary_secmat, &p_secondary_secmat);
    if (p_primary_secmat != NULL)
    {
        TEST_ASSERT_EQUAL(expected_phase, nrf_mesh_key_refresh_phase_get(p_primary_secmat));
    }
    else if (p_secondary_secmat != NULL)
    {
        TEST_ASSERT_EQUAL(expected_phase, nrf_mesh_key_refresh_phase_get(p_secondary_secmat));
    }
    else
    {
        char msg[64];
        (void) sprintf(msg, "Neither primary or secondary secmat matched NID %#x", nid);
        TEST_FAIL_MESSAGE(msg);
    }
}

static void friendship_key_refresh_phase_Verify(uint16_t lpn_address,
                                                nrf_mesh_key_refresh_phase_t expected_phase)
{
    const nrf_mesh_network_secmat_t * p_friendship_secmat = NULL;
    nrf_mesh_friendship_secmat_get(lpn_address, &p_friendship_secmat);
    TEST_ASSERT_NOT_NULL(p_friendship_secmat);
    TEST_ASSERT_EQUAL(expected_phase, nrf_mesh_key_refresh_phase_get(p_friendship_secmat));
}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_addresses(void)
{
    /* local unicast */
    dsm_local_unicast_address_t unicast_get;

    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, unicast_get.address_start);
    TEST_ASSERT_EQUAL(0, unicast_get.count);

    dsm_local_unicast_address_t unicast = { 0x0000, 1};
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0xF000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0x8000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.address_start = 0x0002;
    unicast.count = 0x8000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    unicast.count = 0x0000;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_local_unicast_addresses_set(&unicast));
    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, unicast_get.address_start);
    TEST_ASSERT_EQUAL(0, unicast_get.count);
    unicast.count = 0x0003;
    persist_expect_unicast(&unicast, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_local_unicast_addresses_set(&unicast));
    check_stored_unicat(&unicast);
    dsm_local_unicast_addresses_get(&unicast_get);
    TEST_ASSERT_EQUAL(2, unicast_get.address_start);
    TEST_ASSERT_EQUAL(3, unicast_get.count);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);

    /**** add addresses ****/
    struct
    {
        dsm_handle_t handle;
        uint16_t raw_address;
        bool in_list;
        bool duplicate;
        uint32_t expected_status;
    } addrs[] =
    {
        /* init the handle with something the module is unlikely to use, to
         * ensure that the module always changes the handle if successful. */
        {0xABAB,  0x1234, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0xFFFF, true,  false, NRF_SUCCESS},            /* group (all) */
        {0xABAB,  0xF010, true,  false, NRF_SUCCESS},            /* group */
        {0xABAB,  0x0000, false, false, NRF_ERROR_INVALID_ADDR}, /* invalid */
        {0xABAB,  0x80AB, false, false, NRF_ERROR_INVALID_ADDR}, /* virtual */
        {0xABAB,  0x1234, true,  true,  NRF_SUCCESS},            /* duplicate */
        {0xABAB,  0x0001, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0002, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0003, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0004, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0005, true,  false, NRF_SUCCESS},            /* unicast */
        {0xABAB,  0x0006, false, false, NRF_ERROR_NO_MEM},       /* full */ //FIXME: As this is a unicast address, it should probably not generate this error?
        {0xABAB,  0x0007, false, false, NRF_ERROR_NO_MEM},       /* full */
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u] (0x%04x)", i, addrs[i].raw_address);
        if (addrs[i].in_list && !addrs[i].duplicate)
        {
            persist_expect_addr_nonvirtual(addrs[i].raw_address, true);
        }
        TEST_ASSERT_EQUAL_MESSAGE(addrs[i].expected_status, dsm_address_publish_add(addrs[i].raw_address, &addrs[i].handle), errormsg);
        if (addrs[i].expected_status == NRF_SUCCESS)
        {
            check_stored_addr_nonvirtual(addrs[i].raw_address, addrs[i].handle);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(DSM_HANDLE_INVALID, addrs[i].handle, errormsg);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(0xABAB, addrs[i].handle, errormsg); /* The handle must have changed */
        }
    }

    /* illegal params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_add(0x0F12, NULL));

    /**** Remove addresses ****/
    persist_invalidate_expect(addrs[6].handle + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[6].handle));
    persist_invalidate_expect(addrs[7].handle + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[7].handle));
    persist_invalidate_expect(addrs[8].handle + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(addrs[8].handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(addrs[6].handle)); /* already deleted */

    const dsm_handle_t deleted_handle = addrs[8].handle;

    nrf_mesh_address_t addr;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(addrs[6].handle, &addr)); /* get deleted handle */

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(0x8888)); /* out of bounds */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_publish_remove(DSM_HANDLE_INVALID));
    addrs[6].in_list = false;
    addrs[7].in_list = false;
    addrs[8].in_list = false;

    /* Addresses 6, 7 and 8 are now removed. */

    /* re-add previous handle 6. */
    persist_expect_addr_nonvirtual(addrs[6].raw_address, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(addrs[6].raw_address, &addrs[6].handle));
    check_stored_addr_nonvirtual(addrs[6].raw_address, addrs[6].handle);
    TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, addrs[6].handle);
    addrs[6].in_list = true;

    /**** Get addresses ****/
    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u] (0x%04x)", i, addrs[i].raw_address);
        if (addrs[i].in_list)
        {
            TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, dsm_address_get(addrs[i].handle, &addr), errormsg);
            nrf_mesh_address_type_t expected_type;
            if ((addr.value & 0xC000) == 0xC000)
            {
                expected_type = NRF_MESH_ADDRESS_TYPE_GROUP;
            }
            else
            {
                expected_type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            }
            TEST_ASSERT_EQUAL_PTR(NULL, addr.p_virtual_uuid);

            //FIXME: This fails for address 0xFFFF, as that is ignored by the module
            TEST_ASSERT_EQUAL_MESSAGE(expected_type, addr.type, errormsg);
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(addrs[i].raw_address, addr.value, errormsg);

            /* Get the handle back */
            dsm_handle_t handle = 0xABAB;
            TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, dsm_address_handle_get(&addr, &handle), errormsg);
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(addrs[i].handle, handle, errormsg);
        }
        else if (addrs[i].handle != DSM_HANDLE_INVALID)
        {
            TEST_ASSERT_EQUAL_MESSAGE(NRF_ERROR_NOT_FOUND, dsm_address_get(addrs[i].handle, &addr), errormsg);
        }
    }
    /* Invalid params: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(DSM_HANDLE_INVALID, &addr));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_get(deleted_handle, &addr)); /* deleted handle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get(0, NULL));
    dsm_handle_t handle = 0xABAB;
    addr.value = 0;
    addr.type = NRF_MESH_ADDRESS_TYPE_INVALID;
    addr.p_virtual_uuid = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, dsm_address_handle_get(&addr, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_handle_get(NULL, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_handle_get(&addr, NULL));
    addr.value = 0x01239;
    addr.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_handle_get(&addr, &handle));

    /**** Virtual addresses ****/
    struct
    {
        uint8_t uuid[NRF_MESH_UUID_SIZE];
        dsm_handle_t handle;
        uint16_t generated_short_addr;
        uint32_t expected_status;
    } virtual_uuids[] =
    {
        {{0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8001, NRF_SUCCESS},
        {{1, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8003, NRF_SUCCESS},
        {{2, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8004, NRF_SUCCESS},
        {{3, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8005, NRF_SUCCESS},
        {{4, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8006, NRF_ERROR_NO_MEM}, /* full */
        {{5, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 0xABAB, 0x8007, NRF_ERROR_NO_MEM}, /* full */
    };
    nrf_mesh_keygen_mock_Verify();
    for (uint32_t i = 0; i < ARRAY_SIZE(virtual_uuids); i++)
    {
        char errormsg[128];
        sprintf(errormsg, "ADDR[%u]", i);
        if (virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuids[i].uuid, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
            nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_uuids[i].generated_short_addr, 2);
            persist_expect_addr_virtual(virtual_uuids[i].uuid, true);
        }
        TEST_ASSERT_EQUAL_MESSAGE(virtual_uuids[i].expected_status, dsm_address_publish_virtual_add(virtual_uuids[i].uuid, &virtual_uuids[i].handle), errormsg);
        nrf_mesh_keygen_mock_Verify();
        if (virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            check_stored_addr_virtual(virtual_uuids[i].uuid, virtual_uuids[i].handle);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(DSM_HANDLE_INVALID, virtual_uuids[i].handle, errormsg);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(0xABAB, virtual_uuids[i].handle, errormsg); /* The handle must have changed. */
        }
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_virtual_add(NULL, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_publish_virtual_add(virtual_uuids[0].uuid, NULL));
    nrf_mesh_keygen_mock_Verify();

    /* delete virtual addresses */
    persist_invalidate_expect(virtual_uuids[2].handle - DSM_NONVIRTUAL_ADDR_MAX + MESH_OPT_DSM_VIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(virtual_uuids[2].handle));
    /* re-add, as we have the space again */
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuids[2].uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_uuids[2].generated_short_addr, 2);
    persist_expect_addr_virtual(virtual_uuids[2].uuid, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_virtual_add(virtual_uuids[2].uuid, &virtual_uuids[2].handle));
    check_stored_addr_virtual(virtual_uuids[2].uuid, virtual_uuids[2].handle);
    /* delete again */
    persist_invalidate_expect(virtual_uuids[2].handle - DSM_NONVIRTUAL_ADDR_MAX + MESH_OPT_DSM_VIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(virtual_uuids[2].handle));
    virtual_uuids[2].handle = DSM_HANDLE_INVALID; /* mark as deleted in our book keeping */

    /* get virtual addrs */
    for (uint32_t i = 0; i < ARRAY_SIZE(virtual_uuids); i++)
    {
        if (virtual_uuids[i].handle != DSM_HANDLE_INVALID && virtual_uuids[i].expected_status == NRF_SUCCESS)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get(virtual_uuids[i].handle, &addr));
            TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_VIRTUAL, addr.type);
            TEST_ASSERT_NOT_NULL(addr.p_virtual_uuid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(virtual_uuids[i].uuid, addr.p_virtual_uuid, NRF_MESH_UUID_SIZE);
            TEST_ASSERT_EQUAL_HEX16(virtual_uuids[i].generated_short_addr, addr.value);

            /* Get the handle back */
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_handle_get(&addr, &handle));
            TEST_ASSERT_EQUAL_HEX16(virtual_uuids[i].handle, handle);
        }
    }

    /* Test get all */
    dsm_handle_t addr_list[DSM_ADDR_MAX];
    memset(addr_list, 0x1B, sizeof(addr_list));
    uint32_t count = DSM_ADDR_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(addr_list, &count));

    /* Make sure it holds all our addresses, and nothing else: */
    uint32_t expected_count = 0;
    bool found[DSM_ADDR_MAX];
    memset(found, 0, sizeof(found));

    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); i++)
    {
        bool address_found = false;
        if (addrs[i].in_list && !addrs[i].duplicate)
        {
            expected_count++;
        }
        for (uint32_t j = 0; j < count; j++)
        {
            if (addr_list[j] == addrs[i].handle)
            {
                TEST_ASSERT_TRUE(addrs[i].in_list);

                /* Make sure we don't find multiple of the same handle in the output. */
                TEST_ASSERT_FALSE(address_found);
                /* mark the address in this list found. */
                found[j] = true;
                break;
            }
        }
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(virtual_uuids); i++)
    {
        bool expected_in_list = (virtual_uuids[i].handle != DSM_HANDLE_INVALID && virtual_uuids[i].expected_status == NRF_SUCCESS);
        for (uint32_t j = 0; j < count; j++)
        {
            if (addr_list[j] == virtual_uuids[i].handle)
            {
                TEST_ASSERT_TRUE(expected_in_list);
                /* Make sure we don't find multiple of the same handle in the
                 * output. */
                TEST_ASSERT_FALSE(found[j]);
                /* mark the address in this list found. */
                found[j] = true;
                break;
            }
        }
        if (expected_in_list)
        {
            expected_count++;
        }
    }
    /* Check that all the outputted handles were recognized as something
     * we've inputted ourselves. */
    for (uint32_t i = 0; i < count; i++)
    {
        TEST_ASSERT_TRUE(found[i]);
    }
    TEST_ASSERT_EQUAL(expected_count, count);

    dsm_handle_t addr_list2[DSM_ADDR_MAX];
    /* Run again with exact length, should have the same result. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(addr_list2, &count));
    TEST_ASSERT_EQUAL(expected_count, count);
    TEST_ASSERT_EQUAL_HEX16_ARRAY(addr_list, addr_list2, count);

    /* Can't fit the last entry */
    count--;

    dsm_handle_t addr_list3[DSM_ADDR_MAX];
    memset(addr_list3, 0x1B, sizeof(addr_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_address_get_all(addr_list3, &count));
    TEST_ASSERT_EQUAL_HEX16_ARRAY(addr_list, addr_list3, count); /* Should be the same except for the last (missing) entry. */
    TEST_ASSERT_EQUAL_HEX16(0x1B1B, addr_list3[count]); /* last entry should be untouched. */

    /* invalid params */
    count = 0;
    memset(addr_list3, 0x1B, sizeof(addr_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_address_get_all(addr_list3, &count));
    TEST_ASSERT_EQUAL_HEX16(0x1B1B, addr_list3[0]); /* even the first entry should be untouched. */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(NULL, &count));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(NULL, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_address_get_all(addr_list3, NULL));
}

void test_rx_addr(void)
{
    /* add some addresses */
    dsm_handle_t handles[3];

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    persist_expect_addr_nonvirtual(0xF001, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xF001, &handles[0]));
    check_stored_addr_nonvirtual(0xF001, handles[0]);
    persist_expect_addr_nonvirtual(0xF002, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xF002, &handles[1]));
    check_stored_addr_nonvirtual(0xF002, handles[1]);
    uint8_t uuid[NRF_MESH_UUID_SIZE] = {};
    uint16_t virtual_short_addr = 0x8001;
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnMemThruPtr_p_address(&virtual_short_addr, sizeof(virtual_short_addr));
    persist_expect_addr_virtual(uuid, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(uuid, &handles[2]));
    check_stored_addr_virtual(uuid, handles[2]);

    /* Add them all as rx addresses */
    dsm_local_unicast_address_t unicast = { 0x0001, 1};
    persist_expect_unicast(&unicast, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_local_unicast_addresses_set(&unicast));
    check_stored_unicat(&unicast);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, dsm_local_unicast_addresses_set(&unicast)); /* add again, should generate an error */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(DSM_HANDLE_INVALID));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(0x8887)); /* never referenced before */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(0x8888)); /* never referenced before */

    /* Test subscription getter */
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[1]));
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[2]));
    TEST_ASSERT_EQUAL(true, dsm_address_subscription_get(handles[0]));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(0x8887));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(0x8888));
    TEST_ASSERT_EQUAL(false, dsm_address_subscription_get(DSM_HANDLE_INVALID));

    /* Test _is_rx */
    nrf_mesh_address_t known_addrs[] = {
        {NRF_MESH_ADDRESS_TYPE_GROUP, 0xF001},
        {NRF_MESH_ADDRESS_TYPE_GROUP, 0xF002},
        {NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0x8001, uuid},
        {NRF_MESH_ADDRESS_TYPE_UNICAST, unicast.address_start},
    };
    nrf_mesh_address_t unknown_addrs[] = {
        {NRF_MESH_ADDRESS_TYPE_GROUP, 0x8887},
        {NRF_MESH_ADDRESS_TYPE_GROUP, 0x8888},
        {NRF_MESH_ADDRESS_TYPE_INVALID, 0},
        {NRF_MESH_ADDRESS_TYPE_UNICAST, unicast.address_start + unicast.count},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(known_addrs); ++i)
    {
        TEST_ASSERT_TRUE(nrf_mesh_is_address_rx(&known_addrs[i]));
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(unknown_addrs); ++i)
    {
        TEST_ASSERT_FALSE(nrf_mesh_is_address_rx(&unknown_addrs[i]));
    }

    /* Test core lookup of rx addresses */
    nrf_mesh_address_t addr;

    // ignore all heartbeat polling first, then explicitly test it later:
    heartbeat_subscription_state_t hb_sub = {
        .dst = 0xFF00 // a group address that isn't the rx list
    };
    heartbeat_subscription_get_IgnoreAndReturn(&hb_sub);

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(0xF001, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_GROUP, addr.type);
    TEST_ASSERT_EQUAL_HEX16(0xF001, addr.value);

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(0xF002, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_GROUP, addr.type);
    TEST_ASSERT_EQUAL_HEX16(0xF002, addr.value);

    /* Remove the second address from the RX list */
    persist_invalidate_expect(handles[1] + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(handles[1]));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(handles[1])); /* re-remove, shouldn't work */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_add_handle(handles[1])); /* re-add the handle, shouldn't work */

    /* second address no longer available */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0xF002, &addr));

    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(virtual_short_addr, &addr));
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_VIRTUAL, addr.type);
    TEST_ASSERT_NOT_NULL(addr.p_virtual_uuid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(uuid, addr.p_virtual_uuid, NRF_MESH_UUID_SIZE);
    TEST_ASSERT_EQUAL_HEX16(virtual_short_addr, addr.value);

    /* invalid addresses */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x0002, &addr)); /* unicast */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0xF002, &addr)); /* group */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x8002, &addr)); /* virtual */
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(0x0000, &addr)); /* invalid */

    /* Spec-defined addresses */
    bool is_friend_enabled = true;
    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(NRF_MESH_ALL_NODES_ADDR, &addr));
    mesh_opt_friend_get_ExpectAndReturn(NULL, NRF_SUCCESS);
    mesh_opt_friend_get_IgnoreArg_p_enabled();
    mesh_opt_friend_get_ReturnThruPtr_p_enabled(&is_friend_enabled);
    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(NRF_MESH_ALL_FRIENDS_ADDR, &addr));
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(NRF_MESH_ALL_PROXIES_ADDR, &addr)); // proxy not supported
    mesh_opt_core_adv_t opt;
    opt.enabled = true;
    mesh_opt_core_adv_get_ExpectAndReturn(CORE_TX_ROLE_RELAY, NULL, NRF_SUCCESS);
    mesh_opt_core_adv_get_IgnoreArg_p_entry();
    mesh_opt_core_adv_get_ReturnThruPtr_p_entry(&opt);
    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(NRF_MESH_ALL_RELAYS_ADDR, &addr));
    opt.enabled = false;
    mesh_opt_core_adv_get_ExpectAndReturn(CORE_TX_ROLE_RELAY, NULL, NRF_SUCCESS);
    mesh_opt_core_adv_get_IgnoreArg_p_entry();
    mesh_opt_core_adv_get_ReturnThruPtr_p_entry(&opt);
    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(NRF_MESH_ALL_RELAYS_ADDR, &addr));

    /* Verify that the spec defined addresses work when calling is_rx as well */
    nrf_mesh_address_t all_nodes = {NRF_MESH_ADDRESS_TYPE_GROUP, NRF_MESH_ALL_NODES_ADDR};
    TEST_ASSERT_TRUE(nrf_mesh_is_address_rx(&all_nodes));
    nrf_mesh_address_t all_friends = {NRF_MESH_ADDRESS_TYPE_GROUP, NRF_MESH_ALL_FRIENDS_ADDR};
    mesh_opt_friend_get_ExpectAndReturn(NULL, NRF_SUCCESS);
    mesh_opt_friend_get_IgnoreArg_p_enabled();
    mesh_opt_friend_get_ReturnThruPtr_p_enabled(&is_friend_enabled);
    TEST_ASSERT_TRUE(nrf_mesh_is_address_rx(&all_friends));
    nrf_mesh_address_t all_proxies = {NRF_MESH_ADDRESS_TYPE_GROUP, NRF_MESH_ALL_PROXIES_ADDR};
    TEST_ASSERT_FALSE(nrf_mesh_is_address_rx(&all_proxies)); // proxy not supported
    nrf_mesh_address_t all_relays = {NRF_MESH_ADDRESS_TYPE_GROUP, NRF_MESH_ALL_RELAYS_ADDR};
    opt.enabled = true;
    mesh_opt_core_adv_get_ExpectAndReturn(CORE_TX_ROLE_RELAY, NULL, NRF_SUCCESS);
    mesh_opt_core_adv_get_IgnoreArg_p_entry();
    mesh_opt_core_adv_get_ReturnThruPtr_p_entry(&opt);
    TEST_ASSERT_TRUE(nrf_mesh_is_address_rx(&all_relays));
    opt.enabled = false;
    mesh_opt_core_adv_get_ExpectAndReturn(CORE_TX_ROLE_RELAY, NULL, NRF_SUCCESS);
    mesh_opt_core_adv_get_IgnoreArg_p_entry();
    mesh_opt_core_adv_get_ReturnThruPtr_p_entry(&opt);
    TEST_ASSERT_FALSE(nrf_mesh_is_address_rx(&all_relays));

    // Test heartbeat RX from core:

    heartbeat_subscription_get_ExpectAndReturn(&hb_sub);
    TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(hb_sub.dst, &addr)); // receive on hb sub
    TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_GROUP, addr.type);
    TEST_ASSERT_EQUAL(hb_sub.dst, addr.value);
    TEST_ASSERT_NULL(addr.p_virtual_uuid);


    //TODO: Test sublist overflow
}

void test_net(void)
{
    struct
    {
        bool in_the_list;
        uint16_t key_index;
        uint8_t nid;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } net[] =
    {
        {true,  0,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS},
        {true,  1,      0x11, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_SUCCESS},
        {true,  2,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS}, /* same key as first, different index (allowed) */
        {false, 0,      0x11, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, different key (not allowed) */
        {false, 0,      0x11, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_ERROR_INTERNAL}, /* same index as first, same key (not allowed). However it is valid situation for config server */
        {false, 0xF000, 0x11, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_ERROR_INVALID_PARAM}, /* key index out of bounds */
        {true,  3,      0x22, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, 0xABCD, NRF_SUCCESS},
        {true,  4,      0x22, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_SUCCESS},
        {true,  5,      0x33, {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5}, 0xABCD, NRF_SUCCESS},
        {true,  6,      0x33, {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6}, 0xABCD, NRF_SUCCESS},
        {true,  7,      0x33, {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}, 0xABCD, NRF_SUCCESS},
        {false, 8,      0x22, {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, 0xABCD, NRF_ERROR_NO_MEM}, /* too many keys */
    };

    nrf_mesh_network_secmat_t net_secmat;
    net_secmat.nid = 0x51;
    memset(net_secmat.privacy_key, 0xAA, NRF_MESH_KEY_SIZE);
    memset(net_secmat.encryption_key, 0xBB, NRF_MESH_KEY_SIZE);

    nrf_mesh_beacon_secmat_t beacon_secmat;
    memset(beacon_secmat.net_id, 0xCC, NRF_MESH_NETID_SIZE);
    memset(beacon_secmat.key, 0xDD, NRF_MESH_KEY_SIZE);

    uint8_t identity_key[NRF_MESH_KEY_SIZE];
    memset(identity_key, 0xEE, NRF_MESH_KEY_SIZE);

    uint8_t net_key[NRF_MESH_KEY_SIZE];
    memset(net_key, 0xFF, NRF_MESH_KEY_SIZE);

    /**** Add ****/
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        net_secmat.nid = net[i].nid;
        /* expected keygen calls */
        if (net[i].in_the_list)
        {
            nrf_mesh_keygen_network_secmat_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net_secmat, sizeof(net_secmat));

            nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&beacon_secmat, sizeof(beacon_secmat));

#if MESH_FEATURE_GATT_PROXY_ENABLED
            nrf_mesh_keygen_identitykey_ExpectAndReturn(net[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
            nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(identity_key, NRF_MESH_KEY_SIZE);
#endif
            persist_expect_subnet(net[i].key, net[i].key_index, true);
            nrf_mesh_subnet_added_Expect(net[i].key_index, beacon_secmat.net_id);
        }

        /* add the net */
        TEST_ASSERT_EQUAL(net[i].expected_status, dsm_subnet_add(net[i].key_index, net[i].key, &net[i].handle));
        if (net[i].in_the_list)
        {
            check_stored_subnet(net[i].key, NULL, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].handle);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, net[i].expected_status);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, net[i].handle);
            TEST_ASSERT_NOT_EQUAL(0xABCD, net[i].handle); /* The handle must have changed */
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_key_get(net[i].handle, net_key));
            TEST_ASSERT_EQUAL_HEX8_ARRAY(net[i].key, net_key, NRF_MESH_KEY_SIZE);
        }
        else
        {
            TEST_ASSERT_NOT_EQUAL(NRF_SUCCESS, net[i].expected_status);
            if (net[i].expected_status == NRF_ERROR_INTERNAL)
            {
                TEST_ASSERT_NOT_EQUAL(0xABCD, net[i].handle);
            }
        }
    }

    /* invalid params */
    dsm_handle_t handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_add(9, NULL, &handle));
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_add(9, key, NULL));

    /* Test friendship */
    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params[] =
    {
        {0x0001, 0x0002, 1, 2},
        {0x0003, 0x0004, 2, 3},
    };

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_friendship_secmat_params_set(NULL, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_friendship_secmat_params_set(NULL, &friendship_secmat_params[0]));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_friendship_secmat_params_set(&net_secmat, NULL));

    const nrf_mesh_network_secmat_t *p_net_secmat_valid[2];
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[0].key_index, &p_net_secmat_valid[0]));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[1].key_index, &p_net_secmat_valid[1]));

    nrf_mesh_network_secmat_t net_secmat_unknown;
    net_secmat_unknown.nid = 0x51;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, nrf_mesh_friendship_secmat_params_set(&net_secmat_unknown,  &friendship_secmat_params[0]));

    /* NOTE: Friendship secmat NID is chosen differently from normal secmat. */
    nrf_mesh_network_secmat_t friendship_secmat =
    {
        .nid = 0x7f,
        .privacy_key = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F},
        .encryption_key = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F},
    };
    nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(net[0].key, &friendship_secmat_params[0], NULL, NRF_SUCCESS);
    nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&friendship_secmat, sizeof(nrf_mesh_network_secmat_t));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid[0], &friendship_secmat_params[0]));

    /**** Delete ****/
    uint8_t delete_indexes[] = {0, 8};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        persist_invalidate_expect(net[delete_indexes[i]].handle + MESH_OPT_DSM_SUBNETS_RECORD);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_delete(net[delete_indexes[i]].handle));
        net[delete_indexes[i]].in_the_list = false;
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_delete(net[delete_indexes[0]].handle));      /* already deleted */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(net[delete_indexes[0]].handle, key)); /* already deleted */

    /**** Re-add ****/
    /* since we've deleted a couple, we should be able to add some more again */
    uint8_t readd_indexes[] = {
        3, /* the one that was rejected for having the same index as net[0], which has been deleted */
        11, /* The one that was rejected because we were full */
    };
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        net_secmat.nid = net[readd_indexes[i]].nid;

        /* expected keygen calls */
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net_secmat, sizeof(net_secmat));

        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&beacon_secmat, sizeof(beacon_secmat));

#if MESH_FEATURE_GATT_PROXY_ENABLED
        nrf_mesh_keygen_identitykey_ExpectAndReturn(net[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_identitykey_IgnoreArg_p_key();
        nrf_mesh_keygen_identitykey_ReturnMemThruPtr_p_key(identity_key, NRF_MESH_KEY_SIZE);
#endif
        nrf_mesh_subnet_added_Expect(net[readd_indexes[i]].key_index, beacon_secmat.net_id);

        persist_expect_subnet(net[readd_indexes[i]].key, net[readd_indexes[i]].key_index, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(net[readd_indexes[i]].key_index, net[readd_indexes[i]].key, &net[readd_indexes[i]].handle));
        check_stored_subnet(net[readd_indexes[i]].key, NULL, net[readd_indexes[i]].key_index, NRF_MESH_KEY_REFRESH_PHASE_0, net[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, net[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(0xABCD, net[readd_indexes[i]].handle); /* The handle must have changed */
        net[readd_indexes[i]].in_the_list = true;
    }

    friendship_secmat.nid--;
    /* Now it shall be possible to add friendship creds again */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[3].key_index, &p_net_secmat_valid[0]));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[11].key_index, &p_net_secmat_valid[1]));
    nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(net[3].key, &friendship_secmat_params[0], NULL, NRF_SUCCESS);
    nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&friendship_secmat, sizeof(nrf_mesh_network_secmat_t));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid[0], &friendship_secmat_params[0]));

    /* Remove friendship creds */
    nrf_mesh_evt_t mesh_friendship_terminated_evt;
    mesh_friendship_terminated_evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
    mesh_friendship_terminated_evt.params.friendship_terminated.lpn_src = friendship_secmat_params[0].lpn_address;
    mesh_friendship_terminated_evt.params.friendship_terminated.friend_src = friendship_secmat_params[0].friend_address;
    m_mesh_evt_handler.evt_cb(&mesh_friendship_terminated_evt);

    /* Add friendship creds again */
    nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(net[3].key, &friendship_secmat_params[0], NULL, NRF_SUCCESS);
    nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&friendship_secmat, sizeof(nrf_mesh_network_secmat_t));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid[0], &friendship_secmat_params[0]));

    /**** Friendship Secmat Get ****/
    const nrf_mesh_network_secmat_t * p_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_aux_secmat = NULL;

    /* Ask with p_secmat eq to NULL */
    nrf_mesh_net_secmat_next_get(friendship_secmat.nid, &p_secmat, &p_aux_secmat);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&friendship_secmat, p_secmat, sizeof(nrf_mesh_network_secmat_t));
    TEST_ASSERT_NULL(p_aux_secmat);

    /* Remove friendship creds to not interfere with rest of the test */
    m_mesh_evt_handler.evt_cb(&mesh_friendship_terminated_evt);

    /**** Get ****/
    uint32_t count = ARRAY_SIZE(net);
    bool found_keys[ARRAY_SIZE(net)];
    memset(found_keys, 0, sizeof(found_keys));
    mesh_key_index_t key_indexes[ARRAY_SIZE(net)];
    memset(key_indexes, 0xAB, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_get_all(key_indexes, &count));

    uint32_t found_key_count = 0;
    for (uint32_t i = 0; i < count; i++)
    {
        for (uint32_t j = 0; j < ARRAY_SIZE(net); j++)
        {
            if (net[j].key_index == key_indexes[i] && net[j].in_the_list)
            {
                TEST_ASSERT_FALSE(found_keys[j]);
                found_key_count++;

                found_keys[j] = true;
                /* don't break on found entry, want to check that we only find a single one */
            }
        }
    }
    /* Check that all keys that were expected to be found were found. */
    for (uint32_t j = 0; j < ARRAY_SIZE(net); j++)
    {
        TEST_ASSERT_EQUAL(found_keys[j], net[j].in_the_list);
    }
    TEST_ASSERT_EQUAL(count, found_key_count);

    /* fetch again, with exact size: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_get_all(key_indexes, &count));
    /* fetch with too few spots */
    count--;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(found_key_count - 1, count);
    /* give 1, only the first one should be set */
    count = 1;
    memset(key_indexes, 0xAB, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_NOT_EQUAL(0xABAB, key_indexes[0]);
    for (uint32_t i = 1; i < ARRAY_SIZE(net); i++)
    {
        TEST_ASSERT_EQUAL_HEX16(0xABAB, key_indexes[i]);
    }
    /* give 0, nothing should be done. */
    count = 0;
    memset(key_indexes, 0x10, sizeof(key_indexes));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_subnet_get_all(key_indexes, &count));
    TEST_ASSERT_EQUAL(0, count);
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        TEST_ASSERT_EQUAL_HEX16(0x1010, key_indexes[i]);
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(NULL, &count));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(key_indexes, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_get_all(NULL, NULL));

    /**** Secmat Get ****/
    struct
    {
        uint8_t nid;
        uint32_t count;
    } nid_groups[] =
    {
        {0x11, 3},
        {0x22, 3},
        {0x33, 2},
    };
    p_secmat = NULL;
    p_aux_secmat = NULL;
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    for (uint32_t i = 0; i < ARRAY_SIZE(nid_groups); i++)
    {
        for (uint32_t j = 0; j < nid_groups[i].count; j++)
        {
            nrf_mesh_net_secmat_next_get(nid_groups[i].nid, &p_secmat, &p_aux_secmat);
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(net_secmat.privacy_key,    p_secmat->privacy_key,    NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(net_secmat.encryption_key, p_secmat->encryption_key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_EQUAL_HEX8(nid_groups[i].nid, p_secmat->nid);

            /* Check that the secmats match their handles */
            handle = dsm_subnet_handle_get(p_secmat);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, handle);
            /* search for this handle */
            bool found = false;
            for (uint32_t k = 0; k < ARRAY_SIZE(net); k++)
            {
                if (net[k].in_the_list && net[k].handle == handle)
                {
                    found = true;
                    TEST_ASSERT_EQUAL_HEX8(nid_groups[i].nid, net[k].nid);
                    break;
                }
            }
            TEST_ASSERT_TRUE(found);
        }
        nrf_mesh_net_secmat_next_get(nid_groups[i].nid, &p_secmat, &p_aux_secmat);
        TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, "Found more networks than expected");
    }

    nrf_mesh_net_secmat_next_get(0x44, &p_secmat, &p_aux_secmat); /* no such nid */
    TEST_ASSERT_EQUAL(NULL, p_secmat);
    nrf_mesh_network_secmat_t dummy_secmat = {};
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_subnet_handle_get(&dummy_secmat)); /* not in the list */

    /* Bind one app and one devkey to net[1] */
    uint8_t dummy_key[NRF_MESH_KEY_SIZE] = {};
    dsm_handle_t app_handle;
    nrf_mesh_keygen_aid_IgnoreAndReturn(NRF_SUCCESS);
    persist_expect_appkey(dummy_key, 0, net[1].handle, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(0, net[1].handle, dummy_key, &app_handle));
    check_stored_appkey(dummy_key, NULL, 0, net[1].handle, app_handle,false);
    dsm_handle_t devkey_handle;
    persist_expect_devkey(dummy_key, 0x0001, net[1].handle, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_add(0x0001, net[1].handle, dummy_key, &devkey_handle));
    check_stored_devkey(dummy_key, 0x0001, net[1].handle, devkey_handle);

    /* Check if the devkey handle retrieval function works: */
    dsm_handle_t test_devkey_handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_handle_get(0x0001, &test_devkey_handle));
    TEST_ASSERT_EQUAL(test_devkey_handle, devkey_handle);

    /* Delete the app */
    persist_invalidate_expect(app_handle + MESH_OPT_DSM_APPKEYS_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_delete(app_handle));
    /* Delete the devkey */
    persist_invalidate_expect(devkey_handle - DSM_APP_MAX + MESH_OPT_DSM_DEVKEYS_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(devkey_handle));
    /* Delete the network and the keys. */
    persist_invalidate_expect(net[1].handle + MESH_OPT_DSM_SUBNETS_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_delete(net[1].handle));

    /* Check that the appkey and the devkey were actually deleted: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(app_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(devkey_handle));
}

void test_app(void)
{
    struct
    {
        bool in_the_list;
        uint16_t key_index;
        dsm_handle_t net_handle;
        uint8_t aid;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } app[] =
    {
        {true,  0,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS},
        {true,  1,      DSM_HANDLE_INVALID, 0x01, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_SUCCESS},
        {true,  2,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_SUCCESS}, /* same key as first, different index (allowed) */
        {false, 0,      DSM_HANDLE_INVALID, 0x01, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* same index as first, different key (not allowed) */
        {false, 0,      DSM_HANDLE_INVALID, 0x01, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 0xABCD, NRF_ERROR_INTERNAL}, /* same index as first, same key (not allowed). However it is valid situation for config server */
        {false, 0xF000, DSM_HANDLE_INVALID, 0x01, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, 0xABCD, NRF_ERROR_INVALID_PARAM}, /* key index out of bounds */
        {true,  3,      DSM_HANDLE_INVALID, 0x02, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3}, 0xABCD, NRF_SUCCESS},
        {true,  4,      DSM_HANDLE_INVALID, 0x02, {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 0xABCD, NRF_SUCCESS},
        {true,  5,      DSM_HANDLE_INVALID, 0x03, {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5}, 0xABCD, NRF_SUCCESS},
        {true,  6,      DSM_HANDLE_INVALID, 0x03, {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6}, 0xABCD, NRF_SUCCESS},
        {true,  7,      DSM_HANDLE_INVALID, 0x03, {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}, 0xABCD, NRF_SUCCESS},
        {false, 8,      DSM_HANDLE_INVALID, 0x02, {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, 0xABCD, NRF_ERROR_NO_MEM}, /* too many keys */
    };

    /* Add two networks, and assign the apps to different ones */
    dsm_handle_t net_handles[2];
    uint8_t net_key[NRF_MESH_KEY_SIZE] = {};
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);

    persist_expect_subnet(net_key, 0, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, net_key, &net_handles[0]));
    check_stored_subnet(net_key, NULL, 0, NRF_MESH_KEY_REFRESH_PHASE_0, net_handles[0]);
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);
    persist_expect_subnet(net_key, 1, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(1, net_key, &net_handles[1]));
    check_stored_subnet(net_key, NULL, 1, NRF_MESH_KEY_REFRESH_PHASE_0, net_handles[1]);

    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        app[i].net_handle = net_handles[i % 2];
    }

    /**** Add ****/
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        /* expected keygen calls */
        if (app[i].in_the_list)
        {
            nrf_mesh_keygen_aid_ExpectAndReturn(app[i].key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_aid_IgnoreArg_p_aid();
            nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, 1);
            persist_expect_appkey(app[i].key, app[i].key_index, app[i].net_handle, true);
        }

        /* add the app */
        TEST_ASSERT_EQUAL(app[i].expected_status, dsm_appkey_add(app[i].key_index, app[i].net_handle, app[i].key, &app[i].handle));
        if (app[i].in_the_list)
        {
            check_stored_appkey(app[i].key, NULL, app[i].key_index, app[i].net_handle, app[i].handle, false);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, app[i].expected_status);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, app[i].handle);
            TEST_ASSERT_NOT_EQUAL(0xABCD, app[i].handle); /* The handle must have changed */
        }
        else
        {
            TEST_ASSERT_NOT_EQUAL(NRF_SUCCESS, app[i].expected_status);
            if (app[i].expected_status == NRF_ERROR_INTERNAL)
            {
                TEST_ASSERT_NOT_EQUAL(0xABCD, app[i].handle);
            }
        }
    }
    /* invalid params */

    dsm_handle_t handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_add(9, net_handles[0], NULL, &handle));
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_add(9, net_handles[0], key, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_add(9, 0xABAB, key, &handle)); /* invalid nethandle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_add(4, 7, key, &handle)); /* unallocated nethandle */

    /**** Delete ****/
    uint8_t delete_indexes[] = {0, 1, 6, 8, 9};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        persist_invalidate_expect(app[delete_indexes[i]].handle + MESH_OPT_DSM_APPKEYS_RECORD);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_delete(app[delete_indexes[i]].handle));
        app[delete_indexes[i]].in_the_list = false;
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(0x8888));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_delete(delete_indexes[0]));      /* already deleted */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(delete_indexes[0], key)); /* already deleted */

    /**** Re-add ****/
    /* since we've deleted a couple, we should be able to add some more again */
    uint8_t readd_indexes[] = {
        3, /* the one that was rejected for having the same index as app[0], which has been deleted */
        11, /* The one that was rejected because we were full */
    };
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        /* expected keygen calls */
        nrf_mesh_keygen_aid_ExpectAndReturn(app[readd_indexes[i]].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[readd_indexes[i]].aid, 1);

        persist_expect_appkey(app[readd_indexes[i]].key, app[readd_indexes[i]].key_index, app[readd_indexes[i]].net_handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(app[readd_indexes[i]].key_index, app[readd_indexes[i]].net_handle, app[readd_indexes[i]].key, &app[readd_indexes[i]].handle));
        check_stored_appkey(app[readd_indexes[i]].key, NULL, app[readd_indexes[i]].key_index, app[readd_indexes[i]].net_handle, app[readd_indexes[i]].handle, false);
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, app[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(0xABCD, app[readd_indexes[i]].handle); /* The handle must have changed */
        app[readd_indexes[i]].in_the_list = true;
    }

    /**** Get ****/
    uint32_t keys_in_storage = 0;

    for (uint32_t net = 0; net < 2; net++)
    {
        uint32_t count = ARRAY_SIZE(app);
        bool found_keys[ARRAY_SIZE(app)];
        memset(found_keys, 0, sizeof(found_keys));
        mesh_key_index_t key_indexes[ARRAY_SIZE(app)];
        memset(key_indexes, 0xAB, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_get_all(net_handles[net], key_indexes, &count));

        uint32_t found_key_count = 0;
        for (uint32_t i = 0; i < count; i++)
        {
            for (uint32_t j = 0; j < ARRAY_SIZE(app); j++)
            {
                if (app[j].key_index == key_indexes[i] && app[j].in_the_list)
                {
                    TEST_ASSERT_FALSE(found_keys[j]);
                    found_key_count++;

                    found_keys[j] = true;
                    /* don't break on found entry, want to check that we only find a single one */
                }
            }
        }
        /* Check that all keys that were expected to be found were found. */
        for (uint32_t j = 0; j < ARRAY_SIZE(app); j++)
        {
            char errormsg[128];
            sprintf(errormsg, "(%d [0x%04x])", j, app[j].key_index);
            TEST_ASSERT_EQUAL_MESSAGE(found_keys[j], (app[j].in_the_list && app[j].net_handle == net_handles[net]), errormsg);
        }
        TEST_ASSERT_EQUAL(count, found_key_count);

        /* fetch again, with exact size: */
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        /* fetch with too few spots */
        count--;
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[1], key_indexes, &count));
        TEST_ASSERT_EQUAL(found_key_count - 1, count);
        /* give 1, only the first one should be set */
        count = 1;
        memset(key_indexes, 0xAB, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        TEST_ASSERT_EQUAL(1, count);
        TEST_ASSERT_NOT_EQUAL(0xABAB, key_indexes[0]);
        for (uint32_t i = 1; i < ARRAY_SIZE(app); i++)
        {
            TEST_ASSERT_EQUAL_HEX16(0xABAB, key_indexes[i]);
        }
        /* give 0, nothing should be done. */
        count = 0;
        memset(key_indexes, 0x10, sizeof(key_indexes));
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, dsm_appkey_get_all(net_handles[net], key_indexes, &count));
        TEST_ASSERT_EQUAL(0, count);
        for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
        {
            TEST_ASSERT_EQUAL_HEX16(0x1010, key_indexes[i]);
        }
        keys_in_storage += found_key_count;

        /* invalid params */
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_get_all(0x8888, key_indexes, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_get_all(DSM_HANDLE_INVALID, key_indexes, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], NULL, &count));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], key_indexes, NULL));
        TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_get_all(net_handles[net], NULL, NULL));
    }

    /**** Secmat Get ****/
    struct
    {
        uint8_t network_index;
        uint8_t aid;
        uint32_t count;
    } aid_groups[] =
    {
        {0, 0x01, 1},
        {0, 0x02, 0},
        {0, 0x03, 1},
        {0, 0x04, 0},
        {1, 0x01, 1},
        {1, 0x02, 2},
        {1, 0x03, 0},
        {1, 0x04, 0},
    };
    const nrf_mesh_network_secmat_t * p_net_secmats[2] = {NULL, NULL};
    for (uint32_t net = 0; net < 2; net++)
    {
        /* Find the right network secmat (not really a part of the test,
         * just necessary for getting the input parameters in
         * app_secmat_next_get())*/
        nrf_mesh_secmat_t secmat;
        for (uint32_t j = 0; j < ARRAY_SIZE(app); j++)
        {
            if (app[j].in_the_list && app[j].net_handle == net_handles[net])
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[j].handle, &secmat));
                TEST_ASSERT_NOT_NULL(secmat.p_net);
                p_net_secmats[net] = secmat.p_net;
                break;
            }
        }
    }
    const nrf_mesh_application_secmat_t * p_secmat = NULL;
    const nrf_mesh_application_secmat_t * p_secmat_secondary = NULL;
    uint32_t represented_aids = 0;
    for (uint32_t i = 0; i < ARRAY_SIZE(aid_groups); i++)
    {
        represented_aids += aid_groups[i].count;
        for (uint32_t net = 0; net < 2; net++)
        {
            if (aid_groups[i].network_index != net)
            {
                continue;
            }
            p_secmat = NULL;
            for (uint32_t j = 0; j < aid_groups[i].count; j++)
            {
                nrf_mesh_app_secmat_next_get(p_net_secmats[net], aid_groups[i].aid, &p_secmat, &p_secmat_secondary);
                TEST_ASSERT_NOT_NULL(p_secmat);
                TEST_ASSERT_EQUAL_HEX8(aid_groups[i].aid, p_secmat->aid);

                /* Check that the secmats match their handles */
                handle = dsm_appkey_handle_get(p_secmat);
                TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, handle);
                /* search for this handle */
                bool found = false;
                for (uint32_t k = 0; k < ARRAY_SIZE(app); k++)
                {
                    if (app[k].in_the_list && app[k].handle == handle)
                    {
                        found = true;
                        TEST_ASSERT_EQUAL(aid_groups[i].aid, app[k].aid);
                        break;
                    }
                }
                TEST_ASSERT_TRUE(found);
            }
            nrf_mesh_app_secmat_next_get(p_net_secmats[net], aid_groups[i].aid, &p_secmat, &p_secmat_secondary);
            TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, "Found more applications than expected");
        }
    }
    /* Check that we've covered all aid's in storage */
    TEST_ASSERT_EQUAL(keys_in_storage, represented_aids);

    /* Illegal params */
    nrf_mesh_app_secmat_next_get(p_net_secmats[0], 0x55, &p_secmat, &p_secmat_secondary); /* no such aid */
    TEST_ASSERT_EQUAL(NULL, p_secmat);
    nrf_mesh_application_secmat_t dummy_secmat = {};
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_appkey_handle_get(&dummy_secmat)); /* not in the list */
    nrf_mesh_network_secmat_t dummy_net_secmat = {};
    nrf_mesh_app_secmat_next_get(&dummy_net_secmat, aid_groups[0].aid, &p_secmat, &p_secmat_secondary); /* net secmat not in the list */
    TEST_ASSERT_EQUAL_PTR(NULL, p_secmat);
    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_app_secmat_next_get(NULL, aid_groups[0].aid, &p_secmat, &p_secmat_secondary));
    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_app_secmat_next_get(p_net_secmats[0], aid_groups[0].aid, NULL, &p_secmat_secondary));
    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_app_secmat_next_get(p_net_secmats[0], aid_groups[0].aid, &p_secmat, NULL));
}

void test_devkey(void)
{
    dsm_handle_t subnet_handle;
    uint8_t dummy_key[NRF_MESH_KEY_SIZE] = {};
    nrf_mesh_keygen_network_secmat_IgnoreAndReturn(NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreAndReturn(NRF_SUCCESS);
    persist_expect_subnet(dummy_key, 0, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, dummy_key, &subnet_handle));
    check_stored_subnet(dummy_key, NULL, 0, NRF_MESH_KEY_REFRESH_PHASE_0, subnet_handle);

    struct
    {
        bool in_the_list;
        bool duplicate;
        uint16_t owner;
        dsm_handle_t net_handle;
        uint8_t key[NRF_MESH_KEY_SIZE];
        dsm_handle_t handle;
        uint32_t expected_status;
    } devkey[] =
    {
        {true,  false, 0x1000, subnet_handle, {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}, 0xABCD, NRF_SUCCESS},
        {true,  false, 0x1001, subnet_handle, {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}, 0xABCD, NRF_SUCCESS},
        {true,  false, 0x1002, subnet_handle, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_SUCCESS},
        {false, false, 0x1009, 0xabcd,        {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_ERROR_NOT_FOUND}, /* Non-existing network handle */
        {true,  false, 0x1003, subnet_handle, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33}, 0xABCD, NRF_SUCCESS}, /* duplicate key (okay) */
        {false, false, 0x0000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, false, 0x8000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, false, 0xC000, subnet_handle, {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}, 0xABCD, NRF_ERROR_INVALID_ADDR}, /* invalid addr */
        {false, true,  0x1002, subnet_handle, {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}, 0xABCD, NRF_ERROR_FORBIDDEN}, /* duplicate address */
        {false, false, 0x1004, subnet_handle, {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66}, 0xABCD, NRF_ERROR_NO_MEM}, /* Full */
    };

    /* Add devkeys */
    for (uint32_t i = 0; i < ARRAY_SIZE(devkey); i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        if (devkey[i].in_the_list && !devkey[i].duplicate)
        {
            persist_expect_devkey(devkey[i].key, devkey[i].owner, devkey[i].net_handle, true);
        }
        TEST_ASSERT_EQUAL_MESSAGE(devkey[i].expected_status, dsm_devkey_add(devkey[i].owner, devkey[i].net_handle, devkey[i].key, &devkey[i].handle), err_msg);
        if (devkey[i].in_the_list)
        {
            check_stored_devkey(devkey[i].key, devkey[i].owner, devkey[i].net_handle, devkey[i].handle);
            TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, devkey[i].handle);
        }
    }
    /* invalid params */
    dsm_handle_t dummy_handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, dsm_devkey_add(NRF_MESH_ADDR_UNASSIGNED, subnet_handle, dummy_key, &dummy_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_devkey_add(0x1234, subnet_handle, NULL, &dummy_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_devkey_add(0x1235, subnet_handle, dummy_key, NULL));

    /* delete some */
    uint8_t delete_indexes[] = {0, 1};
    for (uint32_t i = 0; i < sizeof(delete_indexes); i++)
    {
        persist_invalidate_expect(devkey[delete_indexes[i]].handle - DSM_APP_MAX + MESH_OPT_DSM_DEVKEYS_RECORD);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(devkey[delete_indexes[i]].handle));
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(devkey[delete_indexes[i]].handle)); /* already removed */
        devkey[delete_indexes[i]].in_the_list = false;
    }
    /* invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_devkey_delete(0xABCD));

    /* Re-add, as we have space now */
    uint8_t readd_indexes[] = {9};
    for (uint32_t i = 0; i < sizeof(readd_indexes); i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        persist_expect_devkey(devkey[readd_indexes[i]].key, devkey[readd_indexes[i]].owner, devkey[readd_indexes[i]].net_handle, true);
        TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS,
                dsm_devkey_add(devkey[readd_indexes[i]].owner,
                    devkey[readd_indexes[i]].net_handle,
                    devkey[readd_indexes[i]].key,
                    &devkey[readd_indexes[i]].handle), err_msg);
        check_stored_devkey(devkey[readd_indexes[i]].key, devkey[readd_indexes[i]].owner, devkey[readd_indexes[i]].net_handle, devkey[readd_indexes[i]].handle);
        TEST_ASSERT_NOT_EQUAL(DSM_HANDLE_INVALID, devkey[readd_indexes[i]].handle);
        devkey[readd_indexes[i]].in_the_list = true;
    }

    /* Get rx secmats for all devkeys */
    for (uint32_t i = 0; i < ARRAY_SIZE(devkey); i++)
    {
        char err_msg[128];
        sprintf(err_msg, "Devkey %u", i);
        const nrf_mesh_application_secmat_t * p_secmat = NULL;
        nrf_mesh_devkey_secmat_get(devkey[i].owner, &p_secmat);
        if (devkey[i].in_the_list)
        {
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(devkey[i].key, p_secmat->key, NRF_MESH_KEY_SIZE);
            TEST_ASSERT_TRUE(p_secmat->is_device_key);
            TEST_ASSERT_EQUAL(0, p_secmat->aid);
        }
        else if (devkey[i].duplicate)
        {
            TEST_ASSERT_NOT_NULL(p_secmat);
            TEST_ASSERT_TRUE(p_secmat->is_device_key);
            TEST_ASSERT_EQUAL(0, p_secmat->aid);
        }
        else
        {
            TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, p_secmat, err_msg);
        }
    }
}

void test_secmat(void)
{
    nrf_mesh_secmat_t secmat;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_tx_secmat_get(DSM_HANDLE_INVALID, 0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, 0, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, DSM_APP_MAX + DSM_DEVICE_MAX, &secmat));

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, 0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, 0, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, DSM_APP_MAX + DSM_DEVICE_MAX, &secmat));

    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_info_t beacon_info;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_info.secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_info.secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_info.secmat, sizeof(net[i].beacon_info.secmat));
        persist_expect_subnet(key, i, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i, key, &net[i].handle));
        check_stored_subnet(key, NULL, i, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].handle);
    }
    /* Add some dummy apps */
    struct
    {
        dsm_handle_t handle;
        uint8_t aid;
    } app[6];
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, sizeof(app[i].aid));
        persist_expect_appkey(key, i, net[i / 2].handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, net[i / 2].handle, key, &app[i].handle));
        check_stored_appkey(key, NULL, i, net[i / 2].handle, app[i].handle, false);
    }
    /* Add some dummy device keys */
    struct
    {
        uint16_t owner;
        dsm_handle_t handle;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } dev[4];
    for (uint32_t i = 0; i < ARRAY_SIZE(dev); i++)
    {
        dev[i].owner = i + 0x1000;
        memset(dev[i].key, i, NRF_MESH_KEY_SIZE);
        persist_expect_devkey(dev[i].key, dev[i].owner, net[0].handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_add(dev[i].owner, net[0].handle, dev[i].key, &dev[i].handle));
        check_stored_devkey(dev[i].key, dev[i].owner, net[0].handle, dev[i].handle);
    }

    /* Add some dummy friendships */
    struct
    {
        mesh_key_index_t key_index;
        dsm_handle_t subnet_handle;
        nrf_mesh_keygen_friendship_secmat_params_t secmat_params;
        nrf_mesh_network_secmat_t secmat;
    } friendships[] =
    {
        {0, 0, {0x0001, 0x0002, 1, 2}, {0, {0xAA}, {0xBB}}},
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(friendships); i++)
    {
        const nrf_mesh_network_secmat_t *p_net_secmat_valid;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(friendships[i].key_index, &p_net_secmat_valid));

        nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(key, &friendships[i].secmat_params, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&friendships[i].secmat, sizeof(nrf_mesh_network_secmat_t));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, &friendships[i].secmat_params));
    }

    /* Once all keys are added, check against invalid params */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[0].handle, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, DSM_APP_MAX + DSM_DEVICE_MAX, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, DSM_HANDLE_INVALID, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, app[0].handle, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, DSM_APP_MAX + DSM_DEVICE_MAX, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, DSM_HANDLE_INVALID, &secmat));

    /* Get tx secmats for apps */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_NOT_NULL(secmat.p_app);

        TEST_ASSERT_EQUAL_MEMORY(&net[i / 2].secmat, secmat.p_net, sizeof(nrf_mesh_network_secmat_t));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL(app[i].aid, secmat.p_app->aid);
        TEST_ASSERT_EQUAL(false, secmat.p_app->is_device_key);

        /* Also get tx secmat for friendship */
        for (uint32_t j = 0; j < ARRAY_SIZE(friendships); j++)
        {
            if (friendships[j].subnet_handle == net[i / 2].handle)
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));

                TEST_ASSERT_EQUAL_MEMORY(&friendships[j].secmat, secmat.p_net, sizeof(nrf_mesh_network_secmat_t));
                TEST_ASSERT_EQUAL_HEX8_ARRAY(key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
                TEST_ASSERT_EQUAL(app[i].aid, secmat.p_app->aid);
                TEST_ASSERT_EQUAL(false, secmat.p_app->is_device_key);
            }
        }
    }

    /* Get tx secmats for devkeys */
    for (uint32_t i = 0; i < ARRAY_SIZE(dev); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, dev[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_NOT_NULL(secmat.p_app);

        TEST_ASSERT_EQUAL_MEMORY(&net[0].secmat, secmat.p_net, sizeof(nrf_mesh_network_secmat_t));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(dev[i].key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL(0, secmat.p_app->aid);
        TEST_ASSERT_EQUAL(true, secmat.p_app->is_device_key);
    }
    /* Delete a devkey, ensure we can't get a secmat for it anymore. */
    persist_invalidate_expect(dev[0].handle - DSM_APP_MAX + MESH_OPT_DSM_DEVKEYS_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_devkey_delete(dev[0].handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_tx_secmat_get(DSM_HANDLE_INVALID, dev[0].handle, &secmat));

    /* get beacon info structures */
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_beacon_info_get(net[i].handle, &p_beacon_info));
        TEST_ASSERT_EQUAL_MEMORY(&net[i].beacon_info.secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_beacon_info_get(ARRAY_SIZE(net), &p_beacon_info));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_beacon_info_get(0x8888, &p_beacon_info));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_beacon_info_get(0, NULL));

}

void test_beacon_info_get(void)
{
    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};

    // ONLY ADD TWO FIRST NETWORKS, WE'RE SAVING THE THIRD FOR LATER!
    for (uint32_t i = 0; i < 2; i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));
        persist_expect_subnet(key, i + 10, true);
        // Make sure we don't add net key index 0 yet, as that's the primary network
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i + 10, key, &net[i].handle));
        check_stored_subnet(key, NULL, i + 10, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].handle);
    }
    /* Fetch without filters: */
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    nrf_mesh_key_refresh_phase_t kr_phase;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_timestamp);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->interval);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->observation_count);
    /* Manipulate the tx info, to ensure it'll remain the same across multiple gets */
    p_beacon_info->p_tx_info->rx_count = 89;
    p_beacon_info->p_tx_info->tx_timestamp = 456;
    p_beacon_info->p_tx_info->observation_count = 212;
    p_beacon_info->p_tx_info->interval = 600;

    /* Get the first one again, should yield the same thing! */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(89, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(456, p_beacon_info->p_tx_info->tx_timestamp);
    TEST_ASSERT_EQUAL(600, p_beacon_info->p_tx_info->interval);
    TEST_ASSERT_EQUAL(212, p_beacon_info->p_tx_info->observation_count);

    /* Get the next in the list */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // No primary networks!
    TEST_ASSERT_NOT_NULL(p_beacon_info->p_tx_info);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->rx_count);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->tx_timestamp);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->interval);
    TEST_ASSERT_EQUAL(0, p_beacon_info->p_tx_info->observation_count);

    /* No more networks, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Now add the primary network, with a duplicate network ID. */
    memset(net[2].secmat.privacy_key, 2, NRF_MESH_KEY_SIZE);
    memset(net[2].secmat.encryption_key, 2 + 0x10, NRF_MESH_KEY_SIZE);
    memset(net[2].beacon_secmat.key, 2 + 0x20, NRF_MESH_KEY_SIZE);
    memset(net[2].beacon_secmat.net_id, 0x30, NRF_MESH_NETID_SIZE); // same net ID as the first network
    net[2].secmat.nid = 2;
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[2].secmat, sizeof(net[2].secmat));
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[2].beacon_secmat, sizeof(net[2].beacon_secmat));
    persist_expect_subnet(key, 0, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(0, key, &net[2].handle)); // Network index 0 is the primary network
    check_stored_subnet(key, NULL, 0, NRF_MESH_KEY_REFRESH_PHASE_0, net[2].handle);

    /* Run the getters again, this time, only the primary network should have iv_update_permitted set. */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_FALSE(p_beacon_info->iv_update_permitted); // Not a primary network
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_FALSE(p_beacon_info->iv_update_permitted); // Not a primary network
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[2].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    TEST_ASSERT_TRUE(p_beacon_info->iv_update_permitted); // The primary network!

    /* No more networks, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get with NetID filters: */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[0].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));
    /* Since the last network has the same network ID as the first, we should
     * get that one on the next iteration (but skip the one with a different
     * NetID) */
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[2].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));

    /* No more networks with a matching NetID, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(net[0].beacon_secmat.net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get the unique Net ID */
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(net[1].beacon_secmat.net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_NOT_NULL(p_beacon_info);
    TEST_ASSERT_EQUAL_MEMORY(&net[1].beacon_secmat, &p_beacon_info->secmat, sizeof(nrf_mesh_beacon_secmat_t));

    /* No more networks with a matching NetID, should now get NULL back. */
    nrf_mesh_beacon_info_next_get(net[1].beacon_secmat.net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    /* Get non-existing NetID, no results. */
    uint8_t dummy_net_id[NRF_MESH_NETID_SIZE];
    memset(dummy_net_id, 0xFE, NRF_MESH_NETID_SIZE);
    p_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(dummy_net_id, &p_beacon_info, &kr_phase);
    TEST_ASSERT_EQUAL_PTR(NULL, p_beacon_info);

    TEST_NRF_MESH_ASSERT_EXPECT(nrf_mesh_beacon_info_next_get(NULL, NULL, NULL));
}

void test_address_subcount_regular(void)
{
    uint16_t count = 0xffff;
    dsm_handle_t address_handle;
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    persist_expect_addr_nonvirtual(0xc442, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xc442, &address_handle));
    check_stored_addr_nonvirtual(0xc442, address_handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(0xc442, &address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(2, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    persist_invalidate_expect(address_handle + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_count_get(address_handle, &count));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(address_handle));
}

void test_address_subcount_virtual(void)
{
    const uint8_t virtual_uuid[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    uint16_t virtual_address = 0xabcd; /* This is not the actual address corresponding to the above UUID */
    dsm_handle_t address_handle;
    uint16_t count = 0xffff;

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&virtual_address);
    persist_expect_addr_virtual(virtual_uuid, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(virtual_uuid, &address_handle));
    check_stored_addr_virtual(virtual_uuid, address_handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add_handle(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(2, count);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(address_handle, &count));
    TEST_ASSERT_EQUAL_UINT16(1, count);

    persist_invalidate_expect(address_handle - DSM_NONVIRTUAL_ADDR_MAX + MESH_OPT_DSM_VIRTUAL_ADDR_RECORD);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_count_get(address_handle, &count));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_address_subscription_remove(address_handle));
}

void test_subscription_propagation_to_friend(void)
{
#if MESH_FEATURE_LPN_ENABLED
    subman_test_addr_info_t exp_nv_addr[DSM_NONVIRTUAL_ADDR_MAX];
    subman_test_vaddr_info_t exp_vt_addr[DSM_VIRTUAL_ADDR_MAX];
    uint16_t nv_cnt = 0;
    uint16_t vt_cnt = 0;

    memset(exp_nv_addr, 0, sizeof(exp_nv_addr));
    memset(exp_vt_addr, 0, sizeof(exp_vt_addr));
    printf("DSM_NONVIRTUAL_ADDR_MAX: %d DSM_VIRTUAL_ADDR_MAX: %d\n", DSM_NONVIRTUAL_ADDR_MAX, DSM_VIRTUAL_ADDR_MAX);

    /* Test: DSM propagates existing addresses to the subscription manager once friendship is established */
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX - 1; i++)
    {
        exp_nv_addr[nv_cnt].addr = 0xC000 + i;
        persist_expect_addr_nonvirtual(exp_nv_addr[nv_cnt].addr, true);
        mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(exp_nv_addr[nv_cnt].addr, &exp_nv_addr[nv_cnt].handle));
        check_stored_addr_nonvirtual(exp_nv_addr[nv_cnt].addr, exp_nv_addr[nv_cnt].handle);

        /* Add some addresses multiple times */
        if (i % 2)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(exp_nv_addr[nv_cnt].addr, &exp_nv_addr[nv_cnt].handle));
        }
        nv_cnt++;
    }

    mesh_lpn_internal_mock_Verify();
    for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX - 1; i++)
    {
        exp_vt_addr[vt_cnt].addr = 0xA000 + vt_cnt;
        memset(exp_vt_addr[vt_cnt].uuid, vt_cnt, NRF_MESH_UUID_SIZE);

        mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
        nrf_mesh_keygen_virtual_address_ExpectAndReturn(exp_vt_addr[vt_cnt].uuid, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&exp_vt_addr[vt_cnt].addr);
        persist_expect_addr_virtual(exp_vt_addr[vt_cnt].uuid, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(exp_vt_addr[vt_cnt].uuid, &exp_vt_addr[vt_cnt].handle));
        check_stored_addr_virtual(exp_vt_addr[vt_cnt].uuid, exp_vt_addr[vt_cnt].handle);

        /* Add some addresses multiple times */
        if (i % 2)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(exp_vt_addr[vt_cnt].uuid, &exp_vt_addr[vt_cnt].handle));
        }
        vt_cnt++;
    }
    mesh_lpn_internal_mock_Verify();

    for (uint32_t i = 0; i < nv_cnt; i++)
    {
        mesh_lpn_subman_add_ExpectAndReturn(exp_nv_addr[i].addr, NRF_SUCCESS);
    }
    for (uint32_t i = 0; i < vt_cnt; i++)
    {
        mesh_lpn_subman_add_ExpectAndReturn(exp_vt_addr[i].addr, NRF_SUCCESS);
    }

    heartbeat_subscription_state_t hb_sub = {
        .dst = 0xF000 // a group address that isn't the rx list
    };
    heartbeat_subscription_get_ExpectAndReturn(&hb_sub);
    mesh_lpn_subman_add_ExpectAndReturn(hb_sub.dst, NRF_SUCCESS);

    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    mesh_lpn_internal_mock_Verify();

    /* Test: Once in friendship, further new additions are propagated */
    exp_nv_addr[nv_cnt].addr = 0xC000 + nv_cnt;
    persist_expect_addr_nonvirtual(exp_nv_addr[nv_cnt].addr, true);
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    mesh_lpn_subman_add_ExpectAndReturn(exp_nv_addr[nv_cnt].addr, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(exp_nv_addr[nv_cnt].addr, &exp_nv_addr[nv_cnt].handle));
    check_stored_addr_nonvirtual(exp_nv_addr[nv_cnt].addr, exp_nv_addr[nv_cnt].handle);
    nv_cnt++;

    exp_vt_addr[vt_cnt].addr = 0xA000 + vt_cnt;
    memset(exp_vt_addr[vt_cnt].uuid, vt_cnt, NRF_MESH_UUID_SIZE);

    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    nrf_mesh_keygen_virtual_address_ExpectAndReturn(exp_vt_addr[vt_cnt].uuid, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
    nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&exp_vt_addr[vt_cnt].addr);
    persist_expect_addr_virtual(exp_vt_addr[vt_cnt].uuid, true);
    mesh_lpn_subman_add_ExpectAndReturn(exp_vt_addr[vt_cnt].addr, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(exp_vt_addr[vt_cnt].uuid, &exp_vt_addr[vt_cnt].handle));
    check_stored_addr_virtual(exp_vt_addr[vt_cnt].uuid, exp_vt_addr[vt_cnt].handle);
    vt_cnt++;

    /* Test: Addresses in DSM are re-propagated if friendship is re-established */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
    for (uint32_t i = 0; i < nv_cnt; i++)
    {
        mesh_lpn_subman_add_ExpectAndReturn(exp_nv_addr[i].addr, NRF_SUCCESS);
    }
    for (uint32_t i = 0; i < vt_cnt; i++)
    {
        mesh_lpn_subman_add_ExpectAndReturn(exp_vt_addr[i].addr, NRF_SUCCESS);
    }
    hb_sub.dst = 0x0001; // a valid unicast addr, shouldn't be added to the subman
    heartbeat_subscription_get_ExpectAndReturn(&hb_sub);

    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);


    /* Test: Once in friendship, duplicate additions are ignored */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_add(exp_nv_addr[0].addr, &exp_nv_addr[0].handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(exp_vt_addr[vt_cnt-1].uuid, &exp_vt_addr[vt_cnt-1].handle));

    /* Test: Once in friendship, removals are propagated */
    uint16_t count;
    for (uint32_t i = 0; i < nv_cnt; i++)
    {
        do
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(exp_nv_addr[i].handle, &count));
            if (count == 1)
            {
                persist_invalidate_expect(exp_nv_addr[i].handle + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
                mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
                mesh_lpn_subman_remove_ExpectAndReturn(exp_nv_addr[i].addr, NRF_SUCCESS);
            }
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(exp_nv_addr[i].handle));
        } while (count > 1);
    }

    for (uint32_t i = 0; i < vt_cnt; i++)
    {
        do
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_count_get(exp_vt_addr[i].handle, &count));
            if (count == 1)
            {
                persist_invalidate_expect(exp_vt_addr[i].handle - DSM_NONVIRTUAL_ADDR_MAX + MESH_OPT_DSM_VIRTUAL_ADDR_RECORD);
                mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
                mesh_lpn_subman_remove_ExpectAndReturn(exp_vt_addr[i].addr, NRF_SUCCESS);
            }
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_remove(exp_vt_addr[i].handle));
        } while (count > 1);
    }

    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
#endif
}

void test_getters(void)
{
    /* Add some dummy networks */
    struct
    {
        dsm_handle_t handle;
        nrf_mesh_network_secmat_t secmat;
        nrf_mesh_beacon_secmat_t beacon_secmat;
    } net[3];
    uint8_t key[NRF_MESH_KEY_SIZE] = {};
    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        memset(net[i].secmat.privacy_key, i, NRF_MESH_KEY_SIZE);
        memset(net[i].secmat.encryption_key, i + 0x10, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.key, i + 0x20, NRF_MESH_KEY_SIZE);
        memset(net[i].beacon_secmat.net_id, i + 0x30, NRF_MESH_NETID_SIZE);
        net[i].secmat.nid = i;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&net[i].secmat, sizeof(net[i].secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&net[i].beacon_secmat, sizeof(net[i].beacon_secmat));
        persist_expect_subnet(key, i, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(i, key, &net[i].handle));
        check_stored_subnet(key, NULL, i, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].handle);

        uint16_t index = 0xffff;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_handle_to_netkey_index(net[i].handle, &index));
        TEST_ASSERT_EQUAL(i, index);
    }
    /* Add some dummy apps */
    struct
    {
        dsm_handle_t handle;
        uint8_t aid;
    } app[6];
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].aid, sizeof(app[i].aid));
        persist_expect_appkey(key, i, net[i / 2].handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, net[i / 2].handle, key, &app[i].handle));
        check_stored_appkey(key, NULL, i, net[i / 2].handle, app[i].handle, false);
    }

    /* Add some dummy friendships */
    struct
    {
        mesh_key_index_t key_index;
        dsm_handle_t subnet_handle;
        nrf_mesh_keygen_friendship_secmat_params_t secmat_params;
        nrf_mesh_network_secmat_t secmat;
    } friendships[] =
    {
        {0, 0, {0x0001, 0x0002, 1, 2}, {0, {0xAA}, {0xBB}}},
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(friendships); i++)
    {
        const nrf_mesh_network_secmat_t *p_net_secmat_valid;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(friendships[i].key_index, &p_net_secmat_valid));

        nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(key, &friendships[i].secmat_params, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&friendships[i].secmat, sizeof(nrf_mesh_network_secmat_t));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, &friendships[i].secmat_params));
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        TEST_ASSERT_EQUAL(net[i].handle, dsm_net_key_index_to_subnet_handle(i));
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));
        TEST_ASSERT_NOT_NULL(secmat.p_net);
        TEST_ASSERT_EQUAL(net[i/2].handle, dsm_subnet_handle_get(secmat.p_net));
        TEST_ASSERT_NOT_NULL(secmat.p_app);
        TEST_ASSERT_EQUAL(app[i].handle, dsm_appkey_handle_get(secmat.p_app));

        TEST_ASSERT_EQUAL(app[i].handle, dsm_appkey_index_to_appkey_handle(i));

        uint16_t index = 0xffff;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_handle_to_appkey_index(i, &index));
        TEST_ASSERT_EQUAL(i, index);

        /* Also get tx secmat for friendship */
        for (uint32_t j = 0; j < ARRAY_SIZE(friendships); j++)
        {
            if (friendships[j].subnet_handle == net[i / 2].handle)
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));
                TEST_ASSERT_NOT_NULL(secmat.p_net);
                TEST_ASSERT_EQUAL(friendships[j].subnet_handle, dsm_subnet_handle_get(secmat.p_net));
            }
        }
    }

    /* invalid params */
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_appkey_handle_get(NULL));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_subnet_handle_get(NULL));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_net_key_index_to_subnet_handle(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(DSM_HANDLE_INVALID, dsm_appkey_index_to_appkey_handle(DSM_HANDLE_INVALID));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_appkey_handle_to_appkey_index(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, dsm_subnet_handle_to_netkey_index(0, NULL));

    uint16_t testvar;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_handle_to_appkey_index(DSM_HANDLE_INVALID, &testvar));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_handle_to_netkey_index(DSM_HANDLE_INVALID, &testvar));
}

void test_key_refresh_all_phases(void)
{
    dsm_handle_t network_handle;
    nrf_mesh_network_secmat_t old_secmat, new_secmat;
    nrf_mesh_beacon_secmat_t old_beacon_secmat, new_beacon_secmat;

    const unsigned int old_nid = 2, new_nid = 18, key_index = 0;
    const uint8_t old_key[NRF_MESH_KEY_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    const uint8_t new_key[NRF_MESH_KEY_SIZE] = { 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };

    /* Test API functions on non-existent keys: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(0, new_key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update_swap_keys(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update_commit(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(0, new_key));

    /* Add the original network key, which will be refreshed: */
    memset(old_secmat.privacy_key,    1, NRF_MESH_KEY_SIZE);
    memset(old_secmat.encryption_key, 2, NRF_MESH_KEY_SIZE);
    memset(old_beacon_secmat.key,     3, NRF_MESH_KEY_SIZE);
    memset(old_beacon_secmat.net_id,  4, NRF_MESH_NETID_SIZE);
    memset(new_beacon_secmat.key,    33, NRF_MESH_KEY_SIZE);
    memset(new_beacon_secmat.net_id, 44, NRF_MESH_NETID_SIZE);
    old_secmat.nid = old_nid;
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&old_secmat, sizeof(old_secmat));
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&old_beacon_secmat, sizeof(old_beacon_secmat));
    persist_expect_subnet(old_key, key_index, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(key_index, old_key, &network_handle));
    check_stored_subnet(old_key, NULL, key_index, NRF_MESH_KEY_REFRESH_PHASE_0, network_handle);

    nrf_mesh_key_refresh_phase_t current_phase;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(network_handle, &current_phase));
    TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_0, current_phase);

    /* Add a small bunch of application keys: */
    struct
    {
        dsm_handle_t handle;
        uint8_t old_aid;
        uint8_t new_aid;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } app[6];
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        app[i].old_aid = i;
        app[i].new_aid = ARRAY_SIZE(app) + i;
        memset(app[i].key, i, NRF_MESH_KEY_SIZE);

        nrf_mesh_keygen_aid_ExpectAndReturn(app[i].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[i].old_aid, sizeof(app[i].old_aid));
        persist_expect_appkey(app[i].key, i, network_handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(i, network_handle, app[i].key, &app[i].handle));
        check_stored_appkey(app[i].key, NULL, i, network_handle, app[i].handle, false);

        /* Try to update each appkey before starting key refresh: */
        uint8_t updated_key[NRF_MESH_KEY_SIZE] = { 0 };
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_appkey_update(i, updated_key));
    }

    /* Test API functions in the wrong state: */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_subnet_update_swap_keys(network_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_subnet_update_commit(network_handle));

    /* Initiate the key refresh by updating the network key: */
    memset(new_secmat.privacy_key,    11, NRF_MESH_KEY_SIZE);
    memset(new_secmat.encryption_key, 22, NRF_MESH_KEY_SIZE);
    new_secmat.nid = new_nid;
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(new_key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&new_secmat, sizeof(new_secmat));
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(new_key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&new_beacon_secmat, sizeof(new_beacon_secmat));
    persist_expect_subnet_update(old_key, new_key, key_index, NRF_MESH_KEY_REFRESH_PHASE_1, network_handle, true);
    net_state_key_refresh_phase_changed_Expect(key_index, new_beacon_secmat.net_id, NRF_MESH_KEY_REFRESH_PHASE_1);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(key_index, new_key));
    check_stored_subnet(old_key, new_key, key_index, NRF_MESH_KEY_REFRESH_PHASE_1, network_handle);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(network_handle, &current_phase));
    TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_1, current_phase);

    uint8_t net_key[NRF_MESH_KEY_SIZE];
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_key_get(network_handle, net_key));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(old_key, net_key, NRF_MESH_KEY_SIZE);

    uint8_t new_appkey[NRF_MESH_KEY_SIZE] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7};

    /* Update the first application key: */
    nrf_mesh_keygen_aid_ExpectAndReturn(new_appkey, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_aid_IgnoreArg_p_aid();
    nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[0].new_aid, sizeof(app[0].new_aid));
    persist_expect_appkey_update(app[0].key, new_appkey, 0, network_handle, app[0].handle, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_update(app[0].handle, new_appkey));
    check_stored_appkey(app[0].key, new_appkey, 0, network_handle, app[0].handle, true);

    /* Update the second application key: */
    nrf_mesh_keygen_aid_ExpectAndReturn(new_appkey, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_aid_IgnoreArg_p_aid();
    nrf_mesh_keygen_aid_ReturnMemThruPtr_p_aid(&app[1].new_aid, sizeof(app[1].new_aid));
    persist_expect_appkey_update(app[1].key, new_appkey, 1, network_handle, app[1].handle, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_update(app[1].handle, new_appkey));
    check_stored_appkey(app[1].key, new_appkey, 1, network_handle, app[1].handle, true);

    /* Since we are in key refresh phase 1, the old keys should still be used for transmitting packets: */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));

        /* Verify application secmat: */
        TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8(app[i].old_aid, secmat.p_app->aid);

        /* Verify network secmat: */
        TEST_ASSERT_EQUAL_HEX8_ARRAY(old_secmat.encryption_key, secmat.p_net->encryption_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(old_secmat.privacy_key, secmat.p_net->privacy_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8(old_secmat.nid, secmat.p_net->nid);
    }

    /* Both the old and the new security materials should be used when receiving packets: */
    uint8_t expected_key[NRF_MESH_KEY_SIZE];
    const nrf_mesh_network_secmat_t * p_primary = NULL, * p_secondary = NULL;

    /* Check if the old NID can still be used: */
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NOT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    memset(expected_key, 1, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->privacy_key, NRF_MESH_KEY_SIZE);
    memset(expected_key, 2, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->encryption_key, NRF_MESH_KEY_SIZE);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    /* Check that the new NID can also be used: */
    p_primary = NULL;
    p_secondary = NULL;
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NOT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    memset(expected_key, 11, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->privacy_key, NRF_MESH_KEY_SIZE);
    memset(expected_key, 22, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->encryption_key, NRF_MESH_KEY_SIZE);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    /* Old and new application keys should be used to receive packets */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        /* Check if the old AID can still be used: */
        const nrf_mesh_application_secmat_t * p_app_primary = NULL, * p_app_secondary = NULL;

        p_primary = NULL;
        p_secondary = NULL;
        nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);

        nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
        TEST_ASSERT_NOT_NULL(p_app_primary);
        TEST_ASSERT_NULL(p_app_secondary);

        TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, p_app_primary->key, NRF_MESH_KEY_SIZE);

        nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
        TEST_ASSERT_NULL(p_app_primary);
        TEST_ASSERT_NULL(p_app_secondary);

        if (i == 0 || i == 1)
        {
            /* Check that the new AID can also be used: */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NOT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            TEST_ASSERT_EQUAL_HEX8_ARRAY(new_appkey, p_app_primary->key, NRF_MESH_KEY_SIZE);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
        else
        {
            /* The rest application keys should not be updated. */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
    }

    /* Enter key refresh phase 2, by swapping the keys used for transmission of packets: */
    persist_expect_subnet_update(old_key, new_key, key_index, NRF_MESH_KEY_REFRESH_PHASE_2, network_handle, true);
    net_state_key_refresh_phase_changed_Expect(key_index, new_beacon_secmat.net_id, NRF_MESH_KEY_REFRESH_PHASE_2);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update_swap_keys(network_handle));
    check_stored_subnet(old_key, new_key, key_index, NRF_MESH_KEY_REFRESH_PHASE_2, network_handle);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_subnet_update_swap_keys(network_handle));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(network_handle, &current_phase));
    TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_2, current_phase);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_key_get(network_handle, net_key));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(new_key, net_key, NRF_MESH_KEY_SIZE);

    /* In key refresh phase 2, the new keys should be used for transmitting packets: */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));

        /* Verify application secmat: */
        if (i == 0 || i == 1) /* The first two application keys were set to 0 above */
        {
            TEST_ASSERT_EQUAL_HEX8(app[i].new_aid, secmat.p_app->aid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(new_appkey, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        }
        else /* The other application keys have not been updated and should be the same as before */
        {
            TEST_ASSERT_EQUAL_HEX8(app[i].old_aid, secmat.p_app->aid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        }

        /* Verify network secmat: */
        TEST_ASSERT_EQUAL_HEX8_ARRAY(new_secmat.encryption_key, secmat.p_net->encryption_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(new_secmat.privacy_key, secmat.p_net->privacy_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8(new_secmat.nid, secmat.p_net->nid);

        /* Ensure that application keys cannot be updated in key refresh phase 2: */
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_appkey_update(app[i].old_aid, new_appkey));
    }

    /* Both old and new security materials are still used when receiving packets: */
    p_primary = NULL;
    p_secondary = NULL;

    /* Check that the old NID can still be used to receive packets: */
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NOT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    memset(expected_key, 1, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->privacy_key, NRF_MESH_KEY_SIZE);
    memset(expected_key, 2, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->encryption_key, NRF_MESH_KEY_SIZE);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    /* Check that the new NID can still be used to receive packets: */
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NOT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    memset(expected_key, 11, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->privacy_key, NRF_MESH_KEY_SIZE);
    memset(expected_key, 22, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->encryption_key, NRF_MESH_KEY_SIZE);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    /* Old and new application keys should be used to receive packets */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        /* Check if the old AID can still be used: */
        const nrf_mesh_application_secmat_t * p_app_primary = NULL, * p_app_secondary = NULL;

        p_primary = NULL;
        p_secondary = NULL;
        nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);

        nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
        TEST_ASSERT_NOT_NULL(p_app_primary);
        TEST_ASSERT_NULL(p_app_secondary);

        TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, p_app_primary->key, NRF_MESH_KEY_SIZE);

        nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
        TEST_ASSERT_NULL(p_app_primary);
        TEST_ASSERT_NULL(p_app_secondary);

        if (i == 0 || i == 1)
        {
            /* Check that the new AID can also be used */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NOT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            TEST_ASSERT_EQUAL_HEX8_ARRAY(new_appkey, p_app_primary->key, NRF_MESH_KEY_SIZE);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
        else
        {
            /* The rest application keys should not be updated. */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
    }

    /* Move to key refresh phase 3 (which is immediately goes back to phase 0): */
    net_state_key_refresh_phase_changed_Expect(key_index, new_beacon_secmat.net_id, NRF_MESH_KEY_REFRESH_PHASE_0);
    persist_expect_appkey_update(app[0].key, new_appkey, 0, network_handle, app[0].handle, true);
    persist_expect_appkey_update(app[1].key, new_appkey, 1, network_handle, app[1].handle, true);
    persist_expect_subnet(new_key, key_index, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update_commit(network_handle));
    check_stored_subnet(new_key, NULL, key_index, NRF_MESH_KEY_REFRESH_PHASE_0, network_handle);
    check_stored_appkey(new_appkey, NULL, 0, network_handle, app[0].handle, false);
    check_stored_appkey(new_appkey, NULL, 1, network_handle, app[1].handle, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(network_handle, &current_phase));
    TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_0, current_phase);

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_subnet_update_commit(network_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, dsm_subnet_update_swap_keys(network_handle));

    /* In key refresh phase 3, only new keys should be used for transmitting packets: */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        nrf_mesh_secmat_t secmat;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_tx_secmat_get(DSM_HANDLE_INVALID, app[i].handle, &secmat));

        /* Verify application secmat: */
        if (i == 0 || i == 1)
        {
            TEST_ASSERT_EQUAL_HEX8(app[i].new_aid, secmat.p_app->aid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(new_appkey, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX8(app[i].old_aid, secmat.p_app->aid);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, secmat.p_app->key, NRF_MESH_KEY_SIZE);
        }

        /* Verify network secmat: */
        TEST_ASSERT_EQUAL_HEX8_ARRAY(new_secmat.encryption_key, secmat.p_net->encryption_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(new_secmat.privacy_key, secmat.p_net->privacy_key, NRF_MESH_KEY_SIZE);
        TEST_ASSERT_EQUAL_HEX8(new_secmat.nid, secmat.p_net->nid);
    }

    /* Verify application keys */
    for (uint32_t i = 0; i < ARRAY_SIZE(app); i++)
    {
        const nrf_mesh_application_secmat_t * p_app_primary = NULL, * p_app_secondary = NULL;

        p_primary = NULL;
        p_secondary = NULL;
        nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);

        if (i == 0 || i == 1)
        {
            /* Verify new application keys */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NOT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            TEST_ASSERT_EQUAL_HEX8_ARRAY(new_appkey, p_app_primary->key, NRF_MESH_KEY_SIZE);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
        else
        {
            /* The rest application keys should not be updated. */
            nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NOT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            TEST_ASSERT_EQUAL_HEX8_ARRAY(app[i].key, p_app_primary->key, NRF_MESH_KEY_SIZE);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].old_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);

            nrf_mesh_app_secmat_next_get(p_primary, app[i].new_aid, &p_app_primary, &p_app_secondary);
            TEST_ASSERT_NULL(p_app_primary);
            TEST_ASSERT_NULL(p_app_secondary);
        }
    }

    p_primary = NULL;
    p_secondary = NULL;
    /* Check that the new NID can be used to receive packets: */
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NOT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);

    memset(expected_key, 11, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->privacy_key, NRF_MESH_KEY_SIZE);
    memset(expected_key, 22, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_key, p_primary->encryption_key, NRF_MESH_KEY_SIZE);
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(new_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);
    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nrf_mesh_net_secmat_next_get(old_nid, &p_primary, &p_secondary);
    TEST_ASSERT_NULL(p_primary);
    TEST_ASSERT_NULL(p_secondary);
}

void test_key_refresh_1_to_3(void)
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
        {DSM_HANDLE_INVALID, 4, 4,  2, true},  /* Old and new NID are the same, friendship */
        {DSM_HANDLE_INVALID, 5, 5,  3, false}, /* Old and new NID are the same, no friendship */
    };

    nrf_mesh_network_secmat_t old_secmat, new_secmat;
    nrf_mesh_beacon_secmat_t old_beacon_secmat, new_beacon_secmat;
    nrf_mesh_network_secmat_t old_friendship_secmat, new_friendship_secmat;

    const uint8_t old_key[NRF_MESH_KEY_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    const uint8_t new_key[NRF_MESH_KEY_SIZE] = { 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };

    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params = {0x0001, 0x0002, 1, 2};

    const nrf_mesh_network_secmat_t * p_test_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_test_aux_secmat = NULL;

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

    /* Test API functions on non-existent keys: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update(0, new_key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update_swap_keys(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_subnet_update_commit(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_appkey_update(0, new_key));

    for (uint32_t i = 0; i < ARRAY_SIZE(net); i++)
    {
        old_secmat.nid = net[i].old_nid;
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&old_secmat, sizeof(old_secmat));
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(old_key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&old_beacon_secmat, sizeof(old_beacon_secmat));
        persist_expect_subnet(old_key, net[i].key_index, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(net[i].key_index, old_key, &net[i].network_handle));
        check_stored_subnet(old_key, NULL, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].network_handle);

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

        /* Check secmat */
        p_test_secmat = NULL;
        p_test_aux_secmat = NULL;

#if MESH_FEATURE_LPN_ENABLED
        /* If in friendship, don't expect mesh_lpn_is_in_friendship() since friendship and regular
         * secmats have same NID, otherwise expect it. */
        if (!net[i].is_in_friendship)
        {
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
        }
#endif

        nrf_mesh_net_secmat_next_get(net[i].old_nid, &p_test_secmat, &p_test_aux_secmat);
        TEST_ASSERT_NULL(p_test_aux_secmat);
        if (!net[i].is_in_friendship)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));

#if MESH_FEATURE_LPN_ENABLED
            /* This time it should fail. */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].old_nid, &p_test_secmat, &p_test_aux_secmat);

            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_friendship_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));

#if MESH_FEATURE_LPN_ENABLED
            /* We've added both friendship and normal secmat with same NID. I.e., the second time it
             * should return null, as LPN should not process packets arriving on the normal (master) secmat: */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].old_nid, &p_test_secmat, &p_test_aux_secmat);
            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
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
        persist_expect_subnet_update(old_key, new_key, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_1, net[i].network_handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(net[i].network_handle, new_key));
        check_stored_subnet(old_key, new_key, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_1, net[i].network_handle);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(net[i].network_handle, &current_phase));
        TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_1, current_phase);

        /* Check secmats */
        p_test_secmat = NULL;
        p_test_aux_secmat = NULL;

#if MESH_FEATURE_LPN_ENABLED
        /* If in friendship, don't expect mesh_lpn_is_in_friendship() since friendship and regular
         * secmats have same NID, otherwise expect it. */
        if (!net[i].is_in_friendship)
        {
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
        }
#endif

        nrf_mesh_net_secmat_next_get(net[i].new_nid, &p_test_secmat, &p_test_aux_secmat);
        if (!net[i].is_in_friendship)
        {
            /* If old and new NID are the same, both secmats shall be returned */
            if (net[i].old_nid == net[i].new_nid)
            {
                TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));
                TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_secmat, p_test_aux_secmat, sizeof(nrf_mesh_network_secmat_t));
            }
            else
            {
                TEST_ASSERT_NULL(p_test_aux_secmat);
            }

#if MESH_FEATURE_LPN_ENABLED
            /* This time it should fail. */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].old_nid, &p_test_secmat, &p_test_aux_secmat);
            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
        }
        else /* is_in_friendship */
        {
            /* If old and new NID are the same, Both secmats shall be returned */
            if (net[i].old_nid == net[i].new_nid)
            {
                TEST_ASSERT_EQUAL_HEX8_ARRAY(&old_friendship_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));
                TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_friendship_secmat, p_test_aux_secmat, sizeof(nrf_mesh_network_secmat_t));
            }
            else
            {
                TEST_ASSERT_NULL(p_test_aux_secmat);
            }

#if MESH_FEATURE_LPN_ENABLED
            /* We've added both friendship and normal secmat with same NID. I.e., the second time it
             * should return null, as LPN should not process packets arriving on the normal (master) secmat: */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].old_nid, &p_test_secmat, &p_test_aux_secmat);
            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
        }


        /* When key refresh process is in progress, nrf_mesh_friendship_secmat_params_set() call
         * shall generate friendship secmats according to the new network key */
        if (net[i].is_in_friendship)
        {
            /* Remove friendship creds */
            nrf_mesh_evt_t mesh_friendship_terminated_evt;
            mesh_friendship_terminated_evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
            mesh_friendship_terminated_evt.params.friendship_terminated.lpn_src = friendship_secmat_params.lpn_address;
            mesh_friendship_terminated_evt.params.friendship_terminated.friend_src = friendship_secmat_params.friend_address;
            m_mesh_evt_handler.evt_cb(&mesh_friendship_terminated_evt);

            /* Re-add friendship creds again. They shall be created using new network key. */
            const nrf_mesh_network_secmat_t *p_net_secmat_valid;
            TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(net[i].key_index, &p_net_secmat_valid));

            new_friendship_secmat.nid = net[i].new_nid;
            nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(new_key, &friendship_secmat_params, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&new_friendship_secmat, sizeof(nrf_mesh_network_secmat_t));

            TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, &friendship_secmat_params));

            /* Check secmats */
            p_test_secmat = NULL;
            p_test_aux_secmat = NULL;

            nrf_mesh_net_secmat_next_get(net[i].new_nid, &p_test_secmat, &p_test_aux_secmat);

            /* Only one secmat shall be returned */
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_friendship_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));
            TEST_ASSERT_NULL(p_test_aux_secmat);
        }

        /* Skip phase 2 and go directly to phase 3 (which goes immediately to phase 0 again): */
        net_state_key_refresh_phase_changed_Expect(net[i].key_index, new_beacon_secmat.net_id, NRF_MESH_KEY_REFRESH_PHASE_0);
        persist_expect_subnet(new_key, net[i].key_index, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update_commit(net[i].network_handle));
        check_stored_subnet(new_key, NULL, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].network_handle);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(net[i].network_handle, &current_phase));
        TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_0, current_phase);

        /* Check secmats again */
        p_test_secmat = NULL;
        p_test_aux_secmat = NULL;

#if MESH_FEATURE_LPN_ENABLED
        /* If in friendship, don't expect mesh_lpn_is_in_friendship() since friendship and regular
         * secmats have same NID, otherwise expect it. */
        if (!net[i].is_in_friendship)
        {
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
        }
#endif

        nrf_mesh_net_secmat_next_get(net[i].new_nid, &p_test_secmat, &p_test_aux_secmat);
        TEST_ASSERT_NOT_NULL(p_test_secmat);
        TEST_ASSERT_NULL(p_test_aux_secmat);

        if (!net[i].is_in_friendship)
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));

#if MESH_FEATURE_LPN_ENABLED
            /* This time it should fail. */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].new_nid, &p_test_secmat, &p_test_aux_secmat);
            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&new_friendship_secmat, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));

#if MESH_FEATURE_LPN_ENABLED
            /* We've added both friendship and normal secmat with same NID. I.e., the second time it
             * should return null, as LPN should not process packets arriving on the normal (master) secmat: */
            mesh_lpn_is_in_friendship_ExpectAndReturn(net[i].is_in_friendship);
            nrf_mesh_net_secmat_next_get(net[i].new_nid, &p_test_secmat, &p_test_aux_secmat);
            TEST_ASSERT_NULL(p_test_secmat);
            TEST_ASSERT_NULL(p_test_aux_secmat);
#endif
        }


#if MESH_FEATURE_LPN_ENABLED
        /* Remove current friendship credentials to be able to add a new one when LPN feature is enabled */
        if (net[i].is_in_friendship)
        {
            nrf_mesh_evt_t mesh_friendship_terminated_evt;
            mesh_friendship_terminated_evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
            mesh_friendship_terminated_evt.params.friendship_terminated.lpn_src = friendship_secmat_params.lpn_address;
            mesh_friendship_terminated_evt.params.friendship_terminated.friend_src = friendship_secmat_params.friend_address;
            m_mesh_evt_handler.evt_cb(&mesh_friendship_terminated_evt);
        }
#endif
        mesh_lpn_mock_Verify();
    }
}

void test_storage_ordering_addresses(void)
{
    const uint16_t raw_addresses[4] = {0x1234, 0x1237, 0x1643, 0x043f};
    dsm_handle_t handles[8] = {DSM_HANDLE_INVALID};

    for (uint32_t i = 0; i < 4; ++i)
    {
        persist_expect_addr_nonvirtual(raw_addresses[i], true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(raw_addresses[i], &handles[i]));
        check_stored_addr_nonvirtual(raw_addresses[i], handles[i]);
    }

    dsm_handle_t stored_handles[8] = {DSM_HANDLE_INVALID};
    uint32_t count = 8;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(stored_handles, &count));
    TEST_ASSERT_EQUAL_UINT16_ARRAY(handles, stored_handles, count);

    const struct
    {
        uint16_t value;
        uint8_t uuid[NRF_MESH_UUID_SIZE];
    } virtual_addr[4] = {
        {.value = 0x8034, {1}},
        {.value = 0x8037, {2}},
        {.value = 0x8043, {3}},
        {.value = 0x803f, {4}}
    };

    for (uint32_t i = 0; i < 4; ++i)
    {
        nrf_mesh_keygen_virtual_address_ExpectAndReturn(virtual_addr[i].uuid, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        persist_expect_addr_virtual(virtual_addr[i].uuid, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_virtual_add(virtual_addr[i].uuid, &handles[i+count]));
        check_stored_addr_virtual(virtual_addr[i].uuid, handles[i+count]);
    }

    memset(stored_handles, 0xFF, sizeof(stored_handles));
    count = 8;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(stored_handles, &count));
    TEST_ASSERT_EQUAL(8, count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(handles, stored_handles, count);

#if TEST_EXPLICIT_ORDERING
    /* We use our superpowers to know that the virtual addresses start after the end of the non-virtuals. */
    const dsm_handle_t expected_handles[8] = {0, 1, 2, 3,
                                              DSM_NONVIRTUAL_ADDR_MAX, /* Virtual addresses. */
                                              DSM_NONVIRTUAL_ADDR_MAX + 1,
                                              DSM_NONVIRTUAL_ADDR_MAX + 2,
                                              DSM_NONVIRTUAL_ADDR_MAX + 3};
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected_handles, stored_handles, count);
#endif
}

void test_storage_ordering_subnets(void)
{
    const struct
    {
        mesh_key_index_t key_id;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } subnets[4] = {
        {.key_id = 0, .key = {0}},
        {.key_id = 1, .key = {1}},
        {.key_id = 2, .key = {2}},
        {.key_id = 3, .key = {3}}
    };
    dsm_handle_t handles[4] = {DSM_HANDLE_INVALID};
    for (uint32_t i = 0; i < 4; ++i)
    {
        nrf_mesh_keygen_network_secmat_ExpectAndReturn(subnets[i].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
        nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(subnets[i].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
        persist_expect_subnet(subnets[i].key, subnets[i].key_id, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(subnets[i].key_id, subnets[i].key, &handles[i]));
        check_stored_subnet(subnets[i].key, NULL, subnets[i].key_id, NRF_MESH_KEY_REFRESH_PHASE_0, handles[i]);
    }
    dsm_handle_t stored_handles[4] = {DSM_HANDLE_INVALID};
    uint32_t count = 4;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_get_all(stored_handles, &count));
    TEST_ASSERT_EQUAL(4, count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(handles, stored_handles, count);

#if TEST_EXPLICIT_ORDERING
    const dsm_handle_t expected_handles[4] = {0, 1, 2, 3};
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected_handles, stored_handles, count);
#endif
}

void test_storage_ordering_appkeys(void)
{
    const struct
    {
        mesh_key_index_t key_id;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } appkeys[4] = {
        {.key_id = 0, .key = {0}},
        {.key_id = 1, .key = {1}},
        {.key_id = 2, .key = {2}},
        {.key_id = 3, .key = {3}}
    };

    const struct
    {
        mesh_key_index_t key_id;
        uint8_t key[NRF_MESH_KEY_SIZE];
    } subnet = {0, {0}};
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(subnet.key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(subnet.key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    persist_expect_subnet(subnet.key, subnet.key_id, true);
    dsm_handle_t subnet_handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(subnet.key_id, subnet.key, &subnet_handle));
    check_stored_subnet(subnet.key, NULL, subnet.key_id, NRF_MESH_KEY_REFRESH_PHASE_0, subnet_handle);

    dsm_handle_t handles[4] = {DSM_HANDLE_INVALID};
    for (uint32_t i = 0; i < 4; ++i)
    {
        nrf_mesh_keygen_aid_ExpectAndReturn(appkeys[i].key, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_aid_IgnoreArg_p_aid();
        persist_expect_appkey(appkeys[i].key, appkeys[i].key_id, subnet_handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_add(appkeys[i].key_id, subnet_handle, appkeys[i].key, &handles[i]));
        check_stored_appkey(appkeys[i].key, NULL, appkeys[i].key_id, subnet_handle, handles[i], false);
    }
    dsm_handle_t stored_handles[4] = {DSM_HANDLE_INVALID};
    uint32_t count = 4;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_appkey_get_all(0, stored_handles, &count));
    TEST_ASSERT_EQUAL(4, count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(handles, stored_handles, count);

#if TEST_EXPLICIT_ORDERING
    const dsm_handle_t expected_handles[4] = {0, 1, 2, 3};
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected_handles, stored_handles, count);
#endif
}

void test_walking_through_uuid(void)
{
    uint16_t virtual_address = VIRTUAL_ADDR;
    dsm_handle_t address_handle[VIRTUAL_ADDRESS_COUNT];
    uint8_t virtual_uuid[VIRTUAL_ADDRESS_COUNT][NRF_MESH_UUID_SIZE] =
    {
        {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
        {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f},
        {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f}
    };

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);

    for (uint8_t iter = 0; iter < VIRTUAL_ADDRESS_COUNT; iter++)
    {
        nrf_mesh_keygen_virtual_address_ExpectAndReturn(&virtual_uuid[iter][0], NULL, NRF_SUCCESS);
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&virtual_address);
        persist_expect_addr_virtual(&virtual_uuid[iter][0], true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_subscription_virtual_add(&virtual_uuid[iter][0], &address_handle[iter]));
        check_stored_addr_virtual(&virtual_uuid[iter][0], address_handle[iter]);
    }

    nrf_mesh_address_t addr =
    {
        .type = NRF_MESH_ADDRESS_TYPE_INVALID,
        .p_virtual_uuid = NULL
    };

    for (uint8_t iter = 0; iter < VIRTUAL_ADDRESS_COUNT; iter++)
    {
        TEST_ASSERT_TRUE(nrf_mesh_rx_address_get(virtual_address, &addr));
        TEST_ASSERT_EQUAL(NRF_MESH_ADDRESS_TYPE_VIRTUAL, addr.type);
        TEST_ASSERT_TRUE(memcmp(addr.p_virtual_uuid, &virtual_uuid[iter][0], NRF_MESH_UUID_SIZE) == 0);
    }

    TEST_ASSERT_FALSE(nrf_mesh_rx_address_get(virtual_address, &addr));
}

void test_invalid_address_lookup(void)
{
    const uint16_t raw_addresses[4] = {0x1234, 0x1237, 0x1643, 0x043f};
    dsm_handle_t handles[8] = {DSM_HANDLE_INVALID};

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    for (uint32_t i = 0; i < 4; ++i)
    {
        persist_expect_addr_nonvirtual(raw_addresses[i], true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(raw_addresses[i], &handles[i]));
        check_stored_addr_nonvirtual(raw_addresses[i], handles[i]);
    }

    for (uint32_t i = 0; i < 4; ++i)
    {
        persist_invalidate_expect(handles[i] + MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_remove(handles[i]));
    }

    /* The address should be removed, thus a new flash operation is expected. */
    persist_expect_addr_nonvirtual(raw_addresses[1], true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(raw_addresses[1], &handles[0]));
    check_stored_addr_nonvirtual(raw_addresses[1], handles[0]);

    /* This should just bump the alloc counter. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_publish_add(raw_addresses[1], &handles[0]));

    dsm_handle_t stored_handles[1];
    uint32_t count = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_address_get_all(stored_handles, &count));
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(handles, stored_handles, count);
#if TEST_EXPLICIT_ORDERING
    /* We use our superpowers to know that the virtual addresses start after the end of the non-virtuals. */
    const dsm_handle_t expected_handles[1] = {0};
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected_handles, stored_handles, 1);
#endif
}

void test_lpn_request_timeout_handling(void)
{
#if MESH_FEATURE_LPN_ENABLED
    const uint8_t NID = 0x42;
    test_net_t nets;
    memset(&nets, 0, sizeof(nets));

    nrf_mesh_network_secmat_t friendship_secmat = {
        .nid = NID,
        .privacy_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
        .encryption_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
    };

    /* Dummy parameters, not important in this test. */
    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params = {
        .lpn_address = 0x002a,
        .friend_address = 0x0002,
        .lpn_counter = 1,
        .friend_counter = 2
    };

    nets.key_index = 0;
    nets.nid = NID;
    memset(nets.key, 0x55, NRF_MESH_KEY_SIZE);
    nets.handle = 0xBEEF;
    network_add(&nets);

    friendship_network_add(&nets, &friendship_secmat_params, &friendship_secmat);

    dsm_local_unicast_address_t self_address = {friendship_secmat_params.lpn_address, 1};
    persist_expect_unicast(&self_address, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_local_unicast_addresses_set(&self_address));
    check_stored_unicat(&self_address);

    const nrf_mesh_network_secmat_t * p_get_net_secmat;
    nrf_mesh_friendship_secmat_get(friendship_secmat_params.lpn_address, &p_get_net_secmat);
    TEST_ASSERT_EQUAL_MEMORY(&friendship_secmat, p_get_net_secmat, sizeof(nrf_mesh_network_secmat_t));

    helper_trigger_event(NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT);

    p_get_net_secmat = NULL;
    nrf_mesh_friendship_secmat_get(friendship_secmat_params.lpn_address, &p_get_net_secmat);
    TEST_ASSERT_NULL(p_get_net_secmat);
#endif
}

void test_friendship_secmat_overflow(void)
{
#if MESH_FEATURE_LPN_ENABLED && MESH_FEATURE_FRIEND_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS (MESH_FRIEND_FRIENDSHIP_COUNT + 1)
#elif MESH_FEATURE_LPN_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS  1
#elif MESH_FEATURE_FRIEND_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS (MESH_FRIEND_FRIENDSHIP_COUNT)
#endif

    test_net_t nets[MESH_FRIENDSHIP_CREDENTIALS];
    memset(nets, 0, sizeof(nets));

    nrf_mesh_network_secmat_t friendship_secmat = {
        .privacy_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
        .encryption_key = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
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
        friendship_secmat.nid = i + 2;

        nets[i].key_index = i;
        nets[i].nid = friendship_secmat.nid;
        memset(nets[i].key, i, NRF_MESH_KEY_SIZE);
        nets[i].handle = 0xBEEF;

        network_add(&nets[i]);

        friendship_network_add(nets, &friendship_secmat_params, &friendship_secmat);
    }

    const uint8_t nid = 0x2a;
    test_net_t one_more_net;
    memset(&one_more_net, 0, sizeof(one_more_net));

    friendship_secmat.nid = nid;
    one_more_net.key_index = 0x2a;
    one_more_net.nid = friendship_secmat.nid;
    memset(one_more_net.key, 0x2a, NRF_MESH_KEY_SIZE);
    one_more_net.handle = 0xDEAD;

    network_add(&one_more_net);

    const nrf_mesh_network_secmat_t * p_net_secmat_valid;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_net_secmat_from_keyindex_get(one_more_net.key_index, &p_net_secmat_valid));
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, nrf_mesh_friendship_secmat_params_set(p_net_secmat_valid, &friendship_secmat_params));
}

void test_key_refresh_phase_get(void)
{
    test_net_t network;
    memset(&network, 0, sizeof(network));
    network.key_index = 1;
    network.nid = 0x1e;
    memset(network.key, 0x1e, NRF_MESH_KEY_SIZE);
    network.handle = 0xABCD;
    network_add(&network);

    test_net_t new_network;
    memset(&new_network, 0, sizeof(new_network));
    new_network.key_index = 1;
    new_network.nid = 33;
    memset(new_network.key, 0xab, NRF_MESH_KEY_SIZE);
    memset(new_network.secmat.net.privacy_key,    11, NRF_MESH_KEY_SIZE);
    memset(new_network.secmat.net.encryption_key, 22, NRF_MESH_KEY_SIZE);
    new_network.secmat.net.nid = 33;

    new_network.handle = network.handle;

    nrf_mesh_keygen_friendship_secmat_params_t friendship_secmat_params = {
      .lpn_address = 0x1337,
      .friend_address = 0x0001,
      .lpn_counter = 52,
      .friend_counter = 13
    };
    nrf_mesh_network_secmat_t friend_secmat;
    memset(&friend_secmat, 0, sizeof(friend_secmat));
    friend_secmat.nid = 13;
    memset(friend_secmat.encryption_key, 0x13, NRF_MESH_KEY_SIZE);
    memset(friend_secmat.privacy_key, 0x13, NRF_MESH_KEY_SIZE);
    friendship_network_add(&network, &friendship_secmat_params, &friend_secmat);

    nrf_mesh_network_secmat_t new_friend_secmat;
    memset(&new_friend_secmat, 0, sizeof(new_friend_secmat));
    new_friend_secmat.nid = 14;
    memset(new_friend_secmat.encryption_key, 0x14, NRF_MESH_KEY_SIZE);
    memset(new_friend_secmat.privacy_key, 0x14, NRF_MESH_KEY_SIZE);

    subnet_key_refresh_phase_Verify(network.handle, NRF_MESH_KEY_REFRESH_PHASE_0);

    mesh_lpn_is_in_friendship_IgnoreAndReturn(m_test_in_friendship);
    nid_key_refresh_phase_Verify(network.nid, NRF_MESH_KEY_REFRESH_PHASE_0);

    nid_key_refresh_phase_Verify(friend_secmat.nid, NRF_MESH_KEY_REFRESH_PHASE_0);

    /* Initiate the key refresh by updating the network key: */
    nrf_mesh_keygen_network_secmat_ExpectAndReturn(new_network.key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_network_secmat_ReturnMemThruPtr_p_secmat(&new_network.secmat.net, sizeof(nrf_mesh_network_secmat_t));

    nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(new_network.key, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_beacon_secmat_ReturnMemThruPtr_p_secmat(&new_network.secmat.beacon, sizeof(nrf_mesh_beacon_secmat_t));

    nrf_mesh_keygen_friendship_secmat_ExpectAndReturn(new_network.key, &friendship_secmat_params, NULL, NRF_SUCCESS);
    nrf_mesh_keygen_friendship_secmat_IgnoreArg_p_secmat();
    nrf_mesh_keygen_friendship_secmat_ReturnMemThruPtr_p_secmat(&new_friend_secmat, sizeof(nrf_mesh_network_secmat_t));

    persist_expect_subnet_update(network.key, new_network.key, network.key_index, NRF_MESH_KEY_REFRESH_PHASE_1, network.handle, true);
    net_state_key_refresh_phase_changed_Expect(network.key_index, new_network.secmat.beacon.net_id, NRF_MESH_KEY_REFRESH_PHASE_1);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(network.handle, new_network.key));
    check_stored_subnet(network.key, new_network.key, network.key_index, NRF_MESH_KEY_REFRESH_PHASE_1, network.handle);

    subnet_key_refresh_phase_Verify(network.handle, NRF_MESH_KEY_REFRESH_PHASE_1);
    nid_key_refresh_phase_Verify(network.nid, NRF_MESH_KEY_REFRESH_PHASE_1);
    nid_key_refresh_phase_Verify(new_network.nid, NRF_MESH_KEY_REFRESH_PHASE_1);
    nid_key_refresh_phase_Verify(friend_secmat.nid, NRF_MESH_KEY_REFRESH_PHASE_1);
    nid_key_refresh_phase_Verify(new_friend_secmat.nid, NRF_MESH_KEY_REFRESH_PHASE_1);
    friendship_key_refresh_phase_Verify(friendship_secmat_params.lpn_address, NRF_MESH_KEY_REFRESH_PHASE_1);

    /* Swap the keys. */
    net_state_key_refresh_phase_changed_Expect(network.key_index, new_network.secmat.beacon.net_id, NRF_MESH_KEY_REFRESH_PHASE_2);
    persist_expect_subnet_update(network.key, new_network.key, network.key_index, NRF_MESH_KEY_REFRESH_PHASE_2, network.handle, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update_swap_keys(network.handle));
    check_stored_subnet(network.key, new_network.key, network.key_index, NRF_MESH_KEY_REFRESH_PHASE_2, network.handle);

    subnet_key_refresh_phase_Verify(network.handle, NRF_MESH_KEY_REFRESH_PHASE_2);
    nid_key_refresh_phase_Verify(network.nid, NRF_MESH_KEY_REFRESH_PHASE_2);
    nid_key_refresh_phase_Verify(new_network.nid, NRF_MESH_KEY_REFRESH_PHASE_2);
    nid_key_refresh_phase_Verify(friend_secmat.nid, NRF_MESH_KEY_REFRESH_PHASE_2);
    nid_key_refresh_phase_Verify(new_friend_secmat.nid, NRF_MESH_KEY_REFRESH_PHASE_2);
    friendship_key_refresh_phase_Verify(friendship_secmat_params.lpn_address, NRF_MESH_KEY_REFRESH_PHASE_2);

    net_state_key_refresh_phase_changed_Expect(network.key_index, new_network.secmat.beacon.net_id, NRF_MESH_KEY_REFRESH_PHASE_0);
    persist_expect_subnet(new_network.key, new_network.key_index, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update_commit(network.handle));
    check_stored_subnet(new_network.key, NULL, network.key_index, NRF_MESH_KEY_REFRESH_PHASE_0, network.handle);
}

void test_load_data(void)
{
    mesh_config_entry_params_t const * p_entry;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
    };

    const snapshot_entry_t snapshot[] =
    {
        {.id = MESH_OPT_DSM_METADATA_RECORD,            .metainfo_entry = {.max_subnets = DSM_SUBNET_MAX, .max_appkeys = DSM_APP_MAX, .max_devkeys = DSM_DEVICE_MAX,
                                                        .max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX, .max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX}},
        {.id = MESH_OPT_DSM_UNICAST_ADDR_RECORD,        .unicats_entry = {.addr.address_start = 0x0003, .addr.count = 1}},
        {.id = MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + 0, .nonvirtual_entry = {.addr = 0x000A}},
        {.id = MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + 1, .nonvirtual_entry = {.addr = 0x000B}},
        {.id = MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + 0,    .virtual_entry = {.uuid = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f}}},
        {.id = MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + 1,    .virtual_entry = {.uuid = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f}}},
        {.id = MESH_OPT_DSM_SUBNETS_RECORD + 0,         .subnet_entry = {.key_index = 0, .key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0,
                                                        .key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f}}},
        {.id = MESH_OPT_DSM_SUBNETS_RECORD + 1,         .subnet_entry = {.key_index = 0, .key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_1,
                                                        .key = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f},
                                                        .key_updated = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f}}},
        {.id = MESH_OPT_DSM_APPKEYS_RECORD + 0,         .appkey_entry = {.key_index = 0, .subnet_handle = 0, .is_key_updated = false,
                                                        .key = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f}}},
        {.id = MESH_OPT_DSM_APPKEYS_RECORD + 1,         .appkey_entry = {.key_index = 1, .subnet_handle = 1, .is_key_updated = true,
                                                        .key = {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f},
                                                        .key_updated = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f}}},
        {.id = MESH_OPT_DSM_DEVKEYS_RECORD,             .devkey_entry = {.key_owner = 0x0003, .subnet_handle = 0,
                                                        .key = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f}}},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(snapshot); i++)
    {
        id.record = snapshot[i].id;

        if (id.record == MESH_OPT_DSM_METADATA_RECORD)
        {
            p_entry = &m_dsm_metadata_params;
        }
        else if (id.record == MESH_OPT_DSM_UNICAST_ADDR_RECORD)
        {
            p_entry = &m_dsm_unicast_addr_params;
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD,
                        MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX - 1))
        {
            p_entry = &m_dsm_nonvirtual_addr_params;
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD,
                        MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_VIRTUAL_ADDR_MAX - 1))
        {
            p_entry = &m_dsm_virtual_addr_params;
            nrf_mesh_keygen_virtual_address_ExpectAndReturn(snapshot[i].virtual_entry.uuid, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                        MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1))
        {
            p_entry = &m_dsm_subnet_params;
            nrf_mesh_keygen_network_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();

            if (snapshot[i].subnet_entry.key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                nrf_mesh_keygen_network_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key_updated, NULL, NRF_SUCCESS);
                nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            }
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                        MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
        {
            p_entry = &m_dsm_appkey_params;

            nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].appkey_entry.key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_aid_IgnoreArg_p_aid();

            if (snapshot[i].appkey_entry.is_key_updated)
            {
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].appkey_entry.key_updated, NULL, NRF_SUCCESS);
                nrf_mesh_keygen_aid_IgnoreArg_p_aid();
            }
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_DEVKEYS_RECORD,
                        MESH_OPT_DSM_DEVKEYS_RECORD + DSM_DEVICE_MAX - 1))
        {
            p_entry = &m_dsm_devkey_params;
        }
        else
        {
            TEST_FAIL_MESSAGE("Something is wrong in test snapshot.");
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, p_entry->callbacks.setter(id, &snapshot[i].common_stub));
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(snapshot); i++)
    {
        id.record = snapshot[i].id;

        if (id.record == MESH_OPT_DSM_METADATA_RECORD)
        {
            check_metainfo();
        }
        else if (id.record == MESH_OPT_DSM_UNICAST_ADDR_RECORD)
        {
            check_stored_unicat((dsm_local_unicast_address_t *)&snapshot[i].unicats_entry.addr);
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD,
                        MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX - 1))
        {
            check_stored_addr_nonvirtual(snapshot[i].nonvirtual_entry.addr,
                                         snapshot[i].id - MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD);
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD,
                        MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_VIRTUAL_ADDR_MAX - 1))
        {
            check_stored_addr_virtual(snapshot[i].virtual_entry.uuid,
                                      snapshot[i].id - MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX);
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                        MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1))
        {
            check_stored_subnet(snapshot[i].subnet_entry.key,
                                snapshot[i].subnet_entry.key_updated,
                                snapshot[i].subnet_entry.key_index,
                                snapshot[i].subnet_entry.key_refresh_phase,
                                snapshot[i].id - MESH_OPT_DSM_SUBNETS_RECORD);
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                        MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
        {
            check_stored_appkey(snapshot[i].appkey_entry.key,
                                snapshot[i].appkey_entry.key_updated,
                                snapshot[i].appkey_entry.key_index,
                                snapshot[i].appkey_entry.subnet_handle,
                                snapshot[i].id - MESH_OPT_DSM_APPKEYS_RECORD,
                                snapshot[i].appkey_entry.is_key_updated);
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_DEVKEYS_RECORD,
                        MESH_OPT_DSM_DEVKEYS_RECORD + DSM_DEVICE_MAX - 1))
        {
            check_stored_devkey(snapshot[i].devkey_entry.key,
                                snapshot[i].devkey_entry.key_owner,
                                snapshot[i].devkey_entry.subnet_handle,
                                snapshot[i].id - MESH_OPT_DSM_DEVKEYS_RECORD + DSM_APP_MAX);
        }
        else
        {
            TEST_FAIL_MESSAGE("Something is wrong in test snapshot.");
        }
    }

    // apply data
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_load_config_apply());
}

void test_load_legacy_data(void)
{
    mesh_config_entry_params_t const * p_entry;
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_DSM_FILE_ID,
    };

    const snapshot_entry_t snapshot[] =
    {
        {.id = MESH_OPT_DSM_METADATA_RECORD,     .metainfo_entry = {.max_subnets = DSM_SUBNET_MAX, .max_appkeys = DSM_APP_MAX, .max_devkeys = DSM_DEVICE_MAX,
                                                 .max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX, .max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX}},
        {.id = MESH_OPT_DSM_SUBNETS_RECORD + 0,  .legacy_subnet_entry = {.key_index = 0, .key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0,
                                                 .key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f}}},
        {.id = MESH_OPT_DSM_SUBNETS_RECORD + 1,  .subnet_entry = {.key_index = 1, .key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_1,
                                                 .key = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f},
                                                 .key_updated = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f}}},
        {.id = MESH_OPT_DSM_APPKEYS_RECORD + 0,  .reduced_legacy_appkey_entry = {.key_index = 0, .subnet_handle = 0,
                                                 .key = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f}}},
        {.id = MESH_OPT_DSM_APPKEYS_RECORD + 1,  .legacy_appkey_entry = {.key_index = 1, .subnet_handle = 1,
                                                 .key = {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f},
                                                 .key_updated = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f}}},
        {.id = MESH_OPT_DSM_APPKEYS_RECORD + 2,  .appkey_entry = {.key_index = 2, .subnet_handle = 1, .is_key_updated = true,
                                                 .key = {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f},
                                                 .key_updated = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f}}},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(snapshot); i++)
    {
        id.record = snapshot[i].id;

        if (id.record == MESH_OPT_DSM_METADATA_RECORD)
        {
            p_entry = &m_dsm_metadata_params;
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                        MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1))
        {
            if (snapshot[i].id == MESH_OPT_DSM_SUBNETS_RECORD)
            {
                dsm_legacy_pretreatment_do(&id, sizeof(snapshot[i].legacy_subnet_entry));
                TEST_ASSERT_EQUAL_UINT16(MESH_OPT_DSM_LEGACY_SUBNETS_RECORD, id.record);
                p_entry = &m_dsm_legacy_subnet_params;
            }
            else
            {
                dsm_legacy_pretreatment_do(&id, sizeof(snapshot[i].subnet_entry));
                TEST_ASSERT_EQUAL_UINT16(MESH_OPT_DSM_SUBNETS_RECORD + 1, id.record);
                p_entry = &m_dsm_subnet_params;
            }

            nrf_mesh_keygen_network_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            nrf_mesh_keygen_beacon_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key, NULL, NRF_SUCCESS);
            nrf_mesh_keygen_beacon_secmat_IgnoreArg_p_secmat();

            if (snapshot[i].subnet_entry.key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                nrf_mesh_keygen_network_secmat_ExpectAndReturn(snapshot[i].subnet_entry.key_updated, NULL, NRF_SUCCESS);
                nrf_mesh_keygen_network_secmat_IgnoreArg_p_secmat();
            }
        }
        else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                        MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
        {
            if (snapshot[i].id == MESH_OPT_DSM_APPKEYS_RECORD)
            {
                dsm_legacy_pretreatment_do(&id, sizeof(snapshot[i].reduced_legacy_appkey_entry));
                TEST_ASSERT_EQUAL_UINT16(MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD, id.record);
                p_entry = &m_dsm_reduced_legacy_appkey_params;
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].reduced_legacy_appkey_entry.key, NULL, NRF_SUCCESS);
            }
            else if (snapshot[i].id == MESH_OPT_DSM_APPKEYS_RECORD + 1)
            {
                dsm_legacy_pretreatment_do(&id, sizeof(snapshot[i].legacy_appkey_entry));
                TEST_ASSERT_EQUAL_UINT16(MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD + 1, id.record);
                p_entry = &m_dsm_full_legacy_appkey_params;
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].legacy_appkey_entry.key, NULL, NRF_SUCCESS);
            }
            else if (snapshot[i].id == MESH_OPT_DSM_APPKEYS_RECORD + 2)
            {
                dsm_legacy_pretreatment_do(&id, sizeof(snapshot[i].appkey_entry));
                TEST_ASSERT_EQUAL_UINT16(MESH_OPT_DSM_APPKEYS_RECORD + 2, id.record);
                p_entry = &m_dsm_appkey_params;
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].appkey_entry.key, NULL, NRF_SUCCESS);
            }
            else
            {
                TEST_FAIL_MESSAGE("Something is wrong in test snapshot.");
            }

            nrf_mesh_keygen_aid_IgnoreArg_p_aid();

            if (IS_IN_RANGE(id.record, MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD,
                            MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD + DSM_APP_MAX - 1))
            {
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].legacy_appkey_entry.key_updated, NULL, NRF_SUCCESS);
                nrf_mesh_keygen_aid_IgnoreArg_p_aid();
            }

            if (IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                            MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1) && snapshot[i].appkey_entry.is_key_updated)
            {
                nrf_mesh_keygen_aid_ExpectAndReturn(snapshot[i].appkey_entry.key_updated, NULL, NRF_SUCCESS);
                nrf_mesh_keygen_aid_IgnoreArg_p_aid();
            }
        }
        else
        {
            TEST_FAIL_MESSAGE("Something is wrong in test snapshot.");
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, p_entry->callbacks.setter(id, &snapshot[i].common_stub));
        nrf_mesh_keygen_mock_Verify();
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(snapshot); i++)
    {
        id.record = snapshot[i].id;

        if (id.record == MESH_OPT_DSM_METADATA_RECORD)
        {
            check_metainfo();
        }
        else if (id.record == MESH_OPT_DSM_SUBNETS_RECORD)
        {
            check_stored_subnet(snapshot[i].legacy_subnet_entry.key,
                                NULL,
                                snapshot[i].legacy_subnet_entry.key_index,
                                snapshot[i].subnet_entry.key_refresh_phase,
                                snapshot[i].id - MESH_OPT_DSM_SUBNETS_RECORD);
        }
        else if (id.record == MESH_OPT_DSM_SUBNETS_RECORD + 1)
        {
            check_stored_subnet(snapshot[i].subnet_entry.key,
                                snapshot[i].subnet_entry.key_updated,
                                snapshot[i].subnet_entry.key_index,
                                snapshot[i].subnet_entry.key_refresh_phase,
                                snapshot[i].id - MESH_OPT_DSM_SUBNETS_RECORD);
        }
        else if (id.record == MESH_OPT_DSM_APPKEYS_RECORD)
        {
            check_stored_appkey(snapshot[i].reduced_legacy_appkey_entry.key,
                                NULL,
                                snapshot[i].reduced_legacy_appkey_entry.key_index,
                                snapshot[i].reduced_legacy_appkey_entry.subnet_handle,
                                snapshot[i].id - MESH_OPT_DSM_APPKEYS_RECORD,
                                false);
        }
        else if (id.record == MESH_OPT_DSM_APPKEYS_RECORD + 1)
        {
            check_stored_appkey(snapshot[i].legacy_appkey_entry.key,
                                snapshot[i].legacy_appkey_entry.key_updated,
                                snapshot[i].legacy_appkey_entry.key_index,
                                snapshot[i].legacy_appkey_entry.subnet_handle,
                                snapshot[i].id - MESH_OPT_DSM_APPKEYS_RECORD,
                                true);
        }
        else if (id.record == MESH_OPT_DSM_APPKEYS_RECORD + 2)
        {
            check_stored_appkey(snapshot[i].appkey_entry.key,
                                snapshot[i].appkey_entry.key_updated,
                                snapshot[i].appkey_entry.key_index,
                                snapshot[i].appkey_entry.subnet_handle,
                                snapshot[i].id - MESH_OPT_DSM_APPKEYS_RECORD,
                                snapshot[i].appkey_entry.is_key_updated);
        }
        else
        {
            TEST_FAIL_MESSAGE("Something is wrong in test snapshot.");
        }
    }

    // apply legacy data
    mesh_config_entry_set_StubWithCallback(NULL);
    for (uint32_t i = 0; i < ARRAY_SIZE(snapshot); i++)
    {
        id.record = snapshot[i].id;

        if (id.record == MESH_OPT_DSM_METADATA_RECORD)
        {
            continue;
        }

        mesh_config_entry_delete_ExpectAndReturn(id, NRF_SUCCESS);
        mesh_config_entry_set_ExpectAndReturn(id, NULL, NRF_SUCCESS);
        mesh_config_entry_set_IgnoreArg_p_entry();
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_load_config_apply());
}

void test_failed_loading(void)
{
    // 1. There is no metadata
    persist_expected_metadata();
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, dsm_load_config_apply());

    // 2. Fail during data loading from mesh config
    nrf_mesh_evt_t load_failure_evt;
    load_failure_evt.type = NRF_MESH_EVT_CONFIG_LOAD_FAILURE;
    load_failure_evt.params.config_load_failure.id.file = MESH_OPT_DSM_FILE_ID;
    m_mesh_evt_handler.evt_cb(&load_failure_evt);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, dsm_load_config_apply());
}

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
        {DSM_HANDLE_INVALID, 2, 18, 0, true},  /* Old and new NIDs are different, friendship */
        {DSM_HANDLE_INVALID, 3, 19, 1, false}, /* Old and new NIDs are different, no friendship */
        {DSM_HANDLE_INVALID, 4, 20, 2, true}  /* Old and new NIDs are different, friendship */
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
        persist_expect_subnet(old_key, net[i].key_index, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_add(net[i].key_index, old_key, &net[i].network_handle));
        check_stored_subnet(old_key, NULL, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_0, net[i].network_handle);

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
        persist_expect_subnet_update(old_key, new_key, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_1, net[i].network_handle, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_update(net[i].network_handle, new_key));
        check_stored_subnet(old_key, new_key, net[i].key_index, NRF_MESH_KEY_REFRESH_PHASE_1, net[i].network_handle);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, dsm_subnet_kr_phase_get(net[i].network_handle, &current_phase));
        TEST_ASSERT_EQUAL(NRF_MESH_KEY_REFRESH_PHASE_1, current_phase);


        /* Test: When RX secmat is same as OLD or NEW secmat, API returns NULL */
        /*       RX secmat is old */
        TEST_ASSERT_NULL(nrf_mesh_net_master_secmat_get(p_test_old_secmat));
        /*       RX secmat is new */
        p_rx_secmat = p_test_old_secmat;
        p_rx_secmat_secondary = p_test_old_secmat;      /* This is actually unused, but API needs it */
        mesh_lpn_is_in_friendship_IgnoreAndReturn(false);
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
        mesh_lpn_is_in_friendship_IgnoreAndReturn(false);
        nrf_mesh_net_secmat_next_get(NID, &p_test_secmat, &p_test_aux_secmat);
        TEST_ASSERT_NOT_NULL(p_test_secmat);
        TEST_ASSERT_NULL(p_test_aux_secmat);

        /* Check that we're getting the correct key. This test makes an assumption about the order
         * of the keys. */
        TEST_ASSERT_EQUAL_MEMORY(&nets[i].secmat.net, p_test_secmat, sizeof(nrf_mesh_network_secmat_t));
    }

    /* All available networks with the given NID is exhausted. Last attempt should not result in any
     * secmat. */
    mesh_lpn_is_in_friendship_IgnoreAndReturn(false);
    nrf_mesh_net_secmat_next_get(NID, &p_test_secmat, &p_test_aux_secmat);
    TEST_ASSERT_NULL(p_test_secmat);
    TEST_ASSERT_NULL(p_test_aux_secmat);
}
