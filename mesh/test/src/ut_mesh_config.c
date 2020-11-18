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
#include "unity.h"
#include "cmock.h"
#include <string.h>

#include "mesh_config.h"
#include "mesh_config_listener.h"
#include "mesh_opt.h"
#include "mesh_config_backend_mock.h"
#include "event_mock.h"
#include "emergency_cache_mock.h"
#include "nordic_common.h"
#include "manual_mock_queue.h"
#include "test_assert.h"

#define FILE_ID_0   MESH_OPT_ACCESS_FILE_ID
#define FILE_ID_1   MESH_OPT_NET_STATE_FILE_ID

#define TEST_ENTRY(index) (MESH_CONFIG_ENTRY_ID((index < 3 ? FILE_ID_0 : FILE_ID_1), 0x1000 + index))
#define TEST_ENTRY_PARAMS(index) CONCAT_3(m_entry, index, _params)
#define EXTRA_ENTRIES 3


typedef struct
{
    uint32_t var1;
    uint32_t var2;
} entry_t;

typedef struct
{
    mesh_config_entry_id_t id;
    entry_t entry;
    uint32_t return_value;
} entry_set_params_t;

typedef struct
{
    mesh_config_entry_id_t id;
    entry_t entry;
    uint32_t length;
} custom_load_t;

typedef struct
{
    mesh_config_change_reason_t reason;
    mesh_config_entry_id_t id;
    void * p_entry;
} listener_params_t;

MOCK_QUEUE_DEF(config_evt, nrf_mesh_evt_t, NULL);
MOCK_QUEUE_DEF(entry_set, entry_set_params_t, NULL);
MOCK_QUEUE_DEF(custom_load, custom_load_t, NULL);
MOCK_QUEUE_DEF(listener, listener_params_t, NULL);

mesh_config_file_params_t mesh_config_files[NRF_SECTION_ENTRIES];
mesh_config_entry_params_t mesh_config_entries[NRF_SECTION_ENTRIES];
mesh_config_listener_t mesh_config_entry_listeners[NRF_SECTION_ENTRIES];
static entry_t m_entries[NRF_SECTION_ENTRIES + EXTRA_ENTRIES];
static entry_t m_load_entries[NRF_SECTION_ENTRIES];
static uint8_t m_load_ec_items[(sizeof(emergency_cache_item_t) + sizeof(entry_t)) * NRF_SECTION_ENTRIES];
static bool m_active[NRF_SECTION_ENTRIES + EXTRA_ENTRIES];
static mesh_config_backend_evt_cb_t m_backend_evt_cb;
static const mesh_config_entry_id_t m_invalid_id = {0, 0};
static bool m_is_legacy_handled;

static uint32_t entry_set(mesh_config_entry_id_t id, const void * p_entry);
static void entry_get(mesh_config_entry_id_t id, void * p_entry);
static void entry_delete(mesh_config_entry_id_t id);
static void event_handler(const nrf_mesh_evt_t * p_evt, int calls);

MESH_CONFIG_FILE(file0, FILE_ID_0, MESH_CONFIG_STRATEGY_CONTINUOUS);
MESH_CONFIG_FILE(file1, FILE_ID_1, MESH_CONFIG_STRATEGY_CONTINUOUS);
MESH_CONFIG_FILE(emergency_cache, MESH_OPT_EMERGENCY_CACHE_FILE_ID, MESH_CONFIG_STRATEGY_ON_POWER_DOWN);

MESH_CONFIG_ENTRY(entry0, TEST_ENTRY(0), 1, sizeof(entry_t), entry_set, entry_get, entry_delete, true);
MESH_CONFIG_ENTRY(entry1, TEST_ENTRY(1), 1, sizeof(entry_t), entry_set, entry_get, entry_delete, false);
MESH_CONFIG_ENTRY(entry2, TEST_ENTRY(2), 1, sizeof(entry_t), entry_set, entry_get, entry_delete, false);
MESH_CONFIG_ENTRY(entry3, TEST_ENTRY(3), 1, sizeof(entry_t), entry_set, entry_get, entry_delete, false);
MESH_CONFIG_ENTRY(entry4, TEST_ENTRY(4), EXTRA_ENTRIES, sizeof(entry_t), entry_set, entry_get, entry_delete, false);

static void mesh_config_backend_init_callback(const mesh_config_entry_params_t * p_entries,
                                              uint32_t entry_count,
                                              const mesh_config_file_params_t * p_files,
                                              uint32_t file_count,
                                              mesh_config_backend_evt_cb_t cb,
                                              int calls)
{
    TEST_ASSERT_EQUAL(mesh_config_entries, p_entries);
    TEST_ASSERT_EQUAL(mesh_config_files, p_files);
    TEST_ASSERT_EQUAL(NRF_SECTION_ENTRIES, entry_count);
    TEST_ASSERT_EQUAL(NRF_SECTION_ENTRIES, file_count);
    TEST_ASSERT_NOT_NULL(cb);
    m_backend_evt_cb = cb;
}

void dsm_legacy_pretreatment_do(mesh_config_entry_id_t * p_id, uint32_t entry_len)
{
    (void)p_id;
    (void)entry_len;

    m_is_legacy_handled = true;
}

void setUp(void)
{
    memset(m_entries, 0, sizeof(m_entries));
    memset(m_load_entries, 0, sizeof(m_load_entries));
    memset(m_load_ec_items, 0, sizeof(m_load_ec_items));
    memset(m_active, 0, sizeof(m_active));
    memset(mesh_config_entry_listeners, 0, sizeof(mesh_config_entry_listeners));

    mesh_config_entries[0] = TEST_ENTRY_PARAMS(0);
    mesh_config_entries[1] = TEST_ENTRY_PARAMS(1);
    mesh_config_entries[2] = TEST_ENTRY_PARAMS(2);
    mesh_config_entries[3] = TEST_ENTRY_PARAMS(3);
    mesh_config_entries[4] = TEST_ENTRY_PARAMS(4);


    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        /* mesh config hardfaults if the ID is NULL, as that's impossible for listeners made with the macro */
        mesh_config_entry_listeners[i].p_id = &m_invalid_id;
    }

    mesh_config_files[0] = file0;
    mesh_config_files[1] = file1;
    mesh_config_files[2] = emergency_cache;

    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        memset(mesh_config_entries[i].p_state, 0, mesh_config_entries[i].max_count);
    }

    mesh_config_backend_mock_Init();
    config_evt_Init();
    entry_set_Init();
    custom_load_Init();
    listener_Init();
    event_mock_Init();
    event_handle_StubWithCallback(event_handler);
    emergency_cache_mock_Init();

    mesh_config_backend_init_StubWithCallback(mesh_config_backend_init_callback);
    mesh_config_init();
}

void tearDown(void)
{
    mesh_config_backend_mock_Verify();
    mesh_config_backend_mock_Destroy();
    config_evt_Verify();
    config_evt_Destroy();
    entry_set_Verify();
    entry_set_Destroy();
    custom_load_Verify();
    custom_load_Destroy();
    listener_Verify();
    listener_Destroy();
    emergency_cache_mock_Verify();
    emergency_cache_mock_Destroy();
}

static uint32_t entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    const entry_t * p_value = p_entry;
    TEST_ASSERT_NOT_NULL(p_value);

    entry_set_params_t expected_value;
    entry_set_Consume(&expected_value);
    TEST_ASSERT_EQUAL(expected_value.id.file, id.file);
    TEST_ASSERT_EQUAL(expected_value.id.record, id.record);
    TEST_ASSERT_EQUAL(expected_value.entry.var1, p_value->var1);
    TEST_ASSERT_EQUAL(expected_value.entry.var2, p_value->var2);

    m_entries[id.record - TEST_ENTRY(0).record] = *p_value;
    m_active[id.record - TEST_ENTRY(0).record]  = true;
    return expected_value.return_value;
}

static void entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    entry_t * p_value = p_entry;
    TEST_ASSERT_TRUE(id.file == FILE_ID_0 || id.file == FILE_ID_1);
    TEST_ASSERT_TRUE(id.record >= TEST_ENTRY(0).record);
    TEST_ASSERT_TRUE(id.record < TEST_ENTRY(NRF_SECTION_ENTRIES + EXTRA_ENTRIES).record);
    TEST_ASSERT_TRUE(m_active[id.record - TEST_ENTRY(0).record]);

    *p_value = m_entries[id.record - TEST_ENTRY(0).record];
}

static void entry_delete(mesh_config_entry_id_t id)
{
    TEST_ASSERT_TRUE(m_active[id.record - TEST_ENTRY(0).record]);
    TEST_ASSERT_TRUE(id.file == FILE_ID_0 || id.file == FILE_ID_1);
    TEST_ASSERT_TRUE(id.record >= TEST_ENTRY(0).record);
    TEST_ASSERT_TRUE(id.record < TEST_ENTRY(NRF_SECTION_ENTRIES + EXTRA_ENTRIES).record);
    m_active[id.record - TEST_ENTRY(0).record] = false;
}

static void event_handler(const nrf_mesh_evt_t * p_evt, int calls)
{
    nrf_mesh_evt_t expect;
    config_evt_Consume(&expect);
    TEST_ASSERT_EQUAL(expect.type, p_evt->type);

    switch (p_evt->type)
    {
        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            TEST_ASSERT_EQUAL(expect.params.config_load_failure.data_len,
                              p_evt->params.config_load_failure.data_len);
            TEST_ASSERT_EQUAL(expect.params.config_load_failure.id.file,
                              p_evt->params.config_load_failure.id.file);
            TEST_ASSERT_EQUAL(expect.params.config_load_failure.id.record,
                              p_evt->params.config_load_failure.id.record);
            if (p_evt->params.config_load_failure.data_len > 0)
            {
                TEST_ASSERT_EQUAL_HEX8_ARRAY(expect.params.config_load_failure.p_data,
                                            p_evt->params.config_load_failure.p_data,
                                            p_evt->params.config_load_failure.data_len);
            }
            TEST_ASSERT_EQUAL(expect.params.config_load_failure.reason,
                              p_evt->params.config_load_failure.reason);
            break;
        case NRF_MESH_EVT_CONFIG_STABLE:
            break;
        case NRF_MESH_EVT_CONFIG_STORAGE_FAILURE:
            TEST_ASSERT_EQUAL(expect.params.config_storage_failure.id.file,
                              p_evt->params.config_storage_failure.id.file);
            TEST_ASSERT_EQUAL(expect.params.config_storage_failure.id.record,
                              p_evt->params.config_storage_failure.id.record);
            break;
        default:
            TEST_FAIL_MESSAGE("Got unexpected event!");
    }
}

static void mesh_config_backend_read_all_cb(mesh_config_backend_iterate_cb_t cb, int calls)
{
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE,
                          cb(*mesh_config_entries[i].p_id,
                             (const uint8_t *) &m_load_entries[i],
                             sizeof(entry_t)));
    }
}

static void mesh_config_backend_read_all_from_ec_cb(mesh_config_backend_iterate_cb_t cb, int calls)
{
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        mesh_config_entry_id_t id = {.file = MESH_OPT_EMERGENCY_CACHE_FILE_ID, .record = i + 1};

        TEST_ASSERT_EQUAL(MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE,
                          cb(id,
                             (const uint8_t *) &m_load_ec_items[(sizeof(emergency_cache_item_t) + sizeof(entry_t)) * i],
                             sizeof(emergency_cache_item_t) + sizeof(entry_t)));
    }
}

static void mesh_config_backend_read_all_custom_cb(mesh_config_backend_iterate_cb_t cb, int calls)
{
    while (custom_load_Pending())
    {
        custom_load_t custom_load;
        custom_load_Consume(&custom_load);

        TEST_ASSERT_EQUAL(MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE,
                          cb(custom_load.id, (const uint8_t *) &custom_load.entry, custom_load.length));
    }
}

static void listener_cb(mesh_config_change_reason_t reason, mesh_config_entry_id_t id, const void * p_entry)
{
    listener_params_t expected_params;
    listener_Consume(&expected_params);
    TEST_ASSERT_EQUAL(expected_params.reason, reason);
    TEST_ASSERT_EQUAL(expected_params.id.file, id.file);
    TEST_ASSERT_EQUAL(expected_params.id.record, id.record);
    if (expected_params.p_entry == NULL)
    {
        TEST_ASSERT_NULL(p_entry);
    }
    else
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_params.p_entry, p_entry, sizeof(entry_t));
    }

}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_load(void)
{
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        m_load_entries[i].var1 = i * 10;
        m_load_entries[i].var2 = i * 20;
        entry_set_params_t expect_params = {
            .id = *mesh_config_entries[i].p_id,
            .entry = m_load_entries[i],
            .return_value = NRF_SUCCESS
        };
        entry_set_Expect(&expect_params);
    }
    mesh_config_backend_read_all_StubWithCallback(mesh_config_backend_read_all_cb);
    m_is_legacy_handled = false;

    mesh_config_load();

    TEST_ASSERT_EQUAL_MEMORY(m_load_entries, m_entries, NRF_SECTION_ENTRIES * sizeof(entry_t));
    TEST_ASSERT_EACH_EQUAL_UINT8(true, m_active, NRF_SECTION_ENTRIES);
    TEST_ASSERT_TRUE(m_is_legacy_handled);

    /* Load again, should work even if we've already loaded once: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        m_load_entries[i].var1 = i * 30;
        m_load_entries[i].var2 = i * 40;
        entry_set_params_t expect_params = {
            .id = *mesh_config_entries[i].p_id,
            .entry = m_load_entries[i],
            .return_value = NRF_SUCCESS
        };
        entry_set_Expect(&expect_params);
    }
    m_is_legacy_handled = false;

    mesh_config_load();

    TEST_ASSERT_EQUAL_MEMORY(m_load_entries, m_entries, NRF_SECTION_ENTRIES * sizeof(entry_t));
    TEST_ASSERT_EACH_EQUAL_UINT8(true, m_active, NRF_SECTION_ENTRIES);
    TEST_ASSERT_TRUE(m_is_legacy_handled);
}

void test_custom_load(void)
{
    mesh_config_backend_read_all_StubWithCallback(mesh_config_backend_read_all_custom_cb);
    /* Broken values */
    struct
    {
        custom_load_t load;
        mesh_config_load_failure_t failure;
    } vector_broken[] = {
        {{TEST_ENTRY(0), {1,2}, sizeof(entry_t) - 1}, MESH_CONFIG_LOAD_FAILURE_INVALID_LENGTH},
        {{TEST_ENTRY(0), {1,2}, 0}, MESH_CONFIG_LOAD_FAILURE_INVALID_LENGTH},
        {{MESH_CONFIG_ENTRY_ID(0xabab, 0x1234), {1,2}, sizeof(entry_t)}, MESH_CONFIG_LOAD_FAILURE_INVALID_ID},
        {{TEST_ENTRY(NRF_SECTION_ENTRIES + EXTRA_ENTRIES), {1,2}, sizeof(entry_t)}, MESH_CONFIG_LOAD_FAILURE_INVALID_ID},
        {{TEST_ENTRY(0), {1,2}, sizeof(entry_t)}, MESH_CONFIG_LOAD_FAILURE_INVALID_DATA},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(vector_broken); ++i)
    {
        nrf_mesh_evt_t load_fail_evt = {
            .type = NRF_MESH_EVT_CONFIG_LOAD_FAILURE,
            .params.config_load_failure = {
                .reason = vector_broken[i].failure,
                .id = vector_broken[i].load.id,
                .p_data = &vector_broken[i].load.entry,
                .data_len = vector_broken[i].load.length
            }
        };

        if (vector_broken[i].failure == MESH_CONFIG_LOAD_FAILURE_INVALID_DATA)
        {
            entry_set_params_t entry_set = {
                .id = vector_broken[i].load.id,
                .entry = vector_broken[i].load.entry,
                .return_value = NRF_ERROR_INVALID_DATA
            };
            entry_set_Expect(&entry_set);
        }

        config_evt_Expect(&load_fail_evt);
        custom_load_Expect(&vector_broken[i].load);
    }

    m_is_legacy_handled = false;
    mesh_config_load();
    TEST_ASSERT_TRUE(m_is_legacy_handled);

    /* Corner cases */
}

typedef enum
{
    FIRST_CALL_SETS_BUSY_RUNS_BACKEND,
    SECOND_CALL_ADDS_DIRTY_TO_BUSY,
} test_mesh_config_set_stage_t;

typedef enum
{
    FIRST_CALL_CLEARS_DIRTY_RUN_BACKEND,
    SECOND_CALL_CLEARS_BUSY,
} test_backend_evt_stage_t;

void test_set(void)
{
    entry_t entry;
    entry_set_params_t expect_params;

    /* Setting values before the backend is done with them should still trigger the backend, as the contents has changed: */
    for (uint32_t repeats = 0; repeats < 5; repeats++)
    {
        for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
        {
            for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
            {
                entry.var1 = 1 * i;
                entry.var2 = 2 * i + j;
                expect_params.id.file = mesh_config_entries[i].p_id->file;
                expect_params.id.record = mesh_config_entries[i].p_id->record + j;
                expect_params.entry = entry;
                expect_params.return_value = NRF_SUCCESS;

                entry_set_Expect(&expect_params);

                if (repeats == FIRST_CALL_SETS_BUSY_RUNS_BACKEND)
                {
                    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                                       (const uint8_t *) &entry,
                                                                       sizeof(entry),
                                                                       sizeof(entry),
                                                                       NRF_SUCCESS);
                }
                TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));

                TEST_ASSERT_EQUAL(entry.var1, m_entries[i + j].var1);
                TEST_ASSERT_EQUAL(entry.var2, m_entries[i + j].var2);
            }
        }
    }

    /* Backend is done writing everything: */
    TEST_ASSERT_NOT_NULL(m_backend_evt_cb);
    for (uint32_t repeats = 0; repeats <= SECOND_CALL_CLEARS_BUSY; repeats++)
    {
        for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
        {
            for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
            {

                entry.var1 = 1 * i;
                entry.var2 = 2 * i + j;
                expect_params.id.file = mesh_config_entries[i].p_id->file;
                expect_params.id.record = mesh_config_entries[i].p_id->record + j;
                expect_params.entry = entry;
                expect_params.return_value = NRF_SUCCESS;

                if (repeats == FIRST_CALL_CLEARS_DIRTY_RUN_BACKEND)
                {
                    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_DIRTY,
                                      mesh_config_entries[i].p_state[j]);

                    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                                       (const uint8_t *) &entry,
                                                                       sizeof(entry),
                                                                       sizeof(entry),
                                                                       NRF_SUCCESS);
                }
                else
                {
                    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);

                    if ((i == (uint32_t) (NRF_SECTION_ENTRIES - 1)) && (j == (uint32_t) (mesh_config_entries[i].max_count - 1)))
                    {
                        nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
                        config_evt_Expect(&stable_evt);
                    }
                }

                mesh_config_backend_evt_t backend_evt = {MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE,
                                                         .id = {mesh_config_entries[i].p_id->file,
                                                                mesh_config_entries[i].p_id->record + j}};

                m_backend_evt_cb(&backend_evt);

                if (repeats == FIRST_CALL_CLEARS_DIRTY_RUN_BACKEND)
                {
                    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);
                }
                else
                {
                    // SECOND_CALL_CLEARS_BUSY
                    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);
                }
            }
        }
    }

    mesh_config_backend_mock_Verify();
}

void test_fail_store(void)
{
    /* Fail storing, should cause it to re-try the first entry over and over, and result in all active, dirty, non-busy entries */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            entry_t entry = {1 * i, 2 * i + j};
            entry_set_params_t expect_params = {.id           = {mesh_config_entries[i].p_id->file,
                                                                 mesh_config_entries[i].p_id->record + j},
                                                .entry        = entry,
                                                .return_value = NRF_SUCCESS};
            entry_set_Expect(&expect_params);

            /* It'll attempt storing the first entry, as that failed last time. */
            mesh_config_backend_store_ExpectWithArrayAndReturn(*mesh_config_entries[0].p_id,
                                                                (const uint8_t *) &m_entries[0],
                                                                sizeof(m_entries[0]),
                                                                sizeof(m_entries[0]),
                                                                NRF_ERROR_NO_MEM);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));

            TEST_ASSERT_EQUAL(entry.var1, m_entries[i + j].var1);
            TEST_ASSERT_EQUAL(entry.var2, m_entries[i + j].var2);

            TEST_ASSERT_EQUAL(mesh_config_entries[i].p_state[j], MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_DIRTY);
        }
    }
}

void test_get(void)
{
    /* Run set-test first to populate the entries: */
    test_set();

    /* reading known values: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            entry_t entry;
            mesh_config_entry_id_t id = *mesh_config_entries[i].p_id;
            id.record += j;
            TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_get(id, &entry));

            TEST_ASSERT_EQUAL(m_entries[i + j].var1, entry.var1);
            TEST_ASSERT_EQUAL(m_entries[i + j].var2, entry.var2);
        }
    }

    /* reading unknown values: */
    mesh_config_entries[0].p_state[0] = 0; // no longer active
    mesh_config_entries[1].p_state[0] = 0; // no longer active
    entry_t entry;
    /* Has default: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_get(*mesh_config_entries[0].p_id, &entry));
    TEST_ASSERT_EQUAL(m_entries[0].var1, entry.var1);
    TEST_ASSERT_EQUAL(m_entries[0].var2, entry.var2);
    /* No default: */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_config_entry_get(*mesh_config_entries[1].p_id, &entry));
    /* out of bounds: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, mesh_config_entry_get(TEST_ENTRY(NRF_SECTION_ENTRIES + EXTRA_ENTRIES), &entry));
}

void test_delete(void)
{
    /* deleting unset entries results in failure and no backend calls: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            mesh_config_entry_id_t id = *mesh_config_entries[i].p_id;
            id.record += j;
            TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_config_entry_delete(id));

            /* Expect no change in flags */
            TEST_ASSERT_EQUAL_HEX8(0, mesh_config_entries[i].p_state[j]);
        }
    }

    /* Run set-test to populate the entries: */
    test_set();

    /* deleting valid entries should trigger a call to the backend, and mark the entries dirty: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            mesh_config_entry_id_t id = *mesh_config_entries[i].p_id;
            id.record += j;

            mesh_config_backend_erase_ExpectAndReturn(id, NRF_SUCCESS);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_delete(id));

            TEST_ASSERT_EQUAL_HEX8(MESH_CONFIG_ENTRY_FLAG_BUSY, mesh_config_entries[i].p_state[j]); // dirty, busy and inactive
        }
    }

    /* Attempting to erase unknown entries should cause NOT_FOUND and no side effects: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND,
                      mesh_config_entry_delete((mesh_config_entry_id_t)MESH_CONFIG_ENTRY_ID(0, ARRAY_SIZE(m_entries))));

    /* Backend finishes the delete: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            mesh_config_backend_evt_t backend_evt = {.type = MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE,
                                                     .id   = {mesh_config_entries[i].p_id->file,
                                                              mesh_config_entries[i].p_id->record + j}};

            // the final entry should trigger the stable-event:
            if ((i == (uint32_t) (NRF_SECTION_ENTRIES - 1)) && (j == (uint32_t) (mesh_config_entries[i].max_count - 1)))
            {
                nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
                config_evt_Expect(&stable_evt);
            }
            m_backend_evt_cb(&backend_evt);
            TEST_ASSERT_EQUAL_HEX8(0, mesh_config_entries[i].p_state[j]); // clean
        }
    }
}

void test_fail_delete(void)
{
    /* Run set-test to populate the entries: */
    test_set();

    /* deleting valid entries but failing the backend call should cause a repeated retry on the first entry, and mark the entries dirty: */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            mesh_config_entry_id_t id = *mesh_config_entries[i].p_id;
            id.record += j;

            mesh_config_backend_erase_ExpectAndReturn(*mesh_config_entries[0].p_id, NRF_ERROR_NO_MEM);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_delete(id));

            TEST_ASSERT_EQUAL_HEX8(MESH_CONFIG_ENTRY_FLAG_DIRTY, mesh_config_entries[i].p_state[j]); // dirty and inactive
        }
    }
}

void test_delete_when_write_in_progress(void)
{
    /* Run set-test to populate the entries: */
    test_set();

    /* Case when entry removing is started but entry has been changed but not yet written to the backend. */
    mesh_config_entry_id_t id = *mesh_config_entries[0].p_id;

    mesh_config_backend_erase_ExpectAndReturn(*mesh_config_entries[0].p_id, NRF_ERROR_NOT_FOUND);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_delete(id));

    TEST_ASSERT_EQUAL_HEX8(0, mesh_config_entries[0].p_state[0]); // clean and inactive
}

/**
 * Emulate the backend becoming busy from performing a set. Should be able to recover
 * any rejected actions from the backend callback.
 */
void test_backend_failure_recovery(void)
{
    /* Do one successful set */
    entry_t entry = {1,2};
    entry_set_params_t expect_params = {.id           = *mesh_config_entries[0].p_id,
                                        .entry        = entry,
                                        .return_value = NRF_SUCCESS};
    entry_set_Expect(&expect_params);
    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                        (const uint8_t *) &entry,
                                                        sizeof(entry),
                                                        sizeof(entry),
                                                        NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    /* Do another, but this time, fail the backend call */

    entry_set_params_t failed_expect_params = {.id           = *mesh_config_entries[1].p_id,
                                               .entry        = entry,
                                               .return_value = NRF_SUCCESS};
    entry_set_Expect(&failed_expect_params);
    mesh_config_backend_store_ExpectWithArrayAndReturn(failed_expect_params.id,
                                                        (const uint8_t *) &entry,
                                                        sizeof(entry),
                                                        sizeof(entry),
                                                        NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(failed_expect_params.id, &entry)); // no need to alert the user with an error code
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[1].p_state[0]);

    /* Trigger the backend event callback to "finish" the first action: It should attempt the failed action again. */
    mesh_config_backend_store_ExpectWithArrayAndReturn(failed_expect_params.id,
                                                        (const uint8_t *) &entry,
                                                        sizeof(entry),
                                                        sizeof(entry),
                                                        NRF_SUCCESS);
    mesh_config_backend_evt_t backend_evt = {.type = MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE,
                                             .id   = expect_params.id};
    m_backend_evt_cb(&backend_evt);
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[1].p_state[0]);

    /* Finish the retried operation, should trigger the stable event: */

    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    config_evt_Expect(&stable_evt);
    mesh_config_backend_evt_t retry_backend_evt = {.type = MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE,
                                                   .id   = failed_expect_params.id};
    m_backend_evt_cb(&retry_backend_evt);
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[1].p_state[0]);
}

void test_listeners(void)
{
    mesh_config_entry_listeners[0].p_id = &TEST_ENTRY(0);
    mesh_config_entry_listeners[0].callback = listener_cb;

    /* Run load test while a listener is active. The listener shouldn't get called at all: */
    test_load();

    /* Setting a value, but rejecting it in the state owner should not trigger listener: */
    entry_t entry = {1,2};
    entry_set_params_t expect_params = {.id           = *mesh_config_entry_listeners[0].p_id,
                                        .entry        = entry,
                                        .return_value = NRF_ERROR_INVALID_DATA};
    entry_set_Expect(&expect_params);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, mesh_config_entry_set(*mesh_config_entry_listeners[0].p_id, &entry));

    /* Successfully setting a value should cause a listener trigger: */
    listener_params_t listener_params = {
        .p_entry = &entry,
        .reason = MESH_CONFIG_CHANGE_REASON_SET,
        .id = *mesh_config_entry_listeners[0].p_id
    };
    listener_Expect(&listener_params);
    expect_params.return_value = NRF_SUCCESS;
    entry_set_Expect(&expect_params);
    mesh_config_backend_store_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(*mesh_config_entry_listeners[0].p_id, &entry));
    /* Setting, but failing writing - should still cause listener trigger: */
    listener_Expect(&listener_params);
    entry_set_Expect(&expect_params);
    mesh_config_backend_store_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(*mesh_config_entry_listeners[0].p_id, &entry));

    /* Deleting a value should cause a listener trigger with a NULL entry: */
    listener_params.reason = MESH_CONFIG_CHANGE_REASON_DELETE;
    listener_params.p_entry = NULL;
    mesh_config_entries[0].p_state[0] = MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    listener_Expect(&listener_params);
    mesh_config_backend_erase_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_delete(*mesh_config_entry_listeners[0].p_id));
    // reset the delete:
    m_active[0] = true;
    /* even if the backend fails: */
    mesh_config_entries[0].p_state[0] = MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    listener_Expect(&listener_params);
    mesh_config_backend_erase_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_delete(*mesh_config_entry_listeners[0].p_id));
    mesh_config_entries[0].p_state[0] = 0;

    listener_Verify();
    /* Listening to a ranged entry, should cause listener trigger for all entries in that range, with actual IDs */
    mesh_config_entry_listeners[1].p_id = &TEST_ENTRY(NRF_SECTION_ENTRIES); // not exact match, but it shouldn't matter.
    mesh_config_entry_listeners[1].callback = listener_cb;
    for (uint32_t i = 0; i < mesh_config_entries[NRF_SECTION_ENTRIES - 1].max_count; ++i)
    {
        listener_params.reason = MESH_CONFIG_CHANGE_REASON_SET;
        listener_params.id.file = mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->file;
        listener_params.id.record = mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->record + i;
        listener_params.p_entry = &entry;
        expect_params.id = listener_params.id;
        listener_Expect(&listener_params);
        entry_set_Expect(&expect_params);
        mesh_config_backend_store_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(listener_params.id, &entry));
    }
}

void test_storage_failure(void)
{

    /* Do one successful set */
    entry_t entry = {1,2};
    entry_set_params_t expect_params = {.id           = *mesh_config_entries[0].p_id,
                                        .entry        = entry,
                                        .return_value = NRF_SUCCESS};
    entry_set_Expect(&expect_params);
    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                        (const uint8_t *) &entry,
                                                        sizeof(entry),
                                                        sizeof(entry),
                                                        NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    /* Trigger storage failure event on the action:
     * Should both cause a storage failure event and a stable event to the user: */
    nrf_mesh_evt_t storage_fail_evt = {.type = NRF_MESH_EVT_CONFIG_STORAGE_FAILURE,
                                          .params.config_storage_failure.id = expect_params.id};
    config_evt_Expect(&storage_fail_evt);

    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    config_evt_Expect(&stable_evt);
    mesh_config_backend_evt_t retry_backend_evt = {.type = MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE,
                                                   .id = expect_params.id};
    m_backend_evt_cb(&retry_backend_evt);
}

void test_entry_available(void)
{
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        mesh_config_entry_id_t id = *mesh_config_entries[i].p_id;
        TEST_ASSERT_TRUE(mesh_config_entry_available_id(&id));
        TEST_ASSERT_EQUAL(mesh_config_entries[i].p_id->file, id.file);
        TEST_ASSERT_EQUAL(mesh_config_entries[i].p_id->record, id.record);
    }

    /* Try on one that's full: */
    mesh_config_entries[0].p_state[0] |= MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    mesh_config_entry_id_t id0 = *mesh_config_entries[0].p_id;
    TEST_ASSERT_FALSE(mesh_config_entry_available_id(&id0));

    /* Try on one that has the first one full: */
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[0] |= MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    mesh_config_entry_id_t id_last = *mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id;
    TEST_ASSERT_TRUE(mesh_config_entry_available_id(&id_last));
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->file, id_last.file);
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->record + 1, id_last.record);
    /* Repeat from returned offset, should cause the same result: */
    TEST_ASSERT_TRUE(mesh_config_entry_available_id(&id_last));
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->file, id_last.file);
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->record + 1, id_last.record);
    /* Mark it as taken, then try again. Should skip to the next: */
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[1] |= MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    TEST_ASSERT_TRUE(mesh_config_entry_available_id(&id_last));
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->file, id_last.file);
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->record + 2, id_last.record);

    /* Release the first one, should give that one on a fresh query: */
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[0] = 0;
    mesh_config_entry_id_t id_last2 = *mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id;
    TEST_ASSERT_TRUE(mesh_config_entry_available_id(&id_last2));
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->file, id_last2.file);
    TEST_ASSERT_EQUAL(mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id->record, id_last2.record);
}

void test_power_down(void)
{
    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};

    /* Run set-test first to populate the entries: */
    test_set();

    mesh_config_files[0].strategy = MESH_CONFIG_STRATEGY_ON_POWER_DOWN;

    /* mesh_config_power_down shall cause NRF_MESH_EVT_CONFIG_STABLE in any case (even if there is no data for storage) */
    config_evt_Expect(&stable_evt);
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();

    /* Dirty power down, do the action! */
    mesh_config_entries[0].p_state[0] |= MESH_CONFIG_ENTRY_FLAG_DIRTY;
    mesh_config_backend_store_ExpectWithArrayAndReturn(*mesh_config_entries[0].p_id,
                                                       (const uint8_t *) &m_entries[0],
                                                       sizeof(entry_t),
                                                       sizeof(entry_t),
                                                       NRF_SUCCESS);
    TEST_ASSERT_FALSE(mesh_config_is_busy());
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();
    TEST_ASSERT_TRUE(mesh_config_is_busy());

    mesh_config_entries[0].p_state[0] &= ~MESH_CONFIG_ENTRY_FLAG_BUSY;
    mesh_config_entries[3].p_state[0] &= ~MESH_CONFIG_ENTRY_FLAG_DIRTY;

    /* Some of the ranged ones are dirty: */
    mesh_config_files[1].strategy = MESH_CONFIG_STRATEGY_ON_POWER_DOWN;
    mesh_config_entries[0].p_state[0] |= MESH_CONFIG_ENTRY_FLAG_DIRTY;
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[0] |= MESH_CONFIG_ENTRY_FLAG_DIRTY;
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[2] |= MESH_CONFIG_ENTRY_FLAG_DIRTY;
    mesh_config_backend_store_ExpectWithArrayAndReturn(*mesh_config_entries[0].p_id,
                                                       (const uint8_t *) &m_entries[0],
                                                       sizeof(entry_t),
                                                       sizeof(entry_t),
                                                       NRF_SUCCESS);
    mesh_config_backend_store_ExpectWithArrayAndReturn(*mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_id,
                                                       (const uint8_t *) &m_entries[NRF_SECTION_ENTRIES - 1],
                                                       sizeof(entry_t),
                                                       sizeof(entry_t),
                                                       NRF_SUCCESS);

    mesh_config_backend_store_ExpectWithArrayAndReturn(TEST_ENTRY(NRF_SECTION_ENTRIES - 1 + 2),
                                                       (const uint8_t *) &m_entries[NRF_SECTION_ENTRIES - 1 + 2],
                                                       sizeof(entry_t),
                                                       sizeof(entry_t),
                                                       NRF_SUCCESS);
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();

    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[0] &= ~MESH_CONFIG_ENTRY_FLAG_DIRTY;
    mesh_config_entries[NRF_SECTION_ENTRIES - 1].p_state[2] &= ~MESH_CONFIG_ENTRY_FLAG_DIRTY;

    /* Should also support delete: */
    mesh_config_entries[0].p_state[0] = MESH_CONFIG_ENTRY_FLAG_DIRTY; // no longer active
    mesh_config_backend_erase_ExpectAndReturn(*mesh_config_entries[0].p_id, NRF_SUCCESS);
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();

    /* Entries from files with MESH_CONFIG_STRATEGY_CONTINUOUS should go to the emergency cache. */
    mesh_config_files[0].strategy = MESH_CONFIG_STRATEGY_CONTINUOUS;
    mesh_config_entries[0].p_state[0] = MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    emergency_cache_item_store_ExpectAndReturn(&mesh_config_entries[0], *mesh_config_entries[0].p_id, NRF_SUCCESS);
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();
}

void test_collision_check(void)
{
    mesh_config_backend_init_StubWithCallback(mesh_config_backend_init_callback);

    /* separate two entries into a different file so we can test all collision scenarios with those two: */
    mesh_config_entry_id_t ids[] = {
        {mesh_config_entries[0].p_id->file + 1, mesh_config_entries[0].p_id->record},
        {mesh_config_entries[1].p_id->file + 1, mesh_config_entries[1].p_id->record},
    };

    mesh_config_entries[0].p_id = &ids[0];
    mesh_config_entries[1].p_id = &ids[1];

    /* Verify that collision checking doesn't trigger between different files: */
    ids[0].record = mesh_config_entries[2].p_id->record;
    mesh_config_init();

    /* There are four distinct scenarios for the relationship between two ranges:
     *
     * 1. |---|         No collision
     *          |---|
     *
     * 2. |---|         Partial overlap
     *      |---|
     *
     * 3. |---|         Perfect overlap
     *    |---|
     *
     * 4. |-------|     Total eclipse (of the heart <3)
     *      |---|
     */

    /* Scenario 1: No collision */
    ids[0].record = 0;
    mesh_config_entries[0].max_count = 2;
    ids[1].record = 2; // starts right after the previous
    mesh_config_entries[1].max_count = 2;
    mesh_config_init();
    ids[1].record = 0;
    mesh_config_entries[1].max_count = 2;
    ids[0].record = 2;
    mesh_config_entries[0].max_count = 2;
    mesh_config_init();
    /* Scenario 2: Partial overlap */
    ids[0].record = 0;
    mesh_config_entries[0].max_count = 3;
    ids[1].record = 1;
    mesh_config_entries[1].max_count = 3;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_init());
    ids[1].record = 0;
    mesh_config_entries[1].max_count = 3;
    ids[0].record = 1;
    mesh_config_entries[0].max_count = 3;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_init());
    /* Scenario 3: Perfect overlap */
    ids[0].record = 1;
    mesh_config_entries[0].max_count = 3;
    ids[1].record = 1;
    mesh_config_entries[1].max_count = 3;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_init());
    /* Scenario 4: Total eclipse */
    ids[0].record = 0;
    mesh_config_entries[0].max_count = 6;
    ids[1].record = 1;
    mesh_config_entries[1].max_count = 3;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_init());
    ids[1].record = 0;
    mesh_config_entries[1].max_count = 6;
    ids[0].record = 1;
    mesh_config_entries[0].max_count = 3;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_init());
}

void test_clear_file(void)
{
    /* Run set-test first to populate the entries: */
    test_set();
    TEST_ASSERT_EACH_EQUAL_UINT8(true, m_active, NRF_SECTION_ENTRIES);

    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            /* Expect active state in flags */
            TEST_ASSERT_EQUAL_HEX8(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);
        }
    }

    mesh_config_backend_file_clean_Expect(mesh_config_files[1].p_backend_data);

    mesh_config_file_clear(FILE_ID_1);

    uint32_t active_state_checker = 0;
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            if (mesh_config_entries[i].p_id->file == FILE_ID_0)
            { /* Expect active state in flags for file FILE_ID_0 */
                TEST_ASSERT_EQUAL_HEX8(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);
                /* Check that the deleters are not called for active entries in the active file. */
                TEST_ASSERT_TRUE(m_active[active_state_checker]);
            }
            else
            { /* Expect not active state in flags for file FILE_ID_1 */
                TEST_ASSERT_EQUAL_HEX8(0, mesh_config_entries[i].p_state[j]);
                /* Check that the deleters are called for active entries in the removed file. */
                TEST_ASSERT_FALSE(m_active[active_state_checker]);
            }
        }

        active_state_checker++;
    }
}

void test_clear_all(void)
{
    /* Run set-test first to populate the entries: */
    test_set();
    TEST_ASSERT_EACH_EQUAL_UINT8(true, m_active, NRF_SECTION_ENTRIES);

    for (uint32_t i = 2; i < NRF_SECTION_ENTRIES; i++)
    {
        mesh_config_files[i].id = MESH_OPT_FIRST_FREE_ID + i; // the rest files are custom
    }

    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            /* Expect active state in flags */
            TEST_ASSERT_EQUAL_HEX8(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[i].p_state[j]);
        }
    }

    mesh_config_files[0].strategy = MESH_CONFIG_STRATEGY_ON_POWER_DOWN;

    for (uint32_t i = 0; i < 2; i++)
    {
        mesh_config_backend_file_clean_Expect(mesh_config_files[i].p_backend_data);
    }

    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    config_evt_Expect(&stable_evt);

    mesh_config_clear();

    for (uint32_t i = 0; i < 2; i++)
    {
        mesh_config_backend_evt_t backend_evt = {MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE,
                                                 .id.file = mesh_config_entries[i].p_id->file};
        m_backend_evt_cb(&backend_evt);
    }

    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        for (uint32_t j = 0; j < mesh_config_entries[i].max_count; ++j)
        {
            /* Expect clear state in flags */
            TEST_ASSERT_EQUAL_HEX8(0, mesh_config_entries[i].p_state[j]);
        }
    }

    // check that we called deleters for every active entry
    TEST_ASSERT_EACH_EQUAL_UINT8(false, m_active, NRF_SECTION_ENTRIES);
}

void test_file_to_entry_prioritization(void)
{ /* Check that the file changes go first the entry changes go second. */

    TEST_ASSERT_FALSE(mesh_config_is_busy());

    /* clean file 0 */
    mesh_config_backend_file_clean_Expect(mesh_config_files[0].p_backend_data);
    mesh_config_file_clear(FILE_ID_0);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    /* Do one successful set for entry from file 0. */
    entry_t entry = {1,2};
    entry_set_params_t expect_params = {.id           = *mesh_config_entries[0].p_id,
                                        .entry        = entry,
                                        .return_value = NRF_SUCCESS};
    entry_set_Expect(&expect_params);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));
    /* Entry is triggered for processing and waits for file processing completing. */
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    /* clean file 1 */
    mesh_config_backend_file_clean_Expect(mesh_config_files[1].p_backend_data);
    mesh_config_file_clear(FILE_ID_1);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    /* file 0 is ready. */
    mesh_config_backend_evt_t backend_evt1 = {MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE,
                                             .id.file = mesh_config_entries[0].p_id->file};
    m_backend_evt_cb(&backend_evt1);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    /* file 1 is ready. Mesh config fulfilled file related work and starts immediately accumulated entries. */
    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                       (const uint8_t *) &entry,
                                                       sizeof(entry),
                                                       sizeof(entry),
                                                       NRF_SUCCESS);
    mesh_config_backend_evt_t backend_evt2 = {MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE,
                                             .id.file = mesh_config_entries[1].p_id->file};
    m_backend_evt_cb(&backend_evt2);
    /* File handling has been completed. Entry process started. */
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_BUSY | MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    /* Start expectation stable event after finishing entry processing. */
    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    config_evt_Expect(&stable_evt);

    mesh_config_backend_evt_t backend_evt3 = {.type = MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE,
                                             .id   = expect_params.id};
    m_backend_evt_cb(&backend_evt3);
    /* Entry is stored in the persistent memory. */
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_FALSE(mesh_config_is_busy());
}

void test_load_from_emergency_cache(void)
{
    /* Load again but from the emergency cache. */
    for (uint32_t i = 0; i < NRF_SECTION_ENTRIES; ++i)
    {
        emergency_cache_item_t * p_cache_item =
                (emergency_cache_item_t *)&m_load_ec_items[(sizeof(emergency_cache_item_t) + sizeof(entry_t)) * i];

        p_cache_item->id = *mesh_config_entries[i].p_id;

        entry_t * p_entry = (entry_t *)p_cache_item->body;
        p_entry->var1 = (i + 1) * 50;
        p_entry->var2 = (i + 1) * 60;
        m_load_entries[i].var1 = (i + 1) * 50;
        m_load_entries[i].var2 = (i + 1) * 60;
        entry_set_params_t expect_params = {
            .id = *mesh_config_entries[i].p_id,
            .entry = *p_entry,
            .return_value = NRF_SUCCESS
        };
        entry_set_Expect(&expect_params);
    }
    mesh_config_backend_read_all_StubWithCallback(mesh_config_backend_read_all_from_ec_cb);
    /* the emergency cache is removed after restoring from the file. */
    mesh_config_backend_file_clean_Expect(mesh_config_files[2].p_backend_data);
    m_is_legacy_handled = false;

    mesh_config_load();

    TEST_ASSERT_EQUAL_MEMORY(m_load_entries, m_entries, NRF_SECTION_ENTRIES * sizeof(entry_t));
    TEST_ASSERT_EACH_EQUAL_UINT8(true, m_active, NRF_SECTION_ENTRIES);
    TEST_ASSERT_FALSE(m_is_legacy_handled);
}

void test_backend_cb_after_emrgency_item_storing(void)
{
    TEST_ASSERT_FALSE(mesh_config_is_busy());

    mesh_config_files[0].strategy = MESH_CONFIG_STRATEGY_CONTINUOUS;
    mesh_config_entries[0].p_state[0] = MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_ACTIVE;
    emergency_cache_item_store_ExpectAndReturn(&mesh_config_entries[0], *mesh_config_entries[0].p_id, NRF_SUCCESS);
    uint8_t ec_item[sizeof(emergency_cache_item_t) + 10];
    emergency_cache_item_t * p_cache_item = (emergency_cache_item_t *)ec_item;
    p_cache_item->id = *mesh_config_entries[0].p_id;
    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_BUSY, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_TRUE(mesh_config_is_busy());

    nrf_mesh_evt_t stable_evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    config_evt_Expect(&stable_evt);

    mesh_config_backend_evt_t backend_evt =
    {
        .type = MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE,
        .id.file = MESH_OPT_EMERGENCY_CACHE_FILE_ID,
        .id.record = 1,
        .p_data = ec_item
    };
    m_backend_evt_cb(&backend_evt);
    /* Entry is stored in the emergency cache. */
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_FALSE(mesh_config_is_busy());
}

void test_backend_cb_failed_writing_after_power_down(void)
{
    /* test checks that we try to write again into the emergency cache
     *  if we got failed status because of the frozen defragmentation. */

    TEST_ASSERT_FALSE(mesh_config_is_busy());

    entry_t entry;
    entry_set_params_t expect_params;
    entry.var1 = 23;
    entry.var2 = 45;
    expect_params.id.file = mesh_config_entries[0].p_id->file;
    expect_params.id.record = mesh_config_entries[0].p_id->record;
    expect_params.entry = entry;
    expect_params.return_value = NRF_SUCCESS;

    entry_set_Expect(&expect_params);

    mesh_config_backend_store_ExpectWithArrayAndReturn(expect_params.id,
                                                       (const uint8_t *) &entry,
                                                       sizeof(entry),
                                                       sizeof(entry),
                                                       NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_config_entry_set(expect_params.id, &entry));
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_BUSY, mesh_config_entries[0].p_state[0]);
    TEST_ASSERT_TRUE(mesh_config_is_busy());

    mesh_config_backend_power_down_Expect();
    mesh_config_power_down();
    TEST_ASSERT_TRUE(mesh_config_is_busy());

    emergency_cache_item_store_ExpectAndReturn(&mesh_config_entries[0], *mesh_config_entries[0].p_id, NRF_SUCCESS);

    mesh_config_backend_evt_t backend_evt =
    {
        .type = MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE,
        .id = {mesh_config_entries[0].p_id->file,
               mesh_config_entries[0].p_id->record}
    };
    m_backend_evt_cb(&backend_evt);
    /* Entry is stored in the emergency cache. */
    TEST_ASSERT_EQUAL(MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_BUSY, mesh_config_entries[0].p_state[0]);

    TEST_ASSERT_TRUE(mesh_config_is_busy());
}

void test_smart_strategy_power_down_file_cleaning(void)
{
    /* test checks that the files with power down strategy are cleaned only
     * if there is no place to store all data during the next the power down situation.
     * The emergency cache is an exception and has its own rule.  */

    mesh_config_backend_read_all_ExpectAnyArgs();
    /* no checking expected since there are no the power down files. */
    mesh_config_load();

    mesh_config_files[0].strategy = MESH_CONFIG_STRATEGY_ON_POWER_DOWN;
    mesh_config_files[1].strategy = MESH_CONFIG_STRATEGY_ON_POWER_DOWN;

    mesh_config_backend_read_all_ExpectAnyArgs();
    mesh_config_backend_is_there_power_down_place_ExpectAndReturn(mesh_config_files[0].p_backend_data, true);
    mesh_config_backend_is_there_power_down_place_ExpectAndReturn(mesh_config_files[1].p_backend_data, false);
    mesh_config_backend_file_clean_Expect(mesh_config_files[1].p_backend_data);

    mesh_config_load();
}
