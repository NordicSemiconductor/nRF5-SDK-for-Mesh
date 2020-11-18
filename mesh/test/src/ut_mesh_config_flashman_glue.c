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

#include <stdlib.h>

#include "mesh_config_backend_glue.h"
#include "mesh_config_backend_file.h"
#include "event_mock.h"
#include "flash_manager_mock.h"
#include "flash_manager_defrag_mock.h"
#include "utils.h"
#include "test_assert.h"

#define FILE_ID    0
#define FILE_SIZE  PAGE_SIZE
#define RECORD_ID  1
#define ENTRY_SIZE 20
#define ENTRIES  64

typedef enum
{
    EVENT_STORE_COMPLETE,
    EVENT_ERASE_COMPLETE,
    EVENT_ERROR,
    EVENT_FILE_CLEANING_COMPLETE
} event_stage_t;

static flash_manager_queue_empty_cb_t m_flash_stable_cb;
static uint32_t m_flash_recovery_area;
static flash_manager_write_complete_cb_t m_write_complete_cb;
static flash_manager_invalidate_complete_cb_t m_invalidate_complete_cb;
static flash_manager_remove_complete_cb_t m_remove_complete_cb;
static mesh_config_backend_file_t m_file =
{
    .file_id = FILE_ID,
    .size = FILE_SIZE,
    .curr_pos = RECORD_ID
};
static uint8_t m_entry[ENTRY_SIZE];
static event_stage_t m_event_stage;
static struct
{
    fm_handle_t handle;
    uint32_t data_size;
} m_flash_manager_entries[ENTRIES];
static uint32_t m_records_read_cb_count;
static uint8_t * mp_fm_entry;

static void backend_event(const mesh_config_backend_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_TRUE(p_evt->id.file == m_file.file_id);

    switch (m_event_stage)
    {
        case EVENT_STORE_COMPLETE:
            TEST_ASSERT_TRUE(p_evt->id.record == m_file.curr_pos);
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE);
            TEST_ASSERT_TRUE((uint8_t *)(((fm_entry_t *)mp_fm_entry)->data) == p_evt->p_data);
            TEST_ASSERT_TRUE((((fm_entry_t *)mp_fm_entry)->header.len_words * WORD_SIZE - sizeof(fm_header_t)) == p_evt->length);
            break;
        case EVENT_ERASE_COMPLETE:
            TEST_ASSERT_TRUE(p_evt->id.record == m_file.curr_pos);
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE);
            TEST_ASSERT_TRUE(NULL == p_evt->p_data);
            TEST_ASSERT_TRUE(0 == p_evt->length);
            break;
        case EVENT_ERROR:
            TEST_ASSERT_TRUE(p_evt->id.record == m_file.curr_pos);
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE);
            break;
        case EVENT_FILE_CLEANING_COMPLETE:
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE);
            break;
        default:
            TEST_FAIL();
            break;
    }
}

static void flash_manager_action_queue_empty_cb(flash_manager_queue_empty_cb_t queue_empty_cb, int cmock_num_calls)
{
    (void)cmock_num_calls;
    m_flash_stable_cb = queue_empty_cb;
}

static void event_handle_cb(const nrf_mesh_evt_t * p_evt, int cmock_num_calls)
{
    (void)cmock_num_calls;
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_TRUE(p_evt->type == NRF_MESH_EVT_FLASH_STABLE);
}

static uint32_t flash_manager_add_cb(flash_manager_t * p_manager, const flash_manager_config_t * p_config, int cmock_num_calls)
{
    (void)cmock_num_calls;
    TEST_ASSERT_NOT_NULL(p_manager);
    TEST_ASSERT_TRUE(p_manager == &m_file.glue_data.flash_manager);

    TEST_ASSERT_NOT_NULL(p_config);
    TEST_ASSERT_NOT_NULL(p_config->write_complete_cb);
    TEST_ASSERT_NOT_NULL(p_config->invalidate_complete_cb);
    TEST_ASSERT_NOT_NULL(p_config->remove_complete_cb);
    TEST_ASSERT_TRUE(p_config->min_available_space == m_file.size);
    TEST_ASSERT_TRUE((uint8_t *)(p_config->p_area) == (uint8_t *)&m_flash_recovery_area - p_config->page_count * PAGE_SIZE);
    TEST_ASSERT_TRUE(p_config->page_count == (uint32_t)CEIL_DIV(m_file.size, FLASH_MANAGER_DATA_PER_PAGE));

    m_write_complete_cb = p_config->write_complete_cb;
    m_invalidate_complete_cb = p_config->invalidate_complete_cb;
    m_remove_complete_cb = p_config->remove_complete_cb;

    p_manager->config = *p_config;

    return NRF_SUCCESS;
}

static uint32_t flash_manager_entries_read_cb(const flash_manager_t * p_manager,
                                          const fm_handle_filter_t * p_filter,
                                          flash_manager_read_cb_t read_cb,
                                          void * p_args,
                                          int calls)
{
    TEST_ASSERT_NOT_NULL(p_manager);
    TEST_ASSERT_NOT_NULL(read_cb);
    TEST_ASSERT_NULL(p_filter);
    for (uint32_t i = 0; i < ARRAY_SIZE(m_flash_manager_entries); ++i)
    {
        uint8_t buffer[FLASH_MANAGER_ENTRY_MAX_SIZE];
        fm_entry_t * p_entry = (fm_entry_t *) &buffer[0];
        p_entry->header.len_words = CEIL_DIV(m_flash_manager_entries[i].data_size + sizeof(fm_entry_t), WORD_SIZE);
        p_entry->header.handle = m_flash_manager_entries[i].handle;
        TEST_ASSERT_EQUAL(FM_ITERATE_ACTION_CONTINUE, read_cb(p_entry, p_args));
    }
    return ARRAY_SIZE(m_flash_manager_entries);
}

static mesh_config_backend_iterate_action_t records_read_cb(mesh_config_entry_id_t id,
                                                            const uint8_t * p_entry,
                                                            uint32_t entry_len)
{
    TEST_ASSERT_EQUAL(FILE_ID, id.file);
    TEST_ASSERT_EQUAL(m_flash_manager_entries[m_records_read_cb_count].handle, id.record);
    TEST_ASSERT_EQUAL(ALIGN_VAL(m_flash_manager_entries[m_records_read_cb_count].data_size, WORD_SIZE), entry_len);
    TEST_ASSERT_NOT_NULL(p_entry);
    m_records_read_cb_count++;
    return MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE;
}

void setUp(void)
{
    flash_manager_mock_Init();
    flash_manager_defrag_mock_Init();
    event_mock_Init();
    m_records_read_cb_count = 0;

    for (uint8_t itr = 0; itr < sizeof(m_entry); itr++)
    {
        m_entry[itr] = itr;
    }
}

void tearDown(void)
{
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    flash_manager_defrag_mock_Verify();
    flash_manager_defrag_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
}

void test_flashman_glue_init(void)
{
    flash_manager_init_Expect();
    flash_manager_action_queue_empty_cb_set_StubWithCallback(flash_manager_action_queue_empty_cb);
    mesh_config_backend_glue_init(backend_event);
    TEST_ASSERT_NOT_NULL(m_flash_stable_cb);
    event_handle_StubWithCallback(event_handle_cb);
    m_flash_stable_cb();
}

void test_file_create(void)
{
    flash_manager_recovery_page_get_ExpectAndReturn(&m_flash_recovery_area);
    flash_manager_add_StubWithCallback(flash_manager_add_cb);
    mesh_config_backend_file_create(&m_file);
}

void test_record_write(void)
{
    flash_manager_entry_alloc_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, sizeof(m_entry), NULL);
    TEST_ASSERT_TRUE(NRF_ERROR_NO_MEM == mesh_config_backend_record_write(&m_file, m_entry, sizeof(m_entry)));

    uint8_t * p_fm_entry = malloc(sizeof(fm_entry_t) + sizeof(m_entry));

    flash_manager_entry_alloc_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, sizeof(m_entry), (fm_entry_t *)p_fm_entry);
    flash_manager_entry_commit_Expect((fm_entry_t *)p_fm_entry);
    TEST_ASSERT_TRUE(NRF_SUCCESS == mesh_config_backend_record_write(&m_file, m_entry, sizeof(m_entry)));

    TEST_ASSERT_EQUAL_MEMORY(m_entry, ((fm_entry_t *)p_fm_entry)->data, sizeof(m_entry));

    free(p_fm_entry);
}

void test_record_erase(void)
{
    fm_handle_filter_t filter = {
        .mask = 0xffff,
        .match = m_file.curr_pos,
    };
    flash_manager_entry_count_get_ExpectAndReturn(&m_file.glue_data.flash_manager, &filter, 1);
    flash_manager_entry_invalidate_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, NRF_SUCCESS);
    mesh_config_backend_record_erase(&m_file);
}

void test_record_read(void)
{
    uint32_t length = sizeof(m_entry);

    uint8_t * p_fm_entry = malloc(sizeof(fm_entry_t) + sizeof(m_entry));

    ((fm_entry_t *)p_fm_entry)->header.len_words = (length + sizeof(fm_header_t)) / WORD_SIZE;

    uint8_t * p_data = (uint8_t *)((fm_entry_t *)p_fm_entry)->data;
    for (uint32_t itr = 0; itr < sizeof(m_entry); itr++)
    {
        p_data[itr] = 2 * itr;
    }



    flash_manager_entry_read_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, NULL, &length, NRF_SUCCESS);
    flash_manager_entry_read_IgnoreArg_p_data();
    flash_manager_entry_read_ReturnMemThruPtr_p_data(p_data, length);
    TEST_ASSERT_TRUE(NRF_SUCCESS == mesh_config_backend_record_read(&m_file, m_entry, &length));
    TEST_ASSERT_EQUAL_MEMORY(m_entry, p_data, sizeof(m_entry));

    /* Read out invalid length */
    uint32_t new_length = 2 * length;
    flash_manager_entry_read_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, NULL, &length, NRF_SUCCESS);
    flash_manager_entry_read_IgnoreArg_p_data();
    flash_manager_entry_read_ReturnThruPtr_p_length(&new_length);
    TEST_ASSERT_TRUE(NRF_ERROR_INVALID_LENGTH == mesh_config_backend_record_read(&m_file, m_entry, &length));

    flash_manager_entry_read_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, NULL, &length, NRF_ERROR_NOT_FOUND);
    flash_manager_entry_read_IgnoreArg_p_data();
    TEST_ASSERT_TRUE(NRF_ERROR_NOT_FOUND == mesh_config_backend_record_read(&m_file, m_entry, &length));

    free(p_fm_entry);
}

void test_records_read(void)
{
    // setup the mock entries:
    for (uint32_t i = 0; i < ARRAY_SIZE(m_flash_manager_entries); ++i)
    {
        m_flash_manager_entries[i].handle = i;
        m_flash_manager_entries[i].data_size = (i % ENTRY_SIZE) + 1;
    }
    flash_manager_entries_read_StubWithCallback(flash_manager_entries_read_cb);
    mesh_config_backend_records_read(&m_file, records_read_cb);
    TEST_ASSERT_EQUAL(ARRAY_SIZE(m_flash_manager_entries), m_records_read_cb_count);
}

void test_flashman_glue_events(void)
{
    TEST_ASSERT_NOT_NULL(m_write_complete_cb);
    mp_fm_entry = malloc(sizeof(fm_entry_t) + sizeof(m_entry));

    ((fm_entry_t *)mp_fm_entry)->header.handle = 1;
    ((fm_entry_t *)mp_fm_entry)->header.len_words = sizeof(m_entry) / WORD_SIZE + sizeof(fm_entry_t);
    m_file.curr_pos = ((fm_entry_t *)mp_fm_entry)->header.handle;

    m_event_stage = EVENT_STORE_COMPLETE;
    m_write_complete_cb(&m_file.glue_data.flash_manager, (fm_entry_t *)mp_fm_entry, FM_RESULT_SUCCESS);

    m_event_stage = EVENT_ERROR;
    m_write_complete_cb(&m_file.glue_data.flash_manager, (fm_entry_t *)mp_fm_entry, FM_RESULT_ERROR_NOT_FOUND);

    m_event_stage = EVENT_ERASE_COMPLETE;
    m_invalidate_complete_cb(&m_file.glue_data.flash_manager, ((fm_entry_t *)mp_fm_entry)->header.handle, FM_RESULT_SUCCESS);

    m_event_stage = EVENT_ERROR;
    m_invalidate_complete_cb(&m_file.glue_data.flash_manager, ((fm_entry_t *)mp_fm_entry)->header.handle, FM_RESULT_ERROR_NOT_FOUND);

    free(mp_fm_entry);
}

void test_file_clean_normal(void)
{
    m_event_stage = EVENT_ERROR;

    flash_manager_t * p_manager = &m_file.glue_data.flash_manager;
    fm_mem_listener_t * p_listener = &m_file.glue_data.listener;

    /* 1. Run cleaning */
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_SUCCESS);
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    mesh_config_backend_file_clean(&m_file);

    /* 2. Complete cleaning and start metadata restoring.  */
    /* result is checked in flash_manager_add_cb. */
    flash_manager_add_StubWithCallback(flash_manager_add_cb);
    flash_manager_mem_listener_register_Expect(p_listener);
    m_remove_complete_cb(p_manager);

    /* 3. Complete metadata restoring. */
    p_manager->internal.state = FM_STATE_READY;
    m_event_stage = EVENT_FILE_CLEANING_COMPLETE;
    TEST_ASSERT_NOT_NULL(p_listener->callback);
    TEST_ASSERT_NOT_NULL(p_listener->p_args);
    p_listener->callback(p_listener->p_args);
}

void test_file_clean_file_already_building(void)
{
    m_event_stage = EVENT_ERROR;

    flash_manager_t * p_manager = &m_file.glue_data.flash_manager;
    fm_mem_listener_t * p_listener = &m_file.glue_data.listener;

    /* 1. Start file cleaning with invalid state in the flash manager but file is not building. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_ERROR_INVALID_STATE);
    flash_manager_is_building_ExpectAndReturn(p_manager, false);
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_file_clean(&m_file));

    /* 2. Start file cleaning with invalid state in the flash manager and file is building. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_ERROR_INVALID_STATE);
    flash_manager_is_building_ExpectAndReturn(p_manager, true);
    flash_manager_mem_listener_register_Expect(p_listener);
    mesh_config_backend_file_clean(&m_file);

    /* Restart the file cleaning using listener wrapper but expect no memory error. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_ERROR_NO_MEM);
    flash_manager_mem_listener_register_Expect(p_listener);
    p_listener->callback(p_listener->p_args);

    /* Restart the file cleaning using listener wrapper and succeed. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_SUCCESS);
    p_listener->callback(p_listener->p_args);

    /* Complete cleaning and start metadata restoring.  */
    /* result is checked in flash_manager_add_cb. */
    flash_manager_add_StubWithCallback(flash_manager_add_cb);
    flash_manager_mem_listener_register_Expect(p_listener);
    m_remove_complete_cb(p_manager);

    /* Complete metadata restoring. */
    p_manager->internal.state = FM_STATE_READY;
    m_event_stage = EVENT_FILE_CLEANING_COMPLETE;
    TEST_ASSERT_NOT_NULL(p_listener->callback);
    TEST_ASSERT_NOT_NULL(p_listener->p_args);
    p_listener->callback(p_listener->p_args);

    /* 3. Start file cleaning with invalid state in the flash manager and file is removing. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, true);
    mesh_config_backend_file_clean(&m_file);
}

void test_file_clean_lack_of_memory(void)
{
    m_event_stage = EVENT_ERROR;

    flash_manager_t * p_manager = &m_file.glue_data.flash_manager;
    fm_mem_listener_t * p_listener = &m_file.glue_data.listener;

    /* 1. Start file cleaning with lack of memory in the flash manager. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_ERROR_NO_MEM);
    flash_manager_mem_listener_register_Expect(p_listener);
    mesh_config_backend_file_clean(&m_file);

    /* 2. Restart the file cleaning using listener wrapper. */
    flash_manager_is_removing_ExpectAndReturn(p_manager, false);
    flash_manager_remove_ExpectAndReturn(p_manager, NRF_SUCCESS);
    mesh_config_backend_file_clean(&m_file);

    /* 3. Start file restoring with lack of memory in the flash manager. */
    flash_manager_add_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    flash_manager_mem_listener_register_Expect(p_listener);
    m_remove_complete_cb(p_manager);

    /* 4. Restart the file restoring using listener wrapper. */
    flash_manager_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    flash_manager_mem_listener_register_Expect(p_listener);
    p_listener->callback(p_listener->p_args);

    /* 4. Complete metadata restoring.  */
    p_manager->internal.state = FM_STATE_READY;
    m_event_stage = EVENT_FILE_CLEANING_COMPLETE;
    p_listener->callback(p_listener->p_args);
}


void test_backend_flashman_power_down(void)
{
    flash_manager_defrag_freeze_Expect();
    mesh_config_backend_power_down();
}

void test_remaining_place_calculation(void)
{
    /* Emulate the allocated file area that is equal to 3 pages. */
    uint8_t pseudo_flash[PAGE_SIZE * 3] = {0};

    for (uint8_t i = 0; i < 3; i++)
    {
        flash_manager_page_t * p_page = (flash_manager_page_t *)&pseudo_flash[i * PAGE_SIZE];
        p_page->metadata.pages_in_area = 3;
        p_page->metadata.page_index = i;
    }
    m_file.glue_data.flash_manager.config.page_count = 3;
    m_file.glue_data.flash_manager.config.min_available_space = PAGE_SIZE / 2 + 1;

    m_file.glue_data.flash_manager.config.p_area = (flash_manager_page_t *)&pseudo_flash[0];
    m_file.glue_data.flash_manager.internal.p_seal = (const fm_entry_t *)(&pseudo_flash[0] + PAGE_SIZE / 2);

    TEST_ASSERT_TRUE(mesh_config_backend_is_there_power_down_place(&m_file));

    m_file.glue_data.flash_manager.config.p_area = (flash_manager_page_t *)&pseudo_flash[1 * PAGE_SIZE];
    m_file.glue_data.flash_manager.internal.p_seal = (const fm_entry_t *)(&pseudo_flash[1 * PAGE_SIZE] + PAGE_SIZE / 2);

    TEST_ASSERT_TRUE(mesh_config_backend_is_there_power_down_place(&m_file));

    m_file.glue_data.flash_manager.config.p_area = (flash_manager_page_t *)&pseudo_flash[2 * PAGE_SIZE];
    m_file.glue_data.flash_manager.internal.p_seal = (const fm_entry_t *)(&pseudo_flash[2 * PAGE_SIZE] + PAGE_SIZE / 2);

    TEST_ASSERT_FALSE(mesh_config_backend_is_there_power_down_place(&m_file));
}
