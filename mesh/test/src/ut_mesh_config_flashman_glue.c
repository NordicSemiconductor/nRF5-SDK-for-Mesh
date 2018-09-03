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

#include <cmock.h>
#include <unity.h>

#include <stdlib.h>

#include "mesh_config_backend_glue.h"
#include "mesh_config_backend_file.h"
#include "event_mock.h"
#include "flash_manager_mock.h"
#include "access_config_mock.h"
#include "utils.h"

#define FILE_ID    0
#define FILE_SIZE  PAGE_SIZE
#define RECORD_ID  1
#define ENTRY_SIZE 20

typedef enum
{
    EVENT_STORE_COMPLETE,
    EVENT_ERASE_COMPLETE,
    EVENT_ERROR
} event_stage_t;

static flash_manager_queue_empty_cb_t m_flash_stable_cb;
static uint32_t m_access_flash_area;
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

static void backend_event(const mesh_config_backend_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_TRUE(p_evt->id.file == m_file.file_id);
    TEST_ASSERT_TRUE(p_evt->id.record == m_file.curr_pos);

    switch (m_event_stage)
    {
        case EVENT_STORE_COMPLETE:
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE);
            break;
        case EVENT_ERASE_COMPLETE:
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE);
            break;
        case EVENT_ERROR:
            TEST_ASSERT_TRUE(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE);
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
    TEST_ASSERT_TRUE((uint8_t *)(p_config->p_area) == (uint8_t *)&m_access_flash_area - p_config->page_count * PAGE_SIZE);
    TEST_ASSERT_TRUE(p_config->page_count == (uint32_t)CEIL_DIV(m_file.size, FLASH_MANAGER_DATA_PER_PAGE));

    m_write_complete_cb = p_config->write_complete_cb;
    m_invalidate_complete_cb = p_config->invalidate_complete_cb;
    m_remove_complete_cb = p_config->remove_complete_cb;

    return NRF_SUCCESS;
}

void setUp(void)
{
    flash_manager_mock_Init();
    event_mock_Init();
    access_config_mock_Init();

    for (uint8_t itr = 0; itr < sizeof(m_entry); itr++)
    {
        m_entry[itr] = itr;
    }
}

void tearDown(void)
{
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
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
    access_flash_area_get_ExpectAndReturn(&m_access_flash_area);
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
    fm_entry_t entry;

    flash_manager_entry_get_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, &entry);
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

    flash_manager_entry_get_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, (fm_entry_t *)p_fm_entry);
    TEST_ASSERT_TRUE(NRF_SUCCESS == mesh_config_backend_record_read(&m_file, m_entry, &length));
    TEST_ASSERT_EQUAL_MEMORY(m_entry, ((fm_entry_t *)p_fm_entry)->data, sizeof(m_entry));

    ((fm_entry_t *)p_fm_entry)->header.len_words = (2 * length + sizeof(fm_header_t)) / WORD_SIZE;
    flash_manager_entry_get_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, (fm_entry_t *)p_fm_entry);
    TEST_ASSERT_TRUE(NRF_ERROR_INVALID_LENGTH == mesh_config_backend_record_read(&m_file, m_entry, &length));

    flash_manager_entry_get_ExpectAndReturn(&m_file.glue_data.flash_manager, m_file.curr_pos, NULL);
    TEST_ASSERT_TRUE(NRF_ERROR_NOT_FOUND == mesh_config_backend_record_read(&m_file, m_entry, &length));

    free(p_fm_entry);
}

void test_record_iterate(void)
{
    uint8_t * p_data;
    uint32_t length;
    mesh_config_backend_record_iterator_t iterator =
    {
        .iterator.p_entry = NULL
    };

    uint8_t * p_fm_entry = malloc(sizeof(fm_entry_t) + sizeof(m_entry));

    ((fm_entry_t *)p_fm_entry)->header.handle = 1;
    ((fm_entry_t *)p_fm_entry)->header.len_words = (sizeof(m_entry) + sizeof(fm_header_t)) / WORD_SIZE;
    m_file.curr_pos = 0;
    flash_manager_entry_next_get_ExpectAndReturn(&m_file.glue_data.flash_manager, NULL, NULL, (fm_entry_t *)p_fm_entry);
    mesh_config_backend_record_iterate(&m_file, &p_data, &length, &iterator);
    TEST_ASSERT_TRUE(iterator.iterator.p_entry == (fm_entry_t *)p_fm_entry);
    TEST_ASSERT_TRUE(p_data == (uint8_t *)(iterator.iterator.p_entry->data));
    TEST_ASSERT_TRUE(length == sizeof(m_entry));
    TEST_ASSERT_TRUE(m_file.curr_pos == 1);

    flash_manager_entry_next_get_ExpectAndReturn(&m_file.glue_data.flash_manager, NULL, iterator.iterator.p_entry, NULL);
    mesh_config_backend_record_iterate(&m_file, &p_data, &length, &iterator);
    TEST_ASSERT_TRUE(p_data == NULL);
    TEST_ASSERT_TRUE(length == 0);

    free(p_fm_entry);
}

void test_flashman_glue_events(void)
{
    TEST_ASSERT_NOT_NULL(m_write_complete_cb);
    uint8_t * p_fm_entry = malloc(sizeof(fm_entry_t) + sizeof(m_entry));

    ((fm_entry_t *)p_fm_entry)->header.handle = 1;
    m_file.curr_pos = ((fm_entry_t *)p_fm_entry)->header.handle;

    m_event_stage = EVENT_STORE_COMPLETE;
    m_write_complete_cb(&m_file.glue_data.flash_manager, (fm_entry_t *)p_fm_entry, FM_RESULT_SUCCESS);

    m_event_stage = EVENT_ERROR;
    m_write_complete_cb(&m_file.glue_data.flash_manager, (fm_entry_t *)p_fm_entry, FM_RESULT_ERROR_NOT_FOUND);

    m_event_stage = EVENT_ERASE_COMPLETE;
    m_invalidate_complete_cb(&m_file.glue_data.flash_manager, ((fm_entry_t *)p_fm_entry)->header.handle, FM_RESULT_SUCCESS);

    m_event_stage = EVENT_ERROR;
    m_invalidate_complete_cb(&m_file.glue_data.flash_manager, ((fm_entry_t *)p_fm_entry)->header.handle, FM_RESULT_ERROR_NOT_FOUND);

    free(p_fm_entry);
}
