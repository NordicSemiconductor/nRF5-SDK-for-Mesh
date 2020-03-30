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

#include "test_assert.h"
#include "mesh_config_entry.h"
#include "mesh_config_backend.h"
#include "nrf_error.h"
#include "mesh_config_backend_glue_mock.h"

#ifndef NUMBER_OF_FILES
#define NUMBER_OF_FILES     5u
#endif

#ifndef NUMBER_OF_RECORDS
#define NUMBER_OF_RECORDS    20u
#endif

#define TABLE_SIZE    (NUMBER_OF_FILES * NUMBER_OF_RECORDS)

static mesh_config_entry_id_t m_entry_ids[TABLE_SIZE];
static mesh_config_entry_params_t m_entry_table[TABLE_SIZE];
static mesh_config_file_params_t m_file[NUMBER_OF_FILES];
static mesh_config_backend_file_t m_backend_file[NUMBER_OF_FILES];
static uint16_t m_file_size_table[NUMBER_OF_FILES];
static uint8_t m_data;
static uint32_t m_read_callback_counter;

static void entry_table_create(void)
{
    memset(m_entry_table, 0, sizeof(m_entry_table));
    memset(m_file_size_table, 0, sizeof(m_file_size_table));

    for (uint32_t itr = 0; itr < TABLE_SIZE; itr++)
    {
        m_entry_ids[itr].file = itr / NUMBER_OF_RECORDS;
        m_entry_ids[itr].record = itr;
        m_entry_table[itr].p_id = &m_entry_ids[itr];
        m_entry_table[itr].entry_size = itr + 1;
        m_entry_table[itr].max_count = (itr & 1ul) + 1;
        m_file_size_table[m_entry_table[itr].p_id->file] += m_entry_table[itr].entry_size * m_entry_table[itr].max_count;
    }

    for (uint32_t itr = 0; itr < NUMBER_OF_FILES; itr++)
    {
        m_file[itr].id = itr;
        m_file[itr].strategy = MESH_CONFIG_STRATEGY_CONTINUOUS;
        m_file[itr].p_backend_data = &m_backend_file[itr];
    }
}

static void event_cb(const mesh_config_backend_evt_t * p_evt)
{
    (void)p_evt;
}

static uint16_t mesh_config_record_size_calculate_cb(uint16_t entry_size, int cmock_num_calls)
{
    (void) cmock_num_calls;
    return entry_size;
}

static uint32_t mesh_config_backend_file_create_positive_cb(mesh_config_backend_file_t * p_file, int cmock_num_calls)
{
    TEST_ASSERT_NOT_NULL(p_file);
    TEST_ASSERT_EQUAL(cmock_num_calls, p_file->file_id);
    TEST_ASSERT_EQUAL(m_file_size_table[cmock_num_calls], p_file->size);
    TEST_ASSERT_EQUAL_UINT16(0u, p_file->curr_pos);

    return NRF_SUCCESS;
}

static uint32_t mesh_config_backend_record_write_cb(mesh_config_backend_file_t * p_file, const uint8_t * p_entry, uint32_t entry_len, int cmock_num_calls)
{
    TEST_ASSERT_NOT_NULL(p_file);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_TRUE(0ul != entry_len);
    TEST_ASSERT_EQUAL(p_file->file_id, m_entry_table[cmock_num_calls].p_id->file);
    TEST_ASSERT_EQUAL(p_file->curr_pos, m_entry_table[cmock_num_calls].p_id->record);
    TEST_ASSERT_EQUAL(&m_data, p_entry);
    TEST_ASSERT_EQUAL((uint32_t)(m_entry_table[cmock_num_calls].max_count * m_entry_table[cmock_num_calls].entry_size), entry_len);

    return NRF_SUCCESS;
}

static uint32_t mesh_config_backend_record_erase_cb(mesh_config_backend_file_t * p_file, int cmock_num_calls)
{
    TEST_ASSERT_NOT_NULL(p_file);
    TEST_ASSERT_TRUE(m_entry_table[cmock_num_calls].p_id->file == p_file->file_id);
    TEST_ASSERT_TRUE(m_entry_table[cmock_num_calls].p_id->record == p_file->curr_pos);

    return NRF_SUCCESS;
}

static uint32_t mesh_config_backend_record_read_cb(mesh_config_backend_file_t * p_file, uint8_t * p_entry, uint32_t * p_entry_len, int cmock_num_calls)
{
    TEST_ASSERT_NOT_NULL(p_file);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_NOT_NULL(p_entry_len);
    TEST_ASSERT_TRUE(0ul != *p_entry_len);
    TEST_ASSERT_TRUE(m_entry_table[cmock_num_calls].p_id->file == p_file->file_id);
    TEST_ASSERT_TRUE(m_entry_table[cmock_num_calls].p_id->record == p_file->curr_pos);
    TEST_ASSERT_TRUE(p_entry == &m_data);
    TEST_ASSERT_TRUE(*p_entry_len == (uint32_t)(m_entry_table[cmock_num_calls].max_count * m_entry_table[cmock_num_calls].entry_size));

    return NRF_SUCCESS;
}

static mesh_config_backend_iterate_action_t dummy_read_callback(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len)
{
    /* Will never actually get called by the mesh_config_backend module, just here to provide something to feed the lower API with */
    return MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE;
}

void setUp(void)
{
    mesh_config_backend_glue_mock_Init();
    m_read_callback_counter = 0;

    entry_table_create();
}

void tearDown(void)
{
    mesh_config_backend_glue_mock_Verify();
    mesh_config_backend_glue_mock_Destroy();
}

void test_initialization_negative(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_init(NULL,
                                                         TABLE_SIZE,
                                                         m_file,
                                                         NUMBER_OF_FILES,
                                                         event_cb));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_init(m_entry_table,
                                                         0,
                                                         m_file,
                                                         NUMBER_OF_FILES,
                                                         event_cb));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_init(m_entry_table,
                                                         TABLE_SIZE,
                                                         NULL,
                                                         NUMBER_OF_FILES,
                                                         event_cb));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_init(m_entry_table,
                                                         TABLE_SIZE,
                                                         m_file,
                                                         0,
                                                         event_cb));
}

void test_initialization_positive(void)
{
    mesh_config_backend_glue_init_Expect(event_cb);
    mesh_config_record_size_calculate_StubWithCallback(mesh_config_record_size_calculate_cb);
    mesh_config_backend_file_create_StubWithCallback(mesh_config_backend_file_create_positive_cb);
    for (uint32_t i = 0; i < NUMBER_OF_FILES; ++i)
    {
        mesh_config_backend_file_power_down_time_get_ExpectAndReturn(&m_file[i], 2000);
    }
    mesh_config_backend_init(m_entry_table, TABLE_SIZE, m_file, NUMBER_OF_FILES, event_cb);
}

void test_store_negative(void)
{
    mesh_config_entry_id_t * p_id = (mesh_config_entry_id_t *) m_entry_table[0].p_id;
    uint8_t data[m_entry_table[0].entry_size];

    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_store(*p_id, NULL, m_entry_table[0].entry_size));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_store(*p_id, data, 0));

    p_id->file = TABLE_SIZE + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_store(*p_id, data, m_entry_table[0].entry_size));
}

void test_store_positive(void)
{
    mesh_config_backend_record_write_StubWithCallback(mesh_config_backend_record_write_cb);
    for (uint32_t itr = 0; itr < TABLE_SIZE; itr++)
    {
        mesh_config_backend_store(*m_entry_table[itr].p_id, &m_data, m_entry_table[itr].max_count * m_entry_table[itr].entry_size);
    }
}

void test_erase_negative(void)
{
    mesh_config_entry_id_t * p_id = (mesh_config_entry_id_t *) m_entry_table[0].p_id;

    p_id->file = TABLE_SIZE + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_erase(*p_id));
}

void test_erase_positive(void)
{
    mesh_config_backend_record_erase_StubWithCallback(mesh_config_backend_record_erase_cb);
    for (uint32_t itr = 0; itr < TABLE_SIZE; itr++)
    {
        mesh_config_backend_erase(*m_entry_table[itr].p_id);
    }
}

void test_read_negative(void)
{
    mesh_config_entry_id_t * p_id = (mesh_config_entry_id_t *) m_entry_table[0].p_id;
    uint8_t data[m_entry_table[0].entry_size];
    uint32_t length = m_entry_table[0].entry_size;

    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_read(*p_id, NULL, &length));
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_read(*p_id, data, NULL));
    length = 0ul;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_read(*p_id, data, &length));

    length = m_entry_table[0].entry_size;
    p_id->file = TABLE_SIZE + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_read(*p_id, data, &length));
}

void test_read_positive(void)
{
    mesh_config_backend_record_read_StubWithCallback(mesh_config_backend_record_read_cb);
    for (uint32_t itr = 0; itr < TABLE_SIZE; itr++)
    {
        uint32_t length = m_entry_table[itr].max_count * m_entry_table[itr].entry_size;
        mesh_config_backend_read(*m_entry_table[itr].p_id, &m_data, &length);
    }
}

void test_read_all_negative(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_config_backend_read_all(NULL));
}

/**
 * The read all function is just a wrapper on the lower backend that calls its read for every file,
 * so no need to check the lower callback behavior.
 */
void test_read_all_positive(void)
{
    for (uint32_t i = 0; i < NUMBER_OF_FILES; ++i)
    {
        mesh_config_backend_records_read_Expect(m_file[i].p_backend_data, dummy_read_callback);
    }
    mesh_config_backend_read_all(dummy_read_callback);
}

void test_power_down_time(void)
{
    mesh_config_record_size_calculate_StubWithCallback(mesh_config_record_size_calculate_cb);
    mesh_config_backend_file_create_StubWithCallback(mesh_config_backend_file_create_positive_cb);

    /* Initialize with normal parameters, should result in no power down storage requirements */
    memset(m_backend_file, 0, sizeof(m_backend_file));
    mesh_config_backend_glue_init_Expect(event_cb);
    for (uint32_t i = 0; i < NUMBER_OF_FILES; ++i)
    {
        mesh_config_backend_file_power_down_time_get_ExpectAndReturn(&m_file[i], 2000);
    }

    mesh_config_backend_init(m_entry_table, TABLE_SIZE, m_file, NUMBER_OF_FILES, event_cb);

    TEST_ASSERT_EQUAL(2000 * NUMBER_OF_FILES, mesh_config_backend_power_down_time_get());
}
