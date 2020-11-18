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

#include "emergency_cache.h"

#include <stddef.h>
#include <unity.h>
#include "mesh_opt.h"
#include "mesh_config_backend_mock.h"

#define FILE1_ID   0x0001
#define FILE2_ID   0x0002
#define FILE3_ID   0x0003
#define RECORD1_ID 0x0010
#define RECORD2_ID 0x0020
#define RECORD3_ID 0x0030

#define ENTRY_NUMBER 3

#define MESH_OPT_EC_FAKE_RECORD  0x0001

static uint32_t trap_setter(mesh_config_entry_id_t id, const void * p_entry);
static void trap_deleter(mesh_config_entry_id_t id);
static void local_getter(mesh_config_entry_id_t id, void * p_entry);

MESH_CONFIG_ENTRY(entry0, MESH_CONFIG_ENTRY_ID(FILE1_ID, RECORD1_ID), 2, sizeof(uint8_t), \
                  trap_setter, local_getter, trap_deleter, false);
MESH_CONFIG_ENTRY(entry1, MESH_CONFIG_ENTRY_ID(FILE2_ID, RECORD2_ID), 1, MESH_CONFIG_ENTRY_MAX_SIZE, \
                  trap_setter, local_getter, trap_deleter, false);
MESH_CONFIG_ENTRY(entry2, MESH_CONFIG_ENTRY_ID(FILE3_ID, RECORD3_ID), 3, sizeof(uint16_t), \
                  trap_setter, local_getter, trap_deleter, false);

static const mesh_config_entry_params_t * mp_params[ENTRY_NUMBER] = {&m_entry0_params, &m_entry1_params, &m_entry2_params};

static uint16_t m_file;
static uint16_t m_record;
static uint32_t m_length;
static uint16_t m_fake_record = MESH_OPT_EC_FAKE_RECORD;

void setUp(void)
{
    mesh_config_backend_mock_Init();

    emergency_cache_item_init();
}

void tearDown(void)
{
    mesh_config_backend_mock_Verify();
    mesh_config_backend_mock_Destroy();
}

static uint32_t backend_store_cb(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_EQUAL_UINT16(MESH_OPT_EMERGENCY_CACHE_FILE_ID, id.file);
    TEST_ASSERT_EQUAL_UINT16(m_fake_record, id.record);
    m_fake_record++;

    emergency_cache_item_t * p_ec_item = emergency_cache_item_get(p_entry);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL_UINT16(m_file, p_ec_item->id.file);
    TEST_ASSERT_EQUAL_UINT16(m_record, p_ec_item->id.record);
    TEST_ASSERT_EQUAL_UINT32(m_length, emergency_cache_restored_item_length_get(entry_len));

    uint8_t template[MESH_CONFIG_ENTRY_MAX_SIZE];
    for (uint8_t i = 0; i < m_length; i++)
    {
        template[i] = i;
    }

    TEST_ASSERT_EQUAL_HEX8_ARRAY(template, p_ec_item->body, m_length);

    return NRF_SUCCESS;
}

static uint32_t trap_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    (void)id;
    (void)p_entry;
    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

static void trap_deleter(mesh_config_entry_id_t id)
{
    (void)id;
    TEST_FAIL();
}

static void local_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint8_t * p_raw_data = (uint8_t *)p_entry;

    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL_UINT16(m_file, id.file);
    TEST_ASSERT_EQUAL_UINT16(m_record, id.record);

    for (uint8_t i = 0; i < m_length; i++)
    {
        p_raw_data[i] = i;
    }
}

void test_emergency_cache_memory_lack(void)
{
    uint32_t status;

    m_file = mp_params[0]->p_id->file;
    m_record = mp_params[0]->p_id->record;
    m_length = mp_params[0]->entry_size;
    mesh_config_backend_store_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    status = emergency_cache_item_store(mp_params[0], *mp_params[0]->p_id);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);
}

void test_emergency_cache(void)
{
    uint32_t status;

    mesh_config_backend_store_StubWithCallback(backend_store_cb);

    for (uint8_t i = 0; i < ENTRY_NUMBER; i++)
    {
        m_file = mp_params[i]->p_id->file;
        m_record = mp_params[i]->p_id->record;
        m_length = mp_params[i]->entry_size;

        status = emergency_cache_item_store(mp_params[i], *mp_params[i]->p_id);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    }
}
