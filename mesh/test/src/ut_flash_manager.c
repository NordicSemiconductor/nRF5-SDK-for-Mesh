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
#include <stdio.h>

#include "flash_manager.h"
#include "flash_manager_internal.h"
#include "flash_manager_defrag_mock.h"
#include "mesh_flash.h"
#include "bearer_event.h"
#include "fifo.h"
#include "flash_manager_test_util.h"
#include "utils.h"
#include "test_assert.h"

static uint32_t m_expect_mem_listener;
static bool m_recursive_listener; /**< The listener will re-add itself in the callback */
static int m_expect_queue_empty_cb_count;

static void queue_empty_cb_Expect(void)
{
    m_expect_queue_empty_cb_count++;
}

static void queue_empty_cb_Verify(void)
{
    TEST_ASSERT_EQUAL_MESSAGE(0, m_expect_queue_empty_cb_count, "Queue empty callback not called expected amount of times");
    flash_manager_action_queue_empty_cb_set(NULL);
}

static void queue_empty_cb(void)
{
    TEST_ASSERT_MESSAGE(m_expect_queue_empty_cb_count > 0, "Got unexpected queue empty callback");
    m_expect_queue_empty_cb_count--;
}

void setUp(void)
{
    flash_manager_defrag_mock_Init();
    flash_manager_test_util_setup();
}

void tearDown(void)
{
    flash_manager_defrag_mock_Verify();
    flash_manager_defrag_mock_Destroy();
}

static void mem_listener_cb(void * p_args)
{
    TEST_ASSERT_NOT_NULL(p_args);
    TEST_ASSERT_TRUE(m_expect_mem_listener > 0);
    m_expect_mem_listener--;
    if (m_recursive_listener)
    {
        /* the p_args points to the listener. */
        flash_manager_mem_listener_register(p_args);
    }
}

static fm_iterate_action_t read_cb(const fm_entry_t * p_entry, void * p_args)
{
    test_entry_t * p_expect = p_args;
    TEST_ASSERT_EQUAL(p_expect->handle, p_entry->header.handle);
    TEST_ASSERT_EQUAL(p_expect->len, p_entry->header.len_words);
    for (int32_t word = 0; word < p_expect->len - 1; ++word)
    {
        TEST_ASSERT_EQUAL_HEX32(p_expect->data_value, p_entry->data[word]);
    }
    return FM_ITERATE_ACTION_CONTINUE;
}
/*****************************************************************************
* Tests
*****************************************************************************/
void test_init(void)
{
    flash_manager_defrag_init_ExpectAndReturn(true);
    flash_manager_init();
}

void test_manager_add(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    /* Add basic 3 page blank area */
    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 3,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };

    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    TEST_ASSERT_EQUAL_MEMORY(&config, &manager.config, sizeof(config));
    flash_execute();
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
        validate_blank_flash(&area[i].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);
    }

    /* Try adding the area again. Since everything lines up, we should succeed
     * without needing to flash anything. */
    g_flash_queue_slots = 0; /* prevent flashing */
    memset(&manager, 0, sizeof(manager));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
        validate_blank_flash(&area[i].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);
    }
    /* Try adding the last two pages as a separate area. Should fail, and
     * shouldn't attempt flashing anything. */
    g_flash_queue_slots = 0xFFFF;
    memset(&manager, 0, sizeof(manager));
    config.p_area = &area[1];
    config.page_count = 2;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    flash_execute();
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
        validate_blank_flash(&area[i].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);
    }
    /* Try adding the first two pages as a separate area. Should fail, and
     * shouldn't attempt flashing anything. */
    g_flash_queue_slots = 0xFFFF;
    memset(&manager, 0, sizeof(manager));
    config.p_area = &area[0];
    config.page_count = 2;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
        validate_blank_flash(&area[i].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);
    }

    /* Add the area again, but block flashing until later. */
    g_flash_queue_slots = 0xFFFF; /* prevent flashing */
    g_delayed_execution = true;
    memset(&manager, 0, sizeof(manager));
    memset(area, 0xFF, sizeof(area));
    config.p_area = area;
    config.page_count = 3;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    validate_blank_flash(area, sizeof(area) / WORD_SIZE);

    g_process_cb();
    flash_execute();
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
        validate_blank_flash(&area[i].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);
    }
    g_delayed_execution = false;

    /* Add a new, separate area. */
    g_flash_queue_slots = 0xFFFF;
    static flash_manager_page_t area_b[1] __attribute__((aligned(PAGE_SIZE)));
    memset(area_b, 0xFF, sizeof(area_b));
    TEST_ASSERT_EQUAL(PAGE_SIZE, sizeof(area_b));
    flash_manager_t manager_b;
    config.p_area = area_b;
    config.page_count = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager_b, &config));
    flash_execute();
    validate_metadata(&area_b[0].metadata, 0, 1);
    validate_blank_flash(&area_b[0].raw[8], (PAGE_SIZE - 8) / WORD_SIZE);

    /* Add an area with a bunch of entries already in, check that the module
     * gets the right values for the RAM-state */
    test_entry_t entries[] =
    {
        {0x0010, 0x0001, 0x01010101},
        {0x0020, 0x0000, 0xabababab}, /* invalid entry */
        {0x0010, 0x0002, 0x02020202},
        {0x0030, 0x0000, 0xabababab}, /* invalid entry */
        {0x0001, 0x0003, 0x03030303},
        {0x0080, 0x0004, 0x04040404},
        {0x0001, 0x0000, 0xabababab}, /* invalid entry */
    };
    memset(&manager, 0, sizeof(manager));
    config.p_area = area;
    config.page_count = 3;
    build_test_page(area, 3, entries, ARRAY_SIZE(entries), true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    TEST_ASSERT_EQUAL_MEMORY(&config, &manager.config, sizeof(config));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }
    TEST_ASSERT_EQUAL_PTR(&area[0].raw[244*WORD_SIZE], manager.internal.p_seal);
    TEST_ASSERT_EQUAL(0x51 * WORD_SIZE, manager.internal.invalid_bytes);

    /* build a bunch of areas at the same time */
    g_delayed_execution = true;

    /* To avoid looking into the internal definitions in flash_manager.c we calculate the size of
     * each entry based on the action_t struct, remembering to add the overhead from
     * packet_buffer_packet_t for each of the entries.
     */
#define AREA_COUNT 2
#define FMAN_COUNT ((FLASH_MANAGER_POOL_SIZE + sizeof(packet_buffer_packet_t)) / \
                    ((sizeof(flash_manager_metadata_t)                  \
                      + sizeof(flash_manager_t*) + sizeof(uint8_t)      \
                      + sizeof(packet_buffer_packet_t))*AREA_COUNT))

    flash_manager_t managers[FMAN_COUNT];
    static flash_manager_page_t areas[FMAN_COUNT][AREA_COUNT] __attribute__((aligned(PAGE_SIZE)));
    memset(areas, 0xFF, sizeof(areas));
    for (uint32_t i = 0; i < FMAN_COUNT-1; i++)
    {
        config.p_area = &areas[i][0];
        config.page_count = 2;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&managers[i], &config));
    }
    /* execute the flashing */
    g_delayed_execution = false;
    g_process_cb();
    flash_execute();
    for (uint32_t i = 0; i < FMAN_COUNT-1; i++)
    {
        for (uint32_t j = 0; j < 2; j++)
        {
            validate_metadata(&areas[i][j].metadata, j, 2);
        }
    }

    /* Add too many areas to fit */
    g_delayed_execution = true;
    memset(areas, 0xFF, sizeof(areas));
    for (uint32_t i = 0; i < FMAN_COUNT-1; i++)
    {
        config.p_area = &areas[i][0];
        config.page_count = 2;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&managers[i], &config));
    }
    config.p_area = &areas[FMAN_COUNT-1][0];
    config.page_count = 2;
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, flash_manager_add(&managers[FMAN_COUNT-1], &config));
    /* execute the flashing */
    g_delayed_execution = false;
    g_process_cb();
    flash_execute();
    /* read the failed manager, should work this time. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&managers[5], &config));

    /* Add a new area that is being defragged */
    g_flash_queue_slots = 0xFFFF;
    static flash_manager_page_t area_c[1] __attribute__((aligned(PAGE_SIZE)));
    static flash_manager_page_t area_c_expect[1] __attribute__((aligned(PAGE_SIZE)));
    memset(area_c, 0xFF, sizeof(area_c));
    TEST_ASSERT_EQUAL(PAGE_SIZE, sizeof(area_c));
    flash_manager_t manager_c;
    config.p_area = area_c;
    config.page_count = 1;
    build_test_page(area_c, 1, entries, ARRAY_SIZE(entries), false);
    memcpy(area_c_expect, area_c, sizeof(flash_manager_page_t));
    flash_manager_defragging_ExpectAndReturn(&manager_c, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager_c, &config));
    TEST_ASSERT_EQUAL(FM_STATE_DEFRAG, manager_c.internal.state);
    flash_execute();
    TEST_ASSERT_EQUAL_MEMORY(area_c_expect, area_c, sizeof(flash_manager_page_t));
}

void test_invalidate_duplicate_of_last_entry(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t        manager;
    flash_manager_config_t config = {.p_area                 = area,
                                     .page_count             = 3,
                                     .min_available_space    = 0,
                                     .write_complete_cb      = NULL,
                                     .invalidate_complete_cb = NULL};

    test_entry_t entries[] = {
        {0x0010, 0x0001, 0x01010101},
        {0x0020, 0x0000, 0xabababab},
        {0x0010, 0x0002, 0x02020202},
        {0x0080, 0x0004, 0x04040404},
        {0x0001, 0x0000, 0xabababab},
        {0x0080, 0x0004, 0x44444444}, /* duplicate of the entry two entries before this one */
    };
    memset(&manager, 0, sizeof(manager));
    config.p_area     = area;
    config.page_count = 3;
    build_test_page(area, 3, entries, ARRAY_SIZE(entries), true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    TEST_ASSERT_EQUAL_MEMORY(&config, &manager.config, sizeof(config));
    flash_execute();

    /* Ensure that the first 0x0004 entry was invalidated */
    FLASH_EXPECT(&area[0], 0x40*4, 0x80, 0x00, 0x00, 0x00);
}

void test_replace(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 3,
        .min_available_space = 0,
        .write_complete_cb = write_complete_callback,
        .invalidate_complete_cb = NULL
    };
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();

    gp_active_manager = &manager;
    g_expected_result = FM_RESULT_SUCCESS;

    /* try adding an entry without data */
    fm_handle_t handle = 0x1234;
    fm_entry_t * p_entry = flash_manager_entry_alloc(&manager, handle, 0);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(1, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[0].raw[8];
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0x01, 0x00, 0x34, 0x12);
    TEST_ASSERT_EQUAL(1, g_completes);
    /* replace it */
    p_entry = flash_manager_entry_alloc(&manager, handle, 0);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(1, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[0].raw[12];
    flash_execute();
    TEST_ASSERT_EQUAL(2, g_completes);
    FLASH_EXPECT(&area[0], 0, 0x01, 0x00, 0x00, 0x00);
    FLASH_EXPECT(&area[0], 4, 0x01, 0x00, 0x34, 0x12);
    FLASH_EXPECT(&area[0], 8, 0xff, 0xff, 0xff, 0x7f); /* seal */
    /* replace it with data */
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[0].raw[16];
    flash_execute();
    FLASH_EXPECT(&area[0], 4, 0x01, 0x00, 0x00, 0x00);
    FLASH_EXPECT(&area[0], 8, 0x03, 0x00, 0x34, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
    FLASH_EXPECT(&area[0], 20, 0xff, 0xff, 0xff, 0x7f); /* seal */
    /* Add a whole bunch of entries, but don't fill up the first page. */

    uint32_t area_offset = 20;
    for (uint32_t i = 0; i < 30; i++)
    {
        handle = i + 1;
        p_entry = flash_manager_entry_alloc(&manager, handle, 4 * 4);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(5, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(handle, p_entry->header.handle);
        for (uint32_t j = 0; j < 4; j++)
        {
            p_entry->data[j] = (j << 24) | (j << 16) | (j << 8) | (j);
        }
        flash_manager_entry_commit(p_entry);
        gp_expected_entry = (const fm_entry_t *) &area[0].raw[area_offset + 8];
        flash_execute();
        FLASH_EXPECT(&area[0], area_offset, 0x05, 0x00, i + 1, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x01, 0x01, 0x01, 0x01,
                0x02, 0x02, 0x02, 0x02,
                0x03, 0x03, 0x03, 0x03);
        area_offset += 20;
        FLASH_EXPECT(&area[0], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */
    }
    /* replace every second of the ones we added */
    for (uint32_t i = 0; i < 30; i+= 2)
    {
        handle = i + 1;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(handle, p_entry->header.handle);
        p_entry->data[0] = 0x01234567;
        p_entry->data[1] = 0x89abcdef;
        flash_manager_entry_commit(p_entry);
        gp_expected_entry = (const fm_entry_t *) &area[0].raw[area_offset + 8];
        flash_execute();
        FLASH_EXPECT(&area[0], 20 + i * 20, 0x05, 0x00, 0x00, 0x00);
        FLASH_EXPECT(&area[0], area_offset, 0x03, 0x00, i + 1, 0x00,
                0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
        area_offset += 12;
        FLASH_EXPECT(&area[0], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */
    }
    /* replace the other half */
    for (uint32_t i = 1; i < 30; i+= 2)
    {
        handle = i + 1;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(handle, p_entry->header.handle);
        p_entry->data[0] = 0x01234567;
        p_entry->data[1] = 0x89abcdef;
        flash_manager_entry_commit(p_entry);
        gp_expected_entry = (const fm_entry_t *) &area[0].raw[area_offset + 8];
        flash_execute();
        FLASH_EXPECT(&area[0], 20 + i * 20, 0x05, 0x00, 0x00, 0x00);
        FLASH_EXPECT(&area[0], area_offset, 0x03, 0x00, i + 1, 0x00,
                0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
        area_offset += 12;
        FLASH_EXPECT(&area[0], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */
    }
    /* add a big entry that needs to go on the next page: */
    handle++;
    p_entry = flash_manager_entry_alloc(&manager, handle, 100);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(26, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(handle, p_entry->header.handle);
    memset(&p_entry->data[0], 0xAB, 100);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[1].raw[8];
    flash_execute();
    /* pad the old page: */
    FLASH_EXPECT(&area[0], area_offset, 0xff, 0xff, 0x00, 0x7f);
    /* start the entry on the next page: */
    FLASH_EXPECT(&area[1], 0, 26, 0x00, (handle & 0xFF), ((handle >> 8) & 0xFF),
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab,
            0xab);
    area_offset = 104;
    FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */

    /* add some arbitrary additional entry */
    handle = 0x1200;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1200, p_entry->header.handle);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[1].raw[area_offset + 8];
    flash_execute();
    FLASH_EXPECT(&area[1], area_offset, 0x03, 0x00, 0x00, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
    area_offset += 12;
    FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */

    /* replace one of the first entries */
    handle = 0x1234;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[1].raw[area_offset + 8];
    flash_execute();
    FLASH_EXPECT(&area[0], 8, 0x03, 0x00, 0x00, 0x00);
    FLASH_EXPECT(&area[1], area_offset, 0x03, 0x00, 0x34, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
    area_offset += 12;
    FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */


    /* Invalid params */
    g_flash_queue_slots = 0xFFFFFF;
    handle = 0xFF00;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_alloc(&manager, handle, 8));
    handle = 0x7F00;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_alloc(&manager, handle, 8));
    handle = 0x0000;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_alloc(&manager, handle, 8));
    handle = 0x0010;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_alloc(NULL, handle, 8));
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_alloc(&manager, handle, FLASH_MANAGER_ENTRY_MAX_SIZE+1));

    /* corner cases */
    handle = 0x0010;
    /* odd length, should pad with 1s */
    p_entry = flash_manager_entry_alloc(&manager, handle, 9);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(4, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x0010, p_entry->header.handle);
    uint8_t * data_bytes = (uint8_t *) p_entry->data;
    /* Add only nine bytes to data before flashing, to ensure that
    the last bytes of the last word are written as 1s by default */
    for (uint8_t i = 0; i < 9; ++i)
    {
        data_bytes[i] = i+1;
    }
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[1].raw[area_offset + 8];
    flash_execute();
    FLASH_EXPECT(&area[1], area_offset, 0x04, 0x00, 0x10, 0x00,
                0x01, 0x02, 0x03, 0x04,
                0x05, 0x06, 0x07, 0x08,
                0x09, 0xFF, 0xFF, 0xFF);
    area_offset += 16;
    FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */

    /* empty data */
    p_entry = flash_manager_entry_alloc(&manager, handle, 0);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(1, p_entry->header.len_words);
    flash_manager_entry_release(p_entry);
    /* max data */
    p_entry = flash_manager_entry_alloc(&manager, handle, FLASH_MANAGER_ENTRY_MAX_SIZE);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(FLASH_MANAGER_ENTRY_MAX_SIZE / 4 + 1, p_entry->header.len_words);
    flash_manager_entry_release(p_entry);

    /* Gather several actions before executing */
    g_delayed_execution = true;
    handle = 0x1234;
    for (uint32_t i = 0; i < 4; i++)
    {
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
        p_entry->data[0] = 0x01234567;
        p_entry->data[1] = 0x89abcdef;
        flash_manager_entry_commit(p_entry);
        TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    }
    manager.config.write_complete_cb = NULL; /* disable this, as our test can't handle multiple expects in a row */
    /* execute */
    g_process_cb();
    flash_execute();

    for (uint32_t i = 0; i < 3; i++)
    {
        FLASH_EXPECT(&area[1], area_offset,
                0x03, 0x00, 0x00, 0x00); /* invalid */
        area_offset += 12;
    }
    FLASH_EXPECT(&area[1], area_offset, 0x03, 0x00, 0x34, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
    area_offset += 12;
    FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */

    g_delayed_execution = false;

    /* re-enable the callback */
    manager.config.write_complete_cb = write_complete_callback;

    /* Fill up the rest of the page */
    while (area_offset + 12 < PAGE_SIZE - 8)
    {
        g_flash_queue_slots = 0xFFFFFF;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
        p_entry->data[0] = 0x01234567;
        p_entry->data[1] = 0x89abcdef;
        flash_manager_entry_commit(p_entry);
        gp_expected_entry = (const fm_entry_t *) &area[1].raw[area_offset + 8];
        flash_execute();
        FLASH_EXPECT(&area[1], area_offset - 12, 0x03, 0x00, 0x00, 0x00); /* previous is invalid */
        FLASH_EXPECT(&area[1], area_offset, 0x03, 0x00, 0x34, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
        area_offset += 12;
        FLASH_EXPECT(&area[1], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */
    }

    /* Put an entry that perfectly fills the rest of the page */
    uint32_t remaining = ((PAGE_SIZE - 8) - area_offset);
    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(remaining));
    p_entry = flash_manager_entry_alloc(&manager, handle, remaining - 4);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(remaining / WORD_SIZE, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    for (uint16_t i = 0; i < p_entry->header.len_words - 1; ++i)
    {
        p_entry->data[i] = 0x01234567;
    }
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = (const fm_entry_t *) &area[1].raw[area_offset + 8];
    flash_execute();
    FLASH_EXPECT(&area[1], area_offset - 12, 0x03, 0x00, 0x00, 0x00); /* previous is invalid */
    FLASH_EXPECT(&area[1], area_offset, remaining/WORD_SIZE, 0x00, 0x34, 0x12);
    for (uint16_t i = 1; i < p_entry->header.len_words; ++i)
    {
        area_offset += 4;
        FLASH_EXPECT(&area[1], area_offset, 0x67, 0x45, 0x23, 0x01);
    }
    area_offset = 0;
    FLASH_EXPECT(&area[2], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal on next page */

    /* Fill up the rest of the page */
    while (area_offset < PAGE_SIZE - 20)
    {
        g_flash_queue_slots = 0xFFFFFF;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
        p_entry->data[0] = 0x01234567;
        p_entry->data[1] = 0x89abcdef;
        flash_manager_entry_commit(p_entry);
        gp_expected_entry = (const fm_entry_t *) &area[2].raw[area_offset + 8];
        flash_execute();
        FLASH_EXPECT(&area[2], area_offset, 0x03, 0x00, 0x34, 0x12, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab, 0x89);
        area_offset += 12;
        FLASH_EXPECT(&area[2], area_offset, 0xff, 0xff, 0xff, 0x7f); /* seal */
    }

    /* Add an entry that won't fit. This should trigger defrag, regardless of
     * the number of available slots. */
    g_flash_queue_slots = 0;
    g_completes = 0;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(3, p_entry->header.len_words);
    TEST_ASSERT_EQUAL(0x1234, p_entry->header.handle);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_defrag_Expect(&manager);
    flash_manager_entry_commit(p_entry);
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));

    /* Emulate end of defrag, but don't move the seal. This should make the
     * ongoing action fail with "AREA_FULL", as it would exceed the limit
     * without having any invalid bytes to remove */
    g_flash_queue_slots = 0xFFFFFF;
    manager.internal.invalid_bytes = 0;
    g_expected_result = FM_RESULT_ERROR_AREA_FULL;
    gp_expected_entry = p_entry;
    flash_manager_on_defrag_end(&manager);
    TEST_ASSERT_EQUAL(1, g_completes); /* verify that the result callback was called. */

    /* Test uninitialized manager */
    memset(&manager, 0, sizeof(manager));
    TEST_ASSERT_NULL(flash_manager_entry_alloc(&manager, 0x0001, 4));
}

void test_invalidate(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    static flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 2, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 2,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = invalidate_complete_callback
    };
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();


    /* add an entry */
    fm_handle_t handle = 0x1234;
    fm_entry_t * p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    p_entry->data[0] = 0x55555555;
    p_entry->data[1] = 0xaaaaaaaa;
    flash_manager_entry_commit(p_entry);
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0x03, 0x00, 0x34, 0x12, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa,
            0xff, 0xff, 0xff, 0x7f);

    /* Invalidate the entry */
    g_expected_handle = handle;
    gp_active_manager = &manager;
    g_expected_result = FM_RESULT_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_invalidate(&manager, handle));
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0x03, 0x00, 0x00, 0x00);

    /* invalidate non-existing entry */
    g_expected_result = FM_RESULT_ERROR_NOT_FOUND;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_invalidate(&manager, handle));
    flash_execute();

    /* invalidate invalid handles */
    handle = 0xFF00;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_invalidate(&manager, handle));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    handle = 0x7F00;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_invalidate(&manager, handle));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    handle = 0x0000;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_invalidate(&manager, handle));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));

    /* add several entries */
    for (uint32_t i = 0; i < 8; i++)
    {
        handle  = 0x2000 + i;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        p_entry->data[0] = 0x55555555;
        p_entry->data[1] = 0xaaaaaaaa;
        flash_manager_entry_commit(p_entry);
        flash_execute();
    }
    /* invalidate every second entry */
    for (uint32_t i = 0; i < 8; i+= 2)
    {
        handle            = 0x2000 + i;
        g_expected_handle = handle;
        g_expected_result = FM_RESULT_SUCCESS;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_invalidate(&manager, handle));
        flash_execute();
        FLASH_EXPECT(&area[0], 12 * (i + 1), 0x03, 0x00, 0x00, 0x00);
    }
    /* Fill up the rest of the page */
    uint32_t area_offset = 12 * 8;
    while (area_offset < PAGE_SIZE - 32)
    {
        handle = 0x2080;
        p_entry = flash_manager_entry_alloc(&manager, handle, 8);
        TEST_ASSERT_NOT_NULL(p_entry);
        p_entry->data[0] = 0x55555555;
        p_entry->data[1] = 0xaaaaaaaa;
        flash_manager_entry_commit(p_entry);
        flash_execute();
        area_offset += 12;
        FLASH_EXPECT(&area[0], area_offset,
                0x03, 0x00, 0x80, 0x20, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa,
                0xff, 0xff, 0xff, 0x7f);
    }

    /* Add one more entry to push us over to the next page */
    handle = 0x2100;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    p_entry->data[0] = 0x55555555;
    p_entry->data[1] = 0xaaaaaaaa;
    flash_manager_entry_commit(p_entry);
    flash_execute();
    FLASH_EXPECT(&area[1], 0,
            0x03, 0x00, 0x00, 0x21, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa,
            0xff, 0xff, 0xff, 0x7f);

    /* invalidate the instance on the next page */
    g_expected_handle = handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_invalidate(&manager, handle));
    flash_execute();
    FLASH_EXPECT(&area[1], 0, 0x03, 0x00, 0x00, 0x00);

    /* invalid params */
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_invalidate(NULL, handle));
}

void test_getters(void)
{
    test_entry_t entries[] =
    {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0002, 0x02020202},
        {0x0001, 0x0003, 0x03030303}, /* smallest */
        {0x0080, 0x0004, 0x04040404}, /* larger than we're allowed to write */
        {0x0010, 0x00FF, 0x0f0f0f0f}, /* last valid individual entry */
        {0x0010, 0x0102, 0x22222222}, /* group entries */
        {0x0010, 0x0100, 0x00000000},
        {0x0010, 0x0101, 0x11111111},
        {0x0010, 0x0103, 0x33333333},
        {0x0010, 0x0104, 0x44444444},
        {0x0010, 0x0105, 0x55555555},
        {0x0010, 0x0188, 0x88888888},
        {0x0010, 0x0177, 0x77777777},
        {0x00BE, 0x0166, 0x66666666}, /* fill the rest of the page */
        {0x0008, 0x0201, 0x11111111}, /* page aligned final entry */
    };

    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 3,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };

    build_test_page(area, 3, entries, ARRAY_SIZE(entries), true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();

    /* get all the entries as individual entries */
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); i++)
    {
        const fm_entry_t * p_entry = flash_manager_entry_get(&manager, entries[i].handle);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(entries[i].len, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(entries[i].handle, p_entry->header.handle);
    }
    /* get all the entries as filtered entries, should return in the order they're flashed */
    const fm_entry_t * p_entry = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); i++)
    {
        fm_handle_filter_t filter = {.mask = 0xFF00, .match = entries[i].handle};
        p_entry = flash_manager_entry_next_get(&manager, &filter, p_entry);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(entries[i].len, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(entries[i].handle, p_entry->header.handle);
    }
    /* Get all entries without a filter, should match all in the order they're flashed */
    p_entry = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); i++)
    {
        p_entry = flash_manager_entry_next_get(&manager, NULL, p_entry);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(entries[i].len, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(entries[i].handle, p_entry->header.handle);
    }
    /* Get all entries with a match-all filter, should match all in the order they're flashed */
    p_entry = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); i++)
    {
        fm_handle_filter_t filter = {.mask = 0x0000, .match = 0xABCD};
        p_entry = flash_manager_entry_next_get(&manager, &filter, p_entry);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(entries[i].len, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(entries[i].handle, p_entry->header.handle);
    }
    /* Get all entries with a filter only matching top 4 bits, should match all in the order they're flashed */
    p_entry = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); i++)
    {
        fm_handle_filter_t filter = {.mask = 0xF000, .match = 0x0123};
        p_entry = flash_manager_entry_next_get(&manager, &filter, p_entry);
        TEST_ASSERT_NOT_NULL(p_entry);
        TEST_ASSERT_EQUAL(entries[i].len, p_entry->header.len_words);
        TEST_ASSERT_EQUAL(entries[i].handle, p_entry->header.handle);
    }

    /* Get first group 1 entry, starting from the beginning */
    fm_handle_filter_t filter = {.mask = 0xFF00, .match = 0x0100};
    p_entry = flash_manager_entry_next_get(&manager, &filter, NULL);
    TEST_ASSERT_NOT_NULL(p_entry);
    TEST_ASSERT_EQUAL(0x0102, p_entry->header.handle);

    flash_manager_recovery_area_t recovery;
    memset(&recovery, 0, sizeof(recovery));
    flash_manager_defrag_recovery_page_get_IgnoreAndReturn(&recovery);

    /* Check count */
    filter.match = 0x0000;
    TEST_ASSERT_EQUAL(5, flash_manager_entry_count_get(&manager, &filter));
    filter.match = 0x0100;
    TEST_ASSERT_EQUAL(9, flash_manager_entry_count_get(&manager, &filter));
    filter.match = 0x0200;
    TEST_ASSERT_EQUAL(1, flash_manager_entry_count_get(&manager, &filter));
    filter.match = 0x0300;
    TEST_ASSERT_EQUAL(0, flash_manager_entry_count_get(&manager, &filter));
    /* Match all 0xXX01 */
    filter.mask = 0x00FF;
    filter.match = 0x01;
    TEST_ASSERT_EQUAL(3, flash_manager_entry_count_get(&manager, &filter));
    filter.mask = 0x0000; // match all
    TEST_ASSERT_EQUAL(15, flash_manager_entry_count_get(&manager, &filter));
    TEST_ASSERT_EQUAL(15, flash_manager_entry_count_get(&manager, NULL)); // no filter, match all
    filter.mask = 0xFF00;

    /* invalid params */
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&manager, FLASH_MANAGER_HANDLE_INVALID));
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&manager, HANDLE_SEAL));
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&manager, HANDLE_BLANK));
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&manager, HANDLE_PADDING));
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_get(NULL, HANDLE_PADDING));

    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_next_get(NULL, &filter, NULL));
    filter.match = 0xFF00;
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_next_get(&manager, &filter, NULL));
    filter.match = 0x7F00;
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_next_get(&manager, &filter, NULL));
    const fm_entry_t out_of_bounds_entry = {};
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_next_get(&manager, &filter, &out_of_bounds_entry));

    filter.match = 0x0100;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_entry_count_get(NULL, &filter));
    filter.match = 0x7F00;
    TEST_ASSERT_EQUAL(0, flash_manager_entry_count_get(&manager, &filter));
    filter.match = 0xFF00;
    TEST_ASSERT_EQUAL(0, flash_manager_entry_count_get(&manager, &filter));
}

void test_defrag_safe_getters(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_recovery_area_t recovery_area __attribute__((aligned(PAGE_SIZE)));;
    memset(&recovery_area, 0xFF, sizeof(recovery_area));
    flash_manager_defrag_recovery_page_get_IgnoreAndReturn(&recovery_area);

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    flash_manager_t manager;
    const flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = ARRAY_SIZE(area),
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };

    /* Fill the area with all kinds of entries */
    test_entry_t entries[] =
    {
        {0x0010, 0x0001, 0x01010101}, /////////// Page 1
        {0x0010, 0x0002, 0x02020202},
        {0x0001, 0x0003, 0x03030303},
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0010, 0x00FF, 0x0f0f0f0f},
        {0x0010, 0x0102, 0x22222222},
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0008, 0x0100, 0x00000000},
        {0x0020, 0x0000, 0x11111111}, // invalid
        {0x0008, 0x0103, 0x33333333},
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0010, 0x0104, 0x44444444}, /////////// Page 2
        {0x0010, 0x0105, 0x55555555},
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x0020, 0x0000, 0x04040404}, // invalid
        {0x001A, 0x0000, 0x88888888}, // invalid
        {0x001C, 0x0199, 0x99999999},
        {0x0008, 0x0177, 0x77777777},
        {0x0010, 0x0000, 0x77777777},
        {0x0008, 0x0200, 0x22222222},
        {0x0020, 0x0201, 0x22222222},
        {0x001E, 0x0202, 0x20202020}, ////////// Page 3
        {0x0008, 0x0166, 0x66666666},
        {0x0020, 0x0000, 0xfefefefe}, // invalid
        {0x0011, 0x0167, 0x01670167},
        {0x0008, 0x0203, 0x11111111},
    };
    build_test_page(area, ARRAY_SIZE(area), entries, ARRAY_SIZE(entries), true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();

    // Build different kinds of recovery data to cover all recovery scenarios
    const struct {
        const flash_manager_page_t * p_page;
        test_entry_t * p_entries;
        uint32_t entry_count;
    } recovery_sets[] = {
        { // Blank recovery page p_page pointer, nothing else is considered
            .p_page = NULL, .entry_count = 0,
            .p_entries = (test_entry_t[]) {
                {0, 0, 0}
            }
        },
        { // all the valid entries in page 1
            .p_page = &area[0], .entry_count = 7,
            .p_entries = (test_entry_t[]) {
                {0x0010, 0x0001, 0x01010101}, /////////// Page 1
                {0x0010, 0x0002, 0x02020202},
                {0x0001, 0x0003, 0x03030303},
                {0x0010, 0x00FF, 0x0f0f0f0f},
                {0x0010, 0x0102, 0x22222222},
                {0x0008, 0x0100, 0x00000000},
                {0x0008, 0x0103, 0x33333333},
            }
        },
        { // all the valid entries in page 2
            .p_page = &area[1], .entry_count = 6,
            .p_entries = (test_entry_t[]) {
                {0x0010, 0x0104, 0x44444444},
                {0x0010, 0x0105, 0x55555555},
                {0x001C, 0x0199, 0x99999999},
                {0x0008, 0x0177, 0x77777777},
                {0x0008, 0x0200, 0x22222222},
                {0x0020, 0x0201, 0x22222222},
            }
        },
        { // all the valid entries in page 3
            .p_page = &area[2], .entry_count = 4,
            .p_entries = (test_entry_t[]) {
                {0x001E, 0x0202, 0x20202020}, ////////// Page 3
                {0x0008, 0x0166, 0x66666666},
                {0x0011, 0x0167, 0x01670167},
                {0x0008, 0x0203, 0x11111111},
            }
        },
        { // all valid entries in page 1 and 2 plus the first two from page 3
            .p_page = &area[0], .entry_count = 15,
            .p_entries = (test_entry_t[]) {
                {0x0010, 0x0001, 0x01010101}, /////////// Page 1
                {0x0010, 0x0002, 0x02020202},
                {0x0001, 0x0003, 0x03030303},
                {0x0010, 0x00FF, 0x0f0f0f0f},
                {0x0010, 0x0102, 0x22222222},
                {0x0008, 0x0100, 0x00000000},
                {0x0008, 0x0103, 0x33333333},
                {0x0010, 0x0104, 0x44444444}, /////////// Page 2
                {0x001C, 0x0199, 0x99999999},
                {0x0008, 0x0177, 0x77777777},
                {0x0008, 0x0200, 0x22222222},
                {0x0010, 0x0200, 0x22222222},
                {0x0020, 0x0201, 0x22222222},
                {0x001E, 0x0202, 0x20202020}, ////////// Page 3
                {0x0008, 0x0166, 0x66666666},
            }
        },
        { // all valid entries in page 2 and the first two from page 3
            .p_page = &area[1], .entry_count = 8,
            .p_entries = (test_entry_t[]) {
                {0x0010, 0x0104, 0x44444444},
                {0x0010, 0x0105, 0x55555555},
                {0x001C, 0x0199, 0x99999999},
                {0x0008, 0x0177, 0x77777777},
                {0x0008, 0x0200, 0x22222222},
                {0x0020, 0x0201, 0x22222222},
                {0x001E, 0x0202, 0x20202020}, ////////// Page 3
                {0x0008, 0x0166, 0x66666666},
            }
        },
    };

    for (uint32_t set = 0; set < ARRAY_SIZE(recovery_sets); ++set)
    {
        fflush(stdout);
        memset(&recovery_area, 0xff, sizeof(recovery_area));
        build_test_page((flash_manager_page_t *) recovery_area.data, 1, recovery_sets[set].p_entries, recovery_sets[set].entry_count, false);
        recovery_area.p_storage_page = recovery_sets[set].p_page;

        // Verify that regardless of the recovery page state, we'll get the same answer for all read operations:
        uint32_t buffer[FLASH_MANAGER_ENTRY_MAX_SIZE / sizeof(uint32_t)];
        for (uint32_t i = 0; i < ARRAY_SIZE(entries); ++i)
        {
            if (entries[i].handle != FLASH_MANAGER_HANDLE_INVALID)
            {
                memset(&buffer, 0, sizeof(buffer));

                uint32_t length = sizeof(buffer);
                TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, flash_manager_entry_read(&manager, entries[i].handle, &buffer, &length));
                TEST_ASSERT_EQUAL((entries[i].len - 1) * WORD_SIZE, length);

                length += 1; // just too long
                TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, flash_manager_entry_read(&manager, entries[i].handle, &buffer, &length));
                TEST_ASSERT_EQUAL((entries[i].len - 1) * WORD_SIZE, length);

                if (entries[i].len > 2)
                {
                    length -= WORD_SIZE + 1; // just too short
                    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, flash_manager_entry_read(&manager, entries[i].handle, &buffer, &length));
                    TEST_ASSERT_EQUAL((entries[i].len - 1) * WORD_SIZE, length);

                    // within the same word of the expected length
                    length -= WORD_SIZE - 1;
                    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_read(&manager, entries[i].handle, &buffer, &length));
                    TEST_ASSERT_EQUAL((entries[i].len - 1) * WORD_SIZE, length);
                }

                // the expected length
                TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_read(&manager, entries[i].handle, &buffer, &length));
                TEST_ASSERT_EQUAL((entries[i].len - 1) * WORD_SIZE, length);

                for (int word = 0; word < entries[i].len - 1; ++word)
                {
                    TEST_ASSERT_EQUAL_HEX32(entries[i].data_value, buffer[word]);
                }

                fm_handle_filter_t filter = {.mask = 0xffff, .match = entries[i].handle};

                // Even with duplicates between recovery page and area, each entry should just be visited once:
                TEST_ASSERT_EQUAL(1, flash_manager_entry_count_get(&manager, &filter));
                TEST_ASSERT_EQUAL(1, flash_manager_entries_read(&manager, &filter, read_cb, &entries[i]));
            }
        }
    }
}

/**
 * Test behavior for recovering from power failure during various stages of operation.
 */
void test_power_failure(void)
{
    test_entry_t entries[] =
    {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0002, 0x02020202},
        {0x0001, 0x0003, 0x03030303},
        {0x0080, 0x0004, 0x04040404},
        {0x0010, 0x00FF, 0x0f0f0f0f},
        {0x0010, 0x0102, 0x22222222},
        {0x0010, 0x0100, 0x00000000},
        {0x0010, 0x0101, 0x11111111},
        {0x0010, 0x0103, 0x33333333},
        {0x0010, 0x0104, 0x44444444},
        {0x0010, 0x0105, 0x55555555},
        {0x0010, 0x0188, 0x88888888},
        {0x0010, 0x0177, 0x77777777},
        {0x00BE, 0x0166, 0x66666666},
        {0x0008, 0x0201, 0x11111111}, /* Starts on page 3, makes offset calculation easy */
    };

    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 3,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };

    /* No seal present, should mark last entry invalid, and add a seal. */
    build_test_page(area, 3, entries, ARRAY_SIZE(entries), false);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[2], 0, 0x08, 0x00, 0x00, 0x00); /* invalidated last entry */
    FLASH_EXPECT(&area[2], 32, 0xff, 0xff, 0xff, 0x7f); /* added seal after last entry */

    /* erase the seal, so that the last entry is an invalid one */
    memset(&area[2].raw[40], 0xFF, 4);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[1], 256, 0xbe, 0x00, 0x66, 0x01); /* no changes to last _valid_ entry */
    FLASH_EXPECT(&area[2], 0, 0x08, 0x00, 0x00, 0x00); /* no changes to last entry */
    FLASH_EXPECT(&area[2], 32, 0xff, 0xff, 0xff, 0x7f); /* added seal after last entry */

    /* Make last entry one without data */
    memset(&area[2].raw[12], 0xFF, 40);
    ((fm_entry_t *) get_first_entry(&area[2]))->header.len_words = 0x01;
    ((fm_entry_t *) get_first_entry(&area[2]))->header.handle = 0x0001; /* also make it valid */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[2], 0, 0x01, 0x00, 0x00, 0x00); /* invalidated last entry */
    FLASH_EXPECT(&area[2], 4, 0xff, 0xff, 0xff, 0x7f); /* added seal after last entry */

    /* Make last entry fill a page completely, seal should end up on next page */
    memset(&area[2].raw[8], 0xFF, 8);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[1], 256, 0xbe, 0x00, 0x00, 0x00); /* invalidated last entry */
    FLASH_EXPECT(&area[2], 0, 0xff, 0xff, 0xff, 0x7f); /* added seal first thing on the next page */

    /* only data is the metadata */
    memset(area, 0xff, sizeof(area));
    build_test_page(area, 3, NULL, 0, false);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }
    FLASH_EXPECT(&area[0], 0, 0xff, 0xff, 0xff, 0xff); /* still blank */

    /* only first metadata is present */
    memset(&area[1], 0xff, 2 * PAGE_SIZE);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0xff, 0xff, 0xff, 0xff); /* still blank */
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }

    /* Only first word of first metadata is present */
    memset(&area[0].raw[4], 0xff, 4);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0xff, 0xff, 0xff, 0xff); /* still blank */
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }

    /* Nothing is present */
    memset(&area[0], 0xff, 3 * PAGE_SIZE);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    FLASH_EXPECT(&area[0], 0, 0xff, 0xff, 0xff, 0xff); /* still blank */
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }
}

/**
 * Check that the module detects illegal data in the area
 */
void test_invalid_area(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 2, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = area,
        .page_count = 2,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };

    /* add random data throughout entire area */
    for (uint32_t i = 0; i < PAGE_SIZE; i++)
    {
        area[0].raw[i] = i;
    }
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));

    /* Legal metadata, but the rest is garbage */
    build_test_page(area, 2, NULL, 0, false);
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    /* Legal metadata, the rest is 0 */
    memset(&area[0].raw[8], 0, PAGE_SIZE - 8);
    memset(&area[1].raw[8], 0, PAGE_SIZE - 8);
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    /* Legal metadata, the first entry is 0-length but valid */
    ((fm_entry_t *) &area[0].raw[8])->header.handle = 0x0001;
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_add(&manager, &config));
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
}

/**
 * While the module is in defrag state, there shouldn't be any other activity.
 */
void test_defrag_state(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    test_entry_t entries[] =
    {
        {0x0010, 0x0001, 0x01010101},
        {0x0020, 0x0000, 0xabababab}, /* invalid entry */
        {0x0010, 0x0002, 0x02020202},
        {0x0030, 0x0000, 0xabababab}, /* invalid entry */
        {0x0001, 0x0003, 0x03030303},
        {0x0080, 0x0004, 0x04040404},
        {0x0001, 0x0000, 0xabababab}, /* invalid entry */
    };
    /* the same entries, but without the invalid ones. */
    test_entry_t entries_no_invalid[] =
    {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0002, 0x02020202},
        {0x0001, 0x0003, 0x03030303},
        {0x0080, 0x0004, 0x04040404},
    };

    /* set up two identical managers: */
    static flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    flash_manager_t managers[2];
    flash_manager_config_t config[2] =
    {
        {
            .p_area = &area[0],
            .page_count = 1,
            .min_available_space = 0,
            .write_complete_cb = NULL,
            .invalidate_complete_cb = NULL
        },
        {
            .p_area = &area[1],
            .page_count = 1,
            .min_available_space = 0,
            .write_complete_cb = NULL,
            .invalidate_complete_cb = NULL
        }
    };

    build_test_page(&area[0], 1, entries, ARRAY_SIZE(entries), true);
    build_test_page(&area[1], 1, entries, ARRAY_SIZE(entries), true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&managers[0], &config[0]));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&managers[1], &config[1]));

    /* trigger defrag of the first manager by writing an entry that won't fit */
    fm_handle_t handle = 0x1234;
    fm_entry_t * p_entry = flash_manager_entry_alloc(&managers[0], handle, 60);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_defrag_Expect(&managers[0]);
    flash_manager_entry_commit(p_entry);
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    managers[0].internal.state = FM_STATE_DEFRAG;

    /* attempt to flash another entry in the same manager. Should accept the action, but not start it */
    handle = 0x1256;
    p_entry = flash_manager_entry_alloc(&managers[0], handle, 12);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));
    /* attempt to flash another entry in a different manager. Should accept the action, but not start it */
    p_entry = flash_manager_entry_alloc(&managers[1], handle, 12);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    TEST_ASSERT_TRUE(fifo_is_empty(&g_flash_operation_queue));

    /* Attempt to read from the area being defragged. Should return NULL. */
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&managers[0], 0x0001));
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_next_get(&managers[0], NULL, NULL));

    /* Attempt to read from the area being defragged. Should return NULL. */
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_get(&managers[0], 0x0001));
    TEST_ASSERT_EQUAL_PTR(NULL, flash_manager_entry_next_get(&managers[0], NULL, NULL));

    /* end the defrag, should trigger both the action that made this happen, as well as the other
     * two actions. */
    memset(&area[0], 0xFF, PAGE_SIZE);
    build_test_page(&area[0], 1, entries_no_invalid, ARRAY_SIZE(entries_no_invalid), true);
    flash_manager_on_defrag_end(&managers[0]);
    flash_execute();
    FLASH_EXPECT(&area[0], 0xA1*WORD_SIZE, 0x10, 0x00, 0x34, 0x12);
    FLASH_EXPECT(&area[0], 0xA1 * WORD_SIZE + 64, 0x04, 0x00, 0x56, 0x12);
    FLASH_EXPECT(&area[1], 0xF2 * WORD_SIZE, 0x04, 0x00, 0x56, 0x12);
}

void test_remove(void)
{
    /* Build simple area */
    test_entry_t entries[] = {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0002, 0x02020202},
        {0x0001, 0x0003, 0x03030303},
        {0x0080, 0x0004, 0x04040404},
        {0x0010, 0x00FF, 0x0f0f0f0f},
        {0x0010, 0x0102, 0x22222222},
        {0x0010, 0x0100, 0x00000000},
        {0x0010, 0x0101, 0x11111111},
        {0x0010, 0x0103, 0x33333333},
        {0x0010, 0x0104, 0x44444444},
        {0x0010, 0x0105, 0x55555555},
        {0x0010, 0x0188, 0x88888888},
        {0x0010, 0x0177, 0x77777777},
        {0x00BE, 0x0166, 0x66666666},
        {0x0008, 0x0201, 0x11111111},
    };

    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t        manager;
    flash_manager_config_t config = {.p_area                 = area,
                                     .page_count             = 3,
                                     .min_available_space    = 0,
                                     .write_complete_cb      = NULL,
                                     .invalidate_complete_cb = NULL,
                                     .remove_complete_cb     = remove_complete_callback};

    build_test_page(area, 3, entries, ARRAY_SIZE(entries), true);
    gp_active_manager = &manager;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    TEST_ASSERT_EQUAL(FM_STATE_READY, manager.internal.state);
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }

    /* remove */
    g_expected_remove_complete = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_remove(&manager));
    TEST_ASSERT_EQUAL(FM_STATE_REMOVING, manager.internal.state);
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);
    TEST_ASSERT_EQUAL(FM_STATE_UNINITIALIZED, manager.internal.state);
    for (uint32_t i = 0; i < PAGE_SIZE; ++i)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[0].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[1].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[2].raw[i]);
    }
    /* Wipe again, doesn't cause any problems. */
    g_expected_remove_complete = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_remove(&manager));
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);

    /* Add the same area again */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    TEST_ASSERT_EQUAL(FM_STATE_READY, manager.internal.state);
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }
    /* And remove it */
    g_expected_remove_complete = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_remove(&manager));
    TEST_ASSERT_EQUAL(FM_STATE_REMOVING, manager.internal.state);
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);
    TEST_ASSERT_EQUAL(FM_STATE_UNINITIALIZED, manager.internal.state);
    for (uint32_t i = 0; i < PAGE_SIZE; ++i)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[0].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[1].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[2].raw[i]);
    }

    /* Schedule add of area again */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));

    /* Attempt removing while building */
    g_expected_remove_complete = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, flash_manager_remove(&manager));
    TEST_ASSERT_EQUAL(FM_STATE_BUILDING, manager.internal.state);

    /* Finish adding */
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);
    TEST_ASSERT_EQUAL(FM_STATE_READY, manager.internal.state);
    for (uint32_t i = 0; i < 3; i++)
    {
        validate_metadata(&area[i].metadata, i, 3);
    }

    /* And remove it after adding is complete */
    g_expected_remove_complete = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_remove(&manager));
    TEST_ASSERT_EQUAL(FM_STATE_REMOVING, manager.internal.state);
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);
    TEST_ASSERT_EQUAL(FM_STATE_UNINITIALIZED, manager.internal.state);
    for (uint32_t i = 0; i < PAGE_SIZE; ++i)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[0].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[1].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[2].raw[i]);
    }


    /* Add a new area in the same flash-region */
    config.page_count = 2;
    config.p_area++;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();
    TEST_ASSERT_EQUAL(FM_STATE_READY, manager.internal.state);
    for (uint32_t i = 0; i < 2; i++)
    {
        validate_metadata(&area[i+1].metadata, i, 2);
    }
    /* And remove it */
    g_expected_remove_complete = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_remove(&manager));
    TEST_ASSERT_EQUAL(FM_STATE_REMOVING, manager.internal.state);
    flash_execute();
    TEST_ASSERT_EQUAL(0, g_expected_remove_complete);
    TEST_ASSERT_EQUAL(FM_STATE_UNINITIALIZED, manager.internal.state);
    for (uint32_t i = 0; i < PAGE_SIZE; ++i)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[0].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[1].raw[i]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, area[2].raw[i]);
    }
}

void test_flash_malfunction(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t        manager;
    flash_manager_config_t config = {.p_area                 = area,
                                     .page_count             = 3,
                                     .min_available_space    = 0,
                                     .write_complete_cb      = write_complete_callback,
                                     .invalidate_complete_cb = invalidate_complete_callback};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();

    /* fail writing */
    g_flash_malfunctions = true;
    gp_active_manager    = &manager;
    g_expected_result = FM_RESULT_ERROR_FLASH_MALFUNCTION;

    /* Replace and fail */
    fm_handle_t  handle  = 0x1234;
    fm_entry_t * p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = p_entry;
    flash_execute();

    /* Write it again, but successfully, so we can attempt to invalidate it afterwards */
    g_flash_malfunctions = false;
    g_expected_result    = FM_RESULT_SUCCESS;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    p_entry->data[0] = 0x01234567;
    p_entry->data[1] = 0x89abcdef;
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = get_first_entry(&area[0]);
    flash_execute();

    /* invalidate and fail */
    g_flash_malfunctions = true;
    g_expected_handle = handle;
    g_expected_result = FM_RESULT_ERROR_FLASH_MALFUNCTION;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_entry_invalidate(&manager, handle));
    flash_execute();

    /* handle is still there */
    TEST_ASSERT_EQUAL_PTR(get_first_entry(area), flash_manager_entry_get(&manager, handle));
}

void test_mem_listener(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    static flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 3, sizeof(area));
    flash_manager_t        manager;
    flash_manager_config_t config = {.p_area                 = area,
                                     .page_count             = 3,
                                     .min_available_space    = 0,
                                     .write_complete_cb      = NULL,
                                     .invalidate_complete_cb = NULL};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_execute();

    /* Register a memory listener */
    fm_mem_listener_t listener;
    memset(&listener, 0, sizeof(listener));
    listener.callback = mem_listener_cb;
    listener.p_args = &listener;
    m_expect_mem_listener = 1;
    flash_manager_mem_listener_register(&listener);

    g_expected_result    = FM_RESULT_SUCCESS;
    fm_handle_t  handle  = 0x1234;
    fm_entry_t * p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = get_first_entry(&area[0]);
    flash_execute();
    TEST_ASSERT_EQUAL(0, m_expect_mem_listener);

    /* register a ton of listeners */

    fm_mem_listener_t listeners[10];
    memset(listeners, 0, sizeof(listeners));
    for (uint32_t i = 0; i < 10; ++i)
    {
        listeners[i].callback = mem_listener_cb;
        listeners[i].p_args = &listeners[i];
        m_expect_mem_listener++;
        flash_manager_mem_listener_register(&listeners[i]);
    }

    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = get_first_entry(&area[0]);
    flash_execute();
    TEST_ASSERT_EQUAL(0, m_expect_mem_listener); /* They should all have been called once */

    /* re-register the listeners, and make them re-register in their respective callbacks. */
    m_recursive_listener = true;
    for (uint32_t i = 0; i < 10; ++i)
    {
        m_expect_mem_listener++;
        flash_manager_mem_listener_register(&listeners[i]);
    }

    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = get_first_entry(&area[0]);
    flash_execute();
    TEST_ASSERT_EQUAL(0, m_expect_mem_listener); /* They should all have been called ONCE */

    /* Fire the recursively added listeners */
    m_recursive_listener = false;
    m_expect_mem_listener = 10;
    p_entry = flash_manager_entry_alloc(&manager, handle, 8);
    TEST_ASSERT_NOT_NULL(p_entry);
    flash_manager_entry_commit(p_entry);
    gp_expected_entry = get_first_entry(&area[0]);
    flash_execute();
    TEST_ASSERT_EQUAL(0, m_expect_mem_listener); /* They should all have been called ONCE */
}

void test_queue_empty_cb(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    static flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    TEST_ASSERT_EQUAL(PAGE_SIZE * 2, sizeof(area));
    flash_manager_t manager;
    flash_manager_config_t config =
        {
            .p_area = area,
            .page_count = 2,
            .min_available_space = 0,
            .write_complete_cb = NULL,
            .invalidate_complete_cb = invalidate_complete_callback
        };
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));
    flash_manager_action_queue_empty_cb_set(queue_empty_cb);
    queue_empty_cb_Expect();
    flash_execute();
    queue_empty_cb_Verify();
}

void test_queue_empty_double_add(void)
{
    flash_manager_action_queue_empty_cb_set(queue_empty_cb);
    flash_manager_action_queue_empty_cb_set(NULL);
    flash_manager_action_queue_empty_cb_set(queue_empty_cb);
    TEST_NRF_MESH_ASSERT_EXPECT(flash_manager_action_queue_empty_cb_set(queue_empty_cb));
}

void test_defragmentation_invalidated_page(void)
{
    flash_manager_defrag_init_ExpectAndReturn(false);
    flash_manager_init();
    g_flash_queue_slots = 0xFFFFFF;

    flash_manager_defragging_IgnoreAndReturn(false);

    static flash_manager_page_t area __attribute__((aligned(PAGE_SIZE)));
    flash_manager_t manager;
    flash_manager_config_t config =
    {
        .p_area = &area,
        .page_count = 1,
        .min_available_space = 0,
        .write_complete_cb = NULL,
        .invalidate_complete_cb = NULL
    };
    memset(&area, 0xFF, PAGE_SIZE);
    build_test_page(&area, 1, NULL, 0, false);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, flash_manager_add(&manager, &config));

    queue_empty_cb_Expect();
    flash_manager_on_defrag_end(&manager);

    TEST_ASSERT_EQUAL(manager.internal.p_seal, &area.raw[sizeof(flash_manager_metadata_t)]);
    TEST_ASSERT_EACH_EQUAL_UINT8(0xFF, &area.raw[sizeof(flash_manager_metadata_t)], PAGE_SIZE - sizeof(flash_manager_metadata_t));
}
