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
#include <stdlib.h>

#include "flash_manager_defrag.h"
#include "flash_manager_mock.h"
#include "flash_manager_internal.h"
#include "flash_manager_test_util.h"

#include "utils.h"

#define DEFAULT_MANAGER(AREA, PAGE_COUNT)       \
    {                                           \
        .internal = {.state = FM_STATE_DEFRAG}, \
        .config =                               \
        {.p_area = AREA,                        \
         .page_count = PAGE_COUNT,              \
         .min_available_space = 0,              \
         .write_complete_cb = NULL,             \
         .invalidate_complete_cb = NULL }       \
    }

/* Initialize the UICR and FICR peripherals, it will be externed by the headers. */
NRF_UICR_Type * NRF_UICR;
NRF_FICR_Type * NRF_FICR;
static NRF_UICR_Type m_uicr;
static NRF_FICR_Type m_ficr;
static flash_manager_recovery_area_t * mp_recovery_area;
static flash_manager_page_t * mp_page_to_recover;

#if defined(NRF51)
static uint8_t m_buffer[3*PAGE_SIZE];
#elif defined(NRF52) || defined(NRF52_SERIES)
/* On nRF52 we need an extra page for the SoftDevice MBRPARAM page. */
static uint8_t m_buffer[4*PAGE_SIZE];
#endif

static flash_manager_t * mp_on_defrag_end_expected_manager;

/* Externs that are instantiated in the flash_manager.c */
const fm_header_t INVALID_HEADER = {0xFFFF, FLASH_MANAGER_HANDLE_INVALID};
const fm_header_t PADDING_HEADER = {0xFFFF, HANDLE_PADDING};
const fm_header_t SEAL_HEADER    = {0xFFFF, HANDLE_SEAL};

/* Externs which are meant only for unit testing */
void flash_manager_defrag_reset(void);

void assert_handler(uint32_t pc)
{
    char message[50];
    sprintf(message, "ASSERT, PC: 0x%08X", pc);
    TEST_FAIL_MESSAGE(message);
}

void setUp(void)
{
    /* Allocate a PAGE_SIZE aligned address to the recovery page and area. */
    mp_page_to_recover = (flash_manager_page_t *) (((uint32_t)m_buffer / PAGE_SIZE + 1) * PAGE_SIZE);
    mp_recovery_area = (flash_manager_recovery_area_t *) (((uint32_t)m_buffer / PAGE_SIZE + 2) * PAGE_SIZE);
    mp_page_to_recover->metadata.metadata_len = sizeof(flash_manager_metadata_t);
    mp_page_to_recover->metadata.pages_in_area = 1;
    mp_page_to_recover->metadata.page_index = 0;

    NRF_UICR = &m_uicr;
    NRF_FICR = &m_ficr;
    #if NRF51
    BOOTLOADERADDR() = (uint32_t) (mp_recovery_area + 1);
    #else
    /* 52 needs an extra page for softdevice MBR dfu-ing. */
    BOOTLOADERADDR() = (uint32_t) (mp_recovery_area + 2);
    #endif

    NRF_FICR->CODESIZE = BOOTLOADERADDR() / PAGE_SIZE;    /*lint !e123 Usage of symbol declared as function-like macro elsewhere */
    NRF_FICR->CODEPAGESIZE = PAGE_SIZE;
    memset((uint8_t *)mp_recovery_area, 0, sizeof(flash_manager_recovery_area_t));
    flash_manager_defrag_reset();
    flash_manager_test_util_setup();
    flash_manager_mock_Init();
}

void tearDown(void)
{
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
}

/**
 * Setup two test areas, @p p_area being the original, @p p_result_area being the expected outcome
 * after the defrag procedure has finished.
 */
static void setup_test_areas(flash_manager_page_t * p_area,
                             flash_manager_page_t * p_result_area,
                             uint32_t               page_count,
                             uint32_t               entry_count,
                             uint32_t               invalid_entry_interval)
{
    memset(p_result_area, 0xFF, PAGE_SIZE * page_count);
    memset(p_area, 0xFF, PAGE_SIZE * page_count);
    if (entry_count == 0)
    {
        build_test_page(p_result_area, page_count, NULL, 0, true);
        build_test_page(p_area, page_count, NULL, 0, true);
    }
    else
    {
        /* Build an arbitrary set of entries */
        test_entry_t * p_entries           = malloc(sizeof(test_entry_t) * entry_count);
        TEST_ASSERT_NOT_NULL(p_entries);
        test_entry_t * p_resulting_entries = malloc(sizeof(test_entry_t) * entry_count);
        TEST_ASSERT_NOT_NULL(p_resulting_entries);
        uint32_t     result_index = 0;
        for (uint32_t i = 0; i < entry_count; ++i)
        {
            p_entries[i].data_value = ((i + 1) << 24) | ((i + 1) << 16) | ((i + 1) << 8) | (i + 1);
            p_entries[i].len        = WORD_SIZE;
            p_entries[i].handle     = i + 1;
            /* make some entries invalid: */
            if (invalid_entry_interval > 0 && ((i % invalid_entry_interval) == (invalid_entry_interval - 1)))
            {
                p_entries[i].handle = FLASH_MANAGER_HANDLE_INVALID;
            }
            else
            {
                memcpy(&p_resulting_entries[result_index++], &p_entries[i], sizeof(test_entry_t));
            }
        }
        build_test_page(p_result_area, page_count, p_resulting_entries, result_index, true);
        build_test_page(p_area, page_count, p_entries, entry_count, true);

        free(p_entries);
        free(p_resulting_entries);
    }
}

void flash_manager_on_defrag_end(flash_manager_t * p_manager)
{
    TEST_ASSERT_EQUAL_PTR(mp_on_defrag_end_expected_manager, p_manager);
    mp_on_defrag_end_expected_manager = NULL; /* reset it to ensure correct number of ends */
}
/*****************************************************************************
* Tests
*****************************************************************************/

void test_init(void)
{
    flash_manager_t fmanager;
    /* Set storage page to null */
    mp_recovery_area->p_storage_page = 0;
    fmanager.config.p_area = mp_recovery_area->p_storage_page;
    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    TEST_ASSERT_FALSE(flash_manager_defragging(&fmanager));
    /* Set storage page to erased state */
    mp_recovery_area->p_storage_page = (flash_manager_page_t *) 0xFFFFFFFF;
    fmanager.config.p_area = mp_recovery_area->p_storage_page;
    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    TEST_ASSERT_FALSE(flash_manager_defragging(&fmanager));
    /* Set storage page to an unaligned address */
    mp_recovery_area->p_storage_page = (flash_manager_page_t *) ((uint8_t *) mp_page_to_recover + 1);
    fmanager.config.p_area = mp_recovery_area->p_storage_page;
    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    TEST_ASSERT_FALSE(flash_manager_defragging(&fmanager));
}

void test_single_page(void)
{
    test_entry_t entries[] = {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0000, 0x02020202}, /* invalid */
        {0x0001, 0x0003, 0x03030303}, /* smallest */
        {0x0010, 0x00FF, 0x0f0f0f0f},
        {0x0010, 0x0102, 0x22222222},
        {0x0010, 0x0100, 0x00000000},
        {0x0010, 0x0101, 0x11111111},
        {0x0010, 0x0000, 0x33333333}, /* invalid */
        {0x0010, 0x0000, 0x44444444}, /* invalid */
        {0x0010, 0x0105, 0x55555555},
        {0x0010, 0x0000, 0x88888888}, /* invalid */
        {0x0010, 0x0177, 0x77777777}
    };


    /* Build a table of the same entries, but without all the invalid ones: */
    test_entry_t resulting_entries[ARRAY_SIZE(entries)];
    uint32_t     result_index = 0;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); ++i)
    {
        if (entries[i].handle != FLASH_MANAGER_HANDLE_INVALID)
        {
            memcpy(&resulting_entries[result_index], &entries[i], sizeof(test_entry_t));
            result_index++;
        }
    }
    flash_manager_page_t expected_result __attribute__((aligned((PAGE_SIZE))));
    memset(expected_result.raw, 0xFF, PAGE_SIZE);
    build_test_page(&expected_result, 1, resulting_entries, result_index, true);

    flash_manager_page_t area[1] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    build_test_page(area, 1, entries, ARRAY_SIZE(entries), true);

    flash_manager_t manager = DEFAULT_MANAGER(area, 1);

    mp_recovery_area->p_storage_page = 0;
    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    TEST_ASSERT_FALSE(flash_manager_defragging(&manager));
    TEST_ASSERT_EQUAL_PTR(mp_recovery_area, flash_manager_defrag_recovery_page_get());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();
    /* Check that the area now contains all the same entries, but without invalid entries */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result.raw, area[0].raw, PAGE_SIZE);
}

/**
 * Test case where we go from having flash entries on two pages to having them fit on a single page.
 */
void test_two_pages_to_single_page(void)
{
    flash_manager_page_t expected_result[2] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 2, PAGE_SIZE / WORD_SIZE / WORD_SIZE, 4);
    flash_manager_t      manager = DEFAULT_MANAGER(area, 2);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

/** Entries go from spanning over 3 pages to spanning a single page */
void test_three_pages_to_single_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 3, (PAGE_SIZE / WORD_SIZE / WORD_SIZE) - 5, 2);
    /* Add a couple of extra entries at the end to push us over the 3-page line, but make one
       invalid, so we can get below 1 page after defrag: */
    test_entry_t final_entries[] = {{.handle = FLASH_MANAGER_HANDLE_INVALID, .len = 8, .data_value = 0xabababab},
                                    {.handle = 0x1234, .len = 4, .data_value = 0xabababab}};
    for (uint32_t i = 0; i < ARRAY_SIZE(final_entries); ++i)
    {
        test_page_add_entry(area, &final_entries[i], true);
        if (final_entries[i].handle != FLASH_MANAGER_HANDLE_INVALID)
        {
            test_page_add_entry(expected_result, &final_entries[i], true);
        }
    }

    flash_manager_t manager = DEFAULT_MANAGER(area, 3);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();
    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

/** Attempt to defrag a page that's completely full. */
void test_full_page(void)
{
    flash_manager_page_t expected_result[1] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[1] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 1, (PAGE_SIZE / WORD_SIZE/ WORD_SIZE) - 2, 0);
    /* Add a last entry manually to perfectly fill out the page:
       Note that there must be room for a seal. */
    test_entry_t last_entry = {.handle = 0x1234, .len = 5, .data_value = 0xabababab};
    test_page_add_entry(area, &last_entry, true);
    test_page_add_entry(expected_result, &last_entry, true);

    flash_manager_t manager = DEFAULT_MANAGER(area, 1);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    //flash_execute(); There should be no flashing, as the page should be ignored
    flash_manager_mock_Verify();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
}

/** Second page is full (without invalid entries), while first and third isn't */
void test_full_second_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 3, (PAGE_SIZE / WORD_SIZE/ WORD_SIZE) - 2, 4);
    /* Add an entry manually to perfectly fill out the first page:
       Don't need to make room for a seal */
    test_entry_t last_entry = {.handle = 0x1234, .len = 4, .data_value = 0xabababab};
    test_page_add_entry(area, &last_entry, true);
    test_page_add_entry(expected_result, &last_entry, true);
    /* Fill the second page completely with valid entries: */
    for (uint32_t i = 0; i < 62; ++i)
    {
        test_entry_t test_entry = {.handle = 0x1201 + i, .len = 4, .data_value = 0xabababab};
        test_page_add_entry(area, &test_entry, true);
        test_page_add_entry(expected_result, &test_entry, true);
    }
    /* Perfectly fill out the rest of the second page.
       Don't need to make room for a seal */
    last_entry.handle++;
    test_page_add_entry(area, &last_entry, true);
    test_page_add_entry(expected_result, &last_entry, true);
    /* put some entries on the third page, both valid and invalid: */
    for (uint32_t i = 0; i < 24; ++i)
    {
        if (i % 4 == 3)
        {
            test_entry_t test_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
        }
        else
        {
            test_entry_t test_entry = {.handle     = 0x3401 + i,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
            test_page_add_entry(expected_result, &test_entry, true);
        }
    }

    flash_manager_t manager = DEFAULT_MANAGER(area, 1);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

/** There's an edge case where if the remaining entries after a defrag end up fitting a page
 * perfectly, the algorithm does a pad on the last entry, instead of inserting it. This is
 * valid, but suboptimal behavior, and is caused by the reduced space in the recovery page.
 * It will never cause overflow, however, as the last page must have a seal, preventing this
 * edge case from occurring. This test triggers this edge case.
 */
void test_perfect_fit(void)
{
    flash_manager_page_t expected_result[2] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 2, PAGE_SIZE / WORD_SIZE / WORD_SIZE - 2, 0);
    /* We need at least one invalid entry in the page to keep the algorithm from ignoring it: */
    test_entry_t invalid_entry = {.handle = FLASH_MANAGER_HANDLE_INVALID, .len = 4, .data_value = 0xdeadbeef};
    test_page_add_entry(area, &invalid_entry, true);
    /* Add an entry manually that will perfectly fill out the first page after the defrag: */
    test_entry_t last_entry = {.handle = 0x1234, .len = 6, .data_value = 0xabababab};
    test_page_add_entry(area, &last_entry, true);
    /* in the resulting area, this entry will end up on the next page, even though it could fit on
       the first. Construct padding in front of it. */
    fm_entry_t * p_padding = (fm_entry_t *) entry_get(get_first_entry(expected_result),
                                                      &expected_result[1],
                                                      HANDLE_SEAL);
    p_padding->header.handle = HANDLE_PADDING;
    fm_entry_t * p_seal      = (fm_entry_t *) get_first_entry(&expected_result[1]);
    p_seal->header.handle    = HANDLE_SEAL;
    /* now add it at the start of the second page */
    test_page_add_entry(expected_result, &last_entry, true);


    flash_manager_t manager = DEFAULT_MANAGER(area, 1);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

/** All entries are invalid */
void test_all_invalid(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));

    memset(expected_result, 0xFF, PAGE_SIZE * 3);
    memset(area, 0xFF, PAGE_SIZE * 3);
    build_test_page(area, 3, NULL, 0, true);
    build_test_page(expected_result, 3, NULL, 0, false); /* no seal in an empty page */
    /* Add a bunch of arbitrary length invalid handles: */
    for (uint32_t i = 0; i < 128; ++i)
    {
        test_entry_t invalid_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                      .len        = 3 + (i % 6),
                                      .data_value = 0xabababab};
        test_page_add_entry(area, &invalid_entry, true);
    }
    flash_manager_t manager = DEFAULT_MANAGER(area, 3);


    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

/** First page only has invalid entries, but the second page has some valid ones. */
void test_invalid_first_page(void)
{
    flash_manager_page_t expected_result[2] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));

    memset(expected_result, 0xFF, PAGE_SIZE * 2);
    memset(area, 0xFF, PAGE_SIZE * 2);
    build_test_page(area, 2, NULL, 0, true);
    build_test_page(expected_result, 2, NULL, 0, true);
    /* Add a bunch of invalid handles to the first page: */
    test_entry_t invalid_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                  .len        = 4,
                                  .data_value = 0xabababab};
    for (uint32_t i = 0; i < 64; ++i)
    {
        test_page_add_entry(area, &invalid_entry, true);
    }

    /* put some entries on the second page, both valid and invalid: */
    for (uint32_t i = 0; i < 24; ++i)
    {
        if (i % 4 == 3)
        {
            test_entry_t test_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
        }
        else
        {
            test_entry_t test_entry = {.handle     = 0x3401 + i,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
            test_page_add_entry(expected_result, &test_entry, true);
        }
    }
    flash_manager_t manager = DEFAULT_MANAGER(area, 2);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

/** Second page is full of invalid entries, but the first and third page isn't. */
void test_invalid_second_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 3, (PAGE_SIZE / WORD_SIZE / WORD_SIZE) - 1, 4);

    /* Add a bunch of invalid handles to the second page: */
    test_entry_t invalid_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                  .len        = 4,
                                  .data_value = 0xabababab};
    for (uint32_t i = 0; i < 64; ++i)
    {
        test_page_add_entry(area, &invalid_entry, true);
    }

    /* put some entries on the third page, both valid and invalid: */
    for (uint32_t i = 0; i < 24; ++i)
    {
        if (i % 4 == 3)
        {
            test_entry_t test_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
        }
        else
        {
            test_entry_t test_entry = {.handle     = 0x3401 + i,
                                       .len        = 3 + (i % 5),
                                       .data_value = 0xabababab};
            test_page_add_entry(area, &test_entry, true);
            test_page_add_entry(expected_result, &test_entry, true);
        }
    }

    flash_manager_t manager = DEFAULT_MANAGER(area, 3);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

/** Last page is full of invalid entries, but the first two aren't. */
void test_invalid_last_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 3, (PAGE_SIZE / WORD_SIZE / WORD_SIZE) - 2, 4);

    /* Add a bunch of invalid handles to the third page: */
    test_entry_t invalid_entry = {.handle     = FLASH_MANAGER_HANDLE_INVALID,
                                  .len        = 4,
                                  .data_value = 0xabababab};
    for (uint32_t i = 0; i < 50; ++i)
    {
        test_page_add_entry(area, &invalid_entry, true);
    }

    flash_manager_t manager = DEFAULT_MANAGER(area, 3);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    g_flash_queue_slots = 0xFFFFFF;
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    flash_execute();

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

/** Give the module an arbitrary, low number of available flash operations, until it runs to completion */
void test_resource_constrained(void)
{
    flash_manager_page_t expected_result[2] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 2, 78, 4);
    flash_manager_t manager = DEFAULT_MANAGER(area, 2);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    mp_on_defrag_end_expected_manager = &manager;

    g_flash_queue_slots = 0;
    flash_manager_defrag(&manager);
    for (uint32_t i = 0; i < 1000; ++i)
    {
        g_flash_queue_slots = i % 4;
        /* emulate the end of all flash operations, to give the module a chance to continue execution: */
        flash_operation_t all_op;
        all_op.type = FLASH_OP_TYPE_ALL;
        g_flash_cb(MESH_FLASH_USER_MESH, &all_op, 0);

        flash_execute();
    }

    /* Check that the area now contains all the same entries, but without the invalid ones */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
}

void test_ignore_incomplete_recovery_area_on_bootup(void)
{
    flash_manager_page_t expected_result[2] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[2] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 2, 78, 4);

    /* Put some part of the first page in the recovery area, to emulate an incomplete write-back */
    memcpy(mp_recovery_area->data, area, 512);
    /* The first word in the recovery area should be the determining factor.
       If it points to something, it should be handled. Setting it to 0xFFFFFFF or NULL shouldn't
       trigger the procedure */
    mp_recovery_area->p_storage_page = (void*) BLANK_FLASH_WORD;

    TEST_ASSERT_FALSE(flash_manager_defrag_init());

    mp_recovery_area->p_storage_page = NULL;

    TEST_ASSERT_FALSE(flash_manager_defrag_init());
}

void test_recover_defrag_single_page(void)
{
    flash_manager_page_t expected_result[1] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[1] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 1, 50, 4);

    /* Put full expected area into recovery area */
    memcpy(mp_recovery_area->data, expected_result, sizeof(mp_recovery_area->data));
    /* Set the first word of the recovery area to point to the page we're backing up. This should
       trigger defrag to continue recovery. */
    mp_recovery_area->p_storage_page = &area[0];

    g_flash_queue_slots = 0xFFFFFFFF;
    mp_on_defrag_end_expected_manager = NULL;
    TEST_ASSERT_TRUE(flash_manager_defrag_init());
    flash_execute();

    /* Check that the area now has been backed up */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
}

void test_recover_defrag_first_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    setup_test_areas(area, expected_result, 3, 150, 4);

    /* Put full expected area into recovery area */
    memcpy(mp_recovery_area->data, expected_result, sizeof(mp_recovery_area->data));
    /* Set the first word of the recovery area to point to the page we're backing up. This should
       trigger defrag to continue recovery. */
    mp_recovery_area->p_storage_page = &area[0];

    g_flash_queue_slots = 0xFFFFFFFF;
    mp_on_defrag_end_expected_manager = NULL;
    TEST_ASSERT_TRUE(flash_manager_defrag_init());
    flash_execute();

    /* Check that the entire area now has been backed up */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

void test_recover_defrag_middle_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(expected_result, 0xFF, PAGE_SIZE * 3);
    memset(area, 0xFF, PAGE_SIZE * 3);

    test_entry_t test_entries[170];
    test_entry_t expected_entries[170];

    /* Build test area with all valid handles on the first page, then every 4th handle being
       invalid on the second and third. We keep the first page all-valid, as this would be the
       case if we get power failure in the middle of a defrag. */
    uint32_t expected_index = 0;
    for (uint32_t i = 0; i < 170; ++i)
    {
        test_entries[i].data_value = 0xabababab;
        test_entries[i].len        = 16 / WORD_SIZE;

        bool is_on_first_page = (i < ((PAGE_SIZE - sizeof(flash_manager_metadata_t)) / 16));

        if ((i % 4 == 3) && !is_on_first_page)
        {
            test_entries[i].handle = FLASH_MANAGER_HANDLE_INVALID;
        }
        else
        {
            test_entries[i].handle     = i;
            /* only expecting valid handles in result: */
            memcpy(&expected_entries[expected_index++], &test_entries[i], sizeof(test_entry_t));
        }
    }
    build_test_page(area, 3, test_entries, 170, true);
    build_test_page(expected_result, 3, expected_entries, expected_index, true);

    /* Put expected area into recovery area */
    memcpy(mp_recovery_area->data, &expected_result[1], sizeof(mp_recovery_area->data));
    /* Set the first word of the recovery area to point to the page we're backing up. This should
       trigger defrag to continue recovery. */
    mp_recovery_area->p_storage_page = &area[1];

    g_flash_queue_slots = 0xFFFFFFFF;
    mp_on_defrag_end_expected_manager = NULL;
    TEST_ASSERT_TRUE(flash_manager_defrag_init());
    flash_execute();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

void test_recover_defrag_last_page(void)
{
    flash_manager_page_t expected_result[3] __attribute__((aligned((PAGE_SIZE))));
    flash_manager_page_t area[3] __attribute__((aligned(PAGE_SIZE)));
    memset(expected_result, 0xFF, PAGE_SIZE * 3);
    memset(area, 0xFF, PAGE_SIZE * 3);

    test_entry_t test_entries[170];
    test_entry_t expected_entries[170];

    /* Build test area with all valid handles on the first page, then every 4th handle being
       invalid on the second and third. We keep the first page all-valid, as this would be the
       case if we get power failure in the middle of a defrag. */
    uint32_t expected_index = 0;
    for (uint32_t i = 0; i < 170; ++i)
    {
        test_entries[i].data_value = 0xabababab;
        test_entries[i].len        = 16 / WORD_SIZE;

        bool is_on_last_page = (i > (2 * (PAGE_SIZE - sizeof(flash_manager_metadata_t)) / 16));

        if ((i % 4 == 3) && is_on_last_page)
        {
            test_entries[i].handle = FLASH_MANAGER_HANDLE_INVALID;
        }
        else
        {
            test_entries[i].handle = i;
            /* only expecting valid handles in result: */
            memcpy(&expected_entries[expected_index++], &test_entries[i], sizeof(test_entry_t));
        }
    }
    build_test_page(area, 3, test_entries, 170, true);
    build_test_page(expected_result, 3, expected_entries, expected_index, true);

    /* Put expected area into recovery area */
    memcpy(mp_recovery_area->data, &expected_result[2], sizeof(mp_recovery_area->data));
    /* Set the first word of the recovery area to point to the page we're backing up. This should
       trigger defrag to continue recovery. */
    mp_recovery_area->p_storage_page = &area[2];

    g_flash_queue_slots = 0xFFFFFFFF;
    mp_on_defrag_end_expected_manager = NULL;
    TEST_ASSERT_TRUE(flash_manager_defrag_init());
    flash_execute();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[0].raw, area[0].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[1].raw, area[1].raw, PAGE_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_result[2].raw, area[2].raw, PAGE_SIZE);
}

void test_defragmentation_freezing(void)
{
    test_entry_t entries[] = {
        {0x0010, 0x0001, 0x01010101},
        {0x0010, 0x0000, 0x02020202}, /* invalid */
        {0x0001, 0x0003, 0x03030303}, /* smallest */
        {0x0010, 0x00FF, 0x0f0f0f0f},
        {0x0010, 0x0102, 0x22222222},
        {0x0010, 0x0100, 0x00000000},
        {0x0010, 0x0101, 0x11111111},
        {0x0010, 0x0000, 0x33333333}, /* invalid */
        {0x0010, 0x0000, 0x44444444}, /* invalid */
        {0x0010, 0x0105, 0x55555555},
        {0x0010, 0x0000, 0x88888888}, /* invalid */
        {0x0010, 0x0177, 0x77777777}
    };


    /* Build a table of the same entries, but without all the invalid ones: */
    test_entry_t resulting_entries[ARRAY_SIZE(entries)];
    uint32_t     result_index = 0;
    for (uint32_t i = 0; i < ARRAY_SIZE(entries); ++i)
    {
        if (entries[i].handle != FLASH_MANAGER_HANDLE_INVALID)
        {
            memcpy(&resulting_entries[result_index], &entries[i], sizeof(test_entry_t));
            result_index++;
        }
    }
    flash_manager_page_t expected_result __attribute__((aligned((PAGE_SIZE))));
    memset(expected_result.raw, 0xFF, PAGE_SIZE);
    build_test_page(&expected_result, 1, resulting_entries, result_index, true);

    flash_manager_page_t area[1] __attribute__((aligned(PAGE_SIZE)));
    memset(area, 0xFF, sizeof(area));
    build_test_page(area, 1, entries, ARRAY_SIZE(entries), true);

    flash_manager_t manager = DEFAULT_MANAGER(area, 1);

    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    mp_on_defrag_end_expected_manager = &manager;
    flash_manager_defrag(&manager);
    TEST_ASSERT_TRUE(flash_manager_defrag_is_running());
    flash_manager_defrag_reset();
    TEST_ASSERT_FALSE(flash_manager_defrag_init());
    mp_on_defrag_end_expected_manager = NULL;
    flash_manager_defrag_freeze();
    flash_manager_defrag(&manager);
    TEST_ASSERT_FALSE(flash_manager_defrag_is_running());
}
