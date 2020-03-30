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

#ifndef FLASH_MANGER_TEST_UTIL_H__
#define FLASH_MANGER_TEST_UTIL_H__

#include <stdbool.h>
#include <stdint.h>

#include "flash_manager.h"
#include "flash_manager_internal.h"
#include "bearer_event.h"
#include "mesh_flash.h"
#include "fifo.h"


#define _FLASH_EXPECT(START_ADDR, ARRAY)                                          \
    do                                                                            \
    {                                                                             \
        uint8_t * arr = (uint8_t *) ARRAY;                                        \
        TEST_ASSERT_EQUAL_HEX8_ARRAY(arr, (uint8_t *) START_ADDR, sizeof(ARRAY)); \
    } while (0)

/** convenience macro for checking flash entry contents. */
#define FLASH_EXPECT(AREA, OFFSET, ...) \
    _FLASH_EXPECT(&((uint8_t *) (AREA))[(OFFSET) + 8], ((uint8_t[]){__VA_ARGS__}))

typedef struct
{
    uint16_t    len; /**< Entry length in words */
    fm_handle_t handle;
    uint32_t    data_value; /**< Will be repeated for all words in the data */
} test_entry_t;

extern uint32_t                     g_flash_push_return;
extern uint32_t                     g_flash_queue_slots;
extern uint32_t                     g_flash_token;
extern uint32_t                     g_callback_token;
extern bearer_event_flag_callback_t g_process_cb;
extern mesh_flash_op_cb_t           g_flash_cb;
extern bool                         g_delayed_execution;
extern flash_manager_t *            gp_active_manager;
extern fm_handle_t                  g_expected_handle;
extern const fm_entry_t *           gp_expected_entry;
extern fm_result_t                  g_expected_result;
extern uint32_t                     g_completes;
extern uint32_t                     g_expected_remove_complete;
extern fifo_t                       g_flash_operation_queue;
extern bool                         g_flash_malfunctions;

void flash_manager_test_util_setup(void);

void print_page(const flash_manager_page_t * p_page);

void validate_metadata(const flash_manager_metadata_t * p_metadata,
                       uint8_t                          page_index,
                       uint8_t                          page_count);

void validate_blank_flash(void * p_start, uint32_t words);

void build_test_page(flash_manager_page_t * p_area,
                     uint32_t               pages,
                     test_entry_t *         p_entries,
                     uint32_t               entry_count,
                     bool                   put_seal);

void test_page_add_entry(flash_manager_page_t * p_area, test_entry_t * p_entry, bool put_seal);

void flash_execute(void);

void write_complete_callback(const flash_manager_t * p_manager,
                             const fm_entry_t *      p_entry,
                             fm_result_t             result);

void invalidate_complete_callback(const flash_manager_t * p_manager,
                                  fm_handle_t             handle,
                                  fm_result_t             result);

void remove_complete_callback(const flash_manager_t * p_manager);
#endif