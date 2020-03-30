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
#include "flash_manager_test_util.h"

#include <stdio.h>
#include "unity.h"
#include "cmock.h"

#define PROCESS_FLAG (0x12345678)
#define FLASH_OP_QUEUE_MAXLEN (256)

#define COLOR_PRINT (1) /**< Whether to use colors when printing pages */

#if COLOR_PRINT
#define COLOR_METADATA "[1m[35m"
#define COLOR_HEADER "[1m[32m"
#define COLOR_PADDING "[1m[33m"
#define COLOR_INVALID "[2m[31m"
#define COLOR_INVALID_DATA "[1m[30m"
#define COLOR_SEAL "[2m[30m[41m"
#define COLOR_BLANK "[1m[30m"
#define COLOR_NORMAL "[0m"
#else
#define COLOR_METADATA
#define COLOR_HEADER
#define COLOR_PADDING
#define COLOR_INVALID
#define COLOR_INVALID_DATA
#define COLOR_SEAL
#define COLOR_BLANK
#define COLOR_NORMAL
#endif

uint32_t                     g_flash_push_return;
uint32_t                     g_flash_queue_slots;
uint32_t                     g_flash_token;
uint32_t                     g_callback_token;
bearer_event_flag_callback_t g_process_cb;
mesh_flash_op_cb_t           g_flash_cb;
bool                         g_delayed_execution;
flash_manager_t *            gp_active_manager;
fm_handle_t                  g_expected_handle;
const fm_entry_t *           gp_expected_entry;
fm_result_t                  g_expected_result;
uint32_t                     g_completes;
bool                         g_flash_malfunctions;
uint32_t                     g_expected_remove_complete;
fifo_t                       g_flash_operation_queue;
static flash_operation_t     m_flash_operation_queue_buffer[FLASH_OP_QUEUE_MAXLEN];

static fm_entry_t * test_page_put_entry(fm_entry_t * p_dst, test_entry_t * p_entry)
{
    if (p_dst + p_entry->len > (const fm_entry_t *) (PAGE_START_ALIGN(p_dst) + PAGE_SIZE))
    {
        /* pad the page */
        p_dst->header.handle = HANDLE_PADDING;
        p_dst->header.len_words    = 0xFFFF;
        p_dst                = (fm_entry_t *) get_next_entry(p_dst);
    }
    p_dst->header.handle = p_entry->handle;
    p_dst->header.len_words    = p_entry->len;
    for (uint32_t j = 0; j < (uint32_t)(p_entry->len - 1); j++)
    {
        p_dst->data[j] = p_entry->data_value;
    }
    return p_dst;
}

void flash_manager_test_util_setup(void)
{
    g_delayed_execution                = false;
    g_flash_push_return                = NRF_SUCCESS;
    g_flash_queue_slots                = 0;
    g_process_cb                       = NULL;
    g_flash_cb                         = NULL;
    g_flash_operation_queue.array_len  = FLASH_OP_QUEUE_MAXLEN;
    g_flash_operation_queue.elem_array = m_flash_operation_queue_buffer;
    g_flash_operation_queue.elem_size  = sizeof(flash_operation_t);
    g_flash_token                      = 0;
    g_callback_token                   = 0;
    g_completes                        = 0;
    fifo_init(&g_flash_operation_queue);
    gp_active_manager = NULL;
    gp_expected_entry = NULL;
    g_flash_malfunctions = false;
}

void print_page(const flash_manager_page_t * p_page)
{
    printf("-----------------------------------------------------------------------\n");
    printf("len: %02x header: [tot len: %02x handle: %02x len: %02x] index: %02x count: %02x\n",
           p_page->metadata.metadata_len,
           p_page->metadata.entry_header_length,
           p_page->metadata.entry_type_length_bits,
           p_page->metadata.entry_len_length_bits,
           p_page->metadata.page_index,
           p_page->metadata.pages_in_area);
    printf("-----------------------------------------------------------------------\n");
    const fm_entry_t * p_entry = get_first_entry(p_page);
    uint32_t *         p       = (uint32_t *) p_page->raw;
    printf(COLOR_METADATA "%08x %08x " COLOR_NORMAL, p[0], p[1]);
    bool invalid = false;
    for (uint32_t i = 2; i < PAGE_SIZE / 4; i++)
    {
        if (&p[i] == (uint32_t *) p_entry)
        {
            invalid = false;
            if (p_entry->header.handle == FLASH_MANAGER_HANDLE_INVALID)
            {
                invalid = true;
                printf(COLOR_INVALID);
            }
            else if (p_entry->header.handle == HANDLE_SEAL)
            {
                printf(COLOR_SEAL);
            }
            else if (p_entry->header.handle == HANDLE_PADDING)
            {
                printf(COLOR_PADDING);
            }
            else if (p_entry->header.handle == HANDLE_BLANK)
            {
                printf(COLOR_BLANK);
            }
            else
            {
                printf(COLOR_HEADER);
            }
            p_entry = get_next_entry(p_entry);
        }
        else if (p[i] == BLANK_FLASH_WORD)
        {
            printf(COLOR_BLANK);
        }
        printf("%08x" COLOR_NORMAL " ", p[i]);
        if (invalid)
        {
            printf(COLOR_INVALID_DATA);
        }
        if ((i % 8) == 7)
        {
            printf("\n");
        }
    }
    printf(COLOR_NORMAL);
}

/*lint -save -e429 Custodial pointer has not been freed or returned - handled correctly by function. */
void build_test_page(flash_manager_page_t * p_area,
                     uint32_t               pages,
                     test_entry_t *         p_entries,
                     uint32_t               entry_count,
                     bool                   put_seal)
{
    for (uint32_t i = 0; i < pages; i++)
    {
        p_area[i].metadata.metadata_len           = 8;
        p_area[i].metadata.entry_header_length    = 4;
        p_area[i].metadata.entry_type_length_bits = 16;
        p_area[i].metadata.entry_len_length_bits  = 16;
        p_area[i].metadata.pages_in_area          = pages;
        p_area[i].metadata.page_index             = i;
        p_area[i].metadata._padding               = 0xFFFF;
    }
    fm_entry_t * p_entry = (fm_entry_t *) get_first_entry(p_area);
    for (uint32_t i = 0; i < entry_count; i++)
    {
        p_entry = (fm_entry_t *) get_next_entry(test_page_put_entry(p_entry, &p_entries[i]));
        TEST_ASSERT(p_entry < (fm_entry_t *) &p_area[pages]);
    }
    if (put_seal)
    {
        p_entry->header.handle = HANDLE_SEAL;
    }
}
/*lint -restore */

void test_page_add_entry(flash_manager_page_t * p_area, test_entry_t * p_entry, bool put_seal)
{
    fm_entry_t * p_dst = (fm_entry_t *) entry_get(get_first_entry(p_area), get_area_end(p_area), HANDLE_SEAL);
    TEST_ASSERT_NOT_NULL_MESSAGE(p_dst, "Couldn't find seal when adding test entry, please put seal into the page first.");

    p_dst = test_page_put_entry(p_dst, p_entry);
    if (put_seal)
    {
        ((fm_entry_t *) get_next_entry(p_dst))->header.handle = HANDLE_SEAL;
    }
}

void validate_metadata(const flash_manager_metadata_t * p_metadata,
                              uint8_t                          page_index,
                              uint8_t                          page_count)
{
    TEST_ASSERT_EQUAL(8, p_metadata->metadata_len);
    TEST_ASSERT_EQUAL(4, p_metadata->entry_header_length);
    TEST_ASSERT_EQUAL(16, p_metadata->entry_type_length_bits);
    TEST_ASSERT_EQUAL(16, p_metadata->entry_len_length_bits);
    TEST_ASSERT_EQUAL(page_count, p_metadata->pages_in_area);
    TEST_ASSERT_EQUAL(page_index, p_metadata->page_index);
    TEST_ASSERT_EQUAL(0xFFFF, p_metadata->_padding);
}

void validate_blank_flash(void * p_start, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++)
    {
        TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFF, ((uint32_t *) p_start)[i]);
    }
}

void flash_execute(void)
{
    flash_operation_t op;
    TEST_ASSERT_NOT_NULL(g_flash_cb);
    while (!fifo_is_empty(&g_flash_operation_queue))
    {
        while (fifo_pop(&g_flash_operation_queue, &op) == NRF_SUCCESS)
        {
            switch (op.type)
            {
                case FLASH_OP_TYPE_WRITE:
                    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(op.params.write.p_data));
                    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(op.params.write.length));
                    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(op.params.write.p_start_addr));
                    if (!g_flash_malfunctions)
                    {
                        for (uint32_t i = 0; i < op.params.write.length / sizeof(uint32_t); i++)
                        {
                            op.params.write.p_start_addr[i] &= op.params.write.p_data[i];
                        }
                    }
                    break;
                case FLASH_OP_TYPE_ERASE:
                    TEST_ASSERT_TRUE(IS_PAGE_ALIGNED(op.params.erase.p_start_addr));
                    TEST_ASSERT_TRUE(IS_PAGE_ALIGNED(op.params.erase.length));
                    if (!g_flash_malfunctions)
                    {
                        for (uint32_t i = 0; i < op.params.erase.length / sizeof(uint32_t); i++)
                        {
                            op.params.erase.p_start_addr[i] = 0xFFFFFFFF;
                        }
                    }
                    break;
                default:
                    TEST_FAIL_MESSAGE("Only read and write can be scheduled.");
            }
            g_flash_cb(MESH_FLASH_USER_MESH, &op, g_callback_token++);
        }

        flash_operation_t all_op;
        all_op.type = FLASH_OP_TYPE_ALL;
        g_flash_cb(MESH_FLASH_USER_MESH, &all_op, 0);
    }
}

void write_complete_callback(const flash_manager_t * p_manager,
                                    const fm_entry_t *      p_entry,
                                    fm_result_t             result)
{
    TEST_ASSERT_EQUAL_PTR(gp_active_manager, p_manager);
    TEST_ASSERT_EQUAL_PTR(gp_expected_entry, p_entry);
    TEST_ASSERT_EQUAL(g_expected_result, result);
    g_completes++;
}

void invalidate_complete_callback(const flash_manager_t * p_manager,
                                         fm_handle_t             handle,
                                         fm_result_t             result)
{
    TEST_ASSERT_EQUAL_PTR(gp_active_manager, p_manager);
    TEST_ASSERT_EQUAL_MEMORY(&g_expected_handle, &handle, sizeof(handle));
    TEST_ASSERT_EQUAL(g_expected_result, result);
}

void remove_complete_callback(const flash_manager_t * p_manager)
{
    TEST_ASSERT_EQUAL_PTR(gp_active_manager, p_manager);
    TEST_ASSERT_TRUE(g_expected_remove_complete > 0);
    g_expected_remove_complete--;
}
/*****************************************************************************
* Mocks
*****************************************************************************/
uint32_t mesh_flash_op_push(mesh_flash_user_t         user,
                            const flash_operation_t * p_op,
                            uint16_t *                p_token)
{
    TEST_ASSERT_EQUAL(MESH_FLASH_USER_MESH, user);
    TEST_ASSERT_NOT_NULL(p_op);

    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            TEST_ASSERT_NOT_NULL(p_op->params.write.p_data);
            TEST_ASSERT_NOT_NULL(p_op->params.write.p_start_addr);
            TEST_ASSERT_TRUE(IS_WORD_ALIGNED(p_op->params.write.p_data));
            TEST_ASSERT_TRUE(IS_WORD_ALIGNED(p_op->params.write.length));
            TEST_ASSERT_TRUE(IS_WORD_ALIGNED(p_op->params.write.p_start_addr));
            break;
        case FLASH_OP_TYPE_ERASE:
            TEST_ASSERT_NOT_NULL(p_op->params.erase.p_start_addr);
            TEST_ASSERT_TRUE(IS_PAGE_ALIGNED(p_op->params.erase.p_start_addr));
            TEST_ASSERT_TRUE(IS_PAGE_ALIGNED(p_op->params.erase.length));
            break;
        default:
            TEST_FAIL_MESSAGE("Only read and write can be scheduled.");
    }

    if (g_flash_queue_slots == 0)
    {
        return NRF_ERROR_NO_MEM;
    }
    g_flash_queue_slots--;
    if (g_flash_push_return == NRF_SUCCESS)
    {
        TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS,
                                  fifo_push(&g_flash_operation_queue, p_op),
                                  "Flash queue exceeded limit, rewrite test");
    }
    if (p_token)
    {
        *p_token = g_flash_token;
    }
    g_flash_token++;
    return g_flash_push_return;
}

uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user)
{
    TEST_ASSERT_EQUAL(MESH_FLASH_USER_MESH, user);
    return g_flash_queue_slots;
}

void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb)
{
    TEST_ASSERT_EQUAL(MESH_FLASH_USER_MESH, user);
    g_flash_cb = cb;
}

void mesh_flash_set_suspended(bool suspended)
{

}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback)
{
    TEST_ASSERT_EQUAL(NULL, g_process_cb);
    g_process_cb = callback;
    return PROCESS_FLAG;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    TEST_ASSERT_EQUAL(PROCESS_FLAG, flag);
    TEST_ASSERT_NOT_NULL(g_process_cb);
    if (!g_delayed_execution)
    {
        g_process_cb();
    }
}

