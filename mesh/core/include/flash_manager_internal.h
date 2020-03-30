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

#ifndef FLASH_MANAGER_INTERNAL_H__
#define FLASH_MANAGER_INTERNAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "flash_manager.h"
#include "mesh_flash.h"

/**
 * @internal
 * @defgroup FLASH_MANAGER_INTERNAL Flash Manager internal declarations
 * Internal functions and types used by the flash manager.
 * @{
 */

#define HANDLE_SEAL    (0x7FFF)
#define HANDLE_BLANK   (0xFFFF)
#define HANDLE_PADDING (0x7F00)

extern const fm_header_t INVALID_HEADER;
extern const fm_header_t PADDING_HEADER;
extern const fm_header_t SEAL_HEADER;

#define BLANK_FLASH_WORD    (0xFFFFFFFF)
#define FLASH_MANAGER_FLASH_USER    MESH_FLASH_USER_MESH

#define PAGE_START_ALIGN(VALUE) ((uint32_t) (VALUE) & ~(PAGE_SIZE - 1UL))


typedef struct
{
    const flash_manager_page_t * p_storage_page;
    uint32_t data[(PAGE_SIZE - sizeof(flash_manager_page_t *)) / sizeof(uint32_t)];
} flash_manager_recovery_area_t;

static inline uint32_t flash(const void * p_dst, const void * p_src, uint32_t len, uint16_t * p_token)
{
    flash_operation_t op;
    op.type = FLASH_OP_TYPE_WRITE;
    op.params.write.p_start_addr = (uint32_t *) p_dst;
    op.params.write.p_data = (uint32_t *) p_src;
    op.params.write.length = len;
    return mesh_flash_op_push(FLASH_MANAGER_FLASH_USER, &op, p_token);
}

static inline uint32_t erase(const void * p_dst, uint32_t len, uint16_t * p_token)
{
    flash_operation_t op;
    op.type = FLASH_OP_TYPE_ERASE;
    op.params.erase.p_start_addr = (void *) p_dst;
    op.params.erase.length = len;
    return mesh_flash_op_push(FLASH_MANAGER_FLASH_USER, &op, p_token);
}


static inline const flash_manager_page_t * get_first_page(const flash_manager_page_t * p_page)
{
    return p_page - p_page->metadata.page_index;
}

static inline const flash_manager_page_t * get_last_page(const flash_manager_page_t * p_page)
{
    return p_page + (p_page->metadata.pages_in_area - p_page->metadata.page_index - 1);
}

static inline const void * get_area_end(const flash_manager_page_t * p_page)
{
    return get_last_page(p_page) + 1;
}

static inline bool handle_represents_data(fm_handle_t handle)
{
    return (handle != FLASH_MANAGER_HANDLE_INVALID && handle <= FLASH_MANAGER_HANDLE_MAX);
}

static inline bool handle_is_valid(fm_handle_t handle)
{
    return (handle_represents_data(handle) ||
            handle == HANDLE_SEAL);
}

static inline bool handle_matches_filter(fm_handle_t handle, const fm_handle_filter_t * p_filter)
{
    return (handle_represents_data(handle) &&
        ((p_filter == NULL) || ((handle & p_filter->mask) == (p_filter->match & p_filter->mask))));
}

static inline const fm_entry_t * get_first_entry(const flash_manager_page_t * p_area)
{
    return (const fm_entry_t *) ((uint32_t) p_area + p_area->metadata.metadata_len);
}


static inline const fm_entry_t * get_next_entry(const fm_entry_t * p_entry)
{
    NRF_MESH_ASSERT(p_entry->header.len_words != 0);
    if ((p_entry + p_entry->header.len_words) == (const fm_entry_t *) PAGE_START_ALIGN(p_entry + p_entry->header.len_words) ||
        p_entry->header.handle == HANDLE_PADDING)
    {
        /* For padding entries and entries that fill the current page, the next entry is the first
         * entry on the next page. */
        return get_first_entry((const flash_manager_page_t *) (PAGE_START_ALIGN(p_entry) + PAGE_SIZE));
    }
    else
    {
        return p_entry + p_entry->header.len_words;
    }
}

static inline const fm_entry_t * get_next_data_entry(const fm_entry_t * p_entry, const void * p_end)
{
    const void * p_start = p_entry;

    do
    {
        p_entry = get_next_entry(p_entry);

        if ((const void *) p_entry >= p_end || (void *) p_entry <= p_start)
        {
            return NULL;
        }
    } while (!handle_represents_data(p_entry->header.handle));

    return p_entry;
}

/** Check whether the handle pointer is out of the area owned by this manager. */
static inline bool entry_out_of_bounds(const flash_manager_t * p_manager, const fm_entry_t * p_entry)
{
    return !(p_entry < (const fm_entry_t *) &p_manager->config.p_area[p_manager->config.page_count]);
}

/**
 * Get the first entry matching the given handle, starting the search at @c p_start_entry.
 *
 * @param[in] p_start_entry First entry to check.
 * @param[in] p_end End of area to check through.
 * @param[in] handle Handle of entry to search for.
 *
 * @returns A pointer to the first entry that matches the given handle, or NULL if we reached the
 * end before finding it.
 */
const fm_entry_t * entry_get(const fm_entry_t * p_start_entry,
                             const void * p_end,
                             fm_handle_t handle);

/**
 * Callback function for when defrag procedure ends.
 */
void flash_manager_on_defrag_end(flash_manager_t * p_manager);

/** @} */

#endif /* FLASH_MANAGER_INTERNAL_H__ */

