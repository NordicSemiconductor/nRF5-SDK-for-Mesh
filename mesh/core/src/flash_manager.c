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
#include <stdio.h>
#include <string.h>
#include "flash_manager.h"
#include "flash_manager_internal.h"
#include "flash_manager_defrag.h"
#include "nrf.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "bearer_event.h"
#include "nrf_mesh_assert.h"
#include "internal_event.h"
#include "queue.h"
#include "log.h"

#define HEADER_LEN       (sizeof(fm_header_t))
#define ACTION_BUFFER_SIZE_NO_PARAMS     (offsetof(action_t, params))
#define ACTION_BUFFER_SIZE_ENTRY_NO_DATA (offsetof(action_t, params.entry_data.entry.data))
#define ACTION_BUFFER_SIZE_METADATA      (offsetof(action_t, params.metadata) + sizeof(flash_manager_metadata_t))
#define ACTION_QUEUE_BUFFER_LENGTH       (sizeof(packet_buffer_packet_t) + FLASH_MANAGER_POOL_SIZE)

NRF_MESH_STATIC_ASSERT(HEADER_LEN == WORD_SIZE);
NRF_MESH_STATIC_ASSERT(IS_WORD_ALIGNED(sizeof(flash_manager_metadata_t)));

/** Action to perform */
typedef enum
{
    ACTION_TYPE_REPLACE, /**< Replace an existing entry, or create a new. */
    ACTION_TYPE_INVALIDATE, /**< Invalidate an existing entry. */
    ACTION_TYPE_BUILD_METADATA, /**< Build page metadata. */
    ACTION_TYPE_RECOVER_SEAL, /**< Recover seal at end of entries. */
    ACTION_TYPE_ERASE_AREA, /**< Erase the entire manager area. */
} action_type_t;

/**
 * States an action go through while being processed.
 */
typedef enum
{
    ACTION_STATE_IDLE,           /**< No action is being processed. */
    ACTION_STATE_PROCESSING,     /**< This module is processing an action. */
    ACTION_STATE_WAIT_FOR_FLASH, /**< Waiting for flash operations to finish. */
    ACTION_STATE_DONE,           /**< Done processing the event. */
} action_state_t;

/** Flash manager action buffer, used to keep an entry waiting to be flashed. */
typedef struct
{
    flash_manager_t * p_manager; /**< Flash manager doing the write. */
    action_type_t       action;    /**< Action to perform. */
    union
    {
        struct
        {
            const fm_entry_t * p_target; /**< Pointer to target entry in flash. Used to pass the location to the user. */
            fm_entry_t         entry;    /**< Entry data to write. */
        } entry_data;
        flash_manager_metadata_t metadata; /**< Metadata to write. */
    } params;
} action_t;

typedef struct
{
    void * p_buffer;
    uint32_t * p_length;
    uint32_t status;
} entry_copy_args_t;

static packet_buffer_t m_action_queue;
static uint8_t         m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH] __attribute__((aligned(WORD_SIZE)));

/* Short, common flash entries: */
const fm_header_t INVALID_HEADER __attribute__((aligned(WORD_SIZE))) = {0xFFFF, FLASH_MANAGER_HANDLE_INVALID};
const fm_header_t PADDING_HEADER __attribute__((aligned(WORD_SIZE))) = {0xFFFF, HANDLE_PADDING};
const fm_header_t SEAL_HEADER    __attribute__((aligned(WORD_SIZE))) = {0xFFFF, HANDLE_SEAL};

static bearer_event_flag_t   m_processing_flag;
static fm_state_t            m_state;
static action_state_t        m_action_state;
static uint16_t              m_token; /**< Token dealt by mesh flash that marks all flash operations complete for the current action. */
static queue_t               m_memory_listener_queue;

static flash_manager_queue_empty_cb_t m_queue_empty_cb;
/******************************************************************************
* Static functions
******************************************************************************/

static inline void schedule_processing(void)
{
    bearer_event_flag_set(m_processing_flag);
}

static fm_iterate_action_t iterate_callback_entry_copy(const fm_entry_t * p_entry, void * p_args)
{
    entry_copy_args_t * p_copy = p_args;

    uint32_t entry_len = p_entry->header.len_words * WORD_SIZE - sizeof(fm_header_t);
    if (ALIGN_VAL(*p_copy->p_length, WORD_SIZE) != entry_len)
    {
        p_copy->status = NRF_ERROR_INVALID_LENGTH;
    }
    else
    {
        p_copy->status = NRF_SUCCESS;
        memcpy(p_copy->p_buffer, p_entry->data, *p_copy->p_length);
    }

    *p_copy->p_length = entry_len;

    return FM_ITERATE_ACTION_STOP;
}

/**
 * Get the packet buffer containing the given action in the action queue.
 *
 * @param[in] p_action Action to get packet buffer of
 *
 * @returns Pointer to packet buffer containing the given action.
 */
static inline packet_buffer_packet_t * get_packet_buffer(const action_t * p_action)
{
    NRF_MESH_ASSERT(p_action >= (action_t *) &m_action_queue_buffer[0] &&
                    p_action <= (action_t *) &m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH]);
    return (packet_buffer_packet_t *) ((uint32_t) p_action - offsetof(packet_buffer_packet_t, packet));
}
/**
 * Get the action containing the given entry in the action queue.
 *
 * @param[in] p_entry Entry to get action of.
 *
 * @returns Pointer to action containing the given entry.
 */
static inline action_t * get_entry_action(const fm_entry_t * p_entry)
{
    NRF_MESH_ASSERT(p_entry >= (fm_entry_t *) &m_action_queue_buffer[0] &&
                    p_entry <= (fm_entry_t *) &m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH]);
    return (action_t *) ((uint32_t) p_entry - offsetof(action_t, params.entry_data.entry));
}

static action_t * reserve_action_buffer(uint32_t size)
{
    packet_buffer_packet_t * p_packet_buffer = NULL;
    uint32_t status = packet_buffer_reserve(&m_action_queue,
                    &p_packet_buffer,
                    size);
    if (status == NRF_SUCCESS)
    {
        return (action_t *) p_packet_buffer->packet;
    }
    else
    {
        return NULL;
    }
}

static inline void commit_action_buffer(action_t * p_action)
{
    packet_buffer_packet_t * p_buffer = get_packet_buffer(p_action);
    packet_buffer_commit(&m_action_queue, p_buffer, p_buffer->size);
}

static const fm_entry_t * get_last_entry(const flash_manager_t * p_manager)
{
    const fm_entry_t * p_entry = NULL;
    const fm_entry_t * p_next  = get_first_entry(p_manager->config.p_area);
    while ((void *) p_next < get_area_end(p_manager->config.p_area) &&
           (p_next->header.handle != HANDLE_BLANK))
    {
        p_entry = p_next;
        p_next = get_next_entry(p_next);
    }
    return p_entry;
}

static inline const void * get_defrag_threshold(const flash_manager_t * p_manager)
{
    return (const void *) ((uint32_t) get_area_end(p_manager->config.p_area) -
                           p_manager->config.min_available_space);
}

/** Get the number of bytes remaining before the defragmentation threshold. The seal could be past
 * the threshold, so we need to do this as a signed integer. */
static int32_t get_remaining_free_space(const flash_manager_t * p_manager)
{
    const void * p_defrag_threshold =
        get_defrag_threshold(p_manager);

    return ((uint8_t *) p_defrag_threshold - (uint8_t *) p_manager->internal.p_seal);
}

static bool flash_area_is_blank(const void * p_area, uint32_t size)
{
    NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_area));
    NRF_MESH_ASSERT(IS_WORD_ALIGNED(size));
    for (uint32_t i = 0; i < size / sizeof(uint32_t); i++)
    {
        if (((uint32_t *) p_area)[i] != BLANK_FLASH_WORD)
        {
            return false;
        }
    }
    return true;
}

static inline bool metadata_is_valid(const flash_manager_metadata_t * p_metadata)
{
    return (p_metadata->metadata_len != 0xFF &&
            p_metadata->metadata_len >= 8 &&
            p_metadata->page_index != 0xFF &&
            p_metadata->pages_in_area != 0xFF);
}

static bool flash_area_is_valid(const flash_manager_t * p_manager)
{
    for (uint32_t i = 0; i < p_manager->config.page_count; i++)
    {
        if (metadata_is_valid(&p_manager->config.p_area[i].metadata))
        {
            /* Check that the existing metadata aligns with ours */
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.page_index == i);
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.pages_in_area == p_manager->config.page_count);
        }
        else
        {
            return false;
        }
    }

    return true;
}

static uint32_t flash_area_build(flash_manager_t * p_manager)
{
    for (uint32_t i = 0; i < p_manager->config.page_count; i++)
    {
        /* Only flash pages without complete metadata. */
        if (flash_area_is_blank(&p_manager->config.p_area[i].raw[WORD_SIZE], PAGE_SIZE - WORD_SIZE))
        {
            action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_METADATA);
            if (p_action == NULL)
            {
                return NRF_ERROR_NO_MEM;
            }
            else
            {
                p_action->action = ACTION_TYPE_BUILD_METADATA;
                p_action->params.metadata.metadata_len = sizeof(flash_manager_metadata_t);
                p_action->params.metadata.entry_header_length = sizeof(fm_header_t);
                p_action->params.metadata.entry_type_length_bits = sizeof(fm_handle_t) * 8;
                p_action->params.metadata.entry_len_length_bits = sizeof(((fm_header_t *) NULL)->len_words) * 8;
                p_action->params.metadata.page_index = i;
                p_action->params.metadata.pages_in_area = p_manager->config.page_count;
                p_action->params.metadata._padding = 0xFFFF;
                p_action->p_manager = p_manager;
                commit_action_buffer(p_action);
            }
        }
        else
        {
            /* If the page isn't blank, it has to contain valid metadata. If
             * this fails, the selected area contains data not managed by the
             * flash manager. */
            NRF_MESH_ASSERT(metadata_is_valid(&p_manager->config.p_area[i].metadata));
            /* The metadata page index and count must align with the page index
             * and count set for the given manager. If this fails, we are
             * trying to build two overlapping areas. */
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.page_index == i);
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.pages_in_area == p_manager->config.page_count);
        }
    }
    schedule_processing();
    return NRF_SUCCESS;
}

static uint32_t recover_seal(flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT_DEBUG(!flash_manager_defragging(p_manager));

    p_manager->internal.p_seal = get_last_entry(p_manager);
    if (p_manager->internal.p_seal == NULL)
    {
        const fm_entry_t * p_first = get_first_entry(p_manager->config.p_area);

        if (p_first->header.handle == HANDLE_BLANK)
        {
            /* The area only contained metadata. */
            p_manager->internal.p_seal = p_first;
        }
    }
    else if (p_manager->internal.p_seal->header.handle != HANDLE_SEAL)
    {
        /* The last entry isn't a seal, so we power cycled in the middle of
        * writing an entry to the area. Invalidate this entry if necessary,
        * and add a seal after it.
        */
        action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_NO_PARAMS);

        if (p_action == NULL)
        {
            return NRF_ERROR_NO_MEM;
        }
        else
        {
            p_action->action = ACTION_TYPE_RECOVER_SEAL;
            p_action->p_manager = p_manager;
            commit_action_buffer(p_action);

            schedule_processing();
        }
    }
    return NRF_SUCCESS;
}

/** If the last entry is a duplicate of another entry, we should invalidate its duplicate, as it
 * got a power loss before doing the final step of invalidating in a replace action.
 */
static uint32_t invalidate_duplicate_of_last_entry(flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT_DEBUG(!flash_manager_defragging(p_manager));
    uint32_t status = NRF_SUCCESS;
    /* get the last entry before the seal */
    const fm_entry_t * p_entry = get_first_entry(p_manager->config.p_area);
    if (p_manager->internal.p_seal <= p_entry)
    {
        /* there is no entry before the seal */
        return status;
    }
    const fm_entry_t * p_next = get_next_entry(p_entry);
    while (p_next != p_manager->internal.p_seal)
    {
        p_entry = p_next;
        p_next  = get_next_entry(p_next);
    }

    if (handle_represents_data(p_entry->header.handle))
    {
        const fm_entry_t * p_duplicate =
            entry_get(get_first_entry(p_manager->config.p_area), p_entry, p_entry->header.handle);

        if (p_duplicate != NULL && p_duplicate != p_entry)
        {
            status = flash_manager_entry_invalidate(p_manager, p_duplicate->header.handle);
        }
    }
    return status;
}

static uint32_t get_invalid_bytes(const flash_manager_page_t * p_area, uint32_t pages)
{
    uint32_t invalid_bytes = 0;
    const fm_entry_t * p_end = (fm_entry_t *) get_area_end(p_area);
    const fm_entry_t * p_entry = get_first_entry(p_area);

    while ((p_entry->header.handle != HANDLE_BLANK) && (p_entry->header.handle != HANDLE_SEAL))
    {
        if (p_entry->header.handle == FLASH_MANAGER_HANDLE_INVALID)
        {
            invalid_bytes += p_entry->header.len_words * WORD_SIZE;
        }
        p_entry = get_next_entry(p_entry);
        /* Check that we haven't gone out of bounds before finding the seal or a blank entry,
         * which means the area is invalid. */
        NRF_MESH_ASSERT(p_entry < p_end);
    }

    return invalid_bytes;
}

static bool validate_result(const action_t * p_action)
{
    uint32_t entry_length = p_action->params.entry_data.entry.header.len_words * WORD_SIZE;
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            return (memcmp(p_action->params.entry_data.p_target,
                           &p_action->params.entry_data.entry,
                           entry_length) == 0);
        case ACTION_TYPE_INVALIDATE:
            return (p_action->params.entry_data.p_target->header.handle ==
                    FLASH_MANAGER_HANDLE_INVALID);
        case ACTION_TYPE_BUILD_METADATA:
            return (memcmp(&p_action->p_manager->config.p_area[p_action->params.metadata.page_index].metadata,
                           &p_action->params.metadata,
                           sizeof(flash_manager_metadata_t)) == 0);
        default:
            return true;
    }
}

static void free_packet_buffer(packet_buffer_packet_t * p_packet_buffer)
{
    packet_buffer_free(&m_action_queue, p_packet_buffer);

    /* Notify all memory listeners. Run until we find the last entry, in case some of the entries
     * get re-added in their callbacks. */
    queue_elem_t * p_back = m_memory_listener_queue.p_back;
    if (p_back)
    {
        queue_elem_t * p_elem;
        do
        {
            p_elem = queue_pop(&m_memory_listener_queue);
            NRF_MESH_ASSERT(p_elem != NULL);
            fm_mem_listener_t * p_listener = (fm_mem_listener_t *) p_elem->p_data;
            p_elem->p_data = NULL;
            p_listener->callback(p_listener->p_args);
        } while (p_elem != p_back);
    }
}

static void end_action(action_t * p_action, fm_result_t result, const fm_entry_t * p_entry)
{
    if (result == FM_RESULT_SUCCESS && !validate_result(p_action))
    {
        result = FM_RESULT_ERROR_FLASH_MALFUNCTION;
    }

    if (result != FM_RESULT_SUCCESS)
    {
        /* report in-RAM entry on failure: */
        p_entry = &p_action->params.entry_data.entry;
    }

    flash_manager_t * p_manager = p_action->p_manager;
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            if (result == FM_RESULT_SUCCESS)
            {
                /* Need to reset the seal */
                p_manager->internal.p_seal = get_next_entry(p_action->params.entry_data.p_target);
                NRF_MESH_ASSERT(p_manager->internal.p_seal->header.handle == HANDLE_SEAL);
            }
            if (p_manager->config.write_complete_cb != NULL)
            {
                p_manager->config.write_complete_cb(p_manager, p_entry, result);
            }
            break;
        case ACTION_TYPE_INVALIDATE:
            if (p_manager->config.invalidate_complete_cb != NULL)
            {
                p_manager->config.invalidate_complete_cb(p_manager,
                        p_action->params.entry_data.entry.header.handle,
                        result);
            }
            break;
        case ACTION_TYPE_BUILD_METADATA:
            NRF_MESH_ASSERT(p_manager->internal.state == FM_STATE_BUILDING);
            if (p_action->params.metadata.page_index == p_action->params.metadata.pages_in_area - 1)
            {
                /* Done building the metadata on the last page. */
                p_manager->internal.state = FM_STATE_READY;
            }
            break;
        case ACTION_TYPE_ERASE_AREA:
            NRF_MESH_ASSERT(result == FM_RESULT_SUCCESS);
            if (p_manager->internal.state != FM_STATE_BUILDING)
            { // there is postponed metadata task
                p_manager->internal.state = FM_STATE_UNINITIALIZED;
            }

            if (p_manager->config.remove_complete_cb != NULL)
            {
                p_manager->config.remove_complete_cb(p_manager);
            }
            break;
        default:
            break;
    }

    free_packet_buffer(get_packet_buffer(p_action));
}

static bool defrag_required(action_t * p_next_action)
{
    if (p_next_action->action == ACTION_TYPE_REPLACE ||
        p_next_action->action == ACTION_TYPE_INVALIDATE)
    {
        int remaining_space = get_remaining_free_space(p_next_action->p_manager);
        int required_space;

        if (p_next_action->action == ACTION_TYPE_REPLACE)
        {
            required_space = p_next_action->params.entry_data.entry.header.len_words * WORD_SIZE;
        }
        else
        {
            required_space = 0;
        }

        if (remaining_space <= required_space &&
            p_next_action->p_manager->internal.invalid_bytes != 0)
        {
            return true;
        }
    }
    return false;
}
/******************************************************************************
* Action execution
******************************************************************************/
static fm_result_t execute_action_replace(action_t * p_action)
{
    const fm_entry_t * p_old_entry = entry_get(get_first_entry(p_action->p_manager->config.p_area),
                                               get_area_end(p_action->p_manager->config.p_area),
                                               p_action->params.entry_data.entry.header.handle);

    /* If we were defragging during a power loss, we won't be recovering the seal at the end, as we
     * won't know which manager we're dealing with. */
    if (p_action->p_manager->internal.p_seal == NULL)
    {
        p_action->p_manager->internal.p_seal =
            entry_get(get_first_entry(p_action->p_manager->config.p_area),
                      get_area_end(p_action->p_manager->config.p_area),
                      HANDLE_SEAL);
    }

    const flash_manager_page_t * p_next_page =
        (const flash_manager_page_t *) (PAGE_START_ALIGN(p_action->p_manager->internal.p_seal) + PAGE_SIZE);
    const fm_entry_t * p_new_seal = NULL;
    const fm_entry_t * p_new_entry = NULL;

    uint32_t remaining_space = ((uint32_t) p_next_page - (uint32_t) p_action->p_manager->internal.p_seal);

    if (remaining_space > (p_action->params.entry_data.entry.header.len_words * WORD_SIZE))
    {
        /* The entry and the seal can fit right after the previous entry. */
        p_new_entry = p_action->p_manager->internal.p_seal;
        p_new_seal = &p_action->p_manager->internal.p_seal[p_action->params.entry_data.entry.header.len_words];
    }
    else
    {
        if (p_next_page == get_area_end(p_action->p_manager->config.p_area))
        {
            /* No room for the packet */
            return FM_RESULT_ERROR_AREA_FULL;
        }

        if (remaining_space == (p_action->params.entry_data.entry.header.len_words * WORD_SIZE))
        {
            /* The entry can fills the page completely. Flash the seal on the next page */
            p_new_entry = p_action->p_manager->internal.p_seal;
            p_new_seal = get_first_entry(p_next_page);
        }
        else
        {
            /* The entry can't fit in the current page. Pad the page, and place the
             * entry and seal on the next. */
            NRF_MESH_ERROR_CHECK(flash(p_action->p_manager->internal.p_seal,
                        &PADDING_HEADER,
                        sizeof(PADDING_HEADER),
                        NULL));

            p_new_entry = get_first_entry(p_next_page);
            p_new_seal = p_new_entry + p_action->params.entry_data.entry.header.len_words;
        }
    }

    /* Flash the data */
    NRF_MESH_ERROR_CHECK(flash(p_new_entry,
                &p_action->params.entry_data.entry,
                p_action->params.entry_data.entry.header.len_words * WORD_SIZE,
                NULL));

    /* flash a seal at the end */
    NRF_MESH_ERROR_CHECK(flash(p_new_seal,
                &SEAL_HEADER,
                sizeof(SEAL_HEADER),
                &m_token));

    if (p_old_entry != NULL)
    {
        /* invalidate old entry */
        p_action->p_manager->internal.invalid_bytes += p_old_entry->header.len_words * WORD_SIZE;
        NRF_MESH_ERROR_CHECK(flash(p_old_entry,
                    &INVALID_HEADER,
                    sizeof(INVALID_HEADER),
                    &m_token));
    }

    p_action->params.entry_data.p_target = p_new_entry;
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_invalidate(action_t * p_action)
{
    const fm_entry_t * p_old_entry = entry_get(get_first_entry(p_action->p_manager->config.p_area),
                                               get_area_end(p_action->p_manager->config.p_area),
                                               p_action->params.entry_data.entry.header.handle);

    if (p_old_entry == NULL)
    {
        return FM_RESULT_ERROR_NOT_FOUND;
    }

    p_action->params.entry_data.p_target = p_old_entry;
    p_action->p_manager->internal.invalid_bytes += p_old_entry->header.len_words * WORD_SIZE;

    NRF_MESH_ERROR_CHECK(flash(p_old_entry,
                &INVALID_HEADER,
                sizeof(INVALID_HEADER),
                &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_build_metadata(action_t * p_action)
{
    const flash_manager_page_t * p_page =
        &p_action->p_manager->config.p_area[p_action->params.metadata.page_index];

    /* Ensure that the metadata part of the page is clean or identical to what we are adding. */
    if (!flash_area_is_blank(&p_page->metadata, sizeof(flash_manager_metadata_t)))
    {
        /* It's not enough to memcmp the new and existing metadata, as the existing could be
         * partially written. */
        for (uint32_t i = 0; i < sizeof(flash_manager_metadata_t); ++i)
        {
            /* If this assert fails, we have two overlapping flash manager areas. */
            NRF_MESH_ASSERT(p_page->raw[i] == ((uint8_t *) &p_action->params.metadata)[i] ||
                            p_page->raw[i] == 0xFF);
        }
    }

    NRF_MESH_ERROR_CHECK(flash(p_page,
                               &p_action->params.metadata,
                               sizeof(flash_manager_metadata_t),
                               &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_recover_seal(action_t * p_action)
{
    const fm_entry_t * p_last_entry = get_last_entry(p_action->p_manager);
    NRF_MESH_ASSERT(p_last_entry != NULL);

    bool last_entry_is_invalid = (p_last_entry->header.handle == FLASH_MANAGER_HANDLE_INVALID);

    if (!last_entry_is_invalid)
    {
        NRF_MESH_ERROR_CHECK(flash(&p_last_entry->header,
                    &INVALID_HEADER,
                    sizeof(INVALID_HEADER),
                    &m_token));
    }

    /* place seal at end */
    p_action->p_manager->internal.p_seal = get_next_entry(p_last_entry);
    if ((void *) p_action->p_manager->internal.p_seal < get_area_end(p_action->p_manager->config.p_area))
    {
        NRF_MESH_ERROR_CHECK(flash(get_next_entry(p_last_entry),
                    &SEAL_HEADER,
                    sizeof(SEAL_HEADER),
                    &m_token));
    }
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_erase_area(action_t * p_action)
{
    NRF_MESH_ERROR_CHECK(erase(p_action->p_manager->config.p_area,
                               p_action->p_manager->config.page_count * PAGE_SIZE,
                               &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action(action_t * p_action)
{
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FM_ACTION, p_action->action, 0, NULL);
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            return execute_action_replace(p_action);
        case ACTION_TYPE_INVALIDATE:
            return execute_action_invalidate(p_action);
        case ACTION_TYPE_BUILD_METADATA:
            return execute_action_build_metadata(p_action);
        case ACTION_TYPE_RECOVER_SEAL:
            return execute_action_recover_seal(p_action);
        case ACTION_TYPE_ERASE_AREA:
            return execute_action_erase_area(p_action);
    }
    NRF_MESH_ASSERT(false);
    return FM_RESULT_SUCCESS;
}

static bool process_action_queue(void)
{
    static action_t * p_current = NULL;
    static fm_result_t result;

    while (m_state == FM_STATE_READY)
    {
        switch (m_action_state)
        {
            case ACTION_STATE_IDLE:
            {
                packet_buffer_packet_t * p_buffer = NULL;
                if (packet_buffer_pop(&m_action_queue,
                                      &p_buffer) != NRF_SUCCESS)
                {
                    if (m_queue_empty_cb)
                    {
                        m_queue_empty_cb();
                    }
                    return true;
                }
                p_current = (action_t *) p_buffer->packet;
                m_action_state = ACTION_STATE_PROCESSING;
                if (defrag_required(p_current))
                {
                    /* do defrag, then come back once its finished */
                    m_state = FM_STATE_DEFRAG;
                    p_current->p_manager->internal.state = FM_STATE_DEFRAG;
                    flash_manager_defrag(p_current->p_manager);
                }
                break;
            }
            case ACTION_STATE_PROCESSING:
                result = execute_action(p_current);
                if (result == FM_RESULT_SUCCESS)
                {
                    m_action_state = ACTION_STATE_WAIT_FOR_FLASH;
                }
                else
                {
                    /* The action failed, and we should skip the WAIT_FOR_FLASH state. */
                    m_action_state = ACTION_STATE_DONE;
                }
                break;

            case ACTION_STATE_WAIT_FOR_FLASH:
                /* Exit the loop to let the flash process the action. */
                return true;

            case ACTION_STATE_DONE:
                NRF_MESH_ASSERT(p_current != NULL);
                /* report to user */
                end_action(p_current,
                        result,
                        p_current->params.entry_data.p_target);
                m_action_state = ACTION_STATE_IDLE;
                break;

            default:
                break;
        }
    }
    return true;
}

static void flash_op_ended_callback(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    if (token == m_token && m_action_state == ACTION_STATE_WAIT_FOR_FLASH)
    {
        /* we've received the last token for this action */
        m_action_state = ACTION_STATE_DONE;
        (void)process_action_queue();
    }
}

/******************************************************************************
* Interface functions
******************************************************************************/

void flash_manager_init(void)
{
    packet_buffer_init(&m_action_queue, m_action_queue_buffer, sizeof(m_action_queue_buffer));
    mesh_flash_user_callback_set(MESH_FLASH_USER_MESH, flash_op_ended_callback);
    m_processing_flag = bearer_event_flag_add(process_action_queue);
    m_action_state = ACTION_STATE_IDLE;
    m_token = 0;
    queue_init(&m_memory_listener_queue);

    if (flash_manager_defrag_init())
    {
        m_state = FM_STATE_DEFRAG;
    }
    else
    {
        m_state = FM_STATE_READY;
    }
}

uint32_t flash_manager_add(flash_manager_t * p_manager,
        const flash_manager_config_t * p_config)
{
    __LOG(LOG_SRC_FM, LOG_LEVEL_DBG3, "FM area: 0x%08x\n", p_config->p_area);

    uint32_t status = NRF_SUCCESS;
    p_manager->internal.state = FM_STATE_UNINITIALIZED;

    NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_config->p_area));
    NRF_MESH_ASSERT(p_config->page_count < FLASH_MANAGER_PAGE_COUNT_MAX);

    memcpy(&p_manager->config, p_config, sizeof(flash_manager_config_t));
    p_manager->internal.p_seal = NULL;
    p_manager->internal.invalid_bytes = 0;

    if (flash_area_is_valid(p_manager))
    {
        /* All bets are off for seal and invalid bytes calculation during defragmentation. If defrag
         * is running for this area, the seal and invalid bytes count will be reinstated at the end of the procedure. */
        if (flash_manager_defragging(p_manager))
        {
            p_manager->internal.state = FM_STATE_DEFRAG;
        }
        else
        {
            p_manager->internal.invalid_bytes = get_invalid_bytes(p_manager->config.p_area, p_manager->config.page_count);
            status = recover_seal(p_manager);
            if (status == NRF_SUCCESS)
            {
                p_manager->internal.state = FM_STATE_READY;
                status = invalidate_duplicate_of_last_entry(p_manager);
            }
        }
    }
    else
    {
        p_manager->internal.state = FM_STATE_BUILDING;
        /* If the area build fails, it's because we can't fit all the metadata
         * in the action queue. Consider increasing its size. */
        status = flash_area_build(p_manager);
        if (status == NRF_SUCCESS)
        {
            p_manager->internal.p_seal =
                (const fm_entry_t *) &p_manager->config.p_area->raw[sizeof(flash_manager_metadata_t)];
        }
    }
    return status;
}

uint32_t flash_manager_remove(flash_manager_t * p_manager)
{
    if (!(p_manager->internal.state == FM_STATE_READY || p_manager->internal.state == FM_STATE_UNINITIALIZED))
    {
        return NRF_ERROR_INVALID_STATE;
    }
    action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_NO_PARAMS);
    if (p_action == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        p_manager->internal.state = FM_STATE_REMOVING;
        p_action->action = ACTION_TYPE_ERASE_AREA;
        p_action->p_manager = p_manager;
        commit_action_buffer(p_action);
        schedule_processing();
        return NRF_SUCCESS;
    }
}

const fm_entry_t * flash_manager_entry_get(const flash_manager_t * p_manager, fm_handle_t handle)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    if (!handle_represents_data(handle) || p_manager->internal.state != FM_STATE_READY)
    {
        return NULL;
    }
    return entry_get(get_first_entry(p_manager->config.p_area),
                     get_area_end(p_manager->config.p_area),
                     handle);
}

uint32_t flash_manager_entry_read(const flash_manager_t * p_manager,
                                  fm_handle_t handle,
                                  void * p_data,
                                  uint32_t * p_length)
{
    if (p_manager == NULL || p_data == NULL || p_length == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (!handle_represents_data(handle))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_manager->internal.state != FM_STATE_READY && p_manager->internal.state != FM_STATE_DEFRAG)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    const fm_handle_filter_t filter = {
        .mask  = 0xFFFF,
        .match = handle,
    };
    entry_copy_args_t args = {
        .p_length = p_length,
        .p_buffer = p_data,
        .status = NRF_ERROR_NOT_FOUND, // Will be overwritten by the callback if an entry is found
    };
    (void) flash_manager_entries_read(p_manager, &filter, iterate_callback_entry_copy, &args);

    return args.status;
}

const fm_entry_t * flash_manager_entry_next_get(const flash_manager_t * p_manager,
        const fm_handle_filter_t * p_filter,
        const fm_entry_t * p_start)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    if (p_manager->internal.state != FM_STATE_READY)
    {
        return NULL;
    }
    const fm_entry_t * p_entry;
    if (p_start == NULL)
    {
        p_entry = get_first_entry(p_manager->config.p_area);
    }
    else if (p_start < get_first_entry(p_manager->config.p_area) ||
             p_start > (const fm_entry_t *) get_area_end(p_manager->config.p_area))
    {
        return NULL;
    }
    else
    {
        p_entry = get_next_entry(p_start);
    }
    const fm_entry_t * p_end = get_area_end(p_manager->config.p_area);
    while (p_entry < p_end &&
           !handle_matches_filter(p_entry->header.handle, p_filter))
    {
        p_entry = get_next_entry(p_entry);
    }

    if (p_entry < p_end && handle_matches_filter(p_entry->header.handle, p_filter))
    {
        return p_entry;
    }
    else
    {
        return NULL;
    }
}

uint32_t flash_manager_entries_read(const flash_manager_t * p_manager, const fm_handle_filter_t * p_filter, flash_manager_read_cb_t read_cb, void * p_args)
{
    NRF_MESH_ASSERT(p_manager != NULL);

    mesh_flash_set_suspended(true);

    // first draft at something that considers defrag:
    const flash_manager_recovery_area_t * p_recovery_area = flash_manager_defrag_recovery_page_get();
    fm_iterate_action_t action = FM_ITERATE_ACTION_CONTINUE;
    uint32_t entries_read = 0;

    for (uint32_t page_index = 0;
         (page_index < p_manager->config.page_count && action == FM_ITERATE_ACTION_CONTINUE);
         ++page_index)
    {
        const flash_manager_page_t * p_page = &p_manager->config.p_area[page_index];
        const void * p_end = p_page + 1;

        /* As the recovery page can contain entries from the pages following the one it's backing
         * up, we can have duplicates. We should skip these duplicates to avoid reporting them twice. */
        if (p_page == p_recovery_area->p_storage_page)
        {
            p_end = &p_recovery_area->data[ARRAY_SIZE(p_recovery_area->data)];

            /* Find the first data entry on the next page, and see if it's duplicated on the
             * recovery page. If it is, that'll be the first entry we skip on the recovery. */
            if (page_index + 1 != p_manager->config.page_count)
            {
                const flash_manager_page_t * p_next_page = p_page + 1;
                const fm_entry_t * p_first_on_next_page = get_first_entry(p_next_page);
                if (!handle_represents_data(p_first_on_next_page->header.handle))
                {
                    p_first_on_next_page = get_next_data_entry(p_first_on_next_page, p_next_page + 1);
                }

                if (p_first_on_next_page)
                {
                    const fm_entry_t * p_first_duplicate =
                        entry_get(get_first_entry((const flash_manager_page_t *) p_recovery_area->data),
                                  p_end,
                                  p_first_on_next_page->header.handle);

                    p_end = (p_first_duplicate != NULL) ? p_first_duplicate : p_end;
                }
            }

            p_page = (const flash_manager_page_t *) p_recovery_area->data;
        }

        for (const fm_entry_t * p_it = get_first_entry(p_page);
             ((intptr_t) p_it < (intptr_t) p_end && action == FM_ITERATE_ACTION_CONTINUE);
             p_it = get_next_entry(p_it))
        {
            if (handle_matches_filter(p_it->header.handle, p_filter))
            {
                entries_read++;
                if (read_cb != NULL)
                {
                    action = read_cb(p_it, p_args);
                }
            }
        }
    }

    mesh_flash_set_suspended(false);
    return entries_read;
}

uint32_t flash_manager_entry_count_get(const flash_manager_t *    p_manager,
                                       const fm_handle_filter_t * p_filter)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    return flash_manager_entries_read(p_manager, p_filter, NULL, NULL);
}

fm_entry_t * flash_manager_entry_alloc(flash_manager_t * p_manager,
        fm_handle_t handle,
        uint32_t data_length)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    NRF_MESH_ASSERT(handle_is_valid(handle));
    NRF_MESH_ASSERT(data_length <= FLASH_MANAGER_ENTRY_MAX_SIZE);

    if (p_manager->internal.state == FM_STATE_UNINITIALIZED)
    {
        return NULL;
    }

    uint32_t buffer_length = data_length + ACTION_BUFFER_SIZE_ENTRY_NO_DATA;

    action_t * p_action = reserve_action_buffer(buffer_length);
    if (p_action == NULL)
    {
        return NULL;
    }
    else
    {
        p_action->action = ACTION_TYPE_REPLACE;
        p_action->params.entry_data.entry.header.handle = handle;
        /* length should be in padded words: */
        p_action->params.entry_data.entry.header.len_words = 1 + ALIGN_VAL(data_length, WORD_SIZE) / WORD_SIZE;
        p_action->p_manager = p_manager;

        /* Write 1s to last word (if any) to avoid padding with random data if data_length is not already aligned */
        if (p_action->params.entry_data.entry.header.len_words > 1)
        {
            p_action->params.entry_data.entry.data[p_action->params.entry_data.entry.header.len_words - 2] = UINT32_MAX;
        }

        return &p_action->params.entry_data.entry;
    }
}

void flash_manager_entry_commit(const fm_entry_t * p_entry)
{
    commit_action_buffer(get_entry_action(p_entry));
    schedule_processing();
}

uint32_t flash_manager_entry_invalidate(flash_manager_t * p_manager, fm_handle_t handle)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    NRF_MESH_ASSERT(handle_is_valid(handle));

    action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_ENTRY_NO_DATA);
    if (p_action == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        p_action->action = ACTION_TYPE_INVALIDATE;
        p_action->params.entry_data.entry.header.handle = handle;
        p_action->params.entry_data.entry.header.len_words = 0xFFFF; /* do not alter length of the entry */
        p_action->p_manager = p_manager;
        commit_action_buffer(p_action);
        schedule_processing();
        return NRF_SUCCESS;
    }
}

void flash_manager_entry_release(fm_entry_t * p_entry)
{
    packet_buffer_packet_t * p_packet_buffer = get_packet_buffer(get_entry_action(p_entry));

    free_packet_buffer(p_packet_buffer);

}

void flash_manager_mem_listener_register(fm_mem_listener_t * p_listener)
{
    NRF_MESH_ASSERT(p_listener != NULL);
    NRF_MESH_ASSERT(p_listener->callback != NULL);
    if (p_listener->queue_elem.p_data == NULL)
    {
        p_listener->queue_elem.p_data = p_listener;
        queue_push(&m_memory_listener_queue, &p_listener->queue_elem);
    }
}

bool flash_manager_is_stable(void)
{
    return (packet_buffer_is_empty(&m_action_queue) && !flash_manager_defrag_is_running());
}

void flash_manager_on_defrag_end(flash_manager_t * p_manager)
{
    if (p_manager != NULL)
    {
        const fm_entry_t * p_first_entry = get_first_entry(p_manager->config.p_area);

        p_manager->internal.p_seal = entry_get(p_first_entry,
                                      get_area_end(p_manager->config.p_area),
                                      HANDLE_SEAL);
        if (p_manager->internal.p_seal == NULL)
        {
            NRF_MESH_ERROR_CHECK(recover_seal(p_manager));
        }

        p_manager->internal.state = FM_STATE_READY;
        p_manager->internal.invalid_bytes = 0;
    }
    m_state = FM_STATE_READY;
    mesh_flash_user_callback_set(MESH_FLASH_USER_MESH, flash_op_ended_callback);
    schedule_processing();
}

const void * flash_manager_recovery_page_get(void)
{
    return flash_manager_defrag_recovery_page_get();
}

void flash_manager_action_queue_empty_cb_set(flash_manager_queue_empty_cb_t queue_empty_cb)
{
    if (queue_empty_cb != NULL)
    {
        NRF_MESH_ASSERT(m_queue_empty_cb == NULL);
        m_queue_empty_cb = queue_empty_cb;
    }
    else
    {
        m_queue_empty_cb = NULL;
    }
}

bool flash_manager_is_building(flash_manager_t * p_manager)
{
    return (p_manager->internal.state == FM_STATE_BUILDING);
}

bool flash_manager_is_removing(flash_manager_t * p_manager)
{
    return (p_manager->internal.state == FM_STATE_REMOVING);
}
