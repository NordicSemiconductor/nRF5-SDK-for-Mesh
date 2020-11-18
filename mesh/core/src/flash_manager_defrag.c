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

/* The flash manager defrag procedure defragments a flash manager area, by stripping away
 * all invalidated entries, and rewriting the area with only the valid entries present. This lets
 * us add more entries to the area, as we free up dead space. The defrag procedure is triggered
 * automatically by the flash manager when it approaches its space limit. In order to do this
 * without losing data on power failure, there must always be a copy of every entry stored in flash.
 * The defrag procedure utilizes a recovery page to achieve this, taking a copy of all the valid
 * entries on the original page before erasing and rewriting the original.
 *
 * The defrag procedure is a linear set of procedure steps, that we run through on every page
 * of the flash area being defragged. The procedure consists of the following steps:
 *
 * 1. Check for invalid entries: Run through the page, and make sure there's at least one invalid
 *    entry. If not, skip the page, as there's nothing we can do to improve it.
 * 2. Erase recovery area: There might be garbage data in the recovery area, so we erase it before
 *    moving on.
 * 3. Copy metadata: Each flash manager page starts with a metadata header. We start by copying
 *    this over to the recovery area.
 * 4. Backup entries: Iterates through the target page, and copies all valid entries over into the
 *    recovery area, until we've either copied all the entries, or the recovery area is full.
 * 5. Write defrag start pointer: Write a pointer to the page we're defragging to the recovery
 *    area. This pointer acts as the "seal" of the recovery area, marking that it contains a valid
 *    backup of the area it's pointing to. If the device boots up and finds a valid pointer in the
 *    recovery page, it'll resume the defrag process from the next step, as it indicates that we
 *    lost power in the middle of the procedure. We'll erase this pointer at the end of the
 *    procedure.
 * 6. Erase source: Erase the source page to prepare for write back.
 * 7. Write back: Write the contents of the recovery area back to the target page.
 * 8. Invalidate duplicate entries: While taking a copy of the target page, we might have ran over
 *    to the next page, and copied those entries too. If this is the case, those entries would have
 *    been duplicated when we copied it back in step 7. Invalidate all entries that are present
 *    both in the recovery area, and the pages after the target page.
 * 9. Seal storage page: Put an "end" entry at the end of the storage page, to indicate that there
 *    are no more entries in this page. If the previous step found that there will be more entries
 *    after the end, we'll pad the page, otherwise, we'll mark it as the end of the area.
 * 10. Post process: Cleanup our state, and move on to the next page. If there are no more pages to
 *    backup, we erase the defrag start pointer from the recovery area, and end the procedure.
 *
 * Each procedure step is implemented as a single function that returns whether the procedure
 * should continue, attempt to re-run the step, finish or restart. This allows us to resume the
 * procedure in a clean way if any of the flash functions were to run out of queue space, which is
 * quite likely during a normal defrag procedure with a lot of flash operations.
 */

#include "flash_manager_defrag.h"

#include <string.h>
#include "flash_manager_internal.h"
#include "hal.h"
#include "utils.h"
#include "internal_event.h"
#include "log.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define DEFRAG_RECOVER_STEP (erase_source)
#define MAX_PROCEDURE_STEP (ARRAY_SIZE(m_procedure_steps))

/*****************************************************************************
* Local typedefs
*****************************************************************************/
/** Global defrag states */
typedef enum
{
    DEFRAG_STATE_IDLE, /**< No defrag is currently in progress. */
    DEFRAG_STATE_PROCESSING, /**< Currently in the middle of a defrag procedure. */
    DEFRAG_STATE_STABILIZING /**< Stabilizing flash contents after a finished defrag procedure. */
} defrag_state_t;

/** Procedure step end action. Returned by each procedure step to indicate what the next step in
 *  the procedure should be. */
typedef enum
{
    PROCEDURE_STAY,     /**< Wait for a flash operation to finish, and repeat the current step. */
    PROCEDURE_CONTINUE, /**< Move on to the next step in the procedure. */
    PROCEDURE_END,      /**< End the procedure. */
    PROCEDURE_RESTART,  /**< Start the procedure from the beginning. */
} procedure_action_t;

typedef struct
{
    defrag_state_t state;
    uint32_t step; /**< Current step number in the backup procedure. */
    const flash_manager_t * p_manager; /**< Flash manager owning the page currently being defragged. */
    const flash_manager_page_t * p_storage_page; /**< Page being defragmented. */
    const fm_entry_t * p_src; /**< Next entry to copy */
    const fm_entry_t * p_dst; /**< Next destination in recovery page. */
    bool wait_for_idle;       /**< Flag, that when set makes the procedure wait for all flash operations to end before proceeding. */
    bool found_all_entries;   /**< Whether we've ran through all entries in the original area. */
} defrag_t;

/** Single chunk of entries. */
typedef struct
{
    const fm_entry_t * p_start; /**< First entry in the chunk. */
    uint32_t length; /**< Length of the chunk in bytes. */
} fm_entry_chunk_t;

typedef procedure_action_t (*defrag_procedure_step_t)(void);

/*****************************************************************************
* Static globals
*****************************************************************************/
static flash_manager_recovery_area_t * mp_recovery_area; /**< Recovery area pointer into flash. */
static defrag_t m_defrag; /**< Global defrag state. */
static uint16_t m_token; /**< Flash operation token returned from the mesh flash module. */
static bool m_is_frozen; /**< Is defragmentation functionality frozen or not. */

/* We're iterating through pages with the assumption that one flash_manager_page_t and
 * flash_manager_recovery_area_t are exactly one page long, and that fm_entry_t is exactly one word.
 * Ensure these assumptions are true: */
NRF_MESH_STATIC_ASSERT(sizeof(flash_manager_page_t) == PAGE_SIZE);
NRF_MESH_STATIC_ASSERT(sizeof(flash_manager_recovery_area_t) == PAGE_SIZE);
NRF_MESH_STATIC_ASSERT(sizeof(fm_entry_t) == WORD_SIZE);

/*****************************************************************************
* Local utility functions
*****************************************************************************/
/**
 * Get the largest possible continuous chunk of data entries starting at the first entry
 * representing data after the @p p_src pointer.
 *
 * @param[out] Chunk object to populate.
 * @param[in] p_src Entry to start looking for a chunk from.
 * @param[in] p_end The upper boundary of memory to look through.
 * @param[in] max_length The maximum length of the chunk. The resulting chunk will never exceed
 * this length.
 *
 * @returns The first entry after the chunk, or NULL if there were no more valid data entries
 * before meeting the boundary conditions.
 */
static inline const fm_entry_t * get_chunk_of_data_entries(fm_entry_chunk_t * p_chunk,
                                                           const fm_entry_t * p_src,
                                                           const void *       p_end,
                                                           uint32_t           max_length)
{
    p_chunk->length = 0;
    /* find start of chunk */
    if (!handle_represents_data(p_src->header.handle))
    {
        p_src = get_next_data_entry(p_src, p_end);
        if (p_src == NULL)
        {
            p_chunk->p_start = NULL;
            return NULL;
        }
    }
    p_chunk->p_start = p_src;

    /* find end of chunk */
    while (((const void *) p_src < p_end) &&
           (p_chunk->length + p_src->header.len_words * WORD_SIZE < max_length) &&
           (PAGE_START_ALIGN(p_src) == PAGE_START_ALIGN(p_chunk->p_start)) &&
           handle_represents_data(p_src->header.handle))
    {
        /* include p_src in the chunk */
        p_chunk->length += p_src->header.len_words * WORD_SIZE;

        p_src = get_next_entry(p_src);
    }
    return p_src;
}

/*****************************************************************************
* Defrag procedure m_procedure_steps
*****************************************************************************/
static procedure_action_t check_for_invalid_entries(void)
{
    /* If we can't find any invalid entries in this page, we should skip it. */
    if (entry_get(get_first_entry(m_defrag.p_storage_page),
                  m_defrag.p_storage_page + 1,
                  FLASH_MANAGER_HANDLE_INVALID) == NULL)
    {
        if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
        {
            return PROCEDURE_END;
        }
        else
        {
            /* Jump to the next page */
            m_defrag.p_storage_page++;
            return PROCEDURE_RESTART;
        }
    }
    else
    {
        return PROCEDURE_CONTINUE;
    }
}

static procedure_action_t erase_recovery_area(void)
{
    if (erase(mp_recovery_area, PAGE_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t copy_metadata(void)
{
    uint32_t metadata_length = m_defrag.p_storage_page->metadata.metadata_len;
    if (flash(&mp_recovery_area->data[0], m_defrag.p_storage_page, metadata_length, &m_token) == NRF_SUCCESS)
    {
        /* ready to start copying entries */
        m_defrag.p_dst = (const fm_entry_t *) &mp_recovery_area->data[metadata_length / sizeof(mp_recovery_area->data[0])];
        m_defrag.p_src = get_first_entry(m_defrag.p_storage_page);
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

/**
 * Copy valid handles from storage area to backup page.
 */
static procedure_action_t backup_entries(void)
{
    NRF_MESH_ASSERT(m_defrag.p_src != NULL);
    NRF_MESH_ASSERT(m_defrag.p_dst != NULL);

    bool recovery_page_is_full = false;
    /* Copy entries in chunks of sequential data-entries until we get to the end or are unable to
       fit any more in the recovery page. */
    while (!recovery_page_is_full)
    {
        fm_entry_chunk_t chunk;
        uint32_t remaining_space = (PAGE_START_ALIGN(m_defrag.p_dst) + PAGE_SIZE) - (uint32_t) m_defrag.p_dst;
        const fm_entry_t * p_next =
            get_chunk_of_data_entries(&chunk,
                                      m_defrag.p_src,
                                      get_area_end(m_defrag.p_storage_page),
                                      remaining_space);

        if (chunk.length > 0)
        {
            if (NRF_SUCCESS == flash(m_defrag.p_dst, chunk.p_start, chunk.length, &m_token))
            {
                m_defrag.p_dst += (chunk.length / sizeof(fm_entry_t));
                NRF_MESH_ASSERT(p_next != NULL);
                m_defrag.p_src = p_next;
            }
            else
            {
                /* Early return */
                return PROCEDURE_STAY;
            }
        }
        else
        {
            recovery_page_is_full = true;
        }
        /* Getting a NULL return from the chunk getter means there were no more valid data chunks left. */
        if (p_next == NULL || p_next->header.handle == HANDLE_SEAL)
        {
            break;
        }
    }

    return PROCEDURE_CONTINUE;
}


static procedure_action_t write_defrag_start_pointer(void)
{
    if (flash(&mp_recovery_area->p_storage_page, (void *) &m_defrag.p_storage_page, WORD_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t erase_source(void)
{
    if (erase(m_defrag.p_storage_page, PAGE_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t write_back(void)
{
    if (flash(m_defrag.p_storage_page, mp_recovery_area->data, sizeof(mp_recovery_area->data), &m_token) == NRF_SUCCESS)
    {
        /* In the next step, we'll iterate through the memory we write here, so we need this entire
           flash operation to finish before moving on: */
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

/**
 * Step through the entries on the page AFTER the one we're recovering, and
 * remove any entries that also occur in the backup, as these are duplicates.
 */
static procedure_action_t invalidate_duplicate_entries(void)
{
    if (m_defrag.p_storage_page->metadata.page_index ==
        m_defrag.p_storage_page->metadata.pages_in_area - 1)
    {
        /* The current page was the last page in the area, and there won't be any duplicates. */
        m_defrag.found_all_entries = true;
        return PROCEDURE_CONTINUE;
    }

    const fm_entry_t * p_area_entry = get_first_entry(m_defrag.p_storage_page + 1);
    const fm_entry_t * p_recovery_entry =
        get_first_entry((const flash_manager_page_t *) mp_recovery_area->data);
    const fm_entry_t * p_end = (const fm_entry_t *) (get_area_end(m_defrag.p_storage_page));

    if (!handle_represents_data(p_area_entry->header.handle))
    {
        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }

    while (p_area_entry != NULL)
    {
        p_recovery_entry =
            entry_get(p_recovery_entry, mp_recovery_area + 1, p_area_entry->header.handle);
        if (p_recovery_entry == NULL)
        {
            /* Since the entries would have been added to the recovery area in-order, we can assume
               that once we find an entry that's not duplicated in the recovery area, we won't find
               any more duplicates. */
            break;
        }

        /* Wait for all invalidations to finish before moving on to the next step in the procedure,
           as they will alter the way we'll traverse the page on the next round: */
        m_defrag.wait_for_idle = true;

        if (flash(p_area_entry, &INVALID_HEADER, sizeof(INVALID_HEADER), &m_token) !=
            NRF_SUCCESS)
        {
            return PROCEDURE_STAY;
        }

        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }

    /* If we ran out of recovery entries before running out of area entries, the recovery
     * doesn't contain all remaining entries. */
    m_defrag.found_all_entries = (p_area_entry == NULL);

    return PROCEDURE_CONTINUE;
}

/**
 * Write a seal or padding entry at the end of the current storage page.
 */
static procedure_action_t seal_storage_page(void)
{
    /* Put seal at the first blank header: */
    const fm_entry_t * p_seal_location = entry_get(get_first_entry(m_defrag.p_storage_page),
                                                   m_defrag.p_storage_page + 1,
                                                   HANDLE_BLANK);
    if (p_seal_location == NULL || p_seal_location == get_first_entry(m_defrag.p_storage_page))
    {
        /* Don't seal if the target page is either completely full or completely empty. */
        return PROCEDURE_CONTINUE;
    }

    const fm_header_t * p_header;
    if (m_defrag.found_all_entries)
    {
        p_header = &SEAL_HEADER;
    }
    else
    {
        p_header = &PADDING_HEADER;
    }

    if (flash(p_seal_location, p_header, sizeof(fm_header_t), &m_token) == NRF_SUCCESS)
    {
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

static procedure_action_t post_process(void)
{
    if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
    {
        /* Invalidate area pointer */
        static const uint32_t * p_null_ptr = NULL;
        if (flash(&mp_recovery_area->p_storage_page, &p_null_ptr, sizeof(p_null_ptr), &m_token) == NRF_SUCCESS)
        {
            m_defrag.wait_for_idle = true;
            return PROCEDURE_END;
        }
        else
        {
            return PROCEDURE_STAY;
        }
    }
    else
    {
        /** Start the procedure from the beginning, operating on the next page in the area. */
        m_defrag.p_storage_page++;
        return PROCEDURE_RESTART;
    }
}

/* This is state out of the regular defragmentation state machine. Normal defragmentation never comes here.
 * This state is required only to freeze defragmentation immediately in case of power down situation.
 * All requests to defragmentation state machine will be redirected to the end immediately.
 * Hence the already planned flash manager actions that require defragmentation
 * will be completed with error status FM_RESULT_ERROR_AREA_FULL. */
static procedure_action_t freezing(void)
{
    m_defrag.wait_for_idle = false;
    m_defrag.p_manager = NULL;
    return PROCEDURE_END;
}

static const defrag_procedure_step_t m_procedure_steps[] =
{
    check_for_invalid_entries,
    erase_recovery_area,
    copy_metadata,
    backup_entries,
    write_defrag_start_pointer,
    erase_source,
    write_back,
    invalidate_duplicate_entries,
    seal_storage_page,
    post_process,
    freezing
};
/*****************************************************************************
* Static functions
*****************************************************************************/
static void defrag_on_end(void)
{
    const flash_manager_t * p_manager = m_defrag.p_manager;
    memset(&m_defrag, 0, sizeof(m_defrag));
    flash_manager_on_defrag_end((flash_manager_t *) p_manager);
}

static void jump_to_step(defrag_procedure_step_t step)
{
    for (uint32_t i = 0; i < MAX_PROCEDURE_STEP; i++)
    {
        if (m_procedure_steps[i] == step)
        {
            m_defrag.step = i;
            return;
        }
    }
    /* Couldn't find step */
    NRF_MESH_ASSERT(false);
}

static void execute_procedure_step(void)
{
    /* Verify the validity of the state */
    NRF_MESH_ASSERT(m_defrag.p_storage_page != NULL);
    NRF_MESH_ASSERT(!m_defrag.wait_for_idle);
    NRF_MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
    procedure_action_t action;

    do
    {
        action = m_procedure_steps[m_defrag.step]();
        switch (action)
        {
            case PROCEDURE_STAY:
                break;
            case PROCEDURE_CONTINUE:
                m_defrag.step++;
                NRF_MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
                break;
            case PROCEDURE_RESTART:
                m_defrag.step = 0;
                break;
            case PROCEDURE_END:
                if (m_defrag.wait_for_idle)
                {
                    m_defrag.state = DEFRAG_STATE_STABILIZING;
                }
                else
                {
                    defrag_on_end();
                }
                break;
        }
    } while ((action == PROCEDURE_CONTINUE || action == PROCEDURE_RESTART) &&
             !m_defrag.wait_for_idle);
}

static void on_flash_op_end(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    /*lint -esym(715, user, token) Ignore unused tokens */

    if (p_op->type == FLASH_OP_TYPE_ALL)
    {
        m_defrag.wait_for_idle = false;
    }

    if (m_defrag.state == DEFRAG_STATE_PROCESSING)
    {
        if (!m_defrag.wait_for_idle)
        {
            execute_procedure_step();
        }
    }
    else if (m_defrag.state == DEFRAG_STATE_STABILIZING && p_op->type == FLASH_OP_TYPE_ALL)
    {
        defrag_on_end();
    }
}

/**
 * Checks if a defrag was interrupted by a power cycle, and continues where it left off.
 *
 * @returns Whether there's a defrag in progress.
 */
static bool recover_defrag_progress(void)
{
    if (mp_recovery_area->p_storage_page != NULL &&
        mp_recovery_area->p_storage_page != (void *) BLANK_FLASH_WORD &&
        IS_PAGE_ALIGNED(mp_recovery_area->p_storage_page))
    {
        m_defrag.p_storage_page = mp_recovery_area->p_storage_page;
        m_defrag.wait_for_idle = false;
        m_defrag.found_all_entries = false;
        m_defrag.state = DEFRAG_STATE_PROCESSING;
        m_defrag.p_manager = NULL; /* Can't know which manager this is. */
        jump_to_step(DEFRAG_RECOVER_STEP);
        mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);
        execute_procedure_step();
        return true;
    }
    else
    {
        return false;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

bool flash_manager_defrag_init(void)
{
    m_is_frozen = false;
#ifdef FLASH_MANAGER_RECOVERY_PAGE
    mp_recovery_area = (flash_manager_recovery_area_t *) FLASH_MANAGER_RECOVERY_PAGE;
#else
    flash_manager_recovery_area_t * p_flash_end;
    if (BOOTLOADERADDR() != BLANK_FLASH_WORD &&
        BOOTLOADERADDR() != 0)
    {
#if NRF52_SERIES
        /* The 52-series SoftDevice's MBR needs an extra page for bootloader and SoftDevice DFU-ing. */
        p_flash_end = (flash_manager_recovery_area_t *) (BOOTLOADERADDR() - PAGE_SIZE);
#else
        p_flash_end = (flash_manager_recovery_area_t *) BOOTLOADERADDR();
#endif
    }
    else
    {
        p_flash_end = (flash_manager_recovery_area_t *) DEVICE_FLASH_END_GET();
    }
    /* Recovery area is last page of application controlled flash */
    mp_recovery_area = p_flash_end - FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES - 1; /* pointer arithmetic */
#endif

    __LOG(LOG_SRC_FM, LOG_LEVEL_DBG3, "BOOTLOADERADDR(): 0x%08x fm_recovery_area: 0x%08x\n",
          BOOTLOADERADDR(), mp_recovery_area);
    return recover_defrag_progress();
}

bool flash_manager_defragging(const flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    return (m_defrag.state != DEFRAG_STATE_IDLE &&
            m_defrag.p_storage_page >= get_first_page(p_manager->config.p_area) &&
            m_defrag.p_storage_page <= get_last_page(p_manager->config.p_area));
}

bool flash_manager_defrag_is_running(void)
{
    return (m_defrag.state != DEFRAG_STATE_IDLE);
}

void flash_manager_defrag(const flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT(m_defrag.state == DEFRAG_STATE_IDLE);
    NRF_MESH_ASSERT(p_manager->internal.state == FM_STATE_DEFRAG);

    m_defrag.p_manager = p_manager;
    m_defrag.p_storage_page = p_manager->config.p_area;
    m_defrag.step = m_is_frozen ? MAX_PROCEDURE_STEP - 1 : 0;
    m_defrag.wait_for_idle = false;
    m_defrag.state = DEFRAG_STATE_PROCESSING;
    m_defrag.found_all_entries = false;

    mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);

    execute_procedure_step();
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FM_DEFRAG, 0, 0, NULL);
}

const void * flash_manager_defrag_recovery_page_get(void)
{
    return mp_recovery_area;
}

void flash_manager_defrag_freeze(void)
{
    m_is_frozen = true;
    jump_to_step(freezing);
}

#ifdef UNIT_TEST
void flash_manager_defrag_reset(void)
{
    memset((uint8_t*)&m_defrag, 0, sizeof(m_defrag));
    mp_recovery_area = NULL;
    m_token = 0;
    m_is_frozen = false;
}
#endif
