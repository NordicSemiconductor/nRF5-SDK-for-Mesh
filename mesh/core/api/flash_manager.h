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
#ifndef FLASH_MANAGER_H__
#define FLASH_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet_buffer.h"
#include "nrf_mesh_config_core.h"
#include "queue.h"
#include "toolchain.h"
#include "bearer_event.h"

/**
 * @defgroup FLASH_MANAGER Flash Manager
 * @ingroup NRF_MESH
 * The Flash Manager provides an entry-based persistent storage interface that works with the mesh.
 * The Flash Manager is used to store mesh data, but can also be used by the application.
 *
 * For more information about the Flash Manager, see the @ref md_doc_user_guide_modules_flash_manager library document.
 * @{
 */

/**
 * @defgroup FLASH_MANAGER_DEFINES Defines
 * Flash Manager definitions.
 * @{
 */

/** Number of bytes of data one could fit on a single flash page */
#define FLASH_MANAGER_DATA_PER_PAGE (PAGE_SIZE - sizeof(flash_manager_metadata_t))

/** Minimum number of pages required to store the given amount of data.
 *
 * @note As we'll pad a page if the current page can't fit a new entry, we have to assume the worst
 * case in this calculation, which is that the largest possible entry is added at the end of every
 * page without fitting.
 */
#define FLASH_MANAGER_PAGE_COUNT_MINIMUM(DATA_SIZE, LARGEST_ENTRY_SIZE) (1 + ((DATA_SIZE) / (FLASH_MANAGER_DATA_PER_PAGE - LARGEST_ENTRY_SIZE)))

#define FLASH_MANAGER_HANDLE_MAX        (0x7EFF) /**< Highest handle number allowed. */
#define FLASH_MANAGER_HANDLE_INVALID    (0x0000) /**< Invalid handle. */

#define FLASH_MANAGER_ENTRY_LEN_OVERHEAD (sizeof(fm_header_t) / WORD_SIZE) /**< Overhead in each entry's len field for the header. */

/** @} */

/**
 * @defgroup FLASH_MANAGER_TYPES Types
 * Flash Manager type definitions.
 * @{
 */

/** Flash manager handle type, used to identify entries in a flash manager area. */
typedef uint16_t fm_handle_t;

/** Header prepending every flash entry in a flash manager. */
typedef struct
{
    uint16_t len_words; /**< Length of entry in words, including header. */
    fm_handle_t handle; /**< Entry handle. */
} fm_header_t;

/**
 * Handle filter, used to search through all handles to find matches.
 * Matches any handle that satisfies the following condition: ((handle & mask) == (match & mask))
 */
typedef struct
{
    fm_handle_t mask;  /**< Mask to apply to the match value when comparing with a handle. */
    fm_handle_t match; /**< Value that the masked portion of the handle has to match. */
} fm_handle_filter_t;

/** Single flash manager entry */
typedef struct
{
    fm_header_t header;
    uint32_t data[];
} fm_entry_t;

/** Valid state of a flash manager instance. */
typedef enum
{
    FM_STATE_UNINITIALIZED,
    FM_STATE_BUILDING,
    FM_STATE_READY,
    FM_STATE_DEFRAG,
    FM_STATE_REMOVING
} fm_state_t;

/** Metadata structure denoting properties of a flash manager page. */
typedef struct
{
    uint8_t metadata_len; /**< Length of manager metadata in bytes. */
    uint8_t entry_header_length; /**< Length of each entry header in bytes. */
    uint8_t entry_len_length_bits;  /**< Length of entry len field in bits. */
    uint8_t entry_type_length_bits; /**< Length of entry type field in bits. */

    uint8_t pages_in_area; /**< Total number of pages in this area. */
    uint8_t page_index;    /**< Index of this page in its area. */
    uint16_t _padding; /**< @internal Unused padding. */
} flash_manager_metadata_t;

/** Single flash manager page */
typedef union
{
    flash_manager_metadata_t metadata; /**< Metadata at the start of every managed page. */
    uint8_t raw[PAGE_SIZE]; /**< Raw representation of the full page. */
} flash_manager_page_t;

typedef struct flash_manager flash_manager_t;

/** Flash action result, returned in complete-callback. */
typedef enum
{
    FM_RESULT_SUCCESS, /**< The action completed successfully. */
    FM_RESULT_ERROR_AREA_FULL, /**< The managed flash area is full, and cannot accept any new entries. */
    FM_RESULT_ERROR_NOT_FOUND, /**< The entry wasn't present in the manager. */
    FM_RESULT_ERROR_FLASH_MALFUNCTION /**< The flash HW malfunctioned, and the operation did not finish correctly. */
} fm_result_t;

/**
 * Write complete callback, to give the user the result of an entry write action.  Is called when
 * the action is fully completed, but before any memory associated with it is freed.
 *
 * @warning If the result parameter indicates failure, the memory pointed to in @p p_entry is an
 * in-RAM representation of the requested write, that will be freed after the call completes.
 *
 * @param[in] p_manager Manager that executed the action.
 * @param[in] p_entry Pointer to the resulting entry in flash if the action was successful, or
 * pointer to the in-RAM representation if the action failed.
 * @param[in] result Result of the action.
 */
typedef void (*flash_manager_write_complete_cb_t)(const flash_manager_t * p_manager,
        const fm_entry_t * p_entry,
        fm_result_t result);

/**
 * Invalidate complete callback, to give the user the result of an entry invalidation action. Is
 * called when the action is fully completed, but before any memory associated with it is freed.
 *
 * @param[in] p_manager Manager that executed the action.
 * @param[in] handle Handle that was invalidated.
 * @param[in] result Result of the action.
 */
typedef void (*flash_manager_invalidate_complete_cb_t)(const flash_manager_t * p_manager,
        fm_handle_t handle,
        fm_result_t result);

/**
 * Remove complete callback, indicating that the requested remove operation has been completed.
 *
 * Any flash memory associated with the given manager has been erased, and the manager and its flash
 * area can safely be reused.
 *
 * @param[in] p_manager Flash manager that was removed.
 */
typedef void (*flash_manager_remove_complete_cb_t)(const flash_manager_t * p_manager);

/**
 * Action queue empty callback, indicating that the flash manager is finished processing all its
 * ongoing actions.
 */
typedef void (*flash_manager_queue_empty_cb_t)(void);

/**
 * Flash manager configuration structure, defines the user-configurable parts of the flash manager.
 */
typedef struct
{
    const flash_manager_page_t *           p_area;                 /**< Start of area owned by this flash manager. */
    uint32_t                               page_count;             /**< Number of pages in the area. */
    uint32_t                               min_available_space;    /**< Number of bytes that should always be left available during normal operation.
                                                                        Once the manager has this many bytes or less left, it'll start defragmentation. */
    flash_manager_write_complete_cb_t      write_complete_cb;      /**< Callback called after every completed write action, or @c NULL. */
    flash_manager_invalidate_complete_cb_t invalidate_complete_cb; /**< Callback called after every completed entry invalidation, or @c NULL. */
    flash_manager_remove_complete_cb_t     remove_complete_cb;     /**< Callback called after the manager has been successfully removed. */
} flash_manager_config_t;

/** Internal flash manager state, managed and used internally. */
typedef struct
{
    fm_state_t state;          /**< State of the manager. */
    uint32_t invalid_bytes;    /**< Bytes invalidated in the area. */
    const fm_entry_t * p_seal; /**< Pointer to the seal entry. */
} flash_manager_internal_state_t;

struct flash_manager
{
    flash_manager_internal_state_t internal; /**< Internal run-time state, that shouldn't be altered by the user. */
    flash_manager_config_t config;        /**< Manager configuration, as set by the user. */
};

/**
 * Memory listener callback, that will be called once the flash manager has some memory available.
 *
 * @param[in] p_args Arguments pointer, as set by the listener.
 */
typedef void (*flash_manager_mem_listener_cb_t)(void * p_args);

/** Memory listener. */
typedef struct
{
    queue_elem_t queue_elem; /**< Used for linked list operation, should not be altered by the user. */
    flash_manager_mem_listener_cb_t callback; /**< Callback to call when there's memory available. */
    void * p_args; /**< Arguments pointer, set by the user and returned in the callback. */
} fm_mem_listener_t;

/** Action returned from the read callback, determining whether to continue the iteration. */
typedef enum
{
    FM_ITERATE_ACTION_STOP, /**< Stop iterating through the entries. */
    FM_ITERATE_ACTION_CONTINUE, /**< Continue iterating through the entries. */
} fm_iterate_action_t;

/**
 * Entry read callback type.
 *
 * @see flash_manager_entries_read
 *
 * @param[in] p_entry A pointer to the flash entry that is read.
 * @param[in,out] p_args The @c p_args pointer that was passed to the @ref
 * flash_manager_entries_read function.
 *
 * @retval FM_ITERATE_ACTION_STOP Tell the caller to stop iterating through the entries.
 * @retval FM_ITERATE_ACTION_CONTINUE Tell the caller to continue iterating through the entries.
 */
typedef fm_iterate_action_t (*flash_manager_read_cb_t)(const fm_entry_t * p_entry, void * p_args);

/** @} */

/**
 * Initialize the flash manager.
 */
void flash_manager_init(void);

/**
 * Add a flash manager instance.
 *
 * @warning Adding a manager that's already in use may result in unwanted behavior.
 *
 * @note Two managers cannot have overlapping regions, and adding a flash manager area that's on top
 * of another will trigger an assert.
 *
 * @param[in,out] p_manager Flash manager to initialize.
 * @param[in] p_config Configuration parameters to use for the flash manager.
 *
 * @retval NRF_SUCCESS The manager was added successfully.
 * @retval NRF_ERROR_NO_MEM The internal buffer ran out of space.
 */
uint32_t flash_manager_add(flash_manager_t * p_manager,
        const flash_manager_config_t * p_config);

/**
 * Remove the given flash manager, and erase all of its contents.
 *
 * All entries known to this manager will become permanently inaccessible and forgotten. If set, the
 * managers remove_complete_cb (@ref flash_manager_remove_complete_cb_t) will be called at the end
 * of this operation. Only at that point will it be safe to reuse the flash area, or re-add the
 * flash manager.
 *
 * @param[in,out] p_manager         Flash manager to remove.
 *
 * @retval        NRF_SUCCESS             The given manager has successfully been scheduled for wiping.
 * @retval        NRF_ERROR_NO_MEM        Not enough memory to schedule the action.
 * @retval        NRF_ERROR_INVALID_STATE The flash manager was not in the ready or uninitialized state,
 * and cannot be removed.
 */
uint32_t flash_manager_remove(flash_manager_t * p_manager);

/**
 * Get a pointer to the entry with the given index.
 *
 * @deprecated Deprecated in favor of @ref flash_manager_entry_read, which works in the defrag state.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] handle Entry handle to search for.
 *
 * @returns The flash entry with the given index, or NULL if no such entry exists.
 */
_DEPRECATED const fm_entry_t * flash_manager_entry_get(const flash_manager_t * p_manager, fm_handle_t handle);

/**
 * Read out the contents of the entry with the given handle.
 *
 * Will copy the entry into the @p p_data buffer, up to the size of the buffer.
 *
 * @note The entry length is stored in words, so entries whose size is not word-aligned will get
 * their @p p_length parameter adjusted even if they are technically the right size. If the there's
 * a mismatch between @p p_length and the stored length that can't have been caused by the word
 * alignment, the function will return @c NRF_ERROR_INVALID_LENGTH.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] handle Entry handle to search for.
 * @param[in,out] p_data Buffer to copy entry data into. The buffer must at least be as long as
 * indicated by the initial value of @c p_length.
 * @param[in,out] p_length Pointer to a variable that holds the length of the given entry in bytes.
 * Will be changed to reflect the actual length of the entry with the given handle.
 *
 * @retval NRF_SUCCESS The entry was found, its data was successfully copied into the @p p_data
 * buffer, and @p p_length was changed to reflect the actual length.
 * @retval NRF_ERROR_INVALID_STATE The flash manager is not in a readable state.
 * @retval NRF_ERROR_NULL At least one parameter was NULL.
 * @retval NRF_ERROR_INVALID_PARAMETER The given handle isn't valid.
 * @retval NRF_ERROR_NOT_FOUND The given handle doesn't exist in the manager.
 * @retval NRF_ERROR_INVALID_LENGTH The word aligned value of @p p_length does not match the entry
 * length. @p p_length has been changed to indicate the correct length.
 */
uint32_t flash_manager_entry_read(const flash_manager_t * p_manager,
                                  fm_handle_t handle,
                                  void * p_data,
                                  uint32_t * p_length);

/**
 * Get the next entry matching the given filter.
 *
 * @note The entries will be returned in the order they're stored in, not by index. It's
 * recommended to reset the @p p_start pointer if any changes are made to the area.
 *
 * @deprecated Deprecated in favor of @ref flash_manager_entries_read, which works in the defrag state.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] p_filter Filter to apply to search, or NULL if not filter should be applied.
 * @param[in] p_start Entry to start the search from, or @c NULL if starting from the beginning.
 * This argument acts as an iterator token, allowing the user to get new entries. As long as the
 * area is unchanged, passing the same @p p_start pointer will always return the same entry. To
 * iterate through all entries, keep passing the pointer returned by the previous call.
 *
 * @returns The next entry from @p p_start matching the given filter, or NULL if no such entry exists
 * in the given flash manager.
 */
_DEPRECATED const fm_entry_t * flash_manager_entry_next_get(const flash_manager_t * p_manager,
        const fm_handle_filter_t * p_filter,
        const fm_entry_t * p_start);


/**
 * Read out flash manager entries.
 *
 * Iterates through the given flash manager's area and passes each entry to the read callback.
 * Freezes flash operations to avoid manipulating the data through defrag or other procedures.
 *
 * @note The order in which the entries are passed to the read callback isn't guaranteed to be the
 * same as the order they're stored in.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] p_filter Filter to apply to search, or @c NULL to pass all entries to the read callback.
 * @param[in] read_cb Read callback called for every entry that fits the filter, or @c NULL.
 * @param[in,out] p_args Argument pointer that will be passed to the read callback.
 *
 * @returns The number of entries read out to the read_cb.
 */
uint32_t flash_manager_entries_read(const flash_manager_t * p_manager,
                                    const fm_handle_filter_t * p_filter,
                                    flash_manager_read_cb_t read_cb,
                                    void * p_args);

/**
 * Get the number of entries matching the given filter.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] p_filter Filter to apply to search, or @c NULL to count all entries.
 *
 * @returns The number of entries matching the given filter.
 */
uint32_t flash_manager_entry_count_get(const flash_manager_t * p_manager, const fm_handle_filter_t * p_filter);

/**
 * Allocate a buffer for a flash entry write. Ensures that there's enough space in the manager's
 * flash-area, and reserves a buffer in the process queue.
 *
 * @note The entry length will be rounded up to the nearest word, and any data in the padding may
 * have garbage values.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] handle Entry handle.
 * @param[in] data_length Wanted length of the entry in bytes, excluding the header. Cannot be
 * longer than @ref FLASH_MANAGER_ENTRY_MAX_SIZE.
 *
 * @returns A pointer to a reserved entry in the manager's write queue, that may be committed once
 * the data and handle fields have been filled. The entry length field is prefilled, and shouldn't
 * be altered.
 * @returns @c NULL if no space is available in the process queue.
 */
fm_entry_t * flash_manager_entry_alloc(flash_manager_t * p_manager,
        fm_handle_t handle,
        uint32_t data_length);

/**
 * Commit the given write buffer for flashing.
 *
 * @warning Will assert if p_entry is not a valid entry obtained through @ref
 * flash_manager_entry_alloc.
 *
 * @param[in] p_entry Write buffer allocated through @ref flash_manager_entry_alloc.
 */
void flash_manager_entry_commit(const fm_entry_t * p_entry);

/**
 * Invalidate the entry with the given handle.
 *
 * @param[in] p_manager Flash manager to operate on.
 * @param[in] handle Handle to invalidate.
 *
 * @retval NRF_SUCCESS The entry has successfully been scheduled for invalidation.
 * @retval NRF_ERROR_NO_MEM There's not enough space available in the process queue.
 */
uint32_t flash_manager_entry_invalidate(flash_manager_t * p_manager, fm_handle_t handle);

/**
 * Release an allocated entry buffer that won't be committed after all.
 *
 * @param[in] p_entry Entry to release.
 */
void flash_manager_entry_release(fm_entry_t * p_entry);

/**
 * Register a call back to be notified once memory has been made available in the internal buffer.
 * This can be used to recover from @c NRF_ERROR_NO_MEM errors from the @ref
 * flash_manager_entry_alloc, @ref flash_manager_remove, and @ref flash_manager_entry_invalidate
 * functions. Once called, the listener may re-attempt its allocation call, and if neccessary, call
 * this function again, to get called the next time there's some more memory available.
 *
 * @param[in]  p_listener  Listener to register. Neither the listener or its callback can be NULL.
 *                         The listener memory must be valid until it's called again.
 */
void flash_manager_mem_listener_register(fm_mem_listener_t * p_listener);

/**
 * Checks whether the module is in the progress of flashing anything.
 *
 * @retval     true   if the flash manager's write queue is empty.
 * @retval     false  if the flash manager has some unfinished write operations.
 */
bool flash_manager_is_stable(void);

/**
 * Sets a function to call when the flash manager's action queue is empty.
 *
 * @warning If there is already a callback registered, this function will assert.
 *
 * @param[in] queue_empty_cb Queue empty callback function or @c NULL to de-register the active
 *                           callback.
 */
void flash_manager_action_queue_empty_cb_set(flash_manager_queue_empty_cb_t queue_empty_cb);

/**
 * Get the address of the recovery page.
 *
 * @returns A pointer to the recovery page.
 */
const void * flash_manager_recovery_page_get(void);

/** Checks whether given flash manager is building the file area.
 *
 * @retval     true   if the flash manager is building the file area.
 * @retval     false  if the flash manager is not building the file area.
 */
bool flash_manager_is_building(flash_manager_t * p_manager);

/** Checks whether given flash manager is removing the file area.
 *
 * @retval     true   if the flash manager is removing the file area.
 * @retval     false  if the flash manager is not removing the file area.
 */
bool flash_manager_is_removing(flash_manager_t * p_manager);


/** Waits for the flash manager to complete all its operations. */
static inline void flash_manager_wait(void)
{
#if !defined(HOST)
    while (!flash_manager_is_stable())
    {
        /* Temporary hack to make sure that bearer events are handled while waiting for
         * the flash manager to finish.
         * TODO: Find a solution for this that does not include busy-waiting. */
        if (bearer_event_handler())
        {
            __WFE();
        }
    }
#endif
}


/** @} */

#endif /* FLASH_MANAGER_H__ */
