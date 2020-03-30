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
#ifndef MESH_CONFIG_ENTRY_H__
#define MESH_CONFIG_ENTRY_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_mesh_assert.h"
#include "nrf_mesh_section.h"
#include "nrf_error.h"
#include "mesh_config_backend_file.h"
#include "nordic_common.h"

/**
 * @defgroup MESH_CONFIG_ENTRY Mesh config entry
 * @ingroup MESH_CONFIG
 * API for managing mesh configuration entries.
 * @{
 */

/**
 * Shorthand macro for defining an entry ID
 *
 * Defined as a compound literal so it can be used directly as a parameter in function invocations.
 * See https://gcc.gnu.org/onlinedocs/gcc/Compound-Literals.html
 *
 * @param[in] FILE   File the entry is in.
 * @param[in] RECORD Record within the file.
 */
#define MESH_CONFIG_ENTRY_ID(FILE, RECORD) (const mesh_config_entry_id_t) {(FILE), (RECORD)}
/** Entry max size */
#define MESH_CONFIG_ENTRY_MAX_SIZE 64
/*****************************************************************************
 * State owner interface
 *****************************************************************************/
/**
 * Define a config file.
 *
 * Each file has a unique file ID, and contains multiple entries.
 *
 * @param[in] NAME     Name of the file parameter variable.
 * @param[in] FILE_ID  Identification number of the file.
 * @param[in] STRATEGY Storage strategy for the file.
 */
#define MESH_CONFIG_FILE(NAME, FILE_ID, STRATEGY)                                                   \
    static mesh_config_backend_file_t CONCAT_2(NAME, _backend_data);                                \
    NRF_MESH_SECTION_ITEM_REGISTER_FLASH(mesh_config_files, const mesh_config_file_params_t NAME) = \
        {.id = FILE_ID, .strategy = STRATEGY, .p_backend_data = &CONCAT_2(NAME, _backend_data)}

/**
 * Define a config entry.
 *
 * Each config entry represents a single state, identified by a unique id. The framework will call
 * the provided get and set callbacks to access the live representation of the state.
 *
 * The config entry will get registered at link time.
 *
 * @param[in] NAME              Name of the entry and its variables.
 * @param[in] ID                Unique mesh configuration ID.
 * @param[in] MAX_COUNT         Max number of entries.
 * @param[in] ENTRY_SIZE        Size of each entry.
 * @param[in] SET_CB            Callback to call when setting the value. Cannot be NULL.
 * @param[in] GET_CB            Callback to call to read out the value. Cannot be NULL.
 * @param[in] DELETE_CB         Callback to call to delete the value or NULL.
 * @param[in] HAS_DEFAULT_VALUE Flag for indicating that the entry has a default value that it can
 *                              return before the user explicitly sets it.
 */
#define MESH_CONFIG_ENTRY(NAME, ID, MAX_COUNT, ENTRY_SIZE, SET_CB, GET_CB, DELETE_CB, HAS_DEFAULT_VALUE)            \
    NRF_MESH_STATIC_ASSERT((ENTRY_SIZE) <= MESH_CONFIG_ENTRY_MAX_SIZE);                                             \
    NRF_MESH_STATIC_ASSERT((MAX_COUNT) > 0);                                                                        \
    static mesh_config_entry_flags_t m_##NAME##_state[MAX_COUNT];                                                   \
    NRF_MESH_SECTION_ITEM_REGISTER_FLASH(mesh_config_entries,                                                       \
                                         const mesh_config_entry_params_t m_##NAME##_params) =                      \
        {.p_id        = &(ID),                                                                                      \
         .entry_size  = ENTRY_SIZE,                                                                                 \
         .has_default = HAS_DEFAULT_VALUE,                                                                          \
         .max_count   = MAX_COUNT,                                                                                  \
         .callbacks   = {SET_CB, GET_CB, DELETE_CB},                                                                \
         .p_state     = m_##NAME##_state}

/**
 * Defines a type safe API wrapper for a configuration entry.
 *
 * @param[in] NAME      Name of the API. The functions will be NAME_get() and similar.
 * @param[in] ID        Entry id.
 * @param[in] DATA_TYPE Data type of the state.
 */
#define MESH_CONFIG_ENTRY_API_DEFINE(NAME, ID, DATA_TYPE)       \
    uint32_t NAME##_set(const DATA_TYPE * p_entry)              \
    {                                                           \
        return mesh_config_entry_set((ID), p_entry);            \
    }                                                           \
    uint32_t NAME##_get(DATA_TYPE * p_entry)                    \
    {                                                           \
        return mesh_config_entry_get((ID), p_entry);            \
    }                                                           \
    uint32_t NAME##_delete(void)                                \
    {                                                           \
        return mesh_config_entry_delete((ID));                  \
    }

/**
 * Defines type safe wrapper functions for a configuration entry with multiple items.
 *
 * @param[in] NAME        Base names of the API functions.
 * @param[in] ID          Base entry id.
 * @param[in] DATA_TYPE   Data type of the state.
 * @param[in] INDEX_TYPE  Type of the index variable. Must be cast-able to uint16_t.
 * @param[in] MAX_COUNT   Maximum number of items in the entry.
 */

#define MESH_CONFIG_ENTRY_ARRAY_WRAPPER_DECLARE(NAME, ID, DATA_TYPE, INDEX_TYPE, MAX_COUNT) \
    uint32_t NAME##_set(INDEX_TYPE index, const DATA_TYPE * p_entry)    \
    {                                                                   \
        if (index >= (MAX_COUNT)) return NRF_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_set(id, p_entry);                      \
    }                                                                   \
    uint32_t NAME##_get(INDEX_TYPE index, DATA_TYPE * p_entry)          \
    {                                                                   \
        if (index >= (MAX_COUNT)) return NRF_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_get(id, p_entry);                      \
    }                                                                   \
    uint32_t NAME##_delete(INDEX_TYPE index)                            \
    {                                                                   \
        if (index >= (MAX_COUNT)) return NRF_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_delete(id);                            \
    }

/**
 * Mesh config entry identifier.
 */
typedef struct
{
    uint16_t file;
    uint16_t record;
} mesh_config_entry_id_t;

/**
 * Mesh config entry storage strategy.
 *
 * Defines when the entry should be stored to persistent storage.
 */
typedef enum
{
    MESH_CONFIG_STRATEGY_NON_PERSISTENT, /**< Not stored persistently. */
    MESH_CONFIG_STRATEGY_CONTINUOUS,     /**< Stored as soon as possible after each change. */
    MESH_CONFIG_STRATEGY_ON_POWER_DOWN,  /**< Stored when device is about to power down. */
} mesh_config_strategy_t;

/**
 * State owner entry setter callback.
 *
 * The callback will only be called with IDs within the boundaries specified through @ref MESH_CONFIG_ENTRY.
 * If the callback returns successfully, a subsequent get-callback must return data that is identical to the data passed in this callback.
 * If the callback returns unsuccessfully, the entry data must remain unchanged.
 *
 * @param[in] id      Entry ID to set.
 * @param[in] p_entry Entry data to set. Never NULL.
 *
 * @retval NRF_SUCCESS The entry was successfully set.
 * @retval NRF_ERROR_INVALID_DATA The entry data is invalid, and should be discarded.
 */
typedef uint32_t (*mesh_config_entry_set_t)(mesh_config_entry_id_t id, const void * p_entry);

/**
 * State owner entry getter callback.
 *
 * The callback will only be called on entries that have been set through the state owner's set-callback.
 * The entry data returned through @p p_entry must be identical to the data set in the previous set-callback.
 *
 * @param[in] id          Entry ID to get data of.
 * @param[in,out] p_entry Pointer to entry buffer to copy into.
 */
typedef void (*mesh_config_entry_get_t)(mesh_config_entry_id_t id, void * p_entry);

/**
 * State owner entry delete callback.
 *
 * The callback will only be called on entries that have been set through the state owner's set-callback.
 * The state owner cannot prevent users from deleting entries.
 *
 * @param[in] id Entry ID to be deleted.
 */
typedef void (*mesh_config_entry_delete_t)(mesh_config_entry_id_t id);

/** @internal @{ */

/**
 * File parameters for a mesh config file.
 */
typedef struct
{
    uint16_t id; /**< File ID. */
    mesh_config_strategy_t strategy; /**< Storage strategy. */
    mesh_config_backend_file_t * p_backend_data; /**< Pointer to backend data associated with the file. */
} mesh_config_file_params_t;

/** Entry state */
typedef enum
{
    MESH_CONFIG_ENTRY_FLAG_DIRTY  = (1 << 0), /**< The backend and frontend representation of the entry is not in sync. */
    MESH_CONFIG_ENTRY_FLAG_ACTIVE = (1 << 1), /**< The entry is set to a valid value. */
    MESH_CONFIG_ENTRY_FLAG_BUSY   = (1 << 2), /**< The backend is currently processing the entry. */
} mesh_config_entry_flags_t;

/** Mesh config entry parameters. Should only be instantiated through @ref MESH_CONFIG_ENTRY. */
typedef struct
{
    /** Base-ID for this entry set. Must be a pointer for the @ref MESH_CONFIG_ENTRY_ID macro to
     * work as an initializer, as armcc doesn't support compound literals in static initializers. */
    const mesh_config_entry_id_t * p_id;
    uint16_t entry_size; /**< Size of each entry. */
    uint16_t max_count; /**< Max number of entries in the set. */
    bool has_default; /**< Whether the entry has a default value or not. */
    struct
    {
        mesh_config_entry_set_t setter;
        mesh_config_entry_get_t getter;
        mesh_config_entry_delete_t deleter;
    } callbacks;
    mesh_config_entry_flags_t * p_state; /**< Array of states for each entry. */
} mesh_config_entry_params_t;

/** @} */

/*****************************************************************************
* User interface
*****************************************************************************/
/**
 * Get the first available entry ID in the entry ID-set starting at the @p p_id parameter.
 *
 * For entries with multiple records (i.e. where the state owner defines @c MAX_COUNT > 0), @p p_id will be set to the first
 * unused entry ID in the set. For entries with a single record (i.e. @c MAX_COUNT is 0), @p p_id will be unchanged.
 *
 * @param[in,out] p_id Identifier to start the search from. Will be changed to the first available ID in the set.
 *
 * @returns Whether an unused ID was found.
 */
bool mesh_config_entry_available_id(mesh_config_entry_id_t * p_id);

/**
 * Set an entry value.
 *
 * Setting an entry value will atomically pass the value to the state owner, which validates and stores the entry.
 * The entry will be marked as dirty, and stored according to its storage strategy.
 *
 * @note Even if the storage strategy is @ref MESH_CONFIG_STRATEGY_CONTINUOUS, the entry is not guaranteed to be stored
 * before the function returns. Use @ref mesh_config_is_busy to determine whether all configuration data has been stored.
 *
 * @param[in] id      Entry ID to set the value of.
 * @param[in] p_entry Entry data to set for the given entry ID.
 *
 * @retval NRF_SUCCESS The value was successfully set.
 * @retval NRF_ERROR_NULL A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND The given ID is unknown.
 * @retval NRF_ERROR_INVALID_DATA the state owner has determined that the data in @p p_entry is invalid.
 */
uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry);

/**
 * Get an entry value.
 *
 * Copies the content of the entry represented by the given @id to @p p_entry.
 *
 * @warning: @p p_entry must point to a buffer of sufficient length, as specified by the state owner.
 *
 * @param[in] id       Entry ID to get the value of.
 * @param[out] p_entry Pointer to a buffer to copy the data into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS The entry value was successfully copied into @p p_entry.
 * @retval NRF_ERROR_NULL A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND The given ID is unknown.
 * @retval NRF_ERROR_INVALID_STATE The given ID is known, but has no data associated with it.
 */
uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry);

/**
 * Delete an entry.
 *
 * Deleting an entry which has a default value will make the value return to its default.
 *
 * @note The entry is not guaranteed to be erased before the function returns.
 * Use @ref mesh_config_is_busy to determine whether all configuration data has been stored.
 *
 * @param[in] id Entry ID to delete.
 *
 * @retval NRF_SUCCESS The entry value was successfully deleted.
 * @retval NRF_ERROR_NOT_FOUND The given ID is unknown.
 * @retval NRF_ERROR_INVALID_STATE The given entry was already deleted.
 */
uint32_t mesh_config_entry_delete(mesh_config_entry_id_t id);

/** @} */

#endif /* MESH_CONFIG_ENTRY_H__ */
