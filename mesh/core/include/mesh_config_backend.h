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
#ifndef MESH_CONFIG_BACKEND_H__
#define MESH_CONFIG_BACKEND_H__

#include <stdint.h>
#include "mesh_config_entry.h"

/**
 * @defgroup MESH_CONFIG_BACKEND Mesh configuration backend
 * Defines the API for mesh config backends. Every backend implements this API, but only one should
 * be linked.
 * @{
 */

/** Backend event type. */
typedef enum
{
    MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE, /**< Store complete. */
    MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE, /**< Erase complete. */
    MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE, /**< File clean complete. */
    MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE, /**< The storage medium has irrevocably failed. */
} mesh_config_backend_evt_type_t;

/** Backend events, passed to the frontend through the event handler callback. */
typedef struct
{
    mesh_config_backend_evt_type_t type;
    mesh_config_entry_id_t id;
    uint8_t * p_data;
    uint32_t length;
} mesh_config_backend_evt_t;

/**
 * Event handler callback for backend events.
 *
 * @param[in] p_evt Event to handle.
 */
typedef void (* mesh_config_backend_evt_cb_t)(const mesh_config_backend_evt_t * p_evt);

/** Action returned from the iterate callback, determining whether the iteration should continue. */
typedef enum
{
    MESH_CONFIG_BACKEND_ITERATE_ACTION_STOP, /**< Stop iterating through the entries. */
    MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE, /**< Continue iterating through the entries. */
} mesh_config_backend_iterate_action_t;

/**
 * Callback for iterating through entries.
 *
 * Will be called for every known entry in the backend, until all entries are covered or the callback returns
 * @ref MESH_CONFIG_BACKEND_ITERATE_ACTION_STOP.
 *
 * @param[in] id        ID of the current entry.
 * @param[in] p_entry   Data contained in the current entry.
 * @param[in] entry_len Length of the data.
 *
 * @returns Whether to continue iterating.
 */
typedef mesh_config_backend_iterate_action_t (*mesh_config_backend_iterate_cb_t)(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len);


/**
 * Initializes the backend.
 *
 * The backend can use the given entry parameter and file arrays to determine resource requirements.
 *
 * @param[in] p_entries     List of known configuration entry parameters.
 * @param[in] entry_count   Number of entries.
 * @param[in] p_files       List of known configuration files.
 * @param[in] file_count    Number of configuration files.
 * @param[in] evt_cb        Event callback.
 */
void mesh_config_backend_init(const mesh_config_entry_params_t * p_entries,
                              uint32_t entry_count,
                              const mesh_config_file_params_t * p_files,
                              uint32_t file_count,
                              mesh_config_backend_evt_cb_t evt_cb);

/**
 * Stores a single entry.
 *
 * The backend passes a @ref MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE event to the event handler
 * once the entry is stored.
 *
 * @note The matching event may come before the function returns.
 * @note Data pointed by the p_entry is not required to be preserved. Backend makes the copy of this
 * data if required.
 *
 * @param[in] id                ID of the entry to store.
 * @param[in] p_entry           Entry data.
 * @param[in] entry_len         Entry data length in bytes.
 *
 * @retval NRF_SUCCESS          The backend successfully scheduled the entry to be written.
 * @retval NRF_ERROR_NO_MEM     There's not enough space available in the process queue.
 */
uint32_t mesh_config_backend_store(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len);

/**
 * Erases a single entry.
 *
 * The backend passes a @ref MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE event to the event handler
 * once the entry is erased.
 *
 * @note The matching event may come before the function returns.
 *
 * @param[in] id                ID of the entry to erase.
 *
 * @retval NRF_SUCCESS          The backend successfully scheduled the entry to be erased.
 * @retval NRF_ERROR_NOT_FOUND  No such entry.
 * @retval NRF_ERROR_NO_MEM     There's not enough space available in the process queue.
 */
uint32_t mesh_config_backend_erase(mesh_config_entry_id_t id);

/**
 * Reads a single entry synchronously.
 *
 * The entry will be read into the buffer with the maximal length of @p p_entry_len. If the read
 * entry is longer than the given buffer, the function will return @ref NRF_ERROR_INVALID_LENGTH and
 * set @p p_entry_len to the required length.
 *
 * @param[in] id                    ID of the entry to read.
 * @param[in,out] p_entry           Pointer to an entry buffer to set the value of.
 * @param[in,out] p_entry_len       Pointer to a variable containing the initial length of the @p
 *                                  p_entry buffer in bytes. Will be set to match the actual length
 *                                  of the entry.
 *
 * @retval NRF_SUCCESS              The backed successfully read out the value of the entry into the
 *                                  p_entry parameter.
 * @retval NRF_ERROR_INVALID_LENGTH The given @p_entry_len was too short, and no data has been
 *                                  written. @p p_entry_len has been set to the required length.
 * @retval NRF_ERROR_NOT_FOUND      No such entry.
 */
uint32_t mesh_config_backend_read(mesh_config_entry_id_t id, uint8_t * p_entry, uint32_t * p_entry_len);

/**
 * Reads all entries.
 *
 * The given callback will be called for every valid entry in the backend.
 *
 * @param[in] cb Callback to call for each entry.
 */
void mesh_config_backend_read_all(mesh_config_backend_iterate_cb_t cb);

/**
 * Gets backend power down time.
 *
 * @returns The maximum power down time in microseconds.
 */
uint32_t mesh_config_backend_power_down_time_get(void);

/**
 * Cleans the file content.
 * The backend passes a @ref MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE event to the event handler
 * once the file is cleaned.
 *
 * It is possible to clean several files in parallel. The backend creates queue from requests.
 *
 * @param[in] p_file File to clean content.
 */
void mesh_config_backend_file_clean(mesh_config_backend_file_t * p_file);

/**
 * Puts the backend in the power down mode.
 */
void mesh_config_backend_power_down(void);

/**
 * Checks the available place in the allocated flash area for the file.
 *
 * @param[in] p_file File to check place.
 *
 * @return true    If the file has enough place to store all data in case of the power down.
 *         false   Otherwise.
 */
bool mesh_config_backend_is_there_power_down_place(mesh_config_backend_file_t * p_file);

/** @} */

#endif /* MESH_CONFIG_BACKEND_H__ */
