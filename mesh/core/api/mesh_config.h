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
#ifndef MESH_CONFIG_H__
#define MESH_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config_entry.h"

/**
 * @defgroup MESH_CONFIG Mesh config
 * @ingroup NRF_MESH
 * Common interface for operating on all mesh states.
 *
 * For more information about mesh config, see the @ref md_doc_user_guide_modules_mesh_config library document.
 * @{
 */

typedef enum
{
    MESH_CONFIG_LOAD_FAILURE_INVALID_LENGTH, /**< The loaded entry was shorter than specified by the state owner. */
    MESH_CONFIG_LOAD_FAILURE_INVALID_DATA, /**< The state owner rejected the data. */
    MESH_CONFIG_LOAD_FAILURE_INVALID_ID /**< The loaded ID was unknown to the configuration. */
} mesh_config_load_failure_t;

/**
 * Initialize the configuration module.
 */
void mesh_config_init(void);

/**
 * Load all configuration data from persistent storage.
 *
 * If any of the loaded entries have issues, the mesh event handler will be called with a @c NRF_MESH_EVT_CONFIG_LOAD_FAILURE
 * event. Loading will continue even if there's a load-failure, but the failing entry will be ignored.
 * This should never happen unless the compile-time configuration is changed.
 */
void mesh_config_load(void);

/**
 * Clear all configuration data.
 *
 * @note The configuration data is not cleared until the mesh configuration module leaves the busy
 * state, as indicated by the @ref mesh_config_is_busy function or the @ref NRF_MESH_EVT_CONFIG_STABLE event, or both.
 *
 * @warning  Mesh stack assertion will occur if this API is called when flash manager area of any
 * of the mesh config files is not in `FM_STATE_READY` state. To ensure no assertions are generated
 * this API should be called only when mesh config is [not in busy](@ref mesh_config_is_busy) state.
 */
void mesh_config_clear(void);

/**
 * Store all power-down state.
 *
 * @note The configuration data is not safely stored until the mesh config module goes out of the busy
 * state, as indicated by @ref mesh_config_is_busy or the @ref NRF_MESH_EVT_CONFIG_STABLE event.
 */
void mesh_config_power_down(void);

/**
 * Check whether the configuration is waiting for an operation to finish.
 *
 * @returns Whether an operation is pending.
 */
bool mesh_config_is_busy(void);

/**
 * Calculate the longest time required to store all power down state.
 *
 * To ensure that all power-down state can be safely stored in persistent storage before running out of power, the user must
 * call @ref mesh_config_power_down some time before power loss. This function can be used to gather requirements for
 * a brown-out detection module that can notify the application of an impending power loss.
 *
 * The return value of this function will always be the same for a given configuration, as it always assumes a worst case
 * scenario. It is strongly recommended to rerun this function for every recompilation, to ensure that there's always enough
 * time left.
 *
 * @returns Required storage time in microseconds.
 */
uint32_t mesh_config_power_down_time_get(void);

/**
 * Clean up the file area. It removes all stored file entries. Deleter of all active entries will be called.
 *
 * @note The file data is not safely removed until the mesh config module goes out of the busy
 * state, as indicated by @ref mesh_config_is_busy or the @ref NRF_MESH_EVT_CONFIG_STABLE event.
 *
 * @note If the file identified by given file_id is not found the API returns silently and no
 * events should be expected.
 *
 * @param[in] file_id The unique file identifier.
 */
void mesh_config_file_clear(uint16_t file_id);

/** @} */

#endif /* MESH_CONFIG_H__ */
