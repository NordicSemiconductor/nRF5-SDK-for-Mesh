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

#ifndef GENERIC_LEVEL_MC_H__
#define GENERIC_LEVEL_MC_H__

#include <stdint.h>

#include "generic_level_common.h"
#include "mesh_config.h"
#include "mesh_opt.h"
#include "model_config_file.h"

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "scene_common.h"
#endif

/**
 * @defgroup GENERIC_LEVEL_MC Persistence module for the Generic Level Server model related states
 * @ingroup GENERIC_LEVEL_MODEL
 *
 * This module provides APIs for handling persistence of the Generic Level Server model related states.
 *
 * @{
 */

/** Number of entry instances required to store the current state and state for each scene
 *  @note If @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal to 0, then this is equal to @ref
 *  GENERIC_LEVEL_SERVER_INSTANCES_MAX.
 */
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#define GENERIC_LEVEL_SERVER_STORED_WITH_SCENE_STATES (GENERIC_LEVEL_SERVER_INSTANCES_MAX + \
                                (SCENE_REGISTER_ARRAY_SIZE * GENERIC_LEVEL_SERVER_INSTANCES_MAX))
#else
#define GENERIC_LEVEL_SERVER_STORED_WITH_SCENE_STATES (GENERIC_LEVEL_SERVER_INSTANCES_MAX)
#endif

#define GENERIC_LEVEL_EID_START     (MESH_APP_MODEL_GENERIC_LEVEL_ID_START)
#define GENERIC_LEVEL_EID_END       (GENERIC_LEVEL_EID_START + GENERIC_LEVEL_SERVER_STORED_WITH_SCENE_STATES - 1)

/** Generic Level state entry ID */
#define GENERIC_LEVEL_EID           MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, GENERIC_LEVEL_EID_START)

/** Set internal Level state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t generic_level_mc_level_state_set(uint8_t index, int16_t value);

/** Get internal Level state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t generic_level_mc_level_state_get(uint8_t index, int16_t * p_value);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
/** Store internal Scene Level state variable.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] index         An index to identify an instance of a state variable.
 * @param[in] scene_index   The scene index to idenitfy the scene of a state variable.
 * @param[in] value         Value to store.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t generic_level_mc_scene_level_store(uint8_t index, uint8_t scene_index, int16_t value);

/** Recall internal Scene Level state variable.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in]  index        An index to identify an instance of a state variable.
 * @param[in]  scene_index  The scene index to idenitfy the scene of a state variable.
 * @param[out] p_value      Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t generic_level_mc_scene_level_recall(uint8_t index, uint8_t scene_index, int16_t * p_value);
#endif /* (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN) */

/** Create an instance of the Generic Level Server model states and return the corresponding handle.
 *
 * @param[out] p_handle Pointer to a buffer to copy the handle into to access internal state instance.
 *
 * @retval NRF_SUCCESS              The new instance is successfully created.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created.
 *                                  In that case, increase value of
 *                                  @ref GENERIC_LEVEL_SERVER_INSTANCES_MAX.
 */
uint32_t generic_level_mc_open(uint8_t * p_handle);

/**
 * Clear all stored data and reset state contexts to default values.
 */
void generic_level_mc_clear(void);

/**
 * Initialize the Generic Level Server persistent memory.
 */
void generic_level_mc_init(void);

/** @} end of GENERIC_LEVEL_MC */

#endif /* GENERIC_LEVEL_MC_H__ */
