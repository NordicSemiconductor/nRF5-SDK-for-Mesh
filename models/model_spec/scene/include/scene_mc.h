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

#ifndef SCENE_MC_H__
#define SCENE_MC_H__

#include <stdint.h>

#include "scene_common.h"
#include "mesh_config.h"
#include "mesh_opt.h"
#include "model_config_file.h"

/**
 * @defgroup SCENE_MC Persistence module for the Scene Setup Server model related states
 * @ingroup SCENE_MODELS
 *
 * This module provides APIs for handling persistence of the Scene Setup Server model related states.
 *
 * @{
 */

#define SCENE_NUMBER_EID_START    (MESH_APP_MODEL_SCENE_SERVER_ID_START)

/** Scene Number state entry ID */
#define SCENE_NUMBER_EID    MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, SCENE_NUMBER_EID_START)

/** Store internal Scene Number variable.
 *
 * @param[in]  handle           Handle to access internal state instances.
 * @param[in]  scene_number     Scene number to store for corresponding handle.
 * @param[out] p_scene_index    An index to identify an instance of a state variable.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t scene_mc_store(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index);

/** Recall internal Scene Number variable.
 *
 * @param[in]  handle           Handle to access internal state instances.
 * @param[in]  scene_number     Scene number to recall for corresponding handle.
 * @param[out] p_scene_index    An index to identify an instance of a state variable.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 */
uint32_t scene_mc_recall(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index);

/** Get the scene number at specified scene index
 *
 * The scene register entries are indexed from 0 to @ref SCENE_REGISTER_ARRAY_SIZE - 1 for each handle.
 *
 * @param[in]   handle          Handle to access internal state instances.
 * @param[in]   scene_index     A scene index to retrive.
 * @param[out]  p_scene_number  Pointer to store the value of the scene number at given scene index.
 *                              If no scene number exist at that index @ref SCENE_NUMBER_NO_SCENE
 *                              will be provided.
 *
 * @retval NRF_SUCCESS              The scene number was successfully copied into @p p_scene_number.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index or handle is unknown.
 */
uint32_t scene_mc_scene_number_get(uint8_t handle, uint8_t scene_index, uint16_t * p_scene_number);

/** Delete internal Scene Number variable.
 *
 * @param[in]  handle           Handle to access internal state instances.
 * @param[in]  scene_number     Scene Number to delete for corresponding handle.
 * @param[out] p_scene_index    An index to identify an instance of a state variable.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t scene_mc_delete(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index);

/** Create an instance of the Scene Setup Server model states and return the corresponding handle.
 *
 * @param[out] p_handle         Pointer to a buffer to copy the handle into to access internal state
 *                              instance.
 *
 * @retval NRF_SUCCESS              The new instance is successfully created.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created. In that case, increase value
 *                                  of @ref SCENE_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t scene_mc_open(uint8_t * p_handle);

/**
 * Clear all stored data and reset state contexts to default values.
 */
void scene_mc_clear(void);

/**
 * Initialize the Scene Setup Server persistent memory.
 */
void scene_mc_init(void);

/** @} end of SCENE_MC */

#endif /* SCENE_MC_H__ */
