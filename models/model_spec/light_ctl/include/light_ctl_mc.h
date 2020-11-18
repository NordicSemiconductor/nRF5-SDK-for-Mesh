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

#ifndef LIGHT_CTL_MC_H__
#define LIGHT_CTL_MC_H__

#include <stdint.h>
#include "light_ctl_common.h"
#include "mesh_config.h"
#include "mesh_opt.h"
#include "model_config_file.h"

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "scene_common.h"
#endif

/**
 * @defgroup LIGHT_CTL_MC Persistence module for the Light CTL Setup Server model related states
 * @ingroup LIGHT_CTL_MODELS
 *
 * This module provides APIs for handling persistence of the Light CTL Setup Server model related states.
 *
 * @{
 */

/** Number of entry instances required to store the current state and state for each scene
 *  @note If @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal to 0, then this is equal to @ref
 *  LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX.
 */
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#define LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES \
                                    (LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX + (SCENE_REGISTER_ARRAY_SIZE * LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX))
#else
#define LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES \
                                    (LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX)
#endif

#define LIGHT_CTL_TEMPERATURE_EID_START         (MESH_APP_MODEL_LIGHT_CTL_SERVER_ID_START)
#define LIGHT_CTL_TEMPERATURE_EID_END           (LIGHT_CTL_TEMPERATURE_EID_START + LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES - 1)
#define LIGHT_CTL_DELTA_UV_EID_START            (LIGHT_CTL_TEMPERATURE_EID_END + 1)
#define LIGHT_CTL_DELTA_UV_EID_END              (LIGHT_CTL_DELTA_UV_EID_START + LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES - 1)
#define LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START (LIGHT_CTL_DELTA_UV_EID_END + 1)
#define LIGHT_CTL_DEFAULT_TEMPERATURE_EID_END   (LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START + LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX - 1)
#define LIGHT_CTL_DELTA_UV_DEFAULT_EID_START    (LIGHT_CTL_DEFAULT_TEMPERATURE_EID_END + 1)
#define LIGHT_CTL_DELTA_UV_DEFAULT_EID_END      (LIGHT_CTL_DELTA_UV_DEFAULT_EID_START + LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX - 1)
#define LIGHT_CTL_TEMPERATURE_RANGE_EID_START   (LIGHT_CTL_DELTA_UV_DEFAULT_EID_END + 1)
#define LIGHT_CTL_TEMPERATURE_RANGE_EID_END     (LIGHT_CTL_TEMPERATURE_RANGE_EID_START + LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX - 1)

/** Light CTL Temperature state entry ID */
#define LIGHT_CTL_TEMPERATURE_EID              MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_CTL_TEMPERATURE_EID_START)

/** Light CTL Delta UV state entry ID */
#define LIGHT_CTL_DELTA_UV_EID                 MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_CTL_DELTA_UV_EID_START)

/** Light CTL Temperature Default state entry ID */
#define LIGHT_CTL_TEMPERATURE_DEFAULT_EID      MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START)

/** Light CTL Delta UV Default state entry ID */
#define LIGHT_CTL_DELTA_UV_DEFAULT_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_CTL_DELTA_UV_DEFAULT_EID_START)

/** Light CTL Temperature Range state entry ID */
#define LIGHT_CTL_TEMPERATURE_RANGE_EID        MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_CTL_TEMPERATURE_RANGE_EID_START)


/** Set internal Light CTL Temperature32 state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_temperature32_state_set(uint8_t index, uint32_t value);

/** Get internal Light CTL Temperature32 state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_temperature32_state_get(uint8_t index, uint32_t * p_value);

/** Set internal Light CTL Delta UV state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_delta_uv_state_set(uint8_t index, int16_t value);

/** Get internal Light CTL Delta UV state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_delta_uv_state_get(uint8_t index, int16_t * p_value);

/** Set internal Light CTL Temperature32 Default state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_default_temperature32_state_set(uint8_t index, uint32_t value);

/** Get internal Light CTL Temperature32 Default state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_default_temperature32_state_get(uint8_t index, uint32_t * p_value);

/** Set internal Light CTL Delta UV Default state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_default_delta_uv_state_set(uint8_t index, int16_t value);

/** Get internal Light CTL Delta UV Default state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_default_delta_uv_state_get(uint8_t index, int16_t * p_value);

/** Set internal Range state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] p_value   Pointer to a buffer of the value to be set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_temperature32_range_state_set(uint8_t index, light_ctl_temperature_range_set_params_t * p_value);

/** Get internal Range Status state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_temperature32_range_state_get(uint8_t index, light_ctl_temperature_range_set_params_t * p_value);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)

/** Stores internal Light CTL Temperature32 state variable for a specific scene index.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] index         An index to identify an instance of a state variable.
 * @param[in] scene_index   A scene index for which given value is saved.
 * @param[in] value         Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_scene_temperature32_state_store(uint8_t index, uint8_t scene_index, uint32_t value);

/** Recalls internal Light CTL Temperature32 state variable for a specific scene index.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in]  index        An index to identify an instance of a state variable.
 * @param[in]  scene_index  A scene index for which given value is saved.
 * @param[out] p_value      Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_scene_temperature32_state_recall(uint8_t index, uint8_t scene_index, uint32_t * p_value);

/** Stores internal Light CTL Delta UV state variable for a specific scene index.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] index         An index to identify an instance of a state variable.
 * @param[in] scene_index   A scene index for which given value is saved.
 * @param[in] value         Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_ctl_mc_scene_delta_uv_state_store(uint8_t index, uint8_t scene_index, int16_t value);

/** Recalls internal Light CTL Delta UV state variable for a specific scene index.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in]  index        An index to identify an instance of a state variable.
 * @param[in]  scene_index  A scene index for which given value is saved.
 * @param[out] p_value      Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_ctl_mc_scene_delta_uv_state_recall(uint8_t index, uint8_t scene_index, int16_t * p_value);

#endif /* (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN) */

/** Create an instance of the Light CTL Setup Server model states and return the corresponding handle.
 *
 * @param[out] p_handle     Pointer to a buffer to copy the handle of the internal state instance.
 *
 * @retval NRF_SUCCESS              The new instance is successfully created.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created.
 *                                  In that case, increase value of
 *                                  @ref LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t light_ctl_mc_open(uint8_t * p_handle);

/**
 * Clear all stored data and reset state contexts to default values.
 */
void light_ctl_mc_clear(void);

/**
 * Initialize the Light CTL Setup Server persistent memory.
 */
void light_ctl_mc_init(void);

/** @} end of LIGHT_CTL_MC */
#endif /* LIGHT_CTL_MC_H__ */
