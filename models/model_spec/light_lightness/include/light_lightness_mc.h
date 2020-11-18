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

#ifndef LIGHT_LIGHTNESS_MC_H__
#define LIGHT_LIGHTNESS_MC_H__

#include <stdint.h>

#include "light_lightness_common.h"
#include "mesh_config.h"
#include "mesh_opt.h"
#include "model_config_file.h"

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "scene_common.h"
#endif

/**
 * @defgroup LIGHT_LIGHTNESS_MC Persistence module for the Light Lightness Setup Server model related states
 * @ingroup LIGHT_LIGHTNESS_MODELS
 *
 * This module provides APIs for handling persistence of the Light Lightness Setup Server model related states.
 *
 * @{
 */
/** Defines number of state instances required to store current state and state for each scene
 *  @note If @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal to 0, then this is equal to @ref
 *  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX.
 */
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#define STORED_WITH_SCENE_INSTANCES (LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX + \
                                     (SCENE_REGISTER_ARRAY_SIZE * LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX))
#else
#define STORED_WITH_SCENE_INSTANCES (LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#endif

#define LIGHT_LIGHTNESS_ONPOWERUP_EID_START    (MESH_APP_MODEL_LIGHT_LIGHTNESS_ID_START)
#define LIGHT_LIGHTNESS_LAST_EID_START         (LIGHT_LIGHTNESS_ONPOWERUP_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_DEFAULT_EID_START      (LIGHT_LIGHTNESS_LAST_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_RANGE_MIN_EID_START    (LIGHT_LIGHTNESS_DEFAULT_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_RANGE_MAX_EID_START    (LIGHT_LIGHTNESS_RANGE_MIN_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_RANGE_STATUS_EID_START (LIGHT_LIGHTNESS_RANGE_MAX_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_ACTUAL_EID_START       (LIGHT_LIGHTNESS_RANGE_STATUS_EID_START + LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX)
#define LIGHT_LIGHTNESS_DTT_EID_START          (LIGHT_LIGHTNESS_ACTUAL_EID_START + STORED_WITH_SCENE_INSTANCES)

/** Light Lightness On PowerUp state entry ID */
#define LIGHT_LIGHTNESS_ONPOWERUP_EID    MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_ONPOWERUP_EID_START)

/** Light Lightness Last state entry ID */
#define LIGHT_LIGHTNESS_LAST_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_LAST_EID_START)

/** Light Lightness Default state entry ID */
#define LIGHT_LIGHTNESS_DEFAULT_EID      MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_DEFAULT_EID_START)

/** Light Lightness Range Min state entry ID */
#define LIGHT_LIGHTNESS_RANGE_MIN_EID    MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_RANGE_MIN_EID_START)

/** Light Lightness Range Max state entry ID */
#define LIGHT_LIGHTNESS_RANGE_MAX_EID    MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_RANGE_MAX_EID_START)

/** Light Lightness Range Statue state entry ID */
#define LIGHT_LIGHTNESS_RANGE_STATUS_EID MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_RANGE_STATUS_EID_START)

/** Light Lightness Actual state entry ID */
#define LIGHT_LIGHTNESS_ACTUAL_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_ACTUAL_EID_START)

/** Light Lightness Default Transition Time state entry ID */
#define LIGHT_LIGHTNESS_DTT_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LIGHTNESS_DTT_EID_START)

/** Destination to write the value into */
typedef enum
{
    /** The value is only stored to flash. Calling @ref light_lightness_mc_last_state_get
     *  returns the last value stored with @ref LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL flag. */
    LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_FLASH_ONLY,
    /** The value is stored to flash and RAM. Calling @ref light_lightness_mc_last_state_get
     *  returns the stored value. */
    LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL
} light_lightness_mc_write_destination_t;

/** Set internal Generic OnPowerUp state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_onpowerup_state_set(uint8_t index, uint8_t value);

/** Get internal Generic OnPowerUp state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_onpowerup_state_get(uint8_t index, uint8_t * p_value);


/** Set internal Light Lightness Last state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 * @param[in] write_to  Destination representation to write this value into.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_last_state_set(uint8_t index, uint16_t value, light_lightness_mc_write_destination_t write_to);

/** Get internal Light Lightness Last state variable.
 *
 * @note The function returns the last value stored with
 * @ref LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_ALL flag.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_last_state_get(uint8_t index, uint16_t * p_value);


/** Set internal Light Lightness Default state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_default_state_set(uint8_t index, uint16_t value);

/** Get internal Light Lightness Default state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_default_state_get(uint8_t index, uint16_t * p_value);


/** Set internal Range Status state variable.
 *
 * This is required to persist the value of the Status code for range set operation executed by the
 * previous Light Lightness Range Set message. This enables subsequent Light Lightness Range Get
 * messages to receive status of the last operation.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_range_status_state_set(uint8_t index, uint8_t value);

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
uint32_t light_lightness_mc_range_status_state_get(uint8_t index, uint8_t * p_value);


/** Set internal Light Lightness Range Min state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_range_min_state_set(uint8_t index, uint16_t value);

/** Get internal Light Lightness Range Min state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_range_min_state_get(uint8_t index, uint16_t * p_value);


/** Set internal Light Lightness Range Max state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_range_max_state_set(uint8_t index, uint16_t value);

/** Get internal Light Lightness Range Max state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_range_max_state_get(uint8_t index, uint16_t * p_value);


/** Set internal Light Lightness Actual state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_actual_state_set(uint8_t index, uint16_t value);

/** Get internal Light Lightness Actual state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_actual_state_get(uint8_t index, uint16_t * p_value);


/** Set internal Default Transition Time state variable.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] value     Value to set.
 *
 * @retval NRF_SUCCESS              The value was successfully set.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lightness_mc_dtt_state_set(uint8_t index, uint32_t value);

/** Get internal Default Transition Time state variable.
 *
 * @param[in]  index     An index to identify an instance of a state variable.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lightness_mc_dtt_state_get(uint8_t index, uint32_t * p_value);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
/** Store internal Scene Actual state variable.
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
uint32_t light_lightness_mc_scene_actual_state_store(uint8_t index, uint8_t scene_index,
                                                     uint16_t value);

/** Recall internal Scene Actual state variable.
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
uint32_t light_lightness_mc_scene_actual_state_recall(uint8_t index, uint8_t scene_index,
                                                      uint16_t * p_value);
#endif

/** Create an instance of the Light Lightness Setup Server model states and return the corresponding handle.
 *
 * @param[out] p_handle Pointer to a buffer to copy the handle into to access internal state instance.
 *
 * @retval NRF_SUCCESS              The new instance is successfully created.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created.
 *                                  In that case, increase value of
 *                                  @ref LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t light_lightness_mc_open(uint8_t * p_handle);

/**
 * Clear all stored data and reset state contexts to default values.
 */
void light_lightness_mc_clear(void);

/**
 * Initialize the Light Lightness Setup Server persistent memory.
 */
void light_lightness_mc_init(void);

/** @} end of LIGHT_LIGHTNESS_MC */

#endif /* LIGHT_LIGHTNESS_MC_H__ */
