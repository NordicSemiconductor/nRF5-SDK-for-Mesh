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

#ifndef LIGHT_LC_MC_H__
#define LIGHT_LC_MC_H__

#include <stdint.h>

#include "light_lc_common.h"
#include "mesh_config.h"
#include "mesh_opt.h"

/**
 * @defgroup LIGHT_LC_MC Persistence module for the Light LC Setup Server model related states
 * @ingroup LIGHT_LC_MODELS
 *
 * This module provides APIs for handling persistence of the Light LC Setup Server model related states.
 *
 * @{
 */

#define LIGHT_LC_MODE_EID_START                        (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x01))
#define LIGHT_LC_OCC_MODE_EID_START                    (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x02))
#define LIGHT_LC_LIGHT_ONOFF_EID_START                 (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x03))
#define LIGHT_LC_PR_LUXLEVEL_ON_EID_START              (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x04))
#define LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START         (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x05))
#define LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START         (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x06))
#define LIGHT_LC_PR_LIGHTNESS_ON_EID_START             (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x07))
#define LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START        (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x08))
#define LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START        (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x09))
#define LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START       (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0A))
#define LIGHT_LC_PR_REGULATOR_KID_EID_START            (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0B))
#define LIGHT_LC_PR_REGULATOR_KIU_EID_START            (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0C))
#define LIGHT_LC_PR_REGULATOR_KPD_EID_START            (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0D))
#define LIGHT_LC_PR_REGULATOR_KPU_EID_START            (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0E))
#define LIGHT_LC_PR_TIME_FADE_EID_START                (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x0F))
#define LIGHT_LC_PR_TIME_FADE_ON_EID_START             (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x10))
#define LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START   (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x11))
#define LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x12))
#define LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START     (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x13))
#define LIGHT_LC_PR_TIME_PROLONG_EID_START             (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x14))
#define LIGHT_LC_PR_TIME_RUN_ON_EID_START              (MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX * 0x15))

/** Light LC Mode state entry ID */
#define LIGHT_LC_MODE_EID                        MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_MODE_EID_START)

/** Light LC Occupancy Mode state entry ID */
#define LIGHT_LC_OCC_MODE_EID                    MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_OCC_MODE_EID_START)

/** Light LC Light OnOff state entry ID */
#define LIGHT_LC_LIGHT_ONOFF_EID                 MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_LIGHT_ONOFF_EID_START)

/** Light LC Property LuxLevel On state entry ID */
#define LIGHT_LC_PR_LUXLEVEL_ON_EID              MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LUXLEVEL_ON_EID_START)

/** Light LC Property LuxLevel Prolong state entry ID */
#define LIGHT_LC_PR_LUXLEVEL_PROLONG_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START)

/** Light LC Property LuxLevel Standby state entry ID */
#define LIGHT_LC_PR_LUXLEVEL_STANDBY_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START)

/** Light LC Property Lightness On state entry ID */
#define LIGHT_LC_PR_LIGHTNESS_ON_EID             MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LIGHTNESS_ON_EID_START)

/** Light LC Property Lightness Prolong state entry ID */
#define LIGHT_LC_PR_LIGHTNESS_PROLONG_EID        MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START)

/** Light LC Property Lightness Standby state entry ID */
#define LIGHT_LC_PR_LIGHTNESS_STANDBY_EID        MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START)

/** Light LC Property Regulator Accuracy state entry ID */
#define LIGHT_LC_PR_REGULATOR_ACCURACY_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START)

/** Light LC Property Regulator Kid state entry ID */
#define LIGHT_LC_PR_REGULATOR_KID_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_REGULATOR_KID_EID_START)

/** Light LC Property Regulator Kiu state entry ID */
#define LIGHT_LC_PR_REGULATOR_KIU_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_REGULATOR_KIU_EID_START)

/** Light LC Property Regulator Kpd state entry ID */
#define LIGHT_LC_PR_REGULATOR_KPD_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_REGULATOR_KPD_EID_START)

/** Light LC Property Regulator Kpu state entry ID */
#define LIGHT_LC_PR_REGULATOR_KPU_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_REGULATOR_KPU_EID_START)

/** Light LC Property Time Fade state entry ID */
#define LIGHT_LC_PR_TIME_FADE_EID                MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_FADE_EID_START)

/** Light LC Property Time Fade On state entry ID */
#define LIGHT_LC_PR_TIME_FADE_ON_EID             MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_FADE_ON_EID_START)

/** Light LC Property Time Fade Standby Auto state entry ID */
#define LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID   MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START)

/** Light LC Property Time Fade Standby Manual state entry ID */
#define LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START)

/** Light LC Property Time Occupancy Delay state entry ID */
#define LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID     MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START)

/** Light LC Property Time Prolong state entry ID */
#define LIGHT_LC_PR_TIME_PROLONG_EID             MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_PROLONG_EID_START)

/** Light LC Property Time Run On state entry ID */
#define LIGHT_LC_PR_TIME_RUN_ON_EID              MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, LIGHT_LC_PR_TIME_RUN_ON_EID_START)

/** Set internal LC state variables.
 *
 * @param[in] index         An index to identify an instance of a state variable.
 * @param[in] lc_state      Identifies the internal Light LC state to set.
 * @param[in] p_value       Pointer to a buffer of the value to be set.
 *
 * @retval NRF_SUCCESS if the set succeeded.
 * @retval NRF_ERROR_NOT_FOUND      The given lc state or index is invalid.
 * @retval NRF_ERROR_INVALID_DATA   The value is invalid.
 */
uint32_t light_lc_mc_state_set(uint8_t index, light_lc_state_t lc_state, const void * p_value);

/** Get internal LC state variables.
 *
 * @param[in] index     An index to identify an instance of a state variable.
 * @param[in] lc_state  Identifies the internal Light LC state to set.
 * @param[out] p_value   Pointer to a buffer to copy the value into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS              The entry value was successfully copied into @p p_value.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_NOT_FOUND      The given lc state or index is unknown.
 * @retval NRF_ERROR_INVALID_STATE  The given index is known, but has no data associated with it.
 */
uint32_t light_lc_mc_state_get(uint8_t index, light_lc_state_t lc_state, void * p_value);

/** Create an instance of the Light LC Setup Server model states and return the corresponding handle.
 *
 * @param[out] p_handle Pointer to a buffer to copy the handle into to access internal state instance.
 *
 * @retval NRF_SUCCESS              The new instance is successfully created.
 * @retval NRF_ERROR_NULL           A parameter is NULL.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created.
 *                                  In that case, increase value of
 *                                  @ref LIGHT_LC_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t light_lc_mc_open(uint8_t * p_handle);

/**
 * Clear all stored data and reset state contexts to default values.
 */
void light_lc_mc_clear(void);

/**
 * Initialize the Light LC Setup Server persistent memory.
 */
void light_lc_mc_init(void);

/** @} end of LIGHT_LC_MC */

#endif /* LIGHT_LC_MC_H__ */
