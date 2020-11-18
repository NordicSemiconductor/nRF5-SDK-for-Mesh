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

#ifndef MODEL_CONFIG_FILE_H__
#define MODEL_CONFIG_FILE_H__

#include <stdint.h>

#include "nrf_mesh_config_core.h"

/**
 * @defgroup MODEL_CONFIG_FILE Model config file entry IDs
 * @ingroup MESH_API_GROUP_MODELS
 * Contains common defines for model config file (MESH_OPT_MODEL_FILE_ID) used by some of
 * the Mesh models. Use of this module reserves one flash page for storing model state data.
 * This module is not needed if certain model does not intend to use model config file for
 * its state data storage.
 *
 * Each new model (other than the ones already present in this SDK) that requires state
 * storage can use this config file to store state data. For this purpose `*_mc.c`
 * module have to be developed on the same lines as that of the existing modules available
 * for various models in this SDK.
 *
 * @warning Changing entry IDs across firmware updates will cause provisioned node to boot as
 * an unprovisioned device after DFU.
 *
 * @{
 */

/** Model common record entry ID */
#define MESH_APP_MODEL_CONFIG_ID                    (0x0001)
/** Start of Light Lightness Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_LIGHTNESS_ID_START     (0x1000)
/** End of Light Lightness Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_LIGHTNESS_ID_END       (0x10FF)
/** Start of LC Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START     (0x1100)
/** End of Light LC Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_LC_SERVER_ID_END       (0x20FF)
/** Start of Light CTL Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_CTL_SERVER_ID_START    (0x2100)
/** End of Light CTL Setup Server record entry IDs */
#define MESH_APP_MODEL_LIGHT_CTL_SERVER_ID_END      (0x22FF)
/** Start of Scene Setup Server record entry IDs */
#define MESH_APP_MODEL_SCENE_SERVER_ID_START        (0x2300)
/** End of Scene Setup Server record entry IDs */
#define MESH_APP_MODEL_SCENE_SERVER_ID_END          (0x23FF)
/** Start of Generic OnOff Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_ONOFF_ID_START       (0x2400)
/** End of Generic OnOff Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_ONOFF_ID_END         (0x24FF)
/** Start of Generic Level Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_LEVEL_ID_START       (0x2500)
/** End of Generic Level Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_LEVEL_ID_END         (0x25FF)
/** Start of Generic Default Transition Time Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_DTT_ID_START         (0x2600)
/** End of Generic Default Transition Time Server record entry IDs */
#define MESH_APP_MODEL_GENERIC_DTT_ID_END           (0x26FF)

/**
 * Initialize persistent memory of all models used.
 *
 * @note If models are not linked-in the model state storage will not be initialized and this
 * function will call a dummy funcion for those models.
 */
void model_config_file_init(void);

/**
 * Apply data loaded from the mesh configuration system into persistent memory structures.
 *
 * @note Actual metadata is restored automatically if it was not found or if read out data is not
 * equal configuration parameters.
 *
 * @retval NRF_SUCCESS            Presistent memory data applied successfully. Default values are
 *                                stored if no data existed.
 * @retval NRF_ERROR_INVALID_DATA Data stored in the persistent memory was corrupted, old data was
 *                                cleared and restored with default values. Stack config is cleared.
 */
uint32_t model_config_file_config_apply(void);

/**
 * Clears the persistent model states and erases the persistent copy stored in model config file.
 */
void model_config_file_clear(void);

/** @} end of MODEL_CONFIG_FILE */

#endif /* MODEL_CONFIG_FILE_H__ */

