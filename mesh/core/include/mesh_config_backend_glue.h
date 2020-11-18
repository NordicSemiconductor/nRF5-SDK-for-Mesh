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
#ifndef MESH_CONFIG_BACKEND_GLUE_H_
#define MESH_CONFIG_BACKEND_GLUE_H_

#include <stdlib.h>
#include "mesh_config_backend_file.h"
#include "mesh_config_backend.h"

/**
 * @defgroup mesh_config_backend_glue Mesh configuration backend glue
 * Glue module for the persistent storage backends.
 * @{
 */

typedef struct
{
    const uint32_t * p_start; /**< Start of the flash area */
    uint32_t length;          /**< Length in bytes */
} mesh_config_backend_flash_usage_t;

/**
 * Initializes the hardware\system dependent backend part.
 *
 * @param[in] evt_cb Event callback.
 */
void mesh_config_backend_glue_init(mesh_config_backend_evt_cb_t evt_cb);

/**
 * Creates the file in the persistent memory.
 *
 * @param[in] p_file Pointer to the file descriptor.
 *
 * @retval NRF_SUCCESS The file is created successfully
 * @returns Any other error on failure.
 */
uint32_t mesh_config_backend_file_create(mesh_config_backend_file_t * p_file);

/**
 * Writes the record into the file.
 *
 * @param[in] p_file Pointer to the file descriptor.
 * @param[in] p_data Pointer to the written data.
 * @param[in] length Data length.
 *
 * @retval NRF_SUCCESS The operation is scheduled successfully
 * @returns Any other error on failure.
 */
uint32_t mesh_config_backend_record_write(mesh_config_backend_file_t * p_file, const uint8_t * p_data, uint32_t length);

/**
 * Removes the record from the file.
 *
 * @param[in] p_file Pointer to the file descriptor.
 *
 * @retval NRF_SUCCESS The operation is scheduled successfully
 * @returns Any other error on failure.
 */
uint32_t mesh_config_backend_record_erase(mesh_config_backend_file_t * p_file);

/**
 * Reads the record from the file.
 *
 * @param[in]     p_file Pointer to the file descriptor.
 * @param[in,out] p_data Pointer to the read data.
 * @param[in,out] p_length Pointer to a variable containing the initial length of the @p p_data buffer in bytes. Will be set
 * to match the actual length of the entry.
 *
 * @retval NRF_SUCCESS The read is completed successfully
 * @returns Any other error on failure.
 */
uint32_t mesh_config_backend_record_read(mesh_config_backend_file_t * p_file, uint8_t * p_data, uint32_t * p_length);
/**
 * Iterates through the records in the file.
 *
 * Calls the callback for every record in the file.
 *
 * @note The function reads the records from the beginning to the end of the file.
 *       If all records were read, then @c pp_data is @c NULL.
 *
 * @param[in] p_file Pointer to the file descriptor.
 * @param[in] callback Callback function to call on every entry.
 */
void mesh_config_backend_records_read(mesh_config_backend_file_t * p_file,
                                      mesh_config_backend_iterate_cb_t callback);

/**
 * Calculates allocated place in bytes for the entry.
 *
 * @param[in]     entry_size Size of the entry in bytes.
 *
 * @returns Size of the necessary place in the file for the record.
 */
uint16_t mesh_config_record_size_calculate(uint16_t entry_size);

/**
 * Gets the flash usage by the backend.
 *
 * @param[in,out] p_usage Returns the current flash usage, if any.
 */
void mesh_config_backend_flash_usage_get(mesh_config_backend_flash_usage_t * p_usage);

/**
 * Gets the time required for power down storage of the given file in microseconds.
 *
 * @param[in] p_file File to check
 *
 * @returns The required power down time in microseconds.
 */
uint32_t mesh_config_backend_file_power_down_time_get(const mesh_config_file_params_t * p_file);

/** @} */

#endif /* MESH_CONFIG_BACKEND_GLUE_H_ */
