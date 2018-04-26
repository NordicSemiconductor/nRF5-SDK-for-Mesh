/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef FLASH_HELPER_H__
#define FLASH_HELPER_H__

#include "enocean_switch_example.h"
#include "flash_manager.h"
/**
 * @defgroup FLASH_HELPER_H Flash helper header
 * @{
 */

/**
 * @defgroup FLASH_HELPER_API Flash helper APIs
 * @{
 */

/**
 * Loads the application specific data from flash.
 *
 * Data is loaded to `p_data` if entry is found, else `p_data` is filled with `0x00`.
 *
 * @param[in]  entry_handle Entry handle
 * @param[out] p_data       Pointer to the application data structure which will hold the data
 * @param[in]  length       Length of the data structure
 *
 * @return NRF_SUCCESS          If entry was found and data was loaded
 * @return NRF_ERROR_NOT_FOUND  If entry is not found
 */
uint32_t app_flash_data_load(fm_handle_t entry_handle, void * p_data, uint8_t length);

/**
 * Stores given data structure to the flash
 *
 * @note Data structure cannot be longer than @ref FLASH_MANAGER_ENTRY_MAX_SIZE
 *
 * @param[in]  entry_handle Entry handle
 * @param[in]  p_data        Pointer to the application data structure which will be stored
 * @param[in]  length       Length of the data structure
 *
 * @return NRF_ERROR_BUSY   If flash is busy and no more store requests can be queued
 * @return NRF_SUCCESS      If entry is committed succesfully
 */
uint32_t app_flash_data_store(fm_handle_t entry_handle, const void * p_data, uint8_t length);

/**
 * Adds the flash manger instance for the application.
 *
 * Call this function during bootup.
 */
void app_flash_init(void);

/**
 * Removes the flash manager and resets the data buffer.
 *
 * @param[out] p_data       Pointer to the application data structure which will be reset to zero
 * @param[in]  length       Length of the data structure
 */
void app_flash_clear(void * p_data, uint8_t length);

/** @} FLASH_HELPER_API */


/** @} end of FLASH_HELPER_H */

#endif /* FLASH_HELPER_H__ */
