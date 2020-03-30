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
#ifndef NRF_FLASH_H__
#define NRF_FLASH_H__

#include <stdint.h>

/**
 * @defgroup NRF_FLASH Flash HW API abstraction
 * @ingroup MESH_CORE
 * @{
 */

/**
 * Erase all pages in flash covered by the specified range.
 *
 * @param[in] p_page First address in the first page to be erased.
 * @param[in] size Number of bytes to erase. All pages that have any data in
 * the range will be erased.
 *
 * @retval NRF_SUCCESS The pages starting at @c p_page, covering all bytes in
 * @c size have been erased.
 * @retval NRF_ERROR_INVALID_ADDR The @c p_page parameter wasn't page aligned.
 * @retval NRF_ERROR_INVALID_LENGTH The @c size parameter was 0.
 */
uint32_t nrf_flash_erase(uint32_t * p_page, uint32_t size);

/**
 * Write data array to flash.
 *
 * @param[in,out] p_dst Address of the first word in the page to be filled.
 * @param[in] p_src Array of data to flash.
 * @param[in] size Size of the p_src data in bytes.
 *
 * @retval NRF_SUCCESS All data in @c p_src was successfully written to flash.
 * @retval NRF_ERROR_INVALID_ADDR One or more of the given addresses weren't
 * page aligned.
 * @retval NRF_ERROR_INVALID_LENGTH The @c size parameter was either 0 or not
 * word aligned.
 */
uint32_t nrf_flash_write(uint32_t * p_dst, const uint32_t * p_src, uint32_t size);

/** @} */

#endif //NRF_FLASH_H__

