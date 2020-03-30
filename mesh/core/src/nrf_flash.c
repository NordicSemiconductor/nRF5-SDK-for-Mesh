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

#include <stdint.h>
#include "nrf_flash.h"
#include "nrf.h"
#include "utils.h"
#include "nrf_mesh_assert.h"

#define FLASH_BLANK_WORD    (0xFFFFFFFF)
/*****************************************************************************
* Static functions
*****************************************************************************/
#if !defined(HOST)
static inline void wait_for_ready(void)
{
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        /* Do nothing */
    }
}
#endif
/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t nrf_flash_erase(uint32_t * p_page, uint32_t size)
{
    if (!IS_PAGE_ALIGNED(p_page))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (size == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint32_t num_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

#if !defined(HOST)
    /* Turn on flash erase enable and wait until the NVMC is ready: */
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    wait_for_ready();

    for (uint32_t  i = 0; i < num_pages; i++)
    {
        /* Erase page: */
        NRF_NVMC->ERASEPAGE = (uint32_t) p_page;
        wait_for_ready();

        p_page += PAGE_SIZE / sizeof(uint32_t*);
    }

    /* Turn off flash erase enable and wait until the NVMC is ready: */
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
    wait_for_ready();
#else
    memset(p_page, 0xFF, num_pages * PAGE_SIZE);
#endif
    return NRF_SUCCESS;
}

uint32_t nrf_flash_write(uint32_t * p_dst, const uint32_t* p_src, uint32_t size)
{
    if (!IS_WORD_ALIGNED(p_dst) || !IS_WORD_ALIGNED(p_src))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (size == 0 || !IS_WORD_ALIGNED(size))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
#if !defined(HOST)
    size /= WORD_SIZE;

    /* Turn on flash write enable and wait until the NVMC is ready: */
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    wait_for_ready();

    while (size--)
    {
        if (*p_src != FLASH_BLANK_WORD)
        {
            *p_dst = *p_src;
            wait_for_ready();

        }
        p_dst++;
        p_src++;
    }

    /* Turn off flash write enable and wait until the NVMC is ready: */
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    wait_for_ready();

#else
    memcpy(p_dst, p_src, size);
#endif
    return NRF_SUCCESS;
}
