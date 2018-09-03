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

#include "flash_helper.h"

#include "mesh_app_utils.h"
#include "nrf_mesh_assert.h"
#include "device_state_manager.h"
#include "flash_manager.h"
#include "nrf_mesh_config_app.h"
#include "mesh_stack.h"
#include "log.h"


/**** Flash handling ****/
#if PERSISTENT_STORAGE

static flash_manager_t m_flash_manager;

static void flash_write_complete(const flash_manager_t * p_manager, const fm_entry_t * p_entry, fm_result_t result)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash write complete\n");

    /* If we get an AREA_FULL then our calculations for flash space required are buggy. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_AREA_FULL);

    /* We do not invalidate in this module, so a NOT_FOUND should not be received. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_NOT_FOUND);
    if (result == FM_RESULT_ERROR_FLASH_MALFUNCTION)
    {
        ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
}

static void flash_invalidate_complete(const flash_manager_t * p_manager, fm_handle_t handle, fm_result_t result)
{
    /* This application does not expect invalidate complete calls. */
    ERROR_CHECK(NRF_ERROR_INTERNAL);
}

static void flash_manager_mem_available(void * p_args)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash mem available\n");
    ((void (*)(void)) p_args)(); /*lint !e611 Suspicious cast */
}


static void flash_remove_complete(const flash_manager_t * p_manager)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash remove complete\n");
}

static void flash_reset(void)
{
    static fm_mem_listener_t mem_listener = {
        .callback = flash_manager_mem_available,
        .p_args = flash_reset
    };

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Removing data\n");
    if (flash_manager_remove(&m_flash_manager) != NRF_SUCCESS)
    {
        /* Register the listener and wait for some memory to be freed up before we retry. */
        flash_manager_mem_listener_register(&mem_listener);
    }
}

void app_flash_init(void)
{

    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = app_flash_init
    };

    const uint32_t * start_address;
    uint32_t allocated_area_size;
    ERROR_CHECK(mesh_stack_persistence_flash_usage(&start_address, &allocated_area_size));

    flash_manager_config_t manager_config;
    manager_config.write_complete_cb = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb = flash_remove_complete;
    manager_config.min_available_space = WORD_SIZE;
    manager_config.p_area = (const flash_manager_page_t *)((uint32_t)start_address - PAGE_SIZE * APP_FLASH_PAGE_COUNT);
    manager_config.page_count = APP_FLASH_PAGE_COUNT;

    uint32_t status = flash_manager_add(&m_flash_manager, &manager_config);
    if (NRF_SUCCESS != status)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unable to add flash manager for app data\n");
    }
}

uint32_t app_flash_data_load(fm_handle_t entry_handle, void * p_data, uint8_t length)
{
    flash_manager_wait();
    const fm_entry_t * p_entry = flash_manager_entry_get(&m_flash_manager, entry_handle);
    if (p_entry == NULL)
    {
        memset(p_data, 0x00, length);
        return NRF_ERROR_NOT_FOUND;
    }

    memcpy(p_data, p_entry->data, length);
    return NRF_SUCCESS;
}

uint32_t app_flash_data_store(fm_handle_t entry_handle, const void * p_data, uint8_t length)
{
    flash_manager_wait();
    fm_entry_t * p_entry = flash_manager_entry_alloc(&m_flash_manager, entry_handle, length);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Storing data\n");
    if (p_entry == NULL)
    {
        return NRF_ERROR_BUSY;
    }
    else
    {
        memcpy(p_entry->data, p_data, length);
        flash_manager_entry_commit(p_entry);
        return NRF_SUCCESS;
    }
}

void app_flash_clear(void * p_data, uint8_t length)
{
    memset(p_data, 0x00, length);
    flash_reset();
}

#else

void app_flash_init(void)
{

}

uint32_t app_flash_data_load(fm_handle_t entry_handle, void * p_data, uint8_t length)
{
    memset(p_data, 0x00, length);
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t app_flash_data_store(fm_handle_t entry_handle, const void * p_data, uint8_t length)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

void app_flash_clear(void * p_data, uint8_t length)
{
    memset(p_data, 0x00, length);
}

#endif

