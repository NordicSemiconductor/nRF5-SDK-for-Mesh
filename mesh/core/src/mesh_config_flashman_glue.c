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

#include "nrf_error.h"

#include "mesh_config_backend_glue.h"
#include "nrf_mesh_assert.h"
#include "flash_manager.h"
#include "flash_manager_internal.h"
#include "flash_manager_defrag.h"
#include "event.h"
#include "utils.h"
#include "mesh_opt.h"
#include "nrf_mesh_config_core.h"
#include "log.h"

typedef struct
{
    mesh_config_backend_iterate_cb_t callback;
    mesh_config_backend_file_t * p_file;
} record_iterator_context_t;

#define FLASH_TIME_PER_WORD_US        FLASH_TIME_TO_WRITE_ONE_WORD_US
#define FLASH_ENTRY_TIME_OVERHEAD_US  100
#define FLASH_BASE_TIME_OVERHEAD_US   500

#ifndef NET_FLASH_PAGE_COUNT
/* This is set to (1) to be compatible with old default value. The value of this is not used
 * unless ACCESS_FLASH_PAGE_COUNT and DSM_FLASH_PAGE_COUNT are both defined and non-zero,
 * which will trigger backward compatible persistence.
 */
#define NET_FLASH_PAGE_COUNT (1)
#endif

#ifndef ACCESS_FLASH_PAGE_COUNT
#define ACCESS_FLASH_PAGE_COUNT (0)
#endif

#ifndef DSM_FLASH_PAGE_COUNT
#define DSM_FLASH_PAGE_COUNT (0)
#endif

#define BACKWARD_COMPATIBLE_PERSISTENCE (NET_FLASH_PAGE_COUNT && \
                                         ACCESS_FLASH_PAGE_COUNT && \
                                         DSM_FLASH_PAGE_COUNT)

static mesh_config_backend_evt_cb_t m_evt_cb;
static uint8_t m_allocated_page_count;

static void file_remove(mesh_config_backend_file_t * p_file);
static void file_restore(mesh_config_backend_file_t * p_file);

static const uint8_t * flash_area_end_get(void)
{
    return ((const uint8_t *) flash_manager_recovery_page_get());
}

static void flash_stable_cb(void)
{
    nrf_mesh_evt_t evt;
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_FLASH_STABLE;
    event_handle(&evt);
}

static fm_iterate_action_t entry_read_cb(const fm_entry_t * p_entry, void * p_args)
{
    static const fm_iterate_action_t action_map[] = {
        [MESH_CONFIG_BACKEND_ITERATE_ACTION_STOP] = FM_ITERATE_ACTION_STOP,
        [MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE] = FM_ITERATE_ACTION_CONTINUE,
    };
    record_iterator_context_t * p_context = p_args;
    p_context->p_file->curr_pos           = p_entry->header.handle;
    mesh_config_entry_id_t id = MESH_CONFIG_ENTRY_ID(p_context->p_file->file_id, p_entry->header.handle);
    uint32_t data_len = p_entry->header.len_words * WORD_SIZE - sizeof(fm_header_t);

    return action_map[p_context->callback(id, (const uint8_t *) p_entry->data, data_len)];
}

static void write_complete_cb(const flash_manager_t * p_manager,
                              const fm_entry_t * p_entry,
                              fm_result_t result)
{
    mesh_config_backend_evt_t event;
    mesh_config_backend_glue_data_t * p_glue = PARENT_BY_FIELD_GET(mesh_config_backend_glue_data_t,
                                                            flash_manager,
                                                            p_manager);
    mesh_config_backend_file_t * p_file = PARENT_BY_FIELD_GET(mesh_config_backend_file_t,
                                                              glue_data,
                                                              p_glue);

    event.type = result == FM_RESULT_SUCCESS ? MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE :
            MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE;
    event.id.file = p_file->file_id;
    event.id.record = p_entry->header.handle;
    event.p_data = (uint8_t *)p_entry->data;
    event.length = p_entry->header.len_words * WORD_SIZE - sizeof(fm_header_t);
    m_evt_cb(&event);
}

static void invalidate_complete_cb(const flash_manager_t * p_manager,
                                   fm_handle_t handle,
                                   fm_result_t result)
{
    mesh_config_backend_evt_t event;
    mesh_config_backend_glue_data_t * p_glue = PARENT_BY_FIELD_GET(mesh_config_backend_glue_data_t,
                                                            flash_manager,
                                                            p_manager);
    mesh_config_backend_file_t * p_file = PARENT_BY_FIELD_GET(mesh_config_backend_file_t,
                                                              glue_data,
                                                              p_glue);

    event.type = result == FM_RESULT_SUCCESS ? MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE :
            MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE;
    event.id.file = p_file->file_id;
    event.id.record = handle;
    event.p_data = NULL;
    event.length = 0;
    m_evt_cb(&event);
}

static void remove_complete_cb(const flash_manager_t * p_manager)
{
    mesh_config_backend_glue_data_t * p_glue = PARENT_BY_FIELD_GET(mesh_config_backend_glue_data_t,
                                                            flash_manager,
                                                            p_manager);
    mesh_config_backend_file_t * p_file = PARENT_BY_FIELD_GET(mesh_config_backend_file_t,
                                                              glue_data,
                                                              p_glue);

    __LOG(LOG_SRC_FM, LOG_LEVEL_DBG1, "File %d removed\n", p_file->file_id);
    file_restore(p_file);
}

static void file_remove_listener_wrapper(void * p_args)
{
    file_remove((mesh_config_backend_file_t *)p_args);
}

static void file_restore_listener_wrapper(void * p_args)
{
    file_restore((mesh_config_backend_file_t *)p_args);
}

static void file_ready_listener(void * p_args)
{
    mesh_config_backend_file_t * p_file = (mesh_config_backend_file_t *)p_args;

    if (p_file->glue_data.flash_manager.internal.state != FM_STATE_READY)
    {
        p_file->glue_data.listener.callback = file_ready_listener;
        p_file->glue_data.listener.p_args = p_file;
        flash_manager_mem_listener_register(&p_file->glue_data.listener);
        return;
    }

    mesh_config_backend_evt_t event;
    event.type = MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE;
    event.id.file = p_file->file_id;
    event.p_data = NULL;
    event.length = 0;
    m_evt_cb(&event);
}

static void file_remove(mesh_config_backend_file_t * p_file)
{
    flash_manager_t * p_manager = &p_file->glue_data.flash_manager;

    if (flash_manager_is_removing(p_manager))
    {
        return;
    }

    uint32_t status = flash_manager_remove(p_manager);

    /* It is possible that the removal will fail due to following valid reasons and hence it will
     * be retried.
     * 1. No memory. In such case retry later.
     * 2. Flash manager is already building and hence Invalid state is returned. This can happen
     * post DFU upgrade when mesh config gets cleared due to mismatch but at the same time newly
     * added file starts building. In such case retry later. It is necessary to remove newly built
     * file to ensure all mesh config files are cleaned up when requested by higher module.
     */
    NRF_MESH_ASSERT(status == NRF_ERROR_NO_MEM || status == NRF_SUCCESS ||
                    (status == NRF_ERROR_INVALID_STATE && flash_manager_is_building(p_manager)));

    /* NRF_ERROR_INVALID_STATE status is acceptable only when file is
     * building. */
    if (status == NRF_ERROR_NO_MEM || status == NRF_ERROR_INVALID_STATE)
    {
        p_file->glue_data.listener.callback = file_remove_listener_wrapper;
        p_file->glue_data.listener.p_args = p_file;
        flash_manager_mem_listener_register(&p_file->glue_data.listener);
    }
}

static void file_restore(mesh_config_backend_file_t * p_file)
{
    flash_manager_t * p_manager = &p_file->glue_data.flash_manager;
    const flash_manager_config_t config =
    {
        .write_complete_cb = write_complete_cb,
        .invalidate_complete_cb = invalidate_complete_cb,
        .remove_complete_cb = remove_complete_cb,
        .min_available_space = p_file->size,
        .p_area = p_manager->config.p_area,
        .page_count = p_manager->config.page_count
    };

    uint32_t status = flash_manager_add(p_manager, &config);
    NRF_MESH_ASSERT(status == NRF_ERROR_NO_MEM || status == NRF_SUCCESS);

    if (status == NRF_ERROR_NO_MEM)
    {
        p_file->glue_data.listener.callback = file_restore_listener_wrapper;
        p_file->glue_data.listener.p_args = p_file;
        flash_manager_mem_listener_register(&p_file->glue_data.listener);
    }
    else
    {
        p_file->glue_data.listener.callback = file_ready_listener;
        p_file->glue_data.listener.p_args = p_file;
        flash_manager_mem_listener_register(&p_file->glue_data.listener);
    }
}

void mesh_config_backend_glue_init(mesh_config_backend_evt_cb_t evt_cb)
{
    m_allocated_page_count = 0;
    m_evt_cb = evt_cb;

    flash_manager_init();
    flash_manager_action_queue_empty_cb_set(flash_stable_cb);
}

#if BACKWARD_COMPATIBLE_PERSISTENCE
static const void * legacy_net_location_get(void)
{
#ifdef NET_FLASH_AREA_LOCATION
    return NET_FLASH_AREA_LOCATION;
#else
    return NULL;
#endif
}

static const void * legacy_dsm_location_get(void)
{
#ifdef DSM_FLASH_AREA_LOCATION
    return DSM_FLASH_AREA_LOCATION;
#else
    const void * p_net_location = legacy_net_location_get();
    return p_net_location != NULL ? ((const uint8_t *) p_net_location) - (DSM_FLASH_PAGE_COUNT * PAGE_SIZE) : NULL;
#endif
}

static const void * legacy_access_location_get(void)
{
#ifdef ACCESS_FLASH_AREA_LOCATION
    return ACCESS_FLASH_AREA_LOCATION;
#else
    const void * p_dsm_location = legacy_dsm_location_get();
    return p_dsm_location != NULL ? ((const uint8_t *) p_dsm_location) - (ACCESS_FLASH_PAGE_COUNT * PAGE_SIZE) : NULL;
#endif
}

static void legacy_flash_config_get(mesh_config_backend_file_t * p_file,
                                    uint8_t *                    p_page_count,
                                    flash_manager_page_t **      pp_area)
{
    switch (p_file->file_id)
    {
        case MESH_OPT_NET_STATE_FILE_ID:
            *p_page_count = NET_FLASH_PAGE_COUNT;
            *pp_area      = (flash_manager_page_t *)legacy_net_location_get();
            break;
        case MESH_OPT_ACCESS_FILE_ID:
            *p_page_count = ACCESS_FLASH_PAGE_COUNT;
            *pp_area      = (flash_manager_page_t *)legacy_access_location_get();
            break;
        case MESH_OPT_DSM_FILE_ID:
            *p_page_count = DSM_FLASH_PAGE_COUNT;
            *pp_area      = (flash_manager_page_t *)legacy_dsm_location_get();
            break;
        default:
            *p_page_count = CEIL_DIV(p_file->size, FLASH_MANAGER_DATA_PER_PAGE);
            *pp_area      = NULL;
            break;
    }
}
#endif /* BACKWARD_COMPATIBLE_PERSISTENCE */

uint32_t mesh_config_backend_file_create(mesh_config_backend_file_t * p_file)
{
    uint8_t page_count;
    flash_manager_page_t * p_area;
#if !(BACKWARD_COMPATIBLE_PERSISTENCE)
    page_count = CEIL_DIV(p_file->size, FLASH_MANAGER_DATA_PER_PAGE);
#else
    legacy_flash_config_get(p_file, &page_count, &p_area);
    if (p_area == NULL)
#endif
    {
        m_allocated_page_count += page_count;
        p_area = (flash_manager_page_t *)(flash_area_end_get() - (m_allocated_page_count * PAGE_SIZE));
    }
    flash_manager_t * p_manager = &p_file->glue_data.flash_manager;
    const flash_manager_config_t config =
    {
        .write_complete_cb = write_complete_cb,
        .invalidate_complete_cb = invalidate_complete_cb,
        .remove_complete_cb = remove_complete_cb,
        .min_available_space = p_file->size,
        .p_area = p_area,
        .page_count = page_count
    };

    __LOG(LOG_SRC_FM, LOG_LEVEL_DBG3, "Mesh config area: 0x%08x file_id: 0x%04x\n",
          config.p_area, p_file->file_id);
    return flash_manager_add(p_manager, &config);
}

void mesh_config_backend_file_clean(mesh_config_backend_file_t * p_file)
{
    file_remove(p_file);
}

uint32_t mesh_config_backend_record_write(mesh_config_backend_file_t * p_file, const uint8_t * p_data, uint32_t length)
{
    fm_entry_t * p_new_entry = flash_manager_entry_alloc(&p_file->glue_data.flash_manager, p_file->curr_pos, length);

    if (p_new_entry == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    memcpy(p_new_entry->data, p_data, length);
    flash_manager_entry_commit(p_new_entry);
    return NRF_SUCCESS;
}

uint32_t mesh_config_backend_record_erase(mesh_config_backend_file_t * p_file)
{
    const fm_handle_filter_t filter = {
        .mask = 0xffff,
        .match = p_file->curr_pos,
    };
    if (flash_manager_entry_count_get(&p_file->glue_data.flash_manager, &filter) == 0)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    return flash_manager_entry_invalidate(&p_file->glue_data.flash_manager, p_file->curr_pos);
}

uint32_t mesh_config_backend_record_read(mesh_config_backend_file_t * p_file, uint8_t * p_data, uint32_t * p_length)
{
    uint32_t length = *p_length;
    uint32_t status = flash_manager_entry_read(&p_file->glue_data.flash_manager, p_file->curr_pos, p_data, &length);

    if (status != NRF_SUCCESS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (length != ALIGN_VAL(*p_length, WORD_SIZE))
    {
        *p_length = length;
        return NRF_ERROR_INVALID_LENGTH;
    }
    return NRF_SUCCESS;
}

void mesh_config_backend_records_read(mesh_config_backend_file_t * p_file,
                                      mesh_config_backend_iterate_cb_t callback)
{
    NRF_MESH_ASSERT(p_file != NULL);
    NRF_MESH_ASSERT(callback != NULL);
    record_iterator_context_t context = {
        .callback = callback,
        .p_file = p_file,
    };
    (void) flash_manager_entries_read(&p_file->glue_data.flash_manager, NULL, entry_read_cb, &context);
}

uint16_t mesh_config_record_size_calculate(uint16_t entry_size)
{
    return ALIGN_VAL((sizeof(fm_header_t) + entry_size), WORD_SIZE);
}

void mesh_config_backend_flash_usage_get(mesh_config_backend_flash_usage_t * p_usage)
{
    NRF_MESH_ASSERT(p_usage != NULL);
    p_usage->length = (m_allocated_page_count * PAGE_SIZE);
    p_usage->p_start = (const uint32_t *) (flash_area_end_get() - p_usage->length);
}

uint32_t mesh_config_backend_file_power_down_time_get(const mesh_config_file_params_t * p_file)
{
    if (p_file->strategy != MESH_CONFIG_STRATEGY_ON_POWER_DOWN)
    {
        return 0;
    }
    return FLASH_BASE_TIME_OVERHEAD_US + CEIL_DIV(p_file->p_backend_data->size, WORD_SIZE) * FLASH_TIME_PER_WORD_US + p_file->p_backend_data->entry_count * FLASH_ENTRY_TIME_OVERHEAD_US;
}

void mesh_config_backend_power_down(void)
{
    flash_manager_defrag_freeze();
}

bool mesh_config_backend_is_there_power_down_place(mesh_config_backend_file_t * p_file)
{
    flash_manager_t * p_manager = &p_file->glue_data.flash_manager;
    uint32_t remaining_place = (uint32_t)((uint8_t *) get_area_end(p_manager->config.p_area) -
            (uint8_t *) p_manager->internal.p_seal);

    return p_manager->config.min_available_space <= remaining_place;
}
