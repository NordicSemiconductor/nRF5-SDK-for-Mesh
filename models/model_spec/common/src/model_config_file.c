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

#include "model_common.h"
#include "model_config_file.h"

#include <stdint.h>

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_events.h"
#include "utils.h"
#include "mesh_opt.h"
#include "mesh_config.h"
#include "mesh_stack.h"

/* This stucture should be updated if MESH_OPT_MODEL_FILE_ID is used for storing model data. This
 * structure helps in detecting the changes to device composition based on individual model instances
 * to reset the file data automatically after DFU when changes are detected. */
typedef struct
{
    uint16_t    light_lightness_instance_count;
    uint16_t    light_lc_instance_count;
    uint16_t    light_ctl_instance_count;
    uint16_t    generic_onoff_root_only_instance_count;
    uint16_t    generic_level_root_only_instance_count;
    uint16_t    generic_dtt_root_only_instance_count;
    uint16_t    scene_instance_count;
} model_config_file_metadata_t;

typedef struct
{
    uint8_t is_metadata_stored : 1;
    uint8_t is_load_failed : 1;
} model_config_file_status_t;

#define MODEL_CONFIG_FILE_METADATA_EID   MESH_CONFIG_ENTRY_ID(MESH_OPT_MODEL_FILE_ID, MESH_APP_MODEL_CONFIG_ID)

MESH_CONFIG_FILE(m_model_storage, MESH_OPT_MODEL_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

static uint32_t model_config_file_metadata_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     model_config_file_metadata_getter(mesh_config_entry_id_t id, void * p_entry);

MESH_CONFIG_ENTRY(m_model_config_file_metadata_entry,
                  MODEL_CONFIG_FILE_METADATA_EID,
                  1,
                  sizeof(model_config_file_metadata_t),
                  model_config_file_metadata_setter,
                  model_config_file_metadata_getter,
                  NULL,
                  true);

static model_config_file_status_t m_status;
/** Mesh event handler */
static nrf_mesh_evt_handler_t m_mesh_evt_handler;

static void metadata_store(void)
{
    mesh_config_entry_id_t entry_id = MODEL_CONFIG_FILE_METADATA_EID;
    model_config_file_metadata_t metadata =
    {
        .light_lightness_instance_count = LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
        .light_lc_instance_count = LIGHT_LC_SETUP_SERVER_INSTANCES_MAX,
        .light_ctl_instance_count = LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX,
        .generic_onoff_root_only_instance_count = GENERIC_ONOFF_SERVER_INSTANCES_MAX,
        .generic_level_root_only_instance_count = GENERIC_LEVEL_SERVER_INSTANCES_MAX,
        .generic_dtt_root_only_instance_count = GENERIC_DTT_SERVER_INSTANCES_MAX,
        .scene_instance_count = SCENE_SETUP_SERVER_INSTANCES_MAX
    };

    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &metadata));
}

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            if (p_evt->params.config_load_failure.id.file == MESH_OPT_MODEL_FILE_ID)
            {
                m_status.is_load_failed = 1;
            }
            break;

        default:
            break;
    }
}

/* Setter and getter definitions.
 */
static uint32_t model_config_file_metadata_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_APP_MODEL_CONFIG_ID == id.record);

    const model_config_file_metadata_t * p_metadata = (const model_config_file_metadata_t *) p_entry;

    if ((p_metadata->light_lightness_instance_count == LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX) &&
        (p_metadata->light_lc_instance_count == LIGHT_LC_SETUP_SERVER_INSTANCES_MAX) &&
        (p_metadata->light_ctl_instance_count == LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX) &&
        (p_metadata->generic_onoff_root_only_instance_count == GENERIC_ONOFF_SERVER_INSTANCES_MAX) &&
        (p_metadata->generic_level_root_only_instance_count == GENERIC_LEVEL_SERVER_INSTANCES_MAX) &&
        (p_metadata->generic_dtt_root_only_instance_count == GENERIC_DTT_SERVER_INSTANCES_MAX) &&
        (p_metadata->scene_instance_count == SCENE_SETUP_SERVER_INSTANCES_MAX))
    {
        m_status.is_metadata_stored = 1;
    }
    else
    {
        return NRF_ERROR_INVALID_DATA;
    }

    return NRF_SUCCESS;
}

static void model_config_file_metadata_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_APP_MODEL_CONFIG_ID == id.record);

    model_config_file_metadata_t * p_metadata = (model_config_file_metadata_t *) p_entry;
    p_metadata->light_lightness_instance_count = LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX;
    p_metadata->light_lc_instance_count = LIGHT_LC_SETUP_SERVER_INSTANCES_MAX;
    p_metadata->light_ctl_instance_count = LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX;
    p_metadata->generic_onoff_root_only_instance_count = GENERIC_ONOFF_SERVER_INSTANCES_MAX;
    p_metadata->generic_level_root_only_instance_count = GENERIC_LEVEL_SERVER_INSTANCES_MAX;
    p_metadata->generic_dtt_root_only_instance_count = GENERIC_DTT_SERVER_INSTANCES_MAX;
    p_metadata->scene_instance_count = SCENE_SETUP_SERVER_INSTANCES_MAX;
}


/* Weak functions declearation */
__WEAK void generic_level_mc_init(void)
{}

__WEAK void generic_onoff_mc_init(void)
{}

__WEAK void light_lightness_mc_init(void)
{}

__WEAK void light_lc_mc_init(void)
{}

__WEAK void light_ctl_mc_init(void)
{}

__WEAK void scene_mc_init(void)
{}

__WEAK void generic_dtt_mc_init(void)
{}

__WEAK void generic_level_mc_clear(void)
{}

__WEAK void generic_onoff_mc_clear(void)
{}

__WEAK void light_lightness_mc_clear(void)
{}

__WEAK void light_lc_mc_clear(void)
{}

__WEAK void light_ctl_mc_clear(void)
{}

__WEAK void scene_mc_clear(void)
{}

__WEAK void generic_dtt_mc_clear(void)
{}

static void model_config_clear(void)
{
    generic_level_mc_clear();
    generic_onoff_mc_clear();
    light_lightness_mc_clear();
    light_lc_mc_clear();
    light_ctl_mc_clear();
    scene_mc_clear();
    generic_dtt_mc_clear();
}

void model_config_file_init(void)
{
    m_mesh_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

    m_status.is_load_failed = 0;

    generic_level_mc_init();
    generic_onoff_mc_init();
    light_lightness_mc_init();
    light_lc_mc_init();
    light_ctl_mc_init();
    scene_mc_init();
    generic_dtt_mc_init();
}

uint32_t model_config_file_config_apply(void)
{
    if (m_status.is_load_failed)
    {
        /* Loading of the model failed in some way, so the stack config also need to be cleared */
        mesh_stack_config_clear();
        model_config_clear();

        m_status.is_metadata_stored = 0;
        (void) mesh_config_entry_delete(MODEL_CONFIG_FILE_METADATA_EID);

        metadata_store();
        return NRF_ERROR_INVALID_DATA;
    }

    if (!m_status.is_metadata_stored)
    {
        /* Store default values */
        metadata_store();
    }

    return NRF_SUCCESS;
}

void model_config_file_clear(void)
{
    model_config_clear();

    m_status.is_metadata_stored = 0;
    m_status.is_load_failed = 0;
    mesh_config_file_clear(MESH_OPT_MODEL_FILE_ID);
}
