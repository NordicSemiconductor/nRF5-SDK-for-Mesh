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

#include "generic_level_mc.h"

#include "nrf_mesh_config_app.h"

#include "mesh_config_entry.h"
#include "mesh_config.h"
#include "mesh_opt.h"

#include "nrf_mesh_assert.h"

#include "generic_level_common.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define GENERIC_LEVEL_STATES    (1 + SCENE_REGISTER_ARRAY_SIZE)
#else
#define GENERIC_LEVEL_STATES    (1)
#endif

typedef struct
{
    int16_t level[GENERIC_LEVEL_STATES];
} glevel_flash_storage_state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a generic level server instance.
*/
static uint32_t level_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     level_getter(mesh_config_entry_id_t id, void * p_entry);

NRF_MESH_STATIC_ASSERT(GENERIC_LEVEL_EID_END <= MESH_APP_MODEL_GENERIC_LEVEL_ID_END);
NRF_MESH_STATIC_ASSERT((MESH_APP_MODEL_GENERIC_LEVEL_ID_START + \
                       (GENERIC_LEVEL_SERVER_INSTANCES_MAX) - 1) \
                       <= MESH_APP_MODEL_GENERIC_LEVEL_ID_END);

/* A mesh config entry associates a state variable with GLEVEL_FLASH_INSTANCES
 * file locations each identified by a unique entry ID. Given an integer i in
 * [0, GLEVEL_FLASH_INSTANCES-1], base ID + i identifies the i-th instance. The
 * entry also associates the base entry ID with previously declared setter and
 * getter functions.
 */
MESH_CONFIG_ENTRY(m_glevel_level_entry,
                  GENERIC_LEVEL_EID,                             /* The base entry id */
                  GENERIC_LEVEL_SERVER_STORED_WITH_SCENE_STATES, /* The number of instances. */
                  sizeof(int16_t),                               /* The size of an instance. */
                  level_setter,                       /* Stores a value in primary memory. */
                  level_getter,                       /* Retrieve a value from primary memory. */
                  NULL,                               /* No need for a delete callback */
                  true);                              /* There is a default value */

/* An array for mapping from a handle to a state pointer.
 */
static glevel_flash_storage_state_t m_state_contexts[GENERIC_LEVEL_SERVER_INSTANCES_MAX];
static uint8_t m_next_handle;

static glevel_flash_storage_state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static glevel_flash_storage_state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static void state_contexts_default_set(uint8_t handle)
{
    for (uint32_t i = 0; i < GENERIC_LEVEL_STATES; i++)
    {
        m_state_contexts[handle].level[i] = GENERIC_LEVEL_DEFAULT_LEVEL;
    }
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void glevel_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
    } entries[] = {
        {&GENERIC_LEVEL_EID}
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < (m_next_handle * GENERIC_LEVEL_STATES); i++)
        {
            (void) mesh_config_entry_delete(id);
            id.record++;
        }
    }
}

static void id_record_to_address_array_index(uint16_t id_record, uint16_t * p_address,
                                             uint8_t * p_array_index)
{
    uint16_t shift = id_record - GENERIC_LEVEL_EID_START;
    *p_array_index = shift / GENERIC_LEVEL_SERVER_INSTANCES_MAX;
    *p_address = id_record - (*p_array_index * GENERIC_LEVEL_SERVER_INSTANCES_MAX);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static uint16_t level_instance_scene_index_to_id_record(uint8_t scene_index,
                                                        uint8_t level_instance_index)
{
    return (level_instance_index +  (GENERIC_LEVEL_SERVER_INSTANCES_MAX  * (scene_index + 1)));
}
#endif

/* Setter and getter definitions.
 */

static uint32_t level_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const int16_t * p_value = (const int16_t *) p_entry;

    id_record_to_address_array_index(id.record, &address, &array_index);

    model_context_get(address, GENERIC_LEVEL_EID_START)->level[array_index] = *p_value;

    return NRF_SUCCESS;
}

static void level_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    int16_t * p_value = (int16_t *) p_entry;

    id_record_to_address_array_index(id.record, &address, &array_index);

    *p_value = model_context_get(address, GENERIC_LEVEL_EID_START)->level[array_index];
}

/***********************************************
 * API functions to set and get flash states
 *
 * Note that a mesh config entry associates each secondary memory
 * location with a unique ID and with a setter and a getter function.
 * mesh_config_entry_set() and mesh_config_entry_get() will maintain
 * consistency (from the API caller's perspective) between values
 * in primary and secondary memory locations.
 *
 * mesh_config_entry_set() will note an update to the secondary memory
 * location associated with id, and will call the setter associated
 * with this id (by a mesh config entry) to update primary memory.
 *
 * mesh_config_entry_get() will retrieve a value from primary
 * memory by calling the getter associated with this id.
 */

uint32_t generic_level_mc_level_state_set(uint8_t index, int16_t value)
{
    mesh_config_entry_id_t id = GENERIC_LEVEL_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t generic_level_mc_level_state_get(uint8_t index, int16_t * p_value)
{
    mesh_config_entry_id_t id = GENERIC_LEVEL_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t generic_level_mc_scene_level_store(uint8_t index, uint8_t scene_index, int16_t value)
{
    mesh_config_entry_id_t id = GENERIC_LEVEL_EID;

    id.record += level_instance_scene_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, &value);
}

uint32_t generic_level_mc_scene_level_recall(uint8_t index, uint8_t scene_index, int16_t * p_value)
{
    mesh_config_entry_id_t id = GENERIC_LEVEL_EID;

    id.record += level_instance_scene_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_get(id, p_value);
}
#endif

uint32_t generic_level_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (GENERIC_LEVEL_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void generic_level_mc_clear(void)
{
    glevel_flash_storage_config_clear();
}

void generic_level_mc_init(void)
{
    state_contexts_all_default_set();
}
