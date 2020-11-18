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

#include "scene_mc.h"

#include "nrf_mesh_config_app.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#include "mesh_config_entry.h"
#include "mesh_config.h"
#include "mesh_opt.h"

#include "nrf_mesh_assert.h"

typedef struct
{
    uint16_t scene_number;
} scene_flash_storage_state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a scene setup server instance.
*/
static uint32_t scene_number_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     scene_number_getter(mesh_config_entry_id_t id, void * p_entry);
static void     scene_number_deleter(mesh_config_entry_id_t id);

NRF_MESH_STATIC_ASSERT( ( MESH_APP_MODEL_SCENE_SERVER_ID_START + \
                          (1 * SCENE_SETUP_SERVER_INSTANCES_MAX * SCENE_REGISTER_ARRAY_SIZE) - 1 ) \
                        <= MESH_APP_MODEL_SCENE_SERVER_ID_END );

/* A mesh config entry associates a state variable with SCENE_SETUP_SERVER_INSTANCES_MAX file
 * locations each identified by a unique entry ID. Given an integer i in
 * [0, SCENE_SETUP_SERVER_INSTANCES_MAX-1], base ID + i identifies the i-th instance. The entry also
 * associates the base entry ID with previously declared setter and getter functions.
 */
MESH_CONFIG_ENTRY(m_scene_number_entry,
                  SCENE_NUMBER_EID,                     /* The base entry id */
                  (SCENE_SETUP_SERVER_INSTANCES_MAX * SCENE_REGISTER_ARRAY_SIZE),            /* The number of instances. */
                  sizeof(uint16_t),                     /* The size of an instance. */
                  scene_number_setter,                  /* Stores a value in primary memory. */
                  scene_number_getter,                  /* Retrieve a value from primary memory. */
                  scene_number_deleter,                 /* Delete a value in primary memory. */
                  true);                                /* There is a default value */

/* An array for mapping from a handle to a state pointer.
 */
static scene_flash_storage_state_t m_state_contexts[SCENE_SETUP_SERVER_INSTANCES_MAX * SCENE_REGISTER_ARRAY_SIZE];
static uint8_t m_next_handle;

static scene_flash_storage_state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle * SCENE_REGISTER_ARRAY_SIZE);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static scene_flash_storage_state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static uint32_t scene_index_for_scene_handle_and_number(uint8_t handle, uint16_t scene_number,
                                                        uint8_t * p_scene_index)
{
    uint8_t start_index = handle * SCENE_REGISTER_ARRAY_SIZE;
    uint8_t scene_index = start_index;
    while ( scene_index < (start_index + SCENE_REGISTER_ARRAY_SIZE) )
    {
        if (m_state_contexts[scene_index].scene_number == scene_number)
        {
            *p_scene_index = scene_index;
            return NRF_SUCCESS;
        }
        scene_index++;
    }
    return NRF_ERROR_NOT_FOUND;
}

static void state_contexts_default_set(uint8_t handle)
{
    m_state_contexts[handle].scene_number = SCENE_NUMBER_NO_SCENE;
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void scene_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
        const uint16_t start;
    } entries[] = {
        {&SCENE_NUMBER_EID, SCENE_NUMBER_EID_START},
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < m_next_handle; i++)
        {
            id.record = entries[j].start + (i * SCENE_REGISTER_ARRAY_SIZE);
            for(uint8_t k = 0; k < SCENE_REGISTER_ARRAY_SIZE; k++)
            {
                id.record += k;
                (void) mesh_config_entry_delete(id);
            }
        }
    }
    m_next_handle = 0;
}

/* Setter and getter definitions.
 */

static uint32_t scene_number_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint16_t * p_value = (const uint16_t *) p_entry;

    model_context_get(id.record, SCENE_NUMBER_EID_START)->scene_number = *p_value;
    return NRF_SUCCESS;
}

static void scene_number_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t * p_value = (uint16_t *) p_entry;

    *p_value = model_context_get(id.record, SCENE_NUMBER_EID_START)->scene_number;
}

static void scene_number_deleter(mesh_config_entry_id_t id)
{
    model_context_get(id.record, SCENE_NUMBER_EID_START)->scene_number = SCENE_NUMBER_NO_SCENE;
}


/*****************************************************
 * API functions to set, get and delete flash states
 *
 * Note that a mesh config entry associates each secondary memory location with a unique ID and with
 * a setter, getter and a deleter function. mesh_config_entry_set(), mesh_config_entry_get() and
 * mesh_config_entry_delete() will maintain consistency (from the API caller's perspective) between
 * values in primary and secondary memory locations.
 *
 * mesh_config_entry_set() will note an update to the secondary memory location associated with id,
 * and will call the setter associated with this id (by a mesh config entry) to update primary
 * memory.
 *
 * mesh_config_entry_get() will retrieve a value from primary memory by calling the getter
 * associated with this id.
 *
 * mesh_config_entry_delete() will note a delete to the secondary memory location associated with
 * id, and will call the deleter associated with this id (by a mesh config entry) to update primary
 * memory.
 */

uint32_t scene_mc_store(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index)
{
    uint32_t status;
    mesh_config_entry_id_t id = SCENE_NUMBER_EID;
    if (p_scene_index == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Check if given scene_number already exist, if not find empty entry. */
    status = scene_index_for_scene_handle_and_number(handle, scene_number, p_scene_index);
    if (status != NRF_SUCCESS)
    {
        status = scene_index_for_scene_handle_and_number(handle, SCENE_NUMBER_NO_SCENE, p_scene_index);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
    }

    id.record += *p_scene_index;
    return mesh_config_entry_set(id, &scene_number);
}

uint32_t scene_mc_recall(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index)
{
    uint32_t status;
    if (p_scene_index == NULL)
    {
        return NRF_ERROR_NULL;
    }

    status = scene_index_for_scene_handle_and_number(handle, scene_number, p_scene_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    return NRF_SUCCESS;
}

uint32_t scene_mc_scene_number_get(uint8_t handle, uint8_t scene_index, uint16_t * p_scene_number)
{
    if (p_scene_number == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (scene_index >= SCENE_REGISTER_ARRAY_SIZE || handle >= m_next_handle)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    *p_scene_number = m_state_contexts[(handle * SCENE_REGISTER_ARRAY_SIZE) + scene_index].scene_number;
    return NRF_SUCCESS;
}

uint32_t scene_mc_delete(uint8_t handle, uint16_t scene_number, uint8_t * p_scene_index)
{
    uint32_t status;
    mesh_config_entry_id_t id = SCENE_NUMBER_EID;
    if (p_scene_index == NULL)
    {
        return NRF_ERROR_NULL;
    }

    status = scene_index_for_scene_handle_and_number(handle, scene_number, p_scene_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    id.record += *p_scene_index;
    return mesh_config_entry_delete(id);
}

uint32_t scene_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (SCENE_SETUP_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void scene_mc_clear(void)
{
    scene_flash_storage_config_clear();
}

void scene_mc_init(void)
{
    state_contexts_all_default_set();
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0*/
