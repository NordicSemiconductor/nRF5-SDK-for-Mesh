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

#include "light_ctl_mc.h"

#include "nrf_mesh_config_app.h"

#include "mesh_config_entry.h"
#include "mesh_opt.h"

#include "light_ctl_common.h"
#include "light_ctl_utils.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define STORED_WITH_SCENE_STATE    (1 + SCENE_REGISTER_ARRAY_SIZE)
#else
#define STORED_WITH_SCENE_STATE    (1)
#endif

typedef struct
{
    uint32_t temperature32[STORED_WITH_SCENE_STATE];
    int16_t delta_uv[STORED_WITH_SCENE_STATE];
    uint32_t default_temperature32;
    int16_t default_delta_uv;
    light_ctl_temperature_range_set_params_t temperature32_range;
} state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a CTL setup server instance.
 */
static uint32_t temperature32_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     temperature32_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t delta_uv_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     delta_uv_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t default_temperature32_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     default_temperature32_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t default_delta_uv_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     default_delta_uv_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t temperature32_range_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     temperature32_range_getter(mesh_config_entry_id_t id, void * p_entry);

NRF_MESH_STATIC_ASSERT((MESH_APP_MODEL_LIGHT_CTL_SERVER_ID_START + (6 * LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX) - 1) <= MESH_APP_MODEL_LIGHT_CTL_SERVER_ID_END);

NRF_MESH_STATIC_ASSERT(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN >= LIGHT_CTL_TEMPERATURE_MIN_LIMIT);
NRF_MESH_STATIC_ASSERT(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX <= LIGHT_CTL_TEMPERATURE_MAX_LIMIT);
NRF_MESH_STATIC_ASSERT(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX >= LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN);

/* Each element can host only one ctl instance.*/
STATIC_ASSERT(LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX <= ACCESS_ELEMENT_COUNT, "LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX is out of range");

/* A mesh config entry associates a state variable with LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX file locations each identified
 * by a unique entry ID. Given an integer i in [0, LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX-1], base ID + i identifies the i-th
 * instance. The entry also associates the base entry ID with previously declared setter and getter
 * functions.
 */
MESH_CONFIG_ENTRY(m_temperature32_entry,
                  LIGHT_CTL_TEMPERATURE_EID,            /* The base entry id */
                  LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES,             /* The number of instances. */
                  sizeof(uint32_t),          /* The size of an instance. */
                  temperature32_setter,      /* Stores a value in primary memory. */
                  temperature32_getter,      /* Retrieve a value from primary memory. */
                  NULL,                      /* No need for a delete callback */
                  true);                     /* There is a default value */

MESH_CONFIG_ENTRY(m_delta_uv_entry,
                  LIGHT_CTL_DELTA_UV_EID,
                  LIGHT_CTL_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(int16_t),
                  delta_uv_setter,
                  delta_uv_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_default_temperature32_entry,
                  LIGHT_CTL_TEMPERATURE_DEFAULT_EID,
                  LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint32_t),
                  default_temperature32_setter,
                  default_temperature32_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_default_delta_uv_entry,
                  LIGHT_CTL_DELTA_UV_DEFAULT_EID,
                  LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(int16_t),
                  default_delta_uv_setter,
                  default_delta_uv_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_temperature32_range_entry,
                  LIGHT_CTL_TEMPERATURE_RANGE_EID,
                  LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(light_ctl_temperature_range_set_params_t),
                  temperature32_range_setter,
                  temperature32_range_getter,
                  NULL,
                  true);

/* An array for mapping from a handle to a state pointer.
 */
static state_t m_state_contexts[LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX];
static uint8_t m_next_handle;

static state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static void state_contexts_default_set(uint8_t handle)
{
    for(uint32_t i = 0; i < STORED_WITH_SCENE_STATE; i++)
    {
        m_state_contexts[handle].temperature32[i] =
            light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_TEMPERATURE);
        m_state_contexts[handle].delta_uv[i] = LIGHT_CTL_DEFAULT_DELTA_UV;
    }
    m_state_contexts[handle].default_temperature32 =
        light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_TEMPERATURE_DEFAULT);
    m_state_contexts[handle].default_delta_uv = LIGHT_CTL_DEFAULT_DELTA_UV_DEFAULT;
    m_state_contexts[handle].temperature32_range.temperature32_range_min =
        light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN);
    m_state_contexts[handle].temperature32_range.temperature32_range_max =
        light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX);
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void ctl_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
        const uint16_t start;
    } entries[] = {
        {&LIGHT_CTL_TEMPERATURE_EID, LIGHT_CTL_TEMPERATURE_EID_START},
        {&LIGHT_CTL_DELTA_UV_EID, LIGHT_CTL_DELTA_UV_EID_START},
        {&LIGHT_CTL_TEMPERATURE_DEFAULT_EID, LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START},
        {&LIGHT_CTL_DELTA_UV_DEFAULT_EID, LIGHT_CTL_DELTA_UV_DEFAULT_EID_START},
        {&LIGHT_CTL_TEMPERATURE_RANGE_EID, LIGHT_CTL_TEMPERATURE_RANGE_EID_START},
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < m_next_handle; i++)
        {
            if (entries[j].start == LIGHT_CTL_TEMPERATURE_EID_START ||
                entries[j].start == LIGHT_CTL_DELTA_UV_EID_START)
            {
                for (uint32_t k = 0; k < STORED_WITH_SCENE_STATE; k++)
                {
                    id.record = entries[j].start + (i * STORED_WITH_SCENE_STATE) + k;
                    (void) mesh_config_entry_delete(id);
                }
            }
            else
            {
                id.record = entries[j].start + i;
                (void) mesh_config_entry_delete(id);
            }

        }
    }
}

static void id_record_to_address_array_index(uint16_t id_record, uint16_t start,
                                             uint16_t * p_address, uint8_t * p_array_index)
{
    uint16_t shift = id_record - start;
    *p_array_index = shift / LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX;
    *p_address = id_record - (*p_array_index * LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static uint16_t ctl_instance_index_array_index_to_id_record(uint8_t ctl_instance_index,
                                                            uint8_t array_index)
{
    return (ctl_instance_index + (LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX * array_index));
}
#endif


/* Setter and getter definitions.
 */

static uint32_t temperature32_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_CTL_TEMPERATURE_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_CTL_TEMPERATURE_EID_START)->temperature32[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void temperature32_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_CTL_TEMPERATURE_EID_START, &address, &array_index);
    *p_value = model_context_get(address, LIGHT_CTL_TEMPERATURE_EID_START)->temperature32[array_index];
}

static uint32_t delta_uv_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const int16_t * p_value = (const int16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_CTL_DELTA_UV_EID_START, &address, &array_index);
    model_context_get(address, LIGHT_CTL_DELTA_UV_EID_START)->delta_uv[array_index] = *p_value;

    return NRF_SUCCESS;
}

static void delta_uv_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    int16_t * p_value = (int16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_CTL_DELTA_UV_EID_START, &address, &array_index);
    *p_value = model_context_get(address, LIGHT_CTL_DELTA_UV_EID_START)->delta_uv[array_index];
}

static uint32_t default_temperature32_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint32_t * p_value = (const uint32_t *) p_entry;

    model_context_get(id.record, LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START)->default_temperature32 = *p_value;

    return NRF_SUCCESS;
}

static void default_temperature32_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint32_t * p_value = (uint32_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_CTL_TEMPERATURE_DEFAULT_EID_START)->default_temperature32;
}

static uint32_t default_delta_uv_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const int16_t * p_value = (const int16_t *) p_entry;

    model_context_get(id.record, LIGHT_CTL_DELTA_UV_DEFAULT_EID_START)->default_delta_uv = *p_value;

    return NRF_SUCCESS;
}

static void default_delta_uv_getter(mesh_config_entry_id_t id, void * p_entry)
{
    int16_t * p_value = (int16_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_CTL_DELTA_UV_DEFAULT_EID_START)->default_delta_uv;
}

static uint32_t temperature32_range_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const light_ctl_temperature_range_set_params_t * p_value =
        (const light_ctl_temperature_range_set_params_t  *) p_entry;

    model_context_get(id.record, LIGHT_CTL_TEMPERATURE_RANGE_EID_START)->temperature32_range = *p_value;

    return NRF_SUCCESS;
}

static void temperature32_range_getter(mesh_config_entry_id_t id, void * p_entry)
{
    light_ctl_temperature_range_set_params_t * p_value = (light_ctl_temperature_range_set_params_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_CTL_TEMPERATURE_RANGE_EID_START)->temperature32_range;
}

/***************************************************************************************************
 * API functions to set and get flash states
 *
 * Note that a mesh config entry associates each secondary memory location with a unique ID and with
 * a setter and a getter function.  mesh_config_entry_set() and mesh_config_entry_get() will
 * maintain consistency (from the API caller's perspective) between values in primary and secondary
 * memory locations.
 *
 * mesh_config_entry_set() will note an update to the secondary memory location associated with id,
 * and will call the setter associated with this id (by a mesh config entry) to update primary
 * memory.
 *
 * mesh_config_entry_get() will retrieve a value from primary memory by calling the getter
 * associated with this id.
 */

uint32_t light_ctl_mc_temperature32_state_set(uint8_t index, uint32_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_temperature32_state_get(uint8_t index, uint32_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_ctl_mc_delta_uv_state_set(uint8_t index, int16_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_delta_uv_state_get(uint8_t index, int16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_ctl_mc_default_temperature32_state_set(uint8_t index, uint32_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_default_temperature32_state_get(uint8_t index, uint32_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_ctl_mc_default_delta_uv_state_set(uint8_t index, int16_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_default_delta_uv_state_get(uint8_t index, int16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_ctl_mc_temperature32_range_state_set(uint8_t index, light_ctl_temperature_range_set_params_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_RANGE_EID;

    id.record += index;
    return mesh_config_entry_set(id, p_value);
}

uint32_t light_ctl_mc_temperature32_range_state_get(uint8_t index, light_ctl_temperature_range_set_params_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_RANGE_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0)
uint32_t light_ctl_mc_scene_temperature32_state_store(uint8_t index, uint8_t scene_index, uint32_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_EID;

    id.record += ctl_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_scene_temperature32_state_recall(uint8_t index, uint8_t scene_index, uint32_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_TEMPERATURE_EID;

    id.record += ctl_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_get(id, p_value);
}

uint32_t light_ctl_mc_scene_delta_uv_state_store(uint8_t index, uint8_t scene_index, int16_t value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_EID;

    id.record += ctl_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, &value);
}

uint32_t light_ctl_mc_scene_delta_uv_state_recall(uint8_t index, uint8_t scene_index, int16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_CTL_DELTA_UV_EID;

    id.record += ctl_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, p_value);
}
#endif /* (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN) */

uint32_t light_ctl_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void light_ctl_mc_clear(void)
{
    ctl_flash_storage_config_clear();
}

void light_ctl_mc_init(void)
{
    state_contexts_all_default_set();
}
