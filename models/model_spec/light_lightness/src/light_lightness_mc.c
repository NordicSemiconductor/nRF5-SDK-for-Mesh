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

#include "light_lightness_mc.h"

#include "nrf_mesh_config_app.h"

#include "mesh_config_entry.h"
#include "mesh_config.h"
#include "mesh_opt.h"

#include "nrf_mesh_assert.h"

#include "generic_ponoff_common.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define STORED_WITH_SCENE_STATE (1 + SCENE_REGISTER_ARRAY_SIZE)
#else
#define STORED_WITH_SCENE_STATE (1)
#endif
typedef struct
{
    uint32_t dtt;
    uint16_t last;
    uint16_t actual[STORED_WITH_SCENE_STATE];
    uint16_t default_lightness;
    uint16_t ramonly_state_last;
    uint16_t range_min;
    uint16_t range_max;
    uint8_t range_status;
    uint8_t onpowerup;
    bool last_flash_only_flag;
} ll_flash_storage_state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a light lightness setup server instance.
*/
static uint32_t onpowerup_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     onpowerup_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t last_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     last_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t default_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     default_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t range_status_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     range_status_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t range_min_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     range_min_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t range_max_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     range_max_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t actual_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     actual_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t dtt_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     dtt_getter(mesh_config_entry_id_t id, void * p_entry);

NRF_MESH_STATIC_ASSERT((MESH_APP_MODEL_LIGHT_LIGHTNESS_ID_START + \
                       (9 * LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX) - 1) \
                       <= MESH_APP_MODEL_LIGHT_LIGHTNESS_ID_END);

STATIC_ASSERT(LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN >= LIGHT_LIGHTNESS_RANGE_MIN,
              "LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN is less than allowed.");
STATIC_ASSERT(LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX >= LIGHT_LIGHTNESS_RANGE_MIN,
              "LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX is less than allowed.");
STATIC_ASSERT(LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP <= GENERIC_ON_POWERUP_MAX,
              "LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP is greater than allowed.");
STATIC_ASSERT(LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST >= LIGHT_LIGHTNESS_LAST_MIN,
              "LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST is less than allowed.");

/* A mesh config entry associates a state variable with LL_FLASH_LIGHT_LIGHTNESS_INSTANCES
 * file locations each identified by a unique entry ID. Given an integer i in
 * [0, LL_FLASH_LIGHT_LIGHTNESS_INSTANCES-1], base ID + i identifies the i-th instance. The
 * entry also associates the base entry ID with previously declared setter and
 * getter functions.
 */
MESH_CONFIG_ENTRY(m_onpowerup_entry,
                  LIGHT_LIGHTNESS_ONPOWERUP_EID,                   /* The base entry id */
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,      /* The number of instances. */
                  sizeof(uint8_t),                                 /* The size of an instance. */
                  onpowerup_setter,                                /* Stores a value in primary memory. */
                  onpowerup_getter,                                /* Retrieve a value from primary memory. */
                  NULL,                                            /* No need for a delete callback */
                  true);                                           /* There is a default value */

MESH_CONFIG_ENTRY(m_ll_last_entry,
                  LIGHT_LIGHTNESS_LAST_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint16_t),
                  last_setter,
                  last_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_default_entry,
                  LIGHT_LIGHTNESS_DEFAULT_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint16_t),
                  default_setter,
                  default_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_range_status_entry,
                  LIGHT_LIGHTNESS_RANGE_STATUS_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint8_t),
                  range_status_setter,
                  range_status_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_range_min_entry,
                  LIGHT_LIGHTNESS_RANGE_MIN_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint16_t),
                  range_min_setter,
                  range_min_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_range_max_entry,
                  LIGHT_LIGHTNESS_RANGE_MAX_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint16_t),
                  range_max_setter,
                  range_max_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_actual_entry,
                  LIGHT_LIGHTNESS_ACTUAL_EID,
                  STORED_WITH_SCENE_INSTANCES,
                  sizeof(uint16_t),
                  actual_setter,
                  actual_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_ll_dtt_entry,
                  LIGHT_LIGHTNESS_DTT_EID,
                  LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX,
                  sizeof(uint32_t),
                  dtt_setter,
                  dtt_getter,
                  NULL,
                  true);

/* An array for mapping from a handle to a state pointer.
 */
static ll_flash_storage_state_t m_state_contexts[LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX];
static uint8_t m_next_handle;

static ll_flash_storage_state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static ll_flash_storage_state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static void state_contexts_default_set(uint8_t handle)
{
    m_state_contexts[handle].onpowerup         = LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP;
    m_state_contexts[handle].last              = LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST;

    for(uint32_t i = 0; i < STORED_WITH_SCENE_STATE; i++)
    {
        m_state_contexts[handle].actual[i] = LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_ACTUAL;
    }

    m_state_contexts[handle].default_lightness = LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_DEFAULT;
    m_state_contexts[handle].dtt               = LIGHT_LIGHTNESS_DEFAULT_DTT;
    m_state_contexts[handle].range_status      = LIGHT_LIGHTNESS_RANGE_STATUS_SUCCESS;
    m_state_contexts[handle].range_min         = LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN;
    m_state_contexts[handle].range_max         = LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX;
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void ll_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
        const uint16_t start;
    } entries[] = {
        {&LIGHT_LIGHTNESS_ONPOWERUP_EID, LIGHT_LIGHTNESS_ONPOWERUP_EID_START},
        {&LIGHT_LIGHTNESS_LAST_EID, LIGHT_LIGHTNESS_LAST_EID_START},
        {&LIGHT_LIGHTNESS_DEFAULT_EID, LIGHT_LIGHTNESS_DEFAULT_EID_START},
        {&LIGHT_LIGHTNESS_RANGE_MIN_EID, LIGHT_LIGHTNESS_RANGE_MIN_EID_START},
        {&LIGHT_LIGHTNESS_RANGE_MAX_EID, LIGHT_LIGHTNESS_RANGE_MAX_EID_START},
        {&LIGHT_LIGHTNESS_RANGE_STATUS_EID, LIGHT_LIGHTNESS_RANGE_STATUS_EID_START},
        {&LIGHT_LIGHTNESS_ACTUAL_EID, LIGHT_LIGHTNESS_ACTUAL_EID_START},
        {&LIGHT_LIGHTNESS_DTT_EID, LIGHT_LIGHTNESS_DTT_EID_START},
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < m_next_handle; i++)
        {
            if (entries[j].start != LIGHT_LIGHTNESS_ACTUAL_EID_START)
            {
                id.record = entries[j].start + i;
                (void) mesh_config_entry_delete(id);
            }
            else
            {
                for (uint8_t k = 0; k < STORED_WITH_SCENE_STATE; k++)
                {
                    id.record = entries[j].start + i + (k * LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX);
                    (void) mesh_config_entry_delete(id);
                }
            }
        }
    }
}

static void id_record_to_address_array_index(uint16_t id_record,
                                             uint16_t * p_address, uint8_t * p_array_index)
{
    uint16_t shift = id_record - LIGHT_LIGHTNESS_ACTUAL_EID_START;
    *p_array_index = shift / LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX;
    *p_address = id_record - (*p_array_index * LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static uint16_t ll_instance_index_array_index_to_id_record(uint8_t ll_instance_index,
                                                       uint8_t array_index)
{
    return (ll_instance_index + (LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX * array_index));
}
#endif

/* Setter and getter definitions.
 */

static uint32_t onpowerup_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint8_t * p_value = (const uint8_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_ONPOWERUP_EID_START)->onpowerup = *p_value;
    return NRF_SUCCESS;
}

static void onpowerup_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint8_t * p_value = (uint8_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_ONPOWERUP_EID_START)->onpowerup;
}

static uint32_t last_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint16_t * p_value = (const uint16_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_LAST_EID_START)->last = *p_value;

    return NRF_SUCCESS;
}

static void last_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t * p_value = (uint16_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_LAST_EID_START)->last;
}

static uint32_t default_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint16_t * p_value = (const uint16_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_DEFAULT_EID_START)->default_lightness = *p_value;

    return NRF_SUCCESS;
}

static void default_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t * p_value = (uint16_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_DEFAULT_EID_START)->default_lightness;
}

static uint32_t range_status_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint8_t * p_value = (const uint8_t  *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_STATUS_EID_START)->range_status = *p_value;

    return NRF_SUCCESS;
}

static void range_status_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint8_t * p_value = (uint8_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_STATUS_EID_START)->range_status;
}

static uint32_t range_min_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint16_t * p_value = (const uint16_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_MIN_EID_START)->range_min = *p_value;

    return NRF_SUCCESS;
}

static void range_min_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t * p_value = (uint16_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_MIN_EID_START)->range_min;
}

static uint32_t range_max_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint16_t * p_value = (const uint16_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_MAX_EID_START)->range_max = *p_value;

    return NRF_SUCCESS;
}

static void range_max_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t * p_value = (uint16_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_RANGE_MAX_EID_START)->range_max;
}

static uint32_t actual_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint16_t * p_value = (const uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, &address, &array_index);

    model_context_get(address, LIGHT_LIGHTNESS_ACTUAL_EID_START)->actual[array_index] = *p_value;

    return NRF_SUCCESS;
}

static void actual_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint16_t * p_value = (uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LIGHTNESS_ACTUAL_EID_START)->actual[array_index];
}

static uint32_t dtt_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint32_t * p_value = (const uint32_t *) p_entry;

    model_context_get(id.record, LIGHT_LIGHTNESS_DTT_EID_START)->dtt = *p_value;

    return NRF_SUCCESS;
}

static void dtt_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint32_t * p_value = (uint32_t *) p_entry;

    *p_value = model_context_get(id.record, LIGHT_LIGHTNESS_DTT_EID_START)->dtt;
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

uint32_t light_lightness_mc_onpowerup_state_set(uint8_t index, uint8_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ONPOWERUP_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_onpowerup_state_get(uint8_t index, uint8_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ONPOWERUP_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}


/* @tagMeshMdlSp section 3.1.4: store the target of actual to last's flash
 * storage at the start of a transition to actual (without changing the state
 * variable for last until the transition completes). When the transition
 * completes, update both the flash and state variable to the value of latest.
 *
 * The write_to parameter supports this behavior.
 * To write to flash only (e.g. writing the target value at the
 * start of a transition), the caller passes write_to as FLASH_ONLY.
 * For a normal write to last (not at the start of a transition),
 * the caller passes write_to as WRITE_ALL.
 */
uint32_t light_lightness_mc_last_state_set(uint8_t index,
                                         uint16_t value,
                                         light_lightness_mc_write_destination_t write_to)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_LAST_EID;

    id.record += index;

    if (value < LIGHT_LIGHTNESS_LAST_MIN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (write_to == LIGHT_LIGHTNESS_MC_WRITE_DESTINATION_FLASH_ONLY)
    {
        context_get(index)->last_flash_only_flag = true;
    }
    else
    {
        context_get(index)->ramonly_state_last = value;
        context_get(index)->last_flash_only_flag = false;
    }

    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_last_state_get(uint8_t index, uint16_t * p_value)
{
    uint32_t status = NRF_SUCCESS;
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_LAST_EID;

    id.record += index;

    /* If the flash was written without the state var, and a request
     * has been made for the last state var, return just the state
     * var */
    if (context_get(index)->last_flash_only_flag)
    {
        *p_value = context_get(index)->ramonly_state_last;
    }
    else
    {
        status = mesh_config_entry_get(id, p_value);
        if (status == NRF_SUCCESS)
        {
            /* set this to the same as flash unless we intentionally are
             * tracking ram and flash separately */
            context_get(index)->ramonly_state_last = *p_value;
        }
    }

    return status;
}

uint32_t light_lightness_mc_default_state_set(uint8_t index, uint16_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_default_state_get(uint8_t index, uint16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_DEFAULT_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_lightness_mc_range_status_state_set(uint8_t index, uint8_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_STATUS_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_range_status_state_get(uint8_t index, uint8_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_STATUS_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_lightness_mc_range_min_state_set(uint8_t index, uint16_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_MIN_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_range_min_state_get(uint8_t index, uint16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_MIN_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_lightness_mc_range_max_state_set(uint8_t index, uint16_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_MAX_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_range_max_state_get(uint8_t index, uint16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_RANGE_MAX_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_lightness_mc_actual_state_set(uint8_t index, uint16_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ACTUAL_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_actual_state_get(uint8_t index, uint16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ACTUAL_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

uint32_t light_lightness_mc_dtt_state_set(uint8_t index, uint32_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_DTT_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_dtt_state_get(uint8_t index, uint32_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_DTT_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t light_lightness_mc_scene_actual_state_store(uint8_t index, uint8_t scene_index, 
                                                     uint16_t value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ACTUAL_EID;

    id.record += ll_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, &value);
}

uint32_t light_lightness_mc_scene_actual_state_recall(uint8_t index, uint8_t scene_index, 
                                                      uint16_t * p_value)
{
    mesh_config_entry_id_t id = LIGHT_LIGHTNESS_ACTUAL_EID;

    id.record += ll_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_get(id, p_value);
}
#endif

uint32_t light_lightness_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void light_lightness_mc_clear(void)
{
    ll_flash_storage_config_clear();
}

void light_lightness_mc_init(void)
{
    state_contexts_all_default_set();
}
