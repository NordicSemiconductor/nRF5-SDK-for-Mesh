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

#include "generic_dtt_mc.h"
#include "model_common.h"

#include "nrf_mesh_config_app.h"

#include "mesh_config_entry.h"
#include "mesh_config.h"
#include "mesh_opt.h"

#include "nrf_mesh_assert.h"

typedef struct
{
    uint32_t dtt;
} dtt_flash_storage_state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a light lightness setup server instance.
*/
static uint32_t dtt_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     dtt_getter(mesh_config_entry_id_t id, void * p_entry);

NRF_MESH_STATIC_ASSERT((MESH_APP_MODEL_GENERIC_DTT_ID_START + \
                        GENERIC_DTT_SERVER_INSTANCES_MAX - 1) \
                       <= MESH_APP_MODEL_GENERIC_DTT_ID_END);

/* A mesh config entry associates a state variable with DTT entry instances
 * file locations each identified by a unique entry ID. Given an integer i in
 * [0, FLASH_GENERIC_DTT_INSTANCES-1], base ID + i identifies the i-th instance. The
 * entry also associates the base entry ID with previously declared setter and
 * getter functions.
 */
MESH_CONFIG_ENTRY(m_dtt_entry,
                  GENERIC_DTT_DTT_EID,
                  GENERIC_DTT_SERVER_INSTANCES_MAX,
                  sizeof(uint32_t),
                  dtt_setter,
                  dtt_getter,
                  NULL,
                  true);

/* An array for mapping from a handle to a state pointer.
 */
static dtt_flash_storage_state_t m_state_contexts[GENERIC_DTT_SERVER_INSTANCES_MAX];
static uint8_t m_next_handle;

static dtt_flash_storage_state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static dtt_flash_storage_state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static void state_contexts_default_set(uint8_t handle)
{
    m_state_contexts[handle].dtt               = GENERIC_DTT_DEFAULT_DTT;
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void dtt_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
        const uint16_t start;
    } entries[] = {
        {&GENERIC_DTT_DTT_EID, GENERIC_DTT_DTT_EID_START},
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < m_next_handle; i++)
        {
            id.record = entries[j].start + i;
            (void) mesh_config_entry_delete(id);
        }
    }
}

/* Setter and getter definitions.
 */
static uint32_t dtt_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint32_t * p_value = (const uint32_t *) p_entry;

    model_context_get(id.record, GENERIC_DTT_DTT_EID_START)->dtt = *p_value;

    return NRF_SUCCESS;
}

static void dtt_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint32_t * p_value = (uint32_t *) p_entry;

    *p_value = model_context_get(id.record, GENERIC_DTT_DTT_EID_START)->dtt;
}

uint32_t generic_dtt_mc_dtt_state_set(uint8_t index, uint32_t value)
{
    mesh_config_entry_id_t id = GENERIC_DTT_DTT_EID;

    id.record += index;
    return mesh_config_entry_set(id, &value);
}

uint32_t generic_dtt_mc_dtt_state_get(uint8_t index, uint32_t * p_value)
{
    mesh_config_entry_id_t id = GENERIC_DTT_DTT_EID;

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}


uint32_t generic_dtt_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (GENERIC_DTT_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void generic_dtt_mc_clear(void)
{
    dtt_flash_storage_config_clear();
}

void generic_dtt_mc_init(void)
{
    state_contexts_all_default_set();
}
