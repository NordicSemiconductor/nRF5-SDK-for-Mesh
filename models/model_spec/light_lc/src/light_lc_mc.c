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

#include "light_lc_mc.h"

#include "nrf_mesh_config_app.h"

#include "mesh_opt.h"

#include "light_lc_common.h"
#include "light_lc_state_utils.h"

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define STORED_WITH_SCENE_STATE    (1 + SCENE_REGISTER_ARRAY_SIZE)
#else
#define STORED_WITH_SCENE_STATE    (1)
#endif

typedef struct
{
    uint8_t lc_mode[STORED_WITH_SCENE_STATE];
    uint8_t lc_occ_mode[STORED_WITH_SCENE_STATE];
    uint8_t lc_light_onoff[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_luxlevel_on[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_luxlevel_prolong[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_luxlevel_standby[STORED_WITH_SCENE_STATE];
    uint16_t lc_pr_lightness_on[STORED_WITH_SCENE_STATE];
    uint16_t lc_pr_lightness_prolong[STORED_WITH_SCENE_STATE];
    uint16_t lc_pr_lightness_standby[STORED_WITH_SCENE_STATE];
    uint8_t lc_pr_regulator_accuracy[STORED_WITH_SCENE_STATE];
    float lc_pr_regulator_kid[STORED_WITH_SCENE_STATE];
    float lc_pr_regulator_kiu[STORED_WITH_SCENE_STATE];
    float lc_pr_regulator_kpd[STORED_WITH_SCENE_STATE];
    float lc_pr_regulator_kpu[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_fade[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_fade_on[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_fade_standby_auto[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_fade_standby_manual[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_occupancy_delay[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_prolong[STORED_WITH_SCENE_STATE];
    uint32_t lc_pr_time_run_on[STORED_WITH_SCENE_STATE];
} lc_flash_storage_state_t;

/* Setter and getter declarations.
 * A setter or a getter, respectively, stores or retrieves a state variable value from
 * a primary memory location defined by a LC setup server instance.
*/
static uint32_t lc_mode_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_mode_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_occ_mode_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_occ_mode_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_light_onoff_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_light_onoff_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_luxlevel_on_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_luxlevel_on_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_luxlevel_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_luxlevel_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_luxlevel_standby_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_luxlevel_standby_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_lightness_on_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_lightness_on_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_lightness_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_lightness_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_lightness_standby_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_lightness_standby_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_regulator_accuracy_state_setter(mesh_config_entry_id_t id,
                                                      const void * p_entry);
static void     lc_pr_regulator_accuracy_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_regulator_kid_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_regulator_kid_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_regulator_kiu_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_regulator_kiu_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_regulator_kpd_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_regulator_kpd_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_regulator_kpu_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_regulator_kpu_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_time_fade_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_time_fade_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_time_fade_on_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_time_fade_on_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_time_fade_standby_auto_state_setter(mesh_config_entry_id_t id,
                                                          const void * p_entry);
static void     lc_pr_time_fade_standby_auto_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_time_fade_standby_manual_state_setter(mesh_config_entry_id_t id,
                                                            const void * p_entry);
static void     lc_pr_time_fade_standby_manual_state_getter(mesh_config_entry_id_t id,
                                                            void * p_entry);

static uint32_t lc_pr_time_occupancy_delay_state_setter(mesh_config_entry_id_t id,
                                                        const void * p_entry);
static void     lc_pr_time_occupancy_delay_state_getter(mesh_config_entry_id_t id, void * p_entry);


static uint32_t lc_pr_time_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_time_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry);

static uint32_t lc_pr_time_run_on_state_setter(mesh_config_entry_id_t id, const void * p_entry);
static void     lc_pr_time_run_on_state_getter(mesh_config_entry_id_t id, void * p_entry);

NRF_MESH_STATIC_ASSERT(MESH_APP_MODEL_LIGHT_LC_SERVER_ID_END >= (LIGHT_LC_PR_TIME_RUN_ON_EID_START + LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES));

/* A mesh config entry associates a state variable with LIGHT_LC_SETUP_SERVER_INSTANCES_MAX
 * file locations each identified by a unique entry ID. Given an integer i in
 * [0, LIGHT_LC_SETUP_SERVER_INSTANCES_MAX-1], base ID + i identifies the i-th instance. The
 * entry also associates the base entry ID with previously declared setter and
 * getter functions.
 */

MESH_CONFIG_ENTRY(m_lc_mode_entry,
                  LIGHT_LC_MODE_EID,      /* The base entry id */
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,   /* The number of instances. */
                  sizeof(uint8_t),       /* The size of an instance. */
                  lc_mode_state_setter,  /* Stores a value in primary memory. */
                  lc_mode_state_getter,  /* Retrieves a value from primary memory. */
                  NULL,                  /* No need for a delete callback */
                  true);                 /* There is a default value */

MESH_CONFIG_ENTRY(m_lc_occ_mode_entry,
                  LIGHT_LC_OCC_MODE_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint8_t),
                  lc_occ_mode_state_setter,
                  lc_occ_mode_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_light_onoff_entry,
                  LIGHT_LC_LIGHT_ONOFF_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint8_t),
                  lc_light_onoff_state_setter,
                  lc_light_onoff_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_luxlevel_on_entry,
                  LIGHT_LC_PR_LUXLEVEL_ON_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_luxlevel_on_state_setter,
                  lc_pr_luxlevel_on_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_luxlevel_prolong_entry,
                  LIGHT_LC_PR_LUXLEVEL_PROLONG_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_luxlevel_prolong_state_setter,
                  lc_pr_luxlevel_prolong_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_luxlevel_standby_entry,
                  LIGHT_LC_PR_LUXLEVEL_STANDBY_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_luxlevel_standby_state_setter,
                  lc_pr_luxlevel_standby_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_lightness_on_entry,
                  LIGHT_LC_PR_LIGHTNESS_ON_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint16_t),
                  lc_pr_lightness_on_state_setter,
                  lc_pr_lightness_on_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_lightness_prolong_entry,
                  LIGHT_LC_PR_LIGHTNESS_PROLONG_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint16_t),
                  lc_pr_lightness_prolong_state_setter,
                  lc_pr_lightness_prolong_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_lightness_standby_entry,
                  LIGHT_LC_PR_LIGHTNESS_STANDBY_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint16_t),
                  lc_pr_lightness_standby_state_setter,
                  lc_pr_lightness_standby_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_regulator_accuracy_entry,
                  LIGHT_LC_PR_REGULATOR_ACCURACY_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint8_t),
                  lc_pr_regulator_accuracy_state_setter,
                  lc_pr_regulator_accuracy_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_regulator_kid_entry,
                  LIGHT_LC_PR_REGULATOR_KID_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(float),
                  lc_pr_regulator_kid_state_setter,
                  lc_pr_regulator_kid_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_regulator_kiu_entry,
                  LIGHT_LC_PR_REGULATOR_KIU_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(float),
                  lc_pr_regulator_kiu_state_setter,
                  lc_pr_regulator_kiu_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_regulator_kpd_entry,
                  LIGHT_LC_PR_REGULATOR_KPD_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(float),
                  lc_pr_regulator_kpd_state_setter,
                  lc_pr_regulator_kpd_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_regulator_kpu_entry,
                  LIGHT_LC_PR_REGULATOR_KPU_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(float),
                  lc_pr_regulator_kpu_state_setter,
                  lc_pr_regulator_kpu_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_fade_entry,
                  LIGHT_LC_PR_TIME_FADE_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_fade_state_setter,
                  lc_pr_time_fade_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_fade_on_entry,
                  LIGHT_LC_PR_TIME_FADE_ON_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_fade_on_state_setter,
                  lc_pr_time_fade_on_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_fade_standby_auto_entry,
                  LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_fade_standby_auto_state_setter,
                  lc_pr_time_fade_standby_auto_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_fade_standby_manual_entry,
                  LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_fade_standby_manual_state_setter,
                  lc_pr_time_fade_standby_manual_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_occupancy_delay_entry,
                  LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_occupancy_delay_state_setter,
                  lc_pr_time_occupancy_delay_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_prolong_entry,
                  LIGHT_LC_PR_TIME_PROLONG_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_prolong_state_setter,
                  lc_pr_time_prolong_state_getter,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(m_lc_pr_time_run_on_entry,
                  LIGHT_LC_PR_TIME_RUN_ON_EID,
                  LIGHT_LC_SETUP_SERVER_STORED_WITH_SCENE_STATES,
                  sizeof(uint32_t),
                  lc_pr_time_run_on_state_setter,
                  lc_pr_time_run_on_state_getter,
                  NULL,
                  true);

/* An array for mapping from a handle to a state pointer.
 */
static lc_flash_storage_state_t m_state_contexts[LIGHT_LC_SETUP_SERVER_INSTANCES_MAX];
static uint8_t m_next_handle;

static lc_flash_storage_state_t * context_get(uint8_t i)
{
    NRF_MESH_ASSERT(i < m_next_handle);
    return &m_state_contexts[i];
}

/*  Maps from (ID record, ID record base)-pairs to state pointer.
 */
static lc_flash_storage_state_t * model_context_get(uint16_t address, uint16_t base)
{
    NRF_MESH_ASSERT(base <= address);
    return context_get(address - base);
}

static void state_contexts_default_set(uint8_t handle)
{

    for(uint32_t i = 0; i < STORED_WITH_SCENE_STATE; i++)
    {
        m_state_contexts[handle].lc_mode[i] = LIGHT_LC_DEFAULT_MODE;
        m_state_contexts[handle].lc_occ_mode[i] = LIGHT_LC_DEFAULT_OCC_MODE;
        m_state_contexts[handle].lc_light_onoff[i] = LIGHT_LC_DEFAULT_LIGHT_ONOFF;
        m_state_contexts[handle].lc_pr_luxlevel_on[i] = LIGHT_LC_DEFAULT_PR_LUXLEVEL_ON;
        m_state_contexts[handle].lc_pr_luxlevel_prolong[i] = LIGHT_LC_DEFAULT_PR_LUXLEVEL_PROLONG;
        m_state_contexts[handle].lc_pr_luxlevel_standby[i] = LIGHT_LC_DEFAULT_PR_LUXLEVEL_STANDBY;
        m_state_contexts[handle].lc_pr_lightness_on[i] = LIGHT_LC_DEFAULT_PR_LIGHTNESS_ON;
        m_state_contexts[handle].lc_pr_lightness_prolong[i] = LIGHT_LC_DEFAULT_PR_LIGHTNESS_PROLONG;
        m_state_contexts[handle].lc_pr_lightness_standby[i] = LIGHT_LC_DEFAULT_PR_LIGHTNESS_STANDBY;
        m_state_contexts[handle].lc_pr_regulator_accuracy[i] = LIGHT_LC_DEFAULT_PR_REGULATOR_ACCURACY;
        m_state_contexts[handle].lc_pr_regulator_kid[i] = LIGHT_LC_DEFAULT_PR_REGULATOR_KID;
        m_state_contexts[handle].lc_pr_regulator_kiu[i] = LIGHT_LC_DEFAULT_PR_REGULATOR_KIU;
        m_state_contexts[handle].lc_pr_regulator_kpd[i] = LIGHT_LC_DEFAULT_PR_REGULATOR_KPD;
        m_state_contexts[handle].lc_pr_regulator_kpu[i] = LIGHT_LC_DEFAULT_PR_REGULATOR_KPU;
        m_state_contexts[handle].lc_pr_time_fade[i] = LIGHT_LC_DEFAULT_PR_TIME_FADE_MS;
        m_state_contexts[handle].lc_pr_time_fade_on[i] = LIGHT_LC_DEFAULT_PR_TIME_FADE_ON_MS;
        m_state_contexts[handle].lc_pr_time_fade_standby_auto[i] = LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_AUTO_MS;
        m_state_contexts[handle].lc_pr_time_fade_standby_manual[i] = LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_MANUAL_MS;
        m_state_contexts[handle].lc_pr_time_occupancy_delay[i] = LIGHT_LC_DEFAULT_PR_TIME_OCCUPANCY_DELAY_MS;
        m_state_contexts[handle].lc_pr_time_prolong[i] = LIGHT_LC_DEFAULT_PR_TIME_PROLONG_MS;
        m_state_contexts[handle].lc_pr_time_run_on[i] = LIGHT_LC_DEFAULT_PR_TIME_RUN_ON_MS;
    }
}

static void state_contexts_all_default_set()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(m_state_contexts); i++)
    {
        state_contexts_default_set(i);
    }
}

static void lc_flash_storage_config_clear(void)
{
    const struct {
        const mesh_config_entry_id_t * const id;
        const uint16_t start;
    } entries[] = {
        {&LIGHT_LC_MODE_EID, LIGHT_LC_MODE_EID_START},
        {&LIGHT_LC_OCC_MODE_EID, LIGHT_LC_OCC_MODE_EID_START},
        {&LIGHT_LC_LIGHT_ONOFF_EID, LIGHT_LC_LIGHT_ONOFF_EID_START},
        {&LIGHT_LC_PR_LUXLEVEL_ON_EID, LIGHT_LC_PR_LUXLEVEL_ON_EID_START},
        {&LIGHT_LC_PR_LUXLEVEL_PROLONG_EID, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START},
        {&LIGHT_LC_PR_LUXLEVEL_STANDBY_EID, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START},
        {&LIGHT_LC_PR_LIGHTNESS_ON_EID, LIGHT_LC_PR_LIGHTNESS_ON_EID_START},
        {&LIGHT_LC_PR_LIGHTNESS_PROLONG_EID, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START},
        {&LIGHT_LC_PR_LIGHTNESS_STANDBY_EID, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START},
        {&LIGHT_LC_PR_REGULATOR_ACCURACY_EID, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START},
        {&LIGHT_LC_PR_REGULATOR_KID_EID, LIGHT_LC_PR_REGULATOR_KID_EID_START},
        {&LIGHT_LC_PR_REGULATOR_KIU_EID, LIGHT_LC_PR_REGULATOR_KIU_EID_START},
        {&LIGHT_LC_PR_REGULATOR_KPD_EID, LIGHT_LC_PR_REGULATOR_KPD_EID_START},
        {&LIGHT_LC_PR_REGULATOR_KPU_EID, LIGHT_LC_PR_REGULATOR_KPU_EID_START},
        {&LIGHT_LC_PR_TIME_FADE_EID, LIGHT_LC_PR_TIME_FADE_EID_START},
        {&LIGHT_LC_PR_TIME_FADE_ON_EID, LIGHT_LC_PR_TIME_FADE_ON_EID_START},
        {&LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID, LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START},
        {&LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID, LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START},
        {&LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START},
        {&LIGHT_LC_PR_TIME_PROLONG_EID, LIGHT_LC_PR_TIME_PROLONG_EID_START},
        {&LIGHT_LC_PR_TIME_RUN_ON_EID, LIGHT_LC_PR_TIME_RUN_ON_EID_START},
    };

    state_contexts_all_default_set();

    for (uint16_t j=0; j < ARRAY_SIZE(entries); j++)
    {
        mesh_config_entry_id_t id = *entries[j].id;
        for (uint8_t i = 0; i < m_next_handle; i++)
        {
            for (uint8_t k = 0; k < STORED_WITH_SCENE_STATE; k++)
            {
                (void) mesh_config_entry_delete(id);
                id.record++;
            }
        }
    }
}

static void id_record_to_address_array_index(uint16_t id_record, uint16_t start,
                                             uint16_t * p_address, uint8_t * p_array_index)
{
    uint16_t shift = id_record - start;
    *p_array_index = shift / LIGHT_LC_SETUP_SERVER_INSTANCES_MAX;
    *p_address = id_record - (*p_array_index * LIGHT_LC_SETUP_SERVER_INSTANCES_MAX);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static uint16_t lc_instance_index_array_index_to_id_record(uint8_t lc_instance_index,
                                                           uint8_t array_index)
{
    return (lc_instance_index + (LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX * array_index));
}
#endif

static uint32_t lc_mode_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint8_t * p_value = (const uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_MODE_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_MODE_EID_START)->lc_mode[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_mode_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint8_t * p_value = (uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_MODE_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_MODE_EID_START)->lc_mode[array_index];
}

static uint32_t lc_occ_mode_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint8_t * p_value = (const uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_OCC_MODE_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_OCC_MODE_EID_START)->lc_occ_mode[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_occ_mode_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint8_t * p_value = (uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_OCC_MODE_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_OCC_MODE_EID_START)->lc_occ_mode[array_index];
}

static uint32_t lc_light_onoff_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint8_t * p_value = (const uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_LIGHT_ONOFF_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_LIGHT_ONOFF_EID_START)->lc_light_onoff[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_light_onoff_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint8_t * p_value = (uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_LIGHT_ONOFF_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_LIGHT_ONOFF_EID_START)->lc_light_onoff[array_index];
}

static uint32_t lc_pr_luxlevel_on_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_ON_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LUXLEVEL_ON_EID_START)->lc_pr_luxlevel_on[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_luxlevel_on_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_ON_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LUXLEVEL_ON_EID_START)->lc_pr_luxlevel_on[array_index];
}

static uint32_t lc_pr_luxlevel_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START)->lc_pr_luxlevel_prolong[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_luxlevel_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LUXLEVEL_PROLONG_EID_START)->lc_pr_luxlevel_prolong[array_index];
}

static uint32_t lc_pr_luxlevel_standby_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START)->lc_pr_luxlevel_standby[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_luxlevel_standby_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LUXLEVEL_STANDBY_EID_START)->lc_pr_luxlevel_standby[array_index];
}

static uint32_t lc_pr_lightness_on_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint16_t * p_value = (const uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_ON_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LIGHTNESS_ON_EID_START)->lc_pr_lightness_on[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_lightness_on_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint16_t * p_value = (uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_ON_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LIGHTNESS_ON_EID_START)->lc_pr_lightness_on[array_index];
}

static uint32_t lc_pr_lightness_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint16_t * p_value = (const uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START)->lc_pr_lightness_prolong[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_lightness_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint16_t * p_value = (uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LIGHTNESS_PROLONG_EID_START)->lc_pr_lightness_prolong[array_index];
}


static uint32_t lc_pr_lightness_standby_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint16_t * p_value = (const uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START)->lc_pr_lightness_standby[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_lightness_standby_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint16_t * p_value = (uint16_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_LIGHTNESS_STANDBY_EID_START)->lc_pr_lightness_standby[array_index];
}

static uint32_t lc_pr_regulator_accuracy_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint8_t * p_value = (const uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START)->lc_pr_regulator_accuracy[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_regulator_accuracy_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint8_t * p_value = (uint8_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_REGULATOR_ACCURACY_EID_START)->lc_pr_regulator_accuracy[array_index];
}

static uint32_t lc_pr_regulator_kid_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const float * p_value = (const float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KID_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_REGULATOR_KID_EID_START)->lc_pr_regulator_kid[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_regulator_kid_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    float * p_value = (float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KID_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_REGULATOR_KID_EID_START)->lc_pr_regulator_kid[array_index];
}

static uint32_t lc_pr_regulator_kiu_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const float * p_value = (const float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KIU_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_REGULATOR_KIU_EID_START)->lc_pr_regulator_kiu[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_regulator_kiu_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    float * p_value = (float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KIU_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_REGULATOR_KIU_EID_START)->lc_pr_regulator_kiu[array_index];
}

static uint32_t lc_pr_regulator_kpd_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const float * p_value = (const float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KPD_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_REGULATOR_KPD_EID_START)->lc_pr_regulator_kpd[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_regulator_kpd_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    float * p_value = (float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KPD_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_REGULATOR_KPD_EID_START)->lc_pr_regulator_kpd[array_index];
}

static uint32_t lc_pr_regulator_kpu_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const float * p_value = (const float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KPU_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_REGULATOR_KPU_EID_START)->lc_pr_regulator_kpu[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_regulator_kpu_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    float * p_value = (float *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_REGULATOR_KPU_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_REGULATOR_KPU_EID_START)->lc_pr_regulator_kpu[array_index];
}

static uint32_t lc_pr_time_fade_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_TIME_FADE_EID_START)->lc_pr_time_fade[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_fade_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_TIME_FADE_EID_START)->lc_pr_time_fade[array_index];
}

static uint32_t lc_pr_time_fade_on_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_ON_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_TIME_FADE_ON_EID_START)->lc_pr_time_fade_on[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_fade_on_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_ON_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_TIME_FADE_ON_EID_START)->lc_pr_time_fade_on[array_index];
}

static uint32_t lc_pr_time_fade_standby_auto_state_setter(mesh_config_entry_id_t id,
                                                          const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START, &address, &array_index);

    model_context_get(address,
                      LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START)->lc_pr_time_fade_standby_auto[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_fade_standby_auto_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START, &address, &array_index);

    *p_value = model_context_get(address,
                                 LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID_START)->lc_pr_time_fade_standby_auto[array_index];
}

static uint32_t lc_pr_time_fade_standby_manual_state_setter(mesh_config_entry_id_t id,
                                                            const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START, &address, &array_index);

    model_context_get(address,
                      LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START)->lc_pr_time_fade_standby_manual[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_fade_standby_manual_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START, &address, &array_index);

    *p_value = model_context_get(address,
                                 LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID_START)->lc_pr_time_fade_standby_manual[array_index];
}

static uint32_t lc_pr_time_occupancy_delay_state_setter(mesh_config_entry_id_t id,
                                                        const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START)->lc_pr_time_occupancy_delay[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_occupancy_delay_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID_START)->lc_pr_time_occupancy_delay[array_index];
}

static uint32_t lc_pr_time_prolong_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_PROLONG_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_TIME_PROLONG_EID_START)->lc_pr_time_prolong[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_prolong_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_PROLONG_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_TIME_PROLONG_EID_START)->lc_pr_time_prolong[array_index];
}

static uint32_t lc_pr_time_run_on_state_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    const uint32_t * p_value = (const uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_RUN_ON_EID_START, &address, &array_index);

    model_context_get(address, LIGHT_LC_PR_TIME_RUN_ON_EID_START)->lc_pr_time_run_on[array_index] = *p_value;
    return NRF_SUCCESS;
}

static void lc_pr_time_run_on_state_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint16_t address;
    uint8_t array_index;
    uint32_t * p_value = (uint32_t *) p_entry;

    id_record_to_address_array_index(id.record, LIGHT_LC_PR_TIME_RUN_ON_EID_START, &address, &array_index);

    *p_value = model_context_get(address, LIGHT_LC_PR_TIME_RUN_ON_EID_START)->lc_pr_time_run_on[array_index];
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

uint32_t light_lc_mc_state_set(uint8_t index, light_lc_state_t lc_state, const void * p_value)
{
    uint32_t status;
    mesh_config_entry_id_t id;

    status = light_lc_state_utils_flash_id_from_lc_state(lc_state, &id);

    if (status != NRF_SUCCESS)
    {
        return status;
    }

    id.record += index;
    return mesh_config_entry_set(id, p_value);
}

uint32_t light_lc_mc_state_get(uint8_t index, light_lc_state_t lc_state, void * p_value)
{
    uint32_t status;
    mesh_config_entry_id_t id;

    status = light_lc_state_utils_flash_id_from_lc_state(lc_state, &id);

    if (status != NRF_SUCCESS)
    {
        return status;
    }

    id.record += index;
    return mesh_config_entry_get(id, p_value);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
uint32_t light_lc_mc_scene_state_store(uint8_t index, uint8_t scene_index, light_lc_state_t lc_state, const void * p_value)
{
    uint32_t status;
    mesh_config_entry_id_t id;

    status = light_lc_state_utils_flash_id_from_lc_state(lc_state, &id);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    id.record += lc_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_set(id, p_value);
}

uint32_t light_lc_mc_scene_state_recall(uint8_t index, uint8_t scene_index, light_lc_state_t lc_state, void * p_value)
{
    uint32_t status;
    mesh_config_entry_id_t id;

    status = light_lc_state_utils_flash_id_from_lc_state(lc_state, &id);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    id.record += lc_instance_index_array_index_to_id_record(index, (scene_index + 1));

    return mesh_config_entry_get(id, p_value);
}
#endif /* SCENE_SETUP_SERVER_INSTANCES_MAX > 0 */

uint32_t light_lc_mc_open(uint8_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (LIGHT_LC_SETUP_SERVER_INSTANCES_MAX <= m_next_handle)
    {
        return NRF_ERROR_RESOURCES;
    }

    *p_handle = m_next_handle++;
    return NRF_SUCCESS;
}

void light_lc_mc_clear(void)
{
    lc_flash_storage_config_clear();
}

void light_lc_mc_init(void)
{
    state_contexts_all_default_set();
}
