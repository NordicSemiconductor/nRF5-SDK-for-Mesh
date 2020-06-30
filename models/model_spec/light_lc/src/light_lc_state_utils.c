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

#include "light_lc_state_utils.h"

#include "mesh_opt.h"

#include "light_lc_server_property_constants.h"
#include "light_lc_mc.h"
#include "mesh_config_entry.h"

/* Set to 1 to define string names for the properties. */
#ifndef LIGHT_LC_STATE_UTILS_DEBUG
#define LIGHT_LC_STATE_UTILS_DEBUG 0
#endif

#if LIGHT_LC_STATE_UTILS_DEBUG
#define PROPERTY_STRING(x) x,
#else
#define PROPERTY_STRING(x)
#endif /* LIGHT_LC_STATE_UTILS_DEBUG */

NRF_MESH_STATIC_ASSERT((MESH_APP_MODEL_LIGHT_LC_SERVER_ID_START + (0x16 * LIGHT_LC_SETUP_SERVER_INSTANCES_MAX) - 1) <= MESH_APP_MODEL_LIGHT_LC_SERVER_ID_END);

/* Property section of lc state utils */
typedef struct
{
    uint16_t property_id;
    uint8_t data_size;
    const mesh_config_entry_id_t * flash_var_id;
#if LIGHT_LC_STATE_UTILS_DEBUG
    char * name_string;
#endif
    light_lc_state_t lc_state;
} property_data_t;

/* The table linking together lc states, flash entry ids, state size, and property id (if
 * applicable) */
static const property_data_t m_property_info[] =
{
    {
        LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID, LIGHT_LC_SERVER_ILLUMINANCE_24_SIZE,
        &LIGHT_LC_PR_LUXLEVEL_ON_EID, PROPERTY_STRING("LUXLEVEL_ON")
        LIGHT_LC_STATE_AMBIENT_LUXLEVEL_ON,
    },
    {
        LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_PROLONG_PID,  LIGHT_LC_SERVER_ILLUMINANCE_24_SIZE,
        &LIGHT_LC_PR_LUXLEVEL_PROLONG_EID, PROPERTY_STRING("LUXLEVEL_PROLONG")
        LIGHT_LC_STATE_AMBIENT_LUXLEVEL_PROLONG,
    },
    {
        LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID, LIGHT_LC_SERVER_ILLUMINANCE_24_SIZE,
        &LIGHT_LC_PR_LUXLEVEL_STANDBY_EID, PROPERTY_STRING("LUXLEVEL_STANDBY")
        LIGHT_LC_STATE_AMBIENT_LUXLEVEL_STANDBY,
    },
    {
        LIGHT_LC_SERVER_LIGHTNESS_ON_PID, LIGHT_LC_SERVER_PERCEIVED_LIGHTNESS_SIZE,
        &LIGHT_LC_PR_LIGHTNESS_ON_EID, PROPERTY_STRING("LIGHTNESS_ON")
        LIGHT_LC_STATE_LIGHTNESS_ON,
    },
    {
        LIGHT_LC_SERVER_LIGHTNESS_PROLONG_PID, LIGHT_LC_SERVER_PERCEIVED_LIGHTNESS_SIZE,
        &LIGHT_LC_PR_LIGHTNESS_PROLONG_EID, PROPERTY_STRING("LIGHTNESS_PROLONG")
        LIGHT_LC_STATE_LIGHTNESS_PROLONG,
    },
    {
        LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID, LIGHT_LC_SERVER_PERCEIVED_LIGHTNESS_SIZE,
        &LIGHT_LC_PR_LIGHTNESS_STANDBY_EID, PROPERTY_STRING("LIGHTNESS_STANDBY")
        LIGHT_LC_STATE_LIGHTNESS_STANDBY,
    },
    {
        LIGHT_LC_SERVER_REGULATOR_ACCURACY_PID, LIGHT_LC_SERVER_PERCENTAGE_8_SIZE,
        &LIGHT_LC_PR_REGULATOR_ACCURACY_EID, PROPERTY_STRING("REGULATOR_ACCURACY")
        LIGHT_LC_STATE_REGULATOR_ACCURACY,
    },
    {
        LIGHT_LC_SERVER_REGULATOR_KID_PID, LIGHT_LC_SERVER_COEFFICIENT_SIZE,
        &LIGHT_LC_PR_REGULATOR_KID_EID, PROPERTY_STRING("REGULATOR_KID")
        LIGHT_LC_STATE_REGULATOR_KID,
    },
    {
        LIGHT_LC_SERVER_REGULATOR_KIU_PID, LIGHT_LC_SERVER_COEFFICIENT_SIZE,
        &LIGHT_LC_PR_REGULATOR_KIU_EID, PROPERTY_STRING("REGULATOR_KIU")
        LIGHT_LC_STATE_REGULATOR_KIU,
    },
    {
        LIGHT_LC_SERVER_REGULATOR_KPD_PID, LIGHT_LC_SERVER_COEFFICIENT_SIZE,
        &LIGHT_LC_PR_REGULATOR_KPD_EID, PROPERTY_STRING("REGULATOR_KPD")
        LIGHT_LC_STATE_REGULATOR_KPD,
    },
    {
        LIGHT_LC_SERVER_REGULATOR_KPU_PID, LIGHT_LC_SERVER_COEFFICIENT_SIZE,
        &LIGHT_LC_PR_REGULATOR_KPU_EID, PROPERTY_STRING("REGULATOR_KPU")
        LIGHT_LC_STATE_REGULATOR_KPU,
    },
    {
        LIGHT_LC_SERVER_TIME_FADE_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_FADE_EID, PROPERTY_STRING("TIME_FADE")
        LIGHT_LC_STATE_TIME_FADE,
    },
    {
        LIGHT_LC_SERVER_TIME_FADE_ON_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_FADE_ON_EID, PROPERTY_STRING("TIME_FADE_ON")
        LIGHT_LC_STATE_TIME_FADE_ON,
    },
    {
        LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_FADE_STANDBY_AUTO_EID, PROPERTY_STRING("TIME_FADE_STANDBY_AUTO")
        LIGHT_LC_STATE_TIME_FADE_STANDBY_AUTO,
    },
    {
        LIGHT_LC_SERVER_TIME_FADE_STANDBY_MANUAL_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_FADE_STANDBY_MANUAL_EID, PROPERTY_STRING("TIME_FADE_MANUAL")
        LIGHT_LC_STATE_TIME_FADE_STANDBY_MANUAL,
    },
    {
        LIGHT_LC_SERVER_TIME_OCCUPANCY_DELAY_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_OCCUPANCY_DELAY_EID, PROPERTY_STRING("TIME_OCCUPANCY_DELAY")
        LIGHT_LC_STATE_TIME_OCCUPANCY_DELAY,
    },
    {
        LIGHT_LC_SERVER_TIME_PROLONG_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_PROLONG_EID, PROPERTY_STRING("TIME_PROLONG")
        LIGHT_LC_STATE_TIME_PROLONG,
    },
    {
        LIGHT_LC_SERVER_TIME_RUN_ON_PID, LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE,
        &LIGHT_LC_PR_TIME_RUN_ON_EID, PROPERTY_STRING("TIME_RUN_ON")
        LIGHT_LC_STATE_TIME_RUN_ON,
    },
    {
        LIGHT_LC_SERVER_NO_PID, LIGHT_LC_SERVER_UINT8_SIZE,
        &LIGHT_LC_MODE_EID, PROPERTY_STRING("LC_MODE")
        LIGHT_LC_STATE_LIGHT_LC_MODE,
    },
    {
        LIGHT_LC_SERVER_NO_PID, LIGHT_LC_SERVER_UINT8_SIZE,
        &LIGHT_LC_OCC_MODE_EID, PROPERTY_STRING("LC_OCC_MODE")
        LIGHT_LC_STATE_LIGHT_LC_OCC_MODE,
    },
    {
        LIGHT_LC_SERVER_NO_PID, LIGHT_LC_SERVER_UINT8_SIZE,
        &LIGHT_LC_LIGHT_ONOFF_EID, PROPERTY_STRING("LC_LIGHT_ONOFF")
        LIGHT_LC_STATE_LIGHT_LC_LIGHT_ONOFF,
    },
    {
        LIGHT_LC_SERVER_MOTION_SENSED_PID, LIGHT_LC_SERVER_PERCENTAGE_8_SIZE,
        NULL, PROPERTY_STRING("MOTION_SENSED")
        LIGHT_LC_STATE_NULL_ENTRY,
    },
    {
        LIGHT_LC_SERVER_PEOPLE_COUNT_PID, LIGHT_LC_SERVER_COUNT_16_SIZE,
        NULL, PROPERTY_STRING("PEOPLE_COUNT")
        LIGHT_LC_STATE_NULL_ENTRY,
    },
    {
        LIGHT_LC_SERVER_PRESENCE_DETECT_PID, LIGHT_LC_SERVER_BOOLEAN_SIZE,
        NULL, PROPERTY_STRING("PRESENCE_DETECT")
        LIGHT_LC_STATE_NULL_ENTRY,
    },
    {
        LIGHT_LC_SERVER_PRESENT_AMBIENT_LIGHT_LEVEL_PID, LIGHT_LC_SERVER_ILLUMINANCE_24_SIZE,
        NULL, PROPERTY_STRING("PRESENT_AMBIENT_LIGHT")
        LIGHT_LC_STATE_NULL_ENTRY,
    },
};

uint32_t light_lc_state_utils_property_id_from_lc_state(light_lc_state_t lc_state, uint16_t * p_property_id)
{
    if (p_property_id == NULL)
    {
        return NRF_ERROR_NULL;
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (lc_state == m_property_info[i].lc_state)
        {
            *p_property_id = m_property_info[i].property_id;
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "lc_state %d not found\n", lc_state);
    return NRF_ERROR_NOT_FOUND;
}

uint32_t light_lc_state_utils_lc_state_from_property_id(uint16_t property_id, light_lc_state_t * p_lc_state)
{
    if (p_lc_state == NULL)
    {
        return NRF_ERROR_NULL;
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (property_id == m_property_info[i].property_id)
        {
            *p_lc_state = m_property_info[i].lc_state;
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "property_id 0x%X not found\n", property_id);
    return NRF_ERROR_NOT_FOUND;
}

/* Retrieve the size (in bytes) of the property value for this id */
uint32_t light_lc_state_utils_property_data_size_get(uint16_t property_id, uint8_t * p_property_lenght)
{
    if (p_property_lenght == NULL)
    {
        return NRF_ERROR_NULL;
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (property_id == m_property_info[i].property_id)
        {
            *p_property_lenght = m_property_info[i].data_size;
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "property_id 0x%X not found\n", property_id);
    return NRF_ERROR_NOT_FOUND;
}

uint32_t light_lc_state_utils_property_flash_id_get(uint16_t property_id, mesh_config_entry_id_t * p_id)
{
    if (p_id == NULL)
    {
        return NRF_ERROR_NULL;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (property_id == m_property_info[i].property_id &&
            m_property_info[i].flash_var_id != NULL)
        {
            *p_id = *m_property_info[i].flash_var_id;
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "property_id 0x%X not found\n", property_id);
    return NRF_ERROR_NOT_FOUND;
}

uint32_t light_lc_state_utils_flash_id_from_lc_state(light_lc_state_t lc_state, mesh_config_entry_id_t * p_id)
{
    if (p_id == NULL)
    {
        return NRF_ERROR_NULL;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (lc_state == m_property_info[i].lc_state &&
            m_property_info[i].flash_var_id != NULL)
        {
            *p_id = *m_property_info[i].flash_var_id;
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "lc_state %d not found\n", lc_state);
    return NRF_ERROR_NOT_FOUND;
}

char * light_lc_state_utils_property_name_string_get(uint16_t property_id)
{
#if LIGHT_LC_STATE_UTILS_DEBUG
    for (uint32_t i = 0; i < ARRAY_SIZE(m_property_info); i++)
    {
        if (property_id == m_property_info[i].property_id)
        {
            return m_property_info[i].name_string;
        }
    }
#endif
    return NULL;
}

/* End of Property section of lc state utils */

/* When a BLE mesh command comes in and directly controls the light (e.g. Light Lightness, Level,
 * etc), we disable the binding between the LC Light Linear Output and the Light Lightness linear
 * state (in essence turning off the light control - see 6.2.3.1). */


bool light_lc_state_utils_server_control_is_disabled(light_lc_setup_server_t * p_s_server)
{
    uint8_t lc_mode;

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb(p_s_server, LIGHT_LC_STATE_LIGHT_LC_MODE, &lc_mode);
    return lc_mode == LIGHT_LC_MODE_OFF;
}

/* The getter functions */
uint8_t light_lc_state_utils_mode_get(light_lc_setup_server_t * p_s_server)
{
    uint8_t lc_mode;

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb(p_s_server, LIGHT_LC_STATE_LIGHT_LC_MODE, &lc_mode);
    return lc_mode;
}

uint8_t light_lc_state_utils_occ_mode_get(light_lc_setup_server_t * p_s_server)
{
    uint8_t occ_mode;

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb(p_s_server, LIGHT_LC_STATE_LIGHT_LC_OCC_MODE,
                                                               &occ_mode);
    return occ_mode;
}

/* Light OnOff is the one state variable that has present, target, and remaining time associated
 * with it */
light_lc_light_onoff_status_params_t light_lc_state_utils_light_onoff_get(light_lc_setup_server_t * p_s_server)
{
    light_lc_light_onoff_status_params_t ret = {0};
    uint8_t light_onoff;
    uint32_t elapsed_ms = 0;

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb(p_s_server, LIGHT_LC_STATE_LIGHT_LC_LIGHT_ONOFF,
                                                                           &light_onoff);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "current light_onoff: %d\n", light_onoff);

    ret.present_light_onoff = !!light_onoff;
    if (p_s_server->transition_info.requested_delay_ms > 0)
    {
        ret.target_light_onoff = p_s_server->transition_info.requested_light_onoff;
        ret.remaining_time_ms = p_s_server->transition_info.requested_transition_time_ms;

        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "light_onoff_get: return: pr %d  tgt %d  rem_time %d\n",
        ret.present_light_onoff, ret.target_light_onoff, ret.remaining_time_ms);

        return ret;
    }

    ret.remaining_time_ms = p_s_server->transition_info.transition_time_ms;
    if (ret.remaining_time_ms > 0)
    {
        /* We cannot use `light_onoff` value that was fetched above, since FSM will write target
         * value to flash to handle power fail condition. The present_light_onoff indicates
         * the current internal value when transition is going on. */
        ret.present_light_onoff = p_s_server->transition_info.present_light_onoff;
        ret.target_light_onoff = p_s_server->transition_info.target_light_onoff;
        elapsed_ms = MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_s_server->fsm_timer));
        ret.remaining_time_ms -= elapsed_ms;
    }
    else
    {
        /* We're out of sync - that can happen when node powers up. */
        if (ret.present_light_onoff != p_s_server->transition_info.present_light_onoff)
        {
            p_s_server->transition_info.present_light_onoff = (bool)light_onoff;
        }
        /* It is necessary to set target value to present value, even if transition time is zero
         * to let higher layers use it during power up. */
        ret.target_light_onoff = ret.present_light_onoff;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "light_onoff_get: return: pr %d  tgt %d  rem_time %d\n",
    ret.present_light_onoff, ret.target_light_onoff, ret.remaining_time_ms);

    return ret;
}

uint32_t light_lc_state_utils_property_get(light_lc_setup_server_t * p_s_server, uint16_t property_id)
{
    light_lc_state_t lc_state = LIGHT_LC_STATE_NULL_ENTRY;
    uint32_t value = 0;

    NRF_MESH_ASSERT(p_s_server);
    NRF_MESH_ERROR_CHECK(light_lc_state_utils_lc_state_from_property_id(property_id, &lc_state));

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_get_cb(p_s_server, lc_state, &value);
    return value;
}


/* The setter functions */
void light_lc_state_utils_mode_set(light_lc_setup_server_t * p_s_server, uint8_t set_value)
{
    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_set_cb(p_s_server,
                                                                           LIGHT_LC_STATE_LIGHT_LC_MODE,
                                                                           &set_value);
}

void light_lc_state_utils_occ_mode_set(light_lc_setup_server_t * p_s_server, uint8_t set_value)
{
    NRF_MESH_ASSERT(p_s_server);
    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_set_cb(p_s_server,
                                                                           LIGHT_LC_STATE_LIGHT_LC_OCC_MODE,
                                                                           &set_value);
}

void light_lc_state_utils_light_onoff_set(light_lc_setup_server_t * p_s_server, uint8_t set_value)
{
    NRF_MESH_ASSERT(p_s_server);
    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_set_cb(p_s_server,
                                                                           LIGHT_LC_STATE_LIGHT_LC_LIGHT_ONOFF,
                                                                           &set_value);
}

void light_lc_state_utils_property_set(light_lc_setup_server_t * p_s_server,
                                       uint32_t set_value,
                                       uint16_t property_id)
{
    light_lc_state_t lc_state =  LIGHT_LC_STATE_NULL_ENTRY;

    NRF_MESH_ASSERT(p_s_server);
    NRF_MESH_ERROR_CHECK(light_lc_state_utils_lc_state_from_property_id(property_id, &lc_state));

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_persist_set_cb(p_s_server, lc_state, &set_value);
}

void light_lc_state_utils_ambient_luxlevel_set(light_lc_setup_server_t * p_s_server, uint32_t set_value)
{
    NRF_MESH_ASSERT(p_s_server);
    p_s_server->state.ambient_luxlevel_valid = true;
    p_s_server->state.ambient_luxlevel = set_value;
}

void light_lc_state_utils_luxlevel_out_set(light_lc_setup_server_t * p_s_server, uint32_t set_value)
{
    NRF_MESH_ASSERT(p_s_server);
    p_s_server->state.luxlevel_out = set_value;
}

void light_lc_state_utils_lightness_out_set(light_lc_setup_server_t * p_s_server, uint16_t set_value)
{
    NRF_MESH_ASSERT(p_s_server);
    p_s_server->state.lightness_out = set_value;
}

bool light_lc_state_utils_ambient_luxlevel_is_valid(light_lc_setup_server_t * p_s_server)
{
    NRF_MESH_ASSERT(p_s_server);
    return p_s_server->state.ambient_luxlevel_valid;
}

uint32_t light_lc_state_utils_ambient_luxlevel_get(light_lc_setup_server_t * p_s_server)
{
    NRF_MESH_ASSERT(p_s_server);
    return p_s_server->state.ambient_luxlevel;
}

uint32_t light_lc_state_utils_luxlevel_out_get(light_lc_setup_server_t * p_s_server)
{
    NRF_MESH_ASSERT(p_s_server);
    return p_s_server->state.luxlevel_out;
}

uint32_t light_lc_state_utils_lightness_out_get(light_lc_setup_server_t * p_s_server)
{
    NRF_MESH_ASSERT(p_s_server);
    return p_s_server->state.lightness_out;
}
