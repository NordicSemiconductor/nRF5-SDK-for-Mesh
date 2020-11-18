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

#include "light_lc_fsm.h"

#include "sdk_config.h"
#include "fsm_assistant.h"

/* Logging and RTT */
#include "log.h"

/* Define to 1 to see debug for the states and associated lightness values set.
   This can be very useful while tuning a hardware set and an installation */
#ifndef LIGHT_LC_FSM_DEBUG
#define LIGHT_LC_FSM_DEBUG 0
#endif

#include "light_lightness_utils.h"
#include "light_lc_state_utils.h"
#include "light_lc_light_pi.h"
#include "light_lc_server_property_constants.h"
#include "nrf_mesh_config_core.h"

static bool m_fsm_init = false;
/**************************************************************************************************/
/* Behavioral state machine */
/* Various states of Light LC State machine - defined in @tagMeshMdlSp section 6.2.5.1 */

/*lint -e123 Lint fails to understand particular use of the all fsm macros and throws the warning:
> Macro '_DECLARE_ENUM' defined with arguments at line 185 this is just a warning  --
> The name of a macro defined with arguments was subsequently used without a following '('.
> This is legal but may be an oversight.
In this case it is intentional.*/

#define EVENT_LIST E_MODE_ON,                                           \
        E_MODE_OFF,                                                     \
        E_LIGHT_ON,                                                     \
        E_LIGHT_OFF,                                                    \
        E_OCC_ON,                                                       \
        E_TIMER_OFF

#define STATE_LIST  S_OFF,                          \
        S_STANDBY,                                  \
        S_FADE_ON,                                  \
        S_RUN,                                      \
        S_FADE,                                     \
        S_PROLONG,                                  \
        S_FADE_STANDBY_AUTO,                        \
        S_FADE_STANDBY_MANUAL

#define ACTION_LIST A_TIMER_ABORT_STANDBY, a_timer_abort_standby,   \
        A_LIGHT_ON_T1_FADE_ON, a_light_on_t1_fade_on,               \
        A_LIGHT_ON_TT_FADE_ON, a_light_on_tt_fade_on,               \
        A_OCC_ON_T1_FADE_ON, a_occ_on_t1_fade_on,                   \
        A_OCC_ON_T2_RUN, a_occ_on_t2_run,                           \
        A_OCC_ON_TT_FADE_ON, a_occ_on_tt_fade_on,                   \
        A_TIMER_T1_FADE_ON, a_timer_t1_fade_on,                     \
        A_TIMER_T2_RUN, a_timer_t2_run,                             \
        A_TIMER_T3_FADE, a_timer_t3_fade,                           \
        A_TIMER_T4_PROLONG, a_timer_t4_prolong,                     \
        A_TIMER_TT_FADE_ON, a_timer_tt_fade_on,                     \
        A_TRANSITION_TICK, a_transition_tick,                       \
        A_LIGHT_OFF_ABORT_OFF, a_light_off_abort_off,               \
        A_LIGHT_OFF_ABORT_STANDBY, a_light_off_abort_standby,       \
        A_LIGHT_OFF_TT_ST_MAN, a_light_off_tt_st_man,               \
        A_LIGHT_OFF_T5_ST_AUTO, a_light_off_t5_st_auto

#define GUARD_LIST G_AUTO_OCCUPANCY_SET,  g_auto_occupancy_set, \
        G_TRANSITION_COMPLETE, g_transition_complete

typedef enum
{
    DECLARE_ENUM(EVENT_LIST)
} lc_fsm_event_ids_t;

typedef enum
{
    DECLARE_ENUM(STATE_LIST)
} lc_fsm_state_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(ACTION_LIST)
} lc_fsm_action_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(GUARD_LIST)
} lc_fsm_guard_ids_t;

typedef void (* lc_fsm_action_t)(void *);
typedef bool (* lc_fsm_guard_t)(void *);

/* Not having a semi-colon is intentional. */
DECLARE_ACTION_PROTOTYPE(ACTION_LIST)
DECLARE_GUARD_PROTOTYPE(GUARD_LIST)

static void lc_fsm_action(fsm_action_id_t action_id, void * p_data);
static bool lc_fsm_guard(fsm_action_id_t action_id, void * p_data);

static const fsm_transition_t m_lc_fsm_transition_table[] =
{
    FSM_STATE     (S_OFF),
    FSM_TRANSITION(E_MODE_ON,   FSM_ALWAYS,            A_TIMER_ABORT_STANDBY,     S_STANDBY),
    FSM_TRANSITION(E_MODE_OFF,  FSM_ALWAYS,            FSM_NO_ACTION,             FSM_SAME_STATE),

    FSM_STATE     (S_STANDBY),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_LIGHT_ON_TT_FADE_ON,     S_FADE_ON),
    FSM_TRANSITION(E_OCC_ON,    G_AUTO_OCCUPANCY_SET,  A_LIGHT_ON_T1_FADE_ON,     S_FADE_ON),

    FSM_STATE     (S_FADE_ON),
    FSM_TRANSITION(E_LIGHT_OFF, FSM_ALWAYS,            A_LIGHT_OFF_TT_ST_MAN,     S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_TIMER_T2_RUN,            S_RUN),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (S_RUN),
    FSM_TRANSITION(E_LIGHT_OFF, FSM_ALWAYS,            A_LIGHT_OFF_TT_ST_MAN,     S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_OCC_ON,    FSM_ALWAYS,            A_OCC_ON_T2_RUN,           FSM_SAME_STATE),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_TIMER_T2_RUN,            FSM_SAME_STATE),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_TIMER_T3_FADE,           S_FADE),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (S_FADE),
    FSM_TRANSITION(E_LIGHT_OFF, FSM_ALWAYS,            A_LIGHT_OFF_TT_ST_MAN,     S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_OCC_ON,    FSM_ALWAYS,            A_OCC_ON_TT_FADE_ON,       S_FADE_ON),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_TIMER_T1_FADE_ON,        S_FADE_ON),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_TIMER_T4_PROLONG,        S_PROLONG),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (S_PROLONG),
    FSM_TRANSITION(E_LIGHT_OFF, FSM_ALWAYS,            A_LIGHT_OFF_TT_ST_MAN,     S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_OCC_ON,    FSM_ALWAYS,            A_OCC_ON_T1_FADE_ON,       S_FADE_ON),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_TIMER_TT_FADE_ON,        S_FADE_ON),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_LIGHT_OFF_T5_ST_AUTO,    S_FADE_STANDBY_AUTO),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (S_FADE_STANDBY_AUTO),
    FSM_TRANSITION(E_LIGHT_OFF, FSM_ALWAYS,            A_LIGHT_OFF_TT_ST_MAN,     S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_OCC_ON,    FSM_ALWAYS,            A_OCC_ON_T1_FADE_ON,       S_FADE_ON),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_TIMER_TT_FADE_ON,        S_FADE_ON),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_TIMER_ABORT_STANDBY,     S_STANDBY),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (S_FADE_STANDBY_MANUAL),
    FSM_TRANSITION(E_LIGHT_ON,  FSM_ALWAYS,            A_TIMER_TT_FADE_ON,        S_FADE_ON),
    FSM_TRANSITION(E_TIMER_OFF, G_TRANSITION_COMPLETE, A_LIGHT_OFF_ABORT_STANDBY, S_STANDBY),
    FSM_TRANSITION(E_TIMER_OFF, FSM_OTHERWISE,         A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE     (FSM_ANY_STATE),
    FSM_TRANSITION(E_MODE_OFF,  FSM_ALWAYS,            A_LIGHT_OFF_ABORT_OFF,     S_OFF)
};

#if FSM_DEBUG
static const char * m_action_lookup_table[] =
{
    DECLARE_STRING_PAIR(ACTION_LIST)
};

static const char * m_guard_lookup_table[] =
{
    DECLARE_STRING_PAIR(GUARD_LIST)
};

static const char * m_event_lookup_table[] =
{
    DECLARE_STRING(EVENT_LIST)
};

static const char * m_state_lookup_table[] =
{
    DECLARE_STRING(STATE_LIST)
};
#endif  /* FSM_DEBUG */

static fsm_const_descriptor_t m_lc_fsm_descriptor =
{
    .transition_table = m_lc_fsm_transition_table,
    .transitions_count = ARRAY_SIZE(m_lc_fsm_transition_table),
    .initial_state = S_OFF,
    .guard = lc_fsm_guard,
    .action = lc_fsm_action,
#if FSM_DEBUG
    .fsm_name = "LC-SRV-fsm",
    .action_lookup = m_action_lookup_table,
    .event_lookup = m_event_lookup_table,
    .guard_lookup = m_guard_lookup_table,
    .state_lookup = m_state_lookup_table
#endif  /* FSM_DEBUG */
};

static const lc_fsm_action_t lc_fsm_actions[] =
{
    DECLARE_HANDLER(ACTION_LIST)
};

static void lc_fsm_action(fsm_action_id_t action_id, void * p_data)
{
    lc_fsm_actions[action_id](p_data);
}

static const lc_fsm_guard_t lc_fsm_guards[] =
{
    DECLARE_HANDLER(GUARD_LIST)
};

static bool lc_fsm_guard(fsm_guard_id_t guard_id, void * p_data)
{
    return lc_fsm_guards[guard_id](p_data);
}
/*lint +e123 */
/**************************************************************************************************/
/***** State machine functions *****/

static bool g_auto_occupancy_set(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t on_luxlevel;
    uint32_t current_luxlevel;

    /* Definition of Auto Occupancy @tagMeshMdlSp section 6.2.5.3 */
    if (light_lc_state_utils_occ_mode_get(p_s_server) == 0)
    {
        return false;
    }

    on_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    current_luxlevel = light_lc_state_utils_luxlevel_out_get(p_s_server);

    if (current_luxlevel >= on_luxlevel)
    {
        return false;
    }
    return true;
}

static bool g_transition_complete(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t transition_time_ms;

    transition_time_ms = p_s_server->transition_info.transition_time_ms;
    return (MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_s_server->fsm_timer)) >=
            transition_time_ms);
}

static void util_set_state_machine_outputs(light_lc_setup_server_t * p_s_server,
                                           uint16_t lightness_property_id,
                                           uint16_t luxlevel_property_id)
{
    uint16_t lightness_actual = 0;
    uint32_t luxlevel = 0;

    /* Get the lightness out and luxlevel out properties */
    if (lightness_property_id != 0)
    {
        lightness_actual = light_lc_state_utils_property_get(p_s_server, lightness_property_id);
    }
    if (luxlevel_property_id != 0)
    {
        luxlevel = light_lc_state_utils_property_get(p_s_server, luxlevel_property_id);
    }

    /* Set those fsm output states */
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "setting lightness out light_actual=0x%X, lux=0x%X\n",
          lightness_actual, luxlevel);
#endif
    light_lc_state_utils_lightness_out_set(p_s_server, light_lightness_utils_actual_to_linear(lightness_actual));
    light_lc_state_utils_luxlevel_out_set(p_s_server, luxlevel);
}

static void light_onoff_status_publish(light_lc_setup_server_t * p_s_server)
{
    light_lc_light_onoff_status_params_t params;
    uint32_t elapsed_ms;

    params.present_light_onoff = p_s_server->transition_info.present_light_onoff;
    params.target_light_onoff = p_s_server->transition_info.target_light_onoff;
    if (p_s_server->transition_info.requested_transition_time_ms == 0)
    {
        params.remaining_time_ms = 0;
    }
    else
    {
        elapsed_ms =
            MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_s_server->fsm_timer));
        params.remaining_time_ms = p_s_server->transition_info.requested_transition_time_ms - elapsed_ms;
    }
    (void) light_lc_server_light_onoff_status_publish(&p_s_server->lc_srv, &params);
}

static void util_abort_timer(light_lc_setup_server_t * p_s_server)
{
    model_timer_abort(&p_s_server->fsm_timer);
    p_s_server->transition_info.transition_time_ms = 0;
    p_s_server->transition_info.requested_transition_time_ms = 0;
    p_s_server->transition_info.requested_delay_ms = 0;
    p_s_server->transition_info.transition_time_is_provided = false;
    p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
    /* This may have been a change to light onoff - so send a publish status */
    light_onoff_status_publish(p_s_server);
}

static void util_start_oneshot_timer(light_lc_setup_server_t * p_s_server, uint32_t timer_ms)
{
    uint64_t timer_us;

    timer_us = MS_TO_US((uint64_t) timer_ms);


    if (timer_us < MS_TO_US(MODEL_TIMER_PERIOD_MS_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS)))
    {
        /* We cannot fire timer more frequently than MODEL_TIMER_TIMEOUT_MIN_TICKS, thus round up to
           a minimum timer size. */
        p_s_server->fsm_timer.timeout_rtc_ticks = MODEL_TIMER_TIMEOUT_MIN_TICKS;
    }
    else
    {
        /* Start a timer using time steps corresponding to requested value as 1 timer. */
        p_s_server->fsm_timer.timeout_rtc_ticks = MODEL_TIMER_TICKS_GET_US(timer_us);
    }

    p_s_server->fsm_timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s: starting timer\n", __func__);
#endif

    (void) model_timer_schedule(&p_s_server->fsm_timer);
}

static void util_start_lightness_transition_timer(light_lc_setup_server_t * p_s_server,
                                                  uint32_t transition_time_ms)
{
    uint16_t current_lightness;
    uint16_t target_lightness;
    uint16_t abs_lightness_delta;
    uint64_t transition_time_us;
    uint64_t transition_step_us;
    uint16_t actual_lightness;

    target_lightness = p_s_server->transition_info.target_lightness;

    p_s_server->settings.p_callbacks->light_lc_cbs.light_lc_actual_get_cb(p_s_server, &actual_lightness);

    current_lightness = light_lightness_utils_actual_to_linear(actual_lightness);

#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "starting transition linear: 0x%04X->0x%04X\n",
          current_lightness, target_lightness);
#endif

    abs_lightness_delta = abs(target_lightness - current_lightness);
    if (abs_lightness_delta == 0)
    {
        /* Avoid divide by zero error - shouldn't ever get here anyway */
        abs_lightness_delta = 1;
    }
    p_s_server->transition_info.transition_time_ms = transition_time_ms;
    transition_time_us = MS_TO_US((uint64_t) transition_time_ms);
    transition_step_us = transition_time_us/abs_lightness_delta;

    /* Make sure not to go shorter than the customer-defined MIN.  It's possible that if the time is
       too short, the lighting hardware may not be able to support it. */
    if (transition_step_us < MS_TO_US((uint64_t) TRANSITION_STEP_MIN_MS))
    {
        transition_step_us = MS_TO_US((uint64_t) TRANSITION_STEP_MIN_MS);
    }

    if (transition_step_us < MS_TO_US(MODEL_TIMER_PERIOD_MS_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS)))
    {
        /* We cannot fire timer more frequently than MODEL_TIMER_TIMEOUT_MIN_TICKS, thus increment
           present level in suitable steps. */
        p_s_server->fsm_timer.timeout_rtc_ticks = MODEL_TIMER_TIMEOUT_MIN_TICKS;
    }
    else
    {
        /* Perform level transition using time steps corresponding to one step change. */
        p_s_server->fsm_timer.timeout_rtc_ticks = MODEL_TIMER_TICKS_GET_US(transition_step_us);
    }

#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "setting initial_present_lightness to 0x%04X\n", current_lightness);
#endif
    p_s_server->transition_info.initial_present_lightness = current_lightness;
    p_s_server->transition_info.initial_present_luxlevel = light_lc_state_utils_luxlevel_out_get(p_s_server);

    p_s_server->fsm_timer.mode = MODEL_TIMER_MODE_REPEATED;
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "starting timer: present=%d, target=%d\n",
          p_s_server->transition_info.present_light_onoff,
          p_s_server->transition_info.target_light_onoff);
#endif

//#define FIXME_MSH_SPEC_6_2_5_4
#ifdef FIXME_MSH_SPEC_6_2_5_4
        /* according to 6.2.5.4, " If the Next State is Fade On, Fade, Fade Standby Auto, Fade Standby
         * Manual, the value provided is the target value of the Light LC OnOff state."  This means that
         * we should not change present to target until the timer is done (which is done in the state
         * machine). If 6.2.5.4 is correct, then delete this ifdef FIXME... code.  If the decision is
         * made that present should follow the binary rules 3.1.1.1, then and keep this ifdef
         * code. */
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "present=%d, target=%d\n",
            p_s_server->transition_info.present_light_onoff,
            p_s_server->transition_info.target_light_onoff);
#endif
    if (p_s_server->transition_info.present_light_onoff != p_s_server->transition_info.target_light_onoff)
    {
        p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
    }
#endif

    /* The transitions implemented here for light_onoff are according to Binary transition rules.
     * However this will be moved elsewhere, once FSM is restructured. */
    if ((p_s_server->transition_info.present_light_onoff == 0) &&
        (p_s_server->transition_info.target_light_onoff == 1))
    {
        p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
    }

    (void) model_timer_schedule(&p_s_server->fsm_timer);
}

static void util_start_timer_lightness(light_lc_setup_server_t * p_s_server,
                                       uint32_t transition_time_ms,
                                       uint32_t target_luxlevel,
                                       uint16_t target_perceptive_lightness)
{
    /* Store away the lux level and the linear target lightness -
     * util_start_lightness_transition_timer need it */
    p_s_server->transition_info.target_luxlevel = target_luxlevel;

    p_s_server->transition_info.target_lightness =
        light_lightness_utils_actual_to_linear(target_perceptive_lightness);
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "setting target to actual 0x%04X linear 0x%04X, trans=%d\n",
          target_perceptive_lightness, p_s_server->transition_info.target_lightness,
          transition_time_ms);
#endif

    /* No delay timer, it is already taken care of, so just start the transition timer */
    util_start_lightness_transition_timer(p_s_server, transition_time_ms);
}


/* Note, Timer T* values: T1 - Time_Fade_On, T2 - Time_Run_On, T3 - Time_Fade, T4 - Time_Prolong
 * T5 - Time_Fade_Standby_Auto, T6 - Time_Fade_Standby_Manual
 */

/* Mesh spec action def'n "Abort Timer". Next State:Standby */
static void a_timer_abort_standby(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;

    /* stop any pending timer */
    util_abort_timer(p_s_server);

    /* If we got here from a Fade * state, set Light OnOff from target value.  @tagMeshMdlSp section 6.2.5.4
     * */
    if (p_s_server->transition_info.target_light_onoff != p_s_server->transition_info.present_light_onoff)
    {
        p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
        p_s_server->transition_info.transition_time_ms = 0;
        /* This may have been a change to light onoff - so send a publish status */
        light_onoff_status_publish(p_s_server);

        /* Don't set the flash - the Fade * state did that already */
    }
    /* Set the State machine outputs */
    util_set_state_machine_outputs(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID,
                                   LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID);
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b0", "Abort Timer".  Next State: Off */
static void a_light_off_abort_off(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;

    /* stop any pending timer */
    util_abort_timer(p_s_server);

    /* Set Light LC Light OnOff to 0b0 */
    p_s_server->transition_info.present_light_onoff = 0;
    p_s_server->transition_info.target_light_onoff = 0;
    p_s_server->transition_info.transition_time_ms = 0;
    light_lc_state_utils_light_onoff_set(p_s_server, false);

    /* This may have been a change to light onoff - so send a publish status */
    light_onoff_status_publish(p_s_server);

    /* Set the State machine outputs */
    util_set_state_machine_outputs(p_s_server, 0, 0);
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b1", "Set Timer to Transition Time".  Next
 * State:Fade On */
static void a_light_on_tt_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* Can't set present_light_onoff until the transition is completed */
    p_s_server->transition_info.target_light_onoff = 1;
    /* set the flash value in case of a power failure */
    light_lc_state_utils_light_onoff_set(p_s_server, true);

    /* Set timer to TT or Fade On time and change target lightness to run's
     * lightness, and target luxlevel to run's luxlevel */
    if (p_s_server->transition_info.transition_time_is_provided)
    {
        time_ms = p_s_server->transition_info.requested_transition_time_ms;
    }
    else
    {
        time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);
    }

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b1", "Set Timer to T1".  Next State:Fade
 * On */
static void a_light_on_t1_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* Can't set present_light_onoff until the transition is completed */
    p_s_server->transition_info.target_light_onoff = 1;
    /* set the flash value in case of a power failure */
    light_lc_state_utils_light_onoff_set(p_s_server, true);

    /* Set timer to T1 (Fade On) time and change target lightness to run's
     * lightness and target luxlevel to run's luxlevel */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);
    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "target_lightness actual %d\n", target_lightness);
#endif
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b0", "Set Timer to Transition Time".  Next
 * State:Fade Standby Manual */
static void a_light_off_tt_st_man(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* Can't set present_light_onoff until the transition is completed */
    p_s_server->transition_info.target_light_onoff = 0;
    /* set the flash value in case of a power failure */
    light_lc_state_utils_light_onoff_set(p_s_server, false);

    /* Set timer to TT or Fade Standby Manual time and change target lightness to Standby's
     * lightness, and target luxlevel to Standby's luxlevel */
    if (p_s_server->transition_info.transition_time_is_provided)
    {
        time_ms = p_s_server->transition_info.requested_transition_time_ms;
    }
    else
    {
        time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_STANDBY_MANUAL_PID);
    }

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);

    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Timer to T1".  Next State: Fade On */
static void a_timer_t1_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;
    uint16_t target_lightness;
    uint32_t target_luxlevel;

    /* Set timer to T1 (Fade On) time and change target lightness to run's lightness and target
     * luxlevel to run's luxlevel */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);
    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Timer to T4".  Next State: Prolong */
static void a_timer_t4_prolong(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;

    /* stop the timer that got us here */
    util_abort_timer(p_s_server);

    /* If we got here from a Fade * state, set Light OnOff from target value.  @tagMeshMdlSp section 6.2.5.4
     * */
    if (p_s_server->transition_info.target_light_onoff != p_s_server->transition_info.present_light_onoff)
    {
        p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
        p_s_server->transition_info.transition_time_ms = 0;

        /* This may have been a change to light onoff - so send a publish status */
        light_onoff_status_publish(p_s_server);

        /* Don't set the flash - the Fade * state did that already */
    }

    /* Start a timer with T4 (prolong) time. */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_PROLONG_PID);

    util_start_oneshot_timer(p_s_server, time_ms);

    /* Now that we have arrived at the time for the next state (Prolong), just set the final prolong
     * lightness and luxlevel (just to be exact about it and avoid any potential round off error) */
    util_set_state_machine_outputs(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_PROLONG_PID,
                                   LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_PROLONG_PID);
}

/* Mesh spec action def'n "Set Timer to T2".  Next State: Run */
static void a_timer_t2_run(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;

    /* stop the timer that got us here */
    util_abort_timer(p_s_server);

    /* If we got here from a Fade * state, set Light OnOff from target value.  @tagMeshMdlSp section 6.2.5.4
     * */
    if (p_s_server->transition_info.target_light_onoff != p_s_server->transition_info.present_light_onoff)
    {
        p_s_server->transition_info.present_light_onoff = p_s_server->transition_info.target_light_onoff;
        p_s_server->transition_info.transition_time_ms = 0;

        /* This may have been a change to light onoff - so send a publish status */
        light_onoff_status_publish(p_s_server);

        /* Don't set the flash - the Fade * state did that already */
    }

    /* Start a timer with T2 (run) time. */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_RUN_ON_PID);

    util_start_oneshot_timer(p_s_server, time_ms);

    /* Now that we have arrived at the time for the next state (Run), just set the final run
     * lightness and luxlevel (just to be exact about it and avoid any potential round off error) */
    util_set_state_machine_outputs(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID,
                                   LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);
}

static void a_transition_tick(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t elapsed_ms;
    int32_t delta;
    uint16_t present_lightness;
    uint32_t present_luxlevel;
    uint32_t init_luxlevel;
    uint16_t init_lightness;
    int32_t transition_time_ms;

    elapsed_ms = MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_s_server->fsm_timer));
    transition_time_ms = (int32_t) p_s_server->transition_info.transition_time_ms;

    init_lightness = p_s_server->transition_info.initial_present_lightness;
    delta = p_s_server->transition_info.target_lightness - init_lightness;
    present_lightness = init_lightness + (delta * (int64_t)elapsed_ms / transition_time_ms);

    init_luxlevel = p_s_server->transition_info.initial_present_luxlevel;
    delta = p_s_server->transition_info.target_luxlevel - init_luxlevel;
    present_luxlevel = init_luxlevel + ((delta * (int64_t) elapsed_ms) / transition_time_ms);

    /* Set the State machine outputs */
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "lightness_out: 0x%04X (linear), remaining t: %d\n",
          present_lightness, transition_time_ms - elapsed_ms);
#endif
    light_lc_state_utils_lightness_out_set(p_s_server, present_lightness);
    light_lc_state_utils_luxlevel_out_set(p_s_server, present_luxlevel);
}

/* Mesh spec action def'n "Set Light LC Occupancy to 0b0", "Set Timer to T2".  Next State: Run */
static void a_occ_on_t2_run(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;

    /* Nothing to do about Set Light LC Occupancy - the "setting to 1" was generating the Occupancy
     * On event */

    /* Start a timer with T2 (run) time. */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_RUN_ON_PID);

    util_start_oneshot_timer(p_s_server, time_ms);

    /* We're already in the Run state, and staying there, so no need to set any state machine
     * outputs - steady state */
}

/* Mesh spec action def'n "Set Light LC Occupancy to 0b0", "Set Timer to Transition Time".  Next
 * State: Fade On */
static void a_occ_on_tt_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;
    uint16_t target_lightness;
    uint32_t target_luxlevel;

    /* Nothing to do about Set Light LC Occupancy - the "setting to 1" was generating the Occupancy
     * On event */

    /* Set timer to TT or Fade On time and change target lightness to run's lightness, and target
     * luxlevel to run's luxlevel */
    if (p_s_server->transition_info.transition_time_is_provided)
    {
        time_ms = p_s_server->transition_info.requested_transition_time_ms;
    }
    else
    {
        time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);
    }

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Timer to T3".  Next State: Fade */
static void a_timer_t3_fade(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* stop the timer that got us here */
    util_abort_timer(p_s_server);

    /* Set timer to T3 (fade) time and change target lightness to Prolong's lightness and target
     * luxlevel to prolong's luxlevel */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_PID);

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_PROLONG_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_PROLONG_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Light LC Occupancy to 0b0", "Set Timer to T1".  Next State: Fade
 * On */
static void a_occ_on_t1_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint32_t time_ms;
    uint16_t target_lightness;
    uint32_t target_luxlevel;

    /* Nothing to do about Set Light LC Occupancy - the "setting to 1" * was generating the
    Occupancy On event */

    /* Start a timer with T1 (Fade On) time and change target * lightness to Run's lightness and
    target luxlevel */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Timer to Transition Time", Next State:Fade On */
static void a_timer_tt_fade_on(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* Set timer to TT or Fade On time and change target lightness to run's lightness, and target
     * luxlevel to run's luxlevel */
    if (p_s_server->transition_info.transition_time_is_provided)
    {
        time_ms = p_s_server->transition_info.requested_transition_time_ms;
    }
    else
    {
        time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_ON_PID);
    }

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_ON_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);
    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b0", "Set Timer to T5", Next State:Fade
 * Standby Auto */
static void a_light_off_t5_st_auto(void * p_data)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;
    uint16_t target_lightness;
    uint32_t target_luxlevel;
    uint32_t time_ms;

    /* stop the timer that got us here */
    util_abort_timer(p_s_server);

    /* Can't set present_light_onoff until the transition is completed */
    p_s_server->transition_info.target_light_onoff = 0;
    /* set the flash value in case of a power failure */
    light_lc_state_utils_light_onoff_set(p_s_server, false);

    /* Set timer T5 (Fade Standby Auto) time and change target lightness to Standby's lightness, and
     * target luxlevel to Standby's luxlevel */
    time_ms = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID);

    target_lightness = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID);
    target_luxlevel = light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID);

    util_start_timer_lightness(p_s_server, time_ms, target_luxlevel, target_lightness);

    /* We don't change the State machine outputs; the transition tick does that a step at a time. */
}

/* Mesh spec action def'n "Set Light LC Light OnOff to 0b0", "Abort Timer", Next State:Standby */
static void a_light_off_abort_standby(void * p_data)
{
     light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_data;

    /* Set Light LC Light OnOff to 0b0 */
    p_s_server->transition_info.present_light_onoff = 0;
    p_s_server->transition_info.target_light_onoff = 0;
    p_s_server->transition_info.transition_time_ms = 0;
    light_lc_state_utils_light_onoff_set(p_s_server, false);

    /* This may have been a change to light onoff - so send a publish status */
    light_onoff_status_publish(p_s_server);

    /* stop any pending timer */
    util_abort_timer(p_s_server);

    /* Set the State machine outputs */
    util_set_state_machine_outputs(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID,
                                   LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID);

}

static void timer_fired_cb(void * p_context)
{
    if (m_fsm_init == false)
    {
        /* You can't generate an event before the fsm has been initialized */
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "event can't be generated\n");
        return;
    }

    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_context;

    fsm_event_post(&p_s_server->fsm, E_TIMER_OFF, p_s_server);
}

/* The light PI regulator is supposed to run periodically (@tagMeshMdlSp section 6.2.6). */
static void light_pi_cb(void * p_context)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_context;

    light_lc_light_pi_update(p_s_server);
}

static void lightonoff_event_generate(light_lc_setup_server_t * p_s_server, bool light_onoff)
{
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "generate event: light %s\n", light_onoff ? "ON" : "OFF");
#endif
    fsm_event_post(&p_s_server->fsm, light_onoff ? E_LIGHT_ON : E_LIGHT_OFF, p_s_server);
}

static void onoff_event_cb(timestamp_t timestamp, void * p_context)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *)p_context;
#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "delay complete\n");
#endif
    p_s_server->transition_info.requested_delay_ms = 0;
    lightonoff_event_generate(p_s_server, p_s_server->transition_info.requested_light_onoff);
}

uint32_t light_lc_fsm_occupancy_event_generate(light_lc_setup_server_t * p_s_server)
{
    if (m_fsm_init == false)
    {
        /* You can't generate an event before the fsm has been initialized */
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "event can't be generated\n");
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_s_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

#if LIGHT_LC_FSM_DEBUG
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "generate event: occ_on\n");
#endif
    fsm_event_post(&p_s_server->fsm, E_OCC_ON, p_s_server);

    return NRF_SUCCESS;
}

uint32_t light_lc_fsm_mode_on_off_event_generate(light_lc_setup_server_t * p_s_server, bool mode_onoff)
{
    uint32_t status;
    light_lc_mode_status_params_t lc_mode;

    if (m_fsm_init == false)
    {
        /* You can't generate an event before the fsm has been initialized */
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "event can't be generated\n");
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_s_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* A mode on/off event always sets the state variable */
    light_lc_state_utils_mode_set(p_s_server, mode_onoff);
    /* Publish the state change */
    lc_mode.mode = mode_onoff;
    status = light_lc_server_mode_status_publish(&p_s_server->lc_srv, &lc_mode);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "mode status publish failed %d\n", status);
    }

    if (!mode_onoff)
    {
#if LIGHT_LC_FSM_DEBUG
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "generate mode_off\n");
#endif
        fsm_event_post(&p_s_server->fsm, E_MODE_OFF, p_s_server);
    }
    else
    {
#if LIGHT_LC_FSM_DEBUG
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "generate mode_on\n");
#endif
        fsm_event_post(&p_s_server->fsm, E_MODE_ON, p_s_server);
    }
    return NRF_SUCCESS;
}

uint32_t light_lc_fsm_light_on_off_event_generate(light_lc_setup_server_t * p_s_server, bool light_onoff,
                                                  model_transition_t * p_transition)
{
    if (m_fsm_init == false)
    {
        /* You can't generate an event before the fsm has been initialized */
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "event can't be generated\n");
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_s_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    timer_sch_abort(&p_s_server->onoff_timer);
    p_s_server->transition_info.requested_light_onoff = light_onoff;
    if (p_transition != NULL)
    {
        p_s_server->transition_info.transition_time_is_provided = true;
        p_s_server->transition_info.requested_transition_time_ms = p_transition->transition_time_ms;
        p_s_server->transition_info.requested_delay_ms = p_transition->delay_ms;
        if (p_transition->delay_ms != 0)
        {
            p_s_server->onoff_timer.timestamp = timer_now() + MS_TO_US(p_transition->delay_ms);
            timer_sch_schedule(&p_s_server->onoff_timer);

            return NRF_SUCCESS;
        }
    }
    else
    {
        p_s_server->transition_info.transition_time_is_provided = false;
        p_s_server->transition_info.requested_transition_time_ms = 0;
        p_s_server->transition_info.requested_delay_ms = 0;
    }

    lightonoff_event_generate(p_s_server, light_onoff);
    return NRF_SUCCESS;
}

uint32_t light_lc_fsm_init(light_lc_setup_server_t * p_s_server, bool initial_fsm_onoff)
{
    uint32_t status;

    if (m_fsm_init)
    {
        /* All ready initialized, no need to to do it once more. */
        return NRF_SUCCESS;
    }

    if (p_s_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Create timer used by the state machine */
    p_s_server->fsm_timer.p_context = p_s_server;
    p_s_server->fsm_timer.cb = timer_fired_cb;
    status = model_timer_create(&p_s_server->fsm_timer);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "model_timer_create fsm_timer %d\n", status);
        return status;
    }

    /* Create and start timer used by the light pi */
    p_s_server->light_pi_timer.p_context = p_s_server;
    p_s_server->light_pi_timer.cb = light_pi_cb;
    status = model_timer_create(&p_s_server->light_pi_timer);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "model_timer_create light_pi_timer %d\n", status);
        return status;
    }

    p_s_server->light_pi_timer.timeout_rtc_ticks =
        MODEL_TIMER_TICKS_GET_US(MS_TO_US(LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MS));

    p_s_server->light_pi_timer.mode = MODEL_TIMER_MODE_REPEATED;
    status = model_timer_schedule(&p_s_server->light_pi_timer);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "model_timer_schedule %d\n", status);
        return status;
    }

    p_s_server->transition_info.present_light_onoff = 0;
    p_s_server->transition_info.target_light_onoff = 0;
    p_s_server->transition_info.transition_time_ms = 0;
    p_s_server->transition_info.requested_transition_time_ms = 0;
    p_s_server->transition_info.requested_delay_ms = 0;
    p_s_server->transition_info.transition_time_is_provided = false;
    p_s_server->transition_info.requested_light_onoff = 0;

    p_s_server->onoff_timer.cb = onoff_event_cb;
    p_s_server->onoff_timer.p_context = p_s_server;

    if (initial_fsm_onoff)
    {
#if LIGHT_LC_FSM_DEBUG
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LC FSM=ON\n");
#endif
        m_lc_fsm_descriptor.initial_state = S_STANDBY;
        /* Set the State machine outputs */
        util_set_state_machine_outputs(p_s_server, LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID,
                                       LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID);
    }
    else
    {
#if LIGHT_LC_FSM_DEBUG
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LC FSM=OFF\n");
#endif
        m_lc_fsm_descriptor.initial_state = S_OFF;
        util_set_state_machine_outputs(p_s_server, 0, 0);
    }
    fsm_init(&p_s_server->fsm, &m_lc_fsm_descriptor);
    m_fsm_init = true;

    return NRF_SUCCESS;
}
