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

#include "app_transition.h"

#include <stdlib.h>

#include "sdk_config.h"
#include "example_common.h"
#include "fsm_assistant.h"
#include "nrf_mesh_assert.h"
#include "log.h"

/**************************************************************************************************/
/* Transition state machine */

/*lint -e123 Lint fails to understand particular use of the all fsm macros and throws the warning:
> Macro '_DECLARE_ENUM' defined with arguments at line 185 this is just a warning  --
> The name of a macro defined with arguments was subsequently used without a following '('.
> This is legal but may be an oversight.
In this case it is intentional.*/

#define EVENT_LIST E_START,                     \
                   E_TIMEOUT,                   \
                   E_DELAY_EXPIRED,             \
                   E_ABORT

#define STATE_LIST  S_IDLE,                     \
                    S_IN_PROGRESS

#define ACTION_LIST A_DELAY_START,              a_delay_start,         \
                    A_TRANSITION_START,         a_transition_start,    \
                    A_TRANSITION_COMPLETE,      a_transition_complete, \
                    A_TRANSITION_COMPLETE_WPS,  a_transition_complete_with_param_swap, \
                    A_TRANSITION_TICK,          a_transition_tick,     \
                    A_ABORT,                    a_abort

#define GUARD_LIST  G_SET_DELAY,            g_set_delay,           \
                    G_SET_TRANSITION,       g_set_transition,      \
                    G_TRANSITION_COMPLETE,  g_transition_complete

typedef enum
{
    DECLARE_ENUM(EVENT_LIST)
} app_transition_event_ids_t;

typedef enum
{
    DECLARE_ENUM(STATE_LIST)
} app_transition_state_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(ACTION_LIST)
} app_transition_action_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(GUARD_LIST)
} app_transition_guard_ids_t;

typedef void (* app_transition_fsm_action_t)(void *);
typedef bool (* app_transition_fsm_guard_t)(void *);

/* Not having a semi-colon is intentional. */
DECLARE_ACTION_PROTOTYPE(ACTION_LIST)
DECLARE_GUARD_PROTOTYPE(GUARD_LIST)

static void app_transition_fsm_action(fsm_action_id_t action_id, void * p_data);
static bool app_transition_fsm_guard(fsm_action_id_t action_id, void * p_data);

static const fsm_transition_t m_app_transition_fsm_transition_table[] =
{
    FSM_STATE(S_IDLE),

    FSM_STATE(S_IN_PROGRESS),
    FSM_TRANSITION(E_DELAY_EXPIRED,  G_SET_TRANSITION,       A_TRANSITION_START,        FSM_SAME_STATE),
    FSM_TRANSITION(E_DELAY_EXPIRED,  FSM_OTHERWISE,          A_TRANSITION_COMPLETE_WPS, S_IDLE),
    FSM_TRANSITION(E_TIMEOUT,        G_TRANSITION_COMPLETE,  A_TRANSITION_COMPLETE,     S_IDLE),
    FSM_TRANSITION(E_TIMEOUT,        FSM_OTHERWISE,          A_TRANSITION_TICK,         FSM_SAME_STATE),

    FSM_STATE(FSM_ANY_STATE),
    FSM_TRANSITION(E_START,          G_SET_DELAY,            A_DELAY_START,             S_IN_PROGRESS),
    FSM_TRANSITION(E_START,          G_SET_TRANSITION,       A_TRANSITION_START,        S_IN_PROGRESS),
    FSM_TRANSITION(E_START,          FSM_OTHERWISE,          A_TRANSITION_COMPLETE_WPS, S_IDLE),
    FSM_TRANSITION(E_ABORT,          FSM_ALWAYS,             A_ABORT,                   S_IDLE)
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

static const fsm_const_descriptor_t m_fsm_descriptor =
{
    .transition_table = m_app_transition_fsm_transition_table,
    .transitions_count = ARRAY_SIZE(m_app_transition_fsm_transition_table),
    .initial_state = S_IDLE,
    .guard = app_transition_fsm_guard,
    .action = app_transition_fsm_action,
#if FSM_DEBUG
    .fsm_name = "app_trans",
    .action_lookup = m_action_lookup_table,
    .event_lookup = m_event_lookup_table,
    .guard_lookup = m_guard_lookup_table,
    .state_lookup = m_state_lookup_table
#endif  /* FSM_DEBUG */
};

static const app_transition_fsm_action_t app_transition_fsm_actions[] =
{
    DECLARE_HANDLER(ACTION_LIST)
};

static void app_transition_fsm_action(fsm_action_id_t action_id, void * p_data)
{
    app_transition_fsm_actions[action_id](p_data);
}

static const app_transition_fsm_guard_t app_transition_fsm_guards[] =
{
    DECLARE_HANDLER(GUARD_LIST)
};

static bool app_transition_fsm_guard(fsm_guard_id_t guard_id, void * p_data)
{
    return app_transition_fsm_guards[guard_id](p_data);
}
/*lint +e123 */
/**************************************************************************************************/
/***** State machine functions *****/
/* Note: The task of this state machine is to implement gradual changes of `present_level` value
 * for all possible time intervals and step size combinations allowed by the level model */

static void a_delay_start(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    timer_sch_reschedule(&p_transition->delay_timer, timer_now() + MS_TO_US(p_transition->delay_ms));

    if (p_transition->delay_start_cb != NULL)
    {
        p_transition->delay_start_cb(p_transition);
    }
}

static void a_transition_start(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    /* Calculate required delta factors required for level change. */
    uint32_t abs_delta;
    uint64_t transition_time_us;
    uint64_t transition_step_us;
    uint64_t min_transition_step_us;

    // abort ongoing transition if there is any
    model_timer_abort(&p_transition->timer);
    p_transition->ongoing_params = p_transition->requested_params;
    app_transition_params_t * p_params = &p_transition->ongoing_params;

    abs_delta = abs(p_params->required_delta);

    NRF_MESH_ASSERT_DEBUG(abs_delta > 0);

    transition_time_us = MS_TO_US((uint64_t) p_params->transition_time_ms);
    transition_step_us = transition_time_us/abs_delta;
    min_transition_step_us = MS_TO_US((uint64_t) p_params->minimum_step_ms);
    if (transition_step_us < min_transition_step_us)
    {
        transition_step_us = min_transition_step_us;
    }

    if (transition_step_us < MS_TO_US(MODEL_TIMER_PERIOD_MS_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS)))
    {
        /* We cannot fire timer more frequently than MODEL_TIMER_TIMEOUT_MIN_TICKS, thus
        increment present level in suitable steps. */
        p_transition->timer.timeout_rtc_ticks = MODEL_TIMER_TIMEOUT_MIN_TICKS;
    }
    else
    {
        /* Perform level transition using time steps corresponding to one step change. */
        p_transition->timer.timeout_rtc_ticks = MODEL_TIMER_TICKS_GET_US(transition_step_us);
    }

    p_transition->timer.mode = MODEL_TIMER_MODE_REPEATED;
    uint32_t status = model_timer_schedule(&p_transition->timer);

    if ((NRF_SUCCESS == status) && p_transition->transition_start_cb)
    {
        p_transition->transition_start_cb(p_transition);
    }
}

static void a_transition_tick(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    if (p_transition->transition_tick_cb != NULL)
    {
        p_transition->transition_tick_cb(p_transition);
    }
}

static void transition_complete_common(app_transition_t * p_transition)
{
    model_timer_abort(&p_transition->timer);
    app_transition_params_t * p_params = &p_transition->ongoing_params;
    p_params->transition_time_ms = 0;

    if (p_transition->transition_complete_cb != NULL)
    {
        p_transition->transition_complete_cb(p_transition);
    }
}

static void a_transition_complete(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    transition_complete_common(p_transition);
}

static void a_transition_complete_with_param_swap(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    p_transition->ongoing_params = p_transition->requested_params;
    transition_complete_common(p_transition);
}

static void a_abort(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    p_transition->delay_ms = 0;
    p_transition->ongoing_params.transition_time_ms = 0;
    timer_sch_abort(&p_transition->delay_timer);
    model_timer_abort(&p_transition->timer);
}

static bool g_set_delay(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;

    return (p_transition->delay_ms > 0);
}

static bool g_set_transition(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;
    app_transition_params_t * p_params = &p_transition->requested_params;

    /* Requirement: If transition time is not within valid range, or given transition delta level
    is zero, do not start the transition. */
    return (p_params->transition_time_ms > 0 &&
            p_params->transition_time_ms != MODEL_TRANSITION_TIME_UNKNOWN &&
            p_params->required_delta != 0);
}

static bool g_transition_complete(void * p_data)
{
    app_transition_t * p_transition = (app_transition_t *) p_data;
    app_transition_params_t * p_params = &p_transition->ongoing_params;

    return (p_params->transition_type != APP_TRANSITION_TYPE_MOVE_SET &&
           (MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_transition->timer)) >= p_params->transition_time_ms));
}

static void delay_cb(timestamp_t timestamp, void * p_context)
{
    (void)timestamp;
    app_transition_t * p_transition = (app_transition_t *) p_context;
    p_transition->delay_ms = 0;

    fsm_event_post(&p_transition->fsm, E_DELAY_EXPIRED, p_transition);
}

static void transition_timer_cb(void * p_context)
{
    app_transition_t * p_transition = (app_transition_t *) p_context;

    fsm_event_post(&p_transition->fsm, E_TIMEOUT, p_transition);
}

/***** Interface functions *****/
void app_transition_trigger(app_transition_t * p_transition)
{
    NRF_MESH_ASSERT(p_transition != NULL);
    fsm_event_post(&p_transition->fsm, E_START, p_transition);
}

void app_transition_abort(app_transition_t *p_transition)
{
    NRF_MESH_ASSERT(p_transition != NULL);
    fsm_event_post(&p_transition->fsm, E_ABORT, p_transition);
}

uint32_t app_transition_remaining_time_get(app_transition_t * p_transition)
{
    NRF_MESH_ASSERT(p_transition != NULL);

    app_transition_params_t * p_params = &p_transition->ongoing_params;
    return (p_params->transition_time_ms - MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_transition->timer)));
}

uint32_t app_transition_elapsed_time_get(app_transition_t * p_transition)
{
    NRF_MESH_ASSERT(p_transition != NULL);

    return (MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&p_transition->timer)));
}

bool app_transition_time_complete_check(app_transition_t * p_transition)
{
    NRF_MESH_ASSERT(p_transition != NULL);
    app_transition_params_t * p_params = &p_transition->ongoing_params;

    return p_params->transition_time_ms == 0 && p_transition->delay_ms == 0;
}

uint32_t app_transition_init(app_transition_t * p_transition)
{
    uint32_t value;

    NRF_MESH_ASSERT(p_transition != NULL);

    p_transition->delay_timer.p_context = p_transition;
    p_transition->delay_timer.cb = delay_cb;

    p_transition->timer.p_context = p_transition;
    p_transition->timer.cb = transition_timer_cb;
    value =  model_timer_create(&p_transition->timer);

    fsm_init(&p_transition->fsm, &m_fsm_descriptor);

    return value;
}
