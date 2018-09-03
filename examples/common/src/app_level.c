/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#include "app_level.h"

#include <stdint.h>
#include <stdlib.h>

#include "sdk_config.h"
#include "example_common.h"
#include "generic_level_server.h"

#include "log.h"
#include "app_timer.h"
#include "fsm.h"
#include "fsm_assistant.h"
#include "nrf_mesh_assert.h"

#define REMAINING_TIME(pserver)             ((pserver)->state.transition_time_ms - MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&(pserver)->timer)))
#define TRANSITION_TIME_COMPLETE(pserver)   (MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&(pserver)->timer)) >= (pserver)->state.transition_time_ms)
#define ELAPSED_TIME(pserver)               (MODEL_TIMER_PERIOD_MS_GET(model_timer_elapsed_ticks_get(&(pserver)->timer)))

/* Forward declaration */
static void generic_level_state_get_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_level_status_params_t * p_out);
static void generic_level_state_set_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_level_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_level_status_params_t * p_out);

static void generic_level_state_delta_set_cb(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_level_delta_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_level_status_params_t * p_out);

static void generic_level_state_move_set_cb(const generic_level_server_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const generic_level_move_set_params_t * p_in,
                                            const model_transition_t * p_in_transition,
                                            generic_level_status_params_t * p_out);

static const generic_level_server_callbacks_t m_level_srv_cbs =
{
    .level_cbs.get_cb = generic_level_state_get_cb,
    .level_cbs.set_cb = generic_level_state_set_cb,
    .level_cbs.delta_set_cb = generic_level_state_delta_set_cb,
    .level_cbs.move_set_cb = generic_level_state_move_set_cb
};

/**************************************************************************************************/
/* Behavioral state machine */
/* Various states of level server behavioral module*/

/*lint -e123 Lint fails to understand particular use of the all fsm macros and throws the warning:
> Macro '_DECLARE_ENUM' defined with arguments at line 185 this is just a warning  --
> The name of a macro defined with arguments was subsequently used without a following '('.
> This is legal but may be an oversight.
In this case it is intentional.*/

#define EVENT_LIST E_SET,                       \
                   E_DELTA_SET,                 \
                   E_MOVE_SET,                  \
                   E_TIMEOUT

#define STATE_LIST  S_IDLE,                     \
                    S_IN_DELAY,                 \
                    S_IN_TRANSITION

#define ACTION_LIST A_DELAY_START,          a_delay_start,         \
                    A_TRANSITION_START,     a_transition_start,    \
                    A_TRANSITION_COMPLETE,  a_transition_complete, \
                    A_TRANSITION_TICK,      a_transition_tick

#define GUARD_LIST  G_SET_DELAY,            g_set_delay,           \
                    G_SET_TRANSITION,       g_set_transition,      \
                    G_TRANSITION_COMPLETE,  g_transition_complete

typedef enum
{
    DECLARE_ENUM(EVENT_LIST)
} app_level_event_ids_t;

typedef enum
{
    DECLARE_ENUM(STATE_LIST)
} app_level_state_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(ACTION_LIST)
} app_level_action_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(GUARD_LIST)
} app_level_guard_ids_t;

typedef void (* app_level_fsm_action_t)(void *);
typedef bool (* app_level_fsm_guard_t)(void *);

/* Not having a semi-colon is intentional. */
DECLARE_ACTION_PROTOTYPE(ACTION_LIST)
DECLARE_GUARD_PROTOTYPE(GUARD_LIST)

static void app_level_fsm_action(fsm_action_id_t action_id, void * p_data);
static bool app_level_fsm_guard(fsm_action_id_t action_id, void * p_data);

static const fsm_transition_t m_app_level_fsm_transition_table[] =
{
    FSM_STATE(S_IDLE),

    FSM_STATE(S_IN_DELAY),
    FSM_TRANSITION(E_TIMEOUT,        G_SET_TRANSITION,       A_TRANSITION_START,    S_IN_TRANSITION),
    FSM_TRANSITION(E_TIMEOUT,        FSM_OTHERWISE,          A_TRANSITION_COMPLETE, S_IDLE),

    FSM_STATE(S_IN_TRANSITION),
    FSM_TRANSITION(E_TIMEOUT,        G_TRANSITION_COMPLETE,  A_TRANSITION_COMPLETE, S_IDLE),
    FSM_TRANSITION(E_TIMEOUT,        FSM_ALWAYS,             A_TRANSITION_TICK,     S_IN_TRANSITION),

    FSM_STATE(FSM_ANY_STATE),
    FSM_TRANSITION(E_SET,             G_SET_DELAY,        A_DELAY_START,        S_IN_DELAY),
    FSM_TRANSITION(E_SET,             G_SET_TRANSITION,   A_TRANSITION_START,   S_IN_TRANSITION),
    FSM_TRANSITION(E_SET,             FSM_OTHERWISE,      A_TRANSITION_COMPLETE,S_IDLE),
    FSM_TRANSITION(E_DELTA_SET,       G_SET_DELAY,        A_DELAY_START,        S_IN_DELAY),
    FSM_TRANSITION(E_DELTA_SET,       G_SET_TRANSITION,   A_TRANSITION_START,   S_IN_TRANSITION),
    FSM_TRANSITION(E_DELTA_SET,       FSM_OTHERWISE,      A_TRANSITION_COMPLETE,S_IDLE),
    FSM_TRANSITION(E_MOVE_SET,        G_SET_DELAY,        A_DELAY_START,        S_IN_DELAY),
    FSM_TRANSITION(E_MOVE_SET,        G_SET_TRANSITION,   A_TRANSITION_START,   S_IN_TRANSITION),
    FSM_TRANSITION(E_MOVE_SET,        FSM_OTHERWISE,      FSM_NO_ACTION,        S_IDLE)
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

static const fsm_const_descriptor_t m_level_behaviour_descriptor =
{
    .transition_table = m_app_level_fsm_transition_table,
    .transitions_count = ARRAY_SIZE(m_app_level_fsm_transition_table),
    .initial_state = S_IDLE,
    .guard = app_level_fsm_guard,
    .action = app_level_fsm_action,
#if FSM_DEBUG
    .fsm_name = "LVL-fsm",
    .action_lookup = m_action_lookup_table,
    .event_lookup = m_event_lookup_table,
    .guard_lookup = m_guard_lookup_table,
    .state_lookup = m_state_lookup_table
#endif  /* FSM_DEBUG */
};

static const app_level_fsm_action_t app_level_fsm_actions[] =
{
    DECLARE_HANDLER(ACTION_LIST)
};

static void app_level_fsm_action(fsm_action_id_t action_id, void * p_data)
{
    app_level_fsm_actions[action_id](p_data);
}

static const app_level_fsm_guard_t app_level_fsm_guards[] =
{
    DECLARE_HANDLER(GUARD_LIST)
};

static bool app_level_fsm_guard(fsm_guard_id_t guard_id, void * p_data)
{
    return app_level_fsm_guards[guard_id](p_data);
}
/*lint +e123 */
/**************************************************************************************************/
/***** State machine functions *****/
/* Note: The task of this state machine is to implement gradual changes of `present_level` value
 * for all possible time intervals and step size combinations allowed by the level model */

static void a_delay_start(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    p_server->timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;
    p_server->timer.timeout_rtc_ticks = MODEL_TIMER_TICKS_GET_MS(p_server->state.delay_ms);

    uint32_t status = model_timer_schedule(&p_server->timer);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Failed to start delay timer.\n");
    }
}

static void a_transition_start(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    /* Calculate required delta factors required for level change. */
    uint32_t abs_delta;
    uint64_t transition_time_us;
    uint64_t transition_step_us;

    if (p_server->state.transition_type == TRANSITION_MOVE_SET)
    {
        abs_delta = abs(p_server->state.params.move.required_move);
    }
    else
    {
        abs_delta = abs(p_server->state.params.set.required_delta);
    }
    transition_time_us = MS_TO_US((uint64_t) p_server->state.transition_time_ms);
    transition_step_us = transition_time_us/abs_delta;

    if (transition_step_us < MS_TO_US(MODEL_TIMER_PERIOD_MS_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS)))
    {
        /* We cannot fire timer more frequently than MODEL_TIMER_TIMEOUT_MIN_TICKS, thus
        increment present level in suitable steps. */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Selecting minimum tick interval\n");
              p_server->timer.timeout_rtc_ticks = MODEL_TIMER_TIMEOUT_MIN_TICKS;
    }
    else
    {
        /* Perform level transition using time steps corresponding to one step change. */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Selecting interval for one step change\n");
              p_server->timer.timeout_rtc_ticks = MODEL_TIMER_TICKS_GET_US(transition_step_us);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "timeout_rtc_ticks: %d ticks\n", (uint32_t) p_server->timer.timeout_rtc_ticks);

    p_server->timer.mode = MODEL_TIMER_MODE_REPEATED;
    uint32_t status = model_timer_schedule(&p_server->timer);

    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Failed to start transition timer\n");
    }
}

static void a_transition_tick(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    if (p_server->state.transition_type != TRANSITION_MOVE_SET)
    {
        /* Calculate new value using linear interpolation and provide to the application. */
        int32_t delta = (p_server->state.target_level - p_server->state.params.set.initial_present_level);
        p_server->state.present_level = p_server->state.params.set.initial_present_level +
                                        (delta * (int32_t)ELAPSED_TIME(p_server) / (int32_t)p_server->state.transition_time_ms);
    }
    else
    {
        p_server->state.present_level = p_server->state.params.move.initial_present_level +
                                        (((int64_t)ELAPSED_TIME(p_server) * (int64_t)p_server->state.params.move.required_move) /
                                          (int64_t)p_server->state.transition_time_ms);
    }

    p_server->level_set_cb(p_server, p_server->state.present_level);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Elapsed time: %d\n", (uint32_t) ELAPSED_TIME(p_server));
}

static void a_transition_complete(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    model_timer_abort(&p_server->timer);
    p_server->state.transition_time_ms = 0;

    p_server->state.present_level = p_server->state.target_level;

    generic_level_status_params_t status_params;
    status_params.present_level = p_server->state.present_level;
    status_params.target_level = p_server->state.target_level;
    status_params.remaining_time_ms = 0;
    (void) generic_level_server_status_publish(&p_server->server, &status_params);

    p_server->level_set_cb(p_server, p_server->state.present_level);
}

static bool g_set_delay(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    return (p_server->state.delay_ms > 0);
}

static bool g_set_transition(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    bool result = false;

    switch (p_server->state.transition_type)
    {
        /* Requirement: If transition time is not within valid range, do not start the transition. */
        case TRANSITION_SET:
        /* fall-through */
        case TRANSITION_DELTA_SET:
            result = (p_server->state.transition_time_ms > 0);
        break;

        /* Requirement: If transition time is not within valid range, or given move level (delta level)
        is zero, do not start the transition. */
        case TRANSITION_MOVE_SET:
            result = (p_server->state.transition_time_ms > 0 &&
                      p_server->state.transition_time_ms != MODEL_TRANSITION_TIME_INVALID &&
                      p_server->state.params.move.required_move != 0);
        break;

        default:
        break;
    }

    return (result);
}

static bool g_transition_complete(void * p_data)
{
    app_level_server_t * p_server = (app_level_server_t *) p_data;

    return (p_server->state.transition_type != TRANSITION_MOVE_SET && TRANSITION_TIME_COMPLETE(p_server));
}

/* Model timer callback. This is used for delay and step-wise transition implementation */
static void level_state_timer_cb(void * p_context)
{
    app_level_server_t * p_server = (app_level_server_t *) p_context;

    fsm_event_post(&p_server->fsm, E_TIMEOUT, p_server);
}

/**************************************************************************************************/
/***** Generic Level model interface callbacks *****/

static void generic_level_state_get_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_level_status_params_t * p_out)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: GET \n");

    app_level_server_t   * p_server = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);

    /* Requirement: Provide the current value of the Level state */
    p_server->level_get_cb(p_server, &p_server->state.present_level);
    p_out->present_level = p_server->state.present_level;
    p_out->target_level = p_server->state.target_level;

    /* Requirement: Report remaining time during processing of SET or DELTA SET,
     *              Report transition time during processing of MOVE */
    if (p_server->state.transition_type == TRANSITION_MOVE_SET)
    {
        p_out->remaining_time_ms = p_server->state.transition_time_ms;
    }
    else
    {
        if (TRANSITION_TIME_COMPLETE(p_server))
        {
            p_out->remaining_time_ms = 0;
        }
        else
        {
            p_out->remaining_time_ms = REMAINING_TIME(p_server);
        }
    }
}

static void generic_level_state_set_cb(const generic_level_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_level_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_level_status_params_t * p_out)
{
    app_level_server_t   * p_server = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);

    /* Requirement: If transition time parameters are unavailable and default transition time state
    is not available, transition shall be instantaneous. */
    if (p_in_transition == NULL)
    {
        p_server->state.delay_ms = 0;
        if (p_server->p_dtt_ms == NULL)
        {
            p_server->state.transition_time_ms = 0;
        }
        else
        {
            p_server->state.transition_time_ms = *p_server->p_dtt_ms;
        }
    }
    else
    {
        p_server->state.delay_ms = p_in_transition->delay_ms;
        p_server->state.transition_time_ms = p_in_transition->transition_time_ms;
    }

    /* Update internal representation of Level value, process timing. */
    p_server->state.target_level = p_in->level;
    p_server->state.transition_type = TRANSITION_SET;
    p_server->state.params.set.initial_present_level = p_server->state.present_level;
    p_server->state.params.set.required_delta = p_server->state.target_level - p_server->state.present_level;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: Level: %d  delay: %d  tt: %d  req-delta: %d \n",
          p_server->state.target_level,  p_server->state.delay_ms, p_server->state.transition_time_ms,
          p_server->state.params.set.required_delta);

    fsm_event_post(&p_server->fsm, E_SET, p_server);

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_server->state.present_level;
        p_out->target_level = p_server->state.target_level;
        p_out->remaining_time_ms = p_server->state.transition_time_ms;
    }
}

static void generic_level_state_delta_set_cb(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_level_delta_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_level_status_params_t * p_out)
{
    app_level_server_t   * p_server = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);

    /* Requirement: If transition time parameters are unavailable and default transition time state
    is not available, transition shall be instantaneous. */
    if (p_in_transition == NULL)
    {
        p_server->state.delay_ms = 0;
        if (p_server->p_dtt_ms == NULL)
        {
            p_server->state.transition_time_ms = 0;
        }
        else
        {
            p_server->state.transition_time_ms = *p_server->p_dtt_ms;
        }
    }
    else
    {
        p_server->state.delay_ms = p_in_transition->delay_ms;
        p_server->state.transition_time_ms = p_in_transition->transition_time_ms;
    }

    p_server->state.transition_type = TRANSITION_DELTA_SET;

    /* Update internal representation of Level value, process timing. */
    /* Requirement: If TID is same as previous TID for the same message, delta value is cumulative. */
    int32_t delta = p_in->delta_level % UINT16_MAX;
    if (!model_transaction_is_new(&p_server->server.tid_tracker))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tid: %d Same TID, assuming cumulative delta set.\n", p_in->tid);
    }
    else
    {
        p_server->state.params.set.initial_present_level = p_server->state.present_level;
    }

    p_server->state.target_level = p_server->state.params.set.initial_present_level + delta;
    p_server->state.params.set.required_delta = delta;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Delta SET: delta: %d  delay: %d  tt: %d\n",
          p_in->delta_level, p_server->state.delay_ms, p_server->state.transition_time_ms);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Delta SET: initial-level: %d  present-level: %d  target-level: %d\n",
          p_server->state.params.set.initial_present_level, p_server->state.present_level, p_server->state.target_level);

    fsm_event_post(&p_server->fsm, E_DELTA_SET, p_server);

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_server->state.present_level;
        p_out->target_level = p_server->state.target_level;
        p_out->remaining_time_ms = p_server->state.transition_time_ms;
    }

}

static void generic_level_state_move_set_cb(const generic_level_server_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const generic_level_move_set_params_t * p_in,
                                            const model_transition_t * p_in_transition,
                                            generic_level_status_params_t * p_out)
{
    app_level_server_t   * p_server = PARENT_BY_FIELD_GET(app_level_server_t, server, p_self);

    /* Update internal representation of Level value, process timing. */
    if (p_in_transition == NULL)
    {
        p_server->state.delay_ms = 0;
        if (p_server->p_dtt_ms == NULL)
        {
            p_server->state.transition_time_ms = 0;
        }
        else
        {
            p_server->state.transition_time_ms = *p_server->p_dtt_ms;
        }

        p_server->state.target_level = p_server->state.present_level;
    }
    else
    {
        p_server->state.delay_ms = p_in_transition->delay_ms;

        /* Requirement: If transition time is out of range, transition cannot be started. However
        this must be stored to respond correctly to the get messages. */
        if (p_in_transition->transition_time_ms > TRANSITION_TIME_STEP_100MS_MAX)
        {
            p_server->state.transition_time_ms = MODEL_TRANSITION_TIME_INVALID;
        }
        else
        {
            p_server->state.transition_time_ms = p_in_transition->transition_time_ms;
        }
    }

    /* Requirement: For the status message: The target Generic Level state is the upper limit of
       the Generic Level state when the transition speed is positive, or the lower limit of the
       Generic Level state when the transition speed is negative. */
    if (p_in->move_level > 0)
    {
        p_server->state.target_level = INT16_MAX;
    }
    else
    {
        p_server->state.target_level = INT16_MIN;
    }

    p_server->state.params.move.required_move = p_in->move_level;
    p_server->state.params.move.initial_present_level = p_server->state.present_level;
    p_server->state.transition_type = TRANSITION_MOVE_SET;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "MOVE SET: move-level: %d  delay: %d  tt: %d \n",
          p_in->move_level, p_server->state.delay_ms, p_server->state.transition_time_ms);

    fsm_event_post(&p_server->fsm, E_MOVE_SET, p_server);

    /* Prepare response */
    if (p_out != NULL)
    {
        p_out->present_level = p_server->state.present_level;
        p_out->target_level = p_server->state.target_level;
        p_out->remaining_time_ms = p_server->state.transition_time_ms;
    }
}


/***** Interface functions *****/

uint32_t app_level_current_value_publish(app_level_server_t * p_server)
{
    p_server->level_get_cb(p_server, &p_server->state.present_level);
    model_timer_abort(&p_server->timer);

    p_server->state.target_level = p_server->state.present_level;
    p_server->state.delay_ms = 0;
    p_server->state.transition_time_ms = 0;

    generic_level_status_params_t status = {
                .present_level = p_server->state.present_level,
                .target_level = p_server->state.target_level,
                .remaining_time_ms = p_server->state.transition_time_ms
            };
    return generic_level_server_status_publish(&p_server->server, &status);
}


uint32_t app_level_init(app_level_server_t * p_server, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_server->level_set_cb == NULL || p_server->level_get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server->server.settings.p_callbacks = &m_level_srv_cbs;
    status = generic_level_server_init(&p_server->server, element_index);

    if (status == NRF_SUCCESS)
    {
        p_server->timer.p_context = p_server;
        p_server->timer.cb = level_state_timer_cb;

        fsm_init(&p_server->fsm, &m_level_behaviour_descriptor);
        status = model_timer_create(&p_server->timer);
    }

    return status;
}

