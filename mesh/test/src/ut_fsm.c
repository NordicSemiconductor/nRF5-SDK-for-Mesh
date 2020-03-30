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

#include "fsm.h"

#include <stddef.h>
#include <unity.h>
#include "utils.h"
#include "nordic_common.h"

typedef enum
{
    S_0,
    S_1,
    S_2,
    S_3
} tst_state_id_t;

typedef enum
{
    E_0,
    E_1,
    E_2,
    E_3,
    E_4
} tst_event_id_t;

typedef enum
{
    G_0,
    G_1,
    G_2
} tst_guard_id_t;

typedef enum
{
    A_0,
    A_1,
    A_2
} tst_action_id_t;

static bool tst_guard(fsm_guard_id_t guard_id, void * p_data);
static void tst_action(fsm_action_id_t action_id, void * p_data);

static uint32_t m_tst_action_data;
static uint32_t m_tst_data;
static fsm_t    m_tst_fsm;

void setUp(void)
{}

void tearDown(void)
{}

static bool tst_guard(fsm_guard_id_t guard_id, void * p_data)
{
    uint32_t * p_tst_data = (uint32_t *)p_data;
    switch (guard_id)
    {
        case G_0:
            return (*p_tst_data == 0);

        case G_1:
            return (*p_tst_data == 1);

        case G_2:
            return (*p_tst_data == 2);
    }
    TEST_FAIL_MESSAGE("Unknown guard id");
    return false;
}

static void tst_action(fsm_action_id_t action_id, void * p_data)
{
    (void)p_data;

    switch (action_id)
    {
    case A_0:
        m_tst_action_data = 100;
        break;

    case A_1:
        m_tst_action_data = 101;
        break;

    case A_2:
        m_tst_action_data = 102;
        break;

    default:
        TEST_FAIL_MESSAGE("Unknown action id");
        break;
    }
}

void test_fsm_actions(void)
{
    /* "test verifies that some action will happen if guard returns true after event sending */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (S_0),
        FSM_TRANSITION  (E_1,   G_1,                 A_1,                S_1),
        FSM_TRANSITION  (E_1,   G_2,                 A_2,                S_2),

        FSM_STATE       (S_1),
        FSM_TRANSITION  (E_2,   G_2,                 FSM_NO_ACTION,      S_2),
        FSM_TRANSITION  (E_2,   FSM_OTHERWISE,       A_0,                S_0),

        FSM_STATE       (S_2),
        FSM_TRANSITION  (E_0,   G_1,                 A_1,                S_1),
        FSM_TRANSITION  (E_0,   G_0,                 A_0,                S_0)
    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        tst_guard,
        tst_action
    };

    // after init, FSM is in its initial state
    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // in S_0:

    // guards G_1 & G_2 are false: state is not changed,
    // no action is performed
    m_tst_data = 0;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    TEST_ASSERT_TRUE(m_tst_action_data == 0);

    // guard G_1 is true: switch to S_1,
    // A_1 is performed
    m_tst_data = 1;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    TEST_ASSERT_TRUE(m_tst_action_data == 101);

    // in S_1:

    // guard G_2 is false, but OTHERWISE guard is present: switch to S_0,
    // A_0 is performed
    m_tst_data = 0;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_2, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    TEST_ASSERT_TRUE(m_tst_action_data == 100);

    // in S_0:

    // guard G_2 is true: switch to S_2,
    // A_2 is performed
    m_tst_data = 2;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);
    TEST_ASSERT_TRUE(m_tst_action_data == 102);

    // in S_2:

    // guard G_1 is true: switch to S_1,
    // A_1 is performed
    m_tst_data = 1;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_0, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    TEST_ASSERT_TRUE(m_tst_action_data == 101);

    // in S_1:

    // guard G_2 is true: switch to S_2,
    // no action is performed
    m_tst_data = 2;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_2, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);
    TEST_ASSERT_TRUE(m_tst_action_data == 0);

    // in S_2:

    // guard G_0 is true: switch to S_0,
    // A_0 is performed
    m_tst_data = 0;
    m_tst_action_data = 0;
    fsm_event_post(&m_tst_fsm, E_0, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    TEST_ASSERT_TRUE(m_tst_action_data == 100);
}

void test_fsm_eventless(void)
{
    /* test verifies events handling and changing states if a state has no events */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (S_0),

        FSM_STATE       (S_1),
        FSM_TRANSITION  (E_0,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_2),

        FSM_STATE       (S_2),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0),

        FSM_STATE       (FSM_ANY_STATE),
        FSM_TRANSITION  (E_2,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_3,   FSM_NO_GUARD,   FSM_NO_ACTION,  FSM_SAME_STATE),
        FSM_TRANSITION  (E_4,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0)
    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        NULL,
        NULL
    };

    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);
    fsm_event_post(&m_tst_fsm, E_3, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);
    fsm_event_post(&m_tst_fsm, E_4, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
}

void test_fsm_guards(void)
{
    /* test verifies correctness of guards working */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (S_0),
        FSM_TRANSITION  (E_1,   G_1,                 FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_1,   G_2,                 FSM_NO_ACTION,  S_2),

        FSM_STATE       (S_1),
        FSM_TRANSITION  (E_2,   G_2,                 FSM_NO_ACTION,  S_2),
        FSM_TRANSITION  (E_2,   FSM_OTHERWISE,       FSM_NO_ACTION,  S_0),

        FSM_STATE       (S_2),
        FSM_TRANSITION  (E_0,   G_1,                 FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_0,   G_0,                 FSM_NO_ACTION,  S_0)
    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        tst_guard,
        NULL
    };

    // after init, FSM is in its initial state
    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // in S_0:

    // guards G_1 & G_2 are false: state is not changed
    m_tst_data = 0;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // guard G_1 is true: switch to S_1
    m_tst_data = 1;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // in S_1:

    // guard G_2 is false, but OTHERWISE guard is present: switch to S_0
    m_tst_data = 0;
    fsm_event_post(&m_tst_fsm, E_2, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // in S_0:

    // guard G_2 is true: switch to S_2
    m_tst_data = 2;
    fsm_event_post(&m_tst_fsm, E_1, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // in S_2:

    // guard G_1 is true: switch to S_1
    m_tst_data = 1;
    fsm_event_post(&m_tst_fsm, E_0, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // in S_1:

    // guard G_2 is true: switch to S_2
    m_tst_data = 2;
    fsm_event_post(&m_tst_fsm, E_2, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // in S_2:

    // guard G_0 is true: switch to S_0
    m_tst_data = 0;
    fsm_event_post(&m_tst_fsm, E_0, &m_tst_data);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
}

void test_fsm_simple(void)
{
    /* test verifies events handling and changing states according events */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (S_0),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),

        FSM_STATE       (S_1),
        FSM_TRANSITION  (E_2,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_2),

        FSM_STATE       (S_2),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_0,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0)

    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        NULL,
        NULL
    };

    // after init, FSM is in its initial state
    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // in S_0:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // in S_1:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // in S_2:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    // back in S_2:
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
}

void test_fsm_unexpected(void)
{
    /* test verifies correct mixed work of expected and unexpected events */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (S_0),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),

        FSM_STATE       (S_1),
        FSM_TRANSITION  (E_2,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_2),

        FSM_STATE       (S_2),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_0,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0),

        FSM_STATE       (S_3),
        // empty state (without specific transitions)

        FSM_STATE       (FSM_ANY_STATE),
        FSM_TRANSITION  (E_3,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_3),
        FSM_TRANSITION  (E_4,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0),
    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        NULL,
        NULL
    };

    // after init, FSM is in its initial state
    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // in S_0:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // in S_1:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // in S_2:
    // non-matching events do not change state
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);
    // back in S_2:
    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    // matching event changes state
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);


    // in S_0:
    // default handler of the event
    fsm_event_post(&m_tst_fsm, E_3, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_3);

    // in S_3:
    // default handler of the event
    fsm_event_post(&m_tst_fsm, E_4, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
}

void test_fsm_unexpected_only(void)
{
    /* test verifies that fsm does not change due to unexpected events */

    const fsm_transition_t tst_fsm_transitions[] =
    {
        FSM_STATE       (FSM_ANY_STATE),
        FSM_TRANSITION  (E_0,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_0),
        FSM_TRANSITION  (E_1,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_1),
        FSM_TRANSITION  (E_2,   FSM_NO_GUARD,   FSM_NO_ACTION,  S_2),
    };

    const fsm_const_descriptor_t m_tst_fsm_const =
    {
        tst_fsm_transitions,
        ARRAY_SIZE(tst_fsm_transitions),
        S_0,
        NULL,
        NULL
    };

    // after init, FSM is in its initial state
    fsm_init(&m_tst_fsm, &m_tst_fsm_const);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    // default handlers of the event
    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    fsm_event_post(&m_tst_fsm, E_1, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_1);

    fsm_event_post(&m_tst_fsm, E_2, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_2);

    fsm_event_post(&m_tst_fsm, E_0, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);

    fsm_event_post(&m_tst_fsm, E_3, NULL);
    TEST_ASSERT_TRUE(m_tst_fsm.current_state == S_0);
}
