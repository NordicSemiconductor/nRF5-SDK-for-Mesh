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

#include "event.h"

#include "unity.h"
#include "cmock.h"

typedef enum
{
    INITIAL_VALUE,
    TEST_IN_CB2,
    TEST_IN_CB5,
    FINAL_VALUE
} test_step_t;

static void event_cb1(const nrf_mesh_evt_t * p_evt);
static void event_cb2(const nrf_mesh_evt_t * p_evt);
static void event_cb3(const nrf_mesh_evt_t * p_evt);
static void event_cb4(const nrf_mesh_evt_t * p_evt);
static void event_cb5(const nrf_mesh_evt_t * p_evt);

static nrf_mesh_evt_handler_t event_handler1 =
{
    .evt_cb = event_cb1
};

static nrf_mesh_evt_handler_t event_handler2 =
{
    .evt_cb = event_cb2
};

static nrf_mesh_evt_handler_t event_handler3 =
{
    .evt_cb = event_cb3
};

static nrf_mesh_evt_handler_t event_handler4 =
{
    .evt_cb = event_cb4
};

static nrf_mesh_evt_handler_t event_handler5 =
{
    .evt_cb = event_cb5
};

static test_step_t test_step;

static void event_cb1(const nrf_mesh_evt_t * p_evt)
{
    TEST_ASSERT_TRUE(test_step == INITIAL_VALUE);
    test_step++;
}

static void event_cb2(const nrf_mesh_evt_t * p_evt)
{
    TEST_ASSERT_TRUE(test_step == TEST_IN_CB2);
    test_step++;
    event_handler_remove(&event_handler3);
    TEST_ASSERT_NOT_NULL(event_handler3.node.p_next);
    event_handler_remove(&event_handler2);
    TEST_ASSERT_NOT_NULL(event_handler2.node.p_next);
    event_handler_remove(&event_handler4);
    event_handler_add(&event_handler5);
    TEST_ASSERT_NOT_NULL(event_handler4.node.p_next);
}

static void event_cb3(const nrf_mesh_evt_t * p_evt)
{
    TEST_FAIL();
}

static void event_cb4(const nrf_mesh_evt_t * p_evt)
{
    TEST_FAIL();
}

static void event_cb5(const nrf_mesh_evt_t * p_evt)
{
    TEST_ASSERT_TRUE(test_step == TEST_IN_CB5);
    test_step++;
}

void setUp(void)
{}

void tearDown(void)
{}

void test_event_handling(void)
{
    nrf_mesh_evt_t event =
    {
       .type = NRF_MESH_EVT_MESSAGE_RECEIVED
    };

    test_step = INITIAL_VALUE;

    event_handler_add(&event_handler1);
    event_handler_add(&event_handler2);
    event_handler_add(&event_handler3);
    event_handler_add(&event_handler4);

    event_handle(&event);

    TEST_ASSERT_NULL(event_handler3.node.p_next);
    TEST_ASSERT_NULL(event_handler2.node.p_next);
    TEST_ASSERT_NULL(event_handler4.node.p_next);

    TEST_ASSERT_TRUE(test_step == FINAL_VALUE);
}
