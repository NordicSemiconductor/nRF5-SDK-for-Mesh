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
#ifndef MANUAL_MOCK_QUEUE_H__
#define MANUAL_MOCK_QUEUE_H__

#include "queue.h"
#include <stdlib.h>


/**
 * Define a mock queue.
 *
 * A mock queue can be used to create sequenced expect calls for modules CMock can't cover, like
 * callbacks. The interface is similar to CMock's.
 *
 * Instantiates:
 * - An init function that initializes the queue.
 * - A pending function that returns whether there are any more expect calls pending.
 * - A verify function that verifies that all expected calls have been fired.
 * - A destroy function that cleans up.
 * - An expect function that enqueues the expected entry.
 * - A consume function that can be used in the expected function.
 *
 * Requires queue.c and uses malloc to allocate the entries.
 *
 * @param[in] NAME Name of the calls and queue. Setting @c NAME to my_func creates functions
 * @c my_func_setup, @c my_func_verify, @c my_func_expect, and @c my_func_consume.
 * @param[in] DATA_TYPE data type of the expected value.
 * @param[in] ON_DESTROY_CB Optional callback to be called while destroying the mock queue. For example, if
 * the DATA_TYPE has nested allocations.
 */
#define MOCK_QUEUE_DEF(NAME, DATA_TYPE, ON_DESTROY_CB)                                             \
    typedef struct                                                                                 \
    {                                                                                              \
        queue_elem_t queue_elem;                                                                   \
        DATA_TYPE data;                                                                            \
    } NAME##_expect_t;                                                                             \
    void (*m_##NAME##_destroy_cb)(DATA_TYPE *) = ON_DESTROY_CB;                                           \
    queue_t m_##NAME##_expect_queue;                                                               \
    static void __attribute__((used)) NAME##_Init(void)                                            \
    {                                                                                              \
        queue_init(&m_##NAME##_expect_queue);                                                      \
    }                                                                                              \
    static bool __attribute__((used)) NAME##_Pending(void)                                         \
    {                                                                                              \
        return (queue_peek(&m_##NAME##_expect_queue) != NULL);                                     \
    }                                                                                              \
    static void __attribute__((used)) NAME##_Verify(void)                                          \
    {                                                                                              \
        UNITY_SET_DETAIL(__FUNCTION__);                                                            \
        TEST_ASSERT_FALSE_MESSAGE(NAME##_Pending(), #NAME " called less times than expected.");    \
    }                                                                                              \
    static void __attribute__((used)) NAME##_Destroy(void)                                         \
    {                                                                                              \
        while (NAME##_Pending())                                                                   \
        {                                                                                          \
            NAME##_expect_t * p_elem = queue_pop(&m_##NAME##_expect_queue)->p_data;                        \
            if (m_##NAME##_destroy_cb)                                                             \
            {                                                                                      \
                m_##NAME##_destroy_cb(&p_elem->data);                                              \
            }                                                                                      \
            free(p_elem);                                                                  \
        }                                                                                          \
    }                                                                                              \
    static void __attribute__((used)) NAME##_Expect(DATA_TYPE const * p_data)                      \
    {                                                                                              \
        UNITY_SET_DETAIL(__FUNCTION__);                                                            \
        NAME##_expect_t * p_expect = malloc(sizeof(NAME##_expect_t));                              \
        TEST_ASSERT_NOT_NULL_MESSAGE(p_expect, "Malloc failed!");                                  \
        p_expect->queue_elem.p_data = p_expect;                                                    \
        p_expect->data              = *p_data;                                                     \
        queue_push(&m_##NAME##_expect_queue, &p_expect->queue_elem);                               \
    }                                                                                              \
    static void __attribute__((used)) NAME##_Consume(DATA_TYPE * p_data)                           \
    {                                                                                              \
        UNITY_SET_DETAIL(__FUNCTION__);                                                            \
        queue_elem_t * p_elem = queue_pop(&m_##NAME##_expect_queue);                               \
        TEST_ASSERT_NOT_NULL_MESSAGE(p_elem, #NAME " called more times than expected.");           \
        *p_data = ((NAME##_expect_t *) p_elem->p_data)->data;                                      \
        free(p_elem->p_data);                                                                      \
    }

#endif /* MANUAL_MOCK_QUEUE_H__ */
