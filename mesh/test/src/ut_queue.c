/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#include "queue.h"
#include <stdint.h>
#include <string.h>
#include <unity.h>
#include <cmock.h>
#include "nrf_mesh_assert.h"

nrf_mesh_assertion_handler_t m_assertion_handler;

typedef struct
{
    int payload;
    queue_elem_t queue_elem;
} test_elem_t;

void setUp(void)
{

}

void tearDown(void)
{

}




/******** Tests ********/
void test_add_single(void)
{
    queue_t q;
    queue_init(&q);
    test_elem_t elem;
    elem.queue_elem.p_data = &elem;
    elem.payload = 42;

    queue_push(&q, &elem.queue_elem);
    TEST_ASSERT_EQUAL(42, elem.payload);
    TEST_ASSERT_NULL(elem.queue_elem.p_next);
    TEST_ASSERT_EQUAL_PTR(&elem.queue_elem, q.p_back);
    TEST_ASSERT_EQUAL_PTR(&elem.queue_elem, queue_pop(&q));
    TEST_ASSERT_NULL(queue_pop(&q));

    queue_push(&q, &elem.queue_elem);
    TEST_ASSERT_EQUAL(42, elem.payload);
    TEST_ASSERT_NULL(elem.queue_elem.p_next);
    TEST_ASSERT_EQUAL_PTR(&elem.queue_elem, queue_pop(&q));
    TEST_ASSERT_NULL(queue_pop(&q));
}

void test_add_multiple(void)
{
    queue_t q;
    queue_init(&q);
    test_elem_t elems[10];
    for (int rounds = 0; rounds < 2; rounds++)
    {
        for (int i = 0; i < 10; ++i)
        {
            elems[i].payload = 42;
            elems[i].queue_elem.p_data = &elems[i];
            queue_push(&q, &elems[i].queue_elem);
        }
        for (int i = 0; i < 10; ++i)
        {
            TEST_ASSERT_EQUAL(42, elems[i].payload);
            if (i < 9)
            {
                TEST_ASSERT_EQUAL_PTR(&elems[i + 1].queue_elem, elems[i].queue_elem.p_next);
            }
            else
            {
                TEST_ASSERT_NULL(elems[i].queue_elem.p_next);
            }
            TEST_ASSERT_EQUAL_PTR(&elems[i].queue_elem, queue_pop(&q));
        }
        TEST_ASSERT_NULL(queue_pop(&q));
    }
}

void test_peek(void)
{
    queue_t q;
    queue_init(&q);
    test_elem_t elems[3];
    elems[0].queue_elem.p_data = &elems[0];
    elems[1].queue_elem.p_data = &elems[1];
    elems[2].queue_elem.p_data = &elems[2];
    TEST_ASSERT_NULL(queue_peek(&q));
    queue_push(&q, &elems[0].queue_elem);
    TEST_ASSERT_EQUAL_PTR(&elems[0].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[0].queue_elem, queue_peek(&q));

    queue_push(&q, &elems[1].queue_elem);
    queue_push(&q, &elems[2].queue_elem);

    TEST_ASSERT_EQUAL_PTR(&elems[0].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[0].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[0].queue_elem, queue_pop(&q));

    TEST_ASSERT_EQUAL_PTR(&elems[1].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[1].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[1].queue_elem, queue_pop(&q));

    TEST_ASSERT_EQUAL_PTR(&elems[2].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[2].queue_elem, queue_peek(&q));
    TEST_ASSERT_EQUAL_PTR(&elems[2].queue_elem, queue_pop(&q));
}
