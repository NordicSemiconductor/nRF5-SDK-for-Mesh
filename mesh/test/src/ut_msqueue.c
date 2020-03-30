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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "msqueue.h"
#include "test_assert.h"

#define INIT_MSQ(QUEUE, ELEMENTS, STAGES, ELEM_TYPE) do {  \
    static ELEM_TYPE elements[ELEMENTS];        \
    static uint8_t stages[STAGES];              \
    QUEUE.elem_count = ELEMENTS;                \
    QUEUE.p_elem_array = elements;              \
    QUEUE.p_stages = stages;                    \
    QUEUE.elem_size = sizeof(ELEM_TYPE);        \
    QUEUE.stage_count = STAGES;                 \
    msq_init(&QUEUE);                           \
} while (0)

typedef struct
{
    uint8_t big_data[255];
} big_struct_t;

void setUp(void)
{
}

void tearDown(void)
{
}


/*****************************************************************************
* Tests
*****************************************************************************/
void test_init(void)
{
    msq_t queue;
    uint32_t elements[8];
    memset(elements, 0xCD, sizeof(elements));
    uint8_t stages[8];
    memset(stages, 0xAB, sizeof(stages));
    queue.elem_count = 8;
    queue.p_elem_array = elements;
    queue.p_stages = stages;
    queue.elem_size = sizeof(uint32_t);
    queue.stage_count = 5; // odd number, shouldn't matter

    msq_init(&queue);
    for (uint32_t i = 0; i < queue.stage_count; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0, stages[i]);
    }
    for (uint32_t i = 0; i < queue.elem_count; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0xCDCDCDCD, elements[i]);
    }
    queue.elem_count = 9;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.elem_count = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.elem_count = 8;
    queue.elem_size = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.elem_size = sizeof(uint32_t);
    queue.p_elem_array = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.p_elem_array = elements;
    queue.p_stages = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.p_stages = stages;
    queue.stage_count = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
    queue.stage_count = 1;
    TEST_NRF_MESH_ASSERT_EXPECT(msq_init(&queue));
}

void test_simple(void)
{
    msq_t queue;
    INIT_MSQ(queue, 8, 3, uint32_t);

    TEST_ASSERT_EQUAL_PTR(queue.p_elem_array, msq_get(&queue, 0));
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 1));
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 2));
    TEST_ASSERT_EQUAL(8, msq_available(&queue, 0));
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 1));
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 2));
    uint32_t * p_value = msq_get(&queue, 0);
    *p_value = 56;

    msq_move(&queue, 1); // no effect, as next stage isn't ready yet
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 1));

    msq_move(&queue, 0);
    TEST_ASSERT_EQUAL_PTR((uint8_t *) queue.p_elem_array + 4, msq_get(&queue, 0));
    TEST_ASSERT_EQUAL_PTR(queue.p_elem_array, msq_get(&queue, 1));
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 2));
    TEST_ASSERT_EQUAL(7, msq_available(&queue, 0));
    TEST_ASSERT_EQUAL(1, msq_available(&queue, 1));
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 2));
    TEST_ASSERT_EQUAL(56, *((uint32_t *) msq_get(&queue, 1)));

    *((uint32_t *) msq_get(&queue, 0)) = 96;
    msq_move(&queue, 0);
    msq_move(&queue, 1);
    TEST_ASSERT_EQUAL_PTR((uint8_t *) queue.p_elem_array + 8, msq_get(&queue, 0));
    TEST_ASSERT_EQUAL_PTR((uint8_t *) queue.p_elem_array + 4, msq_get(&queue, 1));
    TEST_ASSERT_EQUAL_PTR(queue.p_elem_array, msq_get(&queue, 2));
    TEST_ASSERT_EQUAL(6, msq_available(&queue, 0));
    TEST_ASSERT_EQUAL(1, msq_available(&queue, 1));
    TEST_ASSERT_EQUAL(1, msq_available(&queue, 2));
    TEST_ASSERT_EQUAL(96, *((uint32_t *) msq_get(&queue, 1)));

    msq_move(&queue, 1);
    msq_move(&queue, 2);
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 1));
    TEST_ASSERT_EQUAL_PTR((uint8_t *) queue.p_elem_array + 4, msq_get(&queue, 2));
    msq_move(&queue, 2);
    TEST_ASSERT_EQUAL_PTR(NULL, msq_get(&queue, 2));
}

void test_move_multiple(void)
{
    msq_t queue;
    INIT_MSQ(queue, 8, 3, uint32_t);

    *((uint32_t *) msq_get(&queue, 0)) = 0xAA;
    TEST_ASSERT_EQUAL(8, msq_available(&queue, 0));
    msq_move(&queue, 0);
    *((uint32_t *) msq_get(&queue, 0)) = 0xBB;
    TEST_ASSERT_EQUAL(7, msq_available(&queue, 0));
    msq_move(&queue, 0);
    *((uint32_t *) msq_get(&queue, 0)) = 0xCC;
    TEST_ASSERT_EQUAL(6, msq_available(&queue, 0));
    msq_move(&queue, 0);
    TEST_ASSERT_EQUAL(5, msq_available(&queue, 0));

    TEST_ASSERT_EQUAL(0xAA, *((uint32_t *) msq_get(&queue, 1)));
    TEST_ASSERT_EQUAL(3, msq_available(&queue, 1));
    msq_move(&queue, 1);
    TEST_ASSERT_EQUAL(0xBB, *((uint32_t *) msq_get(&queue, 1)));
    TEST_ASSERT_EQUAL(2, msq_available(&queue, 1));
    msq_move(&queue, 1);
    TEST_ASSERT_EQUAL(0xCC, *((uint32_t *) msq_get(&queue, 1)));
    TEST_ASSERT_EQUAL(1, msq_available(&queue, 1));
    msq_move(&queue, 1);
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 1));

    TEST_ASSERT_EQUAL(0xAA, *((uint32_t *) msq_get(&queue, 2)));
    TEST_ASSERT_EQUAL(3, msq_available(&queue, 2));
    msq_move(&queue, 2);
    TEST_ASSERT_EQUAL(0xBB, *((uint32_t *) msq_get(&queue, 2)));
    TEST_ASSERT_EQUAL(2, msq_available(&queue, 2));
    msq_move(&queue, 2);
    TEST_ASSERT_EQUAL(0xCC, *((uint32_t *) msq_get(&queue, 2)));
    TEST_ASSERT_EQUAL(1, msq_available(&queue, 2));
    msq_move(&queue, 2);
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 2));
}

void test_fill_each_stage(void)
{
    msq_t queue;
    INIT_MSQ(queue, 8, 3, uint32_t);

    uint32_t set_value = 0;
    TEST_ASSERT_EQUAL(8, msq_available(&queue, 0));
    while (msq_get(&queue, 0) != NULL)
    {
        *((uint32_t *) msq_get(&queue, 0)) = set_value++;
        msq_move(&queue, 0);
    }
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 0));
    uint32_t get_value = 0;
    TEST_ASSERT_EQUAL(8, msq_available(&queue, 1));
    while (msq_get(&queue, 1) != NULL)
    {
        TEST_ASSERT_EQUAL(get_value++, *((uint32_t *) msq_get(&queue, 1)));
        msq_move(&queue, 1);
    }
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 1));
    get_value = 0;
    TEST_ASSERT_EQUAL(8, msq_available(&queue, 2));
    while (msq_get(&queue, 2) != NULL)
    {
        TEST_ASSERT_EQUAL(get_value++, *((uint32_t *) msq_get(&queue, 2)));
        msq_move(&queue, 2);
    }
    TEST_ASSERT_EQUAL(0, msq_available(&queue, 2));
}

void test_overflow_indexes(void)
{
    msq_t queue;
    INIT_MSQ(queue, 8, 3, uint32_t);

    for (uint32_t i = 0; i < 256; i++)
    {
        *((uint32_t *) msq_get(&queue, 0)) = i;
        msq_move(&queue, 0);
        TEST_ASSERT_EQUAL(i, *((uint32_t *) msq_get(&queue, 1)));
        msq_move(&queue, 1);
        TEST_ASSERT_EQUAL(i, *((uint32_t *) msq_get(&queue, 2)));
        msq_move(&queue, 2);
    }
}

void test_invalid_params(void)
{
    msq_t queue;
    INIT_MSQ(queue, 8, 3, uint32_t);

    TEST_NRF_MESH_ASSERT_EXPECT(msq_get(NULL, 0));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_get(&queue, 3));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_move(NULL, 0));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_move(&queue, 3));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_reset(NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_available(NULL, 0));
    TEST_NRF_MESH_ASSERT_EXPECT(msq_available(&queue, 3));
}

void test_big_elems(void)
{
    msq_t queue;
    INIT_MSQ(queue, 128, 3, big_struct_t);

    for (uint32_t i = 0; i < 256; i++)
    {
        ((big_struct_t *) msq_get(&queue, 0))->big_data[0] = i;
        msq_move(&queue, 0);
        TEST_ASSERT_EQUAL(i, ((big_struct_t *) msq_get(&queue, 1))->big_data[0]);
        msq_move(&queue, 1);
        TEST_ASSERT_EQUAL(i, ((big_struct_t *) msq_get(&queue, 2))->big_data[0]);
        msq_move(&queue, 2);
    }

    uint32_t set_value = 0;
    TEST_ASSERT_EQUAL(128, msq_available(&queue, 0));
    while (msq_get(&queue, 0) != NULL)
    {
        ((big_struct_t *) msq_get(&queue, 0))->big_data[0] = set_value++;
        msq_move(&queue, 0);
    }
    uint32_t get_value = 0;
    TEST_ASSERT_EQUAL(128, msq_available(&queue, 1));
    while (msq_get(&queue, 1) != NULL)
    {
        TEST_ASSERT_EQUAL(get_value++, ((big_struct_t *) msq_get(&queue, 1))->big_data[0]);
        msq_move(&queue, 1);
    }
}
