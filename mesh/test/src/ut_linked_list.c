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

#include <stdint.h>
#include <string.h>

#include <nrf_error.h>

#include "utils.h"
#include "list.h"
#include "test_assert.h"

#define TEST_ELEM_SIZE (16)

typedef struct
{
    list_node_t node;
    uint32_t data;
} test_type_1_t;


typedef struct
{
    uint32_t data;
    uint8_t  data2;
    list_node_t node;
} test_type_2_t;

static test_type_1_t type1_list[TEST_ELEM_SIZE];
static test_type_1_t type2_list[TEST_ELEM_SIZE];

static list_node_t * list_head = NULL;

static bool list_cmp_cb(const list_node_t * p1, const list_node_t * p2)
{
    return (PARENT_BY_FIELD_GET(test_type_1_t, node, p1)->data <
            PARENT_BY_FIELD_GET(test_type_1_t, node, p2)->data);
}

static bool compare_data_cb(const list_node_t * p1, const list_node_t * p2)
{
    return (PARENT_BY_FIELD_GET(test_type_1_t, node, p1)->data ==
            PARENT_BY_FIELD_GET(test_type_1_t, node, p2)->data);
}

static void print_list(void)
{
    for (list_node_t * n = list_head; n != NULL; n = n->p_next)
    {
        printf("(%2u ) -> ", PARENT_BY_FIELD_GET(test_type_1_t, node, n)->data);
    }
    printf("\n");
}


void setUp(void)
{
    memset(type1_list, 0, sizeof(type1_list));
    memset(type2_list, 0, sizeof(type2_list));
    list_head = NULL;
}

void tearDown(void)
{
}

void test_add_remove_get_size(void)
{
    list_add(&list_head, &type1_list[0].node);
    TEST_ASSERT(list_head != NULL);
    TEST_ASSERT(list_head->p_next == NULL);
    TEST_ASSERT_EQUAL(1, list_size_get(list_head));

    for (int i = 1; i < TEST_ELEM_SIZE; ++i)
    {
        list_add(&list_head, &type1_list[i].node);
    }

    TEST_ASSERT_EQUAL(TEST_ELEM_SIZE, list_size_get(list_head));

    for (int i = 0; i < TEST_ELEM_SIZE; ++i)
    {
        list_add(&list_head, &type2_list[i].node);
    }

    TEST_ASSERT_EQUAL(TEST_ELEM_SIZE * 2, list_size_get(list_head));

    /* remove a random item.. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, list_remove(&list_head, &type1_list[4].node));

    TEST_ASSERT_EQUAL(TEST_ELEM_SIZE * 2 - 1, list_size_get(list_head));
}


void test_add_sorted(void)
{
    for (int i = 0; i < TEST_ELEM_SIZE; ++i)
    {
        type1_list[i].data = i;
    }

    list_sorted_add(&list_head, &type1_list[TEST_ELEM_SIZE-1].node, list_cmp_cb);
    list_sorted_add(&list_head, &type1_list[1].node, list_cmp_cb);
    list_sorted_add(&list_head, &type1_list[4].node, list_cmp_cb);
    list_sorted_add(&list_head, &type1_list[2].node, list_cmp_cb);
    list_sorted_add(&list_head, &type1_list[3].node, list_cmp_cb);
    list_sorted_add(&list_head, &type1_list[0].node, list_cmp_cb);

    TEST_ASSERT_EQUAL(list_head, &type1_list[0]);
}

void test_compare_add(void)
{
    for (int i = 0; i < TEST_ELEM_SIZE; ++i)
    {
        type1_list[i].data = i;
    }

    TEST_ASSERT_EQUAL(NRF_SUCCESS, list_compare_add(&list_head, &type1_list[0].node, compare_data_cb));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, list_compare_add(&list_head, &type1_list[2].node, compare_data_cb));

    type1_list[1].data = type1_list[0].data;
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, list_compare_add(&list_head, &type1_list[1].node, compare_data_cb));
    TEST_ASSERT_EQUAL(2, list_size_get(list_head));
    print_list();

    type1_list[1].data = 1000;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, list_compare_add(&list_head, &type1_list[1].node, compare_data_cb));
    TEST_ASSERT_EQUAL(3, list_size_get(list_head));
    print_list();
}

void test_add_item_twice(void)
{
    list_node_t * head = NULL;
    test_type_1_t items[3];

    for (int i = 0; i < 3; ++i)
    {
        items[i].data = i;
    }

    list_add(&head, &items[0].node);
    TEST_NRF_MESH_ASSERT_EXPECT(list_add(&head, &items[0].node));
    TEST_NRF_MESH_ASSERT_EXPECT(list_sorted_add(&head, &items[0].node, list_cmp_cb));
    TEST_NRF_MESH_ASSERT_EXPECT(list_compare_add(&head, &items[0].node, compare_data_cb));

    list_add(&head, &items[1].node);
    list_add(&head, &items[2].node);
    TEST_NRF_MESH_ASSERT_EXPECT(list_add(&head, &items[2].node));
    TEST_NRF_MESH_ASSERT_EXPECT(list_sorted_add(&head, &items[2].node, list_cmp_cb));
    TEST_NRF_MESH_ASSERT_EXPECT(list_compare_add(&head, &items[2].node, compare_data_cb));
}
