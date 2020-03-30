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

#include "friend_sublist.h"

#include <unity.h>

#include "nrf_mesh.h"

#include "test_assert.h"

static friend_sublist_t m_fsl;

static uint16_t get_raw_address(uint16_t value, nrf_mesh_address_type_t type)
{
    return value | (type << NRF_MESH_ADDR_TYPE_BITS_OFFSET);
}

static void fill_list(nrf_mesh_address_type_t type)
{
    for (uint16_t i = 0; i < MESH_FRIEND_SUBLIST_SIZE; i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_add(&m_fsl, get_raw_address(i, type)));

        TEST_ASSERT_EQUAL(i + 1, m_fsl.stats.curr_count);
        TEST_ASSERT_EQUAL(i + 1, m_fsl.stats.max_count);
    }

    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.max_count);
}

void setUp(void)
{
    friend_sublist_init(&m_fsl);
}

void tearDown(void)
{
}

void test_init(void)
{
    friend_sublist_t fsl;

    uint8_t *p_fsl = (uint8_t*) &fsl;
    for (size_t i = 0; i < sizeof(friend_sublist_t); i++)
    {
        p_fsl[i] = (uint8_t) i;
    }

    friend_sublist_init(&fsl);

    for (size_t i = 0; i < MESH_FRIEND_SUBLIST_SIZE; i++)
    {
        TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, fsl.addrs[i]);
    }

    TEST_ASSERT_EQUAL(0, fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(0, fsl.stats.hits);
    TEST_ASSERT_EQUAL(0, fsl.stats.lookups);
    TEST_ASSERT_EQUAL(0, fsl.stats.max_count);
    TEST_ASSERT_EQUAL(0, fsl.stats.removed);
}

void test_add(void)
{
    fill_list(NRF_MESH_ADDRESS_TYPE_VIRTUAL);

    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, friend_sublist_add(&m_fsl, get_raw_address(MESH_FRIEND_SUBLIST_SIZE,
                                                                                   NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    /* Adding duplicate returns NRF_SUCCESS */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_add(&m_fsl, get_raw_address(0,
                                                                              NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    friend_sublist_init(&m_fsl);

    fill_list(NRF_MESH_ADDRESS_TYPE_GROUP);

    /* Adding duplicate returns NRF_SUCCESS */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_add(&m_fsl, get_raw_address(0,
                                                                              NRF_MESH_ADDRESS_TYPE_GROUP)));
}

void test_cointains(void)
{
    fill_list(NRF_MESH_ADDRESS_TYPE_VIRTUAL);

    for (uint16_t i = 0; i < MESH_FRIEND_SUBLIST_SIZE; i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                          friend_sublist_contains(&m_fsl, get_raw_address(i, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

        TEST_ASSERT_EQUAL(i + 1, m_fsl.stats.lookups);
        TEST_ASSERT_EQUAL(i + 1, m_fsl.stats.hits);
    }

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, friend_sublist_contains(&m_fsl,
                                                   get_raw_address(MESH_FRIEND_SUBLIST_SIZE,
                                                                   NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
    /* Because the address was not found, the hits should not be changed. */
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE + 1, m_fsl.stats.lookups);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.hits);
}

void test_wrong_address(void)
{
    /* Unicast address can't be added */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_add(&m_fsl, 0x0001));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_contains(&m_fsl, 0x0001));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_remove(&m_fsl, 0x0001));

    /* Unassigned address can't be added */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_add(&m_fsl, NRF_MESH_ADDR_UNASSIGNED));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_contains(&m_fsl, NRF_MESH_ADDR_UNASSIGNED));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, friend_sublist_remove(&m_fsl, NRF_MESH_ADDR_UNASSIGNED));
}

void test_remove(void)
{
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, friend_sublist_remove(&m_fsl, get_raw_address(1, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    TEST_ASSERT_EQUAL(0, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(0, m_fsl.stats.removed);

    fill_list(NRF_MESH_ADDRESS_TYPE_VIRTUAL);

    for (uint16_t i = 0; i < MESH_FRIEND_SUBLIST_SIZE; i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                          friend_sublist_remove(&m_fsl, get_raw_address(i, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

        TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE - i - 1, m_fsl.stats.curr_count);
        TEST_ASSERT_EQUAL(i + 1, m_fsl.stats.removed);
    }

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, friend_sublist_remove(&m_fsl, get_raw_address(1, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    TEST_ASSERT_EQUAL(0, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.removed);
}

void test_null_list(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(friend_sublist_init(NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(friend_sublist_add(NULL, get_raw_address(1, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
    TEST_NRF_MESH_ASSERT_EXPECT(friend_sublist_contains(NULL, get_raw_address(1, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
    TEST_NRF_MESH_ASSERT_EXPECT(friend_sublist_remove(NULL, get_raw_address(1, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
}

void test_debug_info(void)
{
    fill_list(NRF_MESH_ADDRESS_TYPE_VIRTUAL);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_remove(&m_fsl, get_raw_address(0, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE - 1, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(0, m_fsl.stats.hits);
    TEST_ASSERT_EQUAL(0, m_fsl.stats.lookups);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.max_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.removed);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_contains(&m_fsl,
                                                   get_raw_address(1,
                                                                   NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE - 1, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.hits);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.lookups);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.max_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.removed);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, friend_sublist_contains(&m_fsl,
                                                   get_raw_address(MESH_FRIEND_SUBLIST_SIZE,
                                                                   NRF_MESH_ADDRESS_TYPE_VIRTUAL)));
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE - 1, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.hits);
    TEST_ASSERT_EQUAL(2, m_fsl.stats.lookups);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.max_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.removed);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, friend_sublist_add(&m_fsl, get_raw_address(0, NRF_MESH_ADDRESS_TYPE_VIRTUAL)));

    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.curr_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.hits);
    TEST_ASSERT_EQUAL(2, m_fsl.stats.lookups);
    TEST_ASSERT_EQUAL(MESH_FRIEND_SUBLIST_SIZE, m_fsl.stats.max_count);
    TEST_ASSERT_EQUAL(1, m_fsl.stats.removed);
}
