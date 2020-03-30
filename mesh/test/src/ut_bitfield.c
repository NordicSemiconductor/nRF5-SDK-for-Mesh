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

#include "bitfield.h"

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
    /*lint -save -e550 Local symbol not accessed (variable bitfield is only used with sizeof()) */
#define TEST_SIZE(BITS, EXPECTED_SIZE) do {                 \
        uint32_t bitfield[BITFIELD_BLOCK_COUNT(BITS)];      \
        TEST_ASSERT_EQUAL(EXPECTED_SIZE, sizeof(bitfield)); \
    } while (0)

    TEST_SIZE(1, 4);
    TEST_SIZE(32, 4);
    TEST_SIZE(33, 8);
    TEST_SIZE(48, 8);
    TEST_SIZE(64, 8);
    TEST_SIZE(65, 12);
#undef TEST_SIZE
    /*lint -restore */
}

void test_set_get(void)
{
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(48)];
    memset(bitfield, 0, sizeof(bitfield));
    for (uint32_t i = 0; i < 48; i++)
    {
        TEST_ASSERT_FALSE(bitfield_get(bitfield, i));
    }
    memset(bitfield, 0xFF, sizeof(bitfield));
    for (uint32_t i = 0; i < 48; i++)
    {
        TEST_ASSERT_TRUE(bitfield_get(bitfield, i));
    }
    memset(bitfield, 0, sizeof(bitfield));

    bitfield_set(bitfield, 0);
    TEST_ASSERT_TRUE(bitfield_get(bitfield, 0));
    bitfield_set(bitfield, 5);
    TEST_ASSERT_TRUE(bitfield_get(bitfield, 5));
    bitfield_set(bitfield, 31);
    TEST_ASSERT_TRUE(bitfield_get(bitfield, 31));

    TEST_ASSERT_EQUAL_HEX32((1u << 0) | (1u << 5) | (1u << 31), bitfield[0]);
    TEST_ASSERT_EQUAL_HEX32(0, bitfield[1]);

    bitfield_set(bitfield, 47);
    TEST_ASSERT_TRUE(bitfield_get(bitfield, 47));
    bitfield_set(bitfield, 32);
    TEST_ASSERT_TRUE(bitfield_get(bitfield, 32));

    TEST_ASSERT_EQUAL_HEX32((1u << 0) | (1u << 5) | (1u << 31), bitfield[0]);
    TEST_ASSERT_EQUAL_HEX32((1 << 0) | (1 << 15), bitfield[1]);

    bitfield_clear(bitfield, 0);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 0));
    bitfield_clear(bitfield, 5);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 5));
    bitfield_clear(bitfield, 17);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 17));
    bitfield_clear(bitfield, 31);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 31));

    TEST_ASSERT_EQUAL_HEX32(0, bitfield[0]);
    TEST_ASSERT_EQUAL_HEX32((1 << 0) | (1 << 15), bitfield[1]);

    bitfield_clear(bitfield, 47);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 47));
    bitfield_clear(bitfield, 37);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 37));
    bitfield_clear(bitfield, 32);
    TEST_ASSERT_FALSE(bitfield_get(bitfield, 32));

    TEST_ASSERT_EQUAL_HEX32(0, bitfield[0]);
    TEST_ASSERT_EQUAL_HEX32(0, bitfield[1]);
}

void test_clear_set_all(void)
{
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(48)];
    bitfield_set_all(bitfield, 48);
    uint32_t expected_set[2] = {0xFFFFFFFF, 0xFFFFFFFF};
    TEST_ASSERT_EQUAL_HEX32_ARRAY(expected_set, bitfield, 2);
    bitfield_clear_all(bitfield, 48);
    uint32_t expected_cleared[2] = {0, 0};
    TEST_ASSERT_EQUAL_HEX32_ARRAY(expected_cleared, bitfield, 2);
    bitfield_set(bitfield, 8);
    bitfield_set(bitfield, 18);
    bitfield_set(bitfield, 38);
    bitfield_clear_all(bitfield, 48);
    TEST_ASSERT_EQUAL_HEX32_ARRAY(expected_cleared, bitfield, 2);
}

void test_next_get(void)
{
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(110)];
    bitfield_clear_all(bitfield, 110);

    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 0));
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 110)); // at boundary
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 180000)); // way out of bounds

    bitfield_set(bitfield, 0);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield, 110, 0));
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 1));

    bitfield_set(bitfield, 8);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield, 110, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield, 110, 1));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield, 110, 8));
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 9));

    bitfield_set(bitfield, 9);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield, 110, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield, 110, 1));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield, 110, 8));
    TEST_ASSERT_EQUAL(9, bitfield_next_get(bitfield, 110, 9));
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 10));

    bitfield_set(bitfield, 39);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield, 110, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield, 110, 1));
    TEST_ASSERT_EQUAL(9, bitfield_next_get(bitfield, 110, 9));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield, 110, 10));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield, 110, 39));
    TEST_ASSERT_EQUAL(110, bitfield_next_get(bitfield, 110, 40));

    uint32_t bitfield64[BITFIELD_BLOCK_COUNT(64)];
    bitfield_clear_all(bitfield64, 64);

    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 64)); // at boundary
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 180000)); // way out of bounds

    bitfield_set(bitfield64, 0);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 1));

    bitfield_set(bitfield64, 8);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 1));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 8));
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 9));

    bitfield_set(bitfield64, 9);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 1));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 8));
    TEST_ASSERT_EQUAL(9, bitfield_next_get(bitfield64, 64, 9));
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 10));

    bitfield_set(bitfield64, 39);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 1));
    TEST_ASSERT_EQUAL(9, bitfield_next_get(bitfield64, 64, 9));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 10));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 39));
    TEST_ASSERT_EQUAL(64, bitfield_next_get(bitfield64, 64, 40));

    bitfield_set(bitfield64, 63);
    TEST_ASSERT_EQUAL(0, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(8, bitfield_next_get(bitfield64, 64, 1));
    TEST_ASSERT_EQUAL(9, bitfield_next_get(bitfield64, 64, 9));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 10));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 39));
    TEST_ASSERT_EQUAL(63, bitfield_next_get(bitfield64, 64, 40));

    bitfield_clear(bitfield64, 0);
    bitfield_clear(bitfield64, 8);
    bitfield_clear(bitfield64, 9);
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 0));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 1));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 9));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 10));
    TEST_ASSERT_EQUAL(39, bitfield_next_get(bitfield64, 64, 39));
    TEST_ASSERT_EQUAL(63, bitfield_next_get(bitfield64, 64, 40));
}

void test_popcount(void)
{
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(64)];
    bitfield_clear_all(bitfield, 64);
    TEST_ASSERT_EQUAL(0, bitfield_popcount(bitfield, 64));

    bitfield_set(bitfield, 1);
    TEST_ASSERT_EQUAL(1, bitfield_popcount(bitfield, 64));

    bitfield_set(bitfield, 0);
    TEST_ASSERT_EQUAL(2, bitfield_popcount(bitfield, 64));

    bitfield_set(bitfield, 63);
    TEST_ASSERT_EQUAL(3, bitfield_popcount(bitfield, 64));

    bitfield_set_all(bitfield, 64);
    TEST_ASSERT_EQUAL(64, bitfield_popcount(bitfield, 64));
}

