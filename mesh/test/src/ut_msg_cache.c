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

#include <string.h>
#include <stdbool.h>
#include "unity.h"
#include "msg_cache.h"


void setUp(void)
{
    msg_cache_init();
}

void tearDown(void)
{

}


/********************************************/

void test_msg_cache_entry_add_new(void)
{
    uint16_t src;
    uint32_t seq;

    /* sneaky "empty entry" entry */
    src = NRF_MESH_ADDR_UNASSIGNED;
    seq = 0;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(0, 0);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));

    /* valid entry */
    src = 0x1234;
    seq = 0xAAAA00;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));

    /* same address */
    src = 0x1234;
    seq = 0xAAAA01;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));

    /* same seq */
    src = 0xAAAA;
    seq = 0xAAAA01;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));

    /* lower seq */
    src = 0xAAAA;
    seq = 0x000001;
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));

    /* check if old entries are still there */
    if (MSG_CACHE_ENTRY_COUNT > 3)
    {
        src = 0x1234;
        seq = 0xAAAA00;

        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    }

    if (MSG_CACHE_ENTRY_COUNT > 2)
    {
        src = 0x1234;
        seq = 0xAAAA01;

        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    }

    if (MSG_CACHE_ENTRY_COUNT > 1)
    {
        src = 0xAAAA;
        seq = 0xAAAA01;

        TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));
    }
    else
    {
        TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
    }
}

void test_msg_cache_entry_overflow(void)
{
    uint16_t src = 0x1000;
    uint32_t seq = 0xAAAA00;

    /* fill all */
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        src++;
        seq++;

        TEST_ASSERT_FALSE(msg_cache_entry_exists(src, seq));
        msg_cache_entry_add(src, seq);
        TEST_ASSERT_TRUE(msg_cache_entry_exists(src, seq));
    }

    /* check that all entries are still there */
    src = 0x1000;
    seq = 0xAAAA00;
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        src++;
        seq++;

        TEST_ASSERT_TRUE(msg_cache_entry_exists(src, seq));
    }

    /* overflow */
    src++;
    seq++;
    TEST_ASSERT_FALSE(msg_cache_entry_exists(src, seq));
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_TRUE(msg_cache_entry_exists(src, seq));

    /* first entry should no longer be valid */
    src = 0x1001;
    seq = 0xAAAA01;
    TEST_ASSERT_FALSE(msg_cache_entry_exists(src, seq));
}

void test_clear(void)
{
    uint16_t src = 0x1234;
    uint32_t seq = 0xAAAA00;
    msg_cache_entry_add(src, seq);
    TEST_ASSERT_EQUAL(true, msg_cache_entry_exists(src, seq));
    msg_cache_clear();
    TEST_ASSERT_EQUAL(false, msg_cache_entry_exists(src, seq));
}
