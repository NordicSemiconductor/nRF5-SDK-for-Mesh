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
#include "unity.h"
#include "cmock.h"
#include "test_assert.h"

#include "proxy_filter.h"

#include "nrf_error.h"
#include "utils.h"
#include "nrf_mesh_defines.h"


void setUp(void)
{

}

void tearDown(void)
{

}


/*****************************************************************************
* Test functions
*****************************************************************************/
void test_adding(void)
{
    proxy_filter_t filter;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_WHITELIST));

    /* Add a list of addresses, should ignore duplicates */
    uint16_t addrs[] = {0x0001, 0xFFFF, 0x1234, 0xcccc, 0x0001};
    proxy_filter_add(&filter, addrs, ARRAY_SIZE(addrs));
    TEST_ASSERT_EQUAL(4, filter.count);

    /* Only accept the added addrs: */
    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_TRUE(proxy_filter_accept(&filter, addrs[i]));
    }

    TEST_ASSERT_FALSE(proxy_filter_accept(&filter, 0x0000));
    TEST_ASSERT_FALSE(proxy_filter_accept(&filter, 0xFFFE));
    TEST_ASSERT_FALSE(proxy_filter_accept(&filter, 0x5678));

    /* Append addr, should accept that one, as well as all the old ones: */
    uint16_t new_addr = 0x00EE;
    TEST_ASSERT_FALSE(proxy_filter_accept(&filter, new_addr));
    proxy_filter_add(&filter, &new_addr, 1);

    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, new_addr));
    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_TRUE(proxy_filter_accept(&filter, addrs[i]));
    }

    /* Add some new and some old addrs, should add the new ones, and ignore the old: */
    uint16_t additional_addrs[] = {addrs[0], 0xFEFE, addrs[1], 0x4567};
    proxy_filter_add(&filter, additional_addrs, ARRAY_SIZE(additional_addrs));
    /* Should accept all previously added addrs */
    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_TRUE(proxy_filter_accept(&filter, addrs[i]));
    }
    for (uint32_t i = 0; i < ARRAY_SIZE(additional_addrs); ++i)
    {
        TEST_ASSERT_TRUE(proxy_filter_accept(&filter, additional_addrs[i]));
    }
    /* Should only have added unique addresses: */
    TEST_ASSERT_EQUAL(7, filter.count);

    /* Set the filter type again, should wipe the list */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_WHITELIST));

    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_FALSE(proxy_filter_accept(&filter, addrs[i]));
    }

    /* Unassigned addresses should not be added */
    uint16_t unassigned = NRF_MESH_ADDR_UNASSIGNED;
    TEST_ASSERT_FALSE(proxy_filter_accept(&filter, unassigned));
    proxy_filter_add(&filter, &unassigned, 1);
    TEST_ASSERT_EQUAL(0, filter.count);

}

void test_blacklist(void)
{
    proxy_filter_t filter;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_BLACKLIST));

    /* Should accept anything if the list is empty: */
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0x0000));
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0xFFFE));
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0x5678));

    /* Reject any added addrs */
    uint16_t addrs[] = {0x0001, 0xFFFF, 0x1234, 0xcccc};
    proxy_filter_add(&filter, addrs, ARRAY_SIZE(addrs));

    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_FALSE(proxy_filter_accept(&filter, addrs[i]));
    }

    /* Should still accept anything not in the list: */
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0x0000));
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0xFFFE));
    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, 0x5678));
}

void test_remove(void)
{
    proxy_filter_t filter;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_WHITELIST));

    uint16_t addrs[] = {0x0001, 0xFFFF, 0x1234, 0xcccc};
    proxy_filter_add(&filter, addrs, ARRAY_SIZE(addrs));
    TEST_ASSERT_EQUAL(4, filter.count);

    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_TRUE(proxy_filter_accept(&filter, addrs[i]));
    }

    /* Remove all the addresses, should get no hits */
    proxy_filter_remove(&filter, addrs, ARRAY_SIZE(addrs));
    TEST_ASSERT_EQUAL(0, filter.count);

    for (uint32_t i = 0; i < ARRAY_SIZE(addrs); ++i)
    {
        TEST_ASSERT_FALSE(proxy_filter_accept(&filter, addrs[i]));
    }

    proxy_filter_add(&filter, addrs, ARRAY_SIZE(addrs));
    TEST_ASSERT_EQUAL(ARRAY_SIZE(addrs), filter.count);

    /* Remove some addresses, both some that are in the filter and some that are not. */
    uint16_t removed_addrs[] = {addrs[1], 0xAAAA, addrs[0], addrs[3], 0x8765};
    proxy_filter_remove(&filter, removed_addrs, ARRAY_SIZE(removed_addrs));
    TEST_ASSERT_EQUAL(1, filter.count);

    TEST_ASSERT_TRUE(proxy_filter_accept(&filter, addrs[2]));
    for (uint32_t i = 0; i < ARRAY_SIZE(removed_addrs); ++i)
    {
        TEST_ASSERT_FALSE(proxy_filter_accept(&filter, removed_addrs[i]));
    }

}

void test_invalid_params(void)
{
    proxy_filter_t filter;
    TEST_NRF_MESH_ASSERT_EXPECT(proxy_filter_type_set(NULL, PROXY_FILTER_TYPE_WHITELIST));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_RFU_START));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, proxy_filter_type_set(&filter, PROXY_FILTER_TYPE_RFU_START + 1));

    uint16_t addr = 0x1234;

    TEST_NRF_MESH_ASSERT_EXPECT(proxy_filter_add(NULL, &addr, 1));
    TEST_NRF_MESH_ASSERT_EXPECT(proxy_filter_add(&filter, NULL, 1));

    TEST_NRF_MESH_ASSERT_EXPECT(proxy_filter_accept(NULL, addr));
}
