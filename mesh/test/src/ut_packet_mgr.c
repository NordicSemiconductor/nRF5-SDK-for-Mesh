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

#include <stdint.h>
#include <unity.h>

#include "nrf_mesh.h"
#include "packet_mgr.h"

#define TEST_PACKET_1_SIZE  24
#define TEST_PACKET_2_SIZE  68

void setUp(void)
{
    nrf_mesh_init_params_t init_params;
    memset(&init_params, 0, sizeof(nrf_mesh_init_params_t));
    packet_mgr_init(&init_params);
}

void tearDown(void)
{
}

/* Tests the basic functionality of the packet manager. */
void test_packet_mgr_basic(void)
{
    uint32_t status;
    packet_generic_t * p_test_pkg;

    /* Allocate a test packet: */
    status = packet_mgr_alloc(&p_test_pkg, TEST_PACKET_1_SIZE);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    uint8_t refcount = packet_mgr_refcount_get(p_test_pkg);
    uint16_t size = packet_mgr_size_get(p_test_pkg);
    TEST_ASSERT_EQUAL(1, refcount);
    TEST_ASSERT(size >= TEST_PACKET_1_SIZE);

    /* Free packet: */
    packet_mgr_free(p_test_pkg);
}

/* Tests allocating the largest buffer there is room for. */
void test_packet_mgr_alloc_largest(void)
{
    uint32_t status;
    uint16_t size;
    const uint8_t margin = 4;
    packet_generic_t * p_test_pkg;
    while (packet_mgr_get_free_space() >= PACKET_MGR_PACKET_MAXLEN)
    {
        status = packet_mgr_alloc(&p_test_pkg, PACKET_MGR_PACKET_MAXLEN);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
        /* Check that we are getting the right size every time */
        size = packet_mgr_size_get(p_test_pkg);
        TEST_ASSERT_TRUE(size >= PACKET_MGR_PACKET_MAXLEN);
    }
    /* We should have used up all our allowance */
    status = packet_mgr_alloc(&p_test_pkg, PACKET_MGR_PACKET_MAXLEN);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);

    /* The packet pointer is unchanged, so we still have the last successful allocation: */
    uint8_t refcount = packet_mgr_refcount_get(p_test_pkg);
    TEST_ASSERT_EQUAL(1, refcount);
    /* Release the large packet and allocate a packet smaller than the default size, and the packet should be resized to default value.*/
    packet_mgr_free(p_test_pkg);
    status = packet_mgr_alloc(&p_test_pkg, PACKET_MGR_DEFAULT_PACKET_LEN - margin);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    /* Check that we are getting the right size every time */
    size = packet_mgr_size_get(p_test_pkg);
    TEST_ASSERT_EQUAL(PACKET_MGR_DEFAULT_PACKET_LEN, size);

    /* At this point we know that the next free buffer is at least PACKET_MGR_PACKET_MAXLEN - PACKET_MGR_DEFAULT_PACKET_LEN in size.
       Allocate a packet smaller than this, but larger than the default size. The allocated packet should be resized to be greater than the requested size and a multiple of PACKET_MGR_DEFAULT_PACKET_LEN.*/
    /* The assert below is confirming the assumption made above, if it fails the following test is invalid. */
    TEST_ASSERT_TRUE( (PACKET_MGR_DEFAULT_PACKET_LEN + margin) < (PACKET_MGR_PACKET_MAXLEN - PACKET_MGR_DEFAULT_PACKET_LEN) );
    status = packet_mgr_alloc(&p_test_pkg, PACKET_MGR_DEFAULT_PACKET_LEN + margin);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    /* Check that we are getting the right size every time */
    size = packet_mgr_size_get(p_test_pkg);
    TEST_ASSERT_EQUAL(PACKET_MGR_DEFAULT_PACKET_LEN*2, size);

}


/* Tests allocating the largest buffer there is room for. */
void test_packet_mgr_alloc_mixed_sizes(void)
{
    uint32_t status;
    uint16_t size;

    /* Struct for packet and size pairs. */
    typedef struct
    {
        packet_generic_t * p_test_pkg;
        uint32_t expected_size;
    }mem_expected_size_pair_t;

    /* Test different packet sizes (big ones surrounded by small ones to get defragmentation).*/
    static mem_expected_size_pair_t alloc_set_table[] =
    {
        {NULL, 1}, {NULL, 1}, {NULL, 1},
        {NULL, PACKET_MGR_DEFAULT_PACKET_LEN},  {NULL, 2}, {NULL, 5},
        {NULL, PACKET_MGR_DEFAULT_PACKET_LEN+5}, {NULL,PACKET_MGR_DEFAULT_PACKET_LEN+2}, {NULL, 2},
        {NULL, 3}, {NULL, 1}, {NULL, PACKET_MGR_DEFAULT_PACKET_LEN*2},
        {NULL, PACKET_MGR_DEFAULT_PACKET_LEN+25}, {NULL, PACKET_MGR_DEFAULT_PACKET_LEN}, {NULL, 2},
        {NULL, 1}, {NULL, 4}, {NULL, PACKET_MGR_PACKET_MAXLEN},
        {NULL, 1}, {NULL, 1}, {NULL, 1},
        {NULL, 1}, {NULL, 1}, {NULL, 1}
    };
    /*Keep track of free space to check that we have all the buffers available when nothing is allocated */
    uint16_t starting_free_space = packet_mgr_get_free_space();

    /* Allocate all the buffers with the required sizes from teh allocation set table */
    for (uint32_t i=0; i< sizeof(alloc_set_table)/sizeof(mem_expected_size_pair_t);i++)
    {
        status = packet_mgr_alloc(&alloc_set_table[i].p_test_pkg, alloc_set_table[i].expected_size);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
        /* Check that we are getting the right size every time */
        size = packet_mgr_size_get(alloc_set_table[i].p_test_pkg);
        TEST_ASSERT_TRUE(size >= alloc_set_table[i].expected_size);
    }

    /* release all allocated buffers and then check that we have the expected amount of buffer space available. */
    for (uint32_t i=0; i< sizeof(alloc_set_table)/sizeof(mem_expected_size_pair_t);i++)
    {
        packet_mgr_free(alloc_set_table[i].p_test_pkg);
    }

    TEST_ASSERT_EQUAL(starting_free_space, packet_mgr_get_free_space());

    /* Now that the buffers are fragmented, try to allocate them once more */
    for (uint32_t i=0; i< sizeof(alloc_set_table)/sizeof(mem_expected_size_pair_t);i++)
    {
        status = packet_mgr_alloc(&alloc_set_table[i].p_test_pkg, alloc_set_table[i].expected_size);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
        /* Check that we are getting the right size every time */
        size = packet_mgr_size_get(alloc_set_table[i].p_test_pkg);
        TEST_ASSERT_TRUE(size >= alloc_set_table[i].expected_size);
    }

    /* Release all and check. */
    for (uint32_t i=0; i< sizeof(alloc_set_table)/sizeof(mem_expected_size_pair_t);i++)
    {
        packet_mgr_free(alloc_set_table[i].p_test_pkg);
    }

    TEST_ASSERT_EQUAL(starting_free_space, packet_mgr_get_free_space());
}


/* Tests allocating a buffer too large for the packet manager. */
void test_packet_mgr_alloc_too_large(void)
{
    uint32_t status;
    packet_generic_t * p_test_pkg;

    status = packet_mgr_alloc(&p_test_pkg, PACKET_MGR_PACKET_MAXLEN + 1);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, status);
}

void test_packet_mgr_lots_of_packets(void)
{
    uint32_t status;
    packet_generic_t * p_test_pkg;
    uint32_t test_packet_alloc_size = 42;
    uint16_t starting_free_space = packet_mgr_get_free_space();

    for (uint8_t i = 0; i < 100; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_mgr_alloc(&p_test_pkg, test_packet_alloc_size));
        packet_mgr_free(p_test_pkg);
    }
    TEST_ASSERT_EQUAL(starting_free_space, packet_mgr_get_free_space());

    packet_generic_t * p_test_pkgs[4];
    status = packet_mgr_alloc(&p_test_pkgs[0], test_packet_alloc_size);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    status = packet_mgr_alloc(&p_test_pkgs[1], test_packet_alloc_size);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    status = packet_mgr_alloc(&p_test_pkgs[2], test_packet_alloc_size);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    status = packet_mgr_alloc(&p_test_pkgs[3], test_packet_alloc_size);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    packet_mgr_free(p_test_pkgs[0]);
    packet_mgr_free(p_test_pkgs[1]);
    packet_mgr_free(p_test_pkgs[2]);
    packet_mgr_free(p_test_pkgs[3]);
    TEST_ASSERT_EQUAL(starting_free_space, packet_mgr_get_free_space());

}
