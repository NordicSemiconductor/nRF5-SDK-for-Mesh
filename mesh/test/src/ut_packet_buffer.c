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

#include <stdio.h>
#include <stdint.h>
#include <unity.h>

#include "utils.h"
#include "nrf_mesh.h"
#include "packet_buffer.h"
#include "test_assert.h"

#define MEM_BLOCK_SIZE 2048
#define DEFAULT_PACKET_LEN 40
#define REPETITION_TEST_MAX_COUNT 100000
static uint8_t memory_block[MEM_BLOCK_SIZE] __attribute__((aligned(WORD_SIZE)));


/* Struct for packet and size pairs. */
typedef struct
{
    packet_buffer_packet_t * p_test_pkg;
    uint16_t expected_size;
}mem_expected_size_pair_t;


/* Test vector1:
 * Test different packet sizes (big ones surrounded by small ones to get defragmentation).
 */
static mem_expected_size_pair_t m_mixed_size_alloc_table[] =
{
    {NULL, 1}, {NULL, 1}, {NULL, 1},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, 2}, {NULL, 5},
    {NULL, DEFAULT_PACKET_LEN+5}, {NULL,DEFAULT_PACKET_LEN+2}, {NULL, 2},
    {NULL, DEFAULT_PACKET_LEN+5}, {NULL,DEFAULT_PACKET_LEN+2}, {NULL, 2},
    {NULL, 3}, {NULL, 1}, {NULL, DEFAULT_PACKET_LEN*2},
    {NULL, DEFAULT_PACKET_LEN+25}, {NULL, DEFAULT_PACKET_LEN*3}, {NULL, 2},
    {NULL, 1}, {NULL, 4}, {NULL, DEFAULT_PACKET_LEN*10},
    {NULL, DEFAULT_PACKET_LEN+5}, {NULL,DEFAULT_PACKET_LEN+2}, {NULL, 2},
    {NULL, 1}, {NULL, 1}, {NULL, 1},
    {NULL, 1}, {NULL, 1}, {NULL, 1}
};

void setUp(void)
{
}

void tearDown(void)
{
}


/* Tests the failure paths, most of which are assertions. */
void test_packet_buffer_derp(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;

    /* ASSERT: 0 len */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_init(&my_pacman, memory_block, 0));

    /* ASSERT: Not enough memory */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_init(&my_pacman, memory_block, sizeof(packet_buffer_packet_t)));

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_init(&my_pacman, NULL, DEFAULT_PACKET_LEN));

    /* ASSERT: NULL pacman */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_init(NULL, memory_block, DEFAULT_PACKET_LEN));

    packet_buffer_init(&my_pacman, memory_block, sizeof(packet_buffer_packet_t) + DEFAULT_PACKET_LEN);
    uint16_t pac_max_len = packet_buffer_max_packet_len_get(&my_pacman);

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_reserve(NULL, &p_my_packet, DEFAULT_PACKET_LEN));

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_reserve(&my_pacman, NULL, DEFAULT_PACKET_LEN));

    /* Can't allocate larger than the max possible len */
    uint32_t status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len+1);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, status);

    /* Can't allocate 0 length */
    status = packet_buffer_reserve(&my_pacman, &p_my_packet, 0);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, status);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, DEFAULT_PACKET_LEN);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(NULL, p_my_packet, 0));
    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(&my_pacman, NULL, 0));

    /* ASSERT: commit len larger than reserved len */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(&my_pacman, p_my_packet, p_my_packet->size+1));
    /* ASSERT: commit len can't be less than 1 */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(&my_pacman, p_my_packet, 0));

    /* Nothing to pop */
    status = packet_buffer_pop(&my_pacman, &p_my_packet);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, status);

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_pop(NULL, &p_my_packet));
    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_pop(&my_pacman, NULL));

    packet_buffer_commit(&my_pacman, p_my_packet, p_my_packet->size);
    TEST_ASSERT_EQUAL(DEFAULT_PACKET_LEN, p_my_packet->size);

    /* ASSERT: Not reserved packet */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(&my_pacman, p_my_packet, 0));

    /* ASSERT: Cannot free a committed packet */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_free(&my_pacman, p_my_packet));

    status = packet_buffer_pop(&my_pacman, &p_my_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    /* ASSERT: Cannot pop a packet twice */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_pop(&my_pacman, &p_my_packet));

    /* ASSERT: Can't commit a popped packet */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_commit(&my_pacman, p_my_packet, 0));

    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_free(NULL, p_my_packet));
    /* ASSERT: NULL mem */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_free(&my_pacman, NULL));

    packet_buffer_free(&my_pacman, p_my_packet);

    /* ASSERT: Cannot free a packet twice */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_free(&my_pacman, p_my_packet));
}

/* Tests the basic functionality of the packet buffer module.
 * Allocates the largest buffer possible, releases and allocates again.
 */
void test_packet_buffer_basic(void)
{

    const uint32_t packet_header_size = sizeof(packet_buffer_packet_t);
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    uint16_t pac_max_len = packet_buffer_max_packet_len_get(&my_pacman);

    TEST_ASSERT_EQUAL(MEM_BLOCK_SIZE-packet_header_size, pac_max_len);

    uint32_t status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    TEST_ASSERT_EQUAL(pac_max_len, p_my_packet->size);

    packet_buffer_commit(&my_pacman, p_my_packet, pac_max_len);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, 1);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);
    p_my_packet = NULL;

    status = packet_buffer_pop(&my_pacman, &p_my_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    TEST_ASSERT(NULL != p_my_packet);
    TEST_ASSERT_EQUAL(p_my_packet->size, pac_max_len);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, 1);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);

    packet_buffer_free(&my_pacman, p_my_packet);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    packet_buffer_free(&my_pacman, p_my_packet);
}


/* Tests reserving, committing and popping a single packet buffer only. */
void test_packet_buffer_single_packet(void)
{
    packet_buffer_t pacman;
    packet_buffer_packet_t * p_packet;

    packet_buffer_init(&pacman, memory_block, MEM_BLOCK_SIZE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&pacman, &p_packet, sizeof(packet_t)));
    packet_buffer_commit(&pacman, p_packet, sizeof(packet_t));

    packet_buffer_packet_t * p_popped_packet;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&pacman, &p_popped_packet));
    TEST_ASSERT_EQUAL_PTR(p_packet, p_popped_packet);

    packet_buffer_free(&pacman, p_popped_packet);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&pacman, &p_popped_packet));
}


/* Tests that when a commit reduces a reserved size after the head had wrapped around
 * The wrap-around is handled correctly.
 */
void test_packet_buffer_resize_after_head_wraparound(void)
{

    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    uint16_t pac_max_len = packet_buffer_max_packet_len_get(&my_pacman);


    uint32_t status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    TEST_ASSERT_EQUAL(pac_max_len, p_my_packet->size);

    /* Commit with length reduced to just enough space to be able to initialize
     * another 1 word packet
     */
    packet_buffer_commit(&my_pacman, p_my_packet, pac_max_len - sizeof(packet_buffer_packet_t) - 4);
    /* Check that the packet is resized */
    TEST_ASSERT_EQUAL(pac_max_len - sizeof(packet_buffer_packet_t) - 4, p_my_packet->size);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, 4);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);

    /* Re-initialize the pacman to start clean*/
    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
    TEST_ASSERT_EQUAL(pac_max_len, p_my_packet->size);
}
/* Tests allocating as much as possible with the given test vector.
 * then freeing it all, and then allocating the maximum possible packet size.
 * This is repeated many times.
 */
void test_packet_buffer_alloc_all_free_all(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;
    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);
    uint16_t pac_max_len = packet_buffer_max_packet_len_get(&my_pacman);
    uint32_t status;

     /*Allocate as many of the buffers with the required sizes from the allocation set table
        until there is no more room in the pacman.
        Do this for many iterations to check for possible memory leaks due to incorrect
        calculations in the pacman*/
    uint32_t table_size = sizeof(m_mixed_size_alloc_table)/sizeof(mem_expected_size_pair_t);
    for (uint32_t repeat_count=0; repeat_count < REPETITION_TEST_MAX_COUNT; repeat_count++)
    {
        uint32_t i=0;
        do
        {
            status = packet_buffer_reserve(&my_pacman, &m_mixed_size_alloc_table[i].p_test_pkg, m_mixed_size_alloc_table[i].expected_size);
            if (NRF_SUCCESS == status)
            {
                TEST_ASSERT_TRUE(m_mixed_size_alloc_table[i].p_test_pkg->size >= m_mixed_size_alloc_table[i].expected_size);
                /* The buffer should never be longer than what's required to fit a word-aligned header after it : */
                TEST_ASSERT_TRUE(m_mixed_size_alloc_table[i].p_test_pkg->size <= ALIGN_VAL(m_mixed_size_alloc_table[i].expected_size, WORD_SIZE) + sizeof(packet_buffer_packet_t));
                packet_buffer_commit(&my_pacman, m_mixed_size_alloc_table[i].p_test_pkg, m_mixed_size_alloc_table[i].expected_size);
            }
            i = (i+1)%table_size;
        } while (NRF_SUCCESS == status);
        TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);

        /* Pop and free */
        do
        {
            status = packet_buffer_pop(&my_pacman, &p_my_packet);
            if (NRF_SUCCESS == status)
            {
                packet_buffer_free(&my_pacman, p_my_packet);
            }
        } while (NRF_SUCCESS == status);
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, status);

        /* Now that we have freed all, it should be possible to allocate max size. */
        status = packet_buffer_reserve(&my_pacman, &p_my_packet, pac_max_len);
        if (NRF_SUCCESS != status)
        {
            /* Something went wrong! Print iteration number */
             printf("Max reserve failed with error code: %lu, on iteration: %lu, pacman head: %lu, pacman tail: %lu\n", status, repeat_count, my_pacman.head, my_pacman.tail);
            TEST_ASSERT(false);
        }
        TEST_ASSERT_EQUAL(pac_max_len, p_my_packet->size);
        packet_buffer_free(&my_pacman, p_my_packet);
    }
}


/* Tests allocating as much as possible with the given test vector.
 * then freeing only some, and then allocating more from the test vector.
 * So tail and head never catch up.
 * This is repeated many times.
 */
void test_packet_buffer_alloc_all_free_some(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;
    uint32_t status;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

     /*Allocate as many of the buffers with the required sizes from the allocation set table
        until there is no more room in the pacman.
        Do this for many iterations to check for possible memory leaks due to incorrect
        calculations in the pacman*/
    for (uint32_t repeat_count=0; repeat_count < REPETITION_TEST_MAX_COUNT; repeat_count++)
    {
        uint32_t i=0;
        do
        {
            status = packet_buffer_reserve(&my_pacman, &m_mixed_size_alloc_table[i].p_test_pkg, m_mixed_size_alloc_table[i].expected_size);
            if (NRF_SUCCESS == status)
            {
                TEST_ASSERT_TRUE(m_mixed_size_alloc_table[i].p_test_pkg->size >= m_mixed_size_alloc_table[i].expected_size);
                /* The buffer should never be longer than what's required to fit a word-aligned header after it : */
                TEST_ASSERT_TRUE(m_mixed_size_alloc_table[i].p_test_pkg->size <= ALIGN_VAL(m_mixed_size_alloc_table[i].expected_size, WORD_SIZE) + sizeof(packet_buffer_packet_t));
                packet_buffer_commit(&my_pacman, m_mixed_size_alloc_table[i].p_test_pkg, m_mixed_size_alloc_table[i].expected_size);
            }
            i = (i+1)%ARRAY_SIZE(m_mixed_size_alloc_table);
        } while (NRF_SUCCESS == status);
        TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);
        /* Pop and free */
        for (uint32_t j=0; j < ARRAY_SIZE(m_mixed_size_alloc_table)/2; j++)
        {
            status = packet_buffer_pop(&my_pacman, &p_my_packet);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
            packet_buffer_free(&my_pacman, p_my_packet);
        }
    }
}

/* Tests that a second allocation causes an assert.  */
void test_packet_buffer_double_alloc(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;
    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_my_packet, 256));

    /* Should assert on second alloc */
    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_reserve(&my_pacman, &p_my_packet, 256));
}

/* Test edge case where head == tail and there's not enough space left in the buffer to fit the
 * new packet.
 */
void test_packet_buffer_empty_near_end(void)
{
    const uint32_t packet_header_size = sizeof(packet_buffer_packet_t);
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    const uint16_t remaining_size = packet_header_size + 4;

    struct
    {
        uint16_t size;
        uint16_t expected_offset; /**< Expected offset of the returned packet */
        bool succeed;
    } vectors[] = {
        {remaining_size, 0, true}, /* will not fit because of the header, must wrap around. */
        {remaining_size - packet_header_size, MEM_BLOCK_SIZE - remaining_size, true}, /* will fit perfectly */
        {remaining_size + 4, 0, true}, /* too big, must wrap around */
        {MEM_BLOCK_SIZE - (packet_header_size * 2) - remaining_size, 0, true}, /* Should fit after wrap around */
        {MEM_BLOCK_SIZE - packet_header_size, 0, false} /* Won't fit, as there's no room before the tail */
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(vectors); i++)
    {
        char fail_msg[128];
        sprintf(fail_msg, "Failed at iteration %d, size is %d", i, vectors[i].size);
        my_pacman.head = MEM_BLOCK_SIZE - remaining_size;
        my_pacman.tail = MEM_BLOCK_SIZE - remaining_size;
        packet_buffer_packet_t * p_current_head = ((packet_buffer_packet_t *)&memory_block[my_pacman.head]);
        p_current_head->packet_state = PACKET_BUFFER_MEM_STATE_FREE;

        uint32_t status = packet_buffer_reserve(&my_pacman, &p_my_packet, vectors[i].size);

        /* Should reset head and tail to 0 regardless: */
        TEST_ASSERT_EQUAL_MESSAGE(0, (uint8_t *)p_my_packet - memory_block, fail_msg);
        TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, status, fail_msg);
        packet_buffer_commit(&my_pacman, p_my_packet, vectors[i].size);
    }

    /* Test the same vector, but now, have one small packet committed first, to prevent resetting the buffer. */
    for (uint32_t i = 0; i < ARRAY_SIZE(vectors); i++)
    {
        char fail_msg[128];
        sprintf(fail_msg, "Failed at iteration %d, size is %d", i, vectors[i].size);
        my_pacman.head = MEM_BLOCK_SIZE - remaining_size;
        my_pacman.tail = MEM_BLOCK_SIZE - remaining_size - packet_header_size;
        packet_buffer_packet_t * p_current_head = ((packet_buffer_packet_t *)&memory_block[my_pacman.head]);
        p_current_head->packet_state = PACKET_BUFFER_MEM_STATE_FREE;

        uint32_t status = packet_buffer_reserve(&my_pacman, &p_my_packet, vectors[i].size);

        if (vectors[i].succeed)
        {
            TEST_ASSERT_EQUAL_MESSAGE(vectors[i].expected_offset, (uint8_t *)p_my_packet - memory_block, fail_msg);
            TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, status, fail_msg);
            packet_buffer_commit(&my_pacman, p_my_packet, vectors[i].size);
        }
        else
        {
            TEST_ASSERT_EQUAL_MESSAGE(NRF_ERROR_NO_MEM, status, fail_msg);
        }
    }
}

void test_packet_buffer_flush(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    uint8_t lengths[] = {8, 4, 13, 1, 12};

    /***** Empty buffer *****/
    packet_buffer_flush(&my_pacman);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));

    /***** No popped, no reserved packets *****/
    for (uint32_t i = 0; i < sizeof(lengths); ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, lengths[i]));
        packet_buffer_commit(&my_pacman, p_packet, lengths[i]);
    }

    packet_buffer_flush(&my_pacman);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));

    /* No popped packets, one reserved packet */
    for (uint32_t i = 0; i < sizeof(lengths); ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, lengths[i]));
        packet_buffer_commit(&my_pacman, p_packet, lengths[i]);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, 10));

    TEST_NRF_MESH_ASSERT_EXPECT(packet_buffer_flush(&my_pacman));

    /* reset: */
    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);
    /***** one popped packet, no reserved packets *****/
    for (uint32_t i = 0; i < sizeof(lengths); ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, lengths[i]));
        packet_buffer_commit(&my_pacman, p_packet, lengths[i]);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_packet));

    packet_buffer_flush(&my_pacman);

    packet_buffer_free(&my_pacman, p_packet);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));
    /* Verify that we can reserve+free again */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, 10));
    packet_buffer_free(&my_pacman, p_packet);
}

/**
 * Testing max packet size lockup regression.
 *
 * An empty buffer should always be able to allocate a max-len packet.
 */
void test_maxlen_alloc_lockup(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    uint32_t maxlen = packet_buffer_max_packet_len_get(&my_pacman);

    /* maxlen alloc on untouched buffer */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, maxlen));
    packet_buffer_free(&my_pacman, p_packet);


    /* Previous regression: if the head and tail of an empty buffer is somewhere other than at 0,
     * the buffer would fail allocation of max-length packets, even though there's technically
     * space. */

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, maxlen / 2));
    packet_buffer_commit(&my_pacman, p_packet, p_packet->size);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_packet));
    packet_buffer_free(&my_pacman, p_packet);

    /* Head and tail should now be in the middle of the buffer, but the buffer is empty. Should still succeed alloc of maxlen */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, maxlen));

}

void test_ready_to_pop(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_reserve_packet;
    packet_buffer_packet_t * p_pop_packet;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    /* Empty buffer */
    TEST_ASSERT_FALSE(packet_buffer_packets_ready_to_pop(&my_pacman));

    /* Sending packets A, B and C through the buffer, only expect ready_to_pop being true when
     * there's at least one committed packet somewhere in the buffer. */

    /* Push packet A */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, 10));
    packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);
    /* Push packet B */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, 10));
    packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);

    /* Packet A in committed state */
    TEST_ASSERT_TRUE(packet_buffer_packets_ready_to_pop(&my_pacman));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_pop_packet));
    /* Packet A in popped state, Packet B in committed state */
    TEST_ASSERT_TRUE(packet_buffer_packets_ready_to_pop(&my_pacman));

    /* Free packet A, packet B still in committed state */
    packet_buffer_free(&my_pacman, p_pop_packet);
    TEST_ASSERT_TRUE(packet_buffer_packets_ready_to_pop(&my_pacman));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_pop_packet));
    /* Packet B in popped state, No more packets allocated. */
    TEST_ASSERT_FALSE(packet_buffer_packets_ready_to_pop(&my_pacman));

    /* Reserve packet C */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, 10));
    /* Packet B in popped state, Packet C in reserved */
    TEST_ASSERT_FALSE(packet_buffer_packets_ready_to_pop(&my_pacman));

    /* Free packet B */
    packet_buffer_free(&my_pacman, p_pop_packet);

    /* Packet C still in reserved state */
    TEST_ASSERT_FALSE(packet_buffer_packets_ready_to_pop(&my_pacman));
}


void test_buffer_pop_padding(void)
{
    uint8_t data[256] __attribute__((aligned(4)));
    packet_buffer_packet_t * p_buf;

    p_buf = (packet_buffer_packet_t *) &data[0];
    p_buf->size = 0x30;
    p_buf->packet_state = PACKET_BUFFER_MEM_STATE_COMMITTED;

    p_buf = (packet_buffer_packet_t *) &data[208];
    p_buf->size = (sizeof(data) - 208) - sizeof(packet_buffer_packet_t);
    p_buf->packet_state = PACKET_BUFFER_MEM_STATE_PADDING;

    packet_buffer_t buf;
    /* Point to the last padding packet with both head and tail */
    buf.head = 208;
    buf.tail = 208;
    buf.size = 256;
    buf.buffer = data;

    /* The packet buffer is full of committed packets, it should be possible to pop */
    TEST_ASSERT_TRUE(packet_buffer_can_pop(&buf));
    TEST_ASSERT_TRUE(packet_buffer_packets_ready_to_pop(&buf));

    p_buf = NULL;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&buf, &p_buf));
    TEST_ASSERT_EQUAL(&data[0], p_buf); // should have popped the packet at the beginning of the buffer, skipping the padding.
}

void test_is_empty(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_reserve_packet;
    packet_buffer_packet_t * p_pop_packet;
    const uint32_t packet_size = 16;

    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

    // Should only be empty when there are no packets in the pipeline:
    TEST_ASSERT_TRUE(packet_buffer_is_empty(&my_pacman));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, packet_size));
    TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));

    packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);
    TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_pop_packet));
    TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));

    packet_buffer_free(&my_pacman, p_pop_packet);
    TEST_ASSERT_TRUE(packet_buffer_is_empty(&my_pacman));

    // Verify that padding doesn't cause false positives - First verify that we've chosen a packet size that generates padding:
    TEST_ASSERT_NOT_EQUAL(0, ALIGN_VAL(sizeof(packet_buffer_packet_t) + packet_size, WORD_SIZE) % MEM_BLOCK_SIZE);

    // Setup the queue to actually have some padding:
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, packet_size));
    packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, packet_size));
    packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_pop_packet));
    packet_buffer_free(&my_pacman, p_pop_packet);

    do
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserve_packet, packet_size));
        packet_buffer_commit(&my_pacman, p_reserve_packet, p_reserve_packet->size);
        TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));
    } while (p_reserve_packet != (packet_buffer_packet_t *) &memory_block[0]);

    // The packet reserve is now at the front of the buffer.

    // Since we freed one packet at the beginning, we should now be filling the buffer completely:
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, packet_buffer_reserve(&my_pacman, &p_reserve_packet, packet_size));

    // Spin until we catch up with the head:

    while (packet_buffer_pop(&my_pacman, &p_pop_packet) != NRF_ERROR_NOT_FOUND)
    {
        // Should be non-empty all the way:
        TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));
        packet_buffer_free(&my_pacman, p_pop_packet);

        //
        if (p_pop_packet != p_reserve_packet)
        {
            TEST_ASSERT_FALSE(packet_buffer_is_empty(&my_pacman));
        }
    }

    // Got a NOT_FOUND, buffer is now empty:
    TEST_ASSERT_TRUE(packet_buffer_is_empty(&my_pacman));
}
