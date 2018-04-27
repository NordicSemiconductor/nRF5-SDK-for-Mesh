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

#include <stdio.h>
#include <stdint.h>
#include <unity.h>

#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "packet_buffer.h"

nrf_mesh_assertion_handler_t m_assertion_handler;

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

/* Test vector2:
 * Test identical packet sizes.
 */
static mem_expected_size_pair_t m_uniform_size_alloc_table[] =
{
    {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN},
    {NULL, DEFAULT_PACKET_LEN},  {NULL, DEFAULT_PACKET_LEN}, {NULL, DEFAULT_PACKET_LEN}
};

/* Assertion handler, automatically fails the test. */
void nrf_mesh_assertion_handler(uint32_t pc)
{
    char assert_message[50];
    sprintf(assert_message, "Mesh assertion triggered: PC:%u,", pc);
    TEST_FAIL_MESSAGE(assert_message);
}

void setUp(void)
{
    m_assertion_handler = nrf_mesh_assertion_handler;
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

    /* Commit with length reduced to a size that does not leave enough space
     * for the initialization of another packet
     */
    packet_buffer_commit(&my_pacman, p_my_packet, pac_max_len - sizeof(packet_buffer_packet_t));
    /* Check that the packet is NOT resized */
    TEST_ASSERT_EQUAL(pac_max_len, p_my_packet->size);

    status = packet_buffer_reserve(&my_pacman, &p_my_packet, 1);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, status);

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
        for (uint32_t j=0; j < table_size/2; j++)
        {
            status = packet_buffer_pop(&my_pacman, &p_my_packet);
            TEST_ASSERT_EQUAL(NRF_SUCCESS, status);
            packet_buffer_free(&my_pacman, p_my_packet);
        }
    }
}

/* Tests cases that cause a packet buffer to be put in SKIPPED state */
void test_packet_buffer_skip(void)
{
    packet_buffer_t my_pacman;
    packet_buffer_packet_t * p_my_packet;
    uint32_t status;
    packet_buffer_init(&my_pacman, memory_block, MEM_BLOCK_SIZE);

     /*Allocate as many of the buffers with the required sizes from the allocation set table
        until there is no more room in the pacman.
        Do this for many iterations to check for possible memory leaks due to incorrect
        calculations in the pacman*/
    uint32_t table_size = sizeof(m_uniform_size_alloc_table)/sizeof(mem_expected_size_pair_t);
    for (uint32_t repeat_count=0; repeat_count < REPETITION_TEST_MAX_COUNT; repeat_count++)
    {
        uint32_t i=0;
        /* First reserve as much as possible*/
        do
        {
            status = packet_buffer_reserve(&my_pacman, &m_uniform_size_alloc_table[i].p_test_pkg, m_uniform_size_alloc_table[i].expected_size);
            i++;
        } while (NRF_SUCCESS == status && i < table_size);

        /* Free all reserved except last one. By the end pacman should look like:
         * | SKIPPED | SKIPPED | .... | SKIPPED | RESERVED |
         */
        for (uint32_t j=0; j < i-1; j++)
        {
            packet_buffer_free(&my_pacman, m_uniform_size_alloc_table[j].p_test_pkg);
        }
        /* Commit the last one, with a smaller than reserved size; everything should be OK*/
        packet_buffer_commit(&my_pacman, m_uniform_size_alloc_table[i-1].p_test_pkg, m_uniform_size_alloc_table[i-1].expected_size-1);
        uint32_t no_popped = 0;
        /* Pop and free */
        do
        {
            status = packet_buffer_pop(&my_pacman, &p_my_packet);
            if (NRF_SUCCESS == status)
            {
                packet_buffer_free(&my_pacman, p_my_packet);
                no_popped++;
            }
        } while (NRF_SUCCESS == status);
        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, status);
        TEST_ASSERT_EQUAL(1, no_popped);
    }
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
         {MEM_BLOCK_SIZE - packet_header_size - remaining_size, 0, true}, /* Should fit after wrap around */
         {MEM_BLOCK_SIZE - packet_header_size, 0, false} /* Won't fit, as the end of the buffer is marked as skipped */
        };
    for (uint32_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++)
    {
        char fail_msg[128];
        sprintf(fail_msg, "Failed at iteration %d, size is %d", i, vectors[i].size);
        my_pacman.head = MEM_BLOCK_SIZE - remaining_size;
        my_pacman.tail = MEM_BLOCK_SIZE - remaining_size;
        packet_buffer_packet_t * p_current_head = ((packet_buffer_packet_t *)&memory_block[my_pacman.head]);
        p_current_head->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
#if PACKET_BUFFER_DEBUG_MODE
        /* fake the seal, to give the impression that we got here in a normal fashion. */
        p_current_head->seal = 0x5ea15ea1;
#endif
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

    packet_buffer_flush(&my_pacman);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));
    packet_buffer_commit(&my_pacman, p_packet, 10);
    /* pop the reserved packet */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_packet));
    packet_buffer_free(&my_pacman, p_packet);
    /* Now there's nothing again */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));


    /***** No popped packets, one reserved packet *****/
    for (uint32_t i = 0; i < sizeof(lengths); ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, lengths[i]));
        packet_buffer_commit(&my_pacman, p_packet, lengths[i]);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, 10));

    packet_buffer_flush(&my_pacman);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));
    /* Free the reserved packet again */
    packet_buffer_free(&my_pacman, p_packet);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));

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

    /***** one popped packet, multiple reserved packets *****/
    for (uint32_t i = 0; i < sizeof(lengths); ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, lengths[i]));
        packet_buffer_commit(&my_pacman, p_packet, lengths[i]);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_pop(&my_pacman, &p_packet));
    packet_buffer_packet_t * p_reserved_packets[2];
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserved_packets[0], 10));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_reserved_packets[1], 10));

    packet_buffer_flush(&my_pacman);
    /* Free the popped packet */
    packet_buffer_free(&my_pacman, p_packet);
    /* No more committed packets in the queue */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));
    /* Commit the reserved packets */
    packet_buffer_commit(&my_pacman, p_reserved_packets[0], 10);
    packet_buffer_commit(&my_pacman, p_reserved_packets[1], 10);
    /* Flush them again */
    packet_buffer_flush(&my_pacman);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, packet_buffer_pop(&my_pacman, &p_packet));

    /* Verify that we can reserve+free again */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, packet_buffer_reserve(&my_pacman, &p_packet, 10));
    packet_buffer_free(&my_pacman, p_packet);
}