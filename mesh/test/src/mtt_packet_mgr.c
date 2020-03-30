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

#include <stdlib.h>
#include <stdio.h>
#include <mttest.h>

#include <pthread.h>

#include "nrf_mesh_defines.h"
#include "packet_mgr.h"
#include "toolchain.h"

/* Number of threads to run simulatenously: */
#define TEST_NUM_THREADS    8
/* Number of iterations to run of each thread kernel: */
#define TEST_NUM_ITERATIONS 2000
/* Number of simultaneous allocations to make in each thread kernel in the simultaneous allocation test: */
#define TEST_NUM_SIMALLOC   32

#define TEST_RANDSIZE_MAX   NRF_MESH_SEG_PAYLOAD_SIZE_MAX

uint32_t        m_nomem_counter;
pthread_mutex_t m_nomem_counter_mutex;

void mesh_assertion_handler(uint32_t pc)
{
    __LOG(LOG_SRC_TEST, LOG_LEVEL_ERROR, "Assertion at PC = %.08x\n", pc);
    mttest_fail();
}

bool testloop_fixed_size(uint32_t thread_id, uint32_t invocation, void * p_context)
{
    packet_t * p_packet = NULL;
    uint32_t status = packet_mgr_alloc((packet_generic_t **) &p_packet, sizeof(packet_t));
    if (status != NRF_SUCCESS)
    {
        if (status == NRF_ERROR_NO_MEM)
        {
            /* To be out of memory isn't really a problem, log it and return: */
            pthread_mutex_lock(&m_nomem_counter_mutex);
            ++m_nomem_counter;
            pthread_mutex_unlock(&m_nomem_counter_mutex);

            printf("Test failure: packet_mgr_alloc() failed with error code %d in thread %d, iteration %d for size %d\n",
                    status, thread_id, invocation, sizeof(packet_t));
            return true;
        }
        else
        {
            return false;
        }
    }

    /* Fill the packet with random data: */
    for (unsigned int i = 0; i < sizeof(packet_t); ++i)
    {
        ((uint8_t *) p_packet)[i] = mttest_random(thread_id) & 0xff;
    }

    packet_mgr_free(p_packet);

    return true;
}

bool testloop_rand_size(uint32_t thread_id, uint32_t invocation, void * p_context)
{
    uint8_t * p_buffer;
    uint16_t bufsize = (mttest_random(thread_id) % (TEST_RANDSIZE_MAX - 1) + 1);

    uint32_t status = packet_mgr_alloc((packet_generic_t **) &p_buffer, bufsize);
    if (status != NRF_SUCCESS)
    {
        if (status == NRF_ERROR_NO_MEM)
        {
            pthread_mutex_lock(&m_nomem_counter_mutex);
            ++m_nomem_counter;
            pthread_mutex_unlock(&m_nomem_counter_mutex);

            return true;
        }
        else
        {
            printf("Test failure: packet_mgr_alloc() failed with error code %d in thread %d, iteration %d for size %d\n",
                    status, thread_id, invocation, bufsize);
            return false;
        }
    }

    /* Fill the packet with random data: */
    for (unsigned int i = 0; i < bufsize; ++i)
    {
        ((uint8_t *) p_buffer)[i] = mttest_random(thread_id) & 0xff;
    }

    packet_mgr_free(p_buffer);

    return true;
}

bool testloop_simalloc_fixed(uint32_t thread_id, uint32_t invocation, void * p_context)
{
    packet_t * p_buffers[TEST_NUM_SIMALLOC] = {};
    for (uint32_t i = 0; i < TEST_NUM_SIMALLOC; ++i)
    {
        uint32_t status = packet_mgr_alloc((packet_generic_t **) &p_buffers[i], sizeof(packet_t));
        if (status != NRF_SUCCESS)
        {
            if (status == NRF_ERROR_NO_MEM)
            {
                pthread_mutex_lock(&m_nomem_counter_mutex);
                ++m_nomem_counter;
                pthread_mutex_unlock(&m_nomem_counter_mutex);

                p_buffers[i] = NULL;
            }
            else
            {
                printf("Test failure: packet_mgr_alloc() failed with error code %d in thread %d, iteration %d, packet %d\n",
                        status, thread_id, invocation, i);
                return false;
            }
        }
    }

    /* Fill the packets with random data: */
    for (uint32_t i = 0; i < TEST_NUM_SIMALLOC; ++i)
    {
        for (unsigned int j = 0; p_buffers[i] != NULL && j < sizeof(packet_t); ++j)
        {
            ((uint8_t *) (p_buffers[i]))[j] = mttest_random(thread_id) & 0xff;
        }
    }

    for (uint32_t i = 0; i < TEST_NUM_SIMALLOC; ++i)
    {
        if (p_buffers[i] != NULL)
        {
            packet_mgr_free(p_buffers[i]);
        }
    }

    return true;

}

int main(void)
{
    int retval = 0;

    /* Initialize the logging module so we can know what is happening: */
    __LOG_INIT(LOG_SRC_TEST | LOG_SRC_PACMAN, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    /* Initialize the toolchain module, which provides the global IRQ lock: */
    toolchain_init_irqs();

    /* Initialize the packet manager: */
    nrf_mesh_init_params_t init_params = {};
    packet_mgr_init(&init_params);

    /* Initialize the test framework: */
    mttest_init();

    /* Reset the nomem counter: */
    pthread_mutex_init(&m_nomem_counter_mutex, NULL);
    m_nomem_counter = 0;

    /* Run the packet manager test: */
    bool result = mttest_run(TEST_NUM_THREADS, TEST_NUM_ITERATIONS, testloop_fixed_size, NULL);
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO,
            "Packet allocation test with %d threads with %d iterations of fixed size allocations (size = %d) %s with %.02f %% nomem errors.\n",
            TEST_NUM_THREADS, TEST_NUM_ITERATIONS, sizeof(packet_t), result ? "passed" : "failed",
            ((double) m_nomem_counter / (double) (TEST_NUM_THREADS * TEST_NUM_ITERATIONS)) * 100.0);
    if (!result)
    {
        retval++;
    }

    m_nomem_counter = 0;
    result = mttest_run(TEST_NUM_THREADS, TEST_NUM_ITERATIONS, testloop_rand_size, NULL);
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO,
            "Packet allocation test with %d threads with %d iterations of random sized allocations (size =< %d) %s with %.02f %% nomem errors.\n",
            TEST_NUM_THREADS, TEST_NUM_ITERATIONS, TEST_RANDSIZE_MAX, result ? "passed" : "failed",
            ((double) m_nomem_counter / (double) (TEST_NUM_THREADS * TEST_NUM_ITERATIONS)) * 100.0);
    if (!result)
    {
        retval++;
    }

    m_nomem_counter = 0;
    result = mttest_run(TEST_NUM_THREADS, TEST_NUM_ITERATIONS, testloop_simalloc_fixed, NULL);
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO,
            "Packet allocation test with %d threads with %d iterations of %d fixed size allocations (size = %d) %s with %.02f %% nomem errors.\n",
            TEST_NUM_THREADS, TEST_NUM_ITERATIONS, TEST_NUM_SIMALLOC, sizeof(packet_t), result ? "passed" : "failed",
            ((double) m_nomem_counter / (double) (TEST_NUM_THREADS * TEST_NUM_ITERATIONS * TEST_NUM_SIMALLOC)) * 100.0);
    if (!result)
    {
        retval++;
    }

    return retval;
}
