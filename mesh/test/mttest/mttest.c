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
#include <stdbool.h>
#include <time.h>

#include <pthread.h>
#include "nrf_error.h"

#include "mttest.h"

#define LCG_MULTIPLIER 1103515245
#define LCG_INCREMENT  12345

typedef struct
{
    uint32_t           num_threads;
    uint32_t           num_invocations;
    mttest_test_func_t test_func;
    pthread_t *        p_threads;
    uint32_t *         p_rand_states;
    void *             p_context;
} mttest_test_t;

static mttest_test_t * mp_current_test;

static volatile bool m_kill_test = false;
static volatile bool m_test_ready = false;

static pthread_cond_t  m_test_ready_cond;
static pthread_mutex_t m_test_ready_mutex;

static void * test_thread(void * thread_num)
{
    uint32_t thread_id = (uint32_t) thread_num;

    /* Synchronize the start of the tests: */
    pthread_mutex_lock(&m_test_ready_mutex);
    while (!m_test_ready)
    {
        pthread_cond_wait(&m_test_ready_cond, &m_test_ready_mutex);
    }
    pthread_mutex_unlock(&m_test_ready_mutex);

    bool passed = true;
    for (uint32_t invocations = 0; invocations < mp_current_test->num_invocations; ++invocations)
    {
        if (!mp_current_test->test_func(thread_id, invocations, mp_current_test->p_context))
        {
            passed = false;
            m_kill_test = true;
            break;
        }

        if (m_kill_test)
        {
            passed = false;
            break;
        }
    }

    return (void *) !passed;
}

static void free_current_test(void)
{
    if (mp_current_test == NULL)
    {
        return;
    }

    pthread_cond_destroy(&m_test_ready_cond);
    pthread_mutex_destroy(&m_test_ready_mutex);
    free(mp_current_test->p_rand_states);
    free(mp_current_test->p_threads);
    free(mp_current_test);
    mp_current_test = NULL;
}

void mttest_init(void)
{
    /* Set to a specific value to reproduce results: */
    unsigned int seed = time(NULL);
    srand(seed);
}

bool mttest_run(uint32_t num_threads, uint32_t num_invocations, mttest_test_func_t test_func, void * p_context)
{
    if (mp_current_test != NULL)
    {
        return NRF_ERROR_BUSY;
    }

    mp_current_test = malloc(sizeof(mttest_test_t));
    mp_current_test->p_threads = malloc(sizeof(pthread_t) * num_threads);
    mp_current_test->p_rand_states = malloc(sizeof(uint32_t) * num_threads);
    mp_current_test->test_func = test_func;
    mp_current_test->num_invocations = num_invocations;
    mp_current_test->num_threads = num_threads;

    pthread_mutex_init(&m_test_ready_mutex, NULL);
    pthread_cond_init(&m_test_ready_cond, NULL);
    m_test_ready = false;
    m_kill_test = false;

    /* Seed the random number generators: */
    for (uint32_t i = 0; i < num_threads; ++i)
    {
        mp_current_test->p_rand_states[i] = rand();
    }

    /* Start the threads: */
    for (uint32_t i = 0; i < num_threads; ++i)
    {
        int status = pthread_create(&mp_current_test->p_threads[i], NULL, test_thread, (void *) i);
        if (status != 0)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    /* Synchronize the start of the threads to be as close as possible: */
    pthread_mutex_lock(&m_test_ready_mutex);
    m_test_ready = true;
    pthread_cond_broadcast(&m_test_ready_cond);
    pthread_mutex_unlock(&m_test_ready_mutex);

    /* Finish the test and get the results: */
    bool passed = true;
    for (uint32_t i = 0; i < mp_current_test->num_threads; ++i)
    {
        void * result = 0;
        pthread_join(mp_current_test->p_threads[i], &result);
        if (result != 0)
        {
            passed = false;
        }
    }

    free_current_test();

    if (m_kill_test)
    {
        passed = false;
    }

    return passed;
}

uint32_t mttest_random(uint32_t thread_id)
{
    mp_current_test->p_rand_states[thread_id] = mp_current_test->p_rand_states[thread_id] * LCG_MULTIPLIER + LCG_INCREMENT;
    return mp_current_test->p_rand_states[thread_id];
}

void mttest_fail(void)
{
    m_kill_test = true;
}

