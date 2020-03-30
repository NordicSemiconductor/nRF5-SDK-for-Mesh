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

#ifndef MTTEST_H__
#define MTTEST_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Function run in each thread when testing.
 *
 * @retval true  The iteration succeeded, continue.
 * @retval false The test failed, do not continue.
 */
typedef bool (*mttest_test_func_t)(uint32_t thread_id, uint32_t invocation, void * p_context);

/**
 * Initializes the MTTest library.
 */
void mttest_init(void);

/**
 * Runs a multithreaded testcase.
 *
 * @param[in] num_threads     Number of threads to run with.
 * @param[in] num_invocations Number of times to call the test function.
 * @param[in] test_func       Function to run in the threads. This function is run
 *                            in @c num_threads threads simulataneously. Each thread will
 *                            call the function @c num_invocations times.
 * @param[in,out] p_context   Pointer to a context structure to pass to the test function.
 *
 * @retval true               The test passed.
 * @retval false              The test failed.
 */
bool mttest_run(uint32_t num_threads, uint32_t num_invocations, mttest_test_func_t test_func,
        void * p_context);

/**
 * Thread-safely obtains a random number.
 *
 * @param[in] thread_id ID of the calling thread.
 *
 * @returns A random number between 0 and 2^32 - 1.
 */
uint32_t mttest_random(uint32_t thread_id);

/**
 * Fails a testcase.
 *
 * This causes the test to end. Note that a test might not end instantaneously when this function is
 * called, since threads might be busy running the test function.
 *
 */
void mttest_fail(void);

#endif

