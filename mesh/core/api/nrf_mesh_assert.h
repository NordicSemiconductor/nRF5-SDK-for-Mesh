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
#ifndef NRF_MESH_ASSERT_H__
#define NRF_MESH_ASSERT_H__

#include "nrf_mesh.h"
#include "nrf.h"
#include "app_util.h"

/**
 * @defgroup MESH_ASSERT Assert API
 * @ingroup NRF_MESH
 * Allows the framework to propagate errors that can't be recovered from.
 * @{
 */

/* Include compiler specific definitions: */
#if defined(_lint)
    #include "nrf_mesh_assert_lint.h"
#elif defined(__GNUC__)
    #include "nrf_mesh_assert_gcc.h"
#elif defined(__CC_ARM)
    #include "nrf_mesh_assert_armcc.h"
#else
    #error "Your compiler is currently not supported."
#endif

extern void mesh_assertion_handler(uint32_t pc);

/**
 * Run-time assertion.
 * Will trigger the assertion handler if the specified condition evaluates to false.
 * @param[in] cond Condition to evaluate.
 */
#define NRF_MESH_ASSERT(cond)               \
    do                                      \
    {                                       \
        if (!(cond))                        \
        {                                   \
            uint32_t pc;                    \
            GET_PC(pc);                     \
            mesh_assertion_handler(pc);     \
        }                                   \
    } while (0)

/**
 * Run-time assertion for debug use.
 * Debug assertions are only run if the stack is compiled in debug mode.
 */
#ifdef NDEBUG /* The NDEBUG define is added automatically when compiling in release mode. */
    #define NRF_MESH_ASSERT_DEBUG(cond) (void) (cond)
#else
    #define NRF_MESH_ASSERT_DEBUG(cond) NRF_MESH_ASSERT(cond)
#endif

/**
 * Asserts if an error code is not NRF_SUCCESS.
 * @param[in] err Error code to check.
 */
#define NRF_MESH_ERROR_CHECK(err) NRF_MESH_ASSERT(err == NRF_SUCCESS)

/**
 * Compile-time assertion.
 */
#define NRF_MESH_STATIC_ASSERT(...) STATIC_ASSERT(__VA_ARGS__)

/** @} */

#endif /* NRF_MESH_ASSERT_H__ */
