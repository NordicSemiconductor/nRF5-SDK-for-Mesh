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

#ifndef NRF_MESH_ASSERT_GCC_H__
#define NRF_MESH_ASSERT_GCC_H__

/**
 * @ingroup MESH_ASSERT
 * @{
 */

/* Sanity checks: */
#ifndef NRF_MESH_ASSERT_H__
    #error "This file should only be included by nrf_mesh_assert.h, not directly."
#endif

#ifndef __GNUC__
    #error "This file should only be included when compiling with GCC."
#endif

#ifdef HOST

    /** Produces a hardfault. */
    #define HARD_FAULT() __builtin_trap()

    /**
     * Gets the current value of the program counter.
     * @param[out] pc Variable to assign the obtained value to.
     */
    #define GET_PC(pc) pc = 0xffffffff /* This value is not useful on host, use a debugger. */

#else

    /** Produces a hardfault. */
    #define HARD_FAULT()  __asm__ volatile(".inst.n 0xde00\n")

    /**
     * Gets the current value of the program counter.
     * @param[out] pc Variable to assign the obtained value to.
     */
    #define GET_PC(pc)    __asm__ volatile("mov %0, pc\n\t" : "=r" (pc))

#endif

/** @} */

#endif

