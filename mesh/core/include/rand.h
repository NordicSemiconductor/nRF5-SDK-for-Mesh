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
#ifndef MESH_RAND_H__
#define MESH_RAND_H__

#include <stdint.h>

/**
 * @defgroup RAND Random number generator
 * @ingroup MESH_CORE
 * This module abstracts the hardware RNG, as well as providing a simple PRNG for non-cryptographic
 * random numbers.
 * @{
 */

/** PRNG instance structure. */
typedef struct
{
    uint32_t a; /**< PRNG state variable A */
    uint32_t b; /**< PRNG state variable B */
    uint32_t c; /**< PRNG state variable C */
    uint32_t d; /**< PRNG state variable D */
} prng_t;

/**
 * Seeds a PRNG-instance with random values from the hardware RNG module.
 *
 * @warning The PRNG structure is not seeded by default! All new instances should be seeded
 *           before use. The probability of cycles shorter than 2^64 are assumed neglectable, but
 *           it is still advisable to reseed the PRNG with regular intervals.
 *
 * @param[in,out] p_prng A pointer to a prng_t structure to be seeded.
 */
void rand_prng_seed(prng_t* p_prng);

/**
 * Gets the next 32-bit pseudo-random value from a PRNG instance.
 *
 * @warning The PRNG must NEVER be used for cryptographic purposes, as it is considered unsafe.
 *
 * @param[in,out] p_prng The PRNG instance to get a value from.
 *
 * @return A 32-bit pseudo-random value.
 */
uint32_t rand_prng_get(prng_t* p_prng);

/**
 * Gets a variable length array of random bytes from the hardware RNG module.
 *
 * @param[out] p_result An array to copy the random values into.
 * @param[in]  len      The size of the @c p_result array.
 */
void rand_hw_rng_get(uint8_t* p_result, uint16_t len);

/** @} */

#endif /* MESH_RAND_H__ */
