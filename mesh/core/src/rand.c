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
#include <stddef.h>

#include "rand.h"
#include "nrf.h"
#include "nrf_mesh_assert.h"

#if !defined(HOST)
#include <nrf_soc.h>
#else
#include <stdbool.h>
#include <time.h>
#include <stdlib.h>
#endif

/*****************************************************************************
* Local defines
*****************************************************************************/
#define ROT(x,k) (((x)<<(k))|((x)>>(32-(k)))) /** PRNG cyclic leftshift */
#define SMALL_PRNG_BASE_SEED    (0xf1ea5eed)  /** Base seed for PRNG, defined by the author of the generator. */
/*****************************************************************************
* Interface functions
*****************************************************************************/
void rand_prng_seed(prng_t* p_prng)
{
    uint32_t seed = 0;
    /* Get true random seed from HW. */
    rand_hw_rng_get((uint8_t*) &seed, sizeof(seed));

    /* establish base magic numbers */
    p_prng->a = SMALL_PRNG_BASE_SEED;
    p_prng->b = seed;
    p_prng->c = seed;
    p_prng->d = seed;

    /* run the prng a couple of times to flush out the seeds */
    for (uint32_t i = 0; i < 20; ++i)
    {
        (void) rand_prng_get(p_prng);
    }
}

uint32_t rand_prng_get(prng_t* p_prng)
{
    /* Bob Jenkins' small PRNG
        http://burtleburtle.net/bob/rand/smallprng.html */
    uint32_t e = p_prng->a - ROT(p_prng->b, 27);
    p_prng->a = p_prng->b ^ ROT(p_prng->c, 17);
    p_prng->b = p_prng->c + p_prng->d;
    p_prng->c = p_prng->d + e;
    p_prng->d = e + p_prng->a;
    return p_prng->d;
}

#if !defined(HOST)
void rand_hw_rng_get(uint8_t* p_result, uint16_t len)
{
    NRF_MESH_ASSERT(p_result != NULL);
#ifdef SOFTDEVICE_PRESENT
    uint8_t bytes_available;
    uint32_t count = 0;
    while (count < len)
    {
        do
        {
            (void) sd_rand_application_bytes_available_get(&bytes_available);
        } while (bytes_available == 0);

        if (bytes_available > len - count)
        {
            bytes_available = len - count;
        }

        (void) sd_rand_application_vector_get(&p_result[count],
            bytes_available);

        count += bytes_available;
    }
#else
    NRF_RNG->TASKS_START = 1;
    while (len)
    {
        while (!NRF_RNG->EVENTS_VALRDY);
        p_result[--len] = NRF_RNG->VALUE;
        NRF_RNG->EVENTS_VALRDY = 0;
    }
    NRF_RNG->TASKS_STOP = 1;
#endif
}

#else

static bool m_rand_seeded = false;

void rand_hw_rng_get(uint8_t * p_result, uint16_t len)
{
    /* Seed the random number generator the first time this function is called: */
    if (!m_rand_seeded)
    {
        srand(time(NULL));
        m_rand_seeded = true;
    }

    for (uint16_t i = 0; i < len; ++i)
    {
        p_result[i] = rand() & 0xff;
    }
}

#endif
