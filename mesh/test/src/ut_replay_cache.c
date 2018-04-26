/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#include <string.h>

#include <unity.h>

#include <nrf_error.h>

#include "replay_cache.h"
#include "nrf_mesh_config_core.h"

#define SRC_BASE   0x0100
#define SEQNO_BASE 0x0000
#define IVI_BASE   0x0

void setUp(void)
{
    replay_cache_init();
}

void tearDown(void)
{
}

void test_cache(void)
{
    uint8_t ivi = IVI_BASE;

    for (int i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(SRC_BASE + i,
                                                        SEQNO_BASE,
                                                        ivi));
    }

    /* Cache full. */
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, replay_cache_add(SRC_BASE + REPLAY_CACHE_ENTRIES,
                                                         SEQNO_BASE,
                                                         ivi));

    replay_cache_on_iv_update();

    /* Update IV index bit... */
    ivi = (ivi + 1) & 0x01;
    for (int i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(SRC_BASE + i,
                                                        SEQNO_BASE,
                                                        ivi));
    }

    /* Cache full. */
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, replay_cache_add(SRC_BASE + REPLAY_CACHE_ENTRIES,
                                                         SEQNO_BASE,
                                                         ivi));

    for (int i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(true, replay_cache_has_elem(SRC_BASE + i,
                                                          SEQNO_BASE,
                                                          ivi));

        TEST_ASSERT_EQUAL(false, replay_cache_has_elem(SRC_BASE + i,
                                                          SEQNO_BASE + 1,
                                                          ivi));
    }

    /* Update IV index. Should still get matches on old index. */
    replay_cache_on_iv_update();
    for (int i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(true, replay_cache_has_elem(SRC_BASE + i,
                                                      SEQNO_BASE,
                                                      ivi));

        TEST_ASSERT_EQUAL(false, replay_cache_has_elem(SRC_BASE + i,
                                                          SEQNO_BASE + 1,
                                                          ivi));
    }

    /* Update IV index bit... */
    ivi = (ivi + 1) & 0x01;

    /* Should be able to add entries with the updated bit now. */
    for (int i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(false, replay_cache_has_elem(SRC_BASE + i,
                                                       SEQNO_BASE,
                                                       ivi));
    }
}
