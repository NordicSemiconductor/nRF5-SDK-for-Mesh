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

#include "dfu_transfer_mesh.h"
#include <string.h>
#include <unity.h>
#include <cmock.h>
#include "sha256_mock.h"
#include "bootloader_app_bridge_mock.h"

static void*    mp_expected_flash_write_dest;
static void*    mp_expected_flash_write_data;
static uint32_t m_expected_flash_write_length;
static bool     m_postpone_flash_write;
static uint32_t m_retval;
static uint32_t m_write_calls;
static void*    mp_write_buffer_location;

void setUp(void)
{
    bootloader_app_bridge_mock_Init();
    sha256_mock_Init();
    m_retval = NRF_SUCCESS;
    m_write_calls = 0;
    mp_expected_flash_write_dest = NULL;
    mp_expected_flash_write_data = NULL;
    m_expected_flash_write_length = 0;
    m_postpone_flash_write = false;
    dfu_transfer_init();
}

void tearDown(void)
{
    bootloader_app_bridge_mock_Verify();
    bootloader_app_bridge_mock_Destroy();
    sha256_mock_Verify();
    sha256_mock_Destroy();

}

uint32_t flash_write_callback(void* p_dest, void* p_data, uint32_t length, int call_count)
{
    TEST_ASSERT_EQUAL_PTR(mp_expected_flash_write_dest, p_dest);
    TEST_ASSERT_EQUAL(m_expected_flash_write_length, length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(mp_expected_flash_write_data, p_data, length);
    if (!m_postpone_flash_write)
    {
        memcpy(p_dest, p_data, length);
        dfu_transfer_flash_write_complete(p_data);
    }
    mp_write_buffer_location = p_data;
    m_write_calls++;
    return m_retval;
}


/******** Tests ********/
void test_transfer_single_bank(void)
{
    uint8_t target[64] __attribute__((aligned(PAGE_SIZE)));

    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_ADDR,
            dfu_transfer_start(
                (uint32_t*) &target[1],
                (uint32_t*) target,
                65,
                true));

    flash_erase_ExpectAndReturn(target, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) target,
                64,
                true));

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = target;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));
    mp_expected_flash_write_dest = &target[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));

    /* Skip one */
    printf("Skip one\n");
    mp_expected_flash_write_dest = &target[48];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[48],
                segment, 16));
    mp_expected_flash_write_dest = &target[32];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[32],
                segment, 16));

    /* Out of bounds */
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_ADDR,
            dfu_transfer_data(
                (uint32_t) &target[64],
                segment, 16));
    TEST_ASSERT_EQUAL(4, m_write_calls);
}

void test_transfer_banked(void)
{
    uint8_t target[64] __attribute__((aligned(PAGE_SIZE)));
    uint8_t bank[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(bank, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) bank,
                64,
                true));

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = &bank[0];
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));
    mp_expected_flash_write_dest = &bank[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));

    /* Skip one */
    mp_expected_flash_write_dest = &bank[48];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[48],
                segment, 16));
    mp_expected_flash_write_dest = &bank[32];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[32],
                segment, 16));

    TEST_ASSERT_EQUAL(4, m_write_calls);
}

void test_transfer_busy(void)
{
    m_postpone_flash_write = true;
    uint8_t target[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(target, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) target,
                64,
                true));

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = target;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));

    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_BUSY,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));

    dfu_transfer_flash_write_complete(mp_write_buffer_location);

    mp_expected_flash_write_dest = &target[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));

    TEST_ASSERT_EQUAL(2, m_write_calls);
}

void test_transfer_has_entry(void)
{
    uint8_t dummy[64];
    uint8_t target[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(target, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) target,
                64,
                true));

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = target;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));

    uint8_t segment_copy[16];
    memset(segment_copy, 0, 16);
    uint8_t blank_segment[16];
    memset(blank_segment, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, NULL, 0));
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, NULL, 16));
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 0));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_segment, segment_copy, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[16], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_segment, segment_copy, 16);
    /* Overflow */
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[64], segment_copy, 16));
    /* Underflow */
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) dummy, segment_copy, 16));

    mp_expected_flash_write_dest = &target[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[16], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[32], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_segment, segment_copy, 16);

    /* Skip one */
    mp_expected_flash_write_dest = &target[48];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[48],
                segment, 16));
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[16], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[32], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[48], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);


    /* Test the same banked */
    uint8_t bank[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(bank, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) bank,
                64,
                true));

    mp_expected_flash_write_dest = &bank[0];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);

    mp_expected_flash_write_dest = &bank[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) target, segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
    memset(segment_copy, 0xAB, 16);
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[16], segment_copy, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(segment, segment_copy, 16);
}

void test_transfer_oldest_missing_entry(void)
{
    uint8_t target[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(target, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) target,
                64,
                true));

    /* No missing segments yet */
    uint32_t* p_entry = NULL;
    uint32_t length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = target;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));

    /* No missing segments yet */
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    mp_expected_flash_write_dest = &target[48];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[48],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[16], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[16], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[16], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[32], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    p_entry = NULL;
    length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[48], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    mp_expected_flash_write_dest = &target[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[16], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[32], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    p_entry = NULL;
    length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[48], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    /* Now banked */
    uint8_t bank[64] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(bank, PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) bank,
                64,
                true));

    /* No missing segments yet */
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    mp_expected_flash_write_dest = bank;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));

    /* No missing segments yet */
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    mp_expected_flash_write_dest = &bank[48];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[48],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[16], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[16], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[16], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[32], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    p_entry = NULL;
    length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[48], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    mp_expected_flash_write_dest = &bank[16];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[16], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    TEST_ASSERT_TRUE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[32], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(&target[32], p_entry);
    TEST_ASSERT_EQUAL(16, length);
    p_entry = NULL;
    length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) &target[48], &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);
}

void test_miss_overflow(void)
{
    uint8_t target[2048] __attribute__((aligned(PAGE_SIZE)));

    flash_erase_ExpectAndReturn(target, 2*PAGE_SIZE, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_start(
                (uint32_t*) target,
                (uint32_t*) target,
                2048,
                true));

    /* No missing segments yet */
    uint32_t* p_entry = NULL;
    uint32_t length = 0;
    TEST_ASSERT_FALSE(dfu_transfer_get_oldest_missing_entry((uint32_t*) target, &p_entry, &length));
    TEST_ASSERT_EQUAL_PTR(NULL, p_entry);
    TEST_ASSERT_EQUAL(0, length);

    uint8_t segment[16];
    for (uint32_t i = 0; i < 16; i++)
    {
        segment[i] = i;
    }
    mp_expected_flash_write_dest = target;
    mp_expected_flash_write_data = segment;
    m_expected_flash_write_length = 16;
    flash_write_StubWithCallback(flash_write_callback);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) target,
                segment, 16));

    /* Skip to 31, should NOT cause a dfu_end call */
    mp_expected_flash_write_dest = &target[16*31];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16*31],
                segment, 16));
    uint8_t entry[16];
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[0], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&target[0], entry, 16);
    uint8_t blank_entry[16];
    memset(blank_entry, 0, 16);
    memset(entry, 0, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[16], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_entry, entry, 16);

    mp_expected_flash_write_dest = &target[16*63];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16*63],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[0], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&target[0], entry, 16);
    memset(entry, 0, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[16], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_entry, entry, 16);
    /* Entry 64 should push the missing entry 2 to the last bit */
    mp_expected_flash_write_dest = &target[16*64];
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS,
            dfu_transfer_data(
                (uint32_t) &target[16*64],
                segment, 16));
    TEST_ASSERT_TRUE(dfu_transfer_has_entry((uint32_t*) &target[0], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&target[0], entry, 16);
    memset(entry, 0, 16);
    TEST_ASSERT_FALSE(dfu_transfer_has_entry((uint32_t*) &target[16], entry, 16));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(blank_entry, entry, 16);

    /* Push the first entry off the bitfield. */
    mp_expected_flash_write_dest = &target[16*65];
    send_end_evt_Expect(DFU_END_ERROR_PACKET_LOSS);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_NOT_FOUND,
            dfu_transfer_data(
                (uint32_t) &target[16*65],
                segment, 16));
}
