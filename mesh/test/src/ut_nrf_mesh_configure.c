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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_configure.h"
#include "nrf.h"

/** UUID version - 4 */
#define UUID_VERSION4 (0x04)
/** Variant - reserved bits  */
#define UUID_VERSION4_VARIANT_BITS (0x02)

typedef struct __attribute((packed))
{
    uint64_t uuid_00_51 : 52;
    uint64_t version : 4;
    uint64_t uuid_56_63: 8;

    uint64_t uuid_64_67 : 4;
    uint64_t uuid_68_69 : 2;
    uint64_t variant : 2;

    uint64_t uuid_72_127 : 56;
} uuid4_t;

NRF_FICR_Type m_ficr;
NRF_FICR_Type * NRF_FICR;


void setUp(void)
{
    NRF_FICR = &m_ficr;
}

void tearDown(void)
{

}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_uuid(void)
{

    nrf_mesh_configure_device_uuid_reset();

    uint8_t uuid[NRF_MESH_UUID_SIZE];
    memset(uuid, 0, NRF_MESH_UUID_SIZE);
    const uint8_t * p_uuid = nrf_mesh_configure_device_uuid_get();
    const uuid4_t * p_uuid4 = (const uuid4_t *) p_uuid;
    TEST_ASSERT_EQUAL(p_uuid4->version, UUID_VERSION4);
    TEST_ASSERT_EQUAL(p_uuid4->variant, UUID_VERSION4_VARIANT_BITS);

    for (uint32_t i = 0; i < NRF_MESH_UUID_SIZE; i++)
    {
        uuid[i] = i;
    }
    nrf_mesh_configure_device_uuid_set(uuid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE);
    nrf_mesh_configure_device_uuid_reset();

    p_uuid = nrf_mesh_configure_device_uuid_get();
    p_uuid4 = (const uuid4_t *) p_uuid;
    TEST_ASSERT_EQUAL(p_uuid4->version, UUID_VERSION4);
    TEST_ASSERT_EQUAL(p_uuid4->variant, UUID_VERSION4_VARIANT_BITS);

    /* See @tagMeshSp section 8.4.1 */
    const uint8_t sample_uuid[16] = {0x70, 0xcf, 0x7c, 0x97, 0x32, 0xa3, 0x45, 0xb6, 0x91, 0x49, 0x48, 0x10, 0xd2, 0xe9, 0xcb, 0xf4};
    nrf_mesh_configure_device_uuid_set(sample_uuid);
    p_uuid4 = (const uuid4_t *)nrf_mesh_configure_device_uuid_get();

    TEST_ASSERT_EQUAL(p_uuid4->version, UUID_VERSION4);
    TEST_ASSERT_EQUAL(p_uuid4->variant, UUID_VERSION4_VARIANT_BITS);
}
