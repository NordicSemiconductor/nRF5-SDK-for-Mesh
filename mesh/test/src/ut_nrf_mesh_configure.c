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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_configure.h"
#include "nrf.h"

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
    NRF_FICR->DEVICEID[0] = 0x12345678;
    NRF_FICR->DEVICEID[1] = 0x9ABCDEF0;
    NRF_FICR->DEVICEADDR[0] = 0x01234567;
    NRF_FICR->DEVICEADDR[1] = 0x89ABCDEF;
    nrf_mesh_configure_device_uuid_reset();
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    memset(uuid, 0, NRF_MESH_UUID_SIZE);
    const uint8_t * p_uuid = nrf_mesh_configure_device_uuid_get();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NRF_FICR->DEVICEID, &p_uuid[0], 8);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NRF_FICR->DEVICEADDR, &p_uuid[8], 8);
    for (uint32_t i = 0; i < NRF_MESH_UUID_SIZE; i++)
    {
        uuid[i] = i;
    }
    nrf_mesh_configure_device_uuid_set(uuid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE);
    nrf_mesh_configure_device_uuid_reset();
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NRF_FICR->DEVICEID, &p_uuid[0], 8);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NRF_FICR->DEVICEADDR, &p_uuid[8], 8);
}
