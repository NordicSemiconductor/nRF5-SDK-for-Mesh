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

#include <string.h>
#include "nrf_mesh_configure.h"
#include "nrf_mesh_defines.h"
#include "nrf.h"
#if defined NRF_MESH_TEST_SHIM
#include "test_instrument.h"
#endif

/** UUID version - 4 */
#define UUID_VERSION4 (0x04)
/** Variant - reserved bits  */
#define UUID_VERSION4_VARIANT_BITS (0x02)

#define UUID_TIME_HI_VERSION_MSB_OFFSET     (6)
#define UUID_FIELD_VERSION_MASK             (0xF0)
#define UUID_FILED_VERSION_OFFSET           (4)
#define UUID_CLOCK_SEQ_HI_VARIANT_OFFSET    (8)
#define UUID_FIELD_VARIANT_MASK             (0xC0)
#define UUID_FIELD_VARIANT_OFFSET           (6)

#define VARIANT_SET(_p_uuid, _variant)  _p_uuid[UUID_CLOCK_SEQ_HI_VARIANT_OFFSET] = \
            (_p_uuid[UUID_CLOCK_SEQ_HI_VARIANT_OFFSET] & ~UUID_FIELD_VARIANT_MASK) | (_variant<<UUID_FIELD_VARIANT_OFFSET)

#define VERSION_SET(_p_uuid, _version)  _p_uuid[UUID_TIME_HI_VERSION_MSB_OFFSET] = \
            (_p_uuid[UUID_TIME_HI_VERSION_MSB_OFFSET] & ~UUID_FIELD_VERSION_MASK) | (_version<<UUID_FILED_VERSION_OFFSET)

/*****************************************************************************
* Static locals
*****************************************************************************/
static uint8_t m_uuid[NRF_MESH_UUID_SIZE];
/*****************************************************************************
* Interface functions
*****************************************************************************/

void nrf_mesh_configure_device_uuid_set(const uint8_t* p_uuid)
{
    memcpy(m_uuid, p_uuid, NRF_MESH_UUID_SIZE);
}

void nrf_mesh_configure_device_uuid_reset(void)
{
    /* First half of device UUID is the DEVICE ID, the second half is the
     * advertisement address. */
    /* Device ID */
    memcpy(m_uuid, (void*) &NRF_FICR->DEVICEID[0], 8);
    /* Advertisement address */
    memcpy(&m_uuid[8], (void*) &NRF_FICR->DEVICEADDR[0], 8);

    /* Overwrite the version and variant bits */
    VERSION_SET(m_uuid, UUID_VERSION4);
    VARIANT_SET(m_uuid, UUID_VERSION4_VARIANT_BITS);

#if defined NRF_MESH_TEST_SHIM
    nrf_mesh_test_shim(EDIT_DEVICE_UUID, m_uuid);
#endif
}

const uint8_t* nrf_mesh_configure_device_uuid_get(void)
{
    return m_uuid;
}
