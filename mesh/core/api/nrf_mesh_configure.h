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

#ifndef NRF_MESH_CONFIGURE_H__
#define NRF_MESH_CONFIGURE_H__

#include <stdint.h>

/**
 * @defgroup NRF_MESH_CONFIGURE Runtime configuration
 * Configuration of the core mesh during runtime
 * @ingroup CORE_CONFIG
 * @{
 */

/**
 * Set the UUID of the device. If never called, the framework will use the
 * factory imprinted ID.
 *
 * @note Changes to the UUID are reset on power cycles.
 *
 * @param[in] p_uuid A UUID of size @ref NRF_MESH_UUID_SIZE to set as the device's current UUID.
 */
void nrf_mesh_configure_device_uuid_set(const uint8_t* p_uuid);

/**
 * Set the UUID of the device to the factory imprinted ID.
 *
 * @note Changes to the UUID are reset on power cycles.
 */
void nrf_mesh_configure_device_uuid_reset(void);

/**
 * Obtains the device UUID from the mesh stack.
 *
 * @return A pointer to the internal UUID array.
 */
const uint8_t* nrf_mesh_configure_device_uuid_get(void);

/** @} end of NRF_MESH_CONFIGURE */

#endif /* NRF_MESH_CONFIGURE_H__ */
