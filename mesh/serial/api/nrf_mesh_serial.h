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

#ifndef NRF_MESH_SERIAL_H__
#define NRF_MESH_SERIAL_H__

#include <stdint.h>
#include <limits.h>
#include "nrf_mesh.h"

/**
 * @defgroup NRF_MESH_SERIAL Serial API
 * @ingroup MESH_API_GROUP_SERIAL
 * Target-side serial interface module, providing serialized access to all major mesh APIs.
 * @{
 */
/**
 * @addtogroup SERIAL_DEFINES
 * @{
 */

/** Maximum length of a serial packet's payload. Limited by the size of the length byte minus
 * packet overhead (length and opcode). */
#define NRF_MESH_SERIAL_PAYLOAD_MAXLEN    (UINT8_MAX - 1)
/** Overhead of the packet header, not including the length field. */
#define NRF_MESH_SERIAL_PACKET_OVERHEAD   (1UL)

/** @} */

/**
 * @addtogroup SERIAL_TYPES
 * @{
 */

/** Mesh serial states. */
typedef enum
{
    NRF_MESH_SERIAL_STATE_UNINITIALIZED, /**< The serial module hasn't been initialized. */
    NRF_MESH_SERIAL_STATE_INITIALIZED,   /**< The serial module has been initialized, but not started. */
    NRF_MESH_SERIAL_STATE_RUNNING,       /**< The serial module is running. */
} nrf_mesh_serial_state_t;

/** Serial RX callback function type for application events. */
typedef void (*nrf_mesh_serial_app_rx_cb_t)(const uint8_t* p_data, uint32_t length);

/** @} */

/**
 * Initialize the serial module and all its related submodules. The device will
 * initialize the given serial bearer, but NOT send the device started event,
 * indicating that the device is ready for operation.
 *
 * @param[in] app_rx_cb Application command handler function pointer, or NULL
 * if the application commands shouldn't be handled.
 *
 * @retval NRF_SUCCESS The serial module was successfully initialized.
 * @retval NRF_ERROR_INVALID_STATE The serial module has already been initialized.
 * @retval NRF_ERROR_NULL The p_init_params or one or more of the context lists
 * were NULL.
 */
uint32_t nrf_mesh_serial_init(nrf_mesh_serial_app_rx_cb_t app_rx_cb);

/**
 * Enable the serial connection. Pushes a device started event across the
 * serial line, notifying the controller that the device is ready for
 * operation.
 *
 * @retval NRF_SUCCESS The serial was successfully enabled.
 * @retval NRF_ERROR_INVALID_STATE The serial module has not been initialized,
 * or has already been enabled.
 * @retval NRF_ERROR_INTERNAL The call failed due to an unexpected error in one
 * of the internal sub modules to the serial module.
 */
uint32_t nrf_mesh_serial_enable(void);

/**
 * Get the current state of the serial module.
 *
 * @return An enum in @ref nrf_mesh_serial_state_t, indicating the current state of the serial.
 */
nrf_mesh_serial_state_t nrf_mesh_serial_state_get(void);

/**
 * Transmit a serial packet with the APPLICATION_EVT opcode across the serial.
 *
 * @param[in] p_data The byte array to transmit across the serial.
 * @param[in] length Length of the @c p_data array. May not exceed the maximum
 *            length @ref NRF_MESH_SERIAL_PAYLOAD_MAXLEN.
 *
 * @retval NRF_SUCCESS The packet was successfully scheduled for transmission.
 * @retval NRF_ERROR_INVALID_STATE The serial module has not been initialized
 *                                 and enabled.
 * @retval NRF_ERROR_NULL The @c p_data pointer was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH The length parameter exceeds the maximum
 *                                  serial packet length, or was 0.
 * @retval NRF_ERROR_NO_MEM The serial TX queue was full, and the packet
 *                          couldn't be scheduled for transmission.
 */
uint32_t nrf_mesh_serial_tx(uint8_t* p_data, uint32_t length);

/** @} */

#endif /* NRF_MESH_SERIAL_H__ */

