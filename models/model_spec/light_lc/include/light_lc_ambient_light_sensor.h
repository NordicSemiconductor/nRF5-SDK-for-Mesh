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

#ifndef LIGHT_LIGHT_LC_AMBIENT_LIGHT_SENSOR_H__
#define LIGHT_LIGHT_LC_AMBIENT_LIGHT_SENSOR_H__

#include "light_lc_setup_server.h"

/**
 * @internal
 * @defgroup LIGHT_LC_AMBIENT_LIGHT_SENSOR Light LC Ambient Light sensor model interface
 * @ingroup LIGHT_LC_MODELS
 *
 * The Ambient LuxLevel Input is represented by the Light LC Ambient LuxLevel state and accepts data
 * reported by zero or more sensors reporting the Ambient LuxLevel Property with Sensor Status
 * messages. See @tagMeshMdlSp section 6.2.3.5 for more information.
 * @{
 */

/** Parses an ambient light sensor data coming from a BLE Mesh sensor.
 *
 * When a Sensor Status packet is received, this function call will
 * check to see if it is ambient light sensor data. If so,  the data is saved
 * for future processing by the light PI regulator.
 *
 * @param[in] p_s_server  Pointer to the model structure.
 * @param[in] p_msg_data  Pointer to a byte array containing the sensor data to be parsed.
 * @param[in] msg_length  The number of bytes in the p_msg_data.
 *
 * @retval NRF_SUCCESS              If the occupancy sensor started successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NOT_SUPPORTED  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
 * @retval NRF_ERROR_INTERNAL       Internal error while parsing entry.
 */
uint32_t light_lc_ambient_light_sensor_data_received(light_lc_setup_server_t * p_s_server,
                                                     uint8_t * p_msg_data,
                                                     uint16_t msg_length);

/**@} end of LIGHT_LC_AMBIENT_LIGHT_SENSOR */
#endif /* LIGHT_LIGHT_LC_AMBIENT_LIGHT_SENSOR_H__ */
