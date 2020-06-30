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

#ifndef LIGHT_LC_SENSOR_UTILS_H__
#define LIGHT_LC_SENSOR_UTILS_H__

#include <stdint.h>

/**
 * @internal
 * @defgroup LC_SENSOR_UTILS LC sensor utility functions
 * @ingroup LIGHT_LC_MODELS
 * @{
 */

#define SENSOR_STATUS_MINLEN 3
/* @tagMeshMdlSp section 4.2.14 - max length of single sensor msg */
#define SENSOR_STATUS_MAXLEN_SINGLE_SENSOR 130
/* @tagMeshMdlSp section 2.3.3 - max # of octets with a 1 byte opcode */
#define SENSOR_STATUS_MAXLEN 379

/** Function to parse marshalled sensor data from a BLE mesh sensor
 *
 * BLE mesh occupancy and ambient light sensors send marshalled data (see @tagMeshMdlSp section
 * 4.2.14 for details on the marshalled data format).  LC servers receive this format (see
 * @tagMeshMdlSp section 6.5.1.7.1), and this function parses it.
 *
 * @param[in] p_data_buf             Pointer to the marshalled sensor data to be parsed.
 * @param[out] p_format              Pointer to format value - 0 means data was format A, 1 means B.
 * @param[out] p_data_length         Pointer to data length encoded in the marshalled data.
 * @param[out] p_property_id         Pointer to property id encoded in the marshalled data.
 * @param[out] pp_data_value         Pointer to the raw value parsed out of the marshalled data.
 *
 * @returns The address of the location in the p_data_buf that has been parsed.  If this value
 * indicates that there is more data to be parsed, the calling function should call this function
 * again with this return value as the p_data_buf to parse (i.e. this would happen in the case that
 * there are multiple sensor entries in the buffer)
 */
uint8_t * light_lc_sensor_utils_parse_marshalled_entry(uint8_t * p_data_buf,
                                              uint8_t * p_format,
                                              uint8_t * p_data_length,
                                              uint16_t * p_property_id,
                                              uint8_t ** pp_data_value);
/**@} end of LC_SENSOR_UTILS */
#endif /* LIGHT_LC_SENSOR_UTILS_H__ */
