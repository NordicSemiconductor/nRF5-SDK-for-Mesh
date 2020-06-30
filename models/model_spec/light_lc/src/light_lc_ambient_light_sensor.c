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

#include "light_lc_ambient_light_sensor.h"

#include "light_lc_sensor_utils.h"
#include "light_lc_state_utils.h"
#include "light_lc_server_property_constants.h"

uint32_t light_lc_ambient_light_sensor_data_received(light_lc_setup_server_t * p_s_server,
                                                     uint8_t * p_msg_data,
                                                     uint16_t msg_length)
{
    uint8_t * p_msg_rover;
    uint8_t data_length;
    uint8_t expected_length;
    uint8_t format;
    uint16_t property_id;
    uint8_t * data_value;
    uint32_t data_intval = 0;
    uint32_t status = NRF_SUCCESS;

    if ((p_s_server == NULL) || (p_msg_data == NULL))
    {
        return NRF_ERROR_NULL;
    }

    /* The supported type (property) for a light sensor (@tagMeshMdlSp section 6.5.1.7.1): Ambient Light
       Level (characteristic Illuminance) */

    p_msg_rover = p_msg_data;
    while ((p_msg_rover - p_msg_data) < msg_length)
    {
        p_msg_rover = light_lc_sensor_utils_parse_marshalled_entry(p_msg_rover, &format, &data_length,
                                                          &property_id, &data_value);
        if (p_msg_rover == NULL)
        {
            return NRF_ERROR_INTERNAL;
        }

        /* Note that this sensor type is a Format 0 (@tagMeshMdlSp section 4.2.14) */
        if (format != 0)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unsupported sensor data format  %d\n", format);
            return NRF_ERROR_NOT_SUPPORTED;
        }

        if (property_id == LIGHT_LC_SERVER_PRESENT_AMBIENT_LIGHT_LEVEL_PID)
        {
            /* get the size that the data should be and verify */
            status = light_lc_state_utils_property_data_size_get(property_id, &expected_length);
            if (status != NRF_SUCCESS)
            {
                return status;
            }

            if (expected_length != data_length)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                      "Invalid data length %d, property %d length s.b. %d\n",
                      data_length, property_id, msg_length);
                return NRF_ERROR_DATA_SIZE;
            }

            memcpy(&data_intval, data_value, data_length);

            /* We have processed a BLE ambient light message, now store that data.  The Light PI
             * regulator will read this the next time it runs */
            light_lc_state_utils_ambient_luxlevel_set(p_s_server, data_intval);
        }
    }
    return status;
}
