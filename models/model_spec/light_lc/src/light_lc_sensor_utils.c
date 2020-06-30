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

#include "light_lc_sensor_utils.h"

#include "log.h"
/* Uncomment the next line for debug information - helpful when
 * setting up new hardware. */
/* #define DEBUG_SENSOR 1 */

#define FORMAT_A_MPID_SIZE 2
#define FORMAT_B_MPID_SIZE 3

#define FORMAT_A_BIT 0
#define FORMAT_MASK 0x01
#define LENGTH_MASK_A 0x0F
#define LENGTH_SHIFT_A 1
#define PROPERTY_MASK_A_OCTET0 0x0007
#define PROPERTY_SHIFT_A_OCTET0 5
#define PROPERTY_MASK_A_OCTET1 0x03F8
#define PROPERTY_SHIFT_A_OCTET1 3

#define FORMAT_B_BIT 1
#define LENGTH_MASK_B 0x7F
#define LENGTH_SHIFT_B 1
#define PROPERTY_MASK_B_OCTET1 0x00FF
#define PROPERTY_MASK_B_OCTET2 0xFF00
#define PROPERTY_SHIFT_B_OCTET2 8


uint8_t * light_lc_sensor_utils_parse_marshalled_entry(uint8_t * p_data_buf,
                                                       uint8_t * p_format,
                                                       uint8_t * p_data_length,
                                                       uint16_t * p_property_id,
                                                       uint8_t ** pp_data_value)
{
    uint8_t length;

    if ((p_data_buf == NULL) || (p_format == NULL) || (p_data_length == NULL) || 
        (p_property_id == NULL) || (pp_data_value == NULL))
    {
        return NULL;
    }
    
    /* figure out which format of marshalled data this is first */
    if ((p_data_buf[0] & FORMAT_MASK) == FORMAT_A_BIT)
    {
        /* It's format a */
        /* Length is defined as 0-0xF representing 1-16, therefore + 1 */
        length = ((p_data_buf[0] >> LENGTH_SHIFT_A) & LENGTH_MASK_A) + 1;
#ifdef DEBUG_SENSOR
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "in format a, data length = %d \n", length);
#endif

        *p_property_id = (((uint16_t) p_data_buf[0] >> PROPERTY_SHIFT_A_OCTET0) & PROPERTY_MASK_A_OCTET0);
        *p_property_id |= ((uint16_t) p_data_buf[1] << PROPERTY_SHIFT_A_OCTET1) & PROPERTY_MASK_A_OCTET1;
        *pp_data_value = &p_data_buf[FORMAT_A_MPID_SIZE];
        *p_data_length = length;
        *p_format = 0;
#ifdef DEBUG_SENSOR
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "property_id=0x%X, length = %d\n",
              *p_property_id, *p_data_length);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "data_value=", *pp_data_value, length);
#endif

        return *pp_data_value + length;
    }
    else
    {
        /* It's format b */
        length = ((p_data_buf[0] >> LENGTH_SHIFT_B) & LENGTH_MASK_B);
        /* Length is defined as 0-0x7F representing 1-127, 0x7F = 0 */
        if (length ==  0x7F)
        {
            length = 0;
        }
        else
        {
            length++;
        }
        *p_property_id = ((uint16_t) p_data_buf[1] & PROPERTY_MASK_B_OCTET1);
        *p_property_id |= (((uint16_t) p_data_buf[2] << PROPERTY_SHIFT_B_OCTET2) & PROPERTY_MASK_B_OCTET2);
        *pp_data_value = &p_data_buf[FORMAT_B_MPID_SIZE];
        *p_data_length = length;
        *p_format = 1;
        return *pp_data_value + length;
    }
}
