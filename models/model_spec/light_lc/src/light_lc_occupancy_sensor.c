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

#include "light_lc_occupancy_sensor.h"

#include "light_lc_sensor_utils.h"
#include "light_lc_state_utils.h"
#include "light_lc_fsm.h"
#include "light_lc_server_property_constants.h"

uint32_t light_lc_occupancy_sensor_data_received(light_lc_setup_server_t * p_s_server,
                                                 uint8_t * p_msg_data,
                                                 uint16_t msg_length)
{
    uint8_t * p_msg_rover;
    bool occupancy_event_needed = false;
    uint8_t data_length;
    uint8_t expected_length;
    uint8_t format;
    uint16_t property_id;
    uint8_t * p_data_value;
    uint32_t time_since_motion_sensed_value = 0;
    uint16_t data_intval = 0;
    uint32_t time_occupancy_delay_val;
    uint32_t status = NRF_SUCCESS;

    if ((p_s_server == NULL) || (p_msg_data == NULL))
    {
        return NRF_ERROR_NULL;
    }

    /* Types (properties) of occupancy sensors supported (@tagMeshMdlSp section 6.5.1.7.1): 1) Motion Sensed
       (generate occupancy event if value > 0) (characteristic Percentage 8) 2) People Count
       (generate occupancy event if value > 0) (characteristic People 16) 3) Presence Detected
       (generate occupancy event if value > 0) (characteristic Boolean)

       Additionally, if the message also contains a Time Since Motion Sensed property
       (characteristic Time Second 16), we will delay sending the occupancy event as per
       @tagMeshMdlSp section 6.5.1.7.1 */

    p_msg_rover = p_msg_data;
    while ((p_msg_rover - p_msg_data) < msg_length)
    {
        p_msg_rover = light_lc_sensor_utils_parse_marshalled_entry(p_msg_rover, &format, &data_length,
                                                          &property_id, &p_data_value);
        if (p_msg_rover == NULL)
        {
            return NRF_ERROR_INTERNAL;
        }

        /* Note that all of the sensor types are Format 0 (@tagMeshMdlSp section 4.2.14) */
        if (format != 0)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unsupported sensor data format  %d\n", format);
            return NRF_ERROR_NOT_SUPPORTED;
        }

        if ((property_id == LIGHT_LC_SERVER_MOTION_SENSED_PID) ||
            (property_id == LIGHT_LC_SERVER_PEOPLE_COUNT_PID) ||
            (property_id == LIGHT_LC_SERVER_PRESENCE_DETECT_PID) ||
            (property_id == LIGHT_LC_SERVER_TIME_SINCE_MOTION_SENSED_PID))
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
                return NRF_ERROR_INVALID_LENGTH;
            }

            memcpy(&data_intval, p_data_value, data_length);

            if (property_id == LIGHT_LC_SERVER_TIME_SINCE_MOTION_SENSED_PID)
            {
                if (data_intval > 0)
                {
                    time_since_motion_sensed_value = data_intval;
                }
            }
            else
            {
                /* @tagMeshMdlSp - section 6.5.1.7.1 for all 3 of the occupancy detect property ids,
                 * if the value is > 0, we have occupancy */
                if (data_intval > 0)
                {
                    occupancy_event_needed = true;
                }
            }
        }
    }
    if (occupancy_event_needed)
    {
        /* We need to send an occupancy event. */
        if (time_since_motion_sensed_value > 0)
        {
            /* We received a "time since motion sensed" message - check to see if we need to delay
             * before sending the message (@tagMeshMdlSp section 6.5.1.7.1) */
            time_occupancy_delay_val =
                light_lc_state_utils_property_get(p_s_server, LIGHT_LC_SERVER_TIME_OCCUPANCY_DELAY_PID);
            /* since time_since_motion_sensed is in seconds, convert
             * it to milliseconds before comparing */
            time_since_motion_sensed_value = SEC_TO_MS(time_since_motion_sensed_value);

            if (time_since_motion_sensed_value < time_occupancy_delay_val)
            {
                p_s_server->sensor_delay_timer.timeout_rtc_ticks =
                    MODEL_TIMER_TICKS_GET_US((uint32_t)MS_TO_US(time_occupancy_delay_val -
                                                                 time_since_motion_sensed_value));

                p_s_server->sensor_delay_timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;
                status = model_timer_schedule(&p_s_server->sensor_delay_timer);
            }
            else
            {
                /* No delay needed - send the event */
                status = light_lc_fsm_occupancy_event_generate(p_s_server);
            }
        }
        else
        {
            /* No "time since motion sensed" message was received; send
             * the event */
            status = light_lc_fsm_occupancy_event_generate(p_s_server);
        }
    }
    return status;
}

static void occupancy_delay_event_cb(void * p_context)
{
    light_lc_setup_server_t * p_s_server = (light_lc_setup_server_t *) p_context;

    uint32_t status = light_lc_fsm_occupancy_event_generate(p_s_server);
    NRF_MESH_ASSERT(status == NRF_SUCCESS);
}

uint32_t light_lc_occupancy_sensor_init(light_lc_setup_server_t * p_s_server)
{
    if (p_s_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Create and start timer used by the occupancy sensor event generation */
    p_s_server->sensor_delay_timer.p_context = p_s_server;
    p_s_server->sensor_delay_timer.cb = occupancy_delay_event_cb;
    return(model_timer_create(&p_s_server->sensor_delay_timer));
}
