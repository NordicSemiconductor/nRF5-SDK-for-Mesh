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

#ifndef LIGHT_LC_SERVER_PROPERTY_CONSTANTS_H__
#define LIGHT_LC_SERVER_PROPERTY_CONSTANTS_H__

/**
 * @defgroup LIGHT_LC_SERVER_PROPERTY_CONSTANTS Light LC Server property constants
 * @ingroup LIGHT_LC_MODELS
 *
 * All the information about the format, units and allowed values of the properties
 * can be found in Section 4.1.3 of @link_MeshDeviceProperties, @link_MeshProperties and
 * @link_MeshCharacteristics.
 *
 * @{
 */

/** No Property */
#define LIGHT_LC_SERVER_NO_PID (0)
/** The Light Control Ambient LuxLevel On Property ID */
#define LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID (0x002B)
/** The Light Control Ambient LuxLevel Prolong Property ID */
#define LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_PROLONG_PID (0x002C)
/** The Light Control Ambient LuxLevel Standby Property ID */
#define LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID (0x002D)
/** The Light Control Lightness On Property ID */
#define LIGHT_LC_SERVER_LIGHTNESS_ON_PID (0x002E)
/** The Light Control Lightness Prolong Property ID */
#define LIGHT_LC_SERVER_LIGHTNESS_PROLONG_PID (0x002F)
/** The Light Control Lightness Standby Property ID */
#define LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID (0x0030)
/** The Light Control Regulator Accuracy Property ID */
#define LIGHT_LC_SERVER_REGULATOR_ACCURACY_PID (0x0031)
/** The Light Control Regulator Kid Property ID */
#define LIGHT_LC_SERVER_REGULATOR_KID_PID (0x0032)
/** The Light Control Regulator Kiu Property ID */
#define LIGHT_LC_SERVER_REGULATOR_KIU_PID (0x0033)
/** The Light Control Regulator Kpd Property ID */
#define LIGHT_LC_SERVER_REGULATOR_KPD_PID (0x0034)
/** The Light Control Regulator Kpd Property ID */
#define LIGHT_LC_SERVER_REGULATOR_KPU_PID (0x0035)
/** The Light Control Time Fade Property ID */
#define LIGHT_LC_SERVER_TIME_FADE_PID (0x0036)
/** The Light Control Time Fade On Property ID */
#define LIGHT_LC_SERVER_TIME_FADE_ON_PID (0x0037)
/** The Light Control Time Fade Standby Auto Property ID */
#define LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID (0x0038)
/** The Light Control Time Fade Standby Manual Property ID */
#define LIGHT_LC_SERVER_TIME_FADE_STANDBY_MANUAL_PID (0x0039)
/** The Light Control Time Occupancy Delay Property ID */
#define LIGHT_LC_SERVER_TIME_OCCUPANCY_DELAY_PID (0x003A)
/** The Light Control Time Prolong Property ID */
#define LIGHT_LC_SERVER_TIME_PROLONG_PID (0x003B)
/** The Light Control Time Run On Property ID */
#define LIGHT_LC_SERVER_TIME_RUN_ON_PID (0x003C)
/** The Motion Sensed Property ID */
#define LIGHT_LC_SERVER_MOTION_SENSED_PID (0x0042)
/** The People Count Property ID */
#define LIGHT_LC_SERVER_PEOPLE_COUNT_PID (0x004C)
/** The Presence Detected Property ID */
#define LIGHT_LC_SERVER_PRESENCE_DETECT_PID (0x004D)
/** The Time Since Motion Sensed Property ID */
#define LIGHT_LC_SERVER_TIME_SINCE_MOTION_SENSED_PID (0x0068)
/** The Present Ambient Light Level Property ID */
#define LIGHT_LC_SERVER_PRESENT_AMBIENT_LIGHT_LEVEL_PID (0x004E)
/** Defines size of the Coefficient characteristic */
#define LIGHT_LC_SERVER_COEFFICIENT_SIZE (4)
/** Defines size of the illuminance characteristic */
#define LIGHT_LC_SERVER_ILLUMINANCE_24_SIZE (3)
/** Defines size of the Time Millisecond 24 characteristic */
#define LIGHT_LC_SERVER_TIME_MILLISECOND_24_SIZE (3)
/** Defines size of the Perceived Lightness characteristic */
#define LIGHT_LC_SERVER_PERCEIVED_LIGHTNESS_SIZE (2)
/** Defines size of the Count 16 characteristic */
#define LIGHT_LC_SERVER_COUNT_16_SIZE  (2)
/** Defines size of the Percentage 8 characteristic */
#define LIGHT_LC_SERVER_PERCENTAGE_8_SIZE (1)
/** Defines size of the Boolean characteristic */
#define LIGHT_LC_SERVER_BOOLEAN_SIZE (1)
/** Defines size of uint8_t */
#define LIGHT_LC_SERVER_UINT8_SIZE (1)

/**@} end of LIGHT_LC_SERVER_PROPERTY_CONSTANTS */
#endif /* LIGHT_LC_SERVER_PROPERTY_CONSTANTS_H__ */
