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
#ifndef EXAMPLE_COMMON_H__
#define EXAMPLE_COMMON_H__

#include "sdk_config.h"
#include "uri.h"

#define RTT_INPUT_POLL_PERIOD_MS    (100)

#define LED_BLINK_INTERVAL_MS           (200)
#define LED_BLINK_SHORT_INTERVAL_MS     (50)
#define LED_BLINK_CNT_START             (2)
#define LED_BLINK_CNT_RESET             (3)
#define LED_BLINK_CNT_PROV              (4)
#define LED_BLINK_CNT_NO_REPLY          (6)
#define LED_BLINK_CNT_ERROR             (6)

/* An interval larger than half a second might not show LED blinking effect. */
#define LED_BLINK_ATTENTION_INTERVAL_MS (50)
#define LED_BLINK_ATTENTION_COUNT(s)    (((s) * 500) / LED_BLINK_ATTENTION_INTERVAL_MS)

/**
 * Clock configuration for Nordic development boards.
 */
#if defined(S110)
    #define DEV_BOARD_LF_CLK_CFG  NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#elif NRF_SD_BLE_API_VERSION >= 5
    #define DEV_BOARD_LF_CLK_CFG  { \
        .source = NRF_SDH_CLOCK_LF_SRC, \
        .rc_ctiv = NRF_SDH_CLOCK_LF_RC_CTIV, \
        .rc_temp_ctiv = NRF_SDH_CLOCK_LF_RC_TEMP_CTIV, \
        .accuracy = NRF_SDH_CLOCK_LF_ACCURACY \
    }
#else
    #define DEV_BOARD_LF_CLK_CFG  { \
        .source = NRF_CLOCK_LF_SRC_XTAL, \
        .rc_ctiv = 0, \
        .rc_temp_ctiv = 0, \
        .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM \
    }
#endif

/** Uniform Resource Identifiers (URIs) for the examples.
 *
 *
 * @note Replace the example URI strings with the desired URIs for the end products. The URI strings
 * should be coded as specified in the Bluetooth Core Specification Supplement v6, section 1.18.
 */
#define EX_URI_BEACON        URI_SCHEME_EXAMPLE "URI for Beacon example"
#define EX_URI_DFU           URI_SCHEME_EXAMPLE "URI for DFU example"
#define EX_URI_ENOCEAN       URI_SCHEME_EXAMPLE "URI for Enocean example"
#define EX_URI_DM_CLIENT     URI_SCHEME_EXAMPLE "URI for Dimming Client example"
#define EX_URI_DM_SERVER     URI_SCHEME_EXAMPLE "URI for Dimming Server example"
#define EX_URI_LPN           URI_SCHEME_EXAMPLE "URI for LPN example"
#define EX_URI_LS_CLIENT     URI_SCHEME_EXAMPLE "URI for LS Client example"
#define EX_URI_LS_SERVER     URI_SCHEME_EXAMPLE "URI for LS Server example"
#define EX_URI_LL_CLIENT     URI_SCHEME_EXAMPLE "URI for Light Lightness Client example"
#define EX_URI_LL_SERVER     URI_SCHEME_EXAMPLE "URI for Light Lightness Setup Server example"
#define EX_URI_LC_SERVER     URI_SCHEME_EXAMPLE "URI for Light LC Setup Server example"
#define EX_URI_CTL_CLIENT    URI_SCHEME_EXAMPLE "URI for Light CTL Client example"
#define EX_URI_CTL_SERVER    URI_SCHEME_EXAMPLE "URI for Light CTL Setup Server example"
#define EX_URI_CTL_LC_SERVER URI_SCHEME_EXAMPLE "URI for Light CTL+LC Setup Servers example"
#define EX_URI_PBR_CLIENT    URI_SCHEME_EXAMPLE "URI for PB Remote Client example"
#define EX_URI_PBR_SERVER    URI_SCHEME_EXAMPLE "URI for PB Remote Server example"
#define EX_URI_SERIAL        URI_SCHEME_EXAMPLE "URI for Serial example"
#define EX_URI_SENSOR_SERVER URI_SCHEME_EXAMPLE "URI for Sensor Server example"
#define EX_URI_SENSOR_CLIENT URI_SCHEME_EXAMPLE "URI for Sensor Client example"
#define EX_URI_SCENE_CLIENT  URI_SCHEME_EXAMPLE "URI for Scene Client example"

/** Static authentication data. */
#define STATIC_AUTH_DATA {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

/** Static authentication data for remote provisioning example. */
#define STATIC_AUTH_DATA_PB_REMOTE {0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73}

#endif /* EXAMPLE_COMMON_H__ */
