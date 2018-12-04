/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#endif /* EXAMPLE_COMMON_H__ */
