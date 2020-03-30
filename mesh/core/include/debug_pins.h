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
#ifndef DEBUG_PINS_H__
#define DEBUG_PINS_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

/* Define debug pins that fit the standard development kits for the two chips. Placed to avoid
 * overlapping functionality. */
#ifdef NRF51
#define DEBUG_PIN0 12
#define DEBUG_PIN1 13
#define DEBUG_PIN2 14
#define DEBUG_PIN3 15
#define DEBUG_PIN4 16
#define DEBUG_PIN5 17
#define DEBUG_PIN6 18
#define DEBUG_PIN7 19
#elif defined(NRF52_SERIES) || defined(HOST)
#define DEBUG_PIN0 22
#define DEBUG_PIN1 23
#define DEBUG_PIN2 24
#define DEBUG_PIN3 25
#define DEBUG_PIN4 26
#define DEBUG_PIN5 27
#define DEBUG_PIN6 28
#define DEBUG_PIN7 29
#endif

#define DEBUG_PIN_UINT_BIT_SET                  DEBUG_PIN6
#define DEBUG_PIN_UINT_BIT_CLEARED              DEBUG_PIN7

#ifdef DEBUG_PINS_ENABLED

#define DEBUG_PIN_INIT(PIN_NO)                                                                              \
    do                                                                                                      \
    {                                                                                                       \
        NRF_GPIO->PIN_CNF[PIN_NO] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)       \
                                  | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)     \
                                  | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)      \
                                  | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)     \
                                  | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);    \
        DEBUG_PIN_TOGGLE(PIN_NO);                                                                           \
    } while (0)

#define DEBUG_PINS_INIT()               \
    do                                  \
    {                                   \
        DEBUG_PIN_INIT(DEBUG_PIN0);     \
        DEBUG_PIN_INIT(DEBUG_PIN1);     \
        DEBUG_PIN_INIT(DEBUG_PIN2);     \
        DEBUG_PIN_INIT(DEBUG_PIN3);     \
        DEBUG_PIN_INIT(DEBUG_PIN4);     \
        DEBUG_PIN_INIT(DEBUG_PIN5);     \
        DEBUG_PIN_INIT(DEBUG_PIN6);     \
        DEBUG_PIN_INIT(DEBUG_PIN7);     \
    } while (0)

#define DEBUG_PIN_ON(p)               do { NRF_GPIO->OUTSET = (1UL << (p)); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define DEBUG_PIN_OFF(p)              do { NRF_GPIO->OUTCLR = (1UL << (p)); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define DEBUG_PIN_TOGGLE(p)           do { DEBUG_PIN_ON(p); DEBUG_PIN_OFF(p); } while(0)
#define DEBUG_PIN_TOGGLE_NTIMES(p, n) do { uint8_t CTR = (n); while (CTR--) { DEBUG_PIN_TOGGLE(p); } } while(0)

#define DEBUG_PIN_UINT32(n)                                     \
    do                                                          \
    {                                                           \
        uint32_t N = (n);                                       \
        for (int i = 32; i-- > 0;)                              \
        {                                                       \
            if (N & (1UL << i))                                 \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_SET);       \
            }                                                   \
            else                                                \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_CLEARED);   \
            }                                                   \
        }                                                       \
    } while (0)

#define DEBUG_PIN_UINT16(n)                                     \
    do                                                          \
    {                                                           \
        uint32_t N = (n);                                       \
        for (int i = 16; i-- > 0;)                              \
        {                                                       \
            if (N & (1UL << i))                                 \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_SET);       \
            }                                                   \
            else                                                \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_CLEARED);   \
            }                                                   \
        }                                                       \
    } while (0)

#define DEBUG_PIN_UINT8(n)                                      \
    do                                                          \
    {                                                           \
        uint32_t N = (n);                                       \
        for (int i = 8; i-- > 0;)                               \
        {                                                       \
            if (N & (1UL << i))                                 \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_SET);       \
            }                                                   \
            else                                                \
            {                                                   \
                DEBUG_PIN_TOGGLE(DEBUG_PIN_UINT_BIT_CLEARED);   \
            }                                                   \
        }                                                       \
    } while (0)

/* The inline function auto-generator. This macro should be called for each user of the debug_pins. */
#define DEBUG_PIN_MODULE(MODULE, ENABLED) \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_ON(uint32_t pin)  { if (ENABLED) { DEBUG_PIN_ON(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_OFF(uint32_t pin) { if (ENABLED) { DEBUG_PIN_OFF(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE(uint32_t pin) { if (ENABLED) { DEBUG_PIN_TOGGLE(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE_NTIMES(uint32_t pin, uint32_t n) { if (ENABLED) { DEBUG_PIN_TOGGLE_NTIMES(pin, n); } }

#else

#define DEBUG_PINS_INIT() do {} while (0)

#define DEBUG_PIN_ON(p)
#define DEBUG_PIN_OFF(p)
#define DEBUG_PIN_TOGGLE(p)
#define DEBUG_PIN_TOGGLE_NTIMES(p, n)

#define DEBUG_PIN_MODULE(MODULE, ENABLED) \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_ON(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_OFF(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE_NTIMES(uint32_t pin, uint32_t n) {}

#endif //DEBUG_PINS_ENABLED

DEBUG_PIN_MODULE(TIMESLOT, false)
#define DEBUG_PIN_TS_SIGNAL_CALLBACK         DEBUG_PIN0
#define DEBUG_PIN_TS_IN_TIMESLOT             DEBUG_PIN1
#define DEBUG_PIN_TS_EXTEND_HANDLER          DEBUG_PIN2
#define DEBUG_PIN_TS_TIMER_HANDLER           DEBUG_PIN3
#define DEBUG_PIN_TS_SD_EVT_HANDLER          DEBUG_PIN4
#define DEBUG_PIN_TS_END_TIMER_HANDLER       DEBUG_PIN5
#define DEBUG_PIN_TS_HIGH_PRIORITY           DEBUG_PIN6
#define DEBUG_PIN_TS_EXTEND_SUCCEEDED        DEBUG_PIN7

DEBUG_PIN_MODULE(BEARER_HANDLER, false)
#define DEBUG_PIN_BEARER_HANDLER_ACTION         DEBUG_PIN0
#define DEBUG_PIN_BEARER_HANDLER_SCANNER        DEBUG_PIN1
#define DEBUG_PIN_BEARER_HANDLER_SCANNER_STOP   DEBUG_PIN2
#define DEBUG_PIN_BEARER_HANDLER_TIMER_SETUP    DEBUG_PIN3

DEBUG_PIN_MODULE(SCANNER, false)
#define DEBUG_PIN_SCANNER_RADIO_IN_RX       DEBUG_PIN0
//#define DEBUG_PIN_SCANNER_                  DEBUG_PIN1
//#define DEBUG_PIN_SCANNER_                  DEBUG_PIN2
#define DEBUG_PIN_SCANNER_END_EVENT         DEBUG_PIN3
#define DEBUG_PIN_SCANNER_IN_ACTION         DEBUG_PIN4
#define DEBUG_PIN_SCANNER_START             DEBUG_PIN5
#define DEBUG_PIN_SCANNER_STOP              DEBUG_PIN6
#define DEBUG_PIN_SCANNER_RADIO_IRQ         DEBUG_PIN7

DEBUG_PIN_MODULE(BROADCAST, false)
#define DEBUG_PIN_BROADCAST_START           DEBUG_PIN0
#define DEBUG_PIN_BROADCAST_RADIO_EVT       DEBUG_PIN1
#define DEBUG_PIN_BROADCAST_RADIO_EVT_END   DEBUG_PIN2
#define DEBUG_PIN_BROADCAST_RADIO_EVT_READY DEBUG_PIN3
#define DEBUG_PIN_BROADCAST_ADV_CB          DEBUG_PIN4
#define DEBUG_PIN_BROADCAST_ACTIVE          DEBUG_PIN5


DEBUG_PIN_MODULE(INSTABURST, false)
#define DEBUG_PIN_INSTABURST_START           DEBUG_PIN0
#define DEBUG_PIN_INSTABURST_RADIO_EVT       DEBUG_PIN1
#define DEBUG_PIN_INSTABURST_IN_ACTION       DEBUG_PIN2
#define DEBUG_PIN_INSTABURST_TX_BUFFER       DEBUG_PIN3
#define DEBUG_PIN_INSTABURST_ORDER_TX        DEBUG_PIN4
#define DEBUG_PIN_INSTABURST_SETUP_ADV_EXT_IND DEBUG_PIN5
#define DEBUG_PIN_INSTABURST_RADIO_ACTIVE    DEBUG_PIN7

#define DEBUG_PIN_INSTABURST_GOT_ADV_EXT     DEBUG_PIN3
#define DEBUG_PIN_INSTABURST_RX_OK           DEBUG_PIN4
#define DEBUG_PIN_INSTABURST_TOO_SLOW        DEBUG_PIN5
#define DEBUG_PIN_INSTABURST_KILL_TIMER      DEBUG_PIN7
//#define DEBUG_PIN_INSTABURST_ACTIVE          DEBUG_PIN5

#endif //DEBUG_PINS_H__
