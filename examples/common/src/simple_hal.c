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

#include "simple_hal.h"
#include "nrf_mesh_config_examples.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_error.h"
#include "boards.h"
#include "nrf_delay.h"

#include "nrf_mesh_defines.h"
#include "timer.h"
#include "app_timer.h"
#include "app_error.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#if SIMPLE_HAL_LEDS_ENABLED
#define LED_PIN_CONFIG ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)   | \
                        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       | \
                        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     | \
                        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | \
                        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos))
#endif

#if BUTTON_BOARD
#define BUTTON_PIN_CONFIG ((GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)     | \
                           (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)    | \
                           (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)                 | \
                           (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
                           (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos))
#endif

#define GPIOTE_IRQ_LEVEL NRF_MESH_IRQ_PRIORITY_LOWEST

/*****************************************************************************
 * Static variables
 *****************************************************************************/

#if BUTTON_BOARD
static uint8_t m_buttons_list[BUTTONS_NUMBER] = BUTTONS_LIST;
static uint32_t m_last_button_press;
static hal_button_handler_cb_t m_button_handler_cb;
#endif

#if SIMPLE_HAL_LEDS_ENABLED
APP_TIMER_DEF(m_blink_timer);
static uint32_t m_blink_count;
static uint32_t m_blink_mask;
static uint32_t m_prev_state;
#endif

/*****************************************************************************
 * Public API
 *****************************************************************************/

#if SIMPLE_HAL_LEDS_ENABLED
static void led_timeout_handler(void * p_context)
{
    APP_ERROR_CHECK_BOOL(m_blink_count > 0);
    NRF_GPIO->OUT ^= m_blink_mask;

    m_blink_count--;
    if (m_blink_count == 0)
    {
        (void) app_timer_stop(m_blink_timer);
        NRF_GPIO->OUT = m_prev_state;
    }
}

/** Returns @c true if the led at pin_no is on. */
bool hal_led_pin_get(uint32_t pin)
{
    /* If pin_no is set _low_ (0) the led is on. */
    return ((NRF_GPIO->OUT & (1 << pin)) == 0);
}

void hal_led_pin_set(uint32_t pin, bool value)
{
    if (value)
    {
        NRF_GPIO->OUTCLR = (1 << pin);
    }
    else
    {
        NRF_GPIO->OUTSET = (1 << pin);
    }
}

void hal_led_mask_set(uint32_t led_mask, bool value)
{
    if (value)
    {
        NRF_GPIO->OUTCLR = led_mask;
    }
    else
    {
        NRF_GPIO->OUTSET = led_mask;
    }
}

void hal_led_blink_ms(uint32_t led_mask, uint32_t delay_ms, uint32_t blink_count)
{
    if (blink_count == 0 || delay_ms < HAL_LED_BLINK_PERIOD_MIN_MS)
    {
        return;
    }

    m_blink_mask  = led_mask;
    m_blink_count = blink_count * 2 - 1;
    m_prev_state = NRF_GPIO->OUT;

    if (app_timer_start(m_blink_timer, APP_TIMER_TICKS(delay_ms), NULL) == NRF_SUCCESS)
    {
        /* Start by "clearing" the mask, i.e., turn the LEDs on -- in case a user calls the
         * function twice. */
        NRF_GPIO->OUT &= ~m_blink_mask;
    }
}

void hal_led_blink_stop(void)
{
    (void) app_timer_stop(m_blink_timer);
    hal_led_mask_set(m_blink_mask, LED_MASK_STATE_OFF);
}

void hal_leds_init(void)
{
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
    {
        NRF_GPIO->PIN_CNF[i] = LED_PIN_CONFIG;
        NRF_GPIO->OUTSET = 1UL << i;
    }

    APP_ERROR_CHECK(app_timer_create(&m_blink_timer, APP_TIMER_MODE_REPEATED, led_timeout_handler));
}
#endif /* SIMPLE_HAL_LEDS_ENABLED */

uint32_t hal_buttons_init(hal_button_handler_cb_t cb)
{
#if !BUTTON_BOARD
    return NRF_ERROR_NOT_SUPPORTED;
#else
    if (cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    m_button_handler_cb = cb;

    for (uint32_t i = 0; i < BUTTONS_NUMBER ; ++i)
    {
        NRF_GPIO->PIN_CNF[m_buttons_list[i]] = BUTTON_PIN_CONFIG;
    }

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NRF_GPIOTE->EVENTS_PORT  = 0;

    NVIC_SetPriority(GPIOTE_IRQn, GPIOTE_IRQ_LEVEL);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    return NRF_SUCCESS;
#endif
}

/*****************************************************************************
 * IRQ handler(s)
 *****************************************************************************/

#if BUTTON_BOARD
void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    for (uint8_t i = 0; i < BUTTONS_NUMBER; ++i)
    {
        /* Check that the event was generated by a button press, and reject if it's too soon (debounce).
         * NOTE: There is a bug with this at the wrap-around for the RTC1 where the button could be
         * pressed before HAL_BUTTON_PRESS_FREQUENCY has passed a single time. It doesn't matter practically.
         */
        if ((~NRF_GPIO->IN & (1 << (m_buttons_list[i]))) &&
            TIMER_DIFF(m_last_button_press, NRF_RTC1->COUNTER) > HAL_BUTTON_PRESS_FREQUENCY)
        {
            m_last_button_press = NRF_RTC1->COUNTER;
            m_button_handler_cb(i);
        }
    }
}
#endif
