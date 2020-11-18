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

#ifndef SIMPLE_HAL_H__
#define SIMPLE_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "hal.h"
#include "boards.h"
#include "utils.h"

/**
 * @defgroup SIMPLE_HAL Simple Hardware Abstraction Layer
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Simple hardware abstraction layer for the example applications. This module uses GPIOTE
 * driver module. Therefore, `GPIOTE_ENABLED` must be set to `1` in the `app_config.h` file.
 *
 * @{
 */

/** Acceptable button press interval in microseconds. */
#define HAL_BUTTON_PRESS_DEBOUNCE_INTERVAL  MS_TO_US(400)

/** Lowest possible blinking period in milliseconds. */
#define HAL_LED_BLINK_PERIOD_MIN_MS (20)

/** Set LED Mask state to Off. */
#define LED_MASK_STATE_OFF    (false)
/** Set LED Mask state to On. */
#define LED_MASK_STATE_ON     (true)

/** LEDs mask full */
#define HAL_LED_MASK ((1 << LEDS_NUMBER) - 1)
/** LEDs mask half **/
#if (LEDS_NUMBER == 1)
    #define HAL_LED_MASK_HALF       (HAL_LED_MASK)
    #define HAL_LED_MASK_LOWER_HALF (HAL_LED_MASK)
    #define HAL_LED_MASK_UPPER_HALF (HAL_LED_MASK)
#else
    #define HAL_LED_MASK_HALF       ((1 << (LEDS_NUMBER/2)) - 1)
    #define HAL_LED_MASK_LOWER_HALF (HAL_LED_MASK_HALF)
    #define HAL_LED_MASK_UPPER_HALF (HAL_LED_MASK ^ HAL_LED_MASK_HALF)
#endif

/** Boards with user buttons */
#define BUTTON_BOARD (defined(BOARD_PCA10040) || defined(BOARD_PCA10028) || defined(BOARD_PCA10056) \
                     || defined(BOARD_PCA10100) || defined(BOARD_PCA10059)) //lint -e491 // Suppress "non-standard use of 'defined' preprocessor operator"

/**
 * Button event handler callback type.
 * @param[in] button_number Button number (0-3).
 */
typedef void (*hal_button_handler_cb_t)(uint32_t button_number);

/** Initializes the LEDs. */
void hal_leds_init(void);

/**
 * Initializes the buttons on a DK.
 *
 * To use this API, `GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS` must be set to a value greater than or
 * equal to `BUTTONS_NUMBER` in `app_config.h` file to support buttons on the DK board.
 *
 * @param[in] cb Button event callback.
 *
 * @retval NRF_SUCCESS             Successfully initialized buttons.
 * @retval NRF_ERROR_NULL          Callback was NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED Buttons not supported for this board.
 */
uint32_t hal_buttons_init(hal_button_handler_cb_t cb);


/**
 * Sets the LED for the given PIN.
 * @param[in] pin   LED pin number.
 * @param[in] value @c true for on, @c false for off.
 */
void hal_led_pin_set(uint32_t pin, bool value);

/**
 * Sets the LEDs for the given mask.
 * @param[in] led_mask  Mask of LED pins to set/clear.
 * @param[in] value @c true for on, @c false for off.
 */
void hal_led_mask_set(uint32_t led_mask, bool value);

/**
 * Gets the current state of a (LED) pin.
 * @note The LEDs are active low, i.e., on when it's GPIO is set low.
 *
 * @param[in] pin Pin to get state of.
 *
 * @returns @c true if the LED is on, @c false otherwise.
 */
bool hal_led_pin_get(uint32_t pin);

/**
 * Blinks (one toggle cycle) pin_mask a specified number of times.
 *
 * @note If the API is called twice, the blink sequence is reset.
 * @note If @p delay_ms is less than @ref HAL_LED_BLINK_PERIOD_MIN_MS or @p blink_count is zero, the
 * call will be ignored.
 * @note If the APP_TIMER queue is full, this call may fail silently.
 *
 * @param[in] pin_mask      Mask of LED pins.
 * @param[in] delay_ms      Delay in milliseconds between each state change.
 * @param[in] blink_count   Number of times to blink.
 */
void hal_led_blink_ms(uint32_t pin_mask, uint32_t delay_ms, uint32_t blink_count);

/**
 * Stops blinking the LEDs (previously started by @ref hal_led_blink_ms).
 *
 * @note Sets the LED mask from the @ref hal_led_blink_ms call to off.
 */
void hal_led_blink_stop(void);

/** @} end of SIMPLE_HAL */


#endif /* SIMPLE_HAL_H__ */
