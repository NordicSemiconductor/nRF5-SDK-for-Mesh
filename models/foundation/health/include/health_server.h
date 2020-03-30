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

#ifndef HEALTH_SERVER_H__
#define HEALTH_SERVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "access.h"
#include "bitfield.h"

#include "health_common.h"
#include "nrf_mesh_config_app.h"

/**
 * @defgroup HEALTH_SERVER Health Server
 * @ingroup HEALTH_MODEL
 * Model implementing the Health Server foundation model.
 * @{
 */

/** Size of the health server fault array. */
#define HEALTH_SERVER_FAULT_ARRAY_SIZE  256

/** Object type for health server instances. */
typedef struct __health_server_t health_server_t;

/**
 * Callback function for the attention state.
 * The attention state is enabled when a health client sets the attention timer to a nonzero
 * value, and disabled when the attention timer reaches zero.
 * @param[in] p_server        Pointer to the health server instance structure.
 * @param[in] attention_state @c true if the device should enable the attention state, @c false
 *                            if the device should disable the attention state.
 */
typedef void (*health_server_attention_cb_t)(const health_server_t * p_server, bool attention_state);

/**
 * Self-test function type.
 *
 * A self-test should perform a specific test and set or clear corresponding fault(s) in the fault array
 * using the health_server_fault_register() or health_server_fault_clear() functions.
 *
 * @param[in] p_server   Pointer to the server instance structure.
 * @param[in] company_id Company ID of the requested test.
 * @param[in] test_id    ID of the test that should be run.
 *
 * @see health_server_fault_register(), health_server_fault_clear(), health_server_fault_is_set()
 */
typedef void (*health_server_selftest_cb_t)(health_server_t * p_server, uint16_t company_id, uint8_t test_id);

/**
 * Structure defining a self-test function.
 * An array of these structs should be passed to the health server on initialization if self-tests are
 * supported by the device.
 */
typedef struct
{
    uint8_t                     test_id;            /**< Self-test ID. */
    health_server_selftest_cb_t selftest_function;  /**< Pointer to the self-test function. */
} health_server_selftest_t;

/** Health server fault array type. */
typedef uint32_t health_server_fault_array_t[BITFIELD_BLOCK_COUNT(HEALTH_SERVER_FAULT_ARRAY_SIZE)];

/**
 * Health server instance structure.
 */
struct __health_server_t
{
    access_model_handle_t            model_handle;          /**< Model handle. */
    uint8_t                          fast_period_divisor;   /**< Fast period divisor, used to increase publishing interval when faults are present. */
    const health_server_selftest_t * p_selftests;           /**< Pointer to an array of self-tests. */
    uint8_t                          num_selftests;         /**< Number of self-tests in @c p_selftests. */
    uint8_t                          previous_test_id;      /**< ID of the latest self-test run by the model. */
    uint16_t                         company_id;            /**< Health server company ID. */
    health_server_fault_array_t      registered_faults;     /**< Array of registered faults. */
    health_server_fault_array_t      current_faults;        /**< Array of current faults. */
    health_server_attention_cb_t     attention_handler;     /**< Handler for the attention state. If @c NULL, the attention state is unsupported. */
    uint8_t                          attention_timer;       /**< Timer for the attention state. */
    struct __health_server_t *       p_next;                /**< Pointer to the next instance. Used internally for supporting the attention timer. */
};

/**
 * Registers a fault in the current fault array.
 *
 * If the device is publishing a periodic Current Status message and there are no active faults
 * _before_ this function is called, the model will enable fast publishing of the Current Status
 * message. This is done by dividing the current publishing interval with 2<sup>fast period interval</sup>
 * and using the result as the new publishing interval.
 *
 * @note The fast publishing interval will never go lower than 100 ms.
 *
 * @param[in,out] p_server   Pointer to a server instance.
 * @param[in]     fault_code ID of the fault to register.
 *
 * @see health_server_fault_clear()
 */
void health_server_fault_register(health_server_t * p_server, uint8_t fault_code);

/**
 * Clears a fault from the current fault array.
 *
 * If the device is currently publishing a periodic Current Status message with a fast publishing
 * period, and there are no active faults _after_ this function is called, the publishing interval
 * will be reset back to the original interval.
 *
 * @param[in,out] p_server   Pointer to a server instance.
 * @param[in]     fault_code ID of the fault to clear.
 *
 * @see health_server_fault_register()
 */
void health_server_fault_clear(health_server_t * p_server, uint8_t fault_code);

/**
 * Checks if a fault code is set in current fault array.
 * @param[in,out] p_server   Pointer to a server instance.
 * @param[in]     fault_code ID of the fault to check for.
 * @return Returns @c true if the specified fault is set and @c false otherwise.
 */
bool health_server_fault_is_set(health_server_t * p_server, uint8_t fault_code);

/**
 * Gets the number of currently set faults in the fault array.
 * @param[in] p_server Pointer to a server instance.
 * @return Returns the number of currently set faults in the fault array.
 */
uint8_t health_server_fault_count_get(const health_server_t * p_server);

/**
 * Gets the current value of the attention timer.
 * @param[in] p_server Pointer to a server instance.
 * @return Returns the current value of the attention timer.
 */
uint8_t health_server_attention_get(const health_server_t * p_server);

/**
 * Sets the attention timer value.
 * @param[in] p_server  Pointer to a server instance.
 * @param[in] attention New value for the attention timer.
 */
void health_server_attention_set(health_server_t * p_server, uint8_t attention);

/**
 * Initializes the health server model.
 *
 * @param[in] p_server      Pointer to the server instance structure.
 * @param[in] element_index Element index to use when registering the health server.
 * @param[in] company_id    Company ID supported by this health model server.
 * @param[in] attention_cb  Callback function for enabling or disabling the attention state for
 *                          the device. If @c NULL is passed as this argument, the attention state
 *                          is considered to be unsupported by the model, and the attention timer
 *                          is disabled.
 * @param[in] p_selftests   Pointer to an array of self-tests that will be supported by this device.
 *                          If @c NULL is passed as this argument, self-tests will be unsupported
 *                          by the model. In this case, the @c num_selftests parameter will be ignored.
 * @param[in] num_selftests Number of self-tests provided by @c p_selftests.
 *
 * @retval NRF_SUCCESS The model was successfully initialized.
 *
 * @see access_model_add()
 */
uint32_t health_server_init(health_server_t * p_server, uint16_t element_index, uint16_t company_id,
        health_server_attention_cb_t attention_cb,
        const health_server_selftest_t * p_selftests, uint8_t num_selftests);

/** @} */

#endif

