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

#ifndef HEALTH_CLIENT_H__
#define HEALTH_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>

#include "access.h"
#include "device_state_manager.h"
#include "health_common.h"

/**
 * @defgroup HEALTH_CLIENT Health Client
 * @ingroup HEALTH_MODEL
 * Model implementing the Health Client foundation model.
 * @{
 */

/** Acknowledged message transaction timeout */
#ifndef HEALTH_CLIENT_ACKED_TRANSACTION_TIMEOUT
#define HEALTH_CLIENT_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(60))
#endif

/** Object type for health client instances. */
typedef struct __health_client_t health_client_t;

/** Message types for messages received by the health client. */
typedef enum
{
    HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED,   /**< A Health Current Status message was received. */
    HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED,     /**< A Health Fault Status message was received. */
    HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED,    /**< A Health Period Status message was received. */
    HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED, /**< A Health Attention Status message was received. */
    HEALTH_CLIENT_EVT_TYPE_TIMEOUT,                   /**< A reliable transfer timed out. */
    HEALTH_CLIENT_EVT_TYPE_CANCELLED                  /**< A reliable transfer has been cancelled. */
} health_client_evt_type_t;

/** Fault status event. */
typedef struct
{
    uint8_t                          test_id;            /**< Most recent test run by the peer device. */
    uint16_t                         company_id;         /**< Company ID. */
    uint8_t                          fault_array_length; /**< Length of the fault array. */
    const uint8_t *                  p_fault_array;      /**< Fault array. Provides an array of fault codes. */
    const access_message_rx_meta_t * p_meta_data;        /**< Meta data for the incoming Fault Status or Current Status message. */
} health_client_evt_fault_status_t;

/** Period status event. */
typedef struct
{
    /**
     * Divisor for the publishing interval of the Current Status message in a server.
     * This is used to adjust the publishing interval when one or more faults are currently active.
     */
    uint8_t fast_period_divisor;
} health_client_evt_period_status_t;

/** Attention status event. */
typedef struct
{
    uint8_t attention; /**< Attention timer. This represents the remaining duration of the attention state of a server in seconds. */
} health_client_evt_attention_status_t;

/** Attention status event. */
typedef struct
{
    health_client_evt_type_t         type;                     /**< Type of the event. */
    const access_message_rx_meta_t * p_meta_data;              /**< Meta data for the received message. */
    union {
        health_client_evt_fault_status_t     fault_status;     /**< Fault status data for the Current Status and Fault Status messages. */
        health_client_evt_period_status_t    period_status;    /**< Period status data for the Period Status message. */
        health_client_evt_attention_status_t attention_status; /**< Attention status data for the Attention Status message. */
    } data;
} health_client_evt_t;

/**
 * Event callback function type.
 *
 * This function is called when the health client receives a message from a health server.
 *
 * @param[in] p_client Pointer to the health client instance structure.
 * @param[in] p_event  Pointer to a structure containing information about the received event.
 */
typedef void (*health_client_evt_cb_t)(const health_client_t * p_client, const health_client_evt_t * p_event);

/** Health client instance structure. */
struct __health_client_t
{
    access_model_handle_t  model_handle;      /**< Model handle. */
    health_client_evt_cb_t event_handler;     /**< Event handler. */
    bool                   waiting_for_reply; /**< Set to @c true if the client is currently waiting for a reply to a transmitted message. */
    uint8_t *              p_buffer;          /**< Buffer used to hold an outbound message. */
};

/**
 * Requests the current fault status from a server.
 *
 * @note Response event: @ref HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED
 *
 * @param[in,out] p_client   Pointer to the client instance structure.
 * @param[in]     company_id Company ID of of the server.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_fault_get(health_client_t * p_client, uint16_t company_id);

/**
 * Clears the fault array for a specified company ID.
 *
 * @note Response event if @p acked is @c true: @ref HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED
 *
 * @param[in,out] p_client   Pointer to the client instance structure.
 * @param[in]     company_id Company ID of of the server.
 * @param[in]     acked      Whether to send the message as an acknowledged message or an unacknowledged message.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_fault_clear(health_client_t * p_client, uint16_t company_id, bool acked);

/**
 * Requests a server to run a self-test.
 *
 * @note Response event if @p acked is @c true: @ref HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED
 *
 * @param[in,out] p_client   Pointer to the client instance structure.
 * @param[in]     company_id Company ID of of the server.
 * @param[in]     test_id    ID of the self-test to run.
 * @param[in]     acked      Whether to send the message as an acknowledged message or an unacknowledged message.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_fault_test(health_client_t * p_client, uint16_t company_id, uint8_t test_id, bool acked);

/**
 * Gets the current health period state of an element.
 *
 * @note Response event: @ref HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED
 *
 * @param[in,out] p_client   Pointer to the client instance structure.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_period_get(health_client_t * p_client);

/**
 * Sets the health period state of an element.
 *
 * @note Response event if @p acked is @c true: @ref HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED
 *
 * @param[in,out] p_client            Pointer to the client instance structure.
 * @param[in]     fast_period_divisor Value of the fast period divisor.
 * @param[in]     acked               Whether to send the message as an acknowledged message or an unacknowledged message.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_period_set(health_client_t * p_client, uint8_t fast_period_divisor, bool acked);

/**
 * Gets the attention timer from a server.
 *
 * @note Response event: @ref HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED
 *
 * @param[in,out] p_client    Pointer to the client instance structure.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_attention_get(health_client_t * p_client);

/**
 * Sets the attention timer on a server.
 *
 * @note Response event if @p acked is @c true: @ref HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED
 *
 * @param[in,out] p_client        Pointer to the client instance structure.
 * @param[in]     attention_timer How long the attention state should be active in the server in seconds.
 * @param[in]     acked           Whether to send the message as an acknowledged message or an unacknowledged message.
 *
 * @retval NRF_SUCCESS    The message was successfully sent to the server.
 * @retval NRF_ERROR_BUSY The client is currently waiting for a reply from a server.
 */
uint32_t health_client_attention_set(health_client_t * p_client, uint8_t attention_timer, bool acked);

/**
 * Initializes a health client instance.
 *
 * @param[in,out] p_client      Pointer to the client instance structure.
 * @param[in]     element_index Index of the element to register the model with.
 * @param[in]     evt_handler   Event handler used to process incoming messages.
 *
 * @retval NRF_SUCCESS      The health client was successfully initialized.
 * @retval NRF_ERROR_NULL   The pointer passed as the @c evt_handler argument was @c NULL.
 * @retval NRF_ERROR_NO_MEM There was not enough memory available to allocate a subscription list for the model.
 *
 * @see access_model_add()
 */
uint32_t health_client_init(health_client_t * p_client, uint16_t element_index, health_client_evt_cb_t evt_handler);

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in,out] p_client    Pointer to the client instance structure.
 */
void health_client_pending_msg_cancel(health_client_t * p_client);

/** @} */

#endif

