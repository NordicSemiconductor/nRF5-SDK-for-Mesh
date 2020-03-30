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

#ifndef SERIAL_UART_H__
#define SERIAL_UART_H__

#include <stdint.h>
#include "nrf_mesh_serial.h"

/**
 * @defgroup SERIAL_UART Serial UART bearer
 * @ingroup MESH_SERIAL
 * @{
 */

/**
 * Serial UART RX callback type.
 * Called upon reception of a byte on the UART interface.
 *
 * @param[in] byte The received byte on the UART interface.
 */
typedef void (*serial_uart_rx_cb_t)(uint8_t byte);

/**
 * Serial UART TX callback type.
 * Called after each successful transfer.
 */
typedef void (*serial_uart_tx_cb_t)(void);

/**
 * Initializes the serial interface.
 *
 * @param[in] rx_cb The receive callback.
 * @param[in] tx_cb The transmit callback.
 *
 * @retval NRF_SUCCESS    The UART is successfully initialized.
 * @retval NRF_ERROR_NULL None of the parameters can be an invalid pointer.
 */
uint32_t serial_uart_init(serial_uart_rx_cb_t rx_cb, serial_uart_tx_cb_t tx_cb);

/**
 * Sends and receives any pending data on the UART line if possible.
 */
void serial_uart_process(void);

/**
 * Enable/disable reception of data from the peer device.
 *
 * @param[in] enable_rx Set to @c true in order to enable the reception of data form peer.
 */
void serial_uart_receive_set(bool enable_rx);

/** Starts a transfer. */
static inline void serial_uart_tx_start(void)
{
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TASKS_STARTTX = 1;
}

/** Stops a transfer. */
static inline void serial_uart_tx_stop(void)
{
    NRF_UART0->TASKS_STOPTX = 1;
}

/** Sets the next byte to send. */
static inline void serial_uart_byte_send(uint8_t value)
{
    NRF_UART0->TXD = value;
}

/** @} end of SERIAL_UART */

#endif
