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

#include "serial_uart.h"
#include "nrf_mesh_config_serial_uart.h"

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_defines.h"
#include "nrf_gpio.h"

#define UART_IRQ_LEVEL NRF_MESH_IRQ_PRIORITY_LOWEST

/********** Static variables **********/
static bool m_can_receive;
static serial_uart_rx_cb_t m_char_rx_cb;
static serial_uart_tx_cb_t m_char_tx_cb;

/********** Interrupt handlers **********/

void UART0_IRQHandler(void)
{
    serial_uart_process();
}

/********** Interface Functions **********/
uint32_t serial_uart_init(serial_uart_rx_cb_t rx_cb, serial_uart_tx_cb_t tx_cb)
{
    if (rx_cb == NULL || tx_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    m_char_rx_cb = rx_cb;
    m_char_tx_cb = tx_cb;

    /* Set up GPIOs: */
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(CTS_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_pin_set(TX_PIN_NUMBER);
    nrf_gpio_cfg_output(RTS_PIN_NUMBER);

    /* Initialize UART hardware: */
    NRF_UART0->PSELTXD = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD = RX_PIN_NUMBER;
    NRF_UART0->PSELCTS = CTS_PIN_NUMBER;
    NRF_UART0->PSELRTS = RTS_PIN_NUMBER;
    NRF_UART0->CONFIG  = (HWFC ? UART_CONFIG_HWFC_Enabled : UART_CONFIG_HWFC_Disabled) << UART_CONFIG_HWFC_Pos;
    NRF_UART0->BAUDRATE = SERIAL_UART_BAUDRATE << UART_BAUDRATE_BAUDRATE_Pos;
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos;
    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk | UART_INTENSET_TXDRDY_Msk;

    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TASKS_STARTRX = 1;
    NVIC_SetPriority(UART0_IRQn, UART_IRQ_LEVEL);
    NVIC_EnableIRQ(UART0_IRQn);

    return NRF_SUCCESS;
}

void serial_uart_process(void)
{
    /* receive all pending bytes */
    while (m_can_receive && NRF_UART0->EVENTS_RXDRDY)
    {
        NRF_UART0->EVENTS_RXDRDY = 0;
        (void) NRF_UART0->EVENTS_RXDRDY;
        if (m_char_rx_cb != NULL)
        {
            m_char_rx_cb(NRF_UART0->RXD);
        }
    }

    /* transmit any pending bytes */
    if (NRF_UART0->EVENTS_TXDRDY)
    {
        NRF_UART0->EVENTS_TXDRDY = 0;
        (void) NRF_UART0->EVENTS_TXDRDY;
        if (m_char_tx_cb != NULL)
        {
            m_char_tx_cb();
        }
    }
}

void serial_uart_receive_set(bool enable_rx)
{
    m_can_receive = enable_rx;
}
