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

#include <unity.h>
#include <cmock.h>

#include "serial_uart.h"

#include "fifo_mock.h"

NRF_UART_Type m_uart;
NRF_UART_Type* NRF_UART0 = &m_uart;
NRF_GPIO_Type m_gpio;
NRF_GPIO_Type* NRF_GPIO = &m_gpio;

void setUp(void)
{
}

void tearDown(void)
{

}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_init_params(void)
{
    nrf_mesh_serial_uart_params_t init_params;

    /* Happy path! */
    init_params.baudrate =  4321432;
    init_params.flow_control = true;
    init_params.pin_cts = 0x01;
    init_params.pin_rts = 0x02;
    init_params.pin_rx  = 0x03;
    init_params.pin_tx  = 0x04;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, serial_uart_init(&init_params));
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_cts, NRF_UART0->PSELCTS);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_rts, NRF_UART0->PSELRTS);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_rx,  NRF_UART0->PSELRXD);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_tx,  NRF_UART0->PSELTXD);
    TEST_ASSERT_EQUAL_HEX32(init_params.baudrate, NRF_UART0->BAUDRATE);
    TEST_ASSERT_EQUAL_HEX32(init_params.flow_control, !!(NRF_UART0->CONFIG & UART_CONFIG_HWFC_Msk));
    TEST_ASSERT_EQUAL_HEX32(0, NRF_UART0->CONFIG & UART_CONFIG_PARITY_Msk);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, serial_uart_init(&init_params));
}

void test_tx(void)
{

}

void test_rx(void)
{

}
