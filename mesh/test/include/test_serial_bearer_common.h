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

#ifndef SERIAL_BEARER_UT_COMMON__
#define SERIAL_BEARER_UT_COMMON__

#include <stdint.h>
#include <signal.h>
#include <unity.h>
#include <cmock.h>
#include <stdlib.h>

#include "nrf_mesh_serial.h"
#include "serial_bearer.h"
#include "test_assert.h"

#include "serial_uart_mock.h"
#include "packet_buffer_mock.h"
#include "bearer_event_mock.h"
#include "serial_mock.h"

/* UART IRQ handler */
extern void UART0_IRQHandler(void);

/** Defines for slip encoding: see https://tools.ietf.org/html/rfc1055 */
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

extern bearer_event_callback_t m_serial_transmit;
extern uint32_t m_critical_section;
extern uint8_t test_data[sizeof(serial_packet_t)];
extern uint8_t m_buffer[UINT16_MAX];
extern void (*m_rx_cb)(uint8_t);
extern void (*m_tx_cb)(void);


/*********************************************************************
 * Helper functions                                                  *
 *********************************************************************/

void serial_buffer_get_calls(uint16_t packet_len, packet_buffer_packet_t ** pp_buf_packet);
void serial_transmit_packet_buf_calls(serial_packet_t * p_packet, packet_buffer_packet_t ** pp_buf_packet);
void serial_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet, packet_buffer_packet_t * p_buf_packet);
void serial_transmit(serial_packet_t * p_packet, packet_buffer_packet_t * p_buf_packet);
void transmit_bearer_event(packet_buffer_packet_t * p_buf_packet, bool success);
void receive_char(packet_buffer_packet_t ** pp_buf_packet, uint8_t val, bool check_len, bool sends_error_msg);

#endif /* end of SERIAL_BEARER_UT_COMMON__ */
