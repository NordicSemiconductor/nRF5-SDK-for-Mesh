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

#include "test_serial_bearer_common.h"
#include <limits.h>

/*********************************************************************
 * Tests                                                             *
 *********************************************************************/
void test_uart_tx(void)
{
    serial_packet_t * p_packet;
    packet_buffer_packet_t * p_buf_packet = (packet_buffer_packet_t *) m_buffer;
    packet_buffer_packet_t * p_buf_packet2 = (packet_buffer_packet_t *) &m_buffer[ALIGN_VAL(sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t), 4)];
    p_packet = (serial_packet_t*)test_data;
    packet_buffer_reserve_ExpectAndReturn(NULL, NULL, sizeof(test_data), NRF_SUCCESS);
    packet_buffer_reserve_IgnoreArg_p_buffer();
    packet_buffer_reserve_IgnoreArg_pp_packet();
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_buf_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_bearer_blocking_buffer_get(sizeof(test_data)-1, &p_packet));
    /* p_packet should have been changed */
    TEST_ASSERT_NOT_EQUAL(test_data, (uint8_t*)p_packet);

    p_packet->opcode = SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP;
    serial_buffer_get(sizeof(test_data)-1, &p_packet, p_buf_packet);
    serial_transmit(p_packet, p_buf_packet);
    TEST_ASSERT_NOT_EQUAL(test_data, (uint8_t*)p_packet);
    TEST_ASSERT_EQUAL(p_buf_packet->packet, p_packet);
    /* Copy the contents of test_data */
    memcpy(&p_packet->opcode, test_data, p_packet->length);
    /* State IDLE so can start transmit */
    transmit_bearer_event(p_buf_packet, true);
    TEST_ASSERT_EQUAL(sizeof(test_data)-1, NRF_UART0->TXD);
    /* Second call to transmit produces nothing since we are already in TRANSMIT state*/
    transmit_bearer_event(p_buf_packet, false);
    for (uint32_t i = 0; i < sizeof(test_data)-2; ++i)
    {
        /* Return and event that data is sent */
        m_tx_cb();
        TEST_ASSERT_EQUAL(test_data[i], NRF_UART0->TXD);
        /* Calls to transmit should cause no effect while in transmit state*/
        transmit_bearer_event(p_buf_packet, false);
    }
    /* Last byte sent, should trigger a free, no slip code should be sent.*/
    packet_buffer_free_Expect(NULL, p_buf_packet);
    packet_buffer_free_IgnoreArg_p_buffer();
    m_tx_cb();
    TEST_ASSERT_EQUAL(test_data[sizeof(test_data)-2], NRF_UART0->TXD);
    /* Calls to transmit should cause no effect while in transmit state*/
    transmit_bearer_event(p_buf_packet, false);

    /* No more data to send.*/
    m_tx_cb();
    TEST_ASSERT_EQUAL(1, NRF_UART0->TASKS_STOPTX);
    NRF_UART0->TASKS_STOPTX = 0;

    /* Now that the transfer finished and we are back in idle state and we can start another transmission: */
    /* We should not be receiving another TXDRDY at this point, expect an assert */
    TEST_NRF_MESH_ASSERT_EXPECT(m_tx_cb());
    /*We can queue two packets: */
    serial_buffer_get(1, &p_packet, p_buf_packet);
    p_packet->opcode = 1;
    serial_transmit(p_packet, p_buf_packet);
    serial_buffer_get(2, &p_packet, p_buf_packet2);
    p_packet->opcode = SLIP_END;
    *((uint8_t*) &p_packet->payload) = SLIP_ESC;
    serial_transmit(p_packet, p_buf_packet2);

    /* Transmit the first buffer*/
    transmit_bearer_event(p_buf_packet, true);
    TEST_ASSERT_EQUAL(1, NRF_UART0->TXD); /* Length field of the first packet*/
    /* After sending out the last byte, the packet should be freed in the following interrupt. */
    packet_buffer_free_Expect(NULL, p_buf_packet);
    packet_buffer_free_IgnoreArg_p_buffer();
    m_tx_cb();
    TEST_ASSERT_EQUAL(1, NRF_UART0->TXD); /* Opcode of the first packet */

    /* No more data to send.*/
    m_tx_cb();
    TEST_ASSERT_EQUAL(1, NRF_UART0->TASKS_STOPTX);
    NRF_UART0->TASKS_STOPTX = 0;

    /* Transmit the second buffer */
    transmit_bearer_event(p_buf_packet2, true);
    TEST_ASSERT_EQUAL(2, NRF_UART0->TXD); /* Length field of the second packet*/
    /* SLIP_END should not generate two bytes in being transmitted*/
    m_tx_cb();
    TEST_ASSERT_EQUAL(SLIP_END, NRF_UART0->TXD); /* First byte is an escape character */
    /* Third byte is SLIP_ESC and that will also be treated as an ordinary byte. */
    packet_buffer_free_Expect(NULL, p_buf_packet2);
    packet_buffer_free_IgnoreArg_p_buffer();
    m_tx_cb();
    TEST_ASSERT_EQUAL(SLIP_ESC, NRF_UART0->TXD); /* First byte is an escape character */

    /* No more data to send.*/
    m_tx_cb();
    TEST_ASSERT_EQUAL(1, NRF_UART0->TASKS_STOPTX);
    NRF_UART0->TASKS_STOPTX = 0;
}

void test_uart_rx(void)
{
    serial_packet_t * p_packet = (serial_packet_t*)test_data;
    p_packet->length = sizeof(test_data) - 1;
    p_packet->opcode = SERIAL_OPCODE_CMD_DEVICE_ECHO;
    packet_buffer_packet_t * p_buf_packet = (packet_buffer_packet_t *) m_buffer;

    serial_packet_t rx_packet;
    /* Test pending and get: */
    packet_buffer_can_pop_ExpectAndReturn(NULL, false);
    packet_buffer_can_pop_IgnoreArg_p_buffer();
    TEST_ASSERT_FALSE(serial_bearer_rx_pending());
    packet_buffer_pop_ExpectAndReturn(NULL, NULL, NRF_ERROR_NOT_FOUND);
    packet_buffer_pop_IgnoreArg_pp_packet();
    packet_buffer_pop_IgnoreArg_p_buffer();
    TEST_ASSERT_FALSE(serial_bearer_rx_get(&rx_packet));

    /* Incoming data. */
    /* First byte is 0, which is the length field, this should be discarded and a TX response should be generated. */
    receive_char(NULL, 0, false, true);

    /* Start receiving the packet, the first byte is the packet length */
    receive_char(&p_buf_packet, p_packet->length, true, false);

    /* Buf space will be checked when receiving data. */
    for (uint32_t i = 1; i < sizeof(test_data)-1; ++i)
    {
        /* Incoming data. */
        receive_char(NULL, test_data[i], false, false);
    }
    /* Last byte will trigger commit */
    packet_buffer_commit_Expect(NULL, p_buf_packet, sizeof(test_data));
    packet_buffer_commit_IgnoreArg_p_buffer();
    serial_process_Expect();
    receive_char(NULL, test_data[sizeof(test_data)-1], false, false);

    /* Get the received packet */
    packet_buffer_pop_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
    packet_buffer_pop_IgnoreArg_pp_packet();
    packet_buffer_pop_IgnoreArg_p_buffer();
    packet_buffer_pop_ReturnThruPtr_pp_packet(&p_buf_packet);
    packet_buffer_free_Expect(NULL, p_buf_packet);
    packet_buffer_free_IgnoreArg_p_buffer();
    serial_uart_receive_set_Expect(true);
    TEST_ASSERT_TRUE(serial_bearer_rx_get(&rx_packet));
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t*)p_packet, (uint8_t*) &rx_packet, sizeof(rx_packet));

    /* If we can't receive more, the bearer should stop UART from receiving anymore. */
    packet_buffer_reserve_ExpectAndReturn(NULL, NULL, 2, NRF_ERROR_NO_MEM);
    packet_buffer_reserve_IgnoreArg_p_buffer();
    packet_buffer_reserve_IgnoreArg_pp_packet();
    serial_uart_receive_set_Expect(false);
    receive_char(NULL, 1, true, false);
    /* Even though we can't receive anymore, we should have saved the length byte.
       Any further calls to rx will assume that we have explicitly enabled the hardware to receive
       more, and it will be treated as the rest of the payload. */
    packet_buffer_commit_Expect(NULL, p_buf_packet, 2);
    packet_buffer_commit_IgnoreArg_p_buffer();
    serial_process_Expect();
    receive_char(&p_buf_packet, 1, true, false); /* 1 is Opcode (length is stored from before) */

    /* Receive with the slip encoding characters and treat them as ordinary characters. */
    receive_char(&p_buf_packet, 2, true, false);
    /* An END value will trigger a write, and will not end transmission. */
    receive_char(NULL, SLIP_END, false, false);
    /* An ESC value will trigger a write. */
    packet_buffer_commit_Expect(NULL, p_buf_packet, 3);
    packet_buffer_commit_IgnoreArg_p_buffer();
    serial_process_Expect();
    receive_char(NULL, SLIP_ESC, false, false);
}

void test_too_long(void)
{
#if NRF_MESH_SERIAL_PAYLOAD_MAXLEN < (UINT8_MAX - 1)
    uint8_t packet_buffer[255] = { 0 };
    serial_packet_t * p_packet = (serial_packet_t *) packet_buffer;
    p_packet->length = sizeof(packet_buffer) - 1; /* subtract length field */
    p_packet->opcode = SERIAL_OPCODE_CMD_DEVICE_ECHO;

    for (uint8_t i = 0; i < sizeof(packet_buffer); i++)
    {
        if (i == 0)
        {
            /* The length byte is sent first, and will produce an error response. */
            receive_char(NULL, packet_buffer[i], true, true);
        }
        else
        {
            /* The rest of the packet bytes should be ignored. */
            receive_char(NULL, packet_buffer[i], false, false);
        }
    }

    /* Verify that we are not ignoring too many bytes: */
    p_packet->length = 4;
    p_packet->opcode = SERIAL_OPCODE_CMD_DEVICE_ECHO;
    packet_buffer_packet_t * p_buf_packet = (packet_buffer_packet_t *) m_buffer;
    for (uint8_t i = 0; i < p_packet->length; ++i)
    {
        if (i == 0)
        {
            receive_char(&p_buf_packet, packet_buffer[i], true, false);
        }
        else
        {
            receive_char(NULL, packet_buffer[i], false, false);
        }
    }
    /* Last byte will trigger commit */
    packet_buffer_commit_Expect(NULL, p_buf_packet, p_packet->length + 1);
    packet_buffer_commit_IgnoreArg_p_buffer();
    serial_process_Expect();
    receive_char(NULL, packet_buffer[p_packet->length], false, false);
#endif  /* NRF_MESH_SERIAL_PAYLOAD_MAXLEN < (UINT8_MAX - 1) */
}

