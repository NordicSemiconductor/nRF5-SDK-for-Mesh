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

NRF_UART_Type * NRF_UART0;
static NRF_UART_Type m_uart;
uint32_t m_critical_section;
uint8_t test_data[sizeof(serial_packet_t)];
uint8_t m_buffer[UINT16_MAX];
bearer_event_flag_callback_t m_flag_event_callback;
void (*m_rx_cb)(uint8_t);
void (*m_tx_cb)(void);


/*********************************************************************
 * Mocked functions                                                  *
 *********************************************************************/

void m_bearer_event_critical_section_begin(int cmock_num_calls)
{
    m_critical_section++;
}

void m_bearer_event_critical_section_end(int cmock_num_calls)
{
    m_critical_section--;
}

uint32_t m_serial_uart_init(void (*rx_cb)(uint8_t), void (*tx_cb)(void), int cmock_num_calls)
{
    m_rx_cb = rx_cb;
    m_tx_cb = tx_cb;
    return NRF_SUCCESS;
}

void NVIC_EnableIRQ(uint32_t IRQn)
{

}

void NVIC_DisableIRQ(uint32_t IRQn)
{

}

void NVIC_SetPriority(uint32_t IRQn, uint32_t priority)
{

}

uint32_t bearer_event_flag_add_callback(bearer_event_flag_callback_t callback, int count)
{
    TEST_ASSERT_EQUAL(0, count);
    m_flag_event_callback = callback;
    return 0x1234;
}
void bearer_event_flag_set_callback(uint32_t flag, int count)
{
    TEST_ASSERT_EQUAL(0x1234, flag);
}

/*********************************************************************
 * Helper functions                                                  *
 *********************************************************************/

void segfault_handler(int sig)
{
  printf("FAIL: Segfault\n");
  exit(1);
}

void serial_buffer_get_calls(uint16_t packet_len, packet_buffer_packet_t ** pp_buf_packet)
{
    (*pp_buf_packet)->size = packet_len + 1;
    (*pp_buf_packet)->packet_state = PACKET_BUFFER_MEM_STATE_RESERVED;
    packet_buffer_reserve_ExpectAndReturn(NULL, NULL, packet_len + 1, NRF_SUCCESS);
    packet_buffer_reserve_IgnoreArg_p_buffer();
    packet_buffer_reserve_IgnoreArg_pp_packet();
    packet_buffer_reserve_ReturnThruPtr_pp_packet(pp_buf_packet);
}

void serial_transmit_packet_buf_calls(serial_packet_t * p_packet, packet_buffer_packet_t ** pp_buf_packet)
{
    packet_buffer_commit_Ignore();
    packet_buffer_commit_Expect(NULL, *pp_buf_packet, p_packet->length+1);
    packet_buffer_commit_IgnoreArg_p_buffer();
}
void serial_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet, packet_buffer_packet_t * p_buf_packet)
{
    serial_buffer_get_calls(packet_len, &p_buf_packet);
    (void) serial_bearer_packet_buffer_get(packet_len, pp_packet);
}

void serial_transmit(serial_packet_t * p_packet, packet_buffer_packet_t * p_buf_packet)
{
    serial_transmit_packet_buf_calls(p_packet, &p_buf_packet);
    serial_bearer_tx(p_packet);
}

void transmit_bearer_event(packet_buffer_packet_t * p_buf_packet, bool success)
{
    if (success)
    {
        packet_buffer_pop_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
        packet_buffer_pop_IgnoreArg_pp_packet();
        packet_buffer_pop_IgnoreArg_p_buffer();
        packet_buffer_pop_ReturnThruPtr_pp_packet(&p_buf_packet);
    }

    m_flag_event_callback();
}

void receive_char(packet_buffer_packet_t ** pp_buf_packet, uint8_t val, bool check_len, bool sends_error_msg)
{
    if (check_len)
    {
        packet_buffer_max_packet_len_get_ExpectAndReturn(NULL, sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t));
        packet_buffer_max_packet_len_get_IgnoreArg_p_buffer();
    }
    if (NULL != pp_buf_packet)
    {
        (*pp_buf_packet)->size = val + 1;
        packet_buffer_reserve_ExpectAndReturn(NULL, pp_buf_packet, val + 1, NRF_SUCCESS);
        packet_buffer_reserve_IgnoreArg_p_buffer();
        packet_buffer_reserve_IgnoreArg_pp_packet();
        packet_buffer_reserve_ReturnThruPtr_pp_packet(pp_buf_packet);
    }

    uint8_t local_pacman_buff[100];
    packet_buffer_packet_t * p_local_buf_packet = (packet_buffer_packet_t *) local_pacman_buff;
    if (sends_error_msg)
    {
        serial_buffer_get_calls(SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_local_buf_packet);
        serial_packet_t * p_packet = (serial_packet_t *)p_local_buf_packet->packet;
        p_packet->length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD;
        serial_transmit_packet_buf_calls(p_packet, &p_local_buf_packet);
    }

    m_rx_cb(val);
}

/*********************************************************************
 * Unity setup and teardown functions                                *
 *********************************************************************/

void setUp(void)
{
    serial_uart_mock_Init();
    packet_buffer_mock_Init();
    bearer_event_mock_Init();
    serial_mock_Init();

    signal(SIGSEGV,segfault_handler);
    bearer_event_flag_set_StubWithCallback(bearer_event_flag_set_callback);
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_callback);
    bearer_event_critical_section_begin_StubWithCallback(m_bearer_event_critical_section_begin);
    bearer_event_critical_section_end_StubWithCallback(m_bearer_event_critical_section_end);
    serial_uart_init_StubWithCallback(m_serial_uart_init);

    NRF_UART0 = (NRF_UART_Type *) &m_uart;

    /* Init test data*/
    for (uint32_t i = 0; i < sizeof(test_data); ++i)
    {
        test_data[i] = i % 80;  /* Modulo with a number lower than the slip bytes. */
    }

    packet_buffer_init_Expect(NULL, NULL, 2 * ALIGN_VAL(sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t), WORD_SIZE));
    packet_buffer_init_IgnoreArg_p_pool();
    packet_buffer_init_IgnoreArg_p_buffer();
    packet_buffer_init_Expect(NULL, NULL, 2 * ALIGN_VAL(sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t), WORD_SIZE));
    packet_buffer_init_IgnoreArg_p_pool();
    packet_buffer_init_IgnoreArg_p_buffer();
    serial_uart_receive_set_Expect(true);
    serial_bearer_init();
}

void tearDown(void)
{
    TEST_ASSERT_EQUAL(0, m_critical_section);
    serial_mock_Verify();
    serial_uart_mock_Verify();
    serial_uart_mock_Destroy();
    packet_buffer_mock_Verify();
    packet_buffer_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    serial_mock_Verify();
    serial_mock_Destroy();
}

