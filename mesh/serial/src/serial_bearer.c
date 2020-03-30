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

#include "serial_bearer.h"
#include "serial_uart.h"

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_error.h"

#include "nrf_mesh_serial.h"
#include "nrf_mesh_assert.h"
#include "serial.h"
#include "serial_evt.h"
#include "serial_packet.h"
#include "serial_status.h"
#include "packet_buffer.h"
#include "bearer_event.h"
#include "toolchain.h"

/** Defines for SLIP encoding: see https://tools.ietf.org/html/rfc1055 */
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

/********** Local typedefs **********/

typedef enum
{
    SERIAL_STATE_IDLE,
    SERIAL_STATE_TRANSMIT
} serial_state_t;


/********** Static variables **********/

/* Buffers must be word-aligned, and also be able to fit two instances of the largest possible
 * packet, to avoid it locking up if the head is in the middle of the buffer. */
#define RX_BUFFER_SIZE (2 * ALIGN_VAL(sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t), WORD_SIZE))
#define TX_BUFFER_SIZE (2 * ALIGN_VAL(sizeof(serial_packet_t) + sizeof(packet_buffer_packet_t), WORD_SIZE))

static uint8_t m_tx_buffer[TX_BUFFER_SIZE];
static packet_buffer_t m_tx_packet_buf;
static packet_buffer_packet_t * mp_current_tx_packet;
static uint16_t m_cur_tx_packet_index;
static uint16_t m_stored_pac_len;
static uint8_t m_rx_buffer[RX_BUFFER_SIZE];
static packet_buffer_t m_rx_packet_buf;
static packet_buffer_packet_t * mp_current_rx_packet;
static serial_state_t m_serial_state = SERIAL_STATE_IDLE;
static uint32_t m_event_flag;
static uint16_t m_ignore_rx_count;

#ifdef SERIAL_SLIP_ENCODING
static uint8_t m_tx_slip_byte;
#endif

/* Serial receive relies on the length field of a serial packet being the first byte and opcode being the second byte */
NRF_MESH_STATIC_ASSERT(offsetof(serial_packet_t, length) == 0);

/********** Static Functions **********/

static void schedule_transmit(void)
{
    bearer_event_flag_set(m_event_flag);
}

static void send_cmd_response(uint8_t status, uint8_t opcode)
{
    serial_packet_t * p_rsp;
    if (NRF_SUCCESS == serial_bearer_packet_buffer_get(SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_rsp))
    {
        p_rsp->opcode = SERIAL_OPCODE_EVT_CMD_RSP;
        p_rsp->payload.evt.cmd_rsp.opcode = opcode;
        p_rsp->payload.evt.cmd_rsp.status = status;
        serial_bearer_tx(p_rsp);
    }
}

static void end_reception(uint16_t * p_rx_index)
{
    if (*p_rx_index != 0)
    {
        /* Check if the number of bytes received matched the number of bytes expected. */
        serial_packet_t* packet_received = (serial_packet_t*)(mp_current_rx_packet->packet);
        if (*p_rx_index != packet_received->length + 1)
        {
            /*Send error with the opcode received: */
            send_cmd_response(SERIAL_STATUS_ERROR_INVALID_LENGTH, ((serial_packet_t *)mp_current_rx_packet)->opcode);
            packet_buffer_free(&m_rx_packet_buf, mp_current_rx_packet);
            mp_current_rx_packet = NULL;
        }
        else
        {
            /* The packet is complete, commit and process */
            packet_buffer_commit(&m_rx_packet_buf, mp_current_rx_packet, mp_current_rx_packet->size);
            mp_current_rx_packet = NULL;
            serial_process();
        }
        *p_rx_index = 0;
    }
}

static bool rx_packet_reserve(uint16_t pac_len)
{
    if (pac_len <= 1 || pac_len > packet_buffer_max_packet_len_get(&m_rx_packet_buf)
            || pac_len > sizeof(serial_packet_t))
    {
        m_ignore_rx_count = pac_len - 1; /* Ignore the rest of the bytes in the packet. */
        send_cmd_response(SERIAL_STATUS_ERROR_INVALID_LENGTH, 0);
        return false;
    }
    else if (NRF_SUCCESS != packet_buffer_reserve(&m_rx_packet_buf, &mp_current_rx_packet, pac_len))
    {
        m_stored_pac_len = pac_len;
        serial_uart_receive_set(false);
        return false;
    }
    else
    {
        return true;
    }
}

static bool char_rx_first_byte(uint16_t * p_rx_index, uint8_t byte_received)
{
    /* First index is the length field. Check and reserve if we have enough space. */
    uint16_t packet_length = (uint16_t)byte_received + 1;
    bool packet_reserved = true;
    /* If we had halted the reception of a packet earlier, we should have
     * buffered a packet length already and what we receive now would be the
     * second byte. This assumes that we have explicitly enabled the hardware
     * to receive more, and does not handle bytes being passed in otherwise. */
    if (m_stored_pac_len != 0)
    {
        packet_length = m_stored_pac_len;
    }
    packet_reserved = rx_packet_reserve(packet_length);
    if (packet_reserved && m_stored_pac_len != 0)
    {
        m_stored_pac_len = 0;
        mp_current_rx_packet->packet[*p_rx_index] = packet_length - 1;
        (*p_rx_index)++;
    }

    return packet_reserved;
}

#ifndef SERIAL_SLIP_ENCODING
static inline void char_rx_simple(uint16_t * p_rx_index, uint8_t byte_received)
{
    if (*p_rx_index == 0 && !char_rx_first_byte(p_rx_index, byte_received))
    {
        return; /* Early return since we could not reserve a packet*/
    }

    mp_current_rx_packet->packet[*p_rx_index] = byte_received;
    (*p_rx_index)++;

    serial_packet_t* packet_received = (serial_packet_t*)(mp_current_rx_packet->packet);
    if (NULL != mp_current_rx_packet && *p_rx_index == packet_received->length + 1) /* We have received the complete packet */
    {
        end_reception(p_rx_index);
    }
}

static inline void char_tx_simple(void)
{
    serial_packet_t * p_serial_packet = (serial_packet_t *) mp_current_tx_packet->packet;
    if (m_cur_tx_packet_index == p_serial_packet->length + SERIAL_PACKET_LENGTH_OVERHEAD) /* We are done sending. */
    {
        m_cur_tx_packet_index++;
        packet_buffer_free(&m_tx_packet_buf, mp_current_tx_packet);
        mp_current_tx_packet = NULL;
        serial_uart_tx_stop();
        m_serial_state = SERIAL_STATE_IDLE;
        /* send next packet */
        schedule_transmit();
    }
    else if (m_cur_tx_packet_index < p_serial_packet->length + SERIAL_PACKET_LENGTH_OVERHEAD)
    { /* Send the next byte in line */
        uint8_t value = mp_current_tx_packet->packet[m_cur_tx_packet_index];
        m_cur_tx_packet_index++;
        serial_uart_byte_send(value);
    }
}
#endif

#ifdef SERIAL_SLIP_ENCODING
static inline bool valid_slip_byte(uint8_t byte_val)
{
    switch (byte_val)
    {
        case SLIP_END:
        case SLIP_ESC:
        case SLIP_ESC_END:
        case SLIP_ESC_ESC:
            return true;
        default:
            return false;
    }
}

static inline void slip_encoding_get(uint8_t * p_value, uint8_t * p_slip_byte)
{
    if (SLIP_END == *p_value)
    {
        *p_slip_byte = SLIP_ESC_END;
        *p_value = SLIP_ESC;
    }
    else if (SLIP_ESC == *p_value)
    {
        *p_slip_byte = SLIP_ESC_ESC;
        *p_value = SLIP_ESC;
    }
}

/**
 * @brief          Decodes a byte based on the SLIP encoding as specified by rfc1055.
 *
 * @param[in, out] p_c          The pointer to the character received, the value of it may be
 *                              updated if the received character has been escaped earlier.
 * @param[out]     p_skip_byte  The value of this will be set to @c true if this byte should not be
 *                              recorded by the recipient.
 * @param          p_prev_char  The pointer to the previous character, this should be stored by the
 *                              caller.
 *
 * @return         @true if the received byte is an END character or an invalid combination of SLIP
 *                 characters have been received.
 */
static bool slip_decode(uint8_t * p_c, bool * p_skip_byte, uint8_t * p_prev_char)
{
    bool end_of_reception;
    *p_skip_byte = false;
    if (SLIP_END == *p_c)
    {
        end_of_reception = true;
    }
    else if (*p_prev_char != SLIP_ESC)
    {
        if (SLIP_ESC == *p_c)
        {
            *p_skip_byte = true;
        }
        end_of_reception = false;
    }
    else
    {
        switch (*p_c)
        {
            case SLIP_ESC_END:
                *p_c = SLIP_END;
                end_of_reception = false;
                break;
            case SLIP_ESC_ESC:
                *p_c = SLIP_ESC;
                end_of_reception = false;
                break;
            default:
                end_of_reception = true;
                break;
        }
    }
    *p_prev_char = *p_c;
    return end_of_reception;
}

static inline void char_tx_with_slip_encoding(void)
{
    if (valid_slip_byte(m_tx_slip_byte)) /* Did we store a slip byte during the previous tx? */
    {
        serial_uart_byte_send(m_tx_slip_byte);
        m_tx_slip_byte = 0;
    }
    else if (m_cur_tx_packet_index == mp_current_tx_packet->size) /* Is it time to send the END byte? */
    {
        m_cur_tx_packet_index++;
        packet_buffer_free(&m_tx_packet_buf, mp_current_tx_packet);
        mp_current_tx_packet = NULL;
        serial_uart_byte_send(SLIP_END);
    }
    else /* if (m_cur_tx_packet_index < mp_current_tx_packet->size) */
    { /* Send the next byte in line, and store the slip byte if necessary */
        uint8_t value = mp_current_tx_packet->packet[m_cur_tx_packet_index];
        m_cur_tx_packet_index++;
        slip_encoding_get(&value, &m_tx_slip_byte);
        serial_uart_byte_send(value);
    }
}

static inline void char_rx_with_slip_encoding(uint16_t * p_rx_index, uint8_t byte_received)
{
    static uint8_t prev_char = 0;
    bool skip_byte;
    if (slip_decode(&byte_received, &skip_byte, &prev_char))
    {
        end_reception(p_rx_index);
    }
    else if (NULL != mp_current_rx_packet &&
             *p_rx_index == ((serial_packet_t*)(mp_current_rx_packet->packet))->length + 1)
    {

        send_cmd_response(SERIAL_STATUS_ERROR_INVALID_LENGTH, ((serial_packet_t *)mp_current_rx_packet)->opcode);
        /* We received something else when we were expecting an END byte. */
        packet_buffer_free(&m_rx_packet_buf, mp_current_rx_packet);
        mp_current_rx_packet = NULL;
        *p_rx_index = 0;
    }
    else if (!skip_byte) /* do we need to skip this byte? i.e. is it an escape character? */
    {
        if (*p_rx_index == 0 && !char_rx_first_byte(p_rx_index, byte_received))
        {
            return; /* Early return since we could not reserve a packet*/
        }

        if (mp_current_rx_packet != NULL)
        {
            mp_current_rx_packet->packet[*p_rx_index] = byte_received;
            (*p_rx_index)++;
        }
    }
}
#endif

static void char_rx(uint8_t c)
{
    static uint16_t rx_index = 0;

    if (m_ignore_rx_count > 0)
    {
        m_ignore_rx_count--;
    }
    else
    {
#ifdef SERIAL_SLIP_ENCODING
        char_rx_with_slip_encoding(&rx_index, c);
#else
        char_rx_simple(&rx_index, c);
#endif
    }
}

static void char_tx(void)
{
    /* Unexpected event */
    NRF_MESH_ASSERT(m_serial_state != SERIAL_STATE_IDLE);
    if (NULL == mp_current_tx_packet)
    {
        /* We have nothing to send. */
        serial_uart_tx_stop();
        m_serial_state = SERIAL_STATE_IDLE;
        schedule_transmit();
    }
    else
    {
#ifdef SERIAL_SLIP_ENCODING
        char_tx_with_slip_encoding();
#else
        char_tx_simple();
#endif
    }
}

static bool do_transmit(void)
{
    if (SERIAL_STATE_IDLE == m_serial_state &&
        NRF_SUCCESS == packet_buffer_pop(&m_tx_packet_buf, &mp_current_tx_packet))
    {
        m_serial_state = SERIAL_STATE_TRANSMIT;

#ifdef SERIAL_SLIP_ENCODING
        serial_uart_byte_send(SLIP_END);
        m_cur_tx_packet_index = 0;
#else
        uint8_t value = mp_current_tx_packet->packet[0];
        serial_uart_byte_send(value);
        m_cur_tx_packet_index = 1;
#endif
        serial_uart_tx_start();
    }
    return true;
}

/********** Interface Functions **********/
void serial_bearer_init(void)
{
    packet_buffer_init(&m_tx_packet_buf, (void *) m_tx_buffer, sizeof(m_tx_buffer));
    packet_buffer_init(&m_rx_packet_buf, (void *) m_rx_buffer, sizeof(m_rx_buffer));

    NRF_MESH_ASSERT(NRF_SUCCESS == serial_uart_init(char_rx, char_tx));
    serial_uart_receive_set(true);

    m_serial_state = SERIAL_STATE_IDLE;
    m_stored_pac_len = 0;
    m_cur_tx_packet_index = 0;
#ifdef SERIAL_SLIP_ENCODING
    m_tx_slip_byte = 0;
#endif

    m_event_flag = bearer_event_flag_add(do_transmit);
}

uint32_t serial_bearer_packet_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet)
{
    packet_buffer_packet_t * p_reserved_buffer_packet;

    uint32_t status = packet_buffer_reserve(&m_tx_packet_buf, &p_reserved_buffer_packet, packet_len + SERIAL_PACKET_LENGTH_OVERHEAD);

    if (status == NRF_SUCCESS)
    {
        *pp_packet = (serial_packet_t *) p_reserved_buffer_packet->packet;
        (*pp_packet)->length = packet_len;
    }
    return status;
}

uint32_t serial_bearer_blocking_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet)
{
    uint32_t status;
    while (1)
    {
        status = serial_bearer_packet_buffer_get(packet_len, pp_packet);
        if (status == NRF_ERROR_NO_MEM)
        {
            NVIC_DisableIRQ(UART0_IRQn);
            serial_uart_process();
            if (SERIAL_STATE_IDLE == m_serial_state)
            {
                (void)do_transmit();
            }
            NVIC_EnableIRQ(UART0_IRQn);
        }
        else
        {
            break;
        }
    }
    return status;
}

void serial_bearer_tx(const serial_packet_t * p_packet)
{
    NRF_MESH_ASSERT(NULL != p_packet);

    packet_buffer_packet_t * p_reserved_buffer_packet = (packet_buffer_packet_t *) ((uint32_t) p_packet - offsetof(packet_buffer_packet_t, packet));

    NRF_MESH_ASSERT(PACKET_BUFFER_MEM_STATE_RESERVED == p_reserved_buffer_packet->packet_state);
    NRF_MESH_ASSERT(p_packet->length <= p_reserved_buffer_packet->size);

    bearer_event_critical_section_begin();
    schedule_transmit();
    packet_buffer_commit(&m_tx_packet_buf, p_reserved_buffer_packet, p_reserved_buffer_packet->size);
    bearer_event_critical_section_end();
}

bool serial_bearer_rx_get(serial_packet_t* p_packet)
{
    packet_buffer_packet_t *p_buf_packet;
    if (NRF_SUCCESS == packet_buffer_pop(&m_rx_packet_buf, &p_buf_packet))
    {
        memcpy(p_packet, p_buf_packet->packet, p_buf_packet->size);
        packet_buffer_free(&m_rx_packet_buf, p_buf_packet);
        serial_uart_receive_set(true);
        return true;
    }
    else
    {
        return false;
    }
}

bool serial_bearer_rx_pending(void)
{
    return packet_buffer_can_pop(&m_rx_packet_buf);
}
