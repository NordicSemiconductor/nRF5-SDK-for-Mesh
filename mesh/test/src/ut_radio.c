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

#include <string.h>
#include <stdbool.h>
#include "unity.h"
#include "radio.h"
#include "nrf_error.h"
#include "nrf.h"
#include "nrf_mesh.h"

#define TXPTR   (mp_packet_buf)
#define RXPTR   (mp_packet_buf_rx)

typedef struct
{
  uint32_t  TASKS_TXEN;                        /*!< Enable radio in TX mode.                                              */
  uint32_t  TASKS_RXEN;                        /*!< Enable radio in RX mode.                                              */
  uint32_t  TASKS_START;                       /*!< Start radio.                                                          */
  uint32_t  TASKS_STOP;                        /*!< Stop radio.                                                           */
  uint32_t  TASKS_DISABLE;                     /*!< Disable radio.                                                        */
  uint32_t  TASKS_RSSISTART;                   /*!< Start the RSSI and take one sample of the receive signal strength.    */
  uint32_t  TASKS_RSSISTOP;                    /*!< Stop the RSSI measurement.                                            */
  uint32_t  TASKS_BCSTART;                     /*!< Start the bit counter.                                                */
  uint32_t  TASKS_BCSTOP;                      /*!< Stop the bit counter.                                                 */
  uint32_t  RESERVED0[55];
  uint32_t  EVENTS_READY;                      /*!< Ready event.                                                          */
  uint32_t  EVENTS_ADDRESS;                    /*!< Address event.                                                        */
  uint32_t  EVENTS_PAYLOAD;                    /*!< Payload event.                                                        */
  uint32_t  EVENTS_END;                        /*!< End event.                                                            */
  uint32_t  EVENTS_DISABLED;                   /*!< Disable event.                                                        */
  uint32_t  EVENTS_DEVMATCH;                   /*!< A device address match occurred on the last received packet.          */
  uint32_t  EVENTS_DEVMISS;                    /*!< No device address match occurred on the last received packet.         */
  uint32_t  EVENTS_RSSIEND;                    /*!< Sampling of the receive signal strength complete. A new RSSI
                                                         sample is ready for readout at the RSSISAMPLE register.               */
  uint32_t  RESERVED1[2];
  uint32_t  EVENTS_BCMATCH;                    /*!< Bit counter reached bit count value specified in BCC register.        */
  uint32_t  RESERVED2[53];
  uint32_t  SHORTS;                            /*!< Shortcuts for the radio.                                              */
  uint32_t  RESERVED3[64];
  uint32_t  INTENSET;                          /*!< Interrupt enable set register.                                        */
  uint32_t  INTENCLR;                          /*!< Interrupt enable clear register.                                      */
  uint32_t  RESERVED4[61];
  uint32_t  CRCSTATUS;                         /*!< CRC status of received packet.                                        */
  uint32_t  CD;                                /*!< Carrier detect.                                                       */
  uint32_t  RXMATCH;                           /*!< Received address.                                                     */
  uint32_t  RXCRC;                             /*!< Received CRC.                                                         */
  uint32_t  DAI;                               /*!< Device address match index.                                           */
  uint32_t  RESERVED5[60];
  uint32_t  PACKETPTR;                         /*!< Packet pointer. Decision point: START task.                           */
  uint32_t  FREQUENCY;                         /*!< Frequency.                                                            */
  uint32_t  TXPOWER;                           /*!< Output power.                                                         */
  uint32_t  MODE;                              /*!< Data rate and modulation.                                             */
  uint32_t  PCNF0;                             /*!< Packet configuration 0.                                               */
  uint32_t  PCNF1;                             /*!< Packet configuration 1.                                               */
  uint32_t  BASE0;                             /*!< Radio base address 0. Decision point: START task.                     */
  uint32_t  BASE1;                             /*!< Radio base address 1. Decision point: START task.                     */
  uint32_t  PREFIX0;                           /*!< Prefixes bytes for logical addresses 0 to 3.                          */
  uint32_t  PREFIX1;                           /*!< Prefixes bytes for logical addresses 4 to 7.                          */
  uint32_t  TXADDRESS;                         /*!< Transmit address select.                                              */
  uint32_t  RXADDRESSES;                       /*!< Receive address select.                                               */
  uint32_t  CRCCNF;                            /*!< CRC configuration.                                                    */
  uint32_t  CRCPOLY;                           /*!< CRC polynomial.                                                       */
  uint32_t  CRCINIT;                           /*!< CRC initial value.                                                    */
  uint32_t  TEST;                              /*!< Test features enable register.                                        */
  uint32_t  TIFS;                              /*!< Inter Frame Spacing in microseconds.                                  */
  uint32_t  RSSISAMPLE;                        /*!< RSSI sample.                                                          */
  uint32_t  RESERVED6;
  uint32_t  STATE;                             /*!< Current radio state.                                                  */
  uint32_t  DATAWHITEIV;                       /*!< Data whitening initial value.                                         */
  uint32_t  RESERVED7[2];
  uint32_t  BCC;                               /*!< Bit counter compare.                                                  */
  uint32_t  RESERVED8[39];
  uint32_t  DAB[8];                            /*!< Device address base segment.                                          */
  uint32_t  DAP[8];                            /*!< Device address prefix.                                                */
  uint32_t  DACNF;                             /*!< Device address match configuration.                                   */
  uint32_t  RESERVED9[56];
  uint32_t  OVERRIDE0;                         /*!< Trim value override register 0.                                       */
  uint32_t  OVERRIDE1;                         /*!< Trim value override register 1.                                       */
  uint32_t  OVERRIDE2;                         /*!< Trim value override register 2.                                       */
  uint32_t  OVERRIDE3;                         /*!< Trim value override register 3.                                       */
  uint32_t  OVERRIDE4;                         /*!< Trim value override register 4.                                       */
  uint32_t  RESERVED10[561];
  uint32_t  POWER;                             /*!< Peripheral power control.                                             */
} radio_dummy_t;


radio_dummy_t m_dummy_radio;

NRF_RADIO_Type * NRF_RADIO;

static uint8_t mp_packet_buf[40];
static uint8_t mp_packet_buf_rx[40];
static uint32_t m_rx_cb_count;
static uint32_t m_tx_cb_count;
static uint32_t m_idle_cb_count;
void rx_cb(uint8_t* p_data, bool succes, uint32_t crc, int8_t rssi);
void tx_cb(uint8_t* p_data, bool free_on_end);
void idle_cb(void);


void setUp(void)
{
    NRF_RADIO = (NRF_RADIO_Type*) &m_dummy_radio;
    radio_init_params_t params = {
        .radio_mode = RADIO_MODE_BLE_1MBIT,
        .access_address = 0xAABBCCDD,
        .rx_cb = rx_cb,
        .tx_cb = tx_cb,
        .idle_cb = idle_cb
    };

    radio_init(&params);
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
}

void tearDown(void)
{

}

void rx_cb(uint8_t* p_data, bool success, uint32_t crc, int8_t rssi)
{
    TEST_ASSERT_EQUAL_PTR(RXPTR, p_data);
    if (success)
    {
        TEST_ASSERT_EQUAL_HEX32(0xABCDEF, crc);
    }
    m_rx_cb_count++;
}

void tx_cb(uint8_t* p_data, bool free_packet)
{
    TEST_ASSERT_EQUAL_PTR(TXPTR, p_data);
    m_tx_cb_count++;
}

void idle_cb(void)
{
    m_idle_cb_count++;
}
/********* stubs *********/

/*************************/

void test_radio_ts_begin(void)
{
    memset(NRF_RADIO, 0, sizeof(NRF_RADIO_Type));
    radio_on_ts_begin();
    TEST_ASSERT_EQUAL(1, m_dummy_radio.POWER);
    TEST_ASSERT_EQUAL_HEX32((RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos), m_dummy_radio.TXPOWER);
    TEST_ASSERT_EQUAL_HEX32((RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos), m_dummy_radio.MODE);
    TEST_ASSERT_EQUAL_HEX8(0xAA, m_dummy_radio.PREFIX0);
    TEST_ASSERT_EQUAL_HEX32(0xBBCCDD00, m_dummy_radio.BASE0);
    TEST_ASSERT_EQUAL(0, m_dummy_radio.TXADDRESS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.RXADDRESSES);
    TEST_ASSERT_EQUAL_HEX32((1 << RADIO_PCNF0_S0LEN_Pos)
                          | (6 << RADIO_PCNF0_LFLEN_Pos)
                          | (2 << RADIO_PCNF0_S1LEN_Pos),
                          m_dummy_radio.PCNF0);
    TEST_ASSERT_EQUAL_HEX32((37 << RADIO_PCNF1_MAXLEN_Pos)
                          | (3 << RADIO_PCNF1_BALEN_Pos)
                          | (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos)
                          | (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos),
                          m_dummy_radio.PCNF1);
    TEST_ASSERT_EQUAL_HEX32(0x00065B << RADIO_CRCPOLY_CRCPOLY_Pos, m_dummy_radio.CRCPOLY);
    TEST_ASSERT_EQUAL_HEX32((RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos)
                          | (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos),
                          m_dummy_radio.CRCCNF);
    TEST_ASSERT_EQUAL_HEX32(0x555555 << RADIO_CRCINIT_CRCINIT_Pos, m_dummy_radio.CRCINIT);

    TEST_ASSERT_EQUAL(1, m_idle_cb_count);
}

void test_radio_error_propagation(void)
{
    radio_on_ts_begin();
    m_dummy_radio.RXCRC = 0xABCDEF;
    radio_event_t evt;
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 40; /* Invalid value */
    uint32_t count = 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(0, count);

    count = 1;
    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) NULL; /* Invalid value */
    evt.channel = 30;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(0, count);

    count = 1;
    evt.event_type = 0x33; /*lint !e64 Invalid value */
    evt.p_packet = (uint8_t*) NULL;
    evt.channel = 30;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(0, count);
}

void test_radio_order(void)
{
    uint32_t count = 1;
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_on_ts_begin();
    radio_event_t evt = {};
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 37;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(1, count);
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(37, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(2, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    m_dummy_radio.TASKS_TXEN = 0;
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_TxDisable;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    TEST_ASSERT_EQUAL(2, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 0;

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(1, count);
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(6, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    m_dummy_radio.TASKS_RXEN = 0;
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    m_dummy_radio.RXCRC = 0xABCDEF;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    TEST_ASSERT_EQUAL(3, m_idle_cb_count);

}

void test_radio_overflow(void)
{
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_on_ts_begin();
    radio_event_t evt;
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);
    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 16;
    uint32_t count = 1;

    for (uint32_t i = 0; i < 8; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
        TEST_ASSERT_EQUAL(1, count);
        radio_event_handler();
    }

    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(0, count);
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(16, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(38, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
        RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_END_DISABLE_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);
}

void test_radio_successive(void)
{
#if 0
    /* rx-rx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    radio_on_ts_begin();
    m_dummy_radio.RXCRC = 0xABCDEF;
    radio_event_t evt;
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(62, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 18;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(18, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk | /* go to disable to change channel */
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);
    TEST_ASSERT_EQUAL(0, m_rx_cb_count);

    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_rx_cb_count);
    TEST_ASSERT_EQUAL(2, m_idle_cb_count);

    /* rx-tx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 38;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(38, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(26, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_TXEN_Msk |
            RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* tx-rx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* tx-tx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39; /* same channel, shouldn't go to disabled */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL_HEX32(
        RADIO_SHORTS_READY_START_Msk |
        RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
        RADIO_SHORTS_END_DISABLE_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_tx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* rx-tx-rx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV); /* set channel immediately */
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_TXEN_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
            RADIO_SHORTS_END_DISABLE_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 29;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_TXEN_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_rx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* rx-rx-rx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 29;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk | /* unchanged */
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(3, m_rx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* tx-rx-tx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk,
                      m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_RX;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 28;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 29;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL_PTR(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_DISABLED_RXEN_Msk | /* unchanged */
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_rx_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_tx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    /* tx-tx-tx */
    m_dummy_radio.EVENTS_END = 0;
    m_rx_cb_count = 0;
    m_tx_cb_count = 0;
    m_idle_cb_count = 0;
    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(80, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 39;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 29;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt));
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(39, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    TEST_ASSERT_EQUAL(29, m_dummy_radio.DATAWHITEIV); /* should have setup the last event's ch */
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_DISABLED_TXEN_Msk,
            m_dummy_radio.SHORTS);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_tx_cb_count);
    TEST_ASSERT_EQUAL_HEX32(
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_READY_START_Msk,
            m_dummy_radio.SHORTS);
    m_dummy_radio.EVENTS_END = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(3, m_tx_cb_count);
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);
#endif
}

void test_radio_preempt(void)
{
    m_dummy_radio.EVENTS_END = 0;
    radio_on_ts_begin();
    m_dummy_radio.RXCRC = 0xABCDEF;
    radio_event_t evt = {};
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    evt.event_type = RADIO_EVENT_TYPE_RX_PREEMPTABLE;
    evt.p_packet = (uint8_t*) RXPTR;
    evt.channel = 28;
    uint32_t count = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(1, count);
    radio_event_handler();
    TEST_ASSERT_EQUAL(RXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(28, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(62, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_RXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 18;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(1, count);
    m_dummy_radio.CRCSTATUS = 0;
    radio_event_handler(); /* abort preemptable */
    TEST_ASSERT_EQUAL(1, m_rx_cb_count); /* should have reported back a fail */
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(18, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);

    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_Disabled;
    m_dummy_radio.CRCSTATUS = 1;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    TEST_ASSERT_EQUAL(2, m_idle_cb_count);
}

void test_radio_invalid_irqs(void)
{
    m_dummy_radio.EVENTS_END = 0;
    radio_on_ts_begin();
    m_dummy_radio.RXCRC = 0xABCDEF;
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    radio_event_handler();
    TEST_ASSERT_EQUAL(2, m_idle_cb_count);
    TEST_ASSERT_EQUAL(0, m_rx_cb_count);
    TEST_ASSERT_EQUAL(0, m_tx_cb_count);
}

void test_radio_ts_end_begin(void)
{
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_on_ts_begin();

    radio_event_t evt = {};
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);

    evt.event_type = RADIO_EVENT_TYPE_TX;
    evt.p_packet = (uint8_t*) TXPTR;
    evt.channel = 37;
    uint32_t count = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, radio_order(&evt, &count));
    TEST_ASSERT_EQUAL(1, count);
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(37, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(2, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    radio_on_ts_end();
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_DISABLE);
    TEST_ASSERT_EQUAL(0, m_dummy_radio.SHORTS);

    /* regain state: */
    memset(NRF_RADIO, 0, sizeof(NRF_RADIO_Type));
    radio_on_ts_begin();
    radio_event_handler();
    TEST_ASSERT_EQUAL(TXPTR, m_dummy_radio.PACKETPTR);
    TEST_ASSERT_EQUAL(37, m_dummy_radio.DATAWHITEIV);
    TEST_ASSERT_EQUAL(2, m_dummy_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(
            RADIO_SHORTS_READY_START_Msk |
            RADIO_SHORTS_END_DISABLE_Msk |
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk,
            m_dummy_radio.SHORTS);
    TEST_ASSERT_EQUAL(1, m_dummy_radio.TASKS_TXEN);
    TEST_ASSERT_EQUAL(RADIO_INTENSET_END_Msk, m_dummy_radio.INTENSET);

    m_dummy_radio.TASKS_TXEN = 0;
    m_dummy_radio.EVENTS_END = 1;
    m_dummy_radio.STATE = RADIO_STATE_STATE_TxDisable;
    radio_event_handler();
    TEST_ASSERT_EQUAL(1, m_tx_cb_count);
    TEST_ASSERT_EQUAL(2, m_idle_cb_count);
    m_dummy_radio.EVENTS_END = 0;
}

void test_radio_stop_resume(void)
{
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_resume(); /* illegal */
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_stop();
    TEST_ASSERT_EQUAL(0, m_idle_cb_count);
    radio_resume();
    TEST_ASSERT_EQUAL(1, m_idle_cb_count);
}

