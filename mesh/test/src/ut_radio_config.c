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

#include "radio_config.h"
#include "unity.h"

#include "nrf_error.h"
#include "nrf.h"
#include "test_assert.h"
#include "nrf_mesh_config_core.h"

#include <stdbool.h>
#include <string.h>

/* Initialize the RADIO peripheral, it will be externed by the headers. */
NRF_RADIO_Type * NRF_RADIO;
static NRF_RADIO_Type m_radio;

void setUp(void)
{
    NRF_RADIO = (NRF_RADIO_Type*) &m_radio;
}

void tearDown(void)
{

}

/**
 * As defined by Bluetooth Core Specification v4.2 Vol 6, part B, section 1.4.1:
 * this function will convert the given channel index to a frequency offset;
 * offset from 2400 MHz.
 */
static uint32_t m_channel_frequency_offset_get(uint32_t channel)
{
    uint32_t offset = 0xFFFFFFFF;
    if (channel <= 10)
    {
        offset = 4 + channel * 2;
    }
    else if (channel <= 36)
    {
        offset = 6 + channel * 2;
    }
    else if (channel < RADIO_NO_RF_CHANNELS)
    {
        static const uint8_t adv_freqs[] = {2, 26, 80};
        offset = adv_freqs[(channel - 37)];
    }
    return offset;
}

static void m_radio_config_reset_state_verify(void)
{
    /* A reset should: */
    /* Power up the radio.*/
    TEST_ASSERT_EQUAL(RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos, m_radio.POWER);
    /* Configure CRC to match the BLE core spec 4.2.*/
    uint32_t expected_crcconf = (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos) | (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos);
    TEST_ASSERT_EQUAL(expected_crcconf, m_radio.CRCCNF);
    TEST_ASSERT_EQUAL(0x00065B, m_radio.CRCPOLY);
    TEST_ASSERT_EQUAL(0x555555, m_radio.CRCINIT);

    /* Set TIFS to 150 as per BLE core spec 4.2*/
    TEST_ASSERT_EQUAL(150, m_radio.TIFS);
}

static void m_radio_config_config_state_verify(radio_config_t my_radio_config, uint32_t lflen, uint32_t s0len, uint32_t s1len)
{
    TEST_ASSERT_EQUAL(my_radio_config.tx_power, m_radio.TXPOWER);
    TEST_ASSERT_EQUAL(my_radio_config.radio_mode, m_radio.MODE);
    uint32_t expected_packet_conf_reg0 = ((lflen << RADIO_PCNF0_LFLEN_Pos) |
                                          (s1len << RADIO_PCNF0_S1LEN_Pos) |
                                          (s0len << RADIO_PCNF0_S0LEN_Pos));
    #ifdef NRF52
    expected_packet_conf_reg0 |= ((RADIO_PCNF0_S1INCL_Include << RADIO_PCNF0_S1INCL_Pos) & RADIO_PCNF0_S1INCL_Msk);
    #endif

    TEST_ASSERT_EQUAL_HEX32(expected_packet_conf_reg0, m_radio.PCNF0);

    uint32_t expected_packet_conf_reg1 = (my_radio_config.payload_maxlen << RADIO_PCNF1_MAXLEN_Pos) |
                                         (RADIO_CONFIG_DEFAULT_BA_LEN << RADIO_PCNF1_BALEN_Pos) |
                                         (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                                         (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
    TEST_ASSERT_EQUAL(expected_packet_conf_reg1, m_radio.PCNF1);
}

void test_radio_config(void)
{
    memset(&m_radio, 0, sizeof(NRF_RADIO_Type));
    /* Reset radio adn verify. */
    radio_config_reset();
    m_radio_config_reset_state_verify();

    /* Configure radio with common config settings */
    radio_config_t my_radio_config =
    {
        .tx_power = RADIO_POWER_NRF_NEG4DBM,
        .payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE,
        .radio_mode = RADIO_MODE_BLE_1MBIT
    };
    radio_config_config(&my_radio_config);
    m_radio_config_reset_state_verify();
    m_radio_config_config_state_verify(my_radio_config, 6, 1, 2);

    /* Try a different configuration */
    my_radio_config.radio_mode = RADIO_MODE_NRF_2MBIT;
    my_radio_config.tx_power = RADIO_POWER_NRF_NEG30DBM;
    my_radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE + 1;
    radio_config_config(&my_radio_config);
    m_radio_config_reset_state_verify();
    /* LF should be 8bits now that we have a large packet length */
    m_radio_config_config_state_verify(my_radio_config, 8, 1, 0);

    /* Check that channel settings was untouched until here */
    TEST_ASSERT_EQUAL(0, m_radio.FREQUENCY);
    TEST_ASSERT_EQUAL(0, m_radio.DATAWHITEIV);
    for (int i=0; i<RADIO_NO_RF_CHANNELS; i++)
    {
        radio_config_channel_set(i);
        TEST_ASSERT_EQUAL(m_channel_frequency_offset_get(i), m_radio.FREQUENCY);
        TEST_ASSERT_EQUAL(i, m_radio.DATAWHITEIV);
    }

    radio_config_access_addr_set(0x1234, 0);
    radio_config_access_addr_set(0x4321, 1);
    TEST_ASSERT_EQUAL(0x1234, (m_radio.BASE0 >> 8) | ((m_radio.PREFIX0<<24ul) & (0xFFu << 24ul)));
    TEST_ASSERT_EQUAL(0x4321, (m_radio.BASE1 >> 8) | ((m_radio.PREFIX0<<16ul) & (0xFFu << 24ul)));

    TEST_ASSERT_EQUAL(0x1234, radio_config_access_addr_get(0));
    TEST_ASSERT_EQUAL(0x4321, radio_config_access_addr_get(1));

    /* Still no changes from the reset state... */
    m_radio_config_reset_state_verify();
}

void test_radio_config_unhappy(void)
{
    memset(&m_radio, 0, sizeof(NRF_RADIO_Type));

    /* NULL pointer */
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_config(NULL));

    /* Configure radio with common config settings */
    radio_config_t my_radio_config =
    {
        .tx_power = RADIO_POWER_NRF_NEG4DBM,
        .payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE,
        .radio_mode = RADIO_MODE_BLE_1MBIT
    };

    /* Radio config shouuld not accept 0 payload length */
    my_radio_config.payload_maxlen = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_config(&my_radio_config));

    /* Also not length that's beyond supported value:
      RADIO_MAX_PACKET_LEN = payload_maxlen + S0 + S1 + LENGTH */
    my_radio_config.payload_maxlen = RADIO_MAX_PACKET_LEN-1;
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_config(&my_radio_config));

    /* But should be fine if we leave 2 bytes for S0 and LENGTH, since S1 will be 0 */
    my_radio_config.payload_maxlen = RADIO_MAX_PACKET_LEN-2;
    radio_config_config(&my_radio_config);

    /* Test that the radio config is not happy with invalid modes */
    my_radio_config.radio_mode = RADIO_MODE_END;
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_config(&my_radio_config));

    /* Test that non-existing channel configuration causes the assertion handler to be called. */
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_channel_set(RADIO_NO_RF_CHANNELS));

    memset(&m_radio, 0, sizeof(NRF_RADIO_Type));
    /* When BALEN is not the expected value the access addr functions will derp */
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_access_addr_set(0xAAAAA, 1));
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_access_addr_get(1));

    /* Setting the BALEN to the expected value makes access_addr_set/get happy:  */
    NRF_RADIO->PCNF1 = (RADIO_CONFIG_DEFAULT_BA_LEN << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk;
    radio_config_access_addr_set(0xAAAAA, 1);
    TEST_ASSERT_EQUAL(0xAAAAA, radio_config_access_addr_get(1));

    /* Test that trying to set/get unsupported logical addresses does not fly...*/
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_access_addr_set(0x4321, 2));
    TEST_NRF_MESH_ASSERT_EXPECT(radio_config_access_addr_get(2));
}
