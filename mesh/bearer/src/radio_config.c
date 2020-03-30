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

#include "nrf.h"
#include "nrf_mesh_assert.h"
#include <stddef.h>

/* The CRC polynomial setting and its initial value for advertising channel packets.
 * BT core spec Version 4.2 [Vol 6, Part B], section 3.1.
 * Note that initial value may be reset to a different value if in a connection.
 */
#define RADIO_CONFIG_CRC_POLY_CONFIG 0x00065B
#define RADIO_CONFIG_CRC_ADV_INIT    0x555555

/* Inter frame sppace as defined by BT core spec Version 4.2 [Vol 6, Part B], section 4.1 */
#define RADIO_CONFIG_DEFAULT_TIFS 150

/* Packet Static length in number of bytes.
 * It is added to the total length of the payload (by radio) when sending and receiving packets.
 */
#define RADIO_CONFIG_DEFAULT_STATIC_LEN 0UL

/* The length of the length field of the advertising and data channel PDU headers
 * in bits: as defined by BT core spec version 4.2 [Vol 6, Part B], section 2.3 adn 2.4
 */
#define RADIO_CONFIG_ADV_PAYLOAD_LENGTH_SIZE 6UL
#define RADIO_CONFIG_DATA_PAYLOAD_LENGTH_SIZE 8UL
#define RADIO_CONFIG_MAX_PAYLOAD_LENGTH_SIZE 8UL /* 1 byte */

NRF_MESH_STATIC_ASSERT(RADIO_CONFIG_DATA_PAYLOAD_LENGTH_SIZE <= RADIO_CONFIG_MAX_PAYLOAD_LENGTH_SIZE);

#define RADIO_CONFIG_S0LEN  1UL /* For the data preceding the length field in Bytes */


void radio_config_reset(void)
{
    /* Reset all states in the radio peripheral */
    NRF_RADIO->POWER         = (RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk;
    NRF_RADIO->POWER         = (RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk;

    /* CRC config */
    NRF_RADIO->CRCPOLY = ((RADIO_CONFIG_CRC_POLY_CONFIG << RADIO_CRCPOLY_CRCPOLY_Pos) & RADIO_CRCPOLY_CRCPOLY_Msk);
    NRF_RADIO->CRCCNF = ((RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                       | ((RADIO_CONFIG_CRC_LEN << RADIO_CRCCNF_LEN_Pos) & RADIO_CRCCNF_LEN_Msk);
    NRF_RADIO->CRCINIT = ((RADIO_CONFIG_CRC_ADV_INIT << RADIO_CRCINIT_CRCINIT_Pos) & RADIO_CRCINIT_CRCINIT_Msk);

    NRF_RADIO->TIFS = RADIO_CONFIG_DEFAULT_TIFS;

#ifdef NRF52_SERIES
    /* Enable fast ramup on nRF52 */
    NRF_RADIO->MODECNF0 |= ((RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos) & RADIO_MODECNF0_RU_Msk);
#endif
}

/**
 * As defined by Bluetooth Core Specification v4.2 Vol 6, part B, section 1.4.1:
 * this function will convert the given channel index to a frequency  offset;
 * offset from 2400 MHz.
 */
void radio_config_channel_set(const uint32_t channel)
{
    /* See Table 1.2 in the spec for all the magic values. */
    static const uint8_t channel_rf_offsets[RADIO_NO_RF_CHANNELS] = {4,6,8,10,12,14,16,18,20,22,24,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,2,26,80};

    NRF_MESH_ASSERT(channel < RADIO_NO_RF_CHANNELS);
    NRF_RADIO->FREQUENCY = channel_rf_offsets[channel];
    NRF_RADIO->DATAWHITEIV = channel;
}

void radio_config_config(const radio_config_t * const p_config)
{
    NRF_MESH_ASSERT(NULL != p_config);
    NRF_MESH_ASSERT(RADIO_MODE_END > p_config->radio_mode);
    NRF_MESH_ASSERT( 0 < p_config->payload_maxlen);
    /* payload_maxlen can't be larger than supported value
       +2: 1 byte for LEN field and 1 byte for S0 (S1 is 0 for large values of payload_maxlen). */
    NRF_MESH_ASSERT(RADIO_MAX_PACKET_LEN >= (((uint32_t) p_config->payload_maxlen) + 2));
    NRF_RADIO->TXPOWER = p_config->tx_power;
    NRF_RADIO->MODE    = p_config->radio_mode;

    /* Packet configuration */
    NRF_RADIO->PCNF1         =  (
            /* maximum length of payload in bytes [0-255] */
            ((p_config->payload_maxlen        << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  |
            /* expand the payload with N bytes in addition to LENGTH [0-255] */
            ((RADIO_CONFIG_DEFAULT_STATIC_LEN << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
            /* base address length in number of bytes. */
            ((RADIO_CONFIG_DEFAULT_BA_LEN     << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)   |
            /* endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
            ((RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)  |
            /* enable packet whitening */
            ((RADIO_PCNF1_WHITEEN_Enabled     << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk) );

    /* PCNF-> Packet Configuration. Configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
    uint32_t payload_length_size = p_config->payload_maxlen > RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE ?
                                   RADIO_CONFIG_DATA_PAYLOAD_LENGTH_SIZE :
                                   RADIO_CONFIG_ADV_PAYLOAD_LENGTH_SIZE;
    uint32_t s1len = RADIO_CONFIG_MAX_PAYLOAD_LENGTH_SIZE - payload_length_size;
    NRF_RADIO->PCNF0 = (
        /* length of S0 field in bytes 0-1. */
        ((RADIO_CONFIG_S0LEN << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) |
        /* length of S1 field in bits 0-8. */
        ((s1len << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |
        /* length of length field in bits 0-8. */
        ((payload_length_size << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk) );

#ifdef NRF52_SERIES
    NRF_RADIO->PCNF0 |= ((RADIO_PCNF0_S1INCL_Include << RADIO_PCNF0_S1INCL_Pos) & RADIO_PCNF0_S1INCL_Msk);
#if  NRF_SD_BLE_API_VERSION >= 5
    /* BLE 2Mbit requires a 16bit preamble */
    if (p_config->radio_mode == RADIO_MODE_MODE_Ble_2Mbit)
    {
        NRF_RADIO->PCNF0 |= ((RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos) & RADIO_PCNF0_PLEN_Msk);
    }
#endif

#endif
}

void radio_config_access_addr_set(const uint32_t access_addr, const uint32_t logical_addr)
{
    /* This function expects that we use the default base address length */
    NRF_MESH_ASSERT(RADIO_CONFIG_DEFAULT_BA_LEN == (NRF_RADIO->PCNF1 & RADIO_PCNF1_BALEN_Msk) >> RADIO_PCNF1_BALEN_Pos);

    /* Configure Access Address  */
    switch (logical_addr)
    {
        case 0:
            NRF_RADIO->PREFIX0 = (access_addr >> (24-RADIO_PREFIX0_AP0_Pos)) & RADIO_PREFIX0_AP0_Msk;
            NRF_RADIO->BASE0   = (access_addr << 8) & 0xFFFFFF00;
            break;
        case 1:
            NRF_RADIO->PREFIX0 = (access_addr >> (24-RADIO_PREFIX0_AP1_Pos)) & RADIO_PREFIX0_AP1_Msk;
            NRF_RADIO->BASE1   = (access_addr << 8) & 0xFFFFFF00;
            break;
        default:
            /* logical addresses 2 -7 are not supported */
            NRF_MESH_ASSERT(false);
    }

}

uint32_t radio_config_access_addr_get(const uint32_t logical_addr)
{
    /* This function expects that we use the default base address length */
    NRF_MESH_ASSERT(RADIO_CONFIG_DEFAULT_BA_LEN == (NRF_RADIO->PCNF1 & RADIO_PCNF1_BALEN_Msk) >> RADIO_PCNF1_BALEN_Pos);
    uint32_t access_addr = 0; /* Initialize to 0 to stop gcc complaining*/
    switch (logical_addr)
    {
        case 0:
            access_addr = (NRF_RADIO->BASE0 >> 8) | ((NRF_RADIO->PREFIX0 & RADIO_PREFIX0_AP0_Msk) << (24-RADIO_PREFIX0_AP0_Pos));
            break;
        case 1:
            access_addr = (NRF_RADIO->BASE1 >> 8) | (NRF_RADIO->PREFIX0 & RADIO_PREFIX0_AP1_Msk) << (24-RADIO_PREFIX0_AP1_Pos);
            break;
        default:
            /* logical addresses 2 -7 are not supported */
            NRF_MESH_ASSERT(false);
    }

    return access_addr;
}
