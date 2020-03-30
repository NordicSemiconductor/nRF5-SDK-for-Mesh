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
#ifndef RADIO_CONFIG_H__
#define RADIO_CONFIG_H__

#include <stdint.h>

/**
 * @defgroup RADIO_CONFIG Radio configuration
 * @ingroup MESH_BEARER
 * Provides a common interface for setting-up the radio before use.
 * @{
 */

/** Base address length in bytes. Radio address field is composed of the base address + 1 byte of prefix */
#define RADIO_CONFIG_DEFAULT_BA_LEN   3UL

/** Maximum size of a BLE advertisement packet payload (AdvA + AdvData) as per BT core spec version 4.2 */
#define RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE 37

/** Number of RF channels as per BT core spec version 4.2 */
#define RADIO_NO_RF_CHANNELS          40

#define RADIO_CONFIG_LOGICAL_ADDRS    8

#define RADIO_CONFIG_CRC_LEN 3

#if defined(NRF52_SERIES)
    /**
     * Maximum packet length supported by the radio
     * This includes: S0 + LENGTH + S1 + PAYLOAD
     */
    #define RADIO_MAX_PACKET_LEN          258

    #define PREAMBLE_LEN(MODE) (((MODE) == RADIO_MODE_BLE_2MBIT) ? 2 : 1)
    #define RADIO_TIME_PER_BYTE(MODE) (((MODE) == RADIO_MODE_NRF_2MBIT || (MODE) == RADIO_MODE_BLE_2MBIT) ? 4 : 8)

    #define RADIO_RAMPUP_TIME   (40)

    #define PREAMBLE_LEN(MODE) (((MODE) == RADIO_MODE_BLE_2MBIT) ? 2 : 1)

#elif defined(NRF51)
    /**
     * Maximum packet length supported by the radio
     * This includes: S0 + LENGTH + S1 + PAYLOAD
     */
    #define RADIO_MAX_PACKET_LEN          254

    #define RADIO_TIME_PER_BYTE(MODE) (((MODE) == RADIO_MODE_NRF_2MBIT) ? 4 : 8)
    #define PREAMBLE_LEN(MODE) (1)

    #define RADIO_RAMPUP_TIME   (132)

    #define PREAMBLE_LEN(MODE) (1)
#endif

#define ACCESS_ADDR_LEN     (4)

#define BLE_PACKET_OVERHEAD(MODE) (PREAMBLE_LEN(MODE) + ACCESS_ADDR_LEN + BLE_ADV_PACKET_HEADER_LENGTH + RADIO_CONFIG_CRC_LEN)
#define RADIO_ADDR_EVT_DELAY(MODE) ((PREAMBLE_LEN(MODE) + ACCESS_ADDR_LEN) * RADIO_TIME_PER_BYTE(MODE))

/**
 * @brief The available radio modes.
 */
typedef enum
{
    RADIO_MODE_NRF_1MBIT,   /*!< Proprietary 1Mbit/s mode*/
    RADIO_MODE_NRF_2MBIT,   /*!< High data rate */
    RADIO_MODE_NRF_250KBIT, /*!< Deprecated */
    RADIO_MODE_BLE_1MBIT,   /*!< BLE 1Mbit/s mode*/
#ifdef NRF52_SERIES
    RADIO_MODE_BLE_2MBIT,   /*!< BLE high data rate */
    RADIO_MODE_NRF_62K5BIT, /*!< Long range */
#endif
    RADIO_MODE_END
} radio_mode_t;

/**
 * @brief The tx power configuration values.
 *
 * @note NRF52 does not have -30DBm setting, instead it can be configured to -40DBm,
 * the value of -40DBm and -30DBm are kept the same for code compatibility.
 */
typedef enum
{
#if defined NRF52840_XXAA || defined NRF52833_XXAA
    RADIO_POWER_NRF_POS8DBM  = 0x08,
    RADIO_POWER_NRF_POS7DBM  = 0x07,
    RADIO_POWER_NRF_POS6DBM  = 0x06,
    RADIO_POWER_NRF_POS5DBM  = 0x05,
#endif
    RADIO_POWER_NRF_POS4DBM  = 0x04,
#ifdef NRF52_SERIES
    RADIO_POWER_NRF_POS3DBM  = 0x03,
#endif
#if defined NRF52840_XXAA || defined NRF52833_XXAA
    RADIO_POWER_NRF_POS2DBM  = 0x02,
#endif
    RADIO_POWER_NRF_0DBM     = 0x00,
    RADIO_POWER_NRF_NEG4DBM  = 0xFC,
    RADIO_POWER_NRF_NEG8DBM  = 0xF8,
    RADIO_POWER_NRF_NEG12DBM = 0xF4,
    RADIO_POWER_NRF_NEG16DBM = 0xF0,
    RADIO_POWER_NRF_NEG20DBM = 0xEC,
    RADIO_POWER_NRF_NEG30DBM = 0xD8,
#ifdef NRF52_SERIES
    RADIO_POWER_NRF_NEG40DBM = 0xD8
#endif
} radio_tx_power_t;

/**
 * @brief The radio configuration struct.
 * Each user of the radio should keep a copy of its own config struct and configure the radio
 * everytime they receive control of the radio from another user.
 *
 * @note Only one access address is provided by the config struct, this is the logical address 0,
 * which is what the TX and RX addresses are configured to use. A second access address can be
 * added via @ref radio_config_access_addr_set.
 */
typedef struct
{
    radio_tx_power_t tx_power;       /*!< Transmit power of the radio */
    uint16_t         payload_maxlen; /*!<Maximum length of radio packet payload see @ref RADIO_MAX_PACKET_LEN*/
    radio_mode_t     radio_mode;     /*!< Data rate and modulation */
} radio_config_t;

/**
 * @brief Resets the radio and configures the CRC and TIFS registers.
 *
 * @note Should be called at the start of each timeslot.
 *
 */
void radio_config_reset(void);

/**
 * @brief Sets the radio configuration registers.
 *
 * @note Should be called at the start of each session by the radio's active user.
 *
 * @param[in] p_config The radio configuration parameters as desired by the radio's user.
 *
 */
void radio_config_config(const radio_config_t* const p_config);

/**
 * @brief Changes the channel to transmit/receive on.
 *
 * @param[in] channel The channel number [0-39] as defined by the Bluetooth Core Specification v4.0.
 * Channel numbers outside this range will cause the assert handler to be called.
 *
 * @note \c PCNF0 is left untouched (configured for ADV packets on @ref radio_config_config,
 *  as MESH packets alwaysuse advertising PDU header.
 *
 */
void radio_config_channel_set(const uint32_t channel);

/**
 * @brief Sets the access address that can be used by the radio as the given logical address.
 *
 * @param[in] access_addr The access address to use for setting the radio logical address.
 * @param[in] logical_addr The logical address (0 or 1) of the radio to set.
 *
 * @note Only logical addresses 0 and 1 are valid, any other value will result in
 * the assert handler being called.
 *
 * @note \c TXADDRESS and \c RXADDRESSES registers of the radio are left untouched, which
 * are configured by the @ref radio_config_config to only use the logical address 0.
 *
 * @note This function only works if the base address length in \c PCNF1 is
 * set to @ref RADIO_CONFIG_DEFAULT_BA_LEN.
 *
 */
void radio_config_access_addr_set(const uint32_t access_addr, const uint32_t logical_addr);

/**
 * @brief Gets the access address registered as the relevant logical address (0-7) in radio.
 *
 * @note This function only works if the base address length in \c PCNF1 is
 * set to @ref RADIO_CONFIG_DEFAULT_BA_LEN.
 *
 * @param[in] logical_addr The logical address number (0 or 1) to extract as the access address.
 *
 * @note Only logical addresses 0 and 1 are valid, any other value will result in
 * the assert handler being called.
 *
 * @return The requested access address.
 */
uint32_t radio_config_access_addr_get(const uint32_t logical_addr);

/** @} */

#endif /* RADIO_CONFIG_H__ */
