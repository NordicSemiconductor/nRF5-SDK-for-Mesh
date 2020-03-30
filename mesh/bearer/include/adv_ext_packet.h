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
#ifndef ADV_EXT_PACKET_H__
#define ADV_EXT_PACKET_H__

/**
 * @internal
 * @defgroup ADV_EXT_PACKET Advertising Extension packet formats
 * @ingroup INSTABURST
 * @{
 */

#include <stdint.h>
#include "packet.h"
#include "radio_config.h"
#include "nrf_mesh_assert.h"

#define ADV_EXT_PACKET_LEN_MAX 255
#define ADV_EXT_CHANNEL_MAX 36

#define ADV_EXT_IND_FIELDS_REQUIRED         ((adv_ext_header_bitfield_t) (ADV_EXT_HEADER_ADI_BIT | ADV_EXT_HEADER_AUX_PTR_BIT))
#define AUX_ADV_IND_FIELDS_REQUIRED         ((adv_ext_header_bitfield_t) (ADV_EXT_HEADER_ADI_BIT))
#define AUX_CHAIN_IND_FIELDS_REQUIRED       ((adv_ext_header_bitfield_t) 0)

/**
 * Buffer overhead of an adv ext packet with the given fields
 *
 * @param[in] FIELDS Fields in the header
 */
#define ADV_EXT_OVERHEAD(FIELDS)    (sizeof(adv_ext_header_t) + ADV_EXT_HEADER_LEN(FIELDS))

#define ADV_EXT_HEADER_LEN(FIELDS) (                                                   \
    ((FIELDS == ADV_EXT_HEADER_EMPTY) ? 0 : 1) +                               \
    ((FIELDS & ADV_EXT_HEADER_ADV_ADDR_BIT) ? ADV_EXT_HEADER_ADV_ADDR_LEN : 0) +       \
    ((FIELDS & ADV_EXT_HEADER_TARGET_ADDR_BIT) ? ADV_EXT_HEADER_TARGET_ADDR_LEN : 0) + \
    ((FIELDS & ADV_EXT_HEADER_ADI_BIT) ? ADV_EXT_HEADER_ADI_LEN : 0) +                 \
    ((FIELDS & ADV_EXT_HEADER_AUX_PTR_BIT) ? ADV_EXT_HEADER_AUX_PTR_LEN : 0) +         \
    ((FIELDS & ADV_EXT_HEADER_TX_POWER_BIT) ? ADV_EXT_HEADER_TX_POWER_LEN : 0))

#define ADV_EXT_HEADER_ADV_ADDR_LEN     6
#define ADV_EXT_HEADER_TARGET_ADDR_LEN  6
#define ADV_EXT_HEADER_ADI_LEN          2
#define ADV_EXT_HEADER_AUX_PTR_LEN      3
#define ADV_EXT_HEADER_SYNC_INFO_LEN    18
#define ADV_EXT_HEADER_TX_POWER_LEN     1

typedef enum
{
    ADV_EXT_HEADER_ADV_MODE_NONCONN,
    ADV_EXT_HEADER_ADV_MODE_CONNECTABLE,
    ADV_EXT_HEADER_ADV_MODE_SCANNABLE,
} adv_ext_header_adv_mode_t;

typedef enum
{
    ADV_EXT_HEADER_ADV_ADDR    = 0,
    ADV_EXT_HEADER_TARGET_ADDR = 1,
    ADV_EXT_HEADER_ADI         = 3,
    ADV_EXT_HEADER_AUX_PTR     = 4,
    ADV_EXT_HEADER_SYNC_INFO   = 5,
    ADV_EXT_HEADER_TX_POWER    = 6,

    ADV_EXT_HEADER_FIELD_COUNT /**< Number of fields, RFU included. */
} adv_ext_header_field_t;

typedef enum
{
    ADV_EXT_HEADER_EMPTY,
    ADV_EXT_HEADER_ADV_ADDR_BIT    = (1 << ADV_EXT_HEADER_ADV_ADDR),
    ADV_EXT_HEADER_TARGET_ADDR_BIT = (1 << ADV_EXT_HEADER_TARGET_ADDR),
    ADV_EXT_HEADER_ADI_BIT         = (1 << ADV_EXT_HEADER_ADI),
    ADV_EXT_HEADER_AUX_PTR_BIT     = (1 << ADV_EXT_HEADER_AUX_PTR),
    ADV_EXT_HEADER_SYNC_INFO_BIT   = (1 << ADV_EXT_HEADER_SYNC_INFO),
    ADV_EXT_HEADER_TX_POWER_BIT    = (1 << ADV_EXT_HEADER_TX_POWER),
} adv_ext_header_bitfield_t;


typedef struct __attribute__((packed))
{
    uint8_t header_len: 6;
    uint8_t adv_mode: 2; /**< See @ref adv_ext_header_adv_mode_t for valid values. */
    uint8_t header[];
} adv_ext_header_t;

typedef struct __attribute__((packed))
{
    ble_packet_hdr_t header;
    uint8_t data[]; /**< Adv Ext header and (optionally) Adv data */
} adv_ext_packet_t;

typedef enum
{
    ADV_EXT_CLOCK_ACCURACY_51_to_500ppm,
    ADV_EXT_CLOCK_ACCURACY_0_to_50ppm
} adv_ext_clock_accuracy_t;

typedef enum
{
    ADV_EXT_SLEEP_CLOCK_ACCURACY_251_to_500ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_151_to_250ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_101_to_150ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_76_to_100ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_51_to_75ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_31_to_50ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_21_to_30ppm,
    ADV_EXT_SLEEP_CLOCK_ACCURACY_0_to_20ppm,
} adv_ext_sleep_clock_accuracy_t;

typedef enum
{
    ADV_EXT_TIME_OFFSET_UNIT_30us = 30,
    ADV_EXT_TIME_OFFSET_UNIT_300us = 300
} adv_ext_time_offset_unit_t;

typedef struct
{
    uint8_t channel_index;
    adv_ext_clock_accuracy_t clock_accuracy;
    adv_ext_time_offset_unit_t offset_units_us;
    uint16_t time_offset_us; /**< Should be divisible by the @p offset_units_us. */
    radio_mode_t radio_mode;
} adv_ext_header_aux_ptr_t;

typedef struct
{
    uint16_t time_offset_us; /**< Should be divisible by the @p offset_units_us. */
    adv_ext_time_offset_unit_t offset_units_us;
    uint16_t time_interval_us;
    uint64_t channel_map;
    uint32_t access_address;
    adv_ext_sleep_clock_accuracy_t sleep_clock_accuracy;
    uint32_t crc_init;
    uint16_t event_counter;
} adv_ext_header_sync_info_t;

typedef struct
{
    int8_t tx_power;
} adv_ext_header_tx_power_t;

typedef union
{
    ble_gap_addr_t adv_addr;
    ble_gap_addr_t target_addr;
    nrf_mesh_instaburst_event_id_t adi;
    adv_ext_header_aux_ptr_t aux_ptr;
    adv_ext_header_sync_info_t sync_info;
    adv_ext_header_tx_power_t tx_power;
} adv_ext_header_data_t;

typedef struct
{
    const ble_gap_addr_t * p_adv_addr;
    const ble_gap_addr_t * p_target_addr;
    const nrf_mesh_instaburst_event_id_t * p_adi;
    const adv_ext_header_aux_ptr_t * p_aux_ptr;
    const adv_ext_header_sync_info_t * p_sync_info;
    const adv_ext_header_tx_power_t * p_tx_power;
} adv_ext_header_set_t;


static inline uint32_t adv_ext_header_clock_drift_max(adv_ext_clock_accuracy_t accuracy)
{
    switch (accuracy)
    {
        case ADV_EXT_CLOCK_ACCURACY_51_to_500ppm:
            return 500;
        case ADV_EXT_CLOCK_ACCURACY_0_to_50ppm:
            return 50;
        default:
            NRF_MESH_ASSERT(false);
    }
    return 0;
}

static inline uint8_t adv_ext_header_len(adv_ext_header_bitfield_t fields)
{
    return ADV_EXT_HEADER_LEN(fields);
}

static inline const uint8_t * adv_ext_packet_adv_data_get(const adv_ext_packet_t * p_packet)
{
    adv_ext_header_t * p_extended_header = (adv_ext_header_t *) p_packet->data;
    if (p_extended_header->header_len + 1 >= p_packet->header.length)
    {
        return NULL;
    }
    else
    {
        return &p_packet->data[p_extended_header->header_len + 1];
    }
}

static inline adv_ext_header_bitfield_t adv_ext_header_bitfield_get(const adv_ext_header_t * p_header)
{
    if (p_header != NULL && p_header->header_len > 0)
    {
        return (adv_ext_header_bitfield_t) p_header->header[0];
    }
    else
    {
        return (adv_ext_header_bitfield_t) 0;
    }
}

/**
 * Generates a full adv ext header based on the given header set.
 *
 * @param[in,out] p_header Header to generate.
 * @param[in] adv_mode Adv mode to set.
 * @param[in] p_header_set The set of headers to use. All unused headers must be NULL.
 */
void adv_ext_header_generate(adv_ext_header_t * p_header,
                             adv_ext_header_adv_mode_t adv_mode,
                             const adv_ext_header_set_t * p_header_set);

/**
 * Gets field data from header.
 *
 * @param[in] p_header Header to get from.
 * @param[in] field Field to get.
 * @param[in,out] p_data Pointer to data structure to populate.
 *
 * @retval NRF_SUCCESS Found and populated the field data.
 * @retval NRF_ERROR_NOT_FOUND The given header did not contain the specified field.
 * @retval NRF_ERROR_NULL One or more parameter was NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The given field type isn't supported.
 */
uint32_t adv_ext_header_data_get(const adv_ext_header_t * p_header,
                                 adv_ext_header_field_t field,
                                 adv_ext_header_data_t * p_data);

/** @} */

#endif /* ADV_EXT_PACKET_H__ */
