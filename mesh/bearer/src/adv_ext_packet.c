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
#include "adv_ext_packet.h"
#include "nordic_common.h"

#define ADV_EXT_PACKET_FORMAT_US_TO_TIME_UNIT(us) (((us) == ADV_EXT_TIME_OFFSET_UNIT_30us) ? 0 : 1)
#define ADV_EXT_PACKET_FORMAT_TIME_UNIT_TO_US(unit) (((unit) == 0) ? ADV_EXT_TIME_OFFSET_UNIT_30us : ADV_EXT_TIME_OFFSET_UNIT_300us)
#define ADV_EXT_PACKET_FORMAT_INTERVAL_TO_US(interval) ((interval) * 1250)
#define ADV_EXT_PACKET_FORMAT_US_TO_INTERVAL(us) ((us) / 1250)

typedef enum
{
    ADV_EXT_PHY_LE_1M,
    ADV_EXT_PHY_LE_2M,
    ADV_EXT_PHY_LE_CODED
} adv_ext_phy_t;


typedef struct __attribute__((packed))
{
    uint8_t advertising_address[BLE_GAP_ADDR_LEN];
} adv_ext_packet_format_adv_a_t;

typedef adv_ext_packet_format_adv_a_t adv_ext_packet_format_target_a_t;

typedef struct __attribute__((packed))
{
    uint16_t advertising_data_id : 12;
    uint16_t advertising_set_id  : 4;
} adv_ext_packet_format_adi_t;

typedef struct __attribute__((packed))
{
    uint8_t channel_index : 6;
    uint8_t ca : 1;
    uint8_t offset_units : 1;
    uint16_t aux_offset : 13;
    uint16_t aux_phy : 3;
} adv_ext_packet_format_aux_ptr_t;

typedef struct __attribute__((packed))
{
    uint16_t sync_packet_offset : 13;
    uint16_t offset_units : 1;
    uint16_t _rfu : 2;
    uint16_t interval;
    uint64_t ch_m : 37;
    uint8_t sca : 3;
    uint32_t aa;
    uint32_t crc_init : 24;
    uint16_t event_counter;
} adv_ext_packet_format_sync_info_t;

typedef struct __attribute__((packed))
{
    int8_t tx_power_level;
} adv_ext_packet_format_tx_power_t;

typedef union __attribute__((packed))
{
    adv_ext_packet_format_adv_a_t adv_a;
    adv_ext_packet_format_target_a_t target_a;
    adv_ext_packet_format_adi_t adi;
    adv_ext_packet_format_aux_ptr_t aux_ptr;
    adv_ext_packet_format_sync_info_t sync_info;
    adv_ext_packet_format_tx_power_t tx_power;
} adv_ext_packet_format_t;

/** Verify that the size defines in the header match the structs */
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_adv_a_t) == ADV_EXT_HEADER_ADV_ADDR_LEN);
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_target_a_t) == ADV_EXT_HEADER_TARGET_ADDR_LEN);
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_adi_t) == ADV_EXT_HEADER_ADI_LEN);
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_aux_ptr_t) == ADV_EXT_HEADER_AUX_PTR_LEN);
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_sync_info_t) == ADV_EXT_HEADER_SYNC_INFO_LEN);
NRF_MESH_STATIC_ASSERT(sizeof(adv_ext_packet_format_tx_power_t) == ADV_EXT_HEADER_TX_POWER_LEN);

static inline void adv_ext_header_bitfield_set(adv_ext_header_t * p_header, adv_ext_header_bitfield_t bitfield)
{
    p_header->header[0] = bitfield;
}

static inline const uint8_t * adv_ext_header_field_get(const adv_ext_header_t * p_header, adv_ext_header_field_t field)
{
    adv_ext_header_bitfield_t header_bitfield = adv_ext_header_bitfield_get(p_header);
    if ((header_bitfield & (1 << field)) == 0)
    {
        return NULL;
    }
    /* To get the offset of the header, get the length of a header containing all the fields before
     * the desired field. We achieve this by bitwise and-ing the header's bitfield with the bits
     * before the desired field. */
    return &p_header->header[adv_ext_header_len((adv_ext_header_bitfield_t) (header_bitfield & ((1 << field) - 1)))];
}

static void adv_ext_header_adv_addr_set(uint8_t * p_buffer, const ble_gap_addr_t * p_data)
{
    memcpy(((adv_ext_packet_format_adv_a_t *) p_buffer)->advertising_address, p_data->addr, sizeof(p_data->addr));
}

static void adv_ext_header_target_addr_set(uint8_t * p_buffer, const ble_gap_addr_t * p_data)
{
    memcpy(((adv_ext_packet_format_target_a_t *) p_buffer)->advertising_address, p_data->addr, sizeof(p_data->addr));
}

static void adv_ext_header_adi_set(uint8_t * p_buffer, const nrf_mesh_instaburst_event_id_t * p_data)
{
    adv_ext_packet_format_adi_t * p_adi = (adv_ext_packet_format_adi_t *) p_buffer;
    p_adi->advertising_data_id          = p_data->data_id;
    p_adi->advertising_set_id           = p_data->set_id;
}

static void adv_ext_header_aux_ptr_set(uint8_t * p_buffer, const adv_ext_header_aux_ptr_t * p_data)
{
    NRF_MESH_ASSERT(p_data->radio_mode == RADIO_MODE_BLE_1MBIT ||
                    p_data->radio_mode == RADIO_MODE_BLE_2MBIT);

    adv_ext_phy_t phy = (p_data->radio_mode == RADIO_MODE_BLE_1MBIT) ? ADV_EXT_PHY_LE_1M : ADV_EXT_PHY_LE_2M;

    adv_ext_packet_format_aux_ptr_t * p_aux_ptr = (adv_ext_packet_format_aux_ptr_t *) p_buffer;

    p_aux_ptr->aux_offset    = p_data->time_offset_us / p_data->offset_units_us;
    p_aux_ptr->aux_phy       = phy;
    p_aux_ptr->ca            = p_data->clock_accuracy;
    p_aux_ptr->channel_index = p_data->channel_index;
    p_aux_ptr->offset_units  = ADV_EXT_PACKET_FORMAT_US_TO_TIME_UNIT(p_data->offset_units_us);
}

static void adv_ext_header_sync_info_set(uint8_t * p_buffer, const adv_ext_header_sync_info_t * p_data)
{
    adv_ext_packet_format_sync_info_t * p_sync_info = (adv_ext_packet_format_sync_info_t *) p_buffer;

    p_sync_info->_rfu          = 0;
    p_sync_info->aa            = p_data->access_address;
    p_sync_info->ch_m          = p_data->channel_map;
    p_sync_info->crc_init      = p_data->crc_init;
    p_sync_info->event_counter = p_data->event_counter;
    p_sync_info->interval      = ADV_EXT_PACKET_FORMAT_US_TO_INTERVAL(p_data->time_interval_us);
    p_sync_info->offset_units  = ADV_EXT_PACKET_FORMAT_US_TO_TIME_UNIT(p_data->offset_units_us);
    p_sync_info->sca           = p_data->sleep_clock_accuracy;
    p_sync_info->sync_packet_offset = p_data->time_offset_us / p_data->offset_units_us;
}

static void adv_ext_header_tx_power_set(uint8_t * p_buffer, const adv_ext_header_tx_power_t * p_data)
{
    adv_ext_packet_format_tx_power_t * p_tx_power = (adv_ext_packet_format_tx_power_t *) p_buffer;

    p_tx_power->tx_power_level = p_data->tx_power;
}


static uint32_t adv_ext_header_adv_addr_get(ble_gap_addr_t * p_data, const uint8_t * p_source)
{
    memcpy(p_data->addr, p_source, BLE_GAP_ADDR_LEN);
    return NRF_SUCCESS;
}

static uint32_t adv_ext_header_target_addr_get(ble_gap_addr_t * p_data, const uint8_t * p_source)
{
    memcpy(p_data->addr, p_source, BLE_GAP_ADDR_LEN);
    return NRF_SUCCESS;
}

static uint32_t adv_ext_header_aux_ptr_get(adv_ext_header_aux_ptr_t * p_data, const uint8_t * p_source)
{
    adv_ext_packet_format_aux_ptr_t * p_aux_ptr = (adv_ext_packet_format_aux_ptr_t *) p_source;

    if (p_aux_ptr->aux_phy == ADV_EXT_PHY_LE_CODED)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    if (p_aux_ptr->channel_index > ADV_EXT_CHANNEL_MAX)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    p_data->channel_index     = p_aux_ptr->channel_index;
    p_data->clock_accuracy    = (adv_ext_clock_accuracy_t) p_aux_ptr->ca;
    p_data->offset_units_us   = ADV_EXT_PACKET_FORMAT_TIME_UNIT_TO_US(p_aux_ptr->offset_units);
    p_data->time_offset_us    = p_aux_ptr->aux_offset * p_data->offset_units_us;

    p_data->radio_mode = (p_aux_ptr->aux_phy == ADV_EXT_PHY_LE_1M) ? RADIO_MODE_BLE_1MBIT : RADIO_MODE_BLE_2MBIT;

    return NRF_SUCCESS;
}

static uint32_t adv_ext_header_adi_get(nrf_mesh_instaburst_event_id_t * p_data, const uint8_t * p_source)
{
    adv_ext_packet_format_adi_t * p_adi = (adv_ext_packet_format_adi_t *) p_source;
    p_data->data_id = p_adi->advertising_data_id;
    p_data->set_id  = p_adi->advertising_set_id;
    return NRF_SUCCESS;
}

static uint32_t adv_ext_header_sync_info_get(adv_ext_header_sync_info_t * p_data, const uint8_t * p_source)
{
    adv_ext_packet_format_sync_info_t * p_sync_info = (adv_ext_packet_format_sync_info_t *) p_source;

    p_data->access_address       = p_sync_info->aa;
    p_data->channel_map          = p_sync_info->ch_m;
    p_data->crc_init             = p_sync_info->crc_init;
    p_data->event_counter        = p_sync_info->event_counter;
    p_data->offset_units_us      = ADV_EXT_PACKET_FORMAT_TIME_UNIT_TO_US(p_sync_info->offset_units);
    p_data->sleep_clock_accuracy = (adv_ext_sleep_clock_accuracy_t) p_sync_info->sca;
    p_data->time_interval_us     = ADV_EXT_PACKET_FORMAT_INTERVAL_TO_US(p_sync_info->interval);
    p_data->time_offset_us       = p_sync_info->sync_packet_offset * p_data->offset_units_us;
    return NRF_SUCCESS;
}

static uint32_t adv_ext_header_tx_power_get(adv_ext_header_tx_power_t * p_data, const uint8_t * p_source)
{
    adv_ext_packet_format_tx_power_t * p_tx_power = (adv_ext_packet_format_tx_power_t *) p_source;
    p_data->tx_power = p_tx_power->tx_power_level;
    return NRF_SUCCESS;
}

static void adv_ext_header_field_set(adv_ext_header_t * p_header,
                                     adv_ext_header_field_t field,
                                     const adv_ext_header_data_t * p_data)
{

    NRF_MESH_ASSERT(adv_ext_header_bitfield_get(p_header) & (1 << field));

    uint8_t * p_buffer = (uint8_t *) adv_ext_header_field_get(p_header, field);

    switch (field)
    {
        case ADV_EXT_HEADER_ADV_ADDR:
            adv_ext_header_adv_addr_set(p_buffer, &p_data->adv_addr);
            break;
        case ADV_EXT_HEADER_TARGET_ADDR:
            adv_ext_header_target_addr_set(p_buffer, &p_data->target_addr);
            break;
        case ADV_EXT_HEADER_ADI:
            adv_ext_header_adi_set(p_buffer, &p_data->adi);
            break;
        case ADV_EXT_HEADER_AUX_PTR:
            adv_ext_header_aux_ptr_set(p_buffer, &p_data->aux_ptr);
            break;
        case ADV_EXT_HEADER_SYNC_INFO:
            adv_ext_header_sync_info_set(p_buffer, &p_data->sync_info);
            break;
        case ADV_EXT_HEADER_TX_POWER:
            adv_ext_header_tx_power_set(p_buffer, &p_data->tx_power);
            break;
        default:
            /* not implemented */
            NRF_MESH_ASSERT(false);
    }
}
/*****************************************************************************
* Interface functions
*****************************************************************************/

void adv_ext_header_generate(adv_ext_header_t * p_header,
                             adv_ext_header_adv_mode_t adv_mode,
                             const adv_ext_header_set_t * p_header_set)
{
    /* Corresponds to the field indexes in the header bitfield. */
    const adv_ext_header_data_t * p_header_data[ADV_EXT_HEADER_FIELD_COUNT];
    memset(p_header_data, 0, sizeof(p_header_data));
    p_header_data[ADV_EXT_HEADER_ADV_ADDR]    = (const adv_ext_header_data_t *) p_header_set->p_adv_addr;
    p_header_data[ADV_EXT_HEADER_TARGET_ADDR] = (const adv_ext_header_data_t *) p_header_set->p_target_addr;
    p_header_data[ADV_EXT_HEADER_ADI]         = (const adv_ext_header_data_t *) p_header_set->p_adi;
    p_header_data[ADV_EXT_HEADER_AUX_PTR]     = (const adv_ext_header_data_t *) p_header_set->p_aux_ptr;
    p_header_data[ADV_EXT_HEADER_SYNC_INFO]   = (const adv_ext_header_data_t *) p_header_set->p_sync_info;
    p_header_data[ADV_EXT_HEADER_TX_POWER]    = (const adv_ext_header_data_t *) p_header_set->p_tx_power;

    adv_ext_header_bitfield_t bitfield = (adv_ext_header_bitfield_t) 0;
    for (uint32_t i = 0; i < ARRAY_SIZE(p_header_data); ++i)
    {
        if (p_header_data[i] != NULL)
        {
            bitfield |= (1 << i);
            p_header->header_len = adv_ext_header_len(bitfield);
            adv_ext_header_bitfield_set(p_header, bitfield);
            adv_ext_header_field_set(p_header, (adv_ext_header_field_t) i, p_header_data[i]);
        }
    }
    p_header->header_len = ADV_EXT_HEADER_LEN(bitfield);
    p_header->adv_mode   = adv_mode;
}


uint32_t adv_ext_header_data_get(const adv_ext_header_t * p_header,
                                 adv_ext_header_field_t field,
                                 adv_ext_header_data_t * p_data)
{
    if (p_header == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    adv_ext_header_bitfield_t bitfield = adv_ext_header_bitfield_get(p_header);
    uint32_t header_min_len = adv_ext_header_len((adv_ext_header_bitfield_t) (bitfield & ((1 << (field + 1)) - 1)));

    if ((bitfield & (1 << field)) &&
        (p_header->header_len >= header_min_len))
    {
        const uint8_t * p_source = adv_ext_header_field_get(p_header, field);

        switch (field)
        {
            case ADV_EXT_HEADER_ADV_ADDR:
                return adv_ext_header_adv_addr_get(&p_data->adv_addr, p_source);
            case ADV_EXT_HEADER_TARGET_ADDR:
                return adv_ext_header_target_addr_get(&p_data->target_addr, p_source);
            case ADV_EXT_HEADER_ADI:
                return adv_ext_header_adi_get(&p_data->adi, p_source);
            case ADV_EXT_HEADER_AUX_PTR:
                return adv_ext_header_aux_ptr_get(&p_data->aux_ptr, p_source);
            case ADV_EXT_HEADER_SYNC_INFO:
                return adv_ext_header_sync_info_get(&p_data->sync_info, p_source);
            case ADV_EXT_HEADER_TX_POWER:
                return adv_ext_header_tx_power_get(&p_data->tx_power, p_source);
            default:
                return NRF_ERROR_NOT_SUPPORTED;
        }
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}
