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

#include "ad_listener.h"
#include "list.h"
#include "utils.h"
#include "ad_type_filter.h"

#ifdef UNIT_TEST
NRF_MESH_SECTION_DEF_FLASH(ad_listeners, ad_listener_t);
#else
NRF_MESH_SECTION_DEF_FLASH(ad_listeners, const ad_listener_t);
#endif

#ifdef AD_LISTENER_DEBUG_MODE
/* longitudinal redundancy check x^8+1 */
static uint8_t hash_count(const uint8_t * p_data, uint8_t size)
{
    uint8_t lrc = 0;

    for (uint8_t i = 0; i < size; i++)
    {
        lrc += p_data[i];
    }

    return (lrc ^ 0xFF) + 1;
}
#endif

static void ad_to_filter_add(const ad_listener_t * p_adl)
{
    if (p_adl->ad_type == ADL_WILDCARD_AD_TYPE)
    {
        for (uint16_t i = 0; i <= UINT8_MAX; i++)
        {
            bearer_adtype_add(i);
        }
    }
    else
    {
        bearer_adtype_add(p_adl->ad_type);
    }
}

static uint32_t input_param_check(const ad_listener_t * p_adl)
{
    if (p_adl == NULL || p_adl->handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_adl->adv_packet_type > BLE_PACKET_TYPE_ADV_EXT && /*lint !e685 Relational operator '>' always evaluates to 'false' */
        p_adl->adv_packet_type != ADL_WILDCARD_ADV_TYPE)    /*lint !e650 Constant '255' out of range for operator '!=' */
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

void ad_listener_init(void)
{
    bearer_adtype_mode_set(AD_FILTER_WHITELIST_MODE);
    bearer_adtype_filtering_set(true);

    NRF_MESH_SECTION_FOR_EACH(ad_listeners, const ad_listener_t, p_listener)
    {
        /* Check that all AD-listeners are correctly configured: */
        NRF_MESH_ERROR_CHECK(input_param_check(p_listener));
        ad_to_filter_add(p_listener);
    }
}

void ad_listener_process(ble_packet_type_t adv_type, const uint8_t * p_payload, uint32_t payload_length, const nrf_mesh_rx_metadata_t * p_metadata)
{
#ifdef AD_LISTENER_DEBUG_MODE
    uint8_t frame_hash = hash_count(p_payload, payload_length);
#endif

    for (ble_ad_data_t * p_ad_data = (ble_ad_data_t *)p_payload;
         (uint8_t *)p_ad_data < &p_payload[payload_length] && p_ad_data->length > 0;
         p_ad_data = packet_ad_type_get_next((ble_ad_data_t *)p_ad_data))
    {
        NRF_MESH_SECTION_FOR_EACH(ad_listeners, const ad_listener_t, p_listener)
        {
            if ((adv_type != p_listener->adv_packet_type && (uint8_t) p_listener->adv_packet_type != ADL_WILDCARD_ADV_TYPE) ||
                (p_listener->ad_type != p_ad_data->type && p_listener->ad_type != ADL_WILDCARD_AD_TYPE))
            {
                continue;
            }

            p_listener->handler(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, p_metadata);

#ifdef AD_LISTENER_DEBUG_MODE
            NRF_MESH_ASSERT(hash_count(p_payload, payload_length) == frame_hash);
#endif
         }
     }
}
