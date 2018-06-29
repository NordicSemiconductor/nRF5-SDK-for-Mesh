/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

typedef struct
{
    list_node_t * p_list_head;
#ifdef AD_LISTENER_DEBUG_MODE
    /* To check integrity in case of multiple listeners for one AD type. */
    uint8_t     frame_hash;
#endif
} ad_subscribers_t;

static ad_subscribers_t  m_subscribers;

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

static ad_listener_t * item_by_ad_get(list_node_t * p_list, uint8_t ad)
{
    LIST_FOREACH(p_iterator, p_list)
    {
        ad_listener_t * p_listener = PARENT_BY_FIELD_GET(ad_listener_t, node, p_iterator);

        if (p_listener->ad_type == ad || p_listener->ad_type == ADL_WILDCARD_AD_TYPE)
        {
            return p_listener;
        }
    }

    return NULL;
}

static void ad_to_filter_add(ad_listener_t * p_adl)
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

static void ad_from_filter_remove(ad_listener_t * p_adl)
{
    if (item_by_ad_get(m_subscribers.p_list_head, ADL_WILDCARD_AD_TYPE) != NULL)
    {
        return;
    }

    if (p_adl->ad_type == ADL_WILDCARD_AD_TYPE)
    {
        bearer_adtype_clear();

        LIST_FOREACH(p_iterator, m_subscribers.p_list_head)
        {
            ad_listener_t * p_listener = PARENT_BY_FIELD_GET(ad_listener_t, node, p_iterator);
            bearer_adtype_add(p_listener->ad_type);
        }
    }
    else
    {
        bearer_adtype_remove(p_adl->ad_type);
    }
}

static uint32_t input_param_check(ad_listener_t * p_adl)
{
    if (p_adl == NULL || p_adl->handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_adl->adv_packet_type > BLE_PACKET_TYPE_ADV_DISCOVER_IND &&
        p_adl->adv_packet_type != ADL_WILDCARD_ADV_TYPE)    /*lint !e650 Constant '255' out of range for operator '!=' */
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

uint32_t ad_listener_subscribe(ad_listener_t * p_adl)
{
    uint32_t status = input_param_check(p_adl);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    bool empty = m_subscribers.p_list_head == NULL;
    ad_to_filter_add(p_adl);
    list_add(&m_subscribers.p_list_head, &p_adl->node);

    if (empty)
    {
        bearer_adtype_mode_set(AD_FILTER_WHITELIST_MODE);
        bearer_adtype_filtering_set(true);
    }

    return NRF_SUCCESS;
}

uint32_t ad_listener_unsubscribe(ad_listener_t * p_adl)
{
    uint32_t status = input_param_check(p_adl);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = list_remove(&m_subscribers.p_list_head, &p_adl->node);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    ad_from_filter_remove(p_adl);

    if (m_subscribers.p_list_head == NULL)
    {
        bearer_adtype_filtering_set(false);
    }

    return NRF_SUCCESS;
}

void ad_listener_process(ble_packet_type_t adv_type, const uint8_t * p_payload, uint32_t payload_length, const nrf_mesh_rx_metadata_t * p_metadata)
{
#ifdef AD_LISTENER_DEBUG_MODE
    m_subscribers.frame_hash = hash_count(p_payload, payload_length);
#endif

    for (ble_ad_data_t * p_ad_data = (ble_ad_data_t *)p_payload;
         (uint8_t *)p_ad_data < &p_payload[payload_length];
         p_ad_data = packet_ad_type_get_next((ble_ad_data_t *)p_ad_data))
    {
        list_node_t * p_list = m_subscribers.p_list_head;

        while (p_list != NULL)
        {
            ad_listener_t * p_listener = item_by_ad_get(p_list, p_ad_data->type);

            if (p_listener == NULL)
            {
                break;
            }

            p_list = p_listener->node.p_next;

            if (adv_type != p_listener->adv_packet_type &&
                p_listener->adv_packet_type != ADL_WILDCARD_ADV_TYPE)   /*lint !e650 Constant '255' out of range for operator '!=' */
            {
                continue;
            }

            p_listener->handler(p_ad_data->data, p_ad_data->length - BLE_AD_DATA_OVERHEAD, p_metadata);

#ifdef AD_LISTENER_DEBUG_MODE
            uint8_t hash = hash_count(p_payload, payload_length);
            NRF_MESH_ASSERT(hash == m_subscribers.frame_hash);
#endif
         }
     }
}
