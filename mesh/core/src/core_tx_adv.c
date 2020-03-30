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
#include <stdint.h>
#include "core_tx_adv.h"
#include "core_tx.h"
#include "advertiser.h"
#include "nrf_mesh_assert.h"
#include "mesh_opt_core.h"
#include "mesh_config_entry.h"
#include "app_util_platform.h"

typedef struct
{
    uint32_t adv_tx_count;
    advertiser_t advertiser;
} adv_bearer_role_t;

static adv_bearer_role_t m_bearer_roles[CORE_TX_ROLE_COUNT];

static struct
{
    core_tx_role_t role;
    adv_packet_t * p_packet;
} m_current_alloc;


static uint8_t m_originator_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR];

#if MESH_FEATURE_RELAY_ENABLED
static uint8_t m_relay_adv_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_RELAY];
#endif

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params);
static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length);
static void packet_discard(core_tx_bearer_t * p_bearer);

static const core_tx_bearer_interface_t m_interface = {packet_alloc,
                                                       packet_send,
                                                       packet_discard};
static core_tx_bearer_t m_bearer;

/*****************************************************************************
* Static functions
*****************************************************************************/

static void adv_tx_complete_callback(advertiser_t * p_adv,
                                     nrf_mesh_tx_token_t token,
                                     timestamp_t timestamp)
{
    core_tx_role_t role = (core_tx_role_t) (PARENT_BY_FIELD_GET(adv_bearer_role_t, advertiser, p_adv) - &m_bearer_roles[0]);
    core_tx_complete(&m_bearer, role, timestamp, token);
}

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet == NULL);

    m_current_alloc.p_packet =
        advertiser_packet_alloc(&m_bearer_roles[p_params->role].advertiser,
                                sizeof(ble_ad_header_t) + p_params->net_packet_len);

    if (m_current_alloc.p_packet == NULL)
    {
        return CORE_TX_ALLOC_FAIL_NO_MEM;
    }
    else
    {
        m_current_alloc.p_packet->token          = p_params->token;
        m_current_alloc.p_packet->config.repeats = m_bearer_roles[p_params->role].adv_tx_count;
        m_current_alloc.role                     = p_params->role;

        return CORE_TX_ALLOC_SUCCESS;
    }
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    /* Build the packet data */
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) &m_current_alloc.p_packet->packet.payload[0];
    p_ad_data->type           = AD_TYPE_MESH;
    p_ad_data->length         = BLE_AD_DATA_OVERHEAD + packet_length;
    memcpy(p_ad_data->data, p_packet, packet_length);

    advertiser_packet_send(&m_bearer_roles[m_current_alloc.role].advertiser,
                           m_current_alloc.p_packet);
    m_current_alloc.p_packet = NULL;
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    advertiser_packet_discard(&m_bearer_roles[m_current_alloc.role].advertiser,
                              m_current_alloc.p_packet);
    m_current_alloc.p_packet = NULL;
}

/*****************************************************************************
 * Core options interface
 *****************************************************************************/

static uint32_t core_tx_adv_set(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    const mesh_opt_core_adv_t * p_adv = p_entry;

    if (IS_IN_RANGE(p_adv->tx_count, 1, NETWORK_RELAY_RETRANSMITS_MAX + 1) &&
        IS_IN_RANGE(p_adv->tx_interval_ms,
                    BEARER_ADV_INT_MIN_MS,
                    MIN(BEARER_ADV_INT_MAX_MS, NETWORK_RELAY_INTERVAL_MAX_MS)))
    {
        bool is_enabled = advertiser_is_enabled(&m_bearer_roles[role].advertiser);
        if (p_adv->enabled && !is_enabled)
        {
            advertiser_enable(&m_bearer_roles[role].advertiser);
        }
        else if (!p_adv->enabled && is_enabled)
        {
            advertiser_disable(&m_bearer_roles[role].advertiser);
        }
        m_bearer_roles[role].adv_tx_count = p_adv->tx_count;
        advertiser_interval_set(&m_bearer_roles[role].advertiser, p_adv->tx_interval_ms);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_PARAM;
}

static void core_tx_adv_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    mesh_opt_core_adv_t * p_adv = p_entry;
    p_adv->enabled = advertiser_is_enabled(&m_bearer_roles[role].advertiser);
    p_adv->tx_count = m_bearer_roles[role].adv_tx_count;
    p_adv->tx_interval_ms = advertiser_interval_get(&m_bearer_roles[role].advertiser);
}

static void core_tx_adv_delete(mesh_config_entry_id_t entry_id)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);

    if (!advertiser_is_enabled(&m_bearer_roles[role].advertiser))
    {
        advertiser_enable(&m_bearer_roles[role].advertiser);
    }

#if MESH_FEATURE_RELAY_ENABLED
    if (role == CORE_TX_ROLE_RELAY)
    {
        m_bearer_roles[role].adv_tx_count = CORE_TX_REPEAT_RELAY_DEFAULT;
    }
    else
#endif
    {
        m_bearer_roles[role].adv_tx_count = CORE_TX_REPEAT_ORIGINATOR_DEFAULT;
    }
    advertiser_interval_set(&m_bearer_roles[role].advertiser, BEARER_ADV_INT_DEFAULT_MS);
}

static uint32_t core_tx_tx_power_set(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_TX_POWER_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    const radio_tx_power_t * p_tx_power = p_entry;
    advertiser_tx_power_set(&m_bearer_roles[role].advertiser, *p_tx_power);
    return NRF_SUCCESS;
}

static void core_tx_tx_power_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_TX_POWER_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    radio_tx_power_t * p_tx_power = p_entry;
    *p_tx_power = advertiser_tx_power_get(&m_bearer_roles[role].advertiser);
}

static void core_tx_tx_power_delete(mesh_config_entry_id_t entry_id)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_TX_POWER_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    advertiser_tx_power_set(&m_bearer_roles[role].advertiser, RADIO_POWER_NRF_0DBM);
}

static uint32_t core_tx_adv_addr_set(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_ADDR_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    const ble_gap_addr_t * p_addr = p_entry;
    if (p_addr->addr_type == BLE_GAP_ADDR_TYPE_PUBLIC ||
        p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
    {
        advertiser_address_set(&m_bearer_roles[role].advertiser, p_addr);
        return NRF_SUCCESS;
    }
    return NRF_ERROR_INVALID_PARAM;
}

static void core_tx_adv_addr_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_ADDR_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    ble_gap_addr_t * p_addr = p_entry;
    advertiser_address_get(&m_bearer_roles[role].advertiser, p_addr);
}

static void core_tx_adv_addr_delete(mesh_config_entry_id_t entry_id)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_ADDR_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);

    ble_gap_addr_t adv_addr;
    advertiser_address_default_get(&adv_addr);
    advertiser_address_set(&m_bearer_roles[role].advertiser, &adv_addr);
}

/*****************************************************************************
 * Wrapper functions
 *****************************************************************************/

MESH_CONFIG_ENTRY(mesh_opt_core_adv,
                  MESH_OPT_CORE_ADV_EID,
                  CORE_TX_ROLE_COUNT,
                  sizeof(mesh_opt_core_adv_t),
                  core_tx_adv_set,
                  core_tx_adv_get,
                  core_tx_adv_delete,
                  true);

MESH_CONFIG_ENTRY(mesh_opt_core_tx_power,
                  MESH_OPT_CORE_TX_POWER_EID,
                  CORE_TX_ROLE_COUNT,
                  sizeof(radio_tx_power_t),
                  core_tx_tx_power_set,
                  core_tx_tx_power_get,
                  core_tx_tx_power_delete,
                  true);

MESH_CONFIG_ENTRY(mesh_opt_core_adv_addr,
                  MESH_OPT_CORE_ADV_ADDR_EID,
                  CORE_TX_ROLE_COUNT,
                  sizeof(ble_gap_addr_t),
                  core_tx_adv_addr_set,
                  core_tx_adv_addr_get,
                  core_tx_adv_addr_delete,
                  true);


MESH_CONFIG_ENTRY_ARRAY_WRAPPER_DECLARE(mesh_opt_core_adv,
                                        MESH_OPT_CORE_ADV_EID,
                                        mesh_opt_core_adv_t,
                                        core_tx_role_t,
                                        CORE_TX_ROLE_COUNT)
MESH_CONFIG_ENTRY_ARRAY_WRAPPER_DECLARE(mesh_opt_core_adv_addr,
                                        MESH_OPT_CORE_ADV_ADDR_EID,
                                        ble_gap_addr_t,
                                        core_tx_role_t,
                                        CORE_TX_ROLE_COUNT)

uint32_t mesh_opt_core_tx_power_set(core_tx_role_t index, radio_tx_power_t tx_power)
{
    if (index >= (CORE_TX_ROLE_COUNT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    mesh_config_entry_id_t id = MESH_OPT_CORE_TX_POWER_EID;
    id.record += (uint16_t) index;
    return mesh_config_entry_set(id, &tx_power);
}

uint32_t mesh_opt_core_tx_power_get(core_tx_role_t index, radio_tx_power_t * p_tx_power)
{
    if (index >= (CORE_TX_ROLE_COUNT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    mesh_config_entry_id_t id = MESH_OPT_CORE_TX_POWER_EID;
    id.record += (uint16_t) index;
    return mesh_config_entry_get(id, p_tx_power);
}

uint32_t mesh_opt_core_tx_power_delete(core_tx_role_t index)
{
    if (index >= (CORE_TX_ROLE_COUNT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    mesh_config_entry_id_t id = MESH_OPT_CORE_TX_POWER_EID;
    id.record += (uint16_t) index;
    return mesh_config_entry_delete(id);
}
/*****************************************************************************
 * Interface functions
 *****************************************************************************/
void core_tx_adv_init(void)
{
    m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].adv_tx_count = CORE_TX_REPEAT_ORIGINATOR_DEFAULT;
    advertiser_instance_init(&m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].advertiser,
                             adv_tx_complete_callback,
                             m_originator_adv_packet_buffer,
                             sizeof(m_originator_adv_packet_buffer));
    advertiser_enable(&m_bearer_roles[CORE_TX_ROLE_ORIGINATOR].advertiser);

#if MESH_FEATURE_RELAY_ENABLED
    m_bearer_roles[CORE_TX_ROLE_RELAY].adv_tx_count = CORE_TX_REPEAT_RELAY_DEFAULT;
    advertiser_instance_init(&m_bearer_roles[CORE_TX_ROLE_RELAY].advertiser,
                             NULL,
                             m_relay_adv_packet_buffer,
                             sizeof(m_relay_adv_packet_buffer));
    advertiser_enable(&m_bearer_roles[CORE_TX_ROLE_RELAY].advertiser);
#endif

    core_tx_bearer_add(&m_bearer, &m_interface, CORE_TX_BEARER_TYPE_ADV);
}

bool core_tx_adv_is_enabled(core_tx_role_t role)
{
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    return advertiser_is_enabled(&m_bearer_roles[role].advertiser);
}
