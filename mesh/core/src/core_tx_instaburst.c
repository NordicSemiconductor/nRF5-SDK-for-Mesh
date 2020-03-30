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
#include "core_tx_instaburst.h"
#include "core_tx.h"
#include "instaburst_tx.h"
#include "nrf_mesh_assert.h"
#include "nordic_common.h"
#include "nrf_mesh_config_bearer.h"
#include "mesh_opt_core.h"
#include "app_util_platform.h"
#include "advertiser.h"

static instaburst_tx_t m_instaburst[CORE_TX_ROLE_COUNT];

static struct
{
    core_tx_role_t role;
    uint8_t * p_packet;
} m_current_alloc;

static uint8_t m_originator_instaburst_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR];
static uint8_t m_relay_instaburst_packet_buffer[CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY];

static const uint8_t m_instaburst_channels[] = CORE_TX_INSTABURST_CHANNELS;


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

static void instaburst_tx_complete_callback(instaburst_tx_t * p_instaburst,
                                            nrf_mesh_tx_token_t token,
                                            uint32_t timestamp)
{
    core_tx_role_t role = (core_tx_role_t) (p_instaburst - &m_instaburst[0]);
    core_tx_complete(&m_bearer, role, timestamp, token);
}

static core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet == NULL);

    m_current_alloc.p_packet = instaburst_tx_buffer_alloc(&m_instaburst[p_params->role],
                                                          sizeof(ble_ad_header_t) + p_params->net_packet_len,
                                                          p_params->token);
    m_current_alloc.role     = p_params->role;

    return ((m_current_alloc.p_packet != NULL) ? CORE_TX_ALLOC_SUCCESS : CORE_TX_ALLOC_FAIL_NO_MEM);
}

static void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) m_current_alloc.p_packet;
    p_ad_data->type = AD_TYPE_MESH;
    p_ad_data->length = BLE_AD_DATA_OVERHEAD + packet_length;
    memcpy(p_ad_data->data, p_packet, packet_length);

    instaburst_tx_buffer_commit(&m_instaburst[m_current_alloc.role],
                                m_current_alloc.p_packet);

    m_current_alloc.p_packet = NULL;
}

static void packet_discard(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer == &m_bearer);
    NRF_MESH_ASSERT(m_current_alloc.p_packet != NULL);

    instaburst_tx_buffer_discard(&m_instaburst[m_current_alloc.role],
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
        bool is_enabled = instaburst_tx_is_enabled(&m_instaburst[role]);
        if (p_adv->enabled && !is_enabled)
        {
            instaburst_tx_enable(&m_instaburst[role]);
        }
        else if (!p_adv->enabled && is_enabled)
        {
            instaburst_tx_disable(&m_instaburst[role]);
        }
        /* TODO: Add support for TX count (MBTLE-2562). */
        instaburst_tx_interval_set(&m_instaburst[role], p_adv->tx_interval_ms);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_PARAM;
}

static void core_tx_adv_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    mesh_opt_core_adv_t * p_adv = p_entry;
    p_adv->enabled = instaburst_tx_is_enabled(&m_instaburst[role]);
    /* TODO: Add support for TX count (MBTLE-2562). */
    p_adv->tx_count = 1;
    p_adv->tx_interval_ms = instaburst_tx_interval_get(&m_instaburst[role]);
}

static uint32_t core_tx_tx_power_set(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    const radio_tx_power_t * p_tx_power = p_entry;
    instaburst_tx_tx_power_set(&m_instaburst[role], *p_tx_power);
    return NRF_SUCCESS;
}

static void core_tx_tx_power_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    radio_tx_power_t * p_tx_power = p_entry;
    *p_tx_power = instaburst_tx_tx_power_get(&m_instaburst[role]);
}

static uint32_t core_tx_adv_addr_set(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    const ble_gap_addr_t * p_addr = p_entry;
    if (p_addr->addr_type == BLE_GAP_ADDR_TYPE_PUBLIC ||
        p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC)
    {
        /* TODO: Support setting GAP address (MBTLE-2563). */
        return NRF_ERROR_NOT_SUPPORTED;
    }
    return NRF_ERROR_INVALID_PARAM;
}

static void core_tx_adv_addr_get(mesh_config_entry_id_t entry_id, void * p_entry)
{
    core_tx_role_t role = (core_tx_role_t) (entry_id.record - MESH_OPT_CORE_ADV_RECORD_START);
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    ble_gap_addr_t * p_addr = p_entry;
    advertiser_address_default_get(p_addr);
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
                  NULL,
                  true);

MESH_CONFIG_ENTRY(mesh_opt_core_tx_power,
                  MESH_OPT_CORE_TX_POWER_EID,
                  CORE_TX_ROLE_COUNT,
                  sizeof(radio_tx_power_t),
                  core_tx_tx_power_set,
                  core_tx_tx_power_get,
                  NULL,
                  true);

MESH_CONFIG_ENTRY(mesh_opt_core_adv_addr,
                  MESH_OPT_CORE_ADV_ADDR_EID,
                  CORE_TX_ROLE_COUNT,
                  sizeof(ble_gap_addr_t),
                  core_tx_adv_addr_set,
                  core_tx_adv_addr_get,
                  NULL,
                  true);        /* TODO: Support setting GAP address (MBTLE-2563). */


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
void core_tx_instaburst_init(void)
{
    instaburst_tx_config_t instaburst_config;
    instaburst_config.channel_count  = ARRAY_SIZE(m_instaburst_channels);
    instaburst_config.p_channels     = m_instaburst_channels;
    instaburst_config.radio_mode     = RADIO_MODE_BLE_2MBIT;
    instaburst_config.set_id         = 0;
    instaburst_config.tx_power       = RADIO_POWER_NRF_0DBM;
    instaburst_config.callback       = instaburst_tx_complete_callback;
    instaburst_config.interval_ms    = BEARER_ADV_INT_DEFAULT_MS;

    instaburst_tx_instance_init(&m_instaburst[CORE_TX_ROLE_ORIGINATOR],
                             &instaburst_config,
                             m_originator_instaburst_packet_buffer,
                             sizeof(m_originator_instaburst_packet_buffer));
    instaburst_tx_enable(&m_instaburst[CORE_TX_ROLE_ORIGINATOR]);

    /* For consistency, disable the relay callback for instaburst too */
    instaburst_config.callback = NULL;

#if MESH_FEATURE_RELAY_ENABLED
    instaburst_tx_instance_init(&m_instaburst[CORE_TX_ROLE_RELAY],
                             &instaburst_config,
                             m_relay_instaburst_packet_buffer,
                             sizeof(m_relay_instaburst_packet_buffer));
    instaburst_tx_enable(&m_instaburst[CORE_TX_ROLE_RELAY]);
#endif

    core_tx_bearer_add(&m_bearer, &m_interface, CORE_TX_BEARER_TYPE_ADV);
}

bool core_tx_instaburst_is_enabled(core_tx_role_t role)
{
    NRF_MESH_ASSERT_DEBUG(role < CORE_TX_ROLE_COUNT);
    return instaburst_tx_is_enabled(&m_instaburst[role]);
}
