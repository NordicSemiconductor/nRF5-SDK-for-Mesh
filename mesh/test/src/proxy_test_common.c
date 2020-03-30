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
#include <unity.h>
#include <cmock.h>

#include "proxy_test_common.h"
#include "nrf_mesh.h"
#include "proxy.h"
#include "core_tx.h"
#include "event.h"

#include "timer_scheduler_mock.h"
#include "proxy_filter_mock.h"
#include "mesh_gatt_mock.h"

#define UNICAST_ADDR 0x1201

#define SERVICE_UUID 0x1828
#define CHAR_TX_UUID 0x2ade
#define CHAR_RX_UUID 0x2add

#define PACKET_SEAL 0x5EA15EA1

static const nrf_mesh_beacon_info_t * mp_beacon_info;
static uint32_t m_beacon_info_count;

static const nrf_mesh_network_secmat_t * mp_net_secmat;

static mesh_gatt_evt_handler_t m_gatt_evt_handler;
static void * mp_gatt_context;
static struct
{
    uint8_t buffer[100];
    uint16_t size;
    mesh_gatt_pdu_type_t pdu_type;
    nrf_mesh_tx_token_t tx_token;
    uint16_t conn_index;
    uint32_t count;
    bool allocated;
    bool available;
} m_gatt_tx_packet;
static const core_tx_bearer_interface_t * mp_core_tx_if;
static core_tx_bearer_t * mp_core_tx_bearer;
static const nrf_mesh_evt_handler_t * mp_mesh_evt_handler;
/*****************************************************************************
* Mocks
*****************************************************************************/

timestamp_t timer_now(void)
{
    return 0;
}

void nrf_mesh_beacon_info_next_get(const uint8_t * p_network_id,
                                   const nrf_mesh_beacon_info_t ** pp_beacon_info,
                                   nrf_mesh_key_refresh_phase_t * p_kr_phase)
{
    TEST_ASSERT_NOT_NULL(pp_beacon_info);
    TEST_ASSERT_NOT_NULL(p_kr_phase);

    if (mp_beacon_info)
    {
        uint32_t index = ((*pp_beacon_info) ? (*pp_beacon_info - mp_beacon_info) + 1 : 0);
        if (index >= m_beacon_info_count)
        {
            *pp_beacon_info = NULL;
        }
        else
        {
            *pp_beacon_info = &mp_beacon_info[index];
        }
    }
    else
    {
        *pp_beacon_info = NULL;
    }
    *p_kr_phase     = NRF_MESH_KEY_REFRESH_PHASE_0;
}

void nrf_mesh_net_secmat_next_get(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat)
{
    TEST_ASSERT_NOT_NULL(pp_secmat);
    *pp_secmat = mp_net_secmat;
}

void mesh_gatt_init_mock(const mesh_gatt_uuids_t * p_uuids,
                         mesh_gatt_evt_handler_t evt_handler,
                         void * p_context,
                         int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_uuids);
    TEST_ASSERT_EQUAL(SERVICE_UUID, p_uuids->service);
    TEST_ASSERT_EQUAL(CHAR_TX_UUID, p_uuids->tx_char);
    TEST_ASSERT_EQUAL(CHAR_RX_UUID, p_uuids->rx_char);

    m_gatt_evt_handler = evt_handler;
    mp_gatt_context = p_context;
}

uint8_t * mesh_gatt_packet_alloc_mock(uint16_t conn_index,
                                      mesh_gatt_pdu_type_t type,
                                      uint16_t size,
                                      nrf_mesh_tx_token_t token,
                                      int num_calls)
{
    TEST_ASSERT_FALSE(m_gatt_tx_packet.allocated);
    if (m_gatt_tx_packet.available)
    {
        m_gatt_tx_packet.conn_index = conn_index;
        m_gatt_tx_packet.pdu_type   = type;
        m_gatt_tx_packet.size = size;
        uint32_t packet_seal = PACKET_SEAL;
        TEST_ASSERT_TRUE(size + sizeof(packet_seal) < sizeof(m_gatt_tx_packet.buffer));
        memcpy(&m_gatt_tx_packet.buffer[size], &packet_seal, sizeof(packet_seal));
        m_gatt_tx_packet.tx_token = token;
        m_gatt_tx_packet.allocated = true;
        m_gatt_tx_packet.count++;
        return m_gatt_tx_packet.buffer;
    }
    else
    {
        return NULL;
    }
}

uint32_t mesh_gatt_packet_send_mock(uint16_t conn_index, const uint8_t * p_packet, int num_calls)
{
    TEST_ASSERT_EQUAL_PTR(m_gatt_tx_packet.buffer, p_packet);
    TEST_ASSERT_TRUE(m_gatt_tx_packet.allocated);
    TEST_ASSERT_EQUAL(m_gatt_tx_packet.conn_index, conn_index);
    m_gatt_tx_packet.allocated = false;
    uint32_t packet_seal = PACKET_SEAL;
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t *) &packet_seal,
                                 &m_gatt_tx_packet.buffer[m_gatt_tx_packet.size],
                                 sizeof(packet_seal));
    return NRF_SUCCESS;
}

void mesh_gatt_packet_discard_mock(uint16_t conn_index, const uint8_t * p_packet, int num_calls)
{
    TEST_ASSERT_EQUAL_PTR(m_gatt_tx_packet.buffer, p_packet);
    TEST_ASSERT_TRUE(m_gatt_tx_packet.allocated);
    TEST_ASSERT_EQUAL(m_gatt_tx_packet.conn_index, conn_index);
    m_gatt_tx_packet.allocated = false;
}

uint32_t mesh_gatt_disconnect_mock(uint16_t conn_index, int num_calls)
{
    return NRF_SUCCESS;
}

bool nrf_mesh_rx_address_get(uint16_t raw_address, nrf_mesh_address_t * p_address)
{
    if (raw_address == UNICAST_ADDR)
    {
        p_address->value = UNICAST_ADDR;
        p_address->type  = NRF_MESH_ADDRESS_TYPE_UNICAST;
        p_address->p_virtual_uuid = NULL;
        return true;
    }
    return false;
}

void nrf_mesh_unicast_address_get(uint16_t * p_addr_start, uint16_t * p_addr_count)
{
    TEST_ASSERT_NOT_NULL(p_addr_start);
    TEST_ASSERT_NOT_NULL(p_addr_count);
    *p_addr_start = UNICAST_ADDR;
    *p_addr_count = 1;
}

void core_tx_bearer_add(core_tx_bearer_t * p_bearer,
                        const core_tx_bearer_interface_t * p_interface,
                        core_tx_bearer_type_t type)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_interface);
    TEST_ASSERT_EQUAL(CORE_TX_BEARER_TYPE_GATT_SERVER, type);
    p_bearer->p_interface = p_interface;
    p_bearer->type        = type;
    mp_core_tx_if         = p_interface;
    mp_core_tx_bearer     = p_bearer;
}

void core_tx_complete(core_tx_bearer_t * p_bearer,
                      core_tx_role_t role,
                      uint32_t timestamp,
                      nrf_mesh_tx_token_t token)
{

}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    mp_mesh_evt_handler = p_handler_params;
}

uint32_t beacon_tx(uint8_t beacon_type, const void * p_payload, uint8_t payload_len, uint8_t count)
{
    TEST_FAIL_MESSAGE("Unexpectedly attempted to send a beacon");
    return NRF_SUCCESS;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void init(void)
{
    mesh_gatt_init_StubWithCallback(mesh_gatt_init_mock);
    mesh_gatt_packet_alloc_StubWithCallback(mesh_gatt_packet_alloc_mock);
    mesh_gatt_packet_send_StubWithCallback(mesh_gatt_packet_send_mock);
    mesh_gatt_packet_discard_StubWithCallback(mesh_gatt_packet_discard_mock);
    mesh_gatt_disconnect_StubWithCallback(mesh_gatt_disconnect_mock);

    m_gatt_tx_packet.available = true;
    m_gatt_tx_packet.count     = 0;
    mp_beacon_info             = NULL;
    mp_net_secmat              = NULL;
    proxy_init();
}

void establish_connection(uint16_t conn_index)
{
    timer_sch_abort_ExpectAnyArgs();
    proxy_filter_clear_ExpectAnyArgs();
    mesh_gatt_evt_t evt;
    evt.conn_index = conn_index;
    evt.type       = MESH_GATT_EVT_TYPE_CONNECTED;
    m_gatt_evt_handler(&evt, mp_gatt_context);
}

void disconnect(uint16_t conn_index)
{
    mesh_gatt_evt_t evt;
    evt.conn_index = conn_index;
    evt.type       = MESH_GATT_EVT_TYPE_DISCONNECTED;
    m_gatt_evt_handler(&evt, mp_gatt_context);
}

void beacon_info_set(const nrf_mesh_beacon_info_t * p_beacon_info, uint32_t count)
{
    mp_beacon_info = p_beacon_info;
    m_beacon_info_count = count;
}

void net_secmat_set(const nrf_mesh_network_secmat_t * p_secmat)
{
    mp_net_secmat = p_secmat;
}

void gatt_evt_post(const mesh_gatt_evt_t * p_evt)
{
    m_gatt_evt_handler(p_evt, mp_gatt_context);
}

void mesh_evt_post(const nrf_mesh_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(mp_mesh_evt_handler);
    mp_mesh_evt_handler->evt_cb(p_evt);
}


uint8_t * gatt_tx_packet_get(uint16_t * p_size)
{
    if (m_gatt_tx_packet.allocated)
    {
        return NULL;
    }
    if (p_size)
    {
        *p_size = m_gatt_tx_packet.size;
    }
    return m_gatt_tx_packet.buffer;
}

bool gatt_tx_packet_is_allocated(void)
{
    return m_gatt_tx_packet.allocated;
}

void gatt_tx_packet_availability_set(bool available)
{
    m_gatt_tx_packet.available = available;
}

uint32_t gatt_tx_packet_alloc_count(void)
{
    return m_gatt_tx_packet.count;
}

const core_tx_bearer_interface_t * core_tx_if_get(void)
{
    return mp_core_tx_if;
}

core_tx_bearer_t * core_tx_bearer_get(void)
{
    return mp_core_tx_bearer;
}
