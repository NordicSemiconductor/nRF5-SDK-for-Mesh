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
#include "proxy.h"

#include "nrf_mesh_config_core.h"
#if MESH_FEATURE_GATT_PROXY_ENABLED

#include "proxy_config_packet.h"
#include "proxy_filter.h"
#include "core_tx.h"
#include "mesh_gatt.h"
#include "network.h"
#include "beacon.h"
#include "net_beacon.h"
#include "net_state.h"
#include "net_packet.h"
#include "nordic_common.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_utils.h"
#include "log.h"
#include "rand.h"
#include "enc.h"
#include "mesh_adv.h"
#include "timer_scheduler.h"
#include "cache.h"
#include "mesh_config_entry.h"
#include "mesh_opt_gatt.h"
#include "event.h"
#include "bearer_event.h"

#if MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS > 60000
#error The MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS shall not be greater than 60,000 ms
#endif

#define PROXY_UUID_SERVICE 0x1828
#define PROXY_UUID_CHAR_TX 0x2ade
#define PROXY_UUID_CHAR_RX 0x2add

#define PROXY_ADV_DATA_NODE_ID_HASH_SIZE 8
#define PROXY_ADV_DATA_NODE_ID_RAND_SIZE 8

#define ADV_TIMEOUT_INFINITE (0)

#define ADV_NETWORK_ITERATE_INTERVAL_US     SEC_TO_US(10)

/** Macro for copying a packed big-endian address list into a regular uint16_t array. */
#define ADDRS_LIST_ENDIANESS_SWAP_AND_COPY(P_ADDRS, P_ADDRS_PACKED, ADDR_COUNT) \
    do {                                                                \
        for (uint32_t i = 0; i < addr_count; ++i)                       \
        {                                                               \
            P_ADDRS[i] = LE2BE16(P_ADDRS_PACKED[i]);                    \
        }                                                               \
    } while (0)

NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(MESH_GATT_PROXY_BEACON_CACHE_SIZE));

typedef uint8_t beacon_cache_entry_t[NET_BEACON_CMAC_SIZE];

typedef enum
{
    PROXY_ADV_TYPE_NETWORK_ID,
    PROXY_ADV_TYPE_NODE_ID,
} proxy_adv_type_t;

typedef struct
{
    proxy_filter_t filter;
    bool connected;
    nrf_mesh_key_refresh_phase_t kr_phase;
    const nrf_mesh_beacon_info_t * p_pending_beacon_info;
    uint8_t * p_alloc_packet;
    core_tx_bearer_t bearer;
} proxy_connection_t;


typedef struct __attribute((packed))
{
    uint8_t type; /**< See @ref proxy_adv_type_t for legal values. */
    union __attribute((packed))
    {
        struct __attribute((packed))
        {
            uint8_t network_id[NRF_MESH_NETID_SIZE];
        } network_id;
        struct __attribute((packed))
        {
            uint8_t hash[PROXY_ADV_DATA_NODE_ID_HASH_SIZE];
            uint8_t random[PROXY_ADV_DATA_NODE_ID_RAND_SIZE];
        } node_id;
    } params;
} proxy_adv_service_data_t;

typedef struct __attribute((packed))
{
    uint8_t padding[6];
    uint8_t random[PROXY_ADV_DATA_NODE_ID_RAND_SIZE];
    uint16_t address;
} proxy_adv_node_id_hash_input_t;

static core_tx_alloc_result_t core_tx_packet_alloc_cb(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params);
static void core_tx_packet_send_cb(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length);
static void core_tx_packet_discard_cb(core_tx_bearer_t * p_bearer);

static void adv_start(proxy_adv_type_t proxy_adv_type, bool interleave_networks);
/*****************************************************************************
* Static globals
*****************************************************************************/
static const core_tx_bearer_interface_t m_interface = {core_tx_packet_alloc_cb,
                                                       core_tx_packet_send_cb,
                                                       core_tx_packet_discard_cb};
static proxy_connection_t m_connections[MESH_GATT_CONNECTION_COUNT_MAX];
static nrf_mesh_evt_handler_t m_mesh_evt_handler;
static bool m_enabled = PROXY_ENABLED_DEFAULT;
static bool m_initialized;
static bool m_stop_requested;
static bearer_event_flag_t m_key_refresh_flag = BEARER_EVENT_FLAG_INVALID;
static bearer_event_flag_t m_iv_update_flag = BEARER_EVENT_FLAG_INVALID;
static const uint8_t * mp_key_refresh_network_id = NULL;

static struct
{
    bool running;
    proxy_adv_type_t type;
    timer_event_t timer;
    const nrf_mesh_beacon_info_t * p_net_beacon_info;
    nrf_mesh_key_refresh_phase_t kr_phase;
} m_advertising;

static beacon_cache_entry_t m_beacon_cache_entries[MESH_GATT_PROXY_BEACON_CACHE_SIZE];
static cache_t m_beacon_cache;

/*****************************************************************************
* Static functions
*****************************************************************************/

static inline uint16_t connection_index(const proxy_connection_t * p_connection)
{
    return (p_connection - &m_connections[0]);
}

static void connection_reset(proxy_connection_t * p_connection)
{
    proxy_filter_clear(&p_connection->filter);
    p_connection->p_pending_beacon_info = NULL;
}

static inline uint32_t active_connection_count(void)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_connections[i].connected)
        {
            count++;
        }
    }
    return count;
}

static bool has_pending_packets(void)
{
    for (size_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; i++)
    {
        if (m_connections[i].connected
            && mesh_gatt_packet_is_pending(connection_index(&m_connections[i])))
        {
            return true;
        }
    }

    return false;
}

static void adv_data_network_id_set(const uint8_t * p_network_id)
{
    proxy_adv_service_data_t service_data;
    service_data.type = PROXY_ADV_TYPE_NETWORK_ID;
    memcpy(&service_data.params.network_id.network_id[0], p_network_id, NRF_MESH_NETID_SIZE);

    uint32_t size = (offsetof(proxy_adv_service_data_t, params) + NRF_MESH_NETID_SIZE);

    mesh_adv_data_set(PROXY_UUID_SERVICE, (uint8_t *) &service_data, size);
}

static void adv_data_node_id_set(const uint8_t * p_identity_key, uint16_t src)
{
    /* Hash = AES(Padding | Random | Address) */
    proxy_adv_node_id_hash_input_t hash_enc_input;
    memset(hash_enc_input.padding, 0, sizeof(hash_enc_input.padding));
    rand_hw_rng_get(hash_enc_input.random, sizeof(hash_enc_input.random));
    hash_enc_input.address = LE2BE16(src);

    uint8_t hash_enc_output[NRF_MESH_KEY_SIZE];
    enc_aes_encrypt(p_identity_key, (uint8_t *) &hash_enc_input, hash_enc_output);

    proxy_adv_service_data_t service_data;
    service_data.type = PROXY_ADV_TYPE_NODE_ID;
    /* Need last 8 bytes, as the spec considers the hash a big endian number */
    memcpy(service_data.params.node_id.hash,
           &hash_enc_output[NRF_MESH_KEY_SIZE - PROXY_ADV_DATA_NODE_ID_HASH_SIZE],
           PROXY_ADV_DATA_NODE_ID_HASH_SIZE);
    memcpy(service_data.params.node_id.random, hash_enc_input.random, PROXY_ADV_DATA_NODE_ID_RAND_SIZE);

    uint32_t size = (((uint32_t) &service_data.params - (uint32_t) &service_data) +
                     sizeof(service_data.params.node_id));

    mesh_adv_data_set(PROXY_UUID_SERVICE, (uint8_t *) &service_data, size);
}

/**
 * Clean up the advertiser state.
 */
static void on_adv_end(void)
{
    timer_sch_abort(&m_advertising.timer);
    m_advertising.running = false;
}

static uint32_t packet_alloc(proxy_connection_t * p_connection,
                             mesh_gatt_pdu_type_t pdu_type,
                             uint32_t length,
                             nrf_mesh_tx_token_t token)
{
    NRF_MESH_ASSERT(p_connection->p_alloc_packet == NULL);
    p_connection->p_alloc_packet = mesh_gatt_packet_alloc(connection_index(p_connection),
                                                          pdu_type,
                                                          length,
                                                          token);
    return (p_connection->p_alloc_packet ? NRF_SUCCESS : NRF_ERROR_NO_MEM);
}

static void packet_send(proxy_connection_t * p_connection)
{
    NRF_MESH_ASSERT(p_connection->p_alloc_packet);

    if(mesh_gatt_packet_send(connection_index(p_connection), p_connection->p_alloc_packet) != NRF_SUCCESS)
    {
        mesh_gatt_packet_discard(connection_index(p_connection), p_connection->p_alloc_packet);
    }
    p_connection->p_alloc_packet = NULL;
}

static void packet_discard(proxy_connection_t * p_connection)
{
    NRF_MESH_ASSERT(p_connection->p_alloc_packet);
    mesh_gatt_packet_discard(connection_index(p_connection), p_connection->p_alloc_packet);
    p_connection->p_alloc_packet = NULL;
}

static uint32_t beacon_packet_send(proxy_connection_t * p_connection,
                                   const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                                   uint32_t iv_index,
                                   net_state_iv_update_t iv_update,
                                   bool key_refresh)
{
    if (packet_alloc(p_connection, MESH_GATT_PDU_TYPE_MESH_BEACON, NET_BEACON_BUFFER_SIZE, NRF_MESH_INITIAL_TOKEN) ==
        NRF_SUCCESS)
    {
        NRF_MESH_ERROR_CHECK(net_beacon_build(p_beacon_secmat,
                                              iv_index,
                                              iv_update,
                                              key_refresh,
                                              p_connection->p_alloc_packet));
        packet_send(p_connection);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

static void beacon_cycle_trigger(proxy_connection_t * p_connection)
{
    p_connection->p_pending_beacon_info = NULL;
    nrf_mesh_beacon_info_next_get(NULL, &p_connection->p_pending_beacon_info, &p_connection->kr_phase);
}

static void beacon_cycle_send(proxy_connection_t * p_connection)
{
    while (p_connection->p_pending_beacon_info &&
           (beacon_packet_send(p_connection,
                               nrf_mesh_beacon_secmat_from_info(p_connection->p_pending_beacon_info, p_connection->kr_phase),
                               net_state_beacon_iv_index_get(),
                               net_state_iv_update_get(),
                               p_connection->kr_phase) == NRF_SUCCESS))
    {
        nrf_mesh_beacon_info_next_get(NULL,
                                      &p_connection->p_pending_beacon_info,
                                      &p_connection->kr_phase);
    }
}

static uint32_t beacon_packet_send_to_all(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                                          uint32_t iv_index,
                                          net_state_iv_update_t iv_update,
                                          bool key_refresh)
{
    uint32_t status = NRF_SUCCESS;
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_connections[i].connected)
        {
            if (beacon_packet_send(&m_connections[i],
                                   p_beacon_secmat,
                                   iv_index,
                                   iv_update,
                                   key_refresh) != NRF_SUCCESS)
            {
                status = NRF_ERROR_NO_MEM;
            }
        }
    }
    return status;
}

static bool send_beacon_on_iv_update(void)
{
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_connections[i].connected)
        {
            beacon_cycle_trigger(&m_connections[i]);
            beacon_cycle_send(&m_connections[i]);
        }
    }

    return true;
}

static bool send_beacon_on_key_refresh(void)
{
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    nrf_mesh_key_refresh_phase_t kr_phase;

    nrf_mesh_beacon_info_next_get(mp_key_refresh_network_id, &p_beacon_info, &kr_phase);

    NRF_MESH_ASSERT(p_beacon_info);
    mp_key_refresh_network_id = NULL;

    (void) beacon_packet_send_to_all(nrf_mesh_beacon_secmat_from_info(p_beacon_info, kr_phase),
                                     net_state_beacon_iv_index_get(),
                                     net_state_iv_update_get(),
                                     net_beacon_key_refresh_flag(kr_phase));
    return true;
}

static bool config_packet_decrypt(network_packet_metadata_t * p_net_meta,
                                  const uint8_t * p_data,
                                  uint32_t length,
                                  uint8_t * p_decrypted)
{
    uint32_t status = net_packet_decrypt(p_net_meta,
                                         length,
                                         (const packet_mesh_net_packet_t *) p_data,
                                         (packet_mesh_net_packet_t *) p_decrypted,
                                         NET_PACKET_KIND_PROXY_CONFIG);

    if (status == NRF_SUCCESS)
    {
        /* Validate metadata */
        return (p_net_meta->control_packet && p_net_meta->ttl == 0 &&
                p_net_meta->dst.value == NRF_MESH_ADDRESS_TYPE_INVALID);
    }
    else
    {
        return false;
    }
}

static uint32_t config_packet_buffer_len(uint32_t msg_len)
{
    return PACKET_MESH_NET_PDU_OFFSET + msg_len + net_packet_mic_size_get(true);
}

static proxy_config_msg_t * config_packet_alloc(proxy_connection_t * p_connection, uint32_t msg_len)
{
    if (packet_alloc(p_connection,
                     MESH_GATT_PDU_TYPE_PROXY_CONFIG,
                     config_packet_buffer_len(msg_len),
                     NRF_MESH_INITIAL_TOKEN) == NRF_SUCCESS)
    {
        return (proxy_config_msg_t *) net_packet_payload_get(
            (packet_mesh_net_packet_t *) p_connection->p_alloc_packet);
    }

    return NULL;
}

static uint32_t config_packet_send(proxy_connection_t * p_connection,
                                   const nrf_mesh_network_secmat_t * p_secmat,
                                   uint32_t msg_len)
{
    uint16_t src_addr_count;
    network_packet_metadata_t tx_net_meta = {
        .dst = {
            .type = NRF_MESH_ADDRESS_TYPE_INVALID,
            .value = 0
        },
        .ttl = 0,
        .control_packet = true,
        .p_security_material = p_secmat
    };

    nrf_mesh_unicast_address_get(&tx_net_meta.src, &src_addr_count);

    if (net_state_iv_index_and_seqnum_alloc(&tx_net_meta.internal.iv_index,
                                            &tx_net_meta.internal.sequence_number) != NRF_SUCCESS)
    {
        packet_discard(p_connection);
        return NRF_ERROR_BUSY;
    }

    net_packet_header_set((packet_mesh_net_packet_t *) p_connection->p_alloc_packet, &tx_net_meta);

    net_packet_encrypt(&tx_net_meta,
                       msg_len,
                       (packet_mesh_net_packet_t *) p_connection->p_alloc_packet,
                       NET_PACKET_KIND_PROXY_CONFIG);

    packet_send(p_connection);
    return NRF_SUCCESS;
}

static uint32_t filter_status_send(proxy_connection_t * p_connection,
                                   const nrf_mesh_network_secmat_t * p_secmat)
{
    uint32_t filter_status_len =
        PROXY_CONFIG_PARAM_OVERHEAD + sizeof(proxy_config_params_filter_status_t);

    proxy_config_msg_t * p_status = config_packet_alloc(p_connection, filter_status_len);
    if (p_status)
    {
        p_status->opcode                           = PROXY_CONFIG_OPCODE_FILTER_STATUS;
        p_status->params.filter_status.filter_type = p_connection->filter.type;
        p_status->params.filter_status.list_size   = LE2BE16(p_connection->filter.count);

        return config_packet_send(p_connection, p_secmat, filter_status_len);
    }
    return NRF_ERROR_NO_MEM;
}

static uint32_t config_packet_in_filter_type_set(proxy_connection_t * p_connection,
                                                 const proxy_config_params_filter_type_set_t * p_params,
                                                 uint32_t params_len)
{
    if (params_len == sizeof(proxy_config_params_filter_type_set_t))
    {
        return proxy_filter_type_set(&p_connection->filter,
                                     (proxy_filter_type_t) p_params->filter_type);
    }
    else
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
}

static uint32_t config_packet_in_filter_addr_add(proxy_connection_t * p_connection,
                                                 proxy_config_params_filter_addr_add_t * p_params,
                                                 uint32_t params_len)
{
    if (params_len <= sizeof(proxy_config_params_filter_addr_add_t) && !(params_len & 0x01))
    {
        /* Move addresses to array on stack. The array is defined with max size to avoid ARMCC
         * secretly adding it to heap:
         * http://infocenter.arm.com/help/topic/com.arm.doc.dui0472m/chr1359124223721.html
         */
        uint16_t addrs[MESH_GATT_PROXY_FILTER_ADDR_COUNT];
        uint32_t addr_count = params_len / sizeof(uint16_t);
        ADDRS_LIST_ENDIANESS_SWAP_AND_COPY(addrs, p_params->addrs, addr_count);
        proxy_filter_add(&p_connection->filter, addrs, addr_count);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
}

static uint32_t config_packet_in_filter_addr_remove(proxy_connection_t * p_connection,
                                                 proxy_config_params_filter_addr_remove_t * p_params,
                                                 uint32_t params_len)
{
    if (params_len <= sizeof(proxy_config_params_filter_addr_remove_t) && !(params_len & 0x01))
    {
        /* Move addresses to array on stack. The array is defined with max size to avoid ARMCC
         * secretly adding it to heap:
         * http://infocenter.arm.com/help/topic/com.arm.doc.dui0472m/chr1359124223721.html
         */
        uint16_t addrs[MESH_GATT_PROXY_FILTER_ADDR_COUNT];
        uint32_t addr_count = params_len / sizeof(uint16_t);
        ADDRS_LIST_ENDIANESS_SWAP_AND_COPY(addrs, p_params->addrs, addr_count);
        proxy_filter_remove(&p_connection->filter, addrs, addr_count);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
}

static void config_packet_in(proxy_connection_t * p_connection,
                                 const uint8_t * p_data,
                                 uint32_t length)
{
    if (length <= MESH_GATT_PROXY_PDU_MAX_SIZE && length >= PROXY_CONFIG_PARAM_OVERHEAD)
    {
        network_packet_metadata_t net_meta;
        uint8_t decrypted_packet[MESH_GATT_PROXY_PDU_MAX_SIZE];

        if (config_packet_decrypt(&net_meta, p_data, length, decrypted_packet))
        {
            proxy_config_msg_t * p_msg = (proxy_config_msg_t *) net_packet_payload_get(
                (packet_mesh_net_packet_t *) decrypted_packet);

            uint32_t params_len =
                (net_packet_payload_len_get(&net_meta, length) - PROXY_CONFIG_PARAM_OVERHEAD);

            uint32_t status;
            switch (p_msg->opcode)
            {
                case PROXY_CONFIG_OPCODE_FILTER_TYPE_SET:
                    status = config_packet_in_filter_type_set(p_connection,
                                                              &p_msg->params.filter_type_set,
                                                              params_len);
                    break;
                case PROXY_CONFIG_OPCODE_FILTER_ADDR_ADD:
                    status = config_packet_in_filter_addr_add(p_connection,
                                                              &p_msg->params.filter_addr_add,
                                                              params_len);
                    break;
                case PROXY_CONFIG_OPCODE_FILTER_ADDR_REMOVE:
                    status = config_packet_in_filter_addr_remove(p_connection,
                                                                 &p_msg->params.filter_addr_remove,
                                                                 params_len);
                    break;
                default:
                    status = NRF_ERROR_NOT_FOUND;
            }

            if (status == NRF_SUCCESS)
            {
                (void) filter_status_send(p_connection,
                                          net_meta.p_security_material);
            }
        }
        else
        {
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "Decryption failed\n");
        }
    }
    else
    {
        __LOG(LOG_SRC_BEARER, LOG_LEVEL_WARN, "Config packet too long (%u bytes), dropped it\n", length);
    }
}

static void rx_handle(proxy_connection_t * p_connection,
                      mesh_gatt_pdu_type_t pdu_type,
                      const uint8_t * p_data,
                      uint32_t length)
{
    const nrf_mesh_rx_metadata_t rx_metadata = {
        .source = NRF_MESH_RX_SOURCE_GATT,
        .params.gatt = {
            .timestamp = timer_now(), /*lint !e446 side effect in initializer */
            .connection_index = connection_index(p_connection) /*lint !e446 side effect in initializer */
        }
    };

    __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "RX GATT PDU type 0x%x, len %u\n", pdu_type, length);

    switch (pdu_type)
    {
        case MESH_GATT_PDU_TYPE_NETWORK_PDU:
            (void) network_packet_in(p_data, length, &rx_metadata);
            break;
        case MESH_GATT_PDU_TYPE_MESH_BEACON:
            net_beacon_packet_in(((beacon_packet_t *)p_data)->payload, length - BEACON_PACKET_OVERHEAD, &rx_metadata);
            break;
        case MESH_GATT_PDU_TYPE_PROXY_CONFIG:
            config_packet_in(p_connection, p_data, length);
            break;
        default:
            /* Ignore unknown PDU type, according to @tagMeshSp section 6.6 */
            break;
    }
}

static void gatt_evt_handler(const mesh_gatt_evt_t * p_evt, void * p_context)
{
    proxy_connection_t * p_connection = &m_connections[p_evt->conn_index];

    switch (p_evt->type)
    {
        case MESH_GATT_EVT_TYPE_CONNECTED:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "Connected\n");
            on_adv_end();
            p_connection->connected = true;
            connection_reset(p_connection);

            /* Send beacons for all known networks upon connection */
            beacon_cycle_trigger(p_connection);

            /* Advertising must be restarted manually after connecting: */
            if (active_connection_count() < MESH_GATT_CONNECTION_COUNT_MAX)
            {
                adv_start(PROXY_ADV_TYPE_NETWORK_ID, true);
            }
            break;

        case MESH_GATT_EVT_TYPE_DISCONNECTED:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "Disconnected\n");

            p_connection->connected = false;
            if (m_enabled && !m_advertising.running)
            {
                m_advertising.p_net_beacon_info = NULL;
                adv_start(PROXY_ADV_TYPE_NETWORK_ID, true);
            }

            if (m_stop_requested && !active_connection_count())
            {
                m_stop_requested = false;

                nrf_mesh_evt_t evt =
                {
                    .type = NRF_MESH_EVT_PROXY_STOPPED,
                };
                event_handle(&evt);
            }
            break;

        case MESH_GATT_EVT_TYPE_TX_READY:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "TX ready\n");
            beacon_cycle_send(p_connection);
            break;

        case MESH_GATT_EVT_TYPE_RX:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "RX\n");
            rx_handle(p_connection,
                      p_evt->params.rx.pdu_type,
                      p_evt->params.rx.p_data,
                      p_evt->params.rx.length);
            break;

        case MESH_GATT_EVT_TYPE_TX_COMPLETE:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "TX complete\n");

            if (p_evt->params.tx_complete.pdu_type == MESH_GATT_PDU_TYPE_NETWORK_PDU)
            {
#if MESH_FEATURE_RELAY_ENABLED
                core_tx_role_t role = (p_evt->params.tx_complete.token == NRF_MESH_RELAY_TOKEN)
                                          ? CORE_TX_ROLE_RELAY
                                          : CORE_TX_ROLE_ORIGINATOR;
#else
                core_tx_role_t role = CORE_TX_ROLE_ORIGINATOR;
#endif

                core_tx_complete(&p_connection->bearer,
                                 role,
                                 timer_now(),
                                 p_evt->params.tx_complete.token);
            }

            if (!m_stop_requested)
            {
                /* Send pending beacons if requested. */
                beacon_cycle_send(p_connection);
            }
            else if (m_stop_requested && !has_pending_packets())
            {
                proxy_disconnect();
            }
            break;

        case MESH_GATT_EVT_TYPE_ADV_TIMEOUT:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "Adv timeout\n");
            /* The node identity state timed out, go back to network ID advertisements */
            m_advertising.running = false;
            if (m_enabled)
            {
                adv_start(PROXY_ADV_TYPE_NETWORK_ID, true);
            }
            break;

        default:
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_WARN, "Got an unknown GATT evt 0x%x\n", p_evt->type);
    }
}

static void mesh_evt_handle(const nrf_mesh_evt_t * p_evt)
{
    /* According to @tagMeshSp section 6.6, we should forward all
     * beacons with new values for IV index or flags. We'll maintain a small cache of previous beacons
     * to enforce this by best effort.
     */
    if (p_evt->type == NRF_MESH_EVT_NET_BEACON_RECEIVED)
    {
        if (p_evt->params.net_beacon.p_rx_metadata->source != NRF_MESH_RX_SOURCE_GATT &&
            !cache_has_elem(&m_beacon_cache, p_evt->params.net_beacon.p_auth_value))
        {
            if (beacon_packet_send_to_all(p_evt->params.net_beacon.p_beacon_secmat,
                                            p_evt->params.net_beacon.iv_index,
                                            p_evt->params.net_beacon.flags.iv_update,
                                            p_evt->params.net_beacon.flags.key_refresh) == NRF_SUCCESS)
            {
                cache_put(&m_beacon_cache, p_evt->params.net_beacon.p_auth_value);
            }
        }
    }
    else if (p_evt->type == NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION)
    {
        NRF_MESH_ASSERT(p_evt->params.key_refresh.p_network_id);
        mp_key_refresh_network_id = p_evt->params.key_refresh.p_network_id;
        bearer_event_flag_set(m_key_refresh_flag);
    }
    else if (p_evt->type == NRF_MESH_EVT_IV_UPDATE_NOTIFICATION)
    {
        bearer_event_flag_set(m_iv_update_flag);
    }
}

static void adv_net_iterate(void)
{
    nrf_mesh_key_refresh_phase_t kr_phase;
    nrf_mesh_beacon_info_next_get(NULL, &m_advertising.p_net_beacon_info, &kr_phase);
    if (m_advertising.p_net_beacon_info == NULL)
    {
        /* Start from the beginning when we reach the end */
        nrf_mesh_beacon_info_next_get(NULL, &m_advertising.p_net_beacon_info, &kr_phase);
        NRF_MESH_ASSERT(m_advertising.p_net_beacon_info);
    }
}

static void adv_data_set(void)
{
    const nrf_mesh_beacon_secmat_t * p_secmat =
        nrf_mesh_beacon_secmat_from_info(m_advertising.p_net_beacon_info, m_advertising.kr_phase);
    if (m_advertising.type == PROXY_ADV_TYPE_NETWORK_ID)
    {
        adv_data_network_id_set(p_secmat->net_id);
    }
    else if (m_advertising.type == PROXY_ADV_TYPE_NODE_ID)
    {
        uint16_t src_addr;
        uint16_t addr_count;
        nrf_mesh_unicast_address_get(&src_addr, &addr_count);

        adv_data_node_id_set(p_secmat->identity_key, src_addr);
    }
}

static void adv_timer_handler(timestamp_t timeout, void * p_context)
{
    adv_net_iterate();
    adv_data_set();
}

static void adv_start(proxy_adv_type_t proxy_adv_type, bool interleave_networks)
{
    m_advertising.type = proxy_adv_type;

    if (m_advertising.running)
    {
        mesh_adv_stop();
    }

    if (interleave_networks)
    {
        adv_net_iterate();
        timer_sch_reschedule(&m_advertising.timer, timer_now() + m_advertising.timer.interval);
    }
    else
    {
        timer_sch_abort(&m_advertising.timer);
    }

    if (proxy_adv_type == PROXY_ADV_TYPE_NETWORK_ID)
    {
        mesh_adv_params_set(MESH_ADV_TIMEOUT_INFINITE,
                            MSEC_TO_UNITS(MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS,
                                          UNIT_0_625_MS));
    }
    else
    {
        mesh_adv_params_set(MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS,
                            MSEC_TO_UNITS(MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS,
                                          UNIT_0_625_MS));
    }

    adv_data_set();
    mesh_adv_start();
    m_advertising.running = true;
}

/*****************************************************************************
* Core TX callback functions
*****************************************************************************/
core_tx_alloc_result_t core_tx_packet_alloc_cb(core_tx_bearer_t * p_bearer,
                                               const core_tx_alloc_params_t * p_params)
{
    NRF_MESH_ASSERT(p_bearer && p_params);
    proxy_connection_t * p_connection = PARENT_BY_FIELD_GET(proxy_connection_t, bearer, p_bearer);

    if (p_connection->connected && proxy_filter_accept(&p_connection->filter, p_params->p_metadata->dst.value))
    {
        uint32_t status = packet_alloc(p_connection, MESH_GATT_PDU_TYPE_NETWORK_PDU, p_params->net_packet_len, p_params->token);

        return ((status == NRF_SUCCESS) ? CORE_TX_ALLOC_SUCCESS : CORE_TX_ALLOC_FAIL_NO_MEM);
    }
    else
    {
        return CORE_TX_ALLOC_FAIL_REJECTED;
    }
}

void core_tx_packet_send_cb(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    NRF_MESH_ASSERT(p_bearer && p_packet);
    proxy_connection_t * p_connection = PARENT_BY_FIELD_GET(proxy_connection_t, bearer, p_bearer);
    NRF_MESH_ASSERT(p_connection->p_alloc_packet);

    memcpy(p_connection->p_alloc_packet, p_packet, packet_length);
    packet_send(p_connection);
}

void core_tx_packet_discard_cb(core_tx_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer);
    proxy_connection_t * p_connection = PARENT_BY_FIELD_GET(proxy_connection_t, bearer, p_bearer);
    packet_discard(p_connection);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void proxy_init(void)
{
    if (m_initialized)
    {
        return;
    }

    mesh_gatt_uuids_t uuids = {.service = PROXY_UUID_SERVICE,
                               .tx_char = PROXY_UUID_CHAR_TX,
                               .rx_char = PROXY_UUID_CHAR_RX};
    mesh_gatt_init(&uuids, gatt_evt_handler, NULL);
    m_mesh_evt_handler.evt_cb = mesh_evt_handle;
    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

    m_beacon_cache.array_len = MESH_GATT_PROXY_BEACON_CACHE_SIZE;
    m_beacon_cache.elem_array = m_beacon_cache_entries;
    m_beacon_cache.elem_size = sizeof(sizeof(beacon_cache_entry_t));
    cache_init(&m_beacon_cache);

    m_advertising.timer.cb = adv_timer_handler;
    m_advertising.timer.interval = ADV_NETWORK_ITERATE_INTERVAL_US;
    m_advertising.running        = false;
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        core_tx_bearer_add(&m_connections[i].bearer, &m_interface, CORE_TX_BEARER_TYPE_GATT_SERVER);
        m_connections[i].connected = false;
    }

    m_iv_update_flag = bearer_event_flag_add(send_beacon_on_iv_update);
    m_key_refresh_flag = bearer_event_flag_add(send_beacon_on_key_refresh);

    m_initialized = true;

    /* NOTE: We're not setting the m_enabled state here. It is stored in zero-initialized memory and
     * set by mesh_config_load() which _has to be called before this function_.
     *
     * The reason is that this function shall only be called if the device has been provisioned,
     * otherwise it will add the Proxy Service to the GATT database (through mesh_gatt_init(...)).
     */
}

uint32_t proxy_start(void)
{
    if (m_initialized && m_enabled)
    {
        /* Can't start connectable advertisements if run out of connections. */
        if (!m_advertising.running
            && active_connection_count() < MESH_GATT_CONNECTION_COUNT_MAX)
        {
            m_advertising.p_net_beacon_info = NULL;
            adv_start(PROXY_ADV_TYPE_NETWORK_ID, true);
        }
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}

uint32_t proxy_stop(void)
{
    if (m_initialized && m_enabled)
    {
        if (m_advertising.running)
        {
            mesh_adv_stop();
            on_adv_end();
        }

        m_enabled = false;

        if (!active_connection_count())
        {
            nrf_mesh_evt_t evt =
            {
                .type = NRF_MESH_EVT_PROXY_STOPPED,
            };
            event_handle(&evt);
        }
        else
        {
            m_stop_requested = true;

            if (!has_pending_packets())
            {
                proxy_disconnect();
            }
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}

void proxy_subnet_added(uint16_t net_key_index, const uint8_t * p_network_id)
{
    NRF_MESH_ASSERT(p_network_id);
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    nrf_mesh_key_refresh_phase_t kr_phase;
    nrf_mesh_beacon_info_next_get(p_network_id, &p_beacon_info, &kr_phase);
    NRF_MESH_ASSERT(p_beacon_info);

    (void) beacon_packet_send_to_all(nrf_mesh_beacon_secmat_from_info(p_beacon_info, kr_phase),
                                     net_state_beacon_iv_index_get(),
                                     net_state_iv_update_get(),
                                     net_beacon_key_refresh_flag(kr_phase));
}

void proxy_net_packet_processed(const network_packet_metadata_t * p_net_metadata, const nrf_mesh_rx_metadata_t * p_rx_meta)
{
    if (p_rx_meta->source == NRF_MESH_RX_SOURCE_GATT)
    {
        proxy_connection_t * p_connection = &m_connections[p_rx_meta->params.gatt.connection_index];

        if (p_connection->connected)
        {
            if (p_connection->filter.type == PROXY_FILTER_TYPE_WHITELIST)
            {
                proxy_filter_add(&p_connection->filter, &p_net_metadata->src, 1);
            }
            else
            {
                proxy_filter_remove(&p_connection->filter, &p_net_metadata->src, 1);
            }
        }
    }
}

bool proxy_node_id_is_enabled(const nrf_mesh_beacon_info_t * p_beacon_info)
{
    return (m_enabled &&
            m_advertising.running &&
            m_advertising.type == PROXY_ADV_TYPE_NODE_ID &&
            (p_beacon_info == NULL || p_beacon_info == m_advertising.p_net_beacon_info));
}

bool proxy_is_enabled(void)
{
    return m_enabled;
}

bool proxy_is_connected(void)
{
    return (m_enabled && active_connection_count() > 0);
}

void proxy_disconnect(void)
{
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_connections[i].connected)
        {
            (void) mesh_gatt_disconnect(i);
            m_connections[i].connected = false;
        }
    }
}

uint32_t proxy_enable(void)
{
    return mesh_opt_gatt_proxy_set(true);
}

void proxy_disable(void)
{
    (void) mesh_opt_gatt_proxy_set(false);
}

uint32_t proxy_node_id_enable(const nrf_mesh_beacon_info_t * p_beacon_info, nrf_mesh_key_refresh_phase_t kr_phase)
{
    if (!m_initialized)
    {
    	return NRF_ERROR_INVALID_STATE;
    }
    if (active_connection_count() >= MESH_GATT_CONNECTION_COUNT_MAX)
    {
        return NRF_ERROR_BUSY;
    }
    else
    {
        /* If the Node ID is already enabled, we'll restart the counter now. */
        m_advertising.p_net_beacon_info = p_beacon_info;
        m_advertising.kr_phase          = kr_phase;
        adv_start(PROXY_ADV_TYPE_NODE_ID, (p_beacon_info == NULL));
        return NRF_SUCCESS;
    }
}

uint32_t proxy_node_id_disable(void)
{
    if (!m_initialized || !proxy_node_id_is_enabled(NULL))
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        /* If the Proxy state is enabled, we'll go back to advertising
         * the Network ID. */
        if (m_enabled)
        {
            adv_start(PROXY_ADV_TYPE_NETWORK_ID, true);
        }
        else
        {
            mesh_adv_stop();
            on_adv_end();
        }
        return NRF_SUCCESS;
    }
}

/*****************************************************************************
 * Mesh config wrappers
 *****************************************************************************/

static uint32_t proxy_set(mesh_config_entry_id_t id, const void * p_entry)
{
    const bool enable = *((bool *) p_entry);

    if (enable != m_enabled)
    {
        m_enabled = enable;

        if (!m_enabled && m_advertising.running)
        {
            mesh_adv_stop();
            on_adv_end();
        }
    }

    return NRF_SUCCESS;
}

static void proxy_get(mesh_config_entry_id_t id, void * p_entry)
{
    bool * p_enabled = p_entry;
    *p_enabled = proxy_is_enabled();
}

static void proxy_delete(mesh_config_entry_id_t id)
{
    m_enabled = PROXY_ENABLED_DEFAULT;

    if (m_advertising.running && m_enabled == false)
    {
        mesh_adv_stop();
        on_adv_end();
    }
}

MESH_CONFIG_ENTRY(mesh_opt_gatt_proxy,
                  MESH_OPT_GATT_PROXY_EID,
                  1,
                  sizeof(bool),
                  proxy_set,
                  proxy_get,
                  proxy_delete,
                  true);

uint32_t mesh_opt_gatt_proxy_set(bool enabled)
{
    return mesh_config_entry_set(MESH_OPT_GATT_PROXY_EID, &enabled);
}

uint32_t mesh_opt_gatt_proxy_get(bool * p_enabled)
{
    return mesh_config_entry_get(MESH_OPT_GATT_PROXY_EID, p_enabled);
}

#if defined UNIT_TEST
/* This function MUST be only used by unit tests. */
void proxy_deinit(void)
{
    m_initialized = false;
}
#endif /* UNIT_TEST */

#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */
