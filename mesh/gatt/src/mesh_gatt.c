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

#include "mesh_gatt.h"

#include "nrf_mesh_gatt.h"
#if MESH_FEATURE_GATT_ENABLED

#include <stdint.h>
#include "ble_hci.h"
#include "ble_gatt.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_types.h"
#include "nrf_error.h"

#include "app_util.h"
#include "nordic_common.h"

#include "nrf_mesh_assert.h"
#include "packet_buffer.h"
#include "timer_scheduler.h"
#include "timer.h"
#include "bearer_event.h"

#define PROXY_SAR_TYPE_COMPLETE      (0)
#define PROXY_SAR_TYPE_FIRST_SEGMENT (1)
#define PROXY_SAR_TYPE_CONT_SEGMENT  (2)
#define PROXY_SAR_TYPE_LAST_SEGMENT  (3)

#define MESH_GATT_WRITE_OVERHEAD (3)
#define MESH_GATT_CONN_INDEX_INVALID (0xFFFF)

#define MESH_GATT_PDU_TYPE_MASK (0x3f)

typedef struct __attribute((packed))
{
    uint8_t pdu_type : 6;
    uint8_t sar_type : 2;
    uint8_t pdu[];
} mesh_gatt_proxy_pdu_t;

typedef struct __attribute((packed))
{
    nrf_mesh_tx_token_t token;
    uint8_t length;             /**< Length of the PDU. */
    uint8_t pdu[];
} mesh_gatt_proxy_buffer_t;

static bearer_event_flag_t m_flag = BEARER_EVENT_FLAG_INVALID;

static void tx_complete_handle(uint16_t conn_handle);
static uint16_t conn_handle_to_index(uint16_t conn_handle);

/*******************************************************************************
 * Internal static functions
 ******************************************************************************/

#if defined(UNIT_TEST) || defined(_lint)
extern mesh_gatt_t m_gatt;
#else
#include "nrf_sdh_ble.h"
static mesh_gatt_t m_gatt;
NRF_SDH_BLE_OBSERVER(m_gatt_obs, NRF_MESH_GATT_BLE_OBSERVER_PRIO, mesh_gatt_on_ble_evt, &m_gatt);
#endif

static uint8_t sar_type_get(uint8_t pdu_length, uint8_t offset, uint16_t mtu)
{
    if (offset == 0)
    {
        if (pdu_length <= mtu)
        {
            return PROXY_SAR_TYPE_COMPLETE;
        }
        else
        {
            return PROXY_SAR_TYPE_FIRST_SEGMENT;
        }
    }
    else
    {
        if ((pdu_length - offset) <= mtu)
        {
            return PROXY_SAR_TYPE_LAST_SEGMENT;
        }
        else
        {
            return PROXY_SAR_TYPE_CONT_SEGMENT;
        }
    }
}

static void mesh_gatt_pdu_send(mesh_gatt_connection_t * p_conn)
{
    packet_buffer_packet_t * p_packet = p_conn->tx.transaction.p_curr_packet;
    NRF_MESH_ASSERT(p_packet != NULL);

    mesh_gatt_proxy_buffer_t * p_proxy_buffer = (mesh_gatt_proxy_buffer_t *) p_packet->packet;
    /* Ensure the offset is not larger than the PDU.  */
    NRF_MESH_ASSERT(p_conn->tx.transaction.offset < p_proxy_buffer->length);

    uint8_t sar_type = sar_type_get(p_proxy_buffer->length,
                                    p_conn->tx.transaction.offset,
                                    p_conn->effective_mtu);

    uint16_t length = 0;
    if (sar_type == PROXY_SAR_TYPE_FIRST_SEGMENT ||
        sar_type == PROXY_SAR_TYPE_CONT_SEGMENT)
    {
        length = p_conn->effective_mtu;
    }
    else if (sar_type == PROXY_SAR_TYPE_COMPLETE ||
             sar_type == PROXY_SAR_TYPE_LAST_SEGMENT)
    {
        length = p_proxy_buffer->length - p_conn->tx.transaction.offset;
    }

    NRF_MESH_ASSERT(length <= p_conn->effective_mtu);

    mesh_gatt_proxy_pdu_t * p_proxy_pdu =
        (mesh_gatt_proxy_pdu_t *) &p_proxy_buffer->pdu[p_conn->tx.transaction.offset];

    p_proxy_pdu->sar_type = sar_type;

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    uint16_t hvx_length = length;
    hvx_params.handle = m_gatt.handles.tx.value_handle;
    hvx_params.p_data = (const uint8_t *) p_proxy_pdu;
    hvx_params.p_len  = &hvx_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    __LOG_XB(LOG_SRC_BEARER, LOG_LEVEL_DBG3, "HVN data", hvx_params.p_data, hvx_length);
    uint32_t err_code = sd_ble_gatts_hvx(p_conn->conn_handle, &hvx_params);

    /* There might be a system attributes missing event pending. Set it and try again. */
    if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        NRF_MESH_ERROR_CHECK(sd_ble_gatts_sys_attr_set(p_conn->conn_handle, NULL, 0, 0));
        err_code = sd_ble_gatts_hvx(p_conn->conn_handle, &hvx_params);
    }

    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        /* If we're not able to transmit. The client might have disabled notifications. */
        uint16_t conn_index = conn_handle_to_index(p_conn->conn_handle);
        if (conn_index != MESH_GATT_CONN_INDEX_INVALID)
        {
            (void) mesh_gatt_disconnect(conn_handle_to_index(p_conn->conn_handle));
        }
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        /* Try again at the next TX_COMPLETE. */
    }
    else
    {
        /* If we are able to send, we should have sent the full length. */
        NRF_MESH_ASSERT(err_code == NRF_SUCCESS && hvx_length == length);

        /* Next offset starts at the final byte of the previous packet. */
        uint8_t next_offset = p_conn->tx.transaction.offset + length - sizeof(mesh_gatt_proxy_pdu_t);

        if (sar_type == PROXY_SAR_TYPE_FIRST_SEGMENT ||
            sar_type == PROXY_SAR_TYPE_CONT_SEGMENT)
        {
            /* Copy the header in front of the next packet. */
            memcpy(&p_proxy_buffer->pdu[next_offset],
                   &p_proxy_buffer->pdu[p_conn->tx.transaction.offset],
                   sizeof(mesh_gatt_proxy_pdu_t));
        }

        /* We update the offset even though it's a single segment packet, that way we know that
         * _our_ packet is ready to be freed -- in case we have multiple users doing HVX. */
        p_conn->tx.transaction.offset = next_offset;
    }

    if (err_code == NRF_SUCCESS)
    {
        p_conn->tx.tx_complete_process = true;
        bearer_event_flag_set(m_flag);
    }

    __LOG(LOG_SRC_BEARER, LOG_LEVEL_DBG3, "status: %d len: %d usable-mtu:%d sar_type: %d \n", err_code, hvx_length, p_conn->effective_mtu, sar_type);
}

static void tx_state_clear(mesh_gatt_connection_t * p_conn)
{
    if (p_conn->tx.transaction.p_curr_packet != NULL)
    {
        packet_buffer_free(&p_conn->tx.packet_buffer, p_conn->tx.transaction.p_curr_packet);
        p_conn->tx.transaction.p_curr_packet = NULL;
        p_conn->tx.transaction.offset = 0;
    }
}

static void rx_state_clear(mesh_gatt_connection_t * p_conn)
{
    timer_sch_abort(&p_conn->rx.timeout_event);
    p_conn->rx.offset = 0;
    p_conn->rx.pdu_type = MESH_GATT_PDU_TYPE_INVALID;
}

static uint16_t conn_index_alloc(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_gatt.connections[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            m_gatt.connections[i].conn_handle = conn_handle;
            return i;
        }
    }

    return MESH_GATT_CONN_INDEX_INVALID;
}

static void conn_index_free(uint16_t conn_index)
{
    NRF_MESH_ASSERT(conn_index < MESH_GATT_CONNECTION_COUNT_MAX);
    rx_state_clear(&m_gatt.connections[conn_index]);
    tx_state_clear(&m_gatt.connections[conn_index]);
    packet_buffer_flush(&m_gatt.connections[conn_index].tx.packet_buffer);
    m_gatt.connections[conn_index].conn_handle = BLE_CONN_HANDLE_INVALID;
}

static uint16_t conn_handle_to_index(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        if (m_gatt.connections[i].conn_handle == conn_handle)
        {
            return i;
        }
    }

    return MESH_GATT_CONN_INDEX_INVALID;
}

static bool rx_session_in_progress(mesh_gatt_connection_t * p_conn)
{
    return (p_conn->rx.offset > 0);
}

static inline bool pdu_length_valid(mesh_gatt_connection_t * p_conn, uint16_t length)
{

    /* We get the +1 because we ignore the PDU_TYPE header when concatenating
     * the packets in the RX buffer. In addition, the message must actually contain some data in
     * addition to the header. */
    return (length > sizeof(mesh_gatt_proxy_pdu_t) &&
            (length <= (sizeof(p_conn->rx.buffer) - p_conn->rx.offset + sizeof(mesh_gatt_proxy_pdu_t))));
}

static bool tx_transaction_is_complete(mesh_gatt_connection_t * p_conn, mesh_gatt_proxy_buffer_t * p_proxy_buffer)
{
    return (p_conn->tx.transaction.offset == p_proxy_buffer->length - 1);
}

static bool pdu_type_valid(mesh_gatt_connection_t * p_conn, uint8_t pdu_type)
{
    return (pdu_type < MESH_GATT_PDU_TYPE_PROHIBITED &&
            ((p_conn->rx.pdu_type == pdu_type) ||
             (p_conn->rx.pdu_type == MESH_GATT_PDU_TYPE_INVALID)));
}

static void rx_timer_restart(mesh_gatt_connection_t * p_conn)
{
    timer_sch_reschedule(&p_conn->rx.timeout_event, timer_now() + MESH_GATT_RX_SAR_TIMEOUT_US);
}

static void proxy_pdu_rx_cont(uint16_t conn_index, const uint8_t * p_data, uint16_t length)
{
    mesh_gatt_connection_t * p_conn = &m_gatt.connections[conn_index];
    const mesh_gatt_proxy_pdu_t * p_pdu = (const mesh_gatt_proxy_pdu_t *) p_data;
    uint16_t pdu_length = length - sizeof(mesh_gatt_proxy_pdu_t);

    switch (p_pdu->sar_type)
    {
        case PROXY_SAR_TYPE_CONT_SEGMENT:
            rx_timer_restart(p_conn);
            memcpy(&p_conn->rx.buffer[p_conn->rx.offset], p_pdu->pdu, pdu_length);
            p_conn->rx.offset += pdu_length;
            break;

        case PROXY_SAR_TYPE_LAST_SEGMENT:
        {
            memcpy(&p_conn->rx.buffer[p_conn->rx.offset], p_pdu->pdu, pdu_length);

            mesh_gatt_evt_t evt;
            evt.type = MESH_GATT_EVT_TYPE_RX;
            evt.conn_index = conn_index;
            evt.params.rx.pdu_type = p_conn->rx.pdu_type;
            evt.params.rx.p_data = p_conn->rx.buffer;
            evt.params.rx.length = p_conn->rx.offset + pdu_length;
            m_gatt.evt_handler(&evt, m_gatt.p_context);
            rx_state_clear(p_conn);
            break;
        }

        case PROXY_SAR_TYPE_COMPLETE:
        case PROXY_SAR_TYPE_FIRST_SEGMENT:
        default:
            /* Invalid PDUs in this state. */
            (void) mesh_gatt_disconnect(conn_index);
            break;
    }
}

static void proxy_pdu_rx_first(uint16_t conn_index, const uint8_t * p_data, uint16_t length)
{
    mesh_gatt_connection_t * p_conn = &m_gatt.connections[conn_index];
    const mesh_gatt_proxy_pdu_t * p_pdu = (const mesh_gatt_proxy_pdu_t *) p_data;
    uint16_t pdu_length = length - sizeof(mesh_gatt_proxy_pdu_t);

    switch (p_pdu->sar_type)
    {
        case PROXY_SAR_TYPE_COMPLETE:
        {
            mesh_gatt_evt_t evt;
            evt.type = MESH_GATT_EVT_TYPE_RX;
            evt.conn_index = conn_index;
            evt.params.rx.pdu_type = (mesh_gatt_pdu_type_t) p_pdu->pdu_type;
            evt.params.rx.p_data = p_pdu->pdu;
            evt.params.rx.length = pdu_length;
            m_gatt.evt_handler(&evt, m_gatt.p_context);
            break;
        }
        case PROXY_SAR_TYPE_FIRST_SEGMENT:
        {
            rx_timer_restart(p_conn);
            memcpy(p_conn->rx.buffer, p_pdu->pdu, pdu_length);
            p_conn->rx.pdu_type = (mesh_gatt_pdu_type_t) p_pdu->pdu_type;
            p_conn->rx.offset = pdu_length;
            break;
        }

        default:
            /* Invalid PDU in this state. */
            (void) mesh_gatt_disconnect(conn_index);
            break;
    }
}

static void timeout_cb(timestamp_t time_now, void * p_context)
{
    mesh_gatt_connection_t * p_conn = p_context;
    uint32_t err_code = sd_ble_gap_disconnect(p_conn->conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    /* There might be a race condition in the disconnection, let's survive that! */
    NRF_MESH_ASSERT(err_code == NRF_SUCCESS ||
                    err_code == NRF_ERROR_INVALID_STATE);
}

static void pdu_send_if_available(mesh_gatt_connection_t * p_conn)
{
    if (packet_buffer_can_pop(&p_conn->tx.packet_buffer))
    {
        /* Start transmitting new pdu */
        NRF_MESH_ERROR_CHECK(packet_buffer_pop(&p_conn->tx.packet_buffer,
                                               &p_conn->tx.transaction.p_curr_packet));
        NRF_MESH_ASSERT(p_conn->tx.transaction.p_curr_packet != NULL);

        mesh_gatt_pdu_send(p_conn);
    }
}

static void tx_complete_send_and_enqueue(mesh_gatt_connection_t * p_conn)
{
    mesh_gatt_proxy_buffer_t * p_proxy_buffer = (mesh_gatt_proxy_buffer_t *) p_conn->tx.transaction.p_curr_packet->packet;

    if (tx_transaction_is_complete(p_conn, p_proxy_buffer))
    {
        tx_complete_handle(p_conn->conn_handle);

        pdu_send_if_available(p_conn);
    }
    else
    {
        mesh_gatt_pdu_send(p_conn);
    }
}

static bool trigger_tx_complete(void)
{
    /* Find out which connection triggered the event and process it. */
    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; i++)
    {
        if (m_gatt.connections[i].tx.tx_complete_process)
        {
            m_gatt.connections[i].tx.tx_complete_process = false;
            tx_complete_send_and_enqueue(&m_gatt.connections[i]);
        }
    }

    return true;
}

/*******************************************************************************
 * Event handlers
 ******************************************************************************/

static void write_evt_handle(const ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    uint16_t conn_index = conn_handle_to_index(p_ble_evt->evt.gatts_evt.conn_handle);
    if (conn_index == MESH_GATT_CONN_INDEX_INVALID)
    {
        return;
    }

    if ((p_evt_write->handle == m_gatt.handles.tx.cccd_handle)
        && (p_evt_write->len == 2))
    {
        bool notification_enabled =
            (uint16_decode(p_evt_write->data) & BLE_GATT_HVX_NOTIFICATION) != 0;

        if (notification_enabled)
        {
            mesh_gatt_evt_t evt;
            evt.type = MESH_GATT_EVT_TYPE_TX_READY;
            evt.conn_index = conn_index;
            m_gatt.evt_handler(&evt, m_gatt.p_context);
        }
        else
        {
            /* The client disabled the notifications. This is sort of against the spec,
             * but we'll try to play nice and instead disconnect if we're trying to
             * write when in an invalid state. See handling of sd_ble_gatts_hvx() returns.
             */
        }

    }
    else if ((p_evt_write->handle == m_gatt.handles.rx.value_handle))
    {
        if (pdu_length_valid(&m_gatt.connections[conn_index], p_evt_write->len) &&
            pdu_type_valid(&m_gatt.connections[conn_index], p_evt_write->data[0] & MESH_GATT_PDU_TYPE_MASK))
        {
            if (rx_session_in_progress(&m_gatt.connections[conn_index]))
            {
                proxy_pdu_rx_cont(conn_index, p_evt_write->data, p_evt_write->len);
            }
            else
            {
                proxy_pdu_rx_first(conn_index, p_evt_write->data, p_evt_write->len);
            }
        }
        else
        {
            (void) mesh_gatt_disconnect(conn_index);
        }
    }
    else
    {
        // Do Nothing.
    }
}

static void tx_complete_handle(uint16_t conn_handle)
{
    /**
     * NOTE: We're assuming that a successful call to sd_ble_gatts_hvx() means guarantee of
     * delivery. Hence, the TX_COMPLETE event is merely used for flow control.
     */

    uint16_t conn_index = conn_handle_to_index(conn_handle);
    if (conn_index == MESH_GATT_CONN_INDEX_INVALID)
    {
        return;
    }

    mesh_gatt_connection_t * p_conn = &m_gatt.connections[conn_index];
    packet_buffer_packet_t * p_packet = p_conn->tx.transaction.p_curr_packet;

    if (p_packet == NULL)
    {
        /* Some other HVX user transmitted a packet. We don't have anything to send for this
         * connection index, so we'll return here. */
        return;
    }

    mesh_gatt_proxy_buffer_t * p_proxy_buffer = (mesh_gatt_proxy_buffer_t *) p_packet->packet;

    tx_state_clear(p_conn);

    mesh_gatt_evt_t evt;
    evt.type = MESH_GATT_EVT_TYPE_TX_COMPLETE;
    evt.conn_index = conn_index;
    evt.params.tx_complete.pdu_type = (mesh_gatt_pdu_type_t) ((mesh_gatt_proxy_pdu_t *) p_proxy_buffer->pdu)->pdu_type;
    evt.params.tx_complete.token = p_proxy_buffer->token;

    m_gatt.evt_handler(&evt, m_gatt.p_context);
}

static void disconnect_evt_handle(const ble_evt_t * p_ble_evt)
{
    uint16_t conn_index = conn_handle_to_index(p_ble_evt->evt.gap_evt.conn_handle);
    if (conn_index != MESH_GATT_CONN_INDEX_INVALID)
    {
        conn_index_free(conn_index);

        mesh_gatt_evt_t evt;
        evt.type = MESH_GATT_EVT_TYPE_DISCONNECTED;
        evt.conn_index = conn_index;
        m_gatt.evt_handler(&evt, m_gatt.p_context);
    }
}

static void connect_evt_handle(const ble_evt_t * p_ble_evt)
{
    uint16_t conn_index = conn_index_alloc(p_ble_evt->evt.gap_evt.conn_handle);
    if (conn_index != MESH_GATT_CONN_INDEX_INVALID)
    {
        /* Clear the buffer state. */
        packet_buffer_init(&m_gatt.connections[conn_index].tx.packet_buffer,
                           m_gatt.connections[conn_index].tx.packet_buffer_data,
                           sizeof(m_gatt.connections[conn_index].tx.packet_buffer_data));
        m_gatt.connections[conn_index].tx.tx_complete_process = false;
        mesh_gatt_evt_t evt;
        evt.type = MESH_GATT_EVT_TYPE_CONNECTED;
        evt.conn_index = conn_index;
        m_gatt.evt_handler(&evt, m_gatt.p_context);
    }
}

static void exchange_mtu_req_handle(const ble_evt_t * p_ble_evt)
{
    uint16_t conn_index = conn_handle_to_index(p_ble_evt->evt.gatts_evt.conn_handle);
    if (conn_index != MESH_GATT_CONN_INDEX_INVALID)
    {
        uint16_t client_rx_mtu = p_ble_evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu;
        uint16_t server_rx_mtu = MAX(MIN(MIN(client_rx_mtu, NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
                                         MESH_GATT_MTU_SIZE_MAX),
                                     BLE_GATT_ATT_MTU_DEFAULT);

        uint32_t status = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, server_rx_mtu);

        if (status == NRF_SUCCESS)
        {
            m_gatt.connections[conn_index].effective_mtu = server_rx_mtu - MESH_GATT_WRITE_OVERHEAD;
            __LOG(LOG_SRC_BEARER, LOG_LEVEL_INFO, "New MTU: %d\n", m_gatt.connections[conn_index].effective_mtu);
        }
        else
        {
            __LOG(LOG_SRC_INTERNAL,
                  LOG_LEVEL_WARN,
                  "Got %#x on exchange_mtu_reply with client mtu %u and server mtu %u\n",
                  status,
                  client_rx_mtu,
                  server_rx_mtu);
        }
    }
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

void mesh_gatt_init(const mesh_gatt_uuids_t * p_uuids,
                    mesh_gatt_evt_handler_t evt_handler,
                    void * p_context)
{
    NRF_MESH_ASSERT(p_uuids != NULL && evt_handler != NULL);

    for (uint32_t i = 0; i < MESH_GATT_CONNECTION_COUNT_MAX; ++i)
    {
        m_gatt.connections[i].effective_mtu = BLE_GATT_ATT_MTU_DEFAULT - MESH_GATT_WRITE_OVERHEAD;
        m_gatt.connections[i].rx.pdu_type = MESH_GATT_PDU_TYPE_INVALID;
        m_gatt.connections[i].rx.timeout_event.cb = timeout_cb;
        m_gatt.connections[i].rx.timeout_event.p_context = &m_gatt.connections[i];
        m_gatt.connections[i].conn_handle = BLE_CONN_HANDLE_INVALID;
        memset(m_gatt.connections[i].tx.packet_buffer_data,
               0,
               sizeof(m_gatt.connections[i].tx.packet_buffer_data));
        packet_buffer_init(&m_gatt.connections[i].tx.packet_buffer,
                           m_gatt.connections[i].tx.packet_buffer_data,
                           sizeof(m_gatt.connections[i].tx.packet_buffer_data));
    }

    memcpy(&m_gatt.uuids, p_uuids, sizeof(mesh_gatt_uuids_t));
    m_gatt.evt_handler = evt_handler;

    ble_uuid_t uuid = {.type = BLE_UUID_TYPE_BLE, .uuid = p_uuids->service};
    NRF_MESH_ERROR_CHECK(sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, &m_gatt.handles.service));


    /* Set up RX characteristic */
    uuid.uuid = p_uuids->rx_char;
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write_wo_resp = 1;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MESH_GATT_PROXY_PDU_MAX_SIZE;
    attr_char_value.p_value   = NULL; /* VLOC_STACK */

    NRF_MESH_ERROR_CHECK(sd_ble_gatts_characteristic_add(m_gatt.handles.service, &char_md, &attr_char_value, &m_gatt.handles.rx));

    /* Set up TX characteristic */
    uuid.uuid = p_uuids->tx_char;
    /* memset(&attr_char_value, 0, sizeof(attr_char_value)); */
    /* attr_char_value.p_uuid    = &uuid; */
    /* attr_char_value.p_attr_md = &attr_md; */
    /* attr_char_value.init_len  = 0; */
    /* attr_char_value.init_offs = 0; */
    /* attr_char_value.max_len   = MESH_GATT_PACKET_MAX_SIZE; */
    /* attr_char_value.p_value   = NULL; /\* VLOC_STACK *\/ */
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    NRF_MESH_ERROR_CHECK(sd_ble_gatts_characteristic_add(m_gatt.handles.service, &char_md, &attr_char_value, &m_gatt.handles.tx));

    m_gatt.p_context = p_context;

    /* Ensure that bearer event flag gets added only once. */
    if (m_flag == BEARER_EVENT_FLAG_INVALID)
    {
        m_flag = bearer_event_flag_add(trigger_tx_complete);
    }
}

uint8_t * mesh_gatt_packet_alloc(uint16_t conn_index,
                                 mesh_gatt_pdu_type_t type,
                                 uint16_t size,
                                 nrf_mesh_tx_token_t token)
{
    NRF_MESH_ASSERT(conn_index < MESH_GATT_CONNECTION_COUNT_MAX);
    NRF_MESH_ASSERT(type <= MESH_GATT_PDU_TYPE_PROV_PDU);
    NRF_MESH_ASSERT(size <= MESH_GATT_PROXY_PDU_MAX_SIZE);
    if (m_gatt.connections[conn_index].conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NULL;
    }

    uint16_t buffer_size = size + sizeof(mesh_gatt_proxy_buffer_t) + sizeof(mesh_gatt_proxy_pdu_t);
    packet_buffer_packet_t * p_packet = NULL;
    uint32_t status = packet_buffer_reserve(&m_gatt.connections[conn_index].tx.packet_buffer,
                                            &p_packet,
                                            buffer_size);
    if (status == NRF_SUCCESS)
    {
        mesh_gatt_proxy_buffer_t * p_proxy_buffer =  (mesh_gatt_proxy_buffer_t *) p_packet->packet;
        p_proxy_buffer->token = token;
        p_proxy_buffer->length = sizeof(mesh_gatt_proxy_pdu_t) + size;
        mesh_gatt_proxy_pdu_t * p_proxy_pdu = (mesh_gatt_proxy_pdu_t *) p_proxy_buffer->pdu;
        p_proxy_pdu->pdu_type = type;
        return p_proxy_pdu->pdu;
    }
    else
    {
        NRF_MESH_ASSERT(NRF_ERROR_NO_MEM == status);
        return NULL;
    }
}


uint32_t mesh_gatt_packet_send(uint16_t conn_index, const uint8_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL);
    NRF_MESH_ASSERT(conn_index < MESH_GATT_CONNECTION_COUNT_MAX);
    mesh_gatt_connection_t * p_conn = &m_gatt.connections[conn_index];
    if (p_conn->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    /* TODO: Check if notifications are enabled at this point? Or let application keep that responsibility? */
    mesh_gatt_proxy_pdu_t * p_proxy_pdu = PARENT_BY_FIELD_GET(mesh_gatt_proxy_pdu_t, pdu, p_packet);
    mesh_gatt_proxy_buffer_t * p_proxy_buffer = PARENT_BY_FIELD_GET(mesh_gatt_proxy_buffer_t, pdu, p_proxy_pdu);
    packet_buffer_packet_t * p_buf_packet = PARENT_BY_FIELD_GET(packet_buffer_packet_t, packet, p_proxy_buffer);

    packet_buffer_commit(&m_gatt.connections[conn_index].tx.packet_buffer, p_buf_packet, p_buf_packet->size);

    /* If there is an ongoing transaction, the packet buffer shouldn't allow us to pop the next packet. */
    pdu_send_if_available(p_conn);

    return NRF_SUCCESS;
}

void mesh_gatt_packet_discard(uint16_t conn_index, const uint8_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL);
    NRF_MESH_ASSERT(conn_index < MESH_GATT_CONNECTION_COUNT_MAX);
    NRF_MESH_ASSERT(m_gatt.connections[conn_index].conn_handle != BLE_CONN_HANDLE_INVALID);

    mesh_gatt_proxy_pdu_t * p_proxy_pdu = PARENT_BY_FIELD_GET(mesh_gatt_proxy_pdu_t, pdu, p_packet);
    mesh_gatt_proxy_buffer_t * p_proxy_buffer = PARENT_BY_FIELD_GET(mesh_gatt_proxy_buffer_t, pdu, p_proxy_pdu);
    packet_buffer_packet_t * p_buf_packet = PARENT_BY_FIELD_GET(packet_buffer_packet_t, packet, p_proxy_buffer);
    packet_buffer_free(&m_gatt.connections[conn_index].tx.packet_buffer, p_buf_packet);
}

bool mesh_gatt_packet_is_pending(uint16_t conn_index)
{
    return (!packet_buffer_is_empty(&m_gatt.connections[conn_index].tx.packet_buffer));
}

uint32_t mesh_gatt_disconnect(uint16_t conn_index)
{
    NRF_MESH_ASSERT(conn_index < MESH_GATT_CONNECTION_COUNT_MAX);

    /* The timer is stopped in the disconnected event, but we stop it here anyway. */
    timer_sch_abort(&m_gatt.connections[conn_index].rx.timeout_event);
    return sd_ble_gap_disconnect(m_gatt.connections[conn_index].conn_handle,
                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

void mesh_gatt_on_ble_evt(const ble_evt_t * p_ble_evt, void * p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            connect_evt_handle(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            disconnect_evt_handle(p_ble_evt);
            break;
#if NRF_SD_BLE_API_VERSION == 6 || NRF_SD_BLE_API_VERSION == 7
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason ==
                BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
                mesh_gatt_evt_t evt;
                evt.type = MESH_GATT_EVT_TYPE_ADV_TIMEOUT;
                evt.conn_index = p_ble_evt->evt.gap_evt.conn_handle;
                m_gatt.evt_handler(&evt, m_gatt.p_context);
            }
            break;
#elif NRF_SD_BLE_API_VERSION <= 5
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                mesh_gatt_evt_t evt;
                evt.type = MESH_GATT_EVT_TYPE_ADV_TIMEOUT;
                evt.conn_index = p_ble_evt->evt.gap_evt.conn_handle;
                m_gatt.evt_handler(&evt, m_gatt.p_context);
            }
            break;
#else
#error Unsupported SoftDevice version
#endif
        case BLE_GATTS_EVT_WRITE:
            write_evt_handle(p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            {
                uint16_t conn_index = conn_handle_to_index(p_ble_evt->evt.gatts_evt.conn_handle);
                if (conn_index != MESH_GATT_CONN_INDEX_INVALID)
                {
                    /* TX complete is already sent if current packet is NULL, and no fresh PDU was
                       available. */
                    if (m_gatt.connections[conn_index].tx.transaction.p_curr_packet == NULL)
                    {
                        pdu_send_if_available(&m_gatt.connections[conn_index]);
                    }
                    else
                    {
                        m_gatt.connections[conn_index].tx.tx_complete_process = true;
                        bearer_event_flag_set(m_flag);
                    }
                }
            }
            break;


        /* TODO: The following events should be handled by an SDK module/the application. */
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        {
            /* This call might have been called already as a result of a failing gatts call, ignore the error code. */
            (void) sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            break;
        }

#if !defined(S112)
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;
            memset(&dl_params, BLE_GAP_DATA_LENGTH_AUTO, sizeof(dl_params));
            NRF_MESH_ERROR_CHECK(sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle,
                                                               &dl_params,
                                                               NULL));
            break;
        }
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_MESH_ERROR_CHECK(sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                             BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                             NULL,
                                                             NULL));
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            NRF_MESH_ERROR_CHECK(sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys));
            break;
        }

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            exchange_mtu_req_handle(p_ble_evt);
            break;

        case BLE_GATTS_EVT_SC_CONFIRM:
            break;

        default:
            break;
    }
}

#endif /* MESH_FEATURE_GATT_ENABLED */
