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
#include <stddef.h>
#include <nrf_error.h>

#include "provisioning.h"
#include "prov_pdu.h"

#include "fifo.h"
#include "utils.h"
#include "log.h"
#include "access.h"
#include "access_config.h"
#include "access_reliable.h"

#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"

#include "pb_remote_msgs.h"
#include "pb_remote.h"
#include "pb_remote_server.h"

#define __LOG_EVENT(__EVT)                                              \
    if (evt.type != PB_REMOTE_SERVER_EVENT_UNPROV_UUID)                 \
    {                                                                   \
        __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "SEvent: %u\n", __EVT);   \
    }

#define __LOG_STATE_CHANGE(__PREV_STATE, __NEXT_STATE)  \
    if (__PREV_STATE != __NEXT_STATE)                   \
    {                                                   \
        __LOG(LOG_SRC_ACCESS,                           \
              LOG_LEVEL_DBG1,                           \
              "PB-Mesh Server: \"%u\" -> \"%u\"\n",     \
              __PREV_STATE,                             \
              __NEXT_STATE);                            \
    }

#define PB_REMOTE_SERVER_EVENT_QUEUE_SIZE (2)
#define PB_REMOTE_SERVER_ALT_EVENT_QUEUE_SIZE (2)
/*******************************************************************************
 * Local definitions
 *******************************************************************************/

typedef enum
{
    PB_REMOTE_SERVER_EVENT_SCAN_START,
    PB_REMOTE_SERVER_EVENT_SCAN_START_FILTER,
    PB_REMOTE_SERVER_EVENT_SCAN_CANCEL,
    PB_REMOTE_SERVER_EVENT_SCAN_REPORT_STATUS,
    PB_REMOTE_SERVER_EVENT_LINK_OPEN,
    PB_REMOTE_SERVER_EVENT_LINK_ESTABLISHED,
    PB_REMOTE_SERVER_EVENT_LINK_CLOSE,
    PB_REMOTE_SERVER_EVENT_LINK_CLOSED,
    PB_REMOTE_SERVER_EVENT_LINK_STATUS,
    PB_REMOTE_SERVER_EVENT_UNPROV_UUID,
    PB_REMOTE_SERVER_EVENT_PACKET_TRANSFER,
    PB_REMOTE_SERVER_EVENT_LOCAL_ACK,
    PB_REMOTE_SERVER_EVENT_LOCAL_PACKET,
    PB_REMOTE_SERVER_EVENT_TRANSFER_STATUS,
    PB_REMOTE_SERVER_EVENT_TX_FAILED,
    PB_REMOTE_SERVER_EVENT_LAST       /**< Used to set the size of the event handler list. */
} pb_remote_server_event_type_t;

typedef struct
{
    uint16_t length;
    uint8_t payload[PROV_PDU_MAX_LENGTH];
} pb_remote_event_pb_adv_pdu_t;

typedef struct
{
    pb_remote_server_event_type_t type;
    union
    {
        pb_remote_event_pb_adv_pdu_t pdu;
        const access_message_rx_t * p_message;
        nrf_mesh_prov_evt_unprov_t unprov;
        nrf_mesh_prov_link_close_reason_t link_close_reason;
    } evt;
} pb_remote_server_event_t;

typedef struct
{
    bool slot_active;
    uint8_t uuid[NRF_MESH_UUID_SIZE];
} pb_remote_uuid_slot_t;

typedef pb_remote_server_state_t (*pb_remote_server_event_handler_cb_t)(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);

typedef struct
{
    uint16_t opcode;
    uint16_t length;
    pb_remote_server_event_type_t event_type;
} message_length_lut_t;

/*******************************************************************************
 * Static variables
 *******************************************************************************/


static void remote_server_access_rx_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args);

/* TODO: If possible, the each of the opcodes should be handled in their event callback directly,
 * but that would require a bigger re-work of the design. */
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{PB_REMOTE_OP_PACKET_TRANSFER,                  ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_PACKET_TRANSFER_STATUS,           ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_LINK_CLOSE,                       ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_LINK_OPEN,                        ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_LINK_STATUS,                      ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_SCAN_START,                       ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_SCAN_CANCEL,                      ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_SCAN_START_FILTER,                ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb},
    {{PB_REMOTE_OP_SCAN_REPORT_STATUS,               ACCESS_COMPANY_ID_NONE}, remote_server_access_rx_cb}
};

/* Forward declarations */
static pb_remote_server_state_t pb_remote_server_event_scan_start_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_scan_start_filter_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_scan_cancel_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_scan_report_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_link_open_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_link_established_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_link_close_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_link_closed_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_link_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_unprov_uuid_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_packet_transfer_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_local_ack_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_local_packet_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_transfer_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);
static pb_remote_server_state_t pb_remote_server_event_tx_failed_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt);

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt);
static void pb_adv_rx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static void pb_adv_ack_cb(prov_bearer_t * p_bearer);
static void pb_adv_link_opened_cb(prov_bearer_t * p_bearer);
static void pb_adv_link_closed_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);

static const pb_remote_server_event_handler_cb_t
m_pb_remote_server_event_handler_callbacks[PB_REMOTE_SERVER_EVENT_LAST] =
{
    pb_remote_server_event_scan_start_cb,
    pb_remote_server_event_scan_start_filter_cb,
    pb_remote_server_event_scan_cancel_cb,
    pb_remote_server_event_scan_report_status_cb,
    pb_remote_server_event_link_open_cb,
    pb_remote_server_event_link_established_cb,
    pb_remote_server_event_link_close_cb,
    pb_remote_server_event_link_closed_cb,
    pb_remote_server_event_link_status_cb,
    pb_remote_server_event_unprov_uuid_cb,
    pb_remote_server_event_packet_transfer_cb,
    pb_remote_server_event_local_ack_cb,
    pb_remote_server_event_local_packet_cb,
    pb_remote_server_event_transfer_status_cb,
    pb_remote_server_event_tx_failed_cb
};

static pb_remote_server_t * mp_pb_remote_server_model;
static pb_remote_uuid_slot_t m_uuid_list[PB_REMOTE_SERVER_UUID_LIST_SIZE];

/** Buffer for storing reliable packet. */
static union
{
    pb_remote_packet_t packet;
    uint8_t buffer[PROV_PDU_MAX_LENGTH];
} m_packet;

static pb_remote_server_event_t m_pbr_server_event_queue_buffer[PB_REMOTE_SERVER_EVENT_QUEUE_SIZE];
static fifo_t m_pbr_server_event_queue;

static pb_remote_server_event_t m_pbr_server_alt_event_queue_buffer[PB_REMOTE_SERVER_ALT_EVENT_QUEUE_SIZE];
static fifo_t m_pbr_server_alt_event_queue;

static bool m_is_interrupting;

static void pb_remote_server_process(pb_remote_server_t * p_ctx);

/*******************************************************************************
 * Static functions
 *******************************************************************************/

static inline void send_msg(const pb_remote_server_t * p_ctx, const access_message_tx_t * p_tx_msg)
{
    uint32_t status = access_model_publish(p_ctx->model_handle, p_tx_msg);
    NRF_MESH_ASSERT(NRF_SUCCESS == status || NRF_ERROR_NO_MEM == status);
}

static inline void send_reliable_msg(pb_remote_server_t * p_ctx, pb_remote_opcode_t opcode, pb_remote_opcode_t reply_opcode, uint16_t length)
{
    p_ctx->reliable.message.opcode.opcode = opcode;
    p_ctx->reliable.message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_ctx->reliable.message.p_buffer = m_packet.buffer;
    p_ctx->reliable.message.length = length;
    p_ctx->reliable.message.force_segmented = true;
    p_ctx->reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    p_ctx->reliable.message.access_token = nrf_mesh_unique_token_get();
    p_ctx->reliable.reply_opcode.opcode = reply_opcode;
    p_ctx->reliable.reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;

    NRF_MESH_ASSERT(access_model_reliable_publish(&p_ctx->reliable) == NRF_SUCCESS);
}

static inline void send_reply(const pb_remote_server_t * p_ctx, const access_message_rx_t * p_rx_msg, const access_message_tx_t * p_reply)
{
    uint32_t status = access_model_reply(p_ctx->model_handle, p_rx_msg, p_reply);
    NRF_MESH_ASSERT(NRF_SUCCESS == status || NRF_ERROR_NO_MEM == status);
}

static void do_state_change(pb_remote_server_t * p_ctx, pb_remote_server_state_t next_state)
{
    if (next_state != p_ctx->state)
    {
        p_ctx->prev_state = p_ctx->state;
        p_ctx->state      = next_state;
        __LOG_STATE_CHANGE(p_ctx->prev_state, p_ctx->state);
    }
}

/**
 *  Relays a message from the client over the local PB-ADV link.
 *
 *  If unsuccessful, an attempt to close the link is made.
 *
 *  @param[in] p_ctx Remote server context structure.
 *  @param[in] p_evt Remote server event pointer.
 *
 *  @returns The next state of the server based on whether the packet was relayed successfully or
 *  the link was closed successfully.
 */
static pb_remote_server_state_t relay_to_pb_adv_with_ack(pb_remote_server_t * p_ctx,
                                                         pb_remote_server_event_t * p_evt,
                                                         access_message_tx_t * p_reply,
                                                         pb_remote_msg_packet_transfer_status_t * p_transfer_status)
{
    pb_remote_msg_packet_transfer_t * p_packet_transfer = (pb_remote_msg_packet_transfer_t *) &p_evt->evt.p_message->p_data[0];
    if (p_ctx->p_prov_bearer->p_interface->tx(p_ctx->p_prov_bearer,
                                              &p_packet_transfer->buffer[0],
                                              p_evt->evt.p_message->length) == NRF_SUCCESS)
    {
        p_transfer_status->status = PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED;
        send_reply(p_ctx, p_evt->evt.p_message, p_reply);
        return PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL;
    }
    else
    {
        p_ctx->p_prov_bearer->p_interface->link_close(p_ctx->p_prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
        p_transfer_status->status = PB_REMOTE_PACKET_TRANSFER_STATUS_CANNOT_ACCEPT_BUFFER;
        send_reply(p_ctx, p_evt->evt.p_message, p_reply);
        /* The link is closing. When the link_closed callback is called, we will notify the client
         * of the closed link with a LINK_STATUS_REPORT and return to IDLE/SCANNING. */
        return PB_REMOTE_SERVER_STATE_LINK_CLOSING;
    }
}

static void server_reliable_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    NRF_MESH_ASSERT(NULL != p_args);
    pb_remote_server_t * p_self = p_args;

    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Got ACK for [aop: 0x%04x]\n", p_self->reliable.message.opcode.opcode);
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
        {
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unable to send message [aop: 0x%04x]\n", p_self->reliable.message.opcode.opcode);
            pb_remote_server_event_t event = {.type = PB_REMOTE_SERVER_EVENT_TX_FAILED};
            NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &event) == NRF_SUCCESS);
            pb_remote_server_process(p_self);
            break;
        }
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
        {
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Message cancelled [aop: 0x%04x]\n", p_self->reliable.message.opcode.opcode);
            break;
        }
        default:
            NRF_MESH_ASSERT(false);
            break;
    }

}

static pb_remote_server_state_t pb_remote_server_event_tx_failed_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT:
        {
            /* We couldn't send the reliable message. */
            /** @todo Should we give an event to the application here? */

            /* Close the link: */
            p_ctx->p_prov_bearer->p_interface->link_close(p_ctx->p_prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);

            /* Re-init the fifo to flush all enqueued PB-ADV packets. No reason to try to send them
             * if TX failed. */
            fifo_flush(&m_pbr_server_alt_event_queue);
            return PB_REMOTE_SERVER_STATE_IDLE;
        }
        default:
            /* We should not be getting this event in any other state(s). */
            NRF_MESH_ASSERT(false);
            return PB_REMOTE_SERVER_STATE_IDLE;
    }
}

static pb_remote_server_state_t pb_remote_server_event_scan_start_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_SCANNING_FILTER:
            /* Switch to unfiltered mode. */
            /* fall through. */
        case PB_REMOTE_SERVER_STATE_SCANNING:
            /* fall through */
        case PB_REMOTE_SERVER_STATE_IDLE:
        {
            memset(&m_uuid_list[0], 0, sizeof(m_uuid_list));
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_STARTED};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            NRF_MESH_ERROR_CHECK(nrf_mesh_prov_scan_start(prov_evt_handler));
            return PB_REMOTE_SERVER_STATE_SCANNING;
        }
        default:
        {
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            return p_ctx->state;
        }
    }
}


static pb_remote_server_state_t pb_remote_server_event_scan_start_filter_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_SCANNING:
            /* fall through */
        case PB_REMOTE_SERVER_STATE_IDLE:
            /* fall through */
        case PB_REMOTE_SERVER_STATE_SCANNING_FILTER:
        {
            pb_remote_msg_scan_start_filter_t * p_filter_msg = (pb_remote_msg_scan_start_filter_t *)
                (&p_evt->evt.p_message->p_data);

            __LOG_XB(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Adding filter", p_filter_msg->filter_uuid, NRF_MESH_UUID_SIZE);
            memcpy(&m_uuid_list[0].uuid[0], &p_filter_msg->filter_uuid[0], NRF_MESH_UUID_SIZE);

            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_STARTED};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            return PB_REMOTE_SERVER_STATE_SCANNING_FILTER;
        }
        default:
        {
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            return p_ctx->state;
        }
    }
}

static pb_remote_server_state_t pb_remote_server_event_scan_cancel_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_SCANNING_FILTER:
            /* fall through. */
        case PB_REMOTE_SERVER_STATE_SCANNING:
        {
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_CANCELED};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            nrf_mesh_prov_scan_stop();
            return PB_REMOTE_SERVER_STATE_IDLE;
        }

        default:
        {
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_CANNOT_CANCEL_SCANNING};
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            return p_ctx->state;
        }
    }
}

static pb_remote_server_state_t pb_remote_server_event_scan_report_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT:
            return p_ctx->prev_state;
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Unknown event %u in state %u\n", p_evt->type, p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_server_state_t local_link_open(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt, const uint8_t * p_uuid)
{
    NRF_MESH_ASSERT(p_ctx->state == PB_REMOTE_SERVER_STATE_IDLE                     ||
                    p_ctx->state == PB_REMOTE_SERVER_STATE_SCANNING                 ||
                    p_ctx->state == PB_REMOTE_SERVER_STATE_SCANNING_FILTER          ||
                    p_ctx->state == PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE      ||
                    p_ctx->state == PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT     ||
                    p_ctx->state == PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT_FILTER);

    uint32_t status = p_ctx->p_prov_bearer->p_interface->link_open(
        p_ctx->p_prov_bearer, p_uuid,  NRF_MESH_PROV_LINK_TIMEOUT_MIN_US);

    if (status == NRF_SUCCESS)
    {
        access_message_tx_t reply;
        pb_remote_msg_link_status_t link_status;
        reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
        reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
        reply.p_buffer = (const uint8_t *) &link_status;
        reply.length = sizeof(link_status);
        reply.force_segmented = false;
        reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
        reply.access_token = nrf_mesh_unique_token_get();

        p_ctx->current_prov_pdu_type = PROV_PDU_TYPE_INVALID;
        link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_OPENING;
        link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
        send_reply(p_ctx, p_evt->evt.p_message, &reply);
        nrf_mesh_prov_scan_stop();
        return PB_REMOTE_SERVER_STATE_LINK_OPENING;
    }
    else
    {
        __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "[er%u] Establishing local PB-ADV link\n", status);

        m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_CLOSED;
        m_packet.packet.link_status_report.reason = NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR;
        send_reliable_msg(p_ctx,
                          PB_REMOTE_OP_LINK_STATUS_REPORT,
                          PB_REMOTE_OP_LINK_STATUS,
                          sizeof(pb_remote_msg_link_status_report_t));
        return PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE;
    }
}

static pb_remote_server_state_t pb_remote_server_event_link_open_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    pb_remote_msg_link_open_t * p_pb_remote_msg = (pb_remote_msg_link_open_t *) &p_evt->evt.p_message->p_data[0];
    access_message_tx_t reply;
    pb_remote_msg_link_status_t link_status;
    reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = (const uint8_t *) &link_status;
    reply.length = sizeof(link_status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE:
            NRF_MESH_ERROR_CHECK(access_model_reliable_cancel(p_ctx->model_handle));

            /* fall through */
        case PB_REMOTE_SERVER_STATE_IDLE:
        case PB_REMOTE_SERVER_STATE_SCANNING:
            return local_link_open(p_ctx, p_evt, p_pb_remote_msg->uuid);

        case PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT_FILTER:
            if (memcmp(&m_uuid_list[0].uuid[0], &p_pb_remote_msg->uuid[0], NRF_MESH_UUID_SIZE) == 0)
            {
                NRF_MESH_ERROR_CHECK(access_model_reliable_cancel(p_ctx->model_handle));
                return local_link_open(p_ctx, p_evt, p_pb_remote_msg->uuid);
            }
            else
            {
                link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_INVALID_UNPROV_DEVICE_ID;
                link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
                send_reply(p_ctx, p_evt->evt.p_message, &reply);
                return p_ctx->state;
            }

        case PB_REMOTE_SERVER_STATE_SCANNING_FILTER:
            if (memcmp(&m_uuid_list[0].uuid[0], &p_pb_remote_msg->uuid[0], NRF_MESH_UUID_SIZE) == 0)
            {
                return local_link_open(p_ctx, p_evt, p_pb_remote_msg->uuid);
            }
            else
            {
                link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_INVALID_UNPROV_DEVICE_ID;
                link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
                send_reply(p_ctx, p_evt->evt.p_message, &reply);
                return p_ctx->state;
            }

        case PB_REMOTE_SERVER_STATE_LINK_OPENING:
            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_OPENING;
            link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;

        case PB_REMOTE_SERVER_STATE_LINK_CLOSING:
            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN;
            link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;

        case PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL:
        {
            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN;
            link_status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;
        }

        default:
            return p_ctx->state;
    }
}

static pb_remote_server_state_t pb_remote_server_event_link_established_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_LINK_OPENING:
        {
            m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_OPENED;
            m_packet.packet.link_status_report.reason = BEARER_LINK_REASON_NOT_SUPPORTED;
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_LINK_STATUS_REPORT,
                              PB_REMOTE_OP_LINK_STATUS,
                              sizeof(pb_remote_msg_link_status_report_t));

            return PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN;
        }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Got unexpected link established in state: %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_server_state_t pb_remote_server_event_link_close_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    access_message_tx_t reply;
    pb_remote_msg_link_status_t link_status;
    pb_remote_msg_link_close_t * p_msg = (pb_remote_msg_link_close_t *) &p_evt->evt.p_message->p_data[0];
    /* TODO: This should be sanitized at input and NACKed if the input is invalid. */
    nrf_mesh_prov_link_close_reason_t close_reason =
        (p_msg->reason <= NRF_MESH_PROV_LINK_CLOSE_REASON_LAST ?
         (nrf_mesh_prov_link_close_reason_t) p_msg->reason : NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);

    reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = (const uint8_t*) &link_status;
    reply.length = sizeof(link_status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
            NRF_MESH_ASSERT(access_model_reliable_cancel(p_ctx->model_handle) == NRF_SUCCESS);
            /* fall through */
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL:
            /* fall through */
        case PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED:
            p_ctx->p_prov_bearer->p_interface->link_close(p_ctx->p_prov_bearer, close_reason);
            /* fall through */
        case PB_REMOTE_SERVER_STATE_LINK_CLOSING:
            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return PB_REMOTE_SERVER_STATE_LINK_CLOSING;

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got link close in state: %u\n", p_ctx->state);
            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_LINK_NOT_ACTIVE;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;
    }

}

static pb_remote_server_state_t pb_remote_server_event_link_closed_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
            NRF_MESH_ERROR_CHECK(access_model_reliable_cancel(p_ctx->model_handle));
            /* fall through */
        case PB_REMOTE_SERVER_STATE_LINK_OPENING:
            m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_OPEN_TIMEOUT;
            m_packet.packet.link_status_report.reason = p_evt->evt.link_close_reason;
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_LINK_STATUS_REPORT,
                              PB_REMOTE_OP_LINK_STATUS,
                              sizeof(pb_remote_msg_link_status_report_t));

            return PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE;

        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
            /* The ACK should trigger this event again. */
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Queueing event %u\n", p_evt->type);
            NRF_MESH_ASSERT(fifo_push(&m_pbr_server_alt_event_queue, p_evt) == NRF_SUCCESS);
            return PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER;

        case PB_REMOTE_SERVER_STATE_LINK_CLOSING:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL:
        case PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED:
        {
            m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_OPEN_TIMEOUT;
            if (p_evt->evt.link_close_reason == NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS)
            {
                m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_CLOSED;
            }
            else
            {
                m_packet.packet.link_status_report.status = PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_DEVICE;
            }
            m_packet.packet.link_status_report.reason = p_evt->evt.link_close_reason;
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_LINK_STATUS_REPORT,
                              PB_REMOTE_OP_LINK_STATUS,
                              sizeof(pb_remote_msg_link_status_report_t));

            return PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE;
        }
        default:
            return PB_REMOTE_SERVER_STATE_IDLE;
    }
}

static pb_remote_server_state_t pb_remote_server_event_link_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    pb_remote_msg_link_status_t * p_link_status = (pb_remote_msg_link_status_t *) &p_evt->evt.p_message->p_data[0];
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_CLOSE:
            if (p_link_status->status == PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED)
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client accepted close.\n");
                return PB_REMOTE_SERVER_STATE_IDLE;
            }
            else
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client rejected the close.\n");
                return PB_REMOTE_SERVER_STATE_IDLE;
            }

        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
            if (p_link_status->status == PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED)
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client accepted open.\n");
                return PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED;
            }
            else
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client rejected the open %u.\n", p_link_status->status);
                p_ctx->p_prov_bearer->p_interface->link_close(p_ctx->p_prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                return PB_REMOTE_SERVER_STATE_LINK_CLOSING;
            }
        default:
        {
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unexpected LINK_STATUS in state %d\n", p_ctx->state);
            return p_ctx->state;
        }
    }
}

static pb_remote_server_state_t pb_remote_server_event_unprov_uuid_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_SCANNING:
        {
            for (int i = 0; i < PB_REMOTE_SERVER_UUID_LIST_SIZE; ++i)
            {
                if (m_uuid_list[i].slot_active &&
                    memcmp(&m_uuid_list[i].uuid[0],
                           &p_evt->evt.unprov.device_uuid[0],
                           NRF_MESH_UUID_SIZE) == 0)
                {
                    /* Item already in list. */
                    return p_ctx->state;
                }
            }

            for (int i = 0; i < PB_REMOTE_SERVER_UUID_LIST_SIZE; ++i)
            {

                if (!m_uuid_list[i].slot_active)
                {
                    m_uuid_list[i].slot_active = true;

                    memcpy(&m_uuid_list[i].uuid[0],
                           &p_evt->evt.unprov.device_uuid[0],
                           NRF_MESH_UUID_SIZE);

                    memcpy(&m_packet.packet.scan_uuid_report.uuid[0],
                           &p_evt->evt.unprov.device_uuid[0],
                           NRF_MESH_UUID_SIZE);
                    m_packet.packet.scan_uuid_report.unprov_device_id = i;

                    send_reliable_msg(p_ctx,
                                      PB_REMOTE_OP_SCAN_UUID_REPORT,
                                      PB_REMOTE_OP_SCAN_REPORT_STATUS,
                                      sizeof(pb_remote_msg_scan_uuid_report_t));

                    return PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT;
                }
            }

            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "No more slots for UUIDs\n");
            access_message_tx_t reply;
            pb_remote_msg_scan_status_t scan_status = {.status = PB_REMOTE_SCAN_STATUS_CANCELED};
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t*) &scan_status;
            reply.length = sizeof(scan_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_msg(p_ctx, &reply);
            return PB_REMOTE_SERVER_STATE_IDLE;
        }
        case PB_REMOTE_SERVER_STATE_SCANNING_FILTER:
            /* Filter is saved at position '0' in the list */
            if (memcmp(&m_uuid_list[0].uuid[0],
                       &p_evt->evt.unprov.device_uuid[0],
                       NRF_MESH_UUID_SIZE) == 0)
            {
                memcpy(&m_packet.packet.scan_uuid_report.uuid[0],
                       &p_evt->evt.unprov.device_uuid[0],
                       NRF_MESH_UUID_SIZE);
                m_packet.packet.scan_uuid_report.unprov_device_id = 0;

                send_reliable_msg(p_ctx,
                                  PB_REMOTE_OP_SCAN_UUID_REPORT,
                                  PB_REMOTE_OP_SCAN_REPORT_STATUS,
                                  sizeof(pb_remote_msg_scan_uuid_report_t));

                return PB_REMOTE_SERVER_STATE_WAIT_ACK_SCAN_REPORT_FILTER;
            }
            else
            {
                return p_ctx->state;
            }

        default:
            return p_ctx->state;
    }
}

static pb_remote_server_state_t pb_remote_server_event_packet_transfer_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    access_message_tx_t reply;
    pb_remote_msg_packet_transfer_status_t packet_transfer_status;
    reply.opcode.opcode = PB_REMOTE_OP_PACKET_TRANSFER_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = (const uint8_t*) &packet_transfer_status;
    reply.length = sizeof(packet_transfer_status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_LINK_OPENING:
            return p_ctx->state;

        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LINK_OPEN:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL:
        case PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED:
        {
            pb_remote_msg_packet_transfer_t * p_packet_transfer = (pb_remote_msg_packet_transfer_t *) &p_evt->evt.p_message->p_data[0];

            if (p_packet_transfer->buffer[0] == p_ctx->current_prov_pdu_type)
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got old Provisioning PDU: %d, current: %d\n", p_packet_transfer->buffer[0], p_ctx->current_prov_pdu_type);
                return p_ctx->state;
            }
            else
            {
                uint32_t status = access_model_reliable_cancel(p_ctx->model_handle);
                NRF_MESH_ASSERT(status == NRF_SUCCESS || status == NRF_ERROR_NOT_FOUND);

                p_ctx->current_prov_pdu_type = p_packet_transfer->buffer[0];
                return relay_to_pb_adv_with_ack(p_ctx, p_evt, &reply, &packet_transfer_status);
            }
        }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got unexpected packet transfer in state %u.\n", p_ctx->state);
            packet_transfer_status.status = PB_REMOTE_PACKET_TRANSFER_STATUS_LINK_NOT_ACTIVE;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;
    }
}

static pb_remote_server_state_t pb_remote_server_event_local_ack_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_LOCAL:
        {
            m_packet.packet.packet_transfer_report.status = PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_DELIVERED;
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_PACKET_TRANSFER_REPORT,
                              PB_REMOTE_OP_PACKET_TRANSFER_STATUS,
                              sizeof(pb_remote_msg_packet_transfer_report_t));

            return PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER;
        }
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got ACK in state %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_server_state_t pb_remote_server_event_local_packet_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Local packet [ppdu%x]\n", p_evt->evt.pdu.payload[0]);
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Queueing event %u\n", p_evt->type);
            NRF_MESH_ASSERT(fifo_push(&m_pbr_server_alt_event_queue, p_evt) == NRF_SUCCESS);
            return p_ctx->state;

        case PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED:
        {
            /* TODO: We have to copy the data to make room for the opcode. Can
             * we avoid it somehow? */
            NRF_MESH_ASSERT(p_evt->evt.pdu.length <= PROV_PDU_MAX_LENGTH);

            memcpy(&m_packet.packet.packet_transfer.buffer[0],
                   &p_evt->evt.pdu.payload[0],
                   p_evt->evt.pdu.length);
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_PACKET_TRANSFER,
                              PB_REMOTE_OP_PACKET_TRANSFER_STATUS,
                              p_evt->evt.pdu.length);

            return PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER;
        }

        default:
            NRF_MESH_ASSERT(false);
            /* Should never be possible. */
            return PB_REMOTE_SERVER_STATE_IDLE;
    }
}


static pb_remote_server_state_t pb_remote_server_event_transfer_status_cb(pb_remote_server_t * p_ctx, pb_remote_server_event_t * p_evt)
{
    pb_remote_msg_packet_transfer_status_t * p_msg = (pb_remote_msg_packet_transfer_status_t *) &p_evt->evt.p_message->p_data[0];
    switch (p_ctx->state)
    {
        case PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER:
            switch (p_msg->status)
            {
                case PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED:
                case PB_REMOTE_PACKET_TRANSFER_STATUS_ACCEPTED:
                    return PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED;

                case PB_REMOTE_PACKET_TRANSFER_STATUS_CANNOT_ACCEPT_BUFFER:
                case PB_REMOTE_PACKET_TRANSFER_STATUS_LINK_NOT_ACTIVE:
                    p_ctx->p_prov_bearer->p_interface->link_close(p_ctx->p_prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                    /* Go to IDLE in all cases and ignore the LINK_CLOSED event. */
                    return PB_REMOTE_SERVER_STATE_IDLE;

                default:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Got transfer status %u.\n", p_msg->status);
                    return p_ctx->state;
            }
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got transfer status %u in state %u.\n", p_msg->status, p_ctx->state);
            return p_ctx->state;
    }
}

static void pb_remote_server_process(pb_remote_server_t * p_ctx)
{
    if (m_is_interrupting)
    {
        /* Event is queued. */
        return;
    }
    m_is_interrupting = true;

    pb_remote_server_event_t evt;
    uint32_t alt_evt_queue_len = fifo_get_len(&m_pbr_server_alt_event_queue);

    while (fifo_pop(&m_pbr_server_event_queue, &evt) == NRF_SUCCESS)
    {
        if (evt.type != PB_REMOTE_SERVER_EVENT_UNPROV_UUID)
        {
            __LOG_EVENT(evt.type);
        }
        pb_remote_server_state_t next_state = m_pb_remote_server_event_handler_callbacks[evt.type](p_ctx, &evt);
        do_state_change(p_ctx, next_state);
    }

    /* If there are events in the other queue and it was not pushed by the incoming event itself. */
    if (evt.type != PB_REMOTE_SERVER_EVENT_UNPROV_UUID &&
        alt_evt_queue_len > 0 &&
        alt_evt_queue_len == fifo_get_len(&m_pbr_server_alt_event_queue))
    {
        NRF_MESH_ASSERT(fifo_pop(&m_pbr_server_alt_event_queue, &evt) == NRF_SUCCESS);
        __LOG_EVENT(evt.type);

        pb_remote_server_state_t next_state = m_pb_remote_server_event_handler_callbacks[evt.type](p_ctx, &evt);
        do_state_change(p_ctx, next_state);
    }

    if (p_ctx->return_to_scan_enabled &&
        evt.type != PB_REMOTE_SERVER_EVENT_UNPROV_UUID &&
        evt.type != PB_REMOTE_SERVER_EVENT_TX_FAILED &&
        p_ctx->state == PB_REMOTE_SERVER_STATE_IDLE)
    {
        evt.type = PB_REMOTE_SERVER_EVENT_SCAN_START;
        /* Don't need the other stuff */
        __LOG_EVENT(evt.type);
        pb_remote_server_state_t next_state = PB_REMOTE_SERVER_STATE_SCANNING;
        memset(&m_uuid_list[0], 0, sizeof(m_uuid_list));
        NRF_MESH_ERROR_CHECK(nrf_mesh_prov_scan_start(prov_evt_handler));
        do_state_change(p_ctx, next_state);
    }

    m_is_interrupting = false;
}

static bool is_valid_message(const access_message_rx_t * p_message, pb_remote_server_event_type_t * p_evt_type)
{
    const message_length_lut_t message_length_LUT[] =
        {{PB_REMOTE_OP_LINK_CLOSE            , sizeof(pb_remote_msg_link_close_t)            , PB_REMOTE_SERVER_EVENT_LINK_CLOSE},
         {PB_REMOTE_OP_LINK_OPEN             , sizeof(pb_remote_msg_link_open_t)             , PB_REMOTE_SERVER_EVENT_LINK_OPEN},
         {PB_REMOTE_OP_SCAN_START            , 0                                             , PB_REMOTE_SERVER_EVENT_SCAN_START},
         {PB_REMOTE_OP_SCAN_START_FILTER     , sizeof(pb_remote_msg_scan_start_filter_t)     , PB_REMOTE_SERVER_EVENT_SCAN_START_FILTER},
         {PB_REMOTE_OP_SCAN_CANCEL           , 0                                             , PB_REMOTE_SERVER_EVENT_SCAN_CANCEL},
         {PB_REMOTE_OP_SCAN_REPORT_STATUS    , sizeof(pb_remote_msg_scan_report_status_t)    , PB_REMOTE_SERVER_EVENT_SCAN_REPORT_STATUS},
         {PB_REMOTE_OP_PACKET_TRANSFER_STATUS, sizeof(pb_remote_msg_packet_transfer_status_t), PB_REMOTE_SERVER_EVENT_TRANSFER_STATUS},
         {PB_REMOTE_OP_LINK_STATUS           , sizeof(pb_remote_msg_link_status_t)           , PB_REMOTE_SERVER_EVENT_LINK_STATUS}};

    if (PB_REMOTE_OP_PACKET_TRANSFER == p_message->opcode.opcode)
    {
        *p_evt_type = PB_REMOTE_SERVER_EVENT_PACKET_TRANSFER;
        return (p_message->length > 0 &&
                p_message->length <= PROV_PDU_MAX_LENGTH);
    }


    for (uint32_t i = 0; i < sizeof(message_length_LUT) / sizeof(message_length_lut_t); ++i)
    {
        if (p_message->opcode.opcode == message_length_LUT[i].opcode)
        {
            if (message_length_LUT[i].length == p_message->length)
            {
                *p_evt_type = message_length_LUT[i].event_type;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    /* Could not find the message. */
    return false;
}

static bool is_valid_source(const pb_remote_server_t * p_ctx, const access_message_rx_t * p_message)
{
    dsm_handle_t publish_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t publish_address;
    if (access_model_publish_address_get(p_ctx->model_handle, &publish_handle) != NRF_SUCCESS||
        publish_handle == DSM_HANDLE_INVALID ||
        dsm_address_get(publish_handle, &publish_address) != NRF_SUCCESS ||
        publish_address.value != p_message->meta_data.src.value)
    {
        return false;
    }
    else
    {
        return true;
    }
}

static void nack_message(const pb_remote_server_t * p_ctx, const access_message_rx_t * p_message)
{
    access_message_tx_t reply;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;

    switch (p_message->opcode.opcode)
    {
        case PB_REMOTE_OP_PACKET_TRANSFER:
        {
            pb_remote_msg_packet_transfer_status_t status;
            status.status = PB_REMOTE_PACKET_TRANSFER_STATUS_REJECTED;
            reply.opcode.opcode = PB_REMOTE_OP_PACKET_TRANSFER_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }
        case PB_REMOTE_OP_LINK_CLOSE:
        case PB_REMOTE_OP_LINK_OPEN:
        {
            pb_remote_msg_packet_transfer_status_t status;
            status.status = PB_REMOTE_REMOTE_LINK_STATUS_REJECTED;
            reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }
        case PB_REMOTE_OP_SCAN_START:
        case PB_REMOTE_OP_SCAN_START_FILTER:
        {
            pb_remote_msg_scan_status_t status;
            status.status = PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }
        case PB_REMOTE_OP_SCAN_CANCEL:
        {
            pb_remote_msg_scan_status_t status;
            status.status = PB_REMOTE_SCAN_STATUS_CANNOT_CANCEL_SCANNING;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }
        case PB_REMOTE_OP_PACKET_TRANSFER_STATUS:
        case PB_REMOTE_OP_LINK_STATUS:
        case PB_REMOTE_OP_SCAN_REPORT_STATUS:
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void remote_server_access_rx_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    NRF_MESH_ASSERT(NULL != p_args);

    pb_remote_server_t * p_ctx = (pb_remote_server_t*) p_args;
    NRF_MESH_ASSERT(p_ctx->model_handle == handle);
    pb_remote_server_event_t evt;

    if (!is_valid_message(p_message, &evt.type))
    {
        return;
    }
    else if (!is_valid_source(p_ctx, p_message))
    {
        nack_message(p_ctx, p_message);
    }
    else
    {
        evt.evt.p_message = p_message;
        NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &evt) == NRF_SUCCESS);
        pb_remote_server_process(p_ctx);
    }
}

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED:
        {
            pb_remote_server_event_t pb_remote_evt;
            pb_remote_evt.type = PB_REMOTE_SERVER_EVENT_UNPROV_UUID;
            memcpy(&pb_remote_evt.evt.unprov, &p_evt->params.unprov, sizeof(p_evt->params.unprov));

            NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &pb_remote_evt) == NRF_SUCCESS);

            /* TODO: How to find the models that would need this? MBTLE-994
             * ANSWER: Declare a handler for each of the servers? Linked list? */
            pb_remote_server_process(mp_pb_remote_server_model);
            break;
        }

        default:
            break;
    }
}

/*******************************************************************************
 * Provisioning callback functions
 *******************************************************************************/

static void pb_adv_rx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    pb_remote_server_t * p_ctx = p_bearer->p_parent;
    if ((p_ctx->state == PB_REMOTE_SERVER_STATE_LINK_ESTABLISHED ||
         p_ctx->state == PB_REMOTE_SERVER_STATE_WAIT_ACK_TRANSFER))
    {
        pb_remote_server_event_t pb_remote_evt;

        pb_remote_evt.type = PB_REMOTE_SERVER_EVENT_LOCAL_PACKET;
        memcpy(&pb_remote_evt.evt.pdu.payload[0], p_data, length);
        pb_remote_evt.evt.pdu.length = length;

        NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &pb_remote_evt) == NRF_SUCCESS);
        pb_remote_server_process(p_ctx);
    }
}

static void pb_adv_ack_cb(prov_bearer_t * p_bearer)
{
    pb_remote_server_t * p_ctx = p_bearer->p_parent;
    pb_remote_server_event_t pb_remote_evt =
        {
            .type = PB_REMOTE_SERVER_EVENT_LOCAL_ACK,
            .evt  = {{0}}
        };
    NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &pb_remote_evt) == NRF_SUCCESS);
    pb_remote_server_process(p_ctx);
}

static void pb_adv_link_opened_cb(prov_bearer_t * p_bearer)
{
    pb_remote_server_t * p_ctx = p_bearer->p_parent;
    pb_remote_server_event_t pb_remote_evt =
        {
            .type = PB_REMOTE_SERVER_EVENT_LINK_ESTABLISHED,
            .evt  = {{0}}
        };

    NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &pb_remote_evt) == NRF_SUCCESS);

    pb_remote_server_process(p_ctx);
}

static void pb_adv_link_closed_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    pb_remote_server_t * p_ctx = p_bearer->p_parent;
    pb_remote_server_event_t pb_remote_evt =
        {
            .type = PB_REMOTE_SERVER_EVENT_LINK_CLOSED,
            .evt.link_close_reason = reason
        };
    NRF_MESH_ASSERT(fifo_push(&m_pbr_server_event_queue, &pb_remote_evt) == NRF_SUCCESS);
    pb_remote_server_process(p_ctx);
}

/*******************************************************************************
 * Public API
 *******************************************************************************/

uint32_t pb_remote_server_init(pb_remote_server_t * p_remote_server, uint16_t element_index)
{
    if (NULL == p_remote_server)
    {
        return NRF_ERROR_NULL;
    }
    else if (PB_REMOTE_SERVER_STATE_NONE != p_remote_server->state)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    /* TODO: To enable concurrency, these FIFOs must be kept within the context struct somehow or
     * another solution for retaining events must be found. */
    m_pbr_server_event_queue.elem_array = &m_pbr_server_event_queue_buffer[0];
    m_pbr_server_event_queue.elem_size  = sizeof(pb_remote_server_event_t);
    m_pbr_server_event_queue.array_len  = PB_REMOTE_SERVER_EVENT_QUEUE_SIZE;

    fifo_init(&m_pbr_server_event_queue);

    m_pbr_server_alt_event_queue.array_len  = PB_REMOTE_SERVER_ALT_EVENT_QUEUE_SIZE;
    m_pbr_server_alt_event_queue.elem_array = &m_pbr_server_alt_event_queue_buffer[0];
    m_pbr_server_alt_event_queue.elem_size  = sizeof(pb_remote_server_event_t);
    fifo_init(&m_pbr_server_alt_event_queue);

    m_is_interrupting = false;

    access_model_add_params_t init_params;
    init_params.element_index = element_index;
    init_params.model_id.model_id = PB_REMOTE_SERVER_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers)/sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_remote_server;
    init_params.publish_timeout_cb = NULL;

    uint32_t status = access_model_add(&init_params, &p_remote_server->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* TODO: Track state in another way and allocate this struct on stack when it's needed. */
    p_remote_server->reliable.model_handle = p_remote_server->model_handle;
    p_remote_server->reliable.status_cb = server_reliable_status_cb;
    p_remote_server->reliable.timeout = PB_REMOTE_SERVER_ACKED_TRANSACTION_TIMEOUT;
    p_remote_server->state = PB_REMOTE_SERVER_STATE_DISABLED;

    mp_pb_remote_server_model = NULL;
    return NRF_SUCCESS;
}

uint32_t pb_remote_server_enable(pb_remote_server_t * p_remote_server)
{
    if (NULL == p_remote_server)
    {
        return NRF_ERROR_NULL;
    }
    else if (PB_REMOTE_SERVER_STATE_DISABLED != p_remote_server->state ||
             mp_pb_remote_server_model != NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_remote_server->state      = PB_REMOTE_SERVER_STATE_IDLE;
    p_remote_server->prev_state = PB_REMOTE_SERVER_STATE_IDLE;
    mp_pb_remote_server_model = p_remote_server;
    return NRF_SUCCESS;
}

uint32_t pb_remote_server_disable(pb_remote_server_t * p_remote_server)
{
    if (NULL == p_remote_server)
    {
        return NRF_ERROR_NULL;
    }
    else if (PB_REMOTE_SERVER_STATE_IDLE != p_remote_server->state &&
             PB_REMOTE_SERVER_STATE_SCANNING != p_remote_server->state &&
             PB_REMOTE_SERVER_STATE_SCANNING_FILTER != p_remote_server->state)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    fifo_flush(&m_pbr_server_event_queue);
    fifo_flush(&m_pbr_server_alt_event_queue);

    p_remote_server->state      = PB_REMOTE_SERVER_STATE_DISABLED;
    p_remote_server->prev_state = PB_REMOTE_SERVER_STATE_DISABLED;
    mp_pb_remote_server_model = NULL;
    return NRF_SUCCESS;
}

uint32_t pb_remote_server_return_to_scan_set(pb_remote_server_t * p_server, bool state)
{
    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else
    {
        p_server->return_to_scan_enabled = state;
        return NRF_SUCCESS;
    }
}

uint32_t pb_remote_server_prov_bearer_set(pb_remote_server_t * p_server, prov_bearer_t * p_prov_bearer)
{
    static const prov_bearer_callbacks_t prov_callbacks =
        {
            pb_adv_rx_cb,
            pb_adv_ack_cb,
            pb_adv_link_opened_cb,
            pb_adv_link_closed_cb
        };

    if (p_prov_bearer == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else
    {
        p_server->p_prov_bearer = p_prov_bearer;
        p_server->p_prov_bearer->p_callbacks = &prov_callbacks;
        p_server->p_prov_bearer->p_parent = p_server;
        return NRF_SUCCESS;
    }
}
