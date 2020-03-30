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

#include "pb_remote_client.h"
#include "pb_remote.h"
#include "pb_remote_msgs.h"

#include <stdint.h>
#include <stddef.h>

#include "nrf_error.h"

#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_prov.h"
#include "prov_provisioner.h"
#include "prov_pdu.h"

#include "fifo.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"


/*****************************************************************************
 * Static asserts
 *****************************************************************************/

NRF_MESH_STATIC_ASSERT(PB_REMOTE_REPORT_STATUS_ACCEPTED                        <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REPORT_STATUS_REJECTED                        <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_STARTED                           <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_CANCELED                          <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING             <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_CANNOT_CANCEL_SCANNING            <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_ACCEPTED                          <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_SCAN_STATUS_REJECTED                          <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_OPENING                    <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN               <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_CANNOT_CLOSE               <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_LINK_NOT_ACTIVE            <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_INVALID_UNPROV_DEVICE_ID   <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED                   <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_REMOTE_LINK_STATUS_REJECTED                   <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_LINK_STATUS_REPORT_OPENED                     <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_LINK_STATUS_REPORT_OPEN_TIMEOUT               <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_LINK_STATUS_REPORT_CLOSED                     <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_DEVICE           <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_SERVER           <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_DELIVERED     <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_NOT_DELIVERED <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED        <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_STATUS_LINK_NOT_ACTIVE        <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_STATUS_CANNOT_ACCEPT_BUFFER   <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_STATUS_ACCEPTED               <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(PB_REMOTE_PACKET_TRANSFER_STATUS_REJECTED               <= UINT8_MAX);

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#ifndef PB_REMOTE_DEBUG_VERBOSE
#define PB_REMOTE_DEBUG_VERBOSE 0
#endif

#define PB_REMOTE_CLIENT_EVENT_QUEUE_SIZE (2)

typedef enum
{
    PB_REMOTE_CLIENT_EVENT_UNPROV_UUID,
    PB_REMOTE_CLIENT_EVENT_UNPROV_NUMBER,
    PB_REMOTE_CLIENT_EVENT_SCAN_START,
    PB_REMOTE_CLIENT_EVENT_SCAN_CANCEL,
    PB_REMOTE_CLIENT_EVENT_SCAN_STATUS,
    PB_REMOTE_CLIENT_EVENT_LINK_STATUS,
    PB_REMOTE_CLIENT_EVENT_LINK_STATUS_REPORT,
    PB_REMOTE_CLIENT_EVENT_LINK_CLOSE,
    PB_REMOTE_CLIENT_EVENT_PACKET_TRANSFER,
    PB_REMOTE_CLIENT_EVENT_TRANSFER_STATUS,
    PB_REMOTE_CLIENT_EVENT_TRANSFER_STATUS_REPORT,
    PB_REMOTE_CLIENT_EVENT_PROV_START,
    PB_REMOTE_CLIENT_EVENT_LOCAL_PACKET,
    PB_REMOTE_CLIENT_EVENT_TX_FAILED,
    PB_REMOTE_CLIENT_EVENT_LAST       /**< Used to set the size of the event handler list. */
} pb_remote_client_event_type_t;

typedef struct
{
    uint16_t length;
    uint8_t payload[PROV_PDU_MAX_LENGTH];
} pb_remote_event_pb_adv_pdu_t;

typedef struct
{
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    const nrf_mesh_prov_provisioning_data_t * p_prov_data;
} pb_remote_event_prov_start_t;

typedef struct
{
    pb_remote_client_event_type_t type;
    union
    {
        pb_remote_event_pb_adv_pdu_t pb_adv_pdu;
        const access_message_rx_t * p_message;
        nrf_mesh_prov_link_close_reason_t link_close_reason;
        pb_remote_event_prov_start_t prov_start;
    } evt;
} pb_remote_client_event_t;

typedef pb_remote_client_state_t (*pb_remote_client_event_handler_cb_t)(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);

/*******************************************************************************
 * Static variables
 *******************************************************************************/

static void remote_client_access_rx_cb(access_model_handle_t model_handle, const access_message_rx_t * p_message, void * p_args);

/* TODO: If possible, the each of the opcodes should be handled in their event callback directly,
 * but that would require a bigger re-work of the design. */
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{PB_REMOTE_OP_SCAN_UUID_REPORT,        ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_SCAN_STATUS,             ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_LINK_STATUS,             ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_LINK_STATUS_REPORT,      ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_PACKET_TRANSFER,         ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_PACKET_TRANSFER_REPORT,  ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb},
    {{PB_REMOTE_OP_PACKET_TRANSFER_STATUS,  ACCESS_COMPANY_ID_NONE}, remote_client_access_rx_cb}
};

/* Forward declarations */
static pb_remote_client_state_t pb_remote_client_event_unprov_uuid_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_unprov_number_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_scan_start_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_scan_cancel_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_scan_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_link_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_link_status_report_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_link_close_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_packet_transfer_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_transfer_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_transfer_status_report_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_prov_start_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_local_packet_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);
static pb_remote_client_state_t pb_remote_client_event_tx_failed_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt);

const pb_remote_client_event_handler_cb_t
m_pb_remote_client_event_handler_callbacks[PB_REMOTE_CLIENT_EVENT_LAST] =
{
    pb_remote_client_event_unprov_uuid_cb,
    pb_remote_client_event_unprov_number_cb,
    pb_remote_client_event_scan_start_cb,
    pb_remote_client_event_scan_cancel_cb,
    pb_remote_client_event_scan_status_cb,
    pb_remote_client_event_link_status_cb,
    pb_remote_client_event_link_status_report_cb,
    pb_remote_client_event_link_close_cb,
    pb_remote_client_event_packet_transfer_cb,
    pb_remote_client_event_transfer_status_cb,
    pb_remote_client_event_transfer_status_report_cb,
    pb_remote_client_event_prov_start_cb,
    pb_remote_client_event_local_packet_cb,
    pb_remote_client_event_tx_failed_cb
};

static pb_remote_client_t * mp_pb_remote_client_model;

/** Buffer for storing reliable packet. */
static union
{
    pb_remote_packet_t packet;
    uint8_t buffer[PROV_PDU_MAX_LENGTH];
} m_packet;

static pb_remote_client_event_t m_pbr_client_event_queue_buffer[PB_REMOTE_CLIENT_EVENT_QUEUE_SIZE];
static fifo_t m_pbr_client_event_queue;

static bool m_is_interrupting;

static uint32_t pb_if_tx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static void pb_if_link_close_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);
static uint32_t pb_if_link_open_cb(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us);

static void pb_remote_client_process(pb_remote_client_t * p_ctx);

/*******************************************************************************
 * Static functions
 *******************************************************************************/

static void client_reliable_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    NRF_MESH_ASSERT(NULL != p_args);
    pb_remote_client_t * p_self = p_args;

    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Got ACK for [aop: 0x%04x]\n", p_self->reliable.message.opcode.opcode);
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
        {
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unable to send message [aop: 0x%04x]\n", p_self->reliable.message.opcode.opcode);
            pb_remote_client_event_t event = {.type = PB_REMOTE_CLIENT_EVENT_TX_FAILED};
            NRF_MESH_ASSERT(fifo_push(&m_pbr_client_event_queue, &event) == NRF_SUCCESS);
            pb_remote_client_process(p_self);
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

static void status_event_push(pb_remote_client_t * p_ctx, pb_remote_event_type_t status_event_type)
{
    pb_remote_event_t event = {.type = status_event_type};
    p_ctx->event_cb(&event);
}

static inline void send_reliable_msg(pb_remote_client_t * p_ctx, pb_remote_opcode_t opcode, pb_remote_opcode_t reply_opcode, uint16_t length)
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
    NRF_MESH_ERROR_CHECK(access_model_reliable_publish(&p_ctx->reliable));
}

static inline void send_reply(const pb_remote_client_t * p_ctx, const access_message_rx_t * p_rx_msg, const access_message_tx_t * p_reply)
{
    uint32_t status = access_model_reply(p_ctx->model_handle, p_rx_msg, p_reply);
    NRF_MESH_ASSERT(NRF_SUCCESS == status || NRF_ERROR_NO_MEM == status);
}

static pb_remote_client_state_t pb_remote_client_event_tx_failed_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK:
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER:
            /* Push a link closed message if we were waiting for something else than a SCAN_STATUS. */
            p_ctx->prov_bearer.p_callbacks->closed(&p_ctx->prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
            status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_CLOSED);
            /* fall through */
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_SCAN:
            /* We couldn't send the reliable message, no point in trying to close link or whatever,
             * since the message won't get through. */
            status_event_push(p_ctx, PB_REMOTE_EVENT_TX_FAILED);
            return PB_REMOTE_CLIENT_STATE_IDLE;

        default:
            /* We should not be getting this event in any other state(s). */
            NRF_MESH_ASSERT(false);
            return PB_REMOTE_CLIENT_STATE_IDLE;
    }
}

static pb_remote_client_state_t pb_remote_client_event_unprov_uuid_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_scan_report_status_t scan_report_status;
    access_message_tx_t reply;
    reply.opcode.opcode = PB_REMOTE_OP_SCAN_REPORT_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = (const uint8_t*) &scan_report_status;
    reply.length = sizeof(scan_report_status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    pb_remote_msg_scan_uuid_report_t * p_scan_report =
        (pb_remote_msg_scan_uuid_report_t *) &p_evt->evt.p_message->p_data[0];

    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_IDLE:
        {

            scan_report_status.status = PB_REMOTE_REPORT_STATUS_ACCEPTED;
            scan_report_status.unprov_device_id = p_scan_report->unprov_device_id;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            pb_remote_event_t event;
            event.type = PB_REMOTE_EVENT_REMOTE_UUID;
            event.remote_uuid.p_uuid = p_scan_report->uuid;
            event.remote_uuid.device_id = p_scan_report->unprov_device_id;
            p_ctx->event_cb(&event);

            return p_ctx->state;
        }
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Unexpected UUID report in state %u\n", p_ctx->state);
            scan_report_status.status = PB_REMOTE_REPORT_STATUS_REJECTED;
            scan_report_status.unprov_device_id = p_scan_report->unprov_device_id;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_unprov_number_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    /** @todo Support the unprov_number feature. */
    switch (p_ctx->state)
    {
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unexpected UUID number report in state %u\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_scan_start_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_IDLE:
            send_reliable_msg(p_ctx, PB_REMOTE_OP_SCAN_START, PB_REMOTE_OP_SCAN_STATUS, 0);
            return PB_REMOTE_CLIENT_STATE_WAIT_ACK_SCAN;

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got scan start in state %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_scan_cancel_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{

    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_IDLE:
            send_reliable_msg(p_ctx, PB_REMOTE_OP_SCAN_CANCEL, PB_REMOTE_OP_SCAN_STATUS, 0);
            return PB_REMOTE_CLIENT_STATE_WAIT_ACK_SCAN;

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got scan cancel in state %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_scan_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_scan_status_t * p_msg = (pb_remote_msg_scan_status_t *) &p_evt->evt.p_message->p_data[0];
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_SCAN:
        {
            switch (p_msg->status)
            {
                case PB_REMOTE_SCAN_STATUS_STARTED:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "The Server started the scanning.\n");
                    status_event_push(p_ctx, PB_REMOTE_EVENT_SCAN_STARTED);
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_SCAN_STATUS_CANCELED:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "The Server cancelled the scanning.\n");
                    status_event_push(p_ctx, PB_REMOTE_EVENT_SCAN_CANCELED);
                    return PB_REMOTE_CLIENT_STATE_IDLE;


                case PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING:
                {
                    status_event_push(p_ctx, PB_REMOTE_EVENT_CANNOT_START_SCANNING);
                    return PB_REMOTE_CLIENT_STATE_IDLE;
                }
                case PB_REMOTE_SCAN_STATUS_CANNOT_CANCEL_SCANNING:
                {
                    status_event_push(p_ctx, PB_REMOTE_EVENT_CANNOT_CANCEL_SCANNING);
                    return PB_REMOTE_CLIENT_STATE_IDLE;
                }
                default:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unexpected scan status %u.\n", p_msg->status);
                        return PB_REMOTE_CLIENT_STATE_IDLE;
            }
        }
        default:
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_link_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_link_status_t * p_link_msg = (pb_remote_msg_link_status_t *) &p_evt->evt.p_message->p_data[0];
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_IDLE:
            switch (p_link_msg->status)
            {
                case PB_REMOTE_REMOTE_LINK_STATUS_OPENING:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Remote is opening link.\n");
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Link already open.\n");
                    return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;

                default:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Got link status %u.\n", p_link_msg->status);
                    return p_ctx->state;
            }

        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
            /* If we are in LINK_ESTABLISHED, is there any scenario where we would receive a
             * link status? */
            return p_ctx->state;

        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK:
            switch (p_link_msg->status)
            {
                case PB_REMOTE_REMOTE_LINK_STATUS_OPENING:
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN:
                    status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_ALREADY_OPEN);
                    return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;

                case PB_REMOTE_REMOTE_LINK_STATUS_CANNOT_CLOSE:
                    status_event_push(p_ctx, PB_REMOTE_EVENT_CANNOT_CLOSE_LINK);
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_REMOTE_LINK_STATUS_LINK_NOT_ACTIVE:
                    status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_NOT_ACTIVE);
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_REMOTE_LINK_STATUS_INVALID_UNPROV_DEVICE_ID:
                    status_event_push(p_ctx, PB_REMOTE_EVENT_INVALID_UNPROV_DEVICE_ID);
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED:
                    /* The server accepted the close => Wait for the link status report. */
                    return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;

                default:
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got unexpected link status %u.\n", p_link_msg->status);
                    return PB_REMOTE_CLIENT_STATE_IDLE;
            }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got link status %u.\n", p_link_msg->status);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_link_status_report_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_link_status_report_t * p_link_msg = (pb_remote_msg_link_status_report_t *) &p_evt->evt.p_message->p_data[0];

    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK:
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER:
            /* The incoming message is either reporting a PB_REMOTE_LINK_STATUS_REPORT_OPENED (1)
             * or a PB_REMOTE_LINK_STATUS_REPORT_CLOSED (2) status.
             *
             * (1) The first case would be that we missed the PB_REMOTE_OP_LINK_STATUS ACK for our
             * PB_REMOTE_OP_LINK_OPEN command and the server is notifying the client about the new
             * link status.
             * (2) The second case is that the remote local link closed for some reason and we need
             * to handle that.
             *
             * In either case, we should abort what we're doing and handle the link status change.
             */
            NRF_MESH_ASSERT(access_model_reliable_cancel(p_ctx->model_handle) == NRF_SUCCESS);
            /* fall-through */
        case PB_REMOTE_CLIENT_STATE_IDLE:
        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
        {
            pb_remote_msg_link_status_t link_status;
            access_message_tx_t reply;
            reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
            reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
            reply.p_buffer = (const uint8_t *) &link_status;
            reply.length = sizeof(link_status);
            reply.force_segmented = false;
            reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
            reply.access_token = nrf_mesh_unique_token_get();

            link_status.status = PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED;
            link_status.bearer_type = NRF_MESH_PROV_BEARER_ADV;
            send_reply(p_ctx, p_evt->evt.p_message, &reply);

            switch (p_link_msg->status)
            {
                case PB_REMOTE_LINK_STATUS_REPORT_OPEN_TIMEOUT:
                    status_event_push(p_ctx, PB_REMOTE_EVENT_CANNOT_OPEN_LINK);
                    /* fall through */
                case PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_DEVICE:
                case PB_REMOTE_LINK_STATUS_REPORT_CLOSED:
                case PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_SERVER:
                    p_ctx->prov_bearer.p_callbacks->closed(
                        &p_ctx->prov_bearer, (nrf_mesh_prov_link_close_reason_t) p_link_msg->reason);
                    status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_CLOSED);
                    return PB_REMOTE_CLIENT_STATE_IDLE;

                case PB_REMOTE_LINK_STATUS_REPORT_OPENED:
                    p_ctx->prov_bearer.p_callbacks->opened(&p_ctx->prov_bearer);
                    return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;

                default:
                    p_ctx->prov_bearer.p_callbacks->closed(
                        &p_ctx->prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                    status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_CLOSED);
                    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unexpected status %u in state %u.\n", p_link_msg->status, p_ctx->state);
                    return PB_REMOTE_CLIENT_STATE_IDLE;
            }
        }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got unexpected link status %u in state %u.\n", p_link_msg->status, p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_link_close_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
        {
            m_packet.packet.link_close.reason = p_evt->evt.link_close_reason;
            send_reliable_msg(p_ctx,
                              PB_REMOTE_OP_LINK_CLOSE,
                              PB_REMOTE_OP_LINK_STATUS,
                              sizeof(pb_remote_msg_link_close_t));
            return PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK;
        }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Unexpected LINK_CLOSE event in state %u.\n", p_ctx->state);
            return PB_REMOTE_CLIENT_STATE_IDLE;
    }
}

static pb_remote_client_state_t pb_remote_client_event_packet_transfer_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_packet_transfer_status_t transfer_status;
    access_message_tx_t reply;
    reply.opcode.opcode = PB_REMOTE_OP_PACKET_TRANSFER_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = (const uint8_t *) &transfer_status;
    reply.length = sizeof(transfer_status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER:
        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
        {
            pb_remote_packet_t * p_pb_remote_packet = (pb_remote_packet_t *) &p_evt->evt.p_message->p_data[0];

            if (p_pb_remote_packet->packet_transfer.buffer[0] < p_ctx->current_prov_pdu_type)
            {
                __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got old Provisioning PDU: %d, current: %d\n", p_pb_remote_packet->packet_transfer.buffer[0], p_ctx->current_prov_pdu_type);
                return p_ctx->state;
            }
            else
            {
                uint32_t status = access_model_reliable_cancel(p_ctx->model_handle);
                NRF_MESH_ASSERT(status == NRF_SUCCESS || status == NRF_ERROR_NOT_FOUND);

                /** @todo Check length against provisioning PDUs? */

                transfer_status.status = PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED;
                send_reply(p_ctx, p_evt->evt.p_message, &reply);
                p_ctx->prov_bearer.p_callbacks->rx(&p_ctx->prov_bearer,
                                                   &p_pb_remote_packet->packet_transfer.buffer[0],
                                                   p_evt->evt.p_message->length);
                return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;
            }
        }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got unexpected packet transfer in state %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_transfer_status_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    pb_remote_msg_packet_transfer_status_t * p_msg = (pb_remote_msg_packet_transfer_status_t *) &p_evt->evt.p_message->p_data[0];
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER:
            switch (p_msg->status)
            {
                case PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED:
                    return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;

                case PB_REMOTE_PACKET_TRANSFER_STATUS_LINK_NOT_ACTIVE:
                case PB_REMOTE_PACKET_TRANSFER_STATUS_CANNOT_ACCEPT_BUFFER:
                default:
                    p_ctx->prov_bearer.p_callbacks->closed(
                        &p_ctx->prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
                    status_event_push(p_ctx, PB_REMOTE_EVENT_LINK_CLOSED);
                    return PB_REMOTE_CLIENT_STATE_IDLE;
            }

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got transfer status %u in state %u.\n", p_msg->status, p_ctx->state);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_transfer_status_report_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Implicit ack\n");
            NRF_MESH_ASSERT(access_model_reliable_cancel(p_ctx->model_handle) == NRF_SUCCESS);
            /* fall-through */
        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
        {
            pb_remote_msg_packet_transfer_report_t * p_pkt = (pb_remote_msg_packet_transfer_report_t *) &p_evt->evt.p_message->p_data[0];
            if (p_pkt->status == PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_DELIVERED)
            {
                pb_remote_msg_packet_transfer_status_t transfer_status;
                access_message_tx_t reply;
                reply.opcode.opcode = PB_REMOTE_OP_PACKET_TRANSFER_STATUS;
                reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
                reply.p_buffer = (const uint8_t *) &transfer_status;
                reply.length = sizeof(transfer_status);
                reply.force_segmented = false;
                reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
                reply.access_token = nrf_mesh_unique_token_get();
                transfer_status.status = PB_REMOTE_PACKET_TRANSFER_STATUS_ACCEPTED;
                send_reply(p_ctx, p_evt->evt.p_message, &reply);

                p_ctx->prov_bearer.p_callbacks->ack(&p_ctx->prov_bearer);
                return PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED;
            }
            else
            {
                m_packet.packet.link_close.reason = p_evt->evt.link_close_reason;
                send_reliable_msg(p_ctx, PB_REMOTE_OP_LINK_CLOSE, PB_REMOTE_OP_LINK_STATUS, sizeof(pb_remote_msg_link_close_t));
                return PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK;
            }
        }
        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_INFO, "Got unknown message while waiting for packet ack.\n");
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_prov_start_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_IDLE:
        {
            p_ctx->current_prov_pdu_type = PROV_PDU_TYPE_INVALID;
            memcpy(m_packet.packet.link_open.uuid,
                   p_evt->evt.prov_start.uuid,
                   NRF_MESH_UUID_SIZE);
            send_reliable_msg(p_ctx, PB_REMOTE_OP_LINK_OPEN, PB_REMOTE_OP_LINK_STATUS, sizeof(pb_remote_msg_link_open_t));
            return PB_REMOTE_CLIENT_STATE_WAIT_ACK_LINK;
        }

        default:
            /* Should not be possible to get this event in any other state. */
            NRF_MESH_ASSERT(false);
            return p_ctx->state;
    }
}

static pb_remote_client_state_t pb_remote_client_event_local_packet_cb(pb_remote_client_t * p_ctx, pb_remote_client_event_t * p_evt)
{
    switch (p_ctx->state)
    {
        case PB_REMOTE_CLIENT_STATE_LINK_ESTABLISHED:
            NRF_MESH_ASSERT(p_evt->evt.pb_adv_pdu.length <= PROV_PDU_MAX_LENGTH);

            memcpy(&m_packet.packet.packet_transfer.buffer[0],
                   &p_evt->evt.pb_adv_pdu.payload[0],
                   p_evt->evt.pb_adv_pdu.length);

            send_reliable_msg(p_ctx, PB_REMOTE_OP_PACKET_TRANSFER, PB_REMOTE_OP_PACKET_TRANSFER_STATUS, p_evt->evt.pb_adv_pdu.length);

            return PB_REMOTE_CLIENT_STATE_WAIT_ACK_TRANSFER;

        default:
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_WARN, "Got packet_in in state %u.\n", p_ctx->state);
            return p_ctx->state;
    }
}

static void pb_remote_client_process(pb_remote_client_t * p_ctx)
{
    if (m_is_interrupting)
    {
        /* Event is queued. */
        return;
    }
    m_is_interrupting = true;

    pb_remote_client_event_t evt;
    while (fifo_pop(&m_pbr_client_event_queue, &evt) == NRF_SUCCESS)
    {
#if PB_REMOTE_DEBUG_VERBOSE
        __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "CEvent: \"%s\"\n", evt_str[evt.type]);
#else
        __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "CEvent: %u\n", evt.type);
#endif

        pb_remote_client_state_t next_state = m_pb_remote_client_event_handler_callbacks[evt.type](p_ctx, &evt);

        if (next_state != p_ctx->state)
        {
#if PB_REMOTE_DEBUG_VERBOSE
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client: \"%s\" -> \"%s\".\n", state_str[p_ctx->state], state_str[next_state]);
#else
            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "Client: \"%u\" -> \"%u\".\n", p_ctx->state, next_state);
#endif

            p_ctx->state = next_state;
        }
    }

    m_is_interrupting = false;
}


static bool is_valid_message(const access_message_rx_t * p_message, pb_remote_client_event_type_t * p_evt_type)
{
    const struct
    {
        uint16_t opcode;
        uint16_t length;
        pb_remote_client_event_type_t event_type;
    } message_length_LUT[7] =
          {{PB_REMOTE_OP_SCAN_UUID_REPORT,        sizeof(pb_remote_msg_scan_uuid_report_t),                 PB_REMOTE_CLIENT_EVENT_UNPROV_UUID},
           {PB_REMOTE_OP_SCAN_UUID_NUMBER_REPORT, sizeof(pb_remote_msg_scan_unprov_device_number_report_t), PB_REMOTE_CLIENT_EVENT_UNPROV_NUMBER},
           {PB_REMOTE_OP_SCAN_STATUS,             sizeof(pb_remote_msg_scan_status_t),                      PB_REMOTE_CLIENT_EVENT_SCAN_STATUS},
           {PB_REMOTE_OP_LINK_STATUS,             sizeof(pb_remote_msg_link_status_t),                      PB_REMOTE_CLIENT_EVENT_LINK_STATUS},
           {PB_REMOTE_OP_LINK_STATUS_REPORT,      sizeof(pb_remote_msg_link_status_report_t),               PB_REMOTE_CLIENT_EVENT_LINK_STATUS_REPORT},
           {PB_REMOTE_OP_PACKET_TRANSFER_REPORT,  sizeof(pb_remote_msg_packet_transfer_report_t),           PB_REMOTE_CLIENT_EVENT_TRANSFER_STATUS_REPORT},
           {PB_REMOTE_OP_PACKET_TRANSFER_STATUS,  sizeof(pb_remote_msg_packet_transfer_status_t),           PB_REMOTE_CLIENT_EVENT_TRANSFER_STATUS}};

    if (PB_REMOTE_OP_PACKET_TRANSFER == p_message->opcode.opcode)
    {
        *p_evt_type = PB_REMOTE_CLIENT_EVENT_PACKET_TRANSFER;
        return (p_message->length > 0 &&
                p_message->length <= PROV_PDU_MAX_LENGTH);
    }

    for (uint32_t i = 0; i < 7; ++i)
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

static bool is_valid_source(const pb_remote_client_t * p_ctx, const access_message_rx_t * p_message)
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

static void nack_message(const pb_remote_client_t * p_ctx, const access_message_rx_t * p_message)
{
    access_message_tx_t reply;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;

    switch (p_message->opcode.opcode)
    {
        case PB_REMOTE_OP_PACKET_TRANSFER_REPORT:
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
        case PB_REMOTE_OP_SCAN_UUID_REPORT:
        {
            pb_remote_msg_scan_status_t status;
            status.status = PB_REMOTE_SCAN_STATUS_REJECTED;
            reply.opcode.opcode = PB_REMOTE_OP_SCAN_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }
        case PB_REMOTE_OP_LINK_STATUS_REPORT:
        {
            pb_remote_msg_link_status_t status;
            status.status = PB_REMOTE_REMOTE_LINK_STATUS_REJECTED;
            status.bearer_type = PB_REMOTE_BEARER_TYPE_PB_ADV;
            reply.opcode.opcode = PB_REMOTE_OP_LINK_STATUS;
            reply.p_buffer = (const uint8_t*) &status;
            reply.length = sizeof(status);
            reply.access_token = nrf_mesh_unique_token_get();
            send_reply(p_ctx, p_message, &reply);
            break;
        }

        case PB_REMOTE_OP_SCAN_STATUS:
        case PB_REMOTE_OP_LINK_STATUS:
        case PB_REMOTE_OP_PACKET_TRANSFER_STATUS:
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void remote_client_access_rx_cb(access_model_handle_t model_handle,
                                       const access_message_rx_t * p_message,
                                       void * p_args)
{
    pb_remote_client_event_t evt;
    NRF_MESH_ASSERT(p_args != NULL);
    pb_remote_client_t * p_ctx = p_args;

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
        /* TODO: Might be dangerous this. Just call it directly? Copy all the data? */
        evt.evt.p_message = p_message;
        NRF_MESH_ASSERT(fifo_push(&m_pbr_client_event_queue, &evt) == NRF_SUCCESS);
        pb_remote_client_process(p_ctx);
    }
}


/*******************************************************************************
 * Provisioning callbacks
 *******************************************************************************/

static uint32_t pb_if_tx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    if (p_bearer == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    pb_remote_client_event_t evt;
    evt.type = PB_REMOTE_CLIENT_EVENT_LOCAL_PACKET;
    evt.evt.pb_adv_pdu.length = length;
    memcpy(&evt.evt.pb_adv_pdu.payload[0], p_data, length);

    mp_pb_remote_client_model->current_prov_pdu_type = p_data[0];

    uint32_t status = fifo_push(&m_pbr_client_event_queue, &evt);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    pb_remote_client_process(mp_pb_remote_client_model);

    return NRF_SUCCESS;
}


static void pb_if_link_close_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    pb_remote_client_t * p_client = PARENT_BY_FIELD_GET(pb_remote_client_t, prov_bearer, p_bearer);
    pb_remote_client_event_t evt =
        {
            .type = PB_REMOTE_CLIENT_EVENT_LINK_CLOSE,
            .evt.link_close_reason = close_reason
        };

    (void) fifo_push(&m_pbr_client_event_queue, &evt);
    pb_remote_client_process(p_client);
}

static uint32_t pb_if_link_open_cb(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us)
{
    NRF_MESH_ASSERT(p_bearer != NULL && p_uuid != NULL);
    pb_remote_client_t * p_client = PARENT_BY_FIELD_GET(pb_remote_client_t, prov_bearer, p_bearer);
    if (p_client->state != PB_REMOTE_CLIENT_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    pb_remote_client_event_t evt;
    evt.type = PB_REMOTE_CLIENT_EVENT_PROV_START;
    memcpy(evt.evt.prov_start.uuid, p_uuid, NRF_MESH_UUID_SIZE);

    uint32_t status = fifo_push(&m_pbr_client_event_queue, &evt);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    pb_remote_client_process(p_client);

    return NRF_SUCCESS;
}

/*******************************************************************************
 * Public API
 *******************************************************************************/

uint32_t pb_remote_client_init(pb_remote_client_t * p_client,
                               uint16_t element_index,
                               pb_remote_client_event_cb_t event_cb)
{
    if (p_client == NULL || event_cb == NULL)
    {
        return NRF_ERROR_NULL;

    }

    p_client->event_cb = event_cb;
    mp_pb_remote_client_model = p_client;

    m_pbr_client_event_queue.array_len  = PB_REMOTE_CLIENT_EVENT_QUEUE_SIZE;
    m_pbr_client_event_queue.elem_array = &m_pbr_client_event_queue_buffer[0];
    m_pbr_client_event_queue.elem_size  = sizeof(pb_remote_client_event_t);
    fifo_init(&m_pbr_client_event_queue);

    /* Setup model */
    access_model_add_params_t init_params;
    init_params.element_index = element_index;
    init_params.model_id.model_id = PB_REMOTE_CLIENT_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers)/sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = NULL;

    uint32_t status = access_model_add(&init_params, &p_client->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    p_client->reliable.model_handle = p_client->model_handle;
    p_client->reliable.status_cb = client_reliable_status_cb;
    p_client->reliable.timeout = PB_REMOTE_CLIENT_ACKED_TRANSACTION_TIMEOUT;

    p_client->state = PB_REMOTE_CLIENT_STATE_IDLE;
    m_is_interrupting = false;

    return NRF_SUCCESS;
}

uint32_t pb_remote_client_remote_scan_start(pb_remote_client_t * const p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->state != PB_REMOTE_CLIENT_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    pb_remote_client_event_t evt =
        {
            .type = PB_REMOTE_CLIENT_EVENT_SCAN_START
        };

    uint32_t status = fifo_push(&m_pbr_client_event_queue, &evt);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    pb_remote_client_process(p_client);

    return NRF_SUCCESS;
}

uint32_t pb_remote_client_remote_scan_cancel(pb_remote_client_t * const p_client)
{
    if (!p_client)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->state != PB_REMOTE_CLIENT_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    pb_remote_client_event_t evt =
        {
            .type = PB_REMOTE_CLIENT_EVENT_SCAN_CANCEL,
        };
    uint32_t status = fifo_push(&m_pbr_client_event_queue, &evt);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    pb_remote_client_process(p_client);

    return NRF_SUCCESS;
}

prov_bearer_t * pb_remote_client_bearer_interface_get(pb_remote_client_t * p_client)
{
    NRF_MESH_ASSERT(p_client != NULL);

    static const prov_bearer_interface_t interface =
    {
        .tx           = pb_if_tx_cb,
        .listen_start = NULL,
        .listen_stop  = NULL,
        .link_open    = pb_if_link_open_cb,
        .link_close   = pb_if_link_close_cb
    };

    p_client->prov_bearer.bearer_type = NRF_MESH_PROV_BEARER_MESH;
    p_client->prov_bearer.p_interface = &interface;

    return &p_client->prov_bearer;
}
