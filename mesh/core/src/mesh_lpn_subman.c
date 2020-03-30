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

#include "mesh_lpn_internal.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "nrf_mesh.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_app.h"
#include "nordic_common.h"
#include "packet_mesh.h"
#include "event.h"
#include "mesh_lpn.h"
#include "mesh_mem.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "utils.h"
#include "log.h"

#if MESH_FEATURE_LPN_ENABLED

/* Notes:
 * - The behavior of this module mimics the behavior of friend from the perspective of the higher layer
 * that is calling APIs of this module.
 * - Subscription manager can handle address removals or additions only when friendship is established.
 * - Adding a duplicate address to already added/synced address results in success.
 * - Removing a non-existent address results in success.
 */

#define LPN_SUBMAN_PDU_RETRY_COUNT                  (2)
#define LPN_SUBMAN_ADDRESS_LIST_SIZE                (DSM_NONVIRTUAL_ADDR_MAX + DSM_VIRTUAL_ADDR_MAX + 1 /* HEARTBEAT SUBSCRIPTION */)

#define TRANSACTION_NUMBER_GET(_pkt)                (packet_mesh_trs_control_friend_sublist_add_remove_transaction_number_get(_pkt))
#define TRANSACTION_NUMBER_SET(_pkt, _value)        (packet_mesh_trs_control_friend_sublist_add_remove_transaction_number_set(_pkt, _value))

#define SUBMAN_INDEX_INVALID                        (0xFFFF)

typedef enum
{
    SUBMAN_FLAG_NONE,
    SUBMAN_FLAG_ADD_PENDING,
    SUBMAN_FLAG_ADDED_LOCKED,
    SUBMAN_FLAG_REMOVE_PENDING
} entry_state_flag_t;

typedef struct
{
    uint16_t raw_addr;
    entry_state_flag_t flag;
} lpn_subman_address_t;

/* Forward declarations */
static void subman_lpn_event_handler(const nrf_mesh_evt_t * p_evt);
static void subman_friend_subscription_confirm_handle(const transport_control_packet_t * p_control_packet,
                                                      const nrf_mesh_rx_metadata_t * p_rx_metadata);

/* Allocate sufficient space for holding addresses */
static lpn_subman_address_t m_addr_list[LPN_SUBMAN_ADDRESS_LIST_SIZE];

/* Event handler */
static nrf_mesh_evt_handler_t m_lpn_subman_evt_handler = {
    .evt_cb = subman_lpn_event_handler
};

/* Opcode handler */
static const transport_control_packet_handler_t m_fsc_handler = {
    .opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_CONFIRM, .callback = subman_friend_subscription_confirm_handle
};

/* Message PDU */
static uint8_t m_current_transaction_number;
static uint8_t m_pdu_retry_cnt;
static packet_mesh_trs_control_packet_t m_transport_pdu;
static transport_control_packet_t m_transport_ctrl_pkt;

/* Module flags */
static bool m_address_sync_in_progress;
static bool m_established_received;

/***** Internal functions *****/

static inline bool entry_is_used(uint16_t index)
{
    return (m_addr_list[index].raw_addr != NRF_MESH_ADDR_UNASSIGNED);
}

static inline bool entry_is_used_not_synced(uint16_t index)
{
    return (entry_is_used(index) &&
           (m_addr_list[index].flag == SUBMAN_FLAG_ADD_PENDING ||
            m_addr_list[index].flag == SUBMAN_FLAG_REMOVE_PENDING));
}

static inline void entry_addr_set_for_add(uint16_t index, uint16_t addr)
{
    m_addr_list[index].raw_addr = addr;
    m_addr_list[index].flag = SUBMAN_FLAG_ADD_PENDING;
}

static inline void entry_clear_all(uint16_t index)
{
    m_addr_list[index].raw_addr = NRF_MESH_ADDR_UNASSIGNED;
    m_addr_list[index].flag = SUBMAN_FLAG_NONE;
}

static inline bool is_entry_for_add(uint16_t index)
{
    return (m_addr_list[index].flag == SUBMAN_FLAG_ADD_PENDING || m_addr_list[index].flag == SUBMAN_FLAG_ADDED_LOCKED);
}

static inline bool is_entry_for_rem(uint16_t index)
{
    return (m_addr_list[index].flag == SUBMAN_FLAG_REMOVE_PENDING);
}

static inline void mark_entry_for_add(uint16_t index)
{
    m_addr_list[index].flag = SUBMAN_FLAG_ADD_PENDING;
}

static inline void mark_entry_for_rem(uint16_t index)
{
    m_addr_list[index].flag = SUBMAN_FLAG_REMOVE_PENDING;
}

static inline void mark_entry_locked(uint16_t index)
{
    m_addr_list[index].flag = SUBMAN_FLAG_ADDED_LOCKED;
}

static uint16_t subman_address_exist(uint16_t address)
{
    for (int32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE; i++)
    {
        if (m_addr_list[i].raw_addr == address)
        {
            return i;
        }
    }

    return SUBMAN_INDEX_INVALID;
}

static bool subman_next_pdu_create(transport_control_opcode_t opcode)
{
    TRANSACTION_NUMBER_SET(&m_transport_pdu, m_current_transaction_number);

    uint16_t length = PACKET_MESH_TRS_CONTROL_FRIEND_SUBLIST_ADD_REMOVE_ADDRESS_LIST0_OFFSET;
    uint32_t packet_addr_index = 0;

    for (int32_t i = 0;
         (i < LPN_SUBMAN_ADDRESS_LIST_SIZE &&
          packet_addr_index < PACKET_MESH_TRS_CONTROL_FRIEND_SUBLIST_ADD_REMOVE_ADDRESS_LIST_MAX_COUNT);
         i++)
    {
        if (entry_is_used_not_synced(i) &&
            ((opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD &&  is_entry_for_add(i)) ||
             (opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE &&  is_entry_for_rem(i))))
        {
            packet_mesh_trs_control_friend_sublist_add_remove_address_list_set(&m_transport_pdu,
                                                                               packet_addr_index++,
                                                                               m_addr_list[i].raw_addr);
            length += sizeof(m_addr_list[0].raw_addr);

            if (is_entry_for_add(i))
            {
                mark_entry_locked(i);
            }
            else
            {
                entry_clear_all(i);
            }
        }
    }

   m_transport_ctrl_pkt.opcode = opcode;
   m_transport_ctrl_pkt.data_len = length;

   return (packet_addr_index > 0);
}

static void pdu_send_if_not_sending_but_established(transport_control_opcode_t opcode)
{
    if (!m_address_sync_in_progress && m_established_received)
    {
        if (subman_next_pdu_create(opcode))
        {
            m_address_sync_in_progress = true;
            mesh_lpn_subman_data_push(&m_transport_ctrl_pkt);
            m_pdu_retry_cnt = 0;
        }
    }
}

/* Event handler for processing friendship events. */
static void subman_lpn_event_handler(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            if (p_evt->params.friendship_established.role == NRF_MESH_FRIENDSHIP_ROLE_FRIEND)
            {
                return;
            }

            /* Send out first subscription add message */
            if (m_address_sync_in_progress == false && subman_next_pdu_create(TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD))
            {
                m_address_sync_in_progress = true;
                mesh_lpn_subman_data_push(&m_transport_ctrl_pkt);
                m_pdu_retry_cnt = 0;
            }
            m_established_received = true;
            break;

        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            if (p_evt->params.friendship_terminated.role == NRF_MESH_FRIENDSHIP_ROLE_FRIEND)
            {
                return;
            }

            for (int32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE; i++)
            {
                entry_clear_all(i);
            }
            mesh_lpn_subman_data_clear();
            m_current_transaction_number = 0;
            m_address_sync_in_progress = false;
            m_established_received = false;
            break;

        default:
            break;
    }
}

/* Opcode handler for the Friend Subscription Confirm message. */
static void subman_friend_subscription_confirm_handle(const transport_control_packet_t * p_control_packet,
                                                      const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    /* Stop any queued messages */
    mesh_lpn_subman_data_clear();

    if (!m_address_sync_in_progress)
    {
        (void) mesh_lpn_friendship_terminate();

        return;
    }

    if (TRANSACTION_NUMBER_GET(p_control_packet->p_data) == TRANSACTION_NUMBER_GET(&m_transport_pdu))
    {
        m_current_transaction_number++;
        m_pdu_retry_cnt = 0;
    }
    else
    {
        /* If transaction numbers mismatch, something is wrong on the friend side, as subscription
        list propagation happens linearly from LPN side. Resend previous PDU. */

        if (m_pdu_retry_cnt < LPN_SUBMAN_PDU_RETRY_COUNT)
        {
            mesh_lpn_subman_data_push(&m_transport_ctrl_pkt);
            m_pdu_retry_cnt++;
        }
        else
        {
            NRF_MESH_ASSERT_DEBUG(mesh_lpn_friendship_terminate() == NRF_SUCCESS);
        }

        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Warning: Confirm transaction number mismatch\n");

        return;
    }

    /* Always finish pending removals first, so that friend (and this module) will have space
     * for new subscriptions.
     */
    bool pdu_available = subman_next_pdu_create(TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE);
    if (!pdu_available)
    {
        pdu_available = subman_next_pdu_create(TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD);
    }

    /* Send out next Subscription Add/Remove message */
    if (pdu_available)
    {
        mesh_lpn_subman_data_push(&m_transport_ctrl_pkt);
        m_pdu_retry_cnt = 0;
    }
    else
    {
        m_address_sync_in_progress = false;
    }
}

/***** Interface functions *****/
uint32_t mesh_lpn_subman_add(uint16_t address)
{
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(address);

    if (address_type != NRF_MESH_ADDRESS_TYPE_VIRTUAL && address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (!mesh_lpn_is_in_friendship())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint16_t index = subman_address_exist(address);

    if (index < LPN_SUBMAN_ADDRESS_LIST_SIZE)
    {
        if (is_entry_for_rem(index))
        {
            mark_entry_for_add(index);
        }

        return NRF_SUCCESS;
    }

    for (int32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE; i++)
    {
        if (!entry_is_used(i))
        {
            entry_addr_set_for_add(i, address);
            pdu_send_if_not_sending_but_established(TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD);

            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NO_MEM;
}

uint32_t mesh_lpn_subman_remove(uint16_t address)
{
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(address);

    if (address_type != NRF_MESH_ADDRESS_TYPE_VIRTUAL && address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (!mesh_lpn_is_in_friendship())
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        uint16_t index = subman_address_exist(address);

        if (index == SUBMAN_INDEX_INVALID)
        {
            return NRF_SUCCESS;
        }

        mark_entry_for_rem(index);
        pdu_send_if_not_sending_but_established(TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE);
    }

    return NRF_SUCCESS;
}

void mesh_lpn_subman_init(void)
{
    m_current_transaction_number = 0;
    m_transport_ctrl_pkt.p_data = (const packet_mesh_trs_control_packet_t *) &m_transport_pdu;

    m_lpn_subman_evt_handler.evt_cb = subman_lpn_event_handler;
    nrf_mesh_evt_handler_add(&m_lpn_subman_evt_handler);

    NRF_MESH_ASSERT(transport_control_packet_consumer_add(&m_fsc_handler, 1) == NRF_SUCCESS);
}

#endif /* MESH_FEATURE_LPN_ENABLED */
