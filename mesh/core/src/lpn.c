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

#include "mesh_lpn.h"
#include "mesh_friendship_types.h"
#include "mesh_lpn_internal.h"

#include "fsm.h"
#include "fsm_assistant.h"
#include "bearer_event.h"
#include "scanner.h"
#include "mesh_opt_core.h"
#include "core_tx.h"
#include "core_tx_lpn.h"
#include "transport.h"
#include "packet_mesh.h"
#include "long_timer.h"
#include "utils.h"
#include "event.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_keygen.h"
#include "nordic_common.h"
#include "timeslot.h"

#if MESH_FEATURE_LPN_ENABLED

/**************************************************************************************************/

#define TIMER_JITTER_US (280)

#define FRIEND_REQUEST_RSSI_FACTOR_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_rssi_factor_set(PKT, VAL)
#define FRIEND_REQUEST_RX_WINDOW_FACTOR_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_receive_window_factor_set(PKT, VAL)
#define FRIEND_REQUEST_MIN_QUEUE_SIZE_LOG_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_min_queue_size_log_set(PKT, VAL)
#define FRIEND_REQUEST_RX_DELAY_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_receive_delay_set(PKT, VAL)
#define FRIEND_REQUEST_RX_DELAY_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_receive_delay_set(PKT, VAL)
#define FRIEND_REQUEST_POLL_TIMEOUT_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_poll_timeout_set(PKT, VAL)
#define FRIEND_REQUEST_PREVIOUS_ADDRESS_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_previous_address_set(PKT, VAL)
#define FRIEND_REQUEST_NUM_ELEMENTS_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_num_elements_set(PKT, VAL)
#define FRIEND_REQUEST_LPN_COUNTER_PACK(PKT, VAL) packet_mesh_trs_control_friend_request_lpn_counter_set(PKT, VAL)

#define FRIEND_POLL_FSN_PACK(PKT, VAL) packet_mesh_trs_control_friend_poll_fsn_set(PKT, VAL)

#define FRIEND_CLEAR_LPN_ADDRESS_PACK(PKT, VAL) packet_mesh_trs_control_friend_clear_lpn_address_set(PKT, VAL)
#define FRIEND_CLEAR_LPN_COUNTER_PACK(PKT, VAL) packet_mesh_trs_control_friend_clear_lpn_counter_set(PKT, VAL)

#define FRIEND_OFFER_RX_WINDOW_UNPACK(PKT) packet_mesh_trs_control_friend_offer_receive_window_get(PKT)
#define FRIEND_OFFER_QUEUE_SIZE_UNPACK(PKT) packet_mesh_trs_control_friend_offer_queue_size_get(PKT)
#define FRIEND_OFFER_SUBLIST_SIZE_UNPACK(PKT) packet_mesh_trs_control_friend_offer_subscription_list_size_get(PKT)
#define FRIEND_OFFER_RSSI_UNPACK(PKT) packet_mesh_trs_control_friend_offer_rssi_get(PKT)
#define FRIEND_OFFER_FRIEND_COUNTER_UNPACK(PKT) packet_mesh_trs_control_friend_offer_friend_counter_get(PKT)

#define FRIEND_UPDATE_KEY_REFRESH_FLAG_UNPACK(PKT) packet_mesh_trs_control_friend_update_key_refresh_flag_get(PKT)
#define FRIEND_UPDATE_IV_UPDATE_FLAG_UNPACK(PKT) packet_mesh_trs_control_friend_update_iv_update_flag_get(PKT)
#define FRIEND_UPDATE_IV_INDEX_UNPACK(PKT) packet_mesh_trs_control_friend_update_iv_index_get(PKT)
#define FRIEND_UPDATE_MD_UNPACK(PKT) packet_mesh_trs_control_friend_update_md_get(PKT)

/* ReceiveDelay already takes into account the timeslot start time,
 * ReceiveWindow timeout should also consider it:
 *                            |          Actual ReceiveDelay          |  Actual ReceiveWindow  |
 * --|------------------------|--------------|------------------------|------------------------|--> time
 *   |  TS start time (adv)   |   Sleeping   |  TS start time (scan)  |        Listening       |
 *   |                                       |
 *   \ (receive_delay_ms + TIMER_JITTER_US)  \ (receive_window_ms + TIMESLOT_SHORTEST_START_TIME_US)
 */
#define SCANNING_TIME_US(receive_window_ms) (MS_TO_US(receive_window_ms) + TIMESLOT_SHORTEST_START_TIME_US)

/*lint -e123 */
#define EVENT_LIST  E_ESTABLISH,             \
                    E_TIMEOUT,               \
                    E_FRIEND_OFFER_RX,       \
                    E_FRIEND_UPDATE_RX,      \
                    E_POLL,                  \
                    E_NETWORK_RX,            \
                    E_TERMINATE,             \
                    E_OFFER_ACCEPT,          \
                    E_INTERNAL_FAULT

#define STATE_LIST  S_IDLE,                  \
                    S_ESTABLISHMENT,         \
                    S_POLLING,               \
                    S_SLEEP

#define ACTION_LIST A_FRIEND_REQUEST_SEND,           a_friend_request_send,   \
                    A_FRIEND_POLL_SEND,              a_friend_poll_send,      \
                    A_FRIEND_CLEAR_SEND,             a_friend_clear_send,     \
                    A_SUBMAN_DATA_SEND,              a_subman_data_send,      \
                    A_FRIEND_SEARCH_FINISHED_NOTIFY, a_friend_search_finished_notify, \
                    A_FRIENDSHIP_ESTABLISHED_NOTIFY, a_friendship_established_notify, \
                    A_POLLING_TERMINATED_NOTIFY,     a_polling_terminated_notify,     \
                    A_POLLING_FINISHED_NOTIFY,       a_polling_finished_notify,       \
                    A_OFFER_RECEIVED_NOTIFY,         a_offer_received_notify,         \
                    A_POLL_SCHEDULE,                 a_poll_schedule,                 \
                    A_FAULT_NOTIFY,                  a_fault_notify

#define GUARD_LIST  G_IS_LAST_FRNDREQ,       g_is_last_frndreq,       \
                    G_IS_LAST_ATTEMPT,       g_is_last_attempt,       \
                    G_IS_NOT_ESTABLISHED,    g_is_not_established,    \
                    G_IS_NO_MORE_DATA,       g_is_no_more_data,       \
                    G_IS_SUBMAN_DATA,        g_is_subman_data,        \
                    G_IS_OFFER_SUITABLE,     g_is_offer_suitable

typedef enum
{
    DECLARE_ENUM(EVENT_LIST)
} lpn_event_ids_t;

typedef enum
{
    DECLARE_ENUM(STATE_LIST)
} lpn_state_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(ACTION_LIST)
} lpn_action_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(GUARD_LIST)
} lpn_guard_ids_t;

typedef union
{
    struct
    {
        const transport_control_packet_t * p_control_packet;
        const nrf_mesh_rx_metadata_t * p_rx_metadata;
    } incoming_cmd;
    struct
    {
        uint32_t delay_ms;
    } poll_scheduler;
} fsm_transac_data_t;

typedef void (* lpn_internal_task_t)(void);

typedef struct
{
    bool                is_in_friendship;
    lpn_internal_task_t task;
    bearer_event_flag_t lpn_be_flag;
    uint32_t            poll_interval_ms;
    uint32_t            poll_timeout_ms;
    uint32_t            receive_delay_ms;
    uint32_t            request_timeout_ms;
    uint32_t            receive_window_ms;
    uint32_t            delay_ms;
    friend_criteria_t   criteria;
    uint8_t             subscription_list_size;
    uint16_t            lpn_counter;
    uint16_t            friend_address;
    uint8_t             fsn;
    uint8_t             poll_attempts_count;
    uint8_t             frndreq_attempts_count;
    long_timer_t        timeout_scheduler;
    transport_control_packet_t * p_subman_data;
} lpn_t;

typedef void (* lpn_fsm_action_t)(void *);
typedef bool (* lpn_fsm_guard_t)(void *);

/* Not having a semi-colon is intentional. */
DECLARE_ACTION_PROTOTYPE(ACTION_LIST)
DECLARE_GUARD_PROTOTYPE(GUARD_LIST)

static void lpn_fsm_action(fsm_action_id_t action_id, void * p_data);
static bool lpn_fsm_guard(fsm_action_id_t action_id, void * p_data);
static void lpn_fault_task_post(void);
static void friend_request_receive_delay_handle(void * p_context);
static void friend_request_receive_window_handle(void * p_context);
static void receive_delay_handle(void * p_context);
static void timeout_handle(void * p_context);
static void friend_offer_handle(const transport_control_packet_t * p_control_packet,
                                const nrf_mesh_rx_metadata_t * p_rx_metadata);
static void friend_update_handle(const transport_control_packet_t * p_control_packet,
                                 const nrf_mesh_rx_metadata_t * p_rx_metadata);
static void termination_evt_prepare(nrf_mesh_evt_t * p_evt,
                                    nrf_mesh_evt_friendship_terminated_reason_t reason);
static void transmit_and_reschedule(const transport_control_packet_t * p_packet,
                                    nrf_mesh_tx_token_t token,
                                    uint32_t receive_delay_ms,
                                    uint32_t receive_window_ms);
static void event_send(const nrf_mesh_evt_t * p_evt);

static const fsm_transition_t m_lpn_fsm_transition_table[] =
{
    FSM_STATE(S_IDLE),
    FSM_TRANSITION(E_ESTABLISH,        FSM_ALWAYS,           A_FRIEND_REQUEST_SEND,     S_ESTABLISHMENT),

    FSM_STATE(S_ESTABLISHMENT),
    FSM_TRANSITION(E_TIMEOUT,          G_IS_LAST_FRNDREQ,    A_FRIEND_SEARCH_FINISHED_NOTIFY, S_IDLE),
    FSM_TRANSITION(E_TIMEOUT,          FSM_OTHERWISE,        A_FRIEND_REQUEST_SEND,     FSM_SAME_STATE),
    FSM_TRANSITION(E_FRIEND_OFFER_RX,  G_IS_OFFER_SUITABLE,  A_OFFER_RECEIVED_NOTIFY,   FSM_SAME_STATE),
    FSM_TRANSITION(E_OFFER_ACCEPT,     FSM_ALWAYS,           A_FRIEND_POLL_SEND,        S_POLLING),

    FSM_STATE(S_POLLING),
    FSM_TRANSITION(E_TIMEOUT,          G_IS_LAST_ATTEMPT,    A_POLLING_TERMINATED_NOTIFY, S_IDLE),
    FSM_TRANSITION(E_TIMEOUT,          FSM_OTHERWISE,        A_POLL_SCHEDULE,           S_SLEEP),
    FSM_TRANSITION(E_NETWORK_RX,       FSM_ALWAYS,           A_POLL_SCHEDULE,           S_SLEEP),

    FSM_STATE(S_SLEEP),
    FSM_TRANSITION(E_TIMEOUT,          G_IS_SUBMAN_DATA,     A_SUBMAN_DATA_SEND,        S_POLLING),
    FSM_TRANSITION(E_TIMEOUT,          FSM_OTHERWISE,        A_FRIEND_POLL_SEND,        S_POLLING),
    FSM_TRANSITION(E_FRIEND_UPDATE_RX, G_IS_NOT_ESTABLISHED, A_FRIENDSHIP_ESTABLISHED_NOTIFY, FSM_SAME_STATE),
    FSM_TRANSITION(E_FRIEND_UPDATE_RX, G_IS_NO_MORE_DATA,    A_POLLING_FINISHED_NOTIFY, FSM_SAME_STATE),
    FSM_TRANSITION(E_POLL,             FSM_ALWAYS,           A_POLL_SCHEDULE,           FSM_SAME_STATE),

    FSM_STATE(FSM_ANY_STATE),
    FSM_TRANSITION(E_TERMINATE,        FSM_ALWAYS,           A_FRIEND_CLEAR_SEND,       S_IDLE),
    FSM_TRANSITION(E_INTERNAL_FAULT,   FSM_ALWAYS,           A_FAULT_NOTIFY,            S_IDLE),
};

#if FSM_DEBUG
static const char * m_action_lookup_table[] =
{
    DECLARE_STRING_PAIR(ACTION_LIST)
};

static const char * m_guard_lookup_table[] =
{
    DECLARE_STRING_PAIR(GUARD_LIST)
};

static const char * m_event_lookup_table[] =
{
    DECLARE_STRING(EVENT_LIST)
};

static const char * m_state_lookup_table[] =
{
    DECLARE_STRING(STATE_LIST)
};
#endif  /* FSM_DEBUG */

static const fsm_const_descriptor_t m_lpn_fsm_descriptor =
{
    .transition_table = m_lpn_fsm_transition_table,
    .transitions_count = ARRAY_SIZE(m_lpn_fsm_transition_table),
    .initial_state = S_IDLE,
    .guard = lpn_fsm_guard,
    .action = lpn_fsm_action,
#if FSM_DEBUG
    .fsm_name = "lpn_fsm",
    .action_lookup = m_action_lookup_table,
    .event_lookup = m_event_lookup_table,
    .guard_lookup = m_guard_lookup_table,
    .state_lookup = m_state_lookup_table
#endif  /* FSM_DEBUG */
};

/* Opcode handler */
static const transport_control_packet_handler_t m_incoming_command_handler[2] =
{
    {
        .opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_OFFER,
        .callback = friend_offer_handle
    },
    {
        .opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
        .callback = friend_update_handle
    }
};

static fsm_t m_lpn_fsm;
static lpn_t m_lpn;

static const lpn_fsm_action_t lpn_fsm_actions[] =
{
    DECLARE_HANDLER(ACTION_LIST)
};

static const lpn_fsm_guard_t lpn_fsm_guards[] =
{
    DECLARE_HANDLER(GUARD_LIST)
};

static void lpn_fsm_action(fsm_action_id_t action_id, void * p_data)
{
    lpn_fsm_actions[action_id](p_data);
}

static bool lpn_fsm_guard(fsm_guard_id_t guard_id, void * p_data)
{
    return lpn_fsm_guards[guard_id](p_data);
}
/*lint +e123 */

static void a_friend_request_send(void * p_data)
{
    (void) p_data;
    transport_control_packet_t friend_request;
    uint16_t local_address;
    uint16_t local_address_count;
    packet_mesh_trs_control_packet_t control_packet;
    const nrf_mesh_network_secmat_t * p_secmat;

    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    nrf_mesh_primary_net_secmat_get(local_address, &p_secmat);

    if (p_secmat == NULL)
    {
        lpn_fault_task_post();
        return;
    }

    friend_request.opcode             = TRANSPORT_CONTROL_OPCODE_FRIEND_REQUEST;
    friend_request.dst.type           = NRF_MESH_ADDRESS_TYPE_GROUP;
    friend_request.dst.value          = NRF_MESH_ALL_FRIENDS_ADDR;
    friend_request.dst.p_virtual_uuid = NULL;
    friend_request.ttl                = 0;
    friend_request.reliable           = false;
    friend_request.src                = local_address;
    friend_request.p_net_secmat       = p_secmat;
    friend_request.p_data             = (const packet_mesh_trs_control_packet_t *)&control_packet;
    friend_request.data_len           = PACKET_MESH_TRS_CONTROL_FRIEND_REQUEST_SIZE;
    friend_request.bearer_selector    = CORE_TX_BEARER_TYPE_LOW_POWER;

    FRIEND_REQUEST_RSSI_FACTOR_PACK(&control_packet, m_lpn.criteria.rssi_factor);
    FRIEND_REQUEST_RX_WINDOW_FACTOR_PACK(&control_packet, m_lpn.criteria.receive_window_factor);
    FRIEND_REQUEST_MIN_QUEUE_SIZE_LOG_PACK(&control_packet, m_lpn.criteria.friend_queue_size_min_log);
    FRIEND_REQUEST_RX_DELAY_PACK(&control_packet, m_lpn.receive_delay_ms);
    FRIEND_REQUEST_POLL_TIMEOUT_PACK(&control_packet,  CEIL_DIV(m_lpn.poll_timeout_ms, 100ul));
    FRIEND_REQUEST_PREVIOUS_ADDRESS_PACK(&control_packet, m_lpn.friend_address);
    FRIEND_REQUEST_NUM_ELEMENTS_PACK(&control_packet,local_address_count);
    FRIEND_REQUEST_LPN_COUNTER_PACK(&control_packet, ++m_lpn.lpn_counter);

    transmit_and_reschedule(&friend_request,
                            NRF_MESH_FRIEND_REQUEST_TOKEN,
                            MESH_LPN_FRIEND_REQUEST_TIMEOUT_MIN_MS,
                            m_lpn.request_timeout_ms);
}

static void a_friend_poll_send(void * p_data)
{
    (void) p_data;
    transport_control_packet_t friend_poll;
    uint16_t local_address;
    uint16_t local_address_count;
    packet_mesh_trs_control_packet_t control_packet;
    const nrf_mesh_network_secmat_t * p_secmat;

    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    nrf_mesh_friendship_secmat_get(local_address, &p_secmat);

    if (p_secmat == NULL)
    {
        lpn_fault_task_post();
        return;
    }

    friend_poll.opcode             = TRANSPORT_CONTROL_OPCODE_FRIEND_POLL;
    friend_poll.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    friend_poll.dst.value          = m_lpn.friend_address;
    friend_poll.dst.p_virtual_uuid = NULL;
    friend_poll.ttl                = 0;
    friend_poll.reliable           = false;
    friend_poll.src                = local_address;
    friend_poll.p_net_secmat       = p_secmat;
    friend_poll.p_data             = (const packet_mesh_trs_control_packet_t *)&control_packet;
    friend_poll.data_len           = PACKET_MESH_TRS_CONTROL_FRIEND_POLL_SIZE;
    friend_poll.bearer_selector    = CORE_TX_BEARER_TYPE_LOW_POWER;

    control_packet.pdu[0] = 0; // set padding to zero
    FRIEND_POLL_FSN_PACK(&control_packet, m_lpn.fsn);

    transmit_and_reschedule(&friend_poll, NRF_MESH_FRIEND_POLL_TOKEN, m_lpn.receive_delay_ms, m_lpn.receive_window_ms);
}

static void a_friend_clear_send(void * p_data)
{
    (void) p_data;
    transport_control_packet_t friend_clear;
    uint16_t local_address;
    uint16_t local_address_count;
    packet_mesh_trs_control_packet_t control_packet;
    const nrf_mesh_network_secmat_t * p_secmat;

    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    nrf_mesh_friendship_secmat_get(local_address, &p_secmat);

    if (p_secmat == NULL)
    {
        lpn_fault_task_post();
        return;
    }

    friend_clear.opcode             = TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR;
    friend_clear.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    friend_clear.dst.value          = m_lpn.friend_address;
    friend_clear.dst.p_virtual_uuid = NULL;
    friend_clear.ttl                = 0;
    friend_clear.reliable           = false;
    friend_clear.src                = local_address;
    friend_clear.p_net_secmat       = p_secmat;
    friend_clear.p_data             = (const packet_mesh_trs_control_packet_t *)&control_packet;
    friend_clear.data_len           = PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE;
    friend_clear.bearer_selector    = CORE_TX_BEARER_TYPE_LOW_POWER;

    FRIEND_CLEAR_LPN_ADDRESS_PACK(&control_packet, local_address);
    FRIEND_CLEAR_LPN_COUNTER_PACK(&control_packet, m_lpn.lpn_counter);

    if (NRF_SUCCESS != transport_control_tx(&friend_clear, NRF_MESH_FRIEND_CLEAR_TOKEN))
    {
        lpn_fault_task_post();
        return;
    }

#if !MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
    scanner_disable();
#endif
}

static void a_subman_data_send(void * p_data)
{
    (void) p_data;
    transport_control_packet_t * p_subman_data = m_lpn.p_subman_data;
    uint16_t local_address;
    uint16_t local_address_count;
    const nrf_mesh_network_secmat_t * p_secmat;

    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    nrf_mesh_friendship_secmat_get(local_address, &p_secmat);

    if (p_secmat == NULL)
    {
        lpn_fault_task_post();
        return;
    }

    p_subman_data->dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    p_subman_data->dst.value          = m_lpn.friend_address;
    p_subman_data->dst.p_virtual_uuid = NULL;
    p_subman_data->ttl                = 0;
    p_subman_data->reliable           = false;  // there is assumption only unsegmented data is allowed
    p_subman_data->src                = local_address;
    p_subman_data->p_net_secmat       = p_secmat;
    p_subman_data->bearer_selector    = CORE_TX_BEARER_TYPE_LOW_POWER;

    uint32_t token = p_subman_data->opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD ?
            NRF_MESH_SUBMAN_ADD_TOKEN : NRF_MESH_SUBMAN_REMOVE_TOKEN;

    transmit_and_reschedule(p_subman_data, token, m_lpn.receive_delay_ms, m_lpn.receive_window_ms);
}

static void a_friend_search_finished_notify(void * p_data)
{
    (void) p_data;
    nrf_mesh_evt_t evt;

    evt.type = NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT;
    event_send(&evt);
}

static void a_friendship_established_notify(void * p_data)
{
    nrf_mesh_evt_t evt;
    const transport_control_packet_t * p_control_packet =
            ((fsm_transac_data_t *)p_data)->incoming_cmd.p_control_packet;
    nrf_mesh_evt_friendship_established_t * p_establish = &evt.params.friendship_established;

    m_lpn.is_in_friendship = true;

    // schedule the first poll after friendship establishing
    // entry point to the regular polling
    NRF_MESH_ERROR_CHECK(mesh_lpn_friend_poll(m_lpn.poll_interval_ms));

    evt.type = NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED;
    p_establish->role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
    p_establish->friend_src = p_control_packet->src;
    p_establish->lpn_src = p_control_packet->dst.value;
    event_send(&evt);
}

static void a_polling_terminated_notify(void * p_data)
{
    (void)p_data;
    nrf_mesh_evt_t evt;

    if (m_lpn.is_in_friendship)
    {
        termination_evt_prepare(&evt, NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NO_REPLY);
    }
    else
    {
        evt.type = NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT;
    }

    event_send(&evt);
}

static void a_polling_finished_notify(void * p_data)
{
    (void) p_data;
    nrf_mesh_evt_t evt;

    evt.type = NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE;
    event_send(&evt);

    // lpn fulfilled session with friend completely
    // schedule the next session and go to sleep
    NRF_MESH_ERROR_CHECK(mesh_lpn_friend_poll(m_lpn.poll_interval_ms));
}

static void a_offer_received_notify(void * p_data)
{
    const transport_control_packet_t * p_control_packet =
            ((fsm_transac_data_t *)p_data)->incoming_cmd.p_control_packet;
    const nrf_mesh_rx_metadata_t * p_rx_metadata =
            ((fsm_transac_data_t *)p_data)->incoming_cmd.p_rx_metadata;

    nrf_mesh_evt_t evt;
    nrf_mesh_evt_lpn_friend_offer_t * p_offer = &evt.params.friend_offer;

    evt.type = NRF_MESH_EVT_LPN_FRIEND_OFFER;

    p_offer->src = p_control_packet->src;
    p_offer->p_metadata = p_rx_metadata;
    p_offer->p_net = p_control_packet->p_net_secmat;
    p_offer->offer.friend_counter = FRIEND_OFFER_FRIEND_COUNTER_UNPACK(p_control_packet->p_data);
    p_offer->offer.friend_queue_size = FRIEND_OFFER_QUEUE_SIZE_UNPACK(p_control_packet->p_data);
    p_offer->offer.measured_rssi = FRIEND_OFFER_RSSI_UNPACK(p_control_packet->p_data);
    p_offer->offer.receive_window_ms = FRIEND_OFFER_RX_WINDOW_UNPACK(p_control_packet->p_data);
    p_offer->offer.subscription_list_size = FRIEND_OFFER_SUBLIST_SIZE_UNPACK(p_control_packet->p_data);

    event_send(&evt);
}

static void a_poll_schedule(void * p_data)
{
    fsm_transac_data_t * p_trasac_data = p_data;

    lt_schedule(&m_lpn.timeout_scheduler, timeout_handle, NULL,
        p_trasac_data != NULL ? MS_TO_US((uint64_t)p_trasac_data->poll_scheduler.delay_ms) : 0ull);
}

static void a_fault_notify(void * p_data)
{
    nrf_mesh_evt_t evt;

    termination_evt_prepare(&evt, NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_INTERNAL_TX_FAILED);
    event_send(&evt);
}

static bool g_is_last_frndreq(void * p_data)
{
    (void)p_data;

    m_lpn.frndreq_attempts_count--;
    return m_lpn.frndreq_attempts_count == 0;
}

static bool g_is_last_attempt(void * p_data)
{
    (void)p_data;

    m_lpn.poll_attempts_count--;
    return m_lpn.poll_attempts_count == 0;
}

static bool g_is_not_established(void * p_data)
{
    (void)p_data;

    return !m_lpn.is_in_friendship;
}

static bool g_is_no_more_data(void * p_data)
{
    const transport_control_packet_t * p_control_packet =
            ((fsm_transac_data_t *)p_data)->incoming_cmd.p_control_packet;
    uint8_t md = FRIEND_UPDATE_MD_UNPACK(p_control_packet->p_data);

    return md == 0;
}

static bool g_is_subman_data(void * p_data)
{
    (void)p_data;

    return m_lpn.p_subman_data != NULL;
}

static bool g_is_offer_suitable(void * p_data)
{
    const transport_control_packet_t * p_control_packet =
            ((fsm_transac_data_t *)p_data)->incoming_cmd.p_control_packet;
    uint8_t rx_window = FRIEND_OFFER_RX_WINDOW_UNPACK(p_control_packet->p_data);

    return rx_window != 0;
}

static void friend_offer_handle(const transport_control_packet_t * p_control_packet,
                                const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    fsm_transac_data_t data =
    {
        .incoming_cmd =
        {
            .p_control_packet = p_control_packet,
            .p_rx_metadata = p_rx_metadata
        }
    };

    fsm_event_post(&m_lpn_fsm, E_FRIEND_OFFER_RX, &data);
}

static void friend_update_handle(const transport_control_packet_t * p_control_packet,
                                 const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    fsm_transac_data_t data =
    {
        .incoming_cmd =
        {
            .p_control_packet = p_control_packet,
            .p_rx_metadata = p_rx_metadata
        }
    };

    nrf_mesh_evt_t evt;
    nrf_mesh_evt_lpn_friend_update_t * p_update = &evt.params.friend_update;

    evt.type = NRF_MESH_EVT_LPN_FRIEND_UPDATE;
    p_update->p_secmat_net = p_control_packet->p_net_secmat;
    p_update->iv_update_active = !!FRIEND_UPDATE_IV_UPDATE_FLAG_UNPACK(p_control_packet->p_data);
    p_update->iv_index = FRIEND_UPDATE_IV_INDEX_UNPACK(p_control_packet->p_data);
    p_update->key_refresh_in_phase2 = !!FRIEND_UPDATE_KEY_REFRESH_FLAG_UNPACK(p_control_packet->p_data);
    p_update->is_friend_queue_empty = !FRIEND_UPDATE_MD_UNPACK(p_control_packet->p_data);
    event_send(&evt);

    fsm_event_post(&m_lpn_fsm, E_FRIEND_UPDATE_RX, &data);
}

static void friend_request_receive_delay_handle(void * p_context)
{
    uint32_t receive_window_ms = (uint32_t) p_context;
    uint64_t scanning_time_us = SCANNING_TIME_US((uint64_t) receive_window_ms);

    /* MshPRFv1.0.1, section 3.6.6.4.1:
     * "If no acceptable Friend Offer message is received, the node may send a new Friend Request message.
     * The time interval between two consecutive Friend Request messages shall be greater than 1.1 seconds."
     * This is a remaining time until next Friend Request is allowed to send.
     */
    NRF_MESH_ASSERT(receive_window_ms <= MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS);
    uint32_t remaining_time_ms = MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS - receive_window_ms;

    scanner_enable();
    lt_schedule(&m_lpn.timeout_scheduler,
                friend_request_receive_window_handle,
                (void *) remaining_time_ms,
                scanning_time_us);
}

static void friend_request_receive_window_handle(void * p_context)
{
    uint32_t remaining_time_ms = (uint32_t) p_context;

    scanner_disable();
    if (remaining_time_ms > 0)
    {
        lt_schedule(&m_lpn.timeout_scheduler, timeout_handle, NULL, MS_TO_US((uint64_t) remaining_time_ms));
    }
    else
    {
        fsm_event_post(&m_lpn_fsm, E_TIMEOUT, NULL);
    }
}

static void receive_delay_handle(void * p_context)
{
    uint32_t receive_window_ms = (uint32_t) p_context;

    scanner_enable();
    lt_schedule(&m_lpn.timeout_scheduler, timeout_handle, NULL, SCANNING_TIME_US((uint64_t) receive_window_ms));
}

static void timeout_handle(void * p_context)
{
    (void)p_context;

    scanner_disable();
    fsm_event_post(&m_lpn_fsm, E_TIMEOUT, NULL);
}

static void termination_evt_prepare(nrf_mesh_evt_t * p_evt,
                                    nrf_mesh_evt_friendship_terminated_reason_t reason)
{
    nrf_mesh_evt_friendship_terminated_t * p_terminate = &p_evt->params.friendship_terminated;
    uint16_t local_address;
    uint16_t local_address_count;

    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    m_lpn.is_in_friendship = false;

    p_evt->type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
    p_terminate->role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
    p_terminate->friend_src = m_lpn.friend_address;
    p_terminate->lpn_src = local_address;
    p_terminate->reason = reason;
}

static void transmit_and_reschedule(const transport_control_packet_t * p_packet,
                                    nrf_mesh_tx_token_t token,
                                    uint32_t receive_delay_ms,
                                    uint32_t receive_window_ms)
{
    uint32_t err_code = transport_control_tx(p_packet, token);

    if (err_code == NRF_SUCCESS)
    {
        if (token == NRF_MESH_FRIEND_REQUEST_TOKEN)
        {
            lt_schedule(&m_lpn.timeout_scheduler,
                        friend_request_receive_delay_handle,
                        (void*) receive_window_ms,
                        MS_TO_US((uint64_t)receive_delay_ms) + TIMER_JITTER_US);
        }
        else
        {
            lt_schedule(&m_lpn.timeout_scheduler,
                        receive_delay_handle,
                        (void*) receive_window_ms,
                        MS_TO_US((uint64_t)receive_delay_ms) + TIMER_JITTER_US);
        }
    }
    else
    {
        __LOG(LOG_SRC_FSM, LOG_LEVEL_DBG1, "LPN transport TX failed (%u)\n", err_code);
        lt_schedule(&m_lpn.timeout_scheduler,
                    timeout_handle,
                    NULL,
                    MS_TO_US((uint64_t)receive_delay_ms + receive_window_ms));
    }
}

static void event_send(const nrf_mesh_evt_t * p_evt)
{
#if MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            scanner_enable();
            break;
        default:
            break;
    }
#endif

    event_handle(p_evt);
}

static void lpn_establish_task(void)
{
    fsm_event_post(&m_lpn_fsm, E_ESTABLISH, NULL);
}

static void lpn_offer_accept_task(void)
{
    fsm_event_post(&m_lpn_fsm, E_OFFER_ACCEPT, NULL);
}

static void lpn_terminate_task(void)
{
    fsm_event_post(&m_lpn_fsm, E_TERMINATE, NULL);

    lt_abort(&m_lpn.timeout_scheduler);

    nrf_mesh_evt_t evt;
    termination_evt_prepare(&evt, NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_LPN);
    event_send(&evt);
}

static void lpn_poll_task(void)
{
    fsm_transac_data_t data =
    {
        .poll_scheduler.delay_ms = m_lpn.delay_ms
    };

    m_lpn.poll_attempts_count = MESH_LPN_POLL_RETRY_COUNT + 1;
    fsm_event_post(&m_lpn_fsm, E_POLL, &data);
}

static void lpn_fault_task(void)
{
    fsm_event_post(&m_lpn_fsm, E_INTERNAL_FAULT, NULL);
}

static void lpn_fault_task_post(void)
{
    m_lpn.task = lpn_fault_task;
    bearer_event_flag_set(m_lpn.lpn_be_flag);
}

static bool lpn_task_process(void)
{
    NRF_MESH_ASSERT(m_lpn.task != NULL);

    m_lpn.task();

    return true;
}

static bool is_friendship_secmat(const network_packet_metadata_t * p_net_metadata)
{
    const nrf_mesh_network_secmat_t * p_frnd_secmat = NULL;
    uint16_t local_address;
    uint16_t local_address_count;
    nrf_mesh_unicast_address_get(&local_address, &local_address_count);
    nrf_mesh_friendship_secmat_get(local_address, &p_frnd_secmat);

    return (p_frnd_secmat != NULL &&
            (intptr_t) p_frnd_secmat == (intptr_t) p_net_metadata->p_security_material);
}

/******************************* API *****************************************/
void mesh_lpn_init(void)
{
    mesh_config_entry_id_t id =
    {
        .file = MESH_OPT_CORE_FILE_ID,
        .record = MESH_OPT_CORE_TX_POWER_RECORD_START + CORE_TX_ROLE_ORIGINATOR
    };
    radio_tx_power_t tx_power;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_get(id, (void *)(&tx_power)));
    core_tx_lpn_init(tx_power);

    fsm_init(&m_lpn_fsm, &m_lpn_fsm_descriptor);

    NRF_MESH_ERROR_CHECK(transport_control_packet_consumer_add(m_incoming_command_handler,
                                                               ARRAY_SIZE(m_incoming_command_handler)));
    mesh_lpn_subman_init();
    m_lpn.friend_address = NRF_MESH_ADDR_UNASSIGNED;
    m_lpn.lpn_be_flag = bearer_event_flag_add(lpn_task_process);
}

uint32_t mesh_lpn_friend_request(mesh_lpn_friend_request_t friend_params,
                                 uint32_t request_timeout_ms)
{
    if (friend_params.friend_criteria.friend_queue_size_min_log == MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_PROHIBITED)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (friend_params.receive_delay_ms < MESH_LPN_RECEIVE_DELAY_MIN_MS ||
        friend_params.receive_delay_ms > MESH_LPN_RECEIVE_DELAY_MAX_MS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (friend_params.poll_timeout_ms > MESH_LPN_POLL_TIMEOUT_MAX_MS ||
        friend_params.poll_timeout_ms < MESH_LPN_POLL_TIMEOUT_MIN_MS ||
        request_timeout_ms > MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS ||
        request_timeout_ms < MESH_LPN_FRIEND_REQUEST_TIMEOUT_MIN_MS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (friend_params.poll_timeout_ms <
            (friend_params.receive_delay_ms + MESH_LPN_RECEIVE_DELAY_MAX_MS) * (MESH_LPN_POLL_RETRY_COUNT + 1))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (mesh_lpn_is_in_friendship() || m_lpn_fsm.current_state != S_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_lpn.criteria.friend_queue_size_min_log = friend_params.friend_criteria.friend_queue_size_min_log;
    m_lpn.criteria.receive_window_factor = friend_params.friend_criteria.receive_window_factor;
    m_lpn.criteria.rssi_factor = friend_params.friend_criteria.rssi_factor;
    m_lpn.receive_delay_ms = friend_params.receive_delay_ms;
    m_lpn.poll_timeout_ms = friend_params.poll_timeout_ms;
    m_lpn.request_timeout_ms = request_timeout_ms;
    m_lpn.frndreq_attempts_count = MESH_LPN_FRIEND_REQUEST_RETRY_COUNT + 1;

    m_lpn.task = lpn_establish_task;
    bearer_event_flag_set(m_lpn.lpn_be_flag);

    return NRF_SUCCESS;
}


uint32_t mesh_lpn_friend_accept(const nrf_mesh_evt_lpn_friend_offer_t * p_friend_offer)
{
    if (p_friend_offer == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (mesh_lpn_is_in_friendship() || m_lpn_fsm.current_state != S_ESTABLISHMENT)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    scanner_disable();

    uint16_t local_address;
    uint16_t local_address_count;
    nrf_mesh_unicast_address_get(&local_address, &local_address_count);

    nrf_mesh_keygen_friendship_secmat_params_t sec_params =
    {
        .lpn_address = local_address,
        .friend_address = p_friend_offer->src,
        .lpn_counter = m_lpn.lpn_counter,
        .friend_counter = p_friend_offer->offer.friend_counter
    };
    uint32_t status = nrf_mesh_friendship_secmat_params_set(p_friend_offer->p_net, &sec_params);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    m_lpn.friend_address = p_friend_offer->src;
    m_lpn.receive_window_ms = p_friend_offer->offer.receive_window_ms;
    m_lpn.subscription_list_size = p_friend_offer->offer.subscription_list_size;
    m_lpn.fsn = 0;
    m_lpn.poll_attempts_count = MESH_LPN_POLL_RETRY_COUNT + 1;

    m_lpn.poll_interval_ms = m_lpn.poll_timeout_ms -
            (m_lpn.receive_delay_ms + m_lpn.receive_window_ms) * (MESH_LPN_POLL_RETRY_COUNT + 1);

    m_lpn.task = lpn_offer_accept_task;
    bearer_event_flag_set(m_lpn.lpn_be_flag);

    return NRF_SUCCESS;
}

uint32_t mesh_lpn_friend_poll(uint32_t delay_ms)
{
    if (!mesh_lpn_is_in_friendship())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (delay_ms > m_lpn.poll_interval_ms)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_lpn.delay_ms = delay_ms;
    m_lpn.task = lpn_poll_task;
    bearer_event_flag_set(m_lpn.lpn_be_flag);

    return NRF_SUCCESS;
}

uint32_t mesh_lpn_poll_interval_set(uint32_t poll_interval_ms)
{
    if (!mesh_lpn_is_in_friendship())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (m_lpn.poll_timeout_ms -
            (m_lpn.receive_delay_ms + m_lpn.receive_window_ms) * (MESH_LPN_POLL_RETRY_COUNT + 1) <
        poll_interval_ms)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_lpn.poll_interval_ms = poll_interval_ms;

    return NRF_SUCCESS;
}

uint32_t mesh_lpn_friendship_terminate(void)
{
    if (!mesh_lpn_is_in_friendship())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_lpn.task = lpn_terminate_task;
    bearer_event_flag_set(m_lpn.lpn_be_flag);

    return NRF_SUCCESS;
}

bool mesh_lpn_is_in_friendship(void)
{
    return m_lpn.is_in_friendship;
}

void mesh_lpn_rx_notify(const network_packet_metadata_t * p_net_metadata)
{
    if (m_lpn_fsm.current_state == S_IDLE ||
        !is_friendship_secmat(p_net_metadata))
    {
        return;
    }

    scanner_disable();

    if (m_lpn.p_subman_data == NULL)
    {
        m_lpn.fsn++;
    }

    fsm_transac_data_t data =
    {
        .poll_scheduler.delay_ms = MESH_LPN_POLL_SEPARATION_INTERVAL_MS
    };

    m_lpn.poll_attempts_count = MESH_LPN_POLL_RETRY_COUNT + 1;

    fsm_event_post(&m_lpn_fsm, E_NETWORK_RX, &data);
}

void mesh_lpn_subman_data_push(transport_control_packet_t * p_trs_ctrl_pkt)
{
    NRF_MESH_ASSERT_DEBUG(p_trs_ctrl_pkt != NULL);
    NRF_MESH_ASSERT_DEBUG(m_lpn.p_subman_data == NULL);

    m_lpn.p_subman_data = p_trs_ctrl_pkt;
    NRF_MESH_ERROR_CHECK(mesh_lpn_friend_poll(MESH_LPN_POLL_SEPARATION_INTERVAL_MS));
}

void mesh_lpn_subman_data_clear(void)
{
    m_lpn.p_subman_data = NULL;
}

#endif /* MESH_FEATURE_LPN_ENABLED */
