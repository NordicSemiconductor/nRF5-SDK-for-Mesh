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

#include "mesh_friend.h"
#include "nrf_mesh_config_core.h"

#if MESH_FEATURE_FRIEND_ENABLED
#include "nrf_error.h"
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "event.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_assert.h"
#include "mesh_opt_friend.h"
#include "mesh_config_entry.h"
#include "mesh_config_listener.h"
#include "mesh_opt_core.h"
#include "net_state.h"
#include "internal_event.h"

/* For range defines. */
#include "mesh_lpn.h"

#include "friend_queue.h"
#include "friend_sublist.h"
#include "long_timer.h"
#include "core_tx_friend.h"
#include "log.h"

NRF_MESH_STATIC_ASSERT((NRF_MESH_FRIEND_TOKEN_END - NRF_MESH_FRIEND_TOKEN_BEGIN)
                       >= MESH_FRIEND_FRIENDSHIP_COUNT);

#define FRIEND_OFFER_TIMEOUT_MS 1000
#define FRIEND_CLEAR_INITIAL_TIMEOUT_MS 1000ull
#define FRIEND_POLL_TIMEOUT_MAX_US (MS_TO_US((uint64_t)MESH_LPN_POLL_TIMEOUT_MAX_MS))
#define FRIEND_RECENT_LPNS_LIST_COUNT (MESH_FRIEND_FRIENDSHIP_COUNT + 1)

#define FACTOR_MULTIPLY(factor, value) ((factor * 0.5 + 1) * value)

#define FRIEND_QUEUE_IS_EMPTY     0
#define FRIEND_QUEUE_IS_NOT_EMPTY 1

/*****************************************************************************
 * Local typedefs
 *****************************************************************************/

typedef enum
{
    FRIEND_STATE_IDLE,          /**< Ready for new friends. */
    FRIEND_STATE_OFFERING,      /**< Friend Offer sent -- waiting for reply. */
    FRIEND_STATE_ESTABLISHED    /**< In an established friendship. */
} friend_state_t;

/**
 * Friend parameter structure.
 */
typedef struct
{
    mesh_friendship_t friendship;     /**< Friendship parameters. */
    friend_state_t state;             /**< Friend state. */
    uint8_t fsn;                      /**< Friend Sequence Number. */
    friend_queue_t queue;             /**< Friend queue.  */
    friend_sublist_t sublist;         /**< Subscription list of the LPN. */
    long_timer_t poll_timeout;        /**< Poll timeout timer. */
    long_timer_t clear_repeat_timer;  /**< Friend clear repeat timer. */
    uint8_t clear_repeat_count;       /**< Number of times to send the friend clear message. */
    core_tx_friend_t bearer;          /**< Bearer instance. */
} friendship_t;

typedef struct
{
    uint16_t last_lpn;                /**< Address of the recently seen LPN. */
    uint16_t last_req_count;          /**< Value of the LPN counter received in the friend request. */
    nrf_mesh_tx_token_t token;        /**< Bearer TX token. */
    long_timer_t confirm_send_timer;  /**< Respond to valid Friend clear message in this period. */
} recent_lpns_t;

typedef struct
{
    bool enabled;
    uint8_t default_receive_window_ms;
    uint16_t friend_counter;
    nrf_mesh_evt_handler_t mesh_evt_handler;
    friendship_t friends[MESH_FRIEND_FRIENDSHIP_COUNT];
    recent_lpns_t recent_lpns[FRIEND_RECENT_LPNS_LIST_COUNT];
#if FRIEND_TEST_HOOK
    uint16_t tx_delay_ms;
#endif
} friend_t;

/*****************************************************************************
* Static function declarations
*****************************************************************************/
static void poll_timeout_cb(void * p_context);
static void friend_clear_timeout_cb(void * p_context);
static void confirm_send_timer_cb(void * p_context);

/*****************************************************************************
* Static globals
*****************************************************************************/
static friend_t m_friend;

/*****************************************************************************
 * Mesh config wrappers
 *****************************************************************************/

static uint32_t friend_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_FRND_ID_START == entry_id.record);

    const bool enable = *((const bool *) p_entry);

    if (enable && !m_friend.enabled)
    {
        m_friend.enabled = true;
    }
    else if (!enable && m_friend.enabled)
    {
        m_friend.enabled = false;
    }

    return NRF_SUCCESS;
}

static void friend_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_FRND_ID_START == entry_id.record);

    bool * p_enabled = p_entry;
    *p_enabled = m_friend.enabled;
}

static void friend_deleter(mesh_config_entry_id_t entry_id)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_FRND_ID_START == entry_id.record);

    if (m_friend.enabled)
    {
        bool enable = false;
        (void) friend_setter(entry_id, &enable);
    }
}

MESH_CONFIG_ENTRY(mesh_opt_friend,
                  MESH_OPT_FRIEND_EID,
                  1,
                  sizeof(bool),
                  friend_setter,
                  friend_getter,
                  friend_deleter,
                  true);

/* Utility functions */

/* Find entry matching with given LPN address */
static uint32_t confirm_timer_entry_find(uint16_t lpn_addr)
{
    uint32_t i;

    for (i = 0; i < FRIEND_RECENT_LPNS_LIST_COUNT; i++)
    {
        if (m_friend.recent_lpns[i].last_lpn == lpn_addr)
        {
            break;
        }
    }

    return i;
}

/* Find empty entry, and if not found, then nearest expiring entry. */
static uint32_t confirm_timer_entry_get(void)
{
    uint64_t lowest_remaining_time_us = UINT64_MAX;
    uint32_t idx = 0;

    for (uint32_t i = 0; i < FRIEND_RECENT_LPNS_LIST_COUNT; i++)
    {
        if (m_friend.recent_lpns[i].last_lpn == NRF_MESH_ADDR_UNASSIGNED)
        {
            idx = i;
            break;
        }
        else
        {
            uint64_t current_remaining_time_us = lt_remaining_time_get(&m_friend.recent_lpns[i].confirm_send_timer);

            if (current_remaining_time_us < lowest_remaining_time_us)
            {
                idx = i;
                lowest_remaining_time_us = current_remaining_time_us;
            }
        }
    }

    return idx;
}

static void confirm_timer_add(uint16_t lpn_addr, uint32_t poll_timeout_ms, uint32_t req_count)
{
    uint32_t entry;

    /* Find entry to store LPN parameters. If entry already exist, replace existing entry,
     * otherwise, find suitable entry to be filled (either empty or nearest expiring) */
    entry = confirm_timer_entry_find(lpn_addr);
    if (entry == FRIEND_RECENT_LPNS_LIST_COUNT)
    {
        entry = confirm_timer_entry_get();
    }

    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Adding recently seen LPN: LPN 0x%04x  PT %d  entry %d\n",
          lpn_addr, poll_timeout_ms, entry);
    m_friend.recent_lpns[entry].last_lpn = lpn_addr;
    m_friend.recent_lpns[entry].last_req_count = req_count;
    lt_schedule(&m_friend.recent_lpns[entry].confirm_send_timer,
                confirm_send_timer_cb,
                &m_friend.recent_lpns[entry],
                MS_TO_US((uint64_t)poll_timeout_ms));
}

static void confirm_timer_clear(uint16_t lpn_addr)
{
    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Clearing recently seen LPN: LPN 0x%04x\n", lpn_addr);
    for(uint32_t i = 0; i < FRIEND_RECENT_LPNS_LIST_COUNT; i++)
    {
        if (m_friend.recent_lpns[i].last_lpn == lpn_addr)
        {
            __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Cleared\n");
            m_friend.recent_lpns[i].last_lpn = NRF_MESH_ADDR_UNASSIGNED;
            lt_abort(&m_friend.recent_lpns[i].confirm_send_timer);
        }
    }
}

static bool friend_address_is_known(const friendship_t * p_friendship,
                                    nrf_mesh_address_t dst)
{
    if (dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return IS_IN_RANGE(dst.value,
                           p_friendship->friendship.lpn.src,
                           (p_friendship->friendship.lpn.src +
                            p_friendship->friendship.lpn.element_count - 1));
    }
    else
    {
        return (NRF_SUCCESS ==
                friend_sublist_contains(&p_friendship->sublist, dst.value));
    }
}

static void friend_tx(friendship_t * p_friendship,
                      transport_control_opcode_t opcode,
                      const packet_mesh_trs_control_packet_t * p_pdu,
                      uint32_t length,
                      const nrf_mesh_network_secmat_t * p_net_secmat,
                      timestamp_t tx_time)
{
    uint16_t local_address;
    uint16_t local_address_count;
    nrf_mesh_unicast_address_get(&local_address, &local_address_count);

    transport_control_packet_t control_packet;
    memset(&control_packet, 0, sizeof(control_packet));
    control_packet.opcode = opcode;
    control_packet.p_data = p_pdu;
    control_packet.data_len = length;
    control_packet.src = local_address;
    control_packet.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    control_packet.dst.value = p_friendship->friendship.lpn.src;
    control_packet.p_net_secmat = p_net_secmat;
    control_packet.ttl = 0;
    control_packet.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    uint32_t status = transport_control_tx(&control_packet,
                                           p_friendship->bearer.token);

#if FRIEND_TEST_HOOK
    tx_time += MS_TO_US(m_friend.tx_delay_ms);
#endif

    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_WARN, "TX failed: status: %d\n", status);
    }

    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friend TX: opcode:%u nid:0x%02x dst:0x%04x tx_time:%u \n", control_packet.opcode, control_packet.p_net_secmat->nid, control_packet.dst.value, tx_time);
        __LOG_XB(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friend TX", (const uint8_t *) control_packet.p_data, control_packet.data_len);
        core_tx_friend_schedule(&p_friendship->bearer, tx_time);
    }
}

static void friendship_state_reset(friendship_t * p_friendship)
{
    memset(&p_friendship->friendship, 0, sizeof(mesh_friendship_t));
    p_friendship->state = FRIEND_STATE_IDLE;
    core_tx_friend_disable(&p_friendship->bearer);
    friend_queue_clear(&p_friendship->queue);
    friend_sublist_clear(&p_friendship->sublist);
    lt_abort(&p_friendship->poll_timeout);
}

static void friendship_terminate(friendship_t * p_friendship,
                                 nrf_mesh_evt_friendship_terminated_reason_t reason)
{
    NRF_MESH_ASSERT_DEBUG(p_friendship->state != FRIEND_STATE_IDLE);

    nrf_mesh_evt_t evt;
    uint16_t dummy_element_count;
    nrf_mesh_unicast_address_get(&evt.params.friendship_terminated.friend_src,
                                 &dummy_element_count);

    evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
    evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_FRIEND;
    evt.params.friendship_terminated.lpn_src = p_friendship->friendship.lpn.src;
    evt.params.friendship_terminated.reason = reason;

    confirm_timer_add(p_friendship->friendship.lpn.src, p_friendship->friendship.poll_timeout_ms,
                      p_friendship->friendship.lpn.request_count);
    friendship_state_reset(p_friendship);
    event_handle(&evt);

    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friendship terminated reason %u\n", reason);
}

static bool friendship_is_active(const friendship_t * p_friendship)
{
    return (p_friendship->state != FRIEND_STATE_IDLE);
}

static uint32_t sublist_address_count_get(const transport_control_packet_t * p_packet)
{
    /* PDU format: [ transaction_number | addr1, ... , addrN         ].
     *             | byte 0             | 1  2 | ... | size-2 size-1 |
     */
    return (p_packet->data_len - 1) / 2;
}

static timestamp_t friend_offer_delay_get(mesh_friendship_receive_window_factor_t receive_window_factor,
                                          mesh_friendship_rssi_factor_t rssi_factor,
                                          uint16_t receive_window_ms,
                                          int8_t rssi)
{
    int delay = (int)(FACTOR_MULTIPLY(receive_window_factor, receive_window_ms) -
                      FACTOR_MULTIPLY(rssi_factor, rssi));

    /* @tagMeshSp section 3.6.6.3.1 p. 79: Local Delay shall have a lower bound
     * of 100ms. */
    return (timestamp_t) (delay < 100) ? 100 : delay;
}

static bool friend_clear_is_valid_lpn_counter(uint16_t friend_request_lpn_counter,
                                              uint16_t friend_clear_lpn_counter)
{
    /* @tagMeshSp section 3.6.4.6, p 71:
     *
     * A Friend Clear message is considered valid if the result of the
     * subtraction of the value of the LPNCounter field of the Friend Request
     * message (the one that initiated the friendship) from the value of the
     * LPNCounter field of the Friend Clear message, modulo 65536, is in the
     * range 0 to 255 inclusive. */
    return ((friend_clear_lpn_counter - friend_request_lpn_counter) < 256);
}

static friendship_t * friendship_find(uint16_t lpn_address)
{
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].friendship.lpn.src == lpn_address)
        {
            return &m_friend.friends[i];
        }
    }
    return NULL;
}

static void poll_timeout_schedule(friendship_t * p_friendship, timestamp_t rx_timestamp, uint64_t poll_timeout_us)
{
    timestamp_t diff = timer_now() - rx_timestamp;
    lt_schedule(&p_friendship->poll_timeout, poll_timeout_cb, p_friendship,
                diff < poll_timeout_us ? poll_timeout_us - diff : 0);
}

static void friend_clear_timeout_schedule(friendship_t * p_friendship, uint64_t clear_repeat_timeout_us)
{
    lt_schedule(&p_friendship->clear_repeat_timer, friend_clear_timeout_cb, p_friendship, clear_repeat_timeout_us);
}

static friendship_t * free_friendship_context_get(void)
{
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state == FRIEND_STATE_IDLE)
        {
            return &m_friend.friends[i];
        }
    }
    return NULL;
}

static uint32_t friendship_alloc(friendship_t ** pp_friendship,
                                 const transport_control_packet_t * p_control_packet)
{
    uint32_t receive_delay_ms = packet_mesh_trs_control_friend_request_receive_delay_get(p_control_packet->p_data);
    uint64_t poll_timeout_ms =
        (uint64_t) packet_mesh_trs_control_friend_request_poll_timeout_get(p_control_packet->p_data) * 100;
    uint16_t prev_friend_src =
        packet_mesh_trs_control_friend_request_previous_address_get(p_control_packet->p_data);
    uint8_t min_queue_size_log =
        packet_mesh_trs_control_friend_request_min_queue_size_log_get(p_control_packet->p_data);
    uint8_t num_elements =
        packet_mesh_trs_control_friend_request_num_elements_get(p_control_packet->p_data);

    if (p_control_packet->dst.value != NRF_MESH_ALL_FRIENDS_ADDR ||
        !IS_IN_RANGE(receive_delay_ms,
                     MESH_LPN_RECEIVE_DELAY_MIN_MS,
                     MESH_LPN_RECEIVE_DELAY_MAX_MS) ||
        !IS_IN_RANGE(poll_timeout_ms,
                     MESH_LPN_POLL_TIMEOUT_MIN_MS,
                     MESH_LPN_POLL_TIMEOUT_MAX_MS) ||
        (prev_friend_src != NRF_MESH_ADDR_UNASSIGNED &&
         nrf_mesh_address_type_get(prev_friend_src) != NRF_MESH_ADDRESS_TYPE_UNICAST) ||
        min_queue_size_log == 0 ||
        num_elements == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if ((1 << min_queue_size_log) > MESH_FRIEND_QUEUE_SIZE)
    {
        return NRF_ERROR_RESOURCES;
    }

    /* TODO: More parameter sanitazion? */

    *pp_friendship = friendship_find(p_control_packet->src);
    if (*pp_friendship != NULL)
    {
        /* A new friend request from a known friend means we should consider the
         * existing friendship lost (@tagMeshSp section 3.6.6.3.1,  p. 79).*/
        friendship_terminate(*pp_friendship,
                             NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NEW_FRIEND_REQUEST);
    }
    else
    {
        *pp_friendship = free_friendship_context_get();
    }

    if (*pp_friendship == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    core_tx_friend_enable(&(*pp_friendship)->bearer);
    nrf_mesh_keygen_friendship_secmat_params_t secmat_params;
    secmat_params.lpn_address    = p_control_packet->src;
    secmat_params.lpn_counter    = packet_mesh_trs_control_friend_request_lpn_counter_get(p_control_packet->p_data);
    secmat_params.friend_counter = m_friend.friend_counter;

    uint16_t dummy_element_count;
    nrf_mesh_unicast_address_get(&secmat_params.friend_address, &dummy_element_count);

    uint32_t status = nrf_mesh_friendship_secmat_params_set(
        p_control_packet->p_net_secmat, &secmat_params);

    if (status != NRF_SUCCESS)
    {
        return status;
    }

    (*pp_friendship)->friendship.lpn.src = p_control_packet->src;
    (*pp_friendship)->friendship.lpn.prev_friend_src = prev_friend_src;
    (*pp_friendship)->friendship.lpn.element_count = num_elements;

    (*pp_friendship)->friendship.lpn.request_count =
        packet_mesh_trs_control_friend_request_lpn_counter_get(p_control_packet->p_data);

    (*pp_friendship)->friendship.poll_timeout_ms = (uint32_t) poll_timeout_ms;
    (*pp_friendship)->friendship.receive_delay_ms = receive_delay_ms;
    (*pp_friendship)->friendship.receive_window_ms = m_friend.default_receive_window_ms;

    (*pp_friendship)->state = FRIEND_STATE_OFFERING;
    return NRF_SUCCESS;
}


static void rssi_update(friendship_t * p_friendship,
                        const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    /* TODO: decide on weighted average. Do we care for
     * optimizations/configurability here at all? */
    p_friendship->friendship.avg_rssi = (p_friendship->friendship.avg_rssi +
                                         p_rx_metadata->params.scanner.rssi) / 2;
}

static void friend_update_tx(friendship_t * p_friendship,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    const nrf_mesh_network_secmat_t * p_net_secmat = NULL;
    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src,
                                   &p_net_secmat);
    NRF_MESH_ASSERT_DEBUG(p_net_secmat != NULL);
    packet_mesh_trs_control_packet_t friend_update;
    memset(&friend_update, 0, PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE);

    packet_mesh_trs_control_friend_update_key_refresh_flag_set(&friend_update,
        (nrf_mesh_key_refresh_phase_get(p_net_secmat) == NRF_MESH_KEY_REFRESH_PHASE_2));
    packet_mesh_trs_control_friend_update_iv_update_flag_set(&friend_update,
        (net_state_iv_update_get() == NET_STATE_IV_UPDATE_IN_PROGRESS));
    packet_mesh_trs_control_friend_update_iv_index_set(&friend_update, net_state_beacon_iv_index_get());
    packet_mesh_trs_control_friend_update_md_set(&friend_update,
        friend_queue_is_empty(&p_friendship->queue) ? FRIEND_QUEUE_IS_EMPTY : FRIEND_QUEUE_IS_NOT_EMPTY);

    friend_tx(p_friendship,
              TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
              &friend_update,
              PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE,
              p_net_secmat,
              (p_rx_metadata->params.scanner.timestamp +
               MS_TO_US(p_friendship->friendship.receive_delay_ms)));
}

static void friend_relay(friendship_t * p_friendship,
                         const friend_packet_t * p_packet,
                         const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    const nrf_mesh_network_secmat_t * p_friend_secmat = NULL;
    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src,
                                   &p_friend_secmat);

    network_tx_packet_buffer_t net_buf;
    network_packet_metadata_t net_metadata;

    net_metadata.dst.type = nrf_mesh_address_type_get(p_packet->net_metadata.dst);
    net_metadata.dst.value = p_packet->net_metadata.dst;
    net_metadata.src = p_packet->net_metadata.src;
    net_metadata.ttl = p_packet->net_metadata.ttl > 0 ? p_packet->net_metadata.ttl - 1 : 0;
    net_metadata.control_packet = p_packet->net_metadata.control_packet;
    net_metadata.internal.sequence_number = p_packet->net_metadata.seqnum;
    net_metadata.internal.iv_index = p_packet->net_metadata.iv_index;
    net_metadata.p_security_material = p_friend_secmat;

    net_buf.user_data.p_metadata = &net_metadata;
    net_buf.user_data.token = p_friendship->bearer.token;
    net_buf.user_data.payload_len = p_packet->length;
    net_buf.user_data.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    net_buf.user_data.role = p_packet->role;

    if (packet_mesh_trs_control_opcode_get(&p_packet->packet) == TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE)
    {
        /* The queue may have gotten additional packets since we pushed the update.
         * We always keep the ongoing packet in the queue until receive Friend Poll with changed fsn. */
        packet_mesh_trs_control_friend_update_md_set(
            (packet_mesh_trs_control_packet_t*) &p_packet->packet.pdu[PACKET_MESH_TRS_UNSEG_PDU_OFFSET],
            friend_queue_packet_counter_get(&p_friendship->queue) > 1 ? FRIEND_QUEUE_IS_NOT_EMPTY : FRIEND_QUEUE_IS_EMPTY);
    }

    uint32_t status = network_packet_alloc(&net_buf);
    if (status == NRF_SUCCESS)
    {
        memcpy(net_buf.p_payload, p_packet->packet.pdu, p_packet->length);
        network_packet_send(&net_buf);
        core_tx_friend_schedule(&p_friendship->bearer,
                                (p_rx_metadata->params.scanner.timestamp +
                                 MS_TO_US(p_friendship->friendship.receive_delay_ms)));
    }
    else
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_WARN, "Unable to allocate packet for relay %u\n", status);
    }
}

static void friend_sublist_confirm_tx(friendship_t * p_friendship,
                                      uint8_t transaction_number,
                                      const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    const nrf_mesh_network_secmat_t * p_net_secmat = NULL;
    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src,
                                   &p_net_secmat);
    NRF_MESH_ASSERT_DEBUG(p_net_secmat != NULL);
    /* No need to go through the whole packet_mesh.h scheme with only one-byte
     * payload... */
    friend_tx(p_friendship,
              TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_CONFIRM,
              (const packet_mesh_trs_control_packet_t *) &transaction_number,
              sizeof(transaction_number),
              p_net_secmat,
              (p_rx_metadata->params.scanner.timestamp +
               MS_TO_US(p_friendship->friendship.receive_delay_ms)));
}

static void friend_clear_tx(friendship_t * p_friendship)
{
    uint16_t local_address;
    uint16_t local_address_count;
    nrf_mesh_unicast_address_get(&local_address, &local_address_count);

    packet_mesh_trs_control_packet_t friend_clear;
    memset(&friend_clear, 0, sizeof(friend_clear));
    packet_mesh_trs_control_friend_clear_lpn_address_set(
        &friend_clear, p_friendship->friendship.lpn.src);
    packet_mesh_trs_control_friend_clear_lpn_counter_set(
        &friend_clear, p_friendship->friendship.lpn.request_count);

    const nrf_mesh_network_secmat_t * p_friendship_secmat = NULL;
    const nrf_mesh_network_secmat_t * p_master_secmat = NULL;

    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src,
                                   &p_friendship_secmat);
    NRF_MESH_ASSERT_DEBUG(p_friendship_secmat != NULL);

    p_master_secmat = nrf_mesh_net_master_secmat_get(p_friendship_secmat);
    NRF_MESH_ASSERT_DEBUG(p_master_secmat != NULL);

    transport_control_packet_t control_packet;
    memset(&control_packet, 0, sizeof(control_packet));
    control_packet.opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR;
    control_packet.p_data = &friend_clear;
    control_packet.data_len = PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE;
    control_packet.src = local_address;
    control_packet.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    control_packet.dst.value = p_friendship->friendship.lpn.prev_friend_src;
    control_packet.p_net_secmat = p_master_secmat;
    control_packet.ttl = NRF_MESH_TTL_MAX;
    control_packet.bearer_selector = CORE_TX_BEARER_TYPE_ADV;
    uint32_t status = transport_control_tx(&control_packet,
                                           p_friendship->bearer.token);
    NRF_MESH_ASSERT_DEBUG(status == NRF_SUCCESS ||
                          status == NRF_ERROR_FORBIDDEN ||
                          status == NRF_ERROR_NO_MEM);
}

static void friend_clear_confirm_tx(uint16_t lpn_src, uint16_t lpn_req_count,
                                    nrf_mesh_tx_token_t bearer_token,
                                    const transport_control_packet_t * p_clear_packet)
{
    packet_mesh_trs_control_packet_t friend_clear_confirm;
    memset(&friend_clear_confirm, 0, sizeof(friend_clear_confirm));
    packet_mesh_trs_control_friend_clear_confirm_lpn_address_set(
        &friend_clear_confirm, lpn_src);
    packet_mesh_trs_control_friend_clear_confirm_lpn_counter_set(
        &friend_clear_confirm, lpn_req_count);

    transport_control_packet_t control_packet;
    memset(&control_packet, 0, sizeof(control_packet));
    control_packet.opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM;
    control_packet.p_data = &friend_clear_confirm;
    control_packet.data_len = PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_CONFIRM_SIZE;
    control_packet.src = p_clear_packet->dst.value;
    control_packet.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    control_packet.dst.value = p_clear_packet->src;
    control_packet.p_net_secmat = p_clear_packet->p_net_secmat;
    control_packet.ttl = p_clear_packet->ttl == 0 ? 0 : NRF_MESH_TTL_MAX;
    control_packet.bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL;
    uint32_t status = transport_control_tx(&control_packet,
                                           bearer_token);
    NRF_MESH_ASSERT_DEBUG(status == NRF_SUCCESS ||
                          status == NRF_ERROR_FORBIDDEN);
}

static void poll_timeout_cb(void * p_context)
{
    NRF_MESH_ASSERT_DEBUG(p_context != NULL);
    friendship_t * p_friendship = p_context;

    friendship_terminate(p_friendship,
                         NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_TIMEOUT);
}

static void friend_clear_timeout_cb(void * p_context)
{
    friendship_t * p_friendship = p_context;

    p_friendship->clear_repeat_count++;
    uint64_t next_timeout_us = MS_TO_US(FRIEND_CLEAR_INITIAL_TIMEOUT_MS) << p_friendship->clear_repeat_count;

    /* Friend Clear Procedure timer shall be started with the period equal to two times the Friend Poll Timeout value */
    if (next_timeout_us <= 2 * MS_TO_US((uint64_t)p_friendship->friendship.poll_timeout_ms))
    {
        friend_clear_tx(p_friendship);
        friend_clear_timeout_schedule(p_friendship, next_timeout_us);
    }
}

static void confirm_send_timer_cb(void * p_context)
{
    recent_lpns_t * p_recent_lpns = p_context;
    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Removing last seen LPN: 0x%04x\n", p_recent_lpns->last_lpn);

    p_recent_lpns->last_lpn = NRF_MESH_ADDR_UNASSIGNED;
}

static recent_lpns_t * recent_lpns_get(const transport_control_packet_t * p_control_packet,
                                       uint16_t exp_size)
{
    if ((exp_size > 0) && (p_control_packet->data_len != exp_size))
    {
        return NULL;
    }

    uint16_t lpn_addr = packet_mesh_trs_control_friend_clear_lpn_address_get(p_control_packet->p_data);

    for (uint32_t i = 0; i < FRIEND_RECENT_LPNS_LIST_COUNT; i++)
    {
        if (m_friend.recent_lpns[i].last_lpn == lpn_addr)
        {
            return &m_friend.recent_lpns[i];
        }
    }

    return NULL;
}

static friendship_t * friendship_get(const transport_control_packet_t * p_control_packet,
                                     uint16_t exp_size)
{
    if (!m_friend.enabled || ((exp_size > 0) && (p_control_packet->data_len != exp_size)))
    {
        return NULL;
    }

    uint16_t lpn_address = p_control_packet->src;
    if (p_control_packet->opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR ||
        p_control_packet->opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM)
    {
        lpn_address = packet_mesh_trs_control_friend_clear_lpn_address_get(
            p_control_packet->p_data);
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Clearing friendship for LPN 0x%04x\n",
              lpn_address);
    }

    friendship_t * p_friendship = friendship_find(lpn_address);
    if (p_friendship == NULL)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Received opcode %d from unkown LPN 0x%04x (src 0x%04x)\n",
            p_control_packet->opcode, lpn_address, p_control_packet->src);
    }

    return p_friendship;
}

/*******************************************************************************
 * Transport opcode handler callbacks
 *******************************************************************************/

static void friend_poll_handle(const transport_control_packet_t * p_control_packet,
                               const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    friendship_t * p_friendship = friendship_get(p_control_packet,
        PACKET_MESH_TRS_CONTROL_FRIEND_POLL_SIZE);
    if (p_friendship == NULL ||
        (p_control_packet->p_data->pdu[PACKET_MESH_TRS_CONTROL_FRIEND_POLL_FSN_OFFSET] &
         PACKET_MESH_TRS_CONTROL_FRIEND_POLL_FSN_MASK_INV) != 0)
    {
        return;
    }

    uint8_t fsn = packet_mesh_trs_control_friend_poll_fsn_get(p_control_packet->p_data);
    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1,
          "Friend poll (0x%04x): fsn: %u (prev: %u)\n",
          p_friendship->friendship.lpn.src, fsn, p_friendship->fsn);

    rssi_update(p_friendship, p_rx_metadata);
    switch (p_friendship->state)
    {
        case FRIEND_STATE_OFFERING:
        {
            uint16_t unicast_address;
            uint16_t dummy_count;
            nrf_mesh_unicast_address_get(&unicast_address, &dummy_count);

            p_friendship->fsn = fsn;
            p_friendship->state = FRIEND_STATE_ESTABLISHED;
            friend_update_tx(p_friendship, p_rx_metadata);
            if (p_friendship->friendship.lpn.prev_friend_src != NRF_MESH_ADDR_UNASSIGNED &&
                p_friendship->friendship.lpn.prev_friend_src != unicast_address)
            {
                p_friendship->clear_repeat_count = 0;
                friend_clear_tx(p_friendship);
                friend_clear_timeout_schedule(p_friendship, MS_TO_US(FRIEND_CLEAR_INITIAL_TIMEOUT_MS));
            }
            __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friendship established (0x%04x)\n",
                  p_friendship->friendship.lpn.src);

            confirm_timer_clear(p_friendship->friendship.lpn.src);

            nrf_mesh_evt_t evt;
            evt.type = NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED;
            evt.params.friendship_established.role = NRF_MESH_FRIENDSHIP_ROLE_FRIEND;
            evt.params.friendship_established.lpn_src = p_friendship->friendship.lpn.src;
            evt.params.friendship_established.friend_src = unicast_address;
            event_handle(&evt);
            break;
        }
        case FRIEND_STATE_ESTABLISHED:
        {
            if (p_friendship->fsn != fsn)
            {
                friend_queue_packet_release(&p_friendship->queue);
            }
            p_friendship->fsn = fsn;

            const friend_packet_t * p_packet = friend_queue_packet_get(&p_friendship->queue);
            if (p_packet == NULL)
            {
                friend_update_tx(p_friendship, p_rx_metadata);
            }
            else
            {
                friend_relay(p_friendship, p_packet, p_rx_metadata);
            }
            break;
        }
        default:
            NRF_MESH_ASSERT_DEBUG(false);
            break;
    }

    poll_timeout_schedule(p_friendship,
                          p_rx_metadata->params.scanner.timestamp,
                          MS_TO_US((uint64_t)p_friendship->friendship.poll_timeout_ms));
}

static void friend_request_handle(const transport_control_packet_t * p_control_packet,
                                  const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    NRF_MESH_ASSERT_DEBUG(p_control_packet != NULL);
    NRF_MESH_ASSERT_DEBUG(p_rx_metadata != NULL);
    if (!m_friend.enabled ||
        p_control_packet->data_len != PACKET_MESH_TRS_CONTROL_FRIEND_REQUEST_SIZE)
    {
        return;
    }

    friendship_t * p_friendship = NULL;
    uint32_t status = friendship_alloc(&p_friendship, p_control_packet);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_WARN, "Error %u: Not able to allocate friendship\n", status);
        return;
    }
    NRF_MESH_ASSERT_DEBUG(p_friendship != NULL);
    p_friendship->friendship.avg_rssi = p_rx_metadata->params.scanner.rssi;

    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIEND_REQUEST;
    evt.params.friend_request.p_friendship = &p_friendship->friendship;
    evt.params.friend_request.p_net = p_control_packet->p_net_secmat;
    evt.params.friend_request.p_metadata = p_rx_metadata;
    event_handle(&evt);

    /* NOTE: At this point, the application may have called mesh_friend_friendship_terminate()
     * so we need to check if we're still in a friendship or not.
     */
    if (!friendship_is_active(p_friendship))
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Application cancelled friend request\n");
        NRF_MESH_ASSERT_DEBUG(p_friendship->state == FRIEND_STATE_IDLE);

        /* Ensure that the secmat has been removed from DSM. */
        const nrf_mesh_network_secmat_t * p_secmat;
        nrf_mesh_friendship_secmat_get(p_control_packet->src, &p_secmat);
        NRF_MESH_ASSERT_DEBUG(p_secmat == NULL);

        return;
    }

    packet_mesh_trs_control_packet_t friend_offer_pdu;
    memset(&friend_offer_pdu, 0, PACKET_MESH_TRS_CONTROL_FRIEND_OFFER_SIZE);
    packet_mesh_trs_control_friend_offer_receive_window_set(
        &friend_offer_pdu, p_friendship->friendship.receive_window_ms);

    packet_mesh_trs_control_friend_offer_queue_size_set(
        &friend_offer_pdu, MESH_FRIEND_QUEUE_SIZE);

    packet_mesh_trs_control_friend_offer_subscription_list_size_set(
        &friend_offer_pdu, MESH_FRIEND_SUBLIST_SIZE);

    packet_mesh_trs_control_friend_offer_rssi_set(
        &friend_offer_pdu, p_rx_metadata->params.scanner.rssi);

    packet_mesh_trs_control_friend_offer_friend_counter_set(
        &friend_offer_pdu, m_friend.friend_counter++);

    mesh_friendship_receive_window_factor_t receive_window_factor =
        (mesh_friendship_receive_window_factor_t)
        packet_mesh_trs_control_friend_request_receive_window_factor_get(p_control_packet->p_data);
    mesh_friendship_rssi_factor_t rssi_factor =
        (mesh_friendship_rssi_factor_t)
        packet_mesh_trs_control_friend_request_rssi_factor_get(p_control_packet->p_data);

    uint32_t offer_delay_ms =
        friend_offer_delay_get(receive_window_factor,
                               rssi_factor,
                               p_friendship->friendship.receive_window_ms,
                               p_rx_metadata->params.scanner.rssi);

    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friend request (0x%04x): "
          "offer delay: %u, rDelay: %u, pTimeout: %u, prevAddr: 0x%04x, elements: %u\n",
          p_friendship->friendship.lpn.src,
          offer_delay_ms,
          p_friendship->friendship.receive_delay_ms,
          p_friendship->friendship.poll_timeout_ms,
          p_friendship->friendship.lpn.prev_friend_src,
          p_friendship->friendship.lpn.element_count);
    __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friend offer: "
          "rWindow: %u, queueSize: %u, sublistSize: %u, rssi: %d, fcnt: %u\n",
          p_friendship->friendship.receive_window_ms,
          MESH_FRIEND_QUEUE_SIZE,
          MESH_FRIEND_SUBLIST_SIZE,
          p_rx_metadata->params.scanner.rssi,
          m_friend.friend_counter - 1);

    friend_tx(p_friendship,
              TRANSPORT_CONTROL_OPCODE_FRIEND_OFFER,
              &friend_offer_pdu,
              PACKET_MESH_TRS_CONTROL_FRIEND_OFFER_SIZE,
              p_control_packet->p_net_secmat,
              (p_rx_metadata->params.scanner.timestamp +
               MS_TO_US(offer_delay_ms)));

    /* If we don't receive a Friend Poll within 1 second after sending the
     * Friend Offer, the establishment has failed. */
    poll_timeout_schedule(p_friendship,
                          p_rx_metadata->params.scanner.timestamp,
                          MS_TO_US(FRIEND_OFFER_TIMEOUT_MS + (uint64_t)offer_delay_ms));
}

static void friend_clear_handle(const transport_control_packet_t * p_control_packet,
                                const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    uint16_t friend_req_count = 0;
    uint16_t lpn_counter = 0;

    if (p_control_packet->data_len != PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE)
    {
        return;
    }

    lpn_counter = packet_mesh_trs_control_friend_clear_lpn_counter_get(p_control_packet->p_data);

    friendship_t * p_friendship = friendship_get(p_control_packet,
                                                 PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE);

    recent_lpns_t * p_recent_lpn  = recent_lpns_get(p_control_packet,
                                                    PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE);

    /* Handle corner case that can occur during establishment process: `p_friendship` and
     * `p_recent_lpn` are both valid. If this happens, the friend will allow establishment
     * to continue without terminating (otherwise this will cause wasted effort for LPN).*/
    if ((p_friendship != NULL && p_recent_lpn != NULL) ||
        (p_friendship == NULL && p_recent_lpn == NULL))
    {
        return;
    }

    if (p_friendship != NULL)
    {
        friend_req_count = p_friendship->friendship.lpn.request_count;
    }

    if (p_recent_lpn != NULL)
    {
        friend_req_count = p_recent_lpn->last_req_count;
    }

    if (!friend_clear_is_valid_lpn_counter(friend_req_count, lpn_counter))
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Invalid LPNCounter: %u != %u\n",
              lpn_counter, friend_req_count);
        return;
    }

    if (p_friendship != NULL)
    {
        friend_clear_confirm_tx(p_friendship->friendship.lpn.src,
                                p_friendship->friendship.lpn.request_count,
                                p_friendship->bearer.token,
                                p_control_packet);
        friendship_terminate(p_friendship, NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NEW_FRIEND);

    }

    if (p_recent_lpn != NULL)
    {
        friend_clear_confirm_tx(p_recent_lpn->last_lpn,
                                p_recent_lpn->last_req_count,
                                p_recent_lpn->token,
                                p_control_packet);
    }
}

static void friend_clear_confirm_handle(const transport_control_packet_t * p_control_packet,
                                        const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    friendship_t * p_friendship = friendship_get(p_control_packet,
        PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_CONFIRM_SIZE);
    if (p_friendship == NULL)
    {
        return;
    }

    uint16_t lpn_counter = packet_mesh_trs_control_friend_clear_confirm_lpn_counter_get(
        p_control_packet->p_data);
    if (p_friendship->friendship.lpn.request_count != lpn_counter)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Unexpected LPNCounter: %u != %u\n",
              lpn_counter, p_friendship->friendship.lpn.request_count);
        return;
    }

    lt_abort(&p_friendship->clear_repeat_timer);
}

static void friend_sublist_add_handle(const transport_control_packet_t * p_control_packet,
                                      const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    friendship_t * p_friendship = friendship_get(p_control_packet, 0);
    if (p_friendship == NULL)
    {
        return;
    }

    uint8_t transaction_number =
        packet_mesh_trs_control_friend_sublist_add_remove_transaction_number_get(
            p_control_packet->p_data);

    uint32_t status = NRF_SUCCESS;
    const uint32_t address_count = sublist_address_count_get(p_control_packet);

    /* NOTE: There is no NACK support for an LPN "misbehaving", i.e., adding
     * more addresses than the subscription list can hold. */
    for (uint32_t i = 0;
         i < address_count && status == NRF_SUCCESS;
         ++i)
    {
        uint16_t address =
            packet_mesh_trs_control_friend_sublist_add_remove_address_list_get(
                p_control_packet->p_data, i);
        status = friend_sublist_add(&p_friendship->sublist, address);
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Add sublist addr 0x%04x: %u\n", address, status);
    }

    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_WARN, "Error %u: adding to sublist 0x%04x\n",
              status, p_friendship->friendship.lpn.src);
    }
    else
    {
        friend_sublist_confirm_tx(p_friendship, transaction_number, p_rx_metadata);
    }

    poll_timeout_schedule(p_friendship,
                          p_rx_metadata->params.scanner.timestamp,
                          MS_TO_US((uint64_t)p_friendship->friendship.poll_timeout_ms));
}

static void friend_sublist_remove_handle(const transport_control_packet_t * p_control_packet,
                                         const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    friendship_t * p_friendship = friendship_get(p_control_packet, 0);
    if (p_friendship == NULL)
    {
        return;
    }

    uint8_t transaction_number =
        packet_mesh_trs_control_friend_sublist_add_remove_transaction_number_get(
            p_control_packet->p_data);

    for (uint32_t i = 0;
         i < sublist_address_count_get(p_control_packet);
         ++i)
    {
        uint16_t address =
            packet_mesh_trs_control_friend_sublist_add_remove_address_list_get(
                p_control_packet->p_data, i);
        uint32_t status = friend_sublist_remove(&p_friendship->sublist, address);
        (void) status;
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Remove sublist addr 0x%04x: %u\n", address, status);
    }

    friend_sublist_confirm_tx(p_friendship, transaction_number, p_rx_metadata);
    poll_timeout_schedule(p_friendship,
                          p_rx_metadata->params.scanner.timestamp,
                          MS_TO_US((uint64_t)p_friendship->friendship.poll_timeout_ms));
}

static const transport_control_packet_handler_t m_transport_opcode_handlers[] = {
    {TRANSPORT_CONTROL_OPCODE_FRIEND_POLL, friend_poll_handle},
    {TRANSPORT_CONTROL_OPCODE_FRIEND_REQUEST, friend_request_handle},
    {TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR, friend_clear_handle},
    {TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM, friend_clear_confirm_handle},
    {TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD, friend_sublist_add_handle},
    {TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE, friend_sublist_remove_handle}};

static void friend_update_enqueue(friendship_t * p_friendship)
{
    NRF_MESH_ASSERT_DEBUG(m_friend.enabled);

    uint16_t unicast_address;
    uint16_t element_count;
    nrf_mesh_unicast_address_get(&unicast_address, &element_count);

    const nrf_mesh_network_secmat_t * p_secmat = NULL;
    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src, &p_secmat);
    NRF_MESH_ASSERT_DEBUG(p_secmat != NULL);

    transport_packet_metadata_t trs_metadata = {
        .segmented = false,
        .receivers = TRANSPORT_PACKET_RECEIVER_FRIEND,
        .mic_size = PACKET_MESH_TRS_TRANSMIC_CONTROL_SIZE,
        .type.control.opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
        .net = {
            .control_packet = true,
            .dst = {
                .value = p_friendship->friendship.lpn.src,
                .type = NRF_MESH_ADDRESS_TYPE_UNICAST
            },
            .src = unicast_address,
            .p_security_material = p_secmat,
            .ttl = 0
        },
        .tx_bearer_selector = CORE_TX_BEARER_TYPE_FRIEND,
        .token = p_friendship->bearer.token
    };

    packet_mesh_trs_packet_t friend_update_packet;
    memset(&friend_update_packet, 0, sizeof(packet_mesh_trs_packet_t));
    packet_mesh_trs_control_opcode_set(&friend_update_packet,
                                       TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE);

    packet_mesh_trs_control_packet_t * p_friend_update =
        (packet_mesh_trs_control_packet_t *) &friend_update_packet.pdu[PACKET_MESH_TRS_UNSEG_PDU_OFFSET];

    memset(p_friend_update, 0, PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE);

    packet_mesh_trs_control_friend_update_iv_update_flag_set(
        p_friend_update, (net_state_iv_update_get() == NET_STATE_IV_UPDATE_IN_PROGRESS));
    packet_mesh_trs_control_friend_update_iv_index_set(
        p_friend_update, net_state_beacon_iv_index_get());

    packet_mesh_trs_control_friend_update_key_refresh_flag_set(
        p_friend_update, (nrf_mesh_key_refresh_phase_get(p_secmat)
                         == NRF_MESH_KEY_REFRESH_PHASE_2));

    /* The MD bit must be set when we actually relay the packet. */
    __LOG_XB(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Friend Update Queue",
             p_friend_update->pdu,
             PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE);
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FRIEND_PACKET_QUEUED, 0, PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE, p_friend_update->pdu);
    friend_queue_packet_push(&p_friendship->queue,
                             &friend_update_packet,
                             PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE +
                             PACKET_MESH_TRS_UNSEG_PDU_OFFSET,
                             &trs_metadata,
                             CORE_TX_ROLE_ORIGINATOR);
}

static void iv_update_handle(void)
{
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state == FRIEND_STATE_ESTABLISHED)
        {
            friend_update_enqueue(&m_friend.friends[i]);
        }
    }
}

static bool is_subnet_of_friend(const friendship_t * p_friendship,
                                const nrf_mesh_network_secmat_t * p_secmat)
{
    const nrf_mesh_network_secmat_t * p_friend_secmat = NULL;
    nrf_mesh_friendship_secmat_get(p_friendship->friendship.lpn.src, &p_friend_secmat);
    NRF_MESH_ASSERT_DEBUG(p_friend_secmat != NULL);

    const nrf_mesh_network_secmat_t * p_master_secmat = nrf_mesh_net_master_secmat_get(p_friend_secmat);
    NRF_MESH_ASSERT_DEBUG(p_master_secmat != NULL);

    return ((intptr_t) p_master_secmat == (intptr_t) p_secmat);
}

static void key_refresh_handle(const nrf_mesh_evt_key_refresh_notification_t * p_evt)
{
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state != FRIEND_STATE_ESTABLISHED)
        {
            continue;
        }

        if (is_subnet_of_friend(&m_friend.friends[i], nrf_mesh_net_secmat_from_index_get(p_evt->subnet_index)) &&
            p_evt->phase == NRF_MESH_KEY_REFRESH_PHASE_2)
        {
            friend_update_enqueue(&m_friend.friends[i]);
        }
    }
}

static void mesh_evt_cb(const nrf_mesh_evt_t * p_evt)
{
    if (!m_friend.enabled)
    {
        return;
    }

    switch (p_evt->type)
    {
        case NRF_MESH_EVT_IV_UPDATE_NOTIFICATION:
            iv_update_handle();
            break;

        case NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION:
            key_refresh_handle(&p_evt->params.key_refresh);
            break;

        default:
            break;
    }
}

static void tx_power_config_listener_cb(mesh_config_change_reason_t reason, mesh_config_entry_id_t id, const void * p_entry)
{
    if (id.file == MESH_OPT_CORE_FILE_ID &&
        id.record == (MESH_OPT_CORE_TX_POWER_RECORD_START + CORE_TX_ROLE_ORIGINATOR))
    {
        const radio_tx_power_t * p_tx_power = p_entry;
        for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
        {
            switch (reason)
            {
                case MESH_CONFIG_CHANGE_REASON_SET:
                    m_friend.friends[i].bearer.broadcast.params.radio_config.tx_power = *p_tx_power;
                    break;
                case MESH_CONFIG_CHANGE_REASON_DELETE:
                    m_friend.friends[i].bearer.broadcast.params.radio_config.tx_power = RADIO_POWER_NRF_0DBM;
                    break;
                default:
                    break;
            }
        }
    }
}

MESH_CONFIG_LISTENER(m_friend_tx_power_config_listener, MESH_OPT_CORE_TX_POWER_EID, tx_power_config_listener_cb);

/*****************************************************************************
 * Interface functions
 *****************************************************************************/
uint32_t mesh_friend_init(void)
{
#ifdef UNIT_TEST
    memset(&m_friend, 0, sizeof(m_friend));
#endif

    mesh_config_entry_id_t id =
    {
            .file = MESH_OPT_CORE_FILE_ID,
            .record = MESH_OPT_CORE_TX_POWER_RECORD_START + CORE_TX_ROLE_ORIGINATOR
    };
    radio_tx_power_t tx_power;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_get(id, (void *)(&tx_power)));

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        m_friend.friends[i].state = FRIEND_STATE_IDLE;
        friend_queue_init(&m_friend.friends[i].queue);
        friend_sublist_init(&m_friend.friends[i].sublist);

        core_tx_friend_init(&m_friend.friends[i].bearer,
                            NRF_MESH_FRIEND_TOKEN_BEGIN + i,
                            tx_power);
    }

    for (uint32_t i = 0; i < FRIEND_RECENT_LPNS_LIST_COUNT; i++)
    {
        m_friend.recent_lpns[i].last_lpn = NRF_MESH_ADDR_UNASSIGNED;
    }

    /* TODO: Handle enabled/disabled state. */
    m_friend.default_receive_window_ms = MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS;
    NRF_MESH_ASSERT_DEBUG((transport_control_packet_consumer_add(
                               &m_transport_opcode_handlers[0],
                               ARRAY_SIZE(m_transport_opcode_handlers))
                           == NRF_SUCCESS));

    m_friend.mesh_evt_handler.evt_cb = mesh_evt_cb;
    nrf_mesh_evt_handler_add(&m_friend.mesh_evt_handler);
#if FRIEND_TEST_HOOK
    m_friend.tx_delay_ms = 0;
#endif
    return NRF_SUCCESS;
}

void mesh_friend_enable(void)
{
    NRF_MESH_ERROR_CHECK(mesh_opt_friend_set(true));
}

void mesh_friend_disable(void)
{
    NRF_MESH_ERROR_CHECK(mesh_opt_friend_set(false));
    (void) mesh_friend_friendship_terminate_all();
}

bool mesh_friend_is_enabled(void)
{
    return m_friend.enabled;
}

uint32_t mesh_friend_friendship_terminate(const mesh_friendship_t * p_friendship)
{
    if (p_friendship == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else
    {
        for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
        {
            if ((intptr_t) p_friendship == (intptr_t) &m_friend.friends[i].friendship)
            {
                friendship_terminate(&m_friend.friends[i],
                                     NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_USER);
                return NRF_SUCCESS;
            }
        }
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t mesh_friend_friendship_terminate_all(void)
{
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state != FRIEND_STATE_IDLE)
        {
            friendship_terminate(&m_friend.friends[i],
                                 NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_USER);
        }
    }
    return NRF_SUCCESS;
}

uint32_t mesh_friend_receive_window_set(uint8_t receive_window_ms)
{
    /* MESH_FRIEND_RECEIVE_WINDOW_MAX_MS is 0xFF. uint8_t cannot exceed this value. */
    if (receive_window_ms < MESH_FRIEND_RECEIVE_WINDOW_MIN_MS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_friend.default_receive_window_ms = receive_window_ms;
    return NRF_SUCCESS;
}

uint32_t mesh_friend_friendships_get(const mesh_friendship_t ** pp_friendships, uint8_t * p_count)
{
    if (p_count == NULL || pp_friendships == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint8_t input_count = *p_count;
    *p_count = 0;

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT && *p_count < input_count; ++i)
    {
        if (m_friend.friends[i].state == FRIEND_STATE_ESTABLISHED)
        {
            pp_friendships[i] = &m_friend.friends[i].friendship;
            (*p_count)++;
        }
    }
    return NRF_SUCCESS;
}

uint32_t mesh_opt_friend_set(bool enabled)
{
    return mesh_config_entry_set(MESH_OPT_FRIEND_EID, &enabled);
}

uint32_t mesh_opt_friend_get(bool * p_enabled)
{
    return mesh_config_entry_get(MESH_OPT_FRIEND_EID, p_enabled);
}

uint32_t mesh_friend_stats_get(const mesh_friendship_t * p_friendship, mesh_friend_stats_t * p_stats)
{
    if (p_friendship == NULL || p_stats == NULL)
    {
        return NRF_ERROR_NULL;
    }

#if FRIEND_DEBUG
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if ((intptr_t) p_friendship == (intptr_t) &m_friend.friends[i].friendship)
        {
            p_stats->bearer  = m_friend.friends[i].bearer.stats;
            p_stats->queue   = m_friend.friends[i].queue.stats;
            p_stats->sublist = m_friend.friends[i].sublist.stats;
            return NRF_SUCCESS;
        }
    }
    return NRF_ERROR_NOT_FOUND;
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

/* Internal API */
void friend_packet_in(const packet_mesh_trs_packet_t * p_packet,
                      uint8_t length,
                      const transport_packet_metadata_t * p_metadata,
                      core_tx_role_t role)
{
    NRF_MESH_ASSERT_DEBUG(p_packet != NULL);
    NRF_MESH_ASSERT_DEBUG(p_metadata != NULL);
    NRF_MESH_ASSERT_DEBUG(length <= sizeof(packet_mesh_trs_packet_t));

    if (!m_friend.enabled)
    {
        return;
    }

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        friendship_t * p_friendship = &m_friend.friends[i];

        if (p_friendship->state != FRIEND_STATE_ESTABLISHED)
        {
            continue;
        }

        if (friend_address_is_known(p_friendship,
                                    p_metadata->net.dst))
        {
            NRF_MESH_ASSERT_DEBUG(p_friendship != NULL);
            __LOG_XB(LOG_SRC_FRIEND, LOG_LEVEL_DBG1, "Packet Queue", p_packet->pdu, length);
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FRIEND_PACKET_QUEUED, 0, length, p_packet->pdu);
            friend_queue_packet_push(&p_friendship->queue,
                                     p_packet,
                                     length,
                                     p_metadata,
                                     role);
        }
    }
}

bool friend_needs_packet(const transport_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT_DEBUG(p_metadata != NULL);
    if (!m_friend.enabled)
    {
        return false;
    }

    /* No relaying for TTL < 2 (@tagMeshSp section 3.5.5). */
    if (p_metadata->net.ttl < 2)
    {
        return false;
    }

    /* No reason to receive a segmented packet if it doesn't fit into friend queue. */
    if (p_metadata->segmented &&
        (p_metadata->segmentation.last_segment + 1) > MESH_FRIEND_QUEUE_SIZE)
    {
        __LOG(LOG_SRC_FRIEND, LOG_LEVEL_WARN, "Can not receive the segmented message, the friend queue size is too small!\n");
        return false;
    }

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state != FRIEND_STATE_ESTABLISHED)
        {
            continue;
        }

        if (friend_address_is_known(&m_friend.friends[i],
                                    p_metadata->net.dst))
        {
            return true;
        }
    }
    return false;
}

void friend_sar_complete(uint16_t src, uint32_t seqzero, bool success)
{
    if (!m_friend.enabled)
    {
        return;
    }

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state == FRIEND_STATE_ESTABLISHED)
        {
            friend_queue_sar_complete(&m_friend.friends[i].queue, src, success);
        }
    }
}

bool friend_sar_exists(uint16_t src, uint64_t seqauth)
{
    if (!m_friend.enabled)
    {
        return false;
    }

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].state == FRIEND_STATE_ESTABLISHED)
        {
            return friend_queue_sar_exists(&m_friend.friends[i].queue, src, seqauth);
        }
    }

    return false;
}

bool friend_friendship_established(uint16_t src)
{
    if (!m_friend.enabled)
    {
        return false;
    }

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        if (m_friend.friends[i].friendship.lpn.src == src)
        {
            return true;
        }
    }
    return false;
}

uint32_t friend_remaining_poll_timeout_time_get(uint16_t src)
{
    friendship_t * p_friendship = friendship_find(src);
    if (p_friendship != NULL)
    {
        return ROUNDED_DIV(US_TO_MS(lt_remaining_time_get(&p_friendship->poll_timeout)), 100);
    }
    else
    {
        return 0ul;
    }
}

#if FRIEND_TEST_HOOK
void friend_tx_delay_set(uint16_t tx_delay_ms)
{
    m_friend.tx_delay_ms = tx_delay_ms;
}
#endif

#endif  /* MESH_FEATURE_FRIEND_ENABLED */
