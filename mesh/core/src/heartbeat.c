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
#include "heartbeat.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "nordic_common.h"
#include "log.h"
#include "event.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_assert.h"

#include "timer_scheduler.h"
#include "packet_mesh.h"
#include "event.h"
#include "rand.h"
#include "utils.h"
#include "timer.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_externs.h"

#include "mesh_opt_core.h"
#include "mesh_config_listener.h"
#include "mesh_config_entry.h"
#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "mesh_opt_gatt.h"
#endif
#if MESH_FEATURE_FRIEND_ENABLED
#include "mesh_opt_friend.h"
#endif

/*****************************************************************************
 * Local defines
 *****************************************************************************/
#if (MESH_FEATURE_LPN_ENABLED == 1) || (MESH_FEATURE_RELAY_ENABLED == 1) || (MESH_FEATURE_GATT_PROXY_ENABLED == 1) || \
    (MESH_FEATURE_FRIEND_ENABLED == 1)
#define HEARTBEAT_FEATURE_LATCH_ENABLED  1
#else
#define HEARTBEAT_FEATURE_LATCH_ENABLED  0
#endif

#define HEARTBEAT_MIN_HOPS_INIT  (0x7F)
#define HEARTBEAT_MAX_HOPS_INIT  (0x00)

#define HEARTBEAT_PUBLISH_SUB_INTERVAL_S (1800)
#define HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S (1)

static heartbeat_subscription_state_t m_heartbeat_subscription;
static heartbeat_publication_state_t m_heartbeat_publication;

static bool m_publish_pending;
/** Flag indicating whether we've configured the publish parameters since the device started or not.
 * Used to determine what to do with the publish count. */
static bool m_publish_is_loaded;

static struct
{
    uint32_t remaining_time_s;
    timer_event_t timer;
} m_publication_timer;

static timer_event_t m_subscription_timer;
static uint8_t m_latched_features;
static nrf_mesh_evt_handler_t m_hb_core_evt_handler;
static hb_pub_info_getter_t mp_getter;

/*****************************************************************************
 * Static internal functions
 *****************************************************************************/

static inline bool publish_is_enabled(void)
{
    return m_heartbeat_publication.dst != NRF_MESH_ADDR_UNASSIGNED;
}

static inline bool periodic_publication_is_running(void)
{
    return (publish_is_enabled() && m_heartbeat_publication.count != 0x0000);
}

static void periodic_publication_start(void)
{
    uint32_t delay_seconds;

    /* The first message should be sent as soon as possible when setting the publication state. If
     * the starting comes because of a load from persistent storage, we'll start it after a delay. */
    if (m_publish_is_loaded)
    {
        delay_seconds = MIN(m_heartbeat_publication.period, HEARTBEAT_PUBLISH_SUB_INTERVAL_S);
        m_publication_timer.remaining_time_s = m_heartbeat_publication.period - delay_seconds; // set the value for the next timeout now
    }
    else
    {
        m_publication_timer.remaining_time_s = 0;
        delay_seconds = 0;
    }

    timer_sch_reschedule(&m_publication_timer.timer, timer_now() + SEC_TO_US(delay_seconds));
}

static void periodic_publication_stop(void)
{
    m_heartbeat_publication.count = 0;
    m_heartbeat_publication.period = 0;
    timer_sch_abort(&m_publication_timer.timer);
}

static void publication_count_decrement(void)
{
    if ((m_heartbeat_publication.count != 0) && (m_heartbeat_publication.count != HEARTBEAT_INF_COUNT))
    {
        m_heartbeat_publication.count--;
    }
}
/*****************************************************************************/
/* << RX path functions >> */

static void heartbeat_opcode_handle(const transport_control_packet_t * p_control_packet,
                                    const nrf_mesh_rx_metadata_t *     p_rx_metadata)
{
    if ((p_control_packet->opcode != TRANSPORT_CONTROL_OPCODE_HEARTBEAT) ||
        (p_control_packet->src != m_heartbeat_subscription.src) ||
        (p_control_packet->dst.value != m_heartbeat_subscription.dst) ||
        (p_control_packet->data_len != PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE))
    {
        return;
    }

    if (m_heartbeat_subscription.period > 0)
    {
        // @tagMeshSp section 3.6.7.3:
        // "The counter does not wrap. It stops counting at 0xFFFF."
        if (m_heartbeat_subscription.count < HEARTBEAT_MAX_COUNT)
        {
            m_heartbeat_subscription.count++;
        }

        nrf_mesh_evt_t evt;
        evt.type = NRF_MESH_EVT_HB_MESSAGE_RECEIVED;
        evt.params.hb_message.init_ttl =
            packet_mesh_trs_control_heartbeat_init_ttl_get(p_control_packet->p_data);
        evt.params.hb_message.hops =
            packet_mesh_trs_control_heartbeat_init_ttl_get(p_control_packet->p_data) -
            p_control_packet->ttl + 1;
        evt.params.hb_message.features =
            packet_mesh_trs_control_heartbeat_features_get(p_control_packet->p_data) &
            HEARTBEAT_TRIGGER_TYPE_RFU_MASK;
        evt.params.hb_message.src = p_control_packet->src;

        if (evt.params.hb_message.hops < m_heartbeat_subscription.min_hops)
        {
            m_heartbeat_subscription.min_hops = evt.params.hb_message.hops;
        }

        if (evt.params.hb_message.hops > m_heartbeat_subscription.max_hops)
        {
            m_heartbeat_subscription.max_hops = evt.params.hb_message.hops;
        }

        event_handle(&evt);
    }
}

/*****************************************************************************/
/* << TX path functions >> */

static void heartbeat_meta_prepare(transport_control_packet_t *             p_tx,
                                   const packet_mesh_trs_control_packet_t * p_pdu,
                                   uint8_t                                  pdu_len,
                                   heartbeat_publication_information_t    * p_pub_info)
{
    // Network security material must be always valid. This should never assert.
    NRF_MESH_ASSERT(p_pub_info->p_net_secmat != NULL);

    p_tx->opcode          = TRANSPORT_CONTROL_OPCODE_HEARTBEAT;
    p_tx->reliable        = false;
    p_tx->src             = p_pub_info->local_address;
    p_tx->dst.value       = m_heartbeat_publication.dst;
    p_tx->dst.type        = nrf_mesh_address_type_get(m_heartbeat_publication.dst);
    p_tx->p_net_secmat    = p_pub_info->p_net_secmat;
    p_tx->p_data          = p_pdu;
    p_tx->data_len        = pdu_len;
    p_tx->ttl             = m_heartbeat_publication.ttl;
    p_tx->bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL;
}

static uint32_t heartbeat_send(heartbeat_publication_information_t * p_hb_pub_info)
{
    transport_control_packet_t       tx_params;
    packet_mesh_trs_control_packet_t hb_pdu;

    memset(hb_pdu.pdu, 0, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE);
    packet_mesh_trs_control_heartbeat_init_ttl_set(&hb_pdu, m_heartbeat_publication.ttl);
    packet_mesh_trs_control_heartbeat_features_set(&hb_pdu, m_latched_features);

    heartbeat_meta_prepare(&tx_params, &hb_pdu, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE, p_hb_pub_info);

    return (transport_control_tx(&tx_params, NRF_MESH_HEARTBEAT_TOKEN));
}

static void heartbeat_publish(void)
{
    if (publish_is_enabled())
    {
        heartbeat_publication_information_t hb_pub_info;
        NRF_MESH_ASSERT(NULL != mp_getter);
        if (mp_getter(&hb_pub_info) == NRF_SUCCESS)
        {
            m_publish_pending = (heartbeat_send(&hb_pub_info) != NRF_SUCCESS);
        }
    }
}

/**
 * Callback for the subscription timer. this callback triggers every second. As per
 * @tagMeshSp section 4.2.18.4, node shall report the remaining time.
 */
static void heartbeat_subscription_timer_cb(timestamp_t timestamp, void * p_context)
{
    if (m_heartbeat_subscription.period < HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S)
    {
        timer_sch_abort(&m_subscription_timer);
        m_heartbeat_subscription.period = 0;

        const nrf_mesh_evt_t change_evt = {
            .type = NRF_MESH_EVT_HB_SUBSCRIPTION_CHANGE,
            .params.hb_subscription_change = {
                .p_old = &m_heartbeat_subscription,
                .p_new = NULL,
            }
        };
        event_handle(&change_evt);
    }
    else
    {
        m_heartbeat_subscription.period -= HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S;
    }
}

/** Callback for publication timer. If publication period is greater than
 * HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds, this callback triggers every
 * HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds, else it triggers after publication period.
 */
static void heartbeat_publication_timer_cb(timestamp_t timestamp, void * p_context)
{
    if (!periodic_publication_is_running())
    {
        m_publication_timer.timer.interval = 0;
        return;
    }

    if (m_publication_timer.remaining_time_s == 0)
    {
        publication_count_decrement();
        heartbeat_publish();
        /* Reset timer */
        m_publication_timer.remaining_time_s = m_heartbeat_publication.period;
    }

    /* Prepare for next timeout: */
    uint32_t interval_seconds = MIN(m_publication_timer.remaining_time_s, HEARTBEAT_PUBLISH_SUB_INTERVAL_S);
    m_publication_timer.timer.interval = SEC_TO_US(interval_seconds);
    m_publication_timer.remaining_time_s -= interval_seconds;
}

#if HEARTBEAT_FEATURE_LATCH_ENABLED
static void on_feature_update(uint16_t feature, bool enabled)
{
    if (enabled == !(feature & m_latched_features))
    {
        m_latched_features ^= feature; // the guard above makes sure that we'll only xor if the value is changed
        if (m_heartbeat_publication.features & feature)
        {
            heartbeat_publish();
        }
    }
}
#endif

/** Event handler callback sends the pending triggered message on the TX complete */
static void heartbeat_core_evt_cb(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_TX_COMPLETE:
            if (m_publish_pending)
            {
                heartbeat_publish();
            }
            break;
#if MESH_FEATURE_LPN_ENABLED
        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            if (p_evt->params.friendship_established.role == NRF_MESH_FRIENDSHIP_ROLE_LPN)
            {
                on_feature_update(HEARTBEAT_TRIGGER_TYPE_LPN, true);
            }
            break;
        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            if (p_evt->params.friendship_terminated.role == NRF_MESH_FRIENDSHIP_ROLE_LPN)
            {
                on_feature_update(HEARTBEAT_TRIGGER_TYPE_LPN, false);
            }
            break;
#endif
        default:
            break;
    }
}

/*****************************************************************************
 * State listener functions
 *****************************************************************************/

#if MESH_FEATURE_RELAY_ENABLED
static void heartbeat_relay_listener_cb(mesh_config_change_reason_t reason,
                                        mesh_config_entry_id_t id,
                                        const void * p_entry)
{
    const mesh_opt_core_adv_t * p_relay = p_entry;
    const core_tx_role_t role = MESH_OPT_CORE_ADV_ENTRY_ID_TO_ROLE(id);
    if (role == CORE_TX_ROLE_RELAY &&
        reason == MESH_CONFIG_CHANGE_REASON_SET)
    {
        on_feature_update(HEARTBEAT_TRIGGER_TYPE_RELAY, p_relay->enabled);
    }
}

MESH_CONFIG_LISTENER(m_heartbeat_relay_listener, MESH_OPT_CORE_ADV_EID, heartbeat_relay_listener_cb);
#endif /* MESH_FEATURE_RELAY_ENABLED */

#if MESH_FEATURE_GATT_PROXY_ENABLED
static void heartbeat_proxy_listener_cb(mesh_config_change_reason_t reason,
                                        mesh_config_entry_id_t id,
                                        const void * p_entry)
{
    const bool * p_enabled = p_entry;
    if (reason == MESH_CONFIG_CHANGE_REASON_SET)
    {
        on_feature_update(HEARTBEAT_TRIGGER_TYPE_PROXY, *p_enabled);
    }
}

MESH_CONFIG_LISTENER(m_heartbeat_proxy_listener, MESH_OPT_GATT_PROXY_EID, heartbeat_proxy_listener_cb);
#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */

#if MESH_FEATURE_FRIEND_ENABLED
static void heartbeat_friend_listener_cb(mesh_config_change_reason_t reason,
                                         mesh_config_entry_id_t id,
                                         const void * p_entry)
{
    const bool * p_enabled = p_entry;
    if (reason == MESH_CONFIG_CHANGE_REASON_SET)
    {
        on_feature_update(HEARTBEAT_TRIGGER_TYPE_FRIEND, *p_enabled);
    }
}

MESH_CONFIG_LISTENER(m_heartbeat_friend_listener, MESH_OPT_FRIEND_EID, heartbeat_friend_listener_cb);
#endif /* MESH_FEATURE_FRIEND_ENABLED */

/*****************************************************************************
 * Mesh Config wrapper functions
 *****************************************************************************/
static uint32_t hb_pub_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_HB_PUBLICATION_RECORD == entry_id.record);

    const heartbeat_publication_state_t * p_publication = (const heartbeat_publication_state_t *) p_entry;

    nrf_mesh_address_type_t dst_type = nrf_mesh_address_type_get(p_publication->dst);

    if (dst_type == NRF_MESH_ADDRESS_TYPE_VIRTUAL ||
        (p_publication->count > HEARTBEAT_MAX_COUNT && p_publication->count != HEARTBEAT_INF_COUNT) ||
        (p_publication->period > HEARTBEAT_MAX_PERIOD) || (p_publication->ttl > NRF_MESH_TTL_MAX))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_heartbeat_publication = *(heartbeat_publication_state_t *) p_entry;

    if (p_publication->dst == NRF_MESH_ADDR_UNASSIGNED)
    {
        // publish is disabled, wipe values according to @tagMeshSp section 4.4.1.2.15
        m_heartbeat_publication.count = 0;
        m_heartbeat_publication.period = 0;
        m_heartbeat_publication.ttl = 0;
    }

    /* As we don't actively update the count in persistent storage for every publication, we'll
     * ignore non-infinite count values when loading them from flash. Otherwise, we'll end up
     * resetting the counter to the initial value on every power up. */
    bool count_is_valid = (m_publish_is_loaded
                               ? (m_heartbeat_publication.count == HEARTBEAT_INF_COUNT)
                               : (m_heartbeat_publication.count > 0));

    if (count_is_valid && m_heartbeat_publication.period > 0)
    {
        periodic_publication_start();
    }
    else
    {
        periodic_publication_stop();
    }

    return NRF_SUCCESS;
}

static void hb_pub_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_HB_PUBLICATION_RECORD == entry_id.record);

    memcpy(p_entry, &m_heartbeat_publication, sizeof(heartbeat_publication_state_t));
}

static void hb_pub_deleter(mesh_config_entry_id_t entry_id)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_HB_PUBLICATION_RECORD == entry_id.record);
    memset(&m_heartbeat_publication, 0, sizeof(m_heartbeat_publication));
}

MESH_CONFIG_ENTRY(heartbeat_publication,
                  MESH_OPT_CORE_HB_PUBLICATION_EID,
                  1,
                  sizeof(heartbeat_publication_state_t),
                  hb_pub_setter,
                  hb_pub_getter,
                  hb_pub_deleter,
                  true);

/*****************************************************************************
 * Public API
 *****************************************************************************/

void heartbeat_init(void)
{
#ifdef UNIT_TEST
    memset(&m_heartbeat_publication, 0, sizeof(m_heartbeat_publication));
    memset(&m_heartbeat_subscription, 0, sizeof(m_heartbeat_subscription));
    m_latched_features = 0;
#endif
    // register transport event handler to handle heartbeat message opcode
    static const transport_control_packet_handler_t cp_handler = {
        .opcode = TRANSPORT_CONTROL_OPCODE_HEARTBEAT, .callback = heartbeat_opcode_handle
    };
    NRF_MESH_ASSERT(transport_control_packet_consumer_add(&cp_handler, 1) == NRF_SUCCESS);

    // Initialize timer event structures
    m_publication_timer.timer.cb = heartbeat_publication_timer_cb;

    m_subscription_timer.cb        = heartbeat_subscription_timer_cb;

    // Remaining subscription period needs to be reported by heartbeat module as a part of
    // HEARTBEAT SUBSCRIPTION GET message. Also, incoming heartbeat messages should not be accepted
    // once this period runs out. Since the period is specified in seconds, the timer interval
    // of 1 second is good enough resolution.
    m_subscription_timer.interval  = SEC_TO_US(HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S);

    m_hb_core_evt_handler.evt_cb = heartbeat_core_evt_cb;
    event_handler_add(&m_hb_core_evt_handler);

    /* The publish value is loaded the first time the setter gets an update unless someone called
     * the publish_set function */
    m_publish_is_loaded = true;
}

uint32_t heartbeat_subscription_set(const heartbeat_subscription_state_t * p_hb_sub)
{
    NRF_MESH_ASSERT(p_hb_sub != NULL);

    uint16_t addr_start;
    uint16_t addr_count;
    nrf_mesh_unicast_address_get(&addr_start, &addr_count);

    // Validate period, src, and dst
    if (
         (p_hb_sub->period > (HEARTBEAT_MAX_PERIOD)) ||
         !((
           (nrf_mesh_address_type_get(p_hb_sub->src) == NRF_MESH_ADDRESS_TYPE_UNICAST) ||
           (nrf_mesh_address_type_get(p_hb_sub->src) == NRF_MESH_ADDRESS_TYPE_INVALID)
          ) &&
          (
           (nrf_mesh_address_type_get(p_hb_sub->dst) == NRF_MESH_ADDRESS_TYPE_INVALID) ||
           (nrf_mesh_address_type_get(p_hb_sub->dst) == NRF_MESH_ADDRESS_TYPE_GROUP)   ||
           (p_hb_sub->dst == addr_start)
          ))
       )
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    const heartbeat_subscription_state_t old_state = m_heartbeat_subscription;

    // Validate data of received parameters
    if ((p_hb_sub->period == 0x00) ||
        (p_hb_sub->src == NRF_MESH_ADDR_UNASSIGNED) ||
        (p_hb_sub->dst == NRF_MESH_ADDR_UNASSIGNED)
       )
    {
        m_heartbeat_subscription.src = NRF_MESH_ADDR_UNASSIGNED;
        m_heartbeat_subscription.dst = NRF_MESH_ADDR_UNASSIGNED;
        m_heartbeat_subscription.period = 0x0000;

        timer_sch_abort(&m_subscription_timer);
    }
    else
    {
        m_heartbeat_subscription.src = p_hb_sub->src;
        m_heartbeat_subscription.dst = p_hb_sub->dst;
        m_heartbeat_subscription.period = p_hb_sub->period;

        // (Re-)start subscription timer
        timer_sch_reschedule(&m_subscription_timer, m_subscription_timer.interval + timer_now());

        // reset hops and counts to the spec defined initial values
        m_heartbeat_subscription.min_hops = HEARTBEAT_MIN_HOPS_INIT;
        m_heartbeat_subscription.max_hops = HEARTBEAT_MAX_HOPS_INIT;
        m_heartbeat_subscription.count    = 0x00;
    }

    const nrf_mesh_evt_t change_evt = {
        .type = NRF_MESH_EVT_HB_SUBSCRIPTION_CHANGE,
        .params.hb_subscription_change = {
            .p_old = (old_state.dst == NRF_MESH_ADDR_UNASSIGNED ? NULL : &old_state),
            .p_new = &m_heartbeat_subscription,
        }
    };
    event_handle(&change_evt);

    return NRF_SUCCESS;
}

const heartbeat_subscription_state_t * heartbeat_subscription_get(void)
{
    return &m_heartbeat_subscription;
}

const heartbeat_publication_state_t * heartbeat_publication_get(void)
{
    return &m_heartbeat_publication;
}

uint32_t heartbeat_publication_set(const heartbeat_publication_state_t * p_publication_state)
{
    m_publish_is_loaded = false;
    return mesh_config_entry_set(MESH_OPT_CORE_HB_PUBLICATION_EID, p_publication_state);
}

void heartbeat_public_info_getter_register(hb_pub_info_getter_t p_getter)
{
    NRF_MESH_ASSERT(NULL != (mp_getter = p_getter));
}
