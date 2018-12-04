/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

/*****************************************************************************
 * Local defines
 *****************************************************************************/
#define HEARTBEAT_MIN_HOPS_INIT  (0x7F)
#define HEARTBEAT_MAX_HOPS_INIT  (0x00)

#define HEARTBEAT_PUBLISH_SUB_INTERVAL_S (1800)
#define HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S (1)


static bool m_heartbeat_init_done;
static heartbeat_subscription_state_t m_heartbeat_subscription;
static heartbeat_publication_state_t m_heartbeat_publication;

static bool m_hb_pending_pub_msg;
static bool m_hb_pending_feat_msg;

static struct
{
    uint32_t remaining_time_s;
    timer_event_t timer;
} m_publication_timer;

static timer_event_t m_subscription_timer;
static uint8_t m_latched_features;

/* Static callback functions*/
static nrf_mesh_evt_handler_t m_hb_core_evt_handler;

static hb_pub_info_getter_t mp_getter;

/** Forward declarations */
static uint32_t heartbeat_send(heartbeat_publication_information_t * p_hb_pub_info);
static void heartbeat_subscription_timer_cb(timestamp_t timestamp, void * p_context);
static void heartbeat_publication_timer_cb(timestamp_t timestamp, void * p_context);
static void heartbeat_opcode_handle(const transport_control_packet_t * p_control_packet,
                                    const nrf_mesh_rx_metadata_t *     p_rx_metadata);
static void heartbeat_publication_count_decrement(void);

/** Transport handler for the Heartbeat opcode */
static const transport_control_packet_handler_t cp_handler = {
    .opcode = TRANSPORT_CONTROL_OPCODE_HEARTBEAT, .callback = heartbeat_opcode_handle
};

#if MESH_FEATURE_RELAY_ENABLED
static void heartbeat_relay_listener_cb(mesh_config_change_reason_t reason,
                                        mesh_config_entry_id_t id,
                                        const void * p_entry);

MESH_CONFIG_LISTENER(m_heartbeat_relay_listener,
                     MESH_OPT_CORE_ADV_EID,
                     heartbeat_relay_listener_cb);
#endif /* MESH_FEATURE_RELAY_ENABLED */

#if MESH_FEATURE_GATT_PROXY_ENABLED
static void heartbeat_proxy_listener_cb(mesh_config_change_reason_t reason,
                                        mesh_config_entry_id_t id,
                                        const void * p_entry);

MESH_CONFIG_LISTENER(m_heartbeat_proxy_listener,
                     MESH_OPT_GATT_PROXY_EID,
                     heartbeat_proxy_listener_cb);
#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */


/*****************************************************************************
 * Static internal functions
 *****************************************************************************/

static inline bool is_publication_running(void)
{
    return (m_heartbeat_publication.dst   != NRF_MESH_ADDR_UNASSIGNED &&
            m_heartbeat_publication.count != 0x0000);
}

static inline bool can_publish_feature_change(void)
{
    return m_heartbeat_publication.dst != NRF_MESH_ADDR_UNASSIGNED;
}

static void heartbeat_restart_publication(void)
{
    m_publication_timer.remaining_time_s = m_heartbeat_publication.period;
    uint32_t next_timeout = MIN(m_heartbeat_publication.period, HEARTBEAT_PUBLISH_SUB_INTERVAL_S);

    timer_sch_reschedule(&m_publication_timer.timer,
                         timer_now() + SEC_TO_US(next_timeout));
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
        // Mesh Profile Specification v1.0, section 3.6.7.3:
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

    NRF_MESH_ASSERT(m_heartbeat_publication.count != 0 || m_heartbeat_publication.features);

    memset(hb_pdu.pdu, 0, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE);
    packet_mesh_trs_control_heartbeat_init_ttl_set(&hb_pdu, m_heartbeat_publication.ttl);
    packet_mesh_trs_control_heartbeat_features_set(&hb_pdu, m_latched_features);

    heartbeat_meta_prepare(&tx_params, &hb_pdu, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE, p_hb_pub_info);

    return (transport_control_tx(&tx_params, NRF_MESH_HEARTBEAT_TOKEN));
}

/** Callback for the subscription timer. this callback triggers every second. As per
 * Mesh Profile Specification v1.0, section 4.2.18.4, node shall report the remaining time.
 *
 * For this call, p_context must be &m_heartbeat_subscription.period
 * */
static void heartbeat_subscription_timer_cb(timestamp_t timestamp, void * p_context)
{
    uint32_t * p_sub_period = p_context;

    if (*p_sub_period < HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S)
    {
        timer_sch_abort(&m_subscription_timer);
        *p_sub_period = 0;
    }
    else
    {
        *p_sub_period -= HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S;
    }
}

/** Callback for publication timer. If publication period is greater than
 * HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds, this callback triggers every
 * HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds, else it triggers after publication period.
 *
 * The callback parameter p_context points to the m_heartbeat_publication.period.
 */
static void heartbeat_publication_timer_cb(timestamp_t timestamp, void * p_context)
{
    heartbeat_publication_information_t     hb_pub_info;

    /* Publish heartbeat if it is time to publish. Stop sending heartbeats if mp_getter fails */
    NRF_MESH_ASSERT(NULL != mp_getter);
    if (mp_getter(&hb_pub_info) == NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(m_publication_timer.remaining_time_s > 0);

        m_publication_timer.remaining_time_s -=
                MIN(MIN(m_heartbeat_publication.period, HEARTBEAT_PUBLISH_SUB_INTERVAL_S), m_publication_timer.remaining_time_s);

        if (m_publication_timer.remaining_time_s == 0 && is_publication_running() > 0)
        {
            if (heartbeat_send(&hb_pub_info) == NRF_SUCCESS)
            {
                heartbeat_publication_count_decrement();
            }
            m_publication_timer.remaining_time_s = m_heartbeat_publication.period;
        }

        if (m_heartbeat_publication.count > 0)
        {
            timer_sch_reschedule(&m_publication_timer.timer,
                                 timestamp +
                                 SEC_TO_US(MIN(m_publication_timer.remaining_time_s, HEARTBEAT_PUBLISH_SUB_INTERVAL_S)));
        }
        else
        {
            timer_sch_abort(&m_publication_timer.timer);
        }
    }
    else
    {
        timer_sch_abort(&m_publication_timer.timer);
    }
}

/** Event handler callback sends the pending triggered message on the TX complete */
static void heartbeat_core_evt_cb(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_TX_COMPLETE)
    {
        heartbeat_publication_information_t hb_pub_info;

        /* Get the latest value of the heartbeat publication information.
         * dst and count may get modified, re-check for validity */
        NRF_MESH_ASSERT(NULL != mp_getter);
        if ((mp_getter(&hb_pub_info) == NRF_SUCCESS))
        {
            if (m_hb_pending_pub_msg && is_publication_running())
            {
                if (heartbeat_send(&hb_pub_info) == NRF_SUCCESS)
                {
                    heartbeat_publication_count_decrement();
                }
            }

            if (m_hb_pending_feat_msg && can_publish_feature_change())
            {
                (void) heartbeat_send(&hb_pub_info);
            }
        }

        // De-register core event handler for TX complete to prevent unnecessary
        // triggering of this cb.
        event_handler_remove(&m_hb_core_evt_handler);

        m_hb_pending_pub_msg = false;
        m_hb_pending_feat_msg = false;
    }
}

#if MESH_FEATURE_RELAY_ENABLED || MESH_FEATURE_GATT_PROXY_ENABLED
static void heartbeat_on_feature_change_trigger(uint16_t hb_trigger)
{
    if ((m_heartbeat_publication.features & hb_trigger) &&
        can_publish_feature_change())
    {
        // Set internal flag to trigger heartbeat on next tx complete event
        event_handler_add(&m_hb_core_evt_handler);
        m_hb_pending_feat_msg = true;
    }
}
#endif /* MESH_FEATURE_RELAY_ENABLED || MESH_FEATURE_GATT_PROXY_ENABLED */

static void heartbeat_publication_count_decrement(void)
{
    if ((m_heartbeat_publication.count != 0) && (m_heartbeat_publication.count != HEARTBEAT_INF_COUNT))
    {
        m_heartbeat_publication.count--;
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
        reason == MESH_CONFIG_CHANGE_REASON_SET &&
        p_relay->enabled != ((m_latched_features & HEARTBEAT_TRIGGER_TYPE_RELAY) > 0))
    {
        m_latched_features ^= HEARTBEAT_TRIGGER_TYPE_RELAY;
        heartbeat_on_feature_change_trigger(HEARTBEAT_TRIGGER_TYPE_RELAY);
    }
}
#endif /* MESH_FEATURE_RELAY_ENABLED */

#if MESH_FEATURE_GATT_PROXY_ENABLED
static void heartbeat_proxy_listener_cb(mesh_config_change_reason_t reason,
                                        mesh_config_entry_id_t id,
                                        const void * p_entry)
{
    const bool * p_enabled = p_entry;
    if (reason == MESH_CONFIG_CHANGE_REASON_SET &&
        *p_enabled != ((m_latched_features & HEARTBEAT_TRIGGER_TYPE_PROXY) > 0))
    {
        m_latched_features ^= HEARTBEAT_TRIGGER_TYPE_PROXY;
        heartbeat_on_feature_change_trigger(HEARTBEAT_TRIGGER_TYPE_PROXY);
    }
}
#endif /* MESH_FEATURE_GATT_PROXY_ENABLED */

/*****************************************************************************
 * Mesh Config wrapper functions
 *****************************************************************************/
static uint32_t hb_pub_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_HB_PUBLICATION_RECORD == entry_id.record);

    if (&m_heartbeat_publication != (heartbeat_publication_state_t *)p_entry)
    {
        m_heartbeat_publication = *(heartbeat_publication_state_t *)p_entry;
    }

    return NRF_SUCCESS;
}

static void hb_pub_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_HB_PUBLICATION_RECORD == entry_id.record);

    if (&m_heartbeat_publication != (heartbeat_publication_state_t *)p_entry)
    {
        *(heartbeat_publication_state_t *)p_entry = m_heartbeat_publication;
    }
}

MESH_CONFIG_ENTRY(heartbeat_publication,
                  MESH_OPT_CORE_HB_PUBLICATION_EID,
                  1,
                  sizeof(heartbeat_publication_state_t),
                  hb_pub_setter,
                  hb_pub_getter,
                  NULL,
                  &m_heartbeat_publication);

/*****************************************************************************
 * Public API
 *****************************************************************************/

void heartbeat_init(void)
{
    // Note: Initialization of subscription state and publication state is not required. They are
    // static globals and hence reset to zero on boot up. They will be loaded with appropriate
    // values by the DSM, if they had been stored before previous power down.

    // Initialization should occur only once
    NRF_MESH_ASSERT(m_heartbeat_init_done == false);

    // register transport event handler to handle heartbeat message opcode
    NRF_MESH_ASSERT(transport_control_packet_consumer_add(&cp_handler, 1) == NRF_SUCCESS);

    m_hb_pending_pub_msg = false;

    // Initialize timer event structures
    m_publication_timer.timer.cb = heartbeat_publication_timer_cb;

    memset(&m_subscription_timer, 0, sizeof(m_subscription_timer));
    m_subscription_timer.cb        = heartbeat_subscription_timer_cb;
    m_subscription_timer.p_context = &m_heartbeat_subscription.period;

    // Remaining subscription period needs to be reported by heartbeat module as a part of
    // HEARTBEAT SUBSCRIPTION GET message. Also, incoming heartbeat messages should not be accepted
    // once this period runs out. Since the period is specified in seconds, the timer interval
    // of 1 second is good enough resolution.
    m_subscription_timer.interval  = SEC_TO_US(HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S);

    m_hb_core_evt_handler.evt_cb = heartbeat_core_evt_cb;

    m_heartbeat_init_done = true;
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

    //@todo: future: For GATT proxy: Configure dst filters
    //@todo: future: For friendship: Configure heartbeat if DST is for friend LPN node

    return NRF_SUCCESS;
}

const heartbeat_subscription_state_t * heartbeat_subscription_get(void)
{
    return &m_heartbeat_subscription;
}

heartbeat_publication_state_t * heartbeat_publication_get(void)
{
    return &m_heartbeat_publication;
}

void heartbeat_publication_state_updated(void)
{
    // Either abort existing publication timer, or schedule if required
    if (m_heartbeat_publication.dst == NRF_MESH_ADDR_UNASSIGNED ||
        m_heartbeat_publication.count == 0 ||
        m_heartbeat_publication.period == 0)
    {
        timer_sch_abort(&m_publication_timer.timer);
    }
    else if (is_publication_running())
    {
        heartbeat_restart_publication();
        // Set internal flag to trigger heartbeat on next tx complete event
        event_handler_add(&m_hb_core_evt_handler);
        m_hb_pending_pub_msg = true;
    }

    mesh_config_entry_id_t id = MESH_OPT_CORE_HB_PUBLICATION_EID;
    NRF_MESH_ASSERT(NRF_SUCCESS == mesh_config_entry_set(id, &m_heartbeat_publication));
}

void heartbeat_public_info_getter_register(hb_pub_info_getter_t p_getter)
{
    NRF_MESH_ASSERT(NULL != (mp_getter = p_getter));
}
