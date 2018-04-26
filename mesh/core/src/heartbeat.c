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

//@todo:  How frequently should we store heartbeat states in the flash? Need algorithm,
//        or hardware solution, as discussed.
//@todo:  Load heartbeat states on powerup from flash, if already stored.

/*****************************************************************************
* Local defines
*****************************************************************************/
#define HEARTBEAT_MIN_HOPS_INIT  (0x7F)
#define HEARTBEAT_MAX_HOPS_INIT  (0x00)

#define HEARTBEAT_PUBLISH_SUB_INTERVAL_S (1800)
#define HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S (1)

/** Initialization flag to prevent multiple initializations of the module */
static bool m_heartbeat_init_done;

/** Instantiation of Heartbeat subscription state */
static heartbeat_subscription_state_t m_heartbeat_subscription;

/** flag to schedule one off message trigger */
static bool m_hb_pending_pub_msg;
static bool m_hb_pending_feat_msg;


/** Internal timer */
static struct
{
    uint32_t remaining_time_s;
    timer_event_t timer;
} m_publication_timer;

static timer_event_t m_subscription_timer;

/** Handle core events here */
static nrf_mesh_evt_handler_t m_hb_core_evt_handler;

/** Forward declarations */
static uint32_t heartbeat_send(heartbeat_publication_information_t * p_hb_pub_info);
static void heartbeat_subscription_timer_cb(timestamp_t timestamp, void * p_context);
static void heartbeat_publication_timer_cb(timestamp_t timestamp, void * p_context);
static void heartbeat_opcode_handle(const transport_control_packet_t * p_control_packet,
                                    const nrf_mesh_rx_metadata_t *     p_rx_metadata);

/** Transport handler for the Heartbeat opcode */
static const transport_control_packet_handler_t cp_handler = {
    .opcode = TRANSPORT_CONTROL_OPCODE_HEARTBEAT, .callback = heartbeat_opcode_handle
};

/** Holds the pointer to callbacks defined in config_server */
static heartbeat_publication_params_get_cb_t   m_publication_get_cb;
static heartbeat_publication_count_decrement_cb_t m_publication_count_decrement_cb;

/*****************************************************************************
 * Static internal functions
 *****************************************************************************/


static inline bool is_publication_running(heartbeat_publication_information_t * p_pub_info)
{
    return ((p_pub_info->p_publication->dst   != NRF_MESH_ADDR_UNASSIGNED) &&
            (p_pub_info->p_publication->count != 0x0000)
           );
}

static inline bool can_publish_feature_change(heartbeat_publication_information_t * p_pub_info)
{
    return ((p_pub_info->p_publication->dst   != NRF_MESH_ADDR_UNASSIGNED));
}

static void heartbeat_restart_publication(heartbeat_publication_information_t * p_pub_info)
{
    m_publication_timer.remaining_time_s = p_pub_info->p_publication->period;
    uint32_t next_timeout = MIN(p_pub_info->p_publication->period, HEARTBEAT_PUBLISH_SUB_INTERVAL_S);

    timer_sch_reschedule(&m_publication_timer.timer,
                         timer_now() + SEC_TO_US(next_timeout));
}

/*****************************************************************************/
/* << RX path functions >> */

/** Sends the heartbeat packet to heartbeat module for processing. This function is called by the
 * @ref transport_control_packet_in
 */
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

/** Prepares the transport control packet metadata to send the heartbeat message */
static void heartbeat_meta_prepare(transport_control_packet_t *             p_tx,
                                   const packet_mesh_trs_control_packet_t * p_pdu,
                                   uint8_t                                  pdu_len,
                                   heartbeat_publication_information_t    * p_pub_info)
{
    // Network security material must be always valid. This should never assert.
    NRF_MESH_ASSERT(p_pub_info->p_net_secmat != NULL);

    p_tx->opcode       = TRANSPORT_CONTROL_OPCODE_HEARTBEAT;
    p_tx->reliable     = false;
    p_tx->src          = p_pub_info->local_address;
    p_tx->dst.value    = p_pub_info->p_publication->dst;
    p_tx->dst.type     = nrf_mesh_address_type_get(p_pub_info->p_publication->dst);
    p_tx->p_net_secmat = p_pub_info->p_net_secmat;
    p_tx->p_data       = p_pdu;
    p_tx->data_len     = pdu_len;
    p_tx->ttl          = p_pub_info->p_publication->ttl;
}

/** Sends a heartbeat message when timer expires or when triggered */
static uint32_t heartbeat_send(heartbeat_publication_information_t * p_hb_pub_info)
{
    transport_control_packet_t       tx_params;
    packet_mesh_trs_control_packet_t hb_pdu;
    uint8_t                          active_features = 0;
    nrf_mesh_opt_t                   param_value;

    // This function should not be called if count have reached to zero, or no feature based publishing is enabled
    NRF_MESH_ASSERT(p_hb_pub_info->p_publication->count != 0 || p_hb_pub_info->p_publication->features);

    if (p_hb_pub_info->p_publication->features & HEARTBEAT_TRIGGER_TYPE_LPN)
    {
        /* TODO: check the state of the low power feature. */
    }

    active_features <<= 1;
    if (p_hb_pub_info->p_publication->features & HEARTBEAT_TRIGGER_TYPE_FRIEND)
    {
        /* TODO: check the state of the friend feature. */
    }

    active_features <<= 1;
    if (p_hb_pub_info->p_publication->features & HEARTBEAT_TRIGGER_TYPE_PROXY)
    {
        /* TODO: check the state of the proxy feature. */
    }

    // Relay query
    active_features <<= 1;
    if (p_hb_pub_info->p_publication->features & HEARTBEAT_TRIGGER_TYPE_RELAY)
    {
        NRF_MESH_ASSERT(nrf_mesh_opt_get(NRF_MESH_OPT_NET_RELAY_ENABLE, &param_value) ==
                        NRF_SUCCESS);
        active_features |= (param_value.opt.val & 0x01);
    }

    memset(hb_pdu.pdu, 0, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE);
    packet_mesh_trs_control_heartbeat_init_ttl_set(&hb_pdu, p_hb_pub_info->p_publication->ttl);
    packet_mesh_trs_control_heartbeat_features_set(&hb_pdu, active_features);

    heartbeat_meta_prepare(&tx_params, &hb_pdu, PACKET_MESH_TRS_CONTROL_HEARTBEAT_SIZE, p_hb_pub_info);

    return (transport_control_tx(&tx_params, (nrf_mesh_tx_token_t) p_hb_pub_info->p_publication->count));
}

/** Callback for the subscription timer. this callback triggers every second. As per
 * Mesh Profile Specification v1.0, section 4.2.18.4, node shall report the remaining time
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

/** Callback for publication timer. If publication period is greater than 3600 seconds, this
 * callback triggers every 3600 seconds, else it triggers after publication period. The callback
 * parameter p_context points to the m_heartbeat_publication.period.
 */
static void heartbeat_publication_timer_cb(timestamp_t timestamp, void * p_context)
{
    heartbeat_publication_information_t     hb_pub_info;

    /* Publish heartbeat if it is time to publish. Stop sending heartbeats if m_publication_get_cb fails */
    NRF_MESH_ASSERT(m_publication_get_cb != NULL);
    if (m_publication_get_cb(&hb_pub_info) == NRF_SUCCESS)
    {
        NRF_MESH_ASSERT(m_publication_timer.remaining_time_s > 0);

        m_publication_timer.remaining_time_s -= MIN(MIN(hb_pub_info.p_publication->period, HEARTBEAT_PUBLISH_SUB_INTERVAL_S), m_publication_timer.remaining_time_s);

        if (m_publication_timer.remaining_time_s == 0 && is_publication_running(&hb_pub_info) > 0)
        {
            if (heartbeat_send(&hb_pub_info) == NRF_SUCCESS)
            {
                NRF_MESH_ASSERT(m_publication_count_decrement_cb != NULL);
                m_publication_count_decrement_cb();
            }
            m_publication_timer.remaining_time_s = hb_pub_info.p_publication->period;
        }

        if (hb_pub_info.p_publication->count > 0)
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
        heartbeat_publication_information_t     hb_pub_info;

        /* Get the latest value of the heartbeat publication information */
        NRF_MESH_ASSERT(m_publication_get_cb != NULL);
        if ((m_publication_get_cb (&hb_pub_info) == NRF_SUCCESS))  // dst and count may get modified, re-check for validity
        {
            if (m_hb_pending_pub_msg &&
                is_publication_running(&hb_pub_info))
            {
                if (heartbeat_send(&hb_pub_info) == NRF_SUCCESS)
                {
                    NRF_MESH_ASSERT(m_publication_count_decrement_cb != NULL);
                    m_publication_count_decrement_cb();
                }
            }

            if (m_hb_pending_feat_msg && can_publish_feature_change(&hb_pub_info))
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

/** Heartbeat feature change triggers heartbeat message */
void heartbeat_on_feature_change_trigger(uint16_t hb_trigger)
{
    heartbeat_publication_information_t     hb_pub_info;

    /* Get the latest value of the heartbeat publication information */
    NRF_MESH_ASSERT(m_publication_get_cb != NULL);
    if (m_publication_get_cb (&hb_pub_info) == NRF_SUCCESS)
    {
        if ((hb_pub_info.p_publication->features & hb_trigger) &&
            can_publish_feature_change(&hb_pub_info))
        {
            // Set internal flag to trigger heartbeat on next tx complete event
            event_handler_add(&m_hb_core_evt_handler);
            m_hb_pending_feat_msg = true;
        }
    }
}

/** Initializes the heartbeat module */
void heartbeat_init(void)
{
    // Note: Initialization of subscription state and publication state is not required. They are
    // static globals and hence reset to zero on boot up. They will be loaded with appropriate
    // values by the DSM, if they had been stored before previous power down.

    // Initialization should occur only once
    NRF_MESH_ASSERT(m_heartbeat_init_done == false);

    // register transport event handler to handle hearbeat message opcode
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


/** Sets the value of internal heartbeat subscription state */
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

/** Gets the value of internal heartbeat subscription state */
const heartbeat_subscription_state_t * heartbeat_subscription_get(void)
{
    return &m_heartbeat_subscription;
}

void heartbeat_config_server_cb_set(heartbeat_publication_params_get_cb_t p_cb,
                                    heartbeat_publication_count_decrement_cb_t p_pub_cnt_cb)
{
    NRF_MESH_ASSERT(p_cb != NULL);
    NRF_MESH_ASSERT(p_pub_cnt_cb != NULL);

    m_publication_get_cb = p_cb;
    m_publication_count_decrement_cb = p_pub_cnt_cb;
}

void heartbeat_publication_state_updated(void)
{
    heartbeat_publication_information_t     hb_pub_info;

    /* Get the latest value of the heartbeat publication information */
    NRF_MESH_ASSERT(m_publication_get_cb != NULL);
    if (m_publication_get_cb (&hb_pub_info) == NRF_SUCCESS)
    {
        // Either abort existing publication timer, or schedule if required
        if ((hb_pub_info.p_publication->dst == NRF_MESH_ADDR_UNASSIGNED) ||
            (hb_pub_info.p_publication->count == 0) ||
            (hb_pub_info.p_publication->period == 0)
           )
        {
            timer_sch_abort(&m_publication_timer.timer);
        }
        else if (is_publication_running(&hb_pub_info))
        {
            heartbeat_restart_publication(&hb_pub_info);
            // Set internal flag to trigger heartbeat on next tx complete event
            event_handler_add(&m_hb_core_evt_handler);
            m_hb_pending_pub_msg = true;
        }
    }
}
