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
#include "broadcast.h"
#include "radio_config.h"
#include "packet.h"
#include "debug_pins.h"
#include "timeslot.h"
#include "nrf_mesh_config_bearer.h"
#include "nrf_mesh_assert.h"
#include "mesh_pa_lna_internal.h"

#ifndef BROADCAST_DEBUG
#define BROADCAST_DEBUG 0
#endif

#define USR_SOFTWARE_OVERHEAD_US (APPLICATION_TX_COMPLETE_OVERHEAD_US)

/* Measured software overhead */
#if defined(NRF51)
#define BROADCAST_START_OVERHEAD_US 23 /**< Time spent setting up the radio */
#define BROADCAST_RADIO_IRQ_OVERHEAD_US 17 /**< Delay from radio HW event to the broadcast interrupt handler. */
#define BROADCAST_END_OVERHEAD_US 19 /**< Overhead while processing the end of the advertisement, not including application callback. */
#elif defined(NRF52_SERIES)
#define BROADCAST_START_OVERHEAD_US 9 /**< Time spent setting up the radio */
#define BROADCAST_RADIO_IRQ_OVERHEAD_US 7 /**< Delay from radio HW event to the broadcast interrupt handler. */
#define BROADCAST_END_OVERHEAD_US 6 /**< Overhead while processing the end of the advertisement, not including application callback. */
#else
#error "Device must be one of NRF51 or NRF52."
#endif
#define BROADCAST_TIMESLOT_EXTRA_BUFFER_US 50 /* Ask for 50 extra us to account for IRQ locks. */

#define BROADCAST_SOFTWARE_OVERHEAD_US (BROADCAST_START_OVERHEAD_US + \
                                        BROADCAST_RADIO_IRQ_OVERHEAD_US + \
                                        BROADCAST_END_OVERHEAD_US + \
                                        BROADCAST_TIMESLOT_EXTRA_BUFFER_US)

/* According to nRF52 user spec: "For all modes that can be specified in the MODE register, except for MODE = Ble_2Mbit (see below), the
PREAMBLE is one byte long...For MODE = Ble_2Mbit the PREAMBLE has to be set to 2 byte long through the PLEN field in the PCNF0
register" */
#if defined(NRF51)
#define RADIO_DISABLE_TO_DISABLED_DELAY_US 4
#define PA_SETUP_OVERHEAD_US 6
#elif defined(NRF52_SERIES)
#define RADIO_PREAMBLE_LENGTH_2MBIT_EXTRA_BYTES 1
#define RADIO_DISABLE_TO_DISABLED_DELAY_US 6
#define PA_SETUP_OVERHEAD_US 1
#endif

#define BROADCAST_TIMER_INDEX_TIMESTAMP (0)
#define BROADCAST_TIMER_INDEX_PA_SETUP  (1)
#define BROADCAST_PPI_CH (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO)

static uint8_t m_next_channel_index;
static timestamp_t m_action_start_time; /**< Start of the action (in device time, not timeslot time) */

static void configure_timer_capture(void)
{
    /* Capture the timestamp when the packet address has been sent, to use in reporting. */
    NRF_PPI->CH[BROADCAST_PPI_CH].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[BROADCAST_PPI_CH].TEP = (uint32_t) &BEARER_ACTION_TIMER->TASKS_CAPTURE[BROADCAST_TIMER_INDEX_TIMESTAMP];
    NRF_PPI->CHENSET = (1UL << BROADCAST_PPI_CH);
}

static inline void configure_next_channel(const broadcast_params_t * p_params)
{
    radio_config_channel_set(p_params->p_channels[m_next_channel_index]);
}

static inline void prepare_last_tx(void)
{
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                         RADIO_SHORTS_END_DISABLE_Msk);
    mesh_pa_lna_setup_stop();
}

/* Start of the alloted time slice for the broadcast event */
static void broadcast_start(ts_timestamp_t start_time, void* p_args)
{
    DEBUG_PIN_BROADCAST_ON(DEBUG_PIN_BROADCAST_START);
    DEBUG_PIN_BROADCAST_ON(DEBUG_PIN_BROADCAST_ACTIVE);
    broadcast_t * p_broadcast = (broadcast_t *) p_args;
    configure_timer_capture();

    radio_config_reset();
    radio_config_config(&p_broadcast->params.radio_config);
    radio_config_access_addr_set(p_broadcast->params.access_address, 0);
    radio_config_channel_set(p_broadcast->params.p_channels[0]);
    m_next_channel_index = 1;
    m_action_start_time = ts_timer_to_device_time(start_time);
    NRF_RADIO->PACKETPTR = (uint32_t) p_broadcast->params.p_packet;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;


    /* To set the power amplifier timer, we capture the current timestamp and move it to start some
     * time before the radio starts sending. As this code runs in the highest interrupt level, the
     * time between the timer capture and the TXEN trigger will be deterministic.
     */
    BEARER_ACTION_TIMER->TASKS_CAPTURE[BROADCAST_TIMER_INDEX_PA_SETUP] = 1;
    NRF_RADIO->TASKS_TXEN = 1;

    mesh_pa_setup_start(BEARER_ACTION_TIMER->CC[BROADCAST_TIMER_INDEX_PA_SETUP] + PA_SETUP_OVERHEAD_US,
                        BROADCAST_TIMER_INDEX_PA_SETUP);

    if (p_broadcast->params.channel_count > 1)
    {
        /* Fly through DISABLED state right into TXRU after transmission ends */
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
                            RADIO_SHORTS_END_DISABLE_Msk |
                            RADIO_SHORTS_DISABLED_TXEN_Msk;

        /* Can configure the next channel now, as the radio makes the decision at TXEN. */
        configure_next_channel(&p_broadcast->params);
    }
    else
    {
        /* Stop in disabled state after this transmit. */
        prepare_last_tx();
    }

    DEBUG_PIN_BROADCAST_OFF(DEBUG_PIN_BROADCAST_START);
}

static inline void tx_complete_notify_user(broadcast_t * p_broadcast)
{
    const timestamp_t tx_timestamp = m_action_start_time + BEARER_ACTION_TIMER->CC[BROADCAST_TIMER_INDEX_TIMESTAMP];
    const ts_timestamp_t user_cb_time_start = ts_timer_now();
    p_broadcast->params.tx_complete_cb(&p_broadcast->params, tx_timestamp);
    const ts_timestamp_t user_cb_time_end = ts_timer_now();
#if BROADCAST_DEBUG
    p_broadcast->debug.prev_tx_complete_app_time_us = user_cb_time_end - user_cb_time_start;
#endif
    NRF_MESH_ASSERT(TIMER_OLDER_THAN(user_cb_time_end, user_cb_time_start + USR_SOFTWARE_OVERHEAD_US));
}
static inline void end_action(broadcast_t * p_broadcast)
{
    DEBUG_PIN_BROADCAST_OFF(DEBUG_PIN_BROADCAST_ACTIVE);
    mesh_pa_lna_cleanup();
    bearer_handler_action_end();
    p_broadcast->active = false;
}

static void radio_irq_handler(void* p_args)
{
    DEBUG_PIN_BROADCAST_ON(DEBUG_PIN_BROADCAST_RADIO_EVT);

    broadcast_t * p_broadcast = (broadcast_t *) p_args;

    /* We set the interrupt to only trigger on the disabled-event */
    NRF_MESH_ASSERT(NRF_RADIO->EVENTS_DISABLED);
    NRF_RADIO->EVENTS_DISABLED = 0;
    m_next_channel_index++;

    mesh_pa_lna_disable_start();

    if (m_next_channel_index > p_broadcast->params.channel_count)
    {
        tx_complete_notify_user(p_broadcast);
        end_action(p_broadcast);
    }
    else
    {
        /* Configure the next channel to send on, or enable end event if this is the last channel. */
        if (m_next_channel_index < p_broadcast->params.channel_count)
        {
            configure_next_channel(&p_broadcast->params);
        }
        else
        {
            /* We're about to do the last transmission */
            prepare_last_tx();
        }
        /* If we've been through disabled again, we probably didn't change channel in time due to
         * CPU blocking. In theory, the radio might do several loops here without us noticing
         * with this check, but if this happens, we'll fail the event duration check in the bearer
         * handler. */
        NRF_MESH_ASSERT(!NRF_RADIO->EVENTS_DISABLED);
    }

    DEBUG_PIN_BROADCAST_OFF(DEBUG_PIN_BROADCAST_RADIO_EVT);
}

static inline ts_timestamp_t time_required_to_send_us(const packet_t * p_packet, uint8_t channel_count, radio_mode_t radio_mode)
{
    static const uint8_t radio_mode_to_us_per_byte[RADIO_MODE_END] =  {8, 4, 32, 8
                                                            #ifdef NRF52_SERIES
                                                                           ,4, 128
                                                            #endif
                                                                           };
    uint32_t packet_length_in_bytes = BLE_PACKET_OVERHEAD(RADIO_MODE_BLE_1MBIT) + p_packet->header.length;
#ifdef NRF52_SERIES
    if (radio_mode == RADIO_MODE_BLE_2MBIT)
    {
        packet_length_in_bytes += RADIO_PREAMBLE_LENGTH_2MBIT_EXTRA_BYTES;
    }
#endif
    uint32_t radio_time_per_channel = RADIO_RAMPUP_TIME +
                                      (packet_length_in_bytes * radio_mode_to_us_per_byte[radio_mode]) +
                                      RADIO_DISABLE_TO_DISABLED_DELAY_US;

    return (BROADCAST_SOFTWARE_OVERHEAD_US + USR_SOFTWARE_OVERHEAD_US + radio_time_per_channel * channel_count);
}

uint32_t broadcast_send(broadcast_t * p_broadcast)
{
    NRF_MESH_ASSERT(p_broadcast->params.tx_complete_cb != NULL);
    if (p_broadcast->active)
    {
        return NRF_ERROR_BUSY;
    }

    p_broadcast->action.start_cb = broadcast_start;
    p_broadcast->action.radio_irq_handler = radio_irq_handler;
    p_broadcast->action.duration_us =
        time_required_to_send_us(p_broadcast->params.p_packet,
                                 p_broadcast->params.channel_count,
                                 p_broadcast->params.radio_config.radio_mode);
    p_broadcast->action.p_args = p_broadcast;
    p_broadcast->active = true;
    NRF_MESH_ASSERT(NRF_SUCCESS == bearer_handler_action_enqueue(&p_broadcast->action));
    return NRF_SUCCESS;
}
