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

#include "instaburst_rx.h"
#include "scanner.h"
#include "adv_ext_packet.h"
#include "debug_pins.h"
#include "utils.h"
#include "timeslot.h"
#include "packet_buffer.h"
#include "bearer_handler.h"
#include "instaburst_internal.h"
#include "nrf_mesh_config_bearer.h"
#include "mesh_pa_lna_internal.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

#define M_TIMER_INDEX_RADIO_START 0
#define M_TIMER_INDEX_RADIO_DISABLE  1
#define M_TIMER_INDEX_LNA  2
#define M_TIMER_INDEX_ADDR_TIMESTAMP   M_TIMER_INDEX_RADIO_DISABLE /**< Timestamp index must be the same as the stop index, to disable the stop when a packet is received. */
/* PPI indexes used for radio timing: */
#define PPI_INDEX_RADIO_START           (TS_TIMER_PPI_CH_START)
#define PPI_INDEX_TIMESTAMP             (TS_TIMER_PPI_CH_START + 1)
#define PPI_INDEX_RADIO_DISABLE         (TS_TIMER_PPI_CH_START + 2)

/* Debug PPI indexes */
#define PPI_INDEX_DEBUG_READY 2
#define PPI_INDEX_DEBUG_TIMEOUT 2

#define RX_POST_PROCESS_US     (100) /**< Time spent processing after receiving an auxiliary packet. */
#define RX_OFFSET_TIME_MAX_US  (3000) /**< Maximum acceptable RX offset time in an AuxPtr. */
#define START_TIME_OVERHEAD_US (40 + RADIO_RAMPUP_TIME) /**< Expected amount of time to pass from starting the action until the radio is ready */


/**
 * Margin on both sides of the receive window in case of timer drift.
 */
#define AUX_PACKET_RX_MARGIN_US 10
/*****************************************************************************
* Local types
*****************************************************************************/
typedef enum
{
    INSTABURST_RX_STATE_UNINITIALIZED,
    INSTABURST_RX_STATE_IDLE,
    INSTABURST_RX_STATE_RX_ADV_EXT,
} instaburst_rx_state_t;

typedef struct
{
    instaburst_rx_state_t state;
    packet_buffer_t packet_buffer;
    uint8_t buffer_array[INSTABURST_RX_BUFFER_SIZE] __attribute__((aligned(WORD_SIZE)));

    packet_buffer_packet_t * p_rx_buf;

    /** Data for the current extended advertising event. */
    struct
    {
        nrf_mesh_instaburst_event_id_t id; /**< Event ID for the current event. */
        ts_timestamp_t action_start_time; /**< Timestamp marking when the action started. */
        /** Data for the current packet in the current advertising event. */
        struct
        {
            ts_timestamp_t start_time; /**< Time to start receiving the packet */
            uint32_t timeslot_index; /**< Index of the timeslot when the packet starts */
            adv_ext_time_offset_unit_t offset_unit;
            uint8_t channel;
            uint8_t index_in_event;
            radio_mode_t radio_mode;
        } packet;
    } event;

    bearer_action_t bearer_action;
    bearer_event_flag_t process_flag;

#if INSTABURST_RX_DEBUG
    instaburst_rx_stats_t stats[INSTABURST_CHANNEL_INDEX_MAX + 1];
#endif
} instaburst_rx_t;

/**
 * Internal representation of a single received instaburst packet. Contains the actual
 * data buffer that the public representation is pointing into.
 */
typedef struct
{
    instaburst_rx_packet_t packet; /**< Public representation of the instaburst rx packet. */
    adv_ext_packet_t buffer; /**< Packet buffer containing the received on-air packet. */
} instaburst_rx_packet_internal_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static instaburst_rx_t m_instaburst;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void order_rx(const adv_ext_header_aux_ptr_t * p_aux_ptr)
{
    /* Offset is given in units, and the actual packet start can come anywhere in the given unit-step: */
    uint32_t max_offset_us = p_aux_ptr->time_offset_us + p_aux_ptr->offset_units_us;
    /** Account for drift on both sides of the transaction: */
    uint32_t max_drift_us = ((BEARER_HFCLK_DRIFT_PPM_MAX + adv_ext_header_clock_drift_max(p_aux_ptr->clock_accuracy)) * max_offset_us) / 1000000;
    uint32_t packet_len = BLE_PACKET_OVERHEAD(m_instaburst.event.packet.radio_mode) + ADV_EXT_PACKET_LEN_MAX;
    uint32_t packet_duration = packet_len * RADIO_TIME_PER_BYTE(m_instaburst.event.packet.radio_mode);

    m_instaburst.bearer_action.duration_us = (max_offset_us + max_drift_us + packet_duration + RX_POST_PROCESS_US);

    if (bearer_handler_action_fire(&m_instaburst.bearer_action) == NRF_SUCCESS)
    {
        m_instaburst.state = INSTABURST_RX_STATE_RX_ADV_EXT;
    }
    else
    {
#if INSTABURST_RX_DEBUG
        m_instaburst.stats[p_aux_ptr->channel_index].busy++;
#endif
    }
}

static inline uint32_t rx_buffer_len_get(uint8_t data_len)
{
    return offsetof(instaburst_rx_packet_internal_t, buffer.data) + data_len;
}

static void aux_ptr_handle(const adv_ext_header_aux_ptr_t * p_aux_ptr, ts_timestamp_t packet_address_timestamp, radio_mode_t rx_radio_mode)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_GOT_ADV_EXT);

    if (p_aux_ptr->time_offset_us < RX_OFFSET_TIME_MAX_US)
    {
        m_instaburst.event.packet.radio_mode  = p_aux_ptr->radio_mode;
        m_instaburst.event.packet.offset_unit = p_aux_ptr->offset_units_us;
        /* Start time, calculated from preamble start on the packet containing the aux ptr to
         * preamble start on the packet it's pointing to. */
        m_instaburst.event.packet.start_time = packet_address_timestamp + p_aux_ptr->time_offset_us -
                                               RADIO_ADDR_EVT_DELAY(rx_radio_mode);
        m_instaburst.event.packet.timeslot_index = timeslot_count_get();
        m_instaburst.event.packet.channel = p_aux_ptr->channel_index;

        order_rx(p_aux_ptr);
    }
    else
    {
#if INSTABURST_RX_DEBUG
        m_instaburst.stats[p_aux_ptr->channel_index].invalid_offset++;
#endif
    }
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_GOT_ADV_EXT);
}

static void scanner_rx_cb(const scanner_packet_t * p_scanner_packet, ts_timestamp_t rx_timestamp_ts)
{
    if (m_instaburst.state == INSTABURST_RX_STATE_IDLE &&
        p_scanner_packet->packet.header.type == BLE_PACKET_TYPE_ADV_EXT &&
        p_scanner_packet->packet.header.length >= sizeof(adv_ext_header_t))
    {
        adv_ext_packet_t * p_packet = (adv_ext_packet_t *) &p_scanner_packet->packet;
        const adv_ext_header_t * p_header = (adv_ext_header_t *) &p_packet->data[0];

        adv_ext_header_bitfield_t fields = adv_ext_header_bitfield_get(p_header);
        adv_ext_header_bitfield_t required_fields = (adv_ext_header_bitfield_t) (ADV_EXT_HEADER_AUX_PTR_BIT | ADV_EXT_HEADER_ADI_BIT);

        /* Fetch required extended advertising fields and order the event */
        if (p_scanner_packet->packet.header.length >= sizeof(adv_ext_header_t) + adv_ext_header_len(fields) &&
            (fields & required_fields) == required_fields)
        {
            if (adv_ext_header_data_get(p_header,
                                        ADV_EXT_HEADER_ADI,
                                        (adv_ext_header_data_t *) &m_instaburst.event.id) ==
                NRF_SUCCESS)
            {

                if (!instaburst_event_id_cache_has_value(&m_instaburst.event.id))
                {
                    m_instaburst.event.packet.index_in_event = 0;

                    adv_ext_header_aux_ptr_t aux_ptr;
                    if (adv_ext_header_data_get(p_header,
                                                ADV_EXT_HEADER_AUX_PTR,
                                                (adv_ext_header_data_t *) &aux_ptr) == NRF_SUCCESS)
                    {
                        aux_ptr_handle(&aux_ptr,
                                       rx_timestamp_ts,
                                       RADIO_MODE_BLE_1MBIT);
                    }
                }
            }
        }
    }
}

static void action_start(ts_timestamp_t start_time, void * p_args)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_START);
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_IN_ACTION);

    m_instaburst.event.action_start_time = start_time;

    /* The aux ptr timestamp is only valid within the timeslot it was captured in, as the ts_timer
     * is reset at the beginning of every timeslot. We should abort the action if we're in a
     * different timeslot than we were when we captured the timestamp, as the reference is no longer
     * valid. */
    bool in_same_timeslot = (m_instaburst.event.packet.timeslot_index == timeslot_count_get());

    if (in_same_timeslot &&
        packet_buffer_reserve(&m_instaburst.packet_buffer,
                              &m_instaburst.p_rx_buf,
                              rx_buffer_len_get(ADV_EXT_PACKET_LEN_MAX)) == NRF_SUCCESS)
    {
        const radio_config_t radio_config = {.radio_mode     = m_instaburst.event.packet.radio_mode,
                                             .payload_maxlen = ADV_EXT_PACKET_LEN_MAX};

        radio_config_reset();
        radio_config_config(&radio_config);
        radio_config_channel_set(m_instaburst.event.packet.channel);
        radio_config_access_addr_set(BEARER_ACCESS_ADDR_DEFAULT, 0);

        NRF_RADIO->RXADDRESSES = (1 << 0);

        NRF_RADIO->PACKETPTR =
            (uint32_t) &((instaburst_rx_packet_internal_t *) m_instaburst.p_rx_buf->packet)->buffer;
        NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
                             RADIO_SHORTS_END_DISABLE_Msk);

        NRF_RADIO->EVENTS_DISABLED = 0;
        (void) NRF_RADIO->EVENTS_DISABLED;
        NRF_RADIO->EVENTS_END = 0;
        (void) NRF_RADIO->EVENTS_END;
        NRF_RADIO->EVENTS_READY = 0;
        (void) NRF_RADIO->EVENTS_READY;
        NRF_RADIO->EVENTS_ADDRESS = 0;
        (void) NRF_RADIO->EVENTS_ADDRESS;

        NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

        if (TIMER_OLDER_THAN(m_instaburst.event.packet.start_time, ts_timer_now() + START_TIME_OVERHEAD_US))
        {
            /* Can't make it in time */
            DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_TOO_SLOW);

            m_instaburst.state = INSTABURST_RX_STATE_IDLE;
            packet_buffer_free(&m_instaburst.packet_buffer, m_instaburst.p_rx_buf);

#if INSTABURST_RX_DEBUG
            m_instaburst.stats[m_instaburst.event.packet.channel].too_late++;
#endif

            DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_TOO_SLOW);
            DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_IN_ACTION);
            bearer_handler_action_end();
        }
        else
        {
            /* We need a total of 3 PPIs to run the radio RX timing:
             * - One to start the radio when the timer expires
             * - One to stop the radio if it couldn't receive anything
             * - One to cancel the stop-timer by sampling it (and timestamp the packet).
             */

            /* Start ramping up the radio in time to go active right before the earliest announced time. */
            ts_timestamp_t rampup_start_time = m_instaburst.event.packet.start_time - RADIO_RAMPUP_TIME - AUX_PACKET_RX_MARGIN_US;

            /* According to the Bluetooth 5.0 specification, the sender of an extended advertisement
             * event is allowed to start sending anywhere inside the indicated timestep,
             * i.e. from start_time to start_time + offset_unit. Account for this, as well as for
             * the delay of the ADDR event and a safety margin.
             */
            ts_timestamp_t last_addr_time =
                m_instaburst.event.packet.start_time + m_instaburst.event.packet.offset_unit +
                RADIO_ADDR_EVT_DELAY(m_instaburst.event.packet.radio_mode) +
                AUX_PACKET_RX_MARGIN_US;

            BEARER_ACTION_TIMER->CC[M_TIMER_INDEX_RADIO_START]   = rampup_start_time - m_instaburst.event.action_start_time;
            BEARER_ACTION_TIMER->CC[M_TIMER_INDEX_RADIO_DISABLE] = last_addr_time - m_instaburst.event.action_start_time;

            NRF_PPI->CH[PPI_INDEX_RADIO_START].EEP = (uint32_t) &BEARER_ACTION_TIMER->EVENTS_COMPARE[M_TIMER_INDEX_RADIO_START];
            NRF_PPI->CH[PPI_INDEX_RADIO_START].TEP = (uint32_t) &NRF_RADIO->TASKS_RXEN;

            NRF_PPI->CH[PPI_INDEX_RADIO_DISABLE].EEP = (uint32_t) &BEARER_ACTION_TIMER->EVENTS_COMPARE[M_TIMER_INDEX_RADIO_DISABLE];
            NRF_PPI->CH[PPI_INDEX_RADIO_DISABLE].TEP = (uint32_t) &NRF_RADIO->TASKS_DISABLE;

            /* Sampling the timer will annul the CC value previously setup to cancel the radio, as
             * it overwrites the register. By setting a PPI from the ADDRESS event to the
             * timer capture, we make the timer overwrite its own CC register when the packet is
             * successfully received, preventing the timer from firing. */
            NRF_PPI->CH[PPI_INDEX_TIMESTAMP].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
            NRF_PPI->CH[PPI_INDEX_TIMESTAMP].TEP = (uint32_t) &BEARER_ACTION_TIMER->TASKS_CAPTURE[M_TIMER_INDEX_ADDR_TIMESTAMP];

            NRF_PPI->CHENSET = (1UL << PPI_INDEX_RADIO_START) |
                               (1UL << PPI_INDEX_RADIO_DISABLE) |
                               (1UL << PPI_INDEX_TIMESTAMP);

            mesh_lna_setup_start(rampup_start_time - m_instaburst.event.action_start_time, M_TIMER_INDEX_LNA);
            mesh_pa_lna_setup_stop();

            /* Ensure that the timer order above was scheduled in time. If this fails, we were
             * locking IRQs for too long, delaying packet processing. */
            NRF_MESH_ASSERT(TIMER_OLDER_THAN(ts_timer_now(), rampup_start_time));
        }
    }
    else
    {
#if INSTABURST_RX_DEBUG
        if (!in_same_timeslot)
        {
            m_instaburst.stats[m_instaburst.event.packet.channel].switched_timeslot++;
        }
#endif
        m_instaburst.state = INSTABURST_RX_STATE_IDLE;
        DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_IN_ACTION);
        bearer_handler_action_end();
    }
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_START);
}

static void radio_irq_handler(void * p_args)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_RADIO_EVT);

    NRF_MESH_ASSERT(NRF_RADIO->EVENTS_DISABLED);
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_MESH_ASSERT(m_instaburst.p_rx_buf != NULL);
    instaburst_rx_packet_internal_t * p_packet_buf = (instaburst_rx_packet_internal_t *) m_instaburst.p_rx_buf->packet;

    bool has_more = false;
    adv_ext_header_aux_ptr_t aux_ptr;

    if (NRF_RADIO->EVENTS_END)
    {
        if (NRF_RADIO->CRCSTATUS)
        {
            DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_RX_OK);

            /* Look for an aux pointer, in case there are chained packets */
            has_more = (adv_ext_header_data_get((adv_ext_header_t *) p_packet_buf->buffer.data,
                                                ADV_EXT_HEADER_AUX_PTR,
                                                (adv_ext_header_data_t *) &aux_ptr) == NRF_SUCCESS);


            /* Finalize packet structure */
            p_packet_buf->packet.metadata.event.is_last_in_chain = !has_more;
            p_packet_buf->packet.metadata.channel                = m_instaburst.event.packet.channel;
            p_packet_buf->packet.metadata.rssi                   = -((int32_t) NRF_RADIO->RSSISAMPLE);
            p_packet_buf->packet.metadata.timestamp              = m_instaburst.event.packet.start_time;
            p_packet_buf->packet.metadata.event.id               = m_instaburst.event.id;
            p_packet_buf->packet.metadata.event.packet_index     = m_instaburst.event.packet.index_in_event++;

            p_packet_buf->packet.p_payload = adv_ext_packet_adv_data_get(&p_packet_buf->buffer);
            if (p_packet_buf->packet.p_payload == NULL)
            {
                p_packet_buf->packet.payload_len = 0;
            }
            else
            {
                uint8_t payload_offset = p_packet_buf->packet.p_payload - &p_packet_buf->buffer.data[0];
                p_packet_buf->packet.payload_len = p_packet_buf->buffer.header.length - payload_offset;
            }

            packet_buffer_commit(&m_instaburst.packet_buffer,
                                 m_instaburst.p_rx_buf,
                                 rx_buffer_len_get(p_packet_buf->buffer.header.length));

            bearer_event_flag_set(m_instaburst.process_flag);
#if INSTABURST_RX_DEBUG
            m_instaburst.stats[m_instaburst.event.packet.channel].rx_ok++;
#endif
            DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_RX_OK);
        }
        else
        {
#if INSTABURST_RX_DEBUG
            m_instaburst.stats[m_instaburst.event.packet.channel].crc_fail++;
#endif
            packet_buffer_free(&m_instaburst.packet_buffer, m_instaburst.p_rx_buf);
        }
    }
    else
    {
#if INSTABURST_RX_DEBUG
        m_instaburst.stats[m_instaburst.event.packet.channel].no_rx++;
#endif
        packet_buffer_free(&m_instaburst.packet_buffer, m_instaburst.p_rx_buf);
    }

    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_IN_ACTION);
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_RADIO_EVT);
    m_instaburst.state = INSTABURST_RX_STATE_IDLE;
    m_instaburst.p_rx_buf = NULL;

    NRF_PPI->CHENCLR = (1UL << PPI_INDEX_RADIO_START) |
                       (1UL << PPI_INDEX_RADIO_DISABLE) |
                       (1UL << PPI_INDEX_TIMESTAMP);

    mesh_pa_lna_cleanup();

    bearer_handler_action_end();

    if (has_more)
    {
        ts_timestamp_t addr_rx_timestamp = m_instaburst.event.action_start_time + BEARER_ACTION_TIMER->CC[M_TIMER_INDEX_ADDR_TIMESTAMP];

        aux_ptr_handle(&aux_ptr, addr_rx_timestamp, m_instaburst.event.packet.radio_mode);
    }
    else
    {
        instaburst_event_id_cache_put(&m_instaburst.event.id);
    }
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void instaburst_rx_init(bearer_event_flag_callback_t packet_process_cb)
{
    packet_buffer_init(&m_instaburst.packet_buffer,
                       m_instaburst.buffer_array,
                       INSTABURST_RX_BUFFER_SIZE);

    m_instaburst.bearer_action.p_args = &m_instaburst;
    m_instaburst.bearer_action.start_cb = action_start;
    m_instaburst.bearer_action.radio_irq_handler = radio_irq_handler;

    m_instaburst.process_flag = bearer_event_flag_add(packet_process_cb);
    m_instaburst.state = INSTABURST_RX_STATE_IDLE;
}

void instaburst_rx_enable(void)
{
    scanner_rx_callback_set(scanner_rx_cb);
}

void instaburst_rx_disable(void)
{
    scanner_rx_callback_set(NULL);
}

const instaburst_rx_packet_t * instaburst_rx(void)
{
    packet_buffer_packet_t * p_packet;
    if (packet_buffer_pop(&m_instaburst.packet_buffer, &p_packet) == NRF_SUCCESS)
    {
        return (instaburst_rx_packet_t *) p_packet->packet;
    }
    return NULL;
}

bool instaburst_rx_pending(void)
{
    return packet_buffer_can_pop(&m_instaburst.packet_buffer);
}

void instaburst_rx_packet_release(const instaburst_rx_packet_t * p_packet)
{
    packet_buffer_free(&m_instaburst.packet_buffer,
                       PARENT_BY_FIELD_GET(packet_buffer_packet_t, packet, p_packet));
}

const instaburst_rx_stats_t * instaburst_rx_stats_get(uint8_t channel)
{
#if INSTABURST_RX_DEBUG
    if (channel <= INSTABURST_CHANNEL_INDEX_MAX)
    {
        return &m_instaburst.stats[channel];
    }
    else
    {
        return NULL;
    }
#else
    return NULL;
#endif
}
