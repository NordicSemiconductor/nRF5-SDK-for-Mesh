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
#include "scanner.h"
#include "timer_scheduler.h"
#include "packet_buffer.h"
#include "toolchain.h"
#include "timeslot.h"
#include "filter_engine.h"
#include "debug_pins.h"
#include "mesh_pa_lna_internal.h"
#include "nrf_mesh_config_bearer.h"
#include "bearer_handler.h"

#ifdef NRF52_SERIES
#define LNA_SETUP_OVERHEAD_US 1
#elif defined(NRF51)
#define LNA_SETUP_OVERHEAD_US 6
#endif

#define SCANNER_TIMER_INDEX_TIMESTAMP   (TS_TIMER_INDEX_RADIO)
#define SCANNER_TIMER_INDEX_LNA_SETUP   (1)
#define SCANNER_PPI_CH                  (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO)
#define SCANNER_TIMER_LNA_INTERRUPT_Msk (1UL << (TIMER_INTENSET_COMPARE0_Pos + SCANNER_TIMER_INDEX_LNA_SETUP))

/** Scanner packet overhead (i.e. size of packet if length is 0). */
#define SCANNER_PACKET_OVERHEAD (offsetof(scanner_packet_t, packet.addr))

/** Scanner configuration, used to change radio parameters used by the scanner. */
typedef struct
{
    radio_config_t radio_config;                                 /**< Radio configuration. */
    uint32_t       scan_window_us;                               /**< Length of a single scan window in microseconds. Must not be greater than @c scan_interval. */
    uint32_t       scan_interval_us;                             /**< Time between the start of each scan window. */
    uint8_t        channels[SCANNER_CHANNELS_MAX];               /**< Radio channels to be used by the scanner. */
    uint8_t        channel_count;                                /**< Number of radio channels. */
    uint32_t       access_addresses[RADIO_CONFIG_LOGICAL_ADDRS]; /**< Access addresses to be used by the scanner. */
} scanner_config_t;

typedef enum
{
    SCANNER_STATE_UNINITIALIZED,
    SCANNER_STATE_IDLE,
    SCANNER_STATE_RUNNING
} scanner_state_t;

typedef enum
{
    SCAN_WINDOW_STATE_OFF,
    SCAN_WINDOW_STATE_ON,
    SCAN_WINDOW_STATE_NEXT_CHANNEL
} scan_window_state_t;

typedef struct
{
    scanner_state_t          state;
    bool                     has_radio_context;
    bool                     waiting_for_memory;
    bool                     is_radio_cfg_pending;
    scan_window_state_t      window_state;
    uint8_t                  channel_index;         /**< Index in the channel map */
    packet_buffer_packet_t * p_buffer_packet;
    scanner_stats_t          stats;
    scanner_config_t         config;
    timer_event_t            timer_window_start;
    timer_event_t            timer_window_end;
    bearer_event_flag_t      nrf_mesh_process_flag;
    packet_buffer_t          packet_buffer;
    uint8_t                  packet_buffer_data[SCANNER_BUFFER_SIZE];
    scanner_rx_callback_t    rx_callback;
} scanner_t;

/*****************************************************************************
* Static variables
*****************************************************************************/

static scanner_t m_scanner;

/*****************************************************************************
* Static functions
*****************************************************************************/

static inline void channel_iterate(void)
{
    if (++m_scanner.channel_index >= m_scanner.config.channel_count)
    {
        m_scanner.channel_index = 0;
    }
    m_scanner.window_state = SCAN_WINDOW_STATE_NEXT_CHANNEL;
}

static inline bool continuous_scanning(void)
{
    return (m_scanner.config.scan_interval_us == m_scanner.config.scan_window_us);
}

/*****************************************************************************
* Radio operation
*****************************************************************************/

static bool radio_set_packet(void)
{
    NRF_MESH_ASSERT(m_scanner.p_buffer_packet == NULL);

    bool got_packet = (NRF_SUCCESS == packet_buffer_reserve(&m_scanner.packet_buffer,
                                                            &m_scanner.p_buffer_packet,
                                                            sizeof(scanner_packet_t)));
    if (got_packet)
    {
        scanner_packet_t * p_packet = (scanner_packet_t *) m_scanner.p_buffer_packet->packet;
        NRF_RADIO->PACKETPTR = (uint32_t) &p_packet->packet;
    }
    else
    {
        m_scanner.stats.out_of_memory++;
    }

    m_scanner.waiting_for_memory = !got_packet;
    return got_packet;
}

static void radio_configure(void)
{
    radio_config_reset();
    radio_config_config(&m_scanner.config.radio_config);

    NRF_RADIO->RXADDRESSES = 0;
    for (uint32_t i = 0; i < RADIO_CONFIG_LOGICAL_ADDRS; i++)
    {
        if (m_scanner.config.access_addresses[i] != SCANNER_ACCESS_ADDR_INVALID)
        {
            radio_config_access_addr_set(m_scanner.config.access_addresses[i], i);
            NRF_RADIO->RXADDRESSES |= (1UL << i);
        }
    }

    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    NRF_RADIO->EVENTS_ADDRESS = 0;

    /* Set up timestamp capture */
    NRF_PPI->CH[SCANNER_PPI_CH].EEP = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[SCANNER_PPI_CH].TEP = (uint32_t) &NRF_TIMER0->TASKS_CAPTURE[SCANNER_TIMER_INDEX_TIMESTAMP];
    NRF_PPI->CHENSET = (1UL << SCANNER_PPI_CH);
}

/**
 * Switch to higher priority interrupt context by triggering radio interrupt.
 */
static inline void radio_trigger(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (m_scanner.has_radio_context)
    {
        (void) NVIC_SetPendingIRQ(RADIO_IRQn);
    }
    _ENABLE_IRQS(was_masked);
}

static void radio_stop(void)
{
    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_RADIO_IN_RX);
    NRF_RADIO->TASKS_DISABLE = 1;
    /* clear any end events, to avoid a misfire */
    NRF_RADIO->EVENTS_END = 0;
    if (m_scanner.p_buffer_packet != NULL)
    {
        packet_buffer_free(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);
        m_scanner.p_buffer_packet = NULL;
    }
#if !defined(HOST)
    while (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled);
#endif
    mesh_pa_lna_cleanup();
}

static void radio_start(void)
{
    NRF_MESH_ASSERT(NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled);

    radio_config_channel_set(m_scanner.config.channels[m_scanner.channel_index]);
    if (m_scanner.is_radio_cfg_pending)
    {
        radio_configure();
        m_scanner.is_radio_cfg_pending = false;
    }
    if (radio_set_packet())
    {
        BEARER_ACTION_TIMER->TASKS_CAPTURE[SCANNER_TIMER_INDEX_LNA_SETUP] = 1;
        NRF_RADIO->TASKS_RXEN = 1;

        mesh_lna_setup_start(BEARER_ACTION_TIMER->CC[SCANNER_TIMER_INDEX_LNA_SETUP] + LNA_SETUP_OVERHEAD_US,
                             SCANNER_TIMER_INDEX_LNA_SETUP);
        mesh_pa_lna_setup_stop();

        BEARER_ACTION_TIMER->EVENTS_COMPARE[SCANNER_TIMER_INDEX_LNA_SETUP] = 0;
        NVIC_ClearPendingIRQ(BEARER_ACTION_TIMER_IRQn);
        BEARER_ACTION_TIMER->INTENSET = SCANNER_TIMER_LNA_INTERRUPT_Msk;

        DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_RADIO_IN_RX);
    }
    m_scanner.window_state = SCAN_WINDOW_STATE_ON;
}

static void radio_handle_end_event(void)
{
    DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_END_EVENT);

    bool successful_receive = false;
    scanner_packet_t * p_packet = (scanner_packet_t *) m_scanner.p_buffer_packet->packet;

    if (!NRF_RADIO->CRCSTATUS)
    {
        m_scanner.stats.crc_failures++;
    }
    else if (p_packet->packet.header.length > m_scanner.config.radio_config.payload_maxlen)
    {
        m_scanner.stats.length_out_of_bounds++;
    }
    else
    {
        successful_receive = true;
    }

    if (successful_receive)
    {
        p_packet->metadata.rssi = - ((int32_t) NRF_RADIO->RSSISAMPLE);
        p_packet->metadata.access_addr = m_scanner.config.access_addresses[NRF_RADIO->RXMATCH];
        /* The 6 lowest bit of the datawhite IV is the same as the channel number in BLE: */
        p_packet->metadata.channel = NRF_RADIO->DATAWHITEIV & 0x3F;
        ts_timestamp_t rx_timestamp_ts = NRF_TIMER0->CC[SCANNER_TIMER_INDEX_TIMESTAMP];
        p_packet->metadata.timestamp = ts_timer_to_device_time(rx_timestamp_ts);

        memcpy(p_packet->metadata.adv_addr.addr, p_packet->packet.addr, BLE_GAP_ADDR_LEN);
        p_packet->metadata.adv_addr.addr_type = p_packet->packet.header.addr_type;
        p_packet->metadata.adv_type = p_packet->packet.header.type;

        m_scanner.stats.successful_receives++;

        if (m_scanner.rx_callback != NULL)
        {
            m_scanner.rx_callback(p_packet, rx_timestamp_ts);
        }

        if (!fen_filters_apply(FILTER_TYPE_PRE_PROC, p_packet))
        {
            packet_buffer_commit(&m_scanner.packet_buffer,
                                 m_scanner.p_buffer_packet,
                                 SCANNER_PACKET_OVERHEAD + p_packet->packet.header.length);

            bearer_event_flag_set(m_scanner.nrf_mesh_process_flag);
        }
        else
        {
            packet_buffer_free(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);
        }
    }
    else
    {
        packet_buffer_free(&m_scanner.packet_buffer, m_scanner.p_buffer_packet);
    }

    m_scanner.p_buffer_packet = NULL;

    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_END_EVENT);
}

static void radio_state_disabled_handle(void)
{
    switch (m_scanner.window_state)
    {
        case SCAN_WINDOW_STATE_NEXT_CHANNEL:
        case SCAN_WINDOW_STATE_ON:
            radio_start();
            break;
        case SCAN_WINDOW_STATE_OFF:
            // Do nothing
            break;
        default:
            NRF_MESH_ASSERT(false);
    }
}

static void radio_state_rxidle_handle(void)
{
    switch (m_scanner.window_state)
    {
        case SCAN_WINDOW_STATE_NEXT_CHANNEL:
            /* Go to disabled state to change channels */
            radio_stop();
            radio_start();
            break;
        case SCAN_WINDOW_STATE_ON:
            if (radio_set_packet())
            {
                NRF_RADIO->TASKS_START = 1;
                DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_RADIO_IN_RX);
            }
            break;
        case SCAN_WINDOW_STATE_OFF:
            radio_stop();
            break;
        default:
            NRF_MESH_ASSERT(false);
    }
}

static void radio_state_rx_handle(void)
{
    switch (m_scanner.window_state)
    {
        case SCAN_WINDOW_STATE_NEXT_CHANNEL:
            /* Go to disabled state to change channels */
            radio_stop();
            radio_start();
            break;
        case SCAN_WINDOW_STATE_ON:
            // Do nothing
            break;
        case SCAN_WINDOW_STATE_OFF:
            radio_stop();
            break;
        default:
            NRF_MESH_ASSERT(false);
    }
}

static void radio_setup_next_operation(void)
{
    switch (m_scanner.state)
    {
        case SCANNER_STATE_IDLE:
            radio_stop();
            break;

        case SCANNER_STATE_RUNNING:
            switch (NRF_RADIO->STATE)
            {
                case RADIO_STATE_STATE_Disabled:
                    radio_state_disabled_handle();
                    break;
                case RADIO_STATE_STATE_RxIdle:
                    radio_state_rxidle_handle();
                    break;
                case RADIO_STATE_STATE_Rx:
                case RADIO_STATE_STATE_RxRu:
                    radio_state_rx_handle();
                    break;
                case RADIO_STATE_STATE_RxDisable:
                default:
                    NRF_MESH_ASSERT(false);
            }
            break;

        case SCANNER_STATE_UNINITIALIZED:
        default:
            NRF_MESH_ASSERT(false);
    }
}

/*****************************************************************************
* Bearer handler callback functions
*****************************************************************************/

void scanner_radio_start(ts_timestamp_t start_time)
{
    DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_START);
    DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_IN_ACTION);
    m_scanner.has_radio_context = true;
    m_scanner.is_radio_cfg_pending = false;
    radio_configure();

    if (m_scanner.state == SCANNER_STATE_RUNNING &&
        m_scanner.window_state != SCAN_WINDOW_STATE_OFF)
    {
        radio_start();
    }
    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_START);
}

void scanner_radio_stop(void)
{
    DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_STOP);
    m_scanner.has_radio_context = false;
    radio_stop();
    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_IN_ACTION);
    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_STOP);
}

void scanner_radio_irq_handler(void)
{
    DEBUG_PIN_SCANNER_ON(DEBUG_PIN_SCANNER_RADIO_IRQ);
    if (NRF_RADIO->EVENTS_END)
    {
        DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_RADIO_IN_RX);
        NRF_RADIO->EVENTS_END = 0;
        (void) NRF_RADIO->EVENTS_END;
        radio_handle_end_event();
    }

    radio_setup_next_operation();

    DEBUG_PIN_SCANNER_OFF(DEBUG_PIN_SCANNER_RADIO_IRQ);
}

void scanner_timer_irq_handler(void)
{
    /* As soon as the LNA timer fires, we prepare for the stop to avoid getting hit by rollover. */
    NRF_MESH_ASSERT(BEARER_ACTION_TIMER->EVENTS_COMPARE[SCANNER_TIMER_INDEX_LNA_SETUP]);
    BEARER_ACTION_TIMER->INTENCLR = SCANNER_TIMER_LNA_INTERRUPT_Msk;
    BEARER_ACTION_TIMER->EVENTS_COMPARE[SCANNER_TIMER_INDEX_LNA_SETUP] = 0;
    (void) BEARER_ACTION_TIMER->EVENTS_COMPARE[SCANNER_TIMER_INDEX_LNA_SETUP];

    mesh_pa_lna_disable_start();
}
/*****************************************************************************
* Timer handling
*****************************************************************************/

static void scan_window_end(uint32_t timestamp, void * p_context)
{
    m_scanner.window_state = SCAN_WINDOW_STATE_OFF;
    radio_trigger();
}

static void scan_window_start(uint32_t timestamp, void * p_context)
{
    channel_iterate();
    radio_trigger();
}

static void schedule_timers(void)
{
    uint32_t time_now = timer_now();

    if (continuous_scanning())
    {
        timer_sch_abort(&m_scanner.timer_window_end);
    }
    else
    {
        timer_sch_reschedule(&m_scanner.timer_window_end, time_now + m_scanner.config.scan_window_us);
    }

    timer_sch_reschedule(&m_scanner.timer_window_start, time_now);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void scanner_init(bearer_event_flag_callback_t packet_process_cb)
{
    memset(&m_scanner, 0, sizeof(m_scanner));

    packet_buffer_init(&m_scanner.packet_buffer, m_scanner.packet_buffer_data, SCANNER_BUFFER_SIZE);
    scanner_config_reset();
    m_scanner.config.radio_config.tx_power = RADIO_POWER_NRF_0DBM;
    m_scanner.config.radio_config.payload_maxlen = RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE;
    m_scanner.timer_window_end.cb = scan_window_end;
    m_scanner.timer_window_start.cb = scan_window_start;
    m_scanner.state = SCANNER_STATE_IDLE;
    m_scanner.window_state = SCAN_WINDOW_STATE_ON;
    m_scanner.nrf_mesh_process_flag = bearer_event_flag_add(packet_process_cb);
}

void scanner_rx_callback_set(scanner_rx_callback_t callback)
{
    if (m_scanner.rx_callback)
    {
        /* Make sure we don't overwrite an active callback */
        NRF_MESH_ASSERT(callback == NULL);
    }
    m_scanner.rx_callback = callback;
}

void scanner_enable(void)
{
    NRF_MESH_ASSERT(m_scanner.state != SCANNER_STATE_UNINITIALIZED);

    if (m_scanner.state == SCANNER_STATE_IDLE)
    {
        schedule_timers();
        m_scanner.state = SCANNER_STATE_RUNNING;
        bearer_handler_wake_up();
    }
}

void scanner_disable(void)
{
    NRF_MESH_ASSERT(m_scanner.state != SCANNER_STATE_UNINITIALIZED);

    if (m_scanner.state == SCANNER_STATE_RUNNING)
    {
        timer_sch_abort(&m_scanner.timer_window_end);
        timer_sch_abort(&m_scanner.timer_window_start);
        m_scanner.state = SCANNER_STATE_IDLE;
        radio_trigger();
    }
}

bool scanner_is_enabled(void)
{
    return (m_scanner.state == SCANNER_STATE_RUNNING);
}

const scanner_packet_t * scanner_rx(void)
{
    packet_buffer_packet_t * p_packet;

    while (packet_buffer_pop(&m_scanner.packet_buffer, &p_packet) == NRF_SUCCESS)
    {
        if (fen_filters_apply(FILTER_TYPE_POST_PROC, (scanner_packet_t *)p_packet->packet))
        {
            scanner_packet_release((scanner_packet_t *)p_packet->packet);
        }
        else
        {
            return (scanner_packet_t *)p_packet->packet;
        }
    }

    return NULL;
}

void scanner_packet_release(const scanner_packet_t * p_packet)
{
    NRF_MESH_ASSERT(p_packet != NULL);
    packet_buffer_free(&m_scanner.packet_buffer,
                       PARENT_BY_FIELD_GET(packet_buffer_packet_t, packet, p_packet));
    if (m_scanner.waiting_for_memory)
    {
        radio_trigger();
    }
}

const scanner_stats_t * scanner_stats_get(void)
{
    return &m_scanner.stats;
}

bool scanner_rx_pending(void)
{
    return packet_buffer_can_pop(&m_scanner.packet_buffer);
}

/*****************************************************************************
* Configuration interface functions
*****************************************************************************/

void scanner_config_radio_mode_set(radio_mode_t radio_mode)
{
    m_scanner.config.radio_config.radio_mode = radio_mode;
    m_scanner.is_radio_cfg_pending = true;
}

void scanner_config_scan_time_set(uint32_t scan_interval_us, uint32_t scan_window_us)
{
    NRF_MESH_ASSERT(scan_interval_us >= scan_window_us);
    NRF_MESH_ASSERT(scan_interval_us <= MS_TO_US(BEARER_SCAN_INT_MAX_MS));
    NRF_MESH_ASSERT(scan_window_us >= MS_TO_US(BEARER_SCAN_WIN_MIN_MS));
    m_scanner.config.scan_interval_us = scan_interval_us;
    m_scanner.config.scan_window_us = scan_window_us;
    m_scanner.timer_window_end.interval = m_scanner.config.scan_interval_us;
    m_scanner.timer_window_start.interval = m_scanner.config.scan_interval_us;
    if (m_scanner.state == SCANNER_STATE_RUNNING)
    {
        schedule_timers();
    }
}

static bool channels_are_valid(const uint8_t * p_channels, uint8_t channel_count)
{
    uint8_t valid_channels[] = SCANNER_CHANNELS_DEFAULT;

    for (uint8_t i = 0; i < channel_count; i++)
    {
        bool found = false;
        for (uint8_t j = 0; j < sizeof(valid_channels); j++)
        {
            if (p_channels[i] == valid_channels[j])
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            return false;
        }
    }
    return true;
}

void scanner_config_channels_set(const uint8_t * p_channels, uint8_t channel_count)
{
    NRF_MESH_ASSERT(channel_count > 0 && channel_count <= SCANNER_CHANNELS_MAX);
    NRF_MESH_ASSERT(channels_are_valid(p_channels, channel_count));
    memcpy(m_scanner.config.channels, p_channels, channel_count);
    m_scanner.channel_index = 0;
    m_scanner.config.channel_count = channel_count;
}

/* The memset() in scanner_config_access_addresses_set() will not have the expected effect
   unless SCANNER_ACCESS_ADDR_INVALID is zero. */
NRF_MESH_STATIC_ASSERT(SCANNER_ACCESS_ADDR_INVALID == 0);

void scanner_config_access_addresses_set(const uint32_t * p_access_addresses, uint8_t address_count)
{
    NRF_MESH_ASSERT(address_count > 0 && address_count <= RADIO_CONFIG_LOGICAL_ADDRS);
    memset(&m_scanner.config.access_addresses[0], 0, sizeof(m_scanner.config.access_addresses));
    memcpy(&m_scanner.config.access_addresses[0], p_access_addresses, sizeof(uint32_t) * address_count);

    m_scanner.is_radio_cfg_pending = true;
}

void scanner_config_reset(void)
{
    uint8_t channels[] = SCANNER_CHANNELS_DEFAULT;
    scanner_config_channels_set(channels, sizeof(channels));

    uint32_t access_address = BEARER_ACCESS_ADDR_DEFAULT;
    scanner_config_access_addresses_set(&access_address, 1);

    scanner_config_radio_mode_set(RADIO_MODE_BLE_1MBIT);
    scanner_config_scan_time_set(MS_TO_US(BEARER_SCAN_INT_DEFAULT_MS), MS_TO_US(BEARER_SCAN_WINDOW_DEFAULT_MS));
}
