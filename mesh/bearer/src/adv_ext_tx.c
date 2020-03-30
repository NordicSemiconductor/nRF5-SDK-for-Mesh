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
#include "adv_ext_tx.h"

#include "timeslot.h"
#include "nrf_mesh_utils.h"
#include "debug_pins.h"
#include "nordic_common.h"
#include "nrf_mesh_config_bearer.h"
#include "mesh_pa_lna_internal.h"

#define ADV_EXT_TIMER_INDEX_TIMESTAMP       (0) /**< Timer index used to control radio timing. */
#define ADV_EXT_TIMER_INDEX_PA              (1) /**< Timer index used to control power amplifier. */
#define ADV_EXT_TIMER_INDEX_RADIO_TRIGGER   (TS_TIMER_INDEX_RADIO) /**< Timer index used to control radio timing. */
#define ADV_EXT_PPI_CH                      (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO) /**< PPI channel used to trigger timer capture */

/* Timing */
#define OFFSET_TIME_STEP_US    (ADV_EXT_TIME_OFFSET_UNIT_30us) /**< Offset unit used in all AuxPtr packets */
#define OFFSET_TIME_MIN_US     (450) /**< Base offset time from the last EXT IND advertisement packet to the first AUX packet. */
#define OFFSET_TIME_MAX_US     (OFFSET_TIME_MIN_US + OFFSET_TIME_STEP_US) /**< Largest possible base offset time. */
#define CHAIN_DELAY_US         (300) /**< Delay between packets in a chain */

#define TX_POST_PROCESS_US     (200) /**< Time spent in post processing of a TX event. */

#define ADV_EXT_GAP_ADDR_TYPE  (0) /**< GAP address type to put in the advertisement header. */

/** Packet delay for EXT IND packets in microseconds. These must run on the BLE 1Mbit. */
#define ADV_EXT_IND_PACKET_TX_DELAY_US                                                             \
    (RADIO_RAMPUP_TIME +                                                                           \
     (BLE_PACKET_OVERHEAD(RADIO_MODE_BLE_1MBIT) + ADV_EXT_TX_ADV_EXT_IND_OVERHEAD) *               \
         RADIO_TIME_PER_BYTE(RADIO_MODE_BLE_1MBIT))

/** Time spent transmitting the ADV EXT IND packets. */
#define ADV_EXT_IND_COMBINED_TX_TIME_US (ADV_EXT_IND_PACKET_TX_DELAY_US * ARRAY_SIZE(m_adv_channels) + OFFSET_TIME_MAX_US)

/** Time spent transmitting a single aux packet. */
#define AUX_PACKET_TX_TIME_US(radio_mode, len) ((BLE_PACKET_OVERHEAD(radio_mode) + (len)) * RADIO_TIME_PER_BYTE(radio_mode))

/* Packet specification */
#define ADV_EXT_TX_ADV_EXT_IND_FIELDS               (ADV_EXT_IND_FIELDS_REQUIRED)
#define ADV_EXT_TX_AUX_ADV_IND_FIELDS(has_more)     ((adv_ext_header_bitfield_t) (AUX_ADV_IND_FIELDS_REQUIRED | ((has_more) ? ADV_EXT_HEADER_AUX_PTR_BIT : 0)))
#define ADV_EXT_TX_AUX_CHAIN_IND_FIELDS(has_more)   ((adv_ext_header_bitfield_t) (AUX_CHAIN_IND_FIELDS_REQUIRED | ((has_more) ? ADV_EXT_HEADER_AUX_PTR_BIT : 0)))

#define ADV_EXT_TX_ADV_EXT_IND_OVERHEAD             (ADV_EXT_OVERHEAD(ADV_EXT_TX_ADV_EXT_IND_FIELDS))
#define ADV_EXT_TX_AUX_ADV_IND_OVERHEAD(has_more)   (ADV_EXT_OVERHEAD(ADV_EXT_TX_AUX_ADV_IND_FIELDS(has_more)))
#define ADV_EXT_TX_AUX_CHAIN_IND_OVERHEAD(has_more) (ADV_EXT_OVERHEAD(ADV_EXT_TX_AUX_CHAIN_IND_FIELDS(has_more)))

/* All aux packet variants have to fit inside the allocated header buffer */
NRF_MESH_STATIC_ASSERT(ADV_EXT_TX_AUX_ADV_IND_OVERHEAD(true)    <= ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX);
NRF_MESH_STATIC_ASSERT(ADV_EXT_TX_AUX_ADV_IND_OVERHEAD(false)   <= ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX);
NRF_MESH_STATIC_ASSERT(ADV_EXT_TX_AUX_CHAIN_IND_OVERHEAD(false) <= ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX);
NRF_MESH_STATIC_ASSERT(ADV_EXT_TX_AUX_CHAIN_IND_OVERHEAD(true)  <= ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX);

/* Radio configuration for ADV EXT IND packets */
#define ADV_EXT_IND_PAYLOAD_MAXLEN  RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE
#define ADV_EXT_IND_RADIO_MODE      RADIO_MODE_BLE_1MBIT
/*****************************************************************************
* Static globals
*****************************************************************************/
/** Packet buffer for the ADV EXT IND packets. */
static uint8_t m_adv_ext_ind[sizeof(ble_packet_hdr_t) + ADV_EXT_TX_ADV_EXT_IND_OVERHEAD];

typedef enum
{
    TX_STATE_SEND_ADV_EXT_IND_FIRST,
    TX_STATE_SEND_ADV_EXT_IND_FOLLOW_UP,
    TX_STATE_SEND_AUX_ADV_FIRST,
    TX_STATE_SEND_AUX_ADV_FOLLOW_UP,
    TX_STATE_END,
} tx_state_t;

static struct
{
    tx_state_t state;
    uint8_t adv_index;
    uint8_t packet_index;
    adv_ext_tx_packet_t * p_tx_packet;
    ts_timestamp_t start_time;
} m_tx_session;

/** Time to start radio TX for the current auxiliary packet, as reported in the Aux Ptr pointing to
 * the packet. */
static ts_timestamp_t m_tx_time;

/** Advertisement channels for the EXT IND packets. */
static const uint8_t m_adv_channels[] = NRF_MESH_ADV_CHAN_ALL;

/** Local clock accuracy, reported in the packet headers. */
static adv_ext_clock_accuracy_t m_clock_accuracy;
/*****************************************************************************
* Static functions
*****************************************************************************/
static inline uint32_t tx_action_duration(uint32_t packet_count, radio_mode_t radio_mode)
{
    uint32_t tx_time_per_packet = AUX_PACKET_TX_TIME_US(radio_mode, ADV_EXT_PACKET_LEN_MAX);

    return ADV_EXT_IND_COMBINED_TX_TIME_US + (CHAIN_DELAY_US + tx_time_per_packet) * packet_count + TX_POST_PROCESS_US + ADV_EXT_TX_USER_CALLBACK_MAXTIME;
}

static void header_fields_set(const adv_ext_tx_t * p_tx, adv_ext_packet_t * p_packet, adv_ext_header_bitfield_t fields, uint32_t time_offset_us)
{
    adv_ext_header_set_t set;
    memset(&set, 0, sizeof(set));
    adv_ext_header_aux_ptr_t aux_ptr;
    adv_ext_header_tx_power_t tx_power;

    if (fields & ADV_EXT_HEADER_ADV_ADDR_BIT)
    {
        NRF_MESH_ASSERT(false); /* not supported */
    }
    if (fields & ADV_EXT_HEADER_TARGET_ADDR_BIT)
    {
        NRF_MESH_ASSERT(false); /* not supported */
    }
    if (fields & ADV_EXT_HEADER_ADI_BIT)
    {
        set.p_adi = &p_tx->p_tx_event->params.id;
    }
    if (fields & ADV_EXT_HEADER_AUX_PTR_BIT)
    {
        aux_ptr.channel_index   = p_tx->p_tx_event->params.channel;
        aux_ptr.clock_accuracy  = m_clock_accuracy;
        aux_ptr.offset_units_us = OFFSET_TIME_STEP_US;
        aux_ptr.radio_mode      = p_tx->config.radio_config.radio_mode;
        aux_ptr.time_offset_us  = time_offset_us;

        set.p_aux_ptr = &aux_ptr;
    }
    if (fields & ADV_EXT_HEADER_SYNC_INFO_BIT)
    {
        NRF_MESH_ASSERT(false); /* not supported */
    }
    if (fields & ADV_EXT_HEADER_TX_POWER_BIT)
    {
        tx_power.tx_power = (int8_t) p_tx->config.radio_config.tx_power;

        set.p_tx_power = &tx_power;
    }
    adv_ext_header_generate((adv_ext_header_t *) &p_packet->data[0], ADV_EXT_HEADER_ADV_MODE_NONCONN, &set);
}

/**
 * Build an adv ext packet based on the given tx packet.
 *
 * Since the contents and length of the header is decided here, the packet has a byte array in front
 * of the data, and the finished aux packet starts somewhere inside that array.
 *
 * @param[in] p_tx Adv ext tx instance the packet belongs to.
 * @param[in,out] p_packet TX packet to build an aux packet from.
 * @param[in] fields Fields to populate.
 * @param[in,out] p_next_tx_time Pointer to fill with the delay until the next packet, or NULL if
 * this is the last packet in the chain.
 *
 * @returns A pointer to the finished aux packet.
 */
static adv_ext_packet_t * aux_packet_build(const adv_ext_tx_t * p_tx,
                                           adv_ext_tx_packet_t * p_packet,
                                           adv_ext_header_bitfield_t fields,
                                           uint32_t * p_next_tx_time)
{
    uint32_t overhead = sizeof(ble_packet_hdr_t) + ADV_EXT_OVERHEAD(fields);
    NRF_MESH_ASSERT(overhead <= sizeof(p_packet->header));
    adv_ext_packet_t * p_adv_ext_packet = (adv_ext_packet_t *) (p_packet->data - overhead);

    p_adv_ext_packet->header.type       = BLE_PACKET_TYPE_ADV_EXT;
    p_adv_ext_packet->header.addr_type  = ADV_EXT_GAP_ADDR_TYPE;
    p_adv_ext_packet->header._rfu1      = 0;
    p_adv_ext_packet->header.length     = p_packet->data_len + ADV_EXT_OVERHEAD(fields);
    p_adv_ext_packet->header._rfu2      = 0;

    uint32_t time_offset =
        (AUX_PACKET_TX_TIME_US(p_tx->config.radio_config.radio_mode, p_adv_ext_packet->header.length) +
         CHAIN_DELAY_US);
    header_fields_set(p_tx, p_adv_ext_packet, fields, time_offset);

    if (p_next_tx_time != NULL)
    {
        *p_next_tx_time = time_offset;
    }

    return p_adv_ext_packet;
}

static void setup_adv_ext_ind(adv_ext_tx_t * p_tx)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_SETUP_ADV_EXT_IND);

    uint32_t time_until_aux =
        OFFSET_TIME_MIN_US + ADV_EXT_IND_PACKET_TX_DELAY_US * (ARRAY_SIZE(m_adv_channels) - m_tx_session.adv_index - 1);

    header_fields_set(p_tx, (adv_ext_packet_t *) m_adv_ext_ind, ADV_EXT_TX_ADV_EXT_IND_FIELDS, time_until_aux);

    switch (m_tx_session.state)
    {
        case TX_STATE_SEND_ADV_EXT_IND_FIRST:
            radio_config_channel_set(m_adv_channels[m_tx_session.adv_index]);

            BEARER_ACTION_TIMER->TASKS_CAPTURE[ADV_EXT_TIMER_INDEX_PA] = 1;
            NRF_RADIO->TASKS_TXEN = 1;

            mesh_pa_setup_start(BEARER_ACTION_TIMER->CC[ADV_EXT_TIMER_INDEX_PA] + 1, ADV_EXT_TIMER_INDEX_PA);

            NRF_RADIO->EVENTS_READY = 0;
            (void) NRF_RADIO->EVENTS_READY;
            NRF_RADIO->EVENTS_DISABLED = 0;
            (void) NRF_RADIO->EVENTS_DISABLED;

            NRF_RADIO->PACKETPTR = (uint32_t) m_adv_ext_ind;
            NRF_RADIO->SHORTS    = RADIO_SHORTS_READY_START_Msk |
                                   RADIO_SHORTS_END_DISABLE_Msk |
                                   RADIO_SHORTS_DISABLED_TXEN_Msk;
            NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

            /* Get reference point for time offset. Will re-trigger on every READY event, to make
             * the final adv ext ind the counting timestamp. */
            NRF_PPI->CH[ADV_EXT_PPI_CH].EEP = (uint32_t) &NRF_RADIO->EVENTS_READY;
            NRF_PPI->CH[ADV_EXT_PPI_CH].TEP = (uint32_t) &BEARER_ACTION_TIMER->TASKS_CAPTURE[ADV_EXT_TIMER_INDEX_TIMESTAMP];
            NRF_PPI->CHENSET = (1UL << ADV_EXT_PPI_CH);
            break;

        case TX_STATE_SEND_ADV_EXT_IND_FOLLOW_UP:
            /* If we're not in rampup state at this point, we modified the radio packet while it was
             * on air. This can happen if some other process prevented the radio interrupt from
             * firing for over 35us. */
            NRF_MESH_ASSERT(NRF_RADIO->STATE == RADIO_STATE_STATE_TxRu);
            mesh_pa_lna_disable_start();
            break;

        default:
            /* This function only handles the adv ext ind */
            NRF_MESH_ASSERT(false);
    }

    if (ARRAY_SIZE(m_adv_channels) > m_tx_session.adv_index + 1)
    {
        /* Channel decision point is at TXEN, can safely set up the next channel now. */
        radio_config_channel_set(m_adv_channels[++m_tx_session.adv_index]);
        m_tx_session.state = TX_STATE_SEND_ADV_EXT_IND_FOLLOW_UP;
    }
    else
    {
        m_tx_session.state = TX_STATE_SEND_AUX_ADV_FIRST;
        NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
        mesh_pa_lna_setup_stop();
    }
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_SETUP_ADV_EXT_IND);
}

static void setup_aux(adv_ext_tx_t * p_tx)
{
    NRF_MESH_ASSERT(m_tx_session.packet_index < p_tx->p_tx_event->params.packet_count);

    bool last_in_chain = (m_tx_session.packet_index == p_tx->p_tx_event->params.packet_count - 1);
    bool first_in_chain = (m_tx_session.packet_index == 0);

    adv_ext_header_bitfield_t fields = first_in_chain
                                           ? ADV_EXT_TX_AUX_ADV_IND_FIELDS(!last_in_chain)
                                           : ADV_EXT_TX_AUX_CHAIN_IND_FIELDS(!last_in_chain);
    uint32_t next_delay;

    adv_ext_packet_t * p_packet =
        aux_packet_build(p_tx, m_tx_session.p_tx_packet, fields, &next_delay);

    /* Prepare radio to fire */
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
    radio_config_config(&p_tx->config.radio_config);
    radio_config_channel_set(p_tx->p_tx_event->params.channel);

    NRF_RADIO->PACKETPTR = (uint32_t) p_packet;

    const uint32_t txen_trigger_time = m_tx_time - RADIO_RAMPUP_TIME - m_tx_session.start_time;

    BEARER_ACTION_TIMER->CC[ADV_EXT_TIMER_INDEX_RADIO_TRIGGER] = txen_trigger_time;
    BEARER_ACTION_TIMER->EVENTS_COMPARE[ADV_EXT_TIMER_INDEX_RADIO_TRIGGER] = 0;
    NRF_PPI->CH[ADV_EXT_PPI_CH].EEP = (uint32_t) &BEARER_ACTION_TIMER->EVENTS_COMPARE[ADV_EXT_TIMER_INDEX_RADIO_TRIGGER];
    NRF_PPI->CH[ADV_EXT_PPI_CH].TEP = (uint32_t) &NRF_RADIO->TASKS_TXEN;
    NRF_PPI->CHENSET = (1UL << ADV_EXT_PPI_CH);

    mesh_pa_setup_start(txen_trigger_time, ADV_EXT_TIMER_INDEX_PA);
    mesh_pa_lna_setup_stop();

    /* Verify that we were able to service the TX in time */
    NRF_MESH_ASSERT(TIMER_OLDER_THAN(ts_timer_now(), txen_trigger_time + m_tx_session.start_time));

    if (last_in_chain)
    {
        m_tx_session.state = TX_STATE_END;
    }
    else
    {
        m_tx_time += next_delay;
        m_tx_session.state = TX_STATE_SEND_AUX_ADV_FOLLOW_UP;
    }
}

static void action_start(ts_timestamp_t start_time, void * p_args)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_START);
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_IN_ACTION);
    adv_ext_tx_t * p_tx = p_args;

    m_tx_session.state = TX_STATE_SEND_ADV_EXT_IND_FIRST;
    m_tx_session.adv_index = 0;
    m_tx_session.start_time = start_time;

    /* Send first advertisement */
    radio_config_t radio_config =
    {
        .tx_power = p_tx->config.radio_config.tx_power,
        .payload_maxlen = ADV_EXT_IND_PAYLOAD_MAXLEN,
        .radio_mode = ADV_EXT_IND_RADIO_MODE
    };

    radio_config_reset();
    radio_config_config(&radio_config);
    radio_config_access_addr_set(BEARER_ACCESS_ADDR_DEFAULT, 0);

    setup_adv_ext_ind(p_tx);

    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_START);
}

static void radio_irq_handler(void * p_args)
{
    DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_RADIO_EVT);

    NRF_MESH_ASSERT(NRF_RADIO->EVENTS_DISABLED);
    NRF_RADIO->EVENTS_DISABLED = 0;
    (void) NRF_RADIO->EVENTS_DISABLED;

    adv_ext_tx_t * p_tx = p_args;

    switch (m_tx_session.state)
    {
        case TX_STATE_SEND_ADV_EXT_IND_FOLLOW_UP:
            setup_adv_ext_ind(p_tx);
            break;

        case TX_STATE_SEND_AUX_ADV_FIRST:
            m_tx_session.p_tx_packet = (adv_ext_tx_packet_t *) &p_tx->p_tx_event->packet_data[0];
            m_tx_session.packet_index = 0;
            m_tx_time = m_tx_session.start_time + BEARER_ACTION_TIMER->CC[ADV_EXT_TIMER_INDEX_TIMESTAMP] + OFFSET_TIME_MIN_US;
            setup_aux(p_tx);
            break;

        case TX_STATE_SEND_AUX_ADV_FOLLOW_UP:
            m_tx_session.p_tx_packet = (adv_ext_tx_packet_t *) adv_ext_tx_packet_next_get(m_tx_session.p_tx_packet);
            m_tx_session.packet_index++;
            setup_aux(p_tx);
            break;

        case TX_STATE_END:
        {
            DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_IN_ACTION);

            mesh_pa_lna_cleanup();

            bearer_handler_action_end();

            const adv_ext_tx_event_t * p_tx_event = p_tx->p_tx_event;
            p_tx->p_tx_event = NULL;
            if (p_tx->config.callback != NULL)
            {
                ts_timestamp_t header_start_time =
                    m_tx_time + RADIO_ADDR_EVT_DELAY(p_tx->config.radio_config.radio_mode);
                /* Report the TX complete with a device timestamp, not a timeslot timestamp */
                p_tx->config.callback(p_tx, p_tx_event, ts_timer_to_device_time(header_start_time));
            }
            break;
        }
        default:
            NRF_MESH_ASSERT(false);
    }
    DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_RADIO_EVT);
}

static adv_ext_clock_accuracy_t adv_ext_clock_accuracy_level_get(uint32_t accuracy_ppm)
{
    if (accuracy_ppm <= 50)
    {
        return ADV_EXT_CLOCK_ACCURACY_0_to_50ppm;
    }
    else if (accuracy_ppm <= 500)
    {
        return ADV_EXT_CLOCK_ACCURACY_51_to_500ppm;
    }
    /* Clock accuracy below 501ppm required. */
    NRF_MESH_ASSERT(false);
    return ADV_EXT_CLOCK_ACCURACY_51_to_500ppm;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void adv_ext_tx_init(uint32_t lfclk_ppm)
{
    m_clock_accuracy = adv_ext_clock_accuracy_level_get(lfclk_ppm);

    adv_ext_packet_t * p_adv_ext_ind = (adv_ext_packet_t *) &m_adv_ext_ind[0];

    p_adv_ext_ind->header._rfu1     = 0;
    p_adv_ext_ind->header._rfu2     = 0;
    p_adv_ext_ind->header._rfu3     = 0;
    p_adv_ext_ind->header.length    = ADV_EXT_OVERHEAD(ADV_EXT_TX_ADV_EXT_IND_FIELDS);
    p_adv_ext_ind->header.type      = BLE_PACKET_TYPE_ADV_EXT;
    p_adv_ext_ind->header.addr_type = ADV_EXT_GAP_ADDR_TYPE;
}

void adv_ext_tx_instance_init(adv_ext_tx_t * p_tx, const adv_ext_tx_config_t * p_config)
{
    p_tx->config = *p_config;
    p_tx->bearer_action.p_args = p_tx;
    p_tx->bearer_action.start_cb = action_start;
    p_tx->bearer_action.radio_irq_handler = radio_irq_handler;

    p_tx->p_tx_event = NULL;
}

uint32_t adv_ext_tx(adv_ext_tx_t * p_tx, const adv_ext_tx_event_t * p_tx_event)
{
    NRF_MESH_ASSERT(p_tx_event->params.channel < RADIO_NO_RF_CHANNELS);
    NRF_MESH_ASSERT(p_tx_event->params.packet_count > 0);
    NRF_MESH_ASSERT(p_tx_event->params.packet_count <= ADV_EXT_TX_CHAIN_MAX_COUNT);

    if (p_tx->p_tx_event == NULL)
    {
        DEBUG_PIN_INSTABURST_ON(DEBUG_PIN_INSTABURST_ORDER_TX);

        p_tx->p_tx_event = p_tx_event;

        p_tx->bearer_action.duration_us =
            tx_action_duration(p_tx->p_tx_event->params.packet_count,
                                p_tx->config.radio_config.radio_mode);

        NRF_MESH_ERROR_CHECK(bearer_handler_action_enqueue(&p_tx->bearer_action));
        DEBUG_PIN_INSTABURST_OFF(DEBUG_PIN_INSTABURST_ORDER_TX);
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}
