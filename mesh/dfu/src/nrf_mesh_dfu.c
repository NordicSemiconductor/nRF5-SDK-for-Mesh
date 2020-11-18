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

#include <string.h>
#include "nrf.h"
#include "nrf_mesh_dfu.h"
#include "timeslot.h"
#include "nrf_mesh_dfu_types.h"
#include "dfu_types_internal.h"
#include "toolchain.h"
#include "event.h"
#include "bl_if.h"
#include "nrf_flash.h"
#include "timer_scheduler.h"
#include "mesh_flash.h"
#include "rand.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "hal.h"
#include "advertiser.h"
#include "ad_listener.h"
#ifdef NRF_MESH_SERIAL_ENABLE
#include "serial.h"
#endif

/** Check that the enums used for DFU enums for TYPE, STATE, END, and ROLE fit the assumptions */
NRF_MESH_STATIC_ASSERT(NRF_MESH_DFU_TYPE__LAST      <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(NRF_MESH_DFU_STATE__LAST     <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(NRF_MESH_DFU_END_ERROR__LAST <= UINT8_MAX);
NRF_MESH_STATIC_ASSERT(NRF_MESH_DFU_ROLE__LAST      <= UINT8_MAX);

/*****************************************************************************
* Local defines
*****************************************************************************/
#define MAX_NUMBER_INTERRUPTS               (32)        /**< Maximum number of interrupts available. */

#define DFU_TX_REDUNDANCY_MAX               (2)         /**< Max number of observed retransmits before a transmit is regarded redundant. */
#define DFU_TX_INTERVAL_EXPONENTIAL_US      (100000)    /**< Time between the first two transmits in exponential packets. */
#define DFU_TX_INTERVAL_REGULAR_US          (100000)    /**< Time between transmits for fast regular transmits. */
#define DFU_TX_INTERVAL_REGULAR_SLOW_US     (10000000)  /**< Time between transmits for slow regular transmits. */
#define DFU_TX_DELAY_RANDOMIZATION_MASK_US  (0xFFFF)    /**< Maximum variation in TX time offsets. */
#define DFU_TX_TIMER_MARGIN_US              (1000)      /**< Time margin for a timeout to be considered instant. */
/*****************************************************************************
* Local typedefs
*****************************************************************************/

/** DFU TX slot. The DFU can interleave multiple periodic transmits, and each
 * periodic transmit gets its own slot. */
typedef struct
{
    uint32_t order_time;                    /**< Timestamp when the packet was first ordered. */
    uint16_t tx_randomization_offset_us;    /**< Randomized time deviation from the normal for the next transmission. */
    bl_radio_interval_type_t interval_type; /**< Type of interval for this periodic transmit. */
    nrf_mesh_dfu_packet_t dfu_packet;       /**< DFU packet being sent on this slot. */
    uint8_t dfu_packet_len;                 /**< Length of the DFU packet. */
    uint8_t repeats;                        /**< Number of repeats this transmit should do. */
    uint8_t tx_count;                       /**< Number of repeats so far. */
    /** Number of times this packet has been received since the previous
     * transmit. To reduce the traffic on air, a transmit might be skipped if
     * the packet is received @ref DFU_TX_REDUNDANCY_MAX times since the
     * previous transmit. */
    uint8_t rx_count;
} dfu_tx_t;

/** Internal transfer state structure */
typedef struct
{
    nrf_mesh_dfu_role_t  role;           /**< This device's intended role in the transfer. */
    nrf_mesh_dfu_type_t  type;           /**< The DFU type of the transfer. */
    nrf_mesh_fwid_t      fwid;           /**< The FWID of the new data in the transfer. */
    nrf_mesh_dfu_state_t state;          /**< The current global state of the transfer. */
    uint16_t             segment_count;  /**< Number of segments received in current transfer. */
    uint16_t             total_segments; /**< Number of segments required for the entire transfer. */
} dfu_transfer_state_internal_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
bl_if_cmd_handler_t                  m_cmd_handler;            /**< Command handler in shared code space, non-static for unit testing purposes. */
static timer_event_t                 m_timer_evt;              /**< Timer event for scheduler. */
static timer_event_t                 m_tx_timer_evt;           /**< TX event for scheduler. */
static dfu_tx_t                      m_tx_slots[NRF_MESH_DFU_TX_SLOTS]; /**< TX slots for concurrent transmits. */
static prng_t                        m_prng;                   /**< PRNG for time delays. */
static fwid_t*                       mp_curr_fwid;             /**< Current Firmware IDs. */
static dfu_transfer_state_internal_t m_transfer_state;         /**< State of the ongoing transfer. */
static packet_t                      m_adv_packet;             /**< Advertisement packet used when sending. */
static broadcast_t                   m_broadcast;              /**< Broadcast instance used to transmit the advertisement packet. */
static const uint8_t                 m_adv_channels[] = NRF_MESH_ADV_CHAN_DEFAULT;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void ad_data_in(const uint8_t * p_ad_data, uint32_t ad_data_len, const nrf_mesh_rx_metadata_t * p_metadata)
{
    const ble_ad_data_service_data_t * p_service_data = (const ble_ad_data_service_data_t *) p_ad_data;
    if (p_service_data->uuid == BLE_ADV_SERVICE_DATA_UUID_DFU)
    {
        (void) nrf_mesh_dfu_rx(p_service_data->data, ad_data_len - offsetof(ble_ad_data_service_data_t, data), p_metadata);
    }
}
AD_LISTENER(m_dfu_ad_listener) = {
    .ad_type = AD_TYPE_DFU,
    .adv_packet_type = ADL_WILDCARD_ADV_TYPE,
    .handler = ad_data_in,
};

static void reset_transfer_state(void)
{
    timer_sch_abort(&m_timer_evt);
    m_transfer_state.role = NRF_MESH_DFU_ROLE_NONE;
    m_transfer_state.segment_count = 0;
    m_transfer_state.total_segments = 0;
    m_transfer_state.type = NRF_MESH_DFU_TYPE_INVALID;
    m_transfer_state.state = NRF_MESH_DFU_STATE_INITIALIZED;
}

static void end_transfer(nrf_mesh_dfu_end_t end_reason)
{
    nrf_mesh_dfu_state_t old_state = m_transfer_state.state;
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_DFU_END;
    evt.params.dfu.end.transfer.dfu_type = m_transfer_state.type;
    evt.params.dfu.end.transfer.id = m_transfer_state.fwid;
    evt.params.dfu.end.role = m_transfer_state.role;
    evt.params.dfu.end.end_reason = end_reason;
    reset_transfer_state();
    event_handle(&evt);

    if (old_state == NRF_MESH_DFU_STATE_RELAY ||
        old_state == NRF_MESH_DFU_STATE_RELAY_CANDIDATE ||
        end_reason != NRF_MESH_DFU_END_SUCCESS)
    {
        /**
         * After relaying the firmware is finished or the DFU process is failed,
         * the DFU module needs to be restarted so that it has the same state
         * as the application-side DFU module.
         */
        NRF_MESH_ERROR_CHECK(nrf_mesh_dfu_enable());
    }
}

static inline bool tx_slot_in_use(dfu_tx_t* p_tx)
{
    return (p_tx->repeats != 0);
}

static uint32_t get_curr_fwid(nrf_mesh_dfu_type_t type, nrf_mesh_fwid_t* p_fwid)
{
    switch (type)
    {
        case NRF_MESH_DFU_TYPE_SOFTDEVICE:
            p_fwid->softdevice = mp_curr_fwid->sd;
            break;
        case NRF_MESH_DFU_TYPE_BOOTLOADER:
            p_fwid->bootloader = mp_curr_fwid->bootloader;
            break;
        case NRF_MESH_DFU_TYPE_APPLICATION:
            p_fwid->application = mp_curr_fwid->app;
            break;
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    return NRF_SUCCESS;
}

static uint32_t next_tx_timeout(dfu_tx_t* p_tx)
{
    switch (p_tx->interval_type)
    {
        case BL_RADIO_INTERVAL_TYPE_EXPONENTIAL:
            return ((p_tx->order_time + DFU_TX_INTERVAL_EXPONENTIAL_US * ((1 << (p_tx->tx_count)) - 1)) + p_tx->tx_randomization_offset_us);
        case BL_RADIO_INTERVAL_TYPE_REGULAR:
            return ((p_tx->order_time + DFU_TX_INTERVAL_REGULAR_US * p_tx->tx_count) + p_tx->tx_randomization_offset_us);
        case BL_RADIO_INTERVAL_TYPE_REGULAR_SLOW:
            return ((p_tx->order_time + DFU_TX_INTERVAL_REGULAR_SLOW_US * p_tx->tx_count) + p_tx->tx_randomization_offset_us);
        default:
            /* Unknown type */
            NRF_MESH_ASSERT(false);
            return 0;
    }
}

static void tx_end_handle(dfu_tx_t* p_tx, timestamp_t prev_timeout)
{
    if (p_tx->tx_count == BL_IF_TX_REPEATS_INF &&
        p_tx->repeats  == BL_IF_TX_REPEATS_INF)
    {
        p_tx->order_time = prev_timeout;
        p_tx->tx_count = 0;
    }
    else if (p_tx->tx_count >= p_tx->repeats)
    {
        p_tx->repeats = 0;
    }
}

static void interrupts_disable(void)
{
#if !defined(HOST)
    uint32_t interrupt_setting_mask = NVIC->ISER[0];

    /* Loop through and disable all interrupts. */
    for (uint32_t irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (1UL << irq))
        {
            NVIC_DisableIRQ((IRQn_Type) irq);
        }
    }
#endif
}

static void build_adv_packet(void)
{
    ble_gap_addr_t adv_addr;
    (void) advertiser_address_default_get(&adv_addr);
    memcpy(m_adv_packet.addr, adv_addr.addr, BLE_GAP_ADDR_LEN);
    m_adv_packet.header.addr_type = adv_addr.addr_type;
    m_adv_packet.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
}

static bool transmit_dfu_packet(const nrf_mesh_dfu_packet_t * p_dfu_packet, uint32_t dfu_packet_length)
{
    if (m_broadcast.active)
    {
        return false;
    }

    ble_ad_data_t * p_ad = (ble_ad_data_t *) &m_adv_packet.payload[0];
    ble_ad_data_service_data_t * p_service_data = (ble_ad_data_service_data_t *) &p_ad->data[0];
    m_adv_packet.header.length = dfu_packet_length + DFU_PACKET_OVERHEAD;
    p_ad->length            = DFU_PACKET_PAYLOAD_OVERHEAD + dfu_packet_length;
    p_ad->type              = AD_TYPE_DFU;
    p_service_data->uuid    = BLE_ADV_SERVICE_DATA_UUID_DFU;
    memcpy(&p_service_data->data[0], p_dfu_packet, dfu_packet_length);

    NRF_MESH_ERROR_CHECK(broadcast_send(&m_broadcast));
    return true;
}

static inline uint16_t tx_randomization_offset_get(void)
{
    return (uint16_t) rand_prng_get(&m_prng) & DFU_TX_DELAY_RANDOMIZATION_MASK_US;
}

static void tx_timeout_handled(dfu_tx_t * p_tx)
{
    if (p_tx->tx_count < p_tx->repeats)
    {
        p_tx->tx_count++;
        p_tx->tx_randomization_offset_us = tx_randomization_offset_get();
    }
}

static void tx_timeout(uint32_t timestamp, void* p_context)
{
    uint32_t next_timeout = timestamp + (UINT32_MAX / 2);
    for (uint32_t i = 0; i < NRF_MESH_DFU_TX_SLOTS; ++i)
    {
        if (tx_slot_in_use(&m_tx_slots[i]))
        {
            uint32_t timeout = next_tx_timeout(&m_tx_slots[i]);
            if (TIMER_OLDER_THAN(timeout, (timestamp + DFU_TX_TIMER_MARGIN_US)))
            {
                if (m_tx_slots[i].rx_count >= DFU_TX_REDUNDANCY_MAX)
                {
                    /* We've seen this handle on air multiple times, skip this transmit. */
                    tx_timeout_handled(&m_tx_slots[i]);
                }
                else
                {
                    if (transmit_dfu_packet(&m_tx_slots[i].dfu_packet, m_tx_slots[i].dfu_packet_len))
                    {
                        tx_timeout_handled(&m_tx_slots[i]);
                    }
                }
                m_tx_slots[i].rx_count = 0;
                tx_end_handle(&m_tx_slots[i], timeout);
                timeout = next_tx_timeout(&m_tx_slots[i]);
            }
            if (tx_slot_in_use(&m_tx_slots[i]) && TIMER_OLDER_THAN(timeout, next_timeout))
            {
                next_timeout = timeout;
            }
        }
    }
    timer_sch_reschedule(&m_tx_timer_evt, next_timeout);
}

static void abort_timeout(uint32_t timestamp, void* p_context)
{
    __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "ABORT Timeout fired @%d\n", timestamp);
    bl_cmd_t abort_cmd;
    abort_cmd.type = BL_CMD_TYPE_DFU_ABORT;
    if (nrf_mesh_dfu_cmd_send(&abort_cmd) == NRF_SUCCESS)
    {
        end_transfer(NRF_MESH_DFU_END_ERROR_TIMEOUT);
    }
}

static void timer_timeout(uint32_t timestamp, void* p_context)
{
    __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Timeout fired @%d\n", timestamp);
    bl_cmd_t timeout_cmd;
    timeout_cmd.type = BL_CMD_TYPE_TIMEOUT;
    timeout_cmd.params.timeout.timer_index = 0;
    (void) nrf_mesh_dfu_cmd_send(&timeout_cmd);
}

static void flash_op_complete(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    NRF_MESH_ASSERT(user == MESH_FLASH_USER_DFU);
    bl_cmd_t end_cmd;
    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            end_cmd.type = BL_CMD_TYPE_FLASH_WRITE_COMPLETE;
            end_cmd.params.flash.write.p_data = p_op->params.write.p_data;
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Write complete (0x%x)\n", p_op->params.write.p_data);
            break;
        case FLASH_OP_TYPE_ERASE:
            end_cmd.type = BL_CMD_TYPE_FLASH_ERASE_COMPLETE;
            end_cmd.params.flash.erase.p_dest = p_op->params.erase.p_start_addr;
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Erase complete (0x%x)\n", p_op->params.erase.p_start_addr);
            break;
        case FLASH_OP_TYPE_ALL:
            end_cmd.type = BL_CMD_TYPE_FLASH_ALL_COMPLETE;
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Flash idle.\n");
            break;
        default:
            NRF_MESH_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    }

    (void) nrf_mesh_dfu_cmd_send(&end_cmd); /* don't care about the return code */
}

/**
 * Go through the TX slots and bump the redundancy counter on any packet
 * matching the given packet.
 */
static void tx_packet_redundancy_check(const nrf_mesh_dfu_packet_t* p_packet, uint32_t length)
{
    if (p_packet->packet_type == DFU_PACKET_TYPE_DATA &&
        p_packet->payload.data.segment > 0)
    {
        for (uint32_t i = 0; i < NRF_MESH_DFU_TX_SLOTS; ++i)
        {
            if (tx_slot_in_use(&m_tx_slots[i]))
            {
                /* Check for packet match */

                if (m_tx_slots[i].dfu_packet.packet_type                 == DFU_PACKET_TYPE_DATA &&
                    m_tx_slots[i].dfu_packet.payload.data.segment        == p_packet->payload.data.segment &&
                    m_tx_slots[i].dfu_packet.payload.data.transaction_id == p_packet->payload.data.transaction_id &&
                    m_tx_slots[i].rx_count < (1 << (8 * sizeof(m_tx_slots[i].rx_count))) - 1) /* Check that we're not about to wrap around */
                {
                    uint32_t was_masked;
                    _DISABLE_IRQS(was_masked);
                    m_tx_slots[i].rx_count++;
                    _ENABLE_IRQS(was_masked);
                    break;
                }
            }
        }
    }
}

static uint32_t dfu_evt_handler(const bl_evt_t* p_evt)
{
    uint32_t return_code = NRF_SUCCESS;
    switch (p_evt->type)
    {
        case BL_EVT_TYPE_ECHO:
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tEcho: %s\n", p_evt->params.echo.str);
            break;
        case BL_EVT_TYPE_DFU_ABORT:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tAbort event. Reason: 0x%x\n", p_evt->params.dfu.abort.reason);
                end_transfer(p_evt->params.dfu.abort.reason);
            }
            break;

        case BL_EVT_TYPE_DFU_NEW_FW:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tNew firmware!\n");
                nrf_mesh_evt_t evt;
                evt.type = NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH;
                evt.params.dfu.fw_outdated.transfer.dfu_type = p_evt->params.dfu.new_fw.fw_type;
                evt.params.dfu.fw_outdated.transfer.id       = p_evt->params.dfu.new_fw.fwid;
                if (get_curr_fwid(
                            p_evt->params.dfu.new_fw.fw_type,
                            &evt.params.dfu.fw_outdated.current) == NRF_SUCCESS)
                {
                    event_handle(&evt);
                }
            }
            break;

        case BL_EVT_TYPE_DFU_REQ:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tSource/relay request!\n");
                /* Forward to application */
                nrf_mesh_evt_t evt;
                switch (p_evt->params.dfu.req.role)
                {
                    case NRF_MESH_DFU_ROLE_RELAY:
                        evt.type = NRF_MESH_EVT_DFU_REQ_RELAY;
                        evt.params.dfu.req_relay.transfer.dfu_type = p_evt->params.dfu.req.dfu_type;
                        evt.params.dfu.req_relay.transfer.id = p_evt->params.dfu.req.fwid;
                        evt.params.dfu.req_relay.authority = p_evt->params.dfu.req.authority;
                        break;
                    case NRF_MESH_DFU_ROLE_SOURCE:
                        evt.type = NRF_MESH_EVT_DFU_REQ_SOURCE;
                        evt.params.dfu.req_source.dfu_type = p_evt->params.dfu.req.dfu_type;
                        break;
                    default:
                        return NRF_ERROR_NOT_SUPPORTED;
                }
                event_handle(&evt);
            }
            break;

        case BL_EVT_TYPE_DFU_START:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tDFU start\n");
                if (p_evt->params.dfu.start.role == NRF_MESH_DFU_ROLE_TARGET)
                {
                    m_transfer_state.state = NRF_MESH_DFU_STATE_TARGET;
                }
                else
                {
                    m_transfer_state.state = NRF_MESH_DFU_STATE_RELAY;
                }
                m_transfer_state.segment_count = 0;
                m_transfer_state.role = p_evt->params.dfu.start.role;
                m_transfer_state.fwid = p_evt->params.dfu.start.fwid;
                nrf_mesh_evt_t evt;
                evt.type = NRF_MESH_EVT_DFU_START;
                evt.params.dfu.start.transfer.dfu_type = p_evt->params.dfu.start.dfu_type;
                evt.params.dfu.start.transfer.id = p_evt->params.dfu.start.fwid;
                evt.params.dfu.start.role = p_evt->params.dfu.start.role;
                event_handle(&evt);
                m_timer_evt.cb = abort_timeout;
                timer_sch_reschedule(&m_timer_evt, timer_now() + NRF_MESH_DFU_DATA_TRANSFER_TIMEOUT_US);
            }
            break;

        case BL_EVT_TYPE_DFU_DATA_SEGMENT_RX:
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tDFU segment rx: %d/%d\n",
                  p_evt->params.dfu.data_segment.received_segment,
                  p_evt->params.dfu.data_segment.total_segments);
            m_transfer_state.segment_count++;
            m_transfer_state.total_segments = p_evt->params.dfu.data_segment.total_segments;
            m_timer_evt.cb = abort_timeout;
            timer_sch_reschedule(&m_timer_evt, timer_now() + NRF_MESH_DFU_DATA_TRANSFER_TIMEOUT_US);
            break;

        case BL_EVT_TYPE_DFU_END:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tDFU END!\n");
                end_transfer(NRF_MESH_DFU_END_SUCCESS);
            }
            break;

        case BL_EVT_TYPE_BANK_AVAILABLE:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tDFU BANK AVAILABLE\n");
                nrf_mesh_evt_t evt;
                evt.type = NRF_MESH_EVT_DFU_BANK_AVAILABLE;
                evt.params.dfu.bank.transfer.dfu_type   = p_evt->params.bank_available.bank_dfu_type;
                evt.params.dfu.bank.transfer.id         = p_evt->params.bank_available.bank_fwid;
                evt.params.dfu.bank.p_start_addr        = p_evt->params.bank_available.p_bank_addr;
                evt.params.dfu.bank.length              = p_evt->params.bank_available.bank_length;
                evt.params.dfu.bank.is_signed           = p_evt->params.bank_available.is_signed;
                event_handle(&evt);
            }
            break;

        case BL_EVT_TYPE_FLASH_ERASE:
            {
                if (!IS_PAGE_ALIGNED(p_evt->params.flash.erase.p_start_addr))
                {
                    return_code = NRF_ERROR_INVALID_ADDR;
                }
                else
                {
                    flash_operation_t flash_op;
                    flash_op.type = FLASH_OP_TYPE_ERASE;
                    flash_op.params.erase.p_start_addr = (uint32_t *) p_evt->params.flash.erase.p_start_addr;
                    flash_op.params.erase.length = p_evt->params.flash.erase.length;
                    return_code = mesh_flash_op_push(MESH_FLASH_USER_DFU, &flash_op, NULL);
                }
            }
            break;

        case BL_EVT_TYPE_FLASH_WRITE:
            {
                if (!IS_WORD_ALIGNED(p_evt->params.flash.write.p_start_addr) ||
                    !IS_WORD_ALIGNED(p_evt->params.flash.write.p_data))
                {
                    return_code = NRF_ERROR_INVALID_ADDR;
                }
                else if (!IS_WORD_ALIGNED(p_evt->params.flash.write.length))
                {
                    return_code = NRF_ERROR_INVALID_LENGTH;
                }
                else
                {
                    flash_operation_t flash_op;
                    flash_op.type = FLASH_OP_TYPE_WRITE;
                    flash_op.params.write.p_start_addr = p_evt->params.flash.write.p_start_addr;
                    flash_op.params.write.p_data = (const uint32_t *) p_evt->params.flash.write.p_data;
                    flash_op.params.write.length = p_evt->params.flash.write.length;
                    return_code = mesh_flash_op_push(MESH_FLASH_USER_DFU, &flash_op, NULL);
                }
            }
            break;

        case BL_EVT_TYPE_TX_RADIO:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tRADIO TX! SLOT %d, count %d, interval: %s, handle: %x\n",
                        p_evt->params.tx.radio.tx_slot,
                        p_evt->params.tx.radio.tx_count,
                        p_evt->params.tx.radio.interval_type == BL_RADIO_INTERVAL_TYPE_EXPONENTIAL ? "exponential" : "periodic",
                        p_evt->params.tx.radio.p_dfu_packet->packet_type
                     );

                dfu_tx_t* p_tx_slot = &m_tx_slots[p_evt->params.tx.radio.tx_slot];

                if (tx_slot_in_use(p_tx_slot))
                {
                    __LOG(LOG_SRC_DFU, LOG_LEVEL_WARN, "Killing a TX slot prematurely (repeats done: %u).\n", p_tx_slot->tx_count);
                }
                /* Disable the slot */
                p_tx_slot->repeats = 0;

                uint32_t time_now = timer_now();

                /* Fill the TX slot. */
                p_tx_slot->interval_type = p_evt->params.tx.radio.interval_type;
                p_tx_slot->tx_count = 0;
                p_tx_slot->rx_count = 0;
                p_tx_slot->tx_randomization_offset_us = 0;
                p_tx_slot->order_time = time_now + DFU_TX_TIMER_MARGIN_US + (rand_prng_get(&m_prng) & DFU_TX_DELAY_RANDOMIZATION_MASK_US);
                p_tx_slot->dfu_packet_len = p_evt->params.tx.radio.length;
                memcpy(&p_tx_slot->dfu_packet, p_evt->params.tx.radio.p_dfu_packet, p_evt->params.tx.radio.length);

                p_tx_slot->repeats = p_evt->params.tx.radio.tx_count;

                /* Fire away */
                if (m_tx_timer_evt.state == TIMER_EVENT_STATE_UNUSED || TIMER_OLDER_THAN(p_tx_slot->order_time, m_tx_timer_evt.timestamp))
                {
                    timer_sch_reschedule(&m_tx_timer_evt, p_tx_slot->order_time);
                }
            }
            break;
#ifdef NRF_MESH_SERIAL_ENABLE
        case BL_EVT_TYPE_TX_SERIAL:
            {
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tSERIAL TX (type: %u)\n", p_evt->params.tx.serial.p_dfu_packet->packet_type);

                serial_packet_t *p_packet;
                if (serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + p_evt->params.tx.serial.length, &p_packet) != NRF_SUCCESS)
                {
                    break;
                }

                p_packet->opcode = SERIAL_OPCODE_CMD_OPENMESH_DFU_DATA,
                memcpy(p_packet->payload.cmd.openmesh.dfu_data.dfu_packet,
                       p_evt->params.tx.serial.p_dfu_packet,
                       p_evt->params.tx.serial.length);
                serial_tx(p_packet);
            }
            break;
#endif
        case BL_EVT_TYPE_TX_ABORT:
            if (p_evt->params.tx.abort.tx_slot >= NRF_MESH_DFU_TX_SLOTS)
            {
                return_code = NRF_ERROR_INVALID_PARAM;
            }
            else if (!tx_slot_in_use(&m_tx_slots[p_evt->params.tx.abort.tx_slot]))
            {
                return_code = NRF_ERROR_INVALID_STATE;
            }
            else
            {
                m_tx_slots[p_evt->params.tx.abort.tx_slot].repeats = 0;
            }
            break;

        case BL_EVT_TYPE_TIMER_SET:
            {
                m_timer_evt.cb = timer_timeout;
                timestamp_t set_time = timer_now() + p_evt->params.timer.set.delay_us;
                timer_sch_reschedule(&m_timer_evt, set_time);
                __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tTIMER set: %uus delay (@%u)\n", p_evt->params.timer.set.delay_us, set_time);
            }
            break;

        case BL_EVT_TYPE_TIMER_ABORT:
            __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "\tTIMER abort: %d\n", p_evt->params.timer.abort.index);
            timer_sch_abort(&m_timer_evt);
            break;

        case BL_EVT_TYPE_ERROR:
            __LOG(LOG_SRC_DFU, LOG_LEVEL_ERROR, "\tBL ERROR: %x @%s:L%u\n",
                    p_evt->params.error.error_code,
                    p_evt->params.error.p_file,
                    p_evt->params.error.line);
            NRF_MESH_ASSERT(false);
            break;
        default:
            __LOG(LOG_SRC_DFU, LOG_LEVEL_WARN, "\tUNSUPPORTED EVENT\n");
            return_code = NRF_ERROR_NOT_SUPPORTED;
            break;
    }
    return return_code;
}

#if !defined(HOST)
static uint32_t dfu_cmd_handler_set(bl_if_cmd_handler_t handler)
{
    if (handler == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (BOOTLOADERADDR() == 0xFFFFFFFF)
    {
        /* No bootloader present. */
        return NRF_ERROR_NOT_SUPPORTED;
    }
    if (((uint32_t) handler >= DEVICE_FLASH_END_GET()) ||
        ((uint32_t) handler < BOOTLOADERADDR()))
    {
        /* out of bounds. */
        return NRF_ERROR_INVALID_ADDR;
    }
    m_cmd_handler = handler;
    return NRF_SUCCESS;
}
#endif

static void tx_complete_cb(broadcast_params_t * p_broadcast, uint32_t timestamp)
{
    /* Do nothing */
}
/*****************************************************************************
* Interface functions
*****************************************************************************/

uint32_t nrf_mesh_dfu_init(void)
{
    uint32_t error_code;
#if !defined(HOST)
    error_code = dfu_cmd_handler_set(*((bl_if_cmd_handler_t*) (DEVICE_DATA_RAM_END_GET() - sizeof(bl_if_cmd_handler_t))));
    if (error_code != NRF_SUCCESS)
    {
        m_transfer_state.state = NRF_MESH_DFU_STATE_INITIALIZED;
        return NRF_ERROR_NOT_SUPPORTED;
    }
#else
    /* Must have been set externally */
    NRF_MESH_ASSERT(m_cmd_handler != NULL);
#endif

    rand_prng_seed(&m_prng);

    m_timer_evt.cb           = timer_timeout;
    m_timer_evt.interval     = 0;
    m_timer_evt.p_context    = NULL;
    m_timer_evt.p_next       = NULL;
    m_tx_timer_evt.cb        = tx_timeout;
    m_tx_timer_evt.interval  = 0;
    m_tx_timer_evt.p_context = NULL;
    m_tx_timer_evt.p_next    = NULL;

    bl_cmd_t init_cmd =
    {
        .type = BL_CMD_TYPE_INIT,
        .params.init.bl_if_version = BL_IF_VERSION,
        .params.init.event_callback = dfu_evt_handler,
        .params.init.timer_count = 1,
        .params.init.tx_slots = NRF_MESH_DFU_TX_SLOTS,
        .params.init.in_app = true
    };

    error_code = nrf_mesh_dfu_cmd_send(&init_cmd);

    if (error_code != NRF_SUCCESS)
    {
        m_transfer_state.state = NRF_MESH_DFU_STATE_INITIALIZED;
        return error_code;
    }

    build_adv_packet();

    m_broadcast.params.access_address = BEARER_ACCESS_ADDR_DEFAULT;
    m_broadcast.params.channel_count = sizeof(m_adv_channels) / sizeof(m_adv_channels[0]);
    m_broadcast.params.p_channels = m_adv_channels;
    m_broadcast.params.radio_config.radio_mode = RADIO_MODE_BLE_1MBIT;
    m_broadcast.params.radio_config.payload_maxlen = BLE_ADV_PACKET_OVERHEAD + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH;
    m_broadcast.params.radio_config.tx_power = RADIO_POWER_NRF_0DBM;
    m_broadcast.params.tx_complete_cb = tx_complete_cb;
    m_broadcast.params.p_packet = &m_adv_packet;

    bl_cmd_t fwid_cmd;
    fwid_cmd.type = BL_CMD_TYPE_INFO_GET;
    fwid_cmd.params.info.get.type = BL_INFO_TYPE_VERSION;
    fwid_cmd.params.info.get.p_entry = NULL;
    error_code = nrf_mesh_dfu_cmd_send(&fwid_cmd);
    if (error_code != NRF_SUCCESS)
    {
        /* no version info */
        m_transfer_state.state = NRF_MESH_DFU_STATE_INITIALIZED;
        return error_code;
    }

    mp_curr_fwid = &fwid_cmd.params.info.get.p_entry->version;

    __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Version info: BL(ver, id): 0x%02x 0x%02x SD: 0x%04x\n",
    mp_curr_fwid->bootloader.bl_version, mp_curr_fwid->bootloader.bl_id, mp_curr_fwid->sd);
    __LOG(LOG_SRC_DFU, LOG_LEVEL_INFO, "Version info: APP company: 0x%08x id: 0x%04x version: 0x%08x\n",
    mp_curr_fwid->app.company_id, mp_curr_fwid->app.app_id, mp_curr_fwid->app.app_version);

    mesh_flash_user_callback_set(MESH_FLASH_USER_DFU, flash_op_complete);

    m_transfer_state.state = NRF_MESH_DFU_STATE_INITIALIZED;
    return error_code;
}

uint32_t nrf_mesh_dfu_enable(void)
{
    bl_cmd_t enable_cmd =
    {
        .type = BL_CMD_TYPE_ENABLE,
        .params = {{0}}
    };
    return nrf_mesh_dfu_cmd_send(&enable_cmd);
}

uint32_t nrf_mesh_dfu_jump_to_bootloader(void)
{
    if (BOOTLOADERADDR() != 0xFFFFFFFF)
    {
        interrupts_disable();
        hal_device_reset(BL_GPREGRET_FORCED_REBOOT);
        return NRF_SUCCESS; /* unreachable */
    }
    else
    {
        /* the BOOTLOADERADDR() isn't set, and we have no way to find the bootloader-address. */
        return NRF_ERROR_FORBIDDEN;
    }
}

uint32_t nrf_mesh_dfu_rx(const uint8_t * p_packet,
                         uint32_t length,
                         const nrf_mesh_rx_metadata_t * p_metadata)
{
    const nrf_mesh_dfu_packet_t * p_dfu_packet = (const nrf_mesh_dfu_packet_t *) p_packet;
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_dfu_packet == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (p_dfu_packet->packet_type < DFU_HANDLE_RANGE_START)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (mesh_flash_in_progress())
    {
        return NRF_ERROR_BUSY;
    }

    tx_packet_redundancy_check(p_dfu_packet, length);

    bl_cmd_t rx_cmd =
    {
        .type = BL_CMD_TYPE_RX,
        .params.rx.p_dfu_packet = p_dfu_packet,
        .params.rx.length = length
    };

    return nrf_mesh_dfu_cmd_send(&rx_cmd);
}

uint32_t nrf_mesh_dfu_request(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid,
        const uint32_t* p_bank_addr)
{
    if (m_transfer_state.state != NRF_MESH_DFU_STATE_INITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_fwid == NULL || p_bank_addr == NULL)
    {
        return NRF_ERROR_NULL;
    }
    bl_cmd_t cmd;
    cmd.type = BL_CMD_TYPE_DFU_START_TARGET;
    cmd.params.dfu.start.target.type = type;
    cmd.params.dfu.start.target.fwid = *p_fwid;
    cmd.params.dfu.start.target.p_bank_start = p_bank_addr;
    uint32_t error_code = nrf_mesh_dfu_cmd_send(&cmd);
    if (error_code == NRF_SUCCESS)
    {
        m_transfer_state.role = NRF_MESH_DFU_ROLE_TARGET;
        m_transfer_state.type = type;
        m_transfer_state.fwid = *p_fwid;
        m_transfer_state.state = NRF_MESH_DFU_STATE_DFU_REQ;
        m_transfer_state.segment_count = 0;
        m_transfer_state.total_segments = 0;
        m_timer_evt.cb = abort_timeout;
        timer_sch_reschedule(&m_timer_evt, timer_now() + NRF_MESH_DFU_REQ_TIMEOUT_US);
    }
    return error_code;
}

uint32_t nrf_mesh_dfu_relay(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid)
{
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_fwid == NULL)
    {
        return NRF_ERROR_NULL;
    }
    bl_cmd_t cmd;
    cmd.type = BL_CMD_TYPE_DFU_START_RELAY;
    cmd.params.dfu.start.relay.type = type;
    cmd.params.dfu.start.relay.fwid = *p_fwid;
    cmd.params.dfu.start.relay.transaction_id = 0;
    uint32_t error_code = nrf_mesh_dfu_cmd_send(&cmd);
    if (error_code == NRF_SUCCESS)
    {
        m_transfer_state.role = NRF_MESH_DFU_ROLE_RELAY;
        m_transfer_state.type = type;
        m_transfer_state.fwid = *p_fwid;
        m_transfer_state.state = NRF_MESH_DFU_STATE_RELAY_CANDIDATE;
        m_transfer_state.segment_count = 0;
        m_transfer_state.total_segments = 0;
        m_timer_evt.cb = abort_timeout;
        timer_sch_reschedule(&m_timer_evt, timer_now() + NRF_MESH_DFU_RELAY_TIMEOUT_US);
    }
    return error_code;
}

uint32_t nrf_mesh_dfu_abort(void)
{
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    bl_cmd_t cmd;
    cmd.type = BL_CMD_TYPE_DFU_ABORT;
    uint32_t error_code = nrf_mesh_dfu_cmd_send(&cmd);
    if (error_code == NRF_SUCCESS)
    {
        reset_transfer_state();
    }
    return error_code;
}

uint32_t nrf_mesh_dfu_bank_info_get(nrf_mesh_dfu_type_t type, nrf_mesh_dfu_bank_info_t* p_bank_info)
{
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_bank_info == NULL)
    {
        return NRF_ERROR_NULL;
    }
    bl_cmd_t cmd;
    cmd.type = BL_CMD_TYPE_DFU_BANK_INFO_GET;
    cmd.params.dfu.bank_info_get.bank_dfu_type = type;
    cmd.params.dfu.bank_info_get.p_bank_info = p_bank_info;
    return nrf_mesh_dfu_cmd_send(&cmd);
}

uint32_t nrf_mesh_dfu_bank_flash(nrf_mesh_dfu_type_t bank_type)
{
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    bl_cmd_t cmd;
    cmd.type = BL_CMD_TYPE_DFU_BANK_FLASH;
    cmd.params.dfu.bank_flash.bank_dfu_type = bank_type;
    return nrf_mesh_dfu_cmd_send(&cmd);
}

uint32_t nrf_mesh_dfu_state_get(nrf_mesh_dfu_transfer_state_t* p_dfu_transfer_state)
{
    if (m_transfer_state.state == NRF_MESH_DFU_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_dfu_transfer_state == NULL)
    {
        return NRF_ERROR_NULL;
    }
    p_dfu_transfer_state->role  = m_transfer_state.role;
    p_dfu_transfer_state->type  = m_transfer_state.type;
    p_dfu_transfer_state->fwid  = m_transfer_state.fwid;
    p_dfu_transfer_state->state = m_transfer_state.state;
    p_dfu_transfer_state->data_progress = (uint8_t) (
            ((uint32_t) m_transfer_state.segment_count * 100) /
            ((uint32_t) m_transfer_state.total_segments));
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_dfu_cmd_send(bl_cmd_t* p_cmd)
{
    if (m_cmd_handler == NULL)
    {
        __LOG(LOG_SRC_DFU, LOG_LEVEL_ERROR, "ERROR: No CMD handler!\n");
        return NRF_ERROR_NOT_SUPPORTED;
    }
    if (p_cmd == NULL)
    {
        return NRF_ERROR_NULL;
    }
    /* Suspend flash operations for the duration of the command to avoid
       premature flash-events disturbing the flow */
    mesh_flash_set_suspended(true);
    uint32_t error_code = m_cmd_handler(p_cmd);
    mesh_flash_set_suspended(false);
    return error_code;
}
