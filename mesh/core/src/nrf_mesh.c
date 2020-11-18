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
#include "nrf_mesh.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_utils.h"

#include <string.h>
#include <stddef.h>

#include "nrf.h"
#include <nrf_error.h>
#include "nrf_sdm.h"

#include "bearer_event.h"
#include "enc.h"
#include "event.h"
#include "fifo.h"
#include "msg_cache.h"
#include "network.h"
#include "net_state.h"
#include "packet.h"
#include "nrf_mesh_dfu.h"
#include "dfu_types_internal.h"
#include "timer_scheduler.h"
#include "timeslot.h"
#include "toolchain.h"
#include "transport.h"
#include "beacon.h"
#include "dfu_types_internal.h"
#include "list.h"
#include "utils.h"
#include "log.h"
#include "mesh_flash.h"
#include "scanner.h"
#include "ad_listener.h"
#include "mesh_mem.h"
#include "core_tx_adv.h"
#include "core_tx_instaburst.h"
#include "core_tx_lpn.h"
#include "core_tx_local.h"
#include "instaburst_rx.h"
#include "heartbeat.h"
#include "prov_bearer_adv.h"
#include "mesh_config.h"
#include "mesh_opt.h"
#include "ad_type_filter.h"

#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "proxy.h"
#endif  /* MESH_FEATURE_GATT_PROXY_ENABLED */

static void config_stable_catch(const nrf_mesh_evt_t * p_evt);

static enum
{
    MESH_STATE_UNINITIALIZED, /**< Uninitialized state. */
    MESH_STATE_INITIALIZED,   /**< Initialized, but never enabled. */
    MESH_STATE_ENABLING,      /**< Enabling in progress. */
    MESH_STATE_ENABLED,       /**< Enabled state. */
    MESH_STATE_DISABLING,     /**< Disabling in progress. */
    MESH_STATE_DISABLED,      /**< Disabled by user. */
} m_mesh_state;
static nrf_mesh_rx_cb_t m_rx_cb;
static bearer_event_flag_t m_mesh_be_flag;
static nrf_mesh_evt_handler_t m_config_stable_waiter =
{
    .evt_cb = config_stable_catch
};

/** Unique Tx token. */
static nrf_mesh_tx_token_t m_tx_token = NRF_MESH_INITIAL_TOKEN;

static void packet_in(const uint8_t * p_packet,
                      uint32_t ad_packet_length,
                      const nrf_mesh_rx_metadata_t * p_metadata)
{
    uint32_t status = network_packet_in(p_packet, ad_packet_length, p_metadata);
    if (status != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_API, LOG_LEVEL_WARN, "[er%d] Could not process mesh packet...\n", status);
    }
}

AD_LISTENER(m_mesh_packet_listener) = {
    .ad_type = AD_TYPE_MESH,
    .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND,
    .handler = packet_in,
};

#if EXPERIMENTAL_INSTABURST_ENABLED
AD_LISTENER(m_instaburst_mesh_packet_listener) = {
    .ad_type = AD_TYPE_MESH,
    .adv_packet_type = BLE_PACKET_TYPE_ADV_EXT,
    .handler = packet_in,
};
#endif

static bool scanner_packet_process_cb(void)
{
    /* Process all incoming packets: */
    const scanner_packet_t * p_scanner_packet = scanner_rx();

    if (p_scanner_packet != NULL)
    {
        nrf_mesh_rx_metadata_t metadata;

        metadata.source = NRF_MESH_RX_SOURCE_SCANNER;
        metadata.params.scanner = p_scanner_packet->metadata;

        /* Adv Ext packets in the advertising channels don't have regular advertising data */
        if (p_scanner_packet->packet.header.length >= BLE_ADV_PACKET_OVERHEAD &&
            p_scanner_packet->packet.header.type != BLE_PACKET_TYPE_ADV_EXT)
        {
            ad_listener_process((ble_packet_type_t) p_scanner_packet->packet.header.type,
                                p_scanner_packet->packet.payload,
                                p_scanner_packet->packet.header.length - BLE_ADV_PACKET_OVERHEAD,
                                &metadata);
        }

        /* Notify the application */
        if (m_rx_cb)
        {
            nrf_mesh_adv_packet_rx_data_t rx_data;
            rx_data.p_metadata = &metadata;
            rx_data.adv_type = p_scanner_packet->packet.header.type;
            if (p_scanner_packet->packet.header.length > BLE_ADV_PACKET_OVERHEAD)
            {
                rx_data.length = p_scanner_packet->packet.header.length - BLE_ADV_PACKET_OVERHEAD;
                rx_data.p_payload = p_scanner_packet->packet.payload;
            }
            else
            {
                rx_data.length = 0;
                rx_data.p_payload = NULL;
            }

            m_rx_cb(&rx_data);
        }

        scanner_packet_release(p_scanner_packet);
    }

    return !scanner_rx_pending();
}

#if EXPERIMENTAL_INSTABURST_ENABLED
static bool instaburst_packet_process_cb(void)
{
    /* Process all incoming packets: */
    const instaburst_rx_packet_t * p_packet = instaburst_rx();

    if (p_packet != NULL)
    {
        nrf_mesh_rx_metadata_t metadata;

        metadata.source = NRF_MESH_RX_SOURCE_INSTABURST;
        metadata.params.instaburst = p_packet->metadata;

        ad_listener_process(BLE_PACKET_TYPE_ADV_EXT,
                            p_packet->p_payload,
                            p_packet->payload_len,
                            &metadata);

        /* Notify the application */
        if (m_rx_cb)
        {
            nrf_mesh_adv_packet_rx_data_t rx_data;
            rx_data.p_metadata = &metadata;
            rx_data.adv_type   = BLE_PACKET_TYPE_ADV_EXT;
            rx_data.length     = p_packet->payload_len;
            rx_data.p_payload  = p_packet->p_payload;

            m_rx_cb(&rx_data);
        }

        instaburst_rx_packet_release(p_packet);
    }

    return !instaburst_rx_pending();
}
#endif

static void bearer_stopped_cb(void)
{
    if (m_mesh_state == MESH_STATE_DISABLING)
    {
        m_mesh_state = MESH_STATE_DISABLED;
        const nrf_mesh_evt_t disabled = {
            .type = NRF_MESH_EVT_DISABLED,
        };
        event_handle(&disabled);
    }
}

static void config_stable_catch(const nrf_mesh_evt_t * p_evt)
{
    if (NRF_MESH_EVT_CONFIG_STABLE == p_evt->type)
    {
        event_handler_remove(&m_config_stable_waiter);
        bearer_event_flag_set(m_mesh_be_flag);
    }
}

static bool nrf_mesh_enabled_be_cb(void)
{
    if (m_mesh_state != MESH_STATE_ENABLING)
    {
        NRF_MESH_ASSERT_DEBUG(false);
    }
    else if (!net_state_is_seqnum_block_ready())
    {
        event_handler_add(&m_config_stable_waiter);
    }
    else
    {
#if !MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
        scanner_enable();
#endif

        m_mesh_state = MESH_STATE_ENABLED;
        const nrf_mesh_evt_t enabled =
        {
            .type = NRF_MESH_EVT_ENABLED,
        };
        event_handle(&enabled);
    }

    return true;
}

uint32_t nrf_mesh_init(const nrf_mesh_init_params_t * p_init_params)
{
    if (m_mesh_state != MESH_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_init_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint8_t irq_priority = p_init_params->irq_priority;
    if ((irq_priority != NRF_MESH_IRQ_PRIORITY_THREAD) &&
        (irq_priority != NRF_MESH_IRQ_PRIORITY_LOWEST))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    mesh_mem_init();

    if (p_init_params->p_uuid != NULL)
    {
        nrf_mesh_configure_device_uuid_set(p_init_params->p_uuid);
    }
    else
    {
        nrf_mesh_configure_device_uuid_reset();
    }

#if !defined(HOST)
    uint8_t softdevice_enabled;
    uint32_t status = sd_softdevice_is_enabled(&softdevice_enabled);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (softdevice_enabled != 1)
    {
        return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
    }
#endif

#ifdef __linux__
    toolchain_init_irqs();
#endif

    msg_cache_init();
    timer_sch_init();
    bearer_event_init(irq_priority);

#if !defined(HOST)
#if NRF_SD_BLE_API_VERSION >= 5
    /* From SD BLE API 5 and on, both RC and XTAL should use the PPM-defines. */
    uint32_t lfclk_accuracy = hal_lfclk_ppm_get(p_init_params->lfclksrc.accuracy);
#elif defined(S110)
    uint32_t lfclk_accuracy = hal_lfclk_ppm_get(p_init_params->lfclksrc);
#else
    uint32_t lfclk_accuracy = (p_init_params->lfclksrc.source == NRF_CLOCK_LF_SRC_XTAL)
                                  ? hal_lfclk_ppm_get(p_init_params->lfclksrc.xtal_accuracy)
                                  : 250;

#endif
    timeslot_init(lfclk_accuracy);
#if EXPERIMENTAL_INSTABURST_ENABLED
    instaburst_init(lfclk_accuracy, instaburst_packet_process_cb);
#endif
#endif /* !HOST */
    bearer_handler_init();
    scanner_init(scanner_packet_process_cb);
    advertiser_init();
    ad_listener_init();

    mesh_flash_init();
    mesh_config_init();
    mesh_opt_init();

#if EXPERIMENTAL_INSTABURST_ENABLED
    core_tx_instaburst_init();
#else
    core_tx_adv_init();
#endif
    core_tx_local_init();
    network_init(p_init_params);
    transport_init();
    heartbeat_init();

#if !defined(HOST)
    status = nrf_mesh_dfu_init();
    if ((status != NRF_SUCCESS) && (status != NRF_ERROR_NOT_SUPPORTED))
    {
        return status;
    }
#endif

    m_mesh_be_flag = bearer_event_flag_add(nrf_mesh_enabled_be_cb);
    m_rx_cb = NULL;
    m_mesh_state = MESH_STATE_INITIALIZED;

    return NRF_SUCCESS;
}

uint32_t nrf_mesh_enable(void)
{
    uint32_t status = NRF_ERROR_INVALID_STATE;
    switch (m_mesh_state)
    {
        case MESH_STATE_INITIALIZED:
            network_enable();
            transport_enable();

#if EXPERIMENTAL_INSTABURST_ENABLED
            instaburst_rx_enable();
#endif

#if !defined(HOST)
            status = nrf_mesh_dfu_enable();
            if ((status != NRF_SUCCESS) && (status != NRF_ERROR_NOT_SUPPORTED))
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "nrf_mesh_dfu_enable() failed [0x%X]\n",status);
                return status;
            }
#endif
            bearer_event_start();
            /* fallthrough */
        case MESH_STATE_DISABLED:
        case MESH_STATE_DISABLING:
            status = bearer_handler_start();
            break;
        default:
            return NRF_ERROR_INVALID_STATE;
    }

    if (status == NRF_SUCCESS)
    {
        m_mesh_state = MESH_STATE_ENABLING;
        bearer_event_flag_set(m_mesh_be_flag);
    }
    return status;
}

uint32_t nrf_mesh_disable(void)
{
    if (m_mesh_state != MESH_STATE_ENABLED)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_mesh_state = MESH_STATE_DISABLING;
    /* This module owns the bearer handler start/stop process, and should never have called
     * bearer_handler_stop if it was in an invalid state. The bearer handler will call our callback
     * once it has been stopped (may be inline). */
    NRF_MESH_ERROR_CHECK(bearer_handler_stop(bearer_stopped_cb));

    return NRF_SUCCESS;
}

uint32_t nrf_mesh_packet_send(const nrf_mesh_tx_params_t * p_params,
                              uint32_t * const p_packet_reference)
{
    return transport_tx(p_params, p_packet_reference);
}

bool nrf_mesh_process(void)
{
    return bearer_event_handler();
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    event_handler_add(p_handler_params);
}

void nrf_mesh_evt_handler_remove(nrf_mesh_evt_handler_t * p_handler_params)
{
    event_handler_remove(p_handler_params);
}

uint32_t nrf_mesh_on_sd_evt(uint32_t sd_evt)
{
    timeslot_sd_event_handler(sd_evt);
    return NRF_SUCCESS;
}

void nrf_mesh_rx_cb_set(nrf_mesh_rx_cb_t rx_cb)
{
    m_rx_cb = rx_cb;
}

void nrf_mesh_rx_cb_clear(void)
{
    m_rx_cb = NULL;
}

void nrf_mesh_subnet_added(uint16_t net_key_index, const uint8_t * p_network_id)
{
#if MESH_FEATURE_GATT_PROXY_ENABLED
    proxy_subnet_added(net_key_index, p_network_id);
#endif
}

nrf_mesh_tx_token_t nrf_mesh_unique_token_get(void)
{
   if (++m_tx_token == NRF_MESH_SERVICE_BORDER_TOKEN)
   {
       m_tx_token = NRF_MESH_INITIAL_TOKEN;
       m_tx_token++;
   }

   return m_tx_token;
}
