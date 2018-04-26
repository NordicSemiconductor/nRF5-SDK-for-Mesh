/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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


#include <stdio.h>

#include "nrf.h"
#include "ble.h"
#include "boards.h"

#include "nrf_mesh.h"
#include "log.h"
#include "nrf_mesh_node_config.h"

#include "nrf_mesh_sdk.h"
#include "simple_hal.h"

/* For beaconing advertiser */
#include "advertiser.h"


#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
#include "stack_depth.h"
#endif

#define ADVERTISER_BUFFER_SIZE  (128)

#define LED_PIN_NUMBER (BSP_LED_0)
#define LED_PIN_MASK   (1u << LED_PIN_NUMBER)

#define STATIC_AUTH_DATA { 0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73 }

/** Single advertiser instance. May periodically transmit one packet at a time. */
static advertiser_t m_advertiser;
static uint8_t m_adv_buffer[ADVERTISER_BUFFER_SIZE];

static void rx_callback(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    LEDS_OFF(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
    char msg[128];
    (void) sprintf(msg, "RX [@%u]: RSSI: %3d ADV TYPE: %x ADDR: [%02x:%02x:%02x:%02x:%02x:%02x]",
                   p_rx_data->p_metadata->params.scanner.timestamp,
                   p_rx_data->p_metadata->params.scanner.rssi,
                   p_rx_data->adv_type,
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[0],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[1],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[2],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[3],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[4],
                   p_rx_data->p_metadata->params.scanner.adv_addr.addr[5]);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, msg, p_rx_data->p_payload, p_rx_data->length);
    LEDS_ON(BSP_LED_0_MASK);  /* @c LED_RGB_RED_MASK on pca10031 */
}

static void init_advertiser(void)
{
    advertiser_instance_init(&m_advertiser, NULL, m_adv_buffer, ADVERTISER_BUFFER_SIZE);
}

static void start_advertiser(void)
{
    advertiser_enable(&m_advertiser);
    static const uint8_t adv_data[] =
    {
        0x11, /* AD data length (including type, but not itself) */
        0x09, /* AD data type (Complete local name) */
        'N',  /* AD data payload (Name of device) */
        'o',
        'r',
        'd',
        'i',
        'c',
        ' ',
        'S',
        'e',
        'm',
        'i',
        ' ',
        'M',
        'e',
        's',
        'h'
    };

    /* Allocate packet */
    adv_packet_t * p_packet = advertiser_packet_alloc(&m_advertiser, sizeof(adv_data));
    if (p_packet)
    {
        /* Construct packet contents */
        memcpy(p_packet->packet.payload, adv_data, sizeof(adv_data));
        /* Repeat forever */
        p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;

        advertiser_packet_send(&m_advertiser, p_packet);
    }

}

static void configuration_setup(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    /*
    Add model initialization here, if you wish to support a mesh model on this node.
    */
    hal_led_mask_set(LEDS_MASK, true);
}

static void provisioning_complete(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(LED_PIN_MASK, 200, 4);
}

int main(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
    {
        nrf_gpio_pin_set(i);
    }

    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_INFO, log_callback_rtt);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Bluetooth Mesh Beacon Example -----\n");

    static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
    static nrf_mesh_node_config_params_t config_params =
        {.prov_caps = NRF_MESH_PROV_OOB_CAPS_DEFAULT(ACCESS_ELEMENT_COUNT)};
    config_params.p_static_data = static_auth_data;
    config_params.complete_callback = provisioning_complete;
    config_params.setup_callback = configuration_setup;
    config_params.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST;

#if defined(S110)
    config_params.lf_clk_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#elif SD_BLE_API_VERSION >= 5
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
#else
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
#endif

    ERROR_CHECK(nrf_mesh_node_config(&config_params));

    /* Start listening for incoming packets */
    nrf_mesh_rx_cb_set(rx_callback);
    /* Start Advertising own beacon */
    init_advertiser();
    start_advertiser();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");

    while (true)
    {
        (void)sd_app_evt_wait();
    }
}
