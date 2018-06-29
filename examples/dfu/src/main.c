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

#include <boards.h>
#include "nrf_mesh_events.h"
#include "log.h"
#include "nrf_mesh_dfu.h"
#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "mesh_softdevice_init.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "simple_hal.h"
#include "app_timer.h"

#define LED_BLINK_INTERVAL_MS (200)
#define LED_BLINK_CNT_RESET   (3)

#ifndef NRF_MESH_SERIAL_ENABLE
#define NRF_MESH_SERIAL_ENABLE 1
#endif

#if NRF_MESH_SERIAL_ENABLE
#include "nrf_mesh_serial.h"
#endif

#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
#include "stack_depth.h"
#endif


#define STATIC_AUTH_DATA {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

#if defined(NRF51)
    #define FLASH_PAGE_SIZE                 ( 0x400)
    #define FLASH_PAGE_MASK             (0xFFFFFC00)
#elif defined(NRF52_SERIES)
    #define FLASH_PAGE_SIZE                 (0x1000)
    #define FLASH_PAGE_MASK             (0xFFFFF000)
#endif

#if defined(_lint)
    const volatile uint32_t * rom_base   = NULL;
    const volatile uint32_t * rom_length = NULL;
    uint32_t rom_end;
    uint32_t bank_addr;
#elif defined ( __CC_ARM )
    extern uint32_t Image$$ER_IROM1$$Base;
    extern uint32_t Image$$ER_IROM1$$Length;
    const volatile uint32_t * rom_base   = &Image$$ER_IROM1$$Base;
    const volatile uint32_t * rom_length = &Image$$ER_IROM1$$Length;
    uint32_t rom_end;
    uint32_t bank_addr;
#elif defined   ( __GNUC__ )
    extern uint32_t _start;
    extern uint32_t __exidx_end;
    const volatile uint32_t rom_base   = (uint32_t) &_start;
    const volatile uint32_t rom_end    = (uint32_t) &__exidx_end;
    uint32_t rom_length;
    uint32_t bank_addr;
#endif

static nrf_mesh_evt_handler_t m_evt_handler;
static bool m_device_provisioned;


static bool fw_updated_event_is_for_me(const nrf_mesh_evt_dfu_t * p_evt)
{
    switch (p_evt->fw_outdated.transfer.dfu_type)
    {
        case NRF_MESH_DFU_TYPE_APPLICATION:
            return (p_evt->fw_outdated.current.application.app_id == p_evt->fw_outdated.transfer.id.application.app_id &&
                    p_evt->fw_outdated.current.application.company_id == p_evt->fw_outdated.transfer.id.application.company_id &&
                    p_evt->fw_outdated.current.application.app_version < p_evt->fw_outdated.transfer.id.application.app_version);

        case NRF_MESH_DFU_TYPE_BOOTLOADER:
            return (p_evt->fw_outdated.current.bootloader.bl_id == p_evt->fw_outdated.transfer.id.bootloader.bl_id &&
                    p_evt->fw_outdated.current.bootloader.bl_version < p_evt->fw_outdated.transfer.id.bootloader.bl_version);

        case NRF_MESH_DFU_TYPE_SOFTDEVICE:
            return false;

        default:
            return false;
    }
}

static void mesh_evt_handler(const nrf_mesh_evt_t* p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED:
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH:
            if (fw_updated_event_is_for_me(&p_evt->params.dfu))
            {
                ERROR_CHECK(nrf_mesh_dfu_request(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                                 &p_evt->params.dfu.fw_outdated.transfer.id,
                                                 (uint32_t*) bank_addr));
                hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            }
            else
            {
                ERROR_CHECK(nrf_mesh_dfu_relay(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                               &p_evt->params.dfu.fw_outdated.transfer.id));
            }
            break;

        case NRF_MESH_EVT_DFU_START:
            hal_led_mask_set(BSP_LED_0_MASK | BSP_LED_2_MASK, true);
            break;

        case NRF_MESH_EVT_DFU_END:
            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            hal_led_mask_set(BSP_LED_0_MASK | BSP_LED_1_MASK, true); /* Yellow */
            break;

        case NRF_MESH_EVT_DFU_BANK_AVAILABLE:
            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            ERROR_CHECK(nrf_mesh_dfu_bank_flash(p_evt->params.dfu.bank.transfer.dfu_type));
            break;

        default:
            break;

    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));

#if NRF_MESH_SERIAL_ENABLE
    ERROR_CHECK(nrf_mesh_serial_init(NULL));
#endif

    m_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

static void initialize(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    __LOG_INIT(LOG_MSK_DEFAULT | LOG_SRC_DFU | LOG_SRC_APP | LOG_SRC_SERIAL, LOG_LEVEL_INFO, log_callback_rtt);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Bluetooth Mesh DFU Example -----\n");

#if defined ( __CC_ARM )
    rom_end    = (uint32_t) rom_base + (uint32_t) rom_length;
#elif defined   ( __GNUC__ )
    rom_length = (uint32_t) rom_end - rom_base;
#endif
    /* Take the next available page address */
    bank_addr  = (uint32_t) (rom_end & FLASH_PAGE_MASK) + FLASH_PAGE_SIZE;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG2, "rom_base   %X\n", rom_base);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG2, "rom_end    %X\n", rom_end);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG2, "rom_length %X\n", rom_length);
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG2, "bank_addr   %X\n", bank_addr);

    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");
}

static void start(void)
{
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data = static_auth_data,
            .prov_complete_cb = NULL,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

#if NRF_MESH_SERIAL_ENABLE
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling serial interface...\n");
    ERROR_CHECK(nrf_mesh_serial_enable());
#endif

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "DFU example started!\n");
}

int main(void)
{
    initialize();
    execution_start(start);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
