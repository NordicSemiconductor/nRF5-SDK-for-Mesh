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

#include "nrf_mesh_sdk.h"
#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "nrf.h"
#include "log.h"

#include "boards.h"

#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
#include "stack_depth.h"
#endif

#if SD_BLE_API_VERSION >= 4
static uint8_t m_ble_evt_buffer[sizeof(ble_evt_t) + BLE_GATT_ATT_MTU_DEFAULT];
#else
static uint8_t m_ble_evt_buffer[sizeof(ble_evt_t) + GATT_MTU_SIZE_DEFAULT];
#endif

/********** Fatal Error Reporting **********/

static void sleep_forever(uint32_t pc)
{
    NRF_GPIO->OUTSET = LEDS_MASK;
    NRF_GPIO->OUTCLR = LEDS_MASK;
    uint32_t irqs_masked __attribute__((unused));
    _DISABLE_IRQS(irqs_masked);
    while (pc)
    {
        /* Sleep forever */
    }
}

void app_error_handler(uint32_t error_code, uint32_t line_number, const uint8_t * filename)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "APP_ERROR: %s:%lu: code %lu\n", filename, line_number, error_code);
    sleep_forever(1);
}

void mesh_assert_handler(uint32_t pc)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ASSERT, "MESH ASSERT at 0x%.08lx\n", pc);
    sleep_forever(pc);
}

void HardFault_Handler(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ASSERT, "HARDFAULT...\n");
    sleep_forever(1);
}

void SD_EVT_IRQHandler(void)
{
    /* Fetch SOC events. */
    uint32_t evt_id;
    while (sd_evt_get(&evt_id) == NRF_SUCCESS)
    {
        nrf_mesh_on_sd_evt(evt_id);
    }

    /* Fetch BLE events. */
    uint16_t evt_len = sizeof(m_ble_evt_buffer);
    while (sd_ble_evt_get(&m_ble_evt_buffer[0], &evt_len) == NRF_SUCCESS)
    {
        nrf_mesh_on_ble_evt((ble_evt_t*) &m_ble_evt_buffer[0]);
        evt_len = sizeof(m_ble_evt_buffer);
    }
}

#if defined(S130) || defined(S132) || defined(S140)
#include "nrf_nvic.h"
nrf_nvic_state_t nrf_nvic_state;

void softdevice_assert_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Softdevice assert: %u:%u:%u\n", id, pc, info);
    sleep_forever(pc);
}

#if SD_BLE_API_VERSION >= 4
uint32_t mesh_softdevice_enable(void)
#else
uint32_t mesh_softdevice_enable(ble_enable_params_t * p_ble_enable_params)
#endif
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing SoftDevice...\n");
#if defined ( __CC_ARM )
    extern uint32_t Image$$RW_IRAM1$$Base;
    const volatile uint32_t ram_start = (uint32_t) &Image$$RW_IRAM1$$Base;
#elif defined   ( __GNUC__ )
    extern uint32_t __data_start__;
    volatile uint32_t ram_start = (uint32_t) &__data_start__;
#endif
    uint32_t app_ram_base = ram_start;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Ram base: 0x%x\n", ram_start);

#if SD_BLE_API_VERSION >= 4
    uint32_t error_code = sd_ble_enable(&app_ram_base);
#else
    uint32_t error_code = sd_ble_enable(p_ble_enable_params, &app_ram_base);
#endif
    if (app_ram_base != ram_start)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "sd_ble_enable: app_ram_base should be adjusted to 0x%0x\n", app_ram_base);
    }
    return error_code;
}

uint32_t mesh_softdevice_setup(nrf_clock_lf_cfg_t lfc_cfg)
{
    ERROR_CHECK(sd_softdevice_enable(&lfc_cfg, softdevice_assert_handler));

    /* Make sure SoftDevice events are handled in the same IRQ priority as the other Mesh event
     * handling (in S132 v5.0.0, S112 v5.0.0, S212 v5.0.0 and S332 v5.0.0 the priority is set to 6
     * by default). */
    ERROR_CHECK(sd_nvic_SetPriority(SD_EVT_IRQn, NRF_MESH_IRQ_PRIORITY_LOWEST));

    ERROR_CHECK(sd_nvic_EnableIRQ(SD_EVT_IRQn));

#if SD_BLE_API_VERSION >= 4
    ERROR_CHECK(mesh_softdevice_enable());
#else
    ble_enable_params_t ble_enable_params = {{0}};
    ERROR_CHECK(mesh_softdevice_enable(&ble_enable_params));
#endif

    return NRF_SUCCESS;
}

#elif defined(S110)
void softdevice_assert_handler(uint32_t pc, uint16_t line_number, const uint8_t * p_filename)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ASSERT, "SD ASSERT: %s:%hu at 0x%.08lx\n", p_filename, line_number, pc);
    sleep_forever(pc);
}

uint32_t mesh_softdevice_setup(nrf_clock_lfclksrc_t lfccfg)
{
    ERROR_CHECK(sd_softdevice_enable(lfccfg, softdevice_assert_handler));
    ERROR_CHECK(sd_nvic_EnableIRQ(SD_EVT_IRQn));

    ble_enable_params_t ble_enable_params = {{0}};
    ERROR_CHECK(sd_ble_enable(&ble_enable_params));

    return NRF_SUCCESS;
}
#endif

void mesh_core_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing softdevice\n");
#if  SD_BLE_API_VERSION >= 5
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM};
#elif SD_BLE_API_VERSION >= 2
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
#elif defined(S110)
    nrf_clock_lfclksrc_t lfc_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif
    mesh_softdevice_setup(lfc_cfg);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing mesh stack\n");
    nrf_mesh_init_params_t mesh_init_params = {
        .lfclksrc = lfc_cfg,
        .assertion_handler = mesh_assert_handler,
        .irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST
    };
    ERROR_CHECK(nrf_mesh_init(&mesh_init_params));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling mesh stack\n");
    ERROR_CHECK(nrf_mesh_enable());
}
