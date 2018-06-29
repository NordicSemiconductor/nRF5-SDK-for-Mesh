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

#include "mesh_softdevice_init.h"

#include <string.h>

#include "ble.h"
#include "nrf_mesh.h"
#include "toolchain.h"
#include "log.h"
#include "mesh_app_utils.h"

#if NRF_SD_BLE_API_VERSION >= 2
#include "app_error_weak.h"
#endif

#if defined(S130) || defined(S132) || defined(S140) || defined(S112)
    #include "nrf_nvic.h"
#else
    #include "nrf_soc.h"
#endif


#if !defined(HOST)

void SD_EVT_IRQHandler(void)
{
    /* Fetch SOC events. */
    uint32_t evt_id;
    while (sd_evt_get(&evt_id) == NRF_SUCCESS)
    {
        (void)nrf_mesh_on_sd_evt(evt_id);
    }
}

#endif /* !defined(HOST) */

static uint32_t ble_enable(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling BLE...\n");
#if defined(_lint)
    uint32_t ram_start = 0ul;
#elif defined ( __CC_ARM )
    extern uint32_t Image$$RW_IRAM1$$Base;
    const volatile uint32_t ram_start = (uint32_t) &Image$$RW_IRAM1$$Base;
#elif defined   ( __GNUC__ )
    extern uint32_t __data_start__;
    volatile uint32_t ram_start = (uint32_t) &__data_start__;
#endif
    uint32_t app_ram_base = ram_start;

#if NRF_SD_BLE_API_VERSION == 1
    ble_enable_params_t ble_enable_params = {{0}};
    uint32_t error_code = sd_ble_enable(&ble_enable_params);
#elif (NRF_SD_BLE_API_VERSION >= 2) && (NRF_SD_BLE_API_VERSION < 5)
    ble_enable_params_t ble_enable_params = {{0}};
    uint32_t error_code = sd_ble_enable(&ble_enable_params, &app_ram_base);
#elif NRF_SD_BLE_API_VERSION == 5 || NRF_SD_BLE_API_VERSION == 6
    ble_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 0;
    RETURN_ON_ERROR(sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &cfg, app_ram_base));

    memset(&cfg, 0, sizeof(cfg));
    cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_MIN;
    RETURN_ON_ERROR(sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &cfg, app_ram_base));
    uint32_t error_code = sd_ble_enable(&app_ram_base);
#else
    #error Unsupported NRF_SD_BLE_API_VERSION
#endif
    if (app_ram_base != ram_start)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN,
              "sd_ble_enable: app_ram_base should be adjusted to 0x%0x\n", app_ram_base);
    }
    return error_code;
}

uint32_t mesh_softdevice_init(nrf_clock_lf_cfg_t lfc_cfg)
{
#if !defined(HOST)
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing SoftDevice...\n");

#if NRF_SD_BLE_API_VERSION == 1
    RETURN_ON_ERROR(sd_softdevice_enable(lfc_cfg,  (softdevice_assertion_handler_t) app_error_handler ));
#elif NRF_SD_BLE_API_VERSION >= 2
    RETURN_ON_ERROR(sd_softdevice_enable(&lfc_cfg, app_error_fault_handler));
#endif

    /* Make sure SoftDevice events are handled in the same IRQ priority as the other Mesh event
     * handling (in S132 v5.0.0, S112 v5.0.0, S212 v5.0.0 and S332 v5.0.0 the priority is set to 6
     * by default). */
    RETURN_ON_ERROR(sd_nvic_SetPriority(SD_EVT_IRQn, NRF_MESH_IRQ_PRIORITY_LOWEST));
    RETURN_ON_ERROR(sd_nvic_EnableIRQ(SD_EVT_IRQn));

    RETURN_ON_ERROR(ble_enable());
#endif

    return NRF_SUCCESS;
}
