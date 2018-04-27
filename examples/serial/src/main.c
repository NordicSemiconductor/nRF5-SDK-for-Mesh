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

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"

#include "log.h"
#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "utils.h"

#include "access.h"
#include "device_state_manager.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_sdk.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

/********** Application Functionality **********/

int main(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif


    __LOG_INIT(LOG_MSK_DEFAULT | LOG_SRC_ACCESS | LOG_SRC_SERIAL | LOG_SRC_APP, LOG_LEVEL_INFO, log_callback_rtt);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Bluetooth Mesh Serial Interface Application -----\n");

    NRF_GPIO->DIRSET = 0xFFFFFFFF;
    NRF_GPIO->OUT    = LEDS_MASK;

    /* Flash leds at boot */
    for (int i = 0; i < 10; ++i)
    {
        NRF_GPIO->OUT ^= BSP_LED_0_MASK;
        nrf_delay_ms(100);
    }

    mesh_core_setup();

    /* Initialize dsm and access */
    dsm_init();
    access_init();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling ECDH offloading...\n");
    nrf_mesh_opt_t value = {.len = 4, .opt.val = 1 };
    ERROR_CHECK(nrf_mesh_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &value));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling serial interface...\n");
    ERROR_CHECK(nrf_mesh_serial_init(NULL));
    ERROR_CHECK(nrf_mesh_serial_enable());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");
    NRF_GPIO->OUTCLR = BSP_LED_0_MASK;

    while (true)
    {
        (void)sd_app_evt_wait();
    }
}
