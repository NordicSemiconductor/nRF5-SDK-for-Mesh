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

#include "hal.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_mesh_assert.h"
#if defined(S130) || defined(S132) || defined(S140) || defined(S112) || defined(S113)
#include "nrf_nvic.h"
#elif defined(S110)
#include "nrf_soc.h"
#endif

/*****************************************************************************
* Local defines
*****************************************************************************/
/** Mask of all potential reset reasons in the NRF_POWER->RESETREAS hardware
 * register. */
#define RESET_REASON_MASK   (0xFFFFFFFF)

#define IRQn_NONE       ((IRQn_Type) -16)
/*****************************************************************************
* Interface functions
*****************************************************************************/
void hal_device_reset(uint8_t gpregret_value)
{
#if defined(SOFTDEVICE_PRESENT)
    (void) sd_power_reset_reason_clr(RESET_REASON_MASK); /* avoid wrongful state-readout on reboot */
#if defined(S130) || defined(S110)
    (void) sd_power_gpregret_clr(NRF_MESH_GPREGRET_USED_BIT_MASK);
    (void) sd_power_gpregret_set(gpregret_value);
#elif defined(S132)
    (void) sd_power_gpregret_clr(0, NRF_MESH_GPREGRET_USED_BIT_MASK);
    (void) sd_power_gpregret_set(0, gpregret_value);
#endif
    (void) sd_nvic_SystemReset();
#else
    NRF_POWER->RESETREAS = RESET_REASON_MASK; /* avoid wrongful state-readout on reboot */
    NRF_POWER->GPREGRET &= ~(NRF_MESH_GPREGRET_USED_BIT_MASK);
    NRF_POWER->GPREGRET |= gpregret_value;
    NVIC_SystemReset();
#endif
    NRF_MESH_ASSERT(false);
}

uint32_t hal_lfclk_ppm_get(uint32_t lfclksrc)
{
    switch (lfclksrc)
    {

#if (NRF_SD_BLE_API_VERSION >= 5)
        case NRF_CLOCK_LF_ACCURACY_100_PPM:
            return 100;
        case NRF_CLOCK_LF_ACCURACY_150_PPM:
            return 150;
        case NRF_CLOCK_LF_ACCURACY_20_PPM:
            return 20;
        case NRF_CLOCK_LF_ACCURACY_250_PPM:
            return 250;
        case NRF_CLOCK_LF_ACCURACY_30_PPM:
            return 30;
        case NRF_CLOCK_LF_ACCURACY_500_PPM:
            return 500;
        case NRF_CLOCK_LF_ACCURACY_50_PPM:
            return 50;
        case NRF_CLOCK_LF_ACCURACY_75_PPM:
            return 75;

#elif (NRF_SD_BLE_API_VERSION >= 2)
        case NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM:
            return 100;
        case NRF_CLOCK_LF_XTAL_ACCURACY_150_PPM:
            return 150;
        case NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM:
            return 20;
        case NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM:
            return 250;
        case NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM:
            return 30;
        case NRF_CLOCK_LF_XTAL_ACCURACY_500_PPM:
            return 500;
        case NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM:
            return 50;
        case NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM:
            return 75;

#elif defined(S110)
        case NRF_CLOCK_LFCLKSRC_XTAL_100_PPM:
            return 100;
        case NRF_CLOCK_LFCLKSRC_XTAL_150_PPM:
            return 150;
        case NRF_CLOCK_LFCLKSRC_XTAL_20_PPM:
            return 20;
        case NRF_CLOCK_LFCLKSRC_XTAL_250_PPM:
            return 250;
        case NRF_CLOCK_LFCLKSRC_XTAL_30_PPM:
            return 30;
        case NRF_CLOCK_LFCLKSRC_XTAL_500_PPM:
            return 500;
        case NRF_CLOCK_LFCLKSRC_XTAL_50_PPM:
            return 50;
        case NRF_CLOCK_LFCLKSRC_XTAL_75_PPM:
            return 75;
#else
#error "Undefined SoftDevice version"
#endif

        default: /* all RC-sources are 250 */
            return 250;
    }
}

IRQn_Type hal_irq_active_get(void)
{
#if defined(HOST)
    return Reset_IRQn; /* Fallback for other platforms. */
#else
    return (IRQn_Type) (((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) >> SCB_ICSR_VECTACTIVE_Pos) - 16);
#endif
}

bool hal_irq_is_enabled(IRQn_Type irq)
{
#if defined(HOST)
    return true; /* Fallback for other platforms. */
#else
    return 0 != (NVIC->ISER[irq / 32] & (1UL << (irq % 32)));
#endif
}
