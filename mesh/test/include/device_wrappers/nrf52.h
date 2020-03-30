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

/****************************************************************************************************
* HOST-SIDE REPLACEMENT HEADER FOR UNIT TESTS. SHOULD NOT BE INCLUDED IN TARGET CODE.
*
* This file wraps the nrf52.h header from the Nordic Semiconductor MDK. It redefines the hardware
* peripherals to point to extern variables, allowing us to mock hardware behavior during unit tests.
*****************************************************************************************************/

#if !defined(HOST)
#error "Included host side implementation of header file in target build! Remove the path of this file from your include paths."
#endif

#include_next "nrf52.h"

#undef NRF_FICR
#undef NRF_UICR
#undef NRF_BPROT
#undef NRF_POWER
#undef NRF_CLOCK
#undef NRF_RADIO
#undef NRF_UARTE0
#undef NRF_UART0
#undef NRF_SPIM0
#undef NRF_SPIS0
#undef NRF_TWIM0
#undef NRF_TWIS0
#undef NRF_SPI0
#undef NRF_TWI0
#undef NRF_SPIM1
#undef NRF_SPIS1
#undef NRF_TWIM1
#undef NRF_TWIS1
#undef NRF_SPI1
#undef NRF_TWI1
#undef NRF_NFCT
#undef NRF_GPIOTE
#undef NRF_SAADC
#undef NRF_TIMER0
#undef NRF_TIMER1
#undef NRF_TIMER2
#undef NRF_RTC0
#undef NRF_TEMP
#undef NRF_RNG
#undef NRF_ECB
#undef NRF_CCM
#undef NRF_AAR
#undef NRF_WDT
#undef NRF_RTC1
#undef NRF_QDEC
#undef NRF_COMP
#undef NRF_LPCOMP
#undef NRF_SWI0
#undef NRF_EGU0
#undef NRF_SWI1
#undef NRF_EGU1
#undef NRF_SWI2
#undef NRF_EGU2
#undef NRF_SWI3
#undef NRF_EGU3
#undef NRF_SWI4
#undef NRF_EGU4
#undef NRF_SWI5
#undef NRF_EGU5
#undef NRF_TIMER3
#undef NRF_TIMER4
#undef NRF_PWM0
#undef NRF_PDM
#undef NRF_NVMC
#undef NRF_PPI
#undef NRF_MWU
#undef NRF_PWM1
#undef NRF_PWM2
#undef NRF_SPIM2
#undef NRF_SPIS2
#undef NRF_SPI2
#undef NRF_RTC2
#undef NRF_I2S
#undef NRF_FPU
#undef NRF_P0

extern NRF_FICR_Type   * NRF_FICR;
extern NRF_UICR_Type   * NRF_UICR;
extern NRF_BPROT_Type  * NRF_BPROT;
extern NRF_POWER_Type  * NRF_POWER;
extern NRF_CLOCK_Type  * NRF_CLOCK;
extern NRF_RADIO_Type  * NRF_RADIO;
extern NRF_UARTE_Type  * NRF_UARTE0;
extern NRF_UART_Type   * NRF_UART0;
extern NRF_SPIM_Type   * NRF_SPIM0;
extern NRF_SPIS_Type   * NRF_SPIS0;
extern NRF_TWIM_Type   * NRF_TWIM0;
extern NRF_TWIS_Type   * NRF_TWIS0;
extern NRF_SPI_Type    * NRF_SPI0;
extern NRF_TWI_Type    * NRF_TWI0;
extern NRF_SPIM_Type   * NRF_SPIM1;
extern NRF_SPIS_Type   * NRF_SPIS1;
extern NRF_TWIM_Type   * NRF_TWIM1;
extern NRF_TWIS_Type   * NRF_TWIS1;
extern NRF_SPI_Type    * NRF_SPI1;
extern NRF_TWI_Type    * NRF_TWI1;
extern NRF_NFCT_Type   * NRF_NFCT;
extern NRF_GPIOTE_Type * NRF_GPIOTE;
extern NRF_SAADC_Type  * NRF_SAADC;
extern NRF_TIMER_Type  * NRF_TIMER0;
extern NRF_TIMER_Type  * NRF_TIMER1;
extern NRF_TIMER_Type  * NRF_TIMER2;
extern NRF_RTC_Type    * NRF_RTC0;
extern NRF_TEMP_Type   * NRF_TEMP;
extern NRF_RNG_Type    * NRF_RNG;
extern NRF_ECB_Type    * NRF_ECB;
extern NRF_CCM_Type    * NRF_CCM;
extern NRF_AAR_Type    * NRF_AAR;
extern NRF_WDT_Type    * NRF_WDT;
extern NRF_RTC_Type    * NRF_RTC1;
extern NRF_QDEC_Type   * NRF_QDEC;
extern NRF_COMP_Type   * NRF_COMP;
extern NRF_LPCOMP_Type * NRF_LPCOMP;
extern NRF_SWI_Type    * NRF_SWI0;
extern NRF_EGU_Type    * NRF_EGU0;
extern NRF_SWI_Type    * NRF_SWI1;
extern NRF_EGU_Type    * NRF_EGU1;
extern NRF_SWI_Type    * NRF_SWI2;
extern NRF_EGU_Type    * NRF_EGU2;
extern NRF_SWI_Type    * NRF_SWI3;
extern NRF_EGU_Type    * NRF_EGU3;
extern NRF_SWI_Type    * NRF_SWI4;
extern NRF_EGU_Type    * NRF_EGU4;
extern NRF_SWI_Type    * NRF_SWI5;
extern NRF_EGU_Type    * NRF_EGU5;
extern NRF_TIMER_Type  * NRF_TIMER3;
extern NRF_TIMER_Type  * NRF_TIMER4;
extern NRF_PWM_Type    * NRF_PWM0;
extern NRF_PDM_Type    * NRF_PDM;
extern NRF_NVMC_Type   * NRF_NVMC;
extern NRF_PPI_Type    * NRF_PPI;
extern NRF_MWU_Type    * NRF_MWU;
extern NRF_PWM_Type    * NRF_PWM1;
extern NRF_PWM_Type    * NRF_PWM2;
extern NRF_SPIM_Type   * NRF_SPIM2;
extern NRF_SPIS_Type   * NRF_SPIS2;
extern NRF_SPI_Type    * NRF_SPI2;
extern NRF_RTC_Type    * NRF_RTC2;
extern NRF_I2S_Type    * NRF_I2S;
extern NRF_FPU_Type    * NRF_FPU;
extern NRF_GPIO_Type   * NRF_P0;

