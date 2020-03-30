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

#include "nrf.h"
#include "app_error.h"
#include "log.h"
#include "mesh_app_utils.h"
#include "boards.h"


/* Copied from nrf_sdm.h (nRF5 SDK 14.2): */
#define NRF_FAULT_ID_SD_RANGE_START     0x00000000                          /**< SoftDevice ID range start. */
#define NRF_FAULT_ID_APP_RANGE_START    0x00001000                          /**< Application ID range start. */
#define NRF_FAULT_ID_SD_ASSERT          (NRF_FAULT_ID_SD_RANGE_START  + 1)  /**< SoftDevice assertion. The info parameter is reserved for future used. */
#define NRF_FAULT_ID_APP_MEMACC         (NRF_FAULT_ID_APP_RANGE_START + 1)  /**< Application invalid memory access. The info parameter will contain 0x00000000,
                                                                                 in case of SoftDevice RAM access violation. In case of SoftDevice peripheral
                                                                                 register violation the info parameter will contain the sub-region number of
                                                                                 PREGION[0], on whose address range the disallowed write access caused the
                                                                                 memory access fault. */

#if defined(__GNUC__)
#define REG_MAGIC_WORD_WRITE() do {__ASM volatile ("MOV R12, #0xDEAD\n"); } while (0)
#elif defined(__CC_ARM)
/* Not supported on ARMCC:
 * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.kui0097a/armcc_cihccdja.htm
 */
#endif


static inline void sleep_forever(void)
{
    /* Turn on all LEDs on board */
    NRF_GPIO->DIRSET = LEDS_MASK;
    NRF_GPIO->OUTCLR = LEDS_MASK;

    /*lint -save -e438 -e550 Unsused variable irqs_masked */
    uint32_t irqs_masked __attribute__((unused));
    _DISABLE_IRQS(irqs_masked);
#if defined(__GNUC__)
    REG_MAGIC_WORD_WRITE();
#endif

    /* NOTE: Using a volatile variable to avoid armcc removing infinite loop with no side-effect */
    static volatile bool wait = true;
    while (wait)
    {
        __WFI();
    }
    /*lint -restore */
}
#if NRF_SD_BLE_API_VERSION == 1
__WEAK void app_error_fault_handler(uint32_t pc, uint16_t id, const uint8_t * p_file_name)
{
    uint32_t info = (p_file_name[0]) + (p_file_name[1] >> 8) + (p_file_name[2] >> 16) + (p_file_name[3] >> 24);
#elif NRF_SD_BLE_API_VERSION >= 2
__WEAK void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
#endif
    switch (id)
    {
        case NRF_FAULT_ID_SD_ASSERT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Softdevice assert: %u:%u\n", pc, info);
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Application memory access: %u:%u\n", pc, info);
            break;
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            UNUSED_VARIABLE(p_info);
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Mesh assert at 0x%08x (%s:%u)\n",
                                                pc,
                                                p_info->p_file_name,
                                                p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            UNUSED_VARIABLE(p_info);
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Mesh error %u at 0x%08x (%s:%u)\n",
                                                p_info->err_code,
                                                pc,
                                                p_info->p_file_name,
                                                p_info->line_num);
            break;
        }
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "UNKNOWN FAULT at 0x%08X\n", pc);
            break;
    }
    sleep_forever();
}
