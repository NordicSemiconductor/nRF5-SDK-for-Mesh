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

#ifndef HAL_H__
#define HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

/**
 * @defgroup HAL Hardware Abstraction Layer
 * @ingroup MESH_CORE
 * @{
 */

/** Retention register bits that are relevant for the mesh stack. */
#define NRF_MESH_GPREGRET_USED_BIT_MASK  (0x000000FF)

#if defined(MBR_PRESENT) || defined(SOFTDEVICE_PRESENT)
#include "nrf_mbr.h"
/**< The currently configured start address of the bootloader. If 0xFFFFFFFF, no bootloader start
 * address is configured. */
#define BOOTLOADERADDR()      ((*(uint32_t *)MBR_BOOTLOADER_ADDR) == 0xFFFFFFFF ? \
                                *MBR_UICR_BOOTLOADER_ADDR : *(uint32_t *)MBR_BOOTLOADER_ADDR)
#else
/**< Check UICR, just in case. */
#define BOOTLOADERADDR()      (NRF_UICR->NRFFW[0])
#endif

#if NRF51
/** nRF51 flash page size. */
#define PAGE_SIZE        (0x400)
/** nRF51 code RAM start. */
#define CODE_RAM_START   (0x20000000)
/** nRF51 data RAM start. */
#define DATA_RAM_START   (0x20000000)
/** First address outside the data RAM */
#define DEVICE_DATA_RAM_END_GET() (DATA_RAM_START + (NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK))
/** First address outside the code RAM */
#define DEVICE_CODE_RAM_END_GET() (CODE_RAM_START + (NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK))
/** First address outside the device flash */
#define DEVICE_FLASH_END_GET()    (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (22300)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (48)
#elif NRF52_SERIES
/** nRF52 flash page size. */
#define PAGE_SIZE        (0x1000)
/** nRF52 code RAM start address. */
#define CODE_RAM_START   (0x800000)
/** nRF52 data RAM start address. */
#define DATA_RAM_START   (0x20000000)
/** First address outside the data RAM */
#define DEVICE_DATA_RAM_END_GET() (DATA_RAM_START + (1024 * NRF_FICR->INFO.RAM))
/** First address outside the code RAM */
#define DEVICE_CODE_RAM_END_GET() (CODE_RAM_START + (1024 * NRF_FICR->INFO.RAM))
/** First address outside the device flash */
#define DEVICE_FLASH_END_GET()    (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE)

#if defined(NRF52810)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (85000)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (41)
#elif defined(NRF52820)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (87500)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (43)
#elif defined(NRF52832)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (89700)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (338)
#elif defined(NRF52833)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (87500)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (43)
#elif defined(NRF52840)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (85000)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (41)
#endif
#else
#if defined(HOST)
#define PAGE_SIZE      (0x400)
#define CODE_RAM_START (0)
#define DATA_RAM_START (0)
#define DEVICE_DATA_RAM_END_GET()   (0xFFFFFFFF)
#define DEVICE_CODE_RAM_END_GET()   (0xFFFFFFFF)
#define DEVICE_FLASH_END_GET()      (NRF_MESH_ASSERT(false))
#define FLASH_TIME_TO_ERASE_PAGE_US         (20000)
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (50)

typedef enum
{
    Reset_IRQn
} IRQn_Type;

#else
#error "Unsupported hardware platform"
#endif  /* HOST */
#endif  /* NRF51 || NRF52_SERIES */

/* Macros for converting between time units and RTC ticks. */
#define HAL_RTC_TICKS_TO_US(ticks)      (((uint64_t)(ticks) * 1000000ULL) >> 15ULL)
#define HAL_RTC_TICKS_TO_MS(ticks)      (((uint64_t)(ticks) * 1000ULL) >> 15ULL)
#define HAL_RTC_TICKS_TO_SECS(ticks)    (((uint64_t)(ticks)) >> 15ULL)
#define HAL_US_TO_RTC_TICKS(time_us)    (((uint64_t) (time_us) << 15ULL) / 1000000ULL)
#define HAL_MS_TO_RTC_TICKS(time_ms)    (((uint64_t) (time_ms) << 15ULL) / 1000ULL)
#define HAL_SECS_TO_RTC_TICKS(time_s)   (((uint64_t) (time_s) << 15ULL))

/** IRQ value for no IRQ */
#define HAL_IRQn_NONE       ((IRQn_Type) - 16)

/**
 * Clear the reset reason register, set the @ref NRF_MESH_GPREGRET_USED_BIT_MASK bits in the
 * retention register to the value specified by `gpregret_value`, and reset the
 * device. All volatile memory will be lost. This function will never return.
 *
 * @note For SofDevices S110 or S130, this API will set the bits of the GPREGRET register.
 * For the S132 SoftDevice, this API will set the bits of the GPREGRET0 register.
 *
 * @param[in] gpregret_value Value to set the retention register to.
 */
void hal_device_reset(uint8_t gpregret_value);

/**
 * Get the drift in PPM for the given XTAL clock accuracy type.
 *
 * @param[in] lfclksrc Clock accuracy, from @c NRF_CLOCK_LF_XTAL_ACCURACY in nrf_sdm.h in the SDK.
 *
 * @returns The drift of the given XTAL clock accuracy.
 */
uint32_t hal_lfclk_ppm_get(uint32_t lfclksrc);


/**
 * Gets the currently active IRQ type, or @ref HAL_IRQn_NONE if no IRQ is active.
 *
 * @returns Currently active IRQ type.
 */
IRQn_Type hal_irq_active_get(void);

/**
 * Checks whether a specific IRQ is enabled.
 *
 * @param[in] irq IRQ type to check
 *
 * @returns Whether the given IRQ is enabled.
 */
bool hal_irq_is_enabled(IRQn_Type irq);

/** @} */

#endif /* HAL_H__ */

