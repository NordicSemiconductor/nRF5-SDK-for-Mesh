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

#ifndef _TOOLCHAIN_H__
#define _TOOLCHAIN_H__

#include "nrf.h"

#if defined(_lint)
    #define __packed_gcc
    #define __packed_armcc

    #define _DISABLE_IRQS(_was_masked) _was_masked = 0; __disable_irq()
    #define _ENABLE_IRQS(_was_masked) (void) _was_masked; __enable_irq()

#elif defined(__CC_ARM)

/* ARMCC and GCC have different ordering for packed typedefs, must separate macros */
    #define __packed_gcc
    #define __packed_armcc __packed

    #define _DISABLE_IRQS(_was_masked) _was_masked = __disable_irq()
    #define _ENABLE_IRQS(_was_masked) if (!_was_masked) { __enable_irq(); }

#elif defined(__GNUC__)

    #define __packed_armcc
    #define __packed_gcc __attribute__((packed))

    #define _DISABLE_IRQS(_was_masked) do{ \
        __ASM volatile ("MRS %0, primask" : "=r" (_was_masked) );\
        __ASM volatile ("cpsid i" : : : "memory");\
    } while(0)

    #define _ENABLE_IRQS(_was_masked) if (!_was_masked) { __enable_irq(); }
#elif defined(__IAR_SYSTEMS_ICC__)
  #define __packed_gcc
  #define __packed_armcc __packed
  #define _DISABLE_IRQS(_was_masked) do { _was_masked = __get_PRIMASK(); __disable_irq(); } while (0)
  #define _ENABLE_IRQS(_was_masked) __set_PRIMASK(_was_masked)
  #if defined(__cplusplus) && !defined(__STDC_LIMIT_MACROS)
    #error "Please define __STDC_LIMIT_MACROS in your project options!"
  #endif
#else
    #warning "Unsupported toolchain"
#endif

#endif /* _TOOLCHAIN_H__ */
