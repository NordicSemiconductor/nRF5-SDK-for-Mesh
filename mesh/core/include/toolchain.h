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
#ifndef MESH_TOOLCHAIN_H__
#define MESH_TOOLCHAIN_H__

#include "nrf.h"

void toolchain_init_irqs(void);

#if defined(_lint)
    #define _DISABLE_IRQS(_was_masked) _was_masked = 0; __disable_irq()
    #define _ENABLE_IRQS(_was_masked) (void) _was_masked; __enable_irq()
    #define _DEPRECATED
    #define _UNUSED
#elif defined(UNIT_TEST)
    #define _DISABLE_IRQS(_was_masked) (void)_was_masked /* avoid "not used" warning */
    #define _ENABLE_IRQS(_was_masked)
    #define _GET_LR(lr) lr = (uint32_t) __builtin_return_address(0);
    #define _DEPRECATED
    #define _UNUSED __attribute__((unused))
#elif defined(MTT_TEST)
    #include <pthread.h>
    extern pthread_mutex_t irq_mutex;
    #define _GET_LR(lr) lr = (uint32_t) __builtin_return_address(0);
    #define _DISABLE_IRQS(_was_masked) pthread_mutex_lock(&irq_mutex); (void) _was_masked;
    #define _ENABLE_IRQS(_was_masked) pthread_mutex_unlock(&irq_mutex);
    /** Mark a function, variable or type as deprecated. */
    #define _DEPRECATED
#elif defined(__CC_ARM)
    /** Disable all interrupts and get whether it was masked. */
    #define _DISABLE_IRQS(_was_masked) _was_masked = __disable_irq()

    /** Enable all interrupts if they weren't masked. */
    #define _ENABLE_IRQS(_was_masked) do{ if (!(_was_masked)) { __enable_irq(); } } while(0)

    /** Get the value of the link register. */
    #define _GET_LR(lr) do { lr = __return_address(); } while (0)

    /** Mark a function, variable or type as deprecated. */
    #define _DEPRECATED __attribute__((deprecated))
    /** Mark a function, variable, or type as potentially unused. */
    #define _UNUSED __attribute__((unused))
#elif defined(__GNUC__)
/** Disable all interrupts and get whether it was masked. */
    #define _DISABLE_IRQS(_was_masked) do{ \
            __ASM volatile ("MRS %0, primask" : "=r" (_was_masked) );\
            __ASM volatile ("cpsid i" : : : "memory");\
        } while(0)

    /** Enable all interrupts if they weren't masked. */
    #define _ENABLE_IRQS(_was_masked) do{ if (!(_was_masked)) { __enable_irq(); } } while(0)

    /** Get the value of the link register. */
    #define _GET_LR(lr) do { lr = (uint32_t) __builtin_return_address(0); } while (0)

    /** Mark a function, variable or type as deprecated. */
    #define _DEPRECATED __attribute__((deprecated))
    /** Mark a function, variable, or type as potentially unused. */
    #define _UNUSED __attribute__((unused))
#endif

/*Segger embedded studio originally has offsetof macro which cannot be used in macros (like STATIC_ASSERT).
  This redefinition is to allow using that. */
#if defined(__SES_ARM) && defined(__GNUC__)
#undef offsetof
#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#endif

#endif /* TOOLCHAIN_H__ */
