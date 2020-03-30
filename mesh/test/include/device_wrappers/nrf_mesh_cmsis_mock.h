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

#ifndef NRF_MESH_CMSIS_MOCK_H
#define NRF_MESH_CMSIS_MOCK_H
#ifdef __CMSIS_GENERIC
#include <stdint.h>
#ifdef __cplusplus
  #define   __I     volatile
#else
  #define   __I     volatile /* Removed const to enable writing to input registers during unit testing */
#endif
#define     __O     volatile
#define     __IO    volatile

/* following defines should be used for structure members */
#define     __IM     volatile /* Removed const to enable writing to input registers during unit testing */
#define     __OM     volatile
#define     __IOM    volatile

/* ##########################   NVIC functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_NVICFunctions NVIC Functions
  \brief    Functions that manage interrupts and exceptions via the NVIC.
  @{
 */

/* Interrupt Priorities are WORD accessible only under ARMv6M                   */
/* The following MACROS handle generation of the register offset and byte masks */
#define _BIT_SHIFT(IRQn)         (  ((((uint32_t)(int32_t)(IRQn))         )      &  0x03UL) * 8UL)
#define _SHP_IDX(IRQn)           ( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >>    2UL)      )
#define _IP_IDX(IRQn)            (   (((uint32_t)(int32_t)(IRQn))                >>    2UL)      )

/** The inline NVIC functions have been converted to a prototype to allow creating mock-ups.*/
void NVIC_EnableIRQ(uint32_t IRQn);
void NVIC_DisableIRQ(uint32_t IRQn);
uint32_t NVIC_GetPendingIRQ(uint32_t IRQn);
void NVIC_SetPendingIRQ(uint32_t IRQn);
void NVIC_ClearPendingIRQ(uint32_t IRQn);
void NVIC_SetPriority(uint32_t IRQn, uint32_t priority);
uint32_t NVIC_GetPriority(uint32_t IRQn);
void NVIC_SystemReset(void);
/*@} end of CMSIS_Core_NVICFunctions */

#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler */
#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler */
#define __STATIC_INLINE  static inline
#define __REV            __builtin_bswap32

#endif /* __CMSIS_GENERIC */
#endif /* NRF_MESH_CMSIS_MOCK_H */
