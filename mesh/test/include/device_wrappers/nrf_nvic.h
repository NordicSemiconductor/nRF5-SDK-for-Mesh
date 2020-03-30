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
* This file wraps the nrf_nvic.h header from the Nordic Semiconductor SoftDevice SDK. It configures
* the functions making them possible to compile in unit tests.
*****************************************************************************************************/

#if !defined(HOST)
#error "Included host side implementation of header file in target build! Remove the path of this file from your include paths."
#endif

/* Undefine storage-class specifier to make functions' prototypes have external linkage */
#undef __STATIC_INLINE
#define __STATIC_INLINE

/* Prevent inlining sd_nvic_* functions when compiling for host */
#ifndef SUPPRESS_INLINE_IMPLEMENTATION
#define SUPPRESS_INLINE_IMPLEMENTATION

/* Remember that SUPPRESS_INLINE_IMPLEMENTATION was changed */
#define NVIC_INLINE_SUPPRESSED
#endif

#include_next "nrf_nvic.h"

/* Undefine back SUPPRESS_INLINE_IMPLEMENTATION macro */
#ifdef NVIC_INLINE_SUPPRESSED
#undef SUPPRESS_INLINE_IMPLEMENTATION
#endif
