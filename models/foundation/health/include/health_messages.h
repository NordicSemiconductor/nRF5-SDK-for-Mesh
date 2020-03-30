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

#ifndef HEALTH_MESSAGES_H__
#define HEALTH_MESSAGES_H__

#include <stdint.h>

/**
 * @defgroup HEALTH_MESSAGES Health model message definitions
 * @ingroup HEALTH_MODEL
 * Definitions of the various messages supported by the health model.
 * @{
 */

/*lint -align_max(push) -align_max(1) */

/** Health Fault Status message format. */
typedef struct __attribute((packed))
{
    uint8_t  test_id;       /**< Test ID for the most recently run test */
    uint16_t company_id;    /**< Company identifier for the health model. */
    uint8_t  fault_array[]; /**< Fault array. */
} health_msg_fault_status_t;

/** Health Period Status message format. */
typedef struct __attribute((packed))
{
    uint8_t fast_period_divisor; /**< Current fast period divisor. Permitted values are 0 - 15. */
} health_msg_period_status_t;

/** Health Attention Status message format. */
typedef struct __attribute((packed))
{
    uint8_t attention; /**< Current attention timer value, in seconds. */
} health_msg_attention_status_t;

/** Health Fault Get message format. */
typedef struct __attribute((packed))
{
    uint16_t company_id; /**< Company ID. */
} health_msg_fault_get_t;

/** Health Fault Clear message format. */
typedef struct __attribute((packed))
{
    uint16_t company_id; /**< Company ID. */
} health_msg_fault_clear_t;

/** Health Fault Test message format. */
typedef struct __attribute((packed))
{
    uint8_t  test_id;    /**< Test ID. */
    uint16_t company_id; /**< Company ID. */
} health_msg_fault_test_t;

/** Health Period Set message format. */
typedef struct __attribute((packed))
{
    uint8_t fast_period_divisor; /**< Fast period divisor. Permitted values are 0 - 15. */
} health_msg_period_set_t;

/** Health Attention Set message format. */
typedef struct __attribute((packed))
{
    uint8_t attention; /**< Attention timer value, in seconds. */
} health_msg_attention_set_t;

/*lint -align_max(pop) */

/** @} */

#endif

