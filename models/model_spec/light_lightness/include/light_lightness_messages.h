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

#ifndef LIGHT_LIGHTNESS_MESSAGES_H__
#define LIGHT_LIGHTNESS_MESSAGES_H__

#include <stdint.h>
#include "light_lightness_common.h"
/**
 * @internal
 * @defgroup LIGHT_LIGHTNESS_MESSAGES Internal header
 * @ingroup LIGHT_LIGHTNESS_MODELS
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define LIGHT_LIGHTNESS_SET_MINLEN 3
/** Longest allowed length for the Set message. */
#define LIGHT_LIGHTNESS_SET_MAXLEN 5

/** Shortest allowed length for the Linear Set message. */
#define LIGHT_LIGHTNESS_LINEAR_SET_MINLEN 3
/** Longest allowed length for the Linear Set message. */
#define LIGHT_LIGHTNESS_LINEAR_SET_MAXLEN 5

/** Length for the Default Set message */
#define LIGHT_LIGHTNESS_DEFAULT_SET_LEN 2

/** Length for the Range Set message */
#define LIGHT_LIGHTNESS_RANGE_SET_LEN 4

/** Shortest allowed length for the Status message. */
#define LIGHT_LIGHTNESS_STATUS_MINLEN 2
/** Longest allowed length for the Status message. */
#define LIGHT_LIGHTNESS_STATUS_MAXLEN 5

/** Shortest allowed length for the Linear Status message. */
#define LIGHT_LIGHTNESS_LINEAR_STATUS_MINLEN 2
/** Longest allowed length for the Linear Status message. */
#define LIGHT_LIGHTNESS_LINEAR_STATUS_MAXLEN 5

/** Length of the last status message. */
#define LIGHT_LIGHTNESS_LAST_STATUS_LEN 2

/** Length of the default status message. */
#define LIGHT_LIGHTNESS_DEFAULT_STATUS_LEN 2

/** Length of the range status message. */
#define LIGHT_LIGHTNESS_RANGE_STATUS_LEN 5

/** Light Lightness model message opcodes. */
typedef enum
{
    LIGHT_LIGHTNESS_OPCODE_GET = 0x824B,
    LIGHT_LIGHTNESS_OPCODE_SET = 0x824C,
    LIGHT_LIGHTNESS_OPCODE_SET_UNACKNOWLEDGED = 0x824D,
    LIGHT_LIGHTNESS_OPCODE_STATUS = 0x824E,
    LIGHT_LIGHTNESS_OPCODE_LINEAR_GET = 0x824F,
    LIGHT_LIGHTNESS_OPCODE_LINEAR_SET = 0x8250,
    LIGHT_LIGHTNESS_OPCODE_LINEAR_SET_UNACKNOWLEDGED = 0x8251,
    LIGHT_LIGHTNESS_OPCODE_LINEAR_STATUS = 0x8252,
    LIGHT_LIGHTNESS_OPCODE_LAST_GET = 0x8253,
    LIGHT_LIGHTNESS_OPCODE_LAST_STATUS = 0x8254,
    LIGHT_LIGHTNESS_OPCODE_DEFAULT_GET = 0x8255,
    LIGHT_LIGHTNESS_OPCODE_DEFAULT_STATUS = 0x8256,
    LIGHT_LIGHTNESS_OPCODE_RANGE_GET = 0x8257,
    LIGHT_LIGHTNESS_OPCODE_RANGE_STATUS = 0x8258,
    LIGHT_LIGHTNESS_OPCODE_DEFAULT_SET = 0x8259,
    LIGHT_LIGHTNESS_OPCODE_DEFAULT_SET_UNACKNOWLEDGED = 0x825A,
    LIGHT_LIGHTNESS_OPCODE_RANGE_SET = 0x825B,
    LIGHT_LIGHTNESS_OPCODE_RANGE_SET_UNACKNOWLEDGED = 0x825C,
} light_lightness_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the Light Lightness Set message (actual/perceptually uniform state). */
typedef struct __attribute((packed))
{
    uint16_t lightness;                                     /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} light_lightness_set_msg_pkt_t;

/** Message format for the Light Lightness Linear Set message. */
typedef struct __attribute((packed))
{
    uint16_t lightness;                                     /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} light_lightness_linear_set_msg_pkt_t;

/** Message format for the Light Lightness Default Set message. */
typedef struct __attribute((packed))
{
    uint16_t lightness;                                     /**< State to set */
} light_lightness_default_set_msg_pkt_t;

/** Message format for the Light Lightness Range Set message. */
typedef struct __attribute((packed))
{
    uint16_t range_min;                                     /**< The value of the lightness range min state */
    uint16_t range_max;                                     /**< The value of the lightness range max state */
} light_lightness_range_set_msg_pkt_t;

/** Message format for the Light Lightness Status message (actual/perceptually uniform state). */
typedef struct __attribute((packed))
{
    uint16_t present_lightness;                             /**< The present value of the lightness state */
    uint16_t target_lightness;                              /**< The target value of the lightness state (optional) */
    uint8_t remaining_time;                                 /**< Encoded remaining time */
} light_lightness_status_msg_pkt_t;

/** Message format for the Light Lightness Linear Status message. */
typedef struct __attribute((packed))
{
    uint16_t present_lightness;                             /**< The present value of the linear lightness state */
    uint16_t target_lightness;                              /**< The target value of the linear lightness state (optional) */
    uint8_t remaining_time;                                 /**< Encoded remaining time */
} light_lightness_linear_status_msg_pkt_t;

/** Message format for the Light Lightness Last Status message (actual/perceptually uniform state). */
typedef struct __attribute((packed))
{
    uint16_t lightness;                                     /**< The value of the lightness last state */
} light_lightness_last_status_msg_pkt_t;

/** Message format for the Light Lightness Default Status message (actual/perceptually uniform state). */
typedef struct __attribute((packed))
{
    uint16_t lightness;                                     /**< The value of the lightness last state */
} light_lightness_default_status_msg_pkt_t;

/** Message format for the Light Lightness Range Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;
    uint16_t range_min;                                     /**< The value of the lightness range min state */
    uint16_t range_max;                                     /**< The value of the lightness range max state */
} light_lightness_range_status_msg_pkt_t;

/**@} end of LIGHT_LIGHTNESS_MESSAGES */
#endif /* LIGHT_LIGHTNESS_MESSAGES_H__ */
