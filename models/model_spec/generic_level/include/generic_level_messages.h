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

#ifndef GENERIC_LEVEL_MESSAGES_H__
#define GENERIC_LEVEL_MESSAGES_H__

#include <stdint.h>
#include "generic_level_common.h"
/**
 * @internal
 * @defgroup GENERIC_LEVEL_MESSAGES Internal header
 * @ingroup GENERIC_LEVEL_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define GENERIC_LEVEL_SET_MINLEN 3
/** Longest allowed length for the Set message. */
#define GENERIC_LEVEL_SET_MAXLEN 5

/** Shortest allowed length for the Delta Set message. */
#define GENERIC_LEVEL_DELTA_SET_MINLEN 5
/** Longest allowed length for the Delta Set message. */
#define GENERIC_LEVEL_DELTA_SET_MAXLEN 7

/** Shortest allowed length for the Move Set message. */
#define GENERIC_LEVEL_MOVE_SET_MINLEN 3
/** Longest allowed length for the Move Set message. */
#define GENERIC_LEVEL_MOVE_SET_MAXLEN 5

/** Shortest allowed length for the Status message. */
#define GENERIC_LEVEL_STATUS_MINLEN 2
/** Longest allowed length for the Status message. */
#define GENERIC_LEVEL_STATUS_MAXLEN 5



/** Generic On Off model message opcodes. */
typedef enum
{
    GENERIC_LEVEL_OPCODE_GET = 0x8205,
    GENERIC_LEVEL_OPCODE_SET = 0x8206,
    GENERIC_LEVEL_OPCODE_SET_UNACKNOWLEDGED = 0x8207,
    GENERIC_LEVEL_OPCODE_STATUS = 0x8208,
    GENERIC_LEVEL_OPCODE_DELTA_SET = 0x8209,
    GENERIC_LEVEL_OPCODE_DELTA_SET_UNACKNOWLEDGED = 0x820A,
    GENERIC_LEVEL_OPCODE_MOVE_SET = 0x820B,
    GENERIC_LEVEL_OPCODE_MOVE_SET_UNACKNOWLEDGED = 0x820C
} generic_level_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the generic_level Set message. */
typedef struct __attribute((packed))
{
    int16_t level;                                          /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} generic_level_set_msg_pkt_t;

/** Message format for the generic_level Delta Set message. */
typedef struct __attribute((packed))
{
    int32_t delta_level;                                    /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} generic_level_delta_set_msg_pkt_t;

/** Message format for the generic_level Move Set message. */
typedef struct __attribute((packed))
{
    int16_t move_level;                                     /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */
    uint8_t transition_time;                                /**< Encoded transition time value */
    uint8_t delay;                                          /**< Encoded message execution delay in 5 millisecond steps */
} generic_level_move_set_msg_pkt_t;

/** Message format for the generic_level Status message. */
typedef struct __attribute((packed))
{
    int16_t present_level;                                  /**< The present value of the Generic Level state */
    int16_t target_level;                                   /**< The target value of the Generic Level state (optional) */
    uint8_t remaining_time;                                 /**< Encoded remaining time */
} generic_level_status_msg_pkt_t;

/**@} end of GENERIC_LEVEL_MODEL_INTENRAL */
#endif /* GENERIC_LEVEL_MESSAGES_H__ */
