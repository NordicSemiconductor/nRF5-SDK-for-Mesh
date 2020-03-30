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

#ifndef LIGHT_CTL_MESSAGES_H__
#define LIGHT_CTL_MESSAGES_H__

#include <stdint.h>
#include "light_ctl_common.h"
/**
 * @internal
 * @defgroup LIGHT_CTL_MESSAGES Internal header
 * @ingroup LIGHT_CTL_MODELS
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Light CTL Set message. */
#define LIGHT_CTL_SET_MINLEN 7
/** Longest allowed length for the Light CTL Set message. */
#define LIGHT_CTL_SET_MAXLEN 9

/** Shortest allowed length for the Light CTL Temperature Set message. */
#define LIGHT_CTL_TEMPERATURE_SET_MINLEN 5
/** Longest allowed length for the Light CTL Temperature Set message. */
#define LIGHT_CTL_TEMPERATURE_SET_MAXLEN 7

/** Length for the Light CTL Temperature Range Set message. */
#define LIGHT_CTL_TEMPERATURE_RANGE_SET_LEN 4

/** Length for the Light CTL Default Set message. */
#define LIGHT_CTL_DEFAULT_SET_LEN 6

/** Shortest allowed length for the Status message. */
#define LIGHT_CTL_STATUS_MINLEN 4
/** Longest allowed length for the Status message. */
#define LIGHT_CTL_STATUS_MAXLEN 9

/** Shortest allowed length for the Light CTL Temperature Status message. */
#define LIGHT_CTL_TEMPERATURE_STATUS_MINLEN 4
/** Longest allowed length for the Light CTL Temperature Status message. */
#define LIGHT_CTL_TEMPERATURE_STATUS_MAXLEN 9

/** Length for the Light CTL Temperature Range Status message. */
#define LIGHT_CTL_TEMPERATURE_RANGE_STATUS_LEN 5

/** Length for the Light CTL Default Status message. */
#define LIGHT_CTL_DEFAULT_STATUS_LEN 6

/** Light CTL model message opcodes. */
typedef enum
{
    LIGHT_CTL_OPCODE_GET = 0x825D,
    LIGHT_CTL_OPCODE_SET = 0x825E,
    LIGHT_CTL_OPCODE_SET_UNACKNOWLEDGED = 0x825F,
    LIGHT_CTL_OPCODE_STATUS = 0x8260,
    LIGHT_CTL_OPCODE_TEMPERATURE_GET = 0x8261,
    LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_GET = 0x8262,
    LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_STATUS = 0x8263,
    LIGHT_CTL_OPCODE_TEMPERATURE_SET = 0x8264,
    LIGHT_CTL_OPCODE_TEMPERATURE_SET_UNACKNOWLEDGED = 0x8265,
    LIGHT_CTL_OPCODE_TEMPERATURE_STATUS = 0x8266,
    LIGHT_CTL_OPCODE_DEFAULT_GET = 0x8267,
    LIGHT_CTL_OPCODE_DEFAULT_STATUS = 0x8268,
    LIGHT_CTL_OPCODE_DEFAULT_SET = 0x8269,
    LIGHT_CTL_OPCODE_DEFAULT_SET_UNACKNOWLEDGED = 0x826A,
    LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET = 0x826B,
    LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED = 0x826C,
} light_ctl_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the Light CTL Set message. */
typedef struct __attribute((packed))
{
    uint16_t lightness;                    /**< Lightness state to set */
    uint16_t temperature;                  /**< Temperature state to set */
    uint16_t delta_uv;                     /**< Delta UV state to set */
    uint8_t tid;                           /**< Transaction number for application */
    uint8_t transition_time;               /**< Encoded transition time value */
    uint8_t delay;                         /**< Encoded message execution delay in 5 millisecond steps */
} light_ctl_set_msg_pkt_t;

/** Message format for the Light CTL Temperature Set message. */
typedef struct __attribute((packed))
{
    uint16_t temperature;                  /**< Temperature state to set */
    uint16_t delta_uv;                     /**< Delta UV state to set */
    uint8_t tid;                           /**< Transaction number for application */
    uint8_t transition_time;               /**< Encoded transition time value */
    uint8_t delay;                         /**< Encoded message execution delay in 5 millisecond steps */
} light_ctl_temperature_set_msg_pkt_t;

/** Message format for the Light CTL Temperature Range Set message. */
typedef struct __attribute((packed))
{
    uint16_t range_min;                    /**< Temperature Range min state to set */
    uint16_t range_max;                    /**< Temperature Range max state to set */
} light_ctl_temperature_range_set_msg_pkt_t;

/** Message format for the Light CTL Default Set message. */
typedef struct __attribute((packed))
{
    uint16_t lightness;                    /**< Lightness state to set */
    uint16_t temperature;                  /**< Temperature state to set */
    uint16_t delta_uv;                     /**< Delta UV state to set */
} light_ctl_default_set_msg_pkt_t;

/** Message format for the Light CTL Status message. */
typedef struct __attribute((packed))
{
    uint16_t present_lightness;            /**< The present value of the Lightness state */
    uint16_t present_temperature;          /**< The present value of the Temperature state */
    uint16_t target_lightness;             /**< The target value of the Lightness state (optional) */
    uint16_t target_temperature;           /**< The target value of the Temperature state (optional) */
    uint8_t remaining_time;                /**< Encoded remaining time */
} light_ctl_status_msg_pkt_t;

/** Message format for the Light CTL Temperature Status message. */
typedef struct __attribute((packed))
{
    uint16_t present_temperature;          /**< The present value of the Temperature state */
    uint16_t present_delta_uv;             /**< The present value of the Delta UV state */
    uint16_t target_temperature;           /**< The target value of the Temperature state (optional) */
    uint16_t target_delta_uv;              /**< The target value of the Delta UV state (optional) */
    uint8_t remaining_time;                /**< Encoded remaining time */
} light_ctl_temperature_status_msg_pkt_t;

/** Message format for the Light CTL Temperature Range Status message. */
typedef struct __attribute((packed))
{
    uint8_t status_code;                   /**< Status code for the requesting message */
    uint16_t range_min;                    /**< Current value of Temperature Range min state */
    uint16_t range_max;                    /**< Current value of Temperature Range max state */
} light_ctl_temperature_range_status_msg_pkt_t;

/** Message format for the Light CTL Default Status message. */
typedef struct __attribute((packed))
{
    uint16_t lightness;                    /**< The present value of the Lightness default state */
    uint16_t temperature;                  /**< The target value of the Temperature default state */
    uint16_t delta_uv;                     /**< The target value of the Delta UV default state */
} light_ctl_default_status_msg_pkt_t;

/**@} end of LIGHT_CTL_MESSAGES INTERNAL */
#endif /* LIGHT_CTL_MESSAGES_H__ */
