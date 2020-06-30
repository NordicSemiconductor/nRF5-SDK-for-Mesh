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

#ifndef LIGHT_LC_MESSAGES_H__
#define LIGHT_LC_MESSAGES_H__

#include <stdint.h>
#include "light_lc_common.h"

/**
 * @internal
 * @defgroup LIGHT_LC_MESSAGES Light LC models message definitions
 * @ingroup LIGHT_LC_MODELS
 *
 * Definitions of the various messages supported by the Light LC server model.
 * @{
 */

/** Shortest allowed length for the LC Light OnOff Set message. */
#define LIGHT_LC_LIGHT_ONOFF_SET_MINLEN 2
/** Longest allowed length for the LC Light OnOff Set message. */
#define LIGHT_LC_LIGHT_ONOFF_SET_MAXLEN 4

/** Fixed length part of the Light LC Property Set and Status messages. */
#define LIGHT_LC_PROPERTY_SET_STATUS_FIXED_LEN 2
/** The Property set message has a 2 byte property id field and a
 * variable length property value field */

/** Length of the Property Get message */
#define LIGHT_LC_PROPERTY_GET_LEN 2

/** Length of the LC Mode Set and Status message. */
#define LIGHT_LC_MODE_SET_STATUS_LEN 1

/** Length of the LC Occupancy Mode Set and Status message. */
#define LIGHT_LC_OCCUPANCY_MODE_SET_STATUS_LEN 1

/** Shortest allowed length for the LC Light OnOff Status message. */
#define LIGHT_LC_LIGHT_ONOFF_STATUS_MINLEN 1
/** Longest allowed length for the LC Light OnOff Status message. */
#define LIGHT_LC_LIGHT_ONOFF_STATUS_MAXLEN 3

/** Shortest allowed length for the Light LC Property Set and Status message. */
#define LIGHT_LC_PROPERTY_SET_STATUS_MINLEN 3
/** The Property status message has a 2 byte property id field and a
 * variable length property value field */

/** Light LC model message opcodes. */
typedef enum
{
    LIGHT_LC_MODE_OPCODE_GET = 0x8291,
    LIGHT_LC_MODE_OPCODE_SET = 0x8292,
    LIGHT_LC_MODE_OPCODE_SET_UNACKNOWLEDGED = 0x8293,
    LIGHT_LC_MODE_OPCODE_STATUS = 0x8294,
    LIGHT_LC_OCCUPANCY_MODE_OPCODE_GET = 0x8295,
    LIGHT_LC_OCCUPANCY_MODE_OPCODE_SET = 0x8296,
    LIGHT_LC_OCCUPANCY_MODE_OPCODE_SET_UNACKNOWLEDGED = 0x8297,
    LIGHT_LC_OCCUPANCY_MODE_OPCODE_STATUS = 0x8298,
    LIGHT_LC_LIGHT_ONOFF_OPCODE_GET = 0x8299,
    LIGHT_LC_LIGHT_ONOFF_OPCODE_SET = 0x829A,
    LIGHT_LC_LIGHT_ONOFF_OPCODE_SET_UNACKNOWLEDGED = 0x829B,
    LIGHT_LC_LIGHT_ONOFF_OPCODE_STATUS = 0x829C,
    LIGHT_LC_PROPERTY_OPCODE_GET = 0x829D,
    LIGHT_LC_PROPERTY_OPCODE_SET = 0x62,
    LIGHT_LC_PROPERTY_OPCODE_SET_UNACKNOWLEDGED = 0x63,
    LIGHT_LC_PROPERTY_OPCODE_STATUS = 0x64,
} light_lc_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the Light LC Mode Set message. */
typedef struct __attribute((packed))
{
    uint8_t mode;                 /**< The target value of the Mode state */
} light_lc_mode_set_msg_pkt_t;

/** Message format for the Light LC Occupancy Mode set message. */
typedef struct __attribute((packed))
{
    uint8_t occupancy_mode;        /**< The target value of the Occupancy Mode state */
} light_lc_occupancy_mode_set_msg_pkt_t;

/** Message format for the Light LC Light OnOff set message. */
typedef struct __attribute((packed))
{
    uint8_t light_on_off;          /**< The target value of the Light LC Light OnOff state */
    uint8_t tid;                   /**< Transaction ID */
    uint8_t transition_time;       /**< Transition time */
    uint8_t delay;                 /**< Message execution delay (5 ms steps)*/
} light_lc_light_onoff_set_msg_pkt_t;

/** Message format for the Light LC Property set message. */
typedef struct __attribute((packed))
{
    uint16_t property_id; /**< Property ID identifying a Light LC Property */
    uint8_t property_buffer[LIGHT_LC_PROPERTY_BUF_SIZE]; /**< Raw value for the Light LC Property */
} light_lc_property_set_msg_pkt_t;

/** Message format for the Light LC Property get message. */
typedef struct __attribute((packed))
{
    uint16_t property_id;          /**< Property ID identifying a Light LC Property */
} light_lc_property_get_msg_pkt_t;

/** Message format for the Light LC Mode Status message. */

typedef struct __attribute((packed))
{
    uint8_t mode;                  /**< The present value of the Light LC Mode state */
} light_lc_mode_status_msg_pkt_t;

/** Message format for the Light LC Occupancy Mode Status message. */
typedef struct __attribute((packed))
{
    uint8_t occupancy_mode;        /**< The present value of the Occupancy Mode state */
} light_lc_occupancy_mode_status_msg_pkt_t;

/** Message format for the Light LC Light OnOff Status message. */
typedef struct __attribute((packed))
{
    uint8_t present_light_onoff;   /**< The present value of the Light OnOff state */
    uint8_t target_light_onoff;    /**< The target value of the Light OnOff state */
    uint8_t remaining_time;        /**< Remaining transition time */
} light_lc_light_onoff_status_msg_pkt_t;

/** Message format for the Light LC Property Status message. */
typedef struct __attribute((packed))
{
    uint16_t property_id; /**< Property ID identifying a Light LC Property */
    uint8_t property_buffer[LIGHT_LC_PROPERTY_BUF_SIZE]; /**< Raw value for the Light LC Property */
} light_lc_property_status_msg_pkt_t;

/**@} end of LIGHT_LC_MESSAGES */
#endif /* LIGHT_LC_MESSAGES_H__ */
