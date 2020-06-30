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

#ifndef SCENE_MESSAGES_H__
#define SCENE_MESSAGES_H__

#include <stdint.h>
#include "scene_common.h"
/**
 * @internal
 * @defgroup SCENE_MESSAGES Internal header
 * @ingroup SCENE_MODELS
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Length of the Scene Store/Delete message. */
#define SCENE_STORE_DELETE_LEN 2

/** Shortest allowed length of the Scene Recall message. */
#define SCENE_RECALL_MINLEN 3
/** Longest allowed length of the Scene Recall message. */
#define SCENE_RECALL_MAXLEN 5

/** Shortest allowed length of the Scene Status message. */
#define SCENE_STATUS_MINLEN 3
/** Longest allowed length of the Scene Status message. */
#define SCENE_STATUS_MAXLEN 6

/** Shortest allowed length of the Scene Register Status message. */
#define SCENE_REGISTER_STATUS_MINLEN 3
/** Longest allowed length of the Scene Register Status message. */
#define SCENE_REGISTER_STATUS_MAXLEN (SCENE_REGISTER_STATUS_MINLEN + (2 * SCENE_REGISTER_ARRAY_SIZE))

/** Scene model message opcodes. */
typedef enum
{
    SCENE_OPCODE_GET = 0x8241,
    SCENE_OPCODE_RECALL = 0x8242,
    SCENE_OPCODE_RECALL_UNACKNOWLEDGED = 0x8243,
    SCENE_OPCODE_STATUS = 0x5E,
    SCENE_OPCODE_REGISTER_GET = 0x8244,
    SCENE_OPCODE_REGISTER_STATUS = 0x8245,
    SCENE_OPCODE_STORE = 0x8246,
    SCENE_OPCODE_STORE_UNACKNOWLEDGED = 0x8247,
    SCENE_OPCODE_DELETE = 0x829E,
    SCENE_OPCODE_DELETE_UNACKNOWLEDGED = 0x829F,
} scene_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the Scene Store message (actual/perceptually uniform state). */
typedef struct __attribute((packed))
{
    uint16_t scene_number;              /**< Number of the scene to be stored */
} scene_store_msg_pkt_t;

/** Message format for the Scene Recall message. */
typedef struct __attribute((packed))
{
    uint16_t scene_number;              /**< Number of the scene to be stored */
    uint8_t tid;                        /**< Transaction number for application */
    uint8_t transition_time;            /**< Encoded transition time value */
    uint8_t delay;                      /**< Encoded message execution delay in 5 millisecond steps */
} scene_recall_msg_pkt_t;

/** Message format for the Scene Status message. */
typedef struct __attribute((packed))
{
    uint8_t status_code;                /**< Status code for the last operation */
    uint16_t current_scene;             /**< Scene number of a current scene */
    uint16_t target_scene;              /**< Scene number of a target scene */
    uint8_t remaining_time;             /**< Encoded remaining transition time */
} scene_status_msg_pkt_t;

/** Message format for the Scene Register Status message
    5.2.2.8 Scene Register Status
    Scene Register Status is an unacknowledged message that is used to report the current status of
    the Scene Register (see Section 5.1.3.1) of an element.

    The message structure:
                                    Field Size
    Field Name                       (octets)       Notes
    ------------------------------------------------------------------------------------------------
    Status Code                         1           Defined in Sectio 5.2.2.11.
    Current Scene                       2           Scene Number of current scene.
    Scenes                           variable       A list of scenes stored within an element.
 */
typedef struct __attribute((packed))
{
    uint8_t status_code;                /**< Status code for the last operation */
    uint16_t current_scene;             /**< Scene number of a current scene */
    uint16_t scenes[];                  /**< A list of scenes stored within an element */
} scene_register_status_msg_pkt_t;

/** Message format for the Scene Delete message. */
typedef struct __attribute((packed))
{
    uint16_t scene_number;              /**< Number of the scene to be deleted */
} scene_delete_msg_pkt_t;

/**@} end of SCENE_MESSAGES */
#endif /* SCENE_MESSAGES_H__ */
