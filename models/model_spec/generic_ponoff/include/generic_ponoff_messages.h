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

#ifndef GENERIC_PONOFF_MESSAGES_H__
#define GENERIC_PONOFF_MESSAGES_H__

#include <stdint.h>
#include "generic_ponoff_common.h"
/**
 * @internal
 * @defgroup GENERIC_PONOFF_MESSAGES Internal header
 * @ingroup GENERIC_PONOFF_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Generic Power OnOff model message opcodes. */
typedef enum
{
    GENERIC_PONOFF_OPCODE_GET = 0x8211,
    GENERIC_PONOFF_OPCODE_STATUS = 0x8212,
    GENERIC_PONOFF_OPCODE_SET = 0x8213,
    GENERIC_PONOFF_OPCODE_SET_UNACKNOWLEDGED = 0x8214
} generic_ponoff_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the Power OnOff Set message. */
typedef struct __attribute((packed))
{
    uint8_t on_powerup;                                /**< Encoded transition time value */
} generic_ponoff_set_msg_pkt_t;

/** Message format for the Power OnOff Status message. */
typedef struct __attribute((packed))
{
    uint8_t on_powerup;                                /**< Encoded transition time value */
} generic_ponoff_status_msg_pkt_t;


/**@} end of GENERIC_PONOFF_MODEL_INTENRAL */
#endif /* GENERIC_PONOFF_MESSAGES_H__ */
