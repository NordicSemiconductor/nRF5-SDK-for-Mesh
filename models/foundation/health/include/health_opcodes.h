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

#ifndef HEALTH_OPCODES_H__
#define HEALTH_OPCODES_H__

/**
 * @defgroup HEALTH_OPCODES Health model message opcodes
 * @ingroup HEALTH_MODEL
 * Opcodes used in health model messages.
 * @{
 */

/** Health model opcodes. */
typedef enum
{
    /** Opcode for the "Health Current Status" message. */
    HEALTH_OPCODE_CURRENT_STATUS        = 0x04,
    /** Opcode for the "Health Fault Status" message. */
    HEALTH_OPCODE_FAULT_STATUS          = 0x05,
    /** Opcode for the "Health Attention Get" message. */
    HEALTH_OPCODE_ATTENTION_GET         = 0x8004,
    /** Opcode for the "Health Attention Set" message. */
    HEALTH_OPCODE_ATTENTION_SET         = 0x8005,
    /** Opcode for the "Health Attention Set Unacknowledged" message. */
    HEALTH_OPCODE_ATTENTION_SET_UNACKED = 0x8006,
    /** Opcode for the "Health Attention Status" message. */
    HEALTH_OPCODE_ATTENTION_STATUS      = 0x8007,
    /** Opcode for the "Health Fault Clear" message. */
    HEALTH_OPCODE_FAULT_CLEAR           = 0x802f,
    /** Opcode for the "Health Fault Clear Unacknowledged" message. */
    HEALTH_OPCODE_FAULT_CLEAR_UNACKED   = 0x8030,
    /** Opcode for the "Health Fault Get" message. */
    HEALTH_OPCODE_FAULT_GET             = 0x8031,
    /** Opcode for the "Health Fault Test" message. */
    HEALTH_OPCODE_FAULT_TEST            = 0x8032,
    /** Opcode for the "Health Fault Test Unacknowledged" message. */
    HEALTH_OPCODE_FAULT_TEST_UNACKED    = 0x8033,
    /** Opcode for the "Health Period Get" message. */
    HEALTH_OPCODE_PERIOD_GET            = 0x8034,
    /** Opcode for the "Health Period Set" message. */
    HEALTH_OPCODE_PERIOD_SET            = 0x8035,
    /** Opcode for the "Health Period Set Unacknowledged" message. */
    HEALTH_OPCODE_PERIOD_SET_UNACKED    = 0x8036,
    /** Opcode for the "Health Period Status" message. */
    HEALTH_OPCODE_PERIOD_STATUS         = 0x8037,
} health_opcode_t;

/** @} */

#endif

