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

#ifndef SIMPLE_ON_OFF_COMMON_H__
#define SIMPLE_ON_OFF_COMMON_H__

#include <stdint.h>
#include "access.h"

/**
 * @defgroup SIMPLE_ON_OFF_MODEL Simple OnOff model
 * This model implements the message based interface required to
 * set the 1 bit value on the server.
 *
 * This implementation of a simple OnOff model can be used to switch things
 * on or off by manipulating a single on/off state. The intention of this model
 * is to have a simple example model that can be used as a baseline for constructing
 * your own model.
 *
 * Do not confuse the simple OnOff model with the Generic OnOff Model specified
 * in @tagMeshMdlSp. The Generic OnOff Model provides additional
 * features such as control over when and for how long the transition between
 * the on/off state should be performed.
 *
 * @note When the server has a publish address set (as in the light switch example),
 * the server will publish its state to its publish address every time its state changes.
 *
 * For more information about creating models, see
 * @ref md_doc_user_guide_modules_models_creating.
 *
 * Model Identification
 * @par
 * Company ID: @ref SIMPLE_ON_OFF_COMPANY_ID
 * @par
 * Simple OnOff Client Model ID: @ref SIMPLE_ON_OFF_CLIENT_MODEL_ID
 * @par
 * Simple OnOff Server Model ID: @ref SIMPLE_ON_OFF_SERVER_MODEL_ID
 *
 * List of supported messages:
 * @par
 * @copydoc SIMPLE_ON_OFF_OPCODE_SET
 * @par
 * @copydoc SIMPLE_ON_OFF_OPCODE_GET
 * @par
 * @copydoc SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE
 * @par
 * @copydoc SIMPLE_ON_OFF_OPCODE_STATUS
 *
 * @ingroup MESH_API_GROUP_VENDOR_MODELS
 * @{
 * @defgroup SIMPLE_ON_OFF_COMMON Common Simple OnOff definitions
 * Types and definitions shared between the two Simple OnOff models.
 * @{
 */

/*lint -align_max(push) -align_max(1) */

/** Vendor specific company ID for Simple OnOff model */
#define SIMPLE_ON_OFF_COMPANY_ID    (ACCESS_COMPANY_ID_NORDIC)

/** Simple OnOff opcodes. */
typedef enum
{
    SIMPLE_ON_OFF_OPCODE_SET = 0xC1,            /**< Simple OnOff Acknowledged Set. */
    SIMPLE_ON_OFF_OPCODE_GET = 0xC2,            /**< Simple OnOff Get. */
    SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE = 0xC3, /**< Simple OnOff Set Unreliable. */
    SIMPLE_ON_OFF_OPCODE_STATUS = 0xC4          /**< Simple OnOff Status. */
} simple_on_off_opcode_t;

/** Message format for the Simple OnOff Set message. */
typedef struct __attribute((packed))
{
    uint8_t on_off; /**< State to set. */
    uint8_t tid;    /**< Transaction number. */
} simple_on_off_msg_set_t;

/** Message format for th Simple OnOff Set Unreliable message. */
typedef struct __attribute((packed))
{
    uint8_t on_off; /**< State to set. */
    uint8_t tid;    /**< Transaction number. */
} simple_on_off_msg_set_unreliable_t;

/** Message format for the Simple OnOff Status message. */
typedef struct __attribute((packed))
{
    uint8_t present_on_off; /**< Current state. */
} simple_on_off_msg_status_t;

/*lint -align_max(pop) */

/** @} end of SIMPLE_ON_OFF_COMMON */
/** @} end of SIMPLE_ON_OFF_MODEL */
#endif /* SIMPLE_ON_OFF_COMMON_H__ */
