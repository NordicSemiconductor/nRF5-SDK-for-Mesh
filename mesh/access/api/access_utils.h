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

#ifndef ACCESS_UTILS_H__
#define ACCESS_UTILS_H__

#include <stdint.h>
#include "access.h"

/**
 * @defgroup ACCESS_UTILS Utility functions
 * @ingroup ACCESS
 * Utility functions for access layer users.
 * @{
 */

/** Size of a SIG model opcode in bytes. */
#define ACCESS_UTILS_SIG_OPCODE_SIZE(OPCODE) (((OPCODE) > 0x00FF) ? 2 : 1)
/** Size of a vendor model opcode in bytes. */
#define ACCESS_UTILS_VENDOR_OPCODE_SIZE      3

/**
 * Gets the raw size in bytes of an access layer opcode.
 *
 * @param[in] opcode Access layer opcode.
 *
 * @returns Size in bytes of opcode.
 */
static inline uint16_t access_utils_opcode_size_get(access_opcode_t opcode)
{
    if (opcode.company_id != ACCESS_COMPANY_ID_NONE)
    {
        return ACCESS_UTILS_VENDOR_OPCODE_SIZE;
    }
    else
    {
        return ACCESS_UTILS_SIG_OPCODE_SIZE(opcode.opcode);
    }
}

/** @} end of ACCESS_UTILS */

#endif /* ACCESS_UTILS_H__ */

