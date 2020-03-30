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

#include "packed_index_list.h"

void packed_index_list_create(const uint16_t * restrict p_index_list,
        uint8_t * restrict p_packed_list, uint16_t index_count)
{
    uint32_t input_index, output_index;
    for (input_index = 0, output_index = 0; input_index < index_count; input_index += 2, output_index += 3)
    {
        p_packed_list[output_index]     = p_index_list[input_index] & 0xff;                 /* Store the LSBs of the first key index in octet 0 */
        p_packed_list[output_index + 1] = (p_index_list[input_index] >> 8) & 0xf;           /* Store the MSBs of the first key index in octet 1, low bits */

        if (input_index + 1 < index_count)                                                  /* Store the second key index if not at the end of an odd list. */
        {
            p_packed_list[output_index + 1] |= (p_index_list[input_index + 1] << 4) & 0xf0; /* Store the LSBs of the second key index in octet 1, high bits */
            p_packed_list[output_index + 2]  = (p_index_list[input_index + 1] >> 4) & 0xff; /* Store the MSBs of the second key index in octet 2 */
        }
    }
}

