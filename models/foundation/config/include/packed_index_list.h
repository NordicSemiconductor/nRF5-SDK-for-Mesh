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

#ifndef PACKED_INDEX_LIST_H__
#define PACKED_INDEX_LIST_H__

#include <stdint.h>

/**
 * @defgroup PACKED_INDEX_LIST Packed index list
 * @ingroup CONFIG_MODEL
 * Provides support for writing lists of packed 12-bit key indexes.
 * @{
 */

/**
 * Calculates the size of a packed index list.
 * Every pair of indexes needs 3 octets.
 * If the number of octets is odd, the last octet needs 2 octets by itself.
 *
 * @param[in] INDEX_COUNT Number of indexes that the list will contain.
 *
 * @returns The size in bytes of the packed index list with the specified
 *          number of indexes.
 */
#define PACKED_INDEX_LIST_SIZE(INDEX_COUNT) (((INDEX_COUNT) / 2) * 3 + ((INDEX_COUNT) & 1) * 2)

/**
 * Creates a packed index list.
 *
 * @param[in]  p_index_list  List of indexes to pack into the output list.
 * @param[out] p_packed_list Pointer to the start of a buffer where the packed index list
 *                           is written. The size that this buffer must have can be obtained
 *                           by using @ref PACKED_INDEX_LIST_SIZE.
 * @param[in] index_count    Number of indexes to store in the packed index list.
 */
void packed_index_list_create(const uint16_t * restrict p_index_list, uint8_t * restrict p_packed_list, uint16_t index_count);

/** @} */

#endif

