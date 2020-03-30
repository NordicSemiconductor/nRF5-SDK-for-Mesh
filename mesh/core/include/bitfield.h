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

#ifndef BITFIELD_H__
#define BITFIELD_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "toolchain.h"

/**
 * @defgroup BITFIELD Bitfield
 * @ingroup MESH_CORE
 * Generic implementation of a variable length bitfield, implemented as an
 * array. Bitfields should be declared as an array of @c uint32_t with length
 * @c BITFIELD_BLOCK_COUNT(BITS).
 *
 * Example: A 48 bit bitfield:
 * @code{.c} uint32_t bitfield[BITFIELD_BLOCK_COUNT(48)]; @endcode
 *
 * @{
 */

/** Size of a single bitfield block (in bits). */
#define BITFIELD_BLOCK_SIZE (32)
/** Mask of the given bit within its bitfield block. */
#define BITFIELD_MASK(BIT) (1u << ((BIT) & (BITFIELD_BLOCK_SIZE - 1)))
/** Number of blocks required to hold the given number of bits */
#define BITFIELD_BLOCK_COUNT(BITS) (((BITS) + BITFIELD_BLOCK_SIZE - 1) / BITFIELD_BLOCK_SIZE)

/**
 * Gets the value of a single bit from the given bitfield.
 *
 * @param[in] p_bitfield The bitfield to check.
 * @param[in] bit        The bit to get the value of.
 *
 * @returns The value of the given bit in the given bitfield.
 */
static inline bool bitfield_get(const uint32_t * p_bitfield, uint32_t bit)
{
    return !!(p_bitfield[bit / BITFIELD_BLOCK_SIZE] & BITFIELD_MASK(bit));
}

/**
 * Sets the given bit.
 *
 * @param[in,out] p_bitfield Bitfield to operate on.
 * @param[in]     bit        Bit to set.
 */
static inline void bitfield_set(uint32_t * p_bitfield, uint32_t bit)
{
    p_bitfield[bit / BITFIELD_BLOCK_SIZE] |= BITFIELD_MASK(bit);
}

/**
 * Clears the given bit.
 *
 * @param[in,out] p_bitfield Bitfield to operate on.
 * @param[in]     bit        Bit to set.
 */
static inline void bitfield_clear(uint32_t * p_bitfield, uint32_t bit)
{
    p_bitfield[bit / BITFIELD_BLOCK_SIZE] &= ~(BITFIELD_MASK(bit));
}

/**
 * Sets all bits in the given bitfield.
 *
 * @note If @p bits is less than the number of bits allocated for the given
 *       bitfield, all blocks up until and including the block that holds the @p bits
 *       bit will be set.
 *
 * @param[in,out] p_bitfield Bitfield to operate on.
 * @param[in]     bits       Number of bits in the bitfield.
 */
static inline void bitfield_set_all(uint32_t * p_bitfield, uint32_t bits)
{
    memset(p_bitfield, 0xFF, (BITFIELD_BLOCK_SIZE / 8) * BITFIELD_BLOCK_COUNT(bits));
}

/**
 * Clears all bits in the given bitfield.
 *
 * @note If @p bits is less than the number of bits allocated for the given
 *       bitfield, all blocks up until and including the block that holds the @p bits
 *       bit will be cleared.
 *
 * @param[in,out] p_bitfield Bitfield to operate on.
 * @param[in]     bits       Number of bits in the bitfield.
 */
static inline void bitfield_clear_all(uint32_t * p_bitfield, uint32_t bits)
{
    memset(p_bitfield, 0x00, (BITFIELD_BLOCK_SIZE / 8) * BITFIELD_BLOCK_COUNT(bits));
}

/**
 * Gets the index of the next set bit, starting at @p start.
 *
 * The @p start parameter must be incremented for each lookup to get
 * different bits, and can be used to iterate through all set bits in
 * a bitfield using a for loop:
 * @code{.c}
 * for(uint32_t i = bitfield_next_get(bitfield, BITFIELD_SIZE, 0);
 *     i != BITFIELD_SIZE;
 *     i = bitfield_next_get(bitfield, BITFIELD_SIZE, i + 1))
 * {
 *     // Do something to the bit here.
 * }
 * @endcode
 *
 * @note See https://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightMultLookup
 *
 * @param[in] p_bitfield Bitfield to operate on.
 * @param[in] bits       Number of bits in the bitfield.
 * @param[in] start      First index to look at.
 *
 * @returns Index of the next bit that's 1 in the given bitfield, from the
 *          given position, or @p bits, if no more bits are set.
 */
static inline uint32_t bitfield_next_get(const uint32_t * p_bitfield, uint32_t bits, uint32_t start)
{
    /* Lookup table for offset into a single bitfield block. */
    static const uint8_t multiply_de_bruijn_bit_position[32] =
    {
        0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
        31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
    };

    if (start >= bits)
    {
        return bits;
    }

    uint32_t i = start / BITFIELD_BLOCK_SIZE;
    uint32_t v = p_bitfield[i] & ~(BITFIELD_MASK(start) - 1);
    while (v == 0)
    {
        ++i;
        if (i >= BITFIELD_BLOCK_COUNT(bits))
        {
            return bits;
        }
        else
        {
            v = p_bitfield[i];
        }
    }
    return (i * BITFIELD_BLOCK_SIZE) + (uint32_t) multiply_de_bruijn_bit_position[((uint32_t)((v & -v) * 0x077CB531U)) >> 27];
}

/**
 * Gets the number of bits that are set in a bitfield.
 *
 * @param[in] p_bitfield Bitfield to operate on.
 * @param[in] bits Number of bits in the bitfield.
 *
 * @returns Number of bits in the bitfield that are set to 1.
 */
static inline uint32_t bitfield_popcount(const uint32_t * p_bitfield, uint32_t bits)
{
    uint32_t popcount = 0;
    for(uint32_t i = bitfield_next_get(p_bitfield, bits, 0); i != bits; i = bitfield_next_get(p_bitfield, bits, i + 1))
    {
        popcount++;
    }

    return popcount;
}

/**
 * Checks whether the given bitfield has all bits cleared.
 *
 * @param[in] p_bitfield Bitfield to operate on.
 * @param[in] bits       Number of bits in the bitfield.
 *
 * @returns Whether the bitfield has all bits cleared.
 */
static inline bool bitfield_is_all_clear(const uint32_t * p_bitfield, uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        uint32_t mask = 0xFFFFFFFF;
        if (i == (BITFIELD_BLOCK_COUNT(bits) - 1))
        {
            /* Only check the bits that actually are part of the bitfield */
            mask = (1 << (bits - i * BITFIELD_BLOCK_SIZE)) - 1;
        }
        if ((p_bitfield[i] & mask) != 0)
        {
            return false;
        }
    }
    return true;
}

/**
 * Checks whether the given bitfield has all bits set.
 *
 * @param[in] p_bitfield Bitfield to operate on.
 * @param[in] bits       Number of bits in the bitfield.
 *
 * @returns Whether the bitfield has all bits set.
 */
static inline bool bitfield_is_all_set(const uint32_t * p_bitfield, uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        uint32_t mask = 0xFFFFFFFF;
        if (i == (BITFIELD_BLOCK_COUNT(bits) - 1))
        {
            /* Only check the bits that actually are part of the bitfield */
            mask = (1 << (bits - i * BITFIELD_BLOCK_SIZE)) - 1;
        }

        if ((p_bitfield[i] & mask) != mask)
        {
            return false;
        }
    }
    return true;
}

/**
 * Checks whether all the bits set in p_b2 is a subset of p_b1.
 *
 * Example: p_b1: 0b01011, p_b2: 0b01001 => true.
 *          p_b1: 0b01011, p_b2: 0b00111 => false.
 *
 * @warning This will return @c true if p_b2 is zero.
 *
 * @param[in] p_b1 Pointer to reference bitfield.
 * @param[in] p_b2 Pointer to bitfield to verify.
 * @param[in] bits Number of bits in the smallest bitfield.
 *
 * @returns Whether the bits set in p_b2 is a valid subset of p_b1.
 */
static inline bool bitfield_is_subset_of(const uint32_t * p_b1, const uint32_t * p_b2, const uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        if ((~p_b1[i] & p_b2[i]) != 0)
        {
            return false;
        }
    }
    return true;
}


/** @} */

#endif /* BITFIELD_H__ */

