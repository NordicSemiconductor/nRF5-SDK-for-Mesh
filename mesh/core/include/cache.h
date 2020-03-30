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

#ifndef MESH_CACHE_H__
#define MESH_CACHE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup CACHE Generic Cache storage
 * @ingroup MESH_CORE
 * Generic cache storage structure implementing a Least Recently Used-cache.
 * @{
 */

/**
 * Function pointer type for copying memory between two elements in the cache.
 *
 * Used to replace @c memcpy in case the elements require more complex copy mechanisms, such as
 * deep copy.
 *
 * @param[out] p_dest Pointer to the destination memory.
 * @param[in]  p_src  Pointer to the source memory.
 */
typedef void (*cache_memcpy_t)(void * p_dest, const void * p_src);

/**
 * Function pointer type for erasing an element in the cache.
 * If an erase-function pointer isn't provided in the cache structure, erase
 * is done by setting every byte in the element to 0.
 *
 * @param[in,out] p_elem Pointer to the element to erase.
 */
typedef void (*cache_erase_t)(void * p_elem);

/**
 * Function pointer type for comparing memory between two elements in the cache.
 *
 * @param[in] p_elem1 Element to compare.
 * @param[in] p_elem2 Element to compare against.
 *
 * @return @c true if the two elements are equal.
 */
typedef bool (*cache_memcmp_t)(const void* p_elem1, const void* p_elem2);

/** Cache context structure. */
typedef struct
{
    void*           elem_array;     /**< Pointer to a buffer to use as storage */
    uint32_t        elem_size;      /**< Size of a single element in bytes. */
    uint32_t        array_len;      /**< Number of elements in the elem_array. Must be in the power of two. */
    uint32_t        head;           /**< Index of the next element to be overwritten. */
    cache_memcpy_t  memcpy_fptr;    /**< Function pointer to alternate element copy. Must be a valid function or NULL. */
    cache_memcmp_t  memcmp_fptr;    /**< Function pointer to alternate element compare. Must be a valid function or NULL. */
    cache_erase_t   erase_fptr;     /**< Function pointer to alternate element erase. Must be a valid function or NULL. */
} cache_t;

/**
 * Initializes the given cache structure.
 * Erases all elements and resets the head.
 *
 * @param[in,out] p_cache Pointer to a cache structure, with all internal
 *                parameters set.
 */
void cache_init(cache_t* p_cache);

/**
 * Looks for an element matching the specified element in the given cache.
 *
 * Uses the cache structure's @c memcmp_fptr, if one is provided, to do the
 * comparison, if provided.
 *
 * @param[in] p_cache Pointer to the cache structure to look through.
 * @param[in] p_elem  Pointer to an elem to look for.
 *
 * @retval true if the given element exists in the cache.
 */
bool cache_has_elem(const cache_t* p_cache, const void* p_elem);

/**
 * Adds an entry to the cache.
 *
 * Does not look for duplicates, and overwrites the oldest entry.
 *
 * Uses the cache structure's @c memcpy_fptr, if one is provided, to copy the entry
 * into the cache.
 *
 * @param[in, out] p_cache Pointer to a cache structure to post the element to.
 * @param[in]      p_elem  Pointer to an element to add to the cache.
 */
void cache_put(cache_t* p_cache, const void* p_elem);

/**
 * Erases all elements in the cache matching the given element.
 *
 * Uses the cache structure's @c erase_fptr if one is provided.
 *
 * @param[in, out] p_cache Pointer to a cache structure to erase from.
 * @param[in] p_elem Pointer to an element to match against.
 *
 * @return The number of erased elements.
 */
uint32_t cache_erase_elem(cache_t* p_cache, const void* p_elem);

/** @} */

#endif /* CACHE_H__ */
