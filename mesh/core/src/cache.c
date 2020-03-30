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
#include "cache.h"
#include <string.h>

#define CACHE_ELEM_AT(p_cache, index) ((uint8_t*) p_cache->elem_array + index * p_cache->elem_size)

static void erase_at(const cache_t* p_cache, uint32_t index)
{
    if (p_cache->erase_fptr)
    {
        p_cache->erase_fptr(CACHE_ELEM_AT(p_cache, index));
    }
    else
    {
        memset(CACHE_ELEM_AT(p_cache, index), 0, p_cache->elem_size);
    }
}

void cache_init(cache_t* p_cache)
{
    for (uint32_t i = 0; i < p_cache->array_len; ++i)
    {
        erase_at(p_cache, i);
    }
    p_cache->head = p_cache->array_len - 1;
}

bool cache_has_elem(const cache_t* p_cache, const void* p_elem)
{
    uint32_t index = p_cache->head;
    for (uint32_t i = 0; i < p_cache->array_len; ++i)
    {
        if (p_cache->memcmp_fptr)
        {
            if (p_cache->memcmp_fptr(p_elem, CACHE_ELEM_AT(p_cache, index)))
            {
                return true;
            }
        }
        else
        {
            if (memcmp(p_elem, CACHE_ELEM_AT(p_cache, index), p_cache->elem_size) == 0)
            {
                return true;
            }
        }

        if (index-- == 0)
        {
            index = p_cache->array_len - 1;
        }
    }

    return false;

}

void cache_put(cache_t* p_cache, const void* p_elem)
{
    if (++p_cache->head == p_cache->array_len)
    {
        p_cache->head = 0;
    }

    if (p_cache->memcpy_fptr)
    {
        p_cache->memcpy_fptr(CACHE_ELEM_AT(p_cache, p_cache->head), p_elem);
    }
    else
    {
        memcpy(CACHE_ELEM_AT(p_cache, p_cache->head), p_elem, p_cache->elem_size);
    }
}

uint32_t cache_erase_elem(cache_t* p_cache, const void* p_elem)
{
    uint32_t erase_count = 0;
    uint32_t index = p_cache->head;
    for (uint32_t i = 0; i < p_cache->array_len; ++i)
    {
        if (p_cache->memcmp_fptr)
        {
            if (p_cache->memcmp_fptr(p_elem, CACHE_ELEM_AT(p_cache, index)))
            {
                erase_at(p_cache, index);
                erase_count++;
            }
        }
        else
        {
            if (memcmp(p_elem, CACHE_ELEM_AT(p_cache, index), p_cache->elem_size) == 0)
            {
                erase_at(p_cache, index);
                erase_count++;
            }
        }

        if (index-- == 0)
        {
            index = p_cache->array_len - 1;
        }
    }

    return erase_count;
}
