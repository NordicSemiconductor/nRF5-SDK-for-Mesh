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

#include "friend_sublist.h"

#include <string.h>
#include "nordic_common.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_assert.h"

/******************************************************************************
* Static functions
******************************************************************************/

static bool address_check(uint16_t address)
{
    nrf_mesh_address_type_t address_type = nrf_mesh_address_type_get(address);

    if (address_type != NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
        address_type != NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        return false;
    }

    return true;
}

static bool address_lookup(const friend_sublist_t *p_sublist, uint16_t address, size_t *p_i)
{
    for (size_t i = 0; i < MESH_FRIEND_SUBLIST_SIZE; i++)
    {
        if (p_sublist->addrs[i] == address)
        {
            *p_i = i;
            return true;
        }
    }

    return false;
}

/******************************************************************************
* Interface functions
******************************************************************************/

void friend_sublist_init(friend_sublist_t *p_sublist)
{
    NRF_MESH_ASSERT_DEBUG(NULL != p_sublist);

    memset(p_sublist, 0, sizeof(friend_sublist_t));
}

uint32_t friend_sublist_add(friend_sublist_t *p_sublist, uint16_t address)
{
    NRF_MESH_ASSERT_DEBUG(NULL != p_sublist);

    if (!address_check(address))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    size_t i;
    if (address_lookup(p_sublist, address, &i))
    {
        return NRF_SUCCESS;
    }

    if (!address_lookup(p_sublist, NRF_MESH_ADDR_UNASSIGNED, &i))
    {
        return NRF_ERROR_NO_MEM;
    }

#if FRIEND_DEBUG
    p_sublist->stats.curr_count++;
    p_sublist->stats.max_count = MIN(p_sublist->stats.max_count + 1, MESH_FRIEND_SUBLIST_SIZE);
#endif

    p_sublist->addrs[i] = address;
    return NRF_SUCCESS;
}

uint32_t friend_sublist_remove(friend_sublist_t *p_sublist, uint16_t address)
{
    NRF_MESH_ASSERT_DEBUG(NULL != p_sublist);

    if (!address_check(address))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    size_t i;
    if (!address_lookup(p_sublist, address, &i))
    {
        return NRF_ERROR_NOT_FOUND;
    }

#if FRIEND_DEBUG
    p_sublist->stats.curr_count--;
    p_sublist->stats.removed++;
#endif

    p_sublist->addrs[i] = NRF_MESH_ADDR_UNASSIGNED;
    return NRF_SUCCESS;
}

uint32_t friend_sublist_contains(const friend_sublist_t *p_sublist, uint16_t address)
{
    NRF_MESH_ASSERT_DEBUG(NULL != p_sublist);

#if FRIEND_DEBUG
    ((friend_sublist_t *)p_sublist)->stats.lookups++;
#endif

    if (!address_check(address))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    size_t i;
    bool res = address_lookup(p_sublist, address, &i);

#if FRIEND_DEBUG
    if (res)
    {
        ((friend_sublist_t *)p_sublist)->stats.hits++;
    }
#endif

    return res ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}

void friend_sublist_clear(friend_sublist_t * p_sublist)
{
    NRF_MESH_ASSERT_DEBUG(NULL != p_sublist);
    friend_sublist_init(p_sublist);
}
