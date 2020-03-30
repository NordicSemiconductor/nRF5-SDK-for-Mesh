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

#ifndef FRIEND_SUBLIST_H__
#define FRIEND_SUBLIST_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "nrf_mesh_config_core.h"

/**
 * @internal
 * @defgroup FRIEND_SUBLIST Friend Subscription List API
 *
 * This API is intented for managing a collection of group and virtual addresses
 * to which the Low Power node is subscribed.
 * @{
 */

/** Structure of statistics for the Friend Subscription List. */
typedef struct
{
    uint32_t lookups;    /**< Number of lookups performed on the subscription list. */
    uint32_t hits;       /**< Number of hits in the lookups. */
    uint32_t removed;    /**< Number of removed entries. */
    uint32_t max_count;  /**< Highest number of entries. */
    uint32_t curr_count; /**< Current number of entries. */
} friend_sublist_stats_t;

/** Friend Subscription List. */
typedef struct
{
    uint16_t addrs[MESH_FRIEND_SUBLIST_SIZE]; /**< 16-bit raw mesh address array. */

#if FRIEND_DEBUG
    friend_sublist_stats_t stats; /**< Statistics for the subscription list instance. */
#endif
} friend_sublist_t;

/**
 * Initializes the Friend Subscription List.
 *
 * @param[in,out] p_sublist             Pointer to the Friend Subscription List.
 */
void friend_sublist_init(friend_sublist_t *p_sublist);

/**
 * Adds the given address to the Friend Subscription List.
 *
 * @param[in,out] p_sublist             Pointer to the Friend Subscription List.
 * @param[in] address                   16-bit raw mesh address.
 *
 * @retval  NRF_SUCCESS                 Address has been added.
 * @retval  NRF_ERROR_NO_MEM            The list is full.
 * @retval  NRF_ERROR_INVALID_PARAM     The given address is not a group or a virtual address.
 */
uint32_t friend_sublist_add(friend_sublist_t *p_sublist, uint16_t address);

/**
 * Removes the given address from the Friend Subscription List.
 *
 * @param[in,out] p_sublist             Pointer to the Friend Subscription List.
 * @param[in] address                   16-bit raw mesh address.
 *
 * @retval  NRF_SUCCESS                 Address has been removed.
 * @retval  NRF_ERROR_NOT_FOUND         The given address is not on the list.
 * @retval  NRF_ERROR_INVALID_PARAM     The given address is not a group or a virtual address.
 */
uint32_t friend_sublist_remove(friend_sublist_t *p_sublist, uint16_t address);

/**
 * Checks whether the Friend Subscription List contains the given address or not.
 *
 * @param[in] p_sublist                 Pointer to the Friend Subscription List.
 * @param[in] address                   16-bit raw mesh address.
 *
 * @retval  NRF_SUCCESS                 The list contains the given address.
 * @retval  NRF_ERROR_NOT_FOUND         The given address is not on the list.
 * @retval  NRF_ERROR_INVALID_PARAM     The given address is not a group or a virtual address.
 */
uint32_t friend_sublist_contains(const friend_sublist_t *p_sublist, uint16_t address);

/**
 * Clears the Friend Subscription List.
 *
 * @param[in,out] p_sublist Pointer to the Friend Subscription List.
 */
void friend_sublist_clear(friend_sublist_t * p_sublist);

/** @} end of FRIEND_SUBLIST */

#endif /* FRIEND_SUBLIST_H__ */
