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

#ifndef MESH_FRIEND_H__
#define MESH_FRIEND_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_friendship_types.h"
#include "friend_sublist.h"
#include "friend_queue.h"
#include "core_tx_friend.h"

/**
 * @ingroup MESH_API_GROUP_FRIEND
 * @{
 */

/** Shortest receive window supported by the Friend node. */
#define MESH_FRIEND_RECEIVE_WINDOW_MIN_MS (1)
/** Longest receive window supported by the Friend node. */
#define MESH_FRIEND_RECEIVE_WINDOW_MAX_MS (255)
/**
 * Default receive window offered by the Friend node.
 * @note You can later change this with @ref mesh_friend_receive_window_set().
 */
#define MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS (5)


/** Friendship statistics structure. */
typedef struct
{
    friend_queue_stats_t queue; /**< Statistics for the Friend Queue submodule. */
    friend_sublist_stats_t sublist; /**< Statistics for the Friend Subscription List submodule. */
    core_tx_friend_stats_t bearer; /**< Statistics for the Friend Bearer submodule. */
} mesh_friend_stats_t;

/**
 * Initializes the Friend feature.
 *
 * @retval NRF_SUCCESS             Successfully initialized the Friend feature.
 * @retval NRF_ERROR_INVALID_STATE Friend feature has already been initialized.
 */
uint32_t mesh_friend_init(void);

/**
 * Enables the Friend feature.
 *
 * @note After the Friend feature has been enabled, the Friend is required to respond to any Friend
 * Request from a Low Power node (see the @tagMeshSp, section 3.6.6.3.1).
 *
 * The application can receive any of the following events after this API has been called:
 * - @ref NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED
 * - @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED
 */
void mesh_friend_enable(void);

/**
 * Disables the Friend feature.
 *
 * Disabling the Friend feature will terminate all active friendships.
 */
void mesh_friend_disable(void);

/**
 * Gets the state of the Friend feature.
 *
 * @retval true  Friend feature is enabled.
 * @retval false Friend feature is disabled.
 */
bool mesh_friend_is_enabled(void);

/**
 * Terminates the friendship with the given LPN.
 *
 * @note This API may be used to deny a friendship request.
 *
 * When the friendship is considered terminated, an @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED
 * event will be generated. If the API is used to deny a Friend Request, no event will be generated.
 *
 * @param[in] p_friendship Friendship context pointer corresponding to the friendship that is to be
 *                         terminated.
 *
 * @retval NRF_SUCCESS     Successfully terminated the friendship.
 * @retval NRF_ERROR_PARAM No known friendship is associated with the given context.
 */
uint32_t mesh_friend_friendship_terminate(const mesh_friendship_t * p_friendship);

/**
 * Terminates all active friendships.
 *
 * An @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED will be generated for each of the established
 * friendships.
 *
 * @retval NRF_SUCCESS Successfully terminated all active friendships.
 */
uint32_t mesh_friend_friendship_terminate_all(void);

/**
 * Sets the receive window offered in the Friend Offer.
 *
 * The new receive window is only valid for _new_ friendships. Calling this function will have
 * no effect on existing friendships.
 *
 * @param [in] receive_window_ms Receive window in milliseconds.
 *
 * @retval NRF_SUCCESS             Successfully set the receive window.
 * @retval NRF_ERROR_INVALID_PARAM @p receive_window_ms is not between
 *                                 @ref MESH_FRIEND_RECEIVE_WINDOW_MIN_MS and
 *                                 @ref MESH_FRIEND_RECEIVE_WINDOW_MAX_MS.
 */
uint32_t mesh_friend_receive_window_set(uint8_t receive_window_ms);

/**
 * Gets all the current active friendships.
 *
 * Usage:
 * @code
 * const mesh_friendship_t * friendships[MESH_FRIEND_FRIENDSHIP_COUNT];
 * uint8_t count = MESH_FRIEND_FRIENDSHIP_COUNT;
 *
 * uint32_t error_code = mesh_friend_friendships_get(&friendships[0], &count);
 * @endcode
 *
 * @param[in,out] pp_friendships Array of const @ref mesh_friendship_t pointers.
 * @param[in,out] p_count        In: Number of elements in @p pp_friendships.
 *                               Out: Number of elements stored in @p pp_friendships.
 *
 * @retval NRF_SUCCESS    Successfully retrieved all active friendships.
 * @retval NRF_ERROR_NULL One or more of the function arguments were @c NULL.
 */
uint32_t mesh_friend_friendships_get(const mesh_friendship_t ** pp_friendships, uint8_t * p_count);

/**
 * Gets the structures of statistics for the given friendship.
 *
 * Friendship statistics are only gathered if @c FRIEND_DEBUG is enabled.
 *
 * @param[in] p_friendship  Friendship to get the statistics for.
 * @param[in,out] p_stats   Statistics structure to fill.
 *
 * @retval NRF_SUCCESS              Structure of statistics successfully populated.
 * @retval NRF_ERROR_NULL           One or more of the function arguments were @c NULL.
 * @retval NRF_ERROR_NOT_FOUND      No known friendship is associated with the given context.
 * @retval NRF_ERROR_NOT_SUPPORTED  Gathering of Friend statistics not enabled.
 */
uint32_t mesh_friend_stats_get(const mesh_friendship_t * p_friendship, mesh_friend_stats_t * p_stats);

/** @} end of MESH_FRIEND */

#endif /* MESH_FRIEND_H__ */
