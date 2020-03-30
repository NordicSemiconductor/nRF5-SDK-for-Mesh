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

#ifndef MESH_FRIENDSHIP_TYPES_H__
#define MESH_FRIENDSHIP_TYPES_H__

#include <stdint.h>

/**
 * @defgroup MESH_FRIENDSHIP_TYPES Mesh friendship types
 * @ingroup NRF_MESH
 *
 * A collection of types common to both the Low Power node (LPN) and the Friend role.
 * @{
 */

/**
 * Weight factor applied to the received RSSI by the Friend node.
 *
 * Available factors are 1.0, 1.5, 2.0, and 2.5. The lower the factor, the less a low received RSSI
 * increases the delay of the Friend Offer. See the equation in @ref
 * mesh_lpn_friend_request_t.friend_criteria.
 */
typedef enum
{
    MESH_FRIENDSHIP_RSSI_FACTOR_1_0 = 0,
    MESH_FRIENDSHIP_RSSI_FACTOR_1_5 = 1,
    MESH_FRIENDSHIP_RSSI_FACTOR_2_0 = 2,
    MESH_FRIENDSHIP_RSSI_FACTOR_2_5 = 3
} mesh_friendship_rssi_factor_t;

/**
 * Weight factor applied to the offered Receive Window by the Friend node.
 *
 * Available factors are 1.0, 1.5, 2.0, and 2.5. The lower the factor, the less a long offered
 * Receive Window increases the delay of the Friend Offer. See the equation in @ref
 * mesh_lpn_friend_request_t.friend_criteria.
 */
typedef enum
{
    MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_1_0 = 0,
    MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_1_5 = 1,
    MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_2_0 = 2,
    MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_2_5 = 3
} mesh_friendship_receive_window_factor_t;

/**
 * Minimum size of the Friend Queue.
 *
 * This is the minimum number of Lower Transport PDUs that the Friend node can store.
 */
typedef enum
{
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_PROHIBITED = 0,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_2          = 1,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_4          = 2,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_8          = 3,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_16         = 4,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_32         = 5,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_64         = 6,
    MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_128        = 7
} mesh_friendship_min_friend_queue_size_t;

typedef struct
{
    uint16_t src;               /**< LPN source address. */
    uint16_t prev_friend_src;   /**< Source of the previous Friend. */
    uint16_t element_count;     /**< Number of elements in the LPN. */
    uint16_t request_count;     /**< Number of Friend Requests sent by the LPN. */
} mesh_friendship_lpn_t;

typedef struct
{
    mesh_friendship_lpn_t lpn;  /**< Low Power node data. */
    uint32_t poll_timeout_ms;   /**< Poll Timeout in milliseconds. */
    uint32_t poll_count;        /**< Number of polls received from the LPN. */
    uint8_t receive_delay_ms;   /**< Delay before the receive window starts, in milliseconds. */
    uint8_t receive_window_ms;  /**< Length of the LPN's Receive Window in milliseconds. */
    int8_t avg_rssi;            /**< Average RSSI of the LPN messages received. */
} mesh_friendship_t;

/** @} end of MESH_FRIENDSHIP_TYPES */

#endif /* MESH_FRIENDSHIP_TYPES_H__ */
