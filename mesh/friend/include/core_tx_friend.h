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
#ifndef CORE_TX_FRIEND_H__
#define CORE_TX_FRIEND_H__

#include <stdbool.h>
#include <stdint.h>
#include "core_tx.h"
#include "broadcast.h"
#include "timer_scheduler.h"
#include "bearer_event.h"

/**
 * @defgroup CORE_TX_FRIEND Core TX Friend bearer
 * @internal
 * @{
 */

typedef enum
{
    CORE_TX_FRIEND_STATE_UNINITIALIZED,     /**< Bearer instance has not been initialized. */
    CORE_TX_FRIEND_STATE_READY,             /**< Ready to accept packets. */
    CORE_TX_FRIEND_STATE_ALLOCATED,         /**< Packet is allocated, but not marked for sending. */
    CORE_TX_FRIEND_STATE_WAITING,           /**< Waiting for timeout. */
    CORE_TX_FRIEND_STATE_SENDING,           /**< Packet is being sent. */
} core_tx_friend_state_t;

/** Stucture of statistics for the Core TX Friend module. */
typedef struct
{
    int32_t tx_timeout_delay_max_us;   /**< Maximum delay recorded from the desired TX timestamp to the "TX complete" timestamp from the broadcast. */
    int32_t tx_timeout_delay_cur_us;   /**< Delay for the last completed action from the desired TX timestamp to the "TX complete" timestamp from the broadcast. */
    uint32_t last_request_timestamp;   /**< Timestamp of the last requested action. */
} core_tx_friend_stats_t;

typedef struct
{
    bool enabled;                       /**< Whether this bearer instance is enabled. */
    core_tx_friend_state_t state;       /**< State of the instance. */
    broadcast_t broadcast;              /**< Broadcast instance. */
    nrf_mesh_tx_token_t token; 		    /**< TX token associated with this instance. */
    struct
    {
        core_tx_role_t role;            /**< Role associated with this packet. */
        packet_t buffer;                /**< Packet buffer. */
        timestamp_t timestamp;          /**< Timestamp when the packet was sent. */
    } packet;
    timer_event_t tx_timer;             /**< Timer instance for sending. */
    core_tx_bearer_t bearer;            /**< Bearer instance for Core TX. */
    bearer_event_sequential_t tx_complete_event; /**< Processing event for TX complete. */

#if FRIEND_DEBUG
    core_tx_friend_stats_t stats;
#endif
} core_tx_friend_t;

/**
 * Initializes a Friend bearer instance.
 *
 * Leaves the bearer instance in a disabled state.
 *
 * @warning This function asserts if called twice on the same instance.
 *
 * @param[in,out]   p_bearer    Friend bearer instance to initialize.
 * @param[in]       token       TX token associated with all packets that will go out on this bearer. Used to
 *                              determine whether a packet will be enqueued when it comes down from Core TX.
 * @param[in]       tx_power    transmit power
 */
void core_tx_friend_init(core_tx_friend_t * p_bearer, nrf_mesh_tx_token_t token, radio_tx_power_t tx_power);

/**
 * Enables a Friend bearer instance.
 *
 * The bearer instance will be able to start sending packets.
 *
 * @param[in,out] p_bearer Bearer instance to enable.
 */
void core_tx_friend_enable(core_tx_friend_t * p_bearer);

/**
 * Disables a Friend bearer instance.
 *
 * @warning Asserts if the instance is in the @ref CORE_TX_FRIEND_STATE_ALLOCATED state.
 *
 * @param[in,out] p_bearer Bearer instance to disable.
 */
void core_tx_friend_disable(core_tx_friend_t * p_bearer);

/**
 * Schedules the next packet transmission.
 *
 * Instructs the bearer to send the next packet at the given time.
 *
 * @param[in,out]   p_bearer    Friend bearer instance to schedule the TX for.
 * @param[in]       tx_time     Time to send the TX.
 */
void core_tx_friend_schedule(core_tx_friend_t * p_bearer, timestamp_t tx_time);

/** @} */

#endif /* CORE_TX_FRIEND_H__ */
