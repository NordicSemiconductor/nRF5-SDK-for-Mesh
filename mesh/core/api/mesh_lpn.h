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

#ifndef MESH_LPN_H__
#define MESH_LPN_H__

#include <stdint.h>
#include "mesh_friendship_types.h"
#include "nrf_mesh_events.h"

/**
 * @defgroup MESH_LPN Mesh Low Power node (LPN)
 * @ingroup NRF_MESH
 * @{
 */

/**
 * Minimum Friend Request timeout (in milliseconds).
 *
 * @note The timeout starts counting from the time the Friend Request is sent. The device does not start
 * scanning until 100 milliseconds have passed from the Friend Request.
 * See @tagMeshSp section 3.6.6.4.1.
 */
#define MESH_LPN_FRIEND_REQUEST_TIMEOUT_MIN_MS 100
/** Maximum Friend Request timeout (in milliseconds). */
#define MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS 1000

/** Minimum supported poll timeout (in milliseconds). */
#define MESH_LPN_POLL_TIMEOUT_MIN_MS 1000
/** Maximum supported poll timeout (in milliseconds). This translates to slightly less than
 * 96 hours (0x34BBFF * 100 ms). The valid range is defined in Table 3.26, @tagMeshSp section 3.6.5.3 */
#define MESH_LPN_POLL_TIMEOUT_MAX_MS 345599900

/* See @tagMeshSp section 3.6.5.3 Friend Request */
/** Minimum allowed Receive Delay (in milliseconds). */
#define MESH_LPN_RECEIVE_DELAY_MIN_MS 10
/** Maximum allowed Receive Delay (in milliseconds). */
#define MESH_LPN_RECEIVE_DELAY_MAX_MS 255

/** The number of times the LPN will retry polling the Friend before regarding the friendship as
 * terminated. */
#ifndef MESH_LPN_POLL_RETRY_COUNT
#define MESH_LPN_POLL_RETRY_COUNT 5
#endif

/** The number of times the LPN will retry the friend request procedure until @ref NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT. */
#ifndef MESH_LPN_FRIEND_REQUEST_RETRY_COUNT
#define MESH_LPN_FRIEND_REQUEST_RETRY_COUNT 5
#endif

/** The minimum interval between two individual consecutive polls.  */
#ifndef MESH_LPN_POLL_SEPARATION_INTERVAL_MS
#define MESH_LPN_POLL_SEPARATION_INTERVAL_MS 50
#endif


/** Parameters of the Friend node Criteria field. */
typedef struct
{
    /** Weight factor that the Friend node will apply to the received RSSI. See @ref
     * mesh_friendship_rssi_factor_t. */
    uint8_t rssi_factor : 2;
    /** Weight factor that the Friend node will apply to the offered Receive Window. See @ref
     * mesh_friendship_receive_window_factor_t. */
    uint8_t receive_window_factor : 2;
    /** Minimum Friend Queue size. See @ref mesh_friendship_min_friend_queue_size_t. */
    uint8_t friend_queue_size_min_log : 3;
} friend_criteria_t;

/** Friend Request parameters. */
typedef struct
{
    /** Requested receive delay (in milliseconds). */
    uint32_t receive_delay_ms;
    /**
     * Poll timeout (in milliseconds).
     *
     * The valid range is provided by @ref MESH_LPN_POLL_TIMEOUT_MIN_MS and @ref
     * MESH_LPN_POLL_TIMEOUT_MAX_MS. The poll timeout will be rounded up to the nearest 100 ms.
     *
     * @note The poll timeout sets the upper limit for two subsequent Friend Polls. However, the
     * actual interval between each emptying of the Friend Queue (the `poll_interval_ms`) must be
     * lower. Therefore, the `poll_interval_ms` is by default set to the largest value in accordance
     * with limits in mesh_lpn_poll_interval_set().
     */
    uint32_t poll_timeout_ms;
    /**
     * Criteria that a Friend node must support to participate in the friendship negotiation.
     *
     * The received RSSI and Receive Window factors are used by the Friend node to calculate its
     * local delay:
     * @verbatim LocalDelay = ReceiveWindowFactor * ReceiveWindow - RSSIFactor * ReceivedRSS @endverbatim
     *
     * The local delay determines how long the Friend node will wait before sending its Friend
     * Offer.
     */
    friend_criteria_t friend_criteria;
} mesh_lpn_friend_request_t;

/** Initialize the Low Power node. */
void mesh_lpn_init(void);

/**
 * Initiate the friendship establishment procedure.
 *
 * Calling this API can generate the following events:
 * - @ref NRF_MESH_EVT_LPN_FRIEND_OFFER
 * - @ref NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT
 *
 * @note This API should not be called within the handler for the event
 * @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED. Doing this will cause
 * @c NRF_ERROR_INVALID_STATE to be returned, and the friendship establishment procedure
 * will not be initiated.
 *
 * @param[in] friend_params      Friend request parameters. See @ref mesh_lpn_friend_request_t for
 *                               documentation of the individual parameters.
 * @param[in] request_timeout_ms The duration to scan for incoming Friend Offers. After this
 *                               duration, the LPN will stop scanning and the event @ref
 *                               NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT will be generated.
 *                               The timeout must be greater than @ref
 *                               MESH_LPN_FRIEND_REQUEST_TIMEOUT_MIN_MS and less or equal to
 *                               @ref MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS.
 *
 *
 * @retval NRF_SUCCESS             Successfully initiated the friendship establishment procedure.
 * @retval NRF_ERROR_INVALID_STATE Already in an active friendship.
 * @retval NRF_ERROR_INVALID_PARAM Friend request parameters outside of valid ranges.
 */
uint32_t mesh_lpn_friend_request(mesh_lpn_friend_request_t friend_params,
                                 uint32_t request_timeout_ms);

/**
 * Accept a Friend Offer.
 *
 * Calling this API can generate the following events:
 * - @ref NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED
 * - @ref NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE
 *
 * @note This API is called as a response to the event @ref NRF_MESH_EVT_LPN_FRIEND_OFFER. In case
 *       of a successful return, the friendship is not established until the event @ref
 *       NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED is received.
 *
 * @note Calling the API is valid from the time of the mesh_lpn_friend_request() call until the
 *       event @ref NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT is received. The API can be called while
 *       handling the event. This allows the LPN to wait for multiple Friend Requests before
 *       deciding which Friend node to choose.
 *
 * @param[in] p_friend_offer Friend Offer to be accepted, as provided by @ref NRF_MESH_EVT_LPN_FRIEND_OFFER.
 *
 * @retval NRF_SUCCESS             Successfully accepted the Friend Offer.
 * @retval NRF_ERROR_NULL          The @c p_friend_offer parameter was NULL.
 * @retval NRF_ERROR_INVALID_STATE Not in a valid state to accept a Friend Offer.
 * @retval NRF_ERROR_INVALID_PARAM Invalid parameter values in the Friend Offer.
 */
uint32_t mesh_lpn_friend_accept(const nrf_mesh_evt_lpn_friend_offer_t * p_friend_offer);

/**
 * Initiate a Friend Poll.
 *
 * Calling this API can generate the following events:
 * - @ref NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE
 *
 * @param[in] delay_ms Number of milliseconds until the Friend Poll is to be sent out.
 *
 * @retval NRF_SUCCESS             Successfully initiated the Friend Poll procedure.
 * @retval NRF_ERROR_INVALID_PARAM The `delay_ms` cannot exceed the poll timeout. If it does, the
 *                                 friendship will be terminated.
 * @retval NRF_ERROR_INVALID_STATE Not in an active friendship.
 */
uint32_t mesh_lpn_friend_poll(uint32_t delay_ms);

/**
 * Set the poll interval.
 *
 * The poll interval is the interval between two separate Friend Poll actions. This is the interval
 * between each time the node wakes up to empty the Friend Queue. The interval must be set in a way
 * that the LPN is able to complete as many retry attempts as possible before the poll timeout
 * expires. This means that it must satisfy the following equation:
 *
 * @code
 * poll_interval_ms < (mesh_lpn_friend_request_t.poll_timeout_ms -
 *                     (mesh_lpn_friend_request_t.receive_delay_ms +
 *                      nrf_mesh_evt_lpn_friend_offer_t.offer.receive_window_ms) * (MESH_LPN_POLL_RETRY_COUNT + 1))
 * @endcode
 *
 * @note A low poll_timeout_ms and long receive_delay_ms and receive_window_ms can make it
 *       impossible to reach the MESH_LPN_POLL_RETRY_COUNT.
 *
 * @note The new poll interval will change after the next poll. To poll immediately, use the API
 *       mesh_lpn_friend_poll().
 *
 * @retval NRF_SUCCESS             Successfully set the new poll interval.
 * @retval NRF_ERROR_INVALID_PARAM The provided `poll_interval_ms` is out of range for the current poll
 *                                 timeout, receive delay, and Receive Window.
 * @retval NRF_ERROR_INVALID_STATE Not in an active friendship.
 */
uint32_t mesh_lpn_poll_interval_set(uint32_t poll_interval_ms);

/**
 * Terminate the active friendship.
 *
 * Calling this API can generate the following events:
 * - @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED
 *
 * @note The friendship is not terminated until the event @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED
 *       is received. After friendship termination, the scanner is switched off.
 *
 * @retval NRF_SUCCESS             Successfully started the friendship termination.
 * @retval NRF_ERROR_INVALID_STATE Not in an active friendship.
 */
uint32_t mesh_lpn_friendship_terminate(void);

/**
 * Get the state of the LPN.
 * @returns @c true if the friendship is active.
 */
bool mesh_lpn_is_in_friendship(void);

/** @} end of MESH_LPN */

#endif /* MESH_LPN_H__ */
