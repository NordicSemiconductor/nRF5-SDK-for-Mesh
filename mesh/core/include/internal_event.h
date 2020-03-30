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

#ifndef INTERNAL_EVENT_H__
#define INTERNAL_EVENT_H__

#include "nrf_mesh_config_core.h"
#include "nrf_mesh_assert.h"

#include <nrf_error.h>
#include "log.h"
#include "utils.h"

/**
 * @defgroup INTERNAL_EVT Internal event module
 * @ingroup NRF_MESH
 * Module for reporting internal stack events, to help traceability and debugging.
 * @{
 */

/** Internal event types. */
typedef enum
{
    INTERNAL_EVENT_DECRYPT_APP,           /**< @todo Not used. */
    INTERNAL_EVENT_DECRYPT_TRS,           /**< Transport layer decrypted packet. */
    INTERNAL_EVENT_DECRYPT_TRS_SEG,       /**< Transport layer decrypted packet segment. */
    INTERNAL_EVENT_PACKET_DROPPED,        /**< Packet dropped with reason @ref internal_event_packet_dropped_t. */
    INTERNAL_EVENT_PACKET_RELAYED,        /**< Packet was relayed by the network layer. */
    INTERNAL_EVENT_NET_PACKET_QUEUED_TX,  /**< Network queued packet in bearer for TX. */
    INTERNAL_EVENT_TRS_ACK_RECEIVED,      /**< ACK for transport SAR segment received. */
    INTERNAL_EVENT_ACK_QUEUED,            /**< ACK for transport SAR queued in bearer. */
    INTERNAL_EVENT_SAR_CANCELLED,         /**< SAR transaction cancelled. */
    INTERNAL_EVENT_FM_ACTION,             /**< Flash Manager Action Completed. */
    INTERNAL_EVENT_FM_DEFRAG,             /**< Flash Manager Defrag Completed. */
    INTERNAL_EVENT_SAR_SUCCESS,           /**< SAR transaction cancelled. */
    INTERNAL_EVENT_NET_PACKET_RECEIVED,   /**< Network layer packet data used for PTS. */
    INTERNAL_EVENT_GATT_PROV_PDU_IGNORED, /**< Invalid provisioning PDU was ignored. */
    INTERNAL_EVENT_FRIEND_PACKET_QUEUED,  /**< Packet to LPN is queued on Friend node. */
    INTERNAL_EVENT_NET_BEACON_TX,         /**< Network beacon is transmitted. */

    /** @internal Largest number in the enum. */
    INTERNAL_EVENT__LAST
} internal_event_type_t;

/** Internal event packet dropped reasons. */
typedef enum
{
    PACKET_DROPPED_INVALID_APPKEY,         /**< Dropped due to invalid application key. */
    PACKET_DROPPED_INVALID_NETKEY,         /**< Dropped due to invalid network key. */
    PACKET_DROPPED_INVALID_DEVKEY,         /**< Dropped due to invalid device key. */
    PACKET_DROPPED_INVALID_ADDRESS,        /**< Dropped due to invalid address. */
    PACKET_DROPPED_UNKNOWN_ADDRESS,        /**< Dropped due to unknown address. */
    PACKET_DROPPED_REPLAY_CACHE,           /**< Dropped due to packet already in replay cache. */
    PACKET_DROPPED_NETWORK_CACHE,          /**< Dropped due to packet already in network cache. */
    PACKET_DROPPED_NETWORK_DECRYPT_FAILED, /**< Dropped due to network decrypt failed. */
    PACKET_DROPPED_INVALID_ADTYPE,         /**< Dropped due to invalid AD type. */
    PACKET_DROPPED_INVALID_PACKET_LEN,     /**< Dropped due to invalid packet length. */
    PACKET_DROPPED_NO_MEM                  /**< Dropped due to no more memory. */
} internal_event_packet_dropped_t;

/** Internal event structure. */
typedef struct
{
    /** Type of internal event. */
    internal_event_type_t type;

    /** Reason why a packet was dropped. */
    union
    {
        /** Packet dropped reason. */
        internal_event_packet_dropped_t reason;
        /** Raw value of the packet dropped reason (used for the @ref serial_evt_device_internal_event_t). */
        uint8_t value;
    } state;
    /** Size of the packet dropped. */
    uint8_t packet_size;
    /** Packet pointer. */
    uint8_t * p_packet;
} internal_event_t;

/**
 * Callback function for inline handling of internal events.
 *
 * @warning Events may be generated in @c STACK_LOW context and may be highly timing sensitive.
 *
 * @param[in,out] p_event Internal event.
 *
 * @returns NRF_SUCCESS Successfully handled event.
 */
typedef uint32_t (*internal_event_report_cb_t) (internal_event_t * p_event);

/**
 * Initializes the internal event module.
 *
 * @param[in]  report_cb    A callback function for reporting internal events.
 */
void internal_event_init(internal_event_report_cb_t report_cb);

/**
 * Pushes an internal event to the callback function provided at the initialization.
 *
 * @param[in] p_event Pointer to internal event structure.
 *
 * @retval NRF_SUCCESS         Successfully pushed event to the callback function.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 */
uint32_t internal_event_push(internal_event_t * p_event);

/**
 * Pushes an internal event to the callback function provided at the initialization.
 *
 * @warning The macro will not return any error codes as a result of the callback
 * call.
 *
 * @param[in] EVENT_TYPE  Type of internal event.
 * @param[in] ADDATA      Context data for event. May be NULL.
 * @param[in] PACKET_SIZE Size of packet (optional)
 * @param[in] P_PACKET    Pointer to packet.
 */
#if INTERNAL_EVT_ENABLE
#define __INTERNAL_EVENT_PUSH(EVENT_TYPE, ADDATA, PACKET_SIZE, P_PACKET)   \
    do {                                                                   \
        internal_event_t EVT =                                             \
            {                                                              \
                .type = (EVENT_TYPE),                                      \
                .state.value = (ADDATA),                                   \
                .packet_size = (PACKET_SIZE),                              \
                .p_packet = (uint8_t *) (P_PACKET)                         \
            };                                                             \
        uint32_t RESULT = internal_event_push(&EVT);                       \
        if (RESULT != NRF_SUCCESS)                                         \
        {                                                                  \
            __LOG(LOG_SRC_INTERNAL, LOG_LEVEL_WARN, "Unable to push the internal event to the callback function [er%d]", RESULT); \
        }                                                               \
    } while (0)
#else
#define __INTERNAL_EVENT_PUSH(...)

#endif  /* defined(INTERNAL_EVT_ENABLE) */
/** @} */
#endif  /* INTERNAL_EVENT_H__ */
