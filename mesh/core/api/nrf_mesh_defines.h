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

#ifndef NRF_MESH_DEFINES_H__
#define NRF_MESH_DEFINES_H__

/**
 * @defgroup NRF_MESH_DEFINES Defines
 * @ingroup NRF_MESH
 * Core mesh definitions
 *
 * The values in this header file are considered immutable.
 * @{
 */

/* Check platform defines */
#if defined(NRF51)
    #if defined(NRF52_SERIES)
        #error "Both NRF51 and NRF52_SERIES should not be defined."
    #endif
#elif !defined(NRF52_SERIES) && !defined(HOST)
    #error "At least NRF51, NRF52_SERIES or HOST must be defined."
#endif

/**
 * @defgroup MESH_DEFINES_API API level definitions
 * @{
 */

/** Maximum possible segmented payload size (octets). */
#define NRF_MESH_SEG_PAYLOAD_SIZE_MAX (380)

/** Maximum possible upper transport PDU (payload + MIC) size (octets). */
#define NRF_MESH_UPPER_TRANSPORT_PDU_SIZE_MAX (384)

/** Maximum useful access unsegmented payload size (octets). */
#define NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX (11)

/** All advertisement channels */
#define NRF_MESH_ADV_CHAN_ALL {37, 38, 39}

/** Default advertisement channel map (37, 38 and 39). */
#define NRF_MESH_ADV_CHAN_DEFAULT NRF_MESH_ADV_CHAN_ALL

/** Size (in octets) of an encryption key.*/
#define NRF_MESH_KEY_SIZE  (16)

/** Size (in octets) of the network ID. */
#define NRF_MESH_NETID_SIZE (8)

/** Size (in octets) of a UUID. */
#define NRF_MESH_UUID_SIZE (16)

/** Number of bits available for the TTL field. */
#define NRF_MESH_TTL_BIT_COUNT (7)

/** Maximum TTL value. */
#define NRF_MESH_TTL_MAX ((1 << NRF_MESH_TTL_BIT_COUNT) - 1)

/** Maximum global key index allowed, according to @tagMeshSp section 3.8.6.4. */
#define NRF_MESH_GLOBAL_KEY_INDEX_MAX  (0xFFF)

/** Interval for secure network broadcast beacons, in seconds. */
#define NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS 10

/** Number of periods to keep beacon observations of. */
#define NRF_MESH_BEACON_OBSERVATION_PERIODS 2

/** Size (in octets) of the unprovisioned beacon URI hash. */
#define NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE    (4)

/** Size of an ECDH public key. */
#define NRF_MESH_ECDH_PUBLIC_KEY_SIZE    (64)

/** Size of an ECDH private key. */
#define NRF_MESH_ECDH_PRIVATE_KEY_SIZE   (32)

/** Size of an ECDH shared secret. */
#define NRF_MESH_ECDH_SHARED_SECRET_SIZE    (32)

/** Unassigned address. */
#define NRF_MESH_ADDR_UNASSIGNED                  (0x0000)

/** All-proxies fixed group address. */
#define NRF_MESH_ALL_PROXIES_ADDR                 (0xFFFC)

/** All-friends fixed group address. */
#define NRF_MESH_ALL_FRIENDS_ADDR                 (0xFFFD)

/** All-relays fixed group address. */
#define NRF_MESH_ALL_RELAYS_ADDR                  (0xFFFE)

/** All-nodes fixed group address. */
#define NRF_MESH_ALL_NODES_ADDR                   (0xFFFF)

/** Offset of bits determining the address type. */
#define NRF_MESH_ADDR_TYPE_BITS_OFFSET            (14)

/** Mask of bits determining the address type. */
#define NRF_MESH_ADDR_TYPE_BITS_MASK              (0xC000)

/** IRQ priority value if running in thread mode. */
#if defined(NRF51)
#define NRF_MESH_IRQ_PRIORITY_THREAD              (4)
#else
#define NRF_MESH_IRQ_PRIORITY_THREAD              (15)
#endif

/** Lowest available IRQ priority on current architecture. */
#if defined(NRF51)
#define NRF_MESH_IRQ_PRIORITY_LOWEST              (3)
#else
#define NRF_MESH_IRQ_PRIORITY_LOWEST              (6)
#endif

/** @} end of MESH_DEFINES_API */

/**
 * @defgroup MESH_DEFINES_NETWORK Network layer definitions
 * @{
 */

/** Number of bits in the sequence number. */
#define NETWORK_SEQNUM_BITS         24

/** Maximum allowed sequence number. */
#define NETWORK_SEQNUM_MAX          ((1 << NETWORK_SEQNUM_BITS) - 1)

/** Maximum allowed number of retransmissions for relayed packets. */
#define NETWORK_RELAY_RETRANSMITS_MAX   ((1 << 3) - 1)

/** Maximum allowed number of 10 ms steps for the interval between relayed packet retranmissions. */
#define NETWORK_RELAY_INTERVAL_STEPS_MAX ((1 << 5) - 1)

/** Maximum allowed relay interval in milliseconds. */
#define NETWORK_RELAY_INTERVAL_MAX_MS ((NETWORK_RELAY_INTERVAL_STEPS_MAX + 1) * 10)

/** The minimum time between IV updates, in minutes. */
#define NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES (96 * 60)

/** Limit for how much larger IV indices in incoming beacons can be than the current node before recovery is impossible. */
#define NETWORK_IV_RECOVERY_LIMIT 42

/** Which bearer to use when transmitting packets. */
#define NETWORK_BEARER BEARER_ADV_RADIO

/** @} end of MESH_DEFINES_NETWORK*/

/**
 * @defgroup MESH_DEFINES_TRANSPORT Transport layer definitions
 * @{
 */

/** RX timeout lower limit. */
#define TRANSPORT_SAR_RX_TIMEOUT_MIN                            MS_TO_US(10000)
/** RX timeout upper limit. */
#define TRANSPORT_SAR_RX_TIMEOUT_MAX                            MS_TO_US(120000)

/** Base RX acknowledgement timeout lower limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_BASE_MIN                   MS_TO_US(150)
/** Base RX acknowledgement timeout upper limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_BASE_MAX                   MS_TO_US(10000)

/** RX acknowledgement timeout per hop addition lower limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION_MIN       MS_TO_US(50)
/** RX acknowledgement timeout per hop addition upper limit. */
#define TRANSPORT_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION_MAX       MS_TO_US(10000)

/** Base TX retry timeout lower limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_BASE_MIN                 MS_TO_US(200)
/** Base TX retry timeout upper limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_BASE_MAX                 MS_TO_US(10000)

/** TX retry timeout per hop addition lower limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION_MIN     MS_TO_US(200)
/** TX retry timeout per hop addition upper limit. */
#define TRANSPORT_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION_MAX     MS_TO_US(10000)

/** TX timeout lower limit. */
#define TRANSPORT_SAR_TX_TIMEOUT_MIN                            MS_TO_US(150)
/** TX timeout upper limit. */
#define TRANSPORT_SAR_TX_TIMEOUT_MAX                            MS_TO_US(60000)

/** TX retries lower limit (@tagMeshSp section 3.5.3.3) */
#define TRANSPORT_SAR_TX_RETRIES_MIN                            (2)
/** TX retries upper limit (UINT8_MAX). */
#define TRANSPORT_SAR_TX_RETRIES_MAX                            (255)

/** Maximum difference in sequence numbers between two SAR segments of the same session
 * (@tagMeshSp section 3.5.3.1) */
#define TRANSPORT_SAR_SEQNUM_DIFF_MAX                           (8191)

/** @} end of MESH_DEFINES_TRANSPORT */

/**
 * @defgroup MESH_DEFINES_HEARTBEAT Heartbeat definitions
 * @{
 */

/** Heartbeat trigger on Relay state change */
#define HEARTBEAT_TRIGGER_TYPE_RELAY    0x0001
/** Heartbeat trigger on Proxy state change */
#define HEARTBEAT_TRIGGER_TYPE_PROXY    0x0002
/** Heartbeat trigger on Friend state change */
#define HEARTBEAT_TRIGGER_TYPE_FRIEND   0x0004
/** Heartbeat trigger on LPN state change */
#define HEARTBEAT_TRIGGER_TYPE_LPN      0x0008
/** Heartbeat features valid bit mask */
#define HEARTBEAT_TRIGGER_TYPE_RFU_MASK 0x000F

/** @} end of MESH_DEFINES_HEARTBEAT */


/** @} */
#endif  /* NRF_MESH_DEFINES_H__ */
