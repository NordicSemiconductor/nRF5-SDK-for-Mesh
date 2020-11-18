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

#ifndef NRF_MESH_CONFIG_CORE_H__
#define NRF_MESH_CONFIG_CORE_H__

#include "nrf_mesh_defines.h"
#ifdef CONFIG_APP_IN_CORE
#include "nrf_mesh_config_app.h"
#endif

/**
 * @defgroup NRF_MESH_CONFIG_CORE Compile time configuration
 * Configuration of the compilation of the core mesh modules.
 * @ingroup CORE_CONFIG
 * @{
 */

/**
 * @defgroup MESH_CONFIG_GENERAL General configuration
 * @{
 */


/**
 * Enable persistent storage.
 */
#ifndef PERSISTENT_STORAGE
#define PERSISTENT_STORAGE 1
#endif

/**
 * Define to "1" if the uECC libray is linked to the mesh stack.
 */
#ifndef NRF_MESH_UECC_ENABLE
#define NRF_MESH_UECC_ENABLE 1
#endif

/**
 * Switch on the time slotted flash manager as the back end subsystem.
 */
#define FLASH_MANAGER_BACKEND

/** @} end of MESH_CONFIG_GENERAL */


/**
 * @defgroup MESH_CONFIG_ENC Encryption configuration
 * @{
 */

/**
 * Use the hardware AES-ECB block.
 *
 * @warning The S110 SoftDevice protects this hardware peripheral, but does not
 *          use it when we are in a timeslot. If there is not enough time to
 *          finish the AES-ECB operation before our timeslot ends, the module
 *          will use sd_ecb_block_encrypt().
 */
#ifndef AES_USE_HARDWARE
#define AES_USE_HARDWARE 1
#endif

/** @} end of MESH_CONFIG_ENC */

/**
 * @defgroup MESH_CONFIG_CORE_TX Core TX configuration
 * @{
 */

/** Core mesh originator queue buffer size */
#ifndef CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR
#define CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR 256
#endif

/** Core mesh relay queue buffer size */
#ifndef CORE_TX_QUEUE_BUFFER_SIZE_RELAY
#define CORE_TX_QUEUE_BUFFER_SIZE_RELAY 128
#endif

/** Core mesh instaburst originator queue buffer size */
#ifndef CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR
#define CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR 4096
#endif

/** Core mesh instaburst relay queue buffer size */
#ifndef CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY
#define CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY 2048
#endif

/** Core mesh instaburst array */
#ifndef CORE_TX_INSTABURST_CHANNELS
#define CORE_TX_INSTABURST_CHANNELS                                                                \
    {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18,                   \
      19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36}
#endif

/** Default number of repeated transmissions of one original mesh packet. */
#ifndef CORE_TX_REPEAT_ORIGINATOR_DEFAULT
#define CORE_TX_REPEAT_ORIGINATOR_DEFAULT 1
#endif

/** Default number of repeated transmissions of one relayed mesh packet. */
#ifndef CORE_TX_REPEAT_RELAY_DEFAULT
#define CORE_TX_REPEAT_RELAY_DEFAULT 1
#endif

/** Relay feature */
#ifndef MESH_FEATURE_RELAY_ENABLED
#define MESH_FEATURE_RELAY_ENABLED (1)
#endif

/** @} end of MESH_CONFIG_CORE_TX */

/**
 * @defgroup MESH_CONFIG_CCM AES-CCM module configuration
 * @{
 */

/** Enable debug printing from CCM module  */
#ifndef CCM_DEBUG_MODE_ENABLED
#define CCM_DEBUG_MODE_ENABLED 0
#endif

/** @} end of MESH_CONFIG_CCM */

/**
 * @defgroup MESH_CONFIG_FIFO FIFO configuration
 * @{
 */

/**
 * Define to 1 to enable FIFO statistics tracking.
 * This adds additional performance counters to FIFO instances.
 */
#ifndef FIFO_STATS
#define FIFO_STATS 0
#endif

/** @} */

/**
 * @defgroup MESH_CONFIG_INTERNAL_EVENTS Internal event logging configuration
 * @{
 */

/** Define "1" to enable internal event logging.
 *
 * @warning This will be forced to 0 if the build type (@c CMAKE_BUILD_TYPE)
 * is not set to @c Debug or @c RelWithDebInfo when configuring CMake.
 */
#ifndef INTERNAL_EVT_ENABLE
#define INTERNAL_EVT_ENABLE 0
#endif

/** Internal event buffer size. */
#ifndef INTERNAL_EVENT_BUFFER_SIZE
#define INTERNAL_EVENT_BUFFER_SIZE 32
#endif

/** @} end of MESH_CONFIG_INTERNAL */

/**
 * @defgroup MESH_CONFIG_LOG Log module configuration
 * @{
 */

/** Enable logging module. */
#ifndef NRF_MESH_LOG_ENABLE
#define NRF_MESH_LOG_ENABLE 1
#endif

/** Default log level. Messages with lower criticality is filtered. */
#ifndef LOG_LEVEL_DEFAULT
#define LOG_LEVEL_DEFAULT LOG_LEVEL_WARN
#endif

/** Default log mask. Messages with other sources are filtered. */
#ifndef LOG_MSK_DEFAULT
#define LOG_MSK_DEFAULT LOG_GROUP_STACK
#endif

/** Enable logging with RTT callback. */
#ifndef LOG_ENABLE_RTT
#define LOG_ENABLE_RTT 1
#endif

/** The default callback function to use. */
#ifndef LOG_CALLBACK_DEFAULT
#if defined(NRF51) || defined(NRF52_SERIES)
    #define LOG_CALLBACK_DEFAULT log_callback_rtt
#else
    #define LOG_CALLBACK_DEFAULT log_callback_stdout
#endif
#endif

/** @} end of MESH_CONFIG_LOG */

/**
 * @defgroup MESH_CONFIG_MSG_CACHE Message cache configuration
 * @{
 */

/** Number of entries in cache.  */
#ifndef MSG_CACHE_ENTRY_COUNT
#define MSG_CACHE_ENTRY_COUNT 32
#endif

/** @} end of MESH_CONFIG_MSG_CACHE */

/**
 * @defgroup MESH_CONFIG_NETWORK Network configuration
 * @{
 */

/**
 * The sequence number value that triggers the start of an IV update procedure.
 * This value should be set so that there are enough sequence numbers left for running the IV update procedure.
 */
#ifndef NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD (NETWORK_SEQNUM_MAX / 2)
#endif

/**
 * The sequence number value that triggers the end of an IV update procedure.
 * This value should be set so that there are enough sequence numbers left for finishing any ongoing Transport SAR sessions.
 */
#ifndef NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD (NETWORK_SEQNUM_MAX - TRANSPORT_SAR_SEQNUM_DIFF_MAX)
#endif

/* Sanity check for NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD */
#if NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD > NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD
#error "The network sequence number start threshold must be lower than the end threshold."
#endif
#if NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD > (NETWORK_SEQNUM_MAX - TRANSPORT_SAR_SEQNUM_DIFF_MAX)
#error "The network sequence number IV update threshold must be low enough to fit a full SAR session."
#endif

/**
 * Number of sequence numbers between every write to flash. In case of power failure, the device
 * will resume transmissions with the first sequence number in the next block. A larger block size
 * means that the device can do fewer resets between every IV Update, while a smaller will result
 * in a reduced lifetime for the flash hardware.
 */
#ifndef NETWORK_SEQNUM_FLASH_BLOCK_SIZE
#define NETWORK_SEQNUM_FLASH_BLOCK_SIZE 8192ul
#endif

/**
 * Number of sequence numbers left before allocating the next block. Allocating a new block can
 * take at least 200ms, and the device would be blocked from sending new messages if it runs out.
 */
#ifndef NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD
#define NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD 64
#endif

/** @} end of MESH_CONFIG_NETWORK */

/**
 * @defgroup MESH_CONFIG_TRANSPORT Transport layer configuration
 * @{
 */
/**
 * Maximum number of concurrent transport SAR sessions.
 *
 * One context is reserved for RX when @ref MESH_FEATURE_LPN_ENABLED is defined.
 */
#ifndef TRANSPORT_SAR_SESSIONS_MAX
#define TRANSPORT_SAR_SESSIONS_MAX (4)
#endif

/* Number of canceled SAR RX sessions to be cached. Must be power of two. */
#ifndef TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN
#define TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN (8)
#endif

/** Default TTL value for SAR segmentation acknowledgments */
#ifndef TRANSPORT_SAR_SEGACK_TTL_DEFAULT
#define TRANSPORT_SAR_SEGACK_TTL_DEFAULT (8)
#endif

/** @} end of MESH_CONFIG_TRANSPORT */
/**
 * @defgroup MESH_CONFIG_PACMAN Packet manager configuration
 *
 * @note These configuration parameters are only relevant when the `mesh_mem_packet_mgr.c` is used
 * as the dynamic memory backend. The default is the `mesh_mem_stdlib.c` backend. Its memory pool
 * size is directly controlled by the heap size.
 *
 * @{
 */

/**
 * Set to 1 to enable debug mode for the packet manager.
 *
 * @warning In debug mode, the packet manager will print out a lot of
 * information and this _will_ cause the system to assert due to timeslot
 * violation.
 */
#ifndef PACKET_MGR_DEBUG_MODE
#define PACKET_MGR_DEBUG_MODE 0
#endif


/**
 * Size of the packet manager memory pool in bytes.
 * @warning The value of the memory pool cannot currently exceed the value of
 * PACKET_MGR_PACKET_MAXLEN, due to the current design of the memory manager.
 */
#ifndef PACKET_MGR_MEMORY_POOL_SIZE
#define PACKET_MGR_MEMORY_POOL_SIZE 4096
#endif

/**
 * Packet manager blame mode.
 *
 * The blame mode adds an additional field to the packet header which records
 * the last location a packet manager function was called from. This can be
 * useful in some debugging scenarioes, such as when a bug is suspected of being
 * caused by a reference counting error. It is not intended for regular use, so
 * do not enable unless you are debugging the packet manager usage.
 */
#ifndef PACKET_MGR_BLAME_MODE
#define PACKET_MGR_BLAME_MODE 0
#endif

/** @} end of MESH_CONFIG_PACMAN */

/**
 * @defgroup MESH_CONFIG_PACKET_BUFFER Packet buffer configuration
 * @{
 */

/**
 * Set to 1 to enable debug mode for the packet buffer.
 *
 * @warning In debug mode, the packet buffer will print out a lot
 * of information and this _will_ cause the system to assert due to
 * timeslot violation.
 */
#ifndef PACKET_BUFFER_DEBUG_MODE
#define PACKET_BUFFER_DEBUG_MODE 0
#endif

/** @} end of MESH_CONFIG_PACKET_BUFFER */

/**
 * @defgroup MESH_CONFIG_REPLAY_CACHE Replay cache configuration.
 * @{
 */

/**
 * Number of entries in the replay protection cache.
 *
 * @note The number of entries in the replay protection list directly limits the number of elements
 * a node can receive messages from on the current IV index. This means if your device has a replay
 * protection list with 40 entries, a message from a 41st unicast address (element )will be dropped
 * by the transport layer.
 *
 * @note The replay protection list size *does not* affect the node's ability to relay messages.
 *
 * @note This number is indicated in the device composition data of the node and provisioner can
 * make use of this information to prevent unwarranted filling of the replay list on a given node in
 * a mesh network.
 */
#ifndef REPLAY_CACHE_ENTRIES
#define REPLAY_CACHE_ENTRIES 40
#endif

/**
 * Strategy for storing the replay protection cache into persistent memory.
 * The replay protection cache is stored into the persistent memory
 * to avoid replay protection attacks after device is power cycled.
 *
 * For details, see the @ref power_down_replay_protection_cache "power-down documentation".
 *
 */
#ifndef REPLAY_CACHE_STORAGE_STRATEGY
#define REPLAY_CACHE_STORAGE_STRATEGY    MESH_CONFIG_STRATEGY_ON_POWER_DOWN
#endif

/** @} end of MESH_CONFIG_REPLAY_CACHE */

/**
 * @defgroup EMERGENCY_CACHE Power down emergency cache configuration.
 * @{
 */

/**
 * Number of flash pages reserved for the emergency cache.
 *
 * @note Since the emergency cache is created dynamically stack does not have a predefined list of
 * the entry descriptors. The emergency cache requires preliminary allocated flash memory
 * to store data. The size of the cache depends on potentially not stored data in a single moment of time.
 * Generally, one page is sufficient size. If more than 4k unsaved data are expected
 * at the one time please allocate more than 1 page for the emergency cache.
 * However, it is important to understand a large number of not stored data at the moment of power-down causes
 * the growing time required to go in power off.
 *
 * For details, see the @ref power_down_emergency_cache "power-down documentation".
 *
 */
#ifndef EMERGENCY_CACHE_RESERVED_PAGE_NUMBER
#define EMERGENCY_CACHE_RESERVED_PAGE_NUMBER 1
#endif

/** @} end of EMERGENCY_CACHE */

/**
 * @defgroup MESH_CONFIG_FLASH_MANAGER Flash manager configuration defines
 * @{
 */

/** Maximum number of pages that can be owned by a single flash manager */
#ifndef FLASH_MANAGER_PAGE_COUNT_MAX
#define FLASH_MANAGER_PAGE_COUNT_MAX 255
#endif

/** Size of the flash manager data pool, storing pending writes. */
#ifndef FLASH_MANAGER_POOL_SIZE
#define FLASH_MANAGER_POOL_SIZE 256
#endif

/** Maximum size of a single flash entry in bytes. */
#ifndef FLASH_MANAGER_ENTRY_MAX_SIZE
#define FLASH_MANAGER_ENTRY_MAX_SIZE 128
#endif

/** Number of flash pages to be reserved between the flash manager recovery page and the bootloader.
 *  @note This value will be ignored if FLASH_MANAGER_RECOVERY_PAGE is set.
 */
#ifndef FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES
#define FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES 0
#endif

/** @} end of MESH_CONFIG_FLASH_MANAGER */

/**
 * @defgroup MESH_CONFIG_GATT GATT configuration defines
 * @{
 */

/** GATT proxy feature. To be enabled only in combination with linking GATT proxy files. */
#ifndef MESH_FEATURE_GATT_PROXY_ENABLED
#define MESH_FEATURE_GATT_PROXY_ENABLED 0
#endif

/** Maximum number of addresses in the GATT proxy address filter, per connection. */
#ifndef MESH_GATT_PROXY_FILTER_ADDR_COUNT
#define MESH_GATT_PROXY_FILTER_ADDR_COUNT 32
#endif

/**
 * Advertisement interval for Mesh GATT Proxy Network ID advertisements.
 */
#ifndef MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS
#define MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS 2000
#endif

/**
 * Advertisement interval for Mesh GATT Proxy Node Identity advertisements.
 *
 * The Node Identity beacon is used by the Provisioner to identify a specific node with Proxy
 * support. The beacon is automatically advertised after the node has been provisioned,
 * if the node supports the Proxy feature.
 */
#ifndef MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS
#define MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS 200
#endif

/**
 * Duration of the Mesh GATT Proxy Node Identity advertisements.
 *
 * @note The duration of the Node Identity advertisements shall not be greater than 60.
 * See the requirement in @tagMeshSp section 7.2.2.2.3.
 */
#ifndef MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS
#define MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS 60000
#endif

/** Number of network beacons to cache in proxy to limit impact on GATT link bandwidth */
#ifndef MESH_GATT_PROXY_BEACON_CACHE_SIZE
#define MESH_GATT_PROXY_BEACON_CACHE_SIZE 8
#endif
/** @} end of MESH_CONFIG_GATT */

/**
 * @defgroup MESH_CONFIG_ACCESS Access layer configuration defines
 * @{
 */

/** Model publish period restore behavior
 *
 * If publish period setting is restored, model will start periodic publishing automatically if
 * it was configured to do so before power down.
 * Note: If node is configured to restore publish period settings on power-up, and if provisioner
 * malfunctions after setting up the publish period state or user looses access to the provisioner,
 * then there is no way to stop periodic publishing other than manually initiated node reset.
 */
#ifndef ACCESS_MODEL_PUBLISH_PERIOD_RESTORE
#define ACCESS_MODEL_PUBLISH_PERIOD_RESTORE 0
#endif


/** @} end of MESH_CONFIG_ACCESS */

/**
 * @defgroup MESH_CONFIG_FSM Finite State Machine configuration
 * @{
 */

/** Set to 1 to enable debug mode for the Finite State Machine. */
#ifndef FSM_DEBUG
#define FSM_DEBUG (0)
#endif

/** @} end of MESH_CONFIG_FSM */

/**
 * @defgroup MESH_CONFIG_FRIENDSHIP Friendship configuration defines
 * @{
 */

/** LPN feature */
#ifndef MESH_FEATURE_LPN_ENABLED
#define MESH_FEATURE_LPN_ENABLED 0
#endif

/**
 * Allow a node with the LPN feature to participate in a mesh network
 * as a regular node when the friendship is not established.
 *
 * @note When this option is used, the scanner and advertising network beacons
 * are enabled, and the power consumption of the node is increased
 * until a friendship is established.
 */
#ifndef MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
#define MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP 0
#endif

/** Friend feature. */
#ifndef MESH_FEATURE_FRIEND_ENABLED
#define MESH_FEATURE_FRIEND_ENABLED 0
#endif

/** Number of friendships supported simultaneously. */
#ifndef MESH_FRIEND_FRIENDSHIP_COUNT
#define MESH_FRIEND_FRIENDSHIP_COUNT 2
#endif

/** Size of the Friend Subscription List (per friendship). */
#ifndef MESH_FRIEND_SUBLIST_SIZE
#define MESH_FRIEND_SUBLIST_SIZE 16
#endif

/** Size of the Friend Queue (per friendship).
 *
 * @note Set the queue size to a value higher than 32 to be able to receive at least one
 * of the longest segmented messages and one or more of the unsegmented messages. */
#ifndef MESH_FRIEND_QUEUE_SIZE
#define MESH_FRIEND_QUEUE_SIZE 35
#endif

/** @} end of MESH_CONFIG_FRIENDSHIP */

/** @} end of NRF_MESH_CONFIG_CORE */

#endif
