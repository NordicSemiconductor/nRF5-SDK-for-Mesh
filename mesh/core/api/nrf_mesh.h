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

#ifndef NRF_MESH_H__
#define NRF_MESH_H__

#include <stdint.h>
#include <stdbool.h>

#include "timer.h"

#include "ble.h"
#include "ble_gap.h"

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"

#include "nrf.h"
#include "nrf_sdm.h"

/**
 * @defgroup NRF_MESH Core Mesh API
 * @ingroup MESH_API_GROUP_CORE
 * Interface for the message sending part of the mesh stack.
 * @{
 */

/** Initial value of tokens. */
#define NRF_MESH_INITIAL_TOKEN          0x00000000ul
/** The upper border of the token values which are used for general communication. */
#define NRF_MESH_SERVICE_BORDER_TOKEN   0xF0000000ul
/** Reserved token values. */

/**
 * Beginning of the reserved friendship token range.
 *
 * Used by the Core TX Friend to resolve the Friend Bearer.
 */
#define NRF_MESH_FRIEND_TOKEN_BEGIN     0xFFFFFE00ul
/** End of the reserved friendship token range. */
#define NRF_MESH_FRIEND_TOKEN_END       0xFFFFFEFFul

#define NRF_MESH_FRIEND_POLL_TOKEN      0xFFFFFFF8ul
#define NRF_MESH_FRIEND_REQUEST_TOKEN   0xFFFFFFF9ul
#define NRF_MESH_FRIEND_CLEAR_TOKEN     0xFFFFFFFAul
#define NRF_MESH_SUBMAN_ADD_TOKEN       0xFFFFFFFBul
#define NRF_MESH_SUBMAN_REMOVE_TOKEN    0xFFFFFFFCul
#define NRF_MESH_HEARTBEAT_TOKEN        0xFFFFFFFDul
#define NRF_MESH_SAR_TOKEN              0xFFFFFFFEul
#define NRF_MESH_RELAY_TOKEN            0xFFFFFFFFul

/**
 * @defgroup MESH_CORE_COMMON_TYPES Types
 * Core mesh type definitions
 * @{
 */

/**
 * Mesh assertion handler type.
 *
 * When an unexpected, fatal error occurs within the Mesh stack, it will call the Mesh
 * assertion handler callback. The Mesh stack will be in an undefined state when
 * this happens and the only way to recover will be to perform a reset, e.g. by
 * using CMSIS NVIC_SystemReset().
 *
 * @param[in] pc            The program counter of the failed assertion.
 */
typedef void (*nrf_mesh_assertion_handler_t)(uint32_t pc);

/** TX Token type, used as a context parameter to notify the application of ended transmissions. */
typedef uint32_t nrf_mesh_tx_token_t;

/** RX packet type. */
typedef enum
{
    NRF_MESH_RX_SOURCE_SCANNER, /**< The packet came from the scanner. */
    NRF_MESH_RX_SOURCE_GATT,    /**< The packet came from a GATT connection. */
    NRF_MESH_RX_SOURCE_INSTABURST,  /**< The packet came from an Instaburst event. */
    NRF_MESH_RX_SOURCE_LOOPBACK,  /**< The packet came from this device. */
} nrf_mesh_rx_source_t;

/** Metadata structure for packets received with the scanner */
typedef struct
{
    timestamp_t timestamp; /**< Device local timestamp of the start of the advertisement header of the packet in microseconds. */
    uint32_t access_addr; /**< Access address the packet was received on. */
    uint8_t  channel; /**< Channel the packet was received on. */
    int8_t   rssi; /**< RSSI value of the received packet. */
    ble_gap_addr_t adv_addr; /**< Advertisement address in the packet. */
    uint8_t adv_type;  /**< BLE GAP advertising type. */
} nrf_mesh_rx_metadata_scanner_t;

/** Event ID for an extended advertising event with Instaburst. */
typedef struct
{
    uint16_t data_id; /**< ID for this particular event */
    uint8_t set_id;  /**< ID for the set this event belongs in. */
} nrf_mesh_instaburst_event_id_t;

/** Metadata structure for packets received with Instaburst */
typedef struct
{
    timestamp_t timestamp; /**< Device local timestamp of the packet preamble of the adv ext packet in microseconds. */
    uint8_t  channel; /**< Channel the packet was received on. */
    int8_t   rssi; /**< RSSI value of the received packet. */
    struct
    {
        nrf_mesh_instaburst_event_id_t id; /**< Event ID for the full advertising event. */
        uint8_t packet_index; /**< Index of the packet in the advertising event. */
        bool is_last_in_chain; /**< Whether this packet is the last packet in the advertising event. */
    } event;
} nrf_mesh_rx_metadata_instaburst_t;

typedef struct
{
    timestamp_t timestamp; /**< Device local timestamp of the packet being processed in the stack in microseconds. */
    uint16_t connection_index; /**< Proxy connection index the packet was received from. */
} nrf_mesh_rx_metadata_gatt_t;

/** Metadata structure for packets that originated on this device. */
typedef struct
{
    nrf_mesh_tx_token_t tx_token; /**< TX Token attached to the loopback packet. */
} nrf_mesh_rx_metadata_loopback_t;

/** RX packet metadata. */
typedef struct
{
    nrf_mesh_rx_source_t source; /**< Source of the received packet. */
    union
    {
        /** Scanner packet metadata */
        nrf_mesh_rx_metadata_scanner_t scanner;
        /** Instaburst packet metadata */
        nrf_mesh_rx_metadata_instaburst_t instaburst;
        /** GATT packet metadata */
        nrf_mesh_rx_metadata_gatt_t gatt;
        /** Loopback packet metadata */
        nrf_mesh_rx_metadata_loopback_t loopback;
    } params;
} nrf_mesh_rx_metadata_t;

/**
 * Callback function type for checking if a given packet should be relayed on to other nodes.
 *
 * The mesh can be initialized with this callback if the application wants to have control over
 * which packets should be relayed to other nodes. This is done by passing the function pointer
 * to @ref nrf_mesh_init via the @ref nrf_mesh_init_params_t struct.
 *
 * With the default behavior, the mesh will only relay new packets with a TTL larger than 1.
 * This behavior is always applied in addition to the callback function.
 *
 * @warning Devices claiming to be compliant with @tagMeshSp should
 *          not override the default relaying behavior.
 *
 * @see nrf_mesh_init()
 *
 * @param[in] src The packet source address.
 * @param[in] dst The packet destination address.
 * @param[in] ttl The time-to-live value for the packet (6-bits).
 *
 * @returns @c true if the packet received should be relayed on to the other mesh nodes, @c false otherwise.
 */
typedef bool (*nrf_mesh_relay_check_cb_t)(uint16_t src, uint16_t dst, uint8_t ttl);

/** Arguments structure for the RX callback function. */
typedef struct
{
    uint8_t adv_type;  /**< BLE GAP advertising type. */
    uint8_t length;    /**< Length of the advertisement packet payload. */
    const uint8_t * p_payload; /**< Pointer to the raw advertisement packet payload. */
    const nrf_mesh_rx_metadata_t * p_metadata; /**< Metadata structure for the given packet. */
} nrf_mesh_adv_packet_rx_data_t;

/**
 * Advertisement received callback function type.
 *
 * This callback can be used to receive raw advertisement packets.
 * This can be useful to listen for specific packets that are not handled by the mesh,
 * such as beacon packets or regular BLE advertisements.
 *
 * @param[in] p_rx_data Received advertisement packet data and metadata.
 */
typedef void (*nrf_mesh_rx_cb_t)(const nrf_mesh_adv_packet_rx_data_t * p_rx_data);

/**
 * Key refresh phase.
 *
 * @see dsm_subnet_update(), dsm_subnet_update_swap_keys(), dsm_subnet_update_commit()
 */
typedef enum
{
    /** Key refresh phase 0. Indicates normal device operation. */
    NRF_MESH_KEY_REFRESH_PHASE_0,
    /** Key refresh phase 1. Old keys are used for packet transmission, but new keys can be used to receive messages. */
    NRF_MESH_KEY_REFRESH_PHASE_1,
    /** Key refresh phase 2. New keys are used for packet transmission, but old keys can be used to receive messages. */
    NRF_MESH_KEY_REFRESH_PHASE_2,
    /** Key refresh phase 3. Used to complete a key refresh procedure and transition back to phase 0. */
    NRF_MESH_KEY_REFRESH_PHASE_3,
} nrf_mesh_key_refresh_phase_t;

/**
 * Application security material structure.
 *
 * This structure is required for the encryption of the application data.
 *
 * @note This is intended to be managed by the device_state_manager.c, and the setters and getters
 * in that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Indicates whether the device key or the application is used. */
    bool is_device_key;
    /** Application ID. Calculated and used internally. */
    uint8_t aid;
    /** Application or device key storage. */
    uint8_t key[NRF_MESH_KEY_SIZE];
} nrf_mesh_application_secmat_t;

/**
 * Network security material structure.
 *
 * @note This is intended to be managed by the device_state_manager.c, and the setters and getters
 * in that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Network identifier. */
    uint8_t nid;
    /** Encryption key. */
    uint8_t encryption_key[NRF_MESH_KEY_SIZE];
    /** Privacy key. */
    uint8_t privacy_key[NRF_MESH_KEY_SIZE];
} nrf_mesh_network_secmat_t;

/**
 * Security material for the Bluetooth Mesh network beacons.
 * This structure is used when sending a mesh network beacon advertisement.
 *
 * @note This is managed by the device_state_manager.c, and the setters and getters in
 * that module should be used and no direct accesses should be made to this structure.
 */
typedef struct
{
    /** Beacon key */
    uint8_t key[NRF_MESH_KEY_SIZE];
    /** Network ID */
    uint8_t net_id[NRF_MESH_NETID_SIZE];
#if MESH_FEATURE_GATT_PROXY_ENABLED
    /** Identity key used with Proxy identity beacons. */
    uint8_t identity_key[NRF_MESH_KEY_SIZE];
#endif
} nrf_mesh_beacon_secmat_t;

/**
 * Run-time transmission information for individual beacons. Used for internal
 * logic.
 */
typedef struct
{
    /** Rolling number of beacons received in each preceding period. */
    uint16_t rx_count;
    /** Counter of broadcast intervals. Used to measure the observation period. */
    uint8_t observation_count;
    /** Actual beacon interval at the moment. */
    uint16_t interval;
    /** Last transmission time for this beacon. */
    timestamp_t tx_timestamp;
} nrf_mesh_beacon_tx_info_t;

/**
 * Information structure for the Bluetooth Mesh network beacons.
 * This structure keeps track of all information related to a single mesh
 * network beacon.
 */
typedef struct
{
    /** Flag indicating whether the given structure is allowed to initiate an
     * IV update. */
    bool iv_update_permitted;
    /** Pointer to a transmission info structure. */
    nrf_mesh_beacon_tx_info_t * p_tx_info;
    /** Beacon security material. */
    nrf_mesh_beacon_secmat_t secmat;
    /** Beacon security material during key refresh. */
    nrf_mesh_beacon_secmat_t secmat_updated;
} nrf_mesh_beacon_info_t;

/**
 * Bluetooth Mesh security material structure.
 *
 * This structure is used when sending a mesh packet and it includes pointers to the network
 * and application security material structures.
 *
 * @note This can be populated by the @ref dsm_tx_secmat_get function.
 */
typedef struct
{
    /** Required for network layer encryption @see nrf_mesh_network_secmat_t.*/
    const nrf_mesh_network_secmat_t * p_net;
    /** Required for transport layer encryption @see nrf_mesh_application_secmat_t. */
    const nrf_mesh_application_secmat_t * p_app;
} nrf_mesh_secmat_t;

/** State of IV update procedure */
typedef enum
{
    /** In normal operation. */
    NET_STATE_IV_UPDATE_NORMAL,
    /** IV update procedure in progress. */
    NET_STATE_IV_UPDATE_IN_PROGRESS,
} net_state_iv_update_t;

/**
 * Bluetooth Mesh address types.
 *
 * Bluetooth Mesh defines 3 address types:
 *   - Unicast   <pre> 0xxx xxxx xxxx xxxx </pre>
 *   - Virtual   <pre> 10xx xxxx xxxx xxxx </pre>
 *   - Group     <pre> 11xx xxxx xxxx xxxx </pre>
 */
typedef enum
{
    /** Invalid address. */
    NRF_MESH_ADDRESS_TYPE_INVALID,
    /** Unicast address. */
    NRF_MESH_ADDRESS_TYPE_UNICAST,
    /** Virtual address. */
    NRF_MESH_ADDRESS_TYPE_VIRTUAL,
    /** Group address. */
    NRF_MESH_ADDRESS_TYPE_GROUP,
} nrf_mesh_address_type_t;

/**
 * Bluetooth Mesh address.
 *
 * @note When used to add RX addresses to the stack, the struct needs to be statically allocated.
 */
typedef struct
{
    /** Address type. */
    nrf_mesh_address_type_t type;
    /** Address value. */
    uint16_t value;
    /** 128-bit virtual label UUID, will be NULL if type is not @c NRF_MESH_ADDRESS_VIRTUAL . */
    const uint8_t * p_virtual_uuid;
} nrf_mesh_address_t;

/** Message MIC size selection */
typedef enum
{
    /** Selects 4 byte MIC size for the transport PDU */
    NRF_MESH_TRANSMIC_SIZE_SMALL,
    /** Selects 8 byte MIC size for the transport PDU */
    NRF_MESH_TRANSMIC_SIZE_LARGE,
    /** Selects default stack configured MIC size for the transport PDU */
    NRF_MESH_TRANSMIC_SIZE_DEFAULT,
    /** Invalid size */
    NRF_MESH_TRANSMIC_SIZE_INVALID
} nrf_mesh_transmic_size_t;

/**
 * Mesh packet transmission parameters.
 *
 * @note The message will be sent as a segmented message and reassembled on the peer side if one of
 * the following conditions are true:
 * - The length of the message is greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX.
 * - The @p force_segmented field is true.
 * - The @p transmic_size is @ref NRF_MESH_TRANSMIC_SIZE_LARGE.
 */
typedef struct
{
    /** Packet destination address. */
    nrf_mesh_address_t dst;
    /** Address of the element the packet originates from (must be a unicast address). */
    uint16_t src;
    /** Time to live value for the packet, this is a 7 bit value. */
    uint8_t ttl;
    /** See @tagMeshSp section 3.7.5.2 */
    bool force_segmented;
    /** Transport MIC Size selection */
    nrf_mesh_transmic_size_t transmic_size;
    /** Points to the payload to be sent. */
    const uint8_t * p_data;
    /** Length of the payload being sent. */
    uint16_t data_len;
    /** Required for encryption @see nrf_mesh_secmat_t. */
    nrf_mesh_secmat_t security_material;
    /** Token that can be used as a reference in the TX complete callback. */
    nrf_mesh_tx_token_t tx_token;
} nrf_mesh_tx_params_t;

/**
 * Initialization parameters structure.
 */
typedef struct
{
#if (defined(S140) || defined(S130) || defined(S132) || defined(S112) || defined(S113) || defined(HOST))
    nrf_clock_lf_cfg_t lfclksrc; /**< See nrf_sdm.h or SoftDevice documentation. */
#elif defined(S110)
    nrf_clock_lfclksrc_t lfclksrc; /**< See nrf_sdm.h or SoftDevice documentation. */
#else
    #error "Unknown target softdevice version"
#endif
    nrf_mesh_relay_check_cb_t relay_cb; /**< Application call back for relay decisions, can be NULL. */
    uint8_t irq_priority; /**< Application IRQ priority (NRF_MESH_IRQ_PRIORITY_LOWEST or NRF_MESH_IRQ_PRIORITY_THREAD). */
    const uint8_t * p_uuid; /** UUID to be used for unprovisioned node beacons. If NULL, UUID will be auto generated */
} nrf_mesh_init_params_t;

/** @} end of MESH_CORE_COMMON_TYPES */

/**
 * Initializes the Bluetooth Mesh stack.
 *
 * @note The Nordic Semiconductor SoftDevice must be initialized by the
 *       application before this function is called.
 * @note Calling this function only initializes the Mesh stack. To start transmitting
 *       and receiving messages, @ref nrf_mesh_enable() must also be called. In addition,
 *       network and application keys must be added for the device to participate in a
 *       mesh network.
 * @note The Mesh is initialized with default parameters for the radio. To change
 *       these, use the options API, @ref nrf_mesh_opt_set().
 *
 * @warning Enabling _any_ proprietary extensions will break Bluetooth Mesh
 *          compatibility.
 *
 * @see nrf_mesh_enable(), nrf_mesh_opt_set()
 *
 * @param[in] p_init_params  Pointer to initialization parameter structure.
 *
 * @retval NRF_SUCCESS                      The mesh system was successfully initialized.
 * @retval NRF_ERROR_SOFTDEVICE_NOT_ENABLED The SoftDevice has not been enabled.
 * @retval NRF_ERROR_INVALID_STATE          The mesh stack has already been initialized.
 * @retval NRF_ERROR_NULL                   The @c p_init_params parameter was @c NULL.
 */
uint32_t nrf_mesh_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Enables the Mesh.
 *
 * @note Calling this function alone will not generate any events unless:
 *         - Network and application keys have been added.
 *         - At least one RX address has been added.
 *
 * @see nrf_mesh_rx_addr_add()
 *
 * @retval NRF_SUCCESS             The Mesh was started successfully.
 * @retval NRF_ERROR_INVALID_STATE The mesh was not initialized,
 *                                 see @ref nrf_mesh_init().
 */
uint32_t nrf_mesh_enable(void);

/**
 * Starts disabling the Mesh.
 *
 * Calling this function will stop the Mesh, i.e, it will stop ordering
 * time slots from the SoftDevice and will not generate events.
 *
 * The mesh will produce an @ref NRF_MESH_EVT_DISABLED once it has been fully disabled.
 *
 * @warning The mesh should be fully disabled before the Softdevice is disabled. If the application
 * uses the Softdevice Handler module from the nRF5 SDK, this will be handled automatically.
 * Otherwise, the application should wait for @ref NRF_MESH_EVT_DISABLED before disabling the
 * Softdevice.
 *
 * @retval NRF_SUCCESS The Mesh was stopped successfully.
 * @retval NRF_ERROR_INVALID_STATE The mesh was already disabled.
 */
uint32_t nrf_mesh_disable(void);

/**
 * Queues a mesh packet for transmission.
 *
 * @note    Calling this function will give an @ref NRF_MESH_EVT_TX_COMPLETE event
 *          when the packet has been sent on air. The parameter given with the
 *          event is the same reference as returned in p_packet_reference.
 * @note    If the length of the message is greater than @ref
 *          NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX, the message will be sent as a
 *          segmented message and reassembled on the peer side.
 *
 * @param[in]  p_params            Pointer to a structure containing the
 *                                 parameters for the message to send.
 * @param[out] p_packet_reference  Pointer to store a reference to the packet
 *                                 queued for transmission.
 *                                 This parameter may be set to @c NULL for
 *                                 ignoring the reference.
 *
 * @retval NRF_SUCCESS             The message was successfully queued for transmission.
 * @retval NRF_ERROR_NO_MEM        A packet buffer could not be allocated for the packet. The application
 *                                 should try to send the packet again at a later point.
 * @retval NRF_ERROR_INVALID_ADDR  The source address is not a unicast address, or the destination
 *                                 is invalid.
 * @retval NRF_ERROR_INVALID_PARAM TTL was larger than NRF_MESH_TTL_MAX.
 * @retval NRF_ERROR_NULL          @c p_params is a @c NULL pointer or a required field of the struct
 *                                 (other than @c p_data) is @c NULL.
 * @retval NRF_ERROR_FORBIDDEN     Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t nrf_mesh_packet_send(const nrf_mesh_tx_params_t * p_params,
                              uint32_t * const p_packet_reference);

/**
 * Runs the mesh packet processing process.
 *
 * Calling this function allows the mesh to run. The mesh stack will process buffered
 * incoming packets and send outgoing messages.
 *
 * @note This function must be called from the main loop if the mesh is configured to be running in
 *       NRF_MESH_IRQ_PRIORITY_THREAD, and only then.
 *       If the mesh in running in IRQ level, the processing will be triggered from inside the mesh,
 *       and this function must not be called.
 *
 * @note During @c nrf_mesh_process(), all events generated by the Mesh will
 * be directly forwarded to the application if it has registered an event
 * callback using @ref nrf_mesh_evt_handler_add().
 *
 * @warning The Mesh will discard any data as soon as it has passed it on to the
 * application.
 *
 * @retval true   Processing is done, i.e. no more packets are pending.
 *                It is safe to go to sleep by calling sd_app_evt_wait().
 * @retval false  Processing is not done, i.e. packets are still pending.
 */
bool nrf_mesh_process(void);

/**
 * Pass SoftDevice SoC events to the Mesh.
 *
 * Add this function in the SoC event dispatcher function used with the
 * SoftDevice handler module (see softdevice_sys_evt_handler_set()
 * softdevice_handler.h in the SDK).
 *
 * @warning It is vital for the Mesh to retrieve SoC events for it to function.
 *
 * @param[in] sd_evt SoftDevice SoC event.
 *
 * @retval NRF_SUCCESS Event successfully received.
 */
uint32_t nrf_mesh_on_sd_evt(uint32_t sd_evt);

/**
 * Set a callback which will be called for every received non-filtered packet.
 *
 * The main BT Mesh packets are not filtered and will be passed to the callback.
 *
 * See @ref MESH_API_GROUP_BEARER_FILTER for the details about packet's filtering.
 *
 * This function must be called after @ref nrf_mesh_init().
 *
 * @param[in] rx_cb Receive callback function.
 */
void nrf_mesh_rx_cb_set(nrf_mesh_rx_cb_t rx_cb);

/**
 * Unregister the RX callback, if any.
 */
void nrf_mesh_rx_cb_clear(void);

/**
 * Notify the core stack that a subnet was added to the device.
 *
 * @param[in] net_key_index Key index of the added subnet.
 * @param[in] p_network_id Network ID of the added subnet.
 */
void nrf_mesh_subnet_added(uint16_t net_key_index, const uint8_t * p_network_id);

/**
 * Get unique token to be able to recognize tx complete events.
 * The function guarantees that the given token will no be intersected
 * with tokens of services from the mesh stack.
 *
 * @retval 32bits unique token value.
 */
nrf_mesh_tx_token_t nrf_mesh_unique_token_get(void);

/** @} end of MESH_API_GROUP_CORE */
/** @} */
#endif  /* NRF_MESH_H__ */
