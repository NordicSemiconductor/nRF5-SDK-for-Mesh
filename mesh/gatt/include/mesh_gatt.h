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
#ifndef MESH_GATT_H__
#define MESH_GATT_H__

#include <stdint.h>
#include "timer_scheduler.h"
#include "packet_buffer.h"
#include "utils.h"
#include "sdk_config.h"

/**
 * @defgroup MESH_GATT Generic GATT interface for Mesh
 * @{
 */

/** Timeout for an incoming SAR transfer as defined in @tagMeshSp section 6.6. */
#define MESH_GATT_RX_SAR_TIMEOUT_US 20000000

/** The maximum number of concurrent GATT connections supported. */
#define MESH_GATT_CONNECTION_COUNT_MAX (NRF_SDH_BLE_TOTAL_LINK_COUNT)

#define MESH_GATT_PROXY_PDU_MAX_SIZE (66)
#define MESH_GATT_MTU_SIZE_MAX       (69)
#define MESH_GATT_PACKET_MAX_SIZE    (MESH_GATT_PROXY_PDU_MAX_SIZE - 1)
#define MESH_GATT_TX_BUFFER_SIZE     ALIGN_VAL(MESH_GATT_PACKET_MAX_SIZE + \
                                               sizeof(packet_buffer_packet_t), WORD_SIZE)*3

#if NRF_SDH_BLE_GATT_MAX_MTU_SIZE != MESH_GATT_MTU_SIZE_MAX
#warning An MTU size of 69 octets is recommended.
#endif

#if NRF_SDH_BLE_GATT_MAX_MTU_SIZE < BLE_GATT_ATT_MTU_DEFAULT
#error NRF_SDH_BLE_GATT_MAX_MTU_SIZE < 23
#endif

typedef enum
{
    MESH_GATT_PDU_TYPE_NETWORK_PDU,
    MESH_GATT_PDU_TYPE_MESH_BEACON,
    MESH_GATT_PDU_TYPE_PROXY_CONFIG,
    MESH_GATT_PDU_TYPE_PROV_PDU,
    MESH_GATT_PDU_TYPE_PROHIBITED,
    MESH_GATT_PDU_TYPE_INVALID = 0xFF
} mesh_gatt_pdu_type_t;

typedef enum
{
    MESH_GATT_EVT_TYPE_RX,
    MESH_GATT_EVT_TYPE_TX_COMPLETE,
    MESH_GATT_EVT_TYPE_ADV_TIMEOUT,
    MESH_GATT_EVT_TYPE_CONNECTED,
    MESH_GATT_EVT_TYPE_DISCONNECTED,
    // The CCCD has been written to and we're ready to start transmitting notifications
    MESH_GATT_EVT_TYPE_TX_READY,
} mesh_gatt_evt_type_t;

typedef struct
{
    mesh_gatt_pdu_type_t pdu_type;
    const uint8_t * p_data;
    uint16_t length;
} mesh_gatt_evt_rx_t;

typedef struct
{
    nrf_mesh_tx_token_t token;
    mesh_gatt_pdu_type_t pdu_type;
} mesh_gatt_evt_tx_complete_t;

typedef struct
{
    mesh_gatt_evt_type_t type;
    uint8_t conn_index;
    union
    {
        mesh_gatt_evt_rx_t rx;
        mesh_gatt_evt_tx_complete_t tx_complete;
    } params;
} mesh_gatt_evt_t;

typedef struct
{
    uint16_t service;
    uint16_t tx_char;
    uint16_t rx_char;
} mesh_gatt_uuids_t;

typedef struct mesh_gatt mesh_gatt_t;

typedef void (*mesh_gatt_evt_handler_t)(const mesh_gatt_evt_t * p_evt, void * p_context);

typedef struct
{
    /** Pointer to the current active packet buffer packet. */
    packet_buffer_packet_t * p_curr_packet;
    /**
     * Offset into the current active packet.
     *
     * The offset moves along the user PDU and writes a length + header at the last byte of the
     * previous payload.
     */
    uint8_t offset;
} mesh_gatt_transaction_t;

/** Mesh GATT connection context structure. */
typedef struct
{
    /** Softdevice connection handle */
    uint16_t conn_handle;
    /** Effective ATT MTU. Three bytes shorter than the negotiated value. */
    uint16_t effective_mtu;
    struct
    {
        packet_buffer_t packet_buffer;
        uint8_t packet_buffer_data[MESH_GATT_TX_BUFFER_SIZE];
        mesh_gatt_transaction_t transaction;
        bool tx_complete_process;
    } tx;
    struct
    {
        uint8_t buffer[MESH_GATT_PACKET_MAX_SIZE];
        mesh_gatt_pdu_type_t pdu_type;
        uint8_t offset;
        timer_event_t timeout_event;
    } rx;
} mesh_gatt_connection_t;

/** Softdevice GATT handles. */
typedef struct
{
    uint16_t service;
    ble_gatts_char_handles_t tx;
    ble_gatts_char_handles_t rx;
} mesh_gatt_handles_t;

/**
 * Mesh GATT context structure.
 * @note Not modified by the user.
 */
struct mesh_gatt
{
    mesh_gatt_uuids_t uuids;
    mesh_gatt_handles_t handles;
    mesh_gatt_evt_handler_t evt_handler;
    mesh_gatt_connection_t connections[MESH_GATT_CONNECTION_COUNT_MAX];
    void * p_context;
};


/**
 * Initialize a Mesh GATT service.
 *
 * @param[in]     p_uuids     Pointer to the service and characteristics UUID structure.
 * @param[in]     evt_handler Mesh GATT event handler callback function.
 * @param[in]     p_context   Context pointer to be supplied with any Mesh GATT event.
 */
void mesh_gatt_init(const mesh_gatt_uuids_t * p_uuids,
                    mesh_gatt_evt_handler_t evt_handler,
                    void * p_context);

/**
 * Allocates a packet for the given connection index and PDU type.
 *
 * @note If not in a connected state for the given @c conn_index, this call will return @c NULL.
 * @note If the connection corresponding to the @c conn_index of the allocated packet is terminated,
 * the buffer will no longer be valid.
 *
 * @param[in]     conn_index Connection index.
 * @param[in]     type       Type of Mesh GATT PDU.
 * @oaram[in]     size       Size of the packet.
 * @param[in]     token      TX token that shall be present in the TX complete callback for the packet.
 *
 * @returns a pointer to a buffer of @c size bytes or @c NULL if no buffer could be allocated.
 */
uint8_t * mesh_gatt_packet_alloc(uint16_t conn_index,
                                 mesh_gatt_pdu_type_t type,
                                 uint16_t size,
                                 nrf_mesh_tx_token_t token);

/**
 * Sends a previously allocated packet.
 *
 * @param[in]     conn_index Connection index of the Mesh GATT connection to transmit the packet.
 * @param[in]     p_packet   Pointer to the previously allocated (and now filled) packet.
 *
 * @retval NRF_SUCCESS             Successfully started packet transmission.
 * @retval NRF_ERROR_INVALID_STATE The given @c conn_index is not in a connected state.
 */
uint32_t mesh_gatt_packet_send(uint16_t conn_index, const uint8_t * p_packet);

/**
 * Discards a previously allocated packet.
 *
 * @warning Calling this with pointer from a terminated connected will cause an assert.
 *
 * @param[in]     conn_index Connection index where the packet was allocated.
 * @param[in]     p_packet   Pointer to the previously allocated packet to discard.
 */
void mesh_gatt_packet_discard(uint16_t conn_index, const uint8_t * p_packet);

/**
 * Checks if the given Mesh GATT connection has pending packets.
 *
 * @param[in] conn_index Connection index
 *
 * @retval true     If the connection has pending packets.
 * @retval false    If there is no pending packets for the given connection.
 */
bool mesh_gatt_packet_is_pending(uint16_t conn_index);

/**
 * Disconnects the given Mesh GATT connection.
 *
 * @param[in]     conn_index Connection index to terminate.
 *
 * @return Inherits the return values from sd_ble_gap_disconnect().
 */
uint32_t mesh_gatt_disconnect(uint16_t conn_index);

/**
 * BLE event handler.
 *
 * @param[in] p_ble_evt Incoming BLE event from the SoftDevice.
 * @param[in] p_context Context pointer.
 */
void mesh_gatt_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/** @} end of MESH_GATT */

#endif /* MESH_GATT_H__ */
