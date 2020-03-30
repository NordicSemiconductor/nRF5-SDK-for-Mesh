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

#ifndef ACCESS_H__
#define ACCESS_H__

#include <stdint.h>
#include "device_state_manager.h"
#include "nrf_mesh.h"

/**
 * @defgroup ACCESS Access layer API
 * @ingroup MESH_API_GROUP_ACCESS
 * The access layer API is the main API for Mesh Models.
 *
 * The access layer API provides a way for models to register into the device database,
 * and send and receive messages on the mesh. See @ref md_doc_user_guide_modules_models_creating for a
 * walkthrough of the usage of the access layer API.
 *
 * @{
 * @defgroup ACCESS_MSCS Access layer API MSCs
 * @brief Access layer sequence diagrams
 *
 * @{
 * @mscfile model.msc "Basic access layer usage"
 * @mscfile message_rx.msc "Receiving an access layer message"
 * @mscfile periodic.msc "Periodic publishing"
 * @}
 */

/**
 * @defgroup ACCESS_DEFINES Defines
 * Access layer defines.
 * @{
 */

/** Invalid access model handle value. */
#define ACCESS_HANDLE_INVALID (0xFFFF)
/** Company ID value for Bluetooth SIG opcodes or models. */
#define ACCESS_COMPANY_ID_NONE (0xFFFF)
/** Company ID value for Nordic Semiconductor. */
#define ACCESS_COMPANY_ID_NORDIC (0x0059)

/** Invalid element index. */
#define ACCESS_ELEMENT_INDEX_INVALID (0xFFFF)

/**
 * Macro used to define a SIG model opcode.
 * @param[in] opcode Opcode of the SIG model.
 * @return Expands to an initializer for an @ref access_opcode_t struct.
 * @see access_opcode_t, access_message_tx_t
 */
#define ACCESS_OPCODE_SIG(opcode)             { (opcode), ACCESS_COMPANY_ID_NONE }

/**
 * Macro used to define a vendor model opcode.
 * @param[in] opcode  Opcode of the vendor model.
 * @param[in] company Company ID for the vendor model.
 * @return Expands to an initializer for an @ref access_opcode_t struct.
 * @see access_opcode_t, access_message_tx_t
 */
#define ACCESS_OPCODE_VENDOR(opcode, company) { (opcode), (company) }

/**
 * Macro used to define a SIG model ID.
 * @param[in] id Model ID.
 * @return Expands to an initializer for an @ref access_model_id_t struct.
 * @see access_model_id_t
 */
#define ACCESS_MODEL_SIG(id)  {.company_id = ACCESS_COMPANY_ID_NONE, .model_id = (id)}

/**
 * Macro used to define a vendor model ID.
 * @param[in] id      Model ID.
 * @param[in] company Company ID for the vendor model.
 * @return Expands to an initializer for an @ref access_model_id_t struct.
 * @see access_model_id_t
 */
#define ACCESS_MODEL_VENDOR(id, company) {.company_id = (company), .model_id = (id)}

/** Value used for TTL parameters in order to set the TTL to the default value. */
#define ACCESS_TTL_USE_DEFAULT  (0xFF)

/**
 * Maximum payload length for an access layer message.
 *
 * @note Payloads greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX will be segmented.
 */
#define ACCESS_MESSAGE_LENGTH_MAX (NRF_MESH_SEG_PAYLOAD_SIZE_MAX)

/** Max step size used for periodic publishing. */
#define ACCESS_PUBLISH_PERIOD_STEP_MAX (0x3F)

/** Publish step resolution, number of bits. */
#define ACCESS_PUBLISH_STEP_RES_BITS (2)
/** Publish step number, number of bits. */
#define ACCESS_PUBLISH_STEP_NUM_BITS (6)
/** Publish Retransmit Count, number of bits. */
#define ACCESS_PUBLISH_RETRANSMIT_COUNT_BITS (3)
/** Publish Retransmit Interval Steps, number of bits. */
#define ACCESS_PUBLISH_RETRANSMIT_INTERVAL_STEPS_BITS (5)

/** Value used for access_publish_period_t structs when publishing is disabled. */
#define ACCESS_PUBLISH_PERIOD_NONE   { ACCESS_PUBLISH_RESOLUTION_100MS, 0 }

/** @} */

/**
 * @defgroup ACCESS_TYPES Types
 * Access layer type definitions.
 * @{
 */

/*lint -align_max(push) -align_max(1) */

/** Access layer model ID. */
typedef struct __attribute((packed))
{
    /** Company ID. Bluetooth SIG models shall set this to @ref ACCESS_COMPANY_ID_NONE. */
    uint16_t company_id;
    /** Model ID. */
    uint16_t model_id;
} access_model_id_t;

/*lint -align_max(pop) */

/** Access layer handle type. */
typedef uint16_t access_model_handle_t;

/**
 * Access layer publish timeout event callback.
 *
 * @param[in] handle Access layer model handle.
 * @param[in] p_args Optional generic argument pointer.
 */
typedef void (*access_publish_timeout_cb_t)(access_model_handle_t handle, void * p_args);

/**
 * Access layer opcode type.
 *
 * The format of the opcodes is given in the table below:
 * @tagMeshSp table 3.43
 *
 * | Byte 0     | Byte 1     | Byte 2     | Description                                                                                   |
 * | ---------- | ---------- | ---------- | --------------------------------------------------------------------------------------------- |
 * | `0xxxxxxx` |            |            | 1-octet Bluetooth SIG Opcodes (excluding 01111111)                                            |
 * | `01111111` |            |            | Reserved for Future Use                                                                       |
 * | `10xxxxxx` | `xxxxxxxx` |            | 2-octet Bluetooth SIG Opcodes                                                                 |
 * | `11xxxxxx` | `zzzzzzzz` | `zzzzzzzz` | 3-octet Vendor Specific Opcodes. `z` denotes company identifier packed in little-endian order |
 *
 * To initialize an access_opcode_t, use the @ref ACCESS_OPCODE_SIG() or @ref ACCESS_OPCODE_VENDOR() macros.
 */
typedef struct
{
    /** 14-bit or 7-bit Bluetooth SIG defined opcode or 6-bit vendor specific opcode. */
    uint16_t opcode;
    /** Company ID. Set to @ref ACCESS_COMPANY_ID_NONE if it is a Bluetooth SIG defined opcode. */
    uint16_t company_id;
} access_opcode_t;

/** Metadata for received messages. */
typedef struct
{
    /** Source address of the message. */
    nrf_mesh_address_t src;
    /** Destination address of the message. */
    nrf_mesh_address_t dst;
    /** TTL value for the received message. */
    uint8_t ttl;
    /** Application key handle that decrypted the message. */
    dsm_handle_t appkey_handle;
    /** Core RX metadata attached to the packet */
    const nrf_mesh_rx_metadata_t * p_core_metadata;
    /** Network key handle that decrypted the message. */
    dsm_handle_t subnet_handle;
} access_message_rx_meta_t;

/** Access layer RX event structure. */
typedef struct
{
    /** Opcode of the message. */
    access_opcode_t opcode;
    /** Pointer to the first byte of message data (excludes the opcode). */
    const uint8_t * p_data;
    /** Length of @c p_data. */
    uint16_t length;
    /** Meta data for the message. */
    access_message_rx_meta_t meta_data;
} access_message_rx_t;

/** Access layer TX parameter structure. */
typedef struct
{
    /** Opcode for the message. */
    access_opcode_t opcode;
    /** Pointer to the message data. */
    const uint8_t * p_buffer;
    /** Length of the data (excluding the opcode). */
    uint16_t length;
    /** Forces the message to be sent out as a segmented message, if message is shorter than the size
     * required for the unsegmented access message for a given MIC size */
    bool force_segmented;
    /** Select desired transport MIC size. See @ref nrf_mesh_transmic_size_t */
    nrf_mesh_transmic_size_t transmic_size;
    /** Token that can be used as a reference in the TX complete callback. */
    nrf_mesh_tx_token_t access_token;
} access_message_tx_t;

/**
 * Access layer opcode handler callback type.
 *
 * @param[in] handle    Access layer model handle.
 * @param[in] p_message Access RX message structure.
 * @param[in] p_args    Optional generic argument pointer.
 */
typedef void (*access_opcode_handler_cb_t)(access_model_handle_t handle,
                                           const access_message_rx_t * p_message,
                                           void * p_args);

/**
 * Opcode handler type.
 *
 * Each specific model implementation is assumed to statically define an array of "opcode handlers",
 * one handler for each of the expected opcodes of the given model.
 */
typedef struct
{
    /** The model opcode. */
    access_opcode_t opcode;
    /** The opcode handler callback for the given opcode. */
    access_opcode_handler_cb_t handler;
} access_opcode_handler_t;

/**
 * Access model allocation parameter structure.
 */
typedef struct
{
    /** SIG or Vendor Model ID. */
    access_model_id_t model_id;
    /** Element index to add the model to. */
    uint16_t element_index;
    /**
     * Pointer to list of opcode handler callbacks. This can be specified as NULL if
     * @ref access_model_add_params_t::opcode_count is specified as zero.
     */
    const access_opcode_handler_t * p_opcode_handlers;
    /** Number of opcode handles. */
    uint32_t opcode_count;
    /**
     * Generic argument pointer. This pointer will be supplied as an argument in the callbacks from
     * the access layer, e.g., @ref access_opcode_handler_cb_t. May be set to @c NULL if unused.
     */
    void * p_args;
    /**
     * Timeout callback called when the publication timer expires. Set to @c NULL for models that
     * doesn't support periodic publishing.
     */
    access_publish_timeout_cb_t publish_timeout_cb;
} access_model_add_params_t;

/**
 * Model publish period structure.
 */
typedef struct
{
    /** Step resolution. */
    uint8_t step_res : ACCESS_PUBLISH_STEP_RES_BITS;
    /** Number of steps. */
    uint8_t step_num : ACCESS_PUBLISH_STEP_NUM_BITS;
} access_publish_period_t;

/**
 * Model publish retransmit structure.
 */
typedef struct
{
    /** Publish Retransmit Count. */
    uint8_t count : ACCESS_PUBLISH_RETRANSMIT_COUNT_BITS;
    /** Publish Retransmit Interval Steps. */
    uint8_t interval_steps : ACCESS_PUBLISH_RETRANSMIT_INTERVAL_STEPS_BITS;
} access_publish_retransmit_t;

/**
 * Periodic publishing step resolution.
 */
typedef enum
{
    /** Step resolution: 100ms / step. */
    ACCESS_PUBLISH_RESOLUTION_100MS = 0,
    /** Step resolution: 1s / step. */
    ACCESS_PUBLISH_RESOLUTION_1S    = 1,
    /** Step resolution: 10s / step. */
    ACCESS_PUBLISH_RESOLUTION_10S   = 2,
    /** Step resolution: 10min / step. */
    ACCESS_PUBLISH_RESOLUTION_10MIN = 3,
    /** Maximum publish resolution. */
    ACCESS_PUBLISH_RESOLUTION_MAX = ACCESS_PUBLISH_RESOLUTION_10MIN
} access_publish_resolution_t;

/**
 * @}
 */

/**
 * Initializes the access layer.
 */
void access_init(void);

/**
 * Clears the access layer states, and erases the persistent storage copy.
 */
void access_clear(void);

/**
 * Allocates, initializes and adds a model to the element at the given element index.
 *
 * @param[in]  p_model_params            Pointer to model initialization parameter structure.
 * @param[out] p_model_handle            Pointer to store allocated model handle.
 *
 * @retval     NRF_SUCCESS               Successfully added model to the given element.
 * @retval     NRF_ERROR_NO_MEM          @ref ACCESS_MODEL_COUNT number of models already allocated.
 * @retval     NRF_ERROR_NULL            One or more of the function parameters was NULL.
 * @retval     NRF_ERROR_FORBIDDEN       Multiple model instances per element are not allowed
 *                                       or changes to device composition are not allowed.
 *                                       Adding a new model after device is provisioned is not allowed.
 * @retval     NRF_ERROR_NOT_FOUND       Invalid access element index.
 * @retval     NRF_ERROR_INVALID_LENGTH  Number of opcodes was zero and pointer to the list of
 *                                       opcode handler callbacks is not NULL.
 * @retval     NRF_ERROR_INVALID_PARAM   One or more of the opcodes had an invalid format.
 * @see        access_opcode_t for documentation of the valid format.
 */
uint32_t access_model_add(const access_model_add_params_t * p_model_params,
                          access_model_handle_t * p_model_handle);

/**
 * Publishes an access layer message to the publish address of the model.
 *
 * Once the message is published and the Public Retransmit Count
 * (see @ref config_publication_state_t.retransmit_count) for the model
 * is set to non-zero value, the message is queued for later re-transmissions.
 * If there is not enough memory to publish the message during the re-transmission
 * or if the previous transmission of the segmented message is still in progress,
 * the re-transmission attempt will be skipped. If the next message is published
 * before previous re-tranmissions are finished, the remaining re-transmissions
 * attempts of the previous message will be skipped. the re-transmission attempt
 * will be skipped.
 *
 * @note The message will be sent as a segmented message and reassembled on the peer side if one of
 * the following conditions are true:
 * - The length of the message is greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX.
 * - The @p force_segmented field of @p p_message is true.
 * - The @p transmic_size field of @p p_message is @ref NRF_MESH_TRANSMIC_SIZE_LARGE.
 *
 * @param[in] handle    Access handle for the model that wants to send data.
 * @param[in] p_message Access layer TX message parameter structure.
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t access_model_publish(access_model_handle_t handle, const access_message_tx_t * p_message);

/**
 * Replies to an access layer message.
 *
 * This function is intended to be used in pair with the opcode handle callbacks. The model
 * gets a message through the @ref access_opcode_handler_cb_t "opcode handler callback" and
 * replies to the incoming message by calling this function.
 *
 * @note The reply is sent as a segmented message and reassembled on the peer side if one of
 * the following conditions is true:
 * - The length of the message is greater than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX.
 * - The @p force_segmented field of @p p_reply is true.
 * - The @p transmic_size field of @p p_reply is @ref NRF_MESH_TRANSMIC_SIZE_LARGE.
 *
 * @param[in] handle    Access handle for the model that wants to send data.
 * @param[in] p_message Incoming message that the model is replying to.
 * @param[in] p_reply   The reply data.
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set, or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There is already a segmented packet to this destination in
 *                                  progress. Wait for it to finish before sending new segmented
 *                                  packets.
 */
uint32_t access_model_reply(access_model_handle_t handle,
                            const access_message_rx_t * p_message,
                            const access_message_tx_t * p_reply);

/**
 * Returns the element index for the model handle
 *
 * This function is indended to be used inside the state transaction callbacks triggered by the
 * model to quickly resolve the element index on which the message arrived.
 *
 * @param[in]  handle           Access handle for the model that wants to send data.
 * @param[out] p_element_index  Pointer to the hold retrieved element index for the model handle
 *
 * @retval NRF_SUCCESS          Model handle is valid and `element_index` pointer is updated.
 * @retval NRF_ERROR_NULL       NULL pointer supplied to function.
 * @retval NRF_ERROR_NOT_FOUND  Invalid model handle or model not bound to element.
 *
 */
uint32_t access_model_element_index_get(access_model_handle_t handle, uint16_t * p_element_index);

/** @} */
#endif /* ACCESS_H__ */
