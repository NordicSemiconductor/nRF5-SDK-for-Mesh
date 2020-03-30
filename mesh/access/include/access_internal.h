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

#ifndef ACCESS_INTERNAL_H__
#define ACCESS_INTERNAL_H__

#include <stdint.h>
#include "bitfield.h"
#include "device_state_manager.h"
#include "access_publish.h"
#include "utils.h"

/**
 * @internal
 * @defgroup ACCESS_INTERNAL Access Layer internal definitions
 * @ingroup MESH_API_GROUP_ACCESS
 * Provides defines and structures for internal management of access layer state.
 * @{
 */

/** Mask for the two most significant bits that determine the opcode size. */
#define ACCESS_PACKET_OPCODE_FORMAT_MASK   (0xC0)
/** 1 byte opcode on the form 0b00xx xxxx. */
#define ACCESS_PACKET_OPCODE_FORMAT_1BYTE0 (0x00)
/** 1 byte opcode on the form 0b01xx xxxx. */
#define ACCESS_PACKET_OPCODE_FORMAT_1BYTE1 (0x40)
/** 2 byte opcode. */
#define ACCESS_PACKET_OPCODE_FORMAT_2BYTE  (0x80)
/** 3 byte opcode. */
#define ACCESS_PACKET_OPCODE_FORMAT_3BYTE  (0xC0)
/** Invalid opcode format. */
#define ACCESS_OPCODE_INVALID              (0x7F)

/* Internal state defines used for tracking the state of an instance. */
#define ACCESS_INTERNAL_STATE_ALLOCATED   (1 << 0)
#define ACCESS_INTERNAL_STATE_REFRESHED   (1 << 1)
#define ACCESS_INTERNAL_STATE_INVALIDATED (1 << 2)
#define ACCESS_INTERNAL_STATE_ALLOCATED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_ALLOCATED)
#define ACCESS_INTERNAL_STATE_ALLOCATED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_ALLOCATED)
#define ACCESS_INTERNAL_STATE_IS_ALLOCATED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_ALLOCATED))
#define ACCESS_INTERNAL_STATE_REFRESHED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_REFRESHED)
#define ACCESS_INTERNAL_STATE_REFRESHED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_REFRESHED)
#define ACCESS_INTERNAL_STATE_IS_REFRESHED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_REFRESHED))
#define ACCESS_INTERNAL_STATE_INVALIDATED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_INVALIDATED)
#define ACCESS_INTERNAL_STATE_INVALIDATED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_INVALIDATED)
#define ACCESS_INTERNAL_STATE_IS_INVALIDATED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_INVALIDATED))

/* ********** Type definitions ********** */

typedef struct
{
    uint8_t internal_state; /* See the defines ACCESS_INTERNAL_STATE_** */
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
} access_subscription_list_t;

/** Access element structure. */
typedef struct
{
    /**
     * Bluetooth SIG location descriptor.
     * https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors
     */
    uint16_t location;
    /** Number of models defined by SIG. */
    uint8_t sig_model_count;
    /** Number of vendor-specific models. */
    uint8_t vendor_model_count;
    /** Attention timer state. 0 is off, otherwise remaining time in seconds. */
    uint8_t attention_timer;
    /** Used for tracking an element instance: see the defines ACCESS_INTERNAL_STATE_**. */
    uint8_t internal_state;
} access_element_t;

typedef struct
{
    /** The Model and Company ID of this model. */
    access_model_id_t model_id;
    /** The application key handle provided by DSM to the application key this model publishes with. */
    dsm_handle_t publish_appkey_handle;
    /** The address handle provided by DSM to the address this model publishes to. */
    dsm_handle_t publish_address_handle;
    /** Applications bound to the model. Bound by their handle: (1 << handle). */
    uint32_t application_keys_bitfield[BITFIELD_BLOCK_COUNT(DSM_APP_MAX + DSM_DEVICE_MAX)];
    /** Element that owns this model. */
    uint16_t element_index;
    /** Array element index from subscription pool where a subscription list for a model is located.*/
    uint16_t subscription_pool_index;
    /** Friendship credentials flag. */
    bool friendship_credential_flag;
    /** This model's TTL value for each published packet. */
    uint8_t publish_ttl;
    /** Number of steps and step resolution for the publication functionality. */
    access_publish_period_t publication_period;
    /** The publish retransmit count and the interval of the retransmitting steps for the retransmitting functionality. */
    access_publish_retransmit_t publication_retransmit;
} access_model_state_data_t;

typedef struct
{
    /** Data pertaining to a specific model instance, which is crucial for maintaining the model's configuration in a network. */
    access_model_state_data_t model_info;
    /** Model publication state. */
    access_model_publication_state_t publication_state;
    /** Publish period divisor. */
    uint16_t publish_divisor;
    /** Pointer to the list of opcodes with the corresponding callback functions. */
    const access_opcode_handler_t * p_opcode_handlers;
    /** Number of opcodes in list @ref p_opcode_handlers. */
    uint16_t opcode_count;
    /** Generic pointer used to give context in the model callbacks. */
    void  * p_args;
    /** Used for tracking a model instance: see the defines ACCESS_INTERNAL_STATE_**. */
    uint8_t internal_state;
} access_common_t;

typedef struct
{
    uint16_t subscription_list_count;
    uint16_t element_count;
    uint16_t model_count;
} access_flash_metadata_t;

typedef struct
{
    uint32_t inverted_bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
} access_flash_subscription_list_t;

/**
 * Handles the received message and calls the acceptable model handler.
 *
 * @param[in]  p_message Pointer to the structure with information about
 *                       the received message (data, length, metadata).
 */
void access_incoming_handle(const access_message_rx_t * p_message);

/**
 * Internal function for publishing an access layer message.
 *
 * @param[in] handle             Access handle of the model that wants to send data.
 * @param[in] p_tx_message       Message to be published.
 * @param[in] p_access_payload   Access payload containing the access
 *                               message and the opcode.
 * @param[in] access_payload_len Access payload length.
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to an element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Message that was about to be sent is larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 * @retval NRF_ERROR_INVALID_STATE  Segmented packets: There's already a segmented packet that is being sent to this destination.
                                    Wait for this process to finish before sending new segmented packets.
 * @retval NRF_ERROR_FORBIDDEN      Unsegmented packets: Failed to allocate a sequence number from network.
 */
uint32_t access_packet_tx(access_model_handle_t handle,
                          const access_message_tx_t * p_tx_message,
                          const uint8_t *p_access_payload,
                          uint16_t access_payload_len);

/** @} */
#endif /* ACCESS_INTERNAL_H__ */
