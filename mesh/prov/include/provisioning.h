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

#ifndef PROVISIONING_H__
#define PROVISIONING_H__

#include "prov_pdu.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_types.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "utils.h"

/**
 * @defgroup MESH_PROV Provisioning components
 * Internal components of the provisioning subsystem.
 * @ingroup MESH_API_GROUP_PROV
 * @{
 */

/****************** Member helper functions ******************/
/**
 * @defgroup PROVISIONING_MEMBER_HELPER Provisioning structure member helper functions
 * @{
 */

/**
 * Get the prov context object holding the given prov_bearer_t.
 *
 * @param[in] p_bearer Pointer to a provisioning bearer instance contained in a
 * provisioning context structure.
 *
 * @returns A pointer to the provisioning bearer structure's context parent.
 */
static inline nrf_mesh_prov_ctx_t * prov_bearer_ctx_get(prov_bearer_t * p_bearer)
{
    return (nrf_mesh_prov_ctx_t *) p_bearer->p_parent;
}


/**
 * Verifies the format of the provisioning data.
 *
 * @param[in] p_data Data to verify
 *
 * @returns Whether the provisioning data satisfies all boundary conditions.
 */
static inline bool prov_data_is_valid(const nrf_mesh_prov_provisioning_data_t * p_data)
{
    return (p_data->netkey_index <= NRF_MESH_GLOBAL_KEY_INDEX_MAX &&
            nrf_mesh_address_type_get(p_data->address) == NRF_MESH_ADDRESS_TYPE_UNICAST);
}

/**
 * Verifies that the starting address assigned by the provisioner has enough room for all the
 * device's elements.
 *
 * @param[in] p_data Provisioning data structure.
 * @param[in] num_elements Number of elements in the device.
 *
 * @returns @c true if the address is valid.
 */
static inline bool prov_address_is_valid(const nrf_mesh_prov_provisioning_data_t * p_data, uint8_t num_elements)
{
    return (nrf_mesh_address_type_get(p_data->address) == NRF_MESH_ADDRESS_TYPE_UNICAST &&
            nrf_mesh_address_type_get(p_data->address + num_elements - 1) == NRF_MESH_ADDRESS_TYPE_UNICAST);
}

/**
 * Checks if the length of a packet is valid.
 *
 * This is done by looking at the first byte (the PDU type) to determine the type of the
 * packet, and then comparing the specified length with the expected length of the packet.
 *
 * @param[in] p_buffer Pointer to the packet buffer.
 * @param[in] length Length of the packet buffer.
 *
 * @returns Whether the length of the packet is valid.
 */
bool prov_packet_length_valid(const uint8_t * p_buffer, uint16_t length);

/** @} end of PROVISIONING_BEARER_HELPER*/

/****************** Provisioning bearer control/message transmission functions ******************/
/**
 * @defgroup PROVISIONING_TX Provisioning message transmission functions
 * @{
 */

/****************** Provisioning PDU transmit functions ******************/

/**
 * Sends the public key message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_public_key The public key of the user.
 *
 * @retval NRF_SUCCESS Successfully sent the public key.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_public_key(prov_bearer_t * p_bearer, const uint8_t * p_public_key);

/**
 * Sends the confirmation message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_confirmation_value The confirmation value.
 *
 * @retval NRF_SUCCESS Successfully sent the confirmation message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_confirmation(prov_bearer_t * p_bearer, const uint8_t * p_confirmation_value);

/**
 * Sends the provisioning random message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_random The confirmation key (see @ref nrf_mesh_prov_ctx).
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning random message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_random(prov_bearer_t * p_bearer, const uint8_t * p_random);

/**
 * @defgroup PROVISIONING_TX_PROVISIONER TX functions meant only for provisioner role
 * @{
 */

 /**
 * Sends the provisioning invite message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] attention_duration_s The attention timer value in seconds.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, see @ref nrf_mesh_prov_ctx.
 *
 * @retval NRF_SUCCESS Successfully sent a link establishment request.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_invite(prov_bearer_t * p_bearer, uint8_t attention_duration_s, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning start message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_start The assembled provisioning start pdu.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, see @ref nrf_mesh_prov_ctx.
 *
 * @retval NRF_SUCCESS Successfully sent a provisioning start message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_start(prov_bearer_t * p_bearer, const prov_pdu_prov_start_t * p_start, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning data message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_data The confirmation key (see @ref nrf_mesh_prov_ctx).
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning data message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_data(prov_bearer_t * p_bearer, const prov_pdu_data_t * p_data);

/** @} end of PROVISIONING_TX_PROVISIONER */

/**
 * @defgroup PROVISIONING_TX_PROVISIONEE TX functions meant only for provisionee role
 * @{
 */

/**
 * Sends the provisioning capabilities message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] p_caps The assembled provisioning capabilities pdu.
 * @param[out] p_confirmation_inputs The confirmation inputs array to update, see @ref nrf_mesh_prov_ctx.
 *
 * @retval NRF_SUCCESS Successfully sent the provisionee capabilities.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_capabilities(prov_bearer_t * p_bearer, const prov_pdu_caps_t * p_caps, uint8_t * p_confirmation_inputs);

/**
 * Sends the provisioning input complete message
 *
 * @param[in, out] p_bearer The bearer instance to use.
 *
 * @retval NRF_SUCCESS Successfully sent the input complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_input_complete(prov_bearer_t * p_bearer);

/**
 * Sends the provisioning complete message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_complete(prov_bearer_t * p_bearer);

/**
 * Sends the provisioning failed message.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in] failure_code The reason for the provisioning failure.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx_failed(prov_bearer_t * p_bearer, nrf_mesh_prov_failure_code_t failure_code);

/** @} end of PROVISIONING_TX_PROVISIONEE*/

/**
 * Sends a raw provisioning PDU.
 *
 * @param[in, out] p_bearer The bearer instance to use.
 * @param[in]      p_data   PDU data pointer.
 * @param[in]      length   Length of PDU.
 *
 * @retval NRF_SUCCESS Successfully sent the provisioning complete message.
 * @retval NRF_ERROR_NOT_SUPPORTED The given bearer_type is not supported.
 * @retval NRF_ERROR_INVALID_STATE The given bearer is not in an established link.
 * @retval NRF_ERROR_NO_MEM The system is short of resources, try again later.
 * @retval NRF_ERROR_BUSY Another transmission is already in progress, wait for it to finish.
 */
uint32_t prov_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

/** @} end of PROVISIONING_TX*/

/** @} */

#endif /* PROVISIONING_H__ */
