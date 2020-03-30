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

#ifndef NRF_MESH_PROV_TYPES_H_
#define NRF_MESH_PROV_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#include "nrf_mesh_defines.h"

/**
 * @defgroup PROV_FIELD_LENGTHS Provisioning field lengths
 * @ingroup NRF_MESH_PROV_DEFINES
 * Length of various provisioning fields.
 * @{
 */

/** Size of the elliptic curve public key. */
#define NRF_MESH_PROV_PUBKEY_SIZE      64
/** Size of the elliptic curve private key. */
#define NRF_MESH_PROV_PRIVKEY_SIZE     32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_ECDHSECRET_SIZE  32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_DATANONCE_SIZE   13

/** Max OOB size permitted by @tagMeshSp table 5.21 and 5.23. */
#define NRF_MESH_PROV_OOB_SIZE_MAX      8

/** Length of Random value. */
#define PROV_RANDOM_LEN               (16)
/** Length of Confirmation value. */
#define PROV_CONFIRMATION_LEN         (NRF_MESH_KEY_SIZE)
/** Length of Auth value. */
#define PROV_AUTH_LEN                 (NRF_MESH_KEY_SIZE)
/** Length of Salt value. */
#define PROV_SALT_LEN                 (NRF_MESH_KEY_SIZE)
/** Length of Nonce. */
#define PROV_NONCE_LEN                (13)
/** Combined length of confirmation inputs: provisioning invite, capabilities and start PDUs.*/
#define PROV_CONFIRMATION_INPUT_LEN   (17)
/** @} */

/**
 * @defgroup NRF_MESH_PROV_CAPABILITIES Provisioning capabilities bit fields
 * @ingroup NRF_MESH_PROV_DEFINES
 *
 * Bitfield definitions for the provisioning capabilities fields.
 * See @tagMeshSp section 5.4.1.2 Provisioning Capabilities.
 * @{
 */

/** Capabilities bit indicating that the FIPS P256EC algorithm is supported. */
#define NRF_MESH_PROV_ALGORITHM_FIPS_P256EC          (1u << 0)

/**
 * Capabilities bit indicating that the public key is available in-band.
 * If no public key type is set, this is the default
 */
#define NRF_MESH_PROV_OOB_PUBKEY_TYPE_INBAND         (0)
/** Capabilities bit indicating that the public key is available OOB. */
#define NRF_MESH_PROV_OOB_PUBKEY_TYPE_OOB            (1u << 0)


/** Capabilities bit indicating that static OOB authentication is supported. */
#define NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED      (1u << 0)

/** Capabilities bit indicating that the device supports blinking as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_BLINK        (1u << 0)
/** Capabilities bit indicating that the device supports beeping as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_BEEP         (1u << 1)
/** Capabilities bit indicating that the device supports vibrating as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_VIBRATE      (1u << 2)
/** Capabilities bit indicating that the device supports displaying numeric data as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC      (1u << 3)
/** Capabilities bit indicating that the device supports displaying alphanumeric data as output OOB action. */
#define NRF_MESH_PROV_OOB_OUTPUT_ACTION_ALPHANUMERIC (1u << 4)

/** Capabilities bit indicating that the device supports pushing something as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_PUSH          (1u << 0)
/** Capabilities bit indicating that the device supports twisting something as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_TWIST         (1u << 1)
/** Capabilities bit indicating that the device supports entering a number as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER  (1u << 2)
/** Capabilities bit indicating that the device supports entering a string as input OOB action. */
#define NRF_MESH_PROV_OOB_INPUT_ACTION_ENTER_STRING  (1u << 3)
/** @} */

/**
 * @defgroup NRF_MESH_PROV_OOB_INFO_SOURCES Provisioning OOB information sources.
 * @ingroup NRF_MESH_PROV_DEFINES
 *
 * OOB information sources for the OOB info bitfield in unprovisioned beacons.
 * Denotes a set of sources from where the user can get information on
 * available OOB data for the product, to aid the provisioning process.
 * @{
 */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_OTHER                    (1u << 0)   /**< Other location. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ELECTRONIC_OR_URI        (1u << 1)   /**< Electronic / URI. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_2D_MACHINE_READABLE_CODE (1u << 2)   /**< 2D machine-readable code. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_BAR_CODE                 (1u << 3)   /**< Bar code. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_NFC                      (1u << 4)   /**< Near Field Communication (NFC). */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_NUMBER                   (1u << 5)   /**< Number. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_STRING                   (1u << 6)   /**< String. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_BOX                   (1u << 11)  /**< On box. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_BOX               (1u << 12)  /**< Inside box. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_PIECE_OF_PAPER        (1u << 13)  /**< On piece of paper. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL            (1u << 14)  /**< Inside manual. */
#define NRF_MESH_PROV_OOB_INFO_SOURCE_ON_DEVICE                (1u << 15)  /**< On device. */
/** @} */

/**
 * @defgroup NRF_MESH_PROV_TYPES Types
 * @ingroup  NRF_MESH_PROV
 * Provisioning type definitions
 * @{
 */

/**
 * The algorithm used for provisioning.
 */
typedef enum
{
    NRF_MESH_PROV_ALGORITHM_FIPS_P256,   /**< FIPS P-256 Elliptic Curve. */
    NRF_MESH_PROV_ALGORITHM_RFU          /**< Start value of reserved for the future range. */
} nrf_mesh_prov_algorithm_t;

/**
 * The Public Key kind of usage
 */
typedef enum
{
    NRF_MESH_PROV_PUBLIC_KEY_NO_OOB = 0x00, /**< No OOB Public Key is used. */
    NRF_MESH_PROV_PUBLIC_KEY_OOB    = 0x01, /**< OOB Public Key is used. */
    NRF_MESH_PROV_PUBLIC_KEY_PROHIBITED     /**< Start value of prohibited range. */
} nrf_mesh_prov_public_key_usage_t;

/**
 * Out-of-band authentication methods for provisioning.
 */
 typedef enum
 {
     NRF_MESH_PROV_OOB_METHOD_NONE   = 0x00, /**< No authentication method. */
     NRF_MESH_PROV_OOB_METHOD_STATIC = 0x01, /**< Static OOB authentication method. */
     NRF_MESH_PROV_OOB_METHOD_OUTPUT = 0x02, /**< Output OOB authentication method. */
     NRF_MESH_PROV_OOB_METHOD_INPUT  = 0x03, /**< Input OOB authentication method. */
     NRF_MESH_PROV_OOB_METHOD_PROHIBITED     /**< Start value of prohibited range. */
 } nrf_mesh_prov_oob_method_t;

 /**
  * Enumeration for the OOB input actions.
  */
 typedef enum
 {
     NRF_MESH_PROV_INPUT_ACTION_PUSH         = 0x00, /**< The user should do a push action as input action. */
     NRF_MESH_PROV_INPUT_ACTION_TWIST        = 0x01, /**< The user should do a twist action as input action. */
     NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER = 0x02, /**< The user should enter a number into the device as input action. */
     NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING = 0x03, /**< The user should enter a string into the device as input action. */
     NRF_MESH_PROV_INPUT_ACTION_RFU                  /**< Start value of reserved for the future range. */
 } nrf_mesh_prov_input_action_t;

 /**
  * Enumeration for the OOB output actions.
  */
 typedef enum
 {
     NRF_MESH_PROV_OUTPUT_ACTION_BLINK           = 0x00, /**< The device should use blinking as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_BEEP            = 0x01, /**< The device should use beeping as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE         = 0x02, /**< The device should vibrate as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC = 0x03, /**< The device should display a number as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC    = 0x04, /**< The device should display an alpha-numberic value as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_RFU                     /**< Start value of reserved for the future range. */
 } nrf_mesh_prov_output_action_t;

 /**
  * Provisioning failure codes.
  */
 typedef enum
 {
     NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU         = 0x01, /**< An invalid provisioning PDU was received. */
     NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT      = 0x02, /**< An incoming provisioning packet had an invalid format. */
     NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU      = 0x03, /**< The incoming packet was a different packet type than what was expected. */
     NRF_MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED = 0x04, /**< The OOB authentication between provisioner and provisionee failed. */
     NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES    = 0x05, /**< The device does not have enough resources (memory, CPU time) to complete the provisioning. */
     NRF_MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED   = 0x06, /**< The provisioning data could not be decrypted. */
     NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_ERROR    = 0x07, /**< An unexpected error occured. */
     NRF_MESH_PROV_FAILURE_CODE_CANNOT_ASSIGN_ADDR  = 0x08, /**< Consecutive unicast addresses could not be assigned. */
 } nrf_mesh_prov_failure_code_t;

/*lint -align_max(push) -align_max(1) */

/**
 * Provisioning authentication capabilities.
 * This structure is filled in with the preferred values for how authentication is to be performed.
 * Only one method can be chosen for each of input/output authentication.
 */
typedef struct __attribute((packed))
{
    uint8_t  num_elements;       /**< Number of elements in the device. */
    uint16_t algorithms;         /**< Supported authentication algorithms. */
    uint8_t  pubkey_type;        /**< Supported public key types. */
    uint8_t  oob_static_types;   /**< Supported static OOB types. */
    uint8_t  oob_output_size;    /**< Output OOB data size. */
    uint16_t oob_output_actions; /**< Supported output OOB actions. */
    uint8_t  oob_input_size;     /**< Input OOB data size. */
    uint16_t oob_input_actions;  /**< Supported input OOB actions. */
} nrf_mesh_prov_oob_caps_t;

/**
 * Provisioning state machine states.
 * These are used internally in the provisioning stack to run the provisioning protocol.
 */
typedef enum
{
    NRF_MESH_PROV_STATE_IDLE,                   /**< Idle state, no action is taken. */
    NRF_MESH_PROV_STATE_WAIT_LINK,              /**< Waiting for a link to be established. */
    NRF_MESH_PROV_STATE_INVITE,                 /**< Waiting for a provisioning invitation message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS,              /**< Waiting for a provisioning capabilities message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM,      /**< Waiting for the application to confirm the capabilities to use. */
    NRF_MESH_PROV_STATE_WAIT_START,             /**< Waiting for a provisioning start message. */
    NRF_MESH_PROV_STATE_WAIT_START_ACK,         /**< Waiting for the message acknowledgement for the start message. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY_ACK,       /**< Waiting for the public key to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY,           /**< Waiting for the peer node to send its public key. */
    NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY,       /**< Waiting for the OOB public key to be retrieved. */
    NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH,     /**< Waiting for the offloaded ECDH calculation to complete. */
    NRF_MESH_PROV_STATE_WAIT_OOB_INPUT,         /**< Waiting for OOB input. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC,        /**< Waiting for static OOB data. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD, /**< Waiting for static OOB data, confirmation already received. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION_ACK,  /**< Waiting for the confirmation to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION,      /**< Waiting for a provisioning confirmation message. */
    NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE,    /**< Waiting for an input complete message. */
    NRF_MESH_PROV_STATE_WAIT_RANDOM,            /**< Waiting for a provisioning random message. */
    NRF_MESH_PROV_STATE_WAIT_DATA,              /**< Waiting for the provisioning data message. */
    NRF_MESH_PROV_STATE_WAIT_COMPLETE,          /**< Waiting for the provisioning complete message. */
    NRF_MESH_PROV_STATE_COMPLETE,               /**< Provisioning complete state. */
    NRF_MESH_PROV_STATE_FAILED                  /**< Provisioning failed state. */
} nrf_mesh_prov_state_t;

/**
 * Provisioning data to transmit to a device.
 */
typedef struct __attribute((packed))
{
    /** Network key for the device. */
    uint8_t  netkey[NRF_MESH_KEY_SIZE];
    /** Network key index. */
    uint16_t netkey_index;
    /** IV_index value for the device, in little endian format. */
    uint32_t iv_index;
    /** Unicast address for the device. */
    uint16_t address;
    /** Flags. */
    struct __attribute((packed)) {
        /** IV update in progress flag. */
        uint8_t iv_update   : 1;
        /** Key refresh in progress flag. */
        uint8_t key_refresh : 1;
    } flags;
} nrf_mesh_prov_provisioning_data_t;

/*lint -align_max(pop) */

/** Provisioning role */
typedef enum
{
    NRF_MESH_PROV_ROLE_PROVISIONER, /**< The device will act as a provisioner, distributing provisioning data. */
    NRF_MESH_PROV_ROLE_PROVISIONEE  /**< The device will act as a provisionee, receiving provisioning data. */
} nrf_mesh_prov_role_t;

/**
 * Common provisioning context forward declaration.
 * @ingroup NRF_MESH_PROV_TYPES
 */
typedef struct nrf_mesh_prov_ctx nrf_mesh_prov_ctx_t;

/** @} end of NRF_MESH_PROV_TYPES */

#endif /* NRF_MESH_PROV_TYPES_H__ */
