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

#ifndef PROV_PDU_H__
#define PROV_PDU_H__

#include <stdint.h>
#include "nrf_mesh_defines.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_prov_types.h"

/**
 * @defgroup PROV_PDU Provisioning PDUs
 * @ingroup MESH_PROV
 * This module contains definitions of all the provisioning PDUs.
 * @{
 */

/** The largest provisioning PDU length. */
#define PROV_PDU_MAX_LENGTH (sizeof(prov_pdu_pubkey_t))

/** Length of provisioning data MIC */
#define PROV_PDU_DATA_MIC_LENGTH    (8)

/** Valid Provisioning data types */
typedef enum
{
    /** PDU type for the provisioning invitation PDU. */
    PROV_PDU_TYPE_INVITE,
    /** PDU type for the provisioning capabilities PDU. */
    PROV_PDU_TYPE_CAPABILITIES,
    /** PDU type for the provisioning start PDU. */
    PROV_PDU_TYPE_START,
    /** PDU type for the public key PDU. */
    PROV_PDU_TYPE_PUBLIC_KEY,
    /** PDU type for the input complete PDU. */
    PROV_PDU_TYPE_INPUT_COMPLETE,
    /** PDU type for the confirmation PDU. */
    PROV_PDU_TYPE_CONFIRMATION,
    /** PDU type for the provisioning random PDU. */
    PROV_PDU_TYPE_RANDOM,
    /** PDU type for the provisioning data PDU. */
    PROV_PDU_TYPE_DATA,
    /** PDU type for the provisioning complete PDU. */
    PROV_PDU_TYPE_COMPLETE,
    /** PDU type for the provisioning failed PDU. */
    PROV_PDU_TYPE_FAILED,
    /** Other values are invalid */
    PROV_PDU_TYPE_INVALID,
    /** Number of pdu types available, must be set to the last member. */
    PROV_PDU_TYPE_COUNT = PROV_PDU_TYPE_INVALID
} prov_pdu_type_t;

/** Value of the algorithm field in the start message. */
#define PROV_PDU_START_ALGO_FIPS_P256   0x00

/*lint -align_max(push) -align_max(1) */

/** Contents of the provisioning invite PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;               /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t attention_duration_s;   /**< Attention timer value in seconds. */
} prov_pdu_invite_t;

/** Contents of the provisioning capabilities PDU. */
typedef struct __attribute((packed))
{
    uint8_t  pdu_type;           /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t  num_elements;       /**< Number of elements */
    uint16_t algorithms;         /**< Supported authentication algorithms. */
    uint8_t  pubkey_type;        /**< Supported public key types. */
    uint8_t  oob_static_types;   /**< Supported static OOB types. */
    uint8_t  oob_output_size;    /**< Output OOB size. */
    uint16_t oob_output_actions; /**< Supported output OOB types. */
    uint8_t  oob_input_size;     /**< Input OOB size. */
    uint16_t oob_input_actions;  /**< Supported input OOB types. */
} prov_pdu_caps_t;

/** Contents of the provisioning start PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;           /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t algorithm;          /**< Authentication algorithm. */
    uint8_t public_key;         /**< OOB public key. */
    uint8_t auth_method;        /**< Authentication mode selected. */
    uint8_t auth_action;        /**< Authentication action. */
    uint8_t auth_size;          /**< Size of authentication data. */
} prov_pdu_prov_start_t;

/** Contents of the public key PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;       /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t public_key[NRF_MESH_ECDH_PUBLIC_KEY_SIZE]; /**< Public key. */
} prov_pdu_pubkey_t;

/** Contents of the provisioning confirmation PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;           /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t confirmation[16];   /**< Confirmation value. */
} prov_pdu_confirm_t;

/** Contents of the input complete PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;           /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
} prov_pdu_input_complete_t;

/** Provisioning data block. */
typedef struct __attribute((packed))
{
    uint8_t  netkey[NRF_MESH_KEY_SIZE]; /**< Network key. */
    uint16_t netkey_index;              /**< Network key index. */
    struct __attribute((packed))
    {
        uint8_t key_refresh : 1; /**< Key refresh flag. */
        uint8_t iv_update   : 1; /**< IV update flag. */
        uint8_t _rfu        : 6; /**< Reserved for future use. */
    } flags;                     /**< Flags. */
    uint32_t iv_index;           /**< IV index. */
    uint16_t address;            /**< Device address. */
} prov_pdu_data_block_t;

/** Contents of the provisioning data PDU. */
typedef struct __attribute((packed))
{
    uint8_t  pdu_type;          /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    prov_pdu_data_block_t data; /**< Provisioning data fields. */
    uint8_t  mic[PROV_PDU_DATA_MIC_LENGTH]; /**< MIC for the provisioning data. */
} prov_pdu_data_t;

/** Contents of the provisioning random PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;   /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t random[16]; /**< Provisioning random. */
} prov_pdu_random_t;

/** Contents of the provisioning complete PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;   /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
} prov_pdu_complete_t;

/** Contents of the provisioning failed PDU. */
typedef struct __attribute((packed))
{
    uint8_t pdu_type;     /**< Packet PDU type can be one of type @ref prov_pdu_type_t. */
    uint8_t failure_code; /**< Error code representing the error that occured. */
} prov_pdu_failed_t;

NRF_MESH_STATIC_ASSERT((sizeof(prov_pdu_invite_t)     - 1) +
                       (sizeof(prov_pdu_caps_t)       - 1) +
                       (sizeof(prov_pdu_prov_start_t) - 1) == PROV_CONFIRMATION_INPUT_LEN);

/*lint -align_max(pop) */

/** @} */

#endif

