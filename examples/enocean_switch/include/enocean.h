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

#ifndef ENOCEAN_H__
#define ENOCEAN_H__

#include "nrf_mesh.h"
#include "list.h"

/**
 * @defgroup ENCOCEAN_H EnOcean switch translator module
 *
 * This module provides basic APIs to enable integration of the EnOcean switch (PTM215B) in the Mesh
 * eco system. It supports the BLE communication with the switch, when switch is configured to use
 * public GAP address (default setting for the switch).
 *
 * Initialize the module by calling @ref enocean_translator_init() API and provide application
 * callback to be called when EnOcean packet is received. Also provide previously saved security
 * material required for decoding EnOcean data packets by calling @ref enocean_secmat_add() API.
 *
 * To process the data and commissioning telegrams transmitted by the EnOcean switch, register
 * the application level scanner callback using @ref nrf_mesh_rx_cb_set() API. From this callback
 * forward @ref nrf_mesh_adv_packet_rx_data_t to this module by calling @ref enocean_packet_process()
 * API.
 *
 * When this module detects EnOcean data or commissioning telegram, it generates events listed in
 * @ref enocean_evt_types_t. If security material obtained from @ref ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED
 * is acceptable, it should be stored in the flash and should be supplied to the decoder. This
 * module does not have any persistence. Therfore upon device reboot, saved security material should
 * be supplied to this module by calling @ref enocean_secmat_add() API.
 *
 * @{
 */

/**
 * @defgroup ENOCEAN_DEFS EnOcean specific defines
 *
 * @{
 */

/** Length of the product short name field */
#define PTM215B_PRODUCT_SHORT_NAME_SIZE             (5)
/** Number of physical button contacts on PTM215B module */
#define PTM215B_NUMBER_OF_SWITCHES                  (4)
/** Value of the manufacturer ID field */
#define PTM215B_TELEGRAM_MANUFACTURER_ID            (0x03DA)
/** Max size of the EnOcean data packet */
#define PTM215B_DATA_PACKET_MAX_SIZE                (17)
/** Max value for AD Length for data packet */
#define PTM215B_DATA_AD_LEN_MAX_VALUE               (PTM215B_DATA_PACKET_MAX_SIZE - 1)
/** Value for AD Length for commissioning packet as per new format */
#define PTM215B_COMM_AD_LEN_VALUE                   (0x1D)
/** Data packet MIC size */
#define PTM215B_DATA_PACKET_MIC_SIZE                (4)
/** Offset for sequence number field in data packet */
#define PTM215B_DATA_PACKET_SEQ_COUNTER_OFFSET      (4)
/** Size of sequence number field */
#define PTM215B_DATA_PACKET_SEQ_COUNTER_SIZE        (4)
/** Offset for switch status field in data packet */
#define PTM215B_DATA_PACKET_SWITCH_STATUS_OFFSET    (8)
/** Size of switch status field */
#define PTM215B_DATA_PACKET_SWITCH_STATUS_SIZE      (1)
/** Offset for optional data field in data packet */
#define PTM215B_DATA_PACKET_OPTIONAL_DATA_OFFSET    (9)
/** Maximum size of optional data field */
#define PTM215B_DATA_PACKET_OPTIONAL_DATA_MAX_SIZE  (4)
/** Size of the encryption key field */
#define PTM215B_COMM_PACKET_KEY_SIZE                (16)
/** Offset for in commissioning packet */
#define PTM215B_COMM_PACKET_BLE_ADDR_OFFSET         (24)
/** RFU bits in the switch status */
#define PTM215B_SWITCH_STATUS_RFU_MASK              (0xE0)
/** Status field bit index for B1 switch */
#define PTM215B_SWITCH_STATUS_B1_BIT                (4)
/** Status field bit index for B0 switch */
#define PTM215B_SWITCH_STATUS_B0_BIT                (3)
/** Status field bit index for A1 switch */
#define PTM215B_SWITCH_STATUS_A1_BIT                (2)
/** Status field bit index for A0 switch */
#define PTM215B_SWITCH_STATUS_A0_BIT                (1)
/** Status field bit index for action indicator bit */
#define PTM215B_SWITCH_STATUS_ACTION_BIT            (0)

/** @} end of ENOCEAN_DEFS */

/**
 * @defgroup ENOCEAN_TYPES Type definitions
 *
 * @{
 */

/** Switch action types */
typedef enum
{
    /** Indicates release action */
    RELEASE_ACTION,
    /** Indicates press action */
    PRESS_ACTION
} enocean_switch_action_type_t;

/** Switch status data.
 * Value of true (1): indicates a press or release action happened on the switch
 * Value of false (0): indicates no action happened on the switch
 *
 * For PTM215B switches are named as follows:
 * ```
 * #####################
 * @ +-------+-------+ @
 * @ | 1(A0) | 3(B0) | @
 * @ |       |       | @
 * @ |       |       | @
 * @ | 2(A1) | 4(B1) | @
 * @ +---------------+ @
 * #####################
 * ```
 */
typedef struct
{
    /** Status for switch B0 */
    bool     b0;
    /** Status for switch B1 */
    bool     b1;
    /** Status for switch A0 */
    bool     a0;
    /** Status for switch A1 */
    bool     a1;
    /** Action that happened */
    enocean_switch_action_type_t action;
} enocean_switch_status_t;

/** Enocean module event types */
typedef enum
{
    /** Security material is received */
    ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED,
    /** Switch data is received */
    ENOCEAN_EVT_DATA_RECEIVED
} enocean_evt_types_t;

/** @ref ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED event data structure */
typedef struct
{
    /** Sequence number */
    uint32_t seq;
    /** Security key extracted from the commissioning telegram */
    const uint8_t * p_key;
} enocean_evt_commissioning_secmat_params_t;

/** @ref ENOCEAN_EVT_DATA_RECEIVED event data structure */
typedef struct
{
    /** Indicates status of four buttons */
    enocean_switch_status_t status;
    /** Pointer to optional data, if present */
    const uint8_t * p_optional_data;
    /** Length of the optional data, if present */
    uint8_t optional_data_length;
} enocean_evt_data_params_t;

/** Structure passed to the event callback by the EnOcean module */
typedef struct
{
    /** Enocean event type */
    enocean_evt_types_t type;
    /** BLE GAP address of the encoean device */
    const uint8_t * p_ble_gap_addr;
    union
    {
        /** Security material received event */
        enocean_evt_commissioning_secmat_params_t secmat;
        /** Switch data received event */
        enocean_evt_data_params_t data;
    } params;
} enocean_evt_t;

/**
 * Enocean translator callback
 *
 * @param[out] evt  structure containing event type and event parameters
 */
typedef void (*enocean_translator_cb_t)(enocean_evt_t * p_evt);

/** @ref ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED event data structure */
typedef struct
{
    /** Pointer to the BLE GAP address of the device */
    const uint8_t * p_ble_gap_addr;
    /** Pointer to the sequence number */
    uint32_t * p_seq;
    /** Pointer to the security key extracted from the commissioning telegram */
    const uint8_t * p_key;
    /** For internal use */
    list_node_t node;
} enocean_commissioning_secmat_t;

/** @} end of ENOCEAN_TYPES */


/**
 * @defgroup ENOCEAN_API EnOcean API
 * @{
 */

/**
 * Adds the given security material pointer to the list of security materials.
 *
 * @note: Structure pointed to by the p_secmat must be static allocated global.
 *
 * @param[in]  p_secmat     Pointer to the structure containing security material to be used for
 *                          authenticating received radio telegrams. This security material can be
 *                          obtained over BLE by entering radio commissioning mode.
 */
void enocean_secmat_add(enocean_commissioning_secmat_t * p_secmat);

/**
 * User application calls this function after receiving BLE Advertising packet.
 *
 * @param[in]  p_rx_data    Pointer to the scanner data structure to be used for extracting EnOcean
 *                          packets.
 */
void enocean_packet_process(const nrf_mesh_adv_packet_rx_data_t * p_rx_data);

/**
 * Initializes the EnOcean switch translator.
 *
 * @param[in]  callback     User application callback function pointer. This function will be called
 *                          when EnOcean telegrams (either data or commissioning) are received.
 */
void enocean_translator_init(enocean_translator_cb_t app_callback);

/** @} end of ENOCEAN_API */

/** @} end of ENCOCEAN_H */

#endif /* ENOCEAN_H__ */
