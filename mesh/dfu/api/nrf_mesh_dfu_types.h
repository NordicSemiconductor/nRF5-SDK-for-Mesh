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

#ifndef NRF_MESH_DFU_TYPES_H__
#define NRF_MESH_DFU_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh_assert.h"

/**
 * @defgroup NRF_MESH_DFU_DEFINES Defines
 * DFU definitions.
 * @{
 */

/** Length of a firmware public key. */
#define NRF_MESH_DFU_PUBLIC_KEY_LEN          (64)
/** Length of a firmware signature. */
#define NRF_MESH_DFU_SIGNATURE_LEN           (64)
/** Length of a dfu packet data segment. */
#define NRF_MESH_DFU_SEGMENT_LENGTH          (16)

/** @} */

/**
 * @defgroup NRF_MESH_DFU_TYPES Types
 * @ingroup NRF_MESH_DFU
 * DFU type definitions
 * @{
 */

/*lint -align_max(push) -align_max(1) */

/** Bootloader ID structure */
typedef struct __attribute((packed))
{
    /** Bootloader identificator. */
    uint8_t bl_id;
    /** Bootloader version number. */
    uint8_t bl_version;
} nrf_mesh_bootloader_id_t;

/**
 * Application ID structure.
 */
typedef struct __attribute((packed))
{
    /** Company ID. */
    uint32_t company_id;
    /** Application ID. */
    uint16_t app_id;
    /** Application version. */
    uint32_t app_version;
} nrf_mesh_app_id_t;

/** ID of a standalone firmware segment. */
typedef union __attribute((packed))
{
    nrf_mesh_app_id_t        application;   /**< Application ID. */
    nrf_mesh_bootloader_id_t bootloader;    /**< Bootloader ID. */
    uint16_t                 softdevice;    /**< Softdevice revision number. */
} nrf_mesh_fwid_t;

/*lint -align_max(pop) */

/** DFU transfer types. */
typedef enum
{
    /** Invalid transfer type. */
    NRF_MESH_DFU_TYPE_INVALID       = 0,
    /** Softdevice transfer. */
    NRF_MESH_DFU_TYPE_SOFTDEVICE    = (1 << 0),
    /** Bootloader transfer. */
    NRF_MESH_DFU_TYPE_BOOTLOADER    = (1 << 1),
    /** Application transfer. */
    NRF_MESH_DFU_TYPE_APPLICATION   = (1 << 2),
    /** Bootloader info transfer. */
    NRF_MESH_DFU_TYPE_BL_INFO       = (1 << 3),

    /** @internal Largest number in the enum. */
    NRF_MESH_DFU_TYPE__LAST         = NRF_MESH_DFU_TYPE_BL_INFO,
} nrf_mesh_dfu_type_t;

/** States of the DFU module. */
typedef enum
{
    NRF_MESH_DFU_STATE_UNINITIALIZED,    /**< The DFU module has not been initialized. */
    NRF_MESH_DFU_STATE_INITIALIZED,      /**< The DFU module has been initialized, but not started. */
    NRF_MESH_DFU_STATE_FIND_FWID,        /**< There's no DFU operation in progress. */
    NRF_MESH_DFU_STATE_DFU_REQ,          /**< Beaconing requests for transfers. */
    NRF_MESH_DFU_STATE_READY,            /**< Ready to receive a transfer. */
    NRF_MESH_DFU_STATE_TARGET,           /**< Receiving a transfer. */
    NRF_MESH_DFU_STATE_VALIDATE,         /**< Validating and finishing up a transfer. */
    NRF_MESH_DFU_STATE_STABILIZE,        /**< Waiting for metadata about validated transfer to be written. */
    NRF_MESH_DFU_STATE_RELAY_CANDIDATE,  /**< Beaconing intent to relay a transfer. */
    NRF_MESH_DFU_STATE_RELAY,            /**< Passively relaying a transfer. */

    /** @internal Largest number in the enum. */
    NRF_MESH_DFU_STATE__LAST = NRF_MESH_DFU_STATE_RELAY,
} nrf_mesh_dfu_state_t;

/** Reasons for a DFU operation to end. */
typedef enum
{
    NRF_MESH_DFU_END_SUCCESS,                               /**< The transfer ended successfully. */
    NRF_MESH_DFU_END_FWID_VALID,                            /**< The FWID was valid, and the bootloader stopped operation. */
    NRF_MESH_DFU_END_APP_ABORT,                             /**< The application requested to abort the transfer. */
    NRF_MESH_DFU_END_ERROR_PACKET_LOSS,                     /**< Too many packets were lost in the transfer. */
    NRF_MESH_DFU_END_ERROR_UNAUTHORIZED,                    /**< The signature check failed. */
    NRF_MESH_DFU_END_ERROR_NO_START,                        /**< Failed to receive the start packet. */
    NRF_MESH_DFU_END_ERROR_TIMEOUT,                         /**< Timed out waiting for packets. */
    NRF_MESH_DFU_END_ERROR_NO_MEM,                          /**< Not enough memory to handle transfer. */
    NRF_MESH_DFU_END_ERROR_INVALID_PERSISTENT_STORAGE,      /**< Device page contained invalid or corrupted data. */
    NRF_MESH_DFU_END_ERROR_SEGMENT_VIOLATION,               /**< The transfer fell outside its designated flash section. */
    NRF_MESH_DFU_END_ERROR_MBR_CALL_FAILED,                 /**< A call to the MBR failed. */
    NRF_MESH_DFU_END_ERROR_INVALID_TRANSFER,                /**< The transfer does not meet its requirements. */
    NRF_MESH_DFU_END_ERROR_BANK_IN_BOOTLOADER_AREA,         /**< The given bank address results in bootloader invalidation. */
    NRF_MESH_DFU_END_ERROR_BANK_AND_DESTINATION_OVERLAP,    /**< When copying the finished bank to its intended destination, it will have to overwrite itself. */

    /** @internal Largest number in the enum. */
    NRF_MESH_DFU_END_ERROR__LAST = NRF_MESH_DFU_END_ERROR_BANK_IN_BOOTLOADER_AREA,
} nrf_mesh_dfu_end_t;

/** The various roles a device can play in a dfu transfer. */
typedef enum
{
    NRF_MESH_DFU_ROLE_NONE,      /**< No role. */
    NRF_MESH_DFU_ROLE_TARGET,    /**< The target role. A receiver of a transfer. */
    NRF_MESH_DFU_ROLE_RELAY,     /**< The relay role. A passive forwarding role. */
    NRF_MESH_DFU_ROLE_SOURCE,    /**< The source role. The originator of a transfer. */

    /** @internal Largest number in the enum. */
    NRF_MESH_DFU_ROLE__LAST = NRF_MESH_DFU_ROLE_SOURCE,
} nrf_mesh_dfu_role_t;

/** DFU transfer information structure. */
typedef struct
{
    /** Type of transfer. */
    nrf_mesh_dfu_type_t dfu_type;
    /** Firmware target of transfer. */
    nrf_mesh_fwid_t id;
} nrf_mesh_dfu_transfer_t;

/** DFU Bank info structure. */
typedef struct
{
    nrf_mesh_dfu_type_t dfu_type;     /**< DFU type of the bank. */
    nrf_mesh_fwid_t     fwid;         /**< Firmware ID of the bank. */
    bool                is_signed;    /**< Flag indicating whether the bank is signed with an encryption key. */
    uint32_t*           p_start_addr; /**< Start address of the bank. */
    uint32_t            length;       /**< Length of the firmware in the bank. */
} nrf_mesh_dfu_bank_info_t;

/** Current state of a transfer. */
typedef struct
{
    nrf_mesh_dfu_role_t  role;          /**< This device's intended role in the transfer. */
    nrf_mesh_dfu_type_t  type;          /**< The DFU type of the transfer. */
    nrf_mesh_fwid_t      fwid;          /**< The FWID of the new data in the transfer. */
    nrf_mesh_dfu_state_t state;         /**< The current global state of the transfer. */
    uint8_t              data_progress; /**< The progress of the transfer in percent (0-100). */
} nrf_mesh_dfu_transfer_state_t;

/**@}*/


#endif /* NRF_MESH_DFU_TYPES_H__ */
