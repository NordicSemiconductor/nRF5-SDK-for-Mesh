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

#ifndef BL_INFO_TYPES_H__
#define BL_INFO_TYPES_H__

#include <stdint.h>
#include "nrf_mesh_dfu_types.h"

/**
 * @internal
 * @defgroup BL_INFO_TYPES Bootloader info types
 * @ingroup DFU_TYPES_INTERNAL
 * @{
 */

#define BL_INFO_LEN_PUBLIC_KEY      (NRF_MESH_DFU_PUBLIC_KEY_LEN) /**< Length of public key info entry. */
#define BL_INFO_LEN_SEGMENT         (sizeof(bl_info_segment_t))   /**< Length of segment info entry. */
#define BL_INFO_LEN_FWID            (sizeof(fwid_t))              /**< Length of FWID info entry. */
#define BL_INFO_LEN_FLAGS           (sizeof(bl_info_flags_t))     /**< Length of flags info entry. */
#define BL_INFO_LEN_SIGNATURE       (NRF_MESH_DFU_SIGNATURE_LEN)  /**< Length of signature info entry. */
#define BL_INFO_LEN_BANK_SIGNED     (sizeof(bl_info_bank_t))      /**< Length of signed bank info entry. */
#define BL_INFO_LEN_BANK            (BL_INFO_LEN_BANK_SIGNED - BL_INFO_LEN_SIGNATURE) /**< Length of unsigned bank info entry. */

/** Types of framework specified info entries */
typedef enum
{
    BL_INFO_TYPE_INVALID            = 0x00,   /**< Invalid info entry. */
    BL_INFO_TYPE_ECDSA_PUBLIC_KEY   = 0x01,   /**< Public key info entry. */
    BL_INFO_TYPE_VERSION            = 0x02,   /**< Version info entry. */
    BL_INFO_TYPE_FLAGS              = 0x04,   /**< Flags info entry. */

    BL_INFO_TYPE_SEGMENT_SD         = 0x10,   /**< Softdevice segment info entry. */
    BL_INFO_TYPE_SEGMENT_BL         = 0x11,   /**< Bootloader segment info entry. */
    BL_INFO_TYPE_SEGMENT_APP        = 0x12,   /**< Application segment info entry. */

    BL_INFO_TYPE_SIGNATURE_SD       = 0x1A,   /**< Softdevice signature info entry. */
    BL_INFO_TYPE_SIGNATURE_BL       = 0x1B,   /**< Bootloader signature info entry. */
    BL_INFO_TYPE_SIGNATURE_APP      = 0x1C,   /**< Application signature info entry. */
    BL_INFO_TYPE_SIGNATURE_BL_INFO  = 0x1D,   /**< Bootloader info signature info entry. */

    BL_INFO_TYPE_BANK_BASE          = 0x20,   /**< Only for adding offset to get the correct entry. */
    BL_INFO_TYPE_BANK_SD            = 0x21,   /**< Softdevice bank info entry. */
    BL_INFO_TYPE_BANK_BL            = 0x22,   /**< Bootloader bank info entry. */
    BL_INFO_TYPE_BANK_APP           = 0x24,   /**< Application bank info entry. */
    BL_INFO_TYPE_BANK_BL_INFO       = 0x28,   /**< Bootloader info bank info entry. */

    BL_INFO_TYPE_TEST               = 0x100,  /**< Test info entry for debugging. */

    BL_INFO_TYPE_LAST               = 0x7FFF, /**< Last info entry in the page. */
    BL_INFO_TYPE_UNUSED             = 0xFFFF, /**< Unused info entry. */
} bl_info_type_t;

/** Segment info entry parameters. */
typedef struct
{
    uint32_t start;  /**< Start address of the segment. */
    uint32_t length; /**< Length of the segment. */
} bl_info_segment_t;


/** Version info entry parameters. */
typedef fwid_t bl_info_version_t;

/** Flags info entry parameters. */
typedef struct
{
    bool sd_intact;       /**< Whether the Softdevice is intact. */
    bool bl_intact;       /**< Whether the bootloader is intact. */
    bool app_intact;      /**< Whether the application is intact. */
    bool page_is_invalid; /**< Whether the device page is intact. */
} bl_info_flags_t;

/**
 * State of info bank. Written to allow state machine progression being stored
 * in flash without needing erase.
 */
typedef enum
{
    BL_INFO_BANK_STATE_IDLE         = 0xFF, /**< The bank has not been touched since it got transferred. */
    BL_INFO_BANK_STATE_FLASH_FW     = 0xFE, /**< In the process of flashing the bank. */
    BL_INFO_BANK_STATE_FLASH_META   = 0xFC, /**< In the process of flashing metadata (signature and firmware) */
    BL_INFO_BANK_STATE_FLASHED      = 0xF8, /**< The bank has been flashed, and can be invalidated. */
} bl_info_bank_state_t;

/** Bank info entry parameters. */
typedef struct
{
    uint32_t*               p_bank_addr;                      /**< Pointer to bank data. */
    uint32_t                length;                           /**< Length of bank data. */
    nrf_mesh_fwid_t         fwid;                             /**< Firmware ID of the bank. */
    bool                    has_signature;                    /**< Whether the bank info entry has a signature. */
    bl_info_bank_state_t    state;                            /**< State of the bank. */
    uint8_t                 signature[BL_INFO_LEN_SIGNATURE]; /**< Optional signature of the bank. */
} bl_info_bank_t;

/** Union of all info entry parameters. */
typedef union
{
    uint8_t             public_key[BL_INFO_LEN_PUBLIC_KEY]; /**< Public key entry. */
    uint8_t             signature[BL_INFO_LEN_SIGNATURE];   /**< Signature entry. */
    bl_info_segment_t   segment;                            /**< Segment entry parameters. */
    bl_info_version_t   version;                            /**< Version entry parameters. */
    bl_info_flags_t     flags;                              /**< Flags entry parameters. */
    bl_info_bank_t      bank;                               /**< Bank entry parameters. */
} bl_info_entry_t;

/** @} */

#endif /* BL_INFO_TYPES_H__ */
