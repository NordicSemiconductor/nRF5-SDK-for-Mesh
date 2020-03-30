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

#ifndef BL_IF_H__
#define BL_IF_H__

#include <stdint.h>
#include "nrf_mesh_dfu_types.h"
#include "dfu_types_internal.h"
#include "bl_info_types.h"

/**
 * @defgroup BL_IF Bootloader interface
 * @ingroup DFU_TYPES_INTERNAL
 * Internal Bootloader iterface, used to communicate with the shared bootloader core for all
 * side-by-side DFU procedures.
 * @{
 */

/** Version of the bootloader interface. */
#define BL_IF_VERSION   (1)

/** TX Repeat count representing infinite number of repeats. */
#define BL_IF_TX_REPEATS_INF  (0xFF)

/** Maximum length of the echo packet's str field. */
#define BL_IF_ECHO_MAXLEN     (16)

/** Bootloader command types. */
typedef enum
{
    /* General opcodes */
    BL_CMD_TYPE_INIT = 0x00,                 /**< Initialize the bootloader interface. */
    BL_CMD_TYPE_ENABLE,                      /**< Enable the bootloader. */
    BL_CMD_TYPE_RX,                          /**< Forward a received DFU packet to the bootloader. */
    BL_CMD_TYPE_TIMEOUT,                     /**< Notify the bootloader that a timeout has fired. */
    BL_CMD_TYPE_ECHO,                        /**< Send an echo command to the bootloader to test the interface. */

    /* DFU */
    BL_CMD_TYPE_DFU_START_TARGET = 0x20,     /**< Start DFU target behavior. */
    BL_CMD_TYPE_DFU_START_RELAY,             /**< Start DFU relay behavior. */
    BL_CMD_TYPE_DFU_START_SOURCE,            /**< Start DFU source behavior. */
    BL_CMD_TYPE_DFU_ABORT,                   /**< Abort ongoing DFU behavior, and go back to idle. */
    BL_CMD_TYPE_DFU_BANK_FLASH,              /**< Tell the bootloader to flash a bank. App will restart afterwards. */
    BL_CMD_TYPE_DFU_BANK_INFO_GET,           /**< Get info on the bank of the given type. */

    /* info opcodes */
    BL_CMD_TYPE_INFO_GET = 0x30,             /**< Get an info entry from the device page. */
    BL_CMD_TYPE_INFO_PUT,                    /**< Put an info entry into the device page. */
    BL_CMD_TYPE_INFO_ERASE,                  /**< Erase an info entry from the device page. */

    /* uECC opcodes */
    BL_CMD_TYPE_UECC_SHARED_SECRET = 0x40,   /**< Calculate an uECC shared secret. */
    BL_CMD_TYPE_UECC_MAKE_KEY,               /**< Make a uECC public/private key pair. */
    BL_CMD_TYPE_UECC_VALID_PUBLIC_KEY,       /**< Check whether a given uECC public key is valid. */
    BL_CMD_TYPE_UECC_COMPUTE_PUBLIC_KEY,     /**< Compute the uECC public key of a given private key. */
    BL_CMD_TYPE_UECC_SIGN,                   /**< Sign a given data blob. */
    BL_CMD_TYPE_UECC_VERIFY,                 /**< Verify the signature of a given data blob. */

    /* Flash status */
    BL_CMD_TYPE_FLASH_WRITE_COMPLETE = 0x60, /**< Notify the bootloader that a flash write operation finished. */
    BL_CMD_TYPE_FLASH_ERASE_COMPLETE,        /**< Notify the bootloader that a flash erase operation finished. */
    BL_CMD_TYPE_FLASH_ALL_COMPLETE,          /**< Notify the bootloader that all ongoing flash operations finished. */
} bl_cmd_type_t;

/** Bootloader event types. */
typedef enum
{
    /* Generic */
    BL_EVT_TYPE_ECHO = 0x00,                /**< Response to an echo command. */
    BL_EVT_TYPE_ERROR,                      /**< Request to reset a previously requested timer. */
    BL_EVT_TYPE_BANK_AVAILABLE,             /**< A DFU has been received and banked, and is available for flashing. */

    /* DFU */
    BL_EVT_TYPE_DFU_ABORT = 0x20,           /**< DFU aborted, and went back to idle. */
    BL_EVT_TYPE_DFU_NEW_FW,                 /**< A neighbor device has a newer version of our firmware. */
    BL_EVT_TYPE_DFU_REQ,                    /**< Request to participate in a transfer. */
    BL_EVT_TYPE_DFU_START,                  /**< The transfer started. */
    BL_EVT_TYPE_DFU_DATA_SEGMENT_RX,        /**< The DFU module got a new packet in the ongoing DFU transfer. */
    BL_EVT_TYPE_DFU_END,                    /**< The transfer ended. */

    /* Flash */
    BL_EVT_TYPE_FLASH_ERASE = 0x50,         /**< Request to erase a flash section. */
    BL_EVT_TYPE_FLASH_WRITE,                /**< Request to write to a flash section. */

    /* TX */
    BL_EVT_TYPE_TX_RADIO = 0x60,            /**< Request to transmit a packet over the radio. */
    BL_EVT_TYPE_TX_SERIAL,                  /**< Request to transmit a packet over the serial connection. */
    BL_EVT_TYPE_TX_ABORT,                   /**< Request to stop transmitting a given packet. */

    /* Timer */
    BL_EVT_TYPE_TIMER_SET = 0x70,           /**< Request to start a timer. */
    BL_EVT_TYPE_TIMER_ABORT,                /**< Request to abort a running timer. */
} bl_evt_type_t;

/** Radio time interval types ordered by the bootloader. */
typedef enum
{
    BL_RADIO_INTERVAL_TYPE_EXPONENTIAL,  /**< Exponentially increasing intervals. */
    BL_RADIO_INTERVAL_TYPE_REGULAR,      /**< Regular intervals. */
    BL_RADIO_INTERVAL_TYPE_REGULAR_SLOW, /**< Regular, long intervals. */
} bl_radio_interval_type_t;

/** Flash operation types. */
typedef enum
{
    FLASH_OP_TYPE_NONE  = 0x00, /**< No operation type. */
    FLASH_OP_TYPE_WRITE = 0x01, /**< Flash write operation. */
    FLASH_OP_TYPE_ERASE = 0x02, /**< Flash erase operation. */
    FLASH_OP_TYPE_ALL   = 0x03  /**< All flash operations. */
} flash_op_type_t;

/** uECC curve types. */
typedef enum
{
    UECC_CURVE_SECP160R1, /**< SECP160R1 curve. */
    UECC_CURVE_SECP192R1, /**< SECP192R1 curve. */
    UECC_CURVE_SECP224R1, /**< SECP224R1 curve. */
    UECC_CURVE_SECP256R1, /**< SECP256R1 curve. */
    UECC_CURVE_SECP256K1, /**< SECP256K1 curve. */
} uECC_curve_t;

typedef struct bl_evt bl_evt_t; /**< Forward declaration of @c bl_evt_t. */
typedef struct bl_cmd bl_cmd_t; /**< Forward declaration of @c bl_cmd_t. */

/** Bootloader event callback function pointer type. */
typedef uint32_t (*bl_if_cb_evt_t)(const bl_evt_t* p_bl_evt);
/** Bootloader command callback function pointer type. */
typedef uint32_t (*bl_if_cmd_handler_t)(const bl_cmd_t* p_bl_cmd);


/****** Command parameters ******/

/** Init command parameters, used with @ref BL_CMD_TYPE_INIT. */
typedef struct
{
    uint8_t bl_if_version;         /**< Bootloader interface version. */
    bl_if_cb_evt_t event_callback; /**< Application side event callback function pointer. */
    uint8_t timer_count;           /**< Number of timer slots available to the bootloader. */
    uint8_t tx_slots;              /**< Number of radio transmission slots available to the bootloader. */
    bool in_app;                   /**< Whether we're running in application or not. */
} bl_cmd_init_t;

/** RX command parameters, used with @ref BL_CMD_TYPE_RX. */
typedef struct
{
    const nrf_mesh_dfu_packet_t* p_dfu_packet; /**< Pointer to DFU packet that was received. */
    uint32_t length;                           /**< Length of the given DFU packet. */
} bl_cmd_rx_t;

/** Timeout command parameters, used with @ref BL_CMD_TYPE_TIMEOUT. */
typedef struct
{
    uint8_t timer_index; /**< Slot index of the timer that fired. */
} bl_cmd_timeout_t;

/** Echo command parameters, used with @ref BL_CMD_TYPE_ECHO. */
typedef struct
{
    char str[BL_IF_ECHO_MAXLEN]; /**< Data to echo back. */
} bl_cmd_echo_t;

/* DFU commands */

/** DFU start target command parameters, used with @ref BL_CMD_TYPE_DFU_START_TARGET. */
typedef struct
{
    uint8_t type;                     /**< DFU type of the desired transfer. */
    nrf_mesh_fwid_t     fwid;         /**< FWID of the DFU transfer to request. */
    const uint32_t *    p_bank_start; /**< Pointer to start of transfer bank. */
} bl_cmd_dfu_start_target_t;

/** DFU start relay command parameters, used with @ref BL_CMD_TYPE_DFU_START_RELAY. */
typedef struct
{
    nrf_mesh_dfu_type_t type;           /**< DFU type of the desired transfer. */
    nrf_mesh_fwid_t     fwid;           /**< FWID of the DFU transfer to relay. */
    uint32_t            transaction_id; /**< Transaction ID of the transfer to relay. Set to 0 if unknown */
} bl_cmd_dfu_start_relay_t;

/** DFU start commands. */
typedef union
{
    bl_cmd_dfu_start_target_t target; /**< Start target parameters. */
    bl_cmd_dfu_start_relay_t  relay;  /**< Start relay parameters. */
} bl_cmd_dfu_start_t;

/** DFU bank flash command parameters, used with @ref BL_CMD_TYPE_DFU_BANK_FLASH. */
typedef struct
{
    nrf_mesh_dfu_type_t     bank_dfu_type; /**< There's only ever one bank of each DFU type. Specify which bank should be flashed. */
} bl_cmd_dfu_bank_flash_t;

/** DFU bank info get command parameters, used with @ref BL_CMD_TYPE_DFU_BANK_INFO_GET. */
typedef struct
{
    nrf_mesh_dfu_type_t         bank_dfu_type; /**< There's only ever one bank of each DFU type. Specify which bank to get. */
    nrf_mesh_dfu_bank_info_t*   p_bank_info;   /**< Pointer to structure to fill with the bank info. */
} bl_cmd_dfu_bank_info_get_t;

/** DFU command parameters. */
typedef union
{
    bl_cmd_dfu_start_t          start;         /**< DFU start parameters. */
    bl_cmd_dfu_bank_flash_t     bank_flash;    /**< DFU bank flash parameters. */
    bl_cmd_dfu_bank_info_get_t  bank_info_get; /**< DFU bank info get parameters. */
} bl_cmd_dfu_t;

/* Info commands */

/** Info get command parameters, used with @ref BL_CMD_TYPE_INFO_GET. */
typedef struct
{
    bl_info_type_t      type;    /**< Info type to get from the device page. */
    bl_info_entry_t *   p_entry; /**< Pointer to info entry structure to fill with the resulting entry. */
} bl_cmd_info_get_t;

/** Info put command parameters, used with @ref BL_CMD_TYPE_INFO_PUT. */
typedef struct
{
    bl_info_type_t          type;    /**< Info type to put in the device page. */
    const bl_info_entry_t * p_entry; /**< Pointer to info entry structure to put in the device page. */
    uint32_t                length;  /**< Length of the given info entry in bytes. */
} bl_cmd_info_put_t;

/** Info erase command parameters, used with @ref BL_CMD_TYPE_INFO_ERASE. */
typedef struct
{
    bl_info_type_t type; /**< Info type to erase from the device page. */
} bl_cmd_info_erase_t;

/** Info command parameters. */
typedef union
{
    bl_cmd_info_get_t   get;   /**< Info get parameters. */
    bl_cmd_info_put_t   put;   /**< Info put parameters.*/
    bl_cmd_info_erase_t erase; /**< Info erase parameters. */
} bl_cmd_info_t;

/* uECC commands */

/** uECC shared secret command, used with @ref BL_CMD_TYPE_UECC_SHARED_SECRET. */
typedef struct
{
    const uint8_t*  p_public_key;  /**< Pointer to the public key to use for the shared secret calculation. */
    const uint8_t*  p_private_key; /**< Pointer to the private key to use for the shared secret calculation. */
    uint8_t*        p_secret;      /**< Pointer to a byte array to store the shared secret in. */
    uECC_curve_t    curve;         /**< uECC curve to use for the shared secret calculation. */
} bl_cmd_uecc_shared_secret_t;

/** uECC make key command, used with @ref BL_CMD_TYPE_UECC_MAKE_KEY. */
typedef struct
{
    uint8_t*        p_public_key;  /**< Pointer to a byte array to store the calculated public key in. */
    uint8_t*        p_private_key; /**< Pointer to a byte array to store the calculated private key in. */
    uECC_curve_t    curve;         /**< uECC curve to use for the key calculation. */
} bl_cmd_uecc_make_key_t;

/** uECC valid public key command, used with @ref BL_CMD_TYPE_UECC_VALID_PUBLIC_KEY. */
typedef struct
{
    const uint8_t*  p_public_key; /**< Pointer to a public key to validate. */
    uECC_curve_t    curve;        /**< uECC curve to use for the validation. */
} bl_cmd_uecc_valid_public_key_t;

/** uECC compute public key command, used with @ref BL_CMD_TYPE_UECC_COMPUTE_PUBLIC_KEY. */
typedef struct
{
    const uint8_t*  p_private_key; /**< Pointer to the private key to calculate a public key for. */
    uint8_t*        p_public_key;  /**< Pointer to a byte array to put the calculated public key in. */
    uECC_curve_t    curve;         /**< uECC curve to use for the public key calculation. */
} bl_cmd_uecc_compute_public_key_t;

/** uECC sign command parameters, used with @ref BL_CMD_TYPE_UECC_SIGN. */
typedef struct
{
    const uint8_t*  p_private_key; /**< Pointer to the private key to be used for signing. */
    const uint8_t*  p_hash;        /**< Pointer to the hash to sign. */
    uint32_t        hash_size;     /**< Size of the hash. */
    uint8_t*        p_signature;   /**< Pointer to a byte array to store the calculated signature in. */
    uECC_curve_t    curve;         /**< uECC curve to use for the signing. */
} bl_cmd_uecc_sign_t;

/** uECC verify command parameters, used with @ref BL_CMD_TYPE_UECC_VERIFY. */
typedef struct
{
    const uint8_t * p_public_key;  /**< Pointer to the public key to use for verifying the given signature. */
    const uint8_t * p_hash;        /**< Pointer to the hash to verify the signature of. */
    uint32_t        hash_size;     /**< Size of the hash. */
    const uint8_t * p_signature;   /**< Pointer to the signature to verify. */
    uECC_curve_t    curve;         /**< uECC curve to use for the signature verification. */
} bl_cmd_uecc_verify_t;

/** uECC command parameters. */
typedef union
{
    bl_cmd_uecc_make_key_t           make_key;           /**< Make key parameters. */
    bl_cmd_uecc_valid_public_key_t   valid_public_key;   /**< Valid public key parameters. */
    bl_cmd_uecc_compute_public_key_t compute_public_key; /**< Compute public key parameters. */
    bl_cmd_uecc_sign_t               sign;               /**< Sign parameters. */
    bl_cmd_uecc_verify_t             verify;             /**< Verify parameters. */
} bl_cmd_uecc_t;

/* Flash commands */

/** Flash write command parameters, used with @ref BL_CMD_TYPE_FLASH_WRITE_COMPLETE. */
typedef struct
{
    const void * p_data; /**< Pointer to the source data that was written. */
} bl_cmd_flash_write_t;

/** Flash erase command parameters, used with @ref BL_CMD_TYPE_FLASH_ERASE_COMPLETE. */
typedef struct
{
    const void * p_dest; /**< Pointer to the flash data that was erased. */
} bl_cmd_flash_erase_t;

/** Flash command parameters. */
typedef union
{
    bl_cmd_flash_write_t write; /**< Write parameters. */
    bl_cmd_flash_erase_t erase; /**< Erase parameters. */
} bl_cmd_flash_t;

/** Bootloader command structure. */
struct bl_cmd
{
    bl_cmd_type_t type;           /**< Bootloader command type. */
    /** Union of all parameter structures used for the various bootloader commands. */
    union
    {
        bl_cmd_init_t    init;    /**< Init command parameters. */
        bl_cmd_rx_t      rx;      /**< RX command parameters. */
        bl_cmd_timeout_t timeout; /**< Timeout command parameters. */
        bl_cmd_echo_t    echo;    /**< Echo command parameters. */
        bl_cmd_dfu_t     dfu;     /**< DFU command parameters. */
        bl_cmd_info_t    info;    /**< Info command parameters. */
        bl_cmd_uecc_t    uecc;    /**< uECC command parameters. */
        bl_cmd_flash_t   flash;   /**< Flash command parameters. */
    } params;
};


/****** Event parameters ******/

/** Echo event parameters, used with @ref BL_EVT_TYPE_ECHO. */
typedef struct
{
    char str[BL_IF_ECHO_MAXLEN]; /**< Copy of data sent by the echo command. */
} bl_evt_echo_t;

/** Error event parameters, used with @ref BL_EVT_TYPE_ERROR. */
typedef struct
{
    uint32_t     error_code; /**< Error code produced. */
    const char * p_file;     /**< Filename where the error occured, may be NULL. */
    uint32_t     line;       /**< Line number where the error occured. */
} bl_evt_error_t;

/* DFU Events */

/** DFU request event parameters, used with @ref BL_EVT_TYPE_DFU_REQ. */
typedef struct
{
    nrf_mesh_dfu_role_t  role;           /**< Requested DFU role. */
    nrf_mesh_dfu_state_t state;          /**< Current DFU state. */
    nrf_mesh_dfu_type_t  dfu_type;       /**< DFU type of requested transfer. */
    nrf_mesh_fwid_t      fwid;           /**< FWID of requested transfer. */
    uint8_t              authority;      /**< Current authority of requested transfer. */
    uint32_t             transaction_id; /**< Transaction ID of trequested transfer. */
} bl_evt_dfu_req_t;

/** DFU new firmware event parameters, used with @ref BL_EVT_TYPE_DFU_NEW_FW. */
typedef struct
{
    nrf_mesh_dfu_state_t state;   /**< Current DFU state. */
    nrf_mesh_dfu_type_t  fw_type; /**< DFU type of new firmware. */
    nrf_mesh_fwid_t      fwid;    /**< FWID of new firmware. */
} bl_evt_dfu_new_fw_t;

/** DFU start event parameters, used with @ref BL_EVT_TYPE_DFU_START. */
typedef struct
{
    nrf_mesh_dfu_role_t role;     /**< Role in current transfer. */
    nrf_mesh_dfu_type_t dfu_type; /**< DFU type of current transfer. */
    nrf_mesh_fwid_t     fwid;     /**< FWID of current transfer. */
} bl_evt_dfu_start_t;

/** DFU data segment event parameters, used with @ref BL_EVT_TYPE_DFU_DATA_SEGMENT_RX. */
typedef struct
{
    uint16_t received_segment; /**< Segment number of received data segment. */
    uint16_t total_segments;   /**< Total number of fragments to be received. */
} bl_evt_dfu_data_segment_t;

/** DFU end event parameters, used with @ref BL_EVT_TYPE_DFU_END. */
typedef struct
{
    nrf_mesh_dfu_role_t role;     /**< Role in ended transfer. */
    nrf_mesh_dfu_type_t dfu_type; /**< DFU type of ended transfer. */
    nrf_mesh_fwid_t     fwid;     /**< FWID of ended transfer. */
} bl_evt_dfu_end_t;

/** DFU abort event parameters, used with @ref BL_EVT_TYPE_DFU_ABORT. */
typedef struct
{
    nrf_mesh_dfu_end_t reason; /**< Reason for aborting the ongoing transfer. */
} bl_evt_dfu_abort_t;

/** DFU event parameters. */
typedef union
{
    bl_evt_dfu_req_t          req;          /**< DFU request parameters. */
    bl_evt_dfu_new_fw_t       new_fw;       /**< DFU new firmware parameters. */
    bl_evt_dfu_start_t        start;        /**< DFU start parameters. */
    bl_evt_dfu_data_segment_t data_segment; /**< DFU data segment parameters. */
    bl_evt_dfu_end_t          end;          /**< DFU end parameters. */
    bl_evt_dfu_abort_t        abort;        /**< DFU abort parameters. */
} bl_evt_dfu_t;

/* Bank events */

/** Bank available event parameters, used with @ref BL_EVT_TYPE_BANK_AVAILABLE. */
typedef struct
{
    nrf_mesh_dfu_type_t     bank_dfu_type; /**< DFU type of available bank. */
    bool                    is_signed;     /**< Whether the bank is signed. */
    nrf_mesh_fwid_t         bank_fwid;     /**< Firmware ID of the bank. */
    nrf_mesh_fwid_t         current_fwid;  /**< Current FWID of the firmware with the same type as the bank. */
    const uint32_t *        p_bank_addr;   /**< Pointer to the bank data. */
    uint32_t                bank_length;   /**< Length of the bank data. */
} bl_evt_bank_available_t;

/* Flash events */

/** Flash erase event parameters, used with @ref BL_EVT_TYPE_FLASH_ERASE. */
typedef struct
{
    uint32_t *  p_start_addr; /**< Start of section to erase. */
    uint32_t    length;       /**< Length of section to erase. */
} bl_evt_flash_erase_t;

/** Flash write event parameters, used with @ref BL_EVT_TYPE_FLASH_WRITE. */
typedef struct
{
    uint32_t * p_start_addr; /**< Start of section to write to. */
    const uint8_t *  p_data;       /**< Data to write to flash. */
    uint32_t         length;       /**< Length of data to write to flash. */
} bl_evt_flash_write_t;

/** Flash event parameters. */
typedef union
{
    bl_evt_flash_erase_t erase; /**< Flash erase parameters. */
    bl_evt_flash_write_t write; /**< Flash write parameters. */
} bl_evt_flash_t;

/* TX events */

/** TX radio event parameters, used with @ref BL_EVT_TYPE_TX_RADIO. */
typedef struct
{
    const nrf_mesh_dfu_packet_t * p_dfu_packet;  /**< Pointer to DFU packet to send on the radio. */
    uint32_t                      length;        /**< Length of the given DFU packet. */
    bl_radio_interval_type_t      interval_type; /**< Interval type to use for the transmission timing. */
    uint8_t                       tx_count;      /**< Number of transmits to perform. */
    uint8_t                       tx_slot;       /**< TX slot to use for the transmission. */
} bl_evt_tx_radio_t;

/** TX serial event parameters, used with @ref BL_EVT_TYPE_TX_SERIAL. */
typedef struct
{
    const nrf_mesh_dfu_packet_t * p_dfu_packet; /**< Pointer to DFU packet to send over serial. */
    uint32_t                      length;       /**< Length of given DFU packet. */
} bl_evt_tx_serial_t;

/** TX abort event parameters, used with @ref BL_EVT_TYPE_TX_ABORT. */
typedef struct
{
    uint8_t         tx_slot; /**< Radio transmission slot to abort the transfer of. */
} bl_evt_tx_abort_t;

/** TX event parameters. */
typedef union
{
    bl_evt_tx_radio_t  radio;  /**< TX Radio parameters. */
    bl_evt_tx_serial_t serial; /**< TX serial parameters. */
    bl_evt_tx_abort_t  abort;  /**< TX abort parameters. */
} bl_evt_tx_t;

/* Timer events */

/** Timer set event parameters, used with @ref BL_EVT_TYPE_TIMER_SET. */
typedef struct
{
    uint32_t index;    /**< Timer index to order timer on. */
    uint32_t delay_us; /**< Microseconds of delay before the timer should fire. */
} bl_evt_timer_set_t;

/** Timer abort event parameters, used with @ref BL_EVT_TYPE_TIMER_ABORT. */
typedef struct
{
    uint32_t index; /**< Index of the timer to abort. */
} bl_evt_timer_abort_t;

/** Timer event parameters. */
typedef union
{
    bl_evt_timer_set_t set;     /**< Timer set parameters. */
    bl_evt_timer_abort_t abort; /**< Timer abort parameters. */
} bl_evt_timer_t;

/** Bootloader event structure. */
struct bl_evt
{
    bl_evt_type_t type; /**< Bootloader event type. */
    /** Union of all parameter structures used for the various bootloader events. */
    union
    {
        bl_evt_echo_t           echo;           /**< Echo response event parameters. */
        bl_evt_error_t          error;          /**< Error event parameters. */
        bl_evt_dfu_t            dfu;            /**< DFU event parameters. */
        bl_evt_flash_t          flash;          /**< Flash event parameters. */
        bl_evt_bank_available_t bank_available; /**< Bank available event parameters. */
        bl_evt_tx_t             tx;             /**< TX event parameters. */
        bl_evt_timer_t          timer;          /**< Timer event parameters. */
    } params;
};

/** @} */

#endif /* BL_IF_H__ */

