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

#ifndef SERIAL_CMD_RSP_H__
#define SERIAL_CMD_RSP_H__

#include <stdint.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_dfu_types.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_prov.h"
#include "access.h"
#include "device_state_manager.h"


/**
 * @defgroup SERIAL_CMD_RSP Serial Command Response definitions
 * @ingroup MESH_SERIAL
 * @{
 */

/** Overhead of the command response event, before any data starts. */
#define SERIAL_EVT_CMD_RSP_OVERHEAD         (2)
/** Overhead of the command response event, before any data starts, including header overhead. */
#define SERIAL_EVT_CMD_RSP_LEN_OVERHEAD     (NRF_MESH_SERIAL_PACKET_OVERHEAD + SERIAL_EVT_CMD_RSP_OVERHEAD)
/** Max length of the command response data field. */
#define SERIAL_EVT_CMD_RSP_DATA_MAXLEN      (NRF_MESH_SERIAL_PAYLOAD_MAXLEN  - SERIAL_EVT_CMD_RSP_OVERHEAD)

/*lint -align_max(push) -align_max(1) */

/** Serial interface housekeeping data. */
typedef struct __attribute((packed))
{
    uint32_t alloc_fail_count;  /**< Number of failed serial packet allocations. */
} serial_evt_cmd_rsp_data_housekeeping_t;

/** Subnetwork access response data */
typedef struct __attribute((packed))
{
    uint16_t subnet_handle; /**< Subnetwork handle operated on. */
} serial_evt_cmd_rsp_data_subnet_t;

/** Subnetwork list response data */
typedef struct __attribute((packed))
{
    uint16_t subnet_key_index[SERIAL_EVT_CMD_RSP_DATA_MAXLEN / sizeof(uint16_t)]; /**< List of all subnetwork key indexes known by the device. */
} serial_evt_cmd_rsp_data_subnet_list_t;

/** Application key access response data */
typedef struct __attribute((packed))
{
    uint16_t appkey_handle; /**< Application key handle operated on. */
} serial_evt_cmd_rsp_data_appkey_t;

/** Application key list response data */
typedef struct __attribute((packed))
{
    uint16_t subnet_handle; /**< Handle of the Subnetwork associated with the application keys. */
    uint16_t appkey_key_index[(SERIAL_EVT_CMD_RSP_DATA_MAXLEN - sizeof(uint16_t)) / sizeof(uint16_t)]; /**< List of all application key indexes known by the device. */
} serial_evt_cmd_rsp_data_appkey_list_t;

/** Device key access response data */
typedef struct __attribute((packed))
{
    uint16_t devkey_handle; /**< Device key handle operated on. */
} serial_evt_cmd_rsp_data_devkey_t;

/** Address access response data */
typedef struct __attribute((packed))
{
    uint16_t address_handle; /**< Address handle operated on. */
} serial_evt_cmd_rsp_data_addr_t;

/** Unicast address access response data */
typedef struct __attribute((packed))
{
    uint16_t address_start; /**< First address in the range of unicast addresses. */
    uint16_t count;         /**< Number of addresses in the range of unicast addresses. */
} serial_evt_cmd_rsp_data_addr_local_unicast_t;

/** Raw address access response data */
typedef struct __attribute((packed))
{
    uint16_t address_handle; /**< Address handle requested. */
    uint8_t addr_type;       /**< Address type of the given address. See @ref nrf_mesh_address_type_t for accepted values. */
    uint8_t subscribed;      /**< Flag indicating whether the given address is subscribed to or not. */
    uint16_t raw_short_addr; /**< Raw representation of the address. */
    uint8_t virtual_uuid[NRF_MESH_UUID_SIZE]; /**< Optional virtual UUID of the given address. */
} serial_evt_cmd_rsp_data_raw_addr_t;

/** Address handle list response data */
typedef struct __attribute((packed))
{
    uint16_t address_handles[SERIAL_EVT_CMD_RSP_DATA_MAXLEN / sizeof(uint16_t)]; /**< List of all address handles known by the device, not including local unicast addresses. */
} serial_evt_cmd_rsp_data_addr_list_t;

/** Command response data with a list size. */
typedef struct __attribute((packed))
{
    uint16_t list_size; /**< Size of the list requested by the command. */
} serial_evt_cmd_rsp_data_list_size_t;

/** Command response data with context information. */
typedef struct __attribute((packed))
{
    uint8_t context; /**< Provisioning context ID */
} serial_evt_cmd_rsp_data_prov_ctx_t;

/** Command response data with version information. */
typedef struct __attribute((packed))
{
    uint16_t serial_ver; /**< Serial interface version. */
} serial_evt_cmd_rsp_data_serial_version_t;

/** Command response data with firmware info. */
typedef struct __attribute((packed))
{
    nrf_mesh_fwid_t fwid; /**< Firmware ID data. */
} serial_evt_cmd_rsp_data_firmware_info_t;

/** Advertisement address command response. */
typedef struct __attribute((packed))
{
    uint8_t addr_type; /**< Advertisement address type. */
    uint8_t addr[BLE_GAP_ADDR_LEN]; /**< Advertisement address. */
} serial_evt_cmd_rsp_data_adv_addr_t;

/**
 * Device UUID command response.
 */
typedef struct __attribute((packed))
{
    uint8_t device_uuid[NRF_MESH_UUID_SIZE]; /**< Device UUID. */
} serial_evt_cmd_rsp_data_device_uuid_t;


/** Command response data with TX power. */
typedef struct __attribute((packed))
{
    uint8_t tx_power; /**< TX Power value, must be a value from @ref serial_cmd_tx_power_value_t. */
} serial_evt_cmd_rsp_data_tx_power_t;

/** Beacon parameter command response. */
typedef struct __attribute((packed))
{
    uint8_t beacon_slot; /**< Slot number of the beacon to start. */
    uint8_t tx_power; /**< TX Power value, must be a value from @ref serial_cmd_tx_power_value_t. */
    uint8_t channel_map; /**< Channel map bitfield for beacon, starting at channel 37. */
    uint32_t interval_ms; /**< TX interval in milliseconds. */
} serial_evt_cmd_rsp_data_beacon_params_t;

/** Command response data with dfu bank information. */
typedef struct __attribute((packed))
{
    uint8_t             dfu_type;     /**< DFU type of the bank. */
    nrf_mesh_fwid_t     fwid;         /**< Firmware ID of the bank. */
    uint8_t             is_signed;    /**< Flag indicating whether the bank is signed with an encryption key. */
    uint32_t            start_addr;   /**< Start address of the bank. */
    uint32_t            length;       /**< Length of the firmware in the bank. */
} serial_evt_cmd_rsp_data_dfu_bank_info_t;

/** Command response data with dfu state. */
typedef struct __attribute((packed))
{
    uint8_t         role;          /**< This device's intended role in the transfer, see @ref nrf_mesh_dfu_role_t for accepted values. */
    uint8_t         type;          /**< The DFU type of the transfer, see @ref nrf_mesh_dfu_type_t for accepted values. */
    nrf_mesh_fwid_t fwid;          /**< The FWID of the new data in the transfer. */
    uint8_t         state;         /**< The current global state of the transfer, see @ref nrf_mesh_dfu_state_t for accepted values. */
    uint8_t         data_progress; /**< The progress of the transfer in percent (0-100). */
} serial_evt_cmd_rsp_data_dfu_state_t;

/** Command response data with address handle for the publish address. */
typedef struct __attribute((packed))
{
    dsm_handle_t addr_handle;      /**< Address handle for the publish address. */
} serial_evt_cmd_rsp_data_model_pub_addr_get_t;

/** Command response data with appkey handle of the application key used for publishing. */
typedef struct __attribute((packed))
{
    dsm_handle_t appkey_handle;    /**< Handle of the application key used for publishing. */
} serial_evt_cmd_rsp_data_model_pub_app_get_t;

/** Command response data with publish period information. */
typedef struct __attribute((packed))
{
    uint8_t resolution;  /**< Resolution of each step. */
    uint8_t step_number; /**< Number of steps in each period. */
} serial_evt_cmd_rsp_data_model_pub_period_get_t;

/** Command response to @ref SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET command with subscription address handles. */
typedef struct __attribute((packed))
{
    uint16_t count;  /**< Number of available handles in @c address_handles */
    dsm_handle_t address_handles[(SERIAL_EVT_CMD_RSP_DATA_MAXLEN-sizeof(uint16_t)) / sizeof(dsm_handle_t)]; /**< List of the address handles of all subscription addresses bound to the given model */
} serial_evt_cmd_rsp_data_model_subs_get_t;

/** Command response to @ref SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET with application key handles. */
typedef struct __attribute((packed))
{
    uint16_t count;  /**< Number of available handles in @c appkey_handles */
    dsm_handle_t appkey_handles[(SERIAL_EVT_CMD_RSP_DATA_MAXLEN-sizeof(uint16_t)) / sizeof(dsm_handle_t)]; /**< List of the address handles of all subscription addresses bound to the given model */
} serial_evt_cmd_rsp_data_model_apps_get_t;

/** Command response data with the publish ttl value. */
typedef struct __attribute((packed))
{
    uint8_t ttl;                /**< TTL for published messages. */
} serial_evt_cmd_rsp_data_model_pub_ttl_get_t;

/** Command response data with the element location info. */
typedef struct __attribute((packed))
{
    uint16_t location;          /**< Element location info. */
} serial_evt_cmd_rsp_data_elem_loc_get_t;

/** Command response data with the model count. */
typedef struct __attribute((packed))
{
    uint8_t model_count;        /**< Number of existing models. */
} serial_evt_cmd_rsp_data_elem_model_count_get_t;

/** Command response data with the model id. */
typedef struct __attribute((packed))
{
    access_model_id_t model_id; /**< Company and model IDs. */
} serial_evt_cmd_rsp_data_model_id_get_t;

/** Command response data with the model handle. */
typedef struct __attribute((packed))
{
    access_model_handle_t model_handle;  /**< Handle of the requested model. */
} serial_evt_cmd_rsp_data_model_handle_get_t;

/** Command response to @ref SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET with application key handles. */
typedef struct __attribute((packed))
{
    uint16_t count;  /**< Number of available handles in @c model_handles */
    access_model_handle_t model_handles[(SERIAL_EVT_CMD_RSP_DATA_MAXLEN-sizeof(uint16_t)) / sizeof(access_model_handle_t)]; /**< List of the address handles of all subscription addresses bound to the given model */
} serial_evt_cmd_rsp_data_elem_models_get_t;

/** Command response to @ref SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET with available model IDs. */
typedef struct __attribute((packed))
{
    uint16_t count;  /**< Number of available handles in @c model_ids */
    access_model_id_t model_ids[(SERIAL_EVT_CMD_RSP_DATA_MAXLEN-sizeof(uint16_t)) / sizeof(access_model_id_t)]; /**< List of the model ids of all the available models. */
} serial_evt_cmd_rsp_data_models_get_t;

/** Command response to @ref SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT with the reserved model handle. */
typedef struct __attribute((packed))
{
    access_model_handle_t model_handle;  /**< Handle of the initialized model. */
} serial_evt_cmd_rsp_data_model_init_t;

/** Command response to @ref SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND from the model addressed. */
typedef struct __attribute((packed))
{
    uint8_t data_len;                                               /**< Length of data array. Set to 0 to indicate no data to send */
    uint8_t data[SERIAL_EVT_CMD_RSP_DATA_MAXLEN - sizeof(uint8_t)]; /**< Command response data specific to each model. */
} serial_evt_cmd_rsp_data_model_cmd_t;

/** Command response to @ref SERIAL_OPCODE_CMD_MESH_PACKET_SEND with information about the sent packet. */
typedef struct __attribute((packed))
{
    nrf_mesh_tx_token_t token; /**< TX Token assigned to the packet. Can be used to resolve which packet a @ref SERIAL_OPCODE_EVT_MESH_TX_COMPLETE event refers to. */
} serial_evt_cmd_rsp_data_packet_send_t;

/** Command response to @ref SERIAL_OPCODE_CMD_MESH_NET_STATE_GET with the current net state */
typedef struct __attribute((packed))
{
    uint32_t  iv_index; /**< The current IV index. */
    uint8_t   iv_update_in_progress; /**< Value indicating the phase of the IV update process. */
    uint16_t  iv_update_timeout_counter; /**< Current value of timeout counter for IV update. */
    uint32_t  next_seqnum_block; /**< The start of the next unused sequence number block. */
} serial_evt_cmd_rsp_data_net_state_get_t;

/** Command response packet. */
typedef struct __attribute((packed))
{
    uint8_t opcode; /**< Opcode of original command. */
    uint8_t status; /**< Return status of the serial command. */
    union __attribute((packed))
    {
        serial_evt_cmd_rsp_data_housekeeping_t         hk_data;        /**< Housekeeping data response. */
        serial_evt_cmd_rsp_data_subnet_t               subnet;         /**< Subnet response. */
        serial_evt_cmd_rsp_data_subnet_list_t          subnet_list;    /**< List of all subnet key indexes. */
        serial_evt_cmd_rsp_data_appkey_t               appkey;         /**< Appkey response. */
        serial_evt_cmd_rsp_data_appkey_list_t          appkey_list;    /**< List of all appkey key indexes for a given subnetwork. */
        serial_evt_cmd_rsp_data_devkey_t               devkey;         /**< Devkey response. */
        serial_evt_cmd_rsp_data_addr_local_unicast_t   local_unicast;  /**< Local unicast addresses. */
        serial_evt_cmd_rsp_data_addr_t                 addr;           /**< Address response. */
        serial_evt_cmd_rsp_data_list_size_t            list_size;      /**< List size. */
        serial_evt_cmd_rsp_data_adv_addr_t             adv_addr;       /**< Advertisement address. */
        serial_evt_cmd_rsp_data_prov_ctx_t             prov_ctx;       /**< Provisioning context. */
        serial_evt_cmd_rsp_data_firmware_info_t        firmware_info;  /**< Firmware information. */
        serial_evt_cmd_rsp_data_serial_version_t       serial_version; /**< Serial version. */
        serial_evt_cmd_rsp_data_device_uuid_t          device_uuid;    /**< Device UUID. */
        serial_evt_cmd_rsp_data_beacon_params_t        beacon_params;  /**< Beacon parameters. */
        serial_evt_cmd_rsp_data_dfu_bank_info_t        dfu_bank_info;  /**< Bank information. */
        serial_evt_cmd_rsp_data_dfu_state_t            dfu_state;      /**< DFU state. */
        serial_evt_cmd_rsp_data_model_pub_addr_get_t   pub_addr;       /**< Model publish address. */
        serial_evt_cmd_rsp_data_model_pub_app_get_t    pub_app;        /**< Model publish application key. */
        serial_evt_cmd_rsp_data_model_pub_period_get_t pub_period;     /**< Model publish period. */
        serial_evt_cmd_rsp_data_model_subs_get_t       model_subs;     /**< Model subscription list. */
        serial_evt_cmd_rsp_data_model_apps_get_t       model_pub;      /**< Model application keys list. */
        serial_evt_cmd_rsp_data_model_pub_ttl_get_t    pub_ttl;        /**< Model publish ttl value. */
        serial_evt_cmd_rsp_data_elem_loc_get_t         elem_loc;       /**< Element location. */
        serial_evt_cmd_rsp_data_elem_model_count_get_t model_count;    /**< Number of models in the element. */
        serial_evt_cmd_rsp_data_model_id_get_t         model_id;       /**< Company and model IDs. */
        serial_evt_cmd_rsp_data_model_handle_get_t     model_handle;   /**< Handle for the model */
        serial_evt_cmd_rsp_data_elem_models_get_t      model_handles;  /**< Element's list of model handles. */
        serial_evt_cmd_rsp_data_models_get_t           model_ids;      /**< All the available models.*/
        serial_evt_cmd_rsp_data_model_init_t           model_init;     /**< Reserved handle for the initialized model instance. */
        serial_evt_cmd_rsp_data_packet_send_t          packet_send;    /**< Information about the sent packet. */
        serial_evt_cmd_rsp_data_net_state_get_t        net_state_get;  /**< Net state. */

    } data; /**< Optional command response data. */
} serial_evt_cmd_rsp_t;

/*lint -align_max(pop) */

/** @} */

#endif /* SERIAL_CMD_RSP_H__ */
