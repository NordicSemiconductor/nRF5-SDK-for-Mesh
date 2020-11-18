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

#ifndef SERIAL_CMD_H__
#define SERIAL_CMD_H__

#include <stdint.h>
#include <ble_gap.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_serial.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_dfu.h"
#include "nrf_mesh_assert.h"
#include "serial_types.h"
#include "packet.h"
#include "access.h"

/**
 * @defgroup SERIAL_CMD Serial commands
 * @ingroup MESH_SERIAL
 * @{
 */

#define SERIAL_OPCODE_CMD_RANGE_DEVICE_START                  (0x00) /**< DEVICE range start. */
#define SERIAL_OPCODE_CMD_DEVICE_ECHO                         (0x02) /**< Params: @ref serial_cmd_device_echo_t */
#define SERIAL_OPCODE_CMD_DEVICE_INTERNAL_EVENTS_REPORT       (0x03) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DEVICE_SERIAL_VERSION_GET           (0x09) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DEVICE_FW_INFO_GET                  (0x0A) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DEVICE_RADIO_RESET                  (0x0E) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DEVICE_BEACON_START                 (0x10) /**< Params: @ref serial_cmd_device_beacon_start_t */
#define SERIAL_OPCODE_CMD_DEVICE_BEACON_STOP                  (0x11) /**< Params: @ref serial_cmd_device_beacon_stop_t */
#define SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_SET            (0x12) /**< Params: @ref serial_cmd_device_beacon_params_set_t */
#define SERIAL_OPCODE_CMD_DEVICE_BEACON_PARAMS_GET            (0x13) /**< Params: @ref serial_cmd_device_beacon_params_get_t */
#define SERIAL_OPCODE_CMD_DEVICE_HOUSEKEEPING_DATA_GET        (0x14) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DEVICE_HOUSEKEEPING_DATA_CLEAR      (0x15) /**< Params: None. */

#define SERIAL_OPCODE_CMD_RANGE_DEVICE_END                    (0x1F) /**< DEVICE range end. */

#define SERIAL_OPCODE_CMD_RANGE_APP_START                     (0x20) /**< APP range start. */
#define SERIAL_OPCODE_CMD_APP_APPLICATION                     (0x20) /**< Params: @ref serial_cmd_application_t */
#define SERIAL_OPCODE_CMD_RANGE_APP_END                       (0x20) /**< APP range end. */

#define SERIAL_OPCODE_CMD_RANGE_SAR_START                     (0x21) /**< SAR range start. */
#define SERIAL_OPCODE_CMD_SAR_START                           (0x21) /**< Params: None. */
#define SERIAL_OPCODE_CMD_SAR_CONTINUE                        (0x22) /**< Params: None. */
#define SERIAL_OPCODE_CMD_RANGE_SAR_END                       (0x22) /**< SAR range end. */

#define SERIAL_OPCODE_CMD_RANGE_CONFIG_START                  (0x40) /**< CONFIG range start. */
#define SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_SET                 (0x40) /**< Params: @ref serial_cmd_config_adv_addr_t */
#define SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_GET                 (0x41) /**< Params: None. */
#define SERIAL_OPCODE_CMD_CONFIG_CHANNEL_MAP_SET              (0x42) /**< Params: @ref serial_cmd_config_channel_map_t */
#define SERIAL_OPCODE_CMD_CONFIG_CHANNEL_MAP_GET              (0x43) /**< Params: None. */
#define SERIAL_OPCODE_CMD_CONFIG_TX_POWER_SET                 (0x44) /**< Params: @ref serial_cmd_config_tx_power_t */
#define SERIAL_OPCODE_CMD_CONFIG_TX_POWER_GET                 (0x45) /**< Params: None. */
#define SERIAL_OPCODE_CMD_CONFIG_UUID_SET                     (0x53) /**< Params: @ref serial_cmd_config_uuid_t */
#define SERIAL_OPCODE_CMD_CONFIG_UUID_GET                     (0x54) /**< Params: None. */
#define SERIAL_OPCODE_CMD_RANGE_CONFIG_END                    (0x5F) /**< CONFIG range end. */

#define SERIAL_OPCODE_CMD_RANGE_PROV_START                    (0x60) /**< PROVISIONING range start. */
#define SERIAL_OPCODE_CMD_PROV_SCAN_START                     (0x61) /**< Params: None. */
#define SERIAL_OPCODE_CMD_PROV_SCAN_STOP                      (0x62) /**< Params: None. */
#define SERIAL_OPCODE_CMD_PROV_PROVISION                      (0x63) /**< Params: @ref serial_cmd_prov_data_t */
#define SERIAL_OPCODE_CMD_PROV_LISTEN                         (0x64) /**< Params: None. */
#define SERIAL_OPCODE_CMD_PROV_OOB_USE                        (0x66) /**< Params: @ref serial_cmd_prov_oob_use_t */
#define SERIAL_OPCODE_CMD_PROV_AUTH_DATA                      (0x67) /**< Params: @ref serial_cmd_prov_auth_data_t */
#define SERIAL_OPCODE_CMD_PROV_ECDH_SECRET                    (0x68) /**< Params: @ref serial_cmd_prov_ecdh_data_t */
#define SERIAL_OPCODE_CMD_PROV_KEYPAIR_SET                    (0x69) /**< Params: @ref serial_cmd_prov_keypair_t */
#define SERIAL_OPCODE_CMD_PROV_CAPABILITIES_SET               (0x6A) /**< Params: @ref serial_cmd_prov_caps_t */
#define SERIAL_OPCODE_CMD_RANGE_PROV_END                      (0x6F) /**< PROVISIONING range end. */

#define SERIAL_OPCODE_CMD_RANGE_OPENMESH_START                (0x70) /**< OPENMESH range start. */
#define SERIAL_OPCODE_CMD_OPENMESH_INIT                       (0x70) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_VALUE_SET                  (0x71) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_VALUE_ENABLE               (0x72) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_VALUE_DISABLE              (0x73) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_START                      (0x74) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_STOP                       (0x75) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_FLAG_SET                   (0x76) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_FLAG_GET                   (0x77) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_DFU_DATA                   (0x78) /**< Params: @ref serial_cmd_openmesh_dfu_data_t */
#define SERIAL_OPCODE_CMD_OPENMESH_VALUE_GET                  (0x7A) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_BUILD_VERSION_GET          (0x7B) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_ACCESS_ADDR_GET            (0x7C) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_CHANNEL_GET                (0x7D) /**< Params: None. */
#define SERIAL_OPCODE_CMD_OPENMESH_INTERVAL_MIN_MS_GET        (0x7F) /**< Params: None. */
#define SERIAL_OPCODE_CMD_RANGE_OPENMESH_END                  (0x8F) /**< OPENMESH range end. */


#define SERIAL_OPCODE_CMD_RANGE_MESH_START                    (0x90) /**< MESH range start. */

#define SERIAL_OPCODE_CMD_MESH_ENABLE                         (0x90) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_DISABLE                        (0x91) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_SUBNET_ADD                     (0x92) /**< Params: @ref serial_cmd_mesh_subnet_add_t */
#define SERIAL_OPCODE_CMD_MESH_SUBNET_UPDATE                  (0x93) /**< Params: @ref serial_cmd_mesh_subnet_update_t */
#define SERIAL_OPCODE_CMD_MESH_SUBNET_DELETE                  (0x94) /**< Params: @ref serial_cmd_mesh_subnet_delete_t */
#define SERIAL_OPCODE_CMD_MESH_SUBNET_GET_ALL                 (0x95) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_SUBNET_COUNT_MAX_GET           (0x96) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_APPKEY_ADD                     (0x97) /**< Params: @ref serial_cmd_mesh_appkey_add_t */
#define SERIAL_OPCODE_CMD_MESH_APPKEY_UPDATE                  (0x98) /**< Params: @ref serial_cmd_mesh_appkey_update_t */
#define SERIAL_OPCODE_CMD_MESH_APPKEY_DELETE                  (0x99) /**< Params: @ref serial_cmd_mesh_appkey_delete_t */
#define SERIAL_OPCODE_CMD_MESH_APPKEY_GET_ALL                 (0x9A) /**< Params: @ref serial_cmd_mesh_appkey_get_all_t */
#define SERIAL_OPCODE_CMD_MESH_APPKEY_COUNT_MAX_GET           (0x9B) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_DEVKEY_ADD                     (0x9C) /**< Params: @ref serial_cmd_mesh_devkey_add_t */
#define SERIAL_OPCODE_CMD_MESH_DEVKEY_DELETE                  (0x9D) /**< Params: @ref serial_cmd_mesh_devkey_delete_t */
#define SERIAL_OPCODE_CMD_MESH_DEVKEY_COUNT_MAX_GET           (0x9E) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_SET         (0x9F) /**< Params: @ref serial_cmd_mesh_addr_local_unicast_set_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_GET         (0xA0) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD          (0xA1) /**< Params: @ref serial_cmd_mesh_addr_subscription_add_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD_VIRTUAL  (0xA2) /**< Params: @ref serial_cmd_mesh_addr_subscription_add_virtual_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_REMOVE       (0xA3) /**< Params: @ref serial_cmd_mesh_addr_subscription_remove_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD           (0xA4) /**< Params: @ref serial_cmd_mesh_addr_publication_add_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD_VIRTUAL   (0xA5) /**< Params: @ref serial_cmd_mesh_addr_publication_add_virtual_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_REMOVE        (0xA6) /**< Params: @ref serial_cmd_mesh_addr_publication_remove_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_GET                       (0xA7) /**< Params: @ref serial_cmd_mesh_addr_get_t */
#define SERIAL_OPCODE_CMD_MESH_ADDR_GET_ALL                   (0xA8) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_ADDR_NONVIRTUAL_COUNT_MAX_GET  (0xA9) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_ADDR_VIRTUAL_COUNT_MAX_GET     (0xAA) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_PACKET_SEND                    (0xAB) /**< Params: @ref serial_cmd_mesh_packet_send_t */
#define SERIAL_OPCODE_CMD_MESH_STATE_CLEAR                    (0xAC) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MESH_CONFIG_SERVER_BIND             (0xAD) /**< Params: @ref serial_cmd_mesh_config_server_devkey_bind_t */
#define SERIAL_OPCODE_CMD_MESH_NET_STATE_SET                  (0xAE) /**< Params: @ref serial_cmd_mesh_net_state_set_t */
#define SERIAL_OPCODE_CMD_MESH_NET_STATE_GET                  (0xAF) /**< Params: None. */
#define SERIAL_OPCODE_CMD_RANGE_MESH_END                      (0xBF) /**< MESH range end. */

#define SERIAL_OPCODE_CMD_RANGE_DFU_START                     (0xD0) /**< DFU range start. */
#define SERIAL_OPCODE_CMD_DFU_JUMP_TO_BOOTLOADER              (0xD0) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DFU_REQUEST                         (0xD1) /**< Params: @ref serial_cmd_dfu_request_t */
#define SERIAL_OPCODE_CMD_DFU_RELAY                           (0xD2) /**< Params: @ref serial_cmd_dfu_relay_t */
#define SERIAL_OPCODE_CMD_DFU_ABORT                           (0xD3) /**< Params: None. */
#define SERIAL_OPCODE_CMD_DFU_BANK_INFO_GET                   (0xD4) /**< Params: @ref serial_cmd_dfu_bank_info_get_t */
#define SERIAL_OPCODE_CMD_DFU_BANK_FLASH                      (0xD5) /**< Params: @ref serial_cmd_dfu_bank_flash_t */
#define SERIAL_OPCODE_CMD_DFU_STATE_GET                       (0xD6) /**< Params: None. */
#define SERIAL_OPCODE_CMD_RANGE_DFU_END                       (0xDF) /**< DFU range end. */

#define SERIAL_OPCODE_CMD_RANGE_ACCESS_START                  (0xE0) /**< Start of ACCESS command range. */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_SET           (0xE0) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_GET           (0xE1) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_SET         (0xE2) /**< Params: @ref serial_cmd_access_pub_period_set_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_GET         (0xE3) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_ADD               (0xE4) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_REMOVE            (0xE5) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET               (0xE6) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_BIND               (0xE7) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_UNBIND             (0xE8) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET                (0xE9) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_SET            (0xEA) /**< Params: @ref serial_cmd_access_handle_pair_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_GET            (0xEB) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_SET            (0xEC) /**< Params: @ref serial_cmd_access_model_pub_ttl_set_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_GET            (0xED) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_SET                 (0xEE) /**< Params: @ref serial_cmd_access_element_loc_set_t */
#define SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_GET                 (0xEF) /**< Params: @ref serial_cmd_access_element_index_t */
#define SERIAL_OPCODE_CMD_ACCESS_ELEM_SIG_MODEL_COUNT_GET     (0xF0) /**< Params: @ref serial_cmd_access_element_index_t */
#define SERIAL_OPCODE_CMD_ACCESS_ELEM_VENDOR_MODEL_COUNT_GET  (0xF1) /**< Params: @ref serial_cmd_access_element_index_t */
#define SERIAL_OPCODE_CMD_ACCESS_MODEL_ID_GET                 (0xF2) /**< Params: @ref serial_cmd_access_model_handle_t */
#define SERIAL_OPCODE_CMD_ACCESS_HANDLE_GET                   (0xF3) /**< Params: @ref serial_cmd_access_handle_get_t */
#define SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET              (0xF4) /**< Params: @ref serial_cmd_access_element_index_t */
#define SERIAL_OPCODE_CMD_RANGE_ACCESS_END                    (0xF4) /**< End of ACCESS command range. */

#define SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_START          (0xFC) /**< Start of MODEL specific command range. */
#define SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET           (0xFC) /**< Params: None. */
#define SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT                 (0xFD) /**< Params: @ref serial_cmd_model_specific_init_t */
#define SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND              (0xFE) /**< Params: @ref serial_cmd_model_specific_command_t */
#define SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_END            (0xFE) /**< End of MODEL specific command range. */


/************** Device commands **************/
/*lint -align_max(push) -align_max(1) */

/** Echo cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN];   /**< Data to echo back. */
} serial_cmd_device_echo_t;

/** Beacon start cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t beacon_slot; /**< Slot number of the beacon to set the payload for. */
    uint8_t data[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; /**< Beacon payload. */
} serial_cmd_device_beacon_start_t;

/** Beacon stop cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t beacon_slot; /**< Slot number of the beacon to stop. */
} serial_cmd_device_beacon_stop_t;

/** Beacon params set cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t beacon_slot; /**< Slot number of the beacon to start. */
    uint8_t tx_power; /**< TX Power value, must be a value from @ref serial_cmd_tx_power_value_t. */
    uint8_t channel_map; /**< Channel map bitfield for beacon, starting at channel 37. */
    uint32_t interval_ms; /**< TX interval in milliseconds. */
} serial_cmd_device_beacon_params_set_t;

/** Beacon params get cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t beacon_slot; /**< Slot number of the beacon to get the parameters of. */
} serial_cmd_device_beacon_params_get_t;

/** Union of all device command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_device_echo_t echo; /**< Echo parameters. */
    serial_cmd_device_beacon_start_t beacon_start; /**< Beacon start parameters. */
    serial_cmd_device_beacon_stop_t beacon_stop; /**< Beacon stop parameters. */
    serial_cmd_device_beacon_params_set_t beacon_params_set; /**< Beacon params set parameters. */
    serial_cmd_device_beacon_params_get_t beacon_params_get; /**< Beacon params get parameters. */
} serial_cmd_device_t;

/************** Config commands **************/
/** Advertisement address config command parameters. */
typedef struct __attribute((packed))
{
    uint8_t addr_type;                  /**< BLE advertising address type. */
    uint8_t adv_addr[BLE_GAP_ADDR_LEN]; /**< BLE advertising address. */
} serial_cmd_config_adv_addr_t;

/** Channel map config command parameters. */
typedef struct __attribute((packed))
{
    uint8_t channel_map;    /**< Channel map bitfield for mesh to use, starting at channel 37. */
} serial_cmd_config_channel_map_t;

/** TX power config command parameters. */
typedef struct __attribute((packed))
{
    uint8_t tx_power; /**< Transmit power of radio, see @ref serial_cmd_tx_power_value_t for accepted values. */
} serial_cmd_config_tx_power_t;

/** UUID config command parameters. */
typedef struct __attribute((packed))
{
    uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< Device UUID. */
} serial_cmd_config_uuid_t;

/** Union of all config command parameters */
typedef union __attribute((packed))
{
    serial_cmd_config_adv_addr_t    adv_addr;    /**< Advertising address. */
    serial_cmd_config_channel_map_t channel_map; /**< Channel map. */
    serial_cmd_config_tx_power_t    tx_power;    /**< Radio transmit power. */
    serial_cmd_config_uuid_t        uuid;        /**< Device UUID. */
} serial_cmd_config_t;

/************** OpenMesh commands **************/
/** DFU data parameters */
typedef struct __attribute((packed))
{
    uint8_t dfu_packet[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; /**< DFU packet data. */
} serial_cmd_openmesh_dfu_data_t;

/** Union of all config command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_openmesh_dfu_data_t dfu_data; /**< DFU data parameters. */
} serial_cmd_openmesh_t;

/************** Provisioning commands **************/

/** Keypair cmd parameters. */
typedef struct __attribute((packed))
{
    uint8_t private_key[NRF_MESH_ECDH_PRIVATE_KEY_SIZE]; /**< Private key. */
    uint8_t public_key[NRF_MESH_ECDH_PUBLIC_KEY_SIZE];   /**< Public key. */
} serial_cmd_prov_keypair_t;

/** Provisioning set capabilities parameters. */
typedef struct __attribute((packed))
{
    uint8_t  num_elements;       /**< The number of elements in the device */
    uint8_t  public_key_type;    /**< The type of public key used in the device. */

    uint8_t  static_oob_types;   /**< The types of static OOB authentication methods. */

    uint8_t  output_oob_size;    /**< Maximum size of the OOB authentication output. */
    uint16_t output_oob_actions; /**< Available output actions for OOB authentication. */

    uint8_t  input_oob_size;     /**< Maximum size of the OOB authentication input. */
    uint16_t input_oob_actions;  /**< Available input actions for OOB authentication. */
} serial_cmd_prov_caps_t;

/** Provisioning provision command parameters. */
typedef struct __attribute((packed))
{
    uint8_t  context_id; /**< Context ID to use for this provisioning session. */
    uint8_t  target_uuid[NRF_MESH_UUID_SIZE]; /**< UUID of the device to provision. */
    uint8_t  network_key[NRF_MESH_KEY_SIZE];  /**< Network key to give to the device. */
    uint16_t network_key_index;               /**< Network key index. */
    uint32_t iv_index;                        /**< Initial IV index of the network. */
    uint16_t address;                         /**< Unicast address to assign to the device. */
    uint8_t  iv_update_flag;                  /**< IV update in progress flag. */
    uint8_t  key_refresh_flag;                /**< Key refresh in progress flag. */
    uint8_t  attention_duration_s;            /**< Time in seconds during which the device will identify itself using any means it can. */
} serial_cmd_prov_data_t;

/** OOB method selection parameters. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< ID of context to set the oob method for. */
    uint8_t oob_method; /**< OOB method to use, see @ref nrf_mesh_prov_oob_method_t for accepted values. */
    uint8_t oob_action; /**< OOB action to use, see @ref nrf_mesh_prov_input_action_t or @ref nrf_mesh_prov_output_action_t for values. */
    uint8_t size;       /**< Size of the OOB data. */
} serial_cmd_prov_oob_use_t;

/** Authentication data parameters. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< ID of the context to set the authentication data for. */
    uint8_t data[16];  /**< Authentication data. */
} serial_cmd_prov_auth_data_t;

/** ECDH shared secret data parameters. */
typedef struct __attribute((packed))
{
    uint8_t context_id; /**< ID of the context to set the shared secret for. */
    uint8_t shared_secret[NRF_MESH_ECDH_SHARED_SECRET_SIZE]; /**< ECDH shared secret. */
} serial_cmd_prov_ecdh_data_t;

/** Union of all provisioning command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_prov_keypair_t   keypair;   /**< Parameters for the Set keypair packet. */
    serial_cmd_prov_caps_t      caps;      /**< Parameters for the Set capabilities packet. */
    serial_cmd_prov_data_t      data;      /**< Parameters for the Provisioning data packet. */
    serial_cmd_prov_oob_use_t   oob_use;   /**< Parameters for the OOB use packet. */
    serial_cmd_prov_auth_data_t auth_data; /**< Parameters for the Authentication data packet. */
    serial_cmd_prov_ecdh_data_t ecdh_data; /**< Parameters for the ECDH shared secret packet. */
} serial_cmd_prov_t;

/******************* Mesh commands *********************/
/** Mesh subnet add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t net_key_index; /**< Mesh-global key index. */
    uint8_t key[NRF_MESH_KEY_SIZE]; /**< Key to add. */
} serial_cmd_mesh_subnet_add_t;

/** Mesh subnet update command parameters. */
typedef struct __attribute((packed))
{
    uint16_t subnet_handle; /**< Handle of the subnet to change. */
    uint8_t key[NRF_MESH_KEY_SIZE]; /**< Key to change to. */
} serial_cmd_mesh_subnet_update_t;

/** Mesh subnet delete command parameters. */
typedef struct __attribute((packed))
{
    uint16_t subnet_handle; /**< Handle of the subnet to delete. */
} serial_cmd_mesh_subnet_delete_t;

/** Mesh appkey add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t app_key_index; /**< Mesh-global key index. */
    uint16_t subnet_handle; /**< Handle of the subnetwork to add the appkey to. */
    uint8_t key[NRF_MESH_KEY_SIZE]; /**< Key to add. */
} serial_cmd_mesh_appkey_add_t;

/** Mesh appkey update command parameters. */
typedef struct __attribute((packed))
{
    uint16_t appkey_handle; /**< Handle of the appkey to change. */
    uint8_t key[NRF_MESH_KEY_SIZE]; /**< Key to change to. */
} serial_cmd_mesh_appkey_update_t;

/** Mesh appkey delete command parameters. */
typedef struct __attribute((packed))
{
    uint16_t appkey_handle; /**< Handle of the appkey to delete. */
} serial_cmd_mesh_appkey_delete_t;

/** Mesh appkey get all command parameters. */
typedef struct __attribute((packed))
{
    uint16_t subnet_handle; /**< Handle of the subnet to get all appkeys of. */
} serial_cmd_mesh_appkey_get_all_t;

/** Mesh devkey add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t owner_addr; /**< Unicast address of the device that owns the given devkey. */
    uint16_t subnet_handle; /**< Handle of the subnetwork to bind the devkey to. */
    uint8_t key[NRF_MESH_KEY_SIZE]; /**< Key to add. */
} serial_cmd_mesh_devkey_add_t;

/** Mesh devkey delete command parameters. */
typedef struct __attribute((packed))
{
    uint16_t devkey_handle; /**< Handle of the devkey to delete. */
} serial_cmd_mesh_devkey_delete_t;

/** Mesh local unicast address set command parameters. */
typedef struct __attribute((packed))
{
    uint16_t start_address; /**< First address in the range of unicast addresses. */
    uint16_t count;         /**< Number of addresses in the range of unicast addresses. */
} serial_cmd_mesh_addr_local_unicast_set_t;

/** Mesh address add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t raw_address;   /**< Raw representation of the address to add. */
} serial_cmd_mesh_addr_add_t;

/** Mesh address virtual add command parameters. */
typedef struct __attribute((packed))
{
    uint8_t virtual_addr_uuid[NRF_MESH_UUID_SIZE]; /**< Virtual address UUID to add */
} serial_cmd_mesh_addr_virtual_add_t;

/** Mesh address get command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address_handle; /**< Handle of address to get raw representation of. */
} serial_cmd_mesh_addr_get_t;

/** Mesh address subscription add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address; /**< Address to add as a subscription address. */
} serial_cmd_mesh_addr_subscription_add_t;

/** Mes address subscription add virtual command parameters. */
typedef struct __attribute((packed))
{
    uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< Virtual address UUID. */
} serial_cmd_mesh_addr_subscription_add_virtual_t;

/** Mesh address subscription remove command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address_handle;   /**< Handle of address to remove from address subscription list. */
} serial_cmd_mesh_addr_subscription_remove_t;

/** Mesh address publication add command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address; /**< Address to add as a publication address. */
} serial_cmd_mesh_addr_publication_add_t;

/** Mesh address publication add virtual command parameters. */
typedef struct __attribute((packed))
{
    uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< Virtual address UUID. */
} serial_cmd_mesh_addr_publication_add_virtual_t;

/** Mesh address publication remove command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address_handle; /**< Handle of the address to remove from the publication address list. */
} serial_cmd_mesh_addr_publication_remove_t;

/** Mesh packet send command parameters. */
typedef struct __attribute((packed))
{
    uint16_t appkey_handle;             /**< Appkey or devkey handle to use for packet sending. Subnetwork will be picked automatically. */
    uint16_t src_addr;                  /**< Raw unicast address to use as source address. Must be in the range of local unicast addresses. */
    uint16_t dst_addr_handle;           /**< Handle of destination address to use in packet. */
    uint8_t ttl;                        /**< Time To Live value to use in packet. */
    uint8_t force_segmented;            /**< Whether or not to force use of segmented message type for the transmission. */
    uint8_t transmic_size;              /**< Transport MIC size used enum. SMALL=0, LARGE=1, DEFAULT=2. LARGE may only be used with segmented packets. */
    uint8_t friendship_credential_flag; /**< Control parameter for credentials used to publish messages from a model. 0 for master, 1 for friendship. */
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN - SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD];    /**< Payload of the packet. */
} serial_cmd_mesh_packet_send_t;

NRF_MESH_STATIC_ASSERT(sizeof(serial_cmd_mesh_packet_send_t) == NRF_MESH_SERIAL_PAYLOAD_MAXLEN);

/** Configuration Server: device key bind command parameters. */
typedef struct __attribute((packed))
{
    uint16_t address_handle; /**< Handle of the address to get the raw representation of. */
} serial_cmd_mesh_config_server_devkey_bind_t;

/** Mesh net state set command parameters */
typedef struct __attribute((packed))
{
    uint32_t  iv_index; /**< The IV index to set.*/
    uint8_t   iv_update_in_progress; /**< Value indicating the phase of the IV update process. */
    uint16_t  iv_update_timeout_counter; /**< Timeout counter for IV update process. */
    uint32_t  next_seqnum_block; /**< The first sequence number block which is not yet allocated. */
} serial_cmd_mesh_net_state_set_t;

/** Mesh command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_mesh_subnet_add_t                    subnet_add;                    /**< Subnet add parameters. */
    serial_cmd_mesh_subnet_update_t                 subnet_update;                 /**< Subnet update parameters. */
    serial_cmd_mesh_subnet_delete_t                 subnet_delete;                 /**< Subnet delete parameters. */

    serial_cmd_mesh_appkey_add_t                    appkey_add;                    /**< Appkey add parameters. */
    serial_cmd_mesh_appkey_update_t                 appkey_update;                 /**< Appkey update parameters. */
    serial_cmd_mesh_appkey_delete_t                 appkey_delete;                 /**< Appkey delete parameters. */
    serial_cmd_mesh_appkey_get_all_t                appkey_get_all;                /**< Appkey get all parameters. */

    serial_cmd_mesh_devkey_add_t                    devkey_add;                    /**< Devkey add parameters. */
    serial_cmd_mesh_devkey_delete_t                 devkey_delete;                 /**< Devkey delete parameters. */

    serial_cmd_mesh_addr_local_unicast_set_t        local_unicast_addr_set;        /**< Address local unicast set parameters. */
    serial_cmd_mesh_addr_add_t                      addr_add;                      /**< Address add parameters. */
    serial_cmd_mesh_addr_virtual_add_t              addr_virtual_add;              /**< Virtual address add parameters. */
    serial_cmd_mesh_addr_get_t                      addr_get;                      /**< Address get parameters. */

    serial_cmd_mesh_addr_subscription_add_t         addr_subscription_add;         /**< Subscription address add parameters. */
    serial_cmd_mesh_addr_subscription_add_virtual_t addr_subscription_add_virtual; /**< Virtual subscription address add parameters. */
    serial_cmd_mesh_addr_subscription_remove_t      addr_subscription_remove;      /**< Subscription address remove parameters. */

    serial_cmd_mesh_addr_publication_add_t          addr_publication_add;          /**< Publication address add parameters. */
    serial_cmd_mesh_addr_publication_add_virtual_t  addr_publication_add_virtual;  /**< Virtual publication address add parameters. */
    serial_cmd_mesh_addr_publication_remove_t       addr_publication_remove;       /**< Publication address remove parameters. */

    serial_cmd_mesh_packet_send_t                   packet_send;                   /**< Packet send parameters. */
    serial_cmd_mesh_config_server_devkey_bind_t     config_server_devkey_bind;     /**< Configuration Server: device key bind parameters. */
    serial_cmd_mesh_net_state_set_t                 net_state_set;                 /**< Net state set parameters */
} serial_cmd_mesh_t;

/* **** PB-MESH Client **** */

/** Initialize PB-MESH Client. */
typedef struct __attribute((packed))
{
    uint8_t element_index; /**< Element index.*/
    uint8_t prov_context_index; /**< Provisioning context index. */
    uint8_t application_index; /**< Application context index. */
} serial_cmd_pb_remote_client_init_t;

/** Start remote scanning command. */
typedef struct __attribute((packed))
{
    uint16_t server_address; /**< Remote provisioning Server address. */
} serial_cmd_pb_remote_client_remote_scan_start_t;

/** Cancel remote scanning command. */
typedef struct __attribute((packed))
{
    uint16_t server_address; /**< Remote provisioning Server address. */
} serial_cmd_pb_remote_client_remote_scan_cancel_t;

/** Start remote provisioning command. */
typedef struct __attribute((packed))
{
    uint16_t server_address; /**< Remote provisioning Server address. */
    uint8_t unprovisioned_device_index; /**< Unprovisioned device index. */
    uint8_t network_key[NRF_MESH_KEY_SIZE];  /**< Network key to give to the device. */
    uint32_t iv_index; /**< Initial IV index of the network. */
    uint16_t address; /**< Unicast address to assign to the device. */
} serial_cmd_pb_remote_client_remote_provision_t;

/** PB-Mesh client commands. */
typedef union __attribute((packed))
{
    serial_cmd_pb_remote_client_init_t init; /**< Initialize PB-Mesh client. */
    serial_cmd_pb_remote_client_remote_scan_start_t remote_scan_start; /**< Start remote scanning procedure. */
    serial_cmd_pb_remote_client_remote_scan_cancel_t remote_scan_cancel; /**< Cancel remote scanning procedure. */
    serial_cmd_pb_remote_client_remote_provision_t remote_provision; /**< Start remote provisioning */
} serial_cmd_pb_remote_t;

/*********** DFU commands ***************/
/** DFU request command parameters. */
typedef struct __attribute((packed))
{
    uint8_t dfu_type; /**< DFU Firmware type to request. */
    nrf_mesh_fwid_t fwid; /**< Firmware ID to request. */
    uint32_t bank_addr; /**< Address in which to bank firmware. */
} serial_cmd_dfu_request_t;

/** DFU relay command parameters. */
typedef struct __attribute((packed))
{
    uint8_t dfu_type; /**< DFU Firmware type to relay. */
    nrf_mesh_fwid_t fwid; /**< Firmware ID of firmware that should be relayed. */
} serial_cmd_dfu_relay_t;

/** DFU bank info get command parameters. */
typedef struct __attribute((packed))
{
    uint8_t dfu_type; /**< DFU Firmware type to get bank info about. */
} serial_cmd_dfu_bank_info_get_t;

/** DFU bank flash command parameters. */
typedef struct __attribute((packed))
{
    uint8_t dfu_type; /**< DFU Firmware type to flash. */
} serial_cmd_dfu_bank_flash_t;

/** DFU command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_dfu_request_t request;         /**< DFU request parameters. */
    serial_cmd_dfu_relay_t relay;             /**< DFU relay parameters. */
    serial_cmd_dfu_bank_info_get_t bank_info; /**< DFU bank info parameters. */
    serial_cmd_dfu_bank_flash_t bank_flash;   /**< DFU bank flash parameters. */
} serial_cmd_dfu_t;

/*********** Access commands ************/
/** Used by various access commands that work on address handles for a given model */
typedef struct __attribute((packed))
{
    access_model_handle_t model_handle; /**< Handle for the model being modified. */
    dsm_handle_t dsm_handle; /**< Handle for a value (e.g. address) stored by the device state manager. */
} serial_cmd_access_handle_pair_t;

/** Used by access commands that only require the model handle */
typedef struct __attribute((packed))
{
    access_model_handle_t handle; /**< Handle of the model that the access module should operate on. */
} serial_cmd_access_model_handle_t;

/** Used to update the location field of an element. */
typedef struct __attribute((packed))
{
    uint16_t element_index; /**< Index of the addressed element. */
    uint16_t location; /**< Location value for the element. */
} serial_cmd_access_element_loc_set_t;

/** Used to update the ttl value for the messages originating from a given model. */
typedef struct __attribute((packed))
{
    access_model_handle_t model_handle; /**< Handle of the model that the access module should operate on. */
    uint8_t ttl;                        /**< TTL for outgoing messages. */
} serial_cmd_access_model_pub_ttl_set_t;

/** Used to get the handle value for a model instance. */
typedef struct __attribute((packed))
{
    uint16_t element_index;             /**< Index of the addressed element which owns the model. */
    access_model_id_t model_id;         /**< Company and model IDs. */
} serial_cmd_access_handle_get_t;

/** Used to update the publish period of a model by updating resolution and number of steps. */
typedef struct __attribute((packed))
{
    access_model_handle_t model_handle; /**< Handle of the model that the access module should operate on. */
    uint8_t  resolution;   /**< see @ref access_publish_resolution_t for accepted values. */
    uint8_t  step_number;  /**< Must not be larger than @ref ACCESS_PUBLISH_PERIOD_STEP_MAX. */
} serial_cmd_access_pub_period_set_t;

/** Used by access commands that only require the element index */
typedef struct __attribute((packed))
{
    uint16_t element_index;               /**< Index of the addressed element. */
} serial_cmd_access_element_index_t;

/** Used for initializing one of the available models */
typedef struct __attribute((packed))
{
    serial_cmd_model_specific_init_header_t model_init_info; /**< Basic information that is always needed to initialize a model */
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN - sizeof(serial_cmd_model_specific_init_header_t)]; /**< Additional data provided to the initializer */
} serial_cmd_model_specific_init_t;

NRF_MESH_STATIC_ASSERT(sizeof(serial_cmd_model_specific_init_t) == NRF_MESH_SERIAL_PAYLOAD_MAXLEN);

/** Used for sending commands to one of the initialized models */
typedef struct __attribute((packed))
{
    serial_cmd_model_specific_command_header_t model_cmd_info; /**< Contains the handle of the model being addressed. */
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN - sizeof(serial_cmd_model_specific_command_header_t)]; /**< Additional data provided to the event */
} serial_cmd_model_specific_command_t;
NRF_MESH_STATIC_ASSERT(sizeof(serial_cmd_model_specific_command_t) == NRF_MESH_SERIAL_PAYLOAD_MAXLEN);

/** ACCESS layer command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_access_handle_pair_t       handle_pair;
    serial_cmd_access_model_handle_t      model_handle;
    serial_cmd_access_element_loc_set_t   elem_loc;
    serial_cmd_access_model_pub_ttl_set_t model_ttl;
    serial_cmd_access_handle_get_t        handle_get;
    serial_cmd_access_pub_period_set_t    publish_period;
    serial_cmd_access_element_index_t     index;
    serial_cmd_model_specific_init_t      model_init;
    serial_cmd_model_specific_command_t   model_cmd;
} serial_cmd_access_t;

/*********** App commands ***************/
/** Application command parameters. */
typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_PAYLOAD_MAXLEN]; /**< Application data. */
} serial_cmd_application_t;

/** Union of all command parameters. */
typedef union __attribute((packed))
{
    serial_cmd_access_t      access;      /**< Used for Access layer and Model interfacing */
    serial_cmd_device_t      device;      /**< Device parameters. */
    serial_cmd_config_t      config;      /**< Configuration parameters. */
    serial_cmd_openmesh_t    openmesh;    /**< OpenMesh parameters. */
    serial_cmd_prov_t        prov;        /**< Provisioning parameters. */
    serial_cmd_mesh_t        mesh;        /**< Mesh parameters. */
    serial_cmd_dfu_t         dfu;         /**< DFU parameters. */
    serial_cmd_pb_remote_t   pb_remote;   /**< PB-MESH parameters. */
    serial_cmd_application_t application; /**< Application parameters. */
} serial_cmd_t;

/*lint -align_max(pop) */
/** @} */

#endif
