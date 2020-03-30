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

#ifndef CONFIG_OPCODES_H__
#define CONFIG_OPCODES_H__


/**
 * @defgroup CONFIG_OPCODES Opcodes
 * @ingroup CONFIG_MODEL
 * Opcode definitions for the Config models.
 * @{
 */

/**
 * Configuration model opcodes.
 */
typedef enum
{
    /** Opcode for the "AppKey Add" message. */
    CONFIG_OPCODE_APPKEY_ADD = 0x00,
    /** Opcode for the "AppKey Update" message. */
    CONFIG_OPCODE_APPKEY_UPDATE = 0x01,
    /** Opcode for the "Composition Data Status" message. */
    CONFIG_OPCODE_COMPOSITION_DATA_STATUS = 0x02,
    /** Opcode for the "Config Model Publication Set" message. */
    CONFIG_OPCODE_MODEL_PUBLICATION_SET = 0x03,
    /** Opcode for the "Heartbeat Publication Status" message. */
    CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS = 0x06,
    /** Opcode for the "AppKey Delete" message. */
    CONFIG_OPCODE_APPKEY_DELETE = 0x8000,
    /** Opcode for the "AppKey Get" message. */
    CONFIG_OPCODE_APPKEY_GET = 0x8001,
    /** Opcode for the "AppKey List" message. */
    CONFIG_OPCODE_APPKEY_LIST = 0x8002,
    /** Opcode for the "AppKey Status" message. */
    CONFIG_OPCODE_APPKEY_STATUS = 0x8003,
    /** Opcode for the "Composition Data Get" message. */
    CONFIG_OPCODE_COMPOSITION_DATA_GET = 0x8008,
    /** Opcode for the "Config Beacon Get" message. */
    CONFIG_OPCODE_BEACON_GET = 0x8009,
    /** Opcode for the "Config Beacon Set" message. */
    CONFIG_OPCODE_BEACON_SET = 0x800A,
    /** Opcode for the "Config Beacon Status" message. */
    CONFIG_OPCODE_BEACON_STATUS = 0x800B,
    /** Opcode for the "Config Default TTL Get" message. */
    CONFIG_OPCODE_DEFAULT_TTL_GET = 0x800C,
    /** Opcode for the "Config Default TTL Set" message. */
    CONFIG_OPCODE_DEFAULT_TTL_SET = 0x800D,
    /** Opcode for the "Config Default TTL Status" message. */
    CONFIG_OPCODE_DEFAULT_TTL_STATUS = 0x800E,
    /** Opcode for the "Config Friend Get" message. */
    CONFIG_OPCODE_FRIEND_GET = 0x800F,
    /** Opcode for the "Config Friend Set" message. */
    CONFIG_OPCODE_FRIEND_SET = 0x8010,
    /** Opcode for the "Config Friend Status" message. */
    CONFIG_OPCODE_FRIEND_STATUS = 0x8011,
    /** Opcode for the "Config GATT Proxy Get" message. */
    CONFIG_OPCODE_GATT_PROXY_GET = 0x8012,
    /** Opcode for the "Config GATT Proxy Set" message. */
    CONFIG_OPCODE_GATT_PROXY_SET = 0x8013,
    /** Opcode for the "Config GATT Proxy Status" message. */
    CONFIG_OPCODE_GATT_PROXY_STATUS = 0x8014,
    /** Opcode for the "Config Key Refresh Phase Get" message. */
    CONFIG_OPCODE_KEY_REFRESH_PHASE_GET = 0x8015,
    /** Opcode for the "Config Key Refresh Phase Set" message. */
    CONFIG_OPCODE_KEY_REFRESH_PHASE_SET = 0x8016,
    /** Opcode for the "Config Key Refresh Phase Status" message. */
    CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS = 0x8017,
    /** Opcode for the "Config Model Publication Get" message. */
    CONFIG_OPCODE_MODEL_PUBLICATION_GET = 0x8018,
    /** Opcode for the "Config Model Publication Status" message. */
    CONFIG_OPCODE_MODEL_PUBLICATION_STATUS = 0x8019,
    /** Opcode for the "Config Model Publication Virtual Address Set" message. */
    CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET = 0x801A,
    /** Opcode for the "Config Model Subscription Add" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD = 0x801B,
    /** Opcode for the "Config Model Subscription Delete" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE = 0x801C,
    /** Opcode for the "Config Model Subscription Delete All" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL = 0x801D,
    /** Opcode for the "Config Model Subscription Overwrite" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE = 0x801E,
    /** Opcode for the "Config Model Subscription Status" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS = 0x801F,
    /** Opcode for the "Config Model Subscription Virtual Address Add" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD = 0x8020,
    /** Opcode for the "Config Model Subscription Virtual Address Delete" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE = 0x8021,
    /** Opcode for the "Config Model Subscription Virtual Address Overwrite" message. */
    CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE = 0x8022,
    /** Opcode for the "Config Network Transmit Get" message. */
    CONFIG_OPCODE_NETWORK_TRANSMIT_GET = 0x8023,
    /** Opcode for the "Config Network Transmit Set" message. */
    CONFIG_OPCODE_NETWORK_TRANSMIT_SET = 0x8024,
    /** Opcode for the "Config Network Transmit Status" message. */
    CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS = 0x8025,
    /** Opcode for the "Config Relay Get" message. */
    CONFIG_OPCODE_RELAY_GET = 0x8026,
    /** Opcode for the "Config Relay Set" message. */
    CONFIG_OPCODE_RELAY_SET = 0x8027,
    /** Opcode for the "Config Relay Status" message. */
    CONFIG_OPCODE_RELAY_STATUS = 0x8028,
    /** Opcode for the "Config SIG Model Subscription Get" message. */
    CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET = 0x8029,
    /** Opcode for the "Config SIG Model Subscription List" message. */
    CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST = 0x802A,
    /** Opcode for the "Config Vendor Model Subscription Get" message. */
    CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET = 0x802B,
    /** Opcode for the "Config Vendor Model Subscription List" message. */
    CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST = 0x802C,
    /** Opcode for the "Config Low Power node PollTimeout Get" message. */
    CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_GET = 0x802D,
    /** Opcode for the "Config Low Power node PollTimeout Status" message. */
    CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS = 0x802E,
    /** Opcode for the "Heartbeat Publication Get" message. */
    CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET = 0x8038,
    /** Opcode for the "Heartbeat Publication Set" message. */
    CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET = 0x8039,
    /** Opcode for the "Heartbeat Subscription Get" message. */
    CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET = 0x803A,
    /** Opcode for the "Heartbeat Subscription Set" message. */
    CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET = 0x803B,
    /** Opcode for the "Heartbeat Subscription Status" message. */
    CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS = 0x803C,
    /** Opcode for the "Model App Bind" message. */
    CONFIG_OPCODE_MODEL_APP_BIND = 0x803D,
    /** Opcode for the "Model App Status" message. */
    CONFIG_OPCODE_MODEL_APP_STATUS = 0x803E,
    /** Opcode for the "Model App Unbind" message. */
    CONFIG_OPCODE_MODEL_APP_UNBIND = 0x803F,
    /** Opcode for the "NetKey Add" message. */
    CONFIG_OPCODE_NETKEY_ADD = 0x8040,
    /** Opcode for the "NetKey Delete" message. */
    CONFIG_OPCODE_NETKEY_DELETE = 0x8041,
    /** Opcode for the "NetKey Get" message. */
    CONFIG_OPCODE_NETKEY_GET = 0x8042,
    /** Opcode for the "NetKey List" message. */
    CONFIG_OPCODE_NETKEY_LIST = 0x8043,
    /** Opcode for the "NetKey Status" message. */
    CONFIG_OPCODE_NETKEY_STATUS = 0x8044,
    /** Opcode for the "NetKey Update" message. */
    CONFIG_OPCODE_NETKEY_UPDATE = 0x8045,
    /** Opcode for the "Node Identity Get" message. */
    CONFIG_OPCODE_NODE_IDENTITY_GET = 0x8046,
    /** Opcode for the "Node Identity Set" message. */
    CONFIG_OPCODE_NODE_IDENTITY_SET = 0x8047,
    /** Opcode for the "Node Identity Status" message. */
    CONFIG_OPCODE_NODE_IDENTITY_STATUS = 0x8048,
    /** Opcode for the "Node Reset" message. */
    CONFIG_OPCODE_NODE_RESET = 0x8049,
    /** Opcode for the "Node Reset Status" message. */
    CONFIG_OPCODE_NODE_RESET_STATUS = 0x804A,
    /** Opcode for the "SIG Model App Get" message. */
    CONFIG_OPCODE_SIG_MODEL_APP_GET = 0x804B,
    /** Opcode for the "SIG Model App List" message. */
    CONFIG_OPCODE_SIG_MODEL_APP_LIST = 0x804C,
    /** Opcode for the "Vendor Model App Get" message. */
    CONFIG_OPCODE_VENDOR_MODEL_APP_GET = 0x804D,
    /** Opcode for the "Vendor Model App List" message. */
    CONFIG_OPCODE_VENDOR_MODEL_APP_LIST = 0x804E,
} config_opcode_t;

/** @} */

#endif
