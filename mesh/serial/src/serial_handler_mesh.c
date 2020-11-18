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

#include "serial_handler_mesh.h"
#include "serial_handler_common.h"
#include "serial.h"
#include "serial_status.h"
#include "serial_cmd_rsp.h"
#include "nrf_mesh.h"
#include "device_state_manager.h"
#include "access.h"
#include "net_state.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_externs.h"
#include "config_server.h"
#include "hal.h"
#include "mesh_stack.h"
#include "mesh_opt_net_state.h"
#include "mesh_config_entry.h"
#include "mesh_config_listener.h"

/* Ensure that we're mapping the size of the serial parameter to the
 * dsm_handle_t. If this triggers, someone changed the size of the
 * dsm_handle_t. In that case, all handle-parameters in the mesh serial
 * commands must change types. */
NRF_MESH_STATIC_ASSERT(sizeof(uint16_t) == sizeof(dsm_handle_t));

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef void (*mesh_serial_cmd_handler_cb_t)(const serial_packet_t* p_cmd);
typedef struct
{
    uint8_t opcode;
    uint8_t min_param_length;
    uint8_t optional_extra_bytes; /**< Extra bytes allowed, in addition to min_param_length. */
    mesh_serial_cmd_handler_cb_t handler_cb;
} mesh_serial_cmd_handler_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static nrf_mesh_evt_handler_t m_evt_handler;

/*****************************************************************************
* Command handler functions
*****************************************************************************/

static void handle_cmd_enable(const serial_packet_t * p_cmd)
{
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, nrf_mesh_enable(), NULL, 0);
}

static void handle_cmd_disable(const serial_packet_t * p_cmd)
{
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, nrf_mesh_disable(), NULL, 0);
}

static void handle_cmd_subnet_add(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_subnet_t rsp;
    dsm_handle_t subnet_handle;
    uint32_t status = dsm_subnet_add(p_cmd->payload.cmd.mesh.subnet_add.net_key_index,
            p_cmd->payload.cmd.mesh.subnet_add.key,
            &subnet_handle);
    rsp.subnet_handle = subnet_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_subnet_update(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_subnet_update(p_cmd->payload.cmd.mesh.subnet_update.subnet_handle,
            p_cmd->payload.cmd.mesh.subnet_update.key);
    serial_evt_cmd_rsp_data_subnet_t rsp;
    rsp.subnet_handle = p_cmd->payload.cmd.mesh.subnet_update.subnet_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_subnet_delete(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_subnet_delete(p_cmd->payload.cmd.mesh.subnet_delete.subnet_handle);
    serial_evt_cmd_rsp_data_subnet_t rsp;
    rsp.subnet_handle = p_cmd->payload.cmd.mesh.subnet_delete.subnet_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_subnet_get_all(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_subnet_list_t rsp;
    uint16_t subnet_key_index[sizeof(rsp.subnet_key_index)];
    uint32_t count = ARRAY_SIZE(rsp.subnet_key_index);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = dsm_subnet_get_all(subnet_key_index, &count);
    if (count == 0)
    {
        serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
    }
    else
    {
        memcpy((uint16_t *) rsp.subnet_key_index, (uint16_t *) subnet_key_index, sizeof(uint16_t) * count);
        serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(uint16_t) * count);
    }
}

static void handle_cmd_subnet_count_max_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_list_size_t rsp;
    rsp.list_size = DSM_SUBNET_MAX;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_appkey_add(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_appkey_t rsp;
    dsm_handle_t appkey_handle;
    uint32_t status = dsm_appkey_add(p_cmd->payload.cmd.mesh.appkey_add.app_key_index,
            p_cmd->payload.cmd.mesh.appkey_add.subnet_handle,
            p_cmd->payload.cmd.mesh.appkey_add.key,
            &appkey_handle);
    rsp.appkey_handle = appkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_appkey_update(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_appkey_update(p_cmd->payload.cmd.mesh.appkey_update.appkey_handle,
            p_cmd->payload.cmd.mesh.appkey_update.key);
    serial_evt_cmd_rsp_data_appkey_t rsp;
    rsp.appkey_handle = p_cmd->payload.cmd.mesh.appkey_update.appkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_appkey_delete(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_appkey_delete(p_cmd->payload.cmd.mesh.appkey_delete.appkey_handle);
    serial_evt_cmd_rsp_data_appkey_t rsp;
    rsp.appkey_handle = p_cmd->payload.cmd.mesh.appkey_delete.appkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_appkey_get_all(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_appkey_list_t rsp;
    uint16_t appkey_key_index[sizeof(rsp.appkey_key_index)];
    uint32_t count = ARRAY_SIZE(rsp.appkey_key_index);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = dsm_appkey_get_all(p_cmd->payload.cmd.mesh.appkey_get_all.subnet_handle, appkey_key_index, &count);
    if (count == 0)
    {
        serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
    }
    else
    {
        rsp.subnet_handle = p_cmd->payload.cmd.mesh.appkey_get_all.subnet_handle;
        memcpy((uint16_t *) rsp.appkey_key_index, (uint16_t *) appkey_key_index, sizeof(uint16_t) * count);
        serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode,
                status,
                (uint8_t *) &rsp,
                offsetof(serial_evt_cmd_rsp_data_appkey_list_t, appkey_key_index) + sizeof(uint16_t) * count);
    }
}

static void handle_cmd_appkey_count_max_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_list_size_t rsp;
    rsp.list_size = DSM_APP_MAX;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_devkey_add(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_devkey_t rsp;
    uint16_t devkey_handle;
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = dsm_devkey_add(p_cmd->payload.cmd.mesh.devkey_add.owner_addr,
            p_cmd->payload.cmd.mesh.devkey_add.subnet_handle,
            p_cmd->payload.cmd.mesh.devkey_add.key,
            &devkey_handle);
    rsp.devkey_handle = devkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_devkey_delete(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_devkey_delete(p_cmd->payload.cmd.mesh.devkey_delete.devkey_handle);
    serial_evt_cmd_rsp_data_devkey_t rsp;
    rsp.devkey_handle = p_cmd->payload.cmd.mesh.devkey_delete.devkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_devkey_count_max_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_list_size_t rsp;
    rsp.list_size = DSM_DEVICE_MAX;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_local_unicast_set(const serial_packet_t * p_cmd)
{
    dsm_local_unicast_address_t local = {
        .address_start = p_cmd->payload.cmd.mesh.local_unicast_addr_set.start_address,
        .count         = p_cmd->payload.cmd.mesh.local_unicast_addr_set.count
    };
    uint32_t status = dsm_local_unicast_addresses_set(&local);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
}

static void handle_cmd_addr_local_unicast_get(const serial_packet_t * p_cmd)
{
    dsm_local_unicast_address_t local;
    dsm_local_unicast_addresses_get(&local);
    serial_evt_cmd_rsp_data_addr_local_unicast_t rsp;
    rsp.address_start = local.address_start;
    rsp.count         = local.count;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_get(const serial_packet_t * p_cmd)
{
    nrf_mesh_address_t addr;
    uint32_t status = dsm_address_get(p_cmd->payload.cmd.mesh.addr_get.address_handle, &addr);
    serial_evt_cmd_rsp_data_raw_addr_t rsp = {0};
    uint32_t rsp_size = 0;

    if (status == NRF_SUCCESS)
    {
        rsp.address_handle = p_cmd->payload.cmd.mesh.addr_get.address_handle;
        rsp.addr_type = addr.type;
        rsp.subscribed = nrf_mesh_is_address_rx(&addr);
        rsp.raw_short_addr = addr.value;

        if (addr.p_virtual_uuid)
        {
            memcpy(rsp.virtual_uuid, addr.p_virtual_uuid, NRF_MESH_UUID_SIZE);
            rsp_size = sizeof(rsp);
        }
        else
        {
            rsp_size = sizeof(rsp) - NRF_MESH_UUID_SIZE;
        }
    }
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode,
            status,
            (uint8_t *) &rsp,
            rsp_size);
}

static void handle_cmd_addr_get_all(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_addr_list_t rsp;
    uint16_t address_handles[sizeof(rsp.address_handles)];
    uint32_t count = ARRAY_SIZE(rsp.address_handles);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = dsm_address_get_all(address_handles, &count);
    if (count > 0)
    {
        memcpy((uint16_t *) rsp.address_handles, (uint16_t *) address_handles, sizeof(uint16_t) * count);
        serial_handler_common_cmd_rsp_nodata_on_error(
            p_cmd->opcode, status, (uint8_t *) rsp.address_handles, sizeof(uint16_t) * count);
    }
    else
    {
        serial_handler_common_cmd_rsp_nodata_on_error(
            p_cmd->opcode, status, NULL, 0);
    }
}

static void handle_cmd_addr_nonvirtual_count_max_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_list_size_t rsp;
    rsp.list_size = DSM_NONVIRTUAL_ADDR_MAX;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_virtual_count_max_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_list_size_t rsp;
    rsp.list_size = DSM_VIRTUAL_ADDR_MAX;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_subscription_add(const serial_packet_t * p_cmd)
{
    dsm_handle_t address_handle;
    uint32_t status = dsm_address_subscription_add(p_cmd->payload.cmd.mesh.addr_subscription_add.address, &address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_subscription_add_virtual(const serial_packet_t * p_cmd)
{
    dsm_handle_t address_handle;
    uint32_t status = dsm_address_subscription_virtual_add(p_cmd->payload.cmd.mesh.addr_subscription_add_virtual.uuid, &address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_subscription_remove(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_address_subscription_remove(p_cmd->payload.cmd.mesh.addr_subscription_remove.address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = p_cmd->payload.cmd.mesh.addr_subscription_remove.address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_publication_add(const serial_packet_t * p_cmd)
{
    dsm_handle_t address_handle;
    uint32_t status = dsm_address_publish_add(p_cmd->payload.cmd.mesh.addr_publication_add.address, &address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_publication_add_virtual(const serial_packet_t * p_cmd)
{
    dsm_handle_t address_handle;
    uint32_t status = dsm_address_publish_virtual_add(p_cmd->payload.cmd.mesh.addr_publication_add_virtual.uuid, &address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_addr_publication_remove(const serial_packet_t * p_cmd)
{
    uint32_t status = dsm_address_publish_remove(p_cmd->payload.cmd.mesh.addr_publication_remove.address_handle);
    serial_evt_cmd_rsp_data_addr_t rsp;
    rsp.address_handle = p_cmd->payload.cmd.mesh.addr_publication_remove.address_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_packet_send(const serial_packet_t * p_cmd)
{
    nrf_mesh_tx_params_t tx_params;
    memset(&tx_params, 0, sizeof(tx_params));

    serial_evt_cmd_rsp_data_packet_send_t rsp;
    rsp.token = nrf_mesh_unique_token_get();

    uint32_t status = dsm_address_get(p_cmd->payload.cmd.mesh.packet_send.dst_addr_handle, &tx_params.dst);
    if (status == NRF_SUCCESS)
    {
        dsm_local_unicast_address_t valid_src_addrs;
        dsm_local_unicast_addresses_get(&valid_src_addrs);

        if (p_cmd->payload.cmd.mesh.packet_send.src_addr <  valid_src_addrs.address_start ||
            p_cmd->payload.cmd.mesh.packet_send.src_addr >= valid_src_addrs.address_start + valid_src_addrs.count)
        {
            status = NRF_ERROR_INVALID_ADDR;
        }
        else
        {
#if MESH_FEATURE_LPN_ENABLED
            status = NRF_ERROR_NOT_FOUND;
            if (p_cmd->payload.cmd.mesh.packet_send.friendship_credential_flag)
            {
                status = dsm_tx_friendship_secmat_get(DSM_HANDLE_INVALID, p_cmd->payload.cmd.mesh.packet_send.appkey_handle,
                                                      &tx_params.security_material);
            }

            /* @tagMeshSp section 4.2.2.4:
             *
             * When Publish Friendship Credential Flag is set to 1 and the friendship security material is
             * not available, the master security material shall be used. */
            if (NRF_ERROR_NOT_FOUND == status)
#endif
            {
                status = dsm_tx_secmat_get(DSM_HANDLE_INVALID,
                                           p_cmd->payload.cmd.mesh.packet_send.appkey_handle,
                                           &tx_params.security_material);
            }
            if (status == NRF_SUCCESS)
            {
                tx_params.src       = p_cmd->payload.cmd.mesh.packet_send.src_addr;
                tx_params.ttl       = p_cmd->payload.cmd.mesh.packet_send.ttl;
                tx_params.force_segmented  = p_cmd->payload.cmd.mesh.packet_send.force_segmented;
                tx_params.transmic_size = (nrf_mesh_transmic_size_t) p_cmd->payload.cmd.mesh.packet_send.transmic_size;
                if (p_cmd->length > NRF_MESH_SERIAL_PACKET_OVERHEAD + SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD)
                {
                    tx_params.p_data    = p_cmd->payload.cmd.mesh.packet_send.data;
                    tx_params.data_len  = p_cmd->length - SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD - NRF_MESH_SERIAL_PACKET_OVERHEAD;
                }
                else
                {
                    tx_params.p_data = NULL;
                    tx_params.data_len = 0;
                }
                tx_params.tx_token = rsp.token;

                status = nrf_mesh_packet_send(&tx_params, NULL);
            }
        }
    }
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &rsp, sizeof(rsp));
}

static void handle_cmd_clear(const serial_packet_t * p_cmd)
{
    mesh_stack_config_clear();
    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
}

static void handle_config_devkey_bind(const serial_packet_t * p_cmd)
{
    uint32_t status = config_server_bind(p_cmd->payload.cmd.mesh.config_server_devkey_bind.address_handle);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
}

static void handle_net_state_set(const serial_packet_t * p_cmd)
{
#if PERSISTENT_STORAGE
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, NRF_ERROR_INVALID_STATE, NULL, 0);
#else
    uint32_t status = net_state_iv_index_and_seqnum_block_set(
                            p_cmd->payload.cmd.mesh.net_state_set.iv_index,
                            p_cmd->payload.cmd.mesh.net_state_set.iv_update_in_progress,
                            p_cmd->payload.cmd.mesh.net_state_set.next_seqnum_block);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
#endif
}

static void handle_net_state_get(const serial_packet_t * p_cmd)
{
    mesh_opt_iv_index_persist_data_t iv_index_data;
    mesh_opt_seqnum_persist_data_t   seqnum_data;

    uint32_t status = mesh_config_entry_get(MESH_OPT_NET_STATE_IV_INDEX_EID, (void *)&iv_index_data);
    if (status != NRF_SUCCESS)
    {
        serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, NULL, 0);
        return;
    }

    status = mesh_config_entry_get(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, (void *)&seqnum_data);

    serial_evt_cmd_rsp_data_net_state_get_t rsp = {
        .iv_index = iv_index_data.iv_index,
        .iv_update_in_progress = (uint8_t) iv_index_data.iv_update_in_progress,
        .iv_update_timeout_counter = iv_index_data.iv_update_timeout_counter,
        .next_seqnum_block = seqnum_data.next_block
    };
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *)&rsp, sizeof(rsp));
}

/*****************************************************************************
* Static functions
*****************************************************************************/
/**
 * Opcode-handler and -length lookup table.
 */
static const mesh_serial_cmd_handler_t m_handlers[] =
{
    {SERIAL_OPCODE_CMD_MESH_ENABLE,                         0,                                                       0,  handle_cmd_enable},
    {SERIAL_OPCODE_CMD_MESH_DISABLE,                        0,                                                       0,  handle_cmd_disable},
    {SERIAL_OPCODE_CMD_MESH_SUBNET_ADD,                     sizeof(serial_cmd_mesh_subnet_add_t),                    0,  handle_cmd_subnet_add},
    {SERIAL_OPCODE_CMD_MESH_SUBNET_UPDATE,                  sizeof(serial_cmd_mesh_subnet_update_t),                 0,  handle_cmd_subnet_update},
    {SERIAL_OPCODE_CMD_MESH_SUBNET_DELETE,                  sizeof(serial_cmd_mesh_subnet_delete_t),                 0,  handle_cmd_subnet_delete},
    {SERIAL_OPCODE_CMD_MESH_SUBNET_GET_ALL,                 0,                                                       0,  handle_cmd_subnet_get_all},
    {SERIAL_OPCODE_CMD_MESH_SUBNET_COUNT_MAX_GET,           0,                                                       0,  handle_cmd_subnet_count_max_get},
    {SERIAL_OPCODE_CMD_MESH_APPKEY_ADD,                     sizeof(serial_cmd_mesh_appkey_add_t),                    0,  handle_cmd_appkey_add},
    {SERIAL_OPCODE_CMD_MESH_APPKEY_UPDATE,                  sizeof(serial_cmd_mesh_appkey_update_t),                 0,  handle_cmd_appkey_update},
    {SERIAL_OPCODE_CMD_MESH_APPKEY_DELETE,                  sizeof(serial_cmd_mesh_appkey_delete_t),                 0,  handle_cmd_appkey_delete},
    {SERIAL_OPCODE_CMD_MESH_APPKEY_GET_ALL,                 sizeof(serial_cmd_mesh_appkey_get_all_t),                0,  handle_cmd_appkey_get_all},
    {SERIAL_OPCODE_CMD_MESH_APPKEY_COUNT_MAX_GET,           0,                                                       0,  handle_cmd_appkey_count_max_get},
    {SERIAL_OPCODE_CMD_MESH_DEVKEY_ADD,                     sizeof(serial_cmd_mesh_devkey_add_t),                    0,  handle_cmd_devkey_add},
    {SERIAL_OPCODE_CMD_MESH_DEVKEY_DELETE,                  sizeof(serial_cmd_mesh_devkey_delete_t),                 0,  handle_cmd_devkey_delete},
    {SERIAL_OPCODE_CMD_MESH_DEVKEY_COUNT_MAX_GET,           0,                                                       0,  handle_cmd_devkey_count_max_get},
    {SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_SET,         sizeof(serial_cmd_mesh_addr_local_unicast_set_t),        0,  handle_cmd_addr_local_unicast_set},
    {SERIAL_OPCODE_CMD_MESH_ADDR_LOCAL_UNICAST_GET,         0,                                                       0,  handle_cmd_addr_local_unicast_get},
    {SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD,          sizeof(serial_cmd_mesh_addr_subscription_add_t),         0,  handle_cmd_addr_subscription_add},
    {SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD_VIRTUAL,  sizeof(serial_cmd_mesh_addr_subscription_add_virtual_t), 0,  handle_cmd_addr_subscription_add_virtual},
    {SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_REMOVE,       sizeof(serial_cmd_mesh_addr_subscription_remove_t),      0,  handle_cmd_addr_subscription_remove},
    {SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD,           sizeof(serial_cmd_mesh_addr_publication_add_t),          0,  handle_cmd_addr_publication_add},
    {SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD_VIRTUAL,   sizeof(serial_cmd_mesh_addr_publication_add_virtual_t),  0,  handle_cmd_addr_publication_add_virtual},
    {SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_REMOVE,        sizeof(serial_cmd_mesh_addr_publication_remove_t),       0,  handle_cmd_addr_publication_remove},
    {SERIAL_OPCODE_CMD_MESH_ADDR_GET,                       sizeof(serial_cmd_mesh_addr_get_t),                      0,  handle_cmd_addr_get},
    {SERIAL_OPCODE_CMD_MESH_ADDR_GET_ALL,                   0,                                                       0,  handle_cmd_addr_get_all},
    {SERIAL_OPCODE_CMD_MESH_ADDR_NONVIRTUAL_COUNT_MAX_GET,  0,                                                       0,  handle_cmd_addr_nonvirtual_count_max_get},
    {SERIAL_OPCODE_CMD_MESH_ADDR_VIRTUAL_COUNT_MAX_GET,     0,                                                       0,  handle_cmd_addr_virtual_count_max_get},
    {SERIAL_OPCODE_CMD_MESH_PACKET_SEND, SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD, sizeof(serial_cmd_mesh_packet_send_t) - SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD,  handle_cmd_packet_send},
    {SERIAL_OPCODE_CMD_MESH_STATE_CLEAR,                    0,                                                       0,  handle_cmd_clear},
    {SERIAL_OPCODE_CMD_MESH_CONFIG_SERVER_BIND,             sizeof(serial_cmd_mesh_config_server_devkey_bind_t),     0,  handle_config_devkey_bind},
    {SERIAL_OPCODE_CMD_MESH_NET_STATE_SET,                  sizeof(serial_cmd_mesh_net_state_set_t),                 0,  handle_net_state_set},
    {SERIAL_OPCODE_CMD_MESH_NET_STATE_GET,                  0,                                                       0,  handle_net_state_get}
};

static void mesh_config_listener_cb(mesh_config_change_reason_t reason, mesh_config_entry_id_t id, const void * p_entry)
{
    if (reason == MESH_CONFIG_CHANGE_REASON_SET && id.file == MESH_OPT_NET_STATE_FILE_ID)
    {
        static serial_packet_t * p_serial_evt;

        switch (id.record)
        {
            case MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD:
            {
                uint32_t status = serial_packet_buffer_get(sizeof(serial_evt_mesh_seqnum_entry_set_notification_t) + 1, &p_serial_evt);
                if (status != NRF_SUCCESS)
                {
                    break;
                }
                mesh_opt_seqnum_persist_data_t * p_seqnum_data = (mesh_opt_seqnum_persist_data_t *)p_entry;
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_SEQNUM_ENTRY_SET_NOTIFICATION;
                p_serial_evt->payload.evt.mesh.seqnum_entry_set.next_block = p_seqnum_data->next_block;
                serial_tx(p_serial_evt);
                break;
            }
            case MESH_OPT_NET_STATE_IV_INDEX_RECORD:
            {
                uint32_t status = serial_packet_buffer_get(sizeof(serial_evt_mesh_iv_entry_set_notification_t) + 1, &p_serial_evt);
                if (status != NRF_SUCCESS)
                {
                    break;
                }
                mesh_opt_iv_index_persist_data_t * p_iv_index_data = (mesh_opt_iv_index_persist_data_t *)p_entry;
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_IV_ENTRY_SET_NOTIFICATION;
                p_serial_evt->payload.evt.mesh.iv_entry_set.iv_index = p_iv_index_data->iv_index;
                p_serial_evt->payload.evt.mesh.iv_entry_set.iv_update_in_progress = p_iv_index_data->iv_update_in_progress;
                p_serial_evt->payload.evt.mesh.iv_entry_set.iv_update_timout_counter = p_iv_index_data->iv_update_timeout_counter;
                serial_tx(p_serial_evt);
                break;
            }
            default:
                break;
        }
    }
}

MESH_CONFIG_LISTENER(m_seqnum_listener, MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, mesh_config_listener_cb);
MESH_CONFIG_LISTENER(m_iv_index_listener, MESH_OPT_NET_STATE_IV_INDEX_EID, mesh_config_listener_cb);

static void serial_handler_mesh_evt_handle(const nrf_mesh_evt_t* p_evt)
{
    static serial_packet_t * p_serial_evt;
    uint32_t status;
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_MESSAGE_RECEIVED:
        {
            uint32_t data_length = p_evt->params.message.length;

            if (data_length > SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN)
            {
                data_length = SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN;
            }
            status = serial_packet_buffer_get(data_length + SERIAL_EVT_MESH_MESSAGE_RECEIVED_LEN_OVERHEAD, &p_serial_evt);
            if (status != NRF_SUCCESS)
            {
                break;
            }

            serial_evt_mesh_message_received_t * p_msg_rcvd = &p_serial_evt->payload.evt.mesh.message_received;
            p_msg_rcvd->actual_length = p_evt->params.message.length;
            if (p_evt->params.message.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_UNICAST;
                p_msg_rcvd->dst = p_evt->params.message.dst.value;
            }
            else
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_SUBSCRIPTION;
                dsm_handle_t dst;
                /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
                   may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
                   9-2019-q4-major. */
                NRF_MESH_ASSERT(NRF_SUCCESS == dsm_address_handle_get(&p_evt->params.message.dst, &dst));
                p_msg_rcvd->dst = (uint16_t) dst;
            }
            p_msg_rcvd->src = p_evt->params.message.src.value;
            p_msg_rcvd->appkey_handle = dsm_appkey_handle_get(p_evt->params.message.secmat.p_app);
            p_msg_rcvd->subnet_handle = dsm_subnet_handle_get(p_evt->params.message.secmat.p_net);
            p_msg_rcvd->ttl = p_evt->params.message.ttl;
            p_msg_rcvd->adv_addr_type = p_evt->params.message.p_metadata->params.scanner.adv_addr.addr_type;
            memcpy(p_msg_rcvd->adv_addr, p_evt->params.message.p_metadata->params.scanner.adv_addr.addr, BLE_GAP_ADDR_LEN);
            p_msg_rcvd->rssi = p_evt->params.message.p_metadata->params.scanner.rssi;

            if (data_length > 0)
            {
                memcpy(p_msg_rcvd->data,
                        p_evt->params.message.p_buffer,
                        data_length);
            }
            serial_tx(p_serial_evt);
            break;
        }
        case NRF_MESH_EVT_IV_UPDATE_NOTIFICATION:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_mesh_iv_update_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_IV_UPDATE_NOTIFICATION;
                p_serial_evt->payload.evt.mesh.iv_update.iv_index = p_evt->params.iv_update.iv_index;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_mesh_key_refresh_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_KEY_REFRESH_NOTIFICATION;
                p_serial_evt->payload.evt.mesh.key_refresh.netkey_index = p_evt->params.key_refresh.subnet_index;
                p_serial_evt->payload.evt.mesh.key_refresh.phase = p_evt->params.key_refresh.phase;
                serial_tx(p_serial_evt);
            }
            break;
        case NRF_MESH_EVT_HB_MESSAGE_RECEIVED:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_mesh_hb_message_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_HEARTBEAT_RECEIVED;
                p_serial_evt->payload.evt.mesh.heartbeat.init_ttl = p_evt->params.hb_message.init_ttl;
                p_serial_evt->payload.evt.mesh.heartbeat.hops = p_evt->params.hb_message.hops;
                p_serial_evt->payload.evt.mesh.heartbeat.features = p_evt->params.hb_message.features;
                p_serial_evt->payload.evt.mesh.heartbeat.src = p_evt->params.hb_message.src;
                serial_tx(p_serial_evt);
            }
            break;
        case NRF_MESH_EVT_TX_COMPLETE:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_mesh_tx_complete_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_MESH_TX_COMPLETE;
                p_serial_evt->payload.evt.mesh.tx_complete.token = p_evt->params.tx_complete.token;
                serial_tx(p_serial_evt);
            }
            break;

        default:
            break;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void serial_handler_mesh_init(void)
{
    m_evt_handler.evt_cb = serial_handler_mesh_evt_handle;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

void serial_handler_mesh_rx(const serial_packet_t* p_cmd)
{
    NRF_MESH_ASSERT(p_cmd->opcode >= SERIAL_OPCODE_CMD_RANGE_MESH_START &&
                    p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_MESH_END);
    if (p_cmd->opcode >= SERIAL_OPCODE_CMD_RANGE_MESH_START + (sizeof(m_handlers) / sizeof(mesh_serial_cmd_handler_t)))
    {
        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
    }
    else
    {
        const mesh_serial_cmd_handler_t * p_handler = &m_handlers[p_cmd->opcode - SERIAL_OPCODE_CMD_RANGE_MESH_START];

        /* Make sure our handler array is up to date. */
        NRF_MESH_ASSERT(p_cmd->opcode == p_handler->opcode);

        if (p_cmd->length < SERIAL_PACKET_LENGTH_OVERHEAD + p_handler->min_param_length ||
            p_cmd->length > SERIAL_PACKET_LENGTH_OVERHEAD + p_handler->min_param_length + p_handler->optional_extra_bytes)
        {
            serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
        }
        else
        {
            p_handler->handler_cb(p_cmd);
        }
    }
}
