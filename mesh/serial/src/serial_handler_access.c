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

#include "serial_handler_access.h"

#include <stdint.h>

#include "nrf_mesh_config_serial.h"
#include "nrf_mesh_defines.h"
#include "serial.h"
#include "serial_status.h"
#include "serial_cmd.h"
#include "serial_cmd_rsp.h"
#include "serial_handler_common.h"
#include "access_config.h"
#include "access.h"

/*****************************************************************************
 * Static asserts: make sure that our assumptions on packet sizes are true
 *****************************************************************************/
NRF_MESH_STATIC_ASSERT(sizeof(access_model_id_t) == sizeof(uint32_t));
NRF_MESH_STATIC_ASSERT(sizeof(dsm_handle_t) == sizeof(uint16_t));
NRF_MESH_STATIC_ASSERT(sizeof(access_model_handle_t) == sizeof(uint16_t));
NRF_MESH_STATIC_ASSERT(ACCESS_PUBLISH_RESOLUTION_MAX <= UINT8_MAX);


/*****************************************************************************
 * Command handlers
 *****************************************************************************/
static void model_pub_addr_set(const serial_packet_t * p_cmd)
{
    uint32_t status = access_model_publish_address_set(p_cmd->payload.cmd.access.handle_pair.model_handle, p_cmd->payload.cmd.access.handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_pub_addr_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_pub_addr_get_t response;
    dsm_handle_t dsm_handle;
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_model_publish_address_get(p_cmd->payload.cmd.access.model_handle.handle, (dsm_handle_t *) &dsm_handle);
    response.addr_handle = dsm_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void model_pub_period_set(const serial_packet_t * p_cmd)
{
    uint32_t status = access_model_publish_period_set(p_cmd->payload.cmd.access.publish_period.model_handle, (access_publish_resolution_t) p_cmd->payload.cmd.access.publish_period.resolution, p_cmd->payload.cmd.access.publish_period.step_number);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_pub_period_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_pub_period_get_t response;
    uint32_t status = access_model_publish_period_get(p_cmd->payload.cmd.access.model_handle.handle, (access_publish_resolution_t *) &response.resolution, &response.step_number);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void model_subs_add(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_subscription_add(p_access_msg->handle_pair.model_handle, p_access_msg->handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_subs_remove(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_subscription_remove(p_access_msg->handle_pair.model_handle, p_access_msg->handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_subs_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_subs_get_t response;
    dsm_handle_t address_handles[sizeof(response.address_handles)];
    uint16_t count = ARRAY_SIZE(response.address_handles);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_model_subscriptions_get(p_cmd->payload.cmd.access.model_handle.handle, address_handles, &count);
    if (NRF_SUCCESS == status || status == NRF_ERROR_INVALID_LENGTH)
    {
        response.count = count;
        memcpy((dsm_handle_t *)response.address_handles, (dsm_handle_t *)address_handles, sizeof(dsm_handle_t) * count);
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), (uint8_t *) &response, sizeof(response));
    }
    else
    {
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
    }
}

static void model_app_bind(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_application_bind(p_access_msg->handle_pair.model_handle, p_access_msg->handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_app_unbind(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_application_unbind(p_access_msg->handle_pair.model_handle, p_access_msg->handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_app_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_apps_get_t response;
    dsm_handle_t appkey_handles[sizeof(response.appkey_handles)];
    uint16_t count = ARRAY_SIZE(response.appkey_handles);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_model_applications_get(p_cmd->payload.cmd.access.model_handle.handle, appkey_handles, &count);
    if (NRF_SUCCESS == status || status == NRF_ERROR_INVALID_LENGTH)
    {
        response.count = count;
        memcpy((dsm_handle_t *)response.appkey_handles, (dsm_handle_t *) appkey_handles, sizeof(dsm_handle_t) * count);
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), (uint8_t *) &response, sizeof(response));
    }
    else
    {
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
    }
}

static void model_pub_app_set(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_publish_application_set(p_access_msg->handle_pair.model_handle, p_access_msg->handle_pair.dsm_handle);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_pub_app_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_pub_app_get_t response;
    dsm_handle_t appkey_handle;
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_model_publish_application_get(p_cmd->payload.cmd.access.model_handle.handle, &appkey_handle);
    response.appkey_handle = appkey_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void model_pub_ttl_set(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_model_publish_ttl_set(p_access_msg->model_ttl.model_handle, p_access_msg->model_ttl.ttl);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void model_pub_ttl_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_pub_ttl_get_t response;
    uint32_t status = access_model_publish_ttl_get(p_cmd->payload.cmd.access.model_handle.handle, &response.ttl);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void elem_loc_set(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    uint32_t status = access_element_location_set(p_access_msg->elem_loc.element_index, p_access_msg->elem_loc.location);
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

static void elem_loc_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_elem_loc_get_t response;
    uint16_t location; 
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_element_location_get(p_cmd->payload.cmd.access.index.element_index, &location);
    response.location = location;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void elem_sig_model_cnt_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_elem_model_count_get_t response;
    uint32_t status = access_element_sig_model_count_get(p_cmd->payload.cmd.access.index.element_index, &response.model_count);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void elem_vendor_model_cnt_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_elem_model_count_get_t response;
    uint32_t status = access_element_vendor_model_count_get(p_cmd->payload.cmd.access.index.element_index, &response.model_count);
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void model_id_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_model_id_get_t response;
    access_model_id_t model_id;
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_model_id_get(p_cmd->payload.cmd.access.model_handle.handle, &model_id);
    response.model_id = model_id;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void handle_get(const serial_packet_t * p_cmd)
{
    const serial_cmd_access_t * p_access_msg = &p_cmd->payload.cmd.access;
    serial_evt_cmd_rsp_data_model_handle_get_t response;
    access_model_handle_t model_handle;
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_handle_get(p_access_msg->handle_get.element_index, p_access_msg->handle_get.model_id, &model_handle);
    response.model_handle = model_handle;
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &response, sizeof(response));
}

static void elem_models_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_elem_models_get_t response;
    access_model_handle_t model_handles[sizeof(response.model_handles)];
    uint16_t count = ARRAY_SIZE(response.model_handles);
    /* Can not use response directly because taking address of packed member of 'struct <anonymous>'
       may result in an unaligned pointer value. Compiler warning with GNU Tools ARM Embedded
       9-2019-q4-major. */
    uint32_t status = access_element_models_get(p_cmd->payload.cmd.access.index.element_index, model_handles, &count);
    if (NRF_SUCCESS == status || status == NRF_ERROR_INVALID_LENGTH)
    {
        response.count = count;
        memcpy((access_model_handle_t *)response.model_handles, (access_model_handle_t *)model_handles, sizeof(access_model_handle_t) * count);
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), (uint8_t *) &response, sizeof(response));
    }
    else
    {
        (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
    }
}

/*****************************************************************************
 * Callback table to be used with serial_handler_common_rx
 *****************************************************************************/

#define HANDLE_PAIR_T_SIZE       sizeof(serial_cmd_access_handle_pair_t)
#define MODEL_HANDLE_T_SIZE      sizeof(serial_cmd_access_model_handle_t)
#define PUB_PERIOD_SET_T_SIZE    sizeof(serial_cmd_access_pub_period_set_t)
#define MODEL_PUB_TTL_SET_T_SIZE sizeof(serial_cmd_access_model_pub_ttl_set_t)
#define ELEMENT_LOC_SET_T_SIZE   sizeof(serial_cmd_access_element_loc_set_t)
#define ELEMENT_INDEX_T_SIZE     sizeof(serial_cmd_access_element_index_t)
#define HANDLE_GET_T_SIZE        sizeof(serial_cmd_access_handle_get_t)

/* Serial command handler lookup table. */
static const serial_handler_common_opcode_to_fp_map_t m_cmd_handlers[] =
{
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_SET,          HANDLE_PAIR_T_SIZE,       0, model_pub_addr_set},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_GET,          MODEL_HANDLE_T_SIZE,      0, model_pub_addr_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_SET,        PUB_PERIOD_SET_T_SIZE,    0, model_pub_period_set},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_GET,        MODEL_HANDLE_T_SIZE,      0, model_pub_period_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_ADD,              HANDLE_PAIR_T_SIZE,       0, model_subs_add},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_REMOVE,           HANDLE_PAIR_T_SIZE,       0, model_subs_remove},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET,              MODEL_HANDLE_T_SIZE,      0, model_subs_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_BIND,              HANDLE_PAIR_T_SIZE,       0, model_app_bind},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_UNBIND,            HANDLE_PAIR_T_SIZE,       0, model_app_unbind},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET,               MODEL_HANDLE_T_SIZE,      0, model_app_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_SET,           HANDLE_PAIR_T_SIZE,       0, model_pub_app_set},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_GET,           MODEL_HANDLE_T_SIZE,      0, model_pub_app_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_SET,           MODEL_PUB_TTL_SET_T_SIZE, 0, model_pub_ttl_set},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_GET,           MODEL_HANDLE_T_SIZE,      0, model_pub_ttl_get},
    {SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_SET,                ELEMENT_LOC_SET_T_SIZE,   0, elem_loc_set},
    {SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_GET,                ELEMENT_INDEX_T_SIZE,     0, elem_loc_get},
    {SERIAL_OPCODE_CMD_ACCESS_ELEM_SIG_MODEL_COUNT_GET,    ELEMENT_INDEX_T_SIZE,     0, elem_sig_model_cnt_get},
    {SERIAL_OPCODE_CMD_ACCESS_ELEM_VENDOR_MODEL_COUNT_GET, ELEMENT_INDEX_T_SIZE,     0, elem_vendor_model_cnt_get},
    {SERIAL_OPCODE_CMD_ACCESS_MODEL_ID_GET,                MODEL_HANDLE_T_SIZE,      0, model_id_get},
    {SERIAL_OPCODE_CMD_ACCESS_HANDLE_GET,                  HANDLE_GET_T_SIZE,        0, handle_get},
    {SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET,             ELEMENT_INDEX_T_SIZE,     0, elem_models_get}
};

void serial_handler_access_rx(const serial_packet_t* p_cmd)
{
    NRF_MESH_ASSERT(p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_ACCESS_END);
    serial_handler_common_rx(p_cmd, m_cmd_handlers, sizeof(m_cmd_handlers) / sizeof(m_cmd_handlers[0]));
}
