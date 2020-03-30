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

#include "serial_handler_models.h"

#include <stdint.h>

#include "nrf_mesh_config_serial.h"
#include "nrf_mesh_config_app.h"
#include "nrf_mesh_defines.h"
#include "serial.h"
#include "serial_status.h"
#include "serial_cmd.h"
#include "serial_cmd_rsp.h"
#include "serial_handler_common.h"
#include "access_config.h"

static serial_handler_models_info_t m_models_list[ACCESS_MODEL_COUNT];

static serial_handler_models_info_t * registered_model_get(const access_model_id_t * p_model_id)
{
    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        if (m_models_list[i].model_initialize != NULL)
        {
            if (memcmp(p_model_id, &m_models_list[i].model_id, sizeof(access_model_id_t)) == 0)
            {
                return &m_models_list[i];
            }
        }
        else
        {
            break; /* If we have reached a model that's not registered then no need to continue looping */
        }
    }
    return NULL;
}

static void models_get(const serial_packet_t * p_cmd)
{
    serial_evt_cmd_rsp_data_models_get_t models = {0};
    uint32_t status = NRF_SUCCESS;
    uint16_t max_reportable = sizeof(models.model_ids)/sizeof(access_model_id_t);
    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        if (m_models_list[i].model_initialize == NULL)
        {
            break; /* No more to report */
        }
        else if (models.count >= max_reportable)
        {
            status = NRF_ERROR_DATA_SIZE;
            break; /* we have copied more than max_reportable, so break the loop */
        }

        memcpy(&models.model_ids[models.count], &m_models_list[i].model_id, sizeof(access_model_id_t));
        models.count++;
    }

    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &models, sizeof(models.count) + sizeof(access_model_id_t) * models.count);
}

static void model_init(const serial_packet_t * p_cmd)
{
    const serial_cmd_model_specific_init_t * p_init_params = &p_cmd->payload.cmd.access.model_init;
    serial_handler_models_info_t * p_targeted_model = registered_model_get(&p_init_params->model_init_info.model_id);

    uint32_t status;
    serial_evt_cmd_rsp_data_model_init_t model_handle = {0};
    if (NULL == p_targeted_model)
    {
        status = NRF_ERROR_NOT_FOUND;
    }
    else
    {
        access_model_handle_t handle;
        status = p_targeted_model->model_initialize(p_init_params, &handle);
        model_handle.model_handle = handle;
    }
    serial_handler_common_cmd_rsp_nodata_on_error(p_cmd->opcode, status, (uint8_t *) &model_handle, sizeof(model_handle));
}

static void model_command(const serial_packet_t * p_cmd)
{
    const serial_cmd_model_specific_command_t * p_cmd_params = &p_cmd->payload.cmd.access.model_cmd;
    access_model_id_t model_id;
    uint32_t status = access_model_id_get(p_cmd_params->model_cmd_info.model_handle, &model_id);
    if (NRF_SUCCESS == status)
    {
        serial_handler_models_info_t * p_targeted_model = registered_model_get(&model_id);

        if (NULL == p_targeted_model)
        {
            status = NRF_ERROR_NOT_FOUND;
        }
        else
        {
            serial_evt_cmd_rsp_data_model_cmd_t cmd_rsp;
            /* TODO: Decide if the data_len be ommitted from the command response message? */
            cmd_rsp.data_len = 0;
            status = p_targeted_model->model_command(p_cmd_params, &cmd_rsp);
            if (cmd_rsp.data_len > 0)
            {
                (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), (uint8_t *) &cmd_rsp, cmd_rsp.data_len + sizeof(cmd_rsp.data_len));
                return;
            }
        }
    }
    (void) serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(status), NULL, 0);
}

#define MODEL_INIT_SIZE_MIN         sizeof(serial_cmd_model_specific_init_header_t)
#define MODEL_INIT_DATA_SIZE_MAX    (NRF_MESH_SERIAL_PAYLOAD_MAXLEN - MODEL_INIT_SIZE_MIN)
#define MODEL_COMMAND_SIZE_MIN      sizeof(serial_cmd_model_specific_command_header_t)
#define MODEL_COMMAND_DATA_SIZE_MAX (NRF_MESH_SERIAL_PAYLOAD_MAXLEN - MODEL_COMMAND_SIZE_MIN)
/* Serial command handler lookup table. */
static const serial_handler_common_opcode_to_fp_map_t m_cmd_handlers[] =
{
    {SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET,                   0,                           0, models_get},
    {SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT,       MODEL_INIT_SIZE_MIN,    MODEL_INIT_DATA_SIZE_MAX, model_init},
    {SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, MODEL_COMMAND_SIZE_MIN, MODEL_COMMAND_DATA_SIZE_MAX, model_command}
};

void serial_handler_models_rx(const serial_packet_t* p_cmd)
{
    NRF_MESH_ASSERT(p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_END);
    serial_handler_common_rx(p_cmd, m_cmd_handlers, sizeof(m_cmd_handlers) / sizeof(m_cmd_handlers[0]));
}

uint32_t serial_handler_models_register(const serial_handler_models_info_t * p_model_info)
{
    if (p_model_info == NULL ||
        p_model_info->model_initialize == NULL ||
        p_model_info->model_command == NULL)
    {
        return NRF_ERROR_NULL;
    }

    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        if (m_models_list[i].model_initialize == NULL)
        {
            memcpy(&m_models_list[i], p_model_info, sizeof(serial_handler_models_info_t));
            return NRF_SUCCESS;
        }
    }
    return NRF_ERROR_NO_MEM;
}

#ifdef UNIT_TEST
void serial_handler_models_reset(void)
{
    memset(m_models_list, 0, sizeof(m_models_list));
}
#endif
