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

#ifndef SERIAL_HANDLER_MODELS_H__
#define SERIAL_HANDLER_MODELS_H__

#include "serial_packet.h"
#include "serial_cmd.h"

/**
 * @defgroup SERIAL_HANDLER_MODELS Generic model serial handler
 * @ingroup MESH_SERIAL_HANDLER
 *
 * Helps implement model specific serial handler functionality.
 * @{
 */

/**
 * Initializer callback prototype used by the serial handler to forward init calls to each model.
 *
 * @param[in]  p_init_params   The model initializer parameters
 * @param[out] p_model_handle  The pointer to a valid model handle strucure, by which the allocated
 *                             model handle is returned.
 *
 * @retval     NRF_SUCCESS     Command executed successfully.
 */
typedef uint32_t (*serial_handler_models_model_init_cb_t)(const serial_cmd_model_specific_init_t * p_init_params, access_model_handle_t * p_model_handle);

/**
 * Command callback prototype for processing model specific commands.
 *
 * @param[in]  p_command_params  The command parameters
 * @param[out] p_cmd_rsp         The pointer to a valid command response strucure, to be filled out
 *                               by the model handler.
 *
 * @retval     NRF_SUCCESS       Command executed successfully.
 */
typedef uint32_t (*serial_handler_models_model_command_cb_t)(const serial_cmd_model_specific_command_t * p_command_params, serial_evt_cmd_rsp_data_model_cmd_t * p_cmd_rsp);

/** Information struct for registering a model with the serial handler to make it available over serial. */
typedef struct
{
    access_model_id_t                        model_id;         /**< Model ID of the model to be registered. */
    serial_handler_models_model_init_cb_t    model_initialize; /**< Initializer callback of the model registered. */
    serial_handler_models_model_command_cb_t model_command;    /**< Command parser for the model registered. */
} serial_handler_models_info_t;

/**
 * Handle model specific serial commands.
 *
 * @param[in]  p_cmd  Serial command to handle.
 */
void serial_handler_models_rx(const serial_packet_t* p_cmd);

/**
 * Registers a model with the serial handler so that the models presence can be seen via serial and
 * commands can be forwarded to the model.
 *
 * @param[in]  p_model_info    The model information required to register the model with the serial
 *                             handler.
 *
 * @retval     NRF_SUCCESS     The model is successfully registered.
 * @retval     NRF_ERROR_NULL  The callbacks must be valid function pointers.
 *
 */
uint32_t serial_handler_models_register(const serial_handler_models_info_t * p_model_info);

/** @} */

#endif /* SERIAL_HANDLER_MODELS_H__ */

