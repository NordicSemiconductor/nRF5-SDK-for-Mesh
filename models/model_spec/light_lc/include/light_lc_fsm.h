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

#ifndef LIGHT_LC_FSM_H__
#define LIGHT_LC_FSM_H__

#include <stdint.h>

#include "light_lc_setup_server.h"

/**
 * @internal
 * @defgroup LIGHT_LC_FSM Light LC Finite State Machine
 * @ingroup LIGHT_LC_MODELS
 *
 * The behavior of the Light Lightness controller is described in terms of a finite state machine.
 * See @tagMeshMdlSp section 6.2.5 for more information.
 *
 * @{
 */

/** Function to inform the FSM that an occupancy event has occurred.
 *
 * When any of the occupancy sensors detects an occupancy event, this function will be called to
 * notify the FSM.
 *
 * @note There is nothing to do when we receive an occupancy 0 event, according to the mesh model
 *       spec, so this should only be called when occupancy is on.
 *
 * @param[in] p_s_server            Pointer to the model structure.

 * @retval NRF_SUCCESS              If the FSM was updated successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_STATE  The FSM has not been initialized.
 */
uint32_t light_lc_fsm_occupancy_event_generate(light_lc_setup_server_t * p_s_server);

/** Function to inform the FSM that a mode event has occurred.
 *
 * @param[in] p_s_server            Pointer to the model structure.
 * @param[in] mode_onoff            false means mode off, true means mode on.
 *
 * @retval NRF_SUCCESS              If the FSM was updated successfully.
 * @retval NRF_ERROR_INVALID_STATE  The FSM has not been initialized.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect message parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 */
uint32_t light_lc_fsm_mode_on_off_event_generate(light_lc_setup_server_t * p_s_server, bool mode_onoff);

/** Function to inform the FSM that a light on/off event has occurred.
 *
 * @param[in] p_s_server            Pointer to the model structure.
 * @param[in] light_onoff           false means light off, true means light on.
 * @param[in] p_transition          Transition information for light OnOff and lightness levels.
 *
 * @retval NRF_SUCCESS              If the FSM was updated successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_STATE  The FSM has not been initialized.
 */
uint32_t light_lc_fsm_light_on_off_event_generate(light_lc_setup_server_t * p_s_server, bool light_onoff,
                                            model_transition_t * p_transition);

/** Function to initialize the FSM - should only be called 1 time per boot
 *
 * @param[in] p_s_server            Pointer to the model structure.
 * @param[in] initial_fsm_onoff     Setting this to true enables the state machine after
 *                                  initialization, otherwise the state machine is kept disabled.
 *
 * @retval NRF_SUCCESS              If FSM started successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_PARAM  Specified timeout is too short (see @ref model_timer_schedule).
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 */
uint32_t light_lc_fsm_init(light_lc_setup_server_t * p_s_server, bool initial_fsm_onoff);

/**@} end of LIGHT_LC_FSM */
#endif /* LIGHT_LC_FSM_H__ */
