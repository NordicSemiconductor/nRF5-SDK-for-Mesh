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

#ifndef LIGHT_LC_STATE_UTILS_H__
#define LIGHT_LC_STATE_UTILS_H__

#include "mesh_stack.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"
#include "mesh_opt.h"

#include "light_lc_setup_server.h"
#include "nrf_mesh_config_core.h"

/**
 * @internal
 * @defgroup LIGHT_LC_STATE_UTILS Light LC server model utility functions
 * @ingroup LIGHT_LC_MODELS
 * @{
 */

/** Function to retrieve the flash ID for the requested LC state.
 *
 * This function searches through the property info array looking for the requested lc_state and
 * returns the associated flash mesh_config_entry.
 *
 * @param[in]  lc_state         The requested LC state.
 * @param[out] p_id             Pointer to a buffer to copy the entry id into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS          If the entry id for a given lc state is found.
 * @retval NRF_ERROR_NULL       If NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND  If the entry id for a given lc state is not found.
 */
uint32_t light_lc_state_utils_flash_id_from_lc_state(light_lc_state_t lc_state, mesh_config_entry_id_t * p_id);

/** Function to retrieve the property ID for the requested LC state.
 *
 * This function searches through the property info array looking for the requested lc_state and
 * returns the associated property ID.
 *
 * @param[in] lc_state          The requested LC state.
 * @param[out] p_property_id    Pointer to the property_id associated with the LC state.
 *
 * @retval NRF_SUCCESS          If the property ID is found.
 * @retval NRF_ERROR_NULL       If NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND  If the property ID is not found.
 */
uint32_t light_lc_state_utils_property_id_from_lc_state(light_lc_state_t lc_state, uint16_t * p_property_id);

/** Function to retrieve the LC state for the requested property ID.
 *
 * This function searches through the property info array looking for the requested property ID and
 * returns the associated LC state.
 *
 * @param[in] property_id       The requested property ID.
 * @param[out] p_lc_state       Pointer to the LC state associated with the property ID.
 *
 * @retval NRF_SUCCESS          If the property ID is found.
 * @retval NRF_ERROR_NULL       If NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND  If the property ID is not found.
 */
uint32_t light_lc_state_utils_lc_state_from_property_id(uint16_t property_id, light_lc_state_t * p_lc_state);

/** Function to retrieve the property's data length, given the property ID.
 *
 * This function searches through the property info array looking for the requested property ID and
 * returns the associated property's data length.
 *
 * @param[in]  property_id        The requested LC state.
 * @param[out] p_property_length  Returns the property's length
 *
 * @retval NRF_SUCCESS            If the property ID is found.
 * @retval NRF_ERROR_NULL         If NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND    If the property ID is not found.
 */
uint32_t light_lc_state_utils_property_data_size_get(uint16_t property_id, uint8_t * p_property_length);

/** Function to retrieve the flash ID for the requested property ID.
 *
 * This function searches through the property info array looking for the requested property ID and
 * returns the associated mesh_config_entry.
 *
 * @param[in]  property_id          The requested LC state.
 * @param[out] p_id                 Pointer to a buffer to copy the entry id into. Cannot be NULL.
 *
 * @retval NRF_SUCCESS          If the entry id for a given property ID is found.
 * @retval NRF_ERROR_NULL       If NULL pointer given to function.
 * @retval NRF_ERROR_NOT_FOUND  If the entry id for a given property ID is not found.
 */
uint32_t light_lc_state_utils_property_flash_id_get(uint16_t property_id, mesh_config_entry_id_t * p_id);

/** Function to retrieve the string for the requested property ID.
 *
 * Set @c LIGHT_LC_STATE_UTILS_DEBUG to 1 to retrieve the string.
 *
 * This function searches through the property info array looking for the requested property ID and
 * returns the associated string.
 *
 * @param[in] property_id         The requested LC state.
 *
 * @returns Pointer to a null-terminated array of characters for this entry, or NULL if not found.
 */
char * light_lc_state_utils_property_name_string_get(uint16_t property_id);

/** Function to determine if the LC server is controlling the light.
 *
 * This function returns the current value for the LC mode (see @tagMeshMdlSp section 6.2.3.1).
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns true if the server is disabled, false if the server is enabled.
 */
bool light_lc_state_utils_server_control_is_disabled(light_lc_setup_server_t * p_s_server);

/* State Getter functions - called from lc server code to call the mid-app callback */

/** Function to get the current LC mode value.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The current LC mode value.
 */
uint8_t light_lc_state_utils_mode_get(light_lc_setup_server_t * p_s_server);

/** Function to get the current LC occupancy mode value.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The current LC occupancy mode value.
 */
uint8_t light_lc_state_utils_occ_mode_get(light_lc_setup_server_t * p_s_server);

/** Function to get the current LC Light on/off value.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The current LC Light on/off value.
 */
light_lc_light_onoff_status_params_t light_lc_state_utils_light_onoff_get(light_lc_setup_server_t * p_s_server);

/** Function to get the current value for the given property ID.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] property_id       The requested property ID.
 *
 * @returns The current value of the requested property ID.
 */
uint32_t light_lc_state_utils_property_get(light_lc_setup_server_t * p_s_server, uint16_t property_id);

/** Function to set the value for the LC mode.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set LC mode.
 */
void light_lc_state_utils_mode_set(light_lc_setup_server_t * p_s_server, uint8_t set_value);

/** Function to set the value for the LC occupacy mode.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set LC occupancy mode.
 */
void light_lc_state_utils_occ_mode_set(light_lc_setup_server_t * p_s_server, uint8_t set_value);

/** Function to set the value for the LC Light on/off.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set LC Light on/off.
 */
void light_lc_state_utils_light_onoff_set(light_lc_setup_server_t * p_s_server, uint8_t set_value);

/** Function to set the value for a requested property.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set LC Light on/off.
 * @param[in] property_id       The property ID to set.
 */
void light_lc_state_utils_property_set(light_lc_setup_server_t * p_s_server,
                                       uint32_t set_value,
                                       uint16_t property_id);

/* LC local states getters and setters (no mid-app callback involved) - none of these values are
 * ever stored in flash*/

/* getters */

/** Function to get the value for the ambient luxlevel.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The value for the ambient luxlevel.
 */
uint32_t light_lc_state_utils_ambient_luxlevel_get(light_lc_setup_server_t * p_s_server);

/** Function to get the value for Luxlevel out
 *
 * See @tagMeshMdlSp section 6.2.5.13.3 for details on Luxlevel out.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The value for Luxlevel out
 */
uint32_t light_lc_state_utils_luxlevel_out_get(light_lc_setup_server_t * p_s_server);

/** Function to get the value for Luxlevel out
 *
 * See @tagMeshMdlSp section 6.2.5.13.2 for details on Lightness out.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns The value for Lightness out.
 */
uint32_t light_lc_state_utils_lightness_out_get(light_lc_setup_server_t * p_s_server);

/** Function to return the current valid status of the ambient luxlevel value.
 *
 * @param[in] p_s_server          Pointer to the model structure.
 *
 * @returns true if the ambient luxlevel value has been set since boot time, false otherwise.
 */
bool light_lc_state_utils_ambient_luxlevel_is_valid(light_lc_setup_server_t * p_s_server);

/* setters */

/** Function to set the value for the ambient luxlevel.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set.
 */
void light_lc_state_utils_ambient_luxlevel_set(light_lc_setup_server_t * p_s_server, uint32_t set_value);

/** Function to set the value for Luxlevel out.
 *
 * See @tagMeshMdlSp section 6.2.5.13.3 for details on Luxlevel out.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set.
 */
void light_lc_state_utils_luxlevel_out_set(light_lc_setup_server_t * p_s_server, uint32_t set_value);


/** Function to set the value for Lightness out.
 *
 * See @tagMeshMdlSp section 6.2.5.13.2 for details on Lightness out.
 *
 * @param[in] p_s_server        Pointer to the model structure.
 * @param[in] set_value         The value to set.
 */
void light_lc_state_utils_lightness_out_set(light_lc_setup_server_t * p_s_server, uint16_t set_value);

/**@} end of LIGHT_LC_STATE_UTILS */
#endif /* LIGHT_LC_STATE_UTILS_H__ */
