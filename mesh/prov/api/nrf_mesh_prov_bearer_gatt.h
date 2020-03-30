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

#ifndef NRF_MESH_PROV_BEARER_GATT_H__
#define NRF_MESH_PROV_BEARER_GATT_H__

#include <stdint.h>
#include "timer_scheduler.h"
#include "fsm.h"
#include "nrf_mesh_prov_bearer.h"
#include "bearer_event.h"

/**
 * @defgroup NRF_MESH_PROV_BEARER_GATT Provisioning over GATT Bearer (PB-GATT)
 * @ingroup NRF_MESH_PROV_BEARER
 * @{
 *
 * This module provides support for provisioning over GATT. PB-GATT is used for provisioning mesh
 * devices by (legacy) devices that only support the GATT bearer.
 *
 * @note The GATT service exposed by this module _shall_ be removed from the GATT table after the
 * device has been successfully provisionined. As of the current Nordic SoftDevice versions, this is
 * only possible by "restarting" it. This means that the user has to disable and re-enable the
 * SoftDevice after the provisioning has successfully completed.
 *
 * @note The GATT connection should support an MTU of 69 bytes or more when PB-GATT is used, ref.
 * @tagMeshSp section 7.1.2.2.2.
 */

/**
 * @defgroup PB_GATT_UUIDS PB-GATT UUID definitions
 * @{
 */

#define NRF_MESH_PB_GATT_SERVICE_UUID  (0x1827)
#define NRF_MESH_PB_GATT_CHAR_IN_UUID  (0x2ADB)
#define NRF_MESH_PB_GATT_CHAR_OUT_UUID (0x2ADC)

/** @} end of PB_GATT_UUIDS */

/** PB-GATT context structure. */
typedef struct
{
    fsm_t fsm;
    uint16_t conn_index;
    timer_event_t link_timeout_event;
    uint32_t link_timeout_us;
    bearer_event_sequential_t bearer_event_seq;
    prov_bearer_t bearer;
} nrf_mesh_prov_bearer_gatt_t;

/**
 * Initializes and sets up the PB-GATT service (through the Mesh GATT interface).
 *
 * @param[in,out] p_bearer_gatt Pointer to PB-GATT bearer context structure.
 *
 * @retval NRF_SUCCESS    Successfully initialized PB-GATT bearer interface.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer in function arguments.
 */
uint32_t nrf_mesh_prov_bearer_gatt_init(nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt);

/**
 * Gets the provisioning bearer interface for the PB-GATT bearer.
 *
 * @note This function is to be used in conjunction with nrf_mesh_prov_bearer_add().
 *
 * @param[in,out] p_bearer_gatt Pointer to a PB-GATT bearer context structure.
 *
 * @returns A pointer to the bearer interface for the PB-GATT bearer.
 */
prov_bearer_t * nrf_mesh_prov_bearer_gatt_interface_get(nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt);


/** @} end of NRF_MESH_PROV_BEARER_GATT GATT */

#endif /* NRF_MESH_PROV_BEARER_GATT_H__ */
