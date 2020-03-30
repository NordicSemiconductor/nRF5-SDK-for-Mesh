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
#ifndef NETWORK_SETUP_H__
#define NETWORK_SETUP_H__

#include "device_state_manager.h"
#include "health_client.h"
#include "nrf_mesh_defines.h"

/** Structure to store the handles data related to provisioned devices */
typedef struct
{
    dsm_handle_t m_self_devkey_handle;
    dsm_handle_t m_netkey_handle;
    dsm_handle_t m_appkey_handle;

    health_client_t m_health_client_instance;
} network_dsm_handles_data_volatile_t;

/** Structure to store the state of the provisioning process of the network nodes */
typedef struct
{
    uint8_t provisioned_devices;
    uint8_t configured_devices;
    uint16_t last_device_address;
    uint8_t client_counter;
    uint8_t onoff_server_counter;
    uint8_t level_server_counter;
    uint8_t ll_server_counter;
    uint8_t lc_server_counter;
    uint8_t ctl_server_counter;
    uint8_t ctl_lc_server_counter;
    uint8_t sensor_server_counter;

    const char * current_uri;

    uint8_t  netkey[NRF_MESH_KEY_SIZE];
    uint8_t  appkey[NRF_MESH_KEY_SIZE];
    uint8_t  self_devkey[NRF_MESH_KEY_SIZE];
} network_stats_data_stored_t;

#endif /* NETWORK_SETUP_H__ */
