/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#ifndef PROVISIONER_H__
#define PROVISIONER_H__

#include "nrf_mesh_prov_events.h"
#include "config_client.h"

#include "light_switch_example_common.h"

/**
 * @defgroup PROVISIONER Module encapsulating the provisioner role
 * @{
 */

#define APPKEY {0x5f, 0x11, 0x6e, 0x6f, 0x72, 0x64, 0x69, 0x63, 0x5f, 0x5f, 0x73, 0x65, 0x6d, 0x69, 0x5f, 0x5f}
#define NETKEY {0x5f, 0x5f, 0x6e, 0x6f, 0x72, 0x64, 0x69, 0x63, 0x5f, 0x5f, 0x73, 0x65, 0x6d, 0x69, 0x5f, 0x5f}
#define APPKEY_INDEX (0)
#define NETKEY_INDEX (0)

#define GROUP_ADDRESS (0xCAFE)
#define PROVISIONER_ADDRESS  (0x0001)
#define UNPROV_START_ADDRESS (0x0100)

/** Initialize the provisioner. */
void provisioner_init(void);

/**
 * Sets the provisioner in "listening mode".
 * Will provision the next unprovisioned device it sees.
 *
 * @param[in] address The start address assigned to the next provisioned device.
 *
 * @note In this example, we know that our light switch servers only have _one_ element,
 *       therefore their unicast addresses will be sequential. This assumption does not hold
 *       in general.
 */
void provisioner_wait_for_unprov(uint16_t address);

/**
 * Sets the provisioner in the configuration state.
 * Runs through a series of configuration steps, e.g., adding application key(s), setting
 * publication state, etc.
 *
 * @param[in] address The unicast address of the root element for the device to be configured.
 */
void provisioner_configure(uint16_t address);

/** Configuration client event callback.  */
void config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length);

/**
 * @defgroup PROVISIONER_USER_CALLBACKS User callbacks
 * @{
 */

/**
 * Callback to user that the provisioning has completed.
 * @param[in] p_prov_data Pointer to provisioning data.
 */
void provisioner_prov_complete_cb(const nrf_mesh_prov_evt_complete_t * p_prov_data);

/** Callback to user that the configuration was successful. */
void provisioner_config_successful_cb(void);

/** Callback to user that the configuration was unsuccessful. */
void provisioner_config_failed_cb(void);

/** @} end of PROVISIONER_USER_CALLBACKS */

/** @} end of PROVISIONER */

#endif /* PROVISIONER_H__ */
