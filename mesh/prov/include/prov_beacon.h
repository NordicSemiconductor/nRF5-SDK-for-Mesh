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

#ifndef PROV_BEACON_H__
#define PROV_BEACON_H__

#include <stdint.h>
#include "packet.h"
#include "uri.h"
#include "advertiser.h"
#include "nrf_mesh_prov_events.h"

/**
 * @defgroup PROV_BEACON Provisioning Beacon
 * @ingroup MESH_PROV
 * Handles the unprovisioned beacon reception and building.
 * @{
 */

/**
 * Build a provisioning beacon packet.
 *
 * @param[in] p_adv    Advertiser to use for the provisioning beacon.
 * @param[in] p_uri    A 0-terminated URI string or @c NULL if the field should be
 *                     omitted. The string length cannot exceed @ref URI_DATA_MAXLEN.
 * @param[in] oob_info A bitfield of available OOB locations. See @ref NRF_MESH_PROV_OOB_INFO_SOURCES.
 *
 * @return A pointer to a pre-filled packet structure with a single reference.
 */
adv_packet_t * prov_beacon_unprov_build(advertiser_t * p_adv, const char * p_uri, uint16_t oob_info);

/**
 * Set whether the module should report the incoming unprovisioned beacons to
 * the application. On by default.
 *
 * @param[in] unprov_report Whether the unprovisioned beacons should be
 * reported to the application.
 */
void prov_beacon_unprov_report(bool unprov_report);

/**
 * Handle an incoming unprovisioned beacon packet.
 *
 * @param[in] p_data Pointer to provisioning beacon data.
 * @param[in] length Length of the provisioning data pointed to by the @c p_data parameter.
 * @param[in] p_meta The metadata tied to the packet the given beacon data came from.
 */
void prov_beacon_unprov_packet_in(const uint8_t * p_data, uint8_t length, const nrf_mesh_rx_metadata_t * p_meta);

/**
 * Starts scanning for unprovisioned beacons.
 *
 * @param[in] event_handler Provisioning event handler callback function pointer.
 */
void prov_beacon_scan_start(nrf_mesh_prov_evt_handler_cb_t event_handler);

/**
 * Stops scanning for unprovisioned beacons.
 */
void prov_beacon_scan_stop(void);

/** @} */

#endif /* PROV_BEACON_H__ */

