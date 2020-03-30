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
#ifndef AD_LISTENER_H_
#define AD_LISTENER_H_

#include <stdint.h>
#include "packet.h"
#include "nrf_mesh.h"
#include "scanner.h"
#include "nrf_mesh_section.h"

/**
 * @defgroup AD_LISTENER AD listener
 * @ingroup MESH_API_GROUP_BEARER
 * De-multiplexing of the advertising data.
 *
 * Provides a listener-based interface for the incoming advertisement packets.
 * @{
 */

/* Wildcard value enables any AD type. */
#define ADL_WILDCARD_AD_TYPE   0u
/* Wildcard value enables any advertisement packet type. */
#define ADL_WILDCARD_ADV_TYPE  (ble_packet_type_t)0xFFu

typedef void (* ad_handler_t)(const uint8_t * p_packet,
                              uint32_t ad_packet_length,
                              const nrf_mesh_rx_metadata_t * p_metadata);

typedef struct
{
    /* AD value from the specification or ADL_WILDCARD_AD_TYPE. */
    uint8_t           ad_type;
    ble_packet_type_t adv_packet_type;
    ad_handler_t      handler;
} ad_listener_t;

/**
 * Define an AD listener.
 *
 * Its value should be assigned as part of the declaration:
 * @code{.c}
 * AD_LISTENER(my_listener) = {
 *     .ad_type = AD_TYPE_BEACON,
 *     .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND,
 *     .handler = my_handler_function,
 * };
 * @endcode
 */
#define AD_LISTENER(name) NRF_MESH_SECTION_ITEM_REGISTER_FLASH(ad_listeners, const ad_listener_t name)

/**
 * Initialize the AD listener module.
 */
void ad_listener_init(void);

/**
 * Process the incoming data from the scanner.
 *
 * The function reads out the received frame from scanner, performs preliminary
 * parsing of AD fields and sends out parsed AD frames to subscribers.
 *
 * @warning The listener shall be subscribed previously.
 *
 * @param[in] adv_type Advertising type for the packet.
 * @param[in] p_payload Pointer to the packet payload.
 * @param[in] payload_length The length of the given payload.
 * @param[in] p_metadata Metadata attached to the packet.
 */
void ad_listener_process(ble_packet_type_t adv_type, const uint8_t * p_payload, uint32_t payload_length, const nrf_mesh_rx_metadata_t * p_metadata);

/** @} */

#endif /* AD_LISTENER_H_ */
