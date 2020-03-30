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

#ifndef SERIAL_BEARER_H__
#define SERIAL_BEARER_H__

#include <stdbool.h>
#include <stdint.h>

#include "serial_packet.h"

/**
 * @defgroup SERIAL_BEARER Serial hardware abstraction layer
 * @ingroup MESH_SERIAL
 * @{
 */

/**
 * Initializes the serial interface.
 */
void serial_bearer_init(void);

/**
 * Allocates a serial packet from the serial_bearer buffer.
 *
 * @param[in]  packet_len                The value of the packet length field for the serial packet.
 * @param[out] pp_packet                 The allocated packet pointer.
 *
 * @retval     NRF_SUCCESS               The packet is reserved successfully, and pp_packet points
 *                                       to a valid packet pointer.
 * @retval     NRF_ERROR_NO_MEM          The packet buffer does not have enough available memory.
 * @retval     NRF_ERROR_INVALID_LENGTH  The length of the packet requested cannot be 0 or greater
 *                                       than the maximum available packet size.
 */
uint32_t serial_bearer_packet_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet);

/**
 * Allocates a serial packet from the serial_bearer buffer. This call will not return until a buffer
 * is available.
 *
 * @warning This function must be called below timer IRQ context so that timeslot does not trip up.
 *
 * @param[in]  packet_len                The value of the packet length field for the serial packet.
 * @param[out] pp_packet                 The allocated packet pointer.
 *
 * @retval     NRF_SUCCESS               The packet is reserved successfully, and pp_packet points
 *                                       to a valid packet pointer.
 * @retval     NRF_ERROR_INVALID_LENGTH  The length of the packet requested cannot be 0 or greater
 *                                       than the maximum available packet size.
 */
uint32_t serial_bearer_blocking_buffer_get(uint16_t packet_len, serial_packet_t ** pp_packet);

/**
 * Transmits a serial packet.
 *
 * @param[in]  p_packet                  The pointer to the serial packet that shall be transmitted
 */
void serial_bearer_tx(const serial_packet_t * p_packet);

/**
 * Get a packet from the RX queue.
 *
 * @param[in,out] p_packet  Pointer to a packet structure to copy the packet into.
 *
 * @return        true if a complete RX packet is available.
 */
bool serial_bearer_rx_get(serial_packet_t * p_packet);

/**
 * Check if any serial packets have been received from the peer.
 *
 * @return     Whether there are some unprocessed RX packets queued.
 */
bool serial_bearer_rx_pending(void);

/** @} */
#endif /* end of SERIAL_BEARER_H__*/
