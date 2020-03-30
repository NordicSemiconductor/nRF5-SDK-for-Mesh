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

#ifndef FILTER_ENGINE_H__
#define FILTER_ENGINE_H__

#include <stdbool.h>
#include <stdint.h>

#include "list.h"
#include "scanner.h"

/**
 * @defgroup FILTER Filter engine
 * @ingroup MESH_BEARER
 * @{
 */

/** Filter type. */
typedef enum
{
    FILTER_TYPE_PRE_PROC,   /**< Pre-processing filter. */
    FILTER_TYPE_POST_PROC,  /**< Post-processing filter. */
    FILTER_TYPE_END,        /**< Amount of filter types. */
} filter_type_t;

/**
 * Filter handler to perform a filtering procedure
 *
 * @param[in] p_packet The pointer to a received packet from the scanner rx queue.
 * @param[in] p_data   The pointer to a customized data of the filter instance.
 *
 * @return true if packet shall be filtered, false otherwise.
 */
typedef bool (* filter_handler_t)(scanner_packet_t * p_packet, void * p_data);

/** The filter descriptor. */
typedef struct
{
    filter_type_t    type;       /**< Type that specifies when the filter is to be applied. */
    filter_handler_t handler;    /**< Handler that acts as the entrance point into the filtering algorithm of an instance. */
    void *           p_data;     /**< Pointer to the customized filter data. */
    list_node_t      node;       /**< Service field, set and used by the module. */
} filter_t;

/**
 * Adds the filter instance to the managing algorithm.
 *
 * @param[in]  p_filter  Pointer to the filter instance
 *                       or NULL in case of memory allocation
 */
void fen_filter_start(filter_t * p_filter);

/**
 * Removes the filter instance from the managing algorithm.
 *
 * @param[in]  p_filter  Pointer to the filter instance.
 */
void fen_filter_stop(filter_t * p_filter);

/**
 * Applies set of active filters on received messages.
 *
 * @param[in]  type      Type of filters on which the packet is to be applied.
 * @param[in]  p_packet  Pointer to the received packet from the scanner rx queue.
 *
 * @return true if packet shall be filtered, false otherwise.
 */
bool fen_filters_apply(filter_type_t type, scanner_packet_t * p_packet);

/**
 * Reads out the accepted amount of packets.
 *
 * @param[in] type       Specifies type of filters which the accepted amount of packets should be returned for.
 *
 * @return the amount of packets that passed through all the filters.
 */
uint32_t fen_accepted_amount_get(filter_type_t type);

/** @} */

#endif /* FILTER_ENGINE_H__ */
