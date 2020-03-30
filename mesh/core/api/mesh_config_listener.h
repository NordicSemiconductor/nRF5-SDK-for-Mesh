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
#ifndef MESH_CONFIG_LISTENER_H__
#define MESH_CONFIG_LISTENER_H__

#include "nrf_mesh_section.h"
#include "mesh_config_entry.h"

/**
 * @defgroup MESH_CONFIG_LISTENER_MODULE Mesh config listener interface
 * @ingroup MESH_CONFIG
 * Change-listener interface for mesh config.
 *
 * Lets any module passively observe new values for a given config entry. Called after the state owner has accepted the value.
 *
 * @note: The listener will not be triggered for the inital recovery from flash.
 *
 * Usage: Define a function of the @ref mesh_config_listener_on_change_t type, and register it with the
 * @ref MESH_CONFIG_LISTENER macro. The listener is registered at link-time, and cannot be added or removed at runtime.
 * @{
 */

/**
 * Define an entry listener.
 *
 * @param[in] NAME     Name of listener variable.
 * @param[in] ID       Unique mesh config id to listen for.
 * @param[in] CALLBACK Callback to call when the entry changes.
 */
#define MESH_CONFIG_LISTENER(NAME, ID, CALLBACK)                                                   \
    NRF_MESH_SECTION_ITEM_REGISTER_FLASH(mesh_config_entry_listeners,                              \
                                         const mesh_config_listener_t NAME) = {&(ID), CALLBACK}

/** Reason for the listener invocation */
typedef enum
{
    MESH_CONFIG_CHANGE_REASON_SET, /**< The entry was changed by a user. */
    MESH_CONFIG_CHANGE_REASON_DELETE, /**< The entry was deleted. */
} mesh_config_change_reason_t;

/**
 * Mesh config change-listener function type.
 *
 * @param[in] reason  Reason for the change.
 * @param[in] id      ID of the changed entry.
 * @param[in] p_entry Data for the changed entry, or NULL if the @p reason is @ref MESH_CONFIG_CHANGE_REASON_DELETE.
 */
typedef void (*mesh_config_listener_on_change_t)(mesh_config_change_reason_t reason, mesh_config_entry_id_t id, const void * p_entry);

/**
 * @internal
 * Entry listener structure.
 *
 * Should only be instantiated through the @ref MESH_CONFIG_LISTENER macro.
 */
typedef struct
{
    const mesh_config_entry_id_t * p_id; /**< ID of the entry to listen for. */
    mesh_config_listener_on_change_t callback; /**< Callback to call when the entry changes */
} mesh_config_listener_t;

/** @} */

#endif /* MESH_CONFIG_LISTENER_H__ */
