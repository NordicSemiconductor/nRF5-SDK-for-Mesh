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

#include <stdbool.h>
#include <string.h>

#include "nrf_mesh.h"
#include "nrf_mesh_prov.h"

#include "log.h"
#include "nrf_mesh_prov_bearer.h"
#include "prov_provisionee.h"
#include "prov_provisioner.h"
#include "prov_beacon.h"
#include "prov_utils.h"
#include "provisioning.h"
#include "utils.h"
#include "list.h"
#include "bitfield.h"

static bool is_numeric_oob_action(const nrf_mesh_prov_ctx_t * p_ctx)
{
    /* The provisioner's OOB is the reversal of the provisionee's. */
    return ((p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER &&
             (p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_OUTPUT &&
              (p_ctx->oob_action == NRF_MESH_PROV_OUTPUT_ACTION_BLINK   ||
               p_ctx->oob_action == NRF_MESH_PROV_OUTPUT_ACTION_BEEP    ||
               p_ctx->oob_action == NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE ||
               p_ctx->oob_action == NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC))) ||
            (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONEE &&
             (p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_INPUT &&
              (p_ctx->oob_action == NRF_MESH_PROV_INPUT_ACTION_PUSH   ||
               p_ctx->oob_action == NRF_MESH_PROV_INPUT_ACTION_TWIST    ||
               p_ctx->oob_action == NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER))));
}

static bool is_alphanumeric_oob_action(const nrf_mesh_prov_ctx_t * p_ctx)
{
    /* The provisioner's OOB is the reversal of the provisionee's. */
    return ((p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER &&
             (p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_OUTPUT &&
              p_ctx->oob_action == NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC)) ||
            (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONEE &&
             (p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_INPUT &&
              p_ctx->oob_action == NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING)));
}

uint32_t nrf_mesh_prov_init(nrf_mesh_prov_ctx_t *            p_ctx,
                            const uint8_t *                  p_public_key,
                            const uint8_t *                  p_private_key,
                            const nrf_mesh_prov_oob_caps_t * p_caps,
                            nrf_mesh_prov_evt_handler_cb_t   event_handler)
{
    if (p_ctx         == NULL ||
        p_public_key  == NULL ||
        p_private_key == NULL ||
        p_caps        == NULL ||
        event_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_ctx->state != NRF_MESH_PROV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        p_ctx->event_handler = event_handler;
        p_ctx->p_public_key  = p_public_key;
        p_ctx->p_private_key = p_private_key;
        memcpy(&p_ctx->capabilities, p_caps, sizeof(nrf_mesh_prov_oob_caps_t));
        return NRF_SUCCESS;
    }
}

static bool bearer_compare_callback(const list_node_t * p_elem1, const list_node_t * p_elem2)
{
    return (PARENT_BY_FIELD_GET(prov_bearer_t, node, p_elem1)->bearer_type ==
            PARENT_BY_FIELD_GET(prov_bearer_t, node, p_elem2)->bearer_type);
}

static prov_bearer_t * prov_bearer_find(const list_node_t * p_bearers,
                                        nrf_mesh_prov_bearer_type_t bearer_type)
{
    prov_bearer_t * p_bearer;
    LIST_FOREACH(p_item, p_bearers)
    {
        p_bearer = PARENT_BY_FIELD_GET(prov_bearer_t, node, p_item);
        if (p_bearer->bearer_type == bearer_type)
        {
            return p_bearer;
        }
    }
    return NULL;
}

uint32_t nrf_mesh_prov_bearer_add(nrf_mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_prov_bearer)
{
    if (p_ctx == NULL || p_prov_bearer == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t status = list_compare_add(&p_ctx->p_bearers, &p_prov_bearer->node, bearer_compare_callback);
    if (status == NRF_SUCCESS)
    {
        p_prov_bearer->p_parent = p_ctx;
        p_ctx->supported_bearers |= p_prov_bearer->bearer_type;
    }
    return status;
}

uint32_t nrf_mesh_prov_generate_keys(uint8_t * p_public, uint8_t * p_private)
{
    return prov_utils_keys_generate(p_public, p_private);
}

uint32_t nrf_mesh_prov_listen(nrf_mesh_prov_ctx_t *       p_ctx,
                              const char *                URI,
                              uint16_t                    oob_info_sources,
                              uint32_t                    bearer_types)
{
    if (p_ctx == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if(p_ctx->state != NRF_MESH_PROV_STATE_IDLE &&
            p_ctx->state != NRF_MESH_PROV_STATE_WAIT_LINK)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else if (bearer_types == 0 ||
             !bitfield_is_subset_of(&p_ctx->supported_bearers,
                                    &bearer_types,
                                    NRF_MESH_PROV_BEARER_COUNT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    p_ctx->state = NRF_MESH_PROV_STATE_WAIT_LINK;
    p_ctx->role = NRF_MESH_PROV_ROLE_PROVISIONEE;

    for (uint32_t i = bitfield_next_get(&bearer_types, NRF_MESH_PROV_BEARER_COUNT, 0);
         i != NRF_MESH_PROV_BEARER_COUNT;
         i = bitfield_next_get(&bearer_types, NRF_MESH_PROV_BEARER_COUNT, i + 1))
    {
        prov_bearer_t * p_bearer = prov_bearer_find(p_ctx->p_bearers, (nrf_mesh_prov_bearer_type_t) (1 << i));

        /* We've already tested that the bearer should be supported above. */
        NRF_MESH_ASSERT(p_bearer != NULL);
        uint32_t err_code = prov_provisionee_listen(p_ctx, p_bearer, URI, oob_info_sources);
        if (err_code != NRF_SUCCESS)
        {
            (void) nrf_mesh_prov_listen_stop(p_ctx);
            return err_code;
        }
    }
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_prov_listen_stop(nrf_mesh_prov_ctx_t * p_ctx)
{
    if (p_ctx == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_ctx->state != NRF_MESH_PROV_STATE_WAIT_LINK)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    prov_bearer_t * p_bearer;
    LIST_FOREACH(p_item, p_ctx->p_bearers)
    {
        p_bearer = PARENT_BY_FIELD_GET(prov_bearer_t, node, p_item);
        /* Try stopping listening on all available bearers. */
        (void) p_bearer->p_interface->listen_stop(p_bearer);
    }
    p_ctx->state = NRF_MESH_PROV_STATE_IDLE;

    return NRF_SUCCESS;
}

uint32_t nrf_mesh_prov_provision(nrf_mesh_prov_ctx_t *                     p_ctx,
                                 const uint8_t *                           p_target_uuid,
                                 uint8_t                                   attention_duration_s,
                                 const nrf_mesh_prov_provisioning_data_t * p_data,
                                 nrf_mesh_prov_bearer_type_t               bearer_type)
{
    if (p_ctx == NULL || p_target_uuid == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (!prov_data_is_valid(p_data))
    {
        return NRF_ERROR_INVALID_DATA;
    }

    p_ctx->p_active_bearer = prov_bearer_find(p_ctx->p_bearers, bearer_type);

    if (p_ctx->p_active_bearer == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else
    {
        return prov_provisioner_provision(p_ctx, p_target_uuid, attention_duration_s, p_data);
    }
}

uint32_t nrf_mesh_prov_oob_use(nrf_mesh_prov_ctx_t *      p_ctx,
                               nrf_mesh_prov_oob_method_t method,
                               uint8_t                    action,
                               uint8_t                    size)
{
    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_oob_use(p_ctx, method, action, size);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}

uint32_t nrf_mesh_prov_auth_data_provide(nrf_mesh_prov_ctx_t * p_ctx,
                                         const uint8_t *       p_data,
                                         uint8_t               size)
{
    if (p_ctx == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (size > PROV_AUTH_LEN ||
             size != p_ctx->oob_size)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (p_ctx->oob_method == NRF_MESH_PROV_OOB_METHOD_STATIC)
    {
        memcpy(p_ctx->auth_value, p_data, size);
    }
    else if (is_numeric_oob_action(p_ctx) &&
             prov_utils_auth_data_is_valid_number(p_data, size))
    {
        uint8_t oob_length = sizeof(uint32_t);
        memcpy(&p_ctx->auth_value[0], p_data, oob_length);
        memset(&p_ctx->auth_value[oob_length], 0, sizeof(p_ctx->auth_value) - oob_length);

        /* Reverse the endianness according to the spec. */
        utils_reverse_array(p_ctx->auth_value, sizeof(p_ctx->auth_value));
    }
    else if (is_alphanumeric_oob_action(p_ctx) &&
             prov_utils_auth_data_is_alphanumeric(p_data, size))
    {
        uint8_t oob_length = size;
        memcpy(&p_ctx->auth_value[0], p_data, oob_length);
        memset(&p_ctx->auth_value[oob_length], 0, sizeof(p_ctx->auth_value) - oob_length);

        /* Reverse the endianness according to the spec. */
        utils_reverse_array(p_ctx->auth_value, sizeof(p_ctx->auth_value));
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }


    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_auth_data(p_ctx, p_data, size);
    }
    else
    {
        return prov_provisionee_auth_data(p_ctx, p_data, size);
    }
}

uint32_t nrf_mesh_prov_shared_secret_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared)
{
    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return prov_provisioner_shared_secret(p_ctx, p_shared);
    }
    else
    {
        return prov_provisionee_shared_secret(p_ctx, p_shared);
    }
}

uint32_t nrf_mesh_prov_pubkey_provide(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key)
{
    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONEE)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        return prov_provisioner_oob_pubkey(p_ctx, p_key);
    }
}

uint32_t nrf_mesh_prov_scan_start(nrf_mesh_prov_evt_handler_cb_t event_handler)
{
    if (event_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else
    {
        prov_beacon_scan_start(event_handler);
        return NRF_SUCCESS;
    }
}

void nrf_mesh_prov_scan_stop(void)
{
    prov_beacon_scan_stop();
}
