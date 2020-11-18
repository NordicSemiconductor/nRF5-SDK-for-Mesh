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

#include "mesh_provisionee.h"
#include "nrf_mesh_prov.h"
#include "device_state_manager.h"
#include "mesh_stack.h"
#include "mesh_app_utils.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "app_error.h"
#include "mesh_opt_core.h"

#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_events.h"

#if MESH_FEATURE_PB_GATT_ENABLED
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "mesh_adv.h"
#include "nrf_mesh_prov_bearer_gatt.h"
#include "nrf_mesh_gatt.h"
#include "proxy.h"
#include "mesh_opt_gatt.h"
#include "nrf_mesh_events.h"

#define MESH_PROVISIONEE_SDH_STATE_PRIORITY 1

static bool                        m_doing_gatt_reset;

static nrf_mesh_prov_bearer_gatt_t m_prov_bearer_gatt;
#endif  /* MESH_FEATURE_PB_GATT_ENABLED */

#if MESH_FEATURE_PB_ADV_ENABLED
static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;
#endif

#if !MESH_FEATURE_PB_ADV_ENABLED && !MESH_FEATURE_PB_GATT_ENABLED
#error "At least one provisioning bearer shall be enabled"
#endif

static mesh_provisionee_start_params_t m_params;
static nrf_mesh_prov_ctx_t             m_prov_ctx;
static uint8_t                         m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t                         m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];
static bool                            m_device_provisioned;
static bool                            m_device_identification_started;

static void power_down_evt_handle(const nrf_mesh_evt_t * p_evt);
static bool m_is_in_power_down;
static nrf_mesh_evt_handler_t m_power_down_evt_handler =
{
    .evt_cb = power_down_evt_handle,
};

static void power_down_evt_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_READY_TO_POWER_OFF)
    {
        m_is_in_power_down = true;
    }
}

#if MESH_FEATURE_PB_GATT_ENABLED

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt);
static nrf_mesh_evt_handler_t m_mesh_evt_handler = {
    .evt_cb = mesh_evt_handler,
};
static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_DISABLED && m_doing_gatt_reset)
    {
        APP_ERROR_CHECK(nrf_sdh_disable_request());
        nrf_mesh_evt_handler_remove(&m_mesh_evt_handler);
    }
}

static void sd_state_evt_handler(nrf_sdh_state_evt_t state, void * p_context)
{
    (void) p_context;

    if (!m_doing_gatt_reset)
    {
        return;
    }
    switch (state)
    {
        case NRF_SDH_EVT_STATE_ENABLED:
        {
            uint32_t ram_start = 0;
            /* Set the default configuration (as defined through sdk_config.h). */
            uint32_t err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
            APP_ERROR_CHECK(err_code);

            /* Update GAP device name length. */
            ble_cfg_t cfg;
            memset(&cfg, 0, sizeof(cfg));
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cfg.gap_cfg.device_name_cfg.write_perm);
            cfg.gap_cfg.device_name_cfg.vloc        = BLE_GATTS_VLOC_STACK;
            cfg.gap_cfg.device_name_cfg.p_value     = NULL;
            cfg.gap_cfg.device_name_cfg.current_len = 0;
            cfg.gap_cfg.device_name_cfg.max_len     = strlen(GAP_DEVICE_NAME);
            APP_ERROR_CHECK(sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &cfg, ram_start));

            err_code = nrf_sdh_ble_enable(&ram_start);
            APP_ERROR_CHECK(err_code);

            if (m_params.prov_sd_ble_opt_set_cb != NULL)
            {
                m_params.prov_sd_ble_opt_set_cb();
            }

#if MESH_FEATURE_GATT_PROXY_ENABLED
            mesh_key_index_t key_index;
            uint32_t count = 1;
            nrf_mesh_key_refresh_phase_t kr_phase;

            APP_ERROR_CHECK(dsm_subnet_get_all(&key_index, &count));
            APP_ERROR_CHECK(dsm_subnet_kr_phase_get(dsm_net_key_index_to_subnet_handle(key_index),
                                                    &kr_phase));

            proxy_init();

            /* NOTE: Even though the device supports the GATT proxy feature, enabling the proxy
             * state is _not_ required. The Node Identity will be advertised for 60s, if the proxy
             * is enabled, the device will start advertising the Network ID afterwards. The default
             * state for the proxy feature is set with `PROXY_ENABLED_DEFAULT`.
             */
            NRF_MESH_ERROR_CHECK(proxy_node_id_enable(NULL, kr_phase));
#endif  /* MESH_FEATURE_GATT_PROXY_ENABLED */

            /* We deliberately start the mesh _after_ the proxy to ensure that the
             * softdevice gets a timeslot ASAP to advertise. */
            err_code = nrf_mesh_enable();
            APP_ERROR_CHECK(err_code);

            m_doing_gatt_reset = false;

            if (m_params.prov_complete_cb != NULL)
            {
                m_params.prov_complete_cb();
            }
            break;
        }
        case NRF_SDH_EVT_STATE_DISABLED:
        {
            uint32_t err_code = nrf_sdh_enable_request();
            APP_ERROR_CHECK(err_code);
            break;
        }
        default:
            break;
    }
}

NRF_SDH_STATE_OBSERVER(m_sdh_req_obs, MESH_PROVISIONEE_SDH_STATE_PRIORITY) =
{
    .handler   = sd_state_evt_handler,
    .p_context = NULL
};

static void gatt_database_reset(void)
{
    m_doing_gatt_reset = true;
    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

    APP_ERROR_CHECK(nrf_mesh_disable());
    /* Wait for the mesh to be fully disabled before continuing. */
}
#endif /* MESH_FEATURE_PB_GATT_ENABLED */

static uint32_t provisionee_start(void)
{
    if (m_is_in_power_down)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t bearers = 0;

#if MESH_FEATURE_PB_ADV_ENABLED
    bearers = NRF_MESH_PROV_BEARER_ADV;
#endif

#if MESH_FEATURE_PB_GATT_ENABLED
    bearers |= NRF_MESH_PROV_BEARER_GATT;
#endif
    /* Re-generate the keys each round. */
    RETURN_ON_ERROR(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
    return nrf_mesh_prov_listen(&m_prov_ctx, m_params.p_device_uri, 0, bearers);
}

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_INVITE_RECEIVED:
            if (m_params.prov_device_identification_start_cb != NULL
                && p_evt->params.invite_received.attention_duration_s > 0)
            {
                m_device_identification_started = true;
                m_params.prov_device_identification_start_cb(p_evt->params.invite_received.attention_duration_s);
            }

            break;

        case NRF_MESH_PROV_EVT_START_RECEIVED:
            if (m_params.prov_device_identification_stop_cb != NULL
                && m_device_identification_started)
            {
                m_device_identification_started = false;
                m_params.prov_device_identification_stop_cb();
            }

            break;

        case NRF_MESH_PROV_EVT_LINK_CLOSED:
            if (!m_device_provisioned)
            {
                if (m_params.prov_abort_cb != NULL)
                {
                    m_device_identification_started = false;
                    m_params.prov_abort_cb();
                }

                (void) provisionee_start();
            }
            else
            {
#if MESH_FEATURE_PB_GATT_ENABLED
                /* it requires switching GATT service before provisioning complete */
                gatt_database_reset();
#else
                if (m_params.prov_complete_cb != NULL)
                {
                    m_params.prov_complete_cb();
                }
#endif  /* MESH_FEATURE_PB_GATT_ENABLED */
            }
            break;

        case NRF_MESH_PROV_EVT_STATIC_REQUEST:
            APP_ERROR_CHECK(nrf_mesh_prov_auth_data_provide(&m_prov_ctx,
                                                                 m_params.p_static_data,
                                                                 NRF_MESH_KEY_SIZE));
            break;
        case NRF_MESH_PROV_EVT_COMPLETE:
        {
            APP_ERROR_CHECK(mesh_stack_provisioning_data_store(
                                    p_evt->params.complete.p_prov_data,
                                    p_evt->params.complete.p_devkey));
            m_device_provisioned = true;
            break;
        }

        default:
            break;
    }
}

uint32_t mesh_provisionee_prov_start(const mesh_provisionee_start_params_t * p_start_params)
{
    nrf_mesh_prov_oob_caps_t prov_caps =
    {
        ACCESS_ELEMENT_COUNT,
        NRF_MESH_PROV_ALGORITHM_FIPS_P256EC,
        0,
        NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED,
        0,
        0,
        0,
        0
    };

    m_params = *p_start_params;
    if (m_params.p_static_data == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* Public/private keys are (re-)generated in provisionee_start(). */
    RETURN_ON_ERROR(nrf_mesh_prov_init(&m_prov_ctx,
                                       m_public_key,
                                       m_private_key,
                                       &prov_caps, prov_evt_handler));

#if MESH_FEATURE_PB_ADV_ENABLED
    RETURN_ON_ERROR(nrf_mesh_prov_bearer_add(
                        &m_prov_ctx,
                        nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
#endif

#if MESH_FEATURE_PB_GATT_ENABLED
    RETURN_ON_ERROR(nrf_mesh_prov_bearer_gatt_init(&m_prov_bearer_gatt));
    RETURN_ON_ERROR(nrf_mesh_prov_bearer_add(
                        &m_prov_ctx,
                        nrf_mesh_prov_bearer_gatt_interface_get(&m_prov_bearer_gatt)));
#endif

    nrf_mesh_evt_handler_add(&m_power_down_evt_handler);

    return provisionee_start();
}

uint32_t mesh_provisionee_prov_listen_stop(void)
{
    return nrf_mesh_prov_listen_stop(&m_prov_ctx);
}
