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

#include "nrf_mesh_node_config.h"

#include <stddef.h>
#include <stdbool.h>

#if defined(S130) || defined(S132) || defined(S140)
#   include "nrf_nvic.h"
#else
#   include "nrf_soc.h"
#endif

#include "nrf_mesh_assert.h"
#include "nrf_mesh_events.h"

#include "access.h"
#include "access_config.h"
#include "config_server.h"
#include "device_state_manager.h"
#include "flash_manager.h"
#include "health_server.h"
#include "log.h"
#include "net_state.h"
#include "toolchain.h"

#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_bearer_adv.h"

#define RETURN_ON_ERROR(status) \
    do { \
        uint32_t result = status; \
        if (result != NRF_SUCCESS) \
        { \
            return result; \
        } \
    } while (0);

static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;
static nrf_mesh_prov_ctx_t m_prov_ctx;
static bool m_node_provisioned;
static health_server_t m_health_server;
static const nrf_mesh_node_config_params_t * mp_init_params;
static uint8_t m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];

#if !defined(HOST)

static void sleep_forever(uint32_t pc)
{
    /*lint -save -e438 -e550 Unsused variable irqs_masked */
    uint32_t irqs_masked __attribute__((unused));
    _DISABLE_IRQS(irqs_masked);
    while (pc)
    {
        __WFI();
    }
    /*lint -restore */
}

#if defined(S110)

static void default_softdevice_assertion_handler(uint32_t pc, uint16_t line_number, const uint8_t * p_filename)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Softdevice assert: %s:%hu at 0x%.08lx\n", p_filename, line_number, pc);
    sleep_forever(pc);
}

#else /* defined(S110) */

static void default_softdevice_assertion_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Softdevice assert: %u:%u:%u\n", id, pc, info);
    sleep_forever(pc);
}

#endif

#endif /* !defined(HOST) */

void default_mesh_assertion_handler(uint32_t pc)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "MESH ASSERT at 0x%.08lx\n", pc);
#if !defined(HOST)
    sleep_forever(pc);
#endif
}

static uint32_t start_provisionee(void)
{
    return nrf_mesh_prov_listen(
        &m_prov_ctx, mp_init_params->p_device_uri, mp_init_params->oob_info_sources, NRF_MESH_PROV_BEARER_ADV);
}

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_LINK_ESTABLISHED:
            break;
        case NRF_MESH_PROV_EVT_LINK_CLOSED:
            if (!m_node_provisioned)
            {
                (void) start_provisionee();
            }
            else
            {
                if (mp_init_params->complete_callback != NULL)
                {
                    mp_init_params->complete_callback(mp_init_params->p_data);
                }
            }
            break;
        case NRF_MESH_PROV_EVT_STATIC_REQUEST:
            NRF_MESH_ASSERT(nrf_mesh_prov_auth_data_provide(&m_prov_ctx, mp_init_params->p_static_data, NRF_MESH_KEY_SIZE) == NRF_SUCCESS);
            break;
        case NRF_MESH_PROV_EVT_COMPLETE:
        {
            dsm_handle_t netkey_handle, devkey_handle;
            dsm_local_unicast_address_t local_address;
            local_address.address_start = p_evt->params.complete.address;
            local_address.count = ACCESS_ELEMENT_COUNT;

            /* Store received provisioning data in the DSM: */
            NRF_MESH_ASSERT(dsm_local_unicast_addresses_set(&local_address) == NRF_SUCCESS);
            NRF_MESH_ASSERT(dsm_subnet_add(p_evt->params.complete.netkey_index,
                                       p_evt->params.complete.p_netkey,
                                       &netkey_handle) == NRF_SUCCESS);
            NRF_MESH_ASSERT(dsm_devkey_add(p_evt->params.complete.address,
                                       netkey_handle,
                                       p_evt->params.complete.p_devkey,
                                       &devkey_handle) == NRF_SUCCESS);

            NRF_MESH_ERROR_CHECK(net_state_iv_index_set(p_evt->params.complete.iv_index,
                                                        p_evt->params.complete.flags.iv_update));

            /* If there is an ongoing key refresh we have to "update" the key to the correct state. */
            if (p_evt->params.complete.flags.key_refresh)
            {
                NRF_MESH_ERROR_CHECK(dsm_subnet_update(netkey_handle, p_evt->params.complete.p_netkey));
                NRF_MESH_ERROR_CHECK(dsm_subnet_update_swap_keys(netkey_handle));
            }

            /* The config server should be bound to the device key: */
            NRF_MESH_ERROR_CHECK(config_server_bind(devkey_handle));
            m_node_provisioned = true;
            break;
        }
        default:
            break;
    }
}

static void flash_stable_cb(void)
{
    /* This should never return */
    hal_device_reset(0);
    NRF_MESH_ASSERT(false);
}

static void config_server_cb(const config_server_evt_t * p_evt)
{
    /* Forward the event to the user application. */
    if (mp_init_params->config_server_callback)
    {
        mp_init_params->config_server_callback(p_evt);
    }

    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
#if PERSISTENT_STORAGE
        flash_manager_action_queue_empty_cb_set(flash_stable_cb);
#else
        flash_stable_cb();
#endif
    }
}

uint32_t softdevice_setup(nrf_clock_lf_cfg_t lfc_cfg, nrf_fault_handler_t assertion_handler)
{
#if !defined(HOST)
    if (assertion_handler == NULL)
    {
        assertion_handler = default_softdevice_assertion_handler;
    }

#if defined(S110)
    RETURN_ON_ERROR(sd_softdevice_enable(lfc_cfg, assertion_handler));
#else
    RETURN_ON_ERROR(sd_softdevice_enable(&lfc_cfg, assertion_handler));
#endif
    RETURN_ON_ERROR(sd_nvic_EnableIRQ(SD_EVT_IRQn));
#endif

    return NRF_SUCCESS;
}

static uint32_t setup_provisionee(const nrf_mesh_node_config_params_t * p_params)
{
    RETURN_ON_ERROR(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
    RETURN_ON_ERROR(nrf_mesh_prov_init(&m_prov_ctx, m_public_key, m_private_key, &p_params->prov_caps, prov_evt_handler));
    RETURN_ON_ERROR(nrf_mesh_prov_bearer_add(&m_prov_ctx, nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_node_config(const nrf_mesh_node_config_params_t * p_params)
{
    if (p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    mp_init_params = p_params;

    uint8_t softdevice_is_enabled = 0;
    RETURN_ON_ERROR(sd_softdevice_is_enabled(&softdevice_is_enabled));
    if (softdevice_is_enabled == 0)
    {
        RETURN_ON_ERROR(softdevice_setup(p_params->lf_clk_cfg, p_params->sd_assertion_handler));
    }

    /* Initialize and enable the mesh stack: */
    nrf_mesh_init_params_t mesh_init_params;
    memset(&mesh_init_params, 0, sizeof(mesh_init_params));
    mesh_init_params.lfclksrc = p_params->lf_clk_cfg;
    mesh_init_params.irq_priority = p_params->irq_priority;
    if (p_params->mesh_assertion_handler != NULL)
    {
        mesh_init_params.assertion_handler = p_params->mesh_assertion_handler;
    }
    else
    {
        mesh_init_params.assertion_handler = default_mesh_assertion_handler;
    }

    RETURN_ON_ERROR(nrf_mesh_init(&mesh_init_params));
    RETURN_ON_ERROR(nrf_mesh_enable());

    /* Initialize the access layer: */
    dsm_init();
    access_init();

    /* Initialize the configuration server: */
    RETURN_ON_ERROR(config_server_init(config_server_cb));

    /* Initialize the health server for the primary element: */
    RETURN_ON_ERROR(health_server_init(&m_health_server, 0, DEVICE_COMPANY_ID,
                mp_init_params->attention_cb, mp_init_params->p_selftest_array, mp_init_params->num_selftests));

    if (mp_init_params->setup_callback != NULL)
    {
        mp_init_params->setup_callback(mp_init_params->p_data);
    }

    /* Check if the node has already been provisioned */
    m_node_provisioned = dsm_flash_config_load() && access_flash_config_load();

    if (m_node_provisioned &&
        mp_init_params->complete_callback != NULL)
    {
        mp_init_params->complete_callback(mp_init_params->p_data);
    }
    else
    {
        RETURN_ON_ERROR(setup_provisionee(p_params));
        RETURN_ON_ERROR(start_provisionee());
    }

    return NRF_SUCCESS;
}

void nrf_mesh_node_config_clear(void)
{
    access_clear();
    dsm_clear();
    net_state_reset();
#if PERSISTENT_STORAGE
    flash_manager_wait();
#endif
}
