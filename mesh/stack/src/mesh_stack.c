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

#include "mesh_stack.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_externs.h"
#include "access_config.h"
#include "flash_manager.h"
#include "device_state_manager.h"
#include "config_server.h"
#include "health_server.h"
#include "net_state.h"
#include "replay_cache.h"
#include "hal.h"
#include "scanner.h"
#include "bearer_handler.h"
#include "event.h"
#include "timer_scheduler.h"
#include "mesh_config.h"
#include "mesh_config_backend_glue.h"
#include "mesh_opt.h"

#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "proxy.h"
#endif

#if MESH_FEATURE_FRIEND_ENABLED
#include "mesh_friend.h"
#endif

static health_server_t m_health_server;
static nrf_mesh_evt_handler_t m_power_down_evt;
static bool m_load_failed;

static void device_power_down(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_DISABLED)
    {
        nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_READY_TO_POWER_OFF};
        event_handle(&evt);
    }

    if (p_evt->type == NRF_MESH_EVT_CONFIG_STABLE)
    {
        bearer_handler_force_mode_disable();
        NRF_MESH_ERROR_CHECK(nrf_mesh_disable());
    }
}

static void load_monitor(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            if (p_evt->params.config_load_failure.id.file < MESH_OPT_FIRST_FREE_ID)
            {
                m_load_failed = true;
            }
            break;

        default:
            break;
    }
}

static void device_reset(void)
{
    hal_device_reset(0);
}

uint32_t mesh_stack_init(const mesh_stack_init_params_t * p_init_params,
                         bool * p_device_provisioned)
{
    uint32_t status;

    if (p_init_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize the mesh stack */
    status = nrf_mesh_init(&p_init_params->core);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize the access layer */
    dsm_init();
    access_init();

    /* Initialize the configuration server */
    status = config_server_init(p_init_params->models.config_server_cb);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Initialize the health server for the primary element */
    status = health_server_init(&m_health_server, 0, DEVICE_COMPANY_ID,
                                p_init_params->models.health_server_attention_cb,
                                p_init_params->models.p_health_server_selftest_array,
                                p_init_params->models.health_server_num_selftests);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    /* Give application opportunity to initialize application specific models */
    if (p_init_params->models.models_init_cb != NULL)
    {
        p_init_params->models.models_init_cb();
    }

    /* Load configuration, and check if the device has already been provisioned */
    static nrf_mesh_evt_handler_t s_evt_handler = {
        .evt_cb = load_monitor
    };
    nrf_mesh_evt_handler_add(&s_evt_handler);

    mesh_config_load();

    nrf_mesh_evt_handler_remove(&s_evt_handler);

    uint32_t dsm_result = dsm_load_config_apply();
    uint32_t access_result = access_load_config_apply();

    if (dsm_result == NRF_ERROR_INVALID_DATA || access_result == NRF_ERROR_INVALID_DATA || m_load_failed)
    {
        mesh_stack_config_clear();
        status = NRF_ERROR_INVALID_DATA;
    }

    bool is_provisioned = mesh_stack_is_device_provisioned();

#if MESH_FEATURE_GATT_PROXY_ENABLED
    if (is_provisioned)
    {
        proxy_init();
    }
#endif

#if MESH_FEATURE_FRIEND_ENABLED
    NRF_MESH_ERROR_CHECK(mesh_friend_init());
#endif

    if (p_device_provisioned != NULL)
    {
        *p_device_provisioned = is_provisioned;
    }

    return status;
}

uint32_t mesh_stack_start(void)
{
    /* If the device is provisioned, the proxy state is automatically recovered from flash when
     * mesh_config loads the configuration. */
#if MESH_FEATURE_GATT_PROXY_ENABLED
    if (mesh_stack_is_device_provisioned() &&
        proxy_is_enabled())
    {
        (void) proxy_start();
    }
#endif

#if MESH_FEATURE_FRIEND_ENABLED
    mesh_friend_enable();
#endif

    return nrf_mesh_enable();
}

uint32_t mesh_stack_provisioning_data_store(const nrf_mesh_prov_provisioning_data_t * p_prov_data,
                                            const uint8_t * p_devkey)
{
    dsm_handle_t netkey_handle, devkey_handle;
    dsm_local_unicast_address_t local_address;
    local_address.address_start = p_prov_data->address;
    local_address.count = ACCESS_ELEMENT_COUNT;
    uint32_t status;

    /* Store received provisioning data in the DSM */
    status = dsm_local_unicast_addresses_set(&local_address);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    status = dsm_subnet_add(p_prov_data->netkey_index, p_prov_data->netkey, &netkey_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }
    status = dsm_devkey_add(p_prov_data->address, netkey_handle, p_devkey, &devkey_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = net_state_iv_index_set(p_prov_data->iv_index, p_prov_data->flags.iv_update);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (p_prov_data->flags.key_refresh == 1)
    {
        status = dsm_subnet_update(netkey_handle, p_prov_data->netkey);
        if (status != NRF_SUCCESS)
        {
            return status;
        }

        status = dsm_subnet_update_swap_keys(netkey_handle);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
    }

    /* Bind config server to the device key */
    status = config_server_bind(devkey_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    __LOG(LOG_SRC_CORE, LOG_LEVEL_DBG3, "iv_index: 0x%08x  flag:ivu: %d\n", p_prov_data->iv_index, p_prov_data->flags.iv_update);

    return status;
}

void mesh_stack_config_clear(void)
{
    mesh_opt_clear();
    access_clear();
    dsm_clear();
    net_state_reset();
    replay_cache_clear();
}

bool mesh_stack_is_device_provisioned(void)
{
    return nrf_mesh_is_device_provisioned();
}

#if PERSISTENT_STORAGE
static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    /* It may be possible that the `mesh_stack_device_reset()` is triggered from inside of the
     * mesh event handler callback which is receiving NRF_MESH_EVT_FLASH_STABLE. That will
     * result in this event handler getting added at the end of the event handler list and getting
     * called immediately. To prevent incorrect conclusion, this handler must recheck if the
     * flash is really stable before processing NRF_MESH_EVT_FLASH_STABLE. If the flash is not
     * stable, this handler will receive another event anyway.
     */
    if (flash_manager_is_stable() && p_evt->type == NRF_MESH_EVT_FLASH_STABLE)
    {
        device_reset();
    }
}
void mesh_stack_device_reset(void)
{
    /* We have to do this elaborate dance to avoid nested
     * calls to bearer event. It should be possible to call this
     * function from within a bearer_event_process context.
     */
    if (flash_manager_is_stable())
    {
        device_reset();
    }
    else
    {
        static nrf_mesh_evt_handler_t s_evt_handler = {
            .evt_cb = mesh_evt_handler
        };

        nrf_mesh_evt_handler_add(&s_evt_handler);
    }
}

#else

void mesh_stack_device_reset(void)
{
    device_reset();
}
#endif  /* PERSISTENT_STORAGE */

uint32_t mesh_stack_persistence_flash_usage(const uint32_t ** pp_start, uint32_t * p_length)
{
    if (pp_start == NULL || p_length == NULL)
    {
        return NRF_ERROR_NULL;
    }

#if PERSISTENT_STORAGE
    mesh_config_backend_flash_usage_t mesh_config_usage;
    mesh_config_backend_flash_usage_get(&mesh_config_usage);
    if (mesh_config_usage.p_start != NULL)
    {
        *pp_start = mesh_config_usage.p_start;
        *p_length = (((const uint8_t *) flash_manager_recovery_page_get()) - ((const uint8_t *) *pp_start) + PAGE_SIZE);
    }
    else
    {
        /* If there's no mesh config present in flash, stack does not use flash. */
        *pp_start = NULL;
        *p_length = 0;
    }
#else
    *pp_start = NULL;
    *p_length = 0;
#endif
    return NRF_SUCCESS;
}

health_server_t * mesh_stack_health_server_get(void)
{
    return &m_health_server;
}

void mesh_stack_power_down(void)
{
    /* turn off scanner */
    scanner_disable();

#if MESH_FEATURE_GATT_PROXY_ENABLED
    /* disables the node ID beacons if there are any */
    (void)proxy_node_id_disable();
    /* disconnect active connections if there are any */
    proxy_disconnect();
    (void)proxy_stop();
    /* disable proxy to prevent restart of beaconing if it is enabled by default */
    proxy_disable();
#endif

    /* clean timers queue and turn off hw timer */
    timer_sch_stop();

    /* turn off advertiser
     * Advertisers shouldn't be turned off separately since they send data when timers fire.
     * Timers have been stopped at this moment. */

    /* register event to catch flash operation finishing */
    m_power_down_evt.evt_cb = device_power_down;
    event_handler_add(&m_power_down_evt);

    /* Enforce the bearer handler to speed up data storing */
    bearer_handler_force_mode_enable();
    /* start power down storage */
    mesh_config_power_down();
}
