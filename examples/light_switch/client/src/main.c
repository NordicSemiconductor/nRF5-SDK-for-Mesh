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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "nrf.h"
#include "nrf_sdm.h"
#include "boards.h"
#include "nrf_mesh_sdk.h"
#include "nrf_delay.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_assert.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"

#include "config_client.h"
#include "health_client.h"
#include "simple_on_off_client.h"

#include "simple_hal.h"
#include "provisioner.h"

#include "light_switch_example_common.h"
#include "rtt_input.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define CLIENT_COUNT             (SERVER_COUNT + 1)
#define GROUP_CLIENT_INDEX       (SERVER_COUNT)
#define BUTTON_NUMBER_GROUP      (3)
#define RTT_INPUT_POLL_PERIOD_MS (100)

/*****************************************************************************
 * Static data
 *****************************************************************************/

static const uint8_t m_netkey[NRF_MESH_KEY_SIZE] = NETKEY;
static const uint8_t m_appkey[NRF_MESH_KEY_SIZE] = APPKEY;

static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_devkey_handles[SERVER_COUNT];
static dsm_handle_t m_server_handles[SERVER_COUNT];
static dsm_handle_t m_group_handle;

static simple_on_off_client_t m_clients[CLIENT_COUNT];
static health_client_t m_health_client;

static uint16_t m_provisioned_devices;
static uint16_t m_configured_devices;

/* Forward declarations */
static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src);
static void health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event);

/*****************************************************************************
 * Static functions
 *****************************************************************************/

/**
 * Retrieves stored device state manager configuration.
 * The number of provisioned devices is calculated from the number of device keys stored. The device
 * key for each server is stored on provisioning complete in the `provisioner_prov_complete_cb()`.
 *
 * @returns Number of provisioned devices.
 */
static uint16_t provisioned_device_handles_load(void)
{
    uint16_t provisioned_devices = 0;

    /* Load the key handles. */
    uint32_t count = 1;
    ERROR_CHECK(dsm_subnet_get_all(&m_netkey_handle, &count));
    count = 1;
    ERROR_CHECK(dsm_appkey_get_all(m_netkey_handle, &m_appkey_handle, &count));

    /* Load all the address handles. */
    dsm_handle_t address_handles[DSM_ADDR_MAX];
    count = DSM_NONVIRTUAL_ADDR_MAX;
    ERROR_CHECK(dsm_address_get_all(&address_handles[0], &count));

    for (uint32_t i = 0; i < count; ++i)
    {
        nrf_mesh_address_t address;
        ERROR_CHECK(dsm_address_get(address_handles[i], &address));

        /* If the address is a unicast address, it is one of the server's root element address and
         * we have should have a device key stored for it. If not, it is our GROUP_ADDRESS and we
         * load the handle for that.
         */
        if ((address.type == NRF_MESH_ADDRESS_TYPE_UNICAST) &&
            (dsm_devkey_handle_get(address.value, &m_devkey_handles[provisioned_devices]) == NRF_SUCCESS)
            && m_devkey_handles[provisioned_devices] != DSM_HANDLE_INVALID)
        {
            ERROR_CHECK(dsm_address_handle_get(&address, &m_server_handles[provisioned_devices]));
            provisioned_devices++;
        }
        else if (address.type == NRF_MESH_ADDRESS_TYPE_GROUP)
        {
            ERROR_CHECK(dsm_address_handle_get(&address, &m_group_handle));
        }
    }

    return provisioned_devices;
}

/**
 * Gets the number of configured devices.
 *
 * We exploit the fact that the publish address of the Simple OnOff clients is set at configuration
 * complete, i.e., in the `provisioner_config_successful_cb()`, and simply count the number of
 * clients with their publish address' set.
 */
static uint16_t configured_devices_count_get(void)
{
    uint16_t configured_devices = 0;
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        dsm_handle_t address_handle = DSM_HANDLE_INVALID;
        if ((access_model_publish_address_get(m_clients[i].model_handle,
                                              &address_handle) == NRF_SUCCESS)
            && (DSM_HANDLE_INVALID != address_handle))
        {
            configured_devices++;
        }
        else
        {
            /* Clients are configured sequentially. */
            break;
        }
    }

    return configured_devices;
}

static void access_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting up access layer and models\n");

    dsm_init();
    access_init();

    m_netkey_handle = DSM_HANDLE_INVALID;
    m_appkey_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        m_devkey_handles[i] = DSM_HANDLE_INVALID;
        m_server_handles[i] = DSM_HANDLE_INVALID;
    }
    m_group_handle = DSM_HANDLE_INVALID;

    /* Initialize and enable all the models before calling ***_flash_config_load. */
    ERROR_CHECK(config_client_init(config_client_event_cb));
    ERROR_CHECK(health_client_init(&m_health_client, 0, health_event_cb));

    for (uint32_t i = 0; i < CLIENT_COUNT; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i));
    }

    if (dsm_flash_config_load())
    {
        m_provisioned_devices = provisioned_device_handles_load();
    }
    else
    {
        /* Set and add local addresses and keys, if flash recovery fails. */
        dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, ACCESS_ELEMENT_COUNT};
        ERROR_CHECK(dsm_local_unicast_addresses_set(&local_address));
        ERROR_CHECK(dsm_address_publish_add(GROUP_ADDRESS, &m_group_handle));
        ERROR_CHECK(dsm_subnet_add(0, m_netkey, &m_netkey_handle));
        ERROR_CHECK(dsm_appkey_add(0, m_netkey_handle, m_appkey, &m_appkey_handle));
    }

    if (access_flash_config_load())
    {
        m_configured_devices = configured_devices_count_get();
    }
    else
    {
        /* Bind the keys to the health client. */
        ERROR_CHECK(access_model_application_bind(m_health_client.model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_health_client.model_handle, m_appkey_handle));

        /* Bind the keys to the Simple OnOff clients. */
        for (uint32_t i = 0; i < SERVER_COUNT; ++i)
        {
            ERROR_CHECK(access_model_application_bind(m_clients[i].model_handle, m_appkey_handle));
            ERROR_CHECK(access_model_publish_application_set(m_clients[i].model_handle, m_appkey_handle));
        }

        ERROR_CHECK(access_model_application_bind(m_clients[GROUP_CLIENT_INDEX].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_clients[GROUP_CLIENT_INDEX].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_address_set(m_clients[GROUP_CLIENT_INDEX].model_handle, m_group_handle));
        access_flash_config_store();
    }

    provisioner_init();
    if (m_configured_devices < m_provisioned_devices)
    {
        provisioner_configure(UNPROV_START_ADDRESS + m_configured_devices);
    }
    else if (m_provisioned_devices < SERVER_COUNT)
    {
        provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
    }
}

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = (((uint32_t) p_client - ((uint32_t) &m_clients[0]))) / sizeof(m_clients[0]);
    NRF_MESH_ASSERT(index < SERVER_COUNT);
    return index;
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);
    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            hal_led_pin_set(BSP_LED_0 + server_index, true);
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            hal_led_pin_set(BSP_LED_0 + server_index, false);
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            hal_led_blink_ms(LEDS_MASK, 100, 6);
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }

    /* Set 4th LED on when all servers are on. */
    bool all_servers_on = true;
    for (uint32_t i = BSP_LED_0; i < BSP_LED_0 + m_configured_devices; ++i)
    {
        if (!hal_led_pin_get(i))
        {
            all_servers_on = false;
            break;
        }
    }

    hal_led_pin_set(BSP_LED_3, all_servers_on);
}

static void health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    switch (p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node 0x%04x alive with %u active fault(s), RSSI: %d\n",
                  p_event->p_meta_data->src.value, p_event->data.fault_status.fault_array_length,
                  p_event->p_meta_data->rssi);
            break;
        default:
            break;
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    if (m_configured_devices == 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "No devices provisioned\n");
        return;
    }
    else if (m_configured_devices <= button_number && button_number != BUTTON_NUMBER_GROUP)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Device %u not provisioned yet.\n", button_number);
        return;
    }

    uint32_t status = NRF_SUCCESS;
    switch (button_number)
    {
        case 0:
        case 1:
        case 2:
            /* Invert LED. */
            status = simple_on_off_client_set(&m_clients[button_number],
                                              !hal_led_pin_get(BSP_LED_0 + button_number));
            break;
        case 3:
            /* Group message: invert all LEDs. */
            status = simple_on_off_client_set_unreliable(&m_clients[GROUP_CLIENT_INDEX],
                                                         !hal_led_pin_get(BSP_LED_0 + button_number), 3);
            break;
        default:
            break;

    }

    if (status == NRF_ERROR_INVALID_STATE ||
        status == NRF_ERROR_NO_MEM ||
        status == NRF_ERROR_BUSY)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send. Device is busy.\n");
        hal_led_blink_ms(LEDS_MASK, 50, 4);
    }
    else
    {
        ERROR_CHECK(status);
    }
}

/*****************************************************************************
 * Event callbacks from the provisioner
 *****************************************************************************/

void provisioner_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_configured_devices);

    /* Set publish address for the client to the corresponding server. */
    ERROR_CHECK(access_model_publish_address_set(m_clients[m_configured_devices].model_handle,
                                                 m_server_handles[m_configured_devices]));
    access_flash_config_store();

    hal_led_pin_set(BSP_LED_0 + m_configured_devices, false);
    m_configured_devices++;

    if (m_configured_devices < SERVER_COUNT)
    {
        provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
        hal_led_pin_set(BSP_LED_0 + m_configured_devices, true);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "All servers provisioned\n");
        hal_led_blink_ms(LEDS_MASK, 100, 4);
    }
}

void provisioner_config_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed\n", m_configured_devices);

    /* Delete key and address. */
    ERROR_CHECK(dsm_address_publish_remove(m_server_handles[m_configured_devices]));
    ERROR_CHECK(dsm_devkey_delete(m_devkey_handles[m_configured_devices]));
    provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
}

void provisioner_prov_complete_cb(const nrf_mesh_prov_evt_complete_t * p_prov_data)
{
    /* We should not get here if all servers are provisioned. */
    NRF_MESH_ASSERT(m_configured_devices < SERVER_COUNT);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning complete. Adding address 0x%04x.\n", p_prov_data->address);

    /* Add to local storage. */
    ERROR_CHECK(dsm_address_publish_add(p_prov_data->address, &m_server_handles[m_provisioned_devices]));
    ERROR_CHECK(dsm_devkey_add(p_prov_data->address, m_netkey_handle, p_prov_data->p_devkey, &m_devkey_handles[m_provisioned_devices]));

    /* Bind the device key to the configuration server and set the new node as the active server. */
    ERROR_CHECK(config_client_server_bind(m_devkey_handles[m_provisioned_devices]));
    ERROR_CHECK(config_client_server_set(m_devkey_handles[m_provisioned_devices],
                                         m_server_handles[m_provisioned_devices]));

    m_provisioned_devices++;

    /* Move on to the configuration step. */
    provisioner_configure(UNPROV_START_ADDRESS + m_configured_devices);
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    hal_leds_init();
    ERROR_CHECK(hal_buttons_init(button_event_handler));

    /* Set the first LED */
    hal_led_pin_set(BSP_LED_0, true);
    mesh_core_setup();
    access_setup();
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    while (true)
    {
        (void)sd_app_evt_wait();
    }
}
