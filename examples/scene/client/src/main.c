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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "scene_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the Scene client for sending messages. */
#define APP_SCENE_DELAY_MS           (0)
/* Transition time value used by the Scene client for sending messages. */
#define APP_SCENE_TRANSITION_TIME_MS (0)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_scene_client_publish_interval_cb(access_model_handle_t handle, void *p_self);
static void app_scene_client_status_cb(const scene_client_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const scene_status_params_t * p_in);
static void app_scene_client_register_status_cb(const scene_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const scene_register_status_params_t * p_in);
static void app_scene_client_transaction_status_cb(access_model_handle_t model_handle,
                                                   void *p_args,
                                                   access_reliable_status_t status);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static scene_client_t         m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

const scene_client_callbacks_t client_cbs =
{
    .scene_status_cb = app_scene_client_status_cb,
    .scene_register_status_cb = app_scene_client_register_status_cb,
    .ack_transaction_status_cb = app_scene_client_transaction_status_cb,
    .periodic_publish_cb = app_scene_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(HAL_LED_MASK, false);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void app_scene_client_publish_interval_cb(access_model_handle_t handle, void *p_self)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

static void app_scene_client_status_cb(const scene_client_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const scene_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "Scene client: 0x%04x, Current scene: %d, Target scene: %d, Remaining Time: %d ms, Status code: 0x%02x\n",
            p_meta->src.value, p_in->current_scene, p_in->target_scene, p_in->remaining_time_ms, p_in->status_code);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "Scene client: 0x%04x, Current scene: %d, Status code: 0x%02x\n",
            p_meta->src.value, p_in->current_scene, p_in->status_code);
    }
}

static void app_scene_client_register_status_cb(const scene_client_t * p_self,
                                                      const access_message_rx_meta_t * p_meta,
                                                      const scene_register_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Scene client: 0x%04x, Current scene: %d, Status code: 0x%02x, Scene register: \n",
          p_meta->src.value, p_in->current_scene, p_in->status_code);
    ASSERT(SCENE_REGISTER_ARRAY_SIZE == 16);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "\t[%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n",
          p_in->scenes[0], p_in->scenes[1], p_in->scenes[2], p_in->scenes[3],
          p_in->scenes[4], p_in->scenes[5], p_in->scenes[6], p_in->scenes[7],
          p_in->scenes[8], p_in->scenes[9], p_in->scenes[10], p_in->scenes[11],
          p_in->scenes[12], p_in->scenes[13], p_in->scenes[14], p_in->scenes[15]);
}

static void app_scene_client_transaction_status_cb(access_model_handle_t model_handle,
                                                   void *p_args,
                                                   access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) Send a Scene Store message\n"
    "\t\t Button/RTT 2) Send a Scene Recall message\n"
    "\t\t Button/RTT 3) Send a Scene Delete message\n"
    "\t\t Button/RTT 4) Switch between the clients\n"
    "\t\t        RTT 5) Decrease the current scene number by 1\n"
    "\t\t        RTT 6) Increase the current scene number by 1\n"
    "\t\t        RTT 7) Send a Scene Get message\n"
    "\t\t        RTT 8) Send a Scene Register Get message\n"
    "\t\t------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    button_number++; // To match number printed on devkit
    uint32_t status = NRF_SUCCESS;
    static uint8_t client = 0;
    static scene_store_params_t store_params = {
        .scene_number = 1
    };
    static scene_recall_params_t recall_params = {0};
    static scene_delete_params_t delete_params = {0};

    model_transition_t transition_params = {
        .delay_ms = APP_SCENE_DELAY_MS,
        .transition_time_ms = APP_SCENE_TRANSITION_TIME_MS
    };

    recall_params.scene_number = store_params.scene_number;
    delete_params.scene_number = store_params.scene_number;

    switch (button_number)
    {
        case 1:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: Scene Store: Scene number: %d\n", store_params.scene_number);
            status = scene_client_store(&m_clients[client], &store_params);
            break;
        case 2:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: Scene Recall: Scene number: %d Tid: %d Trans time: %d ms Delay: %d ms\n",
                  recall_params.scene_number, recall_params.tid, transition_params.transition_time_ms,
                  transition_params.delay_ms);
            status = scene_client_recall(&m_clients[client], &recall_params, &transition_params);
            recall_params.tid++;
            break;
        case 3:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: Scene Delete: Scene number: %d\n", delete_params.scene_number);
            status = scene_client_delete(&m_clients[client], &delete_params);
            break;
        case 4:
            client++;
            client = (client < CLIENT_MODEL_INSTANCE_COUNT) ? client : 0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to client instance: %d\n", client);
            break;
        case 5:
            store_params.scene_number = (store_params.scene_number == 1) ? UINT16_MAX : store_params.scene_number - 1;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to scene number: %d\n", store_params.scene_number);
            break;
        case 6:
            store_params.scene_number = (store_params.scene_number == UINT16_MAX) ? 1 : store_params.scene_number + 1;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to scene number: %d\n", store_params.scene_number);
            break;
        case 7:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Scene Get\n");
            status = scene_client_get(&m_clients[client]);
            break;
        case 8:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Scene Register Get\n");
            status = scene_client_register_get(&m_clients[client]);
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send: status: %d\n", client, status);
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", client);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '1' && key <= '8')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;

        ERROR_CHECK(scene_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Scene Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_SCENE_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
