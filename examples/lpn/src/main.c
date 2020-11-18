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
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* LPN */
#include "mesh_lpn.h"
#include "mesh_friendship_types.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"
#include "ble_dfu_support.h"

/* nRF5 SDK */
#include "nrf_soc.h"
#include "nrf_pwr_mgmt.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
/* The maximum duration to scan for incoming Friend Offers. */
#define FRIEND_REQUEST_TIMEOUT_MS (MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS)
/* The upper limit for two subsequent Friend Polls. */
#define POLL_TIMEOUT_MS (SEC_TO_MS(10))
/* The time between LPN sending a request and listening for a response. */
#define RECEIVE_DELAY_MS (100)

#define APP_STATE_OFF                   0
#define APP_STATE_ON                    1

/* The time before state ON is switched to OFF */
#define APP_STATE_ON_TIMEOUT_MS         (SEC_TO_MS(5))

#define APP_UNACK_MSG_REPEAT_COUNT      2

/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                    (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_DELAY_MS              (50)
/* Transition time value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_TRANSITION_TIME_MS    (100)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);
static void send_app_state(bool is_state_on);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
static generic_onoff_client_t m_client;
static bool                   m_device_provisioned;

/* The timer emulates an occupancy sensor by turning lights off after a certain interval,
 * when no activity is detected. The timer starts after an On State message is sent
 * and sends an Off State message after the timeout @ref APP_STATE_ON_TIMEOUT_MS. */
APP_TIMER_DEF(m_state_on_timer);

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

static const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = NULL,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_mask_set(HAL_LED_MASK, false);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
#endif
}

static void provisioning_aborted_cb(void)
{
#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_stop();
#endif
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

    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_service_init();
#endif

    unicast_address_print();

#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
#endif
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }
}

static void state_on_timer_handler(void *p_unused)
{
    UNUSED_VARIABLE(p_unused);

    /* Send state off */
    send_app_state(APP_STATE_OFF);
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");

#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
#endif

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

static void send_app_state(bool is_state_on)
{
    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    set_params.on_off = is_state_on;
    set_params.tid = tid++;
    transition_params.delay_ms = APP_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);

    /* Demonstrate un-acknowledged transaction, using the client model instance */
    /* In this examples, users will not be blocked if the model is busy */
    status = generic_onoff_client_set_unack(&m_client,
                                            &set_params,
                                            &transition_params,
                                            APP_UNACK_MSG_REPEAT_COUNT);

    switch (status)
    {
        case NRF_SUCCESS:
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
#endif
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot send the message\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
#endif
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Publication not configured\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void initiate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initiating the friendship establishment procedure.\n");

    mesh_lpn_friend_request_t freq;
    freq.friend_criteria.friend_queue_size_min_log = MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_16;
    freq.friend_criteria.receive_window_factor = MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_1_0;
    freq.friend_criteria.rssi_factor = MESH_FRIENDSHIP_RSSI_FACTOR_2_0;
    freq.poll_timeout_ms = POLL_TIMEOUT_MS;
    freq.receive_delay_ms = RECEIVE_DELAY_MS;

    uint32_t status = mesh_lpn_friend_request(freq, FRIEND_REQUEST_TIMEOUT_MS);
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Already in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        case NRF_ERROR_INVALID_PARAM:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Friend request parameters outside of valid ranges.\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void terminate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Terminating the active friendship\n");

    uint32_t status = mesh_lpn_friendship_terminate();
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Not in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------\n"
    "\t\t Button/RTT 1) Send the on message.\n"
    "\t\t Button/RTT 2) Send the off message.\n"
    "\t\t Button/RTT 3) Establish or terminate friendship.\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    if (!mesh_stack_is_device_provisioned())
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "The device is not provisioned.\n");
        return;
    }

    ERROR_CHECK(app_timer_stop(m_state_on_timer));

    switch(button_number)
    {
        case 1:
            send_app_state(APP_STATE_ON);

            ERROR_CHECK(app_timer_start(m_state_on_timer,
                                        HAL_MS_TO_RTC_TICKS(APP_STATE_ON_TIMEOUT_MS),
                                        NULL));
            break;

        case 2:
            send_app_state(APP_STATE_OFF);
            break;

        case 3:
        {
            if (!mesh_lpn_is_in_friendship())
            {
                initiate_friendship();
            }
            else /* In a friendship */
            {
                terminate_friendship();
            }
            break;
        }

        /* Initiate node reset */
        case 4:
        {
            if (!mesh_stack_is_device_provisioned())
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
                return;
            }

            /* Clear all the states to reset the node. */
            (void) proxy_stop();
            mesh_stack_config_clear();
            node_reset();
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }
}

#if RTT_INPUT_ENABLED
static void rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}
#endif

static void app_mesh_core_event_cb(const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_LPN_FRIEND_OFFER:
        {
            const nrf_mesh_evt_lpn_friend_offer_t *p_offer = &p_evt->params.friend_offer;

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Received friend offer from 0x%04X\n",
                  p_offer->src);

            uint32_t status = mesh_lpn_friend_accept(p_offer);
            switch (status)
            {
                case NRF_SUCCESS:
                    break;

                case NRF_ERROR_INVALID_STATE:
                case NRF_ERROR_INVALID_PARAM:
                case NRF_ERROR_NULL:
                    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                          "Cannot accept friendship: %d\n",
                          status);
#if SIMPLE_HAL_LEDS_ENABLED
                    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS,
                                     LED_BLINK_CNT_ERROR);
#endif
                    break;

                default:
                    ERROR_CHECK(status);
                    break;
            }

            break;
        }

        case NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend poll procedure complete\n");
            break;

        case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend Request timed out\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
        {
            const nrf_mesh_evt_friendship_established_t *p_est =
                    &p_evt->params.friendship_established;
            (void) p_est;

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Friendship established with: 0x%04X\n",
                  p_est->friend_src);

#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_pin_set(BSP_LED_1, true);
#endif
            break;
        }

        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
        {
            const nrf_mesh_evt_friendship_terminated_t *p_term = &p_evt->params.friendship_terminated;
            UNUSED_VARIABLE(p_term);

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Friendship with 0x%04X terminated. Reason: %d\n",
                  p_term->friend_src, p_term->reason);

#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_pin_set(BSP_LED_1, false);
#endif

            ERROR_CHECK(app_timer_stop(m_state_on_timer));
            break;
        }

        default:
            break;
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    m_client.settings.p_callbacks = &client_cbs;
    m_client.settings.timeout = 0;
    m_client.settings.force_segmented = false;
    m_client.settings.transmic_size = APP_MIC_SIZE;

    ERROR_CHECK(generic_onoff_client_init(&m_client, 1));
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

    /* Register event handler to receive LPN and friendship events. */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
}

#if BLE_DFU_SUPPORT_ENABLED
/** Initializes Power Management. Required for BLE DFU. */
static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
#endif

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh LPN Demo -----\n");

    ERROR_CHECK(app_timer_init());

#if SIMPLE_HAL_LEDS_ENABLED
    hal_leds_init();
#endif

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_init();
    power_management_init();
#endif

    ble_stack_init();
    gap_params_init();
    conn_params_init();

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_service_init();
#endif

    mesh_init();
    ERROR_CHECK(sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE));

    mesh_lpn_init();
}

static void start(void)
{
#if RTT_INPUT_ENABLED
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
#endif

    ERROR_CHECK(app_timer_create(&m_state_on_timer, APP_TIMER_MODE_SINGLE_SHOT,
                                 state_on_timer_handler));

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
            .p_device_uri = EX_URI_LPN
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

#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
#endif
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
