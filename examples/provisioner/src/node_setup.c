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
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "nrf_mesh_utils.h"
#include "device_state_manager.h"
#include "timer_scheduler.h"

#include "config_client.h"
#include "config_server.h"
#include "access_config.h"
#include "generic_level_server.h"
#include "generic_level_client.h"
#include "generic_onoff_server.h"
#include "generic_onoff_client.h"
#include "health_common.h"
#include "light_lightness_client.h"
#include "light_lightness_setup_server.h"
#include "generic_ponoff_setup_server.h"
#include "generic_dtt_server.h"
#include "light_lc_setup_server.h"
#include "light_ctl_setup_server.h"
#include "light_ctl_client.h"
#include "sensor_setup_server.h"
#include "sensor_client.h"
#include "scene_client.h"
#include "scene_setup_server.h"

#include "provisioner_helper.h"
#include "node_setup.h"
#include "example_common.h"
#include "example_network_config.h"
#include "mesh_app_utils.h"
#include "config_scenarios.h"

#include "enc.h"
#include "log.h"
#include "nrf_mesh_assert.h"
#include "fsm_assistant.h"

/*lint -e123 */
#define _DECLARE_CONFIGURATION(STEP)   {STEP, STEP##MODEL_ID},
#define DECLARE_CONFIGURATION(...) MACRO_FOR_EACH(_DECLARE_CONFIGURATION, __VA_ARGS__)

NRF_MESH_STATIC_ASSERT(NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE == sizeof(uint32_t));

typedef enum
{
    STATUS_CHECK_PASS,
    STATUS_CHECK_FAIL,
    STATUS_CHECK_UNEXPECTED_OPCODE
} status_check_t;

/* Structure to hold the composition data */
typedef struct
{
    uint16_t    len;
    struct
    {
        uint8_t page_number;
        uint8_t data[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    }composition;
} composition_data_t;

/* Expected status structure, used for setup state machine */
typedef struct
{
    uint8_t  num_statuses;
    uint16_t expected_opcode;
    const access_status_t  * p_statuses;
} expected_status_list_t;

typedef struct
{
    uint8_t count;
    timer_event_t timer;
} client_send_retry_t;

typedef struct
{
    const config_steps_t step;
    const uint16_t model_id;
} prov_cfg_t;

typedef struct
{
    const char * p_uri_str;
    const prov_cfg_t * p_cfg;
} prov_scenario_t;

/* USER_NOTE:
You can define one or more such configuration steps for a given node in your network. The choice
of the steps can be done in @ref setup_select_steps() function.
*/
static const prov_cfg_t m_onoff_client[]  = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_SWITCH_CLIENT_EXAMPLE)};
static const prov_cfg_t m_onoff_server[]  = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_SWITCH_SERVER_EXAMPLE)};
static const prov_cfg_t m_level_client[]  = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_DIMMING_CLIENT_EXAMPLE)};
static const prov_cfg_t m_level_server[]  = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_DIMMING_SERVER_EXAMPLE)};
static const prov_cfg_t m_ll_client[]     = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHTNESS_CLIENT_EXAMPLE)};
static const prov_cfg_t m_ll_server[]     = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHTNESS_SERVER_EXAMPLE)};
static const prov_cfg_t m_lc_server[]     = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_LC_SERVER_EXAMPLE)};
static const prov_cfg_t m_ctl_server[]    = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_CTL_SERVER_EXAMPLE)};
static const prov_cfg_t m_ctl_lc_server[] = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_CTL_LC_SERVER_EXAMPLE)};
static const prov_cfg_t m_ctl_client[]    = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LIGHT_CTL_CLIENT_EXAMPLE)};
static const prov_cfg_t m_sensor_server[] = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_SENSOR_SERVER_EXAMPLE)};
static const prov_cfg_t m_sensor_client[] = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_SENSOR_CLIENT_EXAMPLE)};
static const prov_cfg_t m_scene_client[]  = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_SCENE_CLIENT_EXAMPLE)};
static const prov_cfg_t m_lpn_client[]    = {DECLARE_CONFIGURATION(CONFIG_SCENARIO_LPN_EXAMPLE)};

static const prov_scenario_t m_scenarios[] =
{
    {EX_URI_ENOCEAN,       m_onoff_client},
    {EX_URI_LPN,           m_lpn_client},
    {EX_URI_DM_CLIENT,     m_level_client},
    {EX_URI_DM_SERVER,     m_level_server},
    {EX_URI_LS_CLIENT,     m_onoff_client},
    {EX_URI_LS_SERVER,     m_onoff_server},
    {EX_URI_LL_CLIENT,     m_ll_client},
    {EX_URI_LL_SERVER,     m_ll_server},
    {EX_URI_LC_SERVER,     m_lc_server},
    {EX_URI_CTL_SERVER,    m_ctl_server},
    {EX_URI_CTL_LC_SERVER, m_ctl_lc_server},
    {EX_URI_CTL_CLIENT,    m_ctl_client},
    {EX_URI_SENSOR_SERVER, m_sensor_server},
    {EX_URI_SENSOR_CLIENT, m_sensor_client},
    {EX_URI_SCENE_CLIENT,  m_scene_client}
};

static uint16_t m_current_node_addr;
static uint16_t m_current_element_addr;
static uint16_t m_retry_count;
static client_send_retry_t m_send_timer;
static const uint8_t * mp_appkey;
static uint16_t m_appkey_idx;
static uint16_t m_netkey_idx;

static const prov_cfg_t m_idle_step[] = {DECLARE_CONFIGURATION(NODE_SETUP_IDLE)};
static const prov_cfg_t * mp_config_step = m_idle_step;
static node_setup_successful_cb_t m_node_setup_success_cb;
static node_setup_failed_cb_t m_node_setup_failed_cb;
static expected_status_list_t m_expected_status_list;
/*lint +e123 */

/* Forward declaration */
static void config_step_execute(void);

/*************************************************************************************************/

static void node_setup_state_clear(void)
{
    timer_sch_abort(&m_send_timer.timer);
    mp_config_step = m_idle_step;
}

static void node_setup_succeed(void)
{
    node_setup_state_clear();
    m_node_setup_success_cb();
}

static void node_setup_fail(void)
{
    node_setup_state_clear();
    m_node_setup_failed_cb();
}

/* Set expected status opcode and acceptable value of status codes */
static void expected_status_set(uint32_t opcode, uint32_t n, const access_status_t * p_list)
{
    if (n > 0)
    {
        NRF_MESH_ASSERT(p_list != NULL);
    }

    m_expected_status_list.expected_opcode = opcode;
    m_expected_status_list.num_statuses = n;
    m_expected_status_list.p_statuses = p_list;
}

static void config_step_retry_schedule(void)
{
    NRF_MESH_ASSERT_DEBUG(m_send_timer.count > 0);
    m_send_timer.count--;

    timestamp_t next_timeout = timer_now() + MS_TO_US(CLIENT_BUSY_SEND_RETRY_DELAY_MS);
    timer_sch_reschedule(&m_send_timer.timer, next_timeout);
}

/* Callback for the timer event */
static void client_send_timer_cb(timestamp_t timestamp, void * p_context)
{
    /* retry the last step */
    config_step_execute();
}

/**
 *  When config client status message is received, this function checks for the expected opcode, and
 *  status values. It is required by the node setup state machine.
 */
static status_check_t check_expected_status(uint16_t rx_opcode, const config_msg_t * p_msg)
{
    uint8_t status = 0xFF;
    if (rx_opcode != m_expected_status_list.expected_opcode)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unexpected opcode: exp 0x%04x  rx 0x%04x\n",
              m_expected_status_list.expected_opcode, rx_opcode);
        return STATUS_CHECK_UNEXPECTED_OPCODE;
    }

    switch (rx_opcode)
    {
        /* These messages do not have a STATUS field. */
        case CONFIG_OPCODE_COMPOSITION_DATA_STATUS:
        case CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS:
            break;

        case CONFIG_OPCODE_MODEL_APP_STATUS:
            status = p_msg->app_status.status;
            break;

        case CONFIG_OPCODE_MODEL_PUBLICATION_STATUS:
            status = p_msg->publication_status.status;
            break;

        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS:
            status = p_msg->subscription_status.status;
            break;

        case CONFIG_OPCODE_APPKEY_STATUS:
            status = p_msg->appkey_status.status;
            break;

        default:
            /** USER_TO_CONFIGURE: Resolve additional required statuses in above switch case */
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Handle additional statuses here");
            ERROR_CHECK(NRF_ERROR_NOT_FOUND);
            break;
    }

    if (m_expected_status_list.num_statuses == 0)
    {
        return STATUS_CHECK_PASS;
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "opcode status field: %d \n", status);
    for (uint32_t i = 0; i < m_expected_status_list.num_statuses; i++)
    {
        if (status == m_expected_status_list.p_statuses[i])
        {
            return STATUS_CHECK_PASS;
        }
    }
    return STATUS_CHECK_FAIL;
}
/*************************************************************************************************/
/* Application-specific functions for combining some commonly used structure assignments */

static access_model_id_t step_model_id_get(void)
{
    access_model_id_t model_id =
    {
        .company_id = ACCESS_COMPANY_ID_NONE,
        .model_id = mp_config_step->model_id
    };

    return model_id;
}

static uint32_t pub_state_set(config_publication_state_t * p_pubstate,
                              uint16_t element_addr,
                              uint16_t publish_addr,
                              access_publish_period_t publish_period)
{
    p_pubstate->element_address = element_addr;
    p_pubstate->publish_address.type = nrf_mesh_address_type_get(publish_addr);
    p_pubstate->publish_address.value = publish_addr;
    p_pubstate->appkey_index = m_appkey_idx;
    p_pubstate->frendship_credential_flag = false;
    p_pubstate->publish_ttl = ACCESS_DEFAULT_TTL;
    p_pubstate->publish_period = publish_period;
    p_pubstate->retransmit_count = 1;
    p_pubstate->retransmit_interval = 0;
    p_pubstate->model_id = step_model_id_get();

    static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
    expected_status_set(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, ARRAY_SIZE(exp_status), exp_status);

    return config_client_model_publication_set(p_pubstate);
}

static uint32_t sub_state_set(uint16_t element_address,
                              uint16_t subscription_addr)
{
    nrf_mesh_address_t address =
    {
        .value = subscription_addr
    };
    address.type = nrf_mesh_address_type_get(subscription_addr);

    static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
    expected_status_set(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, ARRAY_SIZE(exp_status), exp_status);

    return config_client_model_subscription_add(element_address, address, step_model_id_get());
}

static uint32_t appkey_to_model_bind(uint16_t element_address)
{
    static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
    expected_status_set(CONFIG_OPCODE_MODEL_APP_STATUS, ARRAY_SIZE(exp_status), exp_status);

    return config_client_model_app_bind(element_address, m_appkey_idx, step_model_id_get());
}

static uint16_t server_pub_address_get(uint16_t address)
{
    return address & 1 ? SERVER_PUB_GROUP_ADDRESS_ODD : SERVER_PUB_GROUP_ADDRESS_EVEN;
}

static uint16_t client_pub_address_get(uint16_t address)
{
    return address & 1 ? CLIENT_PUB_GROUP_ADDRESS_ODD : CLIENT_PUB_GROUP_ADDRESS_EVEN;
}

#if NRF_MESH_LOG_ENABLE
static const char * model_name_by_id_get(uint16_t model_id)
{
    switch (model_id)
    {
        case HEALTH_SERVER_MODEL_ID:
            return "Health server";
        case GENERIC_ONOFF_SERVER_MODEL_ID:
            return "Generic OnOff server";
        case GENERIC_ONOFF_CLIENT_MODEL_ID:
            return "Generic OnOff client";
        case GENERIC_LEVEL_SERVER_MODEL_ID:
            return "Generic Level server";
        case GENERIC_LEVEL_CLIENT_MODEL_ID:
            return "Generic Level client";
        case LIGHT_LIGHTNESS_CLIENT_MODEL_ID:
            return "Light Lightness client";
        case LIGHT_LIGHTNESS_SERVER_MODEL_ID:
            return "Light Lightness server";
        case LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID:
            return "Light lightness setup server";
        case GENERIC_PONOFF_SERVER_MODEL_ID:
            return "Generic Power OnOff server";
        case GENERIC_PONOFF_SETUP_SERVER_MODEL_ID:
            return "Generic Power OnOff setup server";
        case GENERIC_DTT_SERVER_MODEL_ID:
            return "Generic Default transition time server";
        case LIGHT_LC_SERVER_MODEL_ID:
            return "Light LC server";
        case LIGHT_LC_SETUP_SERVER_MODEL_ID:
            return "Light LC setup server";
        case LIGHT_CTL_SERVER_MODEL_ID:
            return "Light CTL server";
        case LIGHT_CTL_SETUP_SERVER_MODEL_ID:
            return "Light CTL setup server";
        case LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID:
            return "Light CTL temperature server";
        case LIGHT_CTL_CLIENT_MODEL_ID:
            return "Light CTL client";
        case SENSOR_SERVER_MODEL_ID:
            return "Sensor server";
        case SENSOR_SETUP_SERVER_MODEL_ID:
            return "Sensor setup server";
        case SENSOR_CLIENT_MODEL_ID:
            return "Sensor client";
        case SCENE_CLIENT_MODEL_ID:
            return "Scene client";
        case SCENE_SERVER_MODEL_ID:
            return "Scene server";
        case SCENE_SETUP_SERVER_MODEL_ID:
            return "Scene setup server";
        default:
            return "Unknown model";
    }
}
#endif

/*************************************************************************************************/
/* Node setup functionality related static functions */
/** Step execution function for the configuration state machine. */
static void config_step_execute(void)
{
    uint32_t status = NRF_ERROR_INVALID_STATE;

    switch (mp_config_step->step)
    {
        /* Read the composition data from the node: */
        case NODE_SETUP_CONFIG_COMPOSITION_GET:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Getting composition data\n");
            status = config_client_composition_data_get(0x00);

            expected_status_set(CONFIG_OPCODE_COMPOSITION_DATA_STATUS, 0, NULL);
            break;
        }

        case NODE_SETUP_CONFIG_NETWORK_TRANSMIT:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Updating network transmit: count: %d steps: %d\n",
                  NETWORK_TRANSMIT_COUNT, NETWORK_TRANSMIT_INTERVAL_STEPS);
            status = config_client_network_transmit_set(NETWORK_TRANSMIT_COUNT, NETWORK_TRANSMIT_INTERVAL_STEPS);

            expected_status_set(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS, 0 , NULL);
            break;
        }

        /* Add the application key to the node: */
        case NODE_SETUP_CONFIG_APPKEY_ADD:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding appkey\n");
            status = config_client_appkey_add(m_netkey_idx, m_appkey_idx, mp_appkey);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS, ACCESS_STATUS_KEY_INDEX_ALREADY_STORED};
            expected_status_set(CONFIG_OPCODE_APPKEY_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        case NODE_SETUP_GET_NEXT_ELEMENT:
        {
            m_current_element_addr++;
            mp_config_step++;
            config_step_execute();
            status = NRF_SUCCESS;
            break;
        }

        /* Bind server to the application key: */
        case NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LC_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_CTL_TEMPERATURE_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LC_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LL_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SETUP_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_DTT_SERVER:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_CLIENT:
        case NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT:
        case NODE_SETUP_CONFIG_APPKEY_BIND_LL_CLIENT:
        case NODE_SETUP_CONFIG_APPKEY_BIND_CTL_CLIENT:
        case NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_CLIENT:
        case NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_CLIENT:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App key bind: %s on element address 0x%04x\n",
                  model_name_by_id_get(mp_config_step->model_id), m_current_element_addr);
            status = appkey_to_model_bind(m_current_element_addr);
            break;
        }

        /* Configure the publication parameters for the Health server: */
        case NODE_SETUP_CONFIG_PUBLICATION_HEALTH:
        {
            access_publish_period_t publish_period =
            {
                .step_num = 1,
                .step_res = ACCESS_PUBLISH_RESOLUTION_10S
            };
            config_publication_state_t pubstate = {0};
            status = pub_state_set(&pubstate,
                                   m_current_element_addr,
                                   PROVISIONER_ADDRESS,
                                   publish_period);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address for %s to 0x%04x\n",
                  model_name_by_id_get(mp_config_step->model_id),
                  PROVISIONER_ADDRESS);
            break;
        }

        /* Configure the publication parameters for generic models servers to publish to the
        corresponding clients (ODD servers: Client on Element 1 and EVEN servers: Client on
        element 2, of the client example). This demonstrates the state change publication due to local
        event on the server. */
        case NODE_SETUP_CONFIG_PUBLICATION_LEVEL_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_LL_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_LL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SETUP_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_DTT_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_LC_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_LC_SETUP_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_CTL_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_CTL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_CTL_TEMPERATURE_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SETUP_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_SCENE_SERVER:
        case NODE_SETUP_CONFIG_PUBLICATION_SCENE_SETUP_SERVER:
        {
            access_publish_period_t publish_period =
            {
                .step_num = 0,
                .step_res = ACCESS_PUBLISH_RESOLUTION_100MS
            };
            config_publication_state_t pubstate = {0};

            status = pub_state_set(&pubstate,
                                   m_current_element_addr,
                                   server_pub_address_get(m_current_node_addr),
                                   publish_period);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address to 0x%04x for %s on element address 0x%04x\n",
                  server_pub_address_get(m_current_node_addr),
                  model_name_by_id_get(mp_config_step->model_id),
                  m_current_element_addr);
            break;
        }

        /* The sensor server should publish state on the same address as a client
         * since the LC server is a consumer of these messages.  */
        case NODE_SETUP_CONFIG_PUBLICATION_LEVEL_CLIENT:
        case NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT:
        case NODE_SETUP_CONFIG_PUBLICATION_LL_CLIENT:
        case NODE_SETUP_CONFIG_PUBLICATION_CTL_CLIENT:
        case NODE_SETUP_CONFIG_PUBLICATION_SENSOR_CLIENT:
        case NODE_SETUP_CONFIG_PUBLICATION_SCENE_CLIENT:
        {
            access_publish_period_t publish_period =
            {
                .step_num = 0,
                .step_res = ACCESS_PUBLISH_RESOLUTION_100MS
            };
            config_publication_state_t pubstate = {0};

            status = pub_state_set(&pubstate,
                                   m_current_element_addr,
                                   client_pub_address_get(m_current_element_addr),
                                   publish_period);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address to 0x%04x for %s on element address 0x%04x\n",
                  client_pub_address_get(m_current_element_addr),
                  model_name_by_id_get(mp_config_step->model_id),
                  m_current_element_addr);
            break;
        }

        case NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SETUP_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_DTT_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SETUP_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SETUP_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_TEMPERATURE_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SETUP_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_SERVER:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_SETUP_SERVER:
        {
            status = sub_state_set(m_current_element_addr,
                                   client_pub_address_get(m_current_node_addr));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription to address 0x%04x for %s on element address 0x%04x\n",
                  client_pub_address_get(m_current_node_addr),
                  model_name_by_id_get(mp_config_step->model_id),
                  m_current_element_addr);
            break;
        }

        case NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER_ON_SENSOR_STATUS:
        {
            status = sub_state_set(m_current_element_addr,
                                   server_pub_address_get(m_current_node_addr));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription to address 0x%04x for %s on element address 0x%04x\n",
                  server_pub_address_get(m_current_node_addr),
                  model_name_by_id_get(mp_config_step->model_id),
                  m_current_element_addr);
            break;
        }

        case NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_CLIENT:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_CLIENT:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_LL_CLIENT:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_CLIENT:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_CLIENT:
        case NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_CLIENT:
        {
            status = sub_state_set(m_current_element_addr,
                                   server_pub_address_get(m_current_element_addr));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription to address 0x%04x for %s on element address 0x%04x\n",
                  server_pub_address_get(m_current_element_addr),
                  model_name_by_id_get(mp_config_step->model_id),
                  m_current_element_addr);
            break;
        }

        default:
            ERROR_CHECK(NRF_ERROR_NOT_FOUND);
            break;
    }

    if (status != NRF_SUCCESS)
    {
        config_client_pending_msg_cancel();
        if (m_send_timer.count > 0)
        {
            config_step_retry_schedule();
        }
        else
        {
            node_setup_fail();
        }
    }
}

/**
 * This function retrieves the device key for the given address, and configures the tx and rx paths
 * of the config client model.
 */
static void setup_config_client(uint16_t target_addr)
{
    dsm_handle_t        addr_handle = DSM_HANDLE_INVALID;
    dsm_handle_t        devkey_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t  addr;

    addr.type  = NRF_MESH_ADDRESS_TYPE_UNICAST;
    addr.value = target_addr;

    /* Provisioner helper has stored address and device keys already. Retrieve them. */
    ERROR_CHECK(dsm_address_handle_get(&addr, &addr_handle));
    ERROR_CHECK(dsm_devkey_handle_get(addr.value, &devkey_handle));

    /* Configure client to communicate with server at the given address */
    ERROR_CHECK(config_client_server_bind(devkey_handle));
    ERROR_CHECK(config_client_server_set(devkey_handle, addr_handle));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Config client setup: devkey_handle:%d addr_handle:%d\n", devkey_handle, addr_handle);
}

static void config_client_msg_handle(const config_client_event_t * p_event,
                                     uint16_t length)
{
    if (mp_config_step->step == NODE_SETUP_IDLE || mp_config_step->step == NODE_SETUP_DONE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Got unexpected config client message in state %u\n", mp_config_step->step);
        return;
    }
    else
    {
        status_check_t status = check_expected_status(p_event->opcode, p_event->p_msg);
        if (status == STATUS_CHECK_PASS)
        {
            mp_config_step++;
            m_send_timer.count = CLIENT_BUSY_SEND_RETRY_LIMIT;

            if (mp_config_step->step == NODE_SETUP_DONE)
            {
                node_setup_succeed();
            }
            else
            {
                config_step_execute();
            }
        }
        else if (status == STATUS_CHECK_FAIL)
        {
            node_setup_fail();
        }
    }
}

/*************************************************************************************************/
/* Public functions */

/**
 * Process the config client model events, and advances the node setup state machine to the next
 * state, if expected status message is received.
 */
void node_setup_config_client_event_process(config_client_event_type_t event_type,
                                        const config_client_event_t * p_event,
                                        uint16_t length)
{
    if (event_type == CONFIG_CLIENT_EVENT_TYPE_TIMEOUT)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged message status not received \n");

        if (m_retry_count > 0)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Retry ...\n");
            m_retry_count--;
            config_step_execute();
        }
        else
        {
            node_setup_fail();
        }
    }
    else if (event_type == CONFIG_CLIENT_EVENT_TYPE_MSG)
    {
        NRF_MESH_ASSERT_DEBUG(p_event != NULL);
        config_client_msg_handle(p_event, length);
    }
}

/**
 * Begins the node setup process.
 */
void node_setup_start(uint16_t address, uint8_t  retry_cnt, const uint8_t * p_appkey,
                      uint16_t appkey_idx, uint16_t netkey_idx, const char * p_uri)
{
    if (mp_config_step->step != NODE_SETUP_IDLE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot start. Node setup procedure is in progress.\n");
        return;
    }
    m_current_node_addr = address;
    m_current_element_addr = address;
    m_retry_count = retry_cnt;
    m_send_timer.timer.cb = client_send_timer_cb;
    m_send_timer.count = CLIENT_BUSY_SEND_RETRY_LIMIT;
    mp_appkey = p_appkey;
    m_appkey_idx = appkey_idx;
    m_netkey_idx = netkey_idx;

    mp_config_step = NULL;
    for (uint32_t i = 0; i < ARRAY_SIZE(m_scenarios); i++)
    {
        if (strcmp(p_uri, m_scenarios[i].p_uri_str) == 0)
        {
            mp_config_step = m_scenarios[i].p_cfg;
            break;
        }
    }

    if (mp_config_step == NULL)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Internal error: Invalid URI identifier.\n");
        NRF_MESH_ASSERT(false);
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuring Node: 0x%04X\n", m_current_node_addr);

    setup_config_client(m_current_node_addr);
    config_step_execute();
}

void node_setup_cb_set(node_setup_successful_cb_t config_success_cb,
                       node_setup_failed_cb_t config_failed_cb)
{
    NRF_MESH_ASSERT(config_success_cb != NULL);
    NRF_MESH_ASSERT(config_failed_cb != NULL);

    m_node_setup_success_cb = config_success_cb;
    m_node_setup_failed_cb = config_failed_cb;
}

const char * node_setup_uri_get(const uint8_t * p_in_uri_hash)
{
    uint8_t uri_hash[NRF_MESH_UUID_SIZE];

    for (uint32_t i = 0; i < ARRAY_SIZE(m_scenarios); i++)
    {
        enc_s1((const uint8_t *)m_scenarios[i].p_uri_str, strlen(m_scenarios[i].p_uri_str), uri_hash);

        if (memcmp(p_in_uri_hash, uri_hash, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE) == 0)
        {
            return m_scenarios[i].p_uri_str;
        }
    }

    return NULL;
}

void node_setup_uri_check(void)
{
    uint8_t uri_hash[NRF_MESH_UUID_SIZE];
    uint32_t uri_hashes[ARRAY_SIZE(m_scenarios)];

    for (uint8_t i = 0; i < ARRAY_SIZE(m_scenarios); i++)
    {
        enc_s1((const uint8_t *)m_scenarios[i].p_uri_str, strlen(m_scenarios[i].p_uri_str), uri_hash);
        memcpy(&uri_hashes[i], uri_hash, NRF_MESH_BEACON_UNPROV_URI_HASH_SIZE);
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "Known examples URI:hash %s : %04x \n",
              m_scenarios[i].p_uri_str, uri_hashes[i]);
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(m_scenarios) - 1; i++)
    {
        for (uint8_t j = i + 1; j < ARRAY_SIZE(m_scenarios); j++)
        {
            if (uri_hashes[i] == uri_hashes[j])
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "URIs %s and %s have the same hash %04x. Please change URIs .\n",
                                      m_scenarios[i].p_uri_str, m_scenarios[j].p_uri_str, uri_hashes[i]);
            }
        }

    }
}
