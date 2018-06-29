/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *
 */

#include "mesh_main.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_sdh_soc.h"
#include "access_config.h"
#include "simple_on_off_client.h"
#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"


#define GROUP_MSG_REPEAT_COUNT  (2)
#define MESH_SOC_OBSERVER_PRIO  0

static simple_on_off_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static const uint8_t          m_client_node_uuid[NRF_MESH_UUID_SIZE] = CLIENT_NODE_UUID;
static bool                   m_device_provisioned;
static bool                   m_client_state_on[CLIENT_MODEL_INSTANCE_COUNT];


static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = p_client - &m_clients[0];
    NRF_MESH_ASSERT(index < SERVER_NODE_COUNT);
    return index;
}

static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     NRF_LOG_ERROR("Acknowledged send timedout\n");
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);

    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            m_client_state_on[server_index] = true;
            NRF_LOG_INFO("OnOff server %u status ON\n", server_index);
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            m_client_state_on[server_index] = false;
            NRF_LOG_INFO("OnOff server %u status OFF\n", server_index);
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            NRF_LOG_INFO("No reply from OnOff server %u\n", server_index);
            break;

        case SIMPLE_ON_OFF_STATUS_CANCELLED:
            NRF_LOG_ERROR("Message to server %u cancelled\n", server_index);
            break;
        default:
            NRF_LOG_ERROR("Unknown status \n");
            break;
    }
}

static void provisioning_complete_cb(void)
{
    NRF_LOG_INFO("Successfully provisioned\n");

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    NRF_LOG_INFO("Node Address: 0x%04x \n", node_address.address_start);
}

void mesh_main_button_event_handler(uint32_t button_number)
{
    NRF_LOG_INFO("Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    switch (button_number)
    {
        case 0:
        case 1:
            /* send unicast message, with inverted GPIO pin value */
            status = simple_on_off_client_set(&m_clients[button_number],
                                              !m_client_state_on[button_number]);
            break;

        case 2:
        case 3:
            /* send a group message to the ODD group, with inverted GPIO pin value */
            status = simple_on_off_client_set_unreliable(&m_clients[button_number],
                                                         !m_client_state_on[button_number],
                                                         GROUP_MSG_REPEAT_COUNT);
            if (status == NRF_SUCCESS)
            {
                m_client_state_on[button_number] = !m_client_state_on[button_number];
            }
            break;
        default:
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            NRF_LOG_INFO("Cannot send - client %u is busy\n", button_number);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            NRF_LOG_WARNING("Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void models_init_cb(void)
{
    NRF_LOG_INFO("Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        m_clients[i].timeout_cb = client_publish_timeout_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i + 1));
        ERROR_CHECK(access_model_subscription_list_alloc(m_clients[i].model_handle));
    }
}

void mesh_main_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority     = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc         = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid           = m_client_node_uuid,
        .models.models_init_cb = models_init_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

void mesh_main_start(void)
{
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
}
