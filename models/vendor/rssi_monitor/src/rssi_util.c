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

/* NOTE: This models needs to be placed in the root element of the unit to function properly */

#include <string.h> 

#include "rssi_util.h"
#include "rssi_common.h"
#include "access_config.h"
#include "log.h"
#include "device_state_manager.h"

/* Structure storing a corresponding mac/element address pair */
typedef struct
{
    uint8_t mac_addr[BLE_GAP_ADDR_LEN];
    dsm_local_unicast_address_t addr_ctx;
} addr_pair_t;

static dsm_local_unicast_address_t m_local_unicast_address; /*< The unicast address context of the unit */
static addr_pair_t m_addr_buffer[RSSI_DATA_BUFFER_SIZE]; /*< Buffer for storing mac/element address pairs for nearby nodes */
static uint8_t m_addr_buffer_entry_counter = 0; 

/* Sends a database beacon to nearby nodes */
static uint32_t send_database_beacon(rssi_util_t * p_util)
{
    dsm_local_unicast_addresses_get(&m_local_unicast_address);

    access_message_tx_t message;
    message.opcode.opcode = RSSI_OPCODE_SEND_DATABASE_BEACON;
    message.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    message.p_buffer = (const uint8_t*) &m_local_unicast_address;
    message.length = sizeof(m_local_unicast_address);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    /* To assure that the message can't make more than one jump */
    uint32_t error_code = access_model_publish_ttl_set(p_util->model_handle, 0);
    if (error_code == NRF_SUCCESS)
    {
        error_code = access_model_publish(p_util->model_handle, &message);
    }
    return error_code;
}

/* Sends a request to nearby nodes to send a database beacon */
static uint32_t database_beacon_req(const rssi_util_t * p_util)
{   
    access_message_tx_t message;
    message.opcode.opcode = RSSI_OPCODE_REQUEST_DATABASE_BEACON;
    message.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    message.p_buffer = NULL;
    message.length = 0;
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    
    /* To assure that the message can't make more than one jump */
    uint32_t error_code = access_model_publish_ttl_set(p_util->model_handle, 0);
    if (error_code == NRF_SUCCESS)
    {
        error_code = access_model_publish(p_util->model_handle, &message);
    }
    return error_code;
}

/********** Incoming message handlers **********/

static void handle_database_beacon_req(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{ 
    (void) send_database_beacon((rssi_util_t*) p_args);
}

static void handle_database_beacon(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    dsm_local_unicast_address_t incoming_unicast_address;
    memcpy(&incoming_unicast_address, p_message->p_data, p_message->length);

    /* Checks that the incoming database beacon message was recived on the scanner.
       This is done to prevent the model from sending beacons to itself */
    if (p_message->meta_data.p_core_metadata->source != NRF_MESH_RX_SOURCE_SCANNER)
    {
        return;
    }

    /* Checks that the incoming database beacon message came from a static address typr */
    if ((p_message->meta_data.p_core_metadata->params.scanner.adv_addr.addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE) ||
        (p_message->meta_data.p_core_metadata->params.scanner.adv_addr.addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE))
    {
        return;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Recived a database beacon from node: 0x%04x\n", p_message->meta_data.src.value);
    
    /* Checks if a entry already exists */
    for (uint8_t i = 0; i < m_addr_buffer_entry_counter; i++)
    {
        if (!(memcmp(m_addr_buffer[i].mac_addr, p_message->meta_data.p_core_metadata->params.scanner.adv_addr.addr, sizeof(m_addr_buffer[0].addr_ctx.address_start))))
        {
            memcpy(m_addr_buffer[i].mac_addr, p_message->meta_data.p_core_metadata->params.scanner.adv_addr.addr, sizeof(m_addr_buffer[0].mac_addr));
            m_addr_buffer[i].addr_ctx.address_start = incoming_unicast_address.address_start;
            m_addr_buffer[i].addr_ctx.count = incoming_unicast_address.count;
            return;
        }
    }
    /* Adds a new entry as long as there is room in the buffer */
    if (m_addr_buffer_entry_counter < RSSI_DATA_BUFFER_SIZE)
    {
        memcpy(m_addr_buffer[m_addr_buffer_entry_counter].mac_addr, p_message->meta_data.p_core_metadata->params.scanner.adv_addr.addr, sizeof(m_addr_buffer[0].mac_addr));
        m_addr_buffer[m_addr_buffer_entry_counter].addr_ctx.address_start = incoming_unicast_address.address_start;
        m_addr_buffer[m_addr_buffer_entry_counter].addr_ctx.count = incoming_unicast_address.count;

        m_addr_buffer_entry_counter++;
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Address buffer is full\n");
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    { ACCESS_OPCODE_VENDOR(RSSI_OPCODE_REQUEST_DATABASE_BEACON, ACCESS_COMPANY_ID_NORDIC),   handle_database_beacon_req },
    { ACCESS_OPCODE_VENDOR(RSSI_OPCODE_SEND_DATABASE_BEACON, ACCESS_COMPANY_ID_NORDIC),   handle_database_beacon },
};

/********** Interface functions **********/ 

uint32_t rssi_util_mac_to_element_addr_find(rssi_util_t * p_util, const uint8_t* p_mac_addr, dsm_local_unicast_address_t* p_element_addr)
{
    for (uint8_t i = 0; i < (m_addr_buffer_entry_counter + 1); i++)
    {
        if (!(memcmp(m_addr_buffer[i].mac_addr, p_mac_addr, sizeof(m_addr_buffer[0].addr_ctx.address_start))))
        {
            p_element_addr->address_start = m_addr_buffer[i].addr_ctx.address_start; 
            p_element_addr->count = m_addr_buffer[i].addr_ctx.count; 
            return NRF_SUCCESS;
        }
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Could not find the corresponding element address. Sending a database beacon request to adjacent nodes\n");
    (void) database_beacon_req(p_util);
    p_element_addr->address_start = 0;
    p_element_addr->count = 0;
    return NRF_ERROR_NOT_FOUND;
}

uint32_t rssi_util_init(rssi_util_t * p_util)
{
    if (p_util == NULL)
    {
        return NRF_ERROR_NULL;
    }   
    access_model_add_params_t add_params =
    {
        .element_index = 0,
        .model_id.model_id = RSSI_UTIL_MODEL_ID, /*lint !e64 Type Mismatch */
        .model_id.company_id = ACCESS_COMPANY_ID_NORDIC,
        .p_opcode_handlers = m_opcode_handlers,
        .opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]),
        .publish_timeout_cb = NULL,
        .p_args = p_util
    };
    return access_model_add(&add_params, &p_util->model_handle);
}