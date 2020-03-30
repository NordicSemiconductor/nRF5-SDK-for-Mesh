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

#include "rssi_server.h"
#include "nrf_mesh_assert.h"
#include "access_reliable.h"
#include "rssi_common.h" 
#include "log.h"
#include "nordic_common.h"
#include "access_utils.h"

static raw_rssi_data_entry_t m_rssi_entries[RSSI_DATA_BUFFER_SIZE]; /**< Buffer for temporary storing data between sending windows */  
static rssi_data_entry_t m_outgoing_rssi_data[RSSI_DATA_BUFFER_SIZE]; /**< Holds the data that is sent to the client */ 
static bool m_active_transfer; /**< Indicates if there is an active transfer going on between the server and the client */
static uint8_t m_rssi_entry_counter;

NRF_MESH_STATIC_ASSERT(sizeof(m_outgoing_rssi_data) + ACCESS_UTILS_VENDOR_OPCODE_SIZE <= ACCESS_MESSAGE_LENGTH_MAX);

/* Forward declarations */
static uint32_t send_rssi_data(rssi_server_t * p_server);

static void rssi_publish_timeout_handler(access_model_handle_t handle, void * p_args) 
{
    (void)send_rssi_data((rssi_server_t*) p_args);
}
 
static void reliable_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reliable transfer of RSSI data has timed out\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            break;

        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }

    /* Allow new data to be sent */
    m_active_transfer = false;
}

static uint32_t send_reliable_message(const rssi_server_t * p_server, uint16_t opcode, const uint8_t * p_data, uint16_t length)
{
    const access_reliable_t reliable = {
        .model_handle = p_server->model_handle,
        .message = {
            .p_buffer = p_data,
            .length = length,
            .opcode = ACCESS_OPCODE_VENDOR(opcode, ACCESS_COMPANY_ID_NORDIC),
            .force_segmented = false,
            .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
        },
        .reply_opcode = {
            .opcode = RSSI_OPCODE_RSSI_ACK,
            .company_id = ACCESS_COMPANY_ID_NORDIC,
        },
        .timeout = SEC_TO_US(30),
        .status_cb = reliable_status_cb,
    };

    return access_model_reliable_publish(&reliable);
}

/* Refine and sends the collected rssi data to the client */
static uint32_t send_rssi_data(rssi_server_t * p_server)
{
    /* Returns if a active transfer is in session */
    if (m_active_transfer)
    {
        return NRF_SUCCESS;
    }
    
    /* Calculates and packs the outgoing data */
    for (uint8_t i = 0; i < m_rssi_entry_counter; i++)
    {
        m_outgoing_rssi_data[i].src_addr = m_rssi_entries[i].src_addr;
        m_outgoing_rssi_data[i].mean_rssi = m_rssi_entries[i].rssi_sum / m_rssi_entries[i].msg_count;
        m_outgoing_rssi_data[i].msg_count = m_rssi_entries[i].msg_count;
    }

    uint32_t error_code = send_reliable_message(p_server, RSSI_OPCODE_SEND_RSSI_DATA, (const uint8_t*) m_outgoing_rssi_data, sizeof(m_outgoing_rssi_data[0]) * m_rssi_entry_counter);

    if (error_code == NRF_SUCCESS)
    {
        m_active_transfer = true;

        /* Reset the local RSSI data buffer */
        memset(m_rssi_entries, NULL, sizeof(m_rssi_entries));
        m_rssi_entry_counter = 0;
    }

    return error_code;
}

/* Handler needed for the acknowledgement messages from the client */
static void acknowledgement_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    
} 

static const access_opcode_handler_t m_opcode_handlers[] =
{
    { ACCESS_OPCODE_VENDOR(RSSI_OPCODE_RSSI_ACK, ACCESS_COMPANY_ID_NORDIC),   acknowledgement_handler },
};

/********** Interface functions **********/

uint32_t rssi_server_add_rssi_data(uint16_t address, int8_t rssi)
{
    if (address == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    /* Searches for a previous entry with the same address. If it exists the current data will be added to the same entry */
    for (uint8_t i = 0; i < m_rssi_entry_counter; i++)
    { 
        if (address == m_rssi_entries[i].src_addr)
        {
            /* Returns if one of the message counters in the buffer overflows. Also blanks the buffer and resets the buffer counter*/
            if (m_rssi_entries[i].msg_count != UINT8_MAX)
            {   
                m_rssi_entries[i].rssi_sum  += rssi;
                m_rssi_entries[i].msg_count++;
            }

            return NRF_SUCCESS;
        }
    }
    /* Adds a new entry for the current data as long as there is still room in the buffer */
    if (m_rssi_entry_counter < RSSI_DATA_BUFFER_SIZE)
    {
        m_rssi_entries[m_rssi_entry_counter].src_addr = address;
        m_rssi_entries[m_rssi_entry_counter].rssi_sum = rssi;
        m_rssi_entries[m_rssi_entry_counter].msg_count = 1;
        m_rssi_entry_counter++;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

uint32_t rssi_server_init(rssi_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }   
    access_model_add_params_t add_params =
    {
        .element_index = element_index,
        .model_id.model_id = RSSI_SERVER_MODEL_ID, /*lint !e64 Type Mismatch */
        .model_id.company_id = ACCESS_COMPANY_ID_NORDIC,
        .p_opcode_handlers = m_opcode_handlers,
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .publish_timeout_cb = rssi_publish_timeout_handler,
        .p_args = p_server
    };
    return access_model_add(&add_params, &p_server->model_handle);
}

