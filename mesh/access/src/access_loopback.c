/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#include <stdlib.h>

#include "access_loopback.h"
#include "access_internal.h"
#include "bearer_event.h"
#include "device_state_manager.h"
#include "proxy.h"
#include "list.h"
#include "nrf_mesh.h"
#include "nrf_mesh_defines.h"
#include "nrf_error.h"
#include "utils.h"

typedef struct
{
    list_node_t            node;
    nrf_mesh_rx_metadata_t rx_metadata;
    access_message_rx_t    rx_message;
    uint8_t                data[];
} access_loopback_item_t;

static bearer_event_flag_t m_access_loopback_flag;
static list_node_t * mp_loopback_list_head;

static bool access_loopback_process(void)
{
    while (mp_loopback_list_head != NULL)
    {
        access_loopback_item_t * p_item = PARENT_BY_FIELD_GET(access_loopback_item_t, node, mp_loopback_list_head);

        access_incoming_handle(&p_item->rx_message);
        (void) list_remove(&mp_loopback_list_head, mp_loopback_list_head);
        free(p_item); /*lint !e424 Inappropriate deallocation (free) for 'modified' data */
    }

    return true;
}

bool is_access_loopback(const nrf_mesh_address_t * p_addr)
{
    return dsm_address_is_rx(p_addr);
}

void access_loopback_init(void)
{
    m_access_loopback_flag = bearer_event_flag_add(access_loopback_process);
}

/*lint -save -e429 Custodial pointer p_item has not been freed or returned */
uint32_t access_loopback_handle(access_loopback_request_t * p_req)
{
    access_loopback_item_t * p_item = malloc(sizeof(access_loopback_item_t) + p_req->length);

    if (NULL != p_item)
    {
        p_item->rx_metadata.source = NRF_MESH_RX_SOURCE_LOOPBACK;
        p_item->rx_metadata.params.loopback.tx_token = p_req->token;

        p_item->rx_message.opcode = p_req->opcode;
        p_item->rx_message.p_data = p_item->data;
        p_item->rx_message.length = p_req->length;
        p_item->rx_message.meta_data.src.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        p_item->rx_message.meta_data.src.value = p_req->src_value;
        p_item->rx_message.meta_data.dst = p_req->dst;
        p_item->rx_message.meta_data.ttl = p_req->ttl;
        p_item->rx_message.meta_data.appkey_handle = p_req->appkey_handle;
        p_item->rx_message.meta_data.p_core_metadata = &p_item->rx_metadata;
        p_item->rx_message.meta_data.subnet_handle = p_req->subnet_handle;

        memcpy(p_item->data, p_req->p_data, p_req->length);
        list_add(&mp_loopback_list_head, &p_item->node);
        bearer_event_flag_set(m_access_loopback_flag);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}
/*lint -restore */
