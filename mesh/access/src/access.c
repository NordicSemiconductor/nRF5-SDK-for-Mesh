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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "access.h"
#include "access_internal.h"
#include "access_config.h"
#include "access_publish_retransmission.h"

#include "access_publish.h"
#include "access_reliable.h"
#include "access_utils.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh.h"

#include "mesh_mem.h"
#include "device_state_manager.h"
#include "log.h"
#include "bitfield.h"
#include "timer.h"
#include "toolchain.h"
#include "event.h"
#include "bearer_event.h"
#include "mesh_opt_access.h"
#include "nrf_mesh_config_app.h"
#include "mesh_config_entry.h"

typedef struct
{
    uint8_t is_metadata_stored : 1;
    uint8_t is_load_failed : 1;
    uint8_t is_restoring_ended : 1;
} local_access_status_t;

/*lint -e415 -e416 Lint fails to understand the boundary checking used for handles in this module (MBTLE-1831). */

/** Access model pool. @ref ACCESS_MODEL_COUNT is set by user at compile time. */
static access_common_t m_model_pool[ACCESS_MODEL_COUNT];

/** Access element pool. @ref ACCESS_ELEMENT_COUNT is set by user at compile time. */
static access_element_t m_element_pool[ACCESS_ELEMENT_COUNT];

/** Access subscription list pool. Makes it possible to share a subscription list  */
static access_subscription_list_t m_subscription_list_pool[ACCESS_SUBSCRIPTION_LIST_COUNT];

/** Mesh event handler. */
static nrf_mesh_evt_handler_t m_evt_handler;

/** Default TTL value for the node. */
static uint8_t m_default_ttl = ACCESS_DEFAULT_TTL;

/** Set of the global flags to keep track of the access layer changes.*/
static local_access_status_t m_status;

/* ********** Static asserts ********** */

NRF_MESH_STATIC_ASSERT(ACCESS_MODEL_COUNT > 0);
NRF_MESH_STATIC_ASSERT((ACCESS_ELEMENT_COUNT > 0) && (ACCESS_ELEMENT_COUNT <= 0xff));
NRF_MESH_STATIC_ASSERT(ACCESS_PUBLISH_RESOLUTION_MAX <=
                       ((1 << ACCESS_PUBLISH_STEP_RES_BITS) - 1));
NRF_MESH_STATIC_ASSERT(ACCESS_PUBLISH_PERIOD_STEP_MAX <=
                       ((1 << ACCESS_PUBLISH_STEP_NUM_BITS) - 1));

/* ********** Static functions ********** */

static inline access_model_handle_t find_available_model(void)
{
    for (unsigned i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        if (!ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[i].internal_state))
        {
            return i;
        }
    }
    return ACCESS_HANDLE_INVALID;
}

static void increment_model_count(uint16_t element_index, uint16_t model_company_id)
{
    if (model_company_id == ACCESS_COMPANY_ID_NONE)
    {
        m_element_pool[element_index].sig_model_count++;
    }
    else
    {
        m_element_pool[element_index].vendor_model_count++;
    }
}

static bool element_has_model_id(uint16_t element_index, access_model_id_t model_id, access_model_handle_t * p_model_handle)
{
    if ((m_element_pool[element_index].sig_model_count +
         m_element_pool[element_index].vendor_model_count) > 0)
    {
        for (access_model_handle_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
        {
            if (m_model_pool[i].model_info.element_index       == element_index &&
                m_model_pool[i].model_info.model_id.model_id   == model_id.model_id &&
                m_model_pool[i].model_info.model_id.company_id == model_id.company_id)
            {
                *p_model_handle = i;
                return true;
            }
        }
    }
    return false;
}

static access_opcode_t opcode_get(const uint8_t * p_buffer)
{
    access_opcode_t opcode = {0};
    switch (p_buffer[0] & ACCESS_PACKET_OPCODE_FORMAT_MASK)
    {
        case ACCESS_PACKET_OPCODE_FORMAT_1BYTE0:
        case ACCESS_PACKET_OPCODE_FORMAT_1BYTE1:
            opcode.opcode = p_buffer[0];
            opcode.company_id = ACCESS_COMPANY_ID_NONE;
            break;
        case ACCESS_PACKET_OPCODE_FORMAT_2BYTE:
            opcode.opcode = ((uint16_t) p_buffer[0] << 8) | (p_buffer[1]);
            opcode.company_id = ACCESS_COMPANY_ID_NONE;
            break;
        case ACCESS_PACKET_OPCODE_FORMAT_3BYTE:
            opcode.opcode = p_buffer[0];
            opcode.company_id = (p_buffer[1]) | ((uint16_t) p_buffer[2] << 8);
            break;
    }
    return opcode;
}

static void opcode_set(access_opcode_t opcode, uint8_t * p_buffer)
{
    if (opcode.company_id != ACCESS_COMPANY_ID_NONE)
    {
        p_buffer[0] = opcode.opcode & 0x00FF;
        p_buffer[1] = opcode.company_id & 0x00FF;
        p_buffer[2] = (opcode.company_id >> 8) & 0x00FF;
    }
    else if ((opcode.opcode & 0xFF00) > 0)
    {
        p_buffer[0] = (opcode.opcode >> 8) & 0x00FF;
        p_buffer[1] = opcode.opcode & 0x00FF;
    }
    else
    {
        p_buffer[0] = opcode.opcode & 0x00FF;
    }
}

static bool is_valid_opcode(access_opcode_t opcode)
{
    /** See opcode format table in @ref access_opcode_t documentation. */
    if (opcode.company_id == ACCESS_COMPANY_ID_NONE)
    {
        if ((opcode.opcode & 0xFF00) > 0)
        {
            /* Upper bits set to 0b10xxxxxx xxxxxxxx */
            return ((opcode.opcode & 0xC000) == 0x8000);
        }
        else
        {
            /* Upper bit set to 0b0xxxxxx, and not 0x01111111*/
            return ((opcode.opcode & 0xFF80) == 0x00) && (opcode.opcode != 0x7F);
        }
    }
    else
    {
        /* Two upper bits set to 0b11xxxxxx */
        return ((opcode.opcode & 0xFFC0) == 0x00C0);
    }
}

static bool opcodes_are_valid(const access_opcode_handler_t * p_opcode_handlers, uint32_t opcode_count)
{
    for (uint32_t i = 0; i < opcode_count; ++i)
    {
        if (!is_valid_opcode(p_opcode_handlers[i].opcode))
        {
            return false;
        }
    }
    return true;
}

static bool is_opcode_of_model(access_common_t * p_model, access_opcode_t opcode, uint32_t * p_opcode_index)
{
    for (uint32_t i = 0; i < p_model->opcode_count; ++i)
    {
        if (p_model->p_opcode_handlers[i].opcode.opcode     == opcode.opcode     &&
            p_model->p_opcode_handlers[i].opcode.company_id == opcode.company_id)
        {
            *p_opcode_index = i;
            return true;
        }
    }
    return false;
}

static inline bool model_handle_valid_and_allocated(access_model_handle_t handle)
{
    return (handle < ACCESS_MODEL_COUNT && ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[handle].internal_state));
}

static void mesh_msg_handle(const nrf_mesh_evt_message_t * p_evt)
{
    NRF_MESH_ASSERT(p_evt != NULL);
    access_opcode_t opcode = opcode_get(p_evt->p_buffer);
    if (opcode.opcode == ACCESS_OPCODE_INVALID)
    {
        return;
    }
    __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "RX: [aop: 0x%04x]\n", opcode.opcode);

    /*lint -save -e64 -e446 Side effects and type mismatches in initializer */
    dsm_handle_t appkey_handle = dsm_appkey_handle_get(p_evt->secmat.p_app);
    dsm_handle_t subnet_handle = dsm_subnet_handle_get(p_evt->secmat.p_net);
    access_message_rx_t message =
    {
        .opcode = opcode,
        .p_data = &p_evt->p_buffer[access_utils_opcode_size_get(opcode)],
        .length = p_evt->length - access_utils_opcode_size_get(opcode),
        .meta_data.src.type = p_evt->src.type,
        .meta_data.src.value = p_evt->src.value,
        .meta_data.src.p_virtual_uuid = p_evt->src.p_virtual_uuid,
        .meta_data.dst.type = p_evt->dst.type,
        .meta_data.dst.value = p_evt->dst.value,
        .meta_data.dst.p_virtual_uuid = p_evt->dst.p_virtual_uuid,
        .meta_data.ttl = p_evt->ttl,
        .meta_data.appkey_handle = appkey_handle,
        .meta_data.p_core_metadata = p_evt->p_metadata,
        .meta_data.subnet_handle = subnet_handle,
    };
    /*lint -restore */

    __LOG_XB(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "RX: Msg", message.p_data, message.length);

    access_incoming_handle(&message);
}

static void mesh_evt_cb(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_MESSAGE_RECEIVED:
            mesh_msg_handle(&p_evt->params.message);
            break;
        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            if (p_evt->params.config_load_failure.id.file == MESH_OPT_ACCESS_FILE_ID)
            {
                m_status.is_load_failed = 1;
            }
            break;
        default:
            /* Ignore */
            break;
    }

}

static bool check_tx_params(access_model_handle_t handle, const access_message_tx_t * p_tx_message, const access_message_rx_t * p_rx_message, uint32_t * p_status)
{
    NRF_MESH_ASSERT(NULL != p_status);

    /* Various checks being done here to ensure that model does not publish a message if
       publication is disabled by setting - Unassigned publish address, or deleted app key */
    if (p_tx_message->length >= ACCESS_MESSAGE_LENGTH_MAX)
    {
        *p_status = NRF_ERROR_INVALID_LENGTH;
    }
    else if (!model_handle_valid_and_allocated(handle) ||
             m_model_pool[handle].model_info.element_index >= ACCESS_ELEMENT_COUNT)
    {
        *p_status = NRF_ERROR_NOT_FOUND;
    }
    else if ((p_rx_message == NULL &&
             (m_model_pool[handle].model_info.publish_appkey_handle  == DSM_HANDLE_INVALID ||
              m_model_pool[handle].model_info.publish_address_handle == DSM_HANDLE_INVALID)) ||
              !is_valid_opcode(p_tx_message->opcode))
    {
        *p_status = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        *p_status = NRF_SUCCESS;
    }

    return (NRF_SUCCESS == *p_status);
}

static uint32_t packet_tx(access_model_handle_t handle,
                          const access_message_tx_t * p_tx_message,
                          const access_message_rx_t * p_rx_message,
                          const uint8_t *p_access_payload,
                          uint16_t access_payload_len)
{
    uint32_t status = NRF_SUCCESS;

    dsm_local_unicast_address_t local_addresses;
    dsm_local_unicast_addresses_get(&local_addresses);
    if (m_model_pool[handle].model_info.element_index >= local_addresses.count)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t src_address = local_addresses.address_start + m_model_pool[handle].model_info.element_index;

    nrf_mesh_address_t dst_address;
    dsm_handle_t appkey_handle;
    dsm_handle_t subnet_handle = DSM_HANDLE_INVALID;
    if (p_rx_message != NULL)
    {
        appkey_handle = p_rx_message->meta_data.appkey_handle;
        subnet_handle = p_rx_message->meta_data.subnet_handle;
        dst_address = p_rx_message->meta_data.src;
    }
    else
    {
        appkey_handle = m_model_pool[handle].model_info.publish_appkey_handle;
        if (dsm_address_get(m_model_pool[handle].model_info.publish_address_handle, &dst_address) != NRF_SUCCESS)
        {
            return NRF_ERROR_NOT_FOUND;
        }
        if  (dst_address.value == NRF_MESH_ADDR_UNASSIGNED)
        {
            return NRF_ERROR_INVALID_ADDR;
        }
    }

    uint8_t ttl;
    if (p_rx_message != NULL && p_rx_message->meta_data.ttl == 0)
    {
        ttl = 0;
    }
    else if (m_model_pool[handle].model_info.publish_ttl == ACCESS_TTL_USE_DEFAULT)
    {
        ttl = m_default_ttl;
    }
    else
    {
        ttl = m_model_pool[handle].model_info.publish_ttl;
    }


    NRF_MESH_ASSERT_DEBUG(p_access_payload != NULL);
    NRF_MESH_ASSERT_DEBUG(access_payload_len != 0);

    nrf_mesh_tx_params_t tx_params;
    memset(&tx_params, 0, sizeof(tx_params));

#if MESH_FEATURE_LPN_ENABLED
    status = NRF_ERROR_NOT_FOUND;
    if (m_model_pool[handle].model_info.friendship_credential_flag)
    {
        status = dsm_tx_friendship_secmat_get(subnet_handle, appkey_handle, &tx_params.security_material);
    }

    /* @tagMeshSp section 4.2.2.4:
     *
     * When Publish Friendship Credential Flag is set to 1 and the friendship security material is
     * not available, the master security material shall be used. */
    if (NRF_ERROR_NOT_FOUND == status)
#endif
    {
        status = dsm_tx_secmat_get(subnet_handle, appkey_handle, &tx_params.security_material);
    }

    if (status != NRF_SUCCESS)
    {
        return status;
    }

    tx_params.dst = dst_address;
    tx_params.src = src_address;
    tx_params.ttl = ttl;
    tx_params.force_segmented = p_tx_message->force_segmented;
    tx_params.transmic_size = p_tx_message->transmic_size;
    tx_params.p_data = p_access_payload;
    tx_params.data_len = access_payload_len;
    tx_params.tx_token = p_tx_message->access_token;

    status = nrf_mesh_packet_send(&tx_params, NULL);
    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "TX: [aop: 0x%04x] \n", p_tx_message->opcode.opcode);
        __LOG_XB(LOG_SRC_ACCESS, LOG_LEVEL_DBG1, "TX: Msg", p_access_payload, access_payload_len);
    }

    return status;
}

static uint32_t packet_alloc_and_tx(access_model_handle_t handle,
                                    const access_message_tx_t * p_tx_message,
                                    const access_message_rx_t * p_rx_message,
                                    uint8_t **pp_access_payload,
                                    uint16_t *p_access_payload_len)
{
    NRF_MESH_ASSERT_DEBUG(p_tx_message != NULL);

    uint32_t status;

    if (!check_tx_params(handle, p_tx_message, p_rx_message, &status))
    {
        return status;
    }

    uint16_t opcode_length = access_utils_opcode_size_get(p_tx_message->opcode);
    uint16_t payload_length = opcode_length + p_tx_message->length;

    uint8_t *p_payload = (uint8_t *) mesh_mem_alloc(payload_length);
    if (NULL == p_payload)
    {
        return NRF_ERROR_NO_MEM;
    }

    opcode_set(p_tx_message->opcode, p_payload);
    memcpy(&p_payload[opcode_length], p_tx_message->p_buffer, p_tx_message->length);

    status = packet_tx(handle, p_tx_message, p_rx_message, p_payload, payload_length);
    if (NRF_SUCCESS != status || NULL == pp_access_payload || NULL == p_access_payload_len)
    {
        mesh_mem_free(p_payload);
        return status;
    }

    *pp_access_payload = p_payload;
    *p_access_payload_len = payload_length;

    return NRF_SUCCESS;
}

static void access_state_clear(void)
{
    memset(&m_model_pool[0], 0, sizeof(m_model_pool));
    memset(&m_element_pool[0], 0, sizeof(m_element_pool));
    memset(&m_subscription_list_pool[0], 0, sizeof(m_subscription_list_pool));
    for (uint16_t i = 0; i < ARRAY_SIZE(m_model_pool); ++i)
    {
        m_model_pool[i].model_info.publish_address_handle = DSM_HANDLE_INVALID;
        m_model_pool[i].model_info.publish_appkey_handle = DSM_HANDLE_INVALID;
        m_model_pool[i].model_info.subscription_pool_index = ACCESS_SUBSCRIPTION_LIST_COUNT;
        m_model_pool[i].model_info.element_index = ACCESS_ELEMENT_INDEX_INVALID;
        m_model_pool[i].publish_divisor = 1;
    }
    m_default_ttl = ACCESS_DEFAULT_TTL;
}

static bool model_subscribes_to_addr(const access_common_t * p_model, dsm_handle_t address_handle)
{
    return (p_model->model_info.subscription_pool_index < ACCESS_SUBSCRIPTION_LIST_COUNT &&
            bitfield_get(m_subscription_list_pool[p_model->model_info.subscription_pool_index].bitfield,
                         address_handle));
}

/**
 * Checks whether a destination address is designated for a specific element.
 *
 * @param[in] p_dst The destination address to check for
 * @param[in,out] p_index Index of the element that should process the message destined for the given address.
 *
 * @returns Whether or not the destination address is designated for a specific element.
 */
static bool is_element_rx_address(const nrf_mesh_address_t * p_dst, uint16_t * p_index)
{
    if (p_dst->type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        dsm_local_unicast_address_t local_addresses;
        dsm_local_unicast_addresses_get(&local_addresses);

        *p_index = p_dst->value - local_addresses.address_start;
        return true;
    }
    else if (p_dst->value == NRF_MESH_ALL_PROXIES_ADDR ||
             p_dst->value == NRF_MESH_ALL_FRIENDS_ADDR ||
             p_dst->value == NRF_MESH_ALL_RELAYS_ADDR ||
             p_dst->value == NRF_MESH_ALL_NODES_ADDR)
    {
        /* According to @tagMeshSp section 3.4.2, only models on the
         * primary element of the node shall process the fixed group addresses. */
        *p_index = 0;
        return true;
    }
    return false;
}

/* Checks if the model's subscription list is shared. Note: Model handle must be valid */
static bool model_subscription_list_is_shared(access_model_handle_t handle)
{
    NRF_MESH_ASSERT_DEBUG(model_handle_valid_and_allocated(handle));
    uint16_t sub_pool_index = m_model_pool[handle].model_info.subscription_pool_index;

    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        if (m_model_pool[i].model_info.subscription_pool_index == sub_pool_index && i != handle)
        {
            return true;
        }
    }

    return false;
}

/* Calculates the value of the fast publish interval. Minimum possible output interval will be 100 ms. */
static void divide_publish_interval(access_publish_resolution_t input_resolution, uint8_t input_steps,
        access_publish_resolution_t * p_output_resolution, uint8_t * p_output_steps, uint16_t divisor)
{
    uint32_t ms100_value = input_steps;

    /* Convert the step value to 100 ms steps depending on the resolution: */
    switch (input_resolution)
    {
        case ACCESS_PUBLISH_RESOLUTION_100MS:
            break;
        case ACCESS_PUBLISH_RESOLUTION_1S:
            ms100_value *= 10;
            break;
        case ACCESS_PUBLISH_RESOLUTION_10S:
            ms100_value *= 100;
            break;
        case ACCESS_PUBLISH_RESOLUTION_10MIN:
            ms100_value *= 6000;
            break;
        default:
            NRF_MESH_ASSERT(false);
    }

    /* Divide the 100 ms value with the fast divisor: */
    ms100_value /= divisor;

    /* Get the divisor corresponding to the new number of 100 ms steps: */
    if (ms100_value <= ACCESS_PUBLISH_PERIOD_STEP_MAX)
    {
        *p_output_resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
        if (input_steps == 0)
        {
            *p_output_steps = 0;
        }
        else
        {
            /* Avoid accidentally disabling publishing by dividing by too high a number */
            *p_output_steps = MAX(1, ms100_value);
        }
    }
    else if (ms100_value <= (ACCESS_PUBLISH_PERIOD_STEP_MAX * 10))
    {
        *p_output_resolution = ACCESS_PUBLISH_RESOLUTION_1S;
        *p_output_steps = ms100_value / 10;
    }
    else if (ms100_value <= (ACCESS_PUBLISH_PERIOD_STEP_MAX * (10*10)))
    {
        *p_output_resolution = ACCESS_PUBLISH_RESOLUTION_10S;
        *p_output_steps = ms100_value / (10*10);
    }
    else
    {
        *p_output_resolution = ACCESS_PUBLISH_RESOLUTION_10MIN;
        *p_output_steps = ms100_value / (10*60*10);
    }

}

static void access_publish_timing_update(access_model_handle_t handle)
{
    access_publish_resolution_t reg_res = ACCESS_PUBLISH_RESOLUTION_100MS;
    access_publish_resolution_t fast_res;
    uint8_t reg_steps = 0;
    uint8_t fast_steps;

    NRF_MESH_ERROR_CHECK(access_model_publish_period_get(handle, &reg_res, &reg_steps));
    divide_publish_interval(reg_res, reg_steps, &fast_res, &fast_steps, m_model_pool[handle].publish_divisor);
    access_publish_period_set(&m_model_pool[handle].publication_state, fast_res, fast_steps);
}

static uint32_t access_metadata_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_ACCESS_METADATA_RECORD == id.record);

    access_flash_metadata_t * p_metadata = (access_flash_metadata_t *)p_entry;

    if (p_metadata->element_count == ACCESS_ELEMENT_COUNT &&
        p_metadata->model_count == ACCESS_MODEL_COUNT &&
        p_metadata->subscription_list_count == ACCESS_SUBSCRIPTION_LIST_COUNT)
    {
        m_status.is_metadata_stored = 1;
    }
    else
    {
        return NRF_ERROR_INVALID_DATA;
    }

    return NRF_SUCCESS;
}

static void access_metadata_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_ACCESS_METADATA_RECORD == id.record);

    access_flash_metadata_t * p_metadata = (access_flash_metadata_t *)p_entry;
    p_metadata->element_count = ACCESS_ELEMENT_COUNT;
    p_metadata->model_count = ACCESS_MODEL_COUNT;
    p_metadata->subscription_list_count = ACCESS_SUBSCRIPTION_LIST_COUNT;
}

static uint32_t subscriptions_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD,
                                      MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD + ACCESS_SUBSCRIPTION_LIST_COUNT - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!m_status.is_restoring_ended)
    {
        access_flash_subscription_list_t * p_data = (access_flash_subscription_list_t *)p_entry;
        uint16_t idx = id.record - MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD;

        for (uint32_t i = 0; i < ARRAY_SIZE(p_data->inverted_bitfield); ++i)
        {
            m_subscription_list_pool[idx].bitfield[i] = ~p_data->inverted_bitfield[i];
        }

        ACCESS_INTERNAL_STATE_INVALIDATED_CLR(m_subscription_list_pool[idx].internal_state);
        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_subscription_list_pool[idx].internal_state);
    }

    return NRF_SUCCESS;
}

static void subscriptions_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD,
                                      MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD + ACCESS_SUBSCRIPTION_LIST_COUNT - 1));

    uint16_t idx = id.record - MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD;
    access_flash_subscription_list_t * p_data = (access_flash_subscription_list_t *)p_entry;

    for (uint32_t i = 0; i < ARRAY_SIZE(p_data->inverted_bitfield); ++i)
    {
        p_data->inverted_bitfield[i] = ~m_subscription_list_pool[idx].bitfield[i];
    }
}

static uint32_t elements_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_ACCESS_ELEMENTS_RECORD,
                                      MESH_OPT_ACCESS_ELEMENTS_RECORD + ACCESS_ELEMENT_COUNT - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!m_status.is_restoring_ended)
    {
        uint16_t * p_data = (uint16_t *)p_entry;
        uint16_t idx = id.record - MESH_OPT_ACCESS_ELEMENTS_RECORD;

        if (*p_data != m_element_pool[idx].location)
        {
            return NRF_ERROR_INVALID_DATA;
        }

        ACCESS_INTERNAL_STATE_INVALIDATED_CLR(m_element_pool[idx].internal_state);
        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_element_pool[idx].internal_state);
    }

    return NRF_SUCCESS;
}

static void elements_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_ACCESS_ELEMENTS_RECORD,
                                      MESH_OPT_ACCESS_ELEMENTS_RECORD + ACCESS_ELEMENT_COUNT - 1));

    uint16_t idx = id.record - MESH_OPT_ACCESS_ELEMENTS_RECORD;
    uint16_t * p_data = (uint16_t *)p_entry;
    memcpy(p_data, &m_element_pool[idx].location, sizeof(uint16_t));
}

static uint32_t models_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_ACCESS_MODELS_RECORD,
                                      MESH_OPT_ACCESS_MODELS_RECORD + ACCESS_MODEL_COUNT - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!m_status.is_restoring_ended)
    {
        access_model_state_data_t * p_data = (access_model_state_data_t *)p_entry;
        uint16_t idx = id.record - MESH_OPT_ACCESS_MODELS_RECORD;

        if (p_data->element_index != m_model_pool[idx].model_info.element_index ||
            p_data->model_id.company_id != m_model_pool[idx].model_info.model_id.company_id ||
            p_data->model_id.model_id != m_model_pool[idx].model_info.model_id.model_id ||
            p_data->subscription_pool_index != m_model_pool[idx].model_info.subscription_pool_index)
        {
            return NRF_ERROR_INVALID_DATA;
        }

        memcpy(&m_model_pool[idx].model_info, p_data, sizeof(access_model_state_data_t));

        ACCESS_INTERNAL_STATE_INVALIDATED_CLR(m_model_pool[idx].internal_state);
        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_model_pool[idx].internal_state);
    }

    return NRF_SUCCESS;
}

static void models_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_ACCESS_MODELS_RECORD,
                                      MESH_OPT_ACCESS_MODELS_RECORD + ACCESS_MODEL_COUNT - 1));

    uint16_t idx = id.record - MESH_OPT_ACCESS_MODELS_RECORD;
    access_model_state_data_t * p_data = (access_model_state_data_t *)p_entry;
    memcpy(p_data, &m_model_pool[idx].model_info, sizeof(access_model_state_data_t));
}

static uint32_t default_ttl_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (id.record != MESH_OPT_ACCESS_DEFAULT_TTL_RECORD)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    m_default_ttl = *((uint8_t *) p_entry);

    return NRF_SUCCESS;
}

static void default_ttl_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(id.record == MESH_OPT_ACCESS_DEFAULT_TTL_RECORD);

    *((uint8_t *) p_entry) = m_default_ttl;
}

MESH_CONFIG_ENTRY(access_metadata,
                  MESH_OPT_ACCESS_METADATA_EID,
                  1,
                  sizeof(access_flash_metadata_t),
                  access_metadata_setter,
                  access_metadata_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(subscriptions,
                  MESH_OPT_ACCESS_SUBSCRIPTIONS_EID,
                  ACCESS_SUBSCRIPTION_LIST_COUNT,
                  sizeof(access_flash_subscription_list_t),
                  subscriptions_setter,
                  subscriptions_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(elements,
                  MESH_OPT_ACCESS_ELEMENTS_EID,
                  ACCESS_ELEMENT_COUNT,
                  sizeof(uint16_t),
                  elements_setter,
                  elements_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(models,
                  MESH_OPT_ACCESS_MODELS_EID,
                  ACCESS_MODEL_COUNT,
                  sizeof(access_model_state_data_t),
                  models_setter,
                  models_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(default_ttl,
                  MESH_OPT_ACCESS_DEFAULT_TTL_EID,
                  1,
                  sizeof(uint8_t),
                  default_ttl_setter,
                  default_ttl_getter,
                  NULL,
                  true);

MESH_CONFIG_FILE(m_access_file, MESH_OPT_ACCESS_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

static void model_store(access_model_handle_t handle)
{
    if (!m_status.is_restoring_ended)
    {
        ACCESS_INTERNAL_STATE_REFRESHED_SET(m_model_pool[handle].internal_state);
        return;
    }

    mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_MODELS_EID;

    entry_id.record += handle;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &m_model_pool[handle].model_info));
}

static void element_store(uint16_t index)
{
    if (!m_status.is_restoring_ended)
    {
        ACCESS_INTERNAL_STATE_REFRESHED_SET(m_element_pool[index].internal_state);
        return;
    }

    mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_ELEMENTS_EID;

    entry_id.record += index;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &m_element_pool[index].location));
}

static void sublist_store(uint16_t index)
{
    if (!m_status.is_restoring_ended)
    {
        ACCESS_INTERNAL_STATE_REFRESHED_SET(m_subscription_list_pool[index].internal_state);
        return;
    }

    access_flash_subscription_list_t subs_list;
    mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_SUBSCRIPTIONS_EID;

    entry_id.record += index;

    for (uint32_t i = 0; i < ARRAY_SIZE(subs_list.inverted_bitfield); ++i)
    {
        subs_list.inverted_bitfield[i] = ~m_subscription_list_pool[index].bitfield[i];
    }

    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &subs_list));
}

static void sublist_invalidate(uint16_t index)
{
    if (!m_status.is_restoring_ended)
    {
        ACCESS_INTERNAL_STATE_INVALIDATED_SET(m_subscription_list_pool[index].internal_state);
        return;
    }

    mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_SUBSCRIPTIONS_EID;
    entry_id.record += index;
    /* Due to the complex relationship between models, it is possible to allocate a subscription
     * list for the generic models but then to deallocate it and to start using the shared
     * subscription list. It causes the situation we have to invalidate entry that does not exist.
     * Mesh config handles that correctly. However, we do not need to check the returned status
     * anymore. */
    (void) mesh_config_entry_delete(entry_id);
}

static void metadata_store(void)
{
    mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_METADATA_EID;
    access_flash_metadata_t metadata =
    {
        .element_count = ACCESS_ELEMENT_COUNT,
        .model_count = ACCESS_MODEL_COUNT,
        .subscription_list_count = ACCESS_SUBSCRIPTION_LIST_COUNT
    };

    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &metadata));
}

static uint32_t restore_addresses_for_model(const access_common_t * p_model)
{
    uint32_t status;

    if (p_model->model_info.publish_address_handle != DSM_HANDLE_INVALID)
    {
        status = dsm_address_publish_add_handle(p_model->model_info.publish_address_handle);
        if (status != NRF_SUCCESS)
        {
            return status;
        }
    }

    if (p_model->model_info.subscription_pool_index != ACCESS_SUBSCRIPTION_LIST_COUNT)
    {
        const access_subscription_list_t * p_sub = &m_subscription_list_pool[p_model->model_info.subscription_pool_index];
        for (uint32_t i = bitfield_next_get(p_sub->bitfield, DSM_ADDR_MAX, 0);
             i != DSM_ADDR_MAX;
             i = bitfield_next_get(p_sub->bitfield, DSM_ADDR_MAX, i+1))
        {
            status = dsm_address_subscription_add_handle(i);
            if (status != NRF_SUCCESS)
            {
                return status;
            }
        }
    }

    return NRF_SUCCESS;
}

#if ACCESS_MODEL_PUBLISH_PERIOD_RESTORE
static void restore_publication_period(access_common_t * p_model)
{
    if (p_model->model_info.publication_period.step_num != 0)
    {
        access_publish_period_set(&p_model->publication_state,
                                  (access_publish_resolution_t) p_model->model_info.publication_period.step_res,
                                  p_model->model_info.publication_period.step_num);
    }
}
#endif

static void initialization_data_store(void)
{
    for (uint16_t i = 0; i < ACCESS_SUBSCRIPTION_LIST_COUNT; i++)
    {
        if (ACCESS_INTERNAL_STATE_IS_INVALIDATED(m_subscription_list_pool[i].internal_state))
        {
            sublist_invalidate(i);
        }
        else if (ACCESS_INTERNAL_STATE_IS_REFRESHED(m_subscription_list_pool[i].internal_state))
        {
            sublist_store(i);
        }

        ACCESS_INTERNAL_STATE_INVALIDATED_CLR(m_subscription_list_pool[i].internal_state);
        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_subscription_list_pool[i].internal_state);
    }

    for (uint16_t i = 0; i < ACCESS_ELEMENT_COUNT; i++)
    {
        if (ACCESS_INTERNAL_STATE_IS_REFRESHED(m_element_pool[i].internal_state))
        {
            element_store(i);
        }

        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_element_pool[i].internal_state);
    }

    for (uint16_t i = 0; i < ACCESS_MODEL_COUNT; i++)
    {
        if (ACCESS_INTERNAL_STATE_IS_REFRESHED(m_model_pool[i].internal_state))
        {
            model_store(i);
        }

        ACCESS_INTERNAL_STATE_REFRESHED_CLR(m_model_pool[i].internal_state);
    }
}

/* ********** Public API ********** */

static void access_mesh_config_clear(void)
{
    m_status.is_metadata_stored = 0;
    m_status.is_load_failed = 0;

    mesh_config_file_clear(MESH_OPT_ACCESS_FILE_ID);
}

/* If the persistent feature is switched off then metadata is never restored.
 * Branch for model restoring will be bypassed. */
uint32_t access_load_config_apply(void)
{
    m_status.is_restoring_ended = 1;

    if (!!m_status.is_load_failed)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if (!!m_status.is_metadata_stored)
    {
        for (access_model_handle_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
        {
            if (m_model_pool[i].model_info.element_index != ACCESS_ELEMENT_INDEX_INVALID)
            {
                if (restore_addresses_for_model(&m_model_pool[i]) != NRF_SUCCESS)
                {
                    return NRF_ERROR_INVALID_DATA;
                }

    #if ACCESS_MODEL_PUBLISH_PERIOD_RESTORE
                restore_publication_period(&m_model_pool[i]);
    #else
                m_model_pool[i].model_info.publication_period.step_res = 0;
                m_model_pool[i].model_info.publication_period.step_num = 0;
    #endif
            }
        }
    }

    initialization_data_store();

    if (!m_status.is_metadata_stored)
    {
        metadata_store();
#if PERSISTENT_STORAGE
        return NRF_ERROR_NOT_FOUND;
#else
        return NRF_SUCCESS;
#endif
    }

    return NRF_SUCCESS;
}

/* ********** Private API ********** */
void access_incoming_handle(const access_message_rx_t * p_message)
{
    const nrf_mesh_address_t * p_dst = &p_message->meta_data.dst;

    if (nrf_mesh_is_address_rx(p_dst))
    {
        uint16_t element_index;
        dsm_handle_t address_handle = DSM_HANDLE_INVALID;
        bool is_element_message = is_element_rx_address(p_dst, &element_index);

        if (!is_element_message)
        {
            /* If it's not one of the element addresses, it has to be a subscription address. */
            NRF_MESH_ERROR_CHECK(dsm_address_handle_get(p_dst, &address_handle));
        }

        for (int i = 0; i < ACCESS_MODEL_COUNT; ++i)
        {
            access_common_t * p_model = &m_model_pool[i];
            uint32_t opcode_index;

            bool address_match =
                (is_element_message ? (p_model->model_info.element_index == element_index)
                                    : (model_subscribes_to_addr(p_model, address_handle)));

            bool model_allocated = ACCESS_INTERNAL_STATE_IS_ALLOCATED(p_model->internal_state);
            bool appkey_bound = bitfield_get(p_model->model_info.application_keys_bitfield, p_message->meta_data.appkey_handle);
            bool valid_opcode = is_opcode_of_model(p_model, p_message->opcode, &opcode_index);

            __LOG(LOG_SRC_ACCESS, LOG_LEVEL_DBG3, "cmp_id: 0x%04x mdl_id: 0x%04x  alloc? %d  addr_match? %d  key_bound? %d  opcode? %d\n",
                  p_model->model_info.model_id.company_id, p_model->model_info.model_id.model_id,
                  model_allocated, address_match, appkey_bound, valid_opcode);

            if (model_allocated && address_match && appkey_bound && valid_opcode)
            {
                if (p_dst->type == NRF_MESH_ADDRESS_TYPE_UNICAST)
                {
                    access_reliable_message_rx_cb(i, p_message, p_model->p_args);
                }
                p_model->p_opcode_handlers[opcode_index].handler(i, p_message, p_model->p_args);
            }
        }
    }
}

uint32_t access_packet_tx(access_model_handle_t handle,
                          const access_message_tx_t * p_tx_message,
                          const uint8_t *p_access_payload,
                          uint16_t access_payload_len)
{
    uint32_t status;
    if (NULL == p_tx_message || NULL == p_access_payload || 0 == access_payload_len)
    {
        return NRF_ERROR_NULL;
    }
    else if (!check_tx_params(handle, p_tx_message, NULL, &status))
    {
        return status;
    }

    return packet_tx(handle, p_tx_message, NULL, p_access_payload,
                     access_payload_len);
}

/* ********** Public API ********** */
void access_clear(void)
{
    access_reliable_cancel_all();
    access_publish_clear();
    access_state_clear();
    access_mesh_config_clear();
}

void access_init(void)
{
    access_state_clear();

    m_evt_handler.evt_cb = mesh_evt_cb;
    nrf_mesh_evt_handler_add(&m_evt_handler);
    access_reliable_init();
    access_publish_init();
    access_publish_retransmission_init();

    m_status.is_metadata_stored = 0;
    m_status.is_load_failed = 0;
    m_status.is_restoring_ended = 0;
}

uint32_t access_model_add(const access_model_add_params_t * p_model_params,
                          access_model_handle_t * p_model_handle)
{
    if (NULL == p_model_params ||
        NULL == p_model_handle ||
        (0 != p_model_params->opcode_count && NULL == p_model_params->p_opcode_handlers))
    {
        return NRF_ERROR_NULL;
    }
    *p_model_handle = ACCESS_HANDLE_INVALID;

    if (0 == p_model_params->opcode_count && NULL != p_model_params->p_opcode_handlers)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    else if (p_model_params->element_index >= ACCESS_ELEMENT_COUNT)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (nrf_mesh_is_device_provisioned() ||
             (element_has_model_id(p_model_params->element_index, p_model_params->model_id, p_model_handle) &&
             ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[*p_model_handle].internal_state)))
    {
        return NRF_ERROR_FORBIDDEN;
    }
    else if (!opcodes_are_valid(p_model_params->p_opcode_handlers, p_model_params->opcode_count))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (*p_model_handle == ACCESS_HANDLE_INVALID) /* The model was not recovered from the flash */
    {
        *p_model_handle = find_available_model();
        if (ACCESS_HANDLE_INVALID == *p_model_handle)
        {
            return NRF_ERROR_NO_MEM;
        }

        m_model_pool[*p_model_handle].model_info.publish_address_handle = DSM_HANDLE_INVALID;
        m_model_pool[*p_model_handle].model_info.publish_appkey_handle = DSM_HANDLE_INVALID;
        m_model_pool[*p_model_handle].model_info.element_index = p_model_params->element_index;
        m_model_pool[*p_model_handle].model_info.model_id.model_id = p_model_params->model_id.model_id;
        m_model_pool[*p_model_handle].model_info.model_id.company_id = p_model_params->model_id.company_id;
        m_model_pool[*p_model_handle].model_info.publish_ttl = ACCESS_TTL_USE_DEFAULT;
        increment_model_count(p_model_params->element_index, p_model_params->model_id.company_id);
        ACCESS_INTERNAL_STATE_ALLOCATED_SET(m_model_pool[*p_model_handle].internal_state);
        model_store(*p_model_handle);
    }

    m_model_pool[*p_model_handle].p_args = p_model_params->p_args;
    m_model_pool[*p_model_handle].p_opcode_handlers = p_model_params->p_opcode_handlers;
    m_model_pool[*p_model_handle].opcode_count = p_model_params->opcode_count;

    m_model_pool[*p_model_handle].publication_state.publish_timeout_cb = p_model_params->publish_timeout_cb;
    m_model_pool[*p_model_handle].publication_state.model_handle = *p_model_handle;

    return NRF_SUCCESS;
}

uint32_t access_model_publish(access_model_handle_t handle, const access_message_tx_t * p_message)
{
    uint32_t status;

    if (p_message == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint16_t payload_length = 0;
    uint8_t *p_payload;

    status = packet_alloc_and_tx(handle, p_message, NULL, &p_payload, &payload_length);
    if (NRF_SUCCESS != status)
    {
        return status;
    }

    if (m_model_pool[handle].model_info.publication_retransmit.count > 0)
    {
        access_publish_retransmission_message_add(handle,
                                                  &m_model_pool[handle].model_info.publication_retransmit,
                                                  p_message,
                                                  p_payload,
                                                  payload_length);
    }
    else
    {
        mesh_mem_free(p_payload);
    }
    return NRF_SUCCESS;
}

uint32_t access_model_reply(access_model_handle_t handle,
                            const access_message_rx_t * p_message,
                            const access_message_tx_t * p_reply)
{
    if (p_message == NULL || p_reply == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return packet_alloc_and_tx(handle, p_reply, p_message, NULL, NULL);
}

uint32_t access_model_element_index_get(access_model_handle_t handle, uint16_t * p_element_index)
{
    if (p_element_index == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_element_index = m_model_pool[handle].model_info.element_index;
        return NRF_SUCCESS;
    }
}

/* ****** Internal API ****** */
uint32_t access_model_publish_address_set(access_model_handle_t handle, dsm_handle_t address_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (DSM_ADDR_MAX <= address_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (m_model_pool[handle].model_info.publish_address_handle == address_handle)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.publish_address_handle = address_handle;
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_address_get(access_model_handle_t handle, dsm_handle_t * p_address_handle)
{
    if (NULL == p_address_handle)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_address_handle = m_model_pool[handle].model_info.publish_address_handle;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_retransmit_set(access_model_handle_t handle,
                                             access_publish_retransmit_t retransmit_params)
{
    if (ACCESS_MODEL_COUNT <= handle ||
        !ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[handle].internal_state))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_model_pool[handle].model_info.publication_retransmit.count == retransmit_params.count &&
             m_model_pool[handle].model_info.publication_retransmit.interval_steps == retransmit_params.interval_steps)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.publication_retransmit = retransmit_params;
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_retransmit_get(access_model_handle_t handle,
                                             access_publish_retransmit_t * p_retransmit_params)
{
    if (p_retransmit_params == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_MODEL_COUNT <= handle ||
        !ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[handle].internal_state))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_retransmit_params = m_model_pool[handle].model_info.publication_retransmit;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_period_divisor_set(access_model_handle_t handle, uint16_t publish_divisor)
{

    if (ACCESS_MODEL_COUNT <= handle ||
        !ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[handle].internal_state))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_model_pool[handle].publication_state.publish_timeout_cb == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else if (publish_divisor == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_model_pool[handle].publish_divisor = publish_divisor;
    access_publish_timing_update(handle);

    return NRF_SUCCESS;
}

uint32_t access_model_publish_period_set(access_model_handle_t handle,
                                         access_publish_resolution_t resolution,
                                         uint8_t step_number)
{
    if (ACCESS_MODEL_COUNT <= handle ||
        !ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[handle].internal_state))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_model_pool[handle].publication_state.publish_timeout_cb == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else if (step_number > ACCESS_PUBLISH_PERIOD_STEP_MAX ||
             resolution > ACCESS_PUBLISH_RESOLUTION_MAX) /*lint !e685 */
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (m_model_pool[handle].model_info.publication_period.step_num == step_number &&
             m_model_pool[handle].model_info.publication_period.step_res == resolution)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.publication_period.step_num = step_number;
        m_model_pool[handle].model_info.publication_period.step_res = resolution;
        access_publish_timing_update(handle);
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_period_get(access_model_handle_t handle,
                                         access_publish_resolution_t * p_resolution,
                                         uint8_t * p_step_number)
{
    if (NULL == p_resolution || NULL == p_step_number)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_model_pool[handle].publication_state.publish_timeout_cb == NULL)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else
    {
        *p_resolution = (access_publish_resolution_t) m_model_pool[handle].model_info.publication_period.step_res;
        *p_step_number = m_model_pool[handle].model_info.publication_period.step_num;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_subscription_list_alloc(access_model_handle_t handle)
{
    if (nrf_mesh_is_device_provisioned())
    {
        return NRF_ERROR_FORBIDDEN;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (ACCESS_SUBSCRIPTION_LIST_COUNT != m_model_pool[handle].model_info.subscription_pool_index)
    {
        return NRF_SUCCESS;
    }
    else
    {
        for (uint32_t i = 0; i < ACCESS_SUBSCRIPTION_LIST_COUNT; ++i)
        {
            if (!ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_subscription_list_pool[i].internal_state))
            {
                ACCESS_INTERNAL_STATE_INVALIDATED_CLR(m_subscription_list_pool[i].internal_state);
                ACCESS_INTERNAL_STATE_ALLOCATED_SET(m_subscription_list_pool[i].internal_state);
                m_model_pool[handle].model_info.subscription_pool_index = i;
                sublist_store(i);
                model_store(handle);
                return NRF_SUCCESS;
            }
        }
        return NRF_ERROR_NO_MEM;
    }
}

uint32_t access_model_subscription_list_dealloc(access_model_handle_t handle)
{
    uint16_t sub_pool_index;

    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    sub_pool_index = m_model_pool[handle].model_info.subscription_pool_index;
    if (ACCESS_SUBSCRIPTION_LIST_COUNT == sub_pool_index)
    {
        return NRF_SUCCESS;
    }
    else if (nrf_mesh_is_device_provisioned())
    {
        return NRF_ERROR_FORBIDDEN;
    }

    if (ACCESS_SUBSCRIPTION_LIST_COUNT > sub_pool_index)
    {
        if (!model_subscription_list_is_shared(handle))
        {
            memset(&m_subscription_list_pool[sub_pool_index], 0, sizeof(m_subscription_list_pool[0]));
            sublist_invalidate(sub_pool_index);
        }

        m_model_pool[handle].model_info.subscription_pool_index = ACCESS_SUBSCRIPTION_LIST_COUNT;
        model_store(handle);

        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_STATE;
}

uint32_t access_model_subscription_lists_share(access_model_handle_t owner, access_model_handle_t other)
{
    if (nrf_mesh_is_device_provisioned())
    {
        return NRF_ERROR_FORBIDDEN;
    }
    else if (!model_handle_valid_and_allocated(owner) || !model_handle_valid_and_allocated(other))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (ACCESS_SUBSCRIPTION_LIST_COUNT > m_model_pool[owner].model_info.subscription_pool_index)
    {

        if (m_model_pool[other].model_info.subscription_pool_index == m_model_pool[owner].model_info.subscription_pool_index)
        {
            return NRF_SUCCESS;
        }

        if (ACCESS_SUBSCRIPTION_LIST_COUNT >= m_model_pool[other].model_info.subscription_pool_index)
        {
            uint32_t status = access_model_subscription_list_dealloc(other);
            if (status != NRF_SUCCESS)
            {
                return status;
            }

            m_model_pool[other].model_info.subscription_pool_index = m_model_pool[owner].model_info.subscription_pool_index;
            model_store(other);
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_INVALID_STATE;
}

uint32_t access_model_subscription_add(access_model_handle_t handle, dsm_handle_t address_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (ACCESS_SUBSCRIPTION_LIST_COUNT == m_model_pool[handle].model_info.subscription_pool_index)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else if (DSM_ADDR_MAX <= address_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (bitfield_get(m_subscription_list_pool[m_model_pool[handle].model_info.subscription_pool_index].bitfield, address_handle))
    {
        return NRF_SUCCESS;
    }
    else
    {
        bitfield_set(m_subscription_list_pool[m_model_pool[handle].model_info.subscription_pool_index].bitfield, address_handle);
        sublist_store(m_model_pool[handle].model_info.subscription_pool_index);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_subscription_remove(access_model_handle_t handle, dsm_handle_t address_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (ACCESS_SUBSCRIPTION_LIST_COUNT == m_model_pool[handle].model_info.subscription_pool_index)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else if (DSM_ADDR_MAX <= address_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (!bitfield_get(m_subscription_list_pool[m_model_pool[handle].model_info.subscription_pool_index].bitfield, address_handle))
    {
        return NRF_SUCCESS;
    }
    else
    {
        bitfield_clear(m_subscription_list_pool[m_model_pool[handle].model_info.subscription_pool_index].bitfield, address_handle);
        sublist_store(m_model_pool[handle].model_info.subscription_pool_index);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_subscriptions_get(access_model_handle_t handle,
                                        dsm_handle_t * p_address_handles,
                                        uint16_t * p_count)
{
    if (NULL == p_address_handles || NULL == p_count)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (ACCESS_SUBSCRIPTION_LIST_COUNT == m_model_pool[handle].model_info.subscription_pool_index)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    else
    {
        const uint32_t max_count = *p_count;
        *p_count = 0;
        for (uint32_t i = 0; i < DSM_ADDR_MAX; ++i)
        {
            if (model_subscribes_to_addr(&m_model_pool[handle], i))
            {
                if (*p_count >= max_count)
                {
                    return NRF_ERROR_INVALID_LENGTH;
                }
                p_address_handles[*p_count] = i;
                (*p_count)++;
            }
        }
        return NRF_SUCCESS;
    }
}

uint32_t access_model_application_bind(access_model_handle_t handle, dsm_handle_t appkey_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (DSM_APP_MAX + DSM_DEVICE_MAX <= appkey_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (bitfield_get(m_model_pool[handle].model_info.application_keys_bitfield, appkey_handle))
    {
        return NRF_SUCCESS;
    }
    else
    {
        bitfield_set(m_model_pool[handle].model_info.application_keys_bitfield, appkey_handle);
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_application_unbind(access_model_handle_t handle, dsm_handle_t appkey_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (DSM_APP_MAX + DSM_DEVICE_MAX <= appkey_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (!bitfield_get(m_model_pool[handle].model_info.application_keys_bitfield, appkey_handle))
    {
        return NRF_SUCCESS;
    }
    else
    {
        bitfield_clear(m_model_pool[handle].model_info.application_keys_bitfield, appkey_handle);
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_applications_get(access_model_handle_t handle,
                                       dsm_handle_t * p_appkey_handles,
                                       uint16_t * p_count)
{
    if (NULL == p_appkey_handles || NULL == p_count)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        const uint32_t max_count = *p_count;
        *p_count = 0;
        for (uint32_t i = 0; i < DSM_APP_MAX; ++i)
        {
            if (bitfield_get(m_model_pool[handle].model_info.application_keys_bitfield, i))
            {
                if (*p_count >= max_count)
                {
                    return NRF_ERROR_INVALID_LENGTH;
                }
                p_appkey_handles[*p_count] = i;
                (*p_count)++;
            }
        }
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_application_set(access_model_handle_t handle, dsm_handle_t appkey_handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (DSM_APP_MAX + DSM_DEVICE_MAX <= appkey_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (m_model_pool[handle].model_info.publish_appkey_handle == appkey_handle)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.publish_appkey_handle = appkey_handle;
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_application_get(access_model_handle_t handle,
                                              dsm_handle_t * p_appkey_handle)
{
    if (NULL == p_appkey_handle)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_appkey_handle = m_model_pool[handle].model_info.publish_appkey_handle;
        return NRF_SUCCESS;
    }
}

uint32_t access_default_ttl_set(uint8_t ttl)
{
    if ((ttl > NRF_MESH_TTL_MAX) || (ttl == 1))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        mesh_config_entry_id_t entry_id = MESH_OPT_ACCESS_DEFAULT_TTL_EID;

        return mesh_config_entry_set(entry_id, &ttl);
    }
}

uint8_t access_default_ttl_get(void)
{
    return m_default_ttl;
}

uint32_t access_model_publish_friendship_credential_flag_set(access_model_handle_t handle, bool flag)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_model_pool[handle].model_info.friendship_credential_flag == flag)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.friendship_credential_flag = flag;
        model_store(handle);
    }

    return NRF_SUCCESS;
}

uint32_t access_model_publish_friendship_credential_flag_get(access_model_handle_t handle, bool * p_flag)
{
    if (p_flag == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_flag = m_model_pool[handle].model_info.friendship_credential_flag;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_ttl_set(access_model_handle_t handle, uint8_t ttl)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (NRF_MESH_TTL_MAX < ttl && ttl != ACCESS_TTL_USE_DEFAULT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (m_model_pool[handle].model_info.publish_ttl == ttl)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_model_pool[handle].model_info.publish_ttl = ttl;
        model_store(handle);
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publish_ttl_get(access_model_handle_t handle, uint8_t * p_ttl)
{
    if (NULL == p_ttl)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_ttl = m_model_pool[handle].model_info.publish_ttl;
        return NRF_SUCCESS;
    }
}

uint32_t access_element_location_set(uint16_t element_index, uint16_t location)
{
    if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_element_pool[element_index].location == location)
    {
        return NRF_SUCCESS;
    }
    else
    {
        m_element_pool[element_index].location = location;
        ACCESS_INTERNAL_STATE_ALLOCATED_SET(m_element_pool[element_index].internal_state);
        element_store(element_index);
        return NRF_SUCCESS;
    }
}

uint32_t access_element_location_get(uint16_t element_index, uint16_t * p_location)
{
    if (NULL == p_location)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_location = m_element_pool[element_index].location;
        return NRF_SUCCESS;
    }
}

uint32_t access_element_sig_model_count_get(uint16_t element_index, uint8_t * p_sig_model_count)
{
    if (NULL == p_sig_model_count)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_sig_model_count = m_element_pool[element_index].sig_model_count;
        return NRF_SUCCESS;
    }
}

uint32_t access_element_vendor_model_count_get(uint16_t element_index, uint8_t * p_vendor_model_count)
{
    if (NULL == p_vendor_model_count)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_vendor_model_count = m_element_pool[element_index].vendor_model_count;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_id_get(access_model_handle_t handle, access_model_id_t * p_model_id)
{
    if (NULL == p_model_id)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        p_model_id->model_id = m_model_pool[handle].model_info.model_id.model_id;
        p_model_id->company_id = m_model_pool[handle].model_info.model_id.company_id;
        return NRF_SUCCESS;
    }
}

uint32_t access_handle_get(uint16_t element_index,
                           access_model_id_t model_id,
                           access_model_handle_t * p_handle)
{
    if (p_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_handle = ACCESS_HANDLE_INVALID;
        for (access_model_handle_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
        {
            if (m_model_pool[i].model_info.element_index == element_index &&
                m_model_pool[i].model_info.model_id.model_id == model_id.model_id &&
                m_model_pool[i].model_info.model_id.company_id == model_id.company_id)
            {
                *p_handle = i;
                return NRF_SUCCESS;
            }
        }
        /* Couldn't find the model. */
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t access_element_models_get(uint16_t element_index, access_model_handle_t * p_models, uint16_t * p_count)
{
    if (NULL == p_models || p_count == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (ACCESS_ELEMENT_COUNT <= element_index)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        const uint16_t max_count = *p_count;
        *p_count = 0;
        for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
        {
            if (*p_count >= max_count)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
            else if ((ACCESS_INTERNAL_STATE_IS_ALLOCATED(m_model_pool[i].internal_state)) &&
                     m_model_pool[i].model_info.element_index == element_index)
            {
                p_models[*p_count] = i;
                (*p_count)++;
            }
        }
        return NRF_SUCCESS;
    }
}

uint32_t access_model_p_args_get(access_model_handle_t handle, void ** pp_args)
{
    if (NULL == pp_args)
    {
        return NRF_ERROR_NULL;
    }
    else if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *pp_args = m_model_pool[handle].p_args;
        return NRF_SUCCESS;
    }
}

uint32_t access_model_publication_stop(access_model_handle_t handle)
{
    if (!model_handle_valid_and_allocated(handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    access_publish_period_set(&m_model_pool[handle].publication_state, (access_publish_resolution_t)0, 0);
    (void) access_model_reliable_cancel(handle);
    (void) dsm_address_publish_remove(m_model_pool[handle].model_info.publish_address_handle);
    m_model_pool[handle].model_info.publish_address_handle = DSM_HANDLE_INVALID;
    m_model_pool[handle].model_info.publish_appkey_handle = DSM_HANDLE_INVALID;
    m_model_pool[handle].model_info.publication_retransmit.count = 0;
    m_model_pool[handle].model_info.publication_retransmit.interval_steps = 0;
    m_model_pool[handle].model_info.publish_ttl = 0;
    m_model_pool[handle].model_info.publication_period.step_num = 0;
    m_model_pool[handle].model_info.publication_period.step_res = 0;
    m_model_pool[handle].publication_state.period.step_num = 0;
    m_model_pool[handle].publication_state.period.step_res = 0;
    model_store(handle);

    return NRF_SUCCESS;
}

uint32_t access_model_publication_by_appkey_stop(dsm_handle_t appkey_handle)
{
    dsm_handle_t local_handle;

    if (DSM_APP_MAX + DSM_DEVICE_MAX <= appkey_handle)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    for (access_model_handle_t i = 0; i < ACCESS_MODEL_COUNT; i++)
    {
        if (NRF_SUCCESS == access_model_publish_application_get(i, &local_handle))
        {
            if (local_handle == appkey_handle)
            {
                (void) access_model_publication_stop(i);
            }
        }
    }

    return NRF_SUCCESS;
}
