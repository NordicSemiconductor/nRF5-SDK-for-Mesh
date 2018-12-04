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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "utils.h"
#include "test_assert.h"

#include "log.h"
#include "fifo.h"

#include "access.h"
#include "access_internal.h"
#include "access_config.h"
#include "access_status.h"

#include "access_publish_mock.h"
#include "access_reliable_mock.h"

#include "device_state_manager_mock.h"
#include "flash_manager_mock.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_utils_mock.h"
#include "event_mock.h"
#include "bearer_event_mock.h"
#include "proxy_mock.h"
#include "manual_mock_queue.h"

#define TEST_REFERENCE ((void*) 0xB00BB00B)
#define TEST_MODEL_ID (0xB00B)

#define SOURCE_ADDRESS (0x0001)
#define ELEMENT_ADDRESS_START (0x0011)
#define GROUP_ADDRESS_START   (0xC001)
#define PUBLISH_ADDRESS_START (0x0101)
#define OPCODE_COUNT (5)
#define ACCESS_LOOPBACK_FLAG  (0x11223344ul)

#define MSG_EVT_MAX_COUNT (ACCESS_ELEMENT_COUNT)
#define TX_EVT_MAX_COUNT (ACCESS_ELEMENT_COUNT)

#define ALLOC_BUFFER_SIZE (380)

#define FLASH_TEST_VECTOR_INSTANCE(MODEL_ID, ELEMENT_INDEX, P_SUB_ADDRS, NO_SUB_ADDRS, SUB_SHARE_IDX,\
                                   PUB_HANDLE, PUB_PERIOD, P_APPKEYS, NO_APPKEYS, PUB_APPKEY, TTL, CRED) \
    {\
        .add_params = {\
                        .model_id = (MODEL_ID),\
                        .element_index = (ELEMENT_INDEX),\
                        .p_opcode_handlers =  &m_opcode_handlers[0][0],\
                        .opcode_count = (OPCODE_COUNT),\
                        .p_args = NULL,\
                        .publish_timeout_cb = publish_timeout_cb\
                      },\
        .p_subsciption_address_handles = (P_SUB_ADDRS),\
        .number_of_subscription_handles = (NO_SUB_ADDRS),\
        .subscription_list_share_index = (SUB_SHARE_IDX),\
        .publish_address_handle = (PUB_HANDLE),\
        .publish_period = (PUB_PERIOD),\
        .p_appkey_handles = (P_APPKEYS),\
        .number_of_appkey_handles = (NO_APPKEYS),\
        .publish_appkey_handle = (PUB_APPKEY),\
        .publish_ttl = (TTL), \
        .credential_flag = (CRED) \
    }

/*******************************************************************************
 * Sample data from the Mesh Profile Specification v1.0
 *******************************************************************************/

/* Message #21 (the Mesh Profile Specification v1.0, page 309) */
#define MSG_21_VENDOR_MODEL_OPCODE      (0x15 | 0xC0)
#define MSG_21_VENDOR_MODEL_COMPANY_ID  (0x000a)
#define MSG_21_ACCESS_PARAMS_RAW        {0x48, 0x65, 0x6c, 0x6c, 0x6f}
#define MSG_21_ACCESS_PAYLOAD_RAW       {0xd5, 0x0a, 0x00, 0x48, 0x65, 0x6c, 0x6c, 0x6f}

/*******************************************************************************
 * Structure definitions used for testing
 *******************************************************************************/

/* Struct used for the alloc and add calls, which always take place on bootup even before restoring */
typedef struct
{
    access_model_add_params_t add_params;
    dsm_handle_t * p_subsciption_address_handles;
    uint32_t number_of_subscription_handles;
    uint32_t subscription_list_share_index;
    dsm_handle_t publish_address_handle;
    access_publish_period_t publish_period;
    dsm_handle_t * p_appkey_handles;
    uint32_t number_of_appkey_handles;
    dsm_handle_t publish_appkey_handle;
    uint8_t publish_ttl;
    bool credential_flag;
} access_flash_test_struct_t;

typedef struct
{
    access_opcode_t opcode;
    uint32_t test_reference;    /* Should be unique for the (element, model) tuple. */
    const uint8_t * p_data;
    uint32_t  length;
} msg_evt_t;

typedef struct
{
    const uint8_t * p_data;
    uint32_t length;
    uint16_t src;
    uint16_t dst;
} tx_evt_t;

typedef struct
{
    fm_handle_filter_t filter;
    fm_entry_t ** pp_expected_entries;
    uint32_t entry_count;
} flash_manager_entries_read_expect_t;
MOCK_QUEUE_DEF(flash_manager_entries_read_mock, flash_manager_entries_read_expect_t, NULL);

typedef enum
{
    TX_SECMAT_TYPE_MASTER,
    TX_SECMAT_TYPE_FRIENDSHIP,
} tx_secmat_type_t;

/*******************************************************************************
 * Static Variables
 *******************************************************************************/
static access_opcode_handler_t m_opcode_handlers[ACCESS_MODEL_COUNT][OPCODE_COUNT];

static const nrf_mesh_evt_handler_t * mp_evt_handler;

static msg_evt_t m_msg_evt_buffer[MSG_EVT_MAX_COUNT];
static fifo_t m_msg_fifo;

static tx_evt_t m_tx_evt_buffer[TX_EVT_MAX_COUNT];
static fifo_t m_tx_fifo;

static nrf_mesh_address_t m_addresses[DSM_ADDR_MAX];

const uint32_t SUBSCRIPTION_ADDRESS_COUNT = DSM_ADDR_MAX - ACCESS_MODEL_COUNT - ACCESS_ELEMENT_COUNT;
dsm_local_unicast_address_t local_addresses = {ELEMENT_ADDRESS_START, ACCESS_ELEMENT_COUNT};

static flash_manager_t * mp_flash_manager;
static flash_manager_config_t m_flash_manager_config;
static fm_mem_listener_t * mp_mem_listener;
static uint32_t m_flash_manager_calls;
static uint32_t m_listener_register_calls;

static bearer_event_flag_callback_t m_bearer_cb;

static uint32_t m_dsm_tx_friendship_secmat_get_retval = NRF_SUCCESS;
static uint16_t m_sub_list_dealloc_index;

/*******************************************************************************
 * Helper Functions // Mocks // Callbacks
 *******************************************************************************/
static void print_configuration(void)
{
    printf("Device: \n");

    access_model_handle_t models[ACCESS_MODEL_COUNT];
    access_model_id_t model_id;
    uint16_t model_count;
    for (unsigned i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        printf("\tElement %u: %u\n", i, i + ELEMENT_ADDRESS_START);

        model_count = ACCESS_MODEL_COUNT;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(i, models, &model_count));
        for (unsigned j = 0; j < model_count; j++)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_id_get(models[j], &model_id));
            printf("\t\tModel %u:0x%04X:0x%04X \n", j, model_id.model_id, model_id.company_id);

            printf("\t\t\tOpcodes: {");
            for (unsigned k = 0; k < OPCODE_COUNT - 1; ++k)
            {
                printf(" 0x%04x,", m_opcode_handlers[models[j]][k].opcode.opcode);
            }
            printf(" 0x%04x }\n", m_opcode_handlers[models[j]][OPCODE_COUNT-1].opcode.opcode);
        }
    }
}

timestamp_t timer_now(void)
{
    return 0;
}

static access_model_handle_t init_test_model_and_subs_list(access_flash_test_struct_t * p_test_data)
{
    access_model_handle_t model_handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&p_test_data->add_params, &model_handle));

    if (p_test_data->subscription_list_share_index == UINT32_MAX)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(model_handle));
    }
    else
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(p_test_data->subscription_list_share_index, model_handle));
    }
    return model_handle;
}

static void update_test_model(access_flash_test_struct_t * p_test_data, access_model_handle_t model_handle, bool expect_publish_period_set)
{
    for (uint32_t subs = 0; subs < p_test_data->number_of_subscription_handles; ++subs)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(model_handle, p_test_data->p_subsciption_address_handles[subs]));
    }
    for (uint32_t apps = 0; apps < p_test_data->number_of_appkey_handles; ++apps)
    {
         TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(model_handle, p_test_data->p_appkey_handles[apps]));
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(model_handle, p_test_data->publish_address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_set(model_handle, p_test_data->credential_flag));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(model_handle, p_test_data->publish_ttl));
    if (expect_publish_period_set)
    {
        access_publish_period_set_Expect(NULL, (access_publish_resolution_t) p_test_data->publish_period.step_res, p_test_data->publish_period.step_num);
        access_publish_period_set_IgnoreArg_p_pubstate();
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(model_handle, (access_publish_resolution_t) p_test_data->publish_period.step_res,
        p_test_data->publish_period.step_num));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_set(model_handle, p_test_data->publish_appkey_handle));
}

#define VAL_IN_ARRAY(VAL, ARR, ARR_SIZE, FOUND) \
                                        do {\
                                            FOUND = false;\
                                            for (uint32_t __IDX = 0; __IDX < ARR_SIZE; ++__IDX)\
                                            {\
                                                if (ARR[__IDX] == VAL)\
                                                {\
                                                    FOUND = true;\
                                                    break;\
                                                }\
                                            }\
                                        } while (0)

static void verify_test_case_and_access_state(access_flash_test_struct_t * p_test_input, access_model_handle_t model_handle)
{
    access_model_id_t model_id;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_id_get(model_handle, &model_id));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&p_test_input->add_params.model_id, &model_id, sizeof(model_id));
    dsm_handle_t dsm_handle;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_get(model_handle, &dsm_handle));
    TEST_ASSERT_EQUAL(p_test_input->publish_appkey_handle, dsm_handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_get(model_handle, &dsm_handle));
    TEST_ASSERT_EQUAL(p_test_input->publish_address_handle, dsm_handle);
    dsm_handle_t appkey_handles[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_applications_get(model_handle, appkey_handles, &appkey_count));
    for (uint32_t appkey_idx = 0; appkey_idx < p_test_input->number_of_appkey_handles; ++appkey_idx)
    {
        TEST_ASSERT_EQUAL(p_test_input->p_appkey_handles[appkey_idx], appkey_handles[appkey_idx]);
    }
    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    uint16_t models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(p_test_input->add_params.element_index, model_handles, &models_count));
    bool model_exists = false;
    VAL_IN_ARRAY(model_handle, model_handles, models_count, model_exists);
    TEST_ASSERT_TRUE(model_exists);
    dsm_handle_t subs_handles[DSM_ADDR_MAX];
    uint16_t subs_count = DSM_ADDR_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscriptions_get(model_handle, subs_handles, &subs_count));
    for (uint32_t i = 0; i < p_test_input->number_of_subscription_handles; ++i)
    {
        bool addr_exists;
        VAL_IN_ARRAY(p_test_input->p_subsciption_address_handles[i], subs_handles, subs_count, addr_exists);
        TEST_ASSERT_TRUE(addr_exists);
    }
    uint8_t ttl;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_get(model_handle, &ttl));
    TEST_ASSERT_EQUAL(p_test_input->publish_ttl, ttl);
    access_publish_resolution_t resolution = (access_publish_resolution_t) p_test_input->publish_period.step_res;
    uint8_t step_number = p_test_input->publish_period.step_num;
    access_publish_period_get_Expect(NULL, &resolution, &step_number);
    access_publish_period_get_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_get(model_handle, &resolution, &step_number));
    TEST_ASSERT_EQUAL(p_test_input->publish_period.step_res, resolution);
    TEST_ASSERT_EQUAL(p_test_input->publish_period.step_num, step_number);
}

static void verify_flash_modeldata(access_flash_test_struct_t * p_test_input, access_model_state_data_t * p_model_data, uint16_t subs_pool_index)
{
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&p_test_input->add_params.model_id, &p_model_data->model_id, sizeof(p_model_data->model_id));
    TEST_ASSERT_EQUAL(p_test_input->publish_appkey_handle, p_model_data->publish_appkey_handle);
    TEST_ASSERT_EQUAL(p_test_input->publish_address_handle, p_model_data->publish_address_handle);
    for (uint32_t appkey_idx = 0; appkey_idx < p_test_input->number_of_appkey_handles; ++appkey_idx)
    {
        TEST_ASSERT_TRUE(bitfield_get(p_model_data->application_keys_bitfield, p_test_input->p_appkey_handles[appkey_idx]));
    }
    TEST_ASSERT_EQUAL(p_test_input->add_params.element_index, p_model_data->element_index);
    TEST_ASSERT_EQUAL(subs_pool_index, p_model_data->subscription_pool_index);
    TEST_ASSERT_EQUAL(p_test_input->publish_ttl, p_model_data->publish_ttl);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&p_test_input->publish_period, &p_model_data->publication_period, sizeof(p_test_input->publish_period));
}


static fm_entry_t * expect_flash_manager_entry(fm_handle_t handle, uint32_t data_length)
{
    static uint8_t buffer[UINT16_MAX+1];
    static uint16_t buffer_index = 0;
    fm_entry_t * p_entry = (fm_entry_t *) &buffer[buffer_index];
    buffer_index += ALIGN_VAL((sizeof(fm_entry_t) + data_length), 4);
    flash_manager_entry_alloc_ExpectAndReturn(mp_flash_manager, handle, data_length, p_entry);
    flash_manager_entry_commit_Expect(p_entry);
    p_entry->header.handle = handle;
    return p_entry;
}


static uint32_t flash_manager_entries_read_mock_cb(const flash_manager_t * p_manager,
                                                   const fm_handle_filter_t * p_filter,
                                                   flash_manager_read_cb_t read_cb,
                                                   void * p_args,
                                                   int calls)
{
    flash_manager_entries_read_expect_t expect;
    flash_manager_entries_read_mock_Consume(&expect);

    TEST_ASSERT_EQUAL(expect.filter.mask, p_filter->mask);
    TEST_ASSERT_EQUAL(expect.filter.match, p_filter->match);
    for (uint32_t i = 0; i < expect.entry_count; ++i)
    {
        TEST_ASSERT_EQUAL(FM_ITERATE_ACTION_CONTINUE,
                          read_cb(expect.pp_expected_entries[i], p_args));
    }
    return expect.entry_count;
}

static void restore_flash(fm_handle_filter_t * p_filter, fm_entry_t ** pp_expected_entries, uint32_t entry_count)
{
    flash_manager_entries_read_StubWithCallback(flash_manager_entries_read_mock_cb);
    flash_manager_entries_read_expect_t expect = {
        .filter = *p_filter,
        .pp_expected_entries = pp_expected_entries,
        .entry_count = entry_count,
    };
    flash_manager_entries_read_mock_Expect(&expect);
}

static void evt_handler_add_stub(nrf_mesh_evt_handler_t * p_evt_handler, int num_calls)
{
    mp_evt_handler = p_evt_handler;
}

static const void * dsm_flash_area_get_stub(int num_calls)
{
    return (void*) PAGE_SIZE;
}

static void flash_manager_mem_listener_register_stub(fm_mem_listener_t * p_listener, int num_calls)
{
    m_listener_register_calls++;
    mp_mem_listener = p_listener;
}

static uint32_t flash_manager_add_stub(flash_manager_t * p_manager, const flash_manager_config_t * p_config, int num_calls)
{
    m_flash_manager_calls++;
    mp_flash_manager = p_manager;
    memcpy(&m_flash_manager_config, p_config, sizeof(flash_manager_config_t));
    return NRF_SUCCESS;

}

bearer_event_flag_t bearer_event_flag_add_cb(bearer_event_flag_callback_t cb, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_NOT_NULL(cb);
    m_bearer_cb = cb;

    return ACCESS_LOOPBACK_FLAG;
}

static uint32_t packet_tx_stub(const nrf_mesh_tx_params_t * p_tx_params, uint32_t * const p_ref, int num_calls)
{
    tx_evt_t tx_evt;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&m_tx_fifo, &tx_evt));
    TEST_ASSERT_EQUAL(tx_evt.length, p_tx_params->data_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tx_evt.p_data, p_tx_params->p_data, tx_evt.length);
    TEST_ASSERT_EQUAL_HEX16(tx_evt.src, p_tx_params->src);
    TEST_ASSERT_EQUAL_HEX16(tx_evt.dst, p_tx_params->dst.value);
    nrf_mesh_packet_send_StubWithCallback(NULL);
    return NRF_SUCCESS;
}

static uint32_t address_get_stub(dsm_handle_t handle, nrf_mesh_address_t * p_address, int num_calls)
{
    if (handle < ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT + SUBSCRIPTION_ADDRESS_COUNT)
    {
        p_address->type = m_addresses[handle].type;
        p_address->value = m_addresses[handle].value;
        p_address->p_virtual_uuid = m_addresses[handle].p_virtual_uuid;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

static void publish_timeout_cb(access_model_handle_t handle, void * p_args)
{
}

static inline void expect_msg(access_opcode_t opcode, uint32_t ref, const uint8_t * p_data, uint32_t length)
{
    msg_evt_t msg_evt;
    msg_evt.opcode = opcode;
    msg_evt.test_reference = ref;
    msg_evt.p_data = p_data;
    msg_evt.length = length;

    TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, fifo_push(&m_msg_fifo, &msg_evt),
                              "Unable to push to expect_msg FIFO.");
}

static inline void expect_tx(const uint8_t * p_data, uint32_t length, uint16_t src, uint16_t dst,
                             dsm_handle_t appkey_handle, dsm_handle_t subnet_handle,
                             tx_secmat_type_t tx_secmat_type)
{
    tx_evt_t tx_evt;
    tx_evt.p_data = p_data;
    tx_evt.length = length;
    tx_evt.src = src;
    tx_evt.dst = dst;

#if MESH_FEATURE_GATT_PROXY_ENABLED
    proxy_is_enabled_ExpectAndReturn(true);
#endif
    nrf_mesh_packet_send_StubWithCallback(packet_tx_stub);
    dsm_address_get_StubWithCallback(address_get_stub);

    switch (tx_secmat_type)
    {
        case TX_SECMAT_TYPE_FRIENDSHIP:
            dsm_tx_friendship_secmat_get_ExpectAndReturn(subnet_handle, appkey_handle, NULL,
                                                         m_dsm_tx_friendship_secmat_get_retval);
            dsm_tx_friendship_secmat_get_IgnoreArg_p_secmat();

            if (NRF_ERROR_NOT_FOUND != m_dsm_tx_friendship_secmat_get_retval)
            {
                break;
            }
            /* fall through */
        case TX_SECMAT_TYPE_MASTER:
            dsm_tx_secmat_get_ExpectAndReturn(subnet_handle, appkey_handle, NULL, NRF_SUCCESS);
            dsm_tx_secmat_get_IgnoreArg_p_secmat();
            break;
    }

    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);

    TEST_ASSERT_EQUAL_MESSAGE(NRF_SUCCESS, fifo_push(&m_tx_fifo, &tx_evt), "Unable to push to expect_tx FIFO.");
}

static uint32_t opcode_raw_write(access_opcode_t opcode, uint8_t * p_buffer)
{

    if (opcode.company_id != ACCESS_COMPANY_ID_NONE)
    {
        /* Make sure only 6-bit opcodes is used for vendor specific codes. */
        TEST_ASSERT_EQUAL_HEX16(0x00C0, (opcode.opcode & 0xFFC0));
        p_buffer[0] = opcode.opcode & 0x00FF;
        p_buffer[1] = opcode.company_id & 0x00FF;
        p_buffer[2] = (opcode.company_id >> 8) & 0x00FF;
        return 3;
    }
    else if ((opcode.opcode & 0xFF00) > 0)
    {
        TEST_ASSERT_EQUAL_HEX16(0x8000, (opcode.opcode & 0xC000));
        p_buffer[0] = (opcode.opcode >> 8) & 0x00FF;
        p_buffer[1] = opcode.opcode & 0x00FF;
        return 2;
    }
    else
    {
        p_buffer[0] = opcode.opcode & 0x00FF;
        return 1;
    }
}

static void send_msg(access_opcode_t opcode, const uint8_t * p_data, uint16_t length, dsm_handle_t dst, uint16_t key_index)
{
    uint8_t rx_buf[512];
    uint16_t len = opcode_raw_write(opcode, &rx_buf[0]);
    memcpy(&rx_buf[len], p_data, length);

    nrf_mesh_evt_t mesh_evt;
    nrf_mesh_rx_metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    memset(&mesh_evt, 0, sizeof(mesh_evt));
    mesh_evt.params.message.p_metadata = &metadata;
    mesh_evt.type = NRF_MESH_EVT_MESSAGE_RECEIVED;
    mesh_evt.params.message.p_buffer = rx_buf;
    mesh_evt.params.message.length = len + length;
    mesh_evt.params.message.src.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    mesh_evt.params.message.src.value= SOURCE_ADDRESS;
    mesh_evt.params.message.secmat.p_net = NULL;
    mesh_evt.params.message.secmat.p_app = NULL;

    dsm_address_is_rx_ExpectAndReturn(&mesh_evt.params.message.dst, true);

    if (dst < ACCESS_ELEMENT_COUNT)
    {
        mesh_evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        mesh_evt.params.message.dst.value = dst + ELEMENT_ADDRESS_START;
        dsm_local_unicast_addresses_get_Expect(NULL);
        dsm_local_unicast_addresses_get_IgnoreArg_p_address();
        dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);
    }
    else if (dst < ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT + SUBSCRIPTION_ADDRESS_COUNT)
    {
        mesh_evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_GROUP;
        mesh_evt.params.message.dst.value = dst - ACCESS_ELEMENT_COUNT + GROUP_ADDRESS_START;
        dsm_address_handle_get_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
        dsm_address_handle_get_IgnoreArg_p_address();
        dsm_address_handle_get_IgnoreArg_p_address_handle();
        dsm_address_handle_get_ReturnThruPtr_p_address_handle(&dst);
    }
    else
    {
        printf("Handle: %u\n", dst);
        TEST_FAIL_MESSAGE("Unknown handle");
    }
    dsm_appkey_handle_get_ExpectAndReturn(NULL, key_index);
    dsm_subnet_handle_get_ExpectAndReturn(NULL, key_index);

    mp_evt_handler->evt_cb(&mesh_evt);
}

static void opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    msg_evt_t msg_evt;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&m_msg_fifo, &msg_evt));
    TEST_ASSERT_EQUAL_HEX32(msg_evt.test_reference, (uint32_t) p_args); /*lint !e611 Cast needed to compare values. */
    TEST_ASSERT_EQUAL(msg_evt.opcode.opcode, p_message->opcode.opcode);
    TEST_ASSERT_EQUAL(msg_evt.opcode.company_id, p_message->opcode.company_id);
    TEST_ASSERT_EQUAL(msg_evt.length, p_message->length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(msg_evt.p_data, p_message->p_data, msg_evt.length);

    /* Reply */
    access_message_tx_t reply;
    uint8_t data[] = "reply";
    reply.opcode.opcode = 0x8234;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    reply.p_buffer = data;
    reply.length = sizeof(data);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    uint8_t expected_data[sizeof(data) + sizeof(uint32_t)];
    uint16_t length = opcode_raw_write(reply.opcode, expected_data);
    memcpy(&expected_data[length], data, sizeof(data));
    length += sizeof(data);

    bool friendship_credentials_flag = false;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_get(handle, &friendship_credentials_flag));

    /* Never loop back the response: */
    dsm_address_is_rx_ExpectAndReturn(&p_message->meta_data.src, false);
    expect_tx(expected_data,
                length,
                ELEMENT_ADDRESS_START + handle * ACCESS_ELEMENT_COUNT / ACCESS_MODEL_COUNT,
                p_message->meta_data.src.value,
                p_message->meta_data.appkey_handle,
                p_message->meta_data.subnet_handle,
                friendship_credentials_flag ? TX_SECMAT_TYPE_FRIENDSHIP : TX_SECMAT_TYPE_MASTER);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reply(handle, p_message, &reply));
}


/*******************************************************************************
 * Test Setup
 *******************************************************************************/

/** Distributes models across the number of elements. */
void build_device_setup(uint32_t elem_count, uint32_t model_count)
{
    const uint32_t MODELS_PER_ELEMENT = (model_count + elem_count - 1)/elem_count;
    access_model_add_params_t init_params;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = NULL;
    init_params.opcode_count = 0;
    init_params.p_args = NULL;
    init_params.publish_timeout_cb = NULL;

    for (uint32_t i = 0; i < elem_count; ++i)
    {
        uint32_t model_count__ = model_count > MODELS_PER_ELEMENT ? MODELS_PER_ELEMENT : model_count;
        model_count -= model_count__;
        for (uint32_t j = 0; j < model_count__; j++)
        {
            access_model_handle_t handle = i*MODELS_PER_ELEMENT + j;
            for (uint32_t k = 0; k < OPCODE_COUNT; ++k)
            {
                m_opcode_handlers[handle][k].opcode.opcode = (init_params.model_id.model_id-TEST_MODEL_ID)*OPCODE_COUNT + k;
                m_opcode_handlers[handle][k].opcode.company_id = ACCESS_COMPANY_ID_NONE;
                m_opcode_handlers[handle][k].handler = opcode_handler;
            }


            /* Model hasn't been allocated yet. */
            init_params.p_opcode_handlers = &m_opcode_handlers[handle][0];
            init_params.opcode_count = OPCODE_COUNT;
            init_params.p_args = ((uint32_t*)TEST_REFERENCE + handle);
            init_params.element_index = i;
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
            if (i > 0)
            {
                /* Try to add the model_id to a the previous element which should already have the
                 * model_id present. */
                TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_add(&init_params, &handle));
            }

            /* Re-adding the same model isn't allowed. */
            TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_add(&init_params, &handle));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(handle, 0));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_set(handle, 0));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(handle, ACCESS_ELEMENT_COUNT + handle));

            if (handle == (ACCESS_MODEL_COUNT - 1))
            {
                /* We compile the test with one list less than the number of models. */
                TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_subscription_list_alloc(handle));
            }
            else
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(handle));
            }
            init_params.model_id.model_id++;
        }

        /* Reset Model ID */
        init_params.model_id.model_id = TEST_MODEL_ID;
    }
}

static void setup_addresses(void)
{
    for (uint32_t i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        m_addresses[i].type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        m_addresses[i].value = ELEMENT_ADDRESS_START + i;
    }

    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        m_addresses[i + ACCESS_ELEMENT_COUNT].type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        m_addresses[i + ACCESS_ELEMENT_COUNT].value = PUBLISH_ADDRESS_START + i;
    }

    for (uint32_t i = 0; i < SUBSCRIPTION_ADDRESS_COUNT; ++i)
    {
        m_addresses[i + ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT].type = NRF_MESH_ADDRESS_TYPE_GROUP;
        m_addresses[i + ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT].value = GROUP_ADDRESS_START + i;
    }
}

void setUp(void)
{
    device_state_manager_mock_Init();
    flash_manager_mock_Init();
    nrf_mesh_mock_Init();
    nrf_mesh_events_mock_Init();
    nrf_mesh_utils_mock_Init();
    event_mock_Init();
    bearer_event_mock_Init();
    access_publish_mock_Init();
    proxy_mock_Init();
    flash_manager_entries_read_mock_Init();

    __LOG_INIT(0xFFFFFFFF, LOG_LEVEL_REPORT, LOG_CALLBACK_DEFAULT);
    memset(&m_msg_fifo, 0, sizeof(m_msg_fifo));
    memset(m_msg_evt_buffer, 0, sizeof(m_msg_evt_buffer));
    m_msg_fifo.array_len = MSG_EVT_MAX_COUNT;
    m_msg_fifo.elem_array = &m_msg_evt_buffer[0];
    m_msg_fifo.elem_size = sizeof(msg_evt_t);

    memset(&m_tx_fifo, 0, sizeof(m_tx_fifo));
    memset(m_tx_evt_buffer, 0, sizeof(m_tx_evt_buffer));
    m_tx_fifo.array_len = TX_EVT_MAX_COUNT;
    m_tx_fifo.elem_array = &m_tx_evt_buffer[0];
    m_tx_fifo.elem_size = sizeof(tx_evt_t);
    setup_addresses();
    fifo_init(&m_msg_fifo);

    m_flash_manager_calls = 0;
    m_listener_register_calls = 0;

    nrf_mesh_evt_handler_add_StubWithCallback(evt_handler_add_stub);
    dsm_flash_area_get_StubWithCallback(dsm_flash_area_get_stub);
    flash_manager_mem_listener_register_StubWithCallback(flash_manager_mem_listener_register_stub);
    flash_manager_add_StubWithCallback(flash_manager_add_stub);
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_cb);
    access_reliable_init_Expect();
    access_publish_init_Expect();
    access_init();

    m_dsm_tx_friendship_secmat_get_retval = NRF_SUCCESS;
}

void tearDown(void)
{
    TEST_ASSERT(fifo_is_empty(&m_msg_fifo));
    TEST_ASSERT(fifo_is_empty(&m_tx_fifo));
    flash_manager_mock_Verify();
    event_mock_Verify();
    bearer_event_mock_Verify();
    access_publish_mock_Verify();
    device_state_manager_mock_Verify();
    access_reliable_mock_Verify();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    nrf_mesh_utils_mock_Verify();
    nrf_mesh_utils_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    access_publish_mock_Verify();
    access_publish_mock_Destroy();
    proxy_mock_Verify();
    proxy_mock_Destroy();
    flash_manager_entries_read_mock_Verify();
    flash_manager_entries_read_mock_Destroy();
}


/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_model_add(void)
{
    access_model_handle_t handle;
    access_model_add_params_t init_params;
    init_params.element_index = 0;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
    init_params.opcode_count = 0;
    init_params.p_args = TEST_REFERENCE;
    init_params.publish_timeout_cb = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_add(&init_params, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_add(NULL, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_add(&init_params, &handle));
    init_params.opcode_count = OPCODE_COUNT;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_add(&init_params, &handle));

    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT-2);
    printf("Elements: %u, Models: %u\n", ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT-2);
    print_configuration();

    init_params.model_id.model_id = TEST_MODEL_ID + ACCESS_MODEL_COUNT/ACCESS_ELEMENT_COUNT - 1;
    init_params.p_opcode_handlers = &m_opcode_handlers[ACCESS_MODEL_COUNT-1][0];

    for (uint32_t i = 0; i < OPCODE_COUNT; ++i)
    {
        m_opcode_handlers[ACCESS_MODEL_COUNT-1][i].opcode.opcode = i + ACCESS_MODEL_COUNT;
        m_opcode_handlers[ACCESS_MODEL_COUNT-1][i].opcode.company_id = ACCESS_COMPANY_ID_NONE;
    }
    m_opcode_handlers[ACCESS_MODEL_COUNT-1][0].opcode.opcode = 0x07F;

    /* Expect an error since the opcode is invalid. */
    init_params.model_id.model_id++;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_add(&init_params, &handle));

    /* Make it valid => Success :) */
    m_opcode_handlers[ACCESS_MODEL_COUNT-1][0].opcode.company_id = 0xF00D;
    m_opcode_handlers[ACCESS_MODEL_COUNT-1][0].opcode.opcode = 0x0CE;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
    TEST_ASSERT_EQUAL(ACCESS_MODEL_COUNT-2, handle);

    /* Adding two models with same ID is not allowed. */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_add(&init_params, &handle));
    init_params.element_index = ACCESS_ELEMENT_COUNT;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_add(&init_params, &handle));

    /* There should be one free spot at the end now. Test, opcode count zero does not work when
     * handlers are present. */
    init_params.element_index = ACCESS_ELEMENT_COUNT - 1;
    init_params.opcode_count = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_add(&init_params, &handle));
    init_params.opcode_count = 1;
    init_params.p_opcode_handlers = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_add(&init_params, &handle));
    init_params.p_opcode_handlers =  &m_opcode_handlers[ACCESS_MODEL_COUNT-1][0];
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
    TEST_ASSERT_EQUAL(ACCESS_MODEL_COUNT-1, handle);

}

void test_access_model_element_index_get(void)
{
    access_model_handle_t handle;
    access_model_add_params_t init_params;
    uint16_t element_idx;

    for (uint32_t i = 0; i < OPCODE_COUNT; ++i)
    {
        m_opcode_handlers[0][i].opcode.opcode = i + ACCESS_MODEL_COUNT;
        m_opcode_handlers[0][i].opcode.company_id = ACCESS_COMPANY_ID_NONE;
    }

    init_params.element_index = ACCESS_ELEMENT_COUNT-1;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
    init_params.opcode_count = OPCODE_COUNT;
    init_params.p_args = TEST_REFERENCE;
    init_params.publish_timeout_cb = NULL;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));

    /* Test null input */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_element_index_get(handle, NULL));

    /* Test invalid model handle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_element_index_get(handle + 1, &element_idx));

    /* Test correct behaviour */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_element_index_get(handle, &element_idx));
    TEST_ASSERT_EQUAL(init_params.element_index, element_idx);
}

void test_rx_tx(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    const uint8_t data[] = "Hello, World!";
    const uint16_t data_length = sizeof(data);
    const uint32_t MODELS_PER_ELEMENT = ((ACCESS_MODEL_COUNT + ACCESS_ELEMENT_COUNT - 1)/ACCESS_ELEMENT_COUNT);

    for (uint32_t i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        for (uint32_t j = 0; j < MODELS_PER_ELEMENT; ++j)
        {
            for (uint32_t k = 0; k < OPCODE_COUNT; ++k)
            {
                access_model_handle_t model = i*MODELS_PER_ELEMENT + j;
                void * p_args = ((uint32_t *)TEST_REFERENCE + model);
                opcode.opcode = m_opcode_handlers[model][k].opcode.opcode;
                expect_msg(opcode, (uint32_t)p_args, data, data_length);
                /* Send on element address handle. */
                access_reliable_message_rx_cb_Expect(model, NULL, p_args);
                access_reliable_message_rx_cb_IgnoreArg_p_message();
                /* TODO: CMock tries to compare the location where p_args points to :( */
                access_reliable_message_rx_cb_IgnoreArg_p_args();

                send_msg(opcode, data, data_length, i, 0);
            }
        }
    }

    /* Test unknowns as well. */
    const access_opcode_t u_opcodes[] = { ACCESS_OPCODE_SIG(0x8234), ACCESS_OPCODE_VENDOR((0x12 | 0xc0), 0x1336) };
    for (uint32_t i = 0; i < ARRAY_SIZE(u_opcodes); ++i)
    {
        send_msg(u_opcodes[i], data, data_length, 0, 0);
    }

    /* Test subscriptions */
    opcode.opcode = m_opcode_handlers[0][0].opcode.opcode;
    /* Send on subscription handle that's not set yet. */
    send_msg(opcode, data, data_length, ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT, 0);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_add(ACCESS_MODEL_COUNT, ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT));

    /* Test reply with friendship secmat */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_set(0, true));

    void * p_args = ((uint32_t *)TEST_REFERENCE);
    opcode.opcode = m_opcode_handlers[0][0].opcode.opcode;
    expect_msg(opcode, (uint32_t)p_args, data, data_length);
    access_reliable_message_rx_cb_Expect(0, NULL, NULL);
    access_reliable_message_rx_cb_IgnoreArg_p_message();
    access_reliable_message_rx_cb_IgnoreArg_p_args();

    opcode.opcode = m_opcode_handlers[0][0].opcode.opcode;
    send_msg(opcode, data, data_length, 0, 0);

    /* Test reply with friendship secmat
     *
     * The Mesh Profile Specification v1.0, Section 4.2.2.4:
     *
     * When Publish Friendship Credential Flag is set to 1 and the friendship security material is
     * not available, the master security material shall be used. */
    m_dsm_tx_friendship_secmat_get_retval = NRF_ERROR_NOT_FOUND;
    expect_msg(opcode, (uint32_t)p_args, data, data_length);

    access_reliable_message_rx_cb_Expect(0, NULL, NULL);
    access_reliable_message_rx_cb_IgnoreArg_p_message();
    access_reliable_message_rx_cb_IgnoreArg_p_args();

    send_msg(opcode, data, data_length, 0, 0);
}

void test_unicast_loopback(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    const uint8_t data[] = "loopback";
    const uint16_t data_length = strlen((const char *) data);

    /* Set publish address handle to 0: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(0, 0));

    access_message_tx_t message =
    {
        .opcode = opcode, /*lint !e64 Type mismatch */
        .p_buffer = data,
        .length = data_length
    };

    /* This function is called once when sending the packet... */
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);

    /* and once when the packet is received again: */
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);

    /* Set publish address to the element's own address: */
    nrf_mesh_address_t destination = { NRF_MESH_ADDRESS_TYPE_UNICAST, ELEMENT_ADDRESS_START, NULL };
    dsm_address_get_StubWithCallback(address_get_stub);
    access_reliable_message_rx_cb_ExpectAnyArgs();

    bearer_event_flag_set_Expect(ACCESS_LOOPBACK_FLAG);
    dsm_address_is_rx_ExpectAndReturn(&destination, true); // accept loopback

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));

    dsm_address_is_rx_ExpectAndReturn(&destination, true); // called again in the bearer cb, as we created a loopback context.
    expect_msg(opcode, (uint32_t) TEST_REFERENCE, data, data_length);
    m_bearer_cb();
}

void test_group_loopback(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    const uint8_t data[] = "loopback";
    const uint16_t data_length = strlen((const char *) data);
    dsm_handle_t address_handle = ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT + 1;

    /* Set publish address handle to 0 and subscribe to it: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(0, address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, address_handle));

    access_message_tx_t message =
    {
        .opcode = opcode, /*lint !e64 Type mismatch */
        .p_buffer = data,
        .length = data_length
    };

    dsm_address_get_StubWithCallback(address_get_stub);

    dsm_address_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&address_handle);

    bearer_event_flag_set_Expect(ACCESS_LOOPBACK_FLAG);
    dsm_address_is_rx_ExpectAndReturn(&m_addresses[address_handle], true); // accept loopback

    // still expect a TX, as the loopback doesn't prevent it when sending to a group address:
    const uint8_t raw_packet_data[] = "\x00loopback";
    expect_tx(raw_packet_data, data_length + 1 /* opcode */, ELEMENT_ADDRESS_START, m_addresses[address_handle].value, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));

    dsm_address_is_rx_ExpectAndReturn(&m_addresses[address_handle], true); // called again in the bearer cb, as we created a loopback context.
    expect_msg(opcode, (uint32_t) TEST_REFERENCE, data, data_length);
    m_bearer_cb();
}

void test_virtual_loopback(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    const uint8_t data[] = "loopback";
    const uint16_t data_length = strlen((const char *) data);
    dsm_handle_t address_handle = ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT + 1;

    /* Replace the current address in the address array with a virtual address: */
    const uint8_t label_uuid[] = { 0xf4, 0xa0, 0x02, 0xc7, 0xfb, 0x1e, 0x4c, 0xa0, 0xa4, 0x69, 0xa0, 0x21, 0xde, 0x0d, 0xb8, 0x75 };
    m_addresses[address_handle].type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    m_addresses[address_handle].value = 0x9736;
    m_addresses[address_handle].p_virtual_uuid = label_uuid;

    /* Set publish address handle to 0 and subscribe to it: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(0, address_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, address_handle));

    access_message_tx_t message =
    {
        .opcode = opcode, /*lint !e64 Type mismatch */
        .p_buffer = data,
        .length = data_length
    };

    dsm_address_get_StubWithCallback(address_get_stub);

    dsm_address_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    dsm_address_handle_get_ReturnThruPtr_p_address_handle(&address_handle);

    bearer_event_flag_set_Expect(ACCESS_LOOPBACK_FLAG);
    dsm_address_is_rx_ExpectAndReturn(&m_addresses[address_handle], true); // accept loopback

    // still expect a TX, as the loopback doesn't prevent it when sending to a group address:
    const uint8_t raw_packet_data[] = "\x00loopback";
    expect_tx(raw_packet_data, data_length + 1 /* opcode */, ELEMENT_ADDRESS_START, m_addresses[address_handle].value, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));

    dsm_address_is_rx_ExpectAndReturn(&m_addresses[address_handle], true); // called again in the bearer cb, as we created a loopback context.
    expect_msg(opcode, (uint32_t) TEST_REFERENCE, data, data_length);
    m_bearer_cb();
}

void test_key_access(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    uint8_t data[] = "Hello, World!";
    uint16_t data_length = sizeof(data);

    /* Sanity check that we can receive a message for key index 0. */
    expect_msg(opcode, (uint32_t) TEST_REFERENCE, data, data_length);
    access_reliable_message_rx_cb_Expect(0, NULL, TEST_REFERENCE);
    access_reliable_message_rx_cb_IgnoreArg_p_message();
    /* TODO: CMock tries to compare the location where p_args points to :( */
    access_reliable_message_rx_cb_IgnoreArg_p_args();
    send_msg(opcode, data, data_length, 0, 0);

    /* Check that we don't receive a message that we don't have the application key for. */
    for (unsigned i = 1; i < DSM_APP_MAX; ++i)
    {
        send_msg(opcode, data, data_length, 0, i);
    }
}

void test_group_addressing(void)
{
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);
    /* Let the two last models share their lists. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(ACCESS_MODEL_COUNT-2, ACCESS_MODEL_COUNT-1));

    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    uint8_t data[] = "Hello, World!";
    uint16_t data_length = sizeof(data);

    /* Add the same subscription for every model instance. */
    const uint32_t MODELS_PER_ELEMENT = ((ACCESS_MODEL_COUNT + ACCESS_ELEMENT_COUNT - 1)/ACCESS_ELEMENT_COUNT);
    for (uint32_t i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        for (uint32_t j = 0; j < MODELS_PER_ELEMENT; ++j)
        {
            access_model_handle_t handle = i*MODELS_PER_ELEMENT + j;
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(handle, ACCESS_ELEMENT_COUNT + j));
        }
    }

    for (uint32_t i = 0; i < MODELS_PER_ELEMENT; ++i)
    {
        opcode.opcode = i*OPCODE_COUNT;
        for (uint32_t j = 0; j < ACCESS_ELEMENT_COUNT; j++)
        {
            /* Expect one message for each of the elements. */
            expect_msg(opcode, (uint32_t) ((uint32_t *)TEST_REFERENCE + j*MODELS_PER_ELEMENT + i), data, data_length);
        }
        send_msg(opcode, data, data_length, ACCESS_ELEMENT_COUNT + i, 0);
    }
}

void test_model_publish(void)
{
    access_message_tx_t message;
    const uint8_t data[] = "Hello, World";
    const access_opcode_t opcodes[] = { ACCESS_OPCODE_SIG(0x0040), ACCESS_OPCODE_SIG(0x8123), ACCESS_OPCODE_VENDOR(0x0c0, 0x1337) };
    const unsigned num_opcodes = ARRAY_SIZE(opcodes);
    message.opcode.opcode = opcodes[0].opcode;
    message.opcode.company_id =opcodes[0].company_id;
    message.p_buffer = data;
    message.length = sizeof(data);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish(ACCESS_MODEL_COUNT, &message));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish(0, &message));
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    message.length = ACCESS_MESSAGE_LENGTH_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_publish(0, &message));
    message.length = sizeof(data);
    message.opcode.opcode = 0x7f;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish(0, &message));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish(0, &message));

    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; ++i)
    {
        message.opcode.opcode = opcodes[i % num_opcodes].opcode;
        message.opcode.company_id =opcodes[i % num_opcodes].company_id;
        message.p_buffer = data;
        message.length = sizeof(data);

        uint16_t src = ELEMENT_ADDRESS_START + i * ACCESS_ELEMENT_COUNT / ACCESS_MODEL_COUNT;
        uint16_t dst = PUBLISH_ADDRESS_START + i;

        uint8_t expected_data[sizeof(data) + sizeof(uint32_t)];
        uint32_t length = opcode_raw_write(message.opcode, expected_data);
        memcpy(&expected_data[length], data, message.length);
        length += message.length;
        nrf_mesh_address_t dst_addr = {NRF_MESH_ADDRESS_TYPE_UNICAST, dst};
        dsm_address_is_rx_ExpectAndReturn(&dst_addr, false); // don't want loopback

        expect_tx(expected_data, length, src, dst, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(i, &message));
    }

    /* Test with frienship secmat */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_set(0, true));

    message.opcode.opcode = opcodes[0].opcode;
    message.opcode.company_id =opcodes[0].company_id;
    message.p_buffer = data;
    message.length = sizeof(data);

    uint16_t src = ELEMENT_ADDRESS_START;
    uint16_t dst = PUBLISH_ADDRESS_START;

    uint8_t expected_data[sizeof(data) + sizeof(uint32_t)];
    uint32_t length = opcode_raw_write(message.opcode, expected_data);
    memcpy(&expected_data[length], data, message.length);
    length += message.length;
    nrf_mesh_address_t dst_addr = {NRF_MESH_ADDRESS_TYPE_UNICAST, dst};
    dsm_address_is_rx_ExpectAndReturn(&dst_addr, false); // don't want loopback

    expect_tx(expected_data, length, src, dst, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_FRIENDSHIP);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));

    /* Test with frienship secmat
     *
     * The Mesh Profile Specification v1.0, Section 4.2.2.4:
     *
     * When Publish Friendship Credential Flag is set to 1 and the friendship security material is
     * not available, the master security material shall be used. */
    m_dsm_tx_friendship_secmat_get_retval = NRF_ERROR_NOT_FOUND;
    dsm_address_is_rx_ExpectAndReturn(&dst_addr, false); // don't want loopback
    expect_tx(expected_data, length, src, dst, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_FRIENDSHIP);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));
}

void test_against_spec_messages(void)
{
    access_opcode_handler_t opcode_handlers[] =
    {
        {ACCESS_OPCODE_VENDOR(MSG_21_VENDOR_MODEL_OPCODE, MSG_21_VENDOR_MODEL_COMPANY_ID), opcode_handler},
    };

    access_model_handle_t model_handle;
    access_model_add_params_t init_params;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = MSG_21_VENDOR_MODEL_COMPANY_ID;
    init_params.p_opcode_handlers = NULL;
    init_params.opcode_count = 0;
    init_params.p_args = NULL;
    init_params.publish_timeout_cb = NULL;
    init_params.p_opcode_handlers = &opcode_handlers[0];
    init_params.opcode_count = sizeof(opcode_handlers) / sizeof(opcode_handlers[0]);
    init_params.p_args = 0;
    init_params.element_index = 0;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &model_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(model_handle, 0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_set(model_handle, 0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(model_handle, ACCESS_ELEMENT_COUNT + model_handle));

    uint8_t access_params[] = MSG_21_ACCESS_PARAMS_RAW;
    uint8_t access_payload[] = MSG_21_ACCESS_PAYLOAD_RAW;

    /* Test tx */
    {

        access_message_tx_t message;
        message.opcode.opcode = MSG_21_VENDOR_MODEL_OPCODE;
        message.opcode.company_id = MSG_21_VENDOR_MODEL_COMPANY_ID;
        message.p_buffer = access_params;
        message.length = sizeof(access_params);
        message.force_segmented = false;
        message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

        uint16_t src = ELEMENT_ADDRESS_START;
        uint16_t dst = PUBLISH_ADDRESS_START;

        nrf_mesh_address_t dst_addr = {NRF_MESH_ADDRESS_TYPE_UNICAST, dst, NULL};

        dsm_address_is_rx_ExpectAndReturn(&dst_addr, false); // don't want loopback

        expect_tx(access_payload, sizeof(access_payload), src, dst, 0, DSM_HANDLE_INVALID, false);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));
    }

    /* Test rx */
    {
        expect_msg(opcode_handlers[0].opcode, 0, access_params, sizeof(access_params));

        /* Send on element address handle. */
        access_reliable_message_rx_cb_Expect(model_handle, NULL, 0);
        access_reliable_message_rx_cb_IgnoreArg_p_message();
        access_reliable_message_rx_cb_IgnoreArg_p_args();

        nrf_mesh_rx_metadata_t metadata;
        memset(&metadata, 0, sizeof(metadata));

        nrf_mesh_evt_t mesh_evt;
        memset(&mesh_evt, 0, sizeof(mesh_evt));
        mesh_evt.params.message.p_metadata = &metadata;
        mesh_evt.type = NRF_MESH_EVT_MESSAGE_RECEIVED;
        mesh_evt.params.message.p_buffer = access_payload;
        mesh_evt.params.message.length = sizeof(access_payload);
        mesh_evt.params.message.src.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        mesh_evt.params.message.src.value= SOURCE_ADDRESS;
        mesh_evt.params.message.secmat.p_net = NULL;
        mesh_evt.params.message.secmat.p_app = NULL;
        mesh_evt.params.message.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        mesh_evt.params.message.dst.value = ELEMENT_ADDRESS_START;

        dsm_address_is_rx_ExpectAndReturn(&mesh_evt.params.message.dst, true);
        dsm_local_unicast_addresses_get_Expect(NULL);
        dsm_local_unicast_addresses_get_IgnoreArg_p_address();
        dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);
        dsm_appkey_handle_get_ExpectAndReturn(NULL, 0);
        dsm_subnet_handle_get_ExpectAndReturn(NULL, 0);

        mp_evt_handler->evt_cb(&mesh_evt);
    }
}

void test_error_conditions(void)
{
    /* Not initialized */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_set(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_set(0, (access_publish_resolution_t) 0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_add(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_remove(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_bind(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_unbind(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_application_set(0, 0));

    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    /* Out of bounds */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_set(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_set(ACCESS_MODEL_COUNT, (access_publish_resolution_t) 0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_add(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_remove(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_bind(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_unbind(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_application_set(ACCESS_MODEL_COUNT, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_element_location_set(ACCESS_ELEMENT_COUNT, 0));

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_address_set(0, DSM_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_subscription_add(0, DSM_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_subscription_remove(0, DSM_ADDR_MAX));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_application_bind(0, DSM_APP_MAX + DSM_DEVICE_MAX));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_application_unbind(0, DSM_APP_MAX + DSM_DEVICE_MAX));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_application_set(0, DSM_APP_MAX + DSM_DEVICE_MAX));

    /* Unknown event shouldn't do anything. */
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION;
    mp_evt_handler->evt_cb(&evt);

    /* Publishing with no mem should give an error. */
    access_message_tx_t tx_message = {};
    access_message_rx_t rx_message = {};
    tx_message.length = 0;
    tx_message.opcode.opcode = 0x00;
    tx_message.opcode.company_id = ACCESS_COMPANY_ID_NONE;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_reply(0, NULL, &tx_message));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_reply(0, &rx_message, NULL));
    tx_message.length = ACCESS_MESSAGE_LENGTH_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_reply(0,&rx_message, &tx_message));
    tx_message.length = 0;
    tx_message.opcode.opcode = 0xFFFF;
    tx_message.opcode.company_id = 0x1337;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_reply(0,&rx_message, &tx_message));

    const dsm_handle_t ADDRESS_COUNT = ACCESS_ELEMENT_COUNT + ACCESS_MODEL_COUNT + SUBSCRIPTION_ADDRESS_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, ADDRESS_COUNT-1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(0, DSM_APP_MAX + DSM_DEVICE_MAX-1));

    /* Sanity checking. */
    const uint8_t data[] = "Test Data";
    access_opcode_t opcode = ACCESS_OPCODE_SIG(0);
    expect_msg(opcode, (uint32_t) TEST_REFERENCE, data, sizeof(data));

    send_msg(opcode, data, sizeof(data), ADDRESS_COUNT-1, DSM_APP_MAX + DSM_DEVICE_MAX-1);

    /* Send with wrong key. */
    send_msg(opcode, data, sizeof(data), ADDRESS_COUNT-1, DSM_APP_MAX + DSM_DEVICE_MAX-2);

    /* Send with wrong address. */
    send_msg(opcode, data, sizeof(data), ADDRESS_COUNT-2, DSM_APP_MAX + DSM_DEVICE_MAX-1);

    /* Remove key. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_unbind(0, DSM_APP_MAX + DSM_DEVICE_MAX-1));
    send_msg(opcode, data, sizeof(data), ADDRESS_COUNT-1, DSM_APP_MAX + DSM_DEVICE_MAX-1);

    /* Add key back in, but remove address. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(0, DSM_APP_MAX + DSM_DEVICE_MAX-1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_remove(0, ADDRESS_COUNT-1));
    send_msg(opcode, data, sizeof(data), ADDRESS_COUNT-1, DSM_APP_MAX + DSM_DEVICE_MAX-1);
}

void test_settergetter(void)
{
    dsm_handle_t address_handle = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_set(0, address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_address_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_get(ACCESS_MODEL_COUNT, &address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_get(0, &address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_address_get(0, &address_handle));


    bool flag = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_friendship_credential_flag_set(0, flag));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_friendship_credential_flag_get(ACCESS_MODEL_COUNT, &flag));

    uint8_t ttl = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_ttl_set(0, ttl));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_ttl_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_ttl_get(ACCESS_MODEL_COUNT, &ttl));

    access_publish_resolution_t resolution = (access_publish_resolution_t) 0;
    uint8_t step_number = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_set(0, resolution, step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_period_get(0, NULL, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_period_get(0, &resolution, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_period_get(0, NULL, &step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_get(ACCESS_MODEL_COUNT, &resolution, &step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_get(0, &resolution, &step_number));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_add(0, address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_remove(0, address_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_list_alloc(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscription_lists_share(0, 1));

    dsm_handle_t addresses[DSM_ADDR_MAX];
    uint16_t size = DSM_ADDR_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_subscriptions_get(0, NULL, &size));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_subscriptions_get(0, addresses, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_subscriptions_get(0, addresses, &size));

    dsm_handle_t applications[DSM_APP_MAX + DSM_DEVICE_MAX] = {0};
    size = DSM_APP_MAX + DSM_DEVICE_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_bind(0, applications[0]));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_application_unbind(0, applications[0]));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_applications_get(0, NULL, &size));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_applications_get(0, applications, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_applications_get(0, applications, &size));

    dsm_handle_t appkey_handle = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_application_set(0, appkey_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_application_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_application_get(0, &appkey_handle));

    access_model_id_t model_id = {TEST_MODEL_ID, ACCESS_COMPANY_ID_NONE};
    access_model_handle_t handle;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_handle_get(0, model_id, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_handle_get(0, model_id, &handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_handle_get(ACCESS_ELEMENT_COUNT, model_id, &handle));

    size = 0;
    access_model_handle_t models[ACCESS_MODEL_COUNT];
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_element_models_get(0, NULL, &size));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_element_models_get(0, models, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_element_models_get(0, models, &size));
    size = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_element_models_get(ACCESS_ELEMENT_COUNT, models, &size));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, models, &size));
    TEST_ASSERT_EQUAL(0, size); /* No models added yet. */

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_id_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_id_get(ACCESS_MODEL_COUNT, &model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_id_get(0, &model_id));

    uint8_t model_count;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_element_vendor_model_count_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_element_vendor_model_count_get(ACCESS_ELEMENT_COUNT, &model_count));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, &model_count));
    TEST_ASSERT_EQUAL(0, model_count);

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_element_sig_model_count_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_element_sig_model_count_get(ACCESS_ELEMENT_COUNT, &model_count));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_sig_model_count_get(0, &model_count));
    TEST_ASSERT_EQUAL(0, model_count);

    uint16_t location;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_element_location_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_element_location_get(ACCESS_ELEMENT_COUNT, &location));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(0, 0x0103)); /* "Bottom", false */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(0, &location));
    TEST_ASSERT_EQUAL(0x0103, location);

    void * p_args;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_p_args_get(0, &p_args));

    /* Allocate a model */
    access_model_add_params_t init_params;
    init_params.element_index = 0;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
    init_params.opcode_count = OPCODE_COUNT;
    init_params.p_args = TEST_REFERENCE;
    init_params.publish_timeout_cb = publish_timeout_cb;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
    TEST_ASSERT_EQUAL(0, handle);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_sig_model_count_get(0, &model_count));
    TEST_ASSERT_EQUAL(1, model_count);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, &model_count));
    TEST_ASSERT_EQUAL(0, model_count);

    init_params.publish_timeout_cb = NULL;
    init_params.model_id.company_id = 0x1337;
    init_params.element_index = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
    TEST_ASSERT_EQUAL(1, handle);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_sig_model_count_get(1, &model_count));
    TEST_ASSERT_EQUAL(0, model_count);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(1, &model_count));
    TEST_ASSERT_EQUAL(1, model_count);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_add(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_remove(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, access_model_subscription_lists_share(0, 1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(0));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(0, 1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(0, 1));

    model_id.model_id = TEST_MODEL_ID;
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_handle_get(0, model_id, &handle));
    TEST_ASSERT_EQUAL(0, handle);

    address_handle = 2;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(0, address_handle));
    address_handle = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_get(0, &address_handle));
    TEST_ASSERT_EQUAL(2, address_handle);

    resolution = (access_publish_resolution_t) 1;
    step_number = 2;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_publish_period_set(1, resolution, step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_publish_period_get(1, &resolution, &step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_period_set(0, (access_publish_resolution_t) (ACCESS_PUBLISH_RESOLUTION_MAX + 1), step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_period_set(0, (access_publish_resolution_t) 0, ACCESS_PUBLISH_PERIOD_STEP_MAX + 1));

    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(0, resolution, step_number));

    access_publish_period_get_Expect(NULL, &resolution, &step_number);
    access_publish_period_get_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_get(0, &resolution, &step_number));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(0, 2));
    size = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_subscriptions_get(0, addresses, &size));
    size = 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_subscriptions_get(0, addresses, &size));
    for (uint32_t i = 0; i < 2; ++i)
    {
        /* Test for both models that share the same list :) */
        size = DSM_ADDR_MAX;
        addresses[0] = 0;
        addresses[1] = 0;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscriptions_get(i, addresses, &size));
        TEST_ASSERT_EQUAL(2, size);
        TEST_ASSERT_EQUAL(2, addresses[0]);
    }

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_remove(0, 2));
    for (uint32_t i = 0; i < 2; ++i)
    {
        /* Should be removed for both. */
        size = DSM_ADDR_MAX;
        addresses[0] = 0;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscriptions_get(0, addresses, &size));
        TEST_ASSERT_EQUAL(1, size);
        TEST_ASSERT_EQUAL(3, addresses[0]);
    }

    applications[0] = 4;
    applications[1] = 3;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(0, applications[0]));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(0, applications[1]));
    size = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_applications_get(0, applications, &size));
    size = 1;
    applications[0] = 0;
    applications[1] = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_applications_get(0, applications, &size));
    size = 2;
    applications[0] = 0;
    applications[1] = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_applications_get(0, applications, &size));
    TEST_ASSERT_EQUAL(3, applications[0]);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_unbind(0, applications[0]));
    size = DSM_APP_MAX + DSM_DEVICE_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_applications_get(0, applications, &size));
    TEST_ASSERT_EQUAL(1, size);

    appkey_handle = 4;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_set(0, appkey_handle));
    appkey_handle = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_get(0, &appkey_handle));
    TEST_ASSERT_EQUAL(4, appkey_handle);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_id_get(0, &model_id));
    TEST_ASSERT_EQUAL(TEST_MODEL_ID, model_id.model_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, model_id.company_id);

    p_args = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_p_args_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_p_args_get(ACCESS_MODEL_COUNT, &p_args));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_p_args_get(0, &p_args));
    TEST_ASSERT_EQUAL_PTR(TEST_REFERENCE, p_args);

    ttl = NRF_MESH_TTL_MAX + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_ttl_set(0, ttl));
    ttl = NRF_MESH_TTL_MAX;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(0, ttl));
    ttl = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_get(0, &ttl));
    TEST_ASSERT_EQUAL(NRF_MESH_TTL_MAX, ttl);

    flag = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_set(0, flag));
    flag = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_get(0, &flag));
    TEST_ASSERT_EQUAL(1, flag);
}

void test_flash_load_reload(void)
{
    /* Build some models, verify their content, store in "flash", wipe access layer state,
     * add the same models again and restore and check the expected state.*/

    /************************************ Build test vectors *************************************/
    dsm_handle_t appkeys[] = {0x0,0x1,0x2,0x3,0x4,0x5};
    dsm_handle_t addresses[] = {0x6,0x7,0x8,0x9,0xA};
    uint32_t sizeof_addresses = sizeof(addresses)/sizeof(dsm_handle_t);
    uint32_t sizeof_appkeys = sizeof(appkeys)/sizeof(dsm_handle_t);
    uint32_t address_bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
    bitfield_set_all(address_bitfield, DSM_ADDR_MAX);
    TEST_ASSERT_EQUAL(address_bitfield[0], UINT32_MAX);
    for (uint32_t i = 0; i < sizeof_addresses; ++i)
    {
        bitfield_clear(address_bitfield, addresses[i]);
    }
    access_model_id_t model_id1 = {0x1337, 0x15};
    access_model_id_t model_id2 = {0xC000, 0x0001};
    access_publish_period_t pub_period1 = {1,5};
    access_publish_period_t pub_period2 = {(1<<ACCESS_PUBLISH_STEP_RES_BITS) - 1, (1<<ACCESS_PUBLISH_STEP_NUM_BITS) - 1};

    /*lint -save -e651 -e64 Disregard warnings about confusing initializers and type mismatches caused by the macros. */
    access_flash_test_struct_t test_vector[] =
    {
        FLASH_TEST_VECTOR_INSTANCE(model_id1, 0, &addresses[0], 2, UINT32_MAX, 0xA, pub_period1, &appkeys[0], 3, 0x1, 4, 0),
        FLASH_TEST_VECTOR_INSTANCE(model_id2, 1, &addresses[2], sizeof_addresses -2, 0, 0xC, pub_period2, &appkeys[3], sizeof_appkeys - 3, 0x0, 127, 1)
    };
    uint32_t test_vector_size = sizeof(test_vector)/sizeof(access_flash_test_struct_t);
    /*lint -restore */

   /****************** Add models and configure as specified by the test vector. ******************/
    access_model_handle_t model_handle[test_vector_size];
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        model_handle[i] = init_test_model_and_subs_list(&test_vector[i]);
        update_test_model(&test_vector[i], model_handle[i], true);
    }

    /***************** Store the necessary configuration for a restore on bootup. *****************/

    /** Expect the metadata to be written since this is the first store call */
    fm_entry_t * p_metadata_flash_entry = expect_flash_manager_entry(FLASH_HANDLE_METADATA, sizeof(access_flash_metadata_t));
    /** Expect only one of the subscription lists to be written since it's shared by both of the models in the test vector. */
    fm_entry_t * p_subs_flash_entry[ACCESS_SUBSCRIPTION_LIST_COUNT];
    p_subs_flash_entry[0] = expect_flash_manager_entry(FLASH_GROUP_SUBS_LIST, sizeof(access_flash_subscription_list_t));
    /** We made no changes to the "location" of the elements so no flash operations expected. */
    /** All the models added by the test vector should be stored to flash. */
    fm_entry_t * p_model_flash_entry[test_vector_size + 1];
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        p_model_flash_entry[i] = expect_flash_manager_entry(FLASH_GROUP_MODEL | model_handle[i], sizeof(access_model_state_data_t));
    }
    /* All stored successfully. */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    /********************* Verify the data sent to the flash_manager: *****************************/
    access_flash_metadata_t * p_metadata = (access_flash_metadata_t *)p_metadata_flash_entry->data;
    TEST_ASSERT_EQUAL_UINT16(ACCESS_ELEMENT_COUNT, p_metadata->element_count);
    TEST_ASSERT_EQUAL_UINT16(ACCESS_SUBSCRIPTION_LIST_COUNT, p_metadata->subscription_list_count);
    TEST_ASSERT_EQUAL_UINT16(ACCESS_MODEL_COUNT, p_metadata->model_count);
    access_flash_subscription_list_t * p_subs_list = (access_flash_subscription_list_t *) p_subs_flash_entry[0]->data;
    TEST_ASSERT_EQUAL(FLASH_GROUP_SUBS_LIST | 0, p_subs_flash_entry[0]->header.handle);
    TEST_ASSERT_EQUAL_UINT32_ARRAY(address_bitfield, p_subs_list->inverted_bitfield, BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX));

    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        TEST_ASSERT_EQUAL(FLASH_GROUP_MODEL | i, p_model_flash_entry[i]->header.handle);
        access_model_state_data_t * p_model_data = (access_model_state_data_t *) p_model_flash_entry[i]->data;
        verify_flash_modeldata(&test_vector[i], p_model_data, 0);
    }

    /****************** Try store without and with further changes to the models ******************/

    /* Since nothing has changed in access configuration another store should not call the flash api. */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    /* Change location of one element and call store. */
    uint16_t location_element2 = 5;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(1, location_element2));
    fm_entry_t * p_element_flash_entry = expect_flash_manager_entry(FLASH_GROUP_ELEMENT | 1, sizeof(uint16_t));
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();
    uint16_t * p_location = (uint16_t *)&p_element_flash_entry->data[0];
    TEST_ASSERT_EQUAL_UINT16(location_element2, *p_location);

    /* Since nothing has changed in access configuration another store should not call the flash api. */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    /* Changing element location to 0 will do nothing since that's the default value*/
    uint16_t location_element1 = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(0, location_element1));
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    /* Updating model info (subscribe/puiblish information) without actually changing any value should not cause flash operations */
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        update_test_model(&test_vector[i], model_handle[i], false);
    }
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    /*  Using a new ttl value for the first model will initiate a flash operation for the whole model, but not other models */
    test_vector[0].publish_ttl = 50;
    update_test_model(&test_vector[0], model_handle[0], false);
    p_model_flash_entry[0] = expect_flash_manager_entry(FLASH_GROUP_MODEL | model_handle[0], sizeof(access_model_state_data_t));
    for (uint32_t i = 1; i < test_vector_size; ++i)
    {
        update_test_model(&test_vector[i], model_handle[i], false);
    }

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();

    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        access_model_state_data_t * p_model_data = (access_model_state_data_t *) p_model_flash_entry[i]->data;
        verify_flash_modeldata(&test_vector[i], p_model_data, 0);
    }

    /* Add a new model to element 1 and allocate a new subscription list for it. */
    access_flash_test_struct_t new_test_case =
        FLASH_TEST_VECTOR_INSTANCE(model_id1, 1, &addresses[0], sizeof_addresses, UINT32_MAX,
                                   0xA, pub_period1, &appkeys[0], 3, 0x1, 4, 0); /*lint !e64 Type mismatch. */
    access_model_handle_t new_test_case_model_handle = init_test_model_and_subs_list(&new_test_case);
    update_test_model(&new_test_case, new_test_case_model_handle, true);
    p_subs_flash_entry[1] = expect_flash_manager_entry(FLASH_GROUP_SUBS_LIST | 1, sizeof(access_flash_subscription_list_t));
    p_model_flash_entry[test_vector_size] = expect_flash_manager_entry(FLASH_GROUP_MODEL | new_test_case_model_handle, sizeof(access_model_state_data_t));
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        update_test_model(&test_vector[i], model_handle[i], false);
    }
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();
    verify_flash_modeldata(&new_test_case, (access_model_state_data_t *) p_model_flash_entry[test_vector_size]->data, 1);
    access_flash_subscription_list_t * p_subs_list2 = (access_flash_subscription_list_t *) p_subs_flash_entry[1]->data;
    TEST_ASSERT_EQUAL_UINT32_ARRAY(address_bitfield, p_subs_list2->inverted_bitfield, BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX));

    /* Verify that reading from access will produce the values given by the test vectors. */
    verify_test_case_and_access_state(&new_test_case, new_test_case_model_handle);
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_test_case_and_access_state(&test_vector[i], model_handle[i]);
    }

    /********************************* Reset access and lose all data:*****************************/
    access_publish_init_Expect();
    access_reliable_init_Expect();
    access_init();
    /* Check that elements are un populated */
    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    uint16_t models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);

    /* If there is nothing stored, and the flash_manager returns not found for metadata we expect nothing to happen: */
    uint32_t entry_length = sizeof(access_flash_metadata_t);
    flash_manager_entry_read_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, NULL, &entry_length, NRF_ERROR_NOT_FOUND);
    flash_manager_entry_read_IgnoreArg_p_data();
    TEST_ASSERT_FALSE(access_flash_config_load());


    /********************************* Restore access state from flash:*****************************/
    /* First read the metadata*/
    flash_manager_entry_read_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, NULL, &entry_length, NRF_SUCCESS);
    flash_manager_entry_read_IgnoreArg_p_data();
    flash_manager_entry_read_ReturnMemThruPtr_p_data(p_metadata_flash_entry->data, sizeof(access_flash_metadata_t));
    /* Metadata is sane so subscriptionlist, elements, and models should be restored */
    fm_handle_filter_t subs_filter = {.mask = FLASH_HANDLE_FILTER_MASK, .match = FLASH_GROUP_SUBS_LIST};
    restore_flash(&subs_filter, p_subs_flash_entry, 2);

    fm_handle_filter_t elem_filter = {.mask = FLASH_HANDLE_FILTER_MASK, .match = FLASH_GROUP_ELEMENT};
    restore_flash(&elem_filter, &p_element_flash_entry, 1);
    fm_handle_filter_t model_filter = {.mask = FLASH_HANDLE_FILTER_MASK, .match = FLASH_GROUP_MODEL};
    restore_flash(&model_filter, p_model_flash_entry, 3);
    for (int i = sizeof_addresses-1; i >= 0; --i)
    {
        dsm_address_publish_add_handle_IgnoreAndReturn(NRF_SUCCESS);
        dsm_address_subscription_add_handle_IgnoreAndReturn(NRF_SUCCESS);
    }
    TEST_ASSERT_TRUE(access_flash_config_load());

    /* Even though we have successfully loaded all the flash data, the elements are still  unpopulated:
     * The user must add the models first. */
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);


    /*  Add models to revive them. */
    access_model_handle_t restored_model_handle;
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
       TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&test_vector[i].add_params, &restored_model_handle));
       TEST_ASSERT_EQUAL(model_handle[i],  restored_model_handle);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&new_test_case.add_params, &restored_model_handle));

    TEST_ASSERT_EQUAL(new_test_case_model_handle,  restored_model_handle);
    /* Verify that reading from access will produce the values given by the test vectors. */
    verify_test_case_and_access_state(&new_test_case, new_test_case_model_handle);
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_test_case_and_access_state(&test_vector[i], model_handle[i]);
    }

    /* Allocation  subscription list should not affect anything and the user should simply get a success: */
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(model_handle[i]));
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(new_test_case_model_handle));
    /* Verify that reading from access will produce the values given by the test vectors. */
    verify_test_case_and_access_state(&new_test_case, new_test_case_model_handle);
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_test_case_and_access_state(&test_vector[i], model_handle[i]);
    }

    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(2, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(1, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(2, models_count);

    uint16_t location = 0xFF;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(0, &location));
    TEST_ASSERT_EQUAL(location_element1, location);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(1, &location));
    TEST_ASSERT_EQUAL(location_element2, location);

    /* Adding models before a restore is also accepted. */
    access_publish_init_Expect();
    access_reliable_init_Expect();
    access_init();
    /* Add models first before restore*/
    access_model_handle_t reinit_handle;
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
       TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&test_vector[i].add_params, &reinit_handle));
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&new_test_case.add_params, &reinit_handle));

    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(2, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(1, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(2, models_count);

    /* First read the metadata */
    flash_manager_entry_read_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, NULL, &entry_length, NRF_SUCCESS);
    flash_manager_entry_read_IgnoreArg_p_data();
    flash_manager_entry_read_ReturnMemThruPtr_p_data(p_metadata_flash_entry->data, sizeof(access_flash_metadata_t));
    restore_flash(&subs_filter, p_subs_flash_entry, 2);
    restore_flash(&elem_filter, &p_element_flash_entry, 1);
    restore_flash(&model_filter, p_model_flash_entry, 3);
    TEST_ASSERT_TRUE(access_flash_config_load());

    /* Verify that reading from access will produce the values given by the test vectors. */
    verify_test_case_and_access_state(&new_test_case, new_test_case_model_handle);
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_test_case_and_access_state(&test_vector[i], model_handle[i]);
    }

    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(2, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(1, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(2, models_count);

    location = 0xFF;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(0, &location));
    TEST_ASSERT_EQUAL(location_element1, location);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(1, &location));
    TEST_ASSERT_EQUAL(location_element2, location);


    /**************************************** Flash erase *****************************************/
    /* Try a direct call to access_clear */
    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_SUCCESS);
    access_reliable_cancel_all_Expect();
    access_clear();

    /* Return unexpected metadata from flash, this should trigger wipe */
    p_metadata = (access_flash_metadata_t *) p_metadata_flash_entry->data;
    p_metadata->model_count--;
    flash_manager_entry_read_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, NULL, &entry_length, NRF_SUCCESS);
    flash_manager_entry_read_IgnoreArg_p_data();
    flash_manager_entry_read_ReturnMemThruPtr_p_data(p_metadata_flash_entry->data, sizeof(access_flash_metadata_t));
    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_SUCCESS);
    TEST_ASSERT_FALSE(access_flash_config_load());

    /************************************** Flash callbacks ***************************************/
    /* A call back about the removal of the flash manager should cause it to be added again. */
    uint32_t expected_add_calls = m_flash_manager_calls + 1;
    /** Expect the metadata to be written since we have erased the flash in earlier tests. (fail this attempt) */
    m_flash_manager_config.remove_complete_cb(mp_flash_manager);
    TEST_ASSERT_EQUAL(expected_add_calls, m_flash_manager_calls);

    /* Test the write complete callback all are ignored except flash malfunction */
    m_flash_manager_config.write_complete_cb(mp_flash_manager, p_metadata_flash_entry, FM_RESULT_SUCCESS);
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_FLASH_FAILED,
            .params.flash_failed.user = NRF_MESH_FLASH_USER_ACCESS,
            .params.flash_failed.p_flash_entry = p_metadata_flash_entry,
            .params.flash_failed.p_flash_page = NULL,
            .params.flash_failed.p_area = mp_flash_manager->config.p_area,
            .params.flash_failed.page_count = mp_flash_manager->config.page_count,
        };
    event_handle_Expect(&evt);
    m_flash_manager_config.write_complete_cb(mp_flash_manager, p_metadata_flash_entry, FM_RESULT_ERROR_FLASH_MALFUNCTION);
    TEST_NRF_MESH_ASSERT_EXPECT(m_flash_manager_config.write_complete_cb(mp_flash_manager, p_metadata_flash_entry, FM_RESULT_ERROR_AREA_FULL));
    TEST_NRF_MESH_ASSERT_EXPECT(m_flash_manager_config.write_complete_cb(mp_flash_manager, p_metadata_flash_entry, FM_RESULT_ERROR_NOT_FOUND));

    /****************************** Retry failed flash operations  ********************************/
    /* Try a store but fail to write the metadata */
    TEST_ASSERT_EQUAL(0, m_listener_register_calls);
    flash_manager_entry_alloc_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, sizeof(access_flash_metadata_t), NULL);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();
    /* The failed attempt will trigger a register call. */
    TEST_ASSERT_EQUAL(1, m_listener_register_calls);
    /* test access_clear */
    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_ERROR_NO_MEM);
    access_reliable_cancel_all_Expect();
    access_clear();
    TEST_ASSERT_EQUAL(2, m_listener_register_calls);

    /* A call-back will trigger config_clear again*/
    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_ERROR_NO_MEM);
    mp_mem_listener->callback(mp_mem_listener->p_args);
    TEST_ASSERT_EQUAL(3, m_listener_register_calls);

    flash_manager_remove_ExpectAndReturn(mp_flash_manager, NRF_SUCCESS);
    mp_mem_listener->callback(mp_mem_listener->p_args);
    TEST_ASSERT_EQUAL(3, m_listener_register_calls);

    /* test config store */
    /* config_store should do nothing since the module is waiting for a remove_complete_cb callback, but it will register with the listener */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    access_flash_config_store();
    TEST_ASSERT_EQUAL(4, m_listener_register_calls);
    /** Send an erase complete event, we should be able to make config store calls then. */
    m_flash_manager_config.remove_complete_cb(mp_flash_manager);

    flash_manager_entry_alloc_ExpectAndReturn(mp_flash_manager, FLASH_HANDLE_METADATA, sizeof(access_flash_metadata_t), NULL);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    mp_mem_listener->callback(mp_mem_listener->p_args);
    TEST_ASSERT_EQUAL(5, m_listener_register_calls);

    /* A call-back will trigger config_store */
    /* Only metadata will be stored since everything else has been cleared*/
    (void) expect_flash_manager_entry(FLASH_HANDLE_METADATA, sizeof(access_flash_metadata_t));
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    mp_mem_listener->callback(mp_mem_listener->p_args);

    /* Since we stored everything the callback should do nothing: */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    mp_mem_listener->callback(mp_mem_listener->p_args);
}

void test_subscription_dealloc_and_dealloc_share_combination(void)
{
    uint16_t i;

    /* Ensure access layer is stable */
    m_flash_manager_config.write_complete_cb(mp_flash_manager, NULL, FM_RESULT_SUCCESS);
    printf("ACCESS_SUBSCRIPTION_LIST_COUNT: %d\n",ACCESS_SUBSCRIPTION_LIST_COUNT);

    /* Allocate a model */
    access_model_add_params_t init_params;
    access_model_handle_t handle;
    uint8_t model_count;
    uint16_t dealloc_index;

    for (i = 0; i < ACCESS_MODEL_COUNT; i++)
    {
        init_params.element_index = 0;
        init_params.model_id.model_id = TEST_MODEL_ID + i;
        init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
        init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
        init_params.opcode_count = OPCODE_COUNT;
        init_params.p_args = TEST_REFERENCE;
        init_params.publish_timeout_cb = publish_timeout_cb;

        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle));
        TEST_ASSERT_EQUAL(i, handle);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_sig_model_count_get(0, &model_count));
        TEST_ASSERT_EQUAL(i + 1, model_count);

        /* Allocate subscription list for all, except first three, for testing purpose */
        if (i > 2)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(handle));
        }
    }

    init_params.model_id.model_id = TEST_MODEL_ID + i;
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_add(&init_params, &handle));

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_add(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_remove(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_add(1, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_subscription_remove(1, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, access_model_subscription_lists_share(0, 1));

    /* Allocate subscription list for 0, and 1 */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(1));
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_subscription_list_alloc(2));

    flash_manager_entry_count_get_IgnoreAndReturn(0);
    /* Test: Share subscription list: owner:0, other:1, and then allocate for third one */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(0, 1));
    dealloc_index = m_sub_list_dealloc_index;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(2));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(2));

    /* Test: De-alloc subscription list of `2` multiple times and try reallocating it. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(2));
    TEST_ASSERT_EQUAL(dealloc_index, m_sub_list_dealloc_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(2));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(2));
    TEST_ASSERT_EQUAL(dealloc_index, m_sub_list_dealloc_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(2));
    TEST_ASSERT_EQUAL(dealloc_index, m_sub_list_dealloc_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(2));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(2));

    /* Test: De-alloc subscription list for owner:0, and try sharing */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(0));
    dealloc_index = m_sub_list_dealloc_index;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, access_model_subscription_lists_share(0, 1));

    /* Test: Try Alloc subscription list for other:1 */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(1));

    /* Test: Try de-alloc for other:1, alloc subscription list for owner:0, and try sharing */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(1));
    TEST_ASSERT_TRUE(dealloc_index == m_sub_list_dealloc_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(0, 1));

    /* Test: De-alloc subscription list for owner:0 and other:1 and try sharing */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(1));
    TEST_ASSERT_TRUE(dealloc_index == m_sub_list_dealloc_index);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, access_model_subscription_lists_share(0, 1));

    /* Test: De-alloc fails, if subscription list entry is found in the flash */
    uint8_t entry_buffer[UINT8_MAX];
    fm_entry_t * p_entry = (fm_entry_t *) entry_buffer;

    flash_manager_entry_count_get_IgnoreAndReturn(1);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_subscription_list_dealloc(2));

    /* Test: De-alloc fails, access is storing its data */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();

    flash_manager_entry_alloc_IgnoreAndReturn(p_entry);
    for (uint32_t i = 0; i < ACCESS_SUBSCRIPTION_LIST_COUNT; i++)
    {
        flash_manager_entry_commit_Ignore();
    }
    for (uint32_t i = 0; i < ACCESS_ELEMENT_COUNT; i++)
    {
        flash_manager_entry_alloc_IgnoreAndReturn(p_entry);
        flash_manager_entry_commit_Ignore();
    }
    for (uint32_t i = 0; i < ACCESS_MODEL_COUNT; i++)
    {
        flash_manager_entry_alloc_IgnoreAndReturn(p_entry);
        flash_manager_entry_commit_Ignore();
    }
    flash_manager_mem_listener_register_Ignore();
    access_flash_config_store();

    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_subscription_list_dealloc(2));

    m_flash_manager_config.write_complete_cb(mp_flash_manager, NULL, FM_RESULT_SUCCESS);
    flash_manager_entry_count_get_IgnoreAndReturn(0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(2));

}

