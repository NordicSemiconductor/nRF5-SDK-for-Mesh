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
#include "access_utils.h"

#include "access_publish_mock.h"
#include "access_reliable_mock.h"
#include "access_publish_retransmission_mock.h"

#include "mesh_opt_access.h"
#include "nrf_mesh_config_app.h"

#include "device_state_manager_mock.h"
#include "mesh_config_entry_mock.h"
#include "mesh_config_mock.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_utils_mock.h"
#include "event_mock.h"
#include "bearer_event_mock.h"
#include "proxy_mock.h"
#include "manual_mock_queue.h"
#include "mesh_mem_mock.h"
#include "nrf_mesh_externs_mock.h"

#define TEST_REFERENCE ((void*) 0xB00BB00B)
#define TEST_MODEL_ID (0xB00B)

#define SOURCE_ADDRESS (0x0001)
#define ELEMENT_ADDRESS_START (0x0011)
#define GROUP_ADDRESS_START   (0xC001)
#define PUBLISH_ADDRESS_START (0x0101)
#define OPCODE_COUNT (5)

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

#define EXPECT(name, times) \
    do { \
        for (size_t i = 0; i < times; i++) \
        { \
            uint32_t value = 0; \
            name##_Expect(&value); \
        } \
    } while(0)

/*******************************************************************************
 * Sample data from @tagMeshSp
 *******************************************************************************/

/* Message #21 (@tagMeshSp page 309) */
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
    uint8_t ttl;
} tx_evt_t;

typedef enum
{
    TX_SECMAT_TYPE_MASTER,
    TX_SECMAT_TYPE_FRIENDSHIP,
} tx_secmat_type_t;

typedef enum
{
    SUBLIST_ALLOCATE,
    SUBLIST_ADD,
    SUBLIST_REMOVE,
} subscription_list_action_t;

typedef struct
{
    access_model_handle_t model_handle;
    uint16_t sublist_index;
} persistent_indexes_t;

typedef struct
{
    bool is_triggered;
    access_model_state_data_t model_info;
} test_model_t;

typedef struct
{
    bool is_triggered;
    access_flash_subscription_list_t sublist_info;
} test_sublist_t;

typedef struct
{
    bool is_triggered;
    uint16_t element_info;
} test_element_t;

typedef struct
{
    bool is_triggered;
} test_metadata_t;

typedef struct
{
    bool is_triggered;
    uint8_t default_ttl;
} test_default_ttl_t;

/*******************************************************************************
 * Static Variables
 *******************************************************************************/
static access_opcode_handler_t m_opcode_handlers[ACCESS_MODEL_COUNT][OPCODE_COUNT];

static list_node_t * mp_evt_handler;

static msg_evt_t m_msg_evt_buffer[MSG_EVT_MAX_COUNT];
static fifo_t m_msg_fifo;

static tx_evt_t m_tx_evt_buffer[TX_EVT_MAX_COUNT];
static fifo_t m_tx_fifo;

static nrf_mesh_address_t m_addresses[DSM_ADDR_MAX];

const uint32_t SUBSCRIPTION_ADDRESS_COUNT = DSM_ADDR_MAX - ACCESS_MODEL_COUNT - ACCESS_ELEMENT_COUNT;
dsm_local_unicast_address_t local_addresses = {ELEMENT_ADDRESS_START, ACCESS_ELEMENT_COUNT};

static uint32_t m_dsm_tx_friendship_secmat_get_retval = NRF_SUCCESS;
static uint16_t m_sub_list_dealloc_index;

MOCK_QUEUE_DEF(access_publish_retransmission_message_add_mock, access_publish_retransmit_t, NULL);
MOCK_QUEUE_DEF(mesh_mem_free_mock, uintptr_t, NULL);

static test_model_t    m_expected_model_info[ACCESS_MODEL_COUNT];
static test_sublist_t  m_expected_subs_list[ACCESS_SUBSCRIPTION_LIST_COUNT];
static test_element_t  m_expected_element_location[ACCESS_ELEMENT_COUNT];
static test_metadata_t m_expected_metadata;
static test_default_ttl_t m_expected_default_ttl;

static persistent_indexes_t m_pst_indexes =
{
    .model_handle = ACCESS_HANDLE_INVALID,
    .sublist_index = ACCESS_HANDLE_INVALID
};

/*****************************************************************************
* Extern stubs
*****************************************************************************/
extern const mesh_config_entry_params_t m_access_metadata_params;
extern const mesh_config_entry_params_t m_subscriptions_params;
extern const mesh_config_entry_params_t m_elements_params;
extern const mesh_config_entry_params_t m_models_params;
extern const mesh_config_entry_params_t m_default_ttl_params;

/*******************************************************************************
 * Helper Functions // Mocks // Callbacks
 *******************************************************************************/
static uint32_t entry_set_cb(mesh_config_entry_id_t id, const void* p_entry, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_EQUAL(MESH_OPT_ACCESS_FILE_ID, id.file);

    if (IS_IN_RANGE(id.record, MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD,
                    MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD + ACCESS_SUBSCRIPTION_LIST_COUNT - 1))
    {
        uint16_t idx = id.record - MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD;
        access_flash_subscription_list_t * p_data = (access_flash_subscription_list_t *)p_entry;
        TEST_ASSERT_TRUE(m_expected_subs_list[idx].is_triggered);
        m_expected_subs_list[idx].is_triggered = false;
        TEST_ASSERT_EQUAL_UINT32_ARRAY(m_expected_subs_list[idx].sublist_info.inverted_bitfield,
                                       p_data->inverted_bitfield,
                                       ARRAY_SIZE(p_data->inverted_bitfield));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_subscriptions_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_ACCESS_ELEMENTS_RECORD,
                    MESH_OPT_ACCESS_ELEMENTS_RECORD + ACCESS_ELEMENT_COUNT - 1))
    {
        uint16_t idx = id.record - MESH_OPT_ACCESS_ELEMENTS_RECORD;
        uint16_t * p_data = (uint16_t *)p_entry;
        TEST_ASSERT_TRUE(m_expected_element_location[idx].is_triggered);
        m_expected_element_location[idx].is_triggered = false;
        TEST_ASSERT_EQUAL_UINT16(m_expected_element_location[idx].element_info, *p_data);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_elements_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_ACCESS_MODELS_RECORD,
                    MESH_OPT_ACCESS_MODELS_RECORD + ACCESS_MODEL_COUNT - 1))
    {
        uint16_t idx = id.record - MESH_OPT_ACCESS_MODELS_RECORD;
        access_model_state_data_t * p_data = (access_model_state_data_t *)p_entry;
        TEST_ASSERT_TRUE(m_expected_model_info[idx].is_triggered);
        m_expected_model_info[idx].is_triggered = false;
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.model_id.company_id, p_data->model_id.company_id);
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.model_id.model_id, p_data->model_id.model_id);
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.publish_appkey_handle, p_data->publish_appkey_handle);
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.publish_address_handle, p_data->publish_address_handle);
        TEST_ASSERT_EQUAL_UINT32_ARRAY(m_expected_model_info[idx].model_info.application_keys_bitfield,
                                       p_data->application_keys_bitfield,
                                       BITFIELD_BLOCK_COUNT(DSM_APP_MAX + DSM_DEVICE_MAX));
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.element_index, p_data->element_index);
        TEST_ASSERT_EQUAL_UINT16(m_expected_model_info[idx].model_info.subscription_pool_index, p_data->subscription_pool_index);
        TEST_ASSERT_EQUAL(m_expected_model_info[idx].model_info.friendship_credential_flag, p_data->friendship_credential_flag);
        TEST_ASSERT_EQUAL_UINT8(m_expected_model_info[idx].model_info.publish_ttl, p_data->publish_ttl);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_expected_model_info[idx].model_info.publication_period,
                                     &p_data->publication_period, sizeof(access_publish_period_t));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_expected_model_info[idx].model_info.publication_retransmit,
                                     &p_data->publication_retransmit, sizeof(access_publish_retransmit_t));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_models_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (id.record == MESH_OPT_ACCESS_METADATA_RECORD)
    {
        access_flash_metadata_t * p_metadata = (access_flash_metadata_t *)p_entry;

        TEST_ASSERT_TRUE(m_expected_metadata.is_triggered);
        m_expected_metadata.is_triggered = false;

        TEST_ASSERT_EQUAL_UINT16(ACCESS_ELEMENT_COUNT, p_metadata->element_count);
        TEST_ASSERT_EQUAL_UINT16(ACCESS_MODEL_COUNT, p_metadata->model_count);
        TEST_ASSERT_EQUAL_UINT16(ACCESS_SUBSCRIPTION_LIST_COUNT, p_metadata->subscription_list_count);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_access_metadata_params.callbacks.setter(id, p_entry));

        return NRF_SUCCESS;
    }

    if (id.record == MESH_OPT_ACCESS_DEFAULT_TTL_RECORD)
    {
        uint8_t * p_ttl = (uint8_t *)p_entry;
        TEST_ASSERT_TRUE(m_expected_default_ttl.is_triggered);
        m_expected_default_ttl.is_triggered = false;

        TEST_ASSERT_EQUAL(m_expected_default_ttl.default_ttl, *p_ttl);
        m_default_ttl_params.callbacks.setter(id, p_ttl);

        return NRF_SUCCESS;
    }

    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

static void access_reliable_cancel_all_cb(int num_calls)
{
    (void)num_calls;

    void * p_args;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_p_args_get(0, &p_args));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_p_args_get(1, &p_args));
}

static void model_change_wrapper(access_model_handle_t handle, access_model_add_params_t * p_param)
{
    access_model_state_data_t * p_data = &m_expected_model_info[handle].model_info;

    m_expected_model_info[handle].is_triggered = true;

    if (p_param != NULL)
    {
        memset(p_data, 0, sizeof(access_model_state_data_t));
        p_data->publish_address_handle = DSM_HANDLE_INVALID;
        p_data->publish_appkey_handle = DSM_HANDLE_INVALID;
        p_data->element_index = p_param->element_index;
        p_data->model_id.model_id = p_param->model_id.model_id;
        p_data->model_id.company_id = p_param->model_id.company_id;
        p_data->publish_ttl = ACCESS_TTL_USE_DEFAULT;
        p_data->subscription_pool_index = ACCESS_SUBSCRIPTION_LIST_COUNT;
    }
}

static void sublist_change_wrapper(uint16_t index, access_model_handle_t model_handle, dsm_handle_t address_handle, subscription_list_action_t action)
{
    access_flash_subscription_list_t * p_data = &m_expected_subs_list[index].sublist_info;
    m_expected_subs_list[index].is_triggered = true;

    switch (action)
    {
        case SUBLIST_ALLOCATE:
            memset(p_data, 0xFF, sizeof(access_flash_subscription_list_t));
            m_expected_model_info[model_handle].is_triggered = true;
            m_expected_model_info[model_handle].model_info.subscription_pool_index = index;
            break;
        case SUBLIST_ADD:
            bitfield_clear(p_data->inverted_bitfield, address_handle);
            break;
        case SUBLIST_REMOVE:
            bitfield_set(p_data->inverted_bitfield, address_handle);
            break;
    }
}

static void element_change_wrapper(uint16_t index, uint16_t location)
{
    m_expected_element_location[index].is_triggered = true;
    m_expected_element_location[index].element_info = location;
}

static void not_triggered_verify(void)
{
    for (uint16_t idx = 0; idx < ACCESS_SUBSCRIPTION_LIST_COUNT; idx++)
    {
        TEST_ASSERT_FALSE_MESSAGE(m_expected_subs_list[idx].is_triggered, "Not triggered subscription list");
    }

    for (uint16_t idx = 0; idx < ACCESS_ELEMENT_COUNT; idx++)
    {
        TEST_ASSERT_FALSE_MESSAGE(m_expected_element_location[idx].is_triggered, "Not triggered element");
    }

    for (uint16_t idx = 0; idx < ACCESS_MODEL_COUNT; idx++)
    {
        TEST_ASSERT_FALSE_MESSAGE(m_expected_model_info[idx].is_triggered, "Not triggered model");
    }

    TEST_ASSERT_FALSE_MESSAGE(m_expected_metadata.is_triggered, "Not triggered metadata");
}

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
    access_model_handle_t handle = m_pst_indexes.model_handle;
    access_model_handle_t gotten_handle = ACCESS_HANDLE_INVALID;

    model_change_wrapper(++handle, &p_test_data->add_params);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&p_test_data->add_params, &gotten_handle));
    m_pst_indexes.model_handle = gotten_handle;
    TEST_ASSERT_EQUAL(handle, m_pst_indexes.model_handle);

    if (p_test_data->subscription_list_share_index == UINT32_MAX)
    {
        sublist_change_wrapper(++m_pst_indexes.sublist_index, handle, 0, SUBLIST_ALLOCATE);
        model_change_wrapper(handle, NULL);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(handle));
    }
    else
    {
        m_expected_model_info[handle].model_info.subscription_pool_index =
                m_expected_model_info[p_test_data->subscription_list_share_index].model_info.subscription_pool_index;
        model_change_wrapper(handle, NULL);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(p_test_data->subscription_list_share_index, handle));
    }

    return handle;
}

static void update_test_model(access_flash_test_struct_t * p_test_data, access_model_handle_t model_handle, bool expect_publish_period_set)
{
    for (uint32_t subs = 0; subs < p_test_data->number_of_subscription_handles; ++subs)
    {
        sublist_change_wrapper(m_expected_model_info[model_handle].model_info.subscription_pool_index, model_handle,
                               p_test_data->p_subsciption_address_handles[subs], SUBLIST_ADD);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_add(model_handle, p_test_data->p_subsciption_address_handles[subs]));
    }

    for (uint32_t apps = 0; apps < p_test_data->number_of_appkey_handles; ++apps)
    {
        bitfield_set(m_expected_model_info[model_handle].model_info.application_keys_bitfield, p_test_data->p_appkey_handles[apps]);
        model_change_wrapper(model_handle, NULL);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(model_handle, p_test_data->p_appkey_handles[apps]));
    }

    m_expected_model_info[model_handle].model_info.publish_address_handle = p_test_data->publish_address_handle;
    model_change_wrapper(model_handle, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(model_handle, p_test_data->publish_address_handle));

    m_expected_model_info[model_handle].model_info.friendship_credential_flag = p_test_data->credential_flag;
    if (p_test_data->credential_flag)
    {
        model_change_wrapper(model_handle, NULL);
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_friendship_credential_flag_set(model_handle, p_test_data->credential_flag));

    m_expected_model_info[model_handle].model_info.publish_ttl = p_test_data->publish_ttl;
    model_change_wrapper(model_handle, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(model_handle, p_test_data->publish_ttl));

    if (expect_publish_period_set)
    {
        access_publish_period_set_Expect(NULL, (access_publish_resolution_t) p_test_data->publish_period.step_res, p_test_data->publish_period.step_num);
        access_publish_period_set_IgnoreArg_p_pubstate();
    }

    m_expected_model_info[model_handle].model_info.publication_period.step_num = p_test_data->publish_period.step_num;
    m_expected_model_info[model_handle].model_info.publication_period.step_res = (access_publish_resolution_t) p_test_data->publish_period.step_res;
    model_change_wrapper(model_handle, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(model_handle, (access_publish_resolution_t) p_test_data->publish_period.step_res,
        p_test_data->publish_period.step_num));

    m_expected_model_info[model_handle].model_info.publish_appkey_handle = p_test_data->publish_appkey_handle;
    model_change_wrapper(model_handle, NULL);
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

static void evt_handler_add_stub(nrf_mesh_evt_handler_t * p_evt_handler, int num_calls)
{
    list_add(&mp_evt_handler, &p_evt_handler->node);
}

static void evt_handler_remove_stub(nrf_mesh_evt_handler_t * p_evt_handler, int num_calls)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, list_remove(&mp_evt_handler, &p_evt_handler->node));
}

static void evt_notify(nrf_mesh_evt_t * p_evt)
{
    LIST_FOREACH(p_node, mp_evt_handler)
    {
        nrf_mesh_evt_handler_t * p_evt_handler = PARENT_BY_FIELD_GET(nrf_mesh_evt_handler_t, node, p_node);
        p_evt_handler->evt_cb(p_evt);
    }
}

static uint32_t packet_tx_stub(const nrf_mesh_tx_params_t * p_tx_params, uint32_t * const p_ref, int num_calls)
{
    tx_evt_t tx_evt;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&m_tx_fifo, &tx_evt));
    TEST_ASSERT_EQUAL(tx_evt.length, p_tx_params->data_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tx_evt.p_data, p_tx_params->p_data, tx_evt.length);
    TEST_ASSERT_EQUAL_HEX16(tx_evt.src, p_tx_params->src);
    TEST_ASSERT_EQUAL_HEX16(tx_evt.dst, p_tx_params->dst.value);
    TEST_ASSERT_EQUAL_HEX16(tx_evt.ttl, p_tx_params->ttl);
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

static void retransmission_Expect(access_model_handle_t model_handle)
{
    access_publish_retransmit_t expected_publish_retransmit;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_get(model_handle,
                                                                       &expected_publish_retransmit));
    access_publish_retransmission_message_add_mock_Expect(&expected_publish_retransmit);
}

static void access_publish_retransmission_message_add_stub(access_model_handle_t model_handle,
                                                           const access_publish_retransmit_t *p_publish_retransmit,
                                                           const access_message_tx_t *p_tx_message,
                                                           const uint8_t *p_access_payload,
                                                           uint16_t access_payload_length,
                                                           int num_calls)
{
    UNUSED_VARIABLE(model_handle);
    UNUSED_VARIABLE(p_tx_message);
    UNUSED_VARIABLE(access_payload_length);
    UNUSED_VARIABLE(num_calls);

    access_publish_retransmit_t expected_publish_retransmit;
    access_publish_retransmission_message_add_mock_Consume(&expected_publish_retransmit);

    TEST_ASSERT_EQUAL_MEMORY(&expected_publish_retransmit,
                             p_publish_retransmit,
                             sizeof(access_publish_retransmit_t));

    TEST_ASSERT(NULL != p_access_payload);

    mesh_mem_free((uint8_t*) p_access_payload);
}

void * mesh_mem_alloc_mock(size_t size, int num_calls)
{
    UNUSED_VARIABLE(num_calls);

    void* p_buffer = malloc(size);

    uintptr_t expected_value = (uintptr_t) p_buffer;
    mesh_mem_free_mock_Expect(&expected_value);

    return p_buffer;
}

void mesh_mem_free_mock(void * ptr, int num_calls)
{
    UNUSED_VARIABLE(num_calls);

    /* No way to check pointer here because of reordering malloc/free in
     * the code */
    uintptr_t expected_value;
    mesh_mem_free_mock_Consume(&expected_value);

    free(ptr);
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

static uint8_t expected_tx_ttl_get(access_model_handle_t handle)
{
    uint8_t model_ttl;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_get(handle, &model_ttl));

    if (model_ttl == ACCESS_TTL_USE_DEFAULT)
    {
        uint8_t ttl = access_default_ttl_get();
        TEST_ASSERT_EQUAL(m_expected_default_ttl.default_ttl, ttl);
        return ttl;
    }

    return model_ttl;
}

static uint8_t expected_response_ttl_get(access_model_handle_t handle, uint8_t rx_ttl)
{
    uint8_t model_ttl;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_get(handle, &model_ttl));

    if (rx_ttl == 0)
    {
        return 0;
    }

    return expected_tx_ttl_get(handle);
}

static inline void expect_tx(const uint8_t * p_data, uint32_t length, uint16_t src, uint16_t dst,
                             uint8_t ttl, dsm_handle_t appkey_handle, dsm_handle_t subnet_handle,
                             tx_secmat_type_t tx_secmat_type)
{
    tx_evt_t tx_evt;
    tx_evt.p_data = p_data;
    tx_evt.length = length;
    tx_evt.src = src;
    tx_evt.dst = dst;
    tx_evt.ttl = ttl;

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

    nrf_mesh_is_address_rx_ExpectAndReturn(&mesh_evt.params.message.dst, true);

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

    evt_notify(&mesh_evt);
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

    /* Expect correct ttl */
    uint8_t ttl = expected_response_ttl_get(handle, p_message->meta_data.ttl);

    /* Never loop back the response: */
    expect_tx(expected_data,
                length,
                ELEMENT_ADDRESS_START + handle * ACCESS_ELEMENT_COUNT / ACCESS_MODEL_COUNT,
                p_message->meta_data.src.value,
                ttl,
                p_message->meta_data.appkey_handle,
                p_message->meta_data.subnet_handle,
                friendship_credentials_flag ? TX_SECMAT_TYPE_FRIENDSHIP : TX_SECMAT_TYPE_MASTER);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reply(handle, p_message, &reply));
}


/*******************************************************************************
 * Test Setup
 *******************************************************************************/
/** Distributes models across the number of elements. */
static void build_device_setup(uint32_t elem_count, uint32_t model_count)
{
    /* Ensure there are enough models to test ttl variations */
    TEST_ASSERT_TRUE(model_count > 2);

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
            uint8_t ttl = 129; //invalid dummy value
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_get(handle, &ttl));
            TEST_ASSERT_EQUAL(ACCESS_TTL_USE_DEFAULT, ttl);

            /* Re-adding the same model isn't allowed. */
            TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_add(&init_params, &handle));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_application_bind(handle, 0));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_application_set(handle, 0));
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_address_set(handle, ACCESS_ELEMENT_COUNT + handle));

            /* Add various publish ttl values */
            if (i % 3 == 0)
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(handle, 0));
            }
            else if (i % 3 == 1)
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(handle, ACCESS_TTL_USE_DEFAULT));
            }
            else
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(handle, i));
            }

            if (handle == (ACCESS_MODEL_COUNT - 1))
            {
                /* We compile the test with one list less than the number of models. */
                TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_subscription_list_alloc(handle));
            }
            else
            {
                TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(handle));
            }

            access_publish_retransmit_t publish_retransmit =
            {
                .count = 1,
                .interval_steps = 1,
            };

            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_set(handle, publish_retransmit));

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
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();
    nrf_mesh_mock_Init();
    nrf_mesh_events_mock_Init();
    nrf_mesh_utils_mock_Init();
    event_mock_Init();
    bearer_event_mock_Init();
    access_publish_mock_Init();
    access_publish_retransmission_mock_Init();
    proxy_mock_Init();
    mesh_mem_mock_Init();
    access_publish_retransmission_message_add_mock_Init();
    mesh_mem_free_mock_Init();
    nrf_mesh_externs_mock_Init();

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

    nrf_mesh_is_device_provisioned_IgnoreAndReturn(false);
    mesh_config_entry_set_StubWithCallback(entry_set_cb);

    mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_mock);
    mesh_mem_free_StubWithCallback(mesh_mem_free_mock);

    nrf_mesh_evt_handler_add_StubWithCallback(evt_handler_add_stub);
    nrf_mesh_evt_handler_remove_StubWithCallback(evt_handler_remove_stub);
    access_publish_retransmission_message_add_StubWithCallback(access_publish_retransmission_message_add_stub);
    access_reliable_init_Expect();
    access_publish_init_Expect();
    access_publish_retransmission_init_Expect();
    access_init();

    m_dsm_tx_friendship_secmat_get_retval = NRF_SUCCESS;
    m_expected_default_ttl.default_ttl = ACCESS_DEFAULT_TTL;
}

void tearDown(void)
{
    mp_evt_handler = NULL;
    TEST_ASSERT(fifo_is_empty(&m_msg_fifo));
    TEST_ASSERT(fifo_is_empty(&m_tx_fifo));
    event_mock_Verify();
    bearer_event_mock_Verify();
    access_publish_mock_Verify();
    device_state_manager_mock_Verify();
    access_reliable_mock_Verify();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
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
    mesh_mem_mock_Verify();
    mesh_mem_mock_Destroy();
    access_publish_retransmission_message_add_mock_Verify();
    access_publish_retransmission_message_add_mock_Destroy();
    mesh_mem_free_mock_Verify();
    mesh_mem_free_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
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

    /* Add model after provisioning */
    nrf_mesh_is_device_provisioned_IgnoreAndReturn(true);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_add(&init_params, &handle));
    nrf_mesh_is_device_provisioned_IgnoreAndReturn(false);

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
     * @tagMeshSp section 4.2.2.4:
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

typedef uint32_t (publication_cb)(access_model_handle_t handle,
                                    access_message_tx_t *p_tx_message);

static void publication_tests(publication_cb *p_pub_func, bool with_retransmission)
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

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, p_pub_func(ACCESS_MODEL_COUNT, &message));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, p_pub_func(0, &message));
    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    message.opcode.opcode = 0x7f;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, p_pub_func(0, &message));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, p_pub_func(0, &message));

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

        uint8_t ttl = expected_tx_ttl_get(i);
        expect_tx(expected_data, length, src, dst, ttl, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

        if (with_retransmission)
        {
            /* Check re-transmission */
            retransmission_Expect(i);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, p_pub_func(i, &message));
    }

    /* Test with friendship secmat */
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

    uint8_t ttl = expected_tx_ttl_get(0);
    expect_tx(expected_data, length, src, dst, ttl, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_FRIENDSHIP);

    if (with_retransmission)
    {
        /* Check re-transmission */
        retransmission_Expect(0);
    }

    TEST_ASSERT_EQUAL(NRF_SUCCESS, p_pub_func(0, &message));

    /* Test with frienship secmat
     *
     * @tagMeshSp section 4.2.2.4:
     *
     * When Publish Friendship Credential Flag is set to 1 and the friendship security material is
     * not available, the master security material shall be used. */
    m_dsm_tx_friendship_secmat_get_retval = NRF_ERROR_NOT_FOUND;
    expect_tx(expected_data, length, src, dst, ttl, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_FRIENDSHIP);

    if (with_retransmission)
    {
        /* Check re-transmission */
        retransmission_Expect(0);
    }

    TEST_ASSERT_EQUAL(NRF_SUCCESS, p_pub_func(0, &message));
}

static uint32_t access_model_publish_publication_cb(access_model_handle_t handle,
                                                    access_message_tx_t *p_tx_message)
{
    return access_model_publish(handle, p_tx_message);
}

static uint32_t access_packet_tx_publication_cb(access_model_handle_t handle,
                                                access_message_tx_t *p_tx_message)
{
    /* Prepare access payload */
    uint16_t opcode_length = access_utils_opcode_size_get(p_tx_message->opcode);
    uint16_t payload_length = opcode_length + p_tx_message->length;
    uint8_t payload[payload_length];

    opcode_raw_write(p_tx_message->opcode, payload);
    memcpy(&payload[opcode_length], p_tx_message->p_buffer, p_tx_message->length);

    return access_packet_tx(handle, p_tx_message, payload, payload_length);
}

void test_access_model_publish(void)
{
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish(0, NULL));

    access_message_tx_t message;
    message.length = ACCESS_MESSAGE_LENGTH_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_publish(0, &message));

    publication_tests(access_model_publish_publication_cb, true);
}

void test_access_packet_tx(void)
{
    const uint8_t data[] = "Hello, World";
    const access_opcode_t opcode = ACCESS_OPCODE_SIG(0x0040);
    access_message_tx_t message;
    message.opcode.opcode = opcode.opcode;
    message.opcode.company_id = opcode.company_id;
    message.p_buffer = data;
    message.length = sizeof(data);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_packet_tx(0, NULL, data, sizeof(data)));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_packet_tx(0, &message, NULL, sizeof(data)));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_packet_tx(0, &message, data, 0));

    message.length = ACCESS_MESSAGE_LENGTH_MAX;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, access_model_publish(0, &message));

    publication_tests(access_packet_tx_publication_cb, false);
}

void test_publish_retransmit(void)
{
    access_publish_retransmit_t publish_retransmit_out = {};
    access_publish_retransmit_t publish_retransmit_in;
    publish_retransmit_in.count = 0;
    publish_retransmit_in.interval_steps = 1;

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_retransmit_set(0, publish_retransmit_in));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_publish_retransmit_get(0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_retransmit_get(0, &publish_retransmit_out));

    build_device_setup(ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT);

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_retransmit_set(ACCESS_HANDLE_INVALID, publish_retransmit_in));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_retransmit_get(ACCESS_HANDLE_INVALID, &publish_retransmit_out));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_set(0, publish_retransmit_in));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_get(0, &publish_retransmit_out));

    TEST_ASSERT_EQUAL_MEMORY(&publish_retransmit_in,
                             &publish_retransmit_out,
                             sizeof(access_publish_retransmit_t));

    /* Test retransmission called */
    const uint8_t data[] = "Hello, World";
    const access_opcode_t opcode = ACCESS_OPCODE_SIG(0x0040);
    access_message_tx_t message;
    message.opcode.opcode = opcode.opcode;
    message.opcode.company_id = opcode.company_id;
    message.p_buffer = data;
    message.length = sizeof(data);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    publish_retransmit_in.count = 2;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_set(0, publish_retransmit_in));
    retransmission_Expect(0);

    uint16_t src = ELEMENT_ADDRESS_START;
    uint16_t dst = PUBLISH_ADDRESS_START;

    uint8_t expected_data[sizeof(data) + sizeof(uint32_t)];
    uint32_t length = opcode_raw_write(message.opcode, expected_data);
    memcpy(&expected_data[length], data, message.length);
    length += message.length;

    uint8_t ttl = expected_tx_ttl_get(0);
    expect_tx(expected_data, length, src, dst, ttl, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish(0, &message));

    /* No retransmit, free shall be called */
    publish_retransmit_in.count = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_retransmit_set(0, publish_retransmit_in));

    expect_tx(expected_data, length, src, dst, ttl, 0, DSM_HANDLE_INVALID, TX_SECMAT_TYPE_MASTER);

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
    init_params.opcode_count = ARRAY_SIZE(opcode_handlers);
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

        uint8_t ttl = expected_tx_ttl_get(0);
        expect_tx(access_payload, sizeof(access_payload), src, dst, ttl, 0, DSM_HANDLE_INVALID, false);

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

        nrf_mesh_is_address_rx_ExpectAndReturn(&mesh_evt.params.message.dst, true);
        dsm_local_unicast_addresses_get_Expect(NULL);
        dsm_local_unicast_addresses_get_IgnoreArg_p_address();
        dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&local_addresses);
        dsm_appkey_handle_get_ExpectAndReturn(NULL, 0);
        dsm_subnet_handle_get_ExpectAndReturn(NULL, 0);

        evt_notify(&mesh_evt);
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
    evt_notify(&evt);

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

    /* Allocation test */
    tx_message.opcode.opcode = 0x00;
    tx_message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    mesh_mem_alloc_StubWithCallback(NULL);
    mesh_mem_alloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_publish(0, &tx_message));
    mesh_mem_alloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, access_model_reply(0, &rx_message, &tx_message));
    mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_mock);

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

    resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
    step_number = 20;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_publish_period_set(1, resolution, step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_publish_period_get(1, &resolution, &step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_period_set(0, (access_publish_resolution_t) (ACCESS_PUBLISH_RESOLUTION_MAX + 1), step_number));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_period_set(0, (access_publish_resolution_t) 0, ACCESS_PUBLISH_PERIOD_STEP_MAX + 1));

    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(0, resolution, step_number));

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

static void update_reloaded_publish_params(access_flash_test_struct_t * p_test_vector)
{
    if (ACCESS_MODEL_PUBLISH_PERIOD_RESTORE == 0)
    {
        p_test_vector->publish_period.step_res = 0;
        p_test_vector->publish_period.step_num = 0;
    }
}

void access_layer_reset(void)
{
    mp_evt_handler = NULL;
    access_publish_init_Expect();
    access_reliable_init_Expect();
    access_publish_retransmission_init_Expect();
    access_init();
    /* Check that elements are un populated */
    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    uint16_t models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(0, models_count);
}

void test_config_load_reload(void)
{
    /* Build some models, verify their content, store in "flash", wipe access layer state,
     * add the same models again and restore and check the expected state.*/

    /************************************ Build test vectors *************************************/
    dsm_handle_t appkeys[] = {0x0,0x1,0x2,0x3,0x4,0x5};
    dsm_handle_t addresses[] = {0x6,0x7,0x8,0x9,0xA};
    uint32_t sizeof_addresses = ARRAY_SIZE(addresses);
    uint32_t sizeof_appkeys = ARRAY_SIZE(appkeys);
    uint32_t address_bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
    access_flash_metadata_t metadata = {ACCESS_SUBSCRIPTION_LIST_COUNT, ACCESS_ELEMENT_COUNT, ACCESS_MODEL_COUNT};
    bitfield_set_all(address_bitfield, DSM_ADDR_MAX);
    TEST_ASSERT_EQUAL(address_bitfield[0], UINT32_MAX);
    for (uint32_t i = 0; i < sizeof_addresses; ++i)
    {
        bitfield_clear(address_bitfield, addresses[i]);
    }
    access_model_id_t model_id1 = {0x1337, 0x15};
    access_model_id_t model_id2 = {0xC000, 0x0001};
    access_publish_period_t pub_period1 = {ACCESS_PUBLISH_RESOLUTION_100MS, 50};
    access_publish_period_t pub_period2 = {(1<<ACCESS_PUBLISH_STEP_RES_BITS) - 1, (1<<ACCESS_PUBLISH_STEP_NUM_BITS) - 1};

    /*lint -save -e651 -e64 Disregard warnings about confusing initializers and type mismatches caused by the macros. */
    access_flash_test_struct_t test_vector[] =
    {
        FLASH_TEST_VECTOR_INSTANCE(model_id1, 0, &addresses[0], 2, UINT32_MAX, 0xA, pub_period1, &appkeys[0], 3, 0x1, 4, 0),
        FLASH_TEST_VECTOR_INSTANCE(model_id2, 1, &addresses[2], sizeof_addresses -2, 0, 0xC, pub_period2, &appkeys[3], sizeof_appkeys - 3, 0x0, 127, 1)
    };
    uint32_t test_vector_size = ARRAY_SIZE(test_vector);
    /*lint -restore */

   /***************** Start from the scratch. *****************/
   /****************** Add models and configure as specified by the test vector. ******************/
    m_expected_metadata.is_triggered = true;

    access_model_handle_t model_handle[test_vector_size];
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        model_handle[i] = init_test_model_and_subs_list(&test_vector[i]);
    }

    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_load_config_apply());
    not_triggered_verify();

    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        update_test_model(&test_vector[i], model_handle[i], true);
    }
    not_triggered_verify();

    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_flash_modeldata(&test_vector[i], &m_expected_model_info[i].model_info, 0);
    }

    /****************** emulate regular work and parameters changing ******************/
    uint16_t location_element2 = 5;
    element_change_wrapper(1, location_element2);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(1, location_element2));
    not_triggered_verify();

    /* Changing element location to 0 will do nothing since that's the default value*/
    uint16_t location_element1 = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(0, location_element1));

    /*  Using a new ttl value for the first model will initiate a flash operation for the whole model, but not other models */
    test_vector[0].publish_ttl = 50;
    m_expected_model_info[0].model_info.publish_ttl = test_vector[0].publish_ttl;
    model_change_wrapper(0, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_ttl_set(0, test_vector[0].publish_ttl));
    not_triggered_verify();

    /********************* Verify the data sent to the mesh config subsystem: *****************************/
    mesh_config_entry_id_t entry_id = {.file = MESH_OPT_ACCESS_FILE_ID, .record = MESH_OPT_ACCESS_METADATA_RECORD};
    access_flash_metadata_t expected_metadata;
    m_access_metadata_params.callbacks.getter(entry_id, &expected_metadata);
    TEST_ASSERT_EQUAL_UINT16(ACCESS_ELEMENT_COUNT, expected_metadata.element_count);
    TEST_ASSERT_EQUAL_UINT16(ACCESS_SUBSCRIPTION_LIST_COUNT, expected_metadata.subscription_list_count);
    TEST_ASSERT_EQUAL_UINT16(ACCESS_MODEL_COUNT, expected_metadata.model_count);

    for (access_model_handle_t hndl = 0; hndl <= m_pst_indexes.model_handle; hndl++)
    {
        access_model_state_data_t expected_data;
        entry_id = MESH_OPT_ACCESS_MODELS_EID;
        entry_id.record += hndl;
        m_models_params.callbacks.getter(entry_id, &expected_data);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_expected_model_info[hndl].model_info, &expected_data, sizeof(access_model_state_data_t));
    }

    for (uint16_t idx = 0; idx <= m_pst_indexes.sublist_index; idx++)
    {
        access_flash_subscription_list_t expected_data;
        entry_id = MESH_OPT_ACCESS_SUBSCRIPTIONS_EID;
        entry_id.record += idx;
        m_subscriptions_params.callbacks.getter(entry_id, &expected_data);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_expected_subs_list[idx].sublist_info, &expected_data, sizeof(access_flash_subscription_list_t));
    }

    {
        uint16_t expected_data;
        entry_id = MESH_OPT_ACCESS_ELEMENTS_EID;
        entry_id.record += 1;
        m_elements_params.callbacks.getter(entry_id, &expected_data);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_expected_element_location[1].element_info, &expected_data, sizeof(uint16_t));
    }

    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        verify_flash_modeldata(&test_vector[i], &m_expected_model_info[i].model_info, 0);
    }

    /********************************* Restore access state from flash:*****************************/
    access_layer_reset(); // Reset access and clean all data
    // Mesh configuration found corrupted data
    entry_id = MESH_OPT_ACCESS_METADATA_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_access_metadata_params.callbacks.setter(entry_id, &metadata));
    nrf_mesh_evt_t load_failure_evt =
    {
        .type = NRF_MESH_EVT_CONFIG_LOAD_FAILURE,
        .params.config_load_failure.id.file = MESH_OPT_ACCESS_FILE_ID
    };
    evt_notify(&load_failure_evt);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, access_load_config_apply());
    not_triggered_verify();

    access_model_handle_t reinit_handle;
    access_layer_reset(); // Reset access and clean all data
    // DSM publish address was corrupted
    entry_id = MESH_OPT_ACCESS_METADATA_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_access_metadata_params.callbacks.setter(entry_id, &metadata));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&test_vector[0].add_params, &reinit_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(reinit_handle));
    entry_id = MESH_OPT_ACCESS_MODELS_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_models_params.callbacks.setter(entry_id, &m_expected_model_info[0].model_info));
    dsm_address_publish_add_handle_IgnoreAndReturn(NRF_ERROR_NOT_FOUND);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, access_load_config_apply());
    not_triggered_verify();

    access_layer_reset(); // Reset access and clean all data
    // DSM subscription address was corrupted
    entry_id = MESH_OPT_ACCESS_METADATA_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_access_metadata_params.callbacks.setter(entry_id, &metadata));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&test_vector[0].add_params, &reinit_handle));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(reinit_handle));
    entry_id = MESH_OPT_ACCESS_MODELS_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_models_params.callbacks.setter(entry_id, &m_expected_model_info[0].model_info));
    entry_id = MESH_OPT_ACCESS_SUBSCRIPTIONS_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_subscriptions_params.callbacks.setter(entry_id, &m_expected_subs_list[0].sublist_info));
    dsm_address_publish_add_handle_IgnoreAndReturn(NRF_SUCCESS);
    dsm_address_subscription_add_handle_IgnoreAndReturn(NRF_ERROR_NOT_FOUND);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, access_load_config_apply());
    not_triggered_verify();

    access_layer_reset(); // Reset access and lose all data again
    /* Add models first before restore*/
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&test_vector[i].add_params, &reinit_handle));
        if (test_vector[i].subscription_list_share_index == UINT32_MAX)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(i));
        }
        else
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_lists_share(i - 1, i));
        }
        TEST_ASSERT_EQUAL(reinit_handle, i);
    }

    for (access_model_handle_t hndl = 0; hndl <= m_pst_indexes.model_handle; hndl++)
    {
        entry_id = MESH_OPT_ACCESS_MODELS_EID;
        entry_id.record += hndl;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_models_params.callbacks.setter(entry_id, &m_expected_model_info[hndl].model_info));
    }

    for (uint16_t idx = 0; idx <= m_pst_indexes.sublist_index; idx++)
    {
        entry_id = MESH_OPT_ACCESS_SUBSCRIPTIONS_EID;
        entry_id.record += idx;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_subscriptions_params.callbacks.setter(entry_id, &m_expected_subs_list[idx].sublist_info));
    }

    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_set(1, location_element2));
        entry_id = MESH_OPT_ACCESS_ELEMENTS_EID;
        entry_id.record += 1;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_elements_params.callbacks.setter(entry_id, &m_expected_element_location[1].element_info));
    }

    for (access_model_handle_t hndl = 0; hndl <= m_pst_indexes.model_handle; hndl++)
    {
        if (m_expected_model_info[hndl].model_info.publish_address_handle != DSM_HANDLE_INVALID)
        {
            dsm_address_publish_add_handle_ExpectAndReturn(m_expected_model_info[hndl].model_info.publish_address_handle, NRF_SUCCESS);
        }

        if (m_expected_model_info[hndl].model_info.subscription_pool_index != ACCESS_SUBSCRIPTION_LIST_COUNT)
        {
            access_subscription_list_t sublist;
            for (uint32_t i = 0; i < ARRAY_SIZE(sublist.bitfield); i++)
            {
                sublist.bitfield[i] = ~m_expected_subs_list[m_expected_model_info[hndl].model_info.subscription_pool_index].sublist_info.inverted_bitfield[i];
            }

            for (uint32_t i = bitfield_next_get(sublist.bitfield, DSM_ADDR_MAX, 0);
                 i != DSM_ADDR_MAX;
                 i = bitfield_next_get(sublist.bitfield, DSM_ADDR_MAX, i+1))
            {
                dsm_address_subscription_add_handle_ExpectAndReturn(i, NRF_SUCCESS);
            }
        }

        if (ACCESS_MODEL_PUBLISH_PERIOD_RESTORE)
        {
            access_publish_period_set_Expect(NULL,
                                            m_expected_model_info[hndl].model_info.publication_period.step_res,
                                            m_expected_model_info[hndl].model_info.publication_period.step_num);
            access_publish_period_set_IgnoreArg_p_pubstate();
        }
    }

    entry_id = MESH_OPT_ACCESS_METADATA_EID;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_access_metadata_params.callbacks.setter(entry_id, &metadata));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_load_config_apply());
    not_triggered_verify();

    /* Verify that reading from access will produce the values given by the test vectors. */
    for (uint32_t i = 0; i < test_vector_size; ++i)
    {
        access_flash_test_struct_t new_test_vector = test_vector[i];
        update_reloaded_publish_params(&new_test_vector);
        verify_test_case_and_access_state(&new_test_vector, model_handle[i]);
    }

    uint16_t models_count = ACCESS_MODEL_COUNT;
    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(0, model_handles, &models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(0, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_models_get(1, model_handles, &models_count));
    TEST_ASSERT_EQUAL(1, models_count);
    models_count = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_vendor_model_count_get(1, (uint8_t *)&models_count));
    TEST_ASSERT_EQUAL(1, models_count);

    uint16_t location = 0xFF;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(0, &location));
    TEST_ASSERT_EQUAL(location_element1, location);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_element_location_get(1, &location));
    TEST_ASSERT_EQUAL(location_element2, location);


    /**************************************** access clear\persistent data erase *****************************************/
    access_reliable_cancel_all_StubWithCallback(access_reliable_cancel_all_cb);
    access_publish_clear_Expect();
    mesh_config_file_clear_Expect(MESH_OPT_ACCESS_FILE_ID);
    access_clear();
}

void test_subscription_dealloc_and_dealloc_share_combination(void)
{
    uint16_t i;

    /* Ensure access layer is stable */
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


    /* Test: De-alloc fails, if the access configuration is frozen */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_alloc(0));
    /* Freeze config. Device was provisioned. */
    nrf_mesh_is_device_provisioned_IgnoreAndReturn(true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_subscription_list_dealloc(1));
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_subscription_list_dealloc(0));

    /* Test: Share fails, if the access configuration is frozen */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, access_model_subscription_lists_share(0, 1));

}

void test_access_model_publish_period_divisor_set(void)
{
    /* Allocate a model - 0 : With publish support */
    access_model_handle_t handle0;
    access_model_add_params_t init_params;
    init_params.element_index = 0;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
    init_params.opcode_count = OPCODE_COUNT;
    init_params.p_args = TEST_REFERENCE;
    init_params.publish_timeout_cb = publish_timeout_cb;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle0));
    TEST_ASSERT_EQUAL(0, handle0);

    /* Allocate a model - 1 : Without publish support */
    access_model_handle_t handle1;
    init_params.element_index = 1;
    init_params.model_id.model_id = TEST_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    init_params.p_opcode_handlers = &m_opcode_handlers[0][0];
    init_params.opcode_count = OPCODE_COUNT;
    init_params.p_args = TEST_REFERENCE;
    init_params.publish_timeout_cb = NULL;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_add(&init_params, &handle1));
    TEST_ASSERT_EQUAL(1, handle1);

    access_publish_resolution_t resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
    uint8_t step_number = 2;
    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(handle0, resolution, step_number));

    /* Test invalid handle */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_publish_period_divisor_set(99, 1));

    /* Test invalid divisor value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_publish_period_divisor_set(handle0, 0));

    /* Test when model does not support publication */
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, access_model_publish_period_divisor_set(handle1, 1));

    /* Test divisor 1 results in same publish period settings */
    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_divisor_set(handle0, 1));

    /* Test large divisor results in minimum publish interval */
    resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
    step_number = 7;

    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(handle0, resolution, step_number));

    access_publish_resolution_t exp_resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
    uint8_t exp_step_number = 1;
    access_publish_period_set_Expect(NULL, exp_resolution, exp_step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_divisor_set(handle0, 0xF000));

    /* Test for correctness of divisor. Period 60s, divisor 100 => result: 0.6 seconds */
    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_divisor_set(handle0, 1));

    resolution = ACCESS_PUBLISH_RESOLUTION_1S;
    step_number = 60;

    access_publish_period_set_Expect(NULL, resolution, step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_set(handle0, resolution, step_number));

    exp_resolution = ACCESS_PUBLISH_RESOLUTION_100MS;
    exp_step_number = 6;
    access_publish_period_set_Expect(NULL, exp_resolution, exp_step_number);
    access_publish_period_set_IgnoreArg_p_pubstate();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_publish_period_divisor_set(handle0, 100));
}

void test_access_default_ttl_get_set(void)
{
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_default_ttl_set(128));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_default_ttl_set(1));
    TEST_ASSERT_EQUAL(ACCESS_DEFAULT_TTL, access_default_ttl_get());

    m_expected_default_ttl.is_triggered = true;
    m_expected_default_ttl.default_ttl = 11;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_default_ttl_set(m_expected_default_ttl.default_ttl));
    TEST_ASSERT_EQUAL(m_expected_default_ttl.default_ttl, access_default_ttl_get());
}
