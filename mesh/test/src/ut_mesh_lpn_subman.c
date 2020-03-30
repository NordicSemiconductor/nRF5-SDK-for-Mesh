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

#include <stdint.h>
#include <stdbool.h>

#include "event_mock.h"
#include "transport_mock.h"
#include "mesh_lpn_internal.h"
#include "mesh_lpn_mock.h"
#include "nrf_mesh_mock.h"
#include "packet_mesh.h"

#define LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX    (DSM_NONVIRTUAL_ADDR_MAX + DSM_VIRTUAL_ADDR_MAX + 1)

typedef enum
{
    ADDR_NONE,
    ADDR_ADD,
    ADDR_REMOVE
} test_flags_t;

typedef struct __attribute((packed))
{
    uint8_t transaction_number;
    uint16_t addr_list[PACKET_MESH_TRS_CONTROL_FRIEND_SUBLIST_ADD_REMOVE_SIZE/2];
} friend_sublist_add_rem_msg_t;

typedef struct
{
    uint16_t addr;
    bool sent;
    test_flags_t add_remove;
} test_addr_list_t;

static test_addr_list_t m_test_addr_list[LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX];

static nrf_mesh_evt_handler_cb_t m_evt_handler_cb;
static transport_control_packet_handler_t m_transport_opcode_handler;
static uint8_t m_exp_transaction_number;
static bool m_skip_transaction_number_test;
static bool m_test_in_friendship;    /* 0: Not in friendship, 1: In friendship */
static uint16_t m_test_sublist_size;

static uint32_t m_test_terminate_exp_cnt;
static int32_t m_test_exp_confirm_cnt;

static transport_control_packet_t * mp_module_pkt_t;
static packet_mesh_trs_control_packet_t m_pdu_confirm;
static transport_control_packet_t m_pkt_confirm_msg = {
            .opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_CONFIRM,
            .p_data = &m_pdu_confirm,
            .data_len = 0x01
            /* Other entries are not required to be populated for this module test */
        };



/******** Helpers for the Unit Test ********/

static void helper_test_addr_list_init(void)
{
    for(uint32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        m_test_addr_list[i].addr = 0xCCC0 + i;
        m_test_addr_list[i].sent = 0;
        m_test_addr_list[i].add_remove = ADDR_NONE;
    }
}

static void helper_test_addr_list_mark_as_unsent(void)
{
    for(uint32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        m_test_addr_list[i].sent = 0;
        m_test_addr_list[i].add_remove = ADDR_NONE;
    }
}

static void helper_test_init(void)
{
    /* Create address list */
    helper_test_addr_list_init();
    m_exp_transaction_number = 0;
    m_test_in_friendship = false;
    m_test_terminate_exp_cnt = 0;
}

static void helper_tag_test_address(uint16_t addr, transport_control_opcode_t opcode)
{
    for (uint32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        if (m_test_addr_list[i].addr == addr && m_test_addr_list[i].sent == false &&
            ((opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD && m_test_addr_list[i].add_remove == ADDR_ADD) ||
             (opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE && m_test_addr_list[i].add_remove == ADDR_REMOVE))
           )
        {
            m_test_addr_list[i].sent = true;
            return;
        }
    }

    TEST_ASSERT_TRUE(false);
}

static void helper_untag_add_address_from_pdu(void)
{
    friend_sublist_add_rem_msg_t * p_pdu = (friend_sublist_add_rem_msg_t *) mp_module_pkt_t->p_data;
    uint16_t addr;
    for (uint32_t i = 0; i < mp_module_pkt_t->data_len/2; i++)
    {
        addr = BE2LE16(p_pdu->addr_list[i]);
        for (uint32_t i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
        {
            if (m_test_addr_list[i].addr == addr && m_test_addr_list[i].sent == true && m_test_addr_list[i].add_remove == ADDR_ADD &&
                mp_module_pkt_t->opcode == TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD)
            {
                m_test_addr_list[i].sent = false;
                break;
            }
        }
    }

    return;
}

static void helper_trigger_event(nrf_mesh_evt_type_t event_type)
{
    nrf_mesh_evt_t evt;

    m_exp_transaction_number = 0;
    if (event_type == NRF_MESH_EVT_FRIENDSHIP_TERMINATED)
    {
        evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
        helper_test_addr_list_mark_as_unsent();
        m_test_in_friendship = false;
        m_test_exp_confirm_cnt = 0;
    }
    else if (event_type == NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED)
    {
        evt.params.friendship_established.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
        m_test_in_friendship = true;
        m_test_exp_confirm_cnt = 0;
    }
    else
    {
        TEST_ASSERT_TRUE(false);
    }

    evt.type = event_type;
    m_evt_handler_cb(&evt);
}

static void helper_trigger_confirm_rx(void)
{
    m_pdu_confirm.pdu[0] = m_exp_transaction_number;
    m_exp_transaction_number++;
    m_transport_opcode_handler.callback(&m_pkt_confirm_msg, NULL);
    m_test_exp_confirm_cnt--;
}

static uint32_t helper_pending_confirm_cnt_get(void)
{
    return m_test_exp_confirm_cnt;
}

static void mesh_lpn_subman_remove_call(uint8_t i)
{
    m_test_addr_list[i].sent = false;
    m_test_addr_list[i].add_remove = ADDR_REMOVE;
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_lpn_subman_remove(m_test_addr_list[i].addr));
}

static void mesh_lpn_subman_add_call(uint8_t i)
{
    m_test_addr_list[i].sent = false;
    m_test_addr_list[i].add_remove = ADDR_ADD;
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_lpn_subman_add(m_test_addr_list[i].addr));
}

static void mesh_lpn_subman_add_call_keep_sent(uint8_t i)
{
    m_test_addr_list[i].add_remove = ADDR_ADD;
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_lpn_subman_add(m_test_addr_list[i].addr));
}

/******** Mocks for the Unit Test ********/

uint32_t transport_control_packet_consumer_add_stub(const transport_control_packet_handler_t * p_handlers, uint32_t handler_count, int count)
{
    m_transport_opcode_handler = *p_handlers;
    TEST_ASSERT_EQUAL(1, handler_count);

    return NRF_SUCCESS;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    m_evt_handler_cb = p_handler_params->evt_cb;
}

void mesh_lpn_subman_data_push(transport_control_packet_t * p_trs_ctrl_pkt)
{
    mp_module_pkt_t = p_trs_ctrl_pkt;
    /* Check addresses and tag them in the test address list */
    friend_sublist_add_rem_msg_t * p_pdu = (friend_sublist_add_rem_msg_t *) p_trs_ctrl_pkt->p_data;
    for (uint32_t i = 0; i < p_trs_ctrl_pkt->data_len/2; i++)
    {
        helper_tag_test_address(BE2LE16(p_pdu->addr_list[i]), p_trs_ctrl_pkt->opcode);
    }

    TEST_ASSERT_TRUE(m_test_exp_confirm_cnt >=0);
    m_test_exp_confirm_cnt++;
    if (!m_skip_transaction_number_test)
    {
        TEST_ASSERT_EQUAL(m_exp_transaction_number, p_pdu->transaction_number);
    }
}

void mesh_lpn_subman_data_clear(void)
{

}

uint16_t mesh_lpn_friend_subscription_size_get(void)
{
    return (m_test_sublist_size);
}

/** Copy of the implementation from nrf_mesh_utils.c. Pulled in to avoid build problems with the other functions in that module */
nrf_mesh_address_type_t nrf_mesh_address_type_get(uint16_t address)
{
    if (address == NRF_MESH_ADDR_UNASSIGNED)
    {
        return NRF_MESH_ADDRESS_TYPE_INVALID;
    }
    else
    {
        static const nrf_mesh_address_type_t types_lookup[] =
        {
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b00 */
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b01 */
            NRF_MESH_ADDRESS_TYPE_VIRTUAL, /* 0b10 */
            NRF_MESH_ADDRESS_TYPE_GROUP,   /* 0b11 */
        };
        return types_lookup[(address & NRF_MESH_ADDR_TYPE_BITS_MASK) >> NRF_MESH_ADDR_TYPE_BITS_OFFSET];
    }
}

/******** Setup and Tear Down ********/

void setUp(void)
{
    transport_mock_Init();
    mesh_lpn_mock_Init();

    helper_test_init();
}

void tearDown(void)
{
    transport_mock_Verify();
    transport_mock_Destroy();
    mesh_lpn_mock_Verify();
    mesh_lpn_mock_Destroy();
}



/******** Tests ********/

void test_mesh_lpn_subman_init(void)
{
    transport_control_packet_consumer_add_StubWithCallback(transport_control_packet_consumer_add_stub);
    mesh_lpn_subman_init();
    printf("Transport consumper ptr: %d\n", m_transport_opcode_handler.callback);
    printf("LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX: %d\n", LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX);
}

void test_mesh_lpn_subman_add(void)
{
    uint32_t i;

    /* Test: Invalid address */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, mesh_lpn_subman_add(0x0000));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, mesh_lpn_subman_add(0x0001));

    /* Test: Addition fails, if not in friendship */
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_lpn_subman_add(0xC000));

    /* Test: All additions: Add addresses, Enter Friendship, check for propagation */
    m_test_sublist_size = LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX;
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));

    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test:continue: Terminate friendship, establish again, is confirm is suddenly received
    without initiating anything, termination occurs. */
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    mesh_lpn_friendship_terminate_ExpectAndReturn(NRF_SUCCESS);
    helper_trigger_confirm_rx();

    /* Test: Establish, add all addresses, Terminate, again all addresses can be added */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);

    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);

    /* Test: Partially fill the addresses after friendship establishment and check propagation */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < 2; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    helper_trigger_confirm_rx();

    /* Test: continue: Add one address and check propagation */
    mesh_lpn_subman_add_call(i);
    i++;
    helper_trigger_confirm_rx();

    /* Test: continue: fill remaining addresses and check propagation */
    while (i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX)
    {
        mesh_lpn_subman_add_call(i);
        i++;
    }
    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test: continue: Add duplicate address, check success */
    mesh_lpn_subman_add_call(0);
    mesh_lpn_subman_add_call(7);
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
}

void test_mesh_lpn_subman_remove(void)
{
    uint32_t i;

    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);

    /* Test: Invalid address */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, mesh_lpn_subman_remove(0x0000));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, mesh_lpn_subman_remove(0x0001));

    /* Test: When not in friendship, addresses removal is invalid operation */
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_lpn_subman_remove(0xC000));

    /* Test: Removing address when none were added, is not a problem */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_lpn_subman_remove(0xC234));

    /* Test: Add all addresses, enter friendship, propogate all, remove one, check propagation */
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    i = 0;
    mesh_lpn_subman_remove_call(i);     /* remove one */
    helper_trigger_confirm_rx();

    /* Test: continue: Add same one, expect success, add second, expect NO_MEM, check propagation */
    mesh_lpn_subman_add_call(i);
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_confirm_rx();

    /* Test: continue: Remove all, check propagation. */
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_remove_call(i);
    }

    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test: continue: all new addresses can be added after all are removed. */
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));

    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
}

void test_mix_add_remove(void)
{
    uint32_t i;

    /* Test: Add all addresses, enter friendship, propogate all, remove one, check propagation */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_confirm_rx();

    /* Test: Mark few unsent entries for Removal */
    mesh_lpn_subman_remove_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 1);
    mesh_lpn_subman_remove_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 2);
    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test: add all, trigger one confirm, remove two unsent entries, add them again */
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        if (m_test_addr_list[i].sent == true && m_test_addr_list[i].add_remove == ADDR_ADD)
        {
            mesh_lpn_subman_add_call_keep_sent(i);
        }
        else
        {
            mesh_lpn_subman_add_call(i);
        }
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_confirm_rx();

    mesh_lpn_subman_remove_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 1);
    mesh_lpn_subman_remove_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 2);
    mesh_lpn_subman_add_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 1);
    mesh_lpn_subman_add_call(LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 2);

    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test: Friendship is established, but not yet propogated to this module, it still works. */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
    m_test_in_friendship = true;
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX - 2; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    mesh_lpn_subman_add_call(i++);
    mesh_lpn_subman_add_call(i++);
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    helper_trigger_confirm_rx();

    while (helper_pending_confirm_cnt_get() > 0)
    {
        helper_trigger_confirm_rx();
    };

    /* Test: Force the wrong confirm PDU and check retries */
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
    helper_trigger_event(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
    for (i = 0; i < LPN_SUBMAN_ADDRESS_LIST_SIZE_MAX; i++)
    {
        mesh_lpn_subman_add_call(i);
    }
    mesh_lpn_is_in_friendship_ExpectAndReturn(m_test_in_friendship);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, mesh_lpn_subman_add(0xD000));
    uint8_t correct_transaction_num = m_exp_transaction_number++;
    m_exp_transaction_number = 0xAA;
    m_skip_transaction_number_test = true;
    helper_untag_add_address_from_pdu();
    helper_trigger_confirm_rx();
    m_exp_transaction_number = correct_transaction_num;
    m_skip_transaction_number_test = false;
    helper_trigger_confirm_rx();

    correct_transaction_num = m_exp_transaction_number++;
    m_exp_transaction_number = 0xAA;
    m_skip_transaction_number_test = true;
    helper_untag_add_address_from_pdu();
    helper_trigger_confirm_rx();
    helper_untag_add_address_from_pdu();
    helper_trigger_confirm_rx();
    mesh_lpn_friendship_terminate_ExpectAndReturn(NRF_SUCCESS);
    helper_untag_add_address_from_pdu();
    helper_trigger_confirm_rx();
}
