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
#include <string.h>
#include "unity.h"
#include "cmock.h"

#include "core_tx.h"

#include "utils.h"
#include "test_assert.h"

#define TOKEN   0x12345678

static core_tx_bearer_interface_t m_interface;
static core_tx_bearer_t m_bearer;

static void setup_bearer(void);
static void clear_interface_mocks(void);

void setUp(void)
{
    memset(&m_bearer, 0, sizeof(m_bearer));
    setup_bearer();
}

void tearDown(void)
{
    core_tx_reset();
    clear_interface_mocks();
}

/*****************************************************************************
* Mock functions
*****************************************************************************/
struct
{
    const core_tx_alloc_params_t * p_params;
    uint32_t expected_calls;
    uint32_t calls;
    core_tx_bearer_t * p_bearers[CORE_TX_BEARER_COUNT_MAX];
    core_tx_alloc_result_t retval[CORE_TX_BEARER_COUNT_MAX];
} m_expect_alloc;
core_tx_alloc_result_t packet_alloc(core_tx_bearer_t * p_bearer, const core_tx_alloc_params_t * p_params)
{
    TEST_ASSERT_TRUE(m_expect_alloc.expected_calls > 0);
    TEST_ASSERT_TRUE(m_expect_alloc.calls < m_expect_alloc.expected_calls);
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_params);
    TEST_ASSERT_EQUAL_PTR(m_expect_alloc.p_bearers[m_expect_alloc.calls], p_bearer);
    TEST_ASSERT_EQUAL_PTR(m_expect_alloc.p_params, p_params);

    return m_expect_alloc.retval[m_expect_alloc.calls++];
}
#define EXPECT_ALLOC(p_BEARER, p_PARAMS, RETVAL)                                                   \
    do                                                                                             \
    {                                                                                              \
        m_expect_alloc.p_params                                 = p_PARAMS;                        \
        m_expect_alloc.retval[m_expect_alloc.expected_calls]    = RETVAL;                          \
        m_expect_alloc.p_bearers[m_expect_alloc.expected_calls] = p_BEARER;                        \
        m_expect_alloc.expected_calls++;                                                           \
    } while (0)

struct
{
    uint8_t * p_packet;
    uint32_t packet_length;
    uint32_t expected_calls;
    uint32_t calls;
    core_tx_bearer_t * p_bearers[CORE_TX_BEARER_COUNT_MAX];
} m_expect_send;
void packet_send(core_tx_bearer_t * p_bearer, const uint8_t * p_packet, uint32_t packet_length)
{
    TEST_ASSERT_TRUE(m_expect_send.expected_calls > 0);
    TEST_ASSERT_TRUE(m_expect_send.calls < m_expect_send.expected_calls);
    TEST_ASSERT_EQUAL_PTR(m_expect_send.p_bearers[m_expect_send.calls], p_bearer);
    TEST_ASSERT_EQUAL(m_expect_send.packet_length, packet_length);
    TEST_ASSERT_EQUAL(m_expect_send.p_packet, p_packet);
    m_expect_send.calls++;
}
#define EXPECT_SEND(p_BEARER, p_PACKET, PACKET_LENGTH)                                             \
    do                                                                                             \
    {                                                                                              \
        m_expect_send.p_packet                                = p_PACKET;                          \
        m_expect_send.packet_length                           = PACKET_LENGTH;                     \
        m_expect_send.p_bearers[m_expect_send.expected_calls] = p_BEARER;                          \
        m_expect_send.expected_calls++;                                                            \
    } while (0)

struct
{
    uint32_t expected_calls;
    uint32_t calls;
    core_tx_bearer_t * p_bearers[CORE_TX_BEARER_COUNT_MAX];
} m_expect_discard;
void packet_discard(core_tx_bearer_t * p_bearer)
{
    TEST_ASSERT_TRUE(m_expect_discard.expected_calls > 0);
    TEST_ASSERT_TRUE(m_expect_discard.calls < m_expect_discard.expected_calls);
    TEST_ASSERT_EQUAL_PTR(m_expect_discard.p_bearers[m_expect_discard.calls], p_bearer);
    m_expect_discard.calls++;
}
#define EXPECT_DISCARD(p_BEARER)                                                                   \
    do                                                                                             \
    {                                                                                              \
        m_expect_discard.p_bearers[m_expect_discard.expected_calls] = p_BEARER;                    \
        m_expect_discard.expected_calls++;                                                         \
    } while (0)

/*****************************************************************************
* Helper functions
*****************************************************************************/
struct
{
    core_tx_role_t role;
    uint32_t bearer_index;
    uint32_t timestamp;
    nrf_mesh_tx_token_t token;
    uint32_t calls;
} m_tx_complete_expect;

void tx_complete_cb(core_tx_role_t role,
                    uint32_t bearer_index,
                    uint32_t timestamp,
                    nrf_mesh_tx_token_t token)
{
    TEST_ASSERT_TRUE(m_tx_complete_expect.calls > 0);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.role, role);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.bearer_index, bearer_index);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.timestamp, timestamp);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.token, token);
    m_tx_complete_expect.calls--;
}

static void setup_bearer(void)
{
    m_interface.packet_alloc = packet_alloc;
    m_interface.packet_send = packet_send;
    m_interface.packet_discard = packet_discard;
    core_tx_bearer_add(&m_bearer, &m_interface, CORE_TX_BEARER_TYPE_ADV);
}

static void clear_interface_mocks(void)
{
    memset(&m_expect_alloc, 0, sizeof(m_expect_alloc));
    memset(&m_expect_discard, 0, sizeof(m_expect_discard));
    memset(&m_expect_send, 0, sizeof(m_expect_send));
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_bearer_add(void)
{
    memset(&m_interface, 0, sizeof(m_interface));
    core_tx_bearer_t bearers[64];

    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(&bearers[0], &m_interface, CORE_TX_BEARER_TYPE_ADV));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(&bearers[0], NULL, CORE_TX_BEARER_TYPE_ADV));

    m_interface.packet_alloc = packet_alloc;
    m_interface.packet_send = packet_send;
    m_interface.packet_discard = packet_discard;

    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(NULL, &m_interface, CORE_TX_BEARER_TYPE_ADV));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(&bearers[0], &m_interface, CORE_TX_BEARER_TYPE_INVALID));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(&bearers[0], &m_interface, CORE_TX_BEARER_TYPE_ALLOW_ALL));

    /* Successful */
    core_tx_reset();
    for (uint32_t i = 0; i < ARRAY_SIZE(bearers); ++i)
    {
        core_tx_bearer_add(&bearers[i], &m_interface, i + CORE_TX_BEARER_TYPE_ADV);
        TEST_ASSERT_EQUAL(i, bearers[i].bearer_index);
        TEST_ASSERT_EQUAL(&m_interface, bearers[i].p_interface);
        TEST_ASSERT_EQUAL(i + CORE_TX_BEARER_TYPE_ADV, bearers[i].type);

        TEST_ASSERT_EQUAL(bearers[i].type, core_tx_bearer_type_get(i));
        TEST_ASSERT_EQUAL(i + 1, core_tx_bearer_count_get());
    }

    /* duplicate */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_add(&bearers[0], &m_interface, CORE_TX_BEARER_TYPE_ADV));
}

void test_alloc(void)
{
    struct
    {
        core_tx_role_t role;
        uint8_t size;
        bool expect_success;
        core_tx_bearer_selector_t bearer;
        bool successful_alloc[2];
        bool calls_alloc[2];
    } vector[] = {
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,            {true, true},   {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,            {true, false},  {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,            {false, true},  {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,            {false, false}, {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 0, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,             {true, true},   {true, true}},
        {3, 12, false},
        {CORE_TX_ROLE_RELAY, 12, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,                 {true, true},   {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 29, true, CORE_TX_BEARER_TYPE_ALLOW_ALL,            {true, true},   {true, true}},
        {CORE_TX_ROLE_ORIGINATOR, 30, false}, /* Invalid length */
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_FRIEND,         {true, true},   {false, false}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_GATT_SERVER,    {true, true},   {false, true}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ADV,            {true, true},   {true, false}},
        {CORE_TX_ROLE_ORIGINATOR, 12, true, CORE_TX_BEARER_TYPE_ADV,            {false, false}, {true, false}},
    };

    uint8_t * p_packet = NULL;

    /* Extra bearer */
    core_tx_bearer_t bearer;
    core_tx_bearer_add(&bearer, &m_interface, CORE_TX_BEARER_TYPE_GATT_SERVER);

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        network_packet_metadata_t metadata;
        core_tx_alloc_params_t params = {.role              = vector[i].role,
                                         .net_packet_len    = vector[i].size,
                                         .p_metadata        = &metadata,
                                         .bearer_selector   = vector[i].bearer,
                                         .token = ((vector[i].role == CORE_TX_ROLE_RELAY) ? NRF_MESH_RELAY_TOKEN
                                                                                          : TOKEN)
                                                                                          };
        p_packet                      = NULL;

        if (vector[i].expect_success)
        {
            if (vector[i].calls_alloc[0])
            {
                EXPECT_ALLOC(&m_bearer, &params, vector[i].successful_alloc[0] ? CORE_TX_ALLOC_SUCCESS : 0x1234);
            }
            if (vector[i].calls_alloc[1])
            {
                EXPECT_ALLOC(&bearer,   &params, vector[i].successful_alloc[1] ? CORE_TX_ALLOC_SUCCESS : 0xABCD);
            }

            core_tx_bearer_bitmap_t result = core_tx_packet_alloc(&params, &p_packet);
            if (vector[i].successful_alloc[0] || vector[i].successful_alloc[1])
            {
                TEST_ASSERT_EQUAL(((vector[i].successful_alloc[0] && vector[i].calls_alloc[0]) << 0) |
                                  ((vector[i].successful_alloc[1] && vector[i].calls_alloc[1]) << 1),
                                  result);

                if (result != 0)
                {
                    TEST_ASSERT_NOT_NULL(p_packet);

                    /* Do an alloc again, should assert without side effects. */
                    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(&params, &p_packet));

                    /* Can't allocate again without discarding the previous packet */
                    if (vector[i].successful_alloc[0] && vector[i].calls_alloc[0])
                    {
                        EXPECT_DISCARD(&m_bearer);
                    }
                    if (vector[i].successful_alloc[1] && vector[i].calls_alloc[1])
                    {
                        EXPECT_DISCARD(&bearer);
                    }
                    core_tx_packet_discard();
                }
            }
            else
            {
                TEST_ASSERT_EQUAL(0, result);
            }
            TEST_ASSERT_EQUAL(m_expect_alloc.expected_calls, m_expect_alloc.calls);
        }
        else
        {
            TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(&params, &p_packet));
            TEST_ASSERT_EQUAL(0, m_expect_alloc.calls); // no change
        }

        clear_interface_mocks();
    }

    /* Valid parameter contents: */
    network_packet_metadata_t metadata;
    core_tx_alloc_params_t params = {.role           = CORE_TX_ROLE_ORIGINATOR,
                                     .net_packet_len = 20,
                                     .p_metadata     = &metadata,
                                     .token          = TOKEN};

    /* Invalid parameters not covered by vector */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(NULL, &p_packet));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(&params, NULL));
}

void test_discard(void)
{
    /* No packet allocated, a discard should cause an assert: */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_discard());

    /* Extra bearer */
    core_tx_bearer_t bearer;
    core_tx_bearer_add(&bearer, &m_interface, CORE_TX_BEARER_TYPE_GATT_SERVER);

    /* Allocate a packet we can discard (on both bearers) */
    uint8_t * p_packet = NULL;
    network_packet_metadata_t metadata;
    core_tx_alloc_params_t params = {.role              = CORE_TX_ROLE_ORIGINATOR,
                                     .net_packet_len    = 20,
                                     .p_metadata        = &metadata,
                                     .bearer_selector   = CORE_TX_BEARER_TYPE_ALLOW_ALL,
                                     .token             = TOKEN};

    EXPECT_ALLOC(&m_bearer, &params, CORE_TX_ALLOC_SUCCESS);
    EXPECT_ALLOC(&bearer,   &params, CORE_TX_ALLOC_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32((1 << 0) | (1 << 1), core_tx_packet_alloc(&params, &p_packet));

    /* Discard, should happen on both bearers: */
    EXPECT_DISCARD(&m_bearer);
    EXPECT_DISCARD(&bearer);
    core_tx_packet_discard();
    TEST_ASSERT_EQUAL(2, m_expect_discard.calls);

    clear_interface_mocks();

    /* Allocate on just one bearer: */
    EXPECT_ALLOC(&m_bearer,   &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    EXPECT_ALLOC(&bearer, &params, CORE_TX_ALLOC_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32((1 << 1), core_tx_packet_alloc(&params, &p_packet));

    /* Discard, should happen only on the second bearer: */
    EXPECT_DISCARD(&bearer);
    core_tx_packet_discard();
    TEST_ASSERT_EQUAL(1, m_expect_discard.calls);

    clear_interface_mocks();

    /* Allocate failing on both bearers: */
    EXPECT_ALLOC(&m_bearer, &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    EXPECT_ALLOC(&bearer,   &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    TEST_ASSERT_EQUAL_HEX32(0, core_tx_packet_alloc(&params, &p_packet));

    /* No packet allocated, a discard should cause an assert: */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_discard());

    clear_interface_mocks();
}

void test_send(void)
{
    /* No packet allocated, a send should cause an assert: */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_send());

    /* Extra bearer */
    core_tx_bearer_t bearer;
    core_tx_bearer_add(&bearer, &m_interface, CORE_TX_BEARER_TYPE_GATT_SERVER);

    /* Allocate a packet we can send (on both bearers) */
    uint8_t * p_packet = NULL;
    network_packet_metadata_t metadata;
    core_tx_alloc_params_t params = {.role              = CORE_TX_ROLE_ORIGINATOR,
                                     .net_packet_len    = 20,
                                     .p_metadata        = &metadata,
                                     .bearer_selector   = CORE_TX_BEARER_TYPE_ALLOW_ALL,
                                     .token             = TOKEN};

    EXPECT_ALLOC(&m_bearer,   &params, CORE_TX_ALLOC_SUCCESS);
    EXPECT_ALLOC(&bearer, &params, CORE_TX_ALLOC_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(CORE_TX_BEARER_TYPE_ADV | CORE_TX_BEARER_TYPE_GATT_SERVER,
                            core_tx_packet_alloc(&params, &p_packet));

    /* send, should happen on both bearers: */
    EXPECT_SEND(&m_bearer, p_packet, params.net_packet_len);
    EXPECT_SEND(&bearer, p_packet, params.net_packet_len);
    core_tx_packet_send();
    TEST_ASSERT_EQUAL(2, m_expect_send.calls);

    clear_interface_mocks();

    /* Allocate on just one bearer: */
    EXPECT_ALLOC(&m_bearer, &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    EXPECT_ALLOC(&bearer,   &params, CORE_TX_ALLOC_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(CORE_TX_BEARER_TYPE_GATT_SERVER, core_tx_packet_alloc(&params, &p_packet));

    /* send, should happen only on the second bearer: */
    EXPECT_SEND(&bearer, p_packet, params.net_packet_len);
    core_tx_packet_send();
    TEST_ASSERT_EQUAL(1, m_expect_send.calls);

    clear_interface_mocks();

    /* Allocate failing on both bearers: */
    EXPECT_ALLOC(&m_bearer, &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    EXPECT_ALLOC(&bearer,   &params, CORE_TX_ALLOC_FAIL_NO_MEM);
    TEST_ASSERT_EQUAL_HEX32(0, core_tx_packet_alloc(&params, &p_packet));

    /* No packet allocated, a send should cause an assert: */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_send());

    clear_interface_mocks();
}

void test_tx_complete(void)
{
    /* No callback set by default, so nothing should happen in the callback: */
    core_tx_complete(&m_bearer, CORE_TX_ROLE_ORIGINATOR, 1234, TOKEN);
    /* even for invalid data: */
    core_tx_complete(&m_bearer, 42, 1234, TOKEN);

    /* Set the callback to get a forwarded cb */
    core_tx_complete_cb_set(tx_complete_cb);

    m_tx_complete_expect.calls        = 1;
    m_tx_complete_expect.role         = CORE_TX_ROLE_ORIGINATOR;
    m_tx_complete_expect.bearer_index = m_bearer.bearer_index;
    m_tx_complete_expect.timestamp    = 1234;
    m_tx_complete_expect.token        = TOKEN;
    core_tx_complete(&m_bearer, CORE_TX_ROLE_ORIGINATOR, 1234, TOKEN);
    TEST_ASSERT_EQUAL(0, m_tx_complete_expect.calls);

    /* Clear it, should go back to not doing anything. */
    core_tx_complete_cb_set(NULL);
    core_tx_complete(&m_bearer, CORE_TX_ROLE_ORIGINATOR, 1234, TOKEN);
    /* even for invalid data: */
    core_tx_complete(&m_bearer, 42, 1234, TOKEN);
}
