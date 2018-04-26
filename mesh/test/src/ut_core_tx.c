/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#include "nordic_common.h"
#include "test_assert.h"

#define TOKEN   0x12345678

static core_tx_bearer_interface_t m_interface;
static bool m_bearer_registered;

void setUp(void)
{
}

void tearDown(void)
{
}

/*****************************************************************************
* Mock functions
*****************************************************************************/
struct
{
    uint32_t net_packet_len;
    core_tx_metadata_t * p_metadata;
    uint32_t expected_calls;
    uint8_t * p_retval;
} m_expect_alloc;
uint8_t * packet_alloc(uint32_t net_packet_len,
                       const core_tx_metadata_t * p_metadata,
                       nrf_mesh_tx_token_t token)
{
    TEST_ASSERT_TRUE(m_expect_alloc.expected_calls-- > 0);
    TEST_ASSERT_EQUAL(m_expect_alloc.net_packet_len, net_packet_len);
    TEST_ASSERT_EQUAL(m_expect_alloc.p_metadata, p_metadata);
    TEST_ASSERT_EQUAL(TOKEN, token);

    return m_expect_alloc.p_retval;
}

struct
{
    const core_tx_metadata_t * p_metadata;
    uint8_t * p_packet;
    uint32_t expected_calls;
} m_expect_send;
void packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    TEST_ASSERT_TRUE(m_expect_send.expected_calls-- > 0);
    TEST_ASSERT_EQUAL(m_expect_send.p_metadata, p_metadata);
    TEST_ASSERT_EQUAL(m_expect_send.p_packet, p_packet);
}

struct
{
    const core_tx_metadata_t * p_metadata;
    uint8_t * p_packet;
    uint32_t expected_calls;
} m_expect_discard;
void packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet)
{
    TEST_ASSERT_TRUE(m_expect_discard.expected_calls-- > 0);
    TEST_ASSERT_EQUAL(m_expect_discard.p_metadata, p_metadata);
    TEST_ASSERT_EQUAL(m_expect_discard.p_packet, p_packet);
}
/*****************************************************************************
* Helper functions
*****************************************************************************/
struct
{
    core_tx_metadata_t metadata;
    uint32_t timestamp;
    nrf_mesh_tx_token_t token;
    uint32_t calls;
} m_tx_complete_expect;

void tx_complete_cb(const core_tx_metadata_t * p_metadata, uint32_t timestamp, nrf_mesh_tx_token_t token)
{
    TEST_ASSERT_TRUE(m_tx_complete_expect.calls > 0);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.metadata.bearer, p_metadata->bearer);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.metadata.role, p_metadata->role);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.timestamp, timestamp);
    TEST_ASSERT_EQUAL(m_tx_complete_expect.token, token);
    m_tx_complete_expect.calls--;
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_bearer_register(void)
{
    memset(&m_interface, 0, sizeof(m_interface));

    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_register(CORE_TX_BEARER_ADV, &m_interface));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_register(CORE_TX_BEARER_ADV, NULL));

    m_interface.packet_alloc = packet_alloc;
    m_interface.packet_send = packet_send;
    m_interface.packet_discard = packet_discard;

    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_register(CORE_TX_BEARER_ADV + 1, &m_interface));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_register(0, &m_interface));
    /* Successful */
    core_tx_bearer_register(CORE_TX_BEARER_ADV, &m_interface);
    m_bearer_registered = true;
    /* duplicate */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_bearer_register(CORE_TX_BEARER_ADV, &m_interface));

}

void test_alloc(void)
{
    TEST_ASSERT_TRUE(m_bearer_registered);
    core_tx_metadata_t metadata;
    uint8_t * p_packet;
    struct
    {
        core_tx_bearer_t bearer;
        core_tx_role_t role;
        uint8_t size;
        bool expect_success;
        bool successful_alloc;
    } vector[] = {
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, 12, true, true},
        {12, CORE_TX_ROLE_ORIGINATOR, 12, false, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, 0, true, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, 12, true, false},
        {CORE_TX_BEARER_ADV, 3, 12, false, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_RELAY, 12, true, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, 29, true, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, 30, true, false}, /* Not up to core tx to police the length */
    };

    uint8_t tx_buffer[30];

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(tx_buffer, 0, sizeof(tx_buffer));

        metadata.bearer = vector[i].bearer;
        metadata.role = vector[i].role;

        if (vector[i].expect_success)
        {
            m_expect_alloc.expected_calls = 1;
            m_expect_alloc.net_packet_len = vector[i].size;
            m_expect_alloc.p_metadata = &metadata;
            m_expect_alloc.p_retval = vector[i].successful_alloc ? tx_buffer : NULL;

            core_tx_bearer_t result =
                core_tx_packet_alloc(vector[i].size, &metadata, &p_packet, TOKEN);
            if (vector[i].successful_alloc)
            {
                TEST_ASSERT_EQUAL(metadata.bearer, result);
            }
            else
            {
                TEST_ASSERT_EQUAL(0, result);
            }
            TEST_ASSERT_EQUAL(0, m_expect_alloc.expected_calls);
        }
        else
        {
            TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(vector[i].size, &metadata, &p_packet, TOKEN));
        }
    }
    /* Invalid params not covered by vector */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(12, NULL, &p_packet, TOKEN));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_alloc(12, &metadata, NULL, TOKEN));
}

void test_discard(void)
{
    TEST_ASSERT_TRUE(m_bearer_registered);
    core_tx_metadata_t metadata;
    struct
    {
        core_tx_bearer_t bearer;
        core_tx_role_t role;
        bool expect_success;
    } vector[] = {
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, true},
        {12, CORE_TX_ROLE_ORIGINATOR, false},
        {CORE_TX_BEARER_ADV, 3, false},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_RELAY, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, true},
    };

    uint8_t tx_buffer[30];

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(&tx_buffer, 0, sizeof(tx_buffer));

        metadata.bearer = vector[i].bearer;
        metadata.role = vector[i].role;
        if (vector[i].expect_success)
        {
            m_expect_discard.expected_calls = 1;
            m_expect_discard.p_metadata     = &metadata;
            m_expect_discard.p_packet       = tx_buffer;
            core_tx_packet_discard(&metadata, tx_buffer);
        }
        else
        {
            TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_discard(&metadata, tx_buffer));
        }
        TEST_ASSERT_EQUAL(0, m_expect_discard.expected_calls);
    }
    /* Invalid params not covered by vector */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_discard(NULL, tx_buffer));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_discard(&metadata, NULL));
}

void test_send(void)
{
    TEST_ASSERT_TRUE(m_bearer_registered);
    core_tx_metadata_t metadata;
    struct
    {
        core_tx_bearer_t bearer;
        core_tx_role_t role;
        bool expect_success;
    } vector[] = {
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, true},
        {12, CORE_TX_ROLE_ORIGINATOR, false},
        {CORE_TX_BEARER_ADV, 3, false},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_RELAY, true},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR, true},
    };

    uint8_t tx_buffer[30];

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(&tx_buffer, 0, sizeof(tx_buffer));

        metadata.bearer = vector[i].bearer;
        metadata.role = vector[i].role;
        if (vector[i].expect_success)
        {
            m_expect_send.expected_calls = 1;
            m_expect_send.p_metadata     = &metadata;
            m_expect_send.p_packet       = tx_buffer;
            core_tx_packet_send(&metadata, tx_buffer);
        }
        else
        {
            TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_send(&metadata, tx_buffer));
        }
        TEST_ASSERT_EQUAL(0, m_expect_send.expected_calls);
    }
    /* Invalid params not covered by vector */
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_send(NULL, tx_buffer));
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_packet_send(&metadata, NULL));
}

void test_tx_complete(void)
{
    TEST_ASSERT_TRUE(m_bearer_registered);
    core_tx_metadata_t metadata = {.bearer = CORE_TX_BEARER_ADV, .role = CORE_TX_ROLE_ORIGINATOR};
    /* No callback set by default, so nothing should happen in the callback: */
    core_tx_complete(&metadata, 1234, TOKEN);
    /* even for invalid data: */
    core_tx_complete(NULL, 1234, TOKEN);

    /* Set the callback to get a forwarded cb */
    core_tx_complete_cb_set(tx_complete_cb);

    m_tx_complete_expect.calls     = 1;
    m_tx_complete_expect.metadata  = metadata;
    m_tx_complete_expect.timestamp = 1234;
    m_tx_complete_expect.token = TOKEN;
    core_tx_complete(&metadata, 1234, TOKEN);
    TEST_ASSERT_EQUAL(0, m_tx_complete_expect.calls);

    /* Clear it, should go back to not doing anything. */
    core_tx_complete_cb_set(NULL);
    core_tx_complete(&metadata, 1234, TOKEN);
    /* even for invalid data: */
    core_tx_complete(NULL, 1234, TOKEN);
}
