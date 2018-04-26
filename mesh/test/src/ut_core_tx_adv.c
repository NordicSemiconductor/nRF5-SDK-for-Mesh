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
#include "unity.h"
#include "cmock.h"

#include "core_tx_adv.h"

#include "core_tx_mock.h"
#include "advertiser_mock.h"
#include "nordic_common.h"

#define TOKEN   0x12345678

static advertiser_t * mp_advertisers[CORE_TX_ROLE_COUNT];

static const core_tx_bearer_interface_t * mp_interface;

void setUp(void)
{
    advertiser_mock_Init();
    core_tx_mock_Init();
    mp_interface = NULL;
}

void tearDown(void)
{
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
}

/*****************************************************************************
* Mock functions
*****************************************************************************/
void advertiser_instance_init_cb(advertiser_t * p_adv, advertiser_tx_complete_cb_t tx_cb, uint8_t * p_buffer, uint32_t buffer_size, int calls)
{
    TEST_ASSERT_NOT_NULL(p_adv);
    TEST_ASSERT_NOT_NULL(p_buffer);
    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(p_buffer));
    TEST_ASSERT_TRUE(IS_WORD_ALIGNED(buffer_size));
    TEST_ASSERT_INT_WITHIN(1, 0, calls);
    mp_advertisers[calls] = p_adv;
    p_adv->buf.buffer = p_buffer;
    p_adv->buf.size = buffer_size;
    p_adv->tx_complete_callback = tx_cb;
    p_adv->enabled = true;
    advertiser_enable_Expect(p_adv);
}

void core_tx_bearer_register_cb(core_tx_bearer_t bearer, const core_tx_bearer_interface_t * p_if, int count)
{
    TEST_ASSERT_EQUAL(CORE_TX_BEARER_ADV, bearer);
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_if->packet_alloc);
    TEST_ASSERT_NOT_NULL(p_if->packet_discard);
    TEST_ASSERT_NOT_NULL(p_if->packet_send);
    mp_interface = p_if;
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_init(void)
{
    advertiser_instance_init_StubWithCallback(advertiser_instance_init_cb);
    core_tx_bearer_register_StubWithCallback(core_tx_bearer_register_cb);
    core_tx_adv_init();
    TEST_ASSERT_NOT_NULL(mp_advertisers[0]);
    TEST_ASSERT_NOT_NULL(mp_advertisers[1]);
    TEST_ASSERT_NOT_EQUAL(mp_advertisers[0], mp_advertisers[1]);
    TEST_ASSERT_NOT_NULL(mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->tx_complete_callback);
    TEST_ASSERT_EQUAL(CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR, mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->buf.size);
    TEST_ASSERT_EQUAL(CORE_TX_QUEUE_BUFFER_SIZE_RELAY, mp_advertisers[CORE_TX_ROLE_RELAY]->buf.size);
    TEST_ASSERT_NOT_EQUAL(mp_advertisers[0]->buf.buffer, mp_advertisers[1]->buf.buffer);
}

void test_alloc(void)
{
    test_init();

    core_tx_metadata_t metadata;
    uint8_t * p_packet;
    adv_packet_t adv_packet;
    struct
    {
        core_tx_role_t role;
        uint8_t size;
        bool successful_alloc;
    } vector[] = {
        {CORE_TX_ROLE_ORIGINATOR, 12, true},
        {CORE_TX_ROLE_ORIGINATOR, 0, true},
        {CORE_TX_ROLE_ORIGINATOR, 12, false},
        {CORE_TX_ROLE_RELAY, 12, true},
        {CORE_TX_ROLE_ORIGINATOR, 29, true},
        {CORE_TX_ROLE_ORIGINATOR, 30, false}, /* Not up to core tx to police the length */
    };

    /* Set repeat count per role, so we can verify that they're correct on the returned packet. */
    uint8_t repeat_counts[CORE_TX_ROLE_COUNT] = {4, 9};
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        core_tx_adv_count_set(i, repeat_counts[i]);
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(&adv_packet, 0, sizeof(adv_packet));

        metadata.bearer = CORE_TX_BEARER_ADV;
        metadata.role = vector[i].role;

        advertiser_packet_alloc_ExpectAndReturn(mp_advertisers[metadata.role],
                                                vector[i].size + 2,
                                                vector[i].successful_alloc ? &adv_packet
                                                                            : NULL);
        p_packet = mp_interface->packet_alloc(vector[i].size, &metadata, TOKEN);
        if (vector[i].successful_alloc)
        {
            TEST_ASSERT_NOT_NULL(p_packet);
            TEST_ASSERT_EQUAL_PTR(&adv_packet.packet.payload[2], p_packet);
            TEST_ASSERT_EQUAL(repeat_counts[vector[i].role], adv_packet.config.repeats);
            TEST_ASSERT_EQUAL(TOKEN, adv_packet.token);
        }
        else
        {
            TEST_ASSERT_NULL(p_packet);
        }

    }
}

void test_discard(void)
{
    test_init();

    core_tx_metadata_t metadata;
    adv_packet_t adv_packet;
    struct
    {
        core_tx_bearer_t bearer;
        core_tx_role_t role;
    } vector[] = {
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_RELAY},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(&adv_packet, 0, sizeof(adv_packet));

        metadata.bearer = vector[i].bearer;
        metadata.role = vector[i].role;
        advertiser_packet_discard_Expect(mp_advertisers[metadata.role], &adv_packet);
        mp_interface->packet_discard(&metadata, &adv_packet.packet.payload[2]);
    }
}

void test_send(void)
{
    test_init();

    core_tx_metadata_t metadata;
    adv_packet_t adv_packet;
    struct
    {
        core_tx_bearer_t bearer;
        core_tx_role_t role;
    } vector[] = {
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_ORIGINATOR},
        {CORE_TX_BEARER_ADV, CORE_TX_ROLE_RELAY},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&metadata, 0, sizeof(metadata));
        memset(&adv_packet, 0, sizeof(adv_packet));

        metadata.bearer = vector[i].bearer;
        metadata.role = vector[i].role;
        advertiser_packet_send_Expect(mp_advertisers[metadata.role], &adv_packet);
        mp_interface->packet_send(&metadata, &adv_packet.packet.payload[2]);
    }
}

void test_tx_complete(void)
{
    test_init();

    /* Only the originator-advertiser will have a callback, so we'll test with that one: */
    TEST_ASSERT_NOT_NULL(mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->tx_complete_callback);
    /* Verify the above claim: */
    TEST_ASSERT_NULL(mp_advertisers[CORE_TX_ROLE_RELAY]->tx_complete_callback);

    adv_packet_t packet;
    packet.token = TOKEN;
    packet.config.repeats = 0;
    core_tx_metadata_t metadata = {.bearer = CORE_TX_BEARER_ADV, .role = CORE_TX_ROLE_ORIGINATOR};

    core_tx_complete_Expect(&metadata, 1234, TOKEN);
    mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->tx_complete_callback(
        mp_advertisers[CORE_TX_ROLE_ORIGINATOR], packet.token, 1234);
}

void test_config(void)
{
    uint8_t repeat_counts[CORE_TX_ROLE_COUNT] = {4, 9};
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        core_tx_adv_count_set(i, repeat_counts[i]);
    }
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        TEST_ASSERT_EQUAL(repeat_counts[i], core_tx_adv_count_get(i));
    }

    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        advertiser_interval_set_Expect(mp_advertisers[i], 100 * i);
        core_tx_adv_interval_set(i, i * 100);
    }
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        mp_advertisers[i]->config.advertisement_interval_us = 25000 * i;
        TEST_ASSERT_EQUAL(mp_advertisers[i]->config.advertisement_interval_us / 1000, core_tx_adv_interval_get(i));
    }
}
