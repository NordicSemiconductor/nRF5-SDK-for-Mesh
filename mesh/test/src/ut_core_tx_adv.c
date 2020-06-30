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
#include "unity.h"
#include "cmock.h"

#include "core_tx_adv.h"

#include "core_tx_mock.h"
#include "advertiser_mock.h"
#include "utils.h"
#include "test_assert.h"
#include "mesh_opt_core.h"
#include "mesh_config_entry.h"
#include "nrf_mesh_config_bearer.h"

#define TOKEN   0x12345678

static advertiser_t * mp_advertisers[CORE_TX_ROLE_COUNT];

static const core_tx_bearer_interface_t * mp_interface;
static core_tx_bearer_t * mp_bearer;

extern const mesh_config_entry_params_t m_mesh_opt_core_adv_params;
extern const mesh_config_entry_params_t m_mesh_opt_core_tx_power_params;
extern const mesh_config_entry_params_t m_mesh_opt_core_adv_addr_params;

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
static const mesh_config_entry_params_t * entry_params_get(mesh_config_entry_id_t entry_id)
{
    if (IS_IN_RANGE(entry_id.record, MESH_OPT_CORE_ADV_RECORD_START, MESH_OPT_CORE_ADV_RECORD_END))
    {
        return &m_mesh_opt_core_adv_params;
    }
    else if (IS_IN_RANGE(entry_id.record, MESH_OPT_CORE_TX_POWER_RECORD_START, MESH_OPT_CORE_TX_POWER_RECORD_END))
    {
        return &m_mesh_opt_core_tx_power_params;
    }
    else if (IS_IN_RANGE(entry_id.record, MESH_OPT_CORE_ADV_ADDR_RECORD_START, MESH_OPT_CORE_ADV_ADDR_RECORD_END))
    {
        return &m_mesh_opt_core_adv_addr_params;
    }
    TEST_FAIL_MESSAGE("Unknown entry id");
    return NULL;
}

uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    return entry_params_get(id)->callbacks.setter(id, p_entry);
}

uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    entry_params_get(id)->callbacks.getter(id, p_entry);
    return NRF_SUCCESS;
}

uint32_t mesh_config_entry_delete(mesh_config_entry_id_t id)
{
    return NRF_SUCCESS;
}

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

void core_tx_bearer_add_cb(core_tx_bearer_t * p_bearer, const core_tx_bearer_interface_t * p_if, core_tx_bearer_type_t type, int count)
{
    TEST_ASSERT_EQUAL(CORE_TX_BEARER_TYPE_ADV, type);
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_if->packet_alloc);
    TEST_ASSERT_NOT_NULL(p_if->packet_discard);
    TEST_ASSERT_NOT_NULL(p_if->packet_send);
    mp_bearer    = p_bearer;
    mp_interface = p_if;
}

void advertiser_interval_set_ExpectAndSave(advertiser_t * p_adv, uint32_t expected_interval_ms)
{
    advertiser_interval_set_Expect(p_adv, expected_interval_ms);
    p_adv->config.advertisement_interval_us = MS_TO_US(expected_interval_ms);
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_init(void)
{
    advertiser_instance_init_StubWithCallback(advertiser_instance_init_cb);
    core_tx_bearer_add_StubWithCallback(core_tx_bearer_add_cb);
    core_tx_adv_init();
    TEST_ASSERT_NOT_NULL(mp_advertisers[0]);
    TEST_ASSERT_NOT_NULL(mp_advertisers[1]);
    TEST_ASSERT_NOT_EQUAL(mp_advertisers[0], mp_advertisers[1]);
    TEST_ASSERT_NOT_NULL(mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->tx_complete_callback);
    TEST_ASSERT_EQUAL(CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR, mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->buf.size);
    TEST_ASSERT_EQUAL(CORE_TX_QUEUE_BUFFER_SIZE_RELAY, mp_advertisers[CORE_TX_ROLE_RELAY]->buf.size);
    TEST_ASSERT_NOT_EQUAL(mp_advertisers[0]->buf.buffer, mp_advertisers[1]->buf.buffer);
    advertiser_mock_Verify();
    mesh_opt_core_adv_t cfg;
    cfg.enabled = true;
    cfg.tx_interval_ms = BEARER_ADV_INT_DEFAULT_MS;
    cfg.tx_count = 1;
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        printf("mp_advertisers[i]: %p \n", mp_advertisers[i]);
        advertiser_interval_set_ExpectAndSave(mp_advertisers[i], BEARER_ADV_INT_DEFAULT_MS);
        mesh_opt_core_adv_set(i, &cfg);
        advertiser_mock_Verify();
    }
}

void test_alloc_send_discard(void)
{
    test_init();

    adv_packet_t adv_packet;
    struct
    {
        core_tx_role_t role;
        uint8_t size;
        bool successful_alloc;
        bool send;
    } vector[] = {
        /* Send all: */
        {CORE_TX_ROLE_ORIGINATOR, 12, true, true},
        {CORE_TX_ROLE_ORIGINATOR, 0, true, true},
        {CORE_TX_ROLE_ORIGINATOR, 12, false},
        {CORE_TX_ROLE_RELAY, 12, true, true},
        {CORE_TX_ROLE_ORIGINATOR, 29, true, true},
        {CORE_TX_ROLE_ORIGINATOR, 30, false},
        /* Discard all: */
        {CORE_TX_ROLE_ORIGINATOR, 12, true, false},
        {CORE_TX_ROLE_ORIGINATOR, 0, true, false},
        {CORE_TX_ROLE_RELAY, 12, true, false},
        {CORE_TX_ROLE_ORIGINATOR, 29, true, false},
    };

    mesh_opt_core_adv_t cfg;
    mesh_opt_core_adv_get(0, &cfg);

    /* Set repeat count per role, so we can verify that they're correct on the returned packet. */
    uint8_t repeat_counts[CORE_TX_ROLE_COUNT] = {4, NETWORK_RELAY_RETRANSMITS_MAX};
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        cfg.tx_count = repeat_counts[i];
        advertiser_interval_set_ExpectAndSave(mp_advertisers[i], cfg.tx_interval_ms);
        mesh_opt_core_adv_set(i, &cfg);
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        memset(&adv_packet, 0, sizeof(adv_packet));
        network_packet_metadata_t metadata;
        core_tx_alloc_params_t params = {.role           = vector[i].role,
                                         .net_packet_len = vector[i].size,
                                         .p_metadata     = &metadata,
                                         .token          = TOKEN};

        advertiser_packet_alloc_ExpectAndReturn(mp_advertisers[vector[i].role],
                                                vector[i].size + 2,
                                                vector[i].successful_alloc ? &adv_packet
                                                                            : NULL);

        core_tx_alloc_result_t result = mp_interface->packet_alloc(mp_bearer, &params);
        if (vector[i].successful_alloc)
        {
            TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, result);

            /* Duplicate alloc should result in assert */
            TEST_NRF_MESH_ASSERT_EXPECT(mp_interface->packet_alloc(mp_bearer, &params));

            /* Discard packet to allow next alloc */
            if (vector[i].send)
            {
                packet_mesh_net_packet_t buffer;
                memset(&buffer, 0xAB, sizeof(buffer));

                advertiser_packet_send_Expect(mp_advertisers[vector[i].role], &adv_packet);
                mp_interface->packet_send(mp_bearer, buffer.pdu, vector[i].size);

                TEST_ASSERT_EQUAL_HEX8(vector[i].size + 1, adv_packet.packet.payload[0]);
                TEST_ASSERT_EQUAL_HEX8(AD_TYPE_MESH, adv_packet.packet.payload[1]);
                if (vector[i].size > 0)
                {
                    TEST_ASSERT_EQUAL_HEX8_ARRAY(buffer.pdu, &adv_packet.packet.payload[2], vector[i].size);
                }
                TEST_ASSERT_EQUAL(repeat_counts[vector[i].role], adv_packet.config.repeats);
                TEST_ASSERT_EQUAL(TOKEN, adv_packet.token);
            }
            else
            {
                advertiser_packet_discard_Expect(mp_advertisers[vector[i].role], &adv_packet);
                mp_interface->packet_discard(mp_bearer);
            }
        }
        else
        {
            TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, result);
        }

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

    core_tx_complete_Expect(mp_bearer, CORE_TX_ROLE_ORIGINATOR, 1234, TOKEN);
    mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->tx_complete_callback(
        mp_advertisers[CORE_TX_ROLE_ORIGINATOR], packet.token, 1234);
}

void test_config(void)
{
    test_init();
    mesh_opt_core_adv_t cfg;
    mesh_opt_core_adv_get(0, &cfg);
    uint8_t repeat_counts[CORE_TX_ROLE_COUNT] = {1, NETWORK_RELAY_RETRANSMITS_MAX + 1};
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        cfg.tx_count = repeat_counts[i];
        advertiser_interval_set_ExpectAndSave(mp_advertisers[i], cfg.tx_interval_ms);
        mesh_opt_core_adv_set(i, &cfg);
        advertiser_mock_Verify();
    }
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        mesh_opt_core_adv_get(i, &cfg);
        TEST_ASSERT_EQUAL(repeat_counts[i], cfg.tx_count);
    }

    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        cfg.tx_interval_ms = MAX(BEARER_ADV_INT_MIN_MS, 10);
        advertiser_interval_set_ExpectAndSave(mp_advertisers[i], cfg.tx_interval_ms);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_core_adv_set(i, &cfg));
        advertiser_mock_Verify();

        cfg.tx_interval_ms = MAX(BEARER_ADV_INT_MIN_MS, (NETWORK_RELAY_INTERVAL_STEPS_MAX + 1) * 10);
        advertiser_interval_set_ExpectAndSave(mp_advertisers[i], cfg.tx_interval_ms);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_core_adv_set(i, &cfg));
        advertiser_mock_Verify();
    }
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        mp_advertisers[i]->config.advertisement_interval_us = 25000 * (i + 1);
        mesh_opt_core_adv_get(i, &cfg);
        TEST_ASSERT_EQUAL(mp_advertisers[i]->config.advertisement_interval_us / 1000, cfg.tx_interval_ms);
    }

    radio_tx_power_t tx_power = RADIO_POWER_NRF_POS4DBM;
    for (uint32_t i = 0; i < CORE_TX_ROLE_COUNT; ++i)
    {
        advertiser_tx_power_set_Expect(mp_advertisers[i], RADIO_POWER_NRF_POS4DBM);
        mesh_opt_core_tx_power_set(i, tx_power);
    }

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, mesh_opt_core_tx_power_set(CORE_TX_ROLE_COUNT, tx_power));
}

void test_opt_deleters(void)
{
    mesh_config_entry_id_t entry_id = MESH_OPT_CORE_ADV_EID;

    TEST_ASSERT_NOT_NULL(m_mesh_opt_core_adv_params.callbacks.getter);
    TEST_ASSERT_NOT_NULL(m_mesh_opt_core_adv_params.callbacks.setter);
    TEST_ASSERT_NOT_NULL(m_mesh_opt_core_adv_params.callbacks.deleter);

    mp_advertisers[CORE_TX_ROLE_ORIGINATOR]->enabled = true;
    advertiser_interval_set_Expect(mp_advertisers[CORE_TX_ROLE_ORIGINATOR], BEARER_ADV_INT_DEFAULT_MS);
    m_mesh_opt_core_adv_params.callbacks.deleter(entry_id);

    entry_id.record += CORE_TX_ROLE_RELAY;
    mp_advertisers[CORE_TX_ROLE_RELAY]->enabled = false;
    advertiser_enable_Expect(mp_advertisers[CORE_TX_ROLE_RELAY]);
    advertiser_interval_set_Expect(mp_advertisers[CORE_TX_ROLE_RELAY], BEARER_ADV_INT_DEFAULT_MS);
    m_mesh_opt_core_adv_params.callbacks.deleter(entry_id);

    entry_id = MESH_OPT_CORE_TX_POWER_EID;
    advertiser_tx_power_set_Expect(mp_advertisers[CORE_TX_ROLE_ORIGINATOR], RADIO_POWER_NRF_0DBM);
    m_mesh_opt_core_tx_power_params.callbacks.deleter(entry_id);
    entry_id.record += CORE_TX_ROLE_RELAY;
    advertiser_tx_power_set_Expect(mp_advertisers[CORE_TX_ROLE_RELAY], RADIO_POWER_NRF_0DBM);
    m_mesh_opt_core_tx_power_params.callbacks.deleter(entry_id);

    entry_id = MESH_OPT_CORE_ADV_ADDR_EID;
    advertiser_address_default_get_ExpectAnyArgs();
    advertiser_address_set_Expect(mp_advertisers[CORE_TX_ROLE_ORIGINATOR], NULL);
    advertiser_address_set_IgnoreArg_p_addr();
    m_mesh_opt_core_adv_addr_params.callbacks.deleter(entry_id);
    entry_id.record += CORE_TX_ROLE_RELAY;
    advertiser_address_default_get_ExpectAnyArgs();
    advertiser_address_set_Expect(mp_advertisers[CORE_TX_ROLE_RELAY], NULL);
    advertiser_address_set_IgnoreArg_p_addr();
    m_mesh_opt_core_adv_addr_params.callbacks.deleter(entry_id);
}
