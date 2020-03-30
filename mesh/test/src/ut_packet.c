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
#include "packet.h"

#define MESH_NET_PACKET_MIN_LENGTH 5
#define MESH_NET_PACKET_MAX_LENGTH 16

void setUp(void)
{

}

void tearDown(void)
{

}

/***************************************/

void test_packet_size(void)
{
    /* test struct defintions and packing */
    /* TODO: should be compiled with all toolchains, as this could be different between them */
    TEST_ASSERT_EQUAL(3, sizeof(ble_packet_hdr_t));
    TEST_ASSERT_EQUAL(40, sizeof(packet_t));
    TEST_ASSERT_EQUAL(2, sizeof(ble_ad_data_t));
}

void test_helper_functions(void)
{
    packet_t packet;
    packet_payload_size_set(&packet, 13);
    TEST_ASSERT_EQUAL(13 + BLE_GAP_ADDR_LEN, packet.header.length);
    TEST_ASSERT_EQUAL(13, packet_payload_size_get(&packet));

    /* iterate through ad types */
    memset(&packet, 0, sizeof(packet));
    const uint8_t ad_lengths[] = {1, 2, 3, 4, 11, 1, 31};
    uint8_t offset = 0;
    for (uint32_t i = 0; i < sizeof(ad_lengths); ++i)
    {
        ble_ad_data_t * p_ad = (ble_ad_data_t *) &packet.payload[offset];
        p_ad->length = ad_lengths[i];
        p_ad->type = i;
        offset += ad_lengths[i] + 1;
    }
    uint8_t index = 0;
    for (ble_ad_data_t * p_ad = (ble_ad_data_t *) &packet.payload[0];
         (uint8_t *) p_ad < &packet.payload[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
         p_ad = packet_ad_type_get_next(p_ad))
    {
        TEST_ASSERT_EQUAL(ad_lengths[index], p_ad->length);
        TEST_ASSERT_EQUAL(index, p_ad->type);
        index++;
    }
    TEST_ASSERT_EQUAL(sizeof(ad_lengths), index);
}
