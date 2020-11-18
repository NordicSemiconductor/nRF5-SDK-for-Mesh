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

#include <stdlib.h>

#include <unity.h>
#include <cmock.h>

#include "nrf_mesh.h"

#include "beacon.h"
#include "log.h"
#include "network.h"
#include "timer.h"
#include "toolchain.h"
#include "utils.h"
#include "nrf_mesh_utils.h"

#include "msg_cache_mock.h"
#include "transport_mock.h"
#include "net_beacon_mock.h"
#include "net_state_mock.h"
#include "core_tx_mock.h"
#include "core_tx_adv_mock.h"
#include "heartbeat_mock.h"

#define TEST_ASSERT_EQUAL_METADATA(m1, m2)                                                         \
    do                                                                                             \
    {                                                                                              \
        TEST_ASSERT_EQUAL_MESSAGE(m1.dst.type, m2.dst.type, "DST type");                           \
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(m1.dst.value, m2.dst.value, "DST value");                  \
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(m1.src, m2.src, "SRC");                                    \
        TEST_ASSERT_EQUAL_MESSAGE(m1.ttl, m2.ttl, "TTL");                                          \
        TEST_ASSERT_EQUAL_MESSAGE(m1.control_packet, m2.control_packet, "CONTROL");                \
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(m1.internal.sequence_number,                               \
                                        m2.internal.sequence_number,                               \
                                        "SEQNUM");                                                 \
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(m1.internal.iv_index, m2.internal.iv_index, "IV INDEX");   \
    } while (0)

/*************** Static Test Parameters ***************/

#define TEST_NETWORK_KEY     { 0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6 }
#define TEST_PRIVACY_KEY     { 0x8b, 0x84, 0xee, 0xde, 0xc1, 0x00, 0x06, 0x7d, 0x67, 0x09, 0x71, 0xdd, 0x2a, 0xa7, 0x00, 0xcf }
#define TEST_ENCRYPTION_KEY  { 0x09, 0x53, 0xfa, 0x93, 0xe7, 0xca, 0xac, 0x96, 0x38, 0xf5, 0x88, 0x20, 0x22, 0x0a, 0x39, 0x8e }
#define TEST_NID               0x68

/*************** Test Vectors ***************/

/* Macro for specifying the complete, encrypted network packet. In the spec, this data is labelled "NetworkPDU". */
#define TEST_PACKET_ENCRYPTED(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_encrypted_packet = buffer; \
                p_vector->lengths.encrypted = sizeof(buffer); \
            }
/* Macro for specifying the transport packet. In the spec, this data is labelled "TransportPDU". */
#define TEST_PACKET_TRANSPORT(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_transport_packet = buffer; \
                p_vector->lengths.transport = sizeof(buffer); \
            }
/*
 * Macro for specifying the unencrypted header data. This is a concatenation of the fields "IVI NID", "CTL TTL",
 * SEQ, SRC and DST in @tagMeshSp sample data.
 */
#define TEST_PACKET_HEADER(p_vector, ...) \
            { \
                static const uint8_t buffer[] = { __VA_ARGS__ }; \
                p_vector->p_header_data = buffer; \
                p_vector->lengths.header = sizeof(buffer); \
            }
/* Macro for specifying metadata about a test packet. */
#define TEST_PACKET_PARAMS(p_vector, iv_, sequence_, src_, dst_, ttl_, ctl_)                       \
    {                                                                                              \
        p_vector->metadata.internal.iv_index        = iv_;                                         \
        p_vector->metadata.internal.sequence_number = sequence_;                                   \
        p_vector->metadata.src                      = src_;                                        \
        p_vector->metadata.dst.type                 = nrf_mesh_address_type_get(dst_);             \
        p_vector->metadata.dst.value                = dst_;                                        \
        p_vector->metadata.ttl                      = ttl_;                                        \
        p_vector->metadata.control_packet           = ctl_;                                        \
    }

typedef struct
{
    const uint8_t * p_encrypted_packet; /* Complete, encrypted packet. */
    const uint8_t * p_transport_packet; /* Transport packet (encrypted) passed from network to transport. */
    const uint8_t * p_header_data;      /* Unobfuscated packet header. */

    network_packet_metadata_t metadata;        /* Metadata describing the packet. */

    struct
    {
        uint8_t encrypted;
        uint8_t transport;
        uint8_t header;
    } lengths;
} test_vector_t;

/* Specifies all available test vectors; corresponds to sample data in the spec, see below: */
#define TEST_VECTORS_ALL                   { 1, 2, 3, 6, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22, 23, 24}

/* Specifies which sets of test vectors to use for specific tests: */
#define NETWORK_PKT_IN_TEST_VECTORS        TEST_VECTORS_ALL
#define SELF_RECEIVE_TEST_VECTORS          TEST_VECTORS_ALL
#define NETWORK_PKT_OUT_TEST_VECTORS       { 1, 2, 3, 6, 7, 8, 9, 16, 17, 18, 19 /* Skip cases with other IV indices */ }
#define INVALID_NETKEY_TEST_VECTORS        TEST_VECTORS_ALL
#define NETWORK_PKT_RELAY_TEST_VECTORS     TEST_VECTORS_ALL

/*
 * Retrieves a test vector. These are indexed by the number used in @tagMeshSp. Some test vectors are skipped,
 * as they use friendship credentials or other unsupported features.
 */
static void get_test_vector(unsigned int vector, test_vector_t * p_vector)
{
    switch (vector)
    {
        case 1:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xec, 0xa4, 0x87, 0x51, 0x67, 0x65, 0xb5, 0xe5, 0xbf, 0xda, 0xcb,
                    0xaf, 0x6c, 0xb7, 0xfb, 0x6b, 0xff, 0x87, 0x1f, 0x03, 0x54, 0x44, 0xce, 0x83, 0xa6, 0x70, 0xdf);
            TEST_PACKET_TRANSPORT(p_vector, 0x03, 0x4b, 0x50, 0x05, 0x7e, 0x40, 0x00, 0x00, 0x01, 0x00, 0x00);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x00, 0x00, 0x01, 0x12, 0x01, 0xff, 0xfd);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000001, 0x1201, 0xfffd, 0, 1);
            break;
        }
        case 2:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xd4, 0xc8, 0x26, 0x29, 0x6d, 0x79, 0x79, 0xd7, 0xdb, 0xc0, 0xc9, 0xb4,
                0xd4, 0x3e, 0xeb, 0xec, 0x12, 0x9d, 0x20, 0xa6, 0x20, 0xd0, 0x1e);
            TEST_PACKET_TRANSPORT(p_vector, 0x04, 0x32, 0x03, 0x08, 0xba, 0x07, 0x2f);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x01, 0x48, 0x20, 0x23, 0x45, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014820, 0x2345, 0x1201, 0, 1);
            break;
        }
        case 3:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xda, 0x06, 0x2b, 0xc9, 0x6d, 0xf2, 0x53, 0x27, 0x30, 0x86, 0xb8, 0xc5,
                0xee, 0x00, 0xbd, 0xd9, 0xcf, 0xcc, 0x62, 0xa2, 0xdd, 0xf5, 0x72);
            TEST_PACKET_TRANSPORT(p_vector, 0x04, 0xfa, 0x02, 0x05, 0xa6, 0x00, 0x0a);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x80, 0x2b, 0x38, 0x32, 0x2f, 0xe3, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x2b3832, 0x2fe3, 0x1201, 0, 1);
            break;
        }
        case 6:
        {
            /* Test message #6 contains two messages, only the first one is included here. */
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xca, 0xb5, 0xc5, 0x34, 0x8a, 0x23, 0x0a, 0xfb, 0xa8, 0xc6, 0x3d, 0x4e,
                    0x68, 0x63, 0x64, 0x97, 0x9d, 0xea, 0xf4, 0xfd, 0x40, 0x96, 0x11, 0x45, 0x93, 0x9c, 0xda, 0x0e);
            TEST_PACKET_TRANSPORT(p_vector, 0x80, 0x26, 0xac, 0x01, 0xee, 0x9d, 0xdd, 0xfd, 0x21, 0x69, 0x32, 0x6d, 0x23, 0xf3, 0xaf, 0xdf);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x04, 0x31, 0x29, 0xab, 0x00, 0x03, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x3129ab, 0x0003, 0x1201, 4, 0);
            break;
        }
        case 7:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xe4, 0x76, 0xb5, 0x57, 0x9c, 0x98, 0x0d, 0x0d, 0x73, 0x0f, 0x94, 0xd7,
                    0xf3, 0x50, 0x9d, 0xf9, 0x87, 0xbb, 0x41, 0x7e, 0xb7, 0xc0, 0x5f);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0xa6, 0xac, 0x00, 0x00, 0x00, 0x02);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x8b, 0x01, 0x48, 0x35, 0x23, 0x45, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014835, 0x2345, 0x0003, 0x0b, 1);
            break;
        }
        case 8:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x4d, 0xaa, 0x62, 0x67, 0xc2, 0xcf, 0x0e, 0x2f, 0x91, 0xad, 0xd6, 0xf0,
                    0x6e, 0x66, 0x00, 0x68, 0x44, 0xce, 0xc9, 0x7f, 0x97, 0x31, 0x05, 0xae, 0x25, 0x34, 0xf9, 0x58);
            TEST_PACKET_TRANSPORT(p_vector, 0x80, 0x26, 0xac, 0x01, 0xee, 0x9d, 0xdd, 0xfd, 0x21, 0x69, 0x32, 0x6d, 0x23, 0xf3, 0xaf, 0xdf);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x04, 0x31, 0x29, 0xad, 0x00, 0x03, 0x12, 0x01);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x3129ad, 0x0003, 0x1201, 0x4, 0);
            break;
        }
        case 9:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xae, 0xc4, 0x67, 0xed, 0x49, 0x01, 0xd8, 0x5d, 0x80, 0x6b, 0xbe, 0xd2,
                    0x48, 0x61, 0x4f, 0x93, 0x80, 0x67, 0xb0, 0xd9, 0x83, 0xbb, 0x7b);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0xa6, 0xac, 0x00, 0x00, 0x00, 0x03);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x8b, 0x01, 0x48, 0x36, 0x23, 0x45, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x014836, 0x2345, 0x0003, 0x0b, 1);
            break;
        }
        case 16:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xe8, 0x0e, 0x5d, 0xa5, 0xaf, 0x0e, 0x6b, 0x9b, 0xe7, 0xf5, 0xa6, 0x42,
                    0xf2, 0xf9, 0x86, 0x80, 0xe6, 0x1c, 0x3a, 0x8b, 0x47, 0xf2, 0x28);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0x89, 0x51, 0x1b, 0xf1, 0xd1, 0xa8, 0x1c, 0x11, 0xdc, 0xef);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x0b, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000006, 0x1201, 0x0003, 0x0b, 0);
            break;
        }
        case 17:
        {
            /* Values from errata 9693 */
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0xb2, 0xbd, 0x2c, 0x1e, 0x1b, 0x6f, 0x2a, 0x80, 0xd3, 0x81, 0xb9, 0x1f,
                    0x82, 0x4d, 0xd4, 0xf0, 0xa3, 0xcd, 0x54, 0xce, 0xa2, 0x3b, 0x7a);
            TEST_PACKET_TRANSPORT(p_vector, 0x00, 0x89, 0x51, 0x1b, 0xf1, 0xd1, 0xa8, 0x1c, 0x11, 0xdc, 0xef);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x0a, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x03);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000006, 0x1201, 0x0003, 0x0a, 0);
            break;
        }
        case 18:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x48, 0xcb, 0xa4, 0x37, 0x86, 0x0e, 0x56, 0x73, 0x72, 0x8a, 0x62, 0x7f,
                    0xb9, 0x38, 0x53, 0x55, 0x08, 0xe2, 0x1a, 0x6b, 0xaf, 0x57);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x5a, 0x8b, 0xde, 0x6d, 0x91, 0x06, 0xea, 0x07, 0x8a);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x03, 0x00, 0x00, 0x07, 0x12, 0x01, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000007, 0x1201, 0xffff, 0x03, 0);
            break;
        }
        case 19:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0x68, 0x11, 0x0e, 0xde, 0xec, 0xd8, 0x3c, 0x30, 0x10, 0xa0, 0x5e, 0x1b, 0x23,
                    0xa9, 0x26, 0x02, 0x3d, 0xa7, 0x5d, 0x25, 0xba, 0x91, 0x79, 0x37, 0x36);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0xca, 0x6c, 0xd8, 0x8e, 0x69, 0x8d, 0x12, 0x65, 0xf4, 0x3f, 0xc5);
            TEST_PACKET_HEADER(p_vector, 0x68, 0x03, 0x00, 0x00, 0x09, 0x12, 0x01, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345678, 0x000009, 0x1201, 0xffff, 0x03, 0);
            break;
        }
        case 20:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x5c, 0xca, 0x51, 0xe2, 0xe8, 0x99, 0x8c, 0x3d, 0xc8, 0x73, 0x44, 0xa1,
                    0x6c, 0x78, 0x7f, 0x6b, 0x08, 0xcc, 0x89, 0x7c, 0x94, 0x1a, 0x53, 0x68);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x9c, 0x98, 0x03, 0xe1, 0x10, 0xfe, 0xa9, 0x29, 0xe9, 0x54, 0x2d);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x09, 0x12, 0x34, 0xff, 0xff);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x070809, 0x1234, 0xffff, 0x03, 0);
            break;
        }
        case 21:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0xb1, 0x05, 0x1f, 0x5e, 0x94, 0x5a, 0xe4, 0xd6, 0x11, 0x35, 0x8e, 0xaf,
                    0x17, 0x79, 0x6a, 0x6c, 0x98, 0x97, 0x7f, 0x69, 0xe5, 0x87, 0x2c, 0x46, 0x20);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x2f, 0xa7, 0x30, 0xfd, 0x98, 0xf6, 0xe4, 0xbd, 0x12, 0x0e, 0xa9, 0xd6);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0a, 0x12, 0x34, 0x81, 0x05);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080a, 0x1234, 0x8105, 0x03, 0);
            break;
        }
        case 22:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0xd8, 0x5c, 0xae, 0xce, 0xf1, 0xe3, 0xed, 0x31, 0xf3, 0xfd, 0xcf, 0x88,
                    0xa4, 0x11, 0x13, 0x5f, 0xea, 0x55, 0xdf, 0x73, 0x0b, 0x6b, 0x28, 0xe2, 0x55);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x38, 0x71, 0xb9, 0x04, 0xd4, 0x31, 0x52, 0x63, 0x16, 0xca, 0x48, 0xa0);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0b, 0x12, 0x34, 0xb5, 0x29);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080b, 0x1234, 0xb529, 0x03, 0);
            break;
        }
        case 23:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x77, 0xa4, 0x8d, 0xd5, 0xfe, 0x2d, 0x7a, 0x9d, 0x69, 0x6d, 0x3d, 0xd1,
                    0x6a, 0x75, 0x48, 0x96, 0x96, 0xf0, 0xb7, 0x0c, 0x71, 0x1b, 0x88, 0x13, 0x85);
            TEST_PACKET_TRANSPORT(p_vector, 0x66, 0x24, 0x56, 0xdb, 0x5e, 0x31, 0x00, 0xee, 0xf6, 0x5d, 0xaa, 0x7a, 0x38);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0c, 0x12, 0x34, 0x97, 0x36);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080c, 0x1234, 0x9736, 0x03, 0);
            break;
        }
        case 24:
        {
            TEST_PACKET_ENCRYPTED(p_vector, 0xe8, 0x34, 0x58, 0x6b, 0xab, 0xde, 0xf3, 0x94, 0xe9, 0x98, 0xb4, 0x08, 0x1f,
                    0x5a, 0x73, 0x08, 0xce, 0x3e, 0xdb, 0xb3, 0xb0, 0x6c, 0xde, 0xcd, 0x02, 0x8e, 0x30, 0x7f, 0x1c);
            TEST_PACKET_TRANSPORT(p_vector, 0xe6, 0xa0, 0x34, 0x01, 0xde, 0x15, 0x47, 0x11, 0x84, 0x63, 0x12, 0x3e, 0x5f, 0x6a, 0x17, 0xb9);
            TEST_PACKET_HEADER(p_vector, 0xe8, 0x03, 0x07, 0x08, 0x0d, 0x12, 0x34, 0x97, 0x36);
            TEST_PACKET_PARAMS(p_vector, 0x12345677, 0x07080d, 0x1234, 0x9736, 0x03, 0);
            break;
        }
        default:
            TEST_FAIL_MESSAGE("Unknown test vector requested!");
    }
}

/*************** Static Variables and Stuff ***************/

static nrf_mesh_network_secmat_t test_network;
static uint32_t m_net_secmat_get_calls_expect;
static nrf_mesh_network_secmat_t * mp_net_secmats;
static uint32_t m_net_secmat_count;
static uint16_t m_rx_address;

static bool m_core_tx_alloc_success;

/*************** Additional Mock Functions ***************/

timestamp_t timer_now(void)
{
    return 0;
}

static struct
{
    packet_mesh_trs_packet_t packet;
    network_packet_metadata_t net_metadata;
    uint32_t calls;
} m_transport_packet_in;

uint32_t transport_packet_in_mock_cb(const packet_mesh_trs_packet_t * p_packet,
                                     uint32_t trs_packet_len,
                                     const network_packet_metadata_t * p_net_metadata,
                                     const nrf_mesh_rx_metadata_t * p_rx_metadata,
                                     int cmock_num_calls)
{
    memcpy(&m_transport_packet_in.packet, p_packet, trs_packet_len);
    memcpy(&m_transport_packet_in.net_metadata, p_net_metadata, sizeof(network_packet_metadata_t));
    m_transport_packet_in.calls++;
    return NRF_SUCCESS;
}

static packet_mesh_net_packet_t m_core_tx_buffer;

core_tx_bearer_bitmap_t core_tx_packet_alloc_cb(const core_tx_alloc_params_t * p_params,
                                                uint8_t ** pp_packet,
                                                int calls)
{
    *pp_packet = (uint8_t *) &m_core_tx_buffer;
    return m_core_tx_alloc_success;
}

/*************** Static Helper Functions and Types ***************/

/*
 * Provisions the test network. Will use test vector keys if indicated, otherwise, it'll be dummies.
 */
static void provision(bool use_real_keys)
{
    if (use_real_keys)
    {
        uint8_t privacy[] = TEST_PRIVACY_KEY;
        uint8_t encryption[] = TEST_ENCRYPTION_KEY;
        test_network.nid = TEST_NID;
        memcpy(test_network.privacy_key, privacy, NRF_MESH_KEY_SIZE);
        memcpy(test_network.encryption_key, encryption, NRF_MESH_KEY_SIZE);
    }
    else
    {
        memset(test_network.privacy_key, 0x11, NRF_MESH_KEY_SIZE);
        memset(test_network.encryption_key, 0x22, NRF_MESH_KEY_SIZE);
        test_network.nid = 0x12;
    }
}

void nrf_mesh_net_secmat_next_get(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat)
{
    TEST_ASSERT_TRUE(m_net_secmat_get_calls_expect > 0);
    TEST_ASSERT_NOT_NULL(mp_net_secmats);
    TEST_ASSERT_NOT_EQUAL(0, m_net_secmat_count);
    m_net_secmat_get_calls_expect--;

    if (*pp_secmat == NULL)
    {
        *pp_secmat = &mp_net_secmats[0];
    }
    else if ((uint32_t)(*pp_secmat - &mp_net_secmats[0]) >= m_net_secmat_count - 1)
    {
        *pp_secmat = NULL;
    }
    else
    {
        (*pp_secmat)++; /* get next */
    }
}

bool nrf_mesh_rx_address_get(uint16_t address, nrf_mesh_address_t * p_address)
{
    return (m_rx_address == address);
}


/*************** Test Initialization and Finalization ***************/

void setUp(void)
{
    core_tx_mock_Init();
    core_tx_adv_mock_Init();
    msg_cache_mock_Init();
    transport_mock_Init();
    net_state_mock_Init();
    net_beacon_mock_Init();
    heartbeat_mock_Init();

    m_net_secmat_get_calls_expect = 0;
    mp_net_secmats = NULL;
    m_net_secmat_count = 0;
    m_rx_address = 0;
    m_core_tx_alloc_success = true;

    __LOG_INIT((LOG_SRC_NETWORK | LOG_SRC_ENC | LOG_SRC_TEST), 3, LOG_CALLBACK_DEFAULT);

    nrf_mesh_init_params_t init_params;
    memset(&init_params, 0, sizeof(nrf_mesh_init_params_t));

    net_state_init_Expect();
    net_beacon_init_Expect();
    network_init(&init_params);
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    core_tx_adv_mock_Verify();
    core_tx_adv_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
}

/*************** Test Cases ***************/

void test_packet_in(void)
{
    nrf_mesh_rx_metadata_t rx_metadata;
    const uint8_t run_testvectors[] = NETWORK_PKT_IN_TEST_VECTORS;
    provision(true);

    transport_packet_in_StubWithCallback(transport_packet_in_mock_cb);
    core_tx_packet_alloc_StubWithCallback(core_tx_packet_alloc_cb);
    core_tx_packet_send_Ignore();

    mp_net_secmats = &test_network;
    m_net_secmat_count = 1;
    /* Processes all the test vectors as incoming packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        msg_cache_entry_exists_IgnoreAndReturn(false);
        msg_cache_entry_add_Expect(test_vector.metadata.src, test_vector.metadata.internal.sequence_number);

        m_net_secmat_get_calls_expect = 1;

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.metadata.internal.iv_index & 0x01, test_vector.metadata.internal.iv_index);

        core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_packet_in(test_vector.p_encrypted_packet, test_vector.lengths.encrypted, &rx_metadata));

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        TEST_ASSERT_EQUAL_METADATA(test_vector.metadata, m_transport_packet_in.net_metadata);
        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_transport_packet, &m_transport_packet_in.packet, test_vector.lengths.transport);
    }

    /* Add more networks, make sure we hit the right one for all test vectors. */
    nrf_mesh_network_secmat_t secmats[4];
    memset(&secmats[0], 0x12, sizeof(nrf_mesh_network_secmat_t));
    memset(&secmats[1], 0x34, sizeof(nrf_mesh_network_secmat_t));
    memcpy(&secmats[2], &test_network, sizeof(nrf_mesh_network_secmat_t));
    memset(&secmats[3], 0x56, sizeof(nrf_mesh_network_secmat_t));
    mp_net_secmats = secmats;
    m_net_secmat_count = 4;

    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        msg_cache_entry_exists_IgnoreAndReturn(false);
        msg_cache_entry_add_Expect(test_vector.metadata.src, test_vector.metadata.internal.sequence_number);

        m_net_secmat_get_calls_expect = 3;

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.metadata.internal.iv_index & 0x01, test_vector.metadata.internal.iv_index);
        core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_packet_in(test_vector.p_encrypted_packet, test_vector.lengths.encrypted, &rx_metadata));

        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);

        TEST_ASSERT_EQUAL_METADATA(test_vector.metadata, m_transport_packet_in.net_metadata);
        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_transport_packet, &m_transport_packet_in.packet, test_vector.lengths.transport);
    }
}

void test_packet_out(void)
{
    const uint8_t run_testvectors[] = NETWORK_PKT_OUT_TEST_VECTORS;
    provision(true);

    m_core_tx_alloc_success = true;
    core_tx_packet_alloc_StubWithCallback(core_tx_packet_alloc_cb);
    core_tx_packet_send_Ignore();

    net_state_iv_index_lock_Ignore();

    /* Processes all the test vectors as outbound packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        net_state_iv_index_and_seqnum_alloc_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
        net_state_iv_index_and_seqnum_alloc_IgnoreArg_p_iv_index();
        net_state_iv_index_and_seqnum_alloc_IgnoreArg_p_seqnum();
        net_state_iv_index_and_seqnum_alloc_ReturnMemThruPtr_p_iv_index(
            &test_vector.metadata.internal.iv_index,
            sizeof(test_vector.metadata.internal.iv_index));
        net_state_iv_index_and_seqnum_alloc_ReturnMemThruPtr_p_seqnum(
            &test_vector.metadata.internal.sequence_number,
            sizeof(test_vector.metadata.internal.sequence_number));

        network_tx_packet_buffer_t net_packet_buffer;
        net_packet_buffer.user_data.p_metadata = &test_vector.metadata;
        net_packet_buffer.user_data.payload_len = test_vector.lengths.transport;
        net_packet_buffer.user_data.role = CORE_TX_ROLE_ORIGINATOR;
        net_packet_buffer.user_data.bearer_selector = CORE_TX_BEARER_TYPE_ADV;

        memcpy(&m_core_tx_buffer.pdu[9],
               test_vector.p_transport_packet,
               test_vector.lengths.transport);
        test_vector.metadata.p_security_material = &test_network;

        TEST_ASSERT_EQUAL(NRF_SUCCESS, network_packet_alloc(&net_packet_buffer));

        network_packet_send(&net_packet_buffer);

        TEST_ASSERT_EQUAL_MEMORY(test_vector.p_encrypted_packet, &m_core_tx_buffer, test_vector.lengths.encrypted);

        net_state_mock_Verify();
    }

    /* Fail sending */
    m_core_tx_alloc_success = false;
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "Running test vector %d\n", run_testvectors[i]);

        network_tx_packet_buffer_t net_packet_buffer;
        net_packet_buffer.user_data.p_metadata = &test_vector.metadata;
        net_packet_buffer.user_data.p_metadata->p_security_material = &test_network;
        net_packet_buffer.user_data.payload_len = test_vector.lengths.transport;
        net_packet_buffer.user_data.role = CORE_TX_ROLE_ORIGINATOR;
        net_packet_buffer.user_data.bearer_selector = CORE_TX_BEARER_TYPE_ADV;

        TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, network_packet_alloc(&net_packet_buffer));
    }
}

void test_invalid_net_key_in(void)
{
    nrf_mesh_rx_metadata_t rx_metadata;
    const uint8_t run_testvectors[] = INVALID_NETKEY_TEST_VECTORS;

    provision(false);

    transport_packet_in_StubWithCallback(transport_packet_in_mock_cb);
    msg_cache_entry_exists_IgnoreAndReturn(false);

    mp_net_secmats = &test_network;
    m_net_secmat_count = 1;

    /* Processes all the test vectors as incoming packets. */
    for (unsigned int i = 0; i < sizeof(run_testvectors); ++i)
    {
        test_vector_t test_vector;
        get_test_vector(run_testvectors[i], &test_vector);

        m_transport_packet_in.calls   = 0;
        m_net_secmat_get_calls_expect = 2;

        net_state_rx_iv_index_get_ExpectAndReturn(test_vector.metadata.internal.iv_index & 0x01, test_vector.metadata.internal.iv_index);

        TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, network_packet_in(test_vector.p_encrypted_packet, test_vector.lengths.encrypted, &rx_metadata));

        TEST_ASSERT_EQUAL(0, m_transport_packet_in.calls);
        TEST_ASSERT_EQUAL(0, m_net_secmat_get_calls_expect);
    }
}
