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
#ifndef TEST_HELPER_H__
#define TEST_HELPER_H__

#include "unity.h"

/**
 * @internal
 * @defgroup TEST_HELPER Test helper
 * Header for defining helper macros for unit tests.
 * @{
 */

/* Define any assert macros for comparing structures.
 *
 * All macros must follow the UNITY_TEST_ASSERT_EQUAL_my_struct_type_t(expected, actual, line, message)
 * format. Their expansion should use the UNITY_TEST_ASSERT_EQUAL_... macros, instead of the
 * TEST_ASSERT_EQUAL macros, and they should pass the line and message parameters forward to these
 * asserts. Do not print anything inside them, and surround them by do {} while(0) if they're
 * multiline. The macros will also be used by the mocks. Optionally create wrappers for
 * TEST_ASSERT_EQUAL_... macros to use in test code.
 *
 * The expected and actual parameters will be passed in as *p_pointer, so to access their members, do
 * (expected).member.
 */

#define UNITY_TEST_ASSERT_EQUAL_transport_control_packet_t(expected, actual, line, message)        \
    do                                                                                             \
    {                                                                                              \
        UNITY_TEST_ASSERT_EQUAL_HEX8((expected).opcode, (actual).opcode, line, message);           \
        UNITY_TEST_ASSERT_EQUAL_PTR((expected).p_net_secmat,                                       \
                                    (actual).p_net_secmat,                                         \
                                    line,                                                          \
                                    message);                                                      \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).dst.type, (actual).dst.type, line, message);      \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).dst.value, (actual).dst.value, line, message);    \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).dst.p_virtual_uuid,                               \
                                      (actual).dst.p_virtual_uuid,                                 \
                                      line,                                                        \
                                      message);                                                    \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).src, (actual).src, line, message);                \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).ttl, (actual).ttl, line, message);                \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).reliable, (actual).reliable, line, message);      \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).data_len, (actual).data_len, line, message);      \
        UNITY_TEST_ASSERT_EQUAL_PTR((expected).p_data, (actual).p_data, line, message);            \
    \
} while (0)
#define TEST_ASSERT_EQUAL_transport_control_packet_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_transport_control_packet_t(expected, actual, __LINE__, "")


#define UNITY_TEST_ASSERT_EQUAL_network_packet_metadata_t(expected, actual, line, message) do { \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).src, (actual).src, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).dst.value, (actual).dst.value, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX8((expected).dst.type, (actual).dst.type, line, message); \
        UNITY_TEST_ASSERT_EQUAL_PTR((expected).dst.p_virtual_uuid, (actual).dst.p_virtual_uuid, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX8((expected).ttl, (actual).ttl, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX8((expected).control_packet, (actual).control_packet, line, message); \
        UNITY_TEST_ASSERT_EQUAL_PTR((expected).p_security_material, (actual).p_security_material, line, message); \
    } while (0)
#define TEST_ASSERT_EQUAL_network_packet_metadata_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_network_packet_metadata_t(expected, actual, __LINE__, "")

/**
 * nrf_mesh_evt_t. Only the events that are actually used are included, will assert if you try to
 * use it for unimplemented events. Add more cases to the switch when necessary, and try to qualify
 * all relevant parameters, if possible.
 */
#define UNITY_TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual, line, message)                    \
    do                                                                                             \
    {                                                                                              \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).type, (actual).type, line, message);                \
        switch ((expected).type)                                                                   \
        {                                                                                          \
            case NRF_MESH_EVT_TX_COMPLETE:                                                         \
                UNITY_TEST_ASSERT_EQUAL_HEX32((expected).params.tx_complete.token,                 \
                                              (actual).params.tx_complete.token,                   \
                                              line,                                                \
                                              message);                                            \
                break;                                                                             \
            case NRF_MESH_EVT_FLASH_FAILED:                                                        \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.flash_failed.user,                   \
                                            (actual).params.flash_failed.user,                     \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_flash_entry,          \
                                            (actual).params.flash_failed.p_flash_entry,            \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_flash_page,           \
                                            (actual).params.flash_failed.p_flash_page,             \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_area,                 \
                                            (actual).params.flash_failed.p_area,                   \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.flash_failed.page_count,             \
                                            (actual).params.flash_failed.page_count,               \
                                            line,                                                  \
                                            message);                                              \
                break;                                                                             \
            case NRF_MESH_EVT_HB_MESSAGE_RECEIVED:                                                 \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.init_ttl,                 \
                                            (actual).params.hb_message.init_ttl,                   \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.hops,                     \
                                            (actual).params.hb_message.hops,                       \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.features,                 \
                                            (actual).params.hb_message.features,                   \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.src,                      \
                                            (actual).params.hb_message.src,                        \
                                            line,                                                  \
                                            message);                                              \
                break;                                                                             \
            case NRF_MESH_EVT_NET_BEACON_RECEIVED:                                                 \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_beacon_info,            \
                                            (actual).params.net_beacon.p_beacon_info,              \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_beacon_secmat,          \
                                            (actual).params.net_beacon.p_beacon_secmat,            \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_rx_metadata,            \
                                            (actual).params.net_beacon.p_rx_metadata,              \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.iv_index,                 \
                                            (actual).params.net_beacon.iv_index,                   \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.flags.iv_update,          \
                                            (actual).params.net_beacon.flags.iv_update,            \
                                            line,                                                  \
                                            message);                                              \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.flags.key_refresh,        \
                                            (actual).params.net_beacon.flags.key_refresh,          \
                                            line,                                                  \
                                            message);                                              \
                break;                                                                             \
            default:                                                                               \
            {                                                                                      \
                char error_msg[256];                                                               \
                sprintf(error_msg, "Unimplemented event type %#x", (expected).type);               \
                UNITY_TEST_FAIL(line, error_msg);                                                  \
                break;                                                                             \
            }                                                                                      \
        }                                                                                          \
    \
} while (0)
#define TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual, __LINE__, "")


#define UNITY_TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual, line, message) do { \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).role, (actual).role, line, message); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).net_packet_len, (actual).net_packet_len, line, message); \
        UNITY_TEST_ASSERT_EQUAL_network_packet_metadata_t(*(expected).p_metadata, *(actual).p_metadata, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).token, (actual).token, line, message); \
    } while (0)
#define TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual, __LINE__, "")

/** @} */

#endif /* TEST_HELPER_H__ */

