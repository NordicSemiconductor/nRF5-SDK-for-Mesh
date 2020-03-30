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
#ifndef TEST_HELPER_H__
#define TEST_HELPER_H__

#include "unity.h"

/* #define STRINGIZE(A) #A */

#define LCONCAT(MACRO, LITERAL) #MACRO LITERAL

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

#define UNITY_TEST_ASSERT_EQUAL_heartbeat_publication_state_t(expected, actual, line, message)                              \
    do                                                                                                                      \
    {                                                                                                                       \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).dst, (actual).dst, line, "publication::dst");                              \
        UNITY_TEST_ASSERT_EQUAL_UINT32((expected).count, (actual).count, line, "publication::count");                       \
        UNITY_TEST_ASSERT_EQUAL_UINT32((expected).period, (actual).period, line, "publication::period");                    \
        UNITY_TEST_ASSERT_EQUAL_UINT8((expected).ttl, (actual).ttl, line, "publication::ttl");                              \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).features, (actual).features, line, "publication::features");               \
        UNITY_TEST_ASSERT_EQUAL_UINT16((expected).netkey_index, (actual).netkey_index, line, "publication::netkey_index");  \
    } while (0)
#define TEST_ASSERT_EQUAL_heartbeat_publication_state_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_heartbeat_publication_state_t(expected, actual, __LINE__, "")

#define UNITY_TEST_ASSERT_EQUAL_heartbeat_subscription_state_t(expected, actual, line, message)                      \
    do                                                                                                               \
    {                                                                                                                \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).src, (actual).src, line, "subscription::src");                      \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).dst, (actual).dst, line, "subscription::dst");                      \
        UNITY_TEST_ASSERT_EQUAL_UINT32((expected).count, (actual).count, line, "subscription::count");               \
        UNITY_TEST_ASSERT_EQUAL_UINT32((expected).period, (actual).period, line, "subscription::period");            \
        UNITY_TEST_ASSERT_EQUAL_UINT16((expected).min_hops, (actual).min_hops, line, "subscription::min_hops");      \
        UNITY_TEST_ASSERT_EQUAL_UINT16((expected).max_hops, (actual).max_hops, line, "subscription::max_hops");      \
    } while (0)
#define TEST_ASSERT_EQUAL_heartbeat_subscription_state_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_heartbeat_subscription_state_t(expected, actual, __LINE__, "")

#define UNITY_TEST_ASSERT_EQUAL_nrf_mesh_address_t(expected, actual, line, message)                \
    do                                                                                             \
    {                                                                                              \
        UNITY_TEST_ASSERT_EQUAL_HEX8((expected).type, (actual).type, line, message);               \
        UNITY_TEST_ASSERT_EQUAL_HEX16((expected).value, (actual).value, line, message);            \
        if ((expected).type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)                                      \
        {                                                                                          \
            UNITY_TEST_ASSERT_EQUAL_HEX8_ARRAY((expected).p_virtual_uuid,                          \
                                               (actual).p_virtual_uuid,                            \
                                               16,                                                 \
                                               line,                                               \
                                               message);                                           \
        }                                                                                          \
    } while (0)
#define TEST_ASSERT_EQUAL_nrf_mesh_address_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_nrf_mesh_address_t(expected, actual, __LINE__, "")

#define UNITY_TEST_ASSERT_EQUAL_mesh_friendship_lpn_t(expected, actual, line, msg) \
    do                                                                  \
    {                                                                   \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).src, (actual).src, line, LCONCAT(msg, "mesh_friendship_lpn_t::src")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).prev_friend_src, (actual).prev_friend_src, line, LCONCAT(msg, "mesh_friendship_lpn_t::prev_friend_src")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).element_count, (actual).element_count, line, LCONCAT(msg, "mesh_friendship_lpn_t::element_count")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).request_count, (actual).request_count, line, LCONCAT(msg, "mesh_friendship_lpn_t::request_count")); \
    } while (0)
#define TEST_ASSERT_EQUAL_mesh_friendship_lpn_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_mesh_friendship_lpn_t(excepted, actual, __LINE__, "")

#define UNITY_TEST_ASSERT_EQUAL_mesh_friendship_t(expected, actual, line, msg) \
    do                                                                  \
    {                                                                   \
        UNITY_TEST_ASSERT_EQUAL_mesh_friendship_lpn_t((expected).lpn, (actual).lpn, line, LCONCAT(msg, "mesh_friendship_t::lpn")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).poll_timeout_ms, (actual).poll_timeout_ms, line, LCONCAT(msg, "mesh_friendship_t::poll_timeout_ms")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).poll_count, (actual).poll_count, line, LCONCAT(msg, "mesh_friendship_t::poll_count")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).receive_delay_ms, (actual).receive_delay_ms, line, LCONCAT(msg, "mesh_friendship_t::receive_delay_ms")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).receive_window_ms, (actual).receive_window_ms, line, LCONCAT(msg, "mesh_friendship_t::receive_window_ms")); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).avg_rssi, (actual).avg_rssi, line, LCONCAT(msg, "mesh_friendship_t::avg_rssi")); \
    } while (0)
#define TEST_ASSERT_EQUAL_mesh_friendship_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_mesh_friendship_t(excepted, actual, __LINE__, "")

/**
 * nrf_mesh_evt_t. Only the events that are actually used are included. The macro will assert if
 * you try to use it for unimplemented events. Add more cases to the switch when necessary, and try
 * to qualify all relevant parameters, if possible.
 */
#define UNITY_TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual, line, msg)                          \
    do                                                                                               \
    {                                                                                                \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).type, (actual).type, line, msg);                      \
        switch ((expected).type)                                                                     \
        {                                                                                            \
            case NRF_MESH_EVT_TX_COMPLETE:                                                           \
                UNITY_TEST_ASSERT_EQUAL_HEX32((expected).params.tx_complete.token,                   \
                                              (actual).params.tx_complete.token,                     \
                                              line,                                                  \
                                              msg);                                                  \
                break;                                                                               \
            case NRF_MESH_EVT_FLASH_FAILED:                                                          \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.flash_failed.user,                     \
                                            (actual).params.flash_failed.user,                       \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_flash_entry,            \
                                            (actual).params.flash_failed.p_flash_entry,              \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_flash_page,             \
                                            (actual).params.flash_failed.p_flash_page,               \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.flash_failed.p_area,                   \
                                            (actual).params.flash_failed.p_area,                     \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.flash_failed.page_count,               \
                                            (actual).params.flash_failed.page_count,                 \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_HB_MESSAGE_RECEIVED:                                                   \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.init_ttl,                   \
                                            (actual).params.hb_message.init_ttl,                     \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.hops,                       \
                                            (actual).params.hb_message.hops,                         \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.features,                   \
                                            (actual).params.hb_message.features,                     \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.hb_message.src,                        \
                                            (actual).params.hb_message.src,                          \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_NET_BEACON_RECEIVED:                                                   \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_beacon_info,              \
                                            (actual).params.net_beacon.p_beacon_info,                \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_beacon_secmat,            \
                                            (actual).params.net_beacon.p_beacon_secmat,              \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.net_beacon.p_rx_metadata,              \
                                            (actual).params.net_beacon.p_rx_metadata,                \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.iv_index,                   \
                                            (actual).params.net_beacon.iv_index,                     \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.flags.iv_update,            \
                                            (actual).params.net_beacon.flags.iv_update,              \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.net_beacon.flags.key_refresh,          \
                                            (actual).params.net_beacon.flags.key_refresh,            \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_MESSAGE_RECEIVED:                                                      \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.message.length,                        \
                                            (actual).params.message.length,                          \
                                            line,                                                    \
                                            msg);                                                    \
                if ((expected).params.message.p_buffer != NULL)                                      \
                {                                                                                    \
                    UNITY_TEST_ASSERT_EQUAL_HEX8_ARRAY((expected).params.message.p_buffer,           \
                                                       (actual).params.message.p_buffer,             \
                                                       (actual).params.message.length,               \
                                                       line,                                         \
                                                       msg);                                         \
                }                                                                                    \
                UNITY_TEST_ASSERT_EQUAL_nrf_mesh_address_t((expected).params.message.src,            \
                                                           (actual).params.message.src,              \
                                                           line,                                     \
                                                           msg);                                     \
                UNITY_TEST_ASSERT_EQUAL_nrf_mesh_address_t((expected).params.message.dst,            \
                                                           (actual).params.message.dst,              \
                                                           line,                                     \
                                                           msg);                                     \
                UNITY_TEST_ASSERT_EQUAL_MEMORY((expected).params.message.secmat.p_app,               \
                                               (actual).params.message.secmat.p_app,                 \
                                               sizeof(nrf_mesh_application_secmat_t),                \
                                               line,                                                 \
                                               msg);                                                 \
                UNITY_TEST_ASSERT_EQUAL_MEMORY((expected).params.message.secmat.p_net,               \
                                               (actual).params.message.secmat.p_net,                 \
                                               sizeof(nrf_mesh_network_secmat_t),                    \
                                               line,                                                 \
                                               msg);                                                 \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.message.ttl,                           \
                                            (actual).params.message.ttl,                             \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_PTR((expected).params.message.p_metadata,                    \
                                            (actual).params.message.p_metadata,                      \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_RX_FAILED:                                                             \
                UNITY_TEST_ASSERT_EQUAL_HEX16((expected).params.rx_failed.src,                       \
                                              (actual).params.rx_failed.src,                         \
                                              line,                                                  \
                                              msg);                                                  \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.rx_failed.ivi,                         \
                                            (actual).params.rx_failed.ivi,                           \
                                            line,                                                    \
                                            msg);                                                    \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.rx_failed.reason,                      \
                                            (actual).params.rx_failed.reason,                        \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_SAR_FAILED:                                                            \
                UNITY_TEST_ASSERT_EQUAL_HEX32((expected).params.sar_failed.token,                    \
                                              (actual).params.sar_failed.token,                      \
                                              line,                                                  \
                                              msg);                                                  \
                UNITY_TEST_ASSERT_EQUAL_INT((expected).params.sar_failed.reason,                     \
                                            (actual).params.sar_failed.reason,                       \
                                            line,                                                    \
                                            msg);                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_DISABLED:                                                              \
                /* No parameters */                                                                  \
                break;                                                                               \
            case NRF_MESH_EVT_HB_SUBSCRIPTION_CHANGE:                                                \
                if ((expected).params.hb_subscription_change.p_old == NULL)                          \
                {                                                                                    \
                    UNITY_TEST_ASSERT_NULL((actual).params.hb_subscription_change.p_old, line, msg); \
                }                                                                                    \
                else                                                                                 \
                {                                                                                    \
                    UNITY_TEST_ASSERT_NOT_NULL((actual).params.hb_subscription_change.p_old,         \
                                               line,                                                 \
                                               msg);                                                 \
                    UNITY_TEST_ASSERT_EQUAL_heartbeat_subscription_state_t(                          \
                        *(expected).params.hb_subscription_change.p_old,                             \
                        *(actual).params.hb_subscription_change.p_old,                               \
                        line,                                                                        \
                        msg);                                                                        \
                }                                                                                    \
                if ((expected).params.hb_subscription_change.p_new == NULL)                          \
                {                                                                                    \
                    UNITY_TEST_ASSERT_NULL((actual).params.hb_subscription_change.p_new, line, msg); \
                }                                                                                    \
                else                                                                                 \
                {                                                                                    \
                    UNITY_TEST_ASSERT_NOT_NULL((actual).params.hb_subscription_change.p_new,         \
                                               line,                                                 \
                                               msg);                                                 \
                    UNITY_TEST_ASSERT_EQUAL_heartbeat_subscription_state_t(                          \
                        *(expected).params.hb_subscription_change.p_new,                             \
                        *(actual).params.hb_subscription_change.p_new,                               \
                        line,                                                                        \
                        msg);                                                                        \
                }                                                                                    \
                break;                                                                               \
            case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:                                                \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_established.role,                                   \
                    (actual).params.friendship_established.role,                                     \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_established.lpn_src,                                \
                    (actual).params.friendship_established.lpn_src,                                  \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_established.friend_src,                             \
                    (actual).params.friendship_established.friend_src,                               \
                    line,                                                                            \
                    msg);                                                                            \
                break;                                                                               \
            case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:                                                 \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_terminated.role,                                    \
                    (actual).params.friendship_terminated.role,                                      \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_terminated.lpn_src,                                 \
                    (actual).params.friendship_terminated.lpn_src,                                   \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_terminated.friend_src,                              \
                    (actual).params.friendship_terminated.friend_src,                                \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_INT(                                                         \
                    (expected).params.friendship_terminated.reason,                                  \
                    (actual).params.friendship_terminated.reason,                                    \
                    line,                                                                            \
                    msg);                                                                            \
                break;                                                                               \
            case NRF_MESH_EVT_FRIEND_REQUEST:                                                        \
                UNITY_TEST_ASSERT_NOT_NULL((actual).params.friend_request.p_friendship, line, msg);  \
                UNITY_TEST_ASSERT_EQUAL_mesh_friendship_t(                                                      \
                    *(expected).params.friend_request.p_friendship,                                   \
                    *(actual).params.friend_request.p_friendship,                                     \
                    line, msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_PTR(                                                         \
                    (expected).params.friend_request.p_net,                                          \
                    (actual).params.friend_request.p_net,                                            \
                    line,                                                                            \
                    msg);                                                                            \
                UNITY_TEST_ASSERT_EQUAL_PTR(                                                         \
                    (expected).params.friend_request.p_metadata,                                     \
                    (actual).params.friend_request.p_metadata,                                       \
                    line,                                                                            \
                    msg);                                                                            \
                break;                                                                               \
            default:                                                                                 \
                {                                                                                    \
                    char error_msg[256];                                                             \
                    sprintf(error_msg, "Unimplemented event type %#x", (expected).type);             \
                    UNITY_TEST_FAIL(line, error_msg);                                                \
                    break;                                                                           \
                }                                                                                    \
        }                                                                                            \
    } while (0)
#define TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, actual, __LINE__, "nrf_mesh_evt_t::")


#define UNITY_TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual, line, message) do { \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).role, (actual).role, line, message); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).net_packet_len, (actual).net_packet_len, line, message); \
        UNITY_TEST_ASSERT_EQUAL_network_packet_metadata_t(*(expected).p_metadata, *(actual).p_metadata, line, message); \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).token, (actual).token, line, message); \
    } while (0)
#define TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_core_tx_alloc_params_t(expected, actual, __LINE__, "")

#define UNITY_TEST_ASSERT_EQUAL_network_tx_packet_buffer_t(expected, actual, line, message) \
    do                                                                  \
    {                                                                   \
        UNITY_TEST_ASSERT_NOT_NULL((actual).user_data.p_metadata, line, "network_tx_packet_buffer_t::user_data::p_metadata"); \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).user_data.token, (actual).user_data.token, line, "network_tx_packet_buffer_t::user_data::token"); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).user_data.payload_len, (actual).user_data.payload_len, line, "network_tx_packet_buffer_t::user_data::payload_len"); \
        UNITY_TEST_ASSERT_EQUAL_HEX32((expected).user_data.bearer_selector, (actual).user_data.bearer_selector, line, "network_tx_packet_buffer_t::user_data::bearer_selector"); \
        UNITY_TEST_ASSERT_EQUAL_INT((expected).user_data.role, (actual).user_data.role, line, "network_tx_packet_buffer_t::user_data::role"); \
    } while (0)
#define TEST_ASSERT_EQUAL_network_tx_packet_buffer_t(expected, actual) UNITY_TEST_ASSERT_EQUAL_network_tx_packet_buffer_t(expected, actual, __LINE__, "")

/** @} */

#endif /* TEST_HELPER_H__ */

