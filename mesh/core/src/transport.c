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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <nrf_error.h>

#include "transport.h"
#include "transport_internal.h"

#include "utils.h"
#include "log.h"
#include "enc.h"
#include "event.h"
#include "nrf_mesh.h"
#include "network.h"
#include "net_state.h"
#include "replay_cache.h"
#include "internal_event.h"
#include "timer_scheduler.h"
#include "bearer_event.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "packet_mesh.h"
#include "mesh_mem.h"

#if MESH_FEATURE_LPN_ENABLED
#include "mesh_lpn.h"
#include "mesh_lpn_internal.h"
#endif

#if MESH_FEATURE_FRIEND_ENABLED
#include "friend_internal.h"
#endif

/* Note on replay protection implementation:
 *
 * Each incoming transport layer packet is checked against the replay list to prevent receiving the
 * same message twice. In addition, the seqzero of an incoming SAR session is checked when the SAR
 * session gets allocated. This is to prevent a SAR session from being received twice over different
 * network packets. It can't be checked later than allocation, as there might be unsegmented packets
 * coming in before all segments are received, which would make us wrongly abandon the SAR session.
 *
 * Packets are only added to the replay protection list once they've been successfully processed in
 * upper transport. This implies that all SAR segments are (implicitly) added only when upper
 * transport has processed the full SAR message. Note that SAR segments for messages we've already
 * processed in upper transport are added retroactively. Messages that aren't processed successfully
 * in upper transport aren't added to the replay protection list.
 *
 * Messages that are only destined for the friend module will not be added to replay protection, but
 * their SAR sessions will be marked as handled, as the friend node is responsible for acking the
 * transaction. The friend module itself can handle duplicates as long as the packets are still in
 * the friend queue.
 */

/*********************
 * Local definitions *
 *********************/

/** Mask for SeqZero value. */
#define TRANSPORT_SAR_SEQZERO_MASK (TRANSPORT_SAR_SEQNUM_DIFF_MAX)
#define TRANSPORT_SAR_SEGMENT_COUNT_MAX 32
#define TRANSPORT_SAR_PDU_LEN(control) ((control) ? PACKET_MESH_TRS_SEG_CONTROL_PDU_MAX_SIZE : PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE)
#define TRANSPORT_SAR_PACKET_MAX_SIZE(control) (TRANSPORT_SAR_SEGMENT_COUNT_MAX * TRANSPORT_SAR_PDU_LEN(control))
#define TRANSPORT_UNSEG_PDU_LEN(control) ((control) ? PACKET_MESH_TRS_UNSEG_CONTROL_PDU_MAX_SIZE : PACKET_MESH_TRS_UNSEG_ACCESS_PDU_MAX_SIZE)
#define TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN_MASK (TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN - 1)

/* The SEQZERO mask must be (power of two - 1) to work as a mask (ie if a bit in the mask is set to
 * 1, all lower bits must also be 1). */
NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(TRANSPORT_SAR_SEQZERO_MASK + 1));
/* Checks whether the maximum access payload is according to 3.7.3 Access payload */
NRF_MESH_STATIC_ASSERT(NRF_MESH_SEG_PAYLOAD_SIZE_MAX == TRANSPORT_SAR_SEGMENT_COUNT_MAX * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE -
                       PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE);
/* Checks whether the maximum unsegmented access payload is according to 3.7.3 Access payload */
NRF_MESH_STATIC_ASSERT(NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX == PACKET_MESH_TRS_UNSEG_ACCESS_PDU_MAX_SIZE - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE);
NRF_MESH_STATIC_ASSERT(TRANSPORT_SAR_SESSIONS_MAX > 1);
NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN));

/*********************
 * Local types *
 *********************/

/**
 * Transport configuration struct.
 *
 * @note All timing values are given in microseconds.
 */
typedef struct
{
    timestamp_t rx_timeout;                /**< Timeout for receiving a SAR message. */
    timestamp_t rx_ack_base_timeout;       /**< RX acknowledgment timer timeout value. */
    timestamp_t rx_ack_per_hop_addition;   /**< RX acknowledgment time delay added per hop in the transmission. */
    timestamp_t tx_retry_base_timeout;     /**< Base timeout for TX retries.*/
    timestamp_t tx_retry_per_hop_addition; /**< TX retry time added per hop in the transmission. */
    uint8_t tx_retries;                    /**< Number of retries before canceling SAR session. */
    uint8_t segack_ttl;                    /**< Default TTL value for segment acknowledgment messages. */
    nrf_mesh_transmic_size_t szmic;        /**< Use 32- or 64-bit MIC for application payload. */
} transport_config_t;

/**
 * Transport SAR session types.
 */
typedef enum
{
    TRS_SAR_SESSION_INACTIVE,   /**< The session isn't active. */
    TRS_SAR_SESSION_TX,         /**< TX session. */
    TRS_SAR_SESSION_RX          /**< RX session. */
} trs_sar_session_t;

/** State of the SAR RX session ack. */
typedef enum
{
    SAR_ACK_STATE_IDLE,   /**< No acking in progress. */
    SAR_ACK_STATE_TIMER_ACTIVE, /**< Timer is waiting to fire */
    SAR_ACK_STATE_PENDING /**< An ack packet should be sent at the first opportunity. */
} sar_ack_state_t;

#if MESH_FEATURE_LPN_ENABLED
typedef enum
{
    SAR_TIMEOUT_STATE_IDLE,
    SAR_TIMEOUT_STATE_ACTIVE,
    SAR_TIMEOUT_STATE_WAIT_POLL_COMPLETE
} sar_timeout_state_t;
#endif

/**
 * Transport SAR PDU structure.
 *
 * @note All multi-octet values are assumed *little* endian.
 */
typedef struct
{
    transport_packet_metadata_t metadata;
    /** Timer used to resend or abort the SAR session. */
    timer_event_t timer_event;
#if MESH_FEATURE_LPN_ENABLED
    sar_timeout_state_t timeout_state;
#endif
    /** Transport SAR session header. */
    struct
    {
        trs_sar_session_t session_type; /**< SAR session type. */
        uint16_t length;                /**< Total length of the session data (payload + mic). */
        uint32_t block_ack;             /**< Bit-field of the acknowledged segments. */
        union
        {
            /** Fields that are only valid for TX-sessions. */
            struct
            {
                uint8_t  start_index;           /**< Segment index to start transmitting from. Used to recover from out-of-memory errors. */
                uint8_t  retries;               /**< Number of retries left. */
                bool payload_encrypted;         /**< Flag indicating whether the payload has been encrypted. */
                bool seqzero_is_set;            /**< Flag indicating whether the seqzero has been set. */
                nrf_mesh_tx_token_t token;      /**< TX Token set by the user. */
                /** Source address of the device that acked the packet. Must be the same for every
                 * ack, according to @tagMeshSp section 3.5.3.3. */
                uint16_t ack_src;
            } tx;
            /** Fields that are only valid for RX-sessions */
            struct
            {
                /** Acknowledgment timer */
                timer_event_t ack_timer;
                sar_ack_state_t ack_state;
            } rx;
        } params;
    } session;
    /**
    * Re-segmented SAR payload with 4 byte MIC at the end.
    */
    uint8_t * payload;
} trs_sar_ctx_t;

/** Canceled SAR RX session. */
typedef struct
{
    uint16_t src;               /**< Source address of the SAR RX session. */
    uint64_t seqauth;           /**< SeqAuth of the SAR RX session. */
} canceled_sar_rx_session_t;

/** A consumer of control packets. */
typedef struct
{
    const transport_control_packet_handler_t * p_handlers; /**< List of opcodes and their handler functions. */
    uint32_t handler_count; /**< Number of handlers. */
} control_packet_consumer_t;
/********************
 * Static variables *
 ********************/

static transport_config_t m_trs_config;
static trs_sar_ctx_t m_trs_sar_sessions[TRANSPORT_SAR_SESSIONS_MAX];
#if MESH_FEATURE_LPN_ENABLED
static uint8_t m_trs_sar_lpn_buffer[NRF_MESH_UPPER_TRANSPORT_PDU_SIZE_MAX];
#endif
static uint32_t m_canceled_sar_rx_sessions_cache_head;
static canceled_sar_rx_session_t m_canceled_sar_rx_sessions_cache[TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN];

/** Flag used to trigger SAR processing. */
static bearer_event_flag_t m_sar_process_flag;

static control_packet_consumer_t m_control_packet_consumers[TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX];
static uint32_t m_control_packet_consumer_count;
/********************
 * Static functions *
 ********************/

static void ack_timeout(timestamp_t timestamp, void * p_context);
static void retry_timeout(timestamp_t timestamp, void * p_context);
static void abort_timeout(timestamp_t timestamp, void * p_context);

#if MESH_FEATURE_LPN_ENABLED
static bool sar_payload_is_heap_allocated(trs_sar_ctx_t * p_sar_ctx)
{
    return ((intptr_t) p_sar_ctx->payload != (intptr_t) &m_trs_sar_lpn_buffer[0]);
}
#endif

static inline bool is_in_lpn_role(void)
{
#if MESH_FEATURE_LPN_ENABLED
    return mesh_lpn_is_in_friendship();
#else
    return false;
#endif
}

static inline uint32_t block_ack_full(const transport_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_metadata->segmented);
    return ((1u << (p_metadata->segmentation.last_segment + 1)) - 1);
}
static void upper_transport_packet_in(const uint8_t * p_upper_trs_packet,
                                      uint32_t upper_trs_packet_len,
                                      transport_packet_metadata_t * p_metadata,
                                      const nrf_mesh_rx_metadata_t * p_rx_metadata);

static void m_send_replay_cache_full_event(uint16_t src, uint8_t ivi, nrf_mesh_rx_failed_reason_t reason)
{
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_RX_FAILED,
            .params.rx_failed.src = src,
            .params.rx_failed.ivi = ivi,
            .params.rx_failed.reason = reason
        };
    event_handle(&evt);
}

static uint32_t replay_list_add(const transport_packet_metadata_t * p_metadata)
{
    uint32_t status;

    if (p_metadata->segmented)
    {
        status = replay_cache_seqauth_add(p_metadata->net.src,
                                          p_metadata->net.internal.sequence_number,
                                          p_metadata->net.internal.iv_index,
                                          p_metadata->segmentation.seq_zero);
    }
    else
    {
        status = replay_cache_add(p_metadata->net.src,
                                  p_metadata->net.internal.sequence_number,
                                  p_metadata->net.internal.iv_index);
    }

    if (status != NRF_SUCCESS)
    {
        m_send_replay_cache_full_event(p_metadata->net.src,
                                       p_metadata->net.internal.iv_index & NETWORK_IVI_MASK,
                                       NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
    }
    return status;
}

static void m_send_sar_cancel_event(nrf_mesh_tx_token_t token, nrf_mesh_sar_session_cancel_reason_t reason)
{
    nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_SAR_FAILED,
            .params.sar_failed.token = token,
            .params.sar_failed.reason = reason
        };
    event_handle(&evt);
}

static inline uint32_t rx_ack_timer_delay_get(uint8_t ttl)
{
    return m_trs_config.rx_ack_base_timeout + m_trs_config.rx_ack_per_hop_addition * ttl;
}

static inline uint32_t tx_retry_timer_delay_get(uint8_t ttl)
{
    return m_trs_config.tx_retry_base_timeout + m_trs_config.tx_retry_per_hop_addition * ttl;
}

static bool sar_rx_session_is_canceled(uint16_t src, uint64_t seqauth)
{
    for (uint32_t i = 0; i < TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN; ++i)
    {
        if (src == m_canceled_sar_rx_sessions_cache[i].src
            && seqauth == m_canceled_sar_rx_sessions_cache[i].seqauth)
        {
            return true;
        }
    }
    return false;
}

static void sar_rx_session_cancel(trs_sar_ctx_t * p_sar_ctx)
{
    uint64_t seqauth = transport_sar_seqauth_get(p_sar_ctx->metadata.net.internal.iv_index,
                                                 p_sar_ctx->metadata.net.internal.sequence_number,
                                                 p_sar_ctx->metadata.segmentation.seq_zero);

    canceled_sar_rx_session_t * p_canceled_session =
        &m_canceled_sar_rx_sessions_cache[m_canceled_sar_rx_sessions_cache_head++ & TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN_MASK];

    p_canceled_session->src = p_sar_ctx->metadata.net.src;
    p_canceled_session->seqauth = seqauth;
}

/**
 * Allocate the given SAR context with the given parameters.
 *
 * @param[in] p_metadata Metadata to use in the context.
 * @param[in] session_type Type of session.
 * @param[in] length Length of SAR data, including MIC.
 *
 * @returns pointer to allocated context if successful. NULL otherwise.
 */
static trs_sar_ctx_t * sar_ctx_alloc(const transport_packet_metadata_t * p_metadata,
                                     trs_sar_session_t session_type,
                                     uint32_t length)
{
    NRF_MESH_ASSERT_DEBUG(session_type == TRS_SAR_SESSION_RX ||
                          session_type == TRS_SAR_SESSION_TX);
    NRF_MESH_ASSERT_DEBUG(length <= NRF_MESH_UPPER_TRANSPORT_PDU_SIZE_MAX);

    trs_sar_ctx_t * p_sar_ctx = NULL;
    uint32_t i = 0;
#if MESH_FEATURE_LPN_ENABLED
    /* Reserve one session for RX if LPN is enabled. */
    if (session_type == TRS_SAR_SESSION_TX)
    {
        i = 1;
    }
#endif
    for (; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_INACTIVE)
        {
            p_sar_ctx = &m_trs_sar_sessions[i];
#if MESH_FEATURE_LPN_ENABLED
            if (i == 0)
            {
                p_sar_ctx->payload = m_trs_sar_lpn_buffer;
            }
            else
#endif
            {
                p_sar_ctx->payload = mesh_mem_alloc(length);
            }
            break;
        }
    }

    if (p_sar_ctx == NULL || p_sar_ctx->payload == NULL)
    {
        return NULL;
    }

    memcpy(&p_sar_ctx->metadata, p_metadata, sizeof(transport_packet_metadata_t));
    p_sar_ctx->session.block_ack = 0;
    p_sar_ctx->session.length = length;

    if (session_type == TRS_SAR_SESSION_TX)
    {
        p_sar_ctx->session.params.tx.retries = m_trs_config.tx_retries;
        p_sar_ctx->session.params.tx.payload_encrypted = false;
        p_sar_ctx->session.params.tx.seqzero_is_set = false;
        p_sar_ctx->session.params.tx.start_index = 0;
        /* Set the TX token to indicate a SAR packet, use the SAR-TX token to keep the user-token.
         * This way we'll know that we got a TX complete on a SAR packet, so we can forward the TX
         * complete when the entire SAR packet is done. */
        p_sar_ctx->session.params.tx.token = p_metadata->token;
        p_sar_ctx->session.params.tx.ack_src = NRF_MESH_ADDR_UNASSIGNED;
        p_sar_ctx->metadata.token = NRF_MESH_SAR_TOKEN;
        p_sar_ctx->timer_event.cb = retry_timeout;
        p_sar_ctx->timer_event.p_context = p_sar_ctx;
    }
    else
    {
        p_sar_ctx->metadata.token = NRF_MESH_SAR_TOKEN;
        p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
        p_sar_ctx->session.params.rx.ack_timer.cb = ack_timeout;
        p_sar_ctx->session.params.rx.ack_timer.p_context = p_sar_ctx;
        p_sar_ctx->timer_event.cb = abort_timeout;
        p_sar_ctx->timer_event.p_context = p_sar_ctx;
    }
    /* The IV index should stay the same for the duration of the session. */
    net_state_iv_index_lock(true);
    p_sar_ctx->session.session_type = session_type;
    return p_sar_ctx;
}

static void sar_ctx_free(trs_sar_ctx_t * p_sar_ctx)
{
#if MESH_FEATURE_LPN_ENABLED
    if (sar_payload_is_heap_allocated(p_sar_ctx))
#endif
    {
        mesh_mem_free(p_sar_ctx->payload);
    }

    /* Abort any ongoing timers. Timers may or may not be running depending on whether we're
     * entering, exiting or in a friendship. */
    timer_sch_abort(&p_sar_ctx->timer_event);

    if (p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX)
    {
        timer_sch_abort(&p_sar_ctx->session.params.rx.ack_timer);
    }

    memset(p_sar_ctx, 0, sizeof(trs_sar_ctx_t));
    p_sar_ctx->session.session_type = TRS_SAR_SESSION_INACTIVE;

    net_state_iv_index_lock(false);
}

static void sar_ctx_cancel(trs_sar_ctx_t * p_sar_ctx, nrf_mesh_sar_session_cancel_reason_t reason)
{
    NRF_MESH_ASSERT(p_sar_ctx != NULL);

    uint32_t seq = transport_sar_first_seq_num_get(p_sar_ctx->metadata.net.internal.sequence_number,
                                                   p_sar_ctx->metadata.segmentation.seq_zero);
    (void) seq;
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_SAR_CANCELLED, reason, sizeof(uint32_t), &seq);

    /* For RX sessions, the friend needs to know what happened. */
#if MESH_FEATURE_FRIEND_ENABLED
    if (p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX &&
        p_sar_ctx->metadata.receivers & TRANSPORT_PACKET_RECEIVER_FRIEND)
    {
        friend_sar_complete(p_sar_ctx->metadata.net.src,
                            p_sar_ctx->metadata.segmentation.seq_zero,
                            false);
    }
#endif

    if (p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX)
    {
        sar_rx_session_cancel(p_sar_ctx);
    }

    m_send_sar_cancel_event(p_sar_ctx->session.params.tx.token, reason);
    sar_ctx_free(p_sar_ctx);

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Dropped SAR session %u, reason %u\n",
          p_sar_ctx->session.session_type, reason);
}

static void sar_ctx_tx_complete(trs_sar_ctx_t * p_sar_ctx)
{
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_TX);
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_TX_COMPLETE;
    evt.params.tx_complete.token = p_sar_ctx->session.params.tx.token;
    evt.params.tx_complete.timestamp = timer_now();
    event_handle(&evt);
    sar_ctx_free(p_sar_ctx);
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_SAR_SUCCESS, 0, 0, NULL);
}

static void ack_timer_reset(trs_sar_ctx_t * p_sar_ctx)
{
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX);
    uint32_t delay = rx_ack_timer_delay_get(m_trs_config.segack_ttl);
    timer_sch_reschedule(&p_sar_ctx->session.params.rx.ack_timer, timer_now() + delay);
    p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_TIMER_ACTIVE;
}

static void rx_incomplete_timer_reset(trs_sar_ctx_t * p_sar_ctx)
{
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX);
    uint32_t delay = m_trs_config.rx_timeout;
    timer_sch_reschedule(&p_sar_ctx->timer_event, timer_now() + delay);
}

static void tx_retry_timer_reset(trs_sar_ctx_t * p_sar_ctx)
{
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_TX);

    uint32_t next_timeout = timer_now();

    /* For non-unicast addresses, we're not going to get any acknowledgements, so there's no
     * point in scaling the retry interval according to TTL. */
    next_timeout += ((p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST) ?
                     tx_retry_timer_delay_get(p_sar_ctx->metadata.net.ttl) :
                     tx_retry_timer_delay_get(0));

#if MESH_FEATURE_LPN_ENABLED
    p_sar_ctx->timeout_state = SAR_TIMEOUT_STATE_ACTIVE;
#endif
    timer_sch_reschedule(&p_sar_ctx->timer_event, next_timeout);
}

static void trs_packet_header_build(const transport_packet_metadata_t * p_metadata, packet_mesh_trs_packet_t * p_packet)
{
    if (p_metadata->net.control_packet)
    {
        packet_mesh_trs_control_opcode_set(p_packet, p_metadata->type.control.opcode);
    }
    else
    {
        packet_mesh_trs_access_akf_set(p_packet, p_metadata->type.access.using_app_key);
        packet_mesh_trs_access_aid_set(p_packet, p_metadata->type.access.app_key_id);
    }

    packet_mesh_trs_common_seg_set(p_packet, p_metadata->segmented);

    if (p_metadata->segmented)
    {
        packet_mesh_trs_seg_szmic_set(p_packet, p_metadata->mic_size == PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE);
        packet_mesh_trs_seg_seqzero_set(p_packet, p_metadata->segmentation.seq_zero);
        packet_mesh_trs_seg_sego_set(p_packet, p_metadata->segmentation.segment_offset);
        packet_mesh_trs_seg_segn_set(p_packet, p_metadata->segmentation.last_segment);
    }
}

static uint32_t upper_trs_packet_alloc(transport_packet_metadata_t * p_metadata,
                                        network_tx_packet_buffer_t * p_net_buf,
                                        uint32_t transport_data_len,
                                        uint8_t ** pp_upper_trs_payload)
{
    uint32_t header_len = (p_metadata->segmented ?
                           PACKET_MESH_TRS_SEG_PDU_OFFSET :
                           PACKET_MESH_TRS_UNSEG_PDU_OFFSET);
    p_net_buf->user_data.payload_len = header_len + transport_data_len;
    p_net_buf->user_data.p_metadata = &p_metadata->net;
    p_net_buf->user_data.token = p_metadata->token;
    p_net_buf->user_data.bearer_selector = p_metadata->tx_bearer_selector;
    p_net_buf->user_data.role = CORE_TX_ROLE_ORIGINATOR;

    uint32_t status = network_packet_alloc(p_net_buf);
    if (status == NRF_SUCCESS)
    {
        *pp_upper_trs_payload = &p_net_buf->p_payload[header_len];
    }
    return status;
}

/**
 * Ack the SAR session.
 *
 * @note Enforces specification rule saying only to ack if the destination address is a unicast
 * address (@tagMeshSp section 3.5.3.4).
 *
 * @param[in] p_metadata Metadata of the SAR session to ack.
 * @param[in] block_ack Ack bitfield value.
 *
 * @retval NRF_SUCCESS The ack was successful, or the session was not directed to a unicast address.
 * @retval NRF_ERROR_NO_MEM
 */
static uint32_t sar_ack_send(const transport_packet_metadata_t * p_metadata, uint32_t block_ack)
{
    /* We only ack on unicast packets */
    uint32_t status = NRF_SUCCESS;
    if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Sending ACK...\n");

        packet_mesh_trs_control_packet_t packet_buffer;
        memset(&packet_buffer, 0, sizeof(packet_buffer));
        packet_mesh_trs_control_segack_seqzero_set(&packet_buffer, p_metadata->segmentation.seq_zero);
        packet_mesh_trs_control_segack_block_ack_set(&packet_buffer, block_ack);
        packet_mesh_trs_control_segack_obo_set(&packet_buffer, (p_metadata->receivers == TRANSPORT_PACKET_RECEIVER_FRIEND));

        transport_control_packet_t control_packet;
        control_packet.data_len           = PACKET_MESH_TRS_CONTROL_SEGACK_SIZE;
        control_packet.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
        control_packet.dst.value          = p_metadata->net.src;
        control_packet.dst.p_virtual_uuid = NULL;
        control_packet.opcode             = TRANSPORT_CONTROL_OPCODE_SEGACK;
        control_packet.p_data             = &packet_buffer;
        control_packet.p_net_secmat       = p_metadata->net.p_security_material;
        control_packet.reliable           = false;
        control_packet.bearer_selector    = p_metadata->tx_bearer_selector;

        if (p_metadata->receivers == TRANSPORT_PACKET_RECEIVER_FRIEND)
        {
            uint16_t local_address;
            uint16_t local_address_count;

            nrf_mesh_unicast_address_get(&local_address, &local_address_count);

            control_packet.src = local_address;
        }
        else
        {
            control_packet.src = p_metadata->net.dst.value;
        }

        if (p_metadata->net.ttl == 0)
        {
            /* Respond with TTL=0 to TTL=0 messages. (@tagMeshSp section 3.5.2.3) */
            control_packet.ttl = 0;
        }
        else
        {
            control_packet.ttl = m_trs_config.segack_ttl;
        }

        status = transport_control_tx(&control_packet, NRF_MESH_SAR_TOKEN);

        if (status == NRF_SUCCESS)
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_ACK_QUEUED, 0, PACKET_MESH_TRS_CONTROL_SEGACK_SIZE, &packet_buffer);
        }
    }
    return status;
}

static void sar_tx_session_end(trs_sar_ctx_t * p_sar_ctx)
{
    /* We're out of retries. Should give up. */
    if (p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_RETRY_OVER);
    }
    else
    {
        /* For non-unicast addresses, timing out is considered a successful way to end, as there
         * are no acknowledgments. */
        sar_ctx_tx_complete(p_sar_ctx);
    }
}

static void sar_tx_session_continue(trs_sar_ctx_t * p_sar_ctx)
{
    if (p_sar_ctx->session.params.tx.retries > 0)
    {
        p_sar_ctx->session.params.tx.retries--;
        p_sar_ctx->session.params.tx.start_index = 0;
        /* Set bearer flag to trigger sending of remaining segments. */
        bearer_event_flag_set(m_sar_process_flag);
    }
    else
    {
        sar_tx_session_end(p_sar_ctx);
    }
}

static trs_sar_ctx_t * sar_active_tx_ctx_get(const transport_packet_metadata_t * p_metadata, uint16_t seq_zero)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_TX &&
            m_trs_sar_sessions[i].metadata.net.src == p_metadata->net.dst.value &&
            m_trs_sar_sessions[i].metadata.segmentation.seq_zero == seq_zero &&
            (m_trs_sar_sessions[i].session.params.tx.ack_src == NRF_MESH_ADDR_UNASSIGNED ||
             m_trs_sar_sessions[i].session.params.tx.ack_src == p_metadata->net.src))
        {
            return &m_trs_sar_sessions[i];
        }
    }

    return NULL;
}

static trs_sar_ctx_t * sar_active_rx_ctx_get(const transport_packet_metadata_t * p_metadata)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_RX &&
            m_trs_sar_sessions[i].metadata.net.src == p_metadata->net.src)
        {
            return &m_trs_sar_sessions[i];
        }
    }
    return NULL;
}

static void sar_rx_ctx_cancel_all(nrf_mesh_sar_session_cancel_reason_t reason)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_RX)
        {
            sar_ctx_cancel(&m_trs_sar_sessions[i], reason);
        }
    }
}

#if MESH_FEATURE_LPN_ENABLED
static void sar_tx_pending_retries_send(void)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_TX &&
            m_trs_sar_sessions[i].timeout_state == SAR_TIMEOUT_STATE_WAIT_POLL_COMPLETE)
        {
            sar_tx_session_continue(&m_trs_sar_sessions[i]);
        }
    }
}

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE:
            sar_rx_ctx_cancel_all(NRF_MESH_SAR_CANCEL_REASON_LPN_RX_NOT_COMPLETE);
            sar_tx_pending_retries_send();
            break;

        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            if (p_evt->params.friendship_terminated.role == NRF_MESH_FRIENDSHIP_ROLE_LPN)
            {
                sar_rx_ctx_cancel_all(NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_TERMINATED);
                /* TODO: If the DST is unicast and we're not scanning ~100%, we might want to cancel any
                 * ongoing TX sessions (MBTLE-2825).  */
                sar_tx_pending_retries_send();
            }
            break;

        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            if (p_evt->params.friendship_established.role == NRF_MESH_FRIENDSHIP_ROLE_LPN)
            {
                sar_rx_ctx_cancel_all(NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_ESTABLISHED);
            }
            break;

        default:
            break;
    }
}
#endif  /* MESH_FEATURE_LPN_ENABLED */

/**
 * Get a SAR ctx for the given metadata.
 *
 * Checks that we're allowed to receive a SAR session with the given metadata, and searches for the
 * active session. If no active session exists for this src+seqzero combination, we'll create one.
 *
 * @param[in] p_metadata Metadata of a segment.
 *
 * @returns A pointer to a SAR context or NULL if the packet can't be received.
 */
static trs_sar_ctx_t * sar_rx_ctx_get(const transport_packet_metadata_t * p_metadata)
{
    uint64_t new_sar_seqauth = transport_sar_seqauth_get(p_metadata->net.internal.iv_index,
                                                         p_metadata->net.internal.sequence_number,
                                                         p_metadata->segmentation.seq_zero);
    if (sar_rx_session_is_canceled(p_metadata->net.src, new_sar_seqauth))
    {
        return NULL;
    }

    /* Find ongoing session by src. */
    trs_sar_ctx_t * p_sar_ctx = sar_active_rx_ctx_get(p_metadata);
    if (NULL != p_sar_ctx)
    {
        uint64_t current_sar_seqauth = transport_sar_seqauth_get(p_sar_ctx->metadata.net.internal.iv_index,
                                                                 p_sar_ctx->metadata.net.internal.sequence_number,
                                                                 p_sar_ctx->metadata.segmentation.seq_zero);
        if (new_sar_seqauth > current_sar_seqauth)
        {
            /* As segments on older SeqAuths are discarded, there's no point in keeping the old session
             * around. Its segments will only get rejected over and over until it times out. */
            sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
            p_sar_ctx = NULL;
        }
        else if (new_sar_seqauth < current_sar_seqauth)
        {
            /* From @tagMeshSp section 3.5.3.4: Segments with a lower SeqAuth
             * value than the most recent SeqAuth should be ignored. Note that this rule takes precendence
             * over all the others - we should never act on old messages. */
            if (is_in_lpn_role())
            {
                /* The Friend node should only send us segments for complete transactions. However, if
                 * the Friend misbehaves, all we can do is ignore the segment. */
                __LOG(LOG_SRC_TRANSPORT,
                      LOG_LEVEL_WARN,
                      "Invalid segment from Friend (seqzero: %d)\n",
                      p_metadata->segmentation.seq_zero);
            }
            return NULL;
        }
        else
        {
            /* New segment for the active sequence authentication value. */
            return p_sar_ctx;
        }
    }

    /* The replay protection cache will not have the SeqAuth until
     * the segmented message is received and decrypted therefore the check
     * will be passed for the active SAR sessions and friend's SAR sessions. */
    if (replay_cache_has_seqauth(p_metadata->net.src,
                                 p_metadata->net.internal.sequence_number,
                                 p_metadata->net.internal.iv_index,
                                 p_metadata->segmentation.seq_zero))
    {
        if (replay_cache_is_seqauth_last(p_metadata->net.src,
                                         p_metadata->net.internal.sequence_number,
                                         p_metadata->net.internal.iv_index,
                                         p_metadata->segmentation.seq_zero))
        {
            /* This is a segment of the successfully received segmented message.
             * The sender likely missed our ack, send ack again.
             * (according to @tagMeshSp section 3.5.3.4). */
            (void) sar_ack_send(p_metadata, block_ack_full(p_metadata));

            /* No need to worry about the receiver, because this might happen only
             * if we added the seqauth to the replay cache earlier. But
             * the message is added to the replay cache only if this node is the receiver. */
            (void) replay_list_add(p_metadata);
        }
        else
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                                  PACKET_DROPPED_REPLAY_CACHE,
                                  sizeof(uint32_t),
                                  p_metadata->net.internal.sequence_number);
        }

        return NULL;
    }

#if MESH_FEATURE_FRIEND_ENABLED
    if (friend_sar_exists(p_metadata->net.src, new_sar_seqauth))
    {
        /* This is a segment of the successfully received segmented message.
         * The sender likely missed our ack, send ack again.
         * (according to @tagMeshSp section 3.5.3.4). */
        (void) sar_ack_send(p_metadata, block_ack_full(p_metadata));
        return NULL;
    }
#endif

    /* Create new session */

    if (is_in_lpn_role())
    {
        /* The Friend node should prevent the scenario where we're starting a new session whilst
        * still receiving another one, but we'll cancel anything ongoing to be sure. */
        sar_rx_ctx_cancel_all(NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
    }

    uint32_t total_length = ((p_metadata->segmentation.last_segment + 1) *
                                TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet));
    if (total_length > TRANSPORT_SAR_PACKET_MAX_SIZE(p_metadata->net.control_packet))
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Invalid length: %u\n", total_length);
        return NULL;
    }

    /* Try allocating the new session */
    p_sar_ctx = sar_ctx_alloc(p_metadata, TRS_SAR_SESSION_RX, total_length);

    if (p_sar_ctx == NULL)
    {
        /* We must assure that we always have room to receive a SAR message when in a
        * friendship as an LPN, so if we got here as an LPN, something is wrong in the implementation. */
        NRF_MESH_ASSERT_DEBUG(!is_in_lpn_role());

        /* Send block ack=0 to indicate failure to receive. Transport is out of resources. */
        (void) sar_ack_send(p_metadata, 0);
        m_send_sar_cancel_event(NRF_MESH_INITIAL_TOKEN, NRF_MESH_SAR_CANCEL_REASON_NO_MEM);

        /* We do NOT add this message to the replay cache, as we hope to handle its SAR session
        * at a later point. Adding it would make us reject the SAR session later, as we're
        * checking for seqzero when we allocate it. */
    }

    return p_sar_ctx;
}

static void trs_seg_packet_in(const packet_mesh_trs_packet_t * p_packet,
                              uint32_t packet_len,
                              const transport_packet_metadata_t * p_metadata,
                              const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    trs_sar_ctx_t * p_sar_ctx = sar_rx_ctx_get(p_metadata);

    if (p_sar_ctx == NULL)
    {
        return;
    }
    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Got segment %u\n", p_metadata->segmentation.segment_offset);

    /* In a friendship, the Friend shall put segments in its Friend Queue when the message has been
     * fully received/assembled (ref. @tagMeshSp section 3.5.5). If we still have an active RX
     * context on NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE, we regard the session as failed. Thus no
     * need to start the incomplete timer. */
    if (!is_in_lpn_role())
    {
        /* Reset timers, even if we already received this segment. */
        rx_incomplete_timer_reset(p_sar_ctx);

        if (p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST &&
            p_sar_ctx->session.params.rx.ack_state == SAR_ACK_STATE_IDLE)
        {
            ack_timer_reset(p_sar_ctx);
        }
    }

    if (p_sar_ctx->session.block_ack & (1u << p_metadata->segmentation.segment_offset))
    {
        /* Segment already received. */
        return;
    }

    p_sar_ctx->session.block_ack |= (1u << p_metadata->segmentation.segment_offset);

    uint32_t segment_len    = packet_len - PACKET_MESH_TRS_SEG_PDU_OFFSET;
    uint32_t segment_offset = p_metadata->segmentation.segment_offset *
                              TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet);
    if (p_metadata->segmentation.segment_offset == p_sar_ctx->metadata.segmentation.last_segment)
    {
        /* Last segment might not be the full length of a normal segment, update total length */
        p_sar_ctx->session.length = segment_offset + segment_len;
    }
    else if (segment_len != TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet))
    {
        /* Got a non-conformant segment length, discard the packet. */
        sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_INVALID_FORMAT);
        return;
    }

    /* Adopt the network metadata of the segment that was sent last to maintain the correct sequence
     * number order in upper transport. This also ensures that once the packet is added to the
     * replay list, the entry covers them all. */
    if (p_sar_ctx->metadata.net.internal.sequence_number < p_metadata->net.internal.sequence_number)
    {
        p_sar_ctx->metadata.net = p_metadata->net;
    }

    memcpy(&p_sar_ctx->payload[segment_offset], packet_mesh_trs_seg_payload_get(p_packet), segment_len);

#if MESH_FEATURE_FRIEND_ENABLED
    // Now that all sanitizing checks have passed, we can send the packet to the friend module
    if (p_metadata->receivers & TRANSPORT_PACKET_RECEIVER_FRIEND)
    {
        friend_packet_in(p_packet, packet_len, p_metadata, CORE_TX_ROLE_RELAY);
    }
#endif

    /* All packets have arrived */
    if (p_sar_ctx->session.block_ack == block_ack_full(&p_sar_ctx->metadata))
    {
        if (!is_in_lpn_role())
        {
            /* Final ack. If it fails, we'll recover when the sender resends a segment. We'll find
             * the cached session and respond. */
            (void) sar_ack_send(&p_sar_ctx->metadata, p_sar_ctx->session.block_ack);
        }

#if MESH_FEATURE_FRIEND_ENABLED
        // Give the packet to the friend first, in case the app decides to send something as a response
        if (p_metadata->receivers & TRANSPORT_PACKET_RECEIVER_FRIEND)
        {
            friend_sar_complete(p_metadata->net.src, p_metadata->segmentation.seq_zero, true);
        }
#endif

        upper_transport_packet_in(p_sar_ctx->payload,
                                  p_sar_ctx->session.length,
                                  &p_sar_ctx->metadata,
                                  p_rx_metadata);

        sar_ctx_free(p_sar_ctx);
    }
}

static void trs_unseg_packet_in(const packet_mesh_trs_packet_t * p_packet,
                                uint32_t packet_len,
                                transport_packet_metadata_t * p_metadata,
                                const nrf_mesh_rx_metadata_t * p_rx_metadata)
{

#if MESH_FEATURE_FRIEND_ENABLED
    if (p_metadata->receivers & TRANSPORT_PACKET_RECEIVER_FRIEND)
    {
        friend_packet_in(p_packet, packet_len, p_metadata, CORE_TX_ROLE_RELAY);
    }
#endif

    upper_transport_packet_in(packet_mesh_trs_unseg_payload_get(p_packet),
                              packet_len - PACKET_MESH_TRS_UNSEG_PDU_OFFSET,
                              p_metadata,
                              p_rx_metadata);
}

static bool test_transport_decrypt(const nrf_mesh_application_secmat_t * p_app_security_material, ccm_soft_data_t * p_ccm_data)
{
    bool mic_passed = false;
    if (p_app_security_material != NULL)
    {
        p_ccm_data->p_key = p_app_security_material->key;
        enc_aes_ccm_decrypt(p_ccm_data, &mic_passed);
        if (mic_passed)
        {
            __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Message decrypted\n");
        }
    }
    return mic_passed;
}

static void upper_trs_packet_encrypt(const uint8_t * p_unencrypted_upper_trs_packet,
                                     uint8_t * p_encrypted_upper_trs_packet,
                                     uint32_t upper_trs_payload_len,
                                     const transport_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_unencrypted_upper_trs_packet != NULL);
    NRF_MESH_ASSERT(p_encrypted_upper_trs_packet != NULL);
    NRF_MESH_ASSERT(p_metadata != NULL);

    ccm_soft_data_t ccm_data;
    ccm_data.p_key = p_metadata->p_security_material->key;

    uint8_t app_nonce_buf[CCM_NONCE_LENGTH];
    enc_nonce_generate(&p_metadata->net,
                       (p_metadata->type.access.using_app_key ? ENC_NONCE_APP : ENC_NONCE_DEV),
                       (p_metadata->mic_size == PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE),
                       app_nonce_buf);

    ccm_data.p_nonce = app_nonce_buf;
    ccm_data.p_m     = p_unencrypted_upper_trs_packet;
    ccm_data.m_len   = upper_trs_payload_len;
    ccm_data.p_out   = p_encrypted_upper_trs_packet;
    ccm_data.p_mic   = (uint8_t *) ccm_data.p_out + ccm_data.m_len;
    ccm_data.mic_len = p_metadata->mic_size;

    if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        NRF_MESH_ASSERT(p_metadata->net.dst.p_virtual_uuid != NULL);
        ccm_data.p_a   = p_metadata->net.dst.p_virtual_uuid;
        ccm_data.a_len = NRF_MESH_KEY_SIZE;
    }
    else
    {
        ccm_data.a_len = 0;
    }

    enc_aes_ccm_encrypt(&ccm_data);
}

static uint8_t mic_size_get(nrf_mesh_transmic_size_t encoded_mic_opt_val)
{
    return (encoded_mic_opt_val == NRF_MESH_TRANSMIC_SIZE_SMALL) ?
           PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE:
           PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE;
}

static void transport_metadata_from_tx_params(transport_packet_metadata_t * p_metadata,
                                              const nrf_mesh_tx_params_t * p_tx_params)
{
    p_metadata->type.access.app_key_id = p_tx_params->security_material.p_app->aid;
    p_metadata->type.access.using_app_key = (!p_tx_params->security_material.p_app->is_device_key);
    p_metadata->p_security_material = p_tx_params->security_material.p_app;

    /* Messages should be segmented if:
     * - They won't fit in an unsegmented message.
     * - The user explicitly requested it.
     * - The user forced a large MIC size.
     */
    p_metadata->segmented = ((p_tx_params->data_len > NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX) ||
                             p_tx_params->force_segmented ||
                             p_tx_params->transmic_size == NRF_MESH_TRANSMIC_SIZE_LARGE);

    if (p_metadata->segmented)
    {
        p_metadata->mic_size =
            mic_size_get((p_tx_params->transmic_size == NRF_MESH_TRANSMIC_SIZE_DEFAULT)
                             ? m_trs_config.szmic
                             : p_tx_params->transmic_size);

        uint32_t num_segments = ((p_tx_params->data_len + p_metadata->mic_size +
                                  PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - 1) /
                                 PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE);
        p_metadata->segmentation.last_segment = num_segments - 1;
        p_metadata->segmentation.segment_offset = 0;
    }
    else
    {
        p_metadata->mic_size = PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE;
    }

    p_metadata->net.control_packet = false;
    p_metadata->net.dst = p_tx_params->dst;
    p_metadata->net.p_security_material = p_tx_params->security_material.p_net;
    p_metadata->net.src = p_tx_params->src;
    p_metadata->net.ttl = p_tx_params->ttl;
    p_metadata->token = p_tx_params->tx_token;
    p_metadata->tx_bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;

    if (nrf_mesh_is_address_rx(&p_tx_params->dst))
    {
        p_metadata->tx_bearer_selector = (p_tx_params->dst.type != NRF_MESH_ADDRESS_TYPE_UNICAST) ?
                CORE_TX_BEARER_TYPE_ALLOW_ALL : CORE_TX_BEARER_TYPE_LOCAL;
    }

#if MESH_FEATURE_FRIEND_ENABLED
    if (friend_needs_packet(p_metadata))
    {
        if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            p_metadata->tx_bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
        }
        else
        {
            p_metadata->tx_bearer_selector |= CORE_TX_BEARER_TYPE_LOCAL;
        }
    }
#endif
}

static void transport_metadata_from_control_tx_params(
    transport_packet_metadata_t * p_metadata,
    const transport_control_packet_t * p_tx_params,
    nrf_mesh_tx_token_t tx_token)
{
    p_metadata->type.control.opcode = p_tx_params->opcode;
    p_metadata->p_security_material = NULL;
    p_metadata->segmented = ((p_tx_params->data_len > PACKET_MESH_TRS_UNSEG_CONTROL_PDU_MAX_SIZE) ||
                             p_tx_params->reliable);
    p_metadata->mic_size = PACKET_MESH_TRS_TRANSMIC_CONTROL_SIZE;

    if (p_metadata->segmented)
    {
        uint32_t num_segments =
            ((p_tx_params->data_len + PACKET_MESH_TRS_SEG_CONTROL_PDU_MAX_SIZE - 1) /
             PACKET_MESH_TRS_SEG_CONTROL_PDU_MAX_SIZE);
        p_metadata->segmentation.last_segment = num_segments - 1;
        p_metadata->segmentation.segment_offset = 0;
    }

    p_metadata->net.control_packet      = true;
    p_metadata->net.dst                 = p_tx_params->dst;
    p_metadata->net.src                 = p_tx_params->src;
    p_metadata->net.p_security_material = p_tx_params->p_net_secmat;
    p_metadata->net.ttl                 = p_tx_params->ttl;
    p_metadata->tx_bearer_selector      = p_tx_params->bearer_selector;

    p_metadata->token = tx_token;

#if MESH_FEATURE_FRIEND_ENABLED
    if (friend_needs_packet(p_metadata))
    {
        if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            p_metadata->tx_bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
        }
        else
        {
            p_metadata->tx_bearer_selector |= CORE_TX_BEARER_TYPE_LOCAL;
        }
    }
#endif
}

static uint32_t unsegmented_packet_tx(transport_packet_metadata_t * p_metadata,
                                      const uint8_t * p_payload,
                                      uint32_t payload_len)
{
    if (payload_len + p_metadata->mic_size > TRANSPORT_UNSEG_PDU_LEN(p_metadata->net.control_packet))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    network_tx_packet_buffer_t net_buf;

    uint8_t * p_packet_buffer_payload;
    uint32_t status = upper_trs_packet_alloc(p_metadata,
                                             &net_buf,
                                             payload_len + p_metadata->mic_size,
                                             &p_packet_buffer_payload);
    if (status == NRF_SUCCESS)
    {
        trs_packet_header_build(p_metadata, (packet_mesh_trs_packet_t *) net_buf.p_payload);

        if (p_metadata->net.control_packet)
        {
            memcpy(p_packet_buffer_payload, p_payload, payload_len);
        }
        else
        {
            upper_trs_packet_encrypt(p_payload, p_packet_buffer_payload, payload_len, p_metadata);
        }

        network_packet_send(&net_buf);
    }
    return status;
}

static uint32_t sar_segment_send(trs_sar_ctx_t * p_sar_ctx, uint32_t segment_index)
{
    uint32_t segment_len = TRANSPORT_SAR_PDU_LEN(p_sar_ctx->metadata.net.control_packet);
    uint32_t payload_offset = segment_len * segment_index;

    /* The last segment won't necessarily fill the whole length: */
    if (segment_index == p_sar_ctx->metadata.segmentation.last_segment)
    {
        segment_len = p_sar_ctx->session.length - payload_offset;
    }

    p_sar_ctx->metadata.segmentation.segment_offset = segment_index;
    network_tx_packet_buffer_t net_buf;
    uint8_t * p_segment_payload;
    uint32_t status = upper_trs_packet_alloc(&p_sar_ctx->metadata,
                                             &net_buf,
                                             segment_len,
                                             &p_segment_payload);
    if (status == NRF_SUCCESS)
    {
        if (segment_index == 0 && !p_sar_ctx->session.params.tx.seqzero_is_set)
        {
            p_sar_ctx->metadata.segmentation.seq_zero =
                (p_sar_ctx->metadata.net.internal.sequence_number & TRANSPORT_SAR_SEQZERO_MASK);
            p_sar_ctx->session.params.tx.seqzero_is_set = true;
        }
        /* The payload couldn't be encrypted before we allocated the first packet, as we needed to
         * allocate a seqnum and iv index for the nonce. */
        if (!p_sar_ctx->metadata.net.control_packet && !p_sar_ctx->session.params.tx.payload_encrypted)
        {
            NRF_MESH_ASSERT(segment_index == 0);

            upper_trs_packet_encrypt(p_sar_ctx->payload,
                                     p_sar_ctx->payload,
                                     p_sar_ctx->session.length - p_sar_ctx->metadata.mic_size,
                                     &p_sar_ctx->metadata);
            p_sar_ctx->session.params.tx.payload_encrypted = true;
        }

        trs_packet_header_build(&p_sar_ctx->metadata, (packet_mesh_trs_packet_t *) net_buf.p_payload);
        memcpy(p_segment_payload, &p_sar_ctx->payload[payload_offset], segment_len);
        network_packet_send(&net_buf);
    }

    return status;
}

/**
 * Send SAR segments.
 *
 * @param[in,out] p_sar_ctx       SAR context to send segments of.
 * @param[out]    p_segment_count Number of segments successfully sent. Ignored if NULL.
 *
 * @retval NRF_SUCCESS Successfully sent all (remaining) segments.
 * @retval NRF_ERROR_NO_MEM No memory to sent all segments.
 * @return Other returns from lower layers. E.g., the network layer may disallow segment allocation
 * because there are no sequence number(s) available.
 */
static uint32_t trs_sar_packet_out(trs_sar_ctx_t * p_sar_ctx, uint32_t * p_segment_count)
{
    uint32_t status = NRF_SUCCESS;

    if (p_segment_count != NULL)
    {
        *p_segment_count = 0;
    }

    /* Starts at start_index if, e.g., there was no memory available last round. */
    for (uint8_t i = p_sar_ctx->session.params.tx.start_index;
         i <= p_sar_ctx->metadata.segmentation.last_segment;
         ++i)
    {
        if ((p_sar_ctx->session.block_ack & (1u << i)) == 0)
        {
            /* packet hasn't been acked yet */
            status = sar_segment_send(p_sar_ctx, i);
            if (status == NRF_SUCCESS)
            {
                if (p_segment_count != NULL)
                {
                    *p_segment_count = *p_segment_count + 1 ;
                }
                /* Set the start index to the next packet, so we know where to pick up in case
                 * of failure: */
                p_sar_ctx->session.params.tx.start_index = i + 1;
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

static uint32_t segmented_packet_tx(const transport_packet_metadata_t * p_metadata,
                                    const uint8_t * p_payload,
                                    uint32_t payload_len)
{
    uint32_t packet_length = payload_len + p_metadata->mic_size;
    if (packet_length > TRANSPORT_SAR_PACKET_MAX_SIZE(p_metadata->net.control_packet))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    /* According to @tagMeshSp section 3.6.4.1, we should only ever send one
     * transport SAR packet at the same time between a given source and destination. Return FORBIDDEN if
     * there's a SAR session in progress with the same parameters. */
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_TX &&
            m_trs_sar_sessions[i].metadata.net.src == p_metadata->net.src &&
            m_trs_sar_sessions[i].metadata.net.dst.value == p_metadata->net.dst.value)
        {
            return NRF_ERROR_INVALID_STATE;
        }
    }

    trs_sar_ctx_t * p_sar_ctx = sar_ctx_alloc(p_metadata, TRS_SAR_SESSION_TX, packet_length);
    if (p_sar_ctx == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    memcpy(p_sar_ctx->payload, p_payload, payload_len);

    uint32_t sent_segments = 0;
    uint32_t status = trs_sar_packet_out(p_sar_ctx, &sent_segments);
    if (status == NRF_SUCCESS ||
        status == NRF_ERROR_NO_MEM)
    {
        /* For NRF_ERROR_NO_MEM we assume there will be more memory available on the next
         * TX_COMPLETE event. */
        if (sent_segments > 0)
        {
            tx_retry_timer_reset(p_sar_ctx);
        }

        __LOG_XB(LOG_SRC_TRANSPORT,
                 LOG_LEVEL_INFO,
                 "TX:SAR packet",
                 p_sar_ctx->payload,
                 p_sar_ctx->session.length);

        return NRF_SUCCESS;
    }
    else
    {
        sar_ctx_free(p_sar_ctx);
        return status;
    }
}


/** Process ongoing SAR TX sessions. */
static void trs_sar_tx_process(void)
{
    /* Loop through all active sessions and (re-)transmit remaining packets. */
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_TX)
        {
            uint32_t sent_segments = 0;
            uint32_t status = trs_sar_packet_out(&m_trs_sar_sessions[i], &sent_segments);

            /* Assume that there will be available memory on the next TX_COMPLETE event. */
            if (status == NRF_ERROR_NO_MEM || sent_segments != 0)
            {
                tx_retry_timer_reset(&m_trs_sar_sessions[i]);
            }
        }
    }
}

static void trs_sar_rx_pending_ack_send(trs_sar_ctx_t * p_sar_ctx)
{
    if (p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX &&
        p_sar_ctx->session.params.rx.ack_state == SAR_ACK_STATE_PENDING)
    {
        if (sar_ack_send(&p_sar_ctx->metadata, p_sar_ctx->session.block_ack) == NRF_SUCCESS)
        {
            p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
        }
    }
}

static void trs_sar_rx_process(void)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        trs_sar_rx_pending_ack_send(&m_trs_sar_sessions[i]);
    }
}

static bool transport_sar_process(void)
{
    trs_sar_tx_process();
    trs_sar_rx_process();
    return true;
}

static uint32_t upper_trs_packet_decrypt(transport_packet_metadata_t * p_metadata,
                                         const uint8_t * p_upper_trs_packet,
                                         uint32_t upper_trs_packet_len,
                                         uint8_t * p_upper_trs_packet_out)
{
    /* Use seq_zero network sequence number if this is a segmented message */
    uint32_t old_network_seqnum = p_metadata->net.internal.sequence_number;
    if (p_metadata->segmented)
    {
        p_metadata->net.internal.sequence_number =
                transport_sar_first_seq_num_get(p_metadata->net.internal.sequence_number,
                                                p_metadata->segmentation.seq_zero);
    }
    /* Calculate nonce */
    uint8_t app_nonce_buf[CCM_NONCE_LENGTH];
    enc_nonce_generate(&p_metadata->net,
                       ((p_metadata->type.access.using_app_key) ? ENC_NONCE_APP : ENC_NONCE_DEV),
                       (p_metadata->mic_size == PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE),
                       app_nonce_buf);
    /* Reset the network sequence number to its original value, to keep a predictable state. */
    p_metadata->net.internal.sequence_number = old_network_seqnum;

    ccm_soft_data_t ccm_data;
    ccm_data.p_nonce = app_nonce_buf;
    ccm_data.p_m     = p_upper_trs_packet;
    ccm_data.m_len   = (upper_trs_packet_len - p_metadata->mic_size);
    ccm_data.p_mic   = (uint8_t *) ccm_data.p_m + ccm_data.m_len;
    ccm_data.p_out   = p_upper_trs_packet_out;
    ccm_data.mic_len = p_metadata->mic_size;
    ccm_data.a_len = 0;

    if (p_metadata->type.access.using_app_key)
    {
        do {
            if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
            {
                ccm_data.a_len = NRF_MESH_UUID_SIZE;
                ccm_data.p_a = p_metadata->net.dst.p_virtual_uuid;
            }

            /* Application key */
            p_metadata->p_security_material = NULL;
            const nrf_mesh_application_secmat_t * p_secmat[2] = { NULL, NULL };
            do {
                nrf_mesh_app_secmat_next_get(p_metadata->net.p_security_material,
                                             p_metadata->type.access.app_key_id,
                                             &p_secmat[0], &p_secmat[1]);

                for (uint32_t i = 0; i < ARRAY_SIZE(p_secmat) && p_secmat[i] != NULL; i++)
                {
                    if (test_transport_decrypt(p_secmat[i], &ccm_data))
                    {
                        p_metadata->p_security_material = p_secmat[i];
                        return NRF_SUCCESS;
                    }
                }
            } while (p_secmat[0] != NULL);
        } while (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
                 nrf_mesh_rx_address_get(p_metadata->net.dst.value, &p_metadata->net.dst));
    }
    else if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST) // Device keys can only be used for unicast addresses.
    {
        /* Device key */
        /* Device keys are always attached to a unicast address. Try the devkey of both the source and the destination address: */
        const uint16_t devkey_addrs[] = {
            p_metadata->net.dst.value,
            p_metadata->net.src
        };
        for (uint32_t i = 0; i < ARRAY_SIZE(devkey_addrs); ++i)
        {
            p_metadata->p_security_material = NULL;
            nrf_mesh_devkey_secmat_get(devkey_addrs[i], &p_metadata->p_security_material);
            if (test_transport_decrypt(p_metadata->p_security_material, &ccm_data))
            {
                return NRF_SUCCESS;
            }
        }
    }

    return NRF_ERROR_NOT_FOUND;
}

/**
 * Validate transport packet metadata for both incoming and outgoing packets.
 *
 * @param[in] p_trs_metadata Transport metadata to validate.
 *
 * @returns A status code for the result of the validation.
 */
static uint32_t transport_metadata_validate(const transport_packet_metadata_t * p_trs_metadata)
{
    if (nrf_mesh_address_type_get(p_trs_metadata->net.src) != NRF_MESH_ADDRESS_TYPE_UNICAST ||
        p_trs_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_INVALID)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (p_trs_metadata->net.ttl > NRF_MESH_TTL_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (p_trs_metadata->net.control_packet)
    {
        /* @tagMeshSp section 3.4.3: Only unicast and group destination addresses are valid for control packets */
        if (p_trs_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
        {
            return NRF_ERROR_INVALID_ADDR;
        }

        if ((uint8_t) p_trs_metadata->type.control.opcode > TRANSPORT_CONTROL_PACKET_OPCODE_MAX ||
            p_trs_metadata->mic_size != PACKET_MESH_TRS_TRANSMIC_CONTROL_SIZE)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
    else
    {
        /* @tagMeshSp section 3.4.3: Only unicast destination addresses are valid with device keys */
        if (!p_trs_metadata->type.access.using_app_key && p_trs_metadata->net.dst.type != NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            return NRF_ERROR_INVALID_PARAM;
        }

        if (!p_trs_metadata->segmented && p_trs_metadata->mic_size != PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }

    return NRF_SUCCESS;
}

static uint32_t transport_metadata_build(const packet_mesh_trs_packet_t * p_transport_packet,
                                         uint32_t packet_len,
                                         const network_packet_metadata_t * p_net_metadata,
                                         const nrf_mesh_rx_metadata_t * p_rx_metadata,
                                         transport_packet_metadata_t * p_trs_metadata)
{
    // Ensure that the read of the "seg" field isn't out of bounds:
    if (packet_len < PACKET_MESH_TRS_COMMON_SEG_OFFSET + 1)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    memcpy(&p_trs_metadata->net, p_net_metadata, sizeof(network_packet_metadata_t));
    p_trs_metadata->receivers = TRANSPORT_PACKET_RECEIVER_NONE; // will be filled in gradually
    p_trs_metadata->segmented = packet_mesh_trs_common_seg_get(p_transport_packet);

    // Ensure that the rest of the fields aren't out of bounds:
    uint32_t min_len = (p_trs_metadata->segmented ? PACKET_MESH_TRS_SEG_PDU_OFFSET : PACKET_MESH_TRS_UNSEG_PDU_OFFSET);
    if (packet_len < min_len)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (p_trs_metadata->net.control_packet)
    {
        p_trs_metadata->type.control.opcode = (transport_control_opcode_t) packet_mesh_trs_control_opcode_get(p_transport_packet);
        p_trs_metadata->mic_size = PACKET_MESH_TRS_TRANSMIC_CONTROL_SIZE;
    }
    else
    {
        p_trs_metadata->type.access.using_app_key = packet_mesh_trs_access_akf_get(p_transport_packet);
        p_trs_metadata->type.access.app_key_id = packet_mesh_trs_access_aid_get(p_transport_packet);
        if (p_trs_metadata->segmented)
        {
            p_trs_metadata->mic_size = ((packet_mesh_trs_seg_szmic_get(p_transport_packet)) ? PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE : PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE);
        }
        else
        {
            p_trs_metadata->mic_size = PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE;

            if (packet_len < min_len + p_trs_metadata->mic_size)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
        }
    }

    if (p_trs_metadata->segmented)
    {
        p_trs_metadata->segmentation.seq_zero       = packet_mesh_trs_seg_seqzero_get(p_transport_packet);
        p_trs_metadata->segmentation.segment_offset = packet_mesh_trs_seg_sego_get(p_transport_packet);
        p_trs_metadata->segmentation.last_segment   = packet_mesh_trs_seg_segn_get(p_transport_packet);
    }
    /* Not enough information to populate the secmat yet. */
    p_trs_metadata->p_security_material = NULL;

    /* Map RX source to the TX bearer we should use to send responses */
    static const core_tx_bearer_selector_t rx_source_to_tx_bearer[] = {
        [NRF_MESH_RX_SOURCE_SCANNER]    = CORE_TX_BEARER_TYPE_ADV,
        [NRF_MESH_RX_SOURCE_GATT]       = CORE_TX_BEARER_TYPE_GATT_SERVER,
        [NRF_MESH_RX_SOURCE_INSTABURST] = CORE_TX_BEARER_TYPE_ADV,
        [NRF_MESH_RX_SOURCE_LOOPBACK]   = CORE_TX_BEARER_TYPE_LOCAL,
    };
    NRF_MESH_ASSERT((uint32_t) p_rx_metadata->source < ARRAY_SIZE(rx_source_to_tx_bearer));
    p_trs_metadata->tx_bearer_selector = rx_source_to_tx_bearer[p_rx_metadata->source];

    return NRF_SUCCESS;
}

static void segack_packet_in(const packet_mesh_trs_control_packet_t * p_trs_control_packet,
                             uint32_t control_packet_len,
                             transport_packet_metadata_t * p_metadata,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    if (control_packet_len != PACKET_MESH_TRS_CONTROL_SEGACK_SIZE)
    {
        return;
    }
    uint16_t seq_zero = packet_mesh_trs_control_segack_seqzero_get(p_trs_control_packet);
    uint32_t block_ack = packet_mesh_trs_control_segack_block_ack_get(p_trs_control_packet);
    bool obo = (bool) packet_mesh_trs_control_segack_obo_get(p_trs_control_packet);

    trs_sar_ctx_t * p_sar_ctx = sar_active_tx_ctx_get(p_metadata, seq_zero);
    if (p_sar_ctx == NULL)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Could not find active session for ACK packet\n");
    }
    else if (!obo && p_sar_ctx->metadata.net.dst.value != p_metadata->net.src)
    {
        __LOG(LOG_SRC_TRANSPORT,
              LOG_LEVEL_WARN,
              "Got ACK packet from unexpected device (expected %04x, got %04x)\n",
              p_sar_ctx->metadata.net.dst.value,
              p_metadata->net.src);
    }
    else
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_TRS_ACK_RECEIVED, 0, sizeof(uint32_t), &p_metadata->net.internal.sequence_number);

        if (obo)
        {
            /* @tagMeshSp section 3.5.3.3 the source address of the first ack
             * must be the source address of all the acks if the obo flag is set. */
            p_sar_ctx->session.params.tx.ack_src = p_metadata->net.src;
        }

        p_sar_ctx->session.block_ack |= block_ack;

        if (block_ack == 0)
        {
            sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_BY_PEER);
        }
        else if (p_sar_ctx->session.block_ack == block_ack_full(&p_sar_ctx->metadata))
        {
            /* All segments received */
            sar_ctx_tx_complete(p_sar_ctx);
        }
        else
        {
            /* @tagMeshSp section 3.5.3.3: "If a Segment Acknowledgment message is received that
             * is a valid acknowledgment for the segmented message, then the lower transport layer
             * shall reset the segment transmission timer and retransmit all unacknowledged Lower
             * Transport PDUs." */
             p_sar_ctx->session.params.tx.start_index = 0;

             /* We reset the timer regardless here => ignore the return. */
             (void) trs_sar_packet_out(p_sar_ctx, NULL);
             tx_retry_timer_reset(p_sar_ctx);
        }
    }
}

static transport_control_packet_callback_t control_packet_callback_get(transport_control_opcode_t opcode)
{
    for (uint32_t consumer = 0; consumer < m_control_packet_consumer_count; ++consumer)
    {
        const control_packet_consumer_t * p_consumer = &m_control_packet_consumers[consumer];
        for (uint32_t handler = 0; handler < p_consumer->handler_count; ++handler)
        {
            const transport_control_packet_handler_t * p_handler = &p_consumer->p_handlers[handler];
            if (p_handler->opcode == opcode)
            {
                return p_handler->callback;
            }
        }
    }
    return NULL;
}

static void transport_control_packet_in(const packet_mesh_trs_control_packet_t * p_trs_control_packet,
                                        uint32_t control_packet_len,
                                        transport_packet_metadata_t * p_metadata,
                                        const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    /* There's no further validation of control packets, add them to the replay list. */
    if (replay_list_add(p_metadata) == NRF_SUCCESS)
    {
        switch (p_metadata->type.control.opcode)
        {
            /* The Segack handler is internal to the lower transport layer, and is handled inline. */
            case TRANSPORT_CONTROL_OPCODE_SEGACK:
                segack_packet_in(p_trs_control_packet, control_packet_len, p_metadata, p_rx_metadata);
                break;

            default:
            {
                transport_control_packet_callback_t callback = control_packet_callback_get(p_metadata->type.control.opcode);
                if (callback != NULL)
                {
                    transport_control_packet_t control_packet;
                    control_packet.opcode          = p_metadata->type.control.opcode;
                    control_packet.p_net_secmat    = p_metadata->net.p_security_material;
                    control_packet.src             = p_metadata->net.src;
                    control_packet.dst             = p_metadata->net.dst;
                    control_packet.ttl             = p_metadata->net.ttl;
                    control_packet.p_data          = p_trs_control_packet;
                    control_packet.data_len        = control_packet_len;
                    control_packet.reliable        = p_metadata->segmented;
                    control_packet.bearer_selector = p_metadata->tx_bearer_selector;
                    callback(&control_packet, p_rx_metadata);
                }
                break;
            }
        }
    }
}

static void upper_transport_access_packet_in(const uint8_t * p_upper_trs_packet,
                                             uint32_t upper_trs_packet_len,
                                             transport_packet_metadata_t * p_metadata,
                                             const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    uint8_t decrypt_buffer[TRANSPORT_SAR_PACKET_MAX_SIZE(false)]; /* 382 bytes! */

    uint32_t status = upper_trs_packet_decrypt(p_metadata, p_upper_trs_packet, upper_trs_packet_len, decrypt_buffer);
    if (status == NRF_SUCCESS)
    {
        /* This message has passed all checks and can be added to the replay list. */
        if (replay_list_add(p_metadata) == NRF_SUCCESS)
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_DECRYPT_TRS,
                                    0,
                                    upper_trs_packet_len - p_metadata->mic_size,
                                    p_upper_trs_packet);

            nrf_mesh_address_t src_address = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST,
                                            .value = p_metadata->net.src,
                                            .p_virtual_uuid = NULL};
            nrf_mesh_evt_t rx_event;
            rx_event.type = NRF_MESH_EVT_MESSAGE_RECEIVED;
            rx_event.params.message.p_buffer = decrypt_buffer;
            rx_event.params.message.length = upper_trs_packet_len - p_metadata->mic_size;
            rx_event.params.message.src = src_address;
            rx_event.params.message.dst = p_metadata->net.dst;
            rx_event.params.message.ttl = p_metadata->net.ttl;
            rx_event.params.message.secmat.p_net = p_metadata->net.p_security_material;
            rx_event.params.message.secmat.p_app = p_metadata->p_security_material;
            rx_event.params.message.p_metadata = p_rx_metadata;
            event_handle(&rx_event);
        }
    }
    else
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_DBG2, "Could not decrypt transport layer data.\n");

        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                                (p_metadata->type.access.using_app_key ? PACKET_DROPPED_INVALID_APPKEY
                                                                : PACKET_DROPPED_INVALID_DEVKEY),
                                upper_trs_packet_len,
                                p_upper_trs_packet);
    }
}

static void upper_transport_packet_in(const uint8_t * p_upper_trs_packet,
                                      uint32_t upper_trs_packet_len,
                                      transport_packet_metadata_t * p_metadata,
                                      const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    if (p_metadata->receivers & TRANSPORT_PACKET_RECEIVER_SELF)
    {
        if (p_metadata->net.control_packet)
        {
            transport_control_packet_in((packet_mesh_trs_control_packet_t *) p_upper_trs_packet,
                                        upper_trs_packet_len,
                                        p_metadata,
                                        p_rx_metadata);
        }
        else
        {
            upper_transport_access_packet_in(p_upper_trs_packet, upper_trs_packet_len, p_metadata, p_rx_metadata);
        }
    }
}

static void ack_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX);
    p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_PENDING;
    trs_sar_rx_pending_ack_send(p_sar_ctx);
}

static void retry_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_TX);

#if MESH_FEATURE_LPN_ENABLED
    if (mesh_lpn_is_in_friendship())
    {
        (void) mesh_lpn_friend_poll(0);
        p_sar_ctx->timeout_state = SAR_TIMEOUT_STATE_WAIT_POLL_COMPLETE;
    }
    else
#endif
    {
        sar_tx_session_continue(p_sar_ctx);
    }
}

static void abort_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_TIMEOUT);
}

static void tx_complete(core_tx_role_t role, uint32_t bearer_index, uint32_t timestamp, nrf_mesh_tx_token_t token)
{
    if (role == CORE_TX_ROLE_ORIGINATOR && token != NRF_MESH_SAR_TOKEN)
    {
        /* This tx complete came from the application. */
        nrf_mesh_evt_t evt;
        evt.type = NRF_MESH_EVT_TX_COMPLETE;
        evt.params.tx_complete.token = token;
        evt.params.tx_complete.timestamp = timestamp;
        event_handle(&evt);
    }
    bearer_event_flag_set(m_sar_process_flag);
}

static uint32_t upper_transport_tx(transport_packet_metadata_t * p_metadata, const uint8_t * p_data, uint32_t data_len)
{
    uint32_t status = transport_metadata_validate(p_metadata);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    if (p_metadata->segmented)
    {
        /* TODO: If LPN is enabled, but we're not in a friendship, we may prohibit sending segmented
         * messages to unicast destinations -- as we're not able to receive the ACKs
         * (MBTLE-2825). */
        return segmented_packet_tx(p_metadata, p_data, data_len);
    }
    else
    {
        return unsegmented_packet_tx(p_metadata, p_data, data_len);
    }
}

/**************
 * Public API *
 **************/
void transport_init(void)
{
    memset(&m_trs_sar_sessions[0], 0, sizeof(m_trs_sar_sessions));

    replay_cache_init();

    m_canceled_sar_rx_sessions_cache_head = 0;
    memset(m_canceled_sar_rx_sessions_cache, 0, sizeof(m_canceled_sar_rx_sessions_cache));

    m_trs_config.rx_timeout                = TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US;
    m_trs_config.rx_ack_base_timeout       = TRANSPORT_SAR_RX_ACK_BASE_TIMEOUT_DEFAULT_US;
    m_trs_config.rx_ack_per_hop_addition   = TRANSPORT_SAR_RX_ACK_PER_HOP_ADDITION_DEFAULT_US;
    m_trs_config.tx_retry_base_timeout     = TRANSPORT_SAR_TX_RETRY_BASE_TIMEOUT_DEFAULT_US;
    m_trs_config.tx_retry_per_hop_addition = TRANSPORT_SAR_TX_RETRY_PER_HOP_ADDITION_DEFAULT_US;
    m_trs_config.tx_retries                = TRANSPORT_SAR_TX_RETRIES_DEFAULT;
    m_trs_config.szmic                     = NRF_MESH_TRANSMIC_SIZE_SMALL;
    m_trs_config.segack_ttl                = TRANSPORT_SAR_SEGACK_TTL_DEFAULT;
    m_sar_process_flag = bearer_event_flag_add(transport_sar_process);
    m_control_packet_consumer_count = 0;

    core_tx_complete_cb_set(tx_complete);

#if MESH_FEATURE_LPN_ENABLED
    static nrf_mesh_evt_handler_t s_mesh_evt_handler = {.evt_cb = mesh_evt_handler};
    nrf_mesh_evt_handler_add(&s_mesh_evt_handler);
#endif
}

void transport_enable(void)
{
    replay_cache_enable();
}

uint32_t transport_packet_in(const packet_mesh_trs_packet_t * p_packet,
                             uint32_t trs_packet_len,
                             const network_packet_metadata_t * p_net_metadata,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    uint32_t status = NRF_SUCCESS;

    if (p_packet == NULL || p_net_metadata == NULL)
    {
        return NRF_ERROR_NULL;
    }

    transport_packet_metadata_t trs_metadata;
    status = transport_metadata_build(p_packet, trs_packet_len, p_net_metadata, p_rx_metadata, &trs_metadata);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    status = transport_metadata_validate(&trs_metadata);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

#if MESH_FEATURE_LPN_ENABLED
    /* Notify the LPN about a successfully decrypted network PDU. It should be notified before
     * checking the replay cache as it means that the Friend Poll got a reply and the LPN can cancel
     * the radio scanning -- given the correct friendship credentials.
     */
    mesh_lpn_rx_notify(p_net_metadata);
#endif

#if MESH_FEATURE_FRIEND_ENABLED
    if (friend_needs_packet(&trs_metadata))
    {
        trs_metadata.receivers |= TRANSPORT_PACKET_RECEIVER_FRIEND;
    }
#endif

    /* Don't overwrite the dst in the transport metadata, in case the message is
     * for one of our friends. */
    nrf_mesh_address_t dst;
    memset(&dst, 0, sizeof(dst));
    if (nrf_mesh_rx_address_get(p_net_metadata->dst.value, &dst))
    {
        trs_metadata.receivers |= TRANSPORT_PACKET_RECEIVER_SELF;
    }

    if (replay_cache_has_elem(p_net_metadata->src,
                              p_net_metadata->internal.sequence_number,
                              p_net_metadata->internal.iv_index))
    {
        /* Apply replay protection only to this node. */
        trs_metadata.receivers &= ~TRANSPORT_PACKET_RECEIVER_SELF; /*lint !e64 Type mismatch */

        /* Continue processing if the packet is for the friendship device. */
        if (trs_metadata.receivers == TRANSPORT_PACKET_RECEIVER_NONE)
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                                  PACKET_DROPPED_REPLAY_CACHE,
                                  sizeof(uint32_t),
                                  &p_net_metadata->internal.sequence_number);

            return status;
        }
    }

    if (trs_metadata.receivers == TRANSPORT_PACKET_RECEIVER_NONE)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                              PACKET_DROPPED_UNKNOWN_ADDRESS,
                              trs_packet_len,
                              p_packet);
        return status;
    }

    /* Copy in the virtual UUID if the address was found, the rest is
     * already there. */
    if (trs_metadata.receivers & TRANSPORT_PACKET_RECEIVER_SELF)
    {
        trs_metadata.net.dst.p_virtual_uuid = dst.p_virtual_uuid;
    }

    if (trs_metadata.segmented)
    {
        trs_seg_packet_in(p_packet, trs_packet_len, &trs_metadata, p_rx_metadata);
    }
    else
    {
        trs_unseg_packet_in(p_packet, trs_packet_len, &trs_metadata, p_rx_metadata);
    }

    return NRF_SUCCESS;
}

uint32_t transport_tx(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference)
{
    if (p_params == NULL ||
        p_params->p_data == NULL ||
        p_params->security_material.p_app == NULL ||
        p_params->security_material.p_net == NULL)
    {
        return NRF_ERROR_NULL;
    }

    transport_packet_metadata_t metadata;
    transport_metadata_from_tx_params(&metadata, p_params);

    return upper_transport_tx(&metadata, p_params->p_data, p_params->data_len);
}

uint32_t transport_control_tx(const transport_control_packet_t * p_params, nrf_mesh_tx_token_t tx_token)
{
    if (p_params == NULL ||
        p_params->p_data == NULL ||
        p_params->p_net_secmat == NULL)
    {
        return NRF_ERROR_NULL;
    }

    transport_packet_metadata_t metadata;
    transport_metadata_from_control_tx_params(&metadata, p_params, tx_token);

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_DBG3, "TX: control opcode: 0x%02x secmat: 0x%08x\n",
          p_params->opcode, p_params->p_net_secmat);

    return upper_transport_tx(&metadata, p_params->p_data->pdu, p_params->data_len);
}

uint32_t transport_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt)
{
    if (!p_opt)
    {
        return NRF_ERROR_NULL;
    }

    switch (id)
    {
        case NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT:
            if (p_opt->opt.val < TRANSPORT_SAR_RX_TIMEOUT_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_RX_TIMEOUT_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.rx_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_BASE:
            if (p_opt->opt.val < TRANSPORT_SAR_RX_ACK_TIMEOUT_BASE_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_RX_ACK_TIMEOUT_BASE_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.rx_ack_base_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION:
            if (p_opt->opt.val < TRANSPORT_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.rx_ack_per_hop_addition = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_BASE:
            if (p_opt->opt.val < TRANSPORT_SAR_TX_RETRY_TIMEOUT_BASE_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_TX_RETRY_TIMEOUT_BASE_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.tx_retry_base_timeout = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION:
            if (p_opt->opt.val < TRANSPORT_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }

            m_trs_config.tx_retry_per_hop_addition = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRIES:
            if (p_opt->opt.val < TRANSPORT_SAR_TX_RETRIES_MIN ||
                p_opt->opt.val > TRANSPORT_SAR_TX_RETRIES_MAX)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.tx_retries = (uint8_t) p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SAR_SEGACK_TTL:
            if (p_opt->opt.val > NRF_MESH_TTL_MAX || p_opt->opt.val < 1)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.segack_ttl = p_opt->opt.val;
            break;

        case NRF_MESH_OPT_TRS_SZMIC:
            if (p_opt->opt.val != NRF_MESH_TRANSMIC_SIZE_LARGE && p_opt->opt.val != NRF_MESH_TRANSMIC_SIZE_SMALL)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            m_trs_config.szmic = (nrf_mesh_transmic_size_t) p_opt->opt.val;
            break;

        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t transport_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt)
{
    if (!p_opt)
    {
        return NRF_ERROR_NULL;
    }

    memset(p_opt, 0, sizeof(nrf_mesh_opt_t));

    switch (id)
    {
        case NRF_MESH_OPT_TRS_SAR_RX_TIMEOUT:
            p_opt->opt.val = m_trs_config.rx_timeout;
            break;

        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_BASE:
            p_opt->opt.val = m_trs_config.rx_ack_base_timeout;
            break;
        case NRF_MESH_OPT_TRS_SAR_RX_ACK_TIMEOUT_PER_HOP_ADDITION:
            p_opt->opt.val = m_trs_config.rx_ack_per_hop_addition;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_BASE:
            p_opt->opt.val = m_trs_config.tx_retry_base_timeout;
            break;
        case NRF_MESH_OPT_TRS_SAR_TX_RETRY_TIMEOUT_PER_HOP_ADDITION:
            p_opt->opt.val = m_trs_config.tx_retry_per_hop_addition;
            break;

        case NRF_MESH_OPT_TRS_SAR_TX_RETRIES:
            p_opt->opt.val = m_trs_config.tx_retries;
            break;

        case NRF_MESH_OPT_TRS_SAR_SEGACK_TTL:
            p_opt->opt.val = m_trs_config.segack_ttl;
            break;

        case NRF_MESH_OPT_TRS_SZMIC:
            p_opt->opt.val = m_trs_config.szmic;
            break;

        default:
            return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t transport_control_packet_consumer_add(const transport_control_packet_handler_t * p_handlers, uint32_t handler_count)
{
    if (p_handlers == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (m_control_packet_consumer_count >= TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX)
    {
        return NRF_ERROR_NO_MEM;
    }

    for (uint32_t i = 0; i < handler_count; ++i)
    {
        if (p_handlers[i].callback == NULL)
        {
            return NRF_ERROR_NULL;
        }
        if ((uint8_t) p_handlers[i].opcode > TRANSPORT_CONTROL_PACKET_OPCODE_MAX)
        {
            return NRF_ERROR_INVALID_DATA;
        }
        if (p_handlers[i].opcode == TRANSPORT_CONTROL_OPCODE_SEGACK ||
            control_packet_callback_get(p_handlers[i].opcode) != NULL)
        {
            /* duplicate */
            return NRF_ERROR_FORBIDDEN;
        }
    }

    m_control_packet_consumers[m_control_packet_consumer_count].p_handlers = p_handlers;
    m_control_packet_consumers[m_control_packet_consumer_count].handler_count = handler_count;
    m_control_packet_consumer_count++;
    return NRF_SUCCESS;
}
