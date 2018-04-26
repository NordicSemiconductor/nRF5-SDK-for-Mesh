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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include <nrf_error.h>

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
#include "transport.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_externs.h"
#include "packet_mesh.h"

/*********************
 * Local definitions *
 *********************/

/** Mask for SeqZero value. */
#define TRANSPORT_SAR_SEQZERO_MASK (TRANSPORT_SAR_SEQNUM_DIFF_MAX)
#define TRANSPORT_SAR_SEGMENT_COUNT_MAX 32
#define TRANSPORT_SAR_PDU_LEN(control) ((control) ? PACKET_MESH_TRS_SEG_CONTROL_PDU_MAX_SIZE : PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE)
#define TRANSPORT_SAR_PACKET_MAX_SIZE(control) (TRANSPORT_SAR_SEGMENT_COUNT_MAX * TRANSPORT_SAR_PDU_LEN(control))
#define TRANSPORT_UNSEG_PDU_LEN(control) ((control) ? PACKET_MESH_TRS_UNSEG_CONTROL_PDU_MAX_SIZE : PACKET_MESH_TRS_UNSEG_ACCESS_PDU_MAX_SIZE)

#define SAR_TOKEN ((nrf_mesh_tx_token_t) 0x5E65E65E)

NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(TRANSPORT_SAR_RX_CACHE_LEN));
/* The SEQZERO mask must be (power of two - 1) to work as a mask (ie if a bit in the mask is set to
 * 1, all lower bits must also be 1). */
NRF_MESH_STATIC_ASSERT(IS_POWER_OF_2(TRANSPORT_SAR_SEQZERO_MASK + 1));

#define TRANSPORT_SAR_RX_CACHE_LEN_MASK    (TRANSPORT_SAR_RX_CACHE_LEN - 1)
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
    timestamp_t rx_ack_base_timeout;       /**< RX acknowledgement timer timeout value. */
    timestamp_t rx_ack_per_hop_addition;   /**< RX acknowledgement time delay added per hop in the transmission. */
    timestamp_t tx_retry_base_timeout;     /**< Base timeout for TX retries.*/
    timestamp_t tx_retry_per_hop_addition; /**< TX retry time added per hop in the transmission. */
    uint8_t tx_retries;                    /**< Number of retries before cancelling SAR session. */
    uint8_t segack_ttl; /**< Default TTL value for segment acknowledgement messages. */
    uint8_t szmic;      /**< Use 32- or 64-bit MIC for application payload. */
} transport_config_t;

typedef struct
{
    uint16_t seq_zero;
    uint8_t segment_offset;
    uint8_t last_segment;
    uint8_t mic_size;
} transport_segmentation_metadata_t;

typedef struct
{
    bool segmented;
    uint8_t mic_size;
    union
    {
        struct
        {
            bool using_app_key;
            uint8_t app_key_id;
        } access;
        struct
        {
            transport_control_opcode_t opcode;
        } control;
    } type;
    transport_segmentation_metadata_t segmentation;
    network_packet_metadata_t net;
    const nrf_mesh_application_secmat_t * p_security_material;
    nrf_mesh_tx_token_t token;
} transport_packet_metadata_t;

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
                bool seqzero_is_set;         /**< Flag indicating whether the seqzero has been set. */
                nrf_mesh_tx_token_t token;      /**< TX Token set by the user. */
            } tx;
            /** Fields that are only valid for RX-sesssions */
            struct
            {
                /** Acknowledgement timer */
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

/** Completed SAR session, used to cache previous sessions. */
typedef struct
{
    bool successful;
    uint8_t ivi;
    uint16_t src;
    uint32_t seqauth_seqnum;
} completed_sar_session_t;

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

static transport_sar_alloc_t    m_sar_alloc;   /**< Allocation function for SAR packets. */
static transport_sar_release_t  m_sar_release; /**< Release function for SAR packets. */

static uint32_t m_sar_session_cache_head;
static completed_sar_session_t m_sar_session_cache[TRANSPORT_SAR_RX_CACHE_LEN];

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

static inline uint32_t block_ack_full(transport_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_metadata->segmented);
    return ((1u << (p_metadata->segmentation.last_segment + 1)) - 1);
}
static void upper_transport_packet_in(const uint8_t * p_upper_trs_packet,
                                      uint32_t upper_trs_packet_len,
                                      transport_packet_metadata_t * p_metadata,
                                      const nrf_mesh_rx_metadata_t * p_rx_metadata);

static uint32_t seqauth_sequence_number_get(uint32_t seqnum, uint16_t seqzero)
{
    if ((seqnum & TRANSPORT_SAR_SEQZERO_MASK) < seqzero)
    {
        return ((seqnum - ((seqnum & TRANSPORT_SAR_SEQZERO_MASK) - seqzero) - (TRANSPORT_SAR_SEQZERO_MASK + 1)));
    }
    else
    {
        return ((seqnum - ((seqnum & TRANSPORT_SAR_SEQZERO_MASK) - seqzero)));
    }
}

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

/**
 * Check whether the RX SAR session has been handled before. As the sessions are stored in a FIFO
 * cache manner, getting a pointer to the completed session.
 *
 * @param[in] p_metadata Metadata to check for.
 *
 * @retval Pointer to the completed session from the cache if there is. NULL otherwise.
 */
static completed_sar_session_t * sar_rx_session_previously_handled(const transport_packet_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_metadata->segmented);

    uint16_t src = p_metadata->net.src;
    uint8_t ivi = p_metadata->net.internal.iv_index & NETWORK_IVI_MASK;
    uint32_t seqauth_seqnum = seqauth_sequence_number_get(p_metadata->net.internal.sequence_number,
                                                          p_metadata->segmentation.seq_zero);

    for (uint32_t i = 0; i < TRANSPORT_SAR_RX_CACHE_LEN; ++i)
    {
        if (seqauth_seqnum == m_sar_session_cache[i].seqauth_seqnum &&
            src == m_sar_session_cache[i].src &&
            ivi == m_sar_session_cache[i].ivi)
        {
            return &m_sar_session_cache[i];
        }
    }
    return NULL;
}


/**
 * Places completed SAR session into the session cache.
 *
 * @param[in] p_metadata SAR metadata handled.
 * @param[in] succeeded  The session has been succeeded or not.
 */
static void sar_rx_session_mark_as_handled(const transport_packet_metadata_t * p_metadata, bool succeeded)
{
    NRF_MESH_ASSERT(p_metadata->segmented);

    completed_sar_session_t * p_completed_session = &m_sar_session_cache[m_sar_session_cache_head++ &
                                                                         TRANSPORT_SAR_RX_CACHE_LEN_MASK];

    p_completed_session->src = p_metadata->net.src;
    p_completed_session->seqauth_seqnum = seqauth_sequence_number_get(p_metadata->net.internal.sequence_number,
                                                                      p_metadata->segmentation.seq_zero);
    p_completed_session->ivi = p_metadata->net.internal.iv_index & NETWORK_IVI_MASK;
    p_completed_session->successful = succeeded;
}

/**
 * Allocate the given SAR context with the given parameters.
 *
 * @param[in,out] p_sar_ctx SAR context
 * @param[in] p_metadata Metadata to use in the context.
 * @param[in] session_type Type of session.
 * @param[in] length Length of SAR data, including MIC.
 *
 * @returns Whether the allocation was successful.
 */
static bool sar_ctx_alloc(trs_sar_ctx_t * p_sar_ctx,
                          const transport_packet_metadata_t * p_metadata,
                          trs_sar_session_t session_type,
                          uint32_t length)
{
    NRF_MESH_ASSERT(session_type == TRS_SAR_SESSION_RX ||
                    session_type == TRS_SAR_SESSION_TX);
    NRF_MESH_ASSERT(p_sar_ctx->payload == NULL);
    p_sar_ctx->payload = m_sar_alloc(length);
    if (p_sar_ctx->payload == NULL)
    {
        return false;
    }
    memcpy(&p_sar_ctx->metadata, p_metadata, sizeof(transport_packet_metadata_t));
    p_sar_ctx->session.block_ack = 0;
    p_sar_ctx->session.length = length;
    /* Set the network metadata sequence number and IV index to the values in the first segment in
     * the session. */
    p_sar_ctx->metadata.net.internal.sequence_number =
        seqauth_sequence_number_get(p_metadata->net.internal.sequence_number,
                                    p_metadata->segmentation.seq_zero);

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
        p_sar_ctx->metadata.token = SAR_TOKEN;
        p_sar_ctx->timer_event.cb = retry_timeout;
        p_sar_ctx->timer_event.p_context = p_sar_ctx;
    }
    else
    {
        p_sar_ctx->metadata.token = SAR_TOKEN;
        p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
        p_sar_ctx->session.params.rx.ack_timer.cb = ack_timeout;
        p_sar_ctx->session.params.rx.ack_timer.p_context = p_sar_ctx;
        p_sar_ctx->timer_event.cb = abort_timeout;
        p_sar_ctx->timer_event.p_context = p_sar_ctx;
    }
    /* The IV index should stay the same for the duration of the session. */
    net_state_iv_index_lock(true);
    p_sar_ctx->session.session_type = session_type;
    return true;
}
static void sar_ctx_free(trs_sar_ctx_t * p_sar_ctx)
{
    m_sar_release(p_sar_ctx->payload);
    p_sar_ctx->payload = NULL;
    timer_sch_abort(&p_sar_ctx->timer_event);
    if (p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX)
    {
        timer_sch_abort(&p_sar_ctx->session.params.rx.ack_timer);
    }
    p_sar_ctx->session.session_type = TRS_SAR_SESSION_INACTIVE;
    net_state_iv_index_lock(false);
}

static void sar_ctx_cancel(trs_sar_ctx_t * p_sar_ctx, nrf_mesh_sar_session_cancel_reason_t reason)
{
    NRF_MESH_ASSERT(p_sar_ctx != NULL);

    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_SAR_CANCELLED, reason, 0, NULL);

    sar_rx_session_mark_as_handled(&p_sar_ctx->metadata, false);
    m_send_sar_cancel_event(p_sar_ctx->session.params.tx.token, reason);
    sar_ctx_free(p_sar_ctx);

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Dropped SAR session %u, reason %u\n",
          p_sar_ctx->session.session_type, reason);
}

static void sar_ctx_rx_complete(trs_sar_ctx_t * p_sar_ctx)
{
    sar_rx_session_mark_as_handled(&p_sar_ctx->metadata, true);
    sar_ctx_free(p_sar_ctx);
}

static void sar_ctx_tx_complete(trs_sar_ctx_t * p_sar_ctx)
{
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_TX);
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_TX_COMPLETE;
    evt.params.tx_complete.token = p_sar_ctx->session.params.tx.token;
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

    if (p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        p_sar_ctx->timer_event.interval = tx_retry_timer_delay_get(p_sar_ctx->metadata.net.ttl);
    }
    else
    {
        /* For non-unicast addresses, we're not going to get any acknowledgements, so there's no
         * point in scaling the retry interval according to TTL. */
        p_sar_ctx->timer_event.interval = tx_retry_timer_delay_get(0);
    }
    timer_sch_reschedule(&p_sar_ctx->timer_event, timer_now() + p_sar_ctx->timer_event.interval);
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
 * address (Mesh Profile Specification v1.0, section 3.5.3.4).
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
        packet_mesh_trs_control_packet_t packet_buffer;
        memset(&packet_buffer, 0, sizeof(packet_buffer));
        packet_mesh_trs_control_segack_seqzero_set(&packet_buffer, p_metadata->segmentation.seq_zero);
        packet_mesh_trs_control_segack_block_ack_set(&packet_buffer, block_ack);

        transport_control_packet_t control_packet;
        control_packet.data_len           = PACKET_MESH_TRS_CONTROL_SEGACK_SIZE;
        control_packet.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
        control_packet.dst.value          = p_metadata->net.src;
        control_packet.dst.p_virtual_uuid = NULL;
        control_packet.opcode             = TRANSPORT_CONTROL_OPCODE_SEGACK;
        control_packet.p_data             = &packet_buffer;
        control_packet.p_net_secmat       = p_metadata->net.p_security_material;
        control_packet.reliable           = false;
        control_packet.src                = p_metadata->net.dst.value;
        if (p_metadata->net.ttl == 0)
        {
            /* Respond with TTL=0 to TTL=0 messages. (Mesh Profile Specification v1.0, section 3.5.2.3) */
            control_packet.ttl = 0;
        }
        else
        {
            control_packet.ttl = m_trs_config.segack_ttl;
        }

        status = transport_control_tx(&control_packet, SAR_TOKEN);

        if (status == NRF_SUCCESS)
        {
            __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_ACK_QUEUED, 0, PACKET_MESH_TRS_CONTROL_SEGACK_SIZE, &packet_buffer);
        }
    }
    return status;
}

static trs_sar_ctx_t * sar_active_tx_ctx_get(transport_packet_metadata_t * p_metadata, uint16_t seq_zero)
{
    for (int i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_TX &&
            m_trs_sar_sessions[i].metadata.net.src == p_metadata->net.dst.value &&
            m_trs_sar_sessions[i].metadata.segmentation.seq_zero == seq_zero)
        {
            return &m_trs_sar_sessions[i];
        }
    }

    return NULL;
}

static trs_sar_ctx_t * sar_active_rx_ctx_get(transport_packet_metadata_t * p_metadata)
{
    for (int i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; i++)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_RX &&
            m_trs_sar_sessions[i].metadata.net.src == p_metadata->net.src)
        {
            return &m_trs_sar_sessions[i];
        }
    }

    return NULL;
}

static trs_sar_ctx_t * sar_rx_ctx_create(transport_packet_metadata_t * p_metadata)
{
    uint32_t total_length = (p_metadata->segmentation.last_segment + 1) *
                            TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet);

    if (total_length > TRANSPORT_SAR_PACKET_MAX_SIZE(p_metadata->net.control_packet))
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_ERROR, "Invalid length: %u\n", total_length);
        return NULL;
    }


    /* Check if the packet is part of an ongoing transaction. */
    for (int i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_INACTIVE)
        {
            if (!sar_ctx_alloc(&m_trs_sar_sessions[i], p_metadata, TRS_SAR_SESSION_RX, total_length))
            {
                break;
            }

            return &m_trs_sar_sessions[i];
        }
    }

    return NULL;
}

static void trs_sar_seg_packet_in(const uint8_t * p_segment_payload,
                                  uint32_t segment_len,
                                  transport_packet_metadata_t * p_metadata,
                                  const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    /* Find ongoing session by src. */
    trs_sar_ctx_t * p_sar_ctx = sar_active_rx_ctx_get(p_metadata);
    if (NULL != p_sar_ctx)
    {
        if (p_metadata->segmentation.seq_zero < p_sar_ctx->metadata.segmentation.seq_zero)
        {
            return;
        }

        if (p_metadata->segmentation.seq_zero > p_sar_ctx->metadata.segmentation.seq_zero)
        {
            sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
            p_sar_ctx = NULL;
        }
    }

    /* Look for session in the cache */
    completed_sar_session_t * p_completed_session = sar_rx_session_previously_handled(p_metadata);
    if (NULL != p_completed_session)
    {
        if (p_completed_session->successful)
        {
            /* Already successfully processed this session. */
            (void) sar_ack_send(p_metadata, block_ack_full(p_metadata));
        }
        return;
    }

    /* Create session */
    if (NULL == p_sar_ctx)
    {
        p_sar_ctx = sar_rx_ctx_create(p_metadata);
        if (p_sar_ctx == NULL)
        {
            /* Send block ack=0 to indicate failure to receive. Transport is out of resources. */
            (void) sar_ack_send(p_metadata, 0);
            m_send_sar_cancel_event(0, NRF_MESH_SAR_CANCEL_REASON_NO_MEM);
            return;
        }
    }

    __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_INFO, "Got segment %u\n", p_metadata->segmentation.segment_offset);

    /* Reset timers, even if we already received this segment. */
    rx_incomplete_timer_reset(p_sar_ctx);

    /* Only ACK if sent to unicast address. */
    if (p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST &&
        p_sar_ctx->session.params.rx.ack_state == SAR_ACK_STATE_IDLE) /* The ack timer should not be reset once it's running */
    {
        ack_timer_reset(p_sar_ctx);
    }

    if (p_sar_ctx->session.block_ack & (1u << p_metadata->segmentation.segment_offset))
    {
        /* Segment already received. */
        return;
    }

    p_sar_ctx->session.block_ack |= (1u << p_metadata->segmentation.segment_offset);

    if (p_metadata->segmentation.segment_offset == p_sar_ctx->metadata.segmentation.last_segment)
    {
        /* Last segment might not be the full length of a normal segment, update total length */
        p_sar_ctx->session.length =
            segment_len + (p_sar_ctx->metadata.segmentation.last_segment *
                           TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet));
    }
    else if (segment_len != TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet))
    {
        /* Got a non-conformant segment length, discard the packet. */
        sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_INVALID_FORMAT);
        return;
    }

    memcpy(&p_sar_ctx->payload[p_metadata->segmentation.segment_offset *
                               TRANSPORT_SAR_PDU_LEN(p_metadata->net.control_packet)],
           p_segment_payload,
           segment_len);

    if (p_sar_ctx->session.block_ack == block_ack_full(&p_sar_ctx->metadata))
    {
        /* Release and ack regardless of whether upper layer succeeds */
        p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_PENDING;
        uint32_t ack_status = sar_ack_send(&p_sar_ctx->metadata, p_sar_ctx->session.block_ack);

        /* All packets have arrived */
        upper_transport_packet_in(p_sar_ctx->payload,
                                p_sar_ctx->session.length,
                                &p_sar_ctx->metadata,
                                p_rx_metadata);

        if (ack_status == NRF_SUCCESS)
        {
            p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
            sar_ctx_rx_complete(p_sar_ctx);
        }
    }
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

static uint8_t mic_size_decode(uint8_t encoded_mic_opt_val)
{
    return (encoded_mic_opt_val == NRF_MESH_TRANSMIC_SIZE_SMALL) ?
           PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE:
           PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE;
}

static uint32_t transport_metadata_from_tx_params(transport_packet_metadata_t * p_metadata,
                                              const nrf_mesh_tx_params_t * p_tx_params)
{
    p_metadata->type.access.app_key_id = p_tx_params->security_material.p_app->aid;
    p_metadata->type.access.using_app_key = (!p_tx_params->security_material.p_app->is_device_key);
    p_metadata->p_security_material = p_tx_params->security_material.p_app;
    p_metadata->mic_size = (p_tx_params->transmic_size == NRF_MESH_TRANSMIC_SIZE_DEFAULT) ?
                            mic_size_decode(m_trs_config.szmic) : mic_size_decode(p_tx_params->transmic_size);

    /* Check for a valid MIC size and resultant packet length combination. Large MIC size is not
    allowed for Unsegmented messages. See Table 3.42 of Mesh Profile Specification v1.0. */
    if (p_tx_params->force_segmented == false &&
       (p_tx_params->data_len <= (PACKET_MESH_TRS_UNSEG_ACCESS_PDU_MAX_SIZE - p_metadata->mic_size)) &&
       (p_metadata->mic_size == PACKET_MESH_TRS_TRANSMIC_LARGE_SIZE))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    p_metadata->segmented = ((p_tx_params->data_len > PACKET_MESH_TRS_UNSEG_ACCESS_PDU_MAX_SIZE - p_metadata->mic_size) ||
                             p_tx_params->force_segmented);

    if (p_metadata->segmented)
    {
        uint32_t num_segments = ((p_tx_params->data_len + p_metadata->mic_size +
                                  PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - 1) /
                                 PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE);
        p_metadata->segmentation.last_segment = num_segments - 1;
        p_metadata->segmentation.segment_offset = 0;
    }

    p_metadata->net.control_packet = false;
    p_metadata->net.dst = p_tx_params->dst;
    p_metadata->net.p_security_material = p_tx_params->security_material.p_net;
    p_metadata->net.src = p_tx_params->src;
    p_metadata->net.ttl = p_tx_params->ttl;
    p_metadata->token = p_tx_params->tx_token;

    return NRF_SUCCESS;
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

    p_metadata->token = tx_token;
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

static bool sar_segment_send(trs_sar_ctx_t * p_sar_ctx, uint32_t segment_index)
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

    return (status == NRF_SUCCESS);
}

/**
 * Send SAR segments.
 *
 * @param[in,out] p_sar_ctx SAR context to send segments of.
 *
 * @returns Number of segments sent.
 */
static uint32_t trs_sar_packet_out(trs_sar_ctx_t * p_sar_ctx)
{
    uint32_t sent_segments = 0;
    /* Starts at start_index if, e.g., there was no memory available last round. */
    for (uint8_t i = p_sar_ctx->session.params.tx.start_index;
         i <= p_sar_ctx->metadata.segmentation.last_segment;
         ++i)
    {
        if ((p_sar_ctx->session.block_ack & (1u << i)) == 0)
        {
            /* packet hasn't been acked yet */
            if (sar_segment_send(p_sar_ctx, i))
            {
                sent_segments++;
                /* Set the start index to the next packet, so we know where to pick up in case of failure: */
                p_sar_ctx->session.params.tx.start_index = i + 1;
            }
            else
            {
                break;
            }
        }
    }
    return sent_segments;
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
    trs_sar_ctx_t * p_sar_ctx = NULL;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked); //TODO: Can this be changed to bearer_event_critical_section, or removed all-together?
    for (int i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_INACTIVE)
        {
            p_sar_ctx = &m_trs_sar_sessions[i];
            break;
        }
    }
    _ENABLE_IRQS(was_masked);

    if (p_sar_ctx == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    if (sar_ctx_alloc(p_sar_ctx, p_metadata, TRS_SAR_SESSION_TX, packet_length))
    {
        memcpy(p_sar_ctx->payload, p_payload, payload_len);

        __LOG_XB(LOG_SRC_TRANSPORT,
                LOG_LEVEL_INFO,
                "TX:SAR packet",
                p_sar_ctx->payload,
                p_sar_ctx->session.length);

        (void) trs_sar_packet_out(p_sar_ctx);/* Ignore return, as we'll reset the retry timer regardless. */
        tx_retry_timer_reset(p_sar_ctx);

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
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
            if (trs_sar_packet_out(&m_trs_sar_sessions[i]) != 0)
            {
                tx_retry_timer_reset(&m_trs_sar_sessions[i]);
            }
        }
    }
}

static void trs_sar_rx_process(void)
{
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        if (m_trs_sar_sessions[i].session.session_type == TRS_SAR_SESSION_RX &&
            m_trs_sar_sessions[i].session.params.rx.ack_state == SAR_ACK_STATE_PENDING)
        {
            if (sar_ack_send(&m_trs_sar_sessions[i].metadata, m_trs_sar_sessions[i].session.block_ack) == NRF_SUCCESS)
            {
                m_trs_sar_sessions[i].session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
                if (m_trs_sar_sessions[i].session.block_ack == block_ack_full(&m_trs_sar_sessions[i].metadata))
                {
                    sar_ctx_rx_complete(&m_trs_sar_sessions[i]);
                }
            }
        }
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
            seqauth_sequence_number_get(p_metadata->net.internal.sequence_number,
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

    if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        ccm_data.a_len = NRF_MESH_UUID_SIZE;
    }

    if (p_metadata->type.access.using_app_key)
    {
        do {
            if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
            {
                ccm_data.p_a = p_metadata->net.dst.p_virtual_uuid;
            }

            /* Application key */
            p_metadata->p_security_material = NULL;
            for (nrf_mesh_app_secmat_next_get(p_metadata->net.p_security_material,
                                              p_metadata->type.access.app_key_id,
                                              &p_metadata->p_security_material);
                 p_metadata->p_security_material != NULL;
                 nrf_mesh_app_secmat_next_get(p_metadata->net.p_security_material,
                                              p_metadata->type.access.app_key_id,
                                              &p_metadata->p_security_material))
            {
                if (test_transport_decrypt(p_metadata->p_security_material, &ccm_data))
                {
                    return NRF_SUCCESS;
                }
            }
        } while (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL &&
                 nrf_mesh_rx_address_get(p_metadata->net.dst.value, &p_metadata->net.dst));
    }
    else
    {
        /* Device key */
        p_metadata->p_security_material = NULL;
        if (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            nrf_mesh_devkey_secmat_get(p_metadata->net.dst.value, &p_metadata->p_security_material);
            if (test_transport_decrypt(p_metadata->p_security_material, &ccm_data))
            {
                return NRF_SUCCESS;
            }
        }

        /* try the src address */
        p_metadata->p_security_material = NULL;
        nrf_mesh_devkey_secmat_get(p_metadata->net.src, &p_metadata->p_security_material);
        if (test_transport_decrypt(p_metadata->p_security_material, &ccm_data))
        {
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NOT_FOUND;
}

static void transport_metadata_build(const packet_mesh_trs_packet_t * p_transport_packet,
                                     const network_packet_metadata_t * p_net_metadata,
                                     transport_packet_metadata_t * p_trs_metadata)
{
    memcpy(&p_trs_metadata->net, p_net_metadata, sizeof(network_packet_metadata_t));
    p_trs_metadata->segmented = packet_mesh_trs_common_seg_get(p_transport_packet);

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

    trs_sar_ctx_t * p_sar_ctx = sar_active_tx_ctx_get(p_metadata, seq_zero);
    if (p_sar_ctx == NULL)
    {
        __LOG(LOG_SRC_TRANSPORT, LOG_LEVEL_WARN, "Could not find active session for ACK packet\n");
    }
    else
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_TRS_ACK_RECEIVED, 0, control_packet_len, p_trs_control_packet);

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
            /* Mesh Profile Specification v1.0, section 3.5.3.3: "If a Segment Acknowledgment message is received that
             * is a valid acknowledgment for the segmented message, then the lower transport layer
             * shall reset the segment transmission timer and retransmit all unacknowledged Lower
             * Transport PDUs." */
             p_sar_ctx->session.params.tx.start_index = 0;
             (void) trs_sar_packet_out(p_sar_ctx); /* Ignore return, as we'll reset the retry timer regardless. */
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
                control_packet.opcode       = p_metadata->type.control.opcode;
                control_packet.p_net_secmat = p_metadata->net.p_security_material;
                control_packet.src          = p_metadata->net.src;
                control_packet.dst          = p_metadata->net.dst;
                control_packet.ttl          = p_metadata->net.ttl;
                control_packet.p_data       = p_trs_control_packet;
                control_packet.data_len     = control_packet_len;
                control_packet.reliable     = p_metadata->segmented;
                callback(&control_packet, p_rx_metadata);
            }
            break;
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

static void ack_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_RX);
    p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_PENDING;
    if (sar_ack_send(&p_sar_ctx->metadata, p_sar_ctx->session.block_ack) == NRF_SUCCESS)
    {
        p_sar_ctx->session.params.rx.ack_state = SAR_ACK_STATE_IDLE;
    }
}

static void retry_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    NRF_MESH_ASSERT(p_sar_ctx->session.session_type == TRS_SAR_SESSION_TX);
    if (p_sar_ctx->session.params.tx.retries == 0)
    {
        /* We're out of retries. Should give up. */
        if (p_sar_ctx->metadata.net.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_TIMEOUT);
        }
        else
        {
            /* For non-unicast addresses, timing out is considered a successful way to end, as there
             * are no acknowledgements. */
            sar_ctx_tx_complete(p_sar_ctx);
        }
    }
    else
    {
        p_sar_ctx->session.params.tx.retries--;
        p_sar_ctx->session.params.tx.start_index = 0;
        (void) trs_sar_packet_out(p_sar_ctx);/* Ignore return, as the timer will be rescheduled regardless. */
    }
}

static void abort_timeout(timestamp_t timestamp, void * p_context)
{
    trs_sar_ctx_t * p_sar_ctx = p_context;
    sar_ctx_cancel(p_sar_ctx, NRF_MESH_SAR_CANCEL_REASON_TIMEOUT);
}

static void tx_complete(core_tx_role_t role, uint32_t bearer_index, uint32_t timestamp, nrf_mesh_tx_token_t token)
{
    if (role == CORE_TX_ROLE_ORIGINATOR && token != SAR_TOKEN)
    {
        /* This tx complete came from the application. */
        nrf_mesh_evt_t evt;
        evt.type = NRF_MESH_EVT_TX_COMPLETE;
        evt.params.tx_complete.token = token;
        event_handle(&evt);
    }
    bearer_event_flag_set(m_sar_process_flag);
}

static uint32_t upper_transport_tx(transport_packet_metadata_t * p_metadata, const uint8_t * p_data, uint32_t data_len)
{
    if (nrf_mesh_address_type_get(p_metadata->net.src) != NRF_MESH_ADDRESS_TYPE_UNICAST ||
        p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_INVALID ||
        (p_metadata->net.dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL && p_metadata->net.dst.p_virtual_uuid == NULL))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (p_metadata->net.ttl > NRF_MESH_TTL_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (p_metadata->segmented)
    {
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
void transport_init(const nrf_mesh_init_params_t * p_init_params)
{
    transport_sar_mem_funcs_reset();

    memset(&m_trs_sar_sessions[0], 0, sizeof(m_trs_sar_sessions));

    replay_cache_init();

    m_sar_session_cache_head = 0;
    memset(m_sar_session_cache, 0, sizeof(m_sar_session_cache));

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

}

uint32_t transport_sar_mem_funcs_set(transport_sar_alloc_t alloc_func, transport_sar_release_t release_func)
{
    if ((alloc_func == NULL) != (release_func == NULL)) /*lint !e731 Boolean arguments to equal/not equal operator */
    {
        /* Both functions must be NULL or non-NULL, but not a mix of both. */
        return NRF_ERROR_NULL;
    }
    else if (alloc_func == NULL && release_func == NULL)
    {
        transport_sar_mem_funcs_reset();
        return NRF_SUCCESS;
    }
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    m_sar_alloc = alloc_func;
    m_sar_release = release_func;
    _ENABLE_IRQS(was_masked);
    return NRF_SUCCESS;
}

void transport_sar_mem_funcs_reset(void)
{
    NRF_MESH_ERROR_CHECK(transport_sar_mem_funcs_set(malloc, free));
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

    /* nrf_mesh_rx_address_get requires a clean address structure on the first call. */
    nrf_mesh_address_t dst_addr;
    memset(&dst_addr, 0, sizeof(nrf_mesh_address_t));

    if (!nrf_mesh_rx_address_get(p_net_metadata->dst.value, &dst_addr))
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                              PACKET_DROPPED_UNKNOWN_ADDRESS,
                              trs_packet_len,
                              p_packet);
        return status;
    }

    if (replay_cache_has_elem(p_net_metadata->src,
                              p_net_metadata->internal.sequence_number,
                              p_net_metadata->internal.iv_index & NETWORK_IVI_MASK))
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_PACKET_DROPPED,
                              PACKET_DROPPED_REPLAY_CACHE,
                              trs_packet_len,
                              p_packet);
        return status;
    }

    status = replay_cache_add(p_net_metadata->src,
                              p_net_metadata->internal.sequence_number,
                              p_net_metadata->internal.iv_index & NETWORK_IVI_MASK);
    if (status != NRF_SUCCESS)
    {
        m_send_replay_cache_full_event(p_net_metadata->src,
                                       p_net_metadata->internal.iv_index & NETWORK_IVI_MASK,
                                       NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
        return status;
    }

    transport_packet_metadata_t trs_metadata;
    transport_metadata_build(p_packet, p_net_metadata, &trs_metadata);
    /* Insert the lookup-address in case it's different from the one in p_net_metadata. */
    trs_metadata.net.dst = dst_addr;

    /* The packet handler functions operate on static global state, that potentially can be accessed
     * from timers. */
    bearer_event_critical_section_begin();

    if (trs_metadata.segmented)
    {
        trs_sar_seg_packet_in(packet_mesh_trs_seg_payload_get(p_packet),
                              trs_packet_len - PACKET_MESH_TRS_SEG_PDU_OFFSET,
                              &trs_metadata,
                              p_rx_metadata);
    }
    else
    {
        upper_transport_packet_in(packet_mesh_trs_unseg_payload_get(p_packet),
                                  trs_packet_len - PACKET_MESH_TRS_UNSEG_PDU_OFFSET,
                                  &trs_metadata,
                                  p_rx_metadata);
    }

    bearer_event_critical_section_end();

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
    uint32_t status = transport_metadata_from_tx_params(&metadata, p_params);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

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
    if (p_params->dst.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL ||
        p_params->dst.type == NRF_MESH_ADDRESS_TYPE_INVALID)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if ((uint8_t) p_params->opcode > TRANSPORT_CONTROL_PACKET_OPCODE_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    transport_packet_metadata_t metadata;
    transport_metadata_from_control_tx_params(&metadata, p_params, tx_token);

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
            m_trs_config.szmic = p_opt->opt.val;
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
