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

#include <stddef.h>
#include <string.h>

#include "nrf_mesh_prov_bearer.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "prov_bearer_adv.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "ad_listener.h"

#if MESH_FEATURE_PB_ADV_ENABLED

#include "log.h"
#include "packet.h"
#include "rand.h"
#include "timer_scheduler.h"
#include "toolchain.h"
#include "utils.h"
#include "provisioning.h"
#include "prov_beacon.h"
#include "prov_pdu.h"
#include "bearer_event.h"
#include "advertiser.h"
#if MESH_FEATURE_LPN_ENABLED
#include "scanner.h"
#endif

#if defined NRF_MESH_TEST_SHIM
#include "test_instrument.h"
#endif


/*****************************************************************************
* Local defines
*****************************************************************************/

/**
 * @defgroup PB_ADV_PACKET_LENGTHS
 * Various lengths and overheads in the PB-ADV packets
 * @{
 */

/** PB-ADV packet overhead */
#define PB_ADV_PACKET_OVERHEAD                                  (4 /* Link ID */ + 1 /* Transaction number */)
/** Overhead before payload of a transaction start packet. */
#define PROV_BEARER_PACKET_TRANSACTION_START_OVERHEAD           (1 /* SegN+GPCF */ + 2 /* Total len */ + 1 /* FCS */)
/** Length of a transaction ack packet. */
#define PROV_BEARER_PACKET_TRANSACTION_ACK_LEN                  (1 /* SegN+GPCF */)
/** Overhead before payload of a transaction continuation packet. */
#define PROV_BEARER_PACKET_TRANSACTION_CONTINUATION_OVERHEAD    (1 /* SegN+GPCF */)
/** Length of a link open packet. */
#define PROV_BEARER_PACKET_LINK_OPEN_LEN                        (1 /* Control ID+GPCF */ + NRF_MESH_UUID_SIZE)
/** Length of a link ack packet. */
#define PROV_BEARER_PACKET_LINK_ACK_LEN                         (1 /* Control ID+GPCF */)
/** Length of a link close packet. */
#define PROV_BEARER_PACKET_LINK_CLOSE_LEN                       (1 /* Control ID+GPCF */ + 1 /* Reason */)

/** Maximum payload length of a transaction start packet */
#define PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN             (BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH - \
        (sizeof(ble_ad_data_t) + PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_TRANSACTION_START_OVERHEAD))
/** Maximum payload length of a transaction continuation packet */
#define PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN      (BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH - \
        (sizeof(ble_ad_data_t) + PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_TRANSACTION_CONTINUATION_OVERHEAD))

/* Assert payload sizes against @tagMeshSp tables 5.2, 5.4, 5.5, and 5.7 */
NRF_MESH_STATIC_ASSERT(PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN == 20);
NRF_MESH_STATIC_ASSERT(PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN == 23);

/** @} */

/** Link close TX retry interval. */
#define PROV_BEARER_ADV_LINK_CLOSE_RETRY_INTERVAL_US            ( 1000000)
/** Time before a transaction is given up. */
#define PROV_BEARER_ADV_TRANSACTION_TIMEOUT_US                  (30000000)
/** Transaction TX retry interval. */
#define PROV_BEARER_ADV_TRANSACTION_BASE_RETRY_INTERVAL_US      ( 2000000)
/** Number of repeats when sending a packet for unacked LINK_CLOSE. */
#define PROV_BEARER_ADV_UNACKED_REPEAT_COUNT                    (6)
/** Advertiser interval for link open sending.
 *  Interval is stretched to avoid collision between the next link open and ack on the previous one. */
#define PROV_BEARER_ADV_LINK_OPEN_ADVERTISER_INTERVAL_MS        (3 * BEARER_ADV_INT_DEFAULT_MS)
/** Length of the link establishment procedure. @tagMeshSp section 5.3.2.
 *  To open a link, the Provisioner shall start the link establishment timer, set to 60 seconds. */
#define PROV_BEARER_ADV_LINK_ESTABLISHMENT_LENGTH_US            SEC_TO_US(60)

/**
 * @defgroup PB_ADV_PACKET_CONTROL_FIELD_VALUES
 * Recognized control field values in the PB-ADV packets.
 * @{
 */

/** Control field value for start packets. */
#define PB_ADV_PACKET_C_TRANSACTION_START       (0)
/** Control field value for transaction ack packets. */
#define PB_ADV_PACKET_C_TRANSACTION_ACK         (1)
/** Control field value for transaction continue packets. */
#define PB_ADV_PACKET_C_TRANSACTION_CONTINUE    (2)
/** Control field value for control packets. */
#define PB_ADV_PACKET_C_CONTROL                 (3)

/** @} */

/**
 * @defgroup PB_ADV_PACKET_CONTROL_ID_VALUES
 * Recognized control packet values
 * @{
 */

/** PB-ADV control id for the link open request packet. */
#define PB_ADV_PACKET_CONTROL_ID_LINK_OPEN      (0x00)
/** PB-ADV control id for the link acknowledgment packet. */
#define PB_ADV_PACKET_CONTROL_ID_LINK_ACK       (0x01)
/** PB-ADV control id for the link close indication packet. */
#define PB_ADV_PACKET_CONTROL_ID_LINK_CLOSE     (0x02)

/** @} */

/** TX token used to identify an unprov beacon packet. */
#define TX_TOKEN_UNPROV_BEACON      ((nrf_mesh_tx_token_t) 0xFFFFFFFF)

#define PB_ADV_TRANSACTION_NUMBER_PROVISIONER_START  (0x00) /**< Initial value of provisioner transaction number. */
#define PB_ADV_TRANSACTION_NUMBER_PROVISIONEE_START  (0x80) /**< Initial value of provisionee transaction number. */
#define PB_ADV_TRANSACTION_NUMBER_ROLLOVER_MASK      (0x7f) /**< Mask applied when rolling over transaction numbers. */
/*****************************************************************************
* Local type declarations
*****************************************************************************/
/*lint -align_max(push) -align_max(1) */

/** Payload of a generic control packet. */
typedef union __attribute((packed))
{
    struct __attribute((packed))
    {
        uint8_t uuid[NRF_MESH_UUID_SIZE]; /**< Advertised device UUID of device to open link to. */
    } link_open;
    struct __attribute((packed))
    {
        uint8_t reason; /**< Reason for closing the link. See @ref nrf_mesh_prov_link_close_reason_t for valid values. */
    } link_close;
} prov_generic_control_t;

/** Payload of a generic transaction packet. */
typedef union __attribute((packed))
{
    struct __attribute((packed))
    {
        uint16_t total_length;                                         /**< Total length of the provisioning PDU. */
        uint8_t  fcs;                                                  /**< Frame Check Sequence, 8 bit CRC for the complete packet. */
        uint8_t  payload[PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN]; /**< Packet payload. */
    } start;
    struct __attribute((packed))
    {
        uint8_t  payload[PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN];     /**< Packet payload. */
    } continuation;
} prov_generic_transaction_t;

/** Generic provisioning PDU, as defined in @tagMeshSp. */
typedef struct __attribute((packed))
{
    uint8_t control : 2; /**< Control field for packet type, see @ref PB_ADV_PACKET_CONTROL_FIELD_VALUES for recognized values. */
    uint8_t id      : 6; /**< Packet ID field, holds one of @ref PB_ADV_PACKET_CONTROL_ID_VALUES for control packets, segment number for transaction packets. */
    union __attribute((packed))
    {
        prov_generic_control_t      control;     /**< Payload of a control PDU. */
        prov_generic_transaction_t  transaction; /**< Payload of a transaction PDU. */
    } payload;
} prov_generic_pdu_t;

typedef struct __attribute((packed))
{
    uint32_t link_id;
    uint8_t transaction_number;
    prov_generic_pdu_t pdu;
} pb_adv_pdu_t;

NRF_MESH_STATIC_ASSERT(PROV_BEARER_ADV_UNACKED_REPEAT_COUNT >= 3);

/*lint -align_max(pop) */
/*****************************************************************************
* Static globals
*****************************************************************************/
static nrf_mesh_prov_bearer_adv_t * mp_bearer_head; /**< Head of adv bearer linked list */
static bearer_event_flag_t m_async_process_flag = BEARER_EVENT_FLAG_INVALID; /**< Flag to enable asynchronous processing. */
/*****************************************************************************
* Static functions
*****************************************************************************/
static void tx_retry_cb(timestamp_t timestamp, void * p_context);
static void close_link(nrf_mesh_prov_bearer_adv_t * p_pb_adv, nrf_mesh_prov_link_close_reason_t reason);
static void tx_complete_cb(advertiser_t * p_adv, nrf_mesh_tx_token_t token, timestamp_t timestamp);

static void prov_bearer_adv_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);

/**
 * Calculates the FCS CRC based on 3GPP TS 27.010, as required by @tagMeshSp section 5.3.1.1.
 *
 * @note The function is not static but it's not public either, it should only be
 *       accessed by the unit test.
 *
 * @param p_input Pointer to the start of the input data.
 * @param size Size of the input data.
 * @returns The CRC of the input data.
 */
uint8_t calculate_3GPP_CRC(const uint8_t * p_input, uint16_t size)
{
    /* CRC lookup table lifted from the 3GPP TS27.010 specification, used for FCS generation. */
    static const uint8_t crctable[256] = {
        0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
        0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
        0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
        0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
        0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
        0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
        0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
        0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
        0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
        0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
        0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
        0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
        0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
        0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
        0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
        0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
    };

    uint8_t fcs = 0xff;

    for (int i = 0; i < size; ++i)
    {
        fcs = crctable[fcs ^ p_input[i]];
    }

    return 0xff - fcs;
}

/** Reset/postpone the link timeout timer. */
static inline void reset_timeout_timer(nrf_mesh_prov_bearer_adv_t * p_bearer)
{
    timer_sch_reschedule(&p_bearer->link_timeout_event, timer_now() + p_bearer->link_timeout);
}

/** Link timeout timer call back, called by the timer_scheduler modeule.
  * This will be called if a PDU is not received before the timeout expires,
  * then the protocol has failed.
  */
static void link_timeout_cb(timestamp_t timestamp, void * p_context)
{
    prov_bearer_t * p_bearer =
        &((nrf_mesh_prov_bearer_adv_t *) p_context)->prov_bearer;
    p_bearer->p_interface->link_close(p_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT);
}

static nrf_mesh_prov_bearer_adv_t * get_bearer_from_state(prov_bearer_adv_state_t state)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = mp_bearer_head;
    while (p_pb_adv != NULL)
    {
        if (p_pb_adv->state == state)
        {
            break;
        }
        p_pb_adv = p_pb_adv->p_next;
    }
    _ENABLE_IRQS(was_masked);
    return p_pb_adv;
}

static nrf_mesh_prov_bearer_adv_t * get_bearer_from_link_id(uint32_t link_id)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = mp_bearer_head;
    while (p_pb_adv != NULL)
    {
        if (p_pb_adv->link_id == link_id)
        {
            break;
        }
        p_pb_adv = p_pb_adv->p_next;
    }
    _ENABLE_IRQS(was_masked);
    return p_pb_adv;
}

static inline uint8_t transaction_total_segment_count_get(uint32_t total_length)
{
    return 1 +
        (total_length
         - 1
         - PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN
         + PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN)
        / PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN;
}

/**
 * According to @tagMeshSp section 5.2.1, transaction numbers are
 * split into two domains at 0x80, and transaction numbers that are 0x7f should roll over to
 * 0, while transaction numbers that are at 0xff should roll over to 0x80.
 */
static inline uint8_t transaction_number_increment(uint8_t transaction_number)
{
    return (transaction_number & ~PB_ADV_TRANSACTION_NUMBER_ROLLOVER_MASK) |
           ((transaction_number + 1) & PB_ADV_TRANSACTION_NUMBER_ROLLOVER_MASK);
}

static inline void reset_adv_int(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    advertiser_interval_set(&p_pb_adv->advertiser, BEARER_ADV_INT_DEFAULT_MS);
}

static void init_bearer_structure(nrf_mesh_prov_bearer_adv_t * p_pb_adv, uint32_t link_timeout_us)
{
    p_pb_adv->link_timeout = link_timeout_us;
    p_pb_adv->link_timeout_event.interval = 0;

    p_pb_adv->buffer.state = PROV_BEARER_ADV_BUF_STATE_UNUSED;
    p_pb_adv->transaction_out = 0;
    p_pb_adv->transaction_in = 0;
    p_pb_adv->state = PROV_BEARER_ADV_STATE_IDLE;
    p_pb_adv->last_token = NRF_MESH_INITIAL_TOKEN;

    /* If the advertiser is already initialized once, we do not re-initialize it.
     * Re-initializing the advertiser will re-initialize the packet buffer as well and this will
     * cause an assert (advertiser.c:169) in the case where the buffer has been flushed, but there
     * is one packet popped for transmitting.
     */
    if (p_pb_adv->instance_state != PROV_BEARER_ADV_INSTANCE_INITIALIZED)
    {
        p_pb_adv->instance_state = PROV_BEARER_ADV_INSTANCE_INITIALIZED;

        p_pb_adv->timeout_event.cb = tx_retry_cb;
        p_pb_adv->timeout_event.p_context = p_pb_adv;

        p_pb_adv->link_timeout_event.cb = link_timeout_cb;
        p_pb_adv->link_timeout_event.p_context = p_pb_adv;

        advertiser_instance_init(&p_pb_adv->advertiser,
                                 tx_complete_cb,
                                 p_pb_adv->tx_buffer,
                                 NRF_MESH_PROV_BEARER_ADV_TX_BUFFER_SIZE);
    }

#if defined NRF_MESH_TEST_SHIM
    nrf_mesh_test_shim(EDIT_PROV_BEARER_ADV_ADDR, &p_pb_adv->advertiser.config.adv_addr);
#endif
}

/**** Link list managment ****/

/**
 * Add a bearer instance to the active bearer list.
 *
 */
static void add_active_bearer(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    p_pb_adv->p_next = mp_bearer_head;
    mp_bearer_head = p_pb_adv;

    _ENABLE_IRQS(was_masked);

    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "PB-ADV: context at 0x%.08x added to bearer\n", (uint32_t) p_pb_adv);
}
/**
 * Remove bearer instance from the active bearer list.
 *
 * @warning This function requires that the bearer instance p_pb_adv it not NULL and that
 *          it has already been added to the active list via @ref add_active_bearer.
 *
 */
static void remove_active_bearer(const nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    NRF_MESH_ASSERT(mp_bearer_head != NULL);
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (p_pb_adv == mp_bearer_head)
    {
        mp_bearer_head = p_pb_adv->p_next;
    }
    else
    {
        nrf_mesh_prov_bearer_adv_t * p_prev = mp_bearer_head;
        while (p_prev->p_next != p_pb_adv)
        {
            /* This should never happen, otherwise it indicates an internal error. */
            NRF_MESH_ASSERT(p_prev->p_next != NULL);
            p_prev = p_prev->p_next;
        }

        p_prev->p_next = p_pb_adv->p_next;
    }

    _ENABLE_IRQS(was_masked);
}

/**** Packet sending ****/

static void send_unprov_beacon(nrf_mesh_prov_bearer_adv_t * p_pb_adv, const char * URI, uint16_t oob_info)
{
    adv_packet_t * p_packet = prov_beacon_unprov_build(&p_pb_adv->advertiser, URI, oob_info);
    /* Since this is the only thing we're doing with the advertiser at this point, we should never
     * fail to allocate the packet. */
    NRF_MESH_ASSERT(p_packet != NULL);
    p_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
    p_packet->token = TX_TOKEN_UNPROV_BEACON;
    advertiser_interval_set(&p_pb_adv->advertiser, NRF_MESH_PROV_BEARER_ADV_UNPROV_BEACON_INTERVAL_MS);

    advertiser_packet_send(&p_pb_adv->advertiser, p_packet);
}

static adv_packet_t * pb_adv_packet_alloc(nrf_mesh_prov_bearer_adv_t * p_pb_adv, uint32_t payload_size)
{
    return advertiser_packet_alloc(&p_pb_adv->advertiser, sizeof(ble_ad_data_t) + payload_size);
}

static inline nrf_mesh_tx_token_t tx_token_alloc(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    nrf_mesh_tx_token_t token = ++p_pb_adv->last_token;
    NRF_MESH_ASSERT(token != TX_TOKEN_UNPROV_BEACON);

    _ENABLE_IRQS(was_masked);

    return token;
}

static uint32_t send_packet(nrf_mesh_prov_bearer_adv_t * p_pb_adv, const pb_adv_pdu_t * p_pdu, uint32_t length, uint8_t repeats)
{
    adv_packet_t * p_packet = pb_adv_packet_alloc(p_pb_adv, length);
    if (p_packet == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    /* Build packet: */
    ble_ad_data_t * p_ad_data = (ble_ad_data_t *) &p_packet->packet.payload[0];
    p_ad_data->length = length + BLE_AD_DATA_OVERHEAD;
    p_ad_data->type = AD_TYPE_PB_ADV;
    memcpy(&p_ad_data->data[0], p_pdu, length);

    p_packet->config.repeats = repeats;
    p_packet->token = tx_token_alloc(p_pb_adv);

    advertiser_packet_send(&p_pb_adv->advertiser, p_packet);

    return NRF_SUCCESS;
}

static uint32_t send_link_open(nrf_mesh_prov_bearer_adv_t * p_pb_adv, const uint8_t * p_peer_uuid)
{
    pb_adv_pdu_t pb_adv_pdu;
    pb_adv_pdu.link_id = LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = 0;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_CONTROL;

    pb_adv_pdu.pdu.id = PB_ADV_PACKET_CONTROL_ID_LINK_OPEN;
    memcpy(pb_adv_pdu.pdu.payload.control.link_open.uuid, p_peer_uuid, NRF_MESH_UUID_SIZE);

    return send_packet(p_pb_adv,
            &pb_adv_pdu,
            PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_OPEN_LEN,
            ADVERTISER_REPEAT_INFINITE);
}

static uint32_t send_link_ack(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    pb_adv_pdu_t pb_adv_pdu;
    pb_adv_pdu.link_id = LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = 0;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_CONTROL;

    pb_adv_pdu.pdu.id = PB_ADV_PACKET_CONTROL_ID_LINK_ACK;

    return send_packet(p_pb_adv, &pb_adv_pdu, PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_ACK_LEN, 1);
}

static uint32_t send_link_close(nrf_mesh_prov_bearer_adv_t * p_pb_adv, nrf_mesh_prov_link_close_reason_t reason)
{
    pb_adv_pdu_t pb_adv_pdu;
    pb_adv_pdu.link_id = LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = 0;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_CONTROL;

    pb_adv_pdu.pdu.id = PB_ADV_PACKET_CONTROL_ID_LINK_CLOSE;
    pb_adv_pdu.pdu.payload.control.link_close.reason = (uint8_t) reason;

    /* The spec recommends sending the close message at least three times to be sure it's received.*/
    return send_packet(p_pb_adv, &pb_adv_pdu, PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_CLOSE_LEN, PROV_BEARER_ADV_UNACKED_REPEAT_COUNT);
}

static void send_transaction_ack(nrf_mesh_prov_bearer_adv_t * p_pb_adv, uint8_t transaction_number)
{
    pb_adv_pdu_t pb_adv_pdu;
    pb_adv_pdu.link_id = LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = transaction_number;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_TRANSACTION_ACK;
    pb_adv_pdu.pdu.id = 0;

    (void) send_packet(p_pb_adv, &pb_adv_pdu, PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_TRANSACTION_ACK_LEN, 1);
}

static uint32_t send_transaction_start(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    NRF_MESH_ASSERT(p_pb_adv != NULL);
    NRF_MESH_ASSERT(p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN);
    NRF_MESH_ASSERT(p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX);

    uint16_t payload_length;
    if (p_pb_adv->buffer.length > PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN)
    {
        payload_length = PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN;
    }
    else
    {
        payload_length = p_pb_adv->buffer.length;
    }

    pb_adv_pdu_t pb_adv_pdu;

    pb_adv_pdu.link_id = LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = p_pb_adv->transaction_out;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_TRANSACTION_START;
    /* Set the ID field to match the segment number of the last packet we'll
     * send in this transaction. */
    pb_adv_pdu.pdu.id = transaction_total_segment_count_get(p_pb_adv->buffer.length) - 1;
    pb_adv_pdu.pdu.payload.transaction.start.fcs = p_pb_adv->buffer.fcs;
    pb_adv_pdu.pdu.payload.transaction.start.total_length = LE2BE16(p_pb_adv->buffer.length); /*lint !e572 Excessive shift of value. */
    memcpy(pb_adv_pdu.pdu.payload.transaction.start.payload, &p_pb_adv->buffer.payload[0], payload_length);

    uint32_t status = send_packet(p_pb_adv, &pb_adv_pdu, PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_TRANSACTION_START_OVERHEAD + payload_length, 1);
    if (status == NRF_SUCCESS)
    {
        p_pb_adv->buffer.finished_segments = 1;
    }
    return status;
}

static uint32_t send_transaction_continuation(nrf_mesh_prov_bearer_adv_t * p_pb_adv, uint8_t segment_index)
{
    const uint16_t data_offset = PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN + ((segment_index - 1) * PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN);
    const uint16_t remaining = p_pb_adv->buffer.length - data_offset;

    uint16_t payload_length;
    if (remaining > PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN)
    {
        payload_length = PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN;
    }
    else
    {
        payload_length = remaining;
    }

    pb_adv_pdu_t pb_adv_pdu;

    pb_adv_pdu.link_id =LE2BE32(p_pb_adv->link_id);
    pb_adv_pdu.transaction_number = p_pb_adv->transaction_out;
    pb_adv_pdu.pdu.control = PB_ADV_PACKET_C_TRANSACTION_CONTINUE;
    pb_adv_pdu.pdu.id = segment_index;
    memcpy(pb_adv_pdu.pdu.payload.transaction.continuation.payload, &p_pb_adv->buffer.payload[data_offset], payload_length);

    uint32_t status = send_packet(p_pb_adv, &pb_adv_pdu, PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_TRANSACTION_CONTINUATION_OVERHEAD + payload_length, 1);
    if (status == NRF_SUCCESS)
    {
        p_pb_adv->buffer.finished_segments++;
    }
    return status;
}

/**** Transaction handling ****/

static void prov_buffer_tx(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    NRF_MESH_ASSERT(p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX);
    uint32_t status;
    /* Only send start packet if we haven't sent any segments yet. */
    if (p_pb_adv->buffer.finished_segments == 0)
    {
        status = send_transaction_start(p_pb_adv);
        if (status != NRF_SUCCESS)
        {
            /* Wait for advertiser queue empty or retry timeout */
            return;
        }
    }
    /* The loop should be able to pick up in the middle of sending. */
    for (uint32_t segment = p_pb_adv->buffer.finished_segments;
            segment < transaction_total_segment_count_get(p_pb_adv->buffer.length);
            segment++)
    {
        status = send_transaction_continuation(p_pb_adv, segment);
        if (status != NRF_SUCCESS)
        {
            /* Wait for advertiser queue empty or retry timeout */
            break;
        }
    }
}

static void prov_buffer_rx(nrf_mesh_prov_bearer_adv_t * p_pb_adv, prov_bearer_adv_buffer_t * p_buf)
{
    NRF_MESH_ASSERT(p_buf->state == PROV_BEARER_ADV_BUF_STATE_RX);

    /* Verify FCS */
    if (p_buf->fcs == calculate_3GPP_CRC(p_buf->payload, p_buf->length))
    {
        /* Send ack, doesn't really matter if it works, the other device side
         * will eventually send again if we fail. */
        send_transaction_ack(p_pb_adv, p_pb_adv->transaction_in);

        /* Prepare for next transaction */
        p_buf->finished_segments = 0;
        p_pb_adv->transaction_in = transaction_number_increment(p_pb_adv->transaction_in);

        /* Copy the buffer to stack, to avoid having a TX call in the
         * callback overwrite the RX data. This could probably be avoided
         * by ensuring that we never look at the incoming data after
         * calling TX in the upper layers, but we can't guarantee this from
         * this module.
         */
        uint32_t length = p_buf->length;
        uint8_t prov_buf[PROV_PDU_MAX_LENGTH];
        memcpy(prov_buf, p_buf->payload, length);
        p_buf->state = PROV_BEARER_ADV_BUF_STATE_UNUSED;

        reset_timeout_timer(p_pb_adv);

        /* Callback to upper layer */
        p_pb_adv->prov_bearer.p_callbacks->rx(&p_pb_adv->prov_bearer, prov_buf, length);
    }
    else
    {
        /* Received with wrong FCS, abort. */
        p_buf->state = PROV_BEARER_ADV_BUF_STATE_UNUSED;
    }
}

/**** Packet handling ****/

static void handle_transaction_start_packet(nrf_mesh_prov_bearer_adv_t * p_pb_adv, pb_adv_pdu_t * p_packet, uint32_t length)
{
    if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_UNUSED)
    {
        if (p_packet->transaction_number == p_pb_adv->transaction_in)
        {
            /* finished_segments, which is used in handle_transaction_continuation_packet
                is 8-bits hence we are limited to receiving a maximum of 7 segments. */
            p_pb_adv->buffer.length = BE2LE16(p_packet->pdu.payload.transaction.start.total_length);
            uint8_t SegN = transaction_total_segment_count_get(p_pb_adv->buffer.length);
            if (SegN > (sizeof(p_pb_adv->buffer.finished_segments)*8) -1)
            {
                prov_bearer_adv_link_close(&p_pb_adv->prov_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
            }
            else
            {
                /* New message */
                p_pb_adv->buffer.fcs = p_packet->pdu.payload.transaction.start.fcs;
                p_pb_adv->buffer.state = PROV_BEARER_ADV_BUF_STATE_RX;
                p_pb_adv->buffer.finished_segments = 1;
                uint32_t payload_length = length - PB_ADV_PACKET_OVERHEAD - PROV_BEARER_PACKET_TRANSACTION_START_OVERHEAD;

                if (payload_length > PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN)
                {
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got malformed packet.\n");
                    return;
                }

                memcpy(p_pb_adv->buffer.payload, p_packet->pdu.payload.transaction.start.payload, payload_length);

                if (p_pb_adv->buffer.length == payload_length)
                {
                    /* Single segment message */
                    prov_buffer_rx(p_pb_adv, &p_pb_adv->buffer);
                }
            }
        }
        else if (p_packet->transaction_number < p_pb_adv->transaction_in)
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_DBG1, "Got start packet for a transaction we've already processed, send the ack again.");
            send_transaction_ack(p_pb_adv, p_packet->transaction_number);
        }
        else
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got segment from unexpected transaction number (was %x, previous is %x).\n", p_packet->transaction_number, p_pb_adv->transaction_in);
        }
    }
    else if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX)
    {
        if (p_packet->transaction_number < p_pb_adv->transaction_in)
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_DBG1, "Got start packet for a transaction we've already processed, send the ack again.");
            send_transaction_ack(p_pb_adv, p_packet->transaction_number);
        }
        else if (p_packet->transaction_number > p_pb_adv->transaction_in)
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got segment from unexpected transaction number (was %x, previous is %x).\n", p_packet->transaction_number, p_pb_adv->transaction_in);
        }
        else
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_DBG1, "Got the next segment before an ACK for our own (%x, %x)\n", p_packet->transaction_number, p_pb_adv->transaction_in);
        }
    }
    else
    {
        __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Received start segment while already in RX mode.\n");
    }
}

static void handle_transaction_continuation_packet(nrf_mesh_prov_bearer_adv_t * p_pb_adv, pb_adv_pdu_t * p_packet, uint32_t length)
{
    if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_RX)
    {
        if (p_packet->transaction_number != p_pb_adv->transaction_in)
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got segment from unexpected transaction number.\n");
            return;
        }

        /* Check segment bitfield, to figure out if we've received this packet before: */
        if (!((1 << p_packet->pdu.id) & p_pb_adv->buffer.finished_segments))
        {
            /* First time we receive this packet. */
            uint32_t data_index = (PROV_BEARER_ADV_PACKET_START_PAYLOAD_MAXLEN + (p_packet->pdu.id - 1) * PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN);
            uint32_t payload_length = length - PB_ADV_PACKET_OVERHEAD - PROV_BEARER_PACKET_TRANSACTION_CONTINUATION_OVERHEAD;

            if (payload_length > PROV_BEARER_ADV_PACKET_CONTINUATION_PAYLOAD_MAXLEN ||
                data_index + payload_length > PROV_PDU_MAX_LENGTH)
            {
                __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got malformed packet.\n");
                return;
            }

            memcpy(&p_pb_adv->buffer.payload[data_index], p_packet->pdu.payload.transaction.continuation.payload, payload_length);

            p_pb_adv->buffer.finished_segments |= (1 << p_packet->pdu.id);

            /* Here's some neat bitfield magic for you:
             * If (and only if) all packets in an N-segment transaction
             * have been received, the bitmap should have N number of 1's
             * on the lower half of the bitfield. Adding 1 to this full
             * bitfield should give a single bit with offset N.
             * E.g. N=5: 0b00011111 + 1 == 0b00100000 == (1 << 5).
             * If a single bit is missing, we'll get a different result.
             */
            if (p_pb_adv->buffer.finished_segments + 1 == (1 << transaction_total_segment_count_get(p_pb_adv->buffer.length)))
            {
                /* All segments received. */
                prov_buffer_rx(p_pb_adv, &p_pb_adv->buffer);
            }
        }
    }
    else
    {
        if (p_packet->transaction_number < p_pb_adv->transaction_in)
        {
            /* Got packet for a transaction we've already processed, send the ack again. */
            send_transaction_ack(p_pb_adv, p_packet->transaction_number);
        }
        else if (p_packet->transaction_number > p_pb_adv->transaction_in)
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got segment from unexpected transaction number (was %x, next is %x).\n", p_packet->transaction_number, p_pb_adv->transaction_in);
        }
    }
}

static void handle_transaction_ack_packet(nrf_mesh_prov_bearer_adv_t * p_pb_adv, pb_adv_pdu_t * p_packet)
{
    if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX &&
        p_packet->transaction_number == p_pb_adv->transaction_out)
    {
        __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "Received ACK for [ppdu%x]\n", p_pb_adv->buffer.payload[0]);

        /* Abort retry timer. If it fails, we'll correct it on the next timeout. */
        p_pb_adv->timeout_event.interval = 0;
        timer_sch_abort(&p_pb_adv->timeout_event);

        /* Stop transmitting the transaction. */
        advertiser_flush(&p_pb_adv->advertiser);

        /* Bump our transaction number, as this one's finished. */
        p_pb_adv->transaction_out = transaction_number_increment(p_pb_adv->transaction_out);
        p_pb_adv->buffer.state = PROV_BEARER_ADV_BUF_STATE_UNUSED;

        reset_timeout_timer(p_pb_adv);
        p_pb_adv->prov_bearer.p_callbacks->ack(&p_pb_adv->prov_bearer);
    }
    else
    {
        __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Got unexpected transaction ack\n");
    }
}

static void handle_control_packet(nrf_mesh_prov_bearer_adv_t * p_pb_adv, pb_adv_pdu_t * p_packet, uint32_t length)
{
    bool packet_len_check;
    switch (p_packet->pdu.id)
    {
        case PB_ADV_PACKET_CONTROL_ID_LINK_OPEN:
            packet_len_check = (length == (PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_OPEN_LEN));
            if (packet_len_check && memcmp(p_packet->pdu.payload.control.link_open.uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE) == 0)
            {
                if (p_pb_adv == NULL)
                {
                    /* link ID of the incoming packet didn't match any known link IDs, find an available bearer */
                    p_pb_adv = get_bearer_from_state(PROV_BEARER_ADV_STATE_LISTEN);
                }

                if (p_pb_adv != NULL)
                {
                    if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LISTEN)
                    {
                        p_pb_adv->link_id = BE2LE32(p_packet->link_id);

                        /* Start using short advertisement interval */
                        reset_adv_int(p_pb_adv);
                        /* No need to flush the bearer TX queue, since we sent
                         * the unprov beacon with infinite repeats, making the
                         * next packet replace it. */

                        if (send_link_ack(p_pb_adv) == NRF_SUCCESS)
                        {
                            __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "PB-ADV: Link opened.\n");
                            p_pb_adv->state = PROV_BEARER_ADV_STATE_LINK_OPEN;
                            reset_timeout_timer(p_pb_adv);

                            p_pb_adv->prov_bearer.p_callbacks->opened(&p_pb_adv->prov_bearer);
                        }
                        else
                        {
                            p_pb_adv->link_id = 0;
                        }
                    }
                    else if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN &&
                            p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_UNUSED)
                    {
                            /* Most likely, the other side missed our first ack, send another. */
                            (void) send_link_ack(p_pb_adv);
                    }
                }
            }
            break;
        case PB_ADV_PACKET_CONTROL_ID_LINK_ACK:
            packet_len_check = (length == (PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_ACK_LEN));
            if (p_pb_adv != NULL && packet_len_check)
            {
                if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPENING)
                {
                    /* Disable the link open transmits and reset advertisement interval. */
                    advertiser_flush(&p_pb_adv->advertiser);
                    reset_adv_int(p_pb_adv);
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "PB-ADV: Link opened.\n");

                    p_pb_adv->state = PROV_BEARER_ADV_STATE_LINK_OPEN;
                    reset_timeout_timer(p_pb_adv);
                    p_pb_adv->prov_bearer.p_callbacks->opened(&p_pb_adv->prov_bearer);
                }
                else
                {
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Received unexpected link ack in PB-ADV state %d\n", p_pb_adv->state);
                }
            }
            break;
        case PB_ADV_PACKET_CONTROL_ID_LINK_CLOSE:
            packet_len_check = (length == (PB_ADV_PACKET_OVERHEAD + PROV_BEARER_PACKET_LINK_CLOSE_LEN));
            if (p_pb_adv != NULL && packet_len_check)
            {
                if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPENING ||
                    p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN)
                {
                    /* Cancel any ongoing TX operations. */
                    if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX)
                    {
                        /* Stop the timer */
                        timer_sch_abort(&p_pb_adv->timeout_event);
                    }
                    close_link(p_pb_adv, (nrf_mesh_prov_link_close_reason_t) p_packet->pdu.payload.control.link_close.reason);
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_INFO, "PB-ADV: Link closed.\n");

                }
                else
                {
                    __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Received unexpected link close in PB-ADV state %d\n", p_pb_adv->state);
                }
            }
            break;
        default:
            __LOG(LOG_SRC_PROV, LOG_LEVEL_WARN, "Received unexpected control packet %x\n", p_packet->pdu.id);
    }
}

/**** Callbacks ****/

static void tx_retry_cb(timestamp_t timestamp, void * p_context)
{
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = p_context;

    if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN &&
        p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX)
    {
        if (TIMER_OLDER_THAN(p_pb_adv->sar_timeout, timestamp))
        {
            /* We've spent all our retries, and can regard the link as lost */
            if (send_link_close(p_pb_adv, NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT) == NRF_SUCCESS)
            {
                /* Wait for packets to finish sending before actually closing the link. */
                p_pb_adv->state = PROV_BEARER_ADV_STATE_LINK_CLOSING;
                p_pb_adv->close_reason = NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT;

                /* Stop this timeout from retriggering. */
                p_pb_adv->timeout_event.interval = 0;
            }
            else
            {
                /* Couldn't send link close. Shorten the interval, and keep trying. */
                p_pb_adv->timeout_event.interval = PROV_BEARER_ADV_LINK_CLOSE_RETRY_INTERVAL_US;

                /* If we keep failing the link close send, the timer will
                 * eventually roll over, and the TIMER_OLDER_THAN check a few
                 * lines up will give different results. Move the timeout to
                 * the current value, so that we'll always overstep it by some
                 * small amount on the next fire.
                 */
                p_pb_adv->sar_timeout = timestamp;
            }
        }
        else
        {
            __LOG(LOG_SRC_PROV, LOG_LEVEL_DBG1, "TX RETRY: [ppdu%x]\n", p_pb_adv->buffer.payload[0]);
            /* resend entire transaction. */
            p_pb_adv->buffer.finished_segments = 0;
            prov_buffer_tx(p_pb_adv);
        }
    }
    else
    {
        /* The timeout was invalid, set interval to 0 to avoid getting this timeout again. */
        p_pb_adv->timeout_event.interval = 0;
    }
}

static void queue_empty_cb(nrf_mesh_prov_bearer_adv_t * p_pb_adv)
{
    if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_CLOSING)
    {
        /* We've sent all link close packets, and may close the link if there
         * are no timers about to expire. */
        if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX)
        {
            timer_sch_abort(&p_pb_adv->timeout_event);
        }
        close_link(p_pb_adv, p_pb_adv->close_reason);
    }
    else if (p_pb_adv->buffer.state == PROV_BEARER_ADV_BUF_STATE_TX)
    {
        /* If there are any segments that haven't been sent, we'll send them now: */
        prov_buffer_tx(p_pb_adv);
    }
}


/* Gets called from the radio IRQ context! */
static void tx_complete_cb(advertiser_t * p_adv, nrf_mesh_tx_token_t token, timestamp_t timestamp)
{
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, advertiser, p_adv);

    if (token == p_pb_adv->last_token)
    {
        p_pb_adv->queue_empty_pending = true;
        bearer_event_flag_set(m_async_process_flag);
    }
}

static bool async_process(void)
{
    for (nrf_mesh_prov_bearer_adv_t *p_pb_adv = mp_bearer_head;
         p_pb_adv != NULL;
         p_pb_adv = p_pb_adv->p_next)
    {
        if (p_pb_adv->queue_empty_pending)
        {
            p_pb_adv->queue_empty_pending = false;
            queue_empty_cb(p_pb_adv);
        }
    }
    return true;
}


static void packet_in(const uint8_t * p_data, uint32_t data_len, const nrf_mesh_rx_metadata_t * p_metadata)
{
    NRF_MESH_ASSERT(p_data != NULL);
    NRF_MESH_ASSERT(p_metadata != NULL);

    pb_adv_pdu_t * p_packet = (pb_adv_pdu_t *) p_data;

    nrf_mesh_prov_bearer_adv_t * p_pb_adv = get_bearer_from_link_id(BE2LE32(p_packet->link_id));

    switch (p_packet->pdu.control)
    {
        case PB_ADV_PACKET_C_TRANSACTION_START:
            if (p_pb_adv != NULL && p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN)
            {
                handle_transaction_start_packet(p_pb_adv, p_packet, data_len);
            }
            break;
        case PB_ADV_PACKET_C_TRANSACTION_ACK:
            if (p_pb_adv != NULL && p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN)
            {
                handle_transaction_ack_packet(p_pb_adv, p_packet);
            }
            break;
        case PB_ADV_PACKET_C_TRANSACTION_CONTINUE:
            if (p_pb_adv != NULL && p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN)
            {
                handle_transaction_continuation_packet(p_pb_adv, p_packet, data_len);
            }
            break;
        case PB_ADV_PACKET_C_CONTROL:
            handle_control_packet(p_pb_adv, p_packet, data_len);
            break;
        default:
            /* Ignore */
            break;
    }
}

AD_LISTENER(m_pb_adv_ad_listener) = {
    .ad_type = AD_TYPE_PB_ADV,
    .adv_packet_type = BLE_PACKET_TYPE_ADV_NONCONN_IND,
    .handler = packet_in,
};

/**** Misc. ****/

static void close_link(nrf_mesh_prov_bearer_adv_t * p_pb_adv, nrf_mesh_prov_link_close_reason_t reason)
{
    NRF_MESH_ASSERT(p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN ||
                    p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPENING ||
                    p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_CLOSING);

    remove_active_bearer(p_pb_adv);
    NRF_MESH_ASSERT(p_pb_adv->instance_state == PROV_BEARER_ADV_INSTANCE_INITIALIZED);

    advertiser_flush(&p_pb_adv->advertiser);
    p_pb_adv->instance_state = PROV_BEARER_ADV_INSTANCE_RELEASED;
    p_pb_adv->buffer.state = PROV_BEARER_ADV_BUF_STATE_UNUSED;
    p_pb_adv->state = PROV_BEARER_ADV_STATE_IDLE;

    timer_sch_abort(&p_pb_adv->link_timeout_event);
    p_pb_adv->prov_bearer.p_callbacks->closed(&p_pb_adv->prov_bearer, reason);
}

/**** Interface functions ****/

static uint32_t prov_bearer_adv_listen(prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info, uint32_t link_timeout_us)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);

    if (p_pb_adv->instance_state == PROV_BEARER_ADV_INSTANCE_INITIALIZED &&
        p_pb_adv->state != PROV_BEARER_ADV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    init_bearer_structure(p_pb_adv, link_timeout_us);

#if MESH_FEATURE_LPN_ENABLED
    scanner_enable();
#endif

    /* Set initial link values */
    p_pb_adv->link_id = 0;
    p_pb_adv->transaction_out = PB_ADV_TRANSACTION_NUMBER_PROVISIONEE_START;
    p_pb_adv->transaction_in  = PB_ADV_TRANSACTION_NUMBER_PROVISIONER_START;

    send_unprov_beacon(p_pb_adv, URI, oob_info);

    advertiser_enable(&p_pb_adv->advertiser);
    add_active_bearer(p_pb_adv);
    p_pb_adv->state = PROV_BEARER_ADV_STATE_LISTEN;
    return NRF_SUCCESS;
}

static uint32_t prov_bearer_adv_listen_stop(prov_bearer_t * p_bearer)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);

    if (p_pb_adv->state != PROV_BEARER_ADV_STATE_LISTEN)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    remove_active_bearer(p_pb_adv);
    /* Stop the unprovisioned beacon */
    advertiser_flush(&p_pb_adv->advertiser);
    p_pb_adv->state = PROV_BEARER_ADV_STATE_IDLE;
    return NRF_SUCCESS;
}

static uint32_t prov_bearer_adv_link_open(prov_bearer_t * p_bearer, const uint8_t * p_peer_uuid, uint32_t link_timeout_us)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);

    if (p_pb_adv->instance_state == PROV_BEARER_ADV_INSTANCE_INITIALIZED &&
        p_pb_adv->state != PROV_BEARER_ADV_STATE_IDLE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    init_bearer_structure(p_pb_adv, link_timeout_us);

    /* Set initial link values */
    rand_hw_rng_get((uint8_t*) &p_pb_adv->link_id, sizeof(p_pb_adv->link_id));

    p_pb_adv->transaction_out = PB_ADV_TRANSACTION_NUMBER_PROVISIONER_START;
    p_pb_adv->transaction_in  = PB_ADV_TRANSACTION_NUMBER_PROVISIONEE_START;

    /* Send packet */
    uint32_t status = send_link_open(p_pb_adv, p_peer_uuid);

    if (status == NRF_SUCCESS)
    {
        /* Start the link timeout timer: */
        timer_sch_reschedule(&p_pb_adv->link_timeout_event,
                             timer_now() + PROV_BEARER_ADV_LINK_ESTABLISHMENT_LENGTH_US);

        advertiser_interval_set(&p_pb_adv->advertiser, PROV_BEARER_ADV_LINK_OPEN_ADVERTISER_INTERVAL_MS);
        advertiser_enable(&p_pb_adv->advertiser);
        add_active_bearer(p_pb_adv);
        p_pb_adv->state = PROV_BEARER_ADV_STATE_LINK_OPENING;
    }

    return status;
}

static void prov_bearer_adv_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    NRF_MESH_ASSERT(p_bearer != NULL);
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);

    /* We do not expect a close reason that is not defined by the enum */
    NRF_MESH_ASSERT(NRF_MESH_PROV_LINK_CLOSE_REASON_LAST >= close_reason);


    /* PB-remote does not track the link state. */
    if (p_pb_adv->state == PROV_BEARER_ADV_STATE_IDLE ||
        p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_CLOSING)
    {
        return;
    }

    /* Ensure the correct state: */
    NRF_MESH_ASSERT(p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPEN ||
                    p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPENING);

    if (p_pb_adv->state == PROV_BEARER_ADV_STATE_LINK_OPENING)
    { /* Provisioner still sends Link Open message (60 seconds have not been expired yet). Stop it. */
        advertiser_flush(&p_pb_adv->advertiser);
    }

    if (send_link_close(p_pb_adv, close_reason) == NRF_SUCCESS)
    {
        p_pb_adv->state = PROV_BEARER_ADV_STATE_LINK_CLOSING;
        p_pb_adv->close_reason = close_reason;
    }
    else
    {
        /* We didn't send the link close packets, so there's no reason to wait
         * for the adv queue empty callback. */
        close_link(p_pb_adv, close_reason);
    }
}

static uint32_t prov_bearer_adv_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    nrf_mesh_prov_bearer_adv_t * p_pb_adv = PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_adv_t, prov_bearer, p_bearer);

    if (p_pb_adv->state != PROV_BEARER_ADV_STATE_LINK_OPEN)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (length > PROV_PDU_MAX_LENGTH)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (p_pb_adv->buffer.state != PROV_BEARER_ADV_BUF_STATE_UNUSED)
    {
        return NRF_ERROR_BUSY;
    }

    reset_timeout_timer(p_pb_adv);

    /* Add the time we'll spend transmitting the packets to the retry interval. */
    uint32_t time_now = timer_now();
    uint32_t retry_interval = PROV_BEARER_ADV_TRANSACTION_BASE_RETRY_INTERVAL_US
        + (transaction_total_segment_count_get(length) * p_pb_adv->advertiser.config.advertisement_interval_us);
    p_pb_adv->timeout_event.interval = retry_interval;
    p_pb_adv->timeout_event.cb = tx_retry_cb;
    timer_sch_reschedule(&p_pb_adv->timeout_event, time_now + retry_interval);

    p_pb_adv->sar_timeout = time_now + PROV_BEARER_ADV_TRANSACTION_TIMEOUT_US;
    p_pb_adv->timeout_event.p_context = p_pb_adv;
    p_pb_adv->buffer.finished_segments = 0;
    p_pb_adv->buffer.state = PROV_BEARER_ADV_BUF_STATE_TX;
    p_pb_adv->buffer.length = length;
    p_pb_adv->buffer.fcs = calculate_3GPP_CRC(p_data, length);
    memcpy(p_pb_adv->buffer.payload, p_data, length);

    prov_buffer_tx(p_pb_adv);
    return NRF_SUCCESS;
}

/*****************************************************************************
 * Public API functions
 *****************************************************************************/

prov_bearer_t * nrf_mesh_prov_bearer_adv_interface_get(nrf_mesh_prov_bearer_adv_t * p_bearer_adv)
{
    NRF_MESH_ASSERT(p_bearer_adv != NULL);

    if (m_async_process_flag == BEARER_EVENT_FLAG_INVALID)
    {
        m_async_process_flag = bearer_event_flag_add(async_process);
    }

    static const prov_bearer_interface_t interface =
    {
        .tx = prov_bearer_adv_tx,
        .listen_start = prov_bearer_adv_listen,
        .listen_stop = prov_bearer_adv_listen_stop,
        .link_open = prov_bearer_adv_link_open,
        .link_close = prov_bearer_adv_link_close
    };

    p_bearer_adv->prov_bearer.bearer_type = NRF_MESH_PROV_BEARER_ADV;
    p_bearer_adv->prov_bearer.p_interface = &interface;

    return &p_bearer_adv->prov_bearer;
}

#endif /* MESH_FEATURE_PB_ADV_ENABLED */
