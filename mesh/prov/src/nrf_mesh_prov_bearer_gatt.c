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

#include "nrf_mesh_config_prov.h"
#include "nrf_mesh_prov_bearer_gatt.h"
#include "mesh_gatt.h"
#include "fsm.h"
#include "fsm_assistant.h"
#include "nordic_common.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_configure.h"
#include "mesh_adv.h"
#include "internal_event.h"

/*******************************************************************************
 * FSM setup
 *******************************************************************************/

/*lint -e123 */
/* We differentiate the CONNECTED and LINK_ACTIVE states because the Client needs
 * to write our CCCD before we can start transmitting. */
#define STATE_LIST  S_IDLE,                     \
        S_LISTENING,                            \
        S_CONNECTED,                            \
        S_LINK_ACTIVE

#define EVENT_LIST E_LISTEN_START,              \
        E_LISTEN_STOP,                          \
        E_LISTEN_TIMEOUT,                       \
        E_CONNECTED,                            \
        E_TX_READY,                             \
        E_DISCONNECTED,                         \
        E_PDU_TX,                               \
        E_TX_COMPLETE,                          \
        E_PDU_RX,                               \
        E_LINK_CLOSE,                           \
        E_LINK_TIMEOUT

#define ACTION_LIST A_LINK_OPEN,          a_link_open,              \
                    A_LINK_TIMER_START,   a_link_timer_start,       \
                    A_LINK_CLOSE_NOTIFY,  a_link_close_notify,      \
                    A_LINK_CLOSE,         a_link_close,             \
                    A_PDU_TX,             a_pdu_tx,                 \
                    A_PDU_RX,             a_pdu_rx,                 \
                    A_PDU_ACK,            a_pdu_ack,                \
                    A_LISTEN_START,       a_listen_start,           \
                    A_LISTEN_STOP,        a_listen_stop

#define GUARD_LIST G_IS_PROV_PDU_TYPE, g_is_prov_pdu_type

typedef enum
{
    DECLARE_ENUM(EVENT_LIST)
} pb_gatt_event_ids_t;

typedef enum
{
    DECLARE_ENUM(STATE_LIST)
} pb_gatt_state_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(ACTION_LIST)
} pb_gatt_action_ids_t;

typedef enum
{
    DECLARE_ENUM_PAIR(GUARD_LIST)
} pb_gatt_guard_ids_t;

typedef void (* pb_gatt_fsm_action_t)(void *);
typedef bool (* pb_gatt_fsm_guard_t)(void *);

/* Not having a semi-colon is intentional. */
DECLARE_ACTION_PROTOTYPE(ACTION_LIST)
DECLARE_GUARD_PROTOTYPE(GUARD_LIST)

static void pb_gatt_fsm_action(fsm_action_id_t action_id, void * p_data);
static bool pb_gatt_fsm_guard(fsm_action_id_t action_id, void * p_data);

static const fsm_transition_t m_pb_gatt_fsm_transition_table[] =
{
    FSM_STATE(S_IDLE),
    FSM_TRANSITION(E_LISTEN_START,    FSM_ALWAYS,         A_LISTEN_START,       S_LISTENING),

    FSM_STATE(S_LISTENING),
    FSM_TRANSITION(E_CONNECTED,       FSM_ALWAYS,         A_LINK_OPEN,          S_CONNECTED),
    FSM_TRANSITION(E_LISTEN_STOP,     FSM_ALWAYS,         A_LISTEN_STOP,        S_IDLE),
    FSM_TRANSITION(E_LISTEN_TIMEOUT,  FSM_ALWAYS,         FSM_NO_ACTION,        S_IDLE),

    FSM_STATE(S_CONNECTED),
    FSM_TRANSITION(E_TX_READY,        FSM_ALWAYS,         A_LINK_TIMER_START,   S_LINK_ACTIVE),
    FSM_TRANSITION(E_LINK_TIMEOUT,    FSM_ALWAYS,         A_LINK_CLOSE,         S_CONNECTED),
    FSM_TRANSITION(E_DISCONNECTED,    FSM_ALWAYS,         A_LINK_CLOSE_NOTIFY,  S_IDLE),

    FSM_STATE(S_LINK_ACTIVE),
    FSM_TRANSITION(E_PDU_TX,          FSM_ALWAYS,         A_PDU_TX,             FSM_SAME_STATE),
    FSM_TRANSITION(E_PDU_RX,          G_IS_PROV_PDU_TYPE, A_PDU_RX,             FSM_SAME_STATE),
    FSM_TRANSITION(E_PDU_RX,          FSM_OTHERWISE,      FSM_NO_ACTION,        FSM_SAME_STATE),
    FSM_TRANSITION(E_TX_COMPLETE,     FSM_ALWAYS,         A_PDU_ACK,            FSM_SAME_STATE),
    FSM_TRANSITION(E_LINK_TIMEOUT,    FSM_ALWAYS,         A_LINK_CLOSE,         S_CONNECTED),
    FSM_TRANSITION(E_LINK_CLOSE,      FSM_ALWAYS,         A_LINK_CLOSE,         S_CONNECTED),
    FSM_TRANSITION(E_DISCONNECTED,    FSM_ALWAYS,         A_LINK_CLOSE_NOTIFY,  S_IDLE)
};

#if FSM_DEBUG
static const char * m_action_lookup_table[] =
{
    DECLARE_STRING_PAIR(ACTION_LIST)
};

static const char * m_guard_lookup_table[] =
{
    DECLARE_STRING_PAIR(GUARD_LIST)
};

static const char * m_event_lookup_table[] =
{
    DECLARE_STRING(EVENT_LIST)
};

static const char * m_state_lookup_table[] =
{
    DECLARE_STRING(STATE_LIST)
};
#endif  /* FSM_DEBUG */

static const fsm_const_descriptor_t m_pb_gatt_fsm_descriptor =
{
    .transition_table = m_pb_gatt_fsm_transition_table,
    .transitions_count = ARRAY_SIZE(m_pb_gatt_fsm_transition_table),
    .initial_state = S_IDLE,
    .guard = pb_gatt_fsm_guard,
    .action = pb_gatt_fsm_action,
#if FSM_DEBUG
    .fsm_name = "PB-GATT bearer",
    .action_lookup = m_action_lookup_table,
    .event_lookup = m_event_lookup_table,
    .guard_lookup = m_guard_lookup_table,
    .state_lookup = m_state_lookup_table
#endif  /* FSM_DEBUG */
};

static const pb_gatt_fsm_action_t pb_gatt_fsm_actions[] =
{
    DECLARE_HANDLER(ACTION_LIST)
};

static void pb_gatt_fsm_action(fsm_action_id_t action_id, void * p_data)
{
    pb_gatt_fsm_actions[action_id](p_data);
}

static const pb_gatt_fsm_guard_t pb_gatt_fsm_guards[] =
{
    DECLARE_HANDLER(GUARD_LIST)
};

static bool pb_gatt_fsm_guard(fsm_guard_id_t guard_id, void * p_data)
{
    return pb_gatt_fsm_guards[guard_id](p_data);
}
/*lint +e123 */

/*******************************************************************************
 * Type definitions
 *******************************************************************************/

typedef struct
{
    nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt;
    uint16_t conn_index;
    const uint8_t * p_data;
    uint16_t length;
    mesh_gatt_pdu_type_t pdu_type;
} fsm_evt_gatt_data_t;

typedef struct
{
    nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt;
    const uint8_t * p_data;
    uint16_t length;
} fsm_evt_prov_data_t;

typedef struct
{
    nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt;
    fsm_event_id_t evt;
    nrf_mesh_prov_link_close_reason_t close_reason;
    uint16_t oob_info;
    uint32_t link_timeout_us;
} fsm_evt_prov_link_t;

/** Advertisement service data for the PB-GATT service (@tagMeshSp section 7.1.2.2.1). */
typedef struct __attribute((packed))
{
    uint8_t device_uuid[NRF_MESH_UUID_SIZE];
    uint16_t oob_info;
} pb_gatt_service_data_t;

/*******************************************************************************
 * FSM handlers
 *******************************************************************************/

static void link_timer_reset(nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt)
{
    timestamp_t next_timeout = timer_now() + p_bearer_gatt->link_timeout_us;
    timer_sch_reschedule(&p_bearer_gatt->link_timeout_event, next_timeout);
}

static bool g_is_prov_pdu_type(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    bool result = (p_evt->pdu_type == MESH_GATT_PDU_TYPE_PROV_PDU);

    /* Print warning for test framework. */
    if (!result)
    {
        __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_GATT_PROV_PDU_IGNORED, 0, 0, NULL);
    }
    return (result);
}

static void a_link_timer_start(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    link_timer_reset(p_evt->p_bearer_gatt);
}

static void a_link_open(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    p_evt->p_bearer_gatt->bearer.p_callbacks->opened(&p_evt->p_bearer_gatt->bearer);
}

static void a_link_close_notify(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    timer_sch_abort(&p_evt->p_bearer_gatt->link_timeout_event);
    p_evt->p_bearer_gatt->bearer.p_callbacks->closed(&p_evt->p_bearer_gatt->bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
}

static void a_link_close(void * p_context)
{
    fsm_evt_prov_link_t * p_evt = p_context;
    NRF_MESH_ERROR_CHECK(mesh_gatt_disconnect(p_evt->p_bearer_gatt->conn_index));
}

static void a_pdu_tx(void * p_context)
{
    fsm_evt_prov_data_t * p_evt = p_context;
    uint8_t * p_packet = mesh_gatt_packet_alloc(p_evt->p_bearer_gatt->conn_index, MESH_GATT_PDU_TYPE_PROV_PDU, p_evt->length, 0);

    /* We should not have any other GATT stuff going on at this point. */
    NRF_MESH_ASSERT(p_packet != NULL);

    memcpy(p_packet, p_evt->p_data, p_evt->length);
    NRF_MESH_ERROR_CHECK(mesh_gatt_packet_send(p_evt->p_bearer_gatt->conn_index,
                                               p_packet));
    link_timer_reset(p_evt->p_bearer_gatt);
}

static void a_pdu_rx(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    prov_bearer_t * p_bearer = &p_evt->p_bearer_gatt->bearer;
    link_timer_reset(p_evt->p_bearer_gatt);
    p_bearer->p_callbacks->rx(p_bearer, p_evt->p_data, p_evt->length);
}

static void a_pdu_ack(void * p_context)
{
    fsm_evt_gatt_data_t * p_evt = p_context;
    prov_bearer_t * p_bearer = &p_evt->p_bearer_gatt->bearer;
    p_bearer->p_callbacks->ack(p_bearer);
}

static void a_listen_start(void * p_context)
{
    fsm_evt_prov_link_t * p_evt = p_context;
    nrf_mesh_prov_bearer_gatt_t * p_pb_gatt = p_evt->p_bearer_gatt;
    p_pb_gatt->link_timeout_us = p_evt->link_timeout_us;

    pb_gatt_service_data_t service_data;
    memcpy(service_data.device_uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE);
    service_data.oob_info = LE2BE16(p_evt->oob_info);

    mesh_adv_params_set(MESH_ADV_TIMEOUT_INFINITE,
                        MSEC_TO_UNITS(NRF_MESH_PROV_BEARER_GATT_UNPROV_BEACON_INTERVAL_MS,
                                      UNIT_0_625_MS));
    mesh_adv_data_set(NRF_MESH_PB_GATT_SERVICE_UUID,
                      (const uint8_t *) &service_data,
                      sizeof(service_data));
    mesh_adv_start();
}

static void a_listen_stop(void * p_context)
{
    mesh_adv_stop();
}

/*******************************************************************************
 * Event translation
 *******************************************************************************/

static void link_timer_cb(timestamp_t time_now, void * p_context)
{
    nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt = p_context;
    fsm_evt_prov_link_t evt;
    evt.evt = E_LINK_TIMEOUT;
    evt.p_bearer_gatt = p_context;
    evt.close_reason = NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT;
    fsm_event_post(&p_bearer_gatt->fsm, E_LINK_TIMEOUT, &evt);
}

static void mesh_gatt_event_handler(const mesh_gatt_evt_t * p_evt, void * p_context)
{
    nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt = p_context;
    fsm_evt_gatt_data_t evt;
    memset(&evt, 0, sizeof(evt));

    evt.p_bearer_gatt = p_bearer_gatt;
    switch (p_evt->type)
    {
        case MESH_GATT_EVT_TYPE_ADV_TIMEOUT:
            fsm_event_post(&p_bearer_gatt->fsm, E_LISTEN_TIMEOUT, &evt);
            break;

        case MESH_GATT_EVT_TYPE_RX:
            evt.p_data = p_evt->params.rx.p_data;
            evt.length = p_evt->params.rx.length;
            evt.conn_index = p_evt->conn_index;
            evt.pdu_type = p_evt->params.rx.pdu_type;
            fsm_event_post(&p_bearer_gatt->fsm, E_PDU_RX, &evt);
            break;

        case MESH_GATT_EVT_TYPE_TX_COMPLETE:
            evt.conn_index = p_evt->conn_index;
            fsm_event_post(&p_bearer_gatt->fsm, E_TX_COMPLETE, &evt);
            break;

        case MESH_GATT_EVT_TYPE_CONNECTED:
            evt.conn_index = p_evt->conn_index;
            fsm_event_post(&p_bearer_gatt->fsm, E_CONNECTED, &evt);
            break;

        case MESH_GATT_EVT_TYPE_DISCONNECTED:
            evt.conn_index = p_evt->conn_index;
            fsm_event_post(&p_bearer_gatt->fsm, E_DISCONNECTED, &evt);
            break;

        case MESH_GATT_EVT_TYPE_TX_READY:
            evt.conn_index = p_evt->conn_index;
            fsm_event_post(&p_bearer_gatt->fsm, E_TX_READY, &evt);
            break;

        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}


static fsm_evt_prov_link_t m_evt_prov_link;


static void link_evt_process(void * p_context)
{
    fsm_evt_prov_link_t * p_evt = p_context;
    fsm_event_post(&p_evt->p_bearer_gatt->fsm, p_evt->evt, p_evt);
}

static void link_evt_send(void)
{
    if (fsm_is_processing(&m_evt_prov_link.p_bearer_gatt->fsm))
    {
        (void) bearer_event_sequential_post(&m_evt_prov_link.p_bearer_gatt->bearer_event_seq);
    }
    else
    {
        link_evt_process(&m_evt_prov_link);
    }
}

/* Provisioning bearer callback interface functions. */
static uint32_t tx_cb(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    fsm_evt_prov_data_t evt;
    memset(&evt, 0, sizeof(evt));

    evt.p_bearer_gatt =
        PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_gatt_t, bearer, p_bearer);

    evt.p_data = p_data;
    evt.length = length;
    fsm_event_post(&evt.p_bearer_gatt->fsm, E_PDU_TX, &evt);
    return NRF_SUCCESS;
}

static void link_close_cb(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    memset(&m_evt_prov_link, 0, sizeof(m_evt_prov_link));
    m_evt_prov_link.evt = E_LINK_CLOSE;
    m_evt_prov_link.p_bearer_gatt =
        PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_gatt_t, bearer, p_bearer);

    link_evt_send();
}

static uint32_t listen_start_cb(prov_bearer_t * p_bearer, const char * p_uri, uint16_t oob_info, uint32_t link_timeout_us)
{
    memset(&m_evt_prov_link, 0, sizeof(m_evt_prov_link));
    m_evt_prov_link.evt = E_LISTEN_START;
    m_evt_prov_link.p_bearer_gatt =
        PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_gatt_t, bearer, p_bearer);
    m_evt_prov_link.oob_info = oob_info;
    m_evt_prov_link.link_timeout_us = link_timeout_us;
    link_evt_send();
    return NRF_SUCCESS;
}

static uint32_t listen_stop_cb(prov_bearer_t * p_bearer)
{
    memset(&m_evt_prov_link, 0, sizeof(m_evt_prov_link));
    m_evt_prov_link.evt = E_LISTEN_STOP;
    m_evt_prov_link.p_bearer_gatt =
        PARENT_BY_FIELD_GET(nrf_mesh_prov_bearer_gatt_t, bearer, p_bearer);
    link_evt_send();
    return NRF_SUCCESS;
}

/*******************************************************************************
 * Public API
 *******************************************************************************/

uint32_t nrf_mesh_prov_bearer_gatt_init(nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt)
{
    if (p_bearer_gatt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    fsm_init(&p_bearer_gatt->fsm, &m_pb_gatt_fsm_descriptor);

    memset(&p_bearer_gatt->link_timeout_event, 0, sizeof(timer_event_t));
    p_bearer_gatt->link_timeout_event.cb = link_timer_cb;
    p_bearer_gatt->link_timeout_event.p_context = p_bearer_gatt;

    bearer_event_sequential_add(&p_bearer_gatt->bearer_event_seq,
                                link_evt_process,
                                &m_evt_prov_link);

    mesh_gatt_uuids_t uuids;
    uuids.service = NRF_MESH_PB_GATT_SERVICE_UUID;
    uuids.rx_char = NRF_MESH_PB_GATT_CHAR_IN_UUID;
    uuids.tx_char = NRF_MESH_PB_GATT_CHAR_OUT_UUID;
    mesh_gatt_init(&uuids, mesh_gatt_event_handler, p_bearer_gatt);
    return NRF_SUCCESS;
}

prov_bearer_t * nrf_mesh_prov_bearer_gatt_interface_get(nrf_mesh_prov_bearer_gatt_t * p_bearer_gatt)
{
    static const prov_bearer_interface_t prov_bearer_interface = {
        .tx           = tx_cb,
        .listen_start = listen_start_cb,
        .listen_stop  = listen_stop_cb,
        .link_open    = NULL,            /* N/A */
        .link_close   = link_close_cb
    };

    p_bearer_gatt->bearer.p_interface = &prov_bearer_interface;
    p_bearer_gatt->bearer.bearer_type = NRF_MESH_PROV_BEARER_GATT;
    return &p_bearer_gatt->bearer;
}
