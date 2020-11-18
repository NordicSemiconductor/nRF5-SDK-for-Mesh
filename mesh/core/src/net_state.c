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

#include "net_state.h"
#include "timer.h"
#include "timer_scheduler.h"
#include "bearer_event.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_config_core.h"
#include "toolchain.h"
#include "event.h"
#include "mesh_opt_net_state.h"
#include "mesh_config_entry.h"

#include <string.h>

/*****************************************************************************
* Local defines
*****************************************************************************/

#define NETWORK_IV_UPDATE_TIMER_INTERVAL_US     (60000000) /**< 60 seconds. */

/** Longest time we're allowed to stay in an IV update state */
#define NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES  (144 * 60)
/** The minimum time between IV Recovery, in minutes. */
#define NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES  (192 * 60)
/** Margin to ensure we're inside of @tagMeshSp time limits */
#define NETWORK_IV_UPDATE_TIME_MARGIN_MINUTES   (10)

/** Maximum clock drift possible, according to Bluetooth Core Specification v4.0 (500ppm). */
#define MAX_CLOCK_DRIFT(T)  ((T) / 2000 + 1)

#define IV_UPDATE_TIMEOUT_MINUTES (NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES + \
                                   MAX_CLOCK_DRIFT(NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES))

#define IV_RECOVERY_TIMEOUT_MINUTES (NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES + \
                                     MAX_CLOCK_DRIFT(NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES))

#define IV_UPDATE_IN_PROGRESS_MAX_MINUTES (NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES - \
                                           MAX_CLOCK_DRIFT(NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES) - \
                                           NETWORK_IV_UPDATE_TIME_MARGIN_MINUTES)

/** Defines a period in minutes with which to record the current IV Update timeout counter.
 * @note A shorter period can wear out flash faster. */
#define IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES (30)

#define IV_UPDATE_TIMEOUT_NEEDS_TO_BE_SAVED(t) (((t) % IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES) == 0)

#define RESET_SEQNUM_MAX() (m_net_state.seqnum_max_available = 0)

/*****************************************************************************
* Local typedefs
*****************************************************************************/

/** Device network state */
typedef struct
{
    uint32_t seqnum; /**< Sequence number. */
    uint32_t seqnum_max_available; /**< The highest sequence number available for use. */
    uint32_t iv_index; /**< IV index. */
    // bool     key_refresh;               /**< Whether or not we're in key refresh. */
    struct
    {
        net_state_iv_update_t state;   /**< IV update state. */
        bool pending; /**< An IV update state change has been triggered, awaiting timer. */
        bool locked; /**< Changes to IV index or update state has been locked by an external module. */
        uint16_t timeout_counter; /**< Counter for IV update procedure timeout, in minutes. */
        uint16_t ivr_timeout_counter; /**< Counter for IV recovery procedure timeout, in minutes. */
    } iv_update;
} network_state_t;

typedef struct
{
    uint8_t is_seqnum_restored : 1;
    uint8_t is_iv_index_restored : 1;
    uint8_t is_seqnum_allocation_in_progress : 1;
    uint8_t is_test_mode : 1;
    uint8_t is_enabled : 1;
    uint8_t is_iv_state_set_externally : 1;
    uint8_t is_synchronized : 1;
} net_state_status_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
/** Current network state */
static network_state_t m_net_state;
/** IV update timer used to trigger IV update states. */
static timer_event_t m_iv_update_timer;
static net_state_status_t m_status;

static nrf_mesh_evt_handler_t m_mesh_evt_handler;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void seqnum_block_allocate(void);
static void iv_index_store(void);
static void iv_update_timeout_counter_store(void);

static inline bool iv_timeout_limit_passed(uint32_t timeout)
{
    return (timeout >= IV_UPDATE_TIMEOUT_MINUTES || !!m_status.is_test_mode);
}

/* Notify user of the new IV index. */
static void iv_index_notify(const uint8_t * p_network_id)
{
    nrf_mesh_evt_t app_event;
    if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_IN_PROGRESS)
    {
        app_event.params.iv_update.iv_index = m_net_state.iv_index - 1;
    }
    else
    {
        app_event.params.iv_update.iv_index = m_net_state.iv_index;
    }
    app_event.params.iv_update.state = m_net_state.iv_update.state;
    app_event.params.iv_update.p_network_id = p_network_id;
    app_event.type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION;
    event_handle(&app_event);
}

/** Trigger any pending IV update state transitions, if the state is unlocked. */
static bool iv_update_trigger_if_pending(void)
{
    bearer_event_critical_section_begin();

    bool pending = (m_net_state.iv_update.pending && !m_net_state.iv_update.locked &&
                    iv_timeout_limit_passed(m_net_state.iv_update.timeout_counter));

    if (pending)
    {
        m_net_state.iv_update.pending = false;
        switch (m_net_state.iv_update.state)
        {
            case NET_STATE_IV_UPDATE_NORMAL:
                NRF_MESH_ASSERT(net_state_iv_update_start() == NRF_SUCCESS);
                break;
            case NET_STATE_IV_UPDATE_IN_PROGRESS:
                m_net_state.seqnum = 0;
                RESET_SEQNUM_MAX();
                m_net_state.iv_update.state = NET_STATE_IV_UPDATE_NORMAL;
                m_net_state.iv_update.timeout_counter = 0;
                iv_index_notify(NULL);
                iv_index_store();
                seqnum_block_allocate();
                break;
            default:
                NRF_MESH_ASSERT(0); /* Unimplemented state. */
        }
    }

    bearer_event_critical_section_end();
    return pending;
}

static void iv_update_timer_handler(timestamp_t timestamp, void * p_context)
{
    bool increment_counter = true;
    switch (m_net_state.iv_update.state)
    {
        case NET_STATE_IV_UPDATE_NORMAL:
            if (iv_timeout_limit_passed(m_net_state.iv_update.timeout_counter))
            {
                /* We've spent enough time in normal state, may transition to
                 * in progress at any time. */
                (void) iv_update_trigger_if_pending();
                increment_counter = false;
            }
            break;
        case NET_STATE_IV_UPDATE_IN_PROGRESS:
            if (iv_timeout_limit_passed(m_net_state.iv_update.timeout_counter))
            {
                /* We've spent enough time in progress - may now transition to
                 * normal state at any time. */
                if (m_net_state.iv_update.timeout_counter >= IV_UPDATE_IN_PROGRESS_MAX_MINUTES)
                {
                    /* Force the move to normal state, as we're not allowed to
                     * stay in progress for any longer. */
                    m_net_state.iv_update.pending = true;
                }

                increment_counter = iv_update_trigger_if_pending() ? false : !m_net_state.iv_update.pending;
            }
            break;
        default:
            NRF_MESH_ASSERT(0); /* Unimplemented state */
    }

    if (increment_counter)
    {
        m_net_state.iv_update.timeout_counter++;

        if (IV_UPDATE_TIMEOUT_NEEDS_TO_BE_SAVED(m_net_state.iv_update.timeout_counter))
        {
            iv_update_timeout_counter_store();
        }
    }

    if (m_net_state.iv_update.ivr_timeout_counter > 0)
    {
        m_net_state.iv_update.ivr_timeout_counter--;
    }
}

static void incoming_data_received(const uint8_t * p_network_id, uint32_t iv_index, bool iv_update, bool key_refresh)
{
    if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_NORMAL)
    {
        if (m_net_state.iv_update.ivr_timeout_counter == 0 &&
            ((!iv_update && iv_index > m_net_state.iv_index) ||
             (iv_update && iv_index > m_net_state.iv_index + 1)))
        {
            if (iv_index <= (m_net_state.iv_index + NETWORK_IV_RECOVERY_LIMIT) &&
                !m_net_state.iv_update.locked)
            {
                /* We've fallen behind on IV indexes, and should silently
                 * adopt the other, higher IV index. */
                m_net_state.iv_index = iv_index;
                m_net_state.seqnum = 0;
                RESET_SEQNUM_MAX();
                m_net_state.iv_update.timeout_counter = 0;
                m_net_state.iv_update.ivr_timeout_counter = IV_RECOVERY_TIMEOUT_MINUTES;
                m_net_state.iv_update.pending = false;
                m_net_state.iv_update.state = NET_STATE_IV_UPDATE_NORMAL;
                iv_index_store();
                seqnum_block_allocate();
                iv_index_notify(p_network_id);
            }
        }
        else if (iv_update && iv_index == m_net_state.iv_index + 1)
        {
            m_net_state.iv_update.pending = true;
            (void) iv_update_trigger_if_pending();
        }
    }
    else if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_IN_PROGRESS)
    {
        if (!iv_update && iv_index == m_net_state.iv_index)
        {
            m_net_state.iv_update.pending = true;
            (void) iv_update_trigger_if_pending();
        }
    }
}

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_NET_BEACON_RECEIVED &&
        p_evt->params.net_beacon.p_beacon_info->iv_update_permitted)
    {
        incoming_data_received(p_evt->params.net_beacon.p_beacon_secmat->net_id,
                               p_evt->params.net_beacon.iv_index,
                               (p_evt->params.net_beacon.flags.iv_update == NET_STATE_IV_UPDATE_IN_PROGRESS),
                               p_evt->params.net_beacon.flags.key_refresh);
    }
#if MESH_FEATURE_LPN_ENABLED
    else if (p_evt->type == NRF_MESH_EVT_LPN_FRIEND_UPDATE)
    {
        incoming_data_received(NULL,
                               p_evt->params.friend_update.iv_index,
                               p_evt->params.friend_update.iv_update_active,
                               p_evt->params.friend_update.key_refresh_in_phase2);
    }
#endif
}
/*****************************************************************************
* FLASH MANAGER CODE
*****************************************************************************/

/* The only actual pair (iv index and sequence number) has a sense in case of restoring.
 * The "synchro index" has been added to understand that the restored values are relevant to each other.
 * Only values stored with the same synchro index should be used after restoring from the persistent subsystem. */
static uint8_t m_synchro_index;
static mesh_opt_iv_index_persist_data_t m_pst_iv_index;
static mesh_opt_seqnum_persist_data_t m_pst_seqnum;
static nrf_mesh_evt_handler_t m_persist_notifier;

static uint32_t seqnum_block_legacy_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_RECORD == entry_id.record);

    mesh_opt_seqnum_persist_data_legacy_t * p_lgcy = (mesh_opt_seqnum_persist_data_legacy_t *)p_entry;

    m_pst_seqnum.next_block = p_lgcy->next_block;
    m_status.is_seqnum_restored = 1;

    /* In the legacy format, the IV index was always stored before the sequence number on IV update,
     * so if we find the legacy sequence number first, it means that we lost power before we had time
     * to flash the new, zeroed sequence number. This means that the loaded sequence number is invalid,
     * and should be ignored. */
    m_pst_seqnum.synchro_index = !!m_status.is_iv_index_restored ? 0 : 1;

    return NRF_SUCCESS;
}

static uint32_t iv_index_legacy_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_RECORD == entry_id.record);

    mesh_opt_iv_index_persist_data_legacy_t * p_lgcy = (mesh_opt_iv_index_persist_data_legacy_t *)p_entry;

    m_pst_iv_index.iv_index = p_lgcy->iv_index;
    m_pst_iv_index.iv_update_in_progress = (net_state_iv_update_t)p_lgcy->iv_update_in_progress;
    m_pst_iv_index.iv_update_timeout_counter = 0;
    m_pst_iv_index.synchro_index = 0;
    m_status.is_iv_index_restored = 1;
    return NRF_SUCCESS;
}

static uint32_t seqnum_block_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD == entry_id.record);

    memcpy(&m_pst_seqnum, p_entry, sizeof(mesh_opt_seqnum_persist_data_t));
    m_status.is_seqnum_restored = 1;
    return NRF_SUCCESS;
}

static void seqnum_block_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD == entry_id.record);

    memcpy(p_entry, &m_pst_seqnum, sizeof(mesh_opt_seqnum_persist_data_t));
}

static void seqnum_block_deleter(mesh_config_entry_id_t entry_id)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD == entry_id.record);

    memset(&m_pst_seqnum, 0, sizeof(mesh_opt_seqnum_persist_data_t));
}

static uint32_t iv_index_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_IV_INDEX_RECORD == entry_id.record);

    memcpy(&m_pst_iv_index, p_entry, sizeof(mesh_opt_iv_index_persist_data_t));
    m_status.is_iv_index_restored = 1;
    return NRF_SUCCESS;
}

static void iv_index_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_IV_INDEX_RECORD == entry_id.record);

    memcpy(p_entry, &m_pst_iv_index, sizeof(mesh_opt_iv_index_persist_data_t));
}

static void iv_index_deleter(mesh_config_entry_id_t entry_id)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_NET_STATE_IV_INDEX_RECORD == entry_id.record);

    memset(&m_pst_iv_index, 0, sizeof(mesh_opt_iv_index_persist_data_t));
}

MESH_CONFIG_ENTRY(seqnum_block_legacy,
                  MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID,
                  1,
                  sizeof(mesh_opt_seqnum_persist_data_legacy_t),
                  seqnum_block_legacy_setter,
                  NULL,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(iv_index_legacy,
                  MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID,
                  1,
                  sizeof(mesh_opt_iv_index_persist_data_legacy_t),
                  iv_index_legacy_setter,
                  NULL,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(seqnum_block,
                  MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID,
                  1,
                  sizeof(mesh_opt_seqnum_persist_data_t),
                  seqnum_block_setter,
                  seqnum_block_getter,
                  seqnum_block_deleter,
                  NULL);

MESH_CONFIG_ENTRY(iv_index,
                  MESH_OPT_NET_STATE_IV_INDEX_EID,
                  1,
                  sizeof(mesh_opt_iv_index_persist_data_t),
                  iv_index_setter,
                  iv_index_getter,
                  iv_index_deleter,
                  NULL);

MESH_CONFIG_FILE(m_net_state_file, MESH_OPT_NET_STATE_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

static void persist_completed_notify(const nrf_mesh_evt_t * p_evt)
{
    if (!m_status.is_seqnum_allocation_in_progress)
    {
        return;
    }

    if (NRF_MESH_EVT_CONFIG_STABLE == p_evt->type)
    {
        m_net_state.seqnum_max_available = m_pst_seqnum.next_block;
        m_status.is_seqnum_allocation_in_progress = 0;
    }
}

static void seqnum_block_allocate(void)
{
    NRF_MESH_ASSERT(!!m_status.is_synchronized);

    if (!!m_status.is_seqnum_allocation_in_progress)
    {
        return;
    }

    mesh_opt_seqnum_persist_data_t seqnum_data =
    {
        .next_block = m_net_state.seqnum_max_available + NETWORK_SEQNUM_FLASH_BLOCK_SIZE
    };

    if (seqnum_data.next_block <= NETWORK_SEQNUM_MAX + 1)
    {
        seqnum_data.synchro_index = m_synchro_index;
        m_status.is_seqnum_allocation_in_progress = 1;
        NRF_MESH_ERROR_CHECK(mesh_config_entry_set(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &seqnum_data));
    }
}

static void iv_index_store(void)
{
    mesh_opt_iv_index_persist_data_t iv_index_data =
    {
        .iv_index = m_net_state.iv_index,
        .iv_update_in_progress = m_net_state.iv_update.state,
        .iv_update_timeout_counter = 0,
    };

    iv_index_data.synchro_index = ++m_synchro_index;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(MESH_OPT_NET_STATE_IV_INDEX_EID, &iv_index_data));
}

static void iv_update_timeout_counter_store(void)
{
    mesh_opt_iv_index_persist_data_t iv_index_data =
    {
        .iv_index = m_net_state.iv_index,
        .iv_update_in_progress = m_net_state.iv_update.state,
        .iv_update_timeout_counter = m_net_state.iv_update.timeout_counter,
    };

    iv_index_data.synchro_index = m_synchro_index;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(MESH_OPT_NET_STATE_IV_INDEX_EID, &iv_index_data));
}

static void restored_result_apply(void)
{
    m_persist_notifier.evt_cb = persist_completed_notify;
    nrf_mesh_evt_handler_add(&m_persist_notifier);

    if (!!m_status.is_iv_state_set_externally)
    { // higher layer changed parameters before start
        return;
    }

    if (!m_status.is_iv_index_restored || !m_status.is_seqnum_restored)
    {
        /* Need both data types to consider the storage valid. Reset state and flash both. */
        m_net_state.seqnum = 0;
        m_net_state.iv_index = 0;
        m_net_state.iv_update.state = NET_STATE_IV_UPDATE_NORMAL;
        m_net_state.iv_update.timeout_counter = 0;
        iv_index_store();
    }
    else
    {
        m_net_state.iv_index = m_pst_iv_index.iv_index;
        m_net_state.iv_update.state = m_pst_iv_index.iv_update_in_progress;
        m_net_state.iv_update.timeout_counter = m_pst_iv_index.iv_update_timeout_counter;
        m_synchro_index = m_pst_iv_index.synchro_index;

        if (m_pst_iv_index.synchro_index != m_pst_seqnum.synchro_index && m_net_state.iv_update.state == NET_STATE_IV_UPDATE_NORMAL)
        {
            /* At the end of the IV update state, we always store the IV index data first, then store
            * the sequence number after. If we found the sequence number entry before the IV index
            * entry that marked the end of the IV update procedure, we should reset the sequence
            * number, as it means we failed to store the new sequence number after an IV update.
            */
            m_net_state.seqnum = 0;
        }
        else
        {
            m_net_state.seqnum = m_pst_seqnum.next_block;
        }
    }

    /* Regardless of flash contents, we must allocate the next block of sequence numbers before we
     * start sending. We set the seqnum max to the seqnum, and trigger the allocation, so no
     * sequence numbers are spent before we get the allocation stored. */
    m_net_state.seqnum_max_available = m_net_state.seqnum;
    m_status.is_synchronized = 1;
    seqnum_block_allocate();
}

static void legacy_remove(void)
{
    (void)mesh_config_entry_delete(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID);
    (void)mesh_config_entry_delete(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID);
}

void net_state_reset(void)
{
    mesh_config_file_clear(MESH_OPT_NET_STATE_FILE_ID);

    memset(&m_net_state, 0, sizeof(m_net_state));
    m_status.is_iv_state_set_externally = 0;
    m_status.is_synchronized = 1;
    seqnum_block_allocate();
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void net_state_init(void)
{
    memset(&m_net_state, 0, sizeof(m_net_state));
    m_iv_update_timer.timestamp = timer_now();
    m_iv_update_timer.cb = iv_update_timer_handler;
    m_iv_update_timer.interval = NETWORK_IV_UPDATE_TIMER_INTERVAL_US;
    m_iv_update_timer.p_context = 0;
    m_iv_update_timer.p_next = NULL;
    m_status.is_test_mode = 0;

    RESET_SEQNUM_MAX();
}

void net_state_enable(void)
{
    if (!m_status.is_enabled)
    {
        m_mesh_evt_handler.evt_cb = mesh_evt_handler;
        nrf_mesh_evt_handler_add(&m_mesh_evt_handler);
        restored_result_apply();
        legacy_remove();
        timer_sch_schedule(&m_iv_update_timer);
        m_status.is_enabled = 1;
    }
}

uint32_t net_state_iv_index_and_seqnum_alloc(uint32_t * p_iv_index, uint32_t * p_seqnum)
{
    // Attempt to allocate the sequence number before it is fully restored from persistent storage.
    if (!m_status.is_synchronized)
    {
        return NRF_ERROR_FORBIDDEN;
    }

    if (m_net_state.seqnum < m_net_state.seqnum_max_available)
    {
        /* Check if we've reached the seqnum threshold for a state transition. */
        uint32_t threshold = NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD;
        if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_IN_PROGRESS)
        {
            threshold = NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD;
        }

        bool ivu_triggered = false;
        if (m_net_state.seqnum >= threshold)
        {
            m_net_state.iv_update.pending = true;
            ivu_triggered = iv_update_trigger_if_pending();
        }

        if (!ivu_triggered &&
            m_net_state.seqnum >= (m_net_state.seqnum_max_available - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD))
        {
            seqnum_block_allocate();
        }

        /* Get the sequence number after doing the state updates, as they might
         * trigger changes to it. */
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        *p_seqnum = m_net_state.seqnum++;
        *p_iv_index = net_state_tx_iv_index_get();
        _ENABLE_IRQS(was_masked);

        return NRF_SUCCESS;
    }
    else
    {
        seqnum_block_allocate();
        return NRF_ERROR_FORBIDDEN;
    }
}

bool net_state_is_seqnum_block_ready(void)
{
    return !m_status.is_seqnum_allocation_in_progress;
}

uint32_t net_state_iv_update_start(void)
{
    uint32_t status;
    bearer_event_critical_section_begin();
    if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_NORMAL &&
        !m_net_state.iv_update.locked &&
        iv_timeout_limit_passed(m_net_state.iv_update.timeout_counter))
    {
        m_net_state.iv_update.state = NET_STATE_IV_UPDATE_IN_PROGRESS;
        m_net_state.iv_update.timeout_counter = 0;
        m_net_state.iv_index = m_net_state.iv_index + 1;
        iv_index_store();
        iv_index_notify(NULL);
        status = NRF_SUCCESS;
    }
    else
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    bearer_event_critical_section_end();
    return status;
}

void net_state_iv_update_test_mode_set(bool test_mode_on)
{
    m_status.is_test_mode = test_mode_on ? 1 : 0;
}

uint32_t net_state_test_mode_transition_run(net_state_iv_update_signals_t signal)
{
    if ((signal == NET_STATE_TO_IV_UPDATE_IN_PROGRESS_SIGNAL &&
         m_net_state.iv_update.state == NET_STATE_IV_UPDATE_IN_PROGRESS) ||
        (signal == NET_STATE_TO_NORMAL_SIGNAL &&
         m_net_state.iv_update.state == NET_STATE_IV_UPDATE_NORMAL))
    {
        return NRF_ERROR_FORBIDDEN;
    }

    m_net_state.iv_update.pending = true;
    (void) iv_update_trigger_if_pending();

    return NRF_SUCCESS;
}

void net_state_iv_index_lock(bool lock)
{
    static uint32_t lock_count = 0;
    if (lock)
    {
        m_net_state.iv_update.locked = true;
        lock_count++;
    }
    else
    {
        NRF_MESH_ASSERT(lock_count > 0);
        lock_count--;
        if (lock_count == 0)
        {
            m_net_state.iv_update.locked = false;
            (void) iv_update_trigger_if_pending();
        }
    }
}

void net_state_key_refresh_phase_changed(uint16_t subnet_index, const uint8_t * p_network_id, nrf_mesh_key_refresh_phase_t new_phase)
{
    nrf_mesh_evt_t app_event;
    app_event.params.key_refresh.subnet_index = subnet_index;
    app_event.params.key_refresh.phase = new_phase;
    app_event.params.key_refresh.p_network_id = p_network_id;
    app_event.type = NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION;
    event_handle(&app_event);
}

uint32_t net_state_beacon_iv_index_get(void)
{
    return m_net_state.iv_index;
}

uint32_t net_state_tx_iv_index_get(void)
{
    /* While an IV update is in progress, we should transmit on the previous IV
     * index. */
    if (m_net_state.iv_update.state == NET_STATE_IV_UPDATE_IN_PROGRESS)
    {
        return m_net_state.iv_index - 1;
    }
    else
    {
        return m_net_state.iv_index;
    }
}

uint32_t net_state_rx_iv_index_get(uint8_t ivi)
{
    /* Always receive on the current IV index and the previous. Compare the lsb
     * to determine which of the two the given ivi belongs to. */
    if ((m_net_state.iv_index & NETWORK_IVI_MASK) != (ivi & 0x01))
    {
        return (m_net_state.iv_index - 1);
    }
    else
    {
        return m_net_state.iv_index;
    }
}

net_state_iv_update_t net_state_iv_update_get(void)
{
    return m_net_state.iv_update.state;
}

uint32_t net_state_iv_index_and_seqnum_block_set(uint32_t iv_index, bool iv_update, uint32_t next_seqnum_block)
{
    if (!!m_status.is_iv_state_set_externally)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        m_status.is_iv_state_set_externally = 1;
        m_net_state.iv_index = iv_index;
        m_net_state.iv_update.state =
            iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL;
        m_net_state.iv_update.timeout_counter = 0;

        iv_index_store();
        iv_index_notify(NULL);

        m_net_state.seqnum = next_seqnum_block;
        m_net_state.seqnum_max_available = m_net_state.seqnum;
        m_status.is_synchronized = 1;
        seqnum_block_allocate();

        return NRF_SUCCESS;
    }
}

uint32_t net_state_iv_index_set(uint32_t iv_index, bool iv_update)
{
    return net_state_iv_index_and_seqnum_block_set(iv_index, iv_update, 0);
}

#if defined UNIT_TEST
void net_state_disable(void)
{
    m_status.is_enabled = 0;
    m_status.is_iv_state_set_externally = 0;
    m_status.is_synchronized = 0;
    m_synchro_index = 0;
    memset(&m_pst_iv_index, 0, sizeof(mesh_opt_iv_index_persist_data_t));
    memset(&m_pst_seqnum, 0, sizeof(mesh_opt_seqnum_persist_data_t));
    memset(&m_status, 0, sizeof(net_state_status_t));
    memset(&m_persist_notifier, 0, sizeof(nrf_mesh_evt_handler_t));
}
#endif
