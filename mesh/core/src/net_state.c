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

#include "net_state.h"
#include "timer.h"
#include "timer_scheduler.h"
#include "bearer_event.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_config_core.h"
#include "toolchain.h"
#include "event.h"
#include "flash_manager.h"

#include <string.h>

/*****************************************************************************
* Local defines
*****************************************************************************/

#define NETWORK_IV_UPDATE_TIMER_INTERVAL_US     (60000000) /**< 60 seconds. */

/** Longest time we're allowed to stay in an IV update state */
#define NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES  (144 * 60)
/** The minimum time between IV Recovery, in minutes. */
#define NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES  (192 * 60)
/** Margin to ensure we're inside of the Mesh Profile Specification v1.0 time limits */
#define NETWORK_IV_UPDATE_TIME_MARGIN_MINUTES   (10)

/** Maximum clock drift possible, according to Bluetooth Core Specification v4.0 (500ppm). */
#define MAX_CLOCK_DRIFT(T)  ((T) / 2000 + 1)

#define IV_UPDATE_TIMEOUT (NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES + \
                           MAX_CLOCK_DRIFT(NETWORK_MIN_IV_UPDATE_INTERVAL_MINUTES))

#define IV_RECOVERY_TIMEOUT (NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES + \
                           MAX_CLOCK_DRIFT(NETWORK_MIN_IV_RECOVERY_INTERVAL_MINUTES))

#define IV_UPDATE_IN_PROGRESS_MAX_TIME (NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES - \
                                        MAX_CLOCK_DRIFT(NETWORK_MAX_IV_UPDATE_INTERVAL_MINUTES) - \
                                        NETWORK_IV_UPDATE_TIME_MARGIN_MINUTES)

#define FLASH_HANDLE_SEQNUM         (0x0001)
#define FLASH_HANDLE_IV_INDEX       (0x0002)


#define SEQNUM_INVALID              (0xFFFFFFFF)
#define SEQNUM_MASK                 (NETWORK_SEQNUM_MAX)

#if PERSISTENT_STORAGE
    #define RESET_SEQNUM_MAX() (m_net_state.seqnum_max_available = 0)
#else
    #define RESET_SEQNUM_MAX() (m_net_state.seqnum_max_available = NETWORK_SEQNUM_MAX + 1)
#endif

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

/*****************************************************************************
* Static globals
*****************************************************************************/
/** Current network state */
static network_state_t m_net_state;
/** IV update timer used to trigger IV update states. */
static timer_event_t m_iv_update_timer;
/** IV update test mode state */
static bool m_test_mode;
static bool m_enabled;
static bool m_iv_state_set;

static nrf_mesh_evt_handler_t m_mesh_evt_handler;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void seqnum_block_allocate(void);
static void flash_store_iv_index(void);

static inline bool iv_timeout_limit_passed(uint32_t timeout)
{
    return (timeout >= IV_UPDATE_TIMEOUT || m_test_mode);
}

/* Notify user of the new IV index. */
static void iv_index_notify(const uint8_t * p_network_id, uint32_t iv_index, net_state_iv_update_t state)
{
    nrf_mesh_evt_t app_event;
    app_event.params.iv_update.iv_index = iv_index;
    app_event.params.iv_update.state = state;
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
                iv_index_notify(NULL, m_net_state.iv_index, m_net_state.iv_update.state);
                flash_store_iv_index();
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
                if (m_net_state.iv_update.timeout_counter >= IV_UPDATE_IN_PROGRESS_MAX_TIME)
                {
                    /* Force the move to normal state, as we're not allowed to
                     * stay in progress for any longer. */
                    m_net_state.iv_update.pending = true;
                    increment_counter = false;
                }
                (void) iv_update_trigger_if_pending();
            }
            break;
        default:
            NRF_MESH_ASSERT(0); /* Unimplemented state */
    }
    if (increment_counter)
    {
        m_net_state.iv_update.timeout_counter++;
    }

    if (m_net_state.iv_update.ivr_timeout_counter > 0)
    {
        m_net_state.iv_update.ivr_timeout_counter--;
    }
}

static void beacon_received(const uint8_t * p_network_id, uint32_t iv_index, bool iv_update, bool key_refresh)
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
                m_net_state.iv_update.ivr_timeout_counter = IV_RECOVERY_TIMEOUT;
                m_net_state.iv_update.pending = false;
                m_net_state.iv_update.state = NET_STATE_IV_UPDATE_NORMAL;
                flash_store_iv_index();
                seqnum_block_allocate();
                iv_index_notify(p_network_id, m_net_state.iv_index, m_net_state.iv_update.state);
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
        beacon_received(p_evt->params.net_beacon.p_beacon_secmat->net_id,
                        p_evt->params.net_beacon.iv_index,
                        (p_evt->params.net_beacon.flags.iv_update == NET_STATE_IV_UPDATE_IN_PROGRESS),
                        p_evt->params.net_beacon.flags.key_refresh);
    }
}
/*****************************************************************************
* FLASH MANAGER CODE
*****************************************************************************/

#if PERSISTENT_STORAGE
typedef struct
{
    uint32_t iv_index;
    uint8_t iv_update_in_progress;
} net_flash_data_iv_index_t;

typedef uint32_t net_flash_data_sequence_number_t;

/** Flash manager handling Network state flash storage. */
static flash_manager_t m_flash_manager;
static bool m_seqnum_allocation_in_progress;
/** Flash operation function to call when the memory returns. */
typedef void (*flash_op_func_t)(void);

static void flash_mem_available(void * p_args)
{
    NRF_MESH_ASSERT(p_args != NULL);
    flash_op_func_t func = (flash_op_func_t) p_args; /*lint !e611 Suspicious cast */
    func();
}

static void seqnum_block_allocate(void)
{
    if (!m_seqnum_allocation_in_progress)
    {
        uint32_t next_block = m_net_state.seqnum_max_available + NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
        if (next_block <= NETWORK_SEQNUM_MAX + 1)
        {
            fm_entry_t * p_new_entry = flash_manager_entry_alloc(&m_flash_manager, FLASH_HANDLE_SEQNUM, sizeof(net_flash_data_sequence_number_t));
            if (p_new_entry == NULL)
            {
                /* try again later */
                static fm_mem_listener_t mem_listener = {.callback = flash_mem_available,
                                                        .p_args = seqnum_block_allocate};
                flash_manager_mem_listener_register(&mem_listener);
            }
            else
            {
                p_new_entry->data[0] = next_block;
                m_seqnum_allocation_in_progress = true;
                flash_manager_entry_commit(p_new_entry);
            }
        }
    }
}

static void flash_store_iv_index(void)
{
    fm_entry_t * p_new_entry = flash_manager_entry_alloc(&m_flash_manager, FLASH_HANDLE_IV_INDEX, sizeof(net_flash_data_iv_index_t));
    if (p_new_entry == NULL)
    {
        /* try again later */
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_available,
                                                    .p_args = flash_store_iv_index};
        flash_manager_mem_listener_register(&mem_listener);
    }
    else
    {
        net_flash_data_iv_index_t * p_iv_index_data = (net_flash_data_iv_index_t *) p_new_entry->data;
        p_iv_index_data->iv_index = m_net_state.iv_index;
        p_iv_index_data->iv_update_in_progress = m_net_state.iv_update.state;

        flash_manager_entry_commit(p_new_entry);
    }
}

static void flash_write_complete(const flash_manager_t * p_manager,
                                 const fm_entry_t * p_entry,
                                 fm_result_t result)
{

    if (result == FM_RESULT_SUCCESS)
    {
        if (p_entry->header.handle == FLASH_HANDLE_SEQNUM)
        {
            NRF_MESH_ASSERT(m_seqnum_allocation_in_progress);
            m_net_state.seqnum_max_available = p_entry->data[0];
            m_seqnum_allocation_in_progress = false;
        }
    }
    else
    {
        NRF_MESH_ASSERT(result == FM_RESULT_ERROR_FLASH_MALFUNCTION);
        nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_FLASH_FAILED,
            .params.flash_failed.user = NRF_MESH_FLASH_USER_CORE,
            .params.flash_failed.p_flash_entry = p_entry,
            .params.flash_failed.p_flash_page = NULL,
            .params.flash_failed.p_area = m_flash_manager.config.p_area,
            .params.flash_failed.page_count = m_flash_manager.config.page_count,
        };
        event_handle(&evt);
    }
}

static void init_flash_storage(void)
{
    flash_manager_config_t config;
    memset(&config, 0, sizeof(config));
    config.min_available_space = 0;
    config.p_area = net_state_flash_area_get();
    config.page_count = NET_FLASH_PAGE_COUNT;
    config.write_complete_cb = flash_write_complete;

    if (flash_manager_add(&m_flash_manager, &config) != NRF_SUCCESS)
    {
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_available,
                                                 .p_args = init_flash_storage};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

void net_state_recover_from_flash(void)
{
    flash_manager_wait();
    bearer_event_critical_section_begin();
    const net_flash_data_iv_index_t * p_iv_index_data = NULL;
    const net_flash_data_sequence_number_t * p_seqnum_data = NULL;

    const fm_entry_t * p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, NULL);

    while (p_entry != NULL)
    {
        switch (p_entry->header.handle)
        {
            case FLASH_HANDLE_IV_INDEX:
                p_iv_index_data = (const net_flash_data_iv_index_t *) p_entry->data;
                break;
            case FLASH_HANDLE_SEQNUM:
                p_seqnum_data = (const net_flash_data_sequence_number_t *) p_entry->data;
                break;
            default:
                break;
        }
        p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, p_entry);
    }

    if (p_iv_index_data == NULL || p_seqnum_data == NULL)
    {
        /* Need both data types to consider the storage valid. Reset state and flash both. */
        m_net_state.seqnum = 0;
        m_net_state.iv_index = 0;
        m_net_state.iv_update.state = NET_STATE_IV_UPDATE_NORMAL;
        flash_store_iv_index();
    }
    else
    {
        m_net_state.iv_index = p_iv_index_data->iv_index;
        m_net_state.iv_update.state = (net_state_iv_update_t) p_iv_index_data->iv_update_in_progress;

        if (((const void *) p_seqnum_data < (const void *) p_iv_index_data) && !p_iv_index_data->iv_update_in_progress)
        {
            /* At the end of the IV update state, we always flash the IV index data first, then flash
            * the sequence number after. If we found the sequence number entry before the IV index
            * entry that marked the end of the IV update procedure, we should reset the sequence
            * number, as it means we failed to flash the new sequence number after an IV update.
            */
            m_net_state.seqnum = 0;
        }
        else
        {
            m_net_state.seqnum = *p_seqnum_data;
        }
    }

    /* Regardless of flash contents, we must allocate the next block of sequence numbers before we
     * start sending. We set the seqnum max to the seqnum, and trigger the allocation, so no
     * sequence numbers are spent before we get the allocation stored. */
    m_net_state.seqnum_max_available = m_net_state.seqnum;
    seqnum_block_allocate();
    bearer_event_critical_section_end();
}

void net_state_reset(void)
{
    if (flash_manager_entry_invalidate(&m_flash_manager, FLASH_HANDLE_SEQNUM) == NRF_SUCCESS &&
        flash_manager_entry_invalidate(&m_flash_manager, FLASH_HANDLE_IV_INDEX) == NRF_SUCCESS)
    {
        memset(&m_net_state, 0, sizeof(m_net_state));
        m_iv_state_set = false;
    }
    else
    {
        /* try again later */
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_available,
                                                 .p_args = net_state_reset};
        flash_manager_mem_listener_register(&mem_listener);
    }
    seqnum_block_allocate();
}

const void * net_state_flash_area_get(void)
{
#ifdef NET_FLASH_AREA_LOCATION
    return (const void *) NET_FLASH_AREA_LOCATION;
#else
    /* Default to putting the area directly before the recovery page */
    return (((const uint8_t *) flash_manager_recovery_page_get()) - (NET_FLASH_PAGE_COUNT * PAGE_SIZE));
#endif
}

#else /* PERSISTENT_STORAGE*/
static void flash_store_iv_index(void)
{

}
static void seqnum_block_allocate(void)
{

}
void net_state_recover_from_flash(void)
{
}
void net_state_reset(void)
{
    memset(&m_net_state, 0, sizeof(m_net_state));
    m_iv_state_set = false;
    RESET_SEQNUM_MAX();
}
const void * net_state_flash_area_get(void)
{
    return 0;
}
#endif /* PERSISTENT_STORAGE*/

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
    m_test_mode = false;

#if PERSISTENT_STORAGE
    init_flash_storage();
#else
    RESET_SEQNUM_MAX();
#endif
}

void net_state_enable(void)
{
    if (!m_enabled)
    {
        m_mesh_evt_handler.evt_cb = mesh_evt_handler;
        nrf_mesh_evt_handler_add(&m_mesh_evt_handler);
        timer_sch_schedule(&m_iv_update_timer);
        m_enabled = true;
    }
}

uint32_t net_state_seqnum_alloc(uint32_t * p_seqnum)
{
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

        if (!ivu_triggered && m_net_state.seqnum >= m_net_state.seqnum_max_available - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD)
        {
            seqnum_block_allocate();
        }
        /* Get the sequence number after doing the state updates, as they might
         * trigger changes to it. */
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        *p_seqnum = m_net_state.seqnum++;
        _ENABLE_IRQS(was_masked);

        return NRF_SUCCESS;
    }
    else
    {
        seqnum_block_allocate();
        return NRF_ERROR_FORBIDDEN;
    }
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
        flash_store_iv_index();
        iv_index_notify(NULL, m_net_state.iv_index - 1, m_net_state.iv_update.state);
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
    m_test_mode = test_mode_on;
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

uint32_t net_state_iv_index_set(uint32_t iv_index, bool iv_update)
{
    if (m_iv_state_set)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        m_iv_state_set = true;
        m_net_state.iv_index = iv_index;
        m_net_state.iv_update.state =
            iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL;
        flash_store_iv_index();

        /* Force reseting of sequence numbers upon IV index set */
        m_net_state.seqnum = 0;
        RESET_SEQNUM_MAX();
        seqnum_block_allocate();

        return NRF_SUCCESS;
    }
}
