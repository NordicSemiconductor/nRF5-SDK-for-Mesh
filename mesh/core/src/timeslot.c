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
#include "timeslot.h"

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_soc.h"

#include "utils.h"
#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "bearer_handler.h"
#include "debug_pins.h"
#include "event.h"

#if NRF_MODULE_ENABLED(NRF_SDH)
#include "nrf_sdh.h"
#endif

/** Worst case HFCLOCK drift allowed when running the Softdevice, according to
 * product specifications:
 * nRF51822 PS v3.2, ch. 8.1.2
 * nRF52832 PS v1.1, ch. 19.4.2
 */
#define HFCLOCK_PPM_WORST_CASE           (40)

/** Timeout to use for the request of earliest timeslot from the SD. Allows the
 * Softdevice to finish an advertisement event (including jitter) before timing
 * out. */
#define TIMESLOT_REQ_EARLIEST_TIMEOUT_US (15000)
#define TIMESLOT_EXTEND_TINY_US          (1000)                   /**< Length of a tiny increment to the timeslot, used to check whether we can get more time. */

/*****************************************************************************
* Local type definitions
*****************************************************************************/
/** State of the timeslot radio session. */
typedef enum
{
    TS_SESSION_STATE_CLOSED, /**< The timeslot radio session hasn't been opened */
    TS_SESSION_STATE_OPEN, /**< The timeslot radio session is open, but not running. */
    TS_SESSION_STATE_RUNNING, /**< The timeslot radio session is open and running. */
} ts_session_state_t;

/**
 * Types of forced command sent to the signal handler.
 */
typedef enum
{
    TS_FORCED_COMMAND_NONE,     /** No command for the signal handler. */
    TS_FORCED_COMMAND_STOP,     /** Stop the current timeslot, and don't order a new one. */
    TS_FORCED_COMMAND_RESTART,  /** Stop the current timeslot, and ordern a new one as early as possible */
    TS_FORCED_COMMAND_RESTART_HIGH,  /** Stop the current timeslot, and ordern a new one with high priority as early as possible */
} ts_forced_command_t;

typedef struct
{
    ts_timestamp_t                           length_us;             /**< Length of current timeslot in microseconds (including extensions). */
    bool                                     in_callback;           /**< Code is in callback-context. */
    bool                                     in_progress;           /**< A timeslot is currently in progress. */
    bool                                     extend;                /**< Whether the timeslot should be extended or not. */
    uint32_t                                 extend_count;          /**< Number of extension attempts made this slot. */
    uint32_t                                 successful_extensions; /**< Number of successful extension attempts made this slot. */
    nrf_radio_signal_callback_return_param_t signal_ret_param;      /**< Return parameter for SD radio signal handler. */
    ts_session_state_t                       session_state;         /**< State of the SD radio session */
} timeslot_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static nrf_radio_request_t m_radio_request_earliest;  /**< Timeslot earliest request, used to get the timeslots from the Softdevice. */
static timeslot_t          m_current_timeslot;        /**< Current timeslot's parameters. */
static ts_forced_command_t m_timeslot_forced_command; /**< Forced command, checked in radio signal callback. */
static bool                m_state_lock;              /**< State lock, prevents the timeslot from ending. */
static uint32_t            m_ts_count;                /**< Number of timeslots since device started. */

static uint32_t            m_lfclk_ppm;               /**< The set drift accuracy for the LF clock source. */
/*****************************************************************************
* Static Functions
*****************************************************************************/
#if NRF_MODULE_ENABLED(NRF_SDH)
static bool                m_sd_disable_pending;      /**< Someone is attempting to disable the softdevice, but we're waiting for our timeslot to end first. */
static bool sdh_req_handler(nrf_sdh_req_evt_t request, void * p_context);

NRF_SDH_REQUEST_OBSERVER(mesh_sdh_req_observer, 0) = {
    .handler = sdh_req_handler,
};

static bool sdh_req_handler(nrf_sdh_req_evt_t request, void * p_context)
{
    /* The SDH asks us if we're ready to disable the softdevice. We should only allow it to do so if
     * the timeslots are fully disabled to avoid state desync with the SD. */
    if (request == NRF_SDH_EVT_DISABLE_REQUEST &&
        m_current_timeslot.session_state != TS_SESSION_STATE_CLOSED)
    {
        if (m_current_timeslot.session_state == TS_SESSION_STATE_RUNNING)
        {
            timeslot_stop();
        }
        /* Note: If the current session state is OPEN, an IDLE SoC event is pending, and we'll
         * eventually stop anyway. */
        m_sd_disable_pending = true;
        return false;
    }
    return true;
}
#endif

static void end_timer_handler(ts_timestamp_t timestamp);

/**
 * Get the amount of clock drift that must be accounted for when calculating
 * the end timer (in microseconds). The Softdevice checks whether we finished
 * our timeslot on time by looking at the 32.768kHz clock, while we calculate
 * by the 16MHz clock. These drift independently, and we have to adjust for
 * this difference when setting up our end-timer.
 */
static inline uint32_t end_timer_drift_margin(const timeslot_t* p_timeslot)
{
    return (p_timeslot->length_us * (m_lfclk_ppm + HFCLOCK_PPM_WORST_CASE)) / 1000000;
}

/** Get the timeslot end timer timestamp. */
static inline ts_timestamp_t get_end_time(const timeslot_t* p_timeslot)
{
    return (p_timeslot->length_us - TIMESLOT_END_SAFETY_MARGIN_US -
            TIMESLOT_END_TIMER_OVERHEAD_US - end_timer_drift_margin(p_timeslot));
}

/** Check whether we can extend the given timeslot any more. */
static inline bool can_extend(const timeslot_t* p_timeslot)
{
    return (p_timeslot->signal_ret_param.params.extend.length_us + p_timeslot->length_us <= TIMESLOT_MAX_LENGTH_US &&
            p_timeslot->signal_ret_param.params.extend.length_us >= TIMESLOT_EXTEND_LENGTH_MIN_US);
}

static void radio_request_params_reset(void)
{
    /* Reset the timeslot priority, so the next timeslot will default to normal */
    m_radio_request_earliest.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_radio_request_earliest.params.earliest.length_us  = TIMESLOT_BASE_LENGTH_LONG_US;
    m_radio_request_earliest.params.earliest.timeout_us = TIMESLOT_REQ_EARLIEST_TIMEOUT_US;
}

static void on_ts_begin(timeslot_t* p_timeslot)
{
    /* Stop ordering more extensions */
    p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

    p_timeslot->in_progress = true;
    m_ts_count++;

    /* Notify listeners */
    ts_timer_on_ts_begin();
    bearer_handler_on_ts_begin();

    /* Order the timer that will cancel the timeslot. */
    NRF_MESH_ERROR_CHECK(ts_timer_order_cb(TS_TIMER_INDEX_TS_END,
                                           get_end_time(p_timeslot),
                                           end_timer_handler));
}

static void on_ts_end(timeslot_t* p_timeslot)
{
    /* Only notify other modules if they actually got the on_ts_begin() signal. */
    if (p_timeslot->in_progress)
    {
        bearer_handler_on_ts_end();
        ts_timer_on_ts_end(get_end_time(p_timeslot));
        p_timeslot->in_progress = false;
    }

    m_state_lock = false;
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_IN_TIMESLOT);
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_HIGH_PRIORITY);
}

static void handle_forced_command(timeslot_t* p_timeslot)
{
    switch (m_timeslot_forced_command)
    {
        case TS_FORCED_COMMAND_STOP:
            p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            break;

        case TS_FORCED_COMMAND_RESTART_HIGH:
            m_radio_request_earliest.params.earliest.priority = NRF_RADIO_PRIORITY_HIGH;
            m_radio_request_earliest.params.earliest.length_us = NRF_RADIO_LENGTH_MAX_US;
            m_radio_request_earliest.params.earliest.timeout_us = NRF_RADIO_EARLIEST_TIMEOUT_MAX_US;
            p_timeslot->extend = false;
            /* Fall-through */
        case TS_FORCED_COMMAND_RESTART:
            p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            p_timeslot->signal_ret_param.params.request.p_next = &m_radio_request_earliest;
            break;

        default:
            NRF_MESH_ASSERT(false);
    }
    m_timeslot_forced_command = TS_FORCED_COMMAND_NONE;
}


/*****************************************************************************
* Radio signal handlers
*****************************************************************************/
static void handle_signal_start(timeslot_t* p_timeslot)
{
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_IN_TIMESLOT);
    p_timeslot->extend_count = 0;
    p_timeslot->successful_extensions = 0;

    const ts_timestamp_t prev_len = p_timeslot->length_us;
    p_timeslot->length_us = m_radio_request_earliest.params.earliest.length_us;

    if (p_timeslot->extend)
    {
        /* Start by extending to fit the previous length. */
        p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;

        /* If the previous slot was zero-length, extend as much as possible. */
        uint32_t extend_len = (prev_len == 0) ? TIMESLOT_MAX_LENGTH_US : prev_len;
        p_timeslot->signal_ret_param.params.extend.length_us = extend_len - p_timeslot->length_us;

        /* If the base timeslot length is already at (approximately) the length of
        * the previous timeslot, we attempt to extend by a tiny amount, to see if
        * we can increase it just a little. This allows us to increase the
        * timeslot when the boundaries grow. */
        if (p_timeslot->signal_ret_param.params.extend.length_us < TIMESLOT_EXTEND_TINY_US || extend_len < p_timeslot->length_us)
        {
            p_timeslot->signal_ret_param.params.extend.length_us = TIMESLOT_EXTEND_TINY_US;
        }
    }
    else
    {
        DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_HIGH_PRIORITY);
        on_ts_begin(p_timeslot);
    }

    radio_request_params_reset();
    /* Default back to extending the next timeslot */
    p_timeslot->extend = true;
}

static void handle_extend_end(timeslot_t* p_timeslot, bool success)
{
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_EXTEND_HANDLER);
    if (success)
    {
        DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_EXTEND_SUCCEEDED);
        p_timeslot->successful_extensions++;
        p_timeslot->length_us += p_timeslot->signal_ret_param.params.extend.length_us;
        DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_EXTEND_SUCCEEDED);
    }
    p_timeslot->extend_count++;

    /* Cut the extension time in half. */
    p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
    p_timeslot->signal_ret_param.params.extend.length_us /= 2;

    switch (p_timeslot->extend_count)
    {
        case 1:
            if ((p_timeslot->signal_ret_param.params.extend.length_us == TIMESLOT_EXTEND_TINY_US / 2) && !success)
            {
                /* If we did a tiny extension on the first attempt, but failed, we
                 * should just accept our short timeslot. */
                p_timeslot->signal_ret_param.params.extend.length_us = 0;
            }
            else if (p_timeslot->signal_ret_param.params.extend.length_us <= TIMESLOT_EXTEND_LENGTH_MIN_US)
            {
                /* The second extension attempt should be made even if our
                 * extension is too short. */
                p_timeslot->signal_ret_param.params.extend.length_us = TIMESLOT_EXTEND_LENGTH_MIN_US;
            }
            break;

        case 2:
            /* The third time we extend is a little special. If we are able to extend
               the first time, but not the second, it's unlikely that the SD has
               changed its upper time limit, so we stop attempting more extensions. If
               we're able to reserve more time than we got in the previous slot, we
               should be greedy, and attempt to reserve as much as possible. */
            if (p_timeslot->successful_extensions == 2)
            {
                /* Since we're able to get 150% of the previous timeslot, the SD
                   has relaxed the upper limit. Let's ask for as much as possible,
                   to see where the new limit is. */
                p_timeslot->signal_ret_param.params.extend.length_us = TIMESLOT_MAX_LENGTH_US - p_timeslot->length_us;
            }
            else if (p_timeslot->successful_extensions == 1 && !success)
            {
                /* If we can't get a length that is 150% of the previous, there's a
                   high likelyhood that the SD activity didn't change, so we'll
                   just start the timeslot operation. */
                p_timeslot->signal_ret_param.params.extend.length_us = 0;
            }
            break;
    }

    if (!can_extend(p_timeslot))
    {
        /* We've reached some boundary condition for extensions, and will
           start normal timeslot operation instead. */
        on_ts_begin(p_timeslot);
    }
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_EXTEND_HANDLER);
}

/*****************************************************************************
* Callback functions
*****************************************************************************/

#if !defined(_lint)
/* Dummy implementation of the state listener, in case no one has implemented one. */
void __attribute__((weak)) timeslot_state_listener(bool timeslot_active) {}
#endif

/** Timeslot end callback. */
static void end_timer_handler(ts_timestamp_t timestamp)
{
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_END_TIMER_HANDLER);
    m_current_timeslot.signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    m_current_timeslot.signal_ret_param.params.request.p_next = &m_radio_request_earliest;
    /* Attempt the long timeslot first. If this creates a BLOCKED event, we'll
     * move to the short version. */
    if (m_current_timeslot.length_us > TIMESLOT_BASE_LENGTH_SHORT_US)
    {
        m_radio_request_earliest.params.earliest.length_us = TIMESLOT_BASE_LENGTH_LONG_US;
    }
    else
    {
        m_radio_request_earliest.params.earliest.length_us = TIMESLOT_BASE_LENGTH_SHORT_US;
    }
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_END_TIMER_HANDLER);
}

/**
* Timeslot related events callback
*   Called whenever the softdevice tries to change the original course of actions
*   related to the timeslots.
*/
void timeslot_sd_event_handler(uint32_t evt)
{
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_SD_EVT_HANDLER);
    switch (evt)
    {
        case NRF_EVT_RADIO_SESSION_IDLE:
            /* As the SD events aren't handled atomically, we could potentially run into this scenario:
             * - stop the timeslot -> SESSION_IDLE event pending, session_state is OPEN
             * - start the timeslot again from the same context -> session_state is RUNNING
             * - The SESSION_IDLE event is handled.
             * Closing the radio session now leads to unexpected behavior, as the user restarted
             * the timeslot already, expecting it to remain active.
             */
            if (m_current_timeslot.session_state == TS_SESSION_STATE_OPEN)
            {
                NRF_MESH_ASSERT(sd_radio_session_close() == NRF_SUCCESS);
            }
            break;

        case NRF_EVT_RADIO_SESSION_CLOSED:
        {
            m_current_timeslot.session_state = TS_SESSION_STATE_CLOSED;

#if NRF_MODULE_ENABLED(NRF_SDH)
            if (m_sd_disable_pending)
            {
                m_sd_disable_pending = false;

                /* If the nRF5 SDK NRF_SDH module is enabled, but its files aren't linked, the
                 * linker will complain that this function doesn't exist. Either add the SDH-files
                 * from the nRF5 SDK, or disable the SDH module. */
                nrf_sdh_request_continue();
            }
#endif
            bearer_handler_on_ts_session_closed();
            break;
        }
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_MESH_ASSERT(false);
            break;

        /* Try to re-order a short one timeslot to avoid the block */
        case NRF_EVT_RADIO_BLOCKED:
            radio_request_params_reset();
            m_radio_request_earliest.params.earliest.length_us = TIMESLOT_BASE_LENGTH_SHORT_US;
            NRF_MESH_ASSERT(sd_radio_request(&m_radio_request_earliest) == NRF_SUCCESS);
            break;
        /* The softdevice revoked our timeslot before it started. Request a new one. */
        case NRF_EVT_RADIO_CANCELED:
            NRF_MESH_ASSERT(sd_radio_request(&m_radio_request_earliest) == NRF_SUCCESS);
            break;
        default:
            break;
    }
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_SD_EVT_HANDLER);
}

/** Radio signal callback handler taking care of all SD radio timeslot signals */
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    m_current_timeslot.in_callback = true;
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_SIGNAL_CALLBACK);

    /* Only treat forced commands if there's no state lock. */
    if (m_timeslot_forced_command == TS_FORCED_COMMAND_NONE || m_state_lock)
    {
        m_current_timeslot.signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

        switch (sig)
        {
            /* Management handlers: */
            case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
                handle_signal_start(&m_current_timeslot);
                break;
            case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
                handle_extend_end(&m_current_timeslot, true);
                break;
            case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
                handle_extend_end(&m_current_timeslot, false);
                break;

            /* IRQ handlers: */
            case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
                bearer_handler_radio_irq_handler();
                break;
            case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
                DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_TIMER_HANDLER);

                ts_timer_event_handler();
                DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_SD_EVT_HANDLER);
                bearer_handler_timer_irq_handler();
                DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_SD_EVT_HANDLER);

                DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_TIMER_HANDLER);
                break;

            default:
                NRF_MESH_ASSERT(false);
        }
    }
    else
    {
        /* As the forced command makes us exit the timeslot earlier than expected, we should get a real
        * timestamp for the length calculation. */
        m_current_timeslot.length_us = ts_timer_now();

        /* The forced commands always ends the timeslot. */
        handle_forced_command(&m_current_timeslot);
    }

    if (m_current_timeslot.signal_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_END ||
        m_current_timeslot.signal_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END)
    {
        on_ts_end(&m_current_timeslot);
    }

    if (m_current_timeslot.signal_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_END)
    {
        m_current_timeslot.session_state = TS_SESSION_STATE_OPEN;
    }

    m_current_timeslot.in_callback = false;
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_SIGNAL_CALLBACK);
    return &m_current_timeslot.signal_ret_param;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void timeslot_init(uint32_t lfclk_accuracy_ppm)
{
    m_radio_request_earliest.request_type               = NRF_RADIO_REQ_TYPE_EARLIEST;
#ifdef S110
    m_radio_request_earliest.params.earliest.hfclk      = NRF_RADIO_HFCLK_CFG_FORCE_XTAL;
#else
    m_radio_request_earliest.params.earliest.hfclk      = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
#endif
    m_radio_request_earliest.params.earliest.priority   = NRF_RADIO_PRIORITY_NORMAL;
    m_radio_request_earliest.params.earliest.length_us  = TIMESLOT_BASE_LENGTH_LONG_US;
    m_radio_request_earliest.params.earliest.timeout_us = TIMESLOT_REQ_EARLIEST_TIMEOUT_US;

    m_lfclk_ppm               = lfclk_accuracy_ppm;
    m_state_lock              = false;
    m_timeslot_forced_command = TS_FORCED_COMMAND_NONE;
    m_ts_count = 0;

    m_current_timeslot.in_callback = false;
    m_current_timeslot.in_progress = false;
    m_current_timeslot.length_us = 0;
    m_current_timeslot.session_state = TS_SESSION_STATE_CLOSED;
    m_current_timeslot.extend = true;
}

uint32_t timeslot_start(void)
{
    switch (m_current_timeslot.session_state)
    {
        case TS_SESSION_STATE_CLOSED:
            NRF_MESH_ERROR_CHECK(sd_radio_session_open(radio_signal_callback));
            /* fallthrough */
        case TS_SESSION_STATE_OPEN:
            NRF_MESH_ERROR_CHECK(sd_radio_request(&m_radio_request_earliest));
            m_current_timeslot.session_state = TS_SESSION_STATE_RUNNING;
            break;

        case TS_SESSION_STATE_RUNNING:
            /* already started */
            return NRF_ERROR_BUSY;
    }
    return NRF_SUCCESS;
}

void timeslot_stop(void)
{
    if (m_current_timeslot.session_state == TS_SESSION_STATE_RUNNING)
    {
        m_timeslot_forced_command = TS_FORCED_COMMAND_STOP;
        timeslot_trigger();
    }
}

void timeslot_restart(timeslot_priority_t priority)
{
    if (m_current_timeslot.session_state == TS_SESSION_STATE_RUNNING)
    {
        m_timeslot_forced_command = (priority == TIMESLOT_PRIORITY_HIGH)
                                        ? TS_FORCED_COMMAND_RESTART_HIGH
                                        : TS_FORCED_COMMAND_RESTART;
        timeslot_trigger();
    }
}

void timeslot_state_lock(bool lock)
{
    bool trigger_pending_forced_command = (m_state_lock && !lock && m_timeslot_forced_command != TS_FORCED_COMMAND_NONE);

    m_state_lock = lock;

    if (trigger_pending_forced_command)
    {
        timeslot_trigger();
    }
}

bool timeslot_end_is_pending(void)
{
    return ((m_current_timeslot.signal_ret_param.callback_action ==
             NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END) ||
            (m_current_timeslot.signal_ret_param.callback_action ==
             NRF_RADIO_SIGNAL_CALLBACK_ACTION_END) ||
            (m_timeslot_forced_command == TS_FORCED_COMMAND_STOP) ||
            (m_timeslot_forced_command == TS_FORCED_COMMAND_RESTART) ||
            (m_timeslot_forced_command == TS_FORCED_COMMAND_RESTART_HIGH));
}

ts_timestamp_t timeslot_end_time_get(void)
{
    if (!m_current_timeslot.in_progress)
    {
        return 0;
    }
    return get_end_time(&m_current_timeslot);
}

ts_timestamp_t timeslot_length_get(void)
{
    return m_current_timeslot.length_us;
}

ts_timestamp_t timeslot_remaining_time_get(void)
{
    if (!m_current_timeslot.in_progress)
    {
        return 0;
    }
    return timer_diff(get_end_time(&m_current_timeslot), ts_timer_now());
}

bool timeslot_is_in_ts(void)
{
    return m_current_timeslot.in_progress;
}

bool timeslot_is_in_cb(void)
{
    return m_current_timeslot.in_callback;
}

bool timeslot_session_is_active(void)
{
    return (m_current_timeslot.session_state != TS_SESSION_STATE_CLOSED);
}

uint32_t timeslot_count_get(void)
{
    return m_ts_count;
}

void timeslot_trigger(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (m_current_timeslot.in_progress)
    {
        (void) NVIC_SetPendingIRQ(TIMER0_IRQn);
    }
    _ENABLE_IRQS(was_masked);
}
