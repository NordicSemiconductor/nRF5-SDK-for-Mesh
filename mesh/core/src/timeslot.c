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
#include "timeslot.h"

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_soc.h"

#include "timer.h"
#include "utils.h"
#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "bearer_handler.h"
#include "debug_pins.h"

/** Attributes to pass to the timer module when ordering the end timer callback. */
#define TIMESLOT_END_TIMER_ATTRIBUTES    ((timer_attr_t) (TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL))

#define RTC_MAX_TIME_TICKS               (0x1000000)              /**< RTC-clock rollover time. */
#define RTC_MAX_TIME_TICKS_MASK          (RTC_MAX_TIME_TICKS - 1) /**< RTC-clock rollover time mask. */

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

/** Time spent in the end timer handler. */
#if defined(HOST)
#define TIMESLOT_END_TIMER_OVERHEAD_US (0)
#elif defined(NRF51)
#define TIMESLOT_END_TIMER_OVERHEAD_US  (55)
#elif defined(NRF52_SERIES)
#define TIMESLOT_END_TIMER_OVERHEAD_US  (20)
#else
#error "Unknown platform"
#endif
/*****************************************************************************
* Local type definitions
*****************************************************************************/
/**
 * Types of forced command sent to the signal handler.
 */
typedef enum
{
    TS_FORCED_COMMAND_NONE,     /** No command for the signal handler. */
    TS_FORCED_COMMAND_STOP,     /** Stop the current timeslot, and don't order a new one. */
    TS_FORCED_COMMAND_RESTART,  /** Stop the current timeslot, and ordern a new one as early as possible */
} ts_forced_command_t;

typedef struct
{
    timestamp_t                              length_us;             /**< Length of current timeslot in microseconds (including extensions). */
    timestamp_t                              start_time_us;         /**< Start time for current timeslot in microseconds. */
    bool                                     in_callback;           /**< Code is in callback-context. */
    bool                                     in_progress;           /**< A timeslot is currently in progress. */
    bool                                     count_slot_delta;      /**< Whether we should account for the time between timeslots when getting the start timer. */
    uint32_t                                 extend_count;          /**< Number of extension attempts made this slot. */
    uint32_t                                 successful_extensions; /**< Number of successful extension attempts made this slot. */
    nrf_radio_signal_callback_return_param_t signal_ret_param;      /**< Return parameter for SD radio signal handler. */
} timeslot_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static nrf_radio_request_t m_radio_request_earliest;  /**< Timeslot earliest request, used to get the timeslots from the Softdevice. */
static timeslot_t          m_current_timeslot;        /**< Current timeslot's parameters. */
static ts_forced_command_t m_timeslot_forced_command; /**< Forced command, checked in radio signal callback. */
static bool                m_state_lock;              /**< State lock, prevents the timeslot from ending. */

static uint32_t            m_lfclk_ppm;               /**< The set drift accuracy for the LF clock source. */
/*****************************************************************************
* Static Functions
*****************************************************************************/
static void end_timer_handler(timestamp_t timestamp);

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
static inline timestamp_t get_end_time(const timeslot_t* p_timeslot)
{
    return (p_timeslot->start_time_us + p_timeslot->length_us - TIMESLOT_END_SAFETY_MARGIN_US -
            TIMESLOT_END_TIMER_OVERHEAD_US - end_timer_drift_margin(p_timeslot));
}

/** Check whether we can extend the given timeslot any more. */
static inline bool can_extend(const timeslot_t* p_timeslot)
{
    return (p_timeslot->signal_ret_param.params.extend.length_us + p_timeslot->length_us <= TIMESLOT_MAX_LENGTH_US &&
            p_timeslot->signal_ret_param.params.extend.length_us >= TIMESLOT_EXTEND_LENGTH_MIN_US);
}

/**
 * At the start of each timeslot, we sample the RTC to see when the timeslot
 * was started. This function handles the timeslot start time setting with
 * wraparound.
 */
static void start_time_update(timeslot_t* p_timeslot)
{
    static uint64_t s_last_rtc_value = 0;
    uint32_t delta_time_us;
    uint64_t rtc_time = NRF_RTC0->COUNTER;

    if (p_timeslot->count_slot_delta)
    {
        p_timeslot->count_slot_delta = false;
        delta_time_us = p_timeslot->length_us;
    }
    else
    {
        /* The RTC rolls over at 0x1000000, so we can't rely on the data type for rollover logic. */
        delta_time_us = HAL_RTC_TICKS_TO_US((rtc_time - s_last_rtc_value + RTC_MAX_TIME_TICKS) & RTC_MAX_TIME_TICKS_MASK);
    }

    p_timeslot->start_time_us += delta_time_us;
    s_last_rtc_value = rtc_time;
}


static void on_ts_begin(timeslot_t* p_timeslot)
{
    DEBUG_PIN_TIMESLOT_ON(DEBUG_PIN_TS_TIMESLOT_OPERATION);
    /* Stop ordering more extensions */
    p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    p_timeslot->in_progress = true;

    /* Notify listeners */
    timer_on_ts_begin(p_timeslot->start_time_us);
    bearer_handler_on_ts_begin();

    /* Order the timer that will cancel the timeslot. */
    NRF_MESH_ERROR_CHECK(timer_order_cb(TIMER_INDEX_TS_END,
                                        get_end_time(p_timeslot),
                                        end_timer_handler,
                                        TIMESLOT_END_TIMER_ATTRIBUTES));
}

static void on_ts_end(timeslot_t* p_timeslot)
{
    /* Only notify other modules if they actually got the on_ts_begin() signal. */
    if (p_timeslot->in_progress)
    {
        bearer_handler_on_ts_end();
        timer_on_ts_end(get_end_time(p_timeslot));
        p_timeslot->in_progress = false;
    }

    m_state_lock = false;
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_TIMESLOT_OPERATION);
    DEBUG_PIN_TIMESLOT_OFF(DEBUG_PIN_TS_IN_TIMESLOT);
}

static void handle_forced_command(timeslot_t* p_timeslot)
{
    switch (m_timeslot_forced_command)
    {
        case TS_FORCED_COMMAND_STOP:
            p_timeslot->signal_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            p_timeslot->count_slot_delta = true;
            break;

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

    start_time_update(p_timeslot);

    const timestamp_t prev_len = p_timeslot->length_us;
    p_timeslot->length_us = m_radio_request_earliest.params.earliest.length_us;

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

/** Timeslot end callback. */
static void end_timer_handler(timestamp_t timestamp)
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
            NRF_MESH_ASSERT(sd_radio_session_close() == NRF_SUCCESS);
            break;
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_MESH_ASSERT(false);
            break;

        /* Our request for a long timeslot failed, let's try to re-order a
         * short one. */
        case NRF_EVT_RADIO_BLOCKED:
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

                timer_event_handler();
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
        uint32_t time_now = timer_now();
        m_current_timeslot.length_us = time_now - m_current_timeslot.start_time_us;

        /* The forced commands always ends the timeslot. */
        handle_forced_command(&m_current_timeslot);
    }

    if (m_current_timeslot.signal_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_END ||
        m_current_timeslot.signal_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END)
    {
        on_ts_end(&m_current_timeslot);
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

    m_current_timeslot.in_callback = false;
    m_current_timeslot.in_progress = false;
    m_current_timeslot.length_us = 0;
    m_current_timeslot.count_slot_delta = true;
    m_current_timeslot.start_time_us = 0;
}

uint32_t timeslot_start(void)
{
    uint32_t status = sd_radio_session_open(radio_signal_callback);
    if (status == NRF_SUCCESS)
    {
        status = sd_radio_request(&m_radio_request_earliest);
    }
    return status;
}

void timeslot_stop(void)
{
    m_timeslot_forced_command = TS_FORCED_COMMAND_STOP;
    timeslot_trigger();
}

void timeslot_restart(void)
{
    m_timeslot_forced_command = TS_FORCED_COMMAND_RESTART;
    timeslot_trigger();
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
            (m_timeslot_forced_command == TS_FORCED_COMMAND_STOP) ||
            (m_timeslot_forced_command == TS_FORCED_COMMAND_RESTART));
}


timestamp_t timeslot_start_time_get(void)
{
    return m_current_timeslot.start_time_us;
}

timestamp_t timeslot_end_time_get(void)
{
    if (!m_current_timeslot.in_progress)
    {
        return 0;
    }

    return get_end_time(&m_current_timeslot);
}

timestamp_t timeslot_remaining_time_get(void)
{
    if (!m_current_timeslot.in_progress)
    {
        return 0;
    }
    return timer_diff(get_end_time(&m_current_timeslot), timer_now());
}

bool timeslot_is_in_ts(void)
{
    return m_current_timeslot.in_progress;
}

bool timeslot_is_in_cb(void)
{
    return m_current_timeslot.in_callback;
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
