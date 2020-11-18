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

#ifndef MODEL_COMMON_H__
#define MODEL_COMMON_H__

#include <stdint.h>

#include "nrf_mesh_config_core.h"
#include "utils.h"
#include "app_timer.h"
#include "timer_scheduler.h"
#include "access.h"

/**
 * @defgroup MODEL_COMMON Common APIs for models.
 * @ingroup MESH_API_GROUP_MODELS
 * Contains common defines, structures, and functions used by the Mesh Models.
 *
 * @{
 */
/**
 * The number of the Generic Default Transition Time Server (independent, root only) instances used
 * by the application.
 */
#ifndef GENERIC_DTT_SERVER_INSTANCES_MAX
#define GENERIC_DTT_SERVER_INSTANCES_MAX  (0)
#endif

/**
 * The number of the Generic Level Server (independent, root only) instances used by the application.
 */
#ifndef GENERIC_LEVEL_SERVER_INSTANCES_MAX
#define GENERIC_LEVEL_SERVER_INSTANCES_MAX  (0)
#endif

/**
 * The number of the Generic OnOff Server (independent, root only) instances used by the application.
 */
#ifndef GENERIC_ONOFF_SERVER_INSTANCES_MAX
#define GENERIC_ONOFF_SERVER_INSTANCES_MAX  (0)
#endif

/**
 * The number of the Light Lightness Setup Server instances used by the application.
 */
#ifndef LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX
#define LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX  (0)
#endif

/**
 * The number of the Light LC Setup Server instances used by the application.
 */
#ifndef LIGHT_LC_SETUP_SERVER_INSTANCES_MAX
#define LIGHT_LC_SETUP_SERVER_INSTANCES_MAX (0)
#endif

/**
 * The number of the Light CTL Setup Server instances used by the application
 */
#ifndef LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX
#define LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX (0)
#endif

/**
 * The number of the Scene Setup Server instances used by the application.
 */
#ifndef SCENE_SETUP_SERVER_INSTANCES_MAX
#define SCENE_SETUP_SERVER_INSTANCES_MAX (0)
#endif

/** Transition time value to indicate unknown transition time */
#define MODEL_TRANSITION_TIME_UNKNOWN               (UINT32_MAX)


/** Transition time value to indicate the default transition time should be used */
#ifndef MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT
#define MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT      (SEC_TO_US(30))
#endif

/** Product-specific transition limitations to align transition data regarding product abilities. */
#ifndef TRANSITION_STEP_MIN_MS
#define TRANSITION_STEP_MIN_MS                  (45)
#endif
/** Maximum value of transition time (in ms) possible in steps of 100 ms */
#define TRANSITION_TIME_STEP_100MS_MAX          (6200ul)
/** Maximum value of transition time (in ms) possible in steps of 1 s */
#define TRANSITION_TIME_STEP_1S_MAX             (SEC_TO_MS(62ul))
/** Maximum value of transition time (in ms) possible in steps of 10 s */
#define TRANSITION_TIME_STEP_10S_MAX            (SEC_TO_MS(620ul))
/** Maximum value of transition time (in ms) possible in steps of 10 min */
#define TRANSITION_TIME_STEP_10M_MAX            (MIN_TO_MS(620ul))
/** Max value of encoded transition time step value */
#define TRANSITION_TIME_MAX                     (0x3E)
/** Unknown encoded transition time value */
#define TRANSITION_TIME_UNKNOWN                 (0x3F)
/** Maximum permissible transition time in milliseconds */
#define TRANSITION_TIME_MAX_MS                  (TRANSITION_TIME_STEP_10M_MAX)

/** Delay field step factor in milliseconds */
#define DELAY_TIME_STEP_FACTOR_MS               (5)
/** Maximum encoded value of the delay field */
#define DELAY_TIME_STEP_MAX                     (0xFF)
/** Maximum permisible delay time in milliseconds */
#define DELAY_TIME_MAX_MS                       (DELAY_TIME_STEP_MAX * DELAY_TIME_STEP_FACTOR_MS)

/** Minimum permissible timeout (1 ms) in RTC ticks for model timer */
#define MODEL_TIMER_TIMEOUT_MIN_TICKS           (APP_TIMER_TICKS(1))
/** Minimum permissible timeout in milliseconds for model timer */
#define MODEL_TIMER_TIMEOUT_MIN_US              (MODEL_TIMER_PERIOD_US_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS))
/** Maximum timout ticks, as per app_timer.h */
#define MODEL_TIMER_MAX_TIMEOUT_TICKS           (APP_TIMER_MAX_CNT_VAL)
/** Get period in milliseconds for given number of ticks */
#define MODEL_TIMER_PERIOD_MS_GET(TICKS)        ((1000ul * (TICKS)) / APP_TIMER_CLOCK_FREQ)
/** Get period in microseconds for given number of ticks */
#define MODEL_TIMER_PERIOD_US_GET(TICKS)        ((1000ul * 1000ul * (TICKS)) / APP_TIMER_CLOCK_FREQ)
/** Get number of model timer ticks for the given time period in milliseconds */
#define MODEL_TIMER_TICKS_GET_MS(MS_TIME)                                           \
            ((uint64_t)ROUNDED_DIV((MS_TIME) * (uint64_t)APP_TIMER_CLOCK_FREQ,      \
             1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))
/** Get number of model timer ticks for the given time period in microseconds */
#define MODEL_TIMER_TICKS_GET_US(US_TIME)                                           \
            ((uint64_t)ROUNDED_DIV(                                                 \
            (US_TIME) * (uint64_t)APP_TIMER_CLOCK_FREQ,                             \
            1000 * 1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))

/** Generic Transition parameters for the model messages. */
typedef struct
{
    uint32_t transition_time_ms;                            /**< Transition time value in milliseconds */
    uint32_t delay_ms;                                         /**< Message execution delay in milliseconds */
} model_transition_t;

/** Structure for tracking TID expiry for the models */
typedef struct
{
    /** Source address */
    uint16_t src;
    /** Destination address */
    uint16_t dst;
    /** Previously received Opcode */
    uint32_t message_id;
    /** Previously received TID */
    uint8_t old_tid;
    /** New transaction indicator flag */
    bool new_transaction;
    /** Expiration timer instance */
    timer_event_t tid_expiry_timer;
} tid_tracker_t;

/** Timer modes. */
typedef enum
{
    /** The timer will expire only once. */
    MODEL_TIMER_MODE_SINGLE_SHOT,
    /** The timer will restart each time it expires. */
    MODEL_TIMER_MODE_REPEATED
} model_timer_mode_t;

 /** Timer callback prototype */
typedef void(*model_timer_cb_t)(void * p_context);

/** Structure for model timers */
typedef struct
{
    /** Timer mode : Single shot, repeated. */
    model_timer_mode_t mode;
    /** Timeout in number of RTC ticks */
    uint64_t timeout_rtc_ticks;
    /** context pointer for the timer callback */
    void * p_context;
    /** Timer callback. */
    model_timer_cb_t cb;
    /** Total rtc ticks since beginning of the timer */
    uint64_t total_rtc_ticks;

    /** APP timer instance pointer */
    app_timer_id_t const * p_timer_id;

    /** Internal variable. */
    uint64_t remaining_ticks;
    /** Internal variable. */
    uint32_t last_rtc_stamp;
    /** Internal variable. */
    bool cb_active;
    /** Internal variable. */
    bool timer_running;
} model_timer_t;

/** Default transition time get callback prototype
 *
 * @param[in]  element_index    Index of the model element that is requesting default transition
 *                              time value.
 *
 * @retval Returns value of default transition time in milliseconds, zero, if unavailable.
 */
 typedef uint32_t (*default_transition_time_value_get_cb_t)(uint16_t element_index);

/**
 * Gets the decoded value of the transition time in milliseconds.
 *
 * @param[in] enc_transition_time Encoded value of the transition time as specified in the
 *            @tagMeshMdlSp.
 *
 * @returns Transition time in milliseconds.
 */
uint32_t model_transition_time_decode(uint8_t enc_transition_time);

/**
 * Gets the encoded value of the transition time as specified in the @tagMeshMdlSp
 * Note that the provided value will be rounded down to the nearest possible representation.
 *
 * @param[in] transition_time Transition time in milliseconds.
 *
 * @returns Encoded value of the transition time as specified in the @tagMeshMdlSp.
 */
uint8_t model_transition_time_encode(uint32_t transition_time);

/**
 * Validates the given transition time value.
 *
 * @param[in]  enc_transition_time  Encoded transition time value
 *
 * @retval True                     If encoded transition time is a valid value for the models.
 * @retval False                    If encoded transition time is invalid for the models and cannot be set.
 *
 */
bool model_transition_time_is_valid(uint8_t enc_transition_time);

/**
 * Gets the decoded value of the delay time in milliseconds
 *
 * @param[in] enc_delay Encoded value of the delay time as specified in the @tagMeshMdlSp.
 *
 * @returns delay time in milliseconds.
 */
uint32_t model_delay_decode(uint8_t enc_delay);

/**
 * Gets the encoded value of the delay time as specified in the @tagMeshMdlSp.
 * Note that the provided value will be rounded down to the nearest possible representation.
 *
 * @param[in] delay Delay time in milliseconds.
 *
 * @returns Encoded value of the delay time as specified in the @tagMeshMdlSp.
 */
uint8_t model_delay_encode(uint32_t delay);

/** Checks if the given message parameters represents a new transaction.
 *
 * The transaction is considered either new or same as previous in the context of
 * a given message ID, TID, access meta data (source address and destination
 * address) and timeout of 6 seconds as specified by @tagMeshMdlSp. This API
 * is used by the model interfaces to reject duplicate transactions.
 *
 * @note User application should use @ref model_transaction_is_new() API to check is the received
 * message callback represents a new transaction.
 *
 * @param[in] p_tid_tracker     Pointer to the tid tracker structure.
 * @param[in] p_meta            Access message metadata containing source and destination addresses.
 * @param[in] message_id        Any kind of unique identifier (e.g opcode) for a given type of message.
 * @param[in] tid               Received TID value.
 *
 * @retval    True              If transaction is new.
 * @retval    False             If transaction is same as the previous transaction.
 */
bool model_tid_validate(tid_tracker_t * p_tid_tracker, const access_message_rx_meta_t * p_meta,
                        uint32_t message_id, uint8_t tid);

/**
 * Checks if given TID tracker instance has recorded a new transaction.
 *
 * This API can be used by the user application to determine if the received message callback
 * represents a new transaction.
 *
 * @note The @ref model_tid_validate API is always called by the model interface internally to
 * keep @ref tid_tracker_t context updated.
 *
 * @param[in] p_tid_tracker     Pointer to the tid tracker structure.
 *
 * @retval    True              If transaction is new.
 * @retval    False             If transaction is same as the previous transaction.
 */
bool model_transaction_is_new(tid_tracker_t * p_tid_tracker);

/**
 * Schedules a model timer for a given interval.
 *
 * This API uses APP_TIMER internally for the managing timeouts.
 * The timing resolution is equal to the resolution offered by period corresponding to
 * APP_TIMER_CLOCK_FREQ. The minimum allowed timeout in ticks is specified by @ref MODEL_TIMER_TIMEOUT_MIN_TICKS.
 *
 * @param[in] p_timer               Pointer to the @ref model_timer_t structure.
 *
 * @retval NRF_SUCCESS              If the timer was successfully scheduled.
 * @retval NRF_ERROR_NULL           If `p_timer` or the timer callback is null.
 * @retval NRF_ERROR_INVALID_PARAM  If specified timout is too short.
 */
uint32_t model_timer_schedule(model_timer_t * p_timer);

/**
 * Aborts the currently scheduled timer
 *
 * @param[in] p_timer               Pointer to the @ref model_timer_t structure
 */
void model_timer_abort(model_timer_t * p_timer);

/**
 * Returns the total elapsed rtc ticks since the last call to @ref model_timer_schedule() API.
 *
 * @param[in] p_timer       Pointer to the @ref model_timer_t structure.
 *
 * @retval  0               If timer is not running.
 * @retval  ticks           Total elapsed rtc ticks, since the last call to @ref model_timer_schedule() API.
 */
uint64_t model_timer_elapsed_ticks_get(model_timer_t * p_timer);

/**
 * Returns whether the specified timer is running.
 *
 * If the current mode is MODEL_TIMER_MODE_REPEATED, returns true
 * until model_timer_abort is called. If the current mode is MODLE_TIMER_MODE_SINGLE_SHOT, returns false after
 * the timeout completes. The behavior also applies when this function is called from within the
 * timer callback.
 *
 * @param[in] p_timer       Pointer to the @ref model_timer_t structure.
 *
 * @retval true             If the timer is running
 * @retval false            If the timer is not running
 */
bool model_timer_is_running(model_timer_t * p_timer);

/**
 * Creates a model timer.
 *
 * This model timer implementation uses App Timer which is based on RTC. The timing parameters
 * of this model timer are specified in number of ticks.
 *
 * @param[in]  p_timer      Pointer to @ref model_timer_t structure.
 *
 * @retval NRF_SUCCESS              If the timer was successfully created.
 * @retval NRF_ERROR_NULL           If p_timer is NULL.
 * @retval NRF_ERROR_INVALID_PARAM  If a parameter was invalid.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer module has not been initialized or
 *                                  the timer is running.
 */
uint32_t model_timer_create(model_timer_t * p_timer);

/** @} end of MODEL_COMMON */

#endif /* MODEL_COMMON_H__ */

