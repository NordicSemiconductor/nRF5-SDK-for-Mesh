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
#ifndef MESH_LOG_H__
#define MESH_LOG_H__

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"


#if defined(HOST)
    #ifdef __linux__
        #include <time.h>
    #endif /* __linux__ */
#else /* HOST */
    #include "nrf.h"
#endif /* HOST */

/**
 * @defgroup LOG Logging functions
 * @ingroup MESH_CORE
 * This module provides a common logging interface for the stack.
 * The module allows the user to choose what backend to use for logging.
 *
 * The following backends are supported:
 * * [**RTT**](https://www.segger.com/products/debug-probes/j-link/technology/real-time-transfer/general-information/):
 *   provides fast logging when using SEGGER's J-Link debuggers. This is used when
 *   `log_callback_rtt()` is passed to `__LOG_INIT()`.
 * * **Logview compatible RTT**: provides logs over RTT that can be viewed using the
 *   [Python Log Viewer](https://pythonhosted.org/logview).
 * * **Standard output**: Provides output to `stdout` when running on host. This is
 *   used when `log_callback_stdout` is passed to `__LOG_INIT()`.
 * @{
 */

/**
 * @defgroup LOG_SOURCES Log sources
 * Defines various sources for logging messages. This can be used in __LOG_INIT() to
 * filter events from different modules.
 * @{
 */

#define LOG_SRC_BEARER          (1 <<  0) /**< Receive logs from the bearer layer. */
#define LOG_SRC_NETWORK         (1 <<  1) /**< Receive logs from the network layer. */
#define LOG_SRC_TRANSPORT       (1 <<  2) /**< Receive logs from the transport layer. */
#define LOG_SRC_PROV            (1 <<  3) /**< Receive logs from the provisioning module. */
#define LOG_SRC_PACMAN          (1 <<  4) /**< Receive logs from the packet manager. */
#define LOG_SRC_INTERNAL        (1 <<  5) /**< Receive logs from the internal event module. */
#define LOG_SRC_API             (1 <<  6) /**< Receive logs from the nRF Mesh API. */
#define LOG_SRC_DFU             (1 <<  7) /**< Receive logs from the DFU module. */
#define LOG_SRC_BEACON          (1 <<  8) /**< Receive logs from the beacon module. */
#define LOG_SRC_TEST            (1 <<  9) /**< Receive logs from unit tests. */
#define LOG_SRC_ENC             (1 << 10) /**< Receive logs from the encryption module. */
#define LOG_SRC_TIMER_SCHEDULER (1 << 11) /**< Receive logs from the timer scheduler. */
#define LOG_SRC_CCM             (1 << 12) /**< Receive logs from the CCM module. */
#define LOG_SRC_ACCESS          (1 << 13) /**< Receive logs from the access layer. */
#define LOG_SRC_APP             (1 << 14) /**< Receive logs from the application. */
#define LOG_SRC_SERIAL          (1 << 15) /**< Receive logs from the serial module. */
#define LOG_SRC_FSM             (1 << 16) /**< Receive logs from the FSM module. */

/** Group for receiving logs from the core stack. */
#define LOG_GROUP_STACK (LOG_SRC_BEARER | LOG_SRC_NETWORK | LOG_SRC_TRANSPORT)

/** @} */

/**
 * @defgroup LOG_LEVELS Log levels
 * Defines possible criticality levels for logged messages. This can be used in
 * __LOG_INIT() to filter events by criticality.
 * @{
 */

#define LOG_LEVEL_ASSERT ( 0) /**< Log level for assertions */
#define LOG_LEVEL_ERROR  ( 1) /**< Log level for error messages. */
#define LOG_LEVEL_WARN   ( 2) /**< Log level for warning messages. */
#define LOG_LEVEL_REPORT ( 3) /**< Log level for report messages. */
#define LOG_LEVEL_INFO   ( 4) /**< Log level for information messages. */
#define LOG_LEVEL_DBG1   ( 5) /**< Log level for debug messages (debug level 1). */
#define LOG_LEVEL_DBG2   ( 6) /**< Log level for debug messages (debug level 2). */
#define LOG_LEVEL_DBG3   ( 7) /**< Log level for debug messages (debug level 3). */
#define EVT_LEVEL_BASE   ( 8) /**< Base level for event logging. For internal use only. */
#define EVT_LEVEL_ERROR  ( 9) /**< Critical error event logging level. For internal use only. */
#define EVT_LEVEL_INFO   (10) /**< Normal event logging level. For internal use only. */
#define EVT_LEVEL_DATA   (11) /**< Event data logging level. For internal use only. */

/** @} */

/** Filename macro used when printing. Provides the filename of the input file without any directory prefix. */
#ifdef __CC_ARM
#define __FILENAME__ __MODULE__
#else
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#endif

#if NRF_MESH_LOG_ENABLE > 0

/** Global debug mask. The value of this variable is used to filter the log messages being printed. */
extern uint32_t g_log_dbg_msk;
/** Global log level. The value of this variable is used to filter the log messages being printed. */
extern int32_t g_log_dbg_lvl;

/** Callback function used for printing log strings. */
typedef void (*log_callback_t)(uint32_t dbg_level, const char * p_filename, uint16_t line,
                                   uint32_t timestamp, const char * format, va_list arguments);

#if (LOG_ENABLE_RTT && !defined(HOST))
/** Callback function for printing debug information over RTT. */
void log_callback_rtt(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments);

/** Callback function for printing debug information over RTT in the format required by LogViewer. */
void log_callback_logview(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments);
#endif

#if defined(HOST)
/** Callback function for printing debug information on stdout. */
void log_callback_stdout(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments);
#endif

/**
 * Initializes the logging module.
 *
 * @param[in] mask     Mask specifying which modules to log information from.
 * @param[in] level    Maximum log level to print messages from.
 * @param[in] callback Callback function for printing log strings.
 */
void log_init(uint32_t mask, uint32_t level, log_callback_t callback);

/**
 * Sets the log callback function.
 *
 * The callback function is called to print strings from the logging module.
 * An application that is interested in internal logging can set this function
 * in order to print the log information in an application-specific way.
 *
 * @param[in] callback The callback function to use for printing log information.
 */
void log_set_callback(log_callback_t callback);

/**
 * Gets a timestamp for use with the log functions.
 *
 * When compiling for target, this is the current value of the RTC0 counter. If
 * compiling for a Linux host, this is the value of the system realtime clock in µs.
 *
 * On Windows hosts, this function will return 0.
 *
 * @return A timestamp for use with the log functions.
 */
static inline uint32_t log_timestamp_get(void)
{
#if defined(HOST)
    #if defined(__linux__)
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        return (now.tv_sec * 1000000UL) + (now.tv_nsec / 1000UL);
    #else
        return 0;
    #endif
#else
    return NRF_RTC0->COUNTER;
#endif
}

/**
 * Prints log data.
 * This function is used by the logging macros, but can also be called directly
 * if desired.
 *
 * @param[in] dbg_level    The debugging level to print the message as.
 * @param[in] p_filename   Name of the file in which the log call originated.
 * @param[in] line         Line number where the function was called.
 * @param[in] timestamp    Timestamp for when the log function was called.
 * @param[in] format       Format string, printf()-compatible.
 *
 * @see log_vprintf()
 */
void __attribute((format(printf, 5, 6))) log_printf(
    uint32_t dbg_level, const char * p_filename, uint16_t line, uint32_t timestamp, const char * format, ...);

/**
 * Prints log data.
 * This function is used by the logging macros, but can also be called directly
 * if desired.
 *
 * @param[in] dbg_level    The debugging level to print the message as.
 * @param[in] p_filename   Name of the file in which the log call originated.
 * @param[in] line         Line number for where the log function was called.
 * @param[in] timestamp    Timestamp for when the log function was called.
 * @param[in] format       Format string, printf()-compatible.
 * @param[in] arguments    Arguments according to the @c format string.
 *
 * @see log_printf()
 */
void log_vprintf(uint32_t dbg_level, const char * p_filename, uint16_t line, uint32_t timestamp,
    const char * format, va_list arguments);

/**
 * Initializes the logging framework.
 * @param[in] msk      Log mask
 * @param[in] level    Log level
 * @param[in] callback Log callback
 */
#define __LOG_INIT(msk, level, callback) log_init(msk, level, callback)

/**
 * Prints a log message.
 * @param[in] source Log source
 * @param[in] level  Log level
 * @param[in] ...    Arguments passed on to the callback (similar to @c printf)
 */
#define __LOG(source, level, ...)                                       \
    if ((source & g_log_dbg_msk) && level <= g_log_dbg_lvl)             \
    {                                                                   \
        log_printf(level, __FILENAME__, __LINE__, log_timestamp_get(), __VA_ARGS__); \
    }

/**
 * Prints an array with a message.
 * @param[in] source Log source
 * @param[in] level  Log level
 * @param[in] msg    Message string
 * @param[in] array  Pointer to array
 * @param[in] len    Length of array (in bytes)
 */
#define __LOG_XB(source, level, msg, array, array_len)                      \
    if ((source & g_log_dbg_msk) && (level <= g_log_dbg_lvl))           \
    {                                                                       \
        unsigned _array_len = (array_len);                                  \
        char array_text[_array_len * 2 + 1];                                \
        for(unsigned _i = 0; _i < _array_len; ++_i)                         \
        {                                                                   \
            extern const char * g_log_hex_digits;                           \
            const uint8_t array_elem = (array)[_i];                         \
            array_text[_i * 2] = g_log_hex_digits[(array_elem >> 4) & 0xf]; \
            array_text[_i * 2 + 1] = g_log_hex_digits[array_elem & 0xf];    \
        }                                                                   \
        array_text[_array_len * 2] = 0;                                     \
        log_printf(level, __FILENAME__, __LINE__, log_timestamp_get(), "%s: %s\n", msg, array_text); \
    }

#else
#define __LOG_INIT(...)
#define __LOG(...)
#define __LOG_XB(...)

#ifndef LOG_CALLBACK_DEFAULT
/** Default log callback. */
#define LOG_CALLBACK_DEFAULT NULL
#endif

#endif  /* NRF_MESH_LOG_ENABLE */

/** @} */
#endif  /* LOG_H__ */
