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

#ifndef HEALTH_COMMON_H__
#define HEALTH_COMMON_H__

/**
 * @defgroup HEALTH_MODEL Health Model
 * @ingroup MESH_API_GROUP_FOUNDATION_MODELS
 * Implementation of the Health Model foundation model.
 * @{
 */

/** Model ID for the Health Server model. */
#define HEALTH_SERVER_MODEL_ID                              0x0002
/** Model ID for the Health Client model. */
#define HEALTH_CLIENT_MODEL_ID                              0x0003

/**
 * @defgroup HEALTH_FAULT_IDS Fault IDs
 * @tagMeshSp defined fault IDs that can be used by self-tests to indicate specific problems.
 * @{
 */

#define HEALTH_FAULT_ID_NO_FAULT                            0x00
#define HEALTH_FAULT_ID_BATTERY_LOW_WARNING                 0x01
#define HEALTH_FAULT_ID_BATTERY_LOW_ERROR                   0x02
#define HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_WARNING          0x03
#define HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_ERROR            0x04
#define HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_WARNING         0x05
#define HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_ERROR           0x06
#define HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_WARNING    0x07
#define HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_ERROR      0x08
#define HEALTH_FAULT_ID_NO_LOAD_WARNING                     0x09
#define HEALTH_FAULT_ID_NO_LOAD_ERROR                       0x0a
#define HEALTH_FAULT_ID_OVERLOAD_WARNING                    0x0b
#define HEALTH_FAULT_ID_OVERLOAD_ERROR                      0x0c
#define HEALTH_FAULT_ID_OVERHEAT_WARNING                    0x0d
#define HEALTH_FAULT_ID_OVERHEAT_ERROR                      0x0e
#define HEALTH_FAULT_ID_CONDENSATION_WARNING                0x0f
#define HEALTH_FAULT_ID_CONDENSATION_ERROR                  0x10
#define HEALTH_FAULT_ID_VIBRATION_WARNING                   0x11
#define HEALTH_FAULT_ID_VIBRATION_ERROR                     0x12
#define HEALTH_FAULT_ID_CONFIGURATION_WARNING               0x13
#define HEALTH_FAULT_ID_CONFIGURATION_ERROR                 0x14
#define HEALTH_FAULT_ID_ELEMENT_NOT_CALIBRATED_WARNING      0x15
#define HEALTH_FAULT_ID_ELEMENT_NOT_CALIBRATED_ERROR        0x16
#define HEALTH_FAULT_ID_MEMORY_WARNING                      0x17
#define HEALTH_FAULT_ID_MEMORY_ERROR                        0x18
#define HEALTH_FAULT_ID_SELFTEST_WARNING                    0x19
#define HEALTH_FAULT_ID_SELFTEST_ERROR                      0x1a
#define HEALTH_FAULT_ID_INPUT_TOO_LOW_WARNING               0x1b
#define HEALTH_FAULT_ID_INPUT_TOO_LOW_ERROR                 0x1c
#define HEALTH_FAULT_ID_INPUT_TOO_HIGH_WARNING              0x1d
#define HEALTH_FAULT_ID_INPUT_TOO_HIGH_ERROR                0x1e
#define HEALTH_FAULT_ID_INPUT_NO_CHANGE_WARNING             0x1f
#define HEALTH_FAULT_ID_INPUT_NO_CHANGE_ERROR               0x20
#define HEALTH_FAULT_ID_ACTUATOR_BLOCKED_WARNING            0x21
#define HEALTH_FAULT_ID_ACTUATOR_BLOCKED_ERROR              0x22
#define HEALTH_FAULT_ID_HOUSING_OPENED_WARNING              0x23
#define HEALTH_FAULT_ID_HOUSING_OPENED_ERROR                0x24
#define HEALTH_FAULT_ID_TAMPER_WARNING                      0x25
#define HEALTH_FAULT_ID_TAMPER_ERROR                        0x26
#define HEALTH_FAULT_ID_DEVICE_MOVED_WARNING                0x27
#define HEALTH_FAULT_ID_DEVICE_MOVED_ERROR                  0x28
#define HEALTH_FAULT_ID_DEVICE_DROPPED_WARNING              0x29
#define HEALTH_FAULT_ID_DEVICE_DROPPED_ERROR                0x2a
#define HEALTH_FAULT_ID_OVERFLOW_WARNING                    0x2b
#define HEALTH_FAULT_ID_OVERFLOW_ERROR                      0x2c
#define HEALTH_FAULT_ID_EMPTY_WARNING                       0x2d
#define HEALTH_FAULT_ID_EMPTY_ERROR                         0x2e
#define HEALTH_FAULT_ID_INTERNAL_BUS_WARNING                0x2f
#define HEALTH_FAULT_ID_INTERNAL_BUS_ERROR                  0x30
#define HEALTH_FAULT_ID_MECHANISM_JAMMED_WARNING            0x31
#define HEALTH_FAULT_ID_MECHANISM_JAMMED_ERROR              0x32
#define HEALTH_FAULT_ID_VENDOR_START                        0x80

/** @} */

/** @} */

#endif

