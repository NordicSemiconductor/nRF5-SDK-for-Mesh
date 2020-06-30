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

#ifndef SENSOR_MESSAGES_H__
#define SENSOR_MESSAGES_H__

#include <stdint.h>
#include "sensor_common.h"
#include "nrf_mesh_assert.h"

/**
 * @internal
 * @defgroup SENSOR_MESSAGES Internal header
 * @ingroup SENSOR_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/* @tagMeshMdlSp - 4.1 "The number of sensors within a multisensor is
 * limited by the size of the message payload for the Sensor
 * Descriptor Status message" ... "up to 38 Sensor Descriptor states
 * may be sent" */
#define MAX_NUM_SENSORS 38

/** Sensor Descriptor */
#define SENSOR_DESCRIPTOR_MINLEN 2
#define SENSOR_DESCRIPTOR_STATE_SIZE 8
#define SENSOR_DESCRIPTOR_MAXLEN (MAX_NUM_SENSORS * SENSOR_DESCRIPTOR_STATE_SIZE)

/** Sensor Setting */
#define SENSOR_SETTING_SET_MINLEN 5

/** Sensor Cadence */
#define SENSOR_CADENCE_SET_MINLEN 8

/** Sensor measurement */
#define SENSOR_STATUS_MINLEN 3
#define SENSOR_STATUS_MAXLEN_SINGLE_SENSOR 130
#define SENSOR_STATUS_MAXLEN 379

/** Sensor model message opcodes. */
typedef enum
{
    SENSOR_OPCODE_DESCRIPTOR_GET = 0x8230,
    SENSOR_OPCODE_DESCRIPTOR_STATUS = 0x51,
    SENSOR_OPCODE_GET = 0x8231,
    SENSOR_OPCODE_STATUS = 0x52,
    SENSOR_OPCODE_COLUMN_GET = 0x8232,
    SENSOR_OPCODE_COLUMN_STATUS = 0x53,
    SENSOR_OPCODE_SERIES_GET = 0X8233,
    SENSOR_OPCODE_SERIES_STATUS = 0x54,
    SENSOR_OPCODE_CADENCE_GET = 0x8234,
    SENSOR_OPCODE_CADENCE_SET = 0x55,
    SENSOR_OPCODE_CADENCE_SET_UNACKNOWLEDGED = 0x56,
    SENSOR_OPCODE_CADENCE_STATUS = 0x57,
    SENSOR_OPCODE_SETTINGS_GET = 0x8235,
    SENSOR_OPCODE_SETTINGS_STATUS = 0x58,
    SENSOR_OPCODE_SETTING_GET = 0x8236,
    SENSOR_OPCODE_SETTING_SET = 0x59,
    SENSOR_OPCODE_SETTING_SET_UNACKNOWLEDGED = 0x5A,
    SENSOR_OPCODE_SETTING_STATUS = 0x5B,
} sensor_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/* Note that the partial message types are assumed to be parsed as uint8_t buffers beyond the few
   fields at the top that are defined in the structure. uint8_t is the straightforward way to handle
   variably-sized fields.
*/

/** Message format for the Sensor Descriptor Get message.
    4.2.1 Sensor Descriptor Get
    Sensor Descriptor Get is an acknowledged message used to get the Sensor Descriptor state of all
    sensors within an element (see Section 4.1.1).
*/
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
} sensor_descriptor_get_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint64_t sensor_property_id        : 16;
    uint64_t sensor_positive_tolerance : 12;
    uint64_t sensor_negative_tolerance : 12;
    uint64_t sensor_sampling_function  : 8;
    uint64_t sensor_measurement_period : 8;
    uint64_t sensor_update_interval    : 8;
} sensor_descriptor_pkt_t;

STATIC_ASSERT(SENSOR_DESCRIPTOR_STATE_SIZE == sizeof(sensor_descriptor_pkt_t),
              "Wrong packed sensor_descriptor_pkt_t.");

/** Message format for the Sensor Descriptor Status message.
    4.2.2 Sensor Descriptor Status
    The Sensor Descriptor Status is an unacknowledged message used to report a sequence of the Sensor
    Descriptor states of an element (see Section 4.1.1).

    The message uses a single-octet Opcode to maximize the payload size. The Descriptor field shall
    contain a sequence of 1 or more Sensor Descriptor states as defined in Section 4.1.1. When the
    message is a response to a Sensor Descriptor Get message that identifies a sensor descriptor
    property that does not exist on the element, the Descriptor field shall contain the requested
    Property ID value and the other fields of the Sensor Descriptor state shall be omitted.

    The message structure:

                      Filed Size
    Field name          (Bits)     Notes
    ------------------------------------------------------------------------------------------------
    Descriptor         8*N or 2    Sequence of 8-octet Sensor Descriptors
 */
typedef union __attribute((packed))
{
    uint16_t property_id;
    sensor_descriptor_pkt_t descriptors;
} sensor_descriptor_status_msg_pkt_t;

/** Message format for the Sensor Cadence Get message.
    4.2.3 Sensor Cadence Get
    Sensor Cadence Get is an acknowledged message used to get the Sensor Cadence state of an element
    (see Section 4.1.3).
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
} sensor_cadence_get_msg_pkt_t;

/** Message format for the Sensor Cadence Status message.
    4.2.6 Sensor Cadence Status
    The Sensor Cadence Status is an unacknowledged message used to report the Sensor Cadence state
    of an element (see Section 4.1.2).

    The message structure:

                               Filed Size
    Field name                   (Bits)   Notes
    ------------------------------------------------------------------------------------------------
    Property ID                    16     Property for the sensor.
    Fast Cadence Period Divisor     7     Divisor for the Publish Period (optional)
    Status Trigger Type             1     Defines the unit and format of the Status Trigger Delta fields.
    Status Trigger Delta Down   variable  Delta down value that triggers a status message.
    Status Trigger Delta Up     variable  Delta up value that triggers a status message.
    Status Min Interval             8     Minimum interval between two consecutive status messages.
    Fast Cadence Low            variable  Low value for the fast cadence range.
    Fast Cadence High           variable  High value for the fast cadence range.

    If the Fast Cadence Period Divisor field is present, the Status Trigger Type, Status Trigger Delta
    Down, Status Trigger Delta Up, Status Min Interval, Fast Cadence Low, and Fast Cadence High fields
    shall also be present; otherwise these fields shall not be present.
 */
typedef uint8_t sensor_cadence_status_msg_pkt_t;

/** Message format for the Sensor Cadence Set/Set Unacknowledged message (partial)
    4.2.4 Sensor Cadence Set
    Sensor Cadence Set is an acknowledged message used to set the Sensor Cadence state of an
    element (see Section 4.1.3). The response to the Sensor Cadence Set message is a Sensor Cadence
    Status message.

    The message structure:
                               Filed Size
    Field name                   (Bits)   Notes
    ------------------------------------------------------------------------------------------------
    Property ID                    16     Property ID for the sensor.
    Fast Cadence Period Divisor     7     Divisor for the Publish Period (see @tagMeshSp).
    Status Trigger Type             1     Defines the unit and format of the Status Trigger Delta
                                          fields.
    Status Trigger Delta Down    variable Delta down value that triggers a status message.
    Status Trigger Delta Up      variable Delta up value that triggers a status message.
    Status Min Interval             8     Minimum interval between two consecutive Status messages.
    Fast Cadence Low             variable Low value for the fast cadence range.
    Fast Cadence High            variable High value for the fast cadence range.
*/
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
} sensor_cadence_set_msg_pkt_t;

/** Message format for the Sensor Settings Get message.
    4.2.7 Sensor Settings Get
    Sensor Settings Get is an acknowledged message used to get the list of Sensor Setting states of
    an element (see Section 4.1.2).
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
} sensor_settings_get_msg_pkt_t;

/** Message format for the Sensor Settings Status message.
    4.2.8 Sensor Settings Status
    The Sensor Settings Status is an unacknowledged message used to report a list of the Sensor
    Setting states of an element (see Section 4.1.2).
    The message is sent as a response to the Sensor Settings Get message or is sent as an unsolicited
    message.
    The message structure:

                               Field Size
    Field Name                  (octets)             Notes
    ------------------------------------------------------------------------------------------------
    Sensor Property ID             2       Property ID identifying a sensor.
    Sensor Setting Property IDs   2*N      A sequence of N Sensor Setting Property IDs identifying
                                           settings within a sensor, where N is the number of
                                           property IDs included in the message. (Optional)
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint16_t setting_property_ids[];
} sensor_settings_status_msg_pkt_t;

/** Message format for the Sensor Setting Get message.
    4.2.9 Sensor Setting Get
    Sensor Setting Get is an acknowledged message used to get the Sensor Setting state of an element
    (see Section 4.1.2).
    The response to the Sensor Setting Get message is a Sensor Setting Status message.
    The message structure:

                              Field Size
    Field Name                 (octets)      Notes
    ------------------------------------------------------------------------------------------------
    Sensor Property ID            2        Property ID identifying a sensor.
    Sensor Setting Property ID    2        Setting Property ID identifying a setting within a sensor.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint16_t setting_property_id;    /**< Setting Property ID identifying a setting within a sensor */
} sensor_setting_get_msg_pkt_t;

/** Message format for the Sensor Setting Set/Set Unacknowledged message (partial)
    4.2.10 Sensor Setting Set
    Sensor Setting Set is an acknowledged message used to set the Sensor Setting state of an element
    (see Section 4.1.2). The response to the Sensor Setting Set message is a Sensor Setting Status
    message.
    The message structure:

                                    Field Size
    Field Name                       (octets)      Notes
    ------------------------------------------------------------------------------------------------
    Sensor Property ID                  2       Property ID identifying a sensor.
    Sensor Setting Property ID          2       Setting ID identifying a setting within a sensor.
    Sensor Setting Raw               variable   Raw value for the setting.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint16_t setting_property_id;    /**< Setting Property ID identifying a setting within a sensor */
    uint8_t  setting_raw[];
} sensor_setting_set_msg_pkt_t;

/** Message format for the Sensor Setting Status message.
  4.2.12 Sensor Setting Status
  Sensor Setting Status is an unacknowledged message used to report the Sensor Setting state of an
  element (see Section 4.1.2). The message is sent as a response to the Sensor Setting Get and Sensor
  Setting Set messages or sent as an unsolicited message.
  The message structure:
                              Field Size
                               (octets)   Notes
    ------------------------------------------------------------------------------------------------
    Sensor Property ID            2      Property ID identifying a sensor.
    Sensor Setting Property ID    2      Setting ID identifying a setting within a sensor.
    Sensor Setting Access         1      Read / Write access rights for the setting. (Optional)
    Sensor Setting Raw         variable  Raw value for the setting. (C.1)

    C.1: If the Sensor Setting Access field is present, the Sensor Setting Raw field shall also be
    present; otherwise this field shall not be present.

 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint16_t setting_property_id;    /**< Setting Property ID identifying a setting within a sensor */
    uint8_t  setting_access;
    uint8_t  setting_raw[];
} sensor_setting_status_msg_pkt_t;

/** Message format for the Sensor Get message
    4.2.13 Sensor Get
    Sensor Get is an acknowledged message used to get the Sensor Data state (see Section 4.1.4).
    The response to the Sensor Get message is a Sensor Status message.
    The message structure:

                                Field Size
    Field Name                   (octets)       Notes
    ------------------------------------------------------------------------------------------------
    Sensor Property ID              2         Property ID identifying a sensor.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
} sensor_get_msg_pkt_t;

/** Message format for the Sensor Status message.
    4.2.14 Sensor Status
    Sensor Status is an unacknowledged message used to report the Sensor Data state of an element
    (see Section 4.1.4). The message contains a Sensor Data state, defined by the Sensor Descriptor
    state (see Section 4.1.1).

    The message structure:
                            Field Size
    Field Name               (octets)   Notes
    ------------------------------------------------------------------------------------------------
    Marshalled Sensor Data   variable   The Sensor Data state.

    See Table 4.30: "Marshalled Sensor Data field" for a description of the Marshalled Sensor Data
    field.
 */
typedef uint8_t sensor_status_msg_pkt_t;

/** Message format for the Sensor Column Get message (partial)
    4.2.15 Sensor Column Get
    Sensor Column Get is an acknowledged message used to get the Sensor Series Column state (see
    Section 4.1.5). The response to the Sensor Column Get message is a Sensor Column Status message
    (see Section 4.2.16).
    The message structure:
                  Field Size
                   (octets)       Notes
    Property ID       2         Property identifying a sensor
    Raw Value X     variable    Raw value identifying a column
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint8_t  raw_value_x[];          /**< Raw value identifying a column. This must be correctly
                                          formatted (or parsed) by the application. */
} sensor_column_get_msg_pkt_t;

/** Message format for the Sensor Column Status message
    4.2.16 Sensor Column Status
    Sensor Column Status is an unacknowledged message used to report the Sensor Series Column state
    of an element (see Section 4.1.5).
    The message structure:
                    Field Size
    Field Name       (octets)       Notes
    ------------------------------------------------------------------------------------------------
    Property ID         2        Property identifying a sensor and the Y axis.
    Raw Value X      variable    Raw value representing the left corner of the column on the X axis.
    Column Width     variable    Raw value representing the width of the column. (Optional)
    Raw Value Y      variable    Raw value representing the height of the column on the Y axis. (C.1)

    C.1: If the Column Width field is present, the Raw Value Y field shall also be present; otherwise
    this field shall not be present.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint8_t  raw_value_xwy[];        /**< Sequence of raw value x, column width and raw value y.
                                          These must be correctly formatted (or parsed) by the application. */
} sensor_column_status_msg_pkt_t;

/** Message format for the Sensor Series Get message (partial)
    4.2.17 Sensor Series Get
    Sensor Series Get is an acknowledged message used to get a sequence of the Sensor Series Column
    states (see Section 4.1.5). The response to the Sensor Series Get message is a Sensor Series
    Status message (see Section 4.2.18).
    The message structure:
                    Field Size
    Field Name       (octets)         Notes
    ------------------------------------------------------------------------------------------------
    Property ID         2          Property identifying a sensor.
    Raw Value X1     variable      Raw value identifying a starting column. (Optional)
    Raw Value X2     variable      Raw value identifying an ending column. (C.1)

    C.1: If the Raw Value X1 field is present, the Raw Value X2 field shall also be present; otherwise this
    field shall not be present.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint8_t raw_value_x1x2[];        /**< Raw value identifying a starting and ending column.
                                          This must be correctly formatted (or parsed) by the
                                          application. */
} sensor_series_get_msg_pkt_t;

/** Message format for the Sensor Series Status message
    4.2.18 Sensor Series Status
    Sensor Series Status is an unacknowledged message used to report a sequence of the Sensor
    Series Column states of an element (see Section 4.1.5).

    The Raw Value X [n], Column Width [n], and Raw Value Y [n] fields are a triplet that may be
    repeated multiple times within the message. The Raw Value X [n] field is followed by the Column
    Width [n] field, which is followed by the Raw Value Y [n], which is followed by the Raw Value X
    [n+1], and so forth.

    The message structure:
                    Field Size
    Field Name       (octets)          Notes
    ------------------------------------------------------------------------------------------------
    Property ID         2           Property identifying a sensor and the Y axis.
    Raw Value X [n]  variable       Raw value representing the left corner of the n th column on the
                                    X axis. (Optional)
    Column Width [n] variable       Raw value representing the width of the n th column. (C.1)
    Raw Value Y [n]  variable       Raw value representing the height of the n th column on the Y
                                    axis. (C.1)

    C.1: If Raw Value X [n] field is present, the Column Width [n], Raw Value Y [n] fields shall also
    be present; otherwise these fields shall not be present.
 */
typedef struct __attribute((packed))
{
    uint16_t property_id;            /**< Property ID for the sensor */
    uint8_t  raw_value_xwy[];        /**< Sequence of raw value x[n], column width[n] and raw
                                          value y[n], followed by raw value x[n+1] and so on.
                                          These must be correctly formatted (or parsed) by the
                                          application. */
} sensor_series_status_msg_pkt_t;

/**@} end of SENSOR_MODEL_INTERNAL */
#endif /* SENSOR_MESSAGES_H__ */
