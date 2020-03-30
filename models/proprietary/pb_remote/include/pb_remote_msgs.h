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

#ifndef PB_REMOTE_MSGS_H__
#define PB_REMOTE_MSGS_H__

#include <stdint.h>
#include "nrf_mesh_defines.h"
#include "prov_pdu.h"

/**
 * @defgroup PB_REMOTE_MSGS Provisioning over Mesh (PB-remote) Messages
 * @ingroup  PB_REMOTE
 * @todo Variable names are directly corresponding to what's in the spec. Some
 * of them are not consistent. E.g., `scan_status` and `status`. Fix that.
 *
 * @{
 */

/** Reason code not supported. */
#define BEARER_LINK_REASON_NOT_SUPPORTED (0xFF)

/**
 * Remote provisioning opcodes.
 *
 * @todo The first two opcodes are already defined for the configuration model.
 * What should we do about it?
 */
typedef enum
{
    /**
     * Sent from the Server to the Client to indicate the state of the _local_
     * packet transfer. Is acknowledged with a @ref PB_REMOTE_OP_PACKET_TRANSFER_STATUS.
     */
    PB_REMOTE_OP_PACKET_TRANSFER_REPORT           = 0x11,
    /** Provisioning PDU from the Remote Client/Server. */
    PB_REMOTE_OP_PACKET_TRANSFER                  = 0x12,
    /**
     * Status message sent as reply to a @ref PB_REMOTE_OP_PACKET_TRANSFER or @ref
     * PB_REMOTE_OP_PACKET_TRANSFER_REPORT.
     */
    PB_REMOTE_OP_PACKET_TRANSFER_STATUS           = 0x8060,
    /** Command to close the local link from the Client to the Server. */
    PB_REMOTE_OP_LINK_CLOSE                       = 0x8061,
    /** Command to open the local link from the Client to the Server. */
    PB_REMOTE_OP_LINK_OPEN                        = 0x8062,
    /**
     * Link status sent as a reply to the @ref PB_REMOTE_OP_LINK_OPEN or @ref
     * PB_REMOTE_OP_LINK_CLOSE commands or as an acknowledgment of the @ref
     * PB_REMOTE_OP_LINK_STATUS_REPORT.
     */
    PB_REMOTE_OP_LINK_STATUS                      = 0x8063,
    /**
     * Command to cancel scanning for unprovisioned devices from the Client to
     * the Server.
     */
    PB_REMOTE_OP_SCAN_CANCEL                      = 0x8064,
    /** Command to start scanning from the Client to the Server. */
    PB_REMOTE_OP_SCAN_START                       = 0x8065,
    /** Command to start scanning with a filter from the Client to the Server. */
    PB_REMOTE_OP_SCAN_START_FILTER                = 0x8066,
    /**
     * Scan status. Used to acknowledge the @ref PB_REMOTE_OP_SCAN_START, @ref
     * PB_REMOTE_OP_SCAN_START_FILTER, @ref PB_REMOTE_OP_SCAN_UNPROVISIONED_DEVICE_NUMBER and @ref
     * PB_REMOTE_OP_SCAN_CANCEL commands.
     */
    PB_REMOTE_OP_SCAN_STATUS                      = 0x8067,
    /**
     * Command to start scanning while reporting the number of unprovisioned
     * devices nearby.
     *
     * @todo Not supported.
     */
    PB_REMOTE_OP_SCAN_UNPROVISIONED_DEVICE_NUMBER = 0x8068,
    /**
     * Report of the UUID and device ID of an unprovisioned device sent from the
     * Server to the Client.
     */
    PB_REMOTE_OP_SCAN_UUID_REPORT                 = 0x806A,
    /** Number of unprovisioned devices report. @todo Not supported. */
    PB_REMOTE_OP_SCAN_UUID_NUMBER_REPORT          = 0x806B,
    /**
     * Link status report. A reliable message used to indicate to the Client
     * that the state of the Local provisioning link has changed.
     */
    PB_REMOTE_OP_LINK_STATUS_REPORT               = 0x806C,
    /** Message used by the Client to acknowledge the scan reports. */
    PB_REMOTE_OP_SCAN_REPORT_STATUS               = 0x806D,
    /** Acknowledgment message from the Server that the scanning was stopped. */
    PB_REMOTE_OP_SCAN_STOPPED                     = 0x806E
} pb_remote_opcode_t;


/**
 * Remote Provisioning Report Status Codes.
 */
typedef enum
{
    /** Report status accepted. @ref pb_remote_msg_scan_report_status_t. */
    PB_REMOTE_REPORT_STATUS_ACCEPTED = 0x00,
    /** Report status rejected. @ref pb_remote_msg_scan_report_status_t. */
    PB_REMOTE_REPORT_STATUS_REJECTED = 0x01
} pb_remote_report_status_t;

/**
 * Remote Provisioning Scan Status Codes.
 */
typedef enum
{
    /** The scanning was successfully started. */
    PB_REMOTE_SCAN_STATUS_STARTED                = 0x00,
    /**
     * The Provisioning Server has stopped scanning upon reception of the Remote
     * Provisioning Scan Cancel message.
     */
    PB_REMOTE_SCAN_STATUS_CANCELED               = 0x01,
    /**
     * The Provisioning Server cannot start the scanning procedure due to
     * internal state of the server.
     */
    PB_REMOTE_SCAN_STATUS_CANNOT_START_SCANNING  = 0x02,
    /**
     * The Provisioning Server cannot cancel the scanning procedure due to
     * internal state of the server: scanning procedure is not running or
     * scanning was started by a different client.
     */
    PB_REMOTE_SCAN_STATUS_CANNOT_CANCEL_SCANNING = 0x03,
    /**
     * The Provisioning Client accepted that the Provisioning Server has
     * stopped scanning.
     */
    PB_REMOTE_SCAN_STATUS_ACCEPTED               = 0x04,
    /**
     * The Provisioning Client received a message, but the message was not
     * expected.
     */
    PB_REMOTE_SCAN_STATUS_REJECTED               = 0x05
} pb_remote_scan_status_t;

/**
 * Remote Provisioning Scan Stopped Status Codes.
 */
typedef enum
{
    /**
     * The Unprovisioned Devices List is full and scanning cannot be
     * continued.
     */
    PB_REMOTE_SCAN_STOPPED_OUT_OF_RESOURCES = 0x01,
    /** The scanning has reached timeout. */
    PB_REMOTE_SCAN_STOPPED_TIMEOUT          = 0x02
} pb_remote_scan_stopped_status_t;

/**
 * Remote Link Status Codes.
 */
typedef enum
{
    /**
     * The Server started the procedure to open the Local Provisioning Bearer
     * link to the unprovisioned device.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_OPENING      = 0x00,
    /**
     * The Local Provisioning Bearer link cannot be opened because the
     * Provisioning Server has already another active Local Provisioning Bearer
     * link opened.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_ALREADY_OPEN             = 0x01,
    /**
     * The Server cannot close the Local Provisioning Bearer link beacause the
     * Client that requested the link to be closed is different than the Client
     * that opened it.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_CANNOT_CLOSE             = 0x02,
    /**
     * The link between the Provisioning Server and the unprovisioned device was
     * not active upon reception of the Link Close message.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_LINK_NOT_ACTIVE          = 0x03,
    /**
     * The provided Unprovisioned Device ID in the Remote Provisioning Link Open
     * message was invalid.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_INVALID_UNPROV_DEVICE_ID = 0x04,
    /**
     * The Provisioning Client accepted that the Local Provisioning Bearer link
     * was closed.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_ACCEPTED                 = 0x05,
    /**
     * The Provisioning Client received a message, but the message was not
     * expected.
     */
    PB_REMOTE_REMOTE_LINK_STATUS_REJECTED                 = 0x06
} pb_remote_link_status_t;

/**
 * Remote Provisioning bearer type.
 */
typedef enum
{
    /** PB-ADV bearer type. */
    PB_REMOTE_BEARER_TYPE_PB_ADV  = 0x00,
    /** PB-GATT bearer type. */
    PB_REMOTE_BEARER_TYPE_PB_GATT = 0x01
} pb_remote_bearer_type_t;

/**
 * Provisioning Bearer Link Status Report.
 */
typedef enum
{
    /** The Local Provisioning Bearer link is opened. */
    PB_REMOTE_LINK_STATUS_REPORT_OPENED           = 0x00,
    /** The Local Provisioning Bearer link open procedure timed out. */
    PB_REMOTE_LINK_STATUS_REPORT_OPEN_TIMEOUT     = 0x01,
    /** The Local Provisioning Bearer link was closed. */
    PB_REMOTE_LINK_STATUS_REPORT_CLOSED           = 0x02,
    /** The unprovisioned device closed the Local Provisioning bearer link. */
    PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_DEVICE = 0x03,
    /** The Provisioning Server closed the Local Provisioning bearer link. */
    PB_REMOTE_LINK_STATUS_REPORT_CLOSED_BY_SERVER = 0x04
} pb_remote_link_status_report_t;

/**
 * Remote Provisioning Packet Transfer Report Delivery Status.
 */
typedef enum
{
    /** The buffer was successfully delivered to the unprovisioned device. */
    PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_DELIVERED     = 0x00,
    /** Delivering the buffer to the unprovisioned device failed. */
    PB_REMOTE_PACKET_TRANSFER_DELIVERY_STATUS_NOT_DELIVERED = 0x01
} pb_remote_packet_transfer_delivery_status_t;

/**
 * Remote Provisioning Packet Transfer Status.
 */
typedef enum
{
    /**
     *  The whole buffer is accepted and the packet transfer is successfully
     *  completed.
     */
    PB_REMOTE_PACKET_TRANSFER_STATUS_BUFFER_ACCEPTED = 0x00,
    /**
     * The Provisioning Server does not have an active Local Provisioning Bearer
     * link.
     */
    PB_REMOTE_PACKET_TRANSFER_STATUS_LINK_NOT_ACTIVE = 0x01,
    /**
     * Sent as a reply to the Remote Provisioning Packet Transfer when the
     * packet transfer cannot be accepted.
     */
    PB_REMOTE_PACKET_TRANSFER_STATUS_CANNOT_ACCEPT_BUFFER = 0x02,
    /**
     * The receiver accepted the Remote Provisioning Packet Transfer Report.
     */
    PB_REMOTE_PACKET_TRANSFER_STATUS_ACCEPTED = 0x03,
    /**
     * The Remote Provisioning Packet Transfer Report message was receive, but
     * the message was not expected.
     */
    PB_REMOTE_PACKET_TRANSFER_STATUS_REJECTED = 0x04
} pb_remote_packet_transfer_status_t;

/*lint -align_max(push) -align_max(1) */

/**
 * Remote Provisioning Scan Start with Filter Message.
 */
typedef struct __attribute((packed))
{
    /** The device UUID used to filter the scanned Device UUIDs. */
    uint8_t filter_uuid[NRF_MESH_UUID_SIZE];
} pb_remote_msg_scan_start_filter_t;

/**
 * Remote Provisioning Scan Unprovisioned Device Number Message.
 */
typedef struct __attribute((packed))
{
    /** Number of the reports sent before scan stop. */
    uint8_t report_count;
} pb_remote_msg_scan_unprov_device_number_t;

/**
 * Remote Provisioning Scan UUID Number Report Message.
 */
typedef struct __attribute((packed))
{
    /** Number of elements in the provisioning server's unprovisioned device list. */
    uint8_t unprov_device_count;
} pb_remote_msg_scan_unprov_device_number_report_t;

/**
 * Remote Provisioning Scan UUID Report.
 */
typedef struct __attribute((packed))
{
    /** Scanned Device UUID. */
    uint8_t uuid[NRF_MESH_UUID_SIZE];
    /**
     * ID of the provisioning Server slot where this device UUID is stored.
     *
     * The Unprovisioned Device ID is used to reference the Device UUID in the
     * Remote Provisioning Link Open message @ref pb_remote_msg_link_open_t.
     */
    uint8_t unprov_device_id;
} pb_remote_msg_scan_uuid_report_t;

/**
 * Remote Provisioning Scan Report Status Message.
 */
typedef struct __attribute((packed))
{
    /** The status of the report. @ref pb_remote_scan_status_t. */
    uint8_t status;
    /** Unprovisioned Device ID of the reported device. */
    uint8_t unprov_device_id;
} pb_remote_msg_scan_report_status_t;

/**
 * Remote Provisioning Scan Status Message.
 */
typedef struct __attribute((packed))
{
    /** Status of the scan. @ref pb_remote_scan_status_t. */
    uint8_t status;
} pb_remote_msg_scan_status_t;

/**
 * Remote Provisioning Scan Stopped Message.
 */
typedef struct __attribute((packed))
{
    /** The status of the scan. @ref pb_remote_scan_stopped_status_t. */
    uint8_t status;
} pb_remote_msg_scan_stopped_t;


/**
 * Remote Provisioning Link Open Message.
 */
typedef struct __attribute((packed))
{
    /** UUID of the device to be provisioned. */
    uint8_t uuid[NRF_MESH_UUID_SIZE];
} pb_remote_msg_link_open_t;

/**
 * Remote Provisioning Link Status Message.
 */
typedef struct __attribute((packed))
{
    /** Link status code. */
    uint8_t status;
    /** Local provisioning bearer type. */
    uint8_t bearer_type;
} pb_remote_msg_link_status_t;

/**
 * Remote Prvisioning Link Close Message.
 */
typedef struct __attribute((packed))
{
    /** Link Close reason code. */
    uint8_t reason;
} pb_remote_msg_link_close_t;

/**
 * Remote Provisioning Link Status Report Message.
 */
typedef struct __attribute((packed))
{
    /** Link Close reason code. */
    uint8_t status;
    /** Bearer specific link close reason. */
    uint8_t reason;
} pb_remote_msg_link_status_report_t;

/**
 * Remote Provisioning Packet Transfer Message.
 */
typedef struct __attribute((packed))
{
    /** PB-ADV data packet. */
    uint8_t buffer[PROV_PDU_MAX_LENGTH];
} pb_remote_msg_packet_transfer_t;

/**
 * Remote Provisioning Packet Transfer Report Message.
 */
typedef struct __attribute((packed))
{
    /**
     * Status of the buffer delivery.
     * @ref pb_remote_packet_transfer_status_t.
     */
    uint8_t status;
} pb_remote_msg_packet_transfer_report_t;

/**
 * Remote Provisioning Packet Transport Status Message.
 */
typedef struct __attribute((packed))
{
    /** Status of the packet transfer. @ref pb_remote_packet_transfer_status_t. */
    uint8_t status;
} pb_remote_msg_packet_transfer_status_t;

/*lint -align_max(pop) */
/** @} */

#endif  /* PB_REMOTE_MSGS_H__ */
