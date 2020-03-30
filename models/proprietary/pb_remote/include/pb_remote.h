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

#ifndef PB_REMOTE_H__
#define PB_REMOTE_H__

#include <stdint.h>

#include "nrf_mesh.h"
#include "pb_remote_msgs.h"

/**
 * @defgroup PB_REMOTE Provisioning over Mesh (PB-remote)
 * @ingroup MESH_API_GROUP_PROPRIETARY_MODELS
 *
 * This module defines the Remote Provisioning Models. This model consists of two submodules, the
 * Remote Provisioning Client and the Remote Provisioning Server.
 * 
 * For conceptual information, see @ref md_doc_user_guide_modules_provisioning_pb_remote.
 *
 * @{
 * @mscfile pb_remote_provisioning.msc  Remote Provisioning Scenario
 * @mscfile pb_remote_link_open.msc     Link Open Scenarios
 * @mscfile pb_remote_link_close.msc    Link Close Scenarios
 * @mscfile pb_remote_scanning.msc      Scanning Scenarios
 * @}
 *
 */

/**
 * Union of all Remote Provisioning messages.
 */
typedef union
{
    /** Scan start filter message. */
    pb_remote_msg_scan_start_filter_t                 scan_filter;
    /** Scan unprov device number message. */
    pb_remote_msg_scan_unprov_device_number_t         scan_unprov_device_number;
    /** Scan unprov device number report message. */
    pb_remote_msg_scan_unprov_device_number_report_t  scan_unprov_device_number_report;
    /** Scan uuid report message. */
    pb_remote_msg_scan_uuid_report_t                  scan_uuid_report;
    /** Scan report status message. */
    pb_remote_msg_scan_report_status_t                scan_report_status;
    /** Scan status message. */
    pb_remote_msg_scan_status_t                       scan_status;
    /** Scan stopped message. */
    pb_remote_msg_scan_stopped_t                      scan_stopped;
    /** Link open message. */
    pb_remote_msg_link_open_t                         link_open;
    /** Link status message. */
    pb_remote_msg_link_status_t                       link_status;
    /** Link close message. */
    pb_remote_msg_link_close_t                        link_close;
    /** Link status report message. */
    pb_remote_msg_link_status_report_t                link_status_report;
    /** Packet transfer message. */
    pb_remote_msg_packet_transfer_t                   packet_transfer;
    /** Packet transfer report message. */
    pb_remote_msg_packet_transfer_report_t            packet_transfer_report;
    /** Packet transfer status message. */
    pb_remote_msg_packet_transfer_status_t            packet_transfer_status;
} pb_remote_packet_t;

/** @} */

#endif  /* PB_REMOTE_H__ */
