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
#ifndef NRF_MESH_DFU_H__
#define NRF_MESH_DFU_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_config_dfu.h"
#include "nrf_mesh.h"
#include "nrf_mesh_dfu_types.h"
#include "dfu_types_internal.h"
#include "bl_if.h"

/**
 * @defgroup NRF_MESH_DFU DFU API
 * @ingroup MESH_API_GROUP_DFU
 * Allows for some basic control of the application-side DFU module.
 * All DFU related events are defined in the main API-header.
 * @{
 */

/**
 * @addtogroup NRF_MESH_DFU_DEFINES
 * @{
 */

/** Lowest authentication level available for DFU requests. */
#define NRF_MESH_DFU_AUTH_LEVEL_MIN (1)
/** Highest authentication level available for DFU requests. */
#define NRF_MESH_DFU_AUTH_LEVEL_MAX (7)

/** @} */

/**
 * @addtogroup NRF_MESH_DFU_TYPES
 * @{
 */
/**
 * Guiding authority values for sourcing a transfer.
 * When picking a device to source a DFU, the target node prioritizes based
 * on a priority parameter presented by each potential source. The source
 * candidate presenting the highest authority gets to source the transfer.
 */
typedef enum
{
    /** Lowest authority level, for nodes that should only be chosen if no
        other option is available. */
    NRF_MESH_DFU_SOURCE_AUTHORITY_WEAK_NODE = NRF_MESH_DFU_AUTH_LEVEL_MIN,
    /** Authority level for nodes which are participating in the network, and
        have plenty of capacity to spare. */
    NRF_MESH_DFU_SOURCE_AUTHORITY_STRONG_NODE = 4,
    /** Authority level for gateway-type nodes, which are able to provide the
        DFU directly from the external source. */
    NRF_MESH_DFU_SOURCE_AUTHORITY_GATEWAY = NRF_MESH_DFU_AUTH_LEVEL_MAX,
} nrf_mesh_dfu_source_authority_t;
/** @} */
/**
 * Initialize the DFU module.
 *
 * @retval NRF_SUCCESS The DFU-module was successfully initialized.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU-module was unable to find the bootloader
 *  or the bootloader device page.
 */
uint32_t nrf_mesh_dfu_init(void);

/**
 * Enable the DFU module in the Bootloader.
 *
 * @note This function is called from nrf_mesh_enable()
 *
 * @retval NRF_SUCCESS The DFU app successfully sent a msg to bootloader to enable DFU.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_* The given command did not succeed. The meaning of each
 * error code depends on the command.
 */
uint32_t nrf_mesh_dfu_enable(void);

/**
* Manually trigger the bootloader. The device will be reset immediately, and
* may not be available for regular operation for several minutes. If
* successful, this function does not return.
*
* @retval NRF_ERROR_FORBIDDEN The NRF_UICR->BOOTLOADERADDR persistent register
*   has not been set, and the bootloader could not start.
*/
uint32_t nrf_mesh_dfu_jump_to_bootloader(void);

/**
 * Pass a DFU packet to the DFU module.
 *
 * @param[in] p_packet A pointer to a DFU packet.
 * @param[in] length The length of the DFU packet.
 * @param[in] p_metadata Metadata attached to the packet that came in.
 *
 * @retval NRF_SUCCESS The packet was successfully handled by the DFU module.
 * @retval NRF_ERROR_BUSY The DFU module can't accept the request at the moment.
 * @retval NRF_ERROR_INVALID_ADDR The packet isn't a known DFU packet.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 */
uint32_t nrf_mesh_dfu_rx(const uint8_t * p_packet,
                         uint32_t length,
                         const nrf_mesh_rx_metadata_t * p_metadata);

/**
 * Request a DFU transfer.
 *
 * The DFU transfer will run alongside the application, and store the firmware
 * in the given bank.
 *
 * Generates events:
 *   * ::NRF_MESH_EVT_DFU_BANK_AVAILABLE: The DFU transfer is finished, and is
 *     available for flashing.
 *   * ::NRF_MESH_EVT_DFU_START: The DFU module got a response to the DFU
 *     request, and started receiving the transfer.
 *   * ::NRF_MESH_EVT_DFU_END: The DFU module finished its transfer.
 *
 * @param[in] type DFU type to request.
 * @param[in] p_fwid Firmware ID to request.
 * @param[in] p_bank_addr Address in which to store the banked data. Any
 * existing data in this location will be erased.
 *
 * @retval NRF_SUCCESS The DFU module has started requesting the given transfer.
 * @retval NRF_ERROR_NULL One of the parameters provided was null.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_INVALID_PARAM The given DFU type is not available.
 * @retval NRF_ERROR_INVALID_STATE The DFU module is not in an idle state, and
 * the operation can't be started. This can either be because the application
 * failed to initialize the module, or because a DFU operation is currently in
 * progress. In the last case, stop the current operation with @ref
 * nrf_mesh_dfu_abort() or wait for an end-event before requesting a new
 * transfer.
 */
uint32_t nrf_mesh_dfu_request(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid,
        const uint32_t* p_bank_addr);

/**
 * Relay an ongoing transfer. Should only be used as a response to an
 * ::NRF_MESH_EVT_DFU_REQ_RELAY.
 *
 * Generates events:
 *   * ::NRF_MESH_EVT_DFU_START: The transfer has started, and the device is
 *     actively relaying data packets.
 *   * ::NRF_MESH_EVT_DFU_END: The DFU module finished its transfer.
 *
 * @param[in] type DFU type to request.
 * @param[in] p_fwid Firmware ID to request.
 *
 * @retval NRF_SUCCESS The DFU module has started advertising its intention to
 *                     relay the given transfer.
 * @retval NRF_ERROR_NULL The FWID pointer provided was NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_INVALID_PARAM The given DFU type is not available.
 * @retval NRF_ERROR_INVALID_STATE The DFU module is not in an idle state, and
 * the operation can't be started. This can either be because the application
 * failed to initialize the module, or because a DFU operation is currently in
 * progress. In the last case, stop the current operation with @ref
 * nrf_mesh_dfu_abort() or wait for an end-event before requesting a new
 * transfer.
 */
uint32_t nrf_mesh_dfu_relay(nrf_mesh_dfu_type_t type,
        const nrf_mesh_fwid_t* p_fwid);

/**
 * Abort the ongoing DFU operation.
 *
 * @retval NRF_SUCCES The ongoing DFU operation was successfully stopped, and
 * the DFU module went back to the idle state.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_INVALID_STATE The DFU module was not doing any DFU
 * operations.
 */
uint32_t nrf_mesh_dfu_abort(void);

/**
 * Get info on the bank of the given type.
 *
 * @param[in] type Type of the bank to get info on.
 * @param[out] p_bank_info Pointer to a structure which the function will put
 * information on the bank in.
 *
 * @retval NRF_SUCCESS The bank was found, and the @c p_bank_info parameter was filled
 * with the correct paramters.
 * @retval NRF_ERROR_NULL The bank info pointer provided was NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_NOT_FOUND No bank of the given type was found.
 * @retval NRF_ERROR_INVALID_STATE The DFU module has not been initialized.
 */
uint32_t nrf_mesh_dfu_bank_info_get(nrf_mesh_dfu_type_t type, nrf_mesh_dfu_bank_info_t * p_bank_info);

/**
 * Flash the bank of the given type.
 *
 * @warning This will trigger a restart of the chip. All non-volatile memory
 * will be lost during this call. If successful, this never returns.
 *
 * @param[in] bank_type The DFU type of the bank to be flashed. There can only
 * be one bank of each DFU type.
 *
 * @retval NRF_ERROR_NOT_FOUND No bank of the given type is available.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_INVALID_STATE The DFU module has not been initialized.
 */
uint32_t nrf_mesh_dfu_bank_flash(nrf_mesh_dfu_type_t bank_type);

/**
 * Get the current state of the DFU module.
 *
 * @param[out] p_dfu_transfer_state A pointer to a DFU transfer state variable,
 * which the framework will fill with the current state/progress of an ongoing
 * transfer, if any.
 *
 * @retval NRF_SUCCESS The DFU state was successfully retrieved.
 * @retval NRF_ERROR_NULL The transfer state pointer provided was NULL.
 * @retval NRF_ERROR_NOT_SUPPORTED The DFU functionality is not available.
 * @retval NRF_ERROR_INVALID_STATE The DFU module has not been initialized.
 */
uint32_t nrf_mesh_dfu_state_get(nrf_mesh_dfu_transfer_state_t* p_dfu_transfer_state);

/**@}*/

#endif /* NRF_MESH_DFU_H__ */
