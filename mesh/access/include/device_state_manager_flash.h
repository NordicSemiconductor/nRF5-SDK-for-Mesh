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

#ifndef DEVICE_STATE_MANAGER_FLASH_H__
#define DEVICE_STATE_MANAGER_FLASH_H__

/**
 * @internal
 * @defgroup DSM_FLASH Device State Manager Flash definitions
 * @ingroup DEVICE_STATE_MANAGER
 * Defines the format of the device state manager's flash entries. Used internally.
 * @{
 */

#include <stdint.h>
#include "nrf_mesh_defines.h"
#include "device_state_manager.h"

#define DSM_FLASH_HANDLE_METAINFO        (0x0001)
#define DSM_FLASH_HANDLE_UNICAST         (0x0002)

#define DSM_FLASH_COLLECTION_HANDLE_FIRST   DSM_FLASH_HANDLE_UNICAST

#define DSM_FLASH_HANDLE_TO_DSM_HANDLE_MASK (0x0FFF) /**< Mask to apply to convert a flash handle to a DSM handle. */
#define DSM_FLASH_HANDLE_FILTER_MASK        (0xF000)
#define DSM_FLASH_GROUP_ADDR_NONVIRTUAL     (0x1000)
#define DSM_FLASH_GROUP_ADDR_VIRTUAL        (0x2000)
#define DSM_FLASH_GROUP_SUBNETS             (0x3000)
#define DSM_FLASH_GROUP_APPKEYS             (0x4000)
#define DSM_FLASH_GROUP_DEVKEYS             (0x5000)

#define DSM_FLASH_HANDLE_TO_DSM_HANDLE(DSM_FLASH_HANDLE)    ((DSM_FLASH_HANDLE) & DSM_FLASH_HANDLE_TO_DSM_HANDLE_MASK)
#define DSM_HANDLE_TO_FLASH_HANDLE(GROUP, DSM_FLASH_HANDLE) ((GROUP) | ((DSM_FLASH_HANDLE) & DSM_FLASH_HANDLE_TO_DSM_HANDLE_MASK))

typedef struct
{
    uint16_t max_subnets;
    uint16_t max_appkeys;
    uint16_t max_devkeys;
    uint16_t max_addrs_nonvirtual;
    uint16_t max_addrs_virtual;
} dsm_flash_entry_metainfo_t;

typedef struct
{
    dsm_local_unicast_address_t addr;
} dsm_flash_entry_addr_unicast_t;

typedef struct
{
    mesh_key_index_t             key_index;
    nrf_mesh_key_refresh_phase_t key_refresh_phase;
    uint8_t                      key[NRF_MESH_KEY_SIZE];
    uint8_t                      key_updated[NRF_MESH_KEY_SIZE];
} dsm_flash_entry_subnet_t;

typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    uint8_t          key[NRF_MESH_KEY_SIZE];
    uint8_t          key_updated[NRF_MESH_KEY_SIZE];
} dsm_flash_entry_appkey_t;

typedef struct
{
    uint16_t     key_owner;
    dsm_handle_t subnet_handle;
    uint8_t      key[NRF_MESH_KEY_SIZE];
} dsm_flash_entry_devkey_t;

typedef struct
{
    uint16_t addr;
} dsm_flash_entry_addr_nonvirtual_t;

typedef struct
{
    uint8_t uuid[NRF_MESH_UUID_SIZE];
} dsm_flash_entry_addr_virtual_t;

/** Union of all DSM flash entries */
typedef union
{
    dsm_flash_entry_metainfo_t        metainfo;
    dsm_flash_entry_addr_unicast_t    addr_unicast;
    dsm_flash_entry_subnet_t          subnet;
    dsm_flash_entry_appkey_t          appkey;
    dsm_flash_entry_devkey_t          devkey;
    dsm_flash_entry_addr_nonvirtual_t addr_nonvirtual;
    dsm_flash_entry_addr_virtual_t    addr_virtual;
} dsm_flash_entry_t;

/** @} */

#endif /* DEVICE_STATE_MANAGER_FLASH_H__ */
