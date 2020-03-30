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

#ifndef MESH_OPT_DSM_H__
#define MESH_OPT_DSM_H__

#include <stdint.h>

#include "mesh_opt.h"
#include "device_state_manager.h"

/**
 * @internal
 * @defgroup MESH_OPT_DSM Device State Manager persistent options
 * @ingroup MESH_OPT
 * Runtime configuration for the DSM functionality of the mesh configuration.
 * @{
 */

/** @internal @{ */
enum
{
    MESH_OPT_DSM_METADATA_RECORD               = 0x0001,
    MESH_OPT_DSM_UNICAST_ADDR_RECORD           = 0x0002,
    MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD        = 0x1000,
    MESH_OPT_DSM_VIRTUAL_ADDR_RECORD           = 0x2000,
    MESH_OPT_DSM_SUBNETS_RECORD                = 0x3000,
    MESH_OPT_DSM_APPKEYS_RECORD                = 0x4000,
    MESH_OPT_DSM_DEVKEYS_RECORD                = 0x5000,
    MESH_OPT_DSM_LEGACY_SUBNETS_RECORD         = 0xD000,
    MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD = 0xE000,
    MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD    = 0xF000,
};

/** DSM mesh config metadata entry. */
#define MESH_OPT_DSM_METADATA_EID                MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_METADATA_RECORD)
/** Device unicast address entry. */
#define MESH_OPT_DSM_UNICAST_ADDR_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_UNICAST_ADDR_RECORD)
/** Non-virtual address pool entries. */
#define MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD)
/** Virtual address pool entries. */
#define MESH_OPT_DSM_VIRTUAL_ADDR_RECORD_EID     MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD)
/** Network key list entries. */
#define MESH_OPT_DSM_SUBNETS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_SUBNETS_RECORD)
/** Application key list entries. */
#define MESH_OPT_DSM_APPKEYS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_APPKEYS_RECORD)
/** Device key list entries. */
#define MESH_OPT_DSM_DEVKEYS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_DEVKEYS_RECORD)

/** Intermediate interpretation of legacy subnet entries. */
#define MESH_OPT_DSM_LEGACY_SUBNETS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_LEGACY_SUBNETS_RECORD)
/** Intermediate interpretation of legacy appkey entries. */
#define MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD)
/** Intermediate interpretation of legacy appkey entries. */
#define MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD_EID     MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD)

typedef struct
{
    uint16_t max_subnets;
    uint16_t max_appkeys;
    uint16_t max_devkeys;
    uint16_t max_addrs_nonvirtual;
    uint16_t max_addrs_virtual;
} dsm_entry_metainfo_t;

typedef struct
{
    dsm_local_unicast_address_t addr;
} dsm_entry_addr_unicast_t;

typedef struct
{
    mesh_key_index_t             key_index;
    nrf_mesh_key_refresh_phase_t key_refresh_phase;
    uint8_t                      key[NRF_MESH_KEY_SIZE];
    uint8_t                      key_updated[NRF_MESH_KEY_SIZE];
} dsm_entry_subnet_t;

typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    bool             is_key_updated;
    uint8_t          key[NRF_MESH_KEY_SIZE];
    uint8_t          key_updated[NRF_MESH_KEY_SIZE];
} dsm_entry_appkey_t;

typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    uint8_t          key[NRF_MESH_KEY_SIZE];
    uint8_t          key_updated[NRF_MESH_KEY_SIZE];
} dsm_legacy_entry_appkey_t;

typedef struct
{
    uint16_t     key_owner;
    dsm_handle_t subnet_handle;
    uint8_t      key[NRF_MESH_KEY_SIZE];
} dsm_entry_devkey_t;

typedef struct
{
    uint16_t addr;
} dsm_entry_addr_nonvirtual_t;

typedef struct
{
    uint8_t uuid[NRF_MESH_UUID_SIZE];
} dsm_entry_addr_virtual_t;

/** @} end of MESH_OPT_DSM */

#endif /* MESH_OPT_DSM_H__ */
