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

#ifndef MESH_OPT_NET_STATE_H__
#define MESH_OPT_NET_STATE_H__

#include <stdint.h>

#include "mesh_opt.h"
#include "nrf_mesh.h"

/**
 * @defgroup MESH_OPT_NET_STATE net state persistent options
 * @ingroup MESH_OPT
 * Runtime configuration for Mesh net state functionality.
 * @{
 */

/** @internal @{ */
enum
{
    MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_RECORD = 1,
    MESH_OPT_NET_STATE_IV_INDEX_LEGACY_RECORD,
    MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD,
    MESH_OPT_NET_STATE_IV_INDEX_RECORD
};

/** Sequence number block of the network state */
#define MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD)
/** iv index  */
#define MESH_OPT_NET_STATE_IV_INDEX_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_IV_INDEX_RECORD)
/** Legacy entry IDs */
#define MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_RECORD)
#define MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_IV_INDEX_LEGACY_RECORD)

typedef struct __attribute((packed))
{
    uint32_t iv_index;
    net_state_iv_update_t  iv_update_in_progress;
    uint16_t  iv_update_timeout_counter;
    uint8_t  synchro_index;
} mesh_opt_iv_index_persist_data_t;

typedef struct __attribute((packed))
{
    uint32_t next_block;
    uint8_t  synchro_index;
} mesh_opt_seqnum_persist_data_t;

/** Legacy format for network state data */
typedef struct
{
    uint32_t iv_index;
    uint8_t  iv_update_in_progress;
} mesh_opt_iv_index_persist_data_legacy_t;

typedef struct
{
    uint32_t next_block;
} mesh_opt_seqnum_persist_data_legacy_t;

/** @} end of MESH_OPT_NET_STATE */

#endif /* MESH_OPT_NET_STATE_H__ */
