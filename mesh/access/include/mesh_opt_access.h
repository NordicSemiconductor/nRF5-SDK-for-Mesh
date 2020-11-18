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

#ifndef MESH_OPT_ACCESS_H__
#define MESH_OPT_ACCESS_H__

#include <stdint.h>

#include "mesh_opt.h"

/**
 * @defgroup MESH_OPT_ACCESS access persistent options
 * @ingroup MESH_OPT
 * Runtime configuration for Mesh config access functionality.
 * @{
 */

/** @internal @{ */
enum
{
    MESH_OPT_ACCESS_METADATA_RECORD      = 0x0001,
    MESH_OPT_ACCESS_ELEMENTS_RECORD      = 0x1000,
    MESH_OPT_ACCESS_MODELS_RECORD        = 0x2000,
    MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD = 0x3000,
    MESH_OPT_ACCESS_DEFAULT_TTL_RECORD   = 0x4000,
};

/** Access layer mesh config metadata entry. */
#define MESH_OPT_ACCESS_METADATA_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_METADATA_RECORD)
/** Model subscription list entries. */
#define MESH_OPT_ACCESS_SUBSCRIPTIONS_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD)
/** Element pool entries. */
#define MESH_OPT_ACCESS_ELEMENTS_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_ELEMENTS_RECORD)
/** Model pool entries. */
#define MESH_OPT_ACCESS_MODELS_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_MODELS_RECORD)
/** Default TTL entry. */
#define MESH_OPT_ACCESS_DEFAULT_TTL_EID    MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_DEFAULT_TTL_RECORD)

/** @} end of MESH_OPT_ACCESS */

#endif /* MESH_OPT_ACCESS_H__ */
