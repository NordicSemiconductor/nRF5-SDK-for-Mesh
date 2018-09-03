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
#ifndef MESH_OPT_H__
#define MESH_OPT_H__
#include "mesh_config_entry.h"

/**
 * @defgroup MESH_OPT Mesh options API
 * Provides a consistent toplevel API for runtime configuration options.
 *
 * Options are stored persistently on every change.
 * @{
 */

/** File ID for the persistent storage options file. */
#define MESH_OPT_CORE_FILE_ID      (0x0003)

/** Macro for mesh option entry IDs */
#define MESH_OPT_CORE_ID(id)      MESH_CONFIG_ENTRY_ID(MESH_OPT_CORE_FILE_ID, id)

/** Range for the core record entry IDs */
#define MESH_OPT_CORE_ID_START (0x0100)
#define MESH_OPT_CORE_ID_END   (0x01FF)
#define MESH_OPT_PROV_ID_START (0x0200)
#define MESH_OPT_PROV_ID_END   (0x020F)
#define MESH_OPT_GATT_ID_START (0x0210)
#define MESH_OPT_GATT_ID_END   (0x021F)

/** Initialize the mesh options module. */
void mesh_opt_init(void);

/** @} */

#endif /* MESH_OPT_H__ */
