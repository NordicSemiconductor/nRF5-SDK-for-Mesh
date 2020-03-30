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

#include "mesh_opt.h"
#include "mesh_config_entry.h"
#include "mesh_opt_core.h"
#if MESH_FEATURE_FRIEND_ENABLED
#include "mesh_opt_friend.h"
#endif
#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "mesh_opt_gatt.h"
#endif
#include "mesh_opt_prov.h"

MESH_CONFIG_FILE(m_mesh_opt_core_file, MESH_OPT_CORE_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

void mesh_opt_init(void)
{
    /* We have to reference a symbol in this file to avoid gcc incorrectly discarding the object if
     * it's building mesh as a library. */
}

void mesh_opt_clear(void)
{
    mesh_config_entry_id_t entry_id;

    for (uint8_t cnt = 0; cnt < MESH_OPT_CORE_ADV_COUNT; cnt++)
    {
        entry_id = MESH_OPT_CORE_ADV_EID;
        entry_id.record += cnt;
        (void)mesh_config_entry_delete(entry_id);
        entry_id = MESH_OPT_CORE_TX_POWER_EID;
        entry_id.record += cnt;
        (void)mesh_config_entry_delete(entry_id);
        entry_id = MESH_OPT_CORE_ADV_ADDR_EID;
        entry_id.record += cnt;
        (void)mesh_config_entry_delete(entry_id);
    }
    entry_id = MESH_OPT_CORE_SEC_NWK_BCN_EID;
    (void)mesh_config_entry_delete(entry_id);
    entry_id = MESH_OPT_CORE_HB_PUBLICATION_EID;
    (void)mesh_config_entry_delete(entry_id);

    entry_id = MESH_OPT_PROV_ECDH_OFFLOADING_EID;
    (void)mesh_config_entry_delete(entry_id);

    entry_id = MESH_OPT_HEALTH_PRIMARY_EID;
    (void)mesh_config_entry_delete(entry_id);

#if MESH_FEATURE_FRIEND_ENABLED
    entry_id = MESH_OPT_FRIEND_EID;
    (void)mesh_config_entry_delete(entry_id);
#endif

#if MESH_FEATURE_GATT_PROXY_ENABLED
    entry_id = MESH_OPT_GATT_PROXY_EID;
    (void)mesh_config_entry_delete(entry_id);
#endif
}
