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

#ifndef MESH_OPT_CORE_H__
#define MESH_OPT_CORE_H__

#include "mesh_opt.h"
#include "radio_config.h"
#include "ble.h"
#include "core_tx.h"

/**
 * @defgroup MESH_OPT_CORE Core runtime configuration options
 * @ingroup MESH_OPT
 * Runtime configuration for Mesh core functionality
 * @{
 */

/**
 * @{
 * @internal
 */
#if MESH_FEATURE_RELAY_ENABLED
#define MESH_OPT_CORE_ADV_COUNT 2
#else
#define MESH_OPT_CORE_ADV_COUNT 1
#endif
/* CORE_TX_ROLE_COUNT should be equal to MESH_OPT_CORE_ADV_COUNT */
NRF_MESH_STATIC_ASSERT(MESH_OPT_CORE_ADV_COUNT == CORE_TX_ROLE_COUNT);

enum
{
    MESH_OPT_CORE_ADV_RECORD_START      = MESH_OPT_CORE_ID_START,
    MESH_OPT_CORE_ADV_RECORD_END        = MESH_OPT_CORE_ADV_RECORD_START + MESH_OPT_CORE_ADV_COUNT - 1,
    MESH_OPT_CORE_TX_POWER_RECORD_START,
    MESH_OPT_CORE_TX_POWER_RECORD_END   = MESH_OPT_CORE_TX_POWER_RECORD_START + MESH_OPT_CORE_ADV_COUNT - 1,
    MESH_OPT_CORE_ADV_ADDR_RECORD_START,
    MESH_OPT_CORE_ADV_ADDR_RECORD_END   = MESH_OPT_CORE_ADV_ADDR_RECORD_START + MESH_OPT_CORE_ADV_COUNT - 1,
    MESH_OPT_CORE_SEC_NWK_BCN_RECORD,
    MESH_OPT_CORE_HB_PUBLICATION_RECORD,
    MESH_OPT_CORE_RECORDS_COUNT
};

/* The last record in the core options shouldn't go past the end of its range. */
NRF_MESH_STATIC_ASSERT((MESH_OPT_CORE_RECORDS_COUNT - 1) <= MESH_OPT_CORE_ID_END);


/** Advertiser entry id */
#define MESH_OPT_CORE_ADV_EID            MESH_OPT_CORE_ID(MESH_OPT_CORE_ADV_RECORD_START)
/** TX power entry id */
#define MESH_OPT_CORE_TX_POWER_EID       MESH_OPT_CORE_ID(MESH_OPT_CORE_TX_POWER_RECORD_START)
/** Advertiser address entry id */
#define MESH_OPT_CORE_ADV_ADDR_EID       MESH_OPT_CORE_ID(MESH_OPT_CORE_ADV_ADDR_RECORD_START)
/** Current secure network beacon state */
#define MESH_OPT_CORE_SEC_NWK_BCN_EID    MESH_OPT_CORE_ID(MESH_OPT_CORE_SEC_NWK_BCN_RECORD)
/** Heartbeat publication state */
#define MESH_OPT_CORE_HB_PUBLICATION_EID MESH_OPT_CORE_ID(MESH_OPT_CORE_HB_PUBLICATION_RECORD)

#define MESH_OPT_CORE_ADV_ENTRY_ID_TO_ROLE(ID)    ((core_tx_role_t) ((ID).record - MESH_OPT_CORE_ADV_RECORD_START))
#define MESH_OPT_CORE_ADV_ROLE_TO_ENTRY_ID(ROLE)  (MESH_OPT_CORE_ID(MESH_OPT_CORE_ADV_RECORD_START + (ROLE)))

/** @} */

typedef struct
{
    bool enabled;            /**< Advertiser enabled. */
    uint8_t tx_count;        /**< Number of transmissions for each packet. */
    uint16_t tx_interval_ms; /**< Advertisement interval in milliseconds. */
} mesh_opt_core_adv_t;

/**
 * Sets the advertisement options for the given role.
 *
 * @param[in] role    Advertiser role
 * @param[in] p_entry Options
 *
 * @retval NRF_SUCCESS The options were successfully set.
 * @retval NRF_ERROR_NULL The @p p_entry pointer was NULL.
 * @retval NRF_ERROR_INVALID_PARAMS Invalid options parameters
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_adv_set(core_tx_role_t role, const mesh_opt_core_adv_t * p_entry);

/**
 * Gets the advertisement options for the given role.
 *
 * @param[in] role        Advertiser role.
 * @param[in,out] p_entry Returns the options for the given role.
 *
 * @retval NRF_SUCCESS The options were successfully returned.
 * @retval NRF_ERROR_NULL The @p p_entry pointer was NULL.
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_adv_get(core_tx_role_t role, mesh_opt_core_adv_t * p_entry);


/**
 * Sets the TX power level for the given role.
 *
 * @param[in] role     Advertiser role
 * @param[in] tx_power TX power to set.
 *
 * @retval NRF_SUCCESS The TX power was successfully set.
 * @retval NRF_ERROR_INVALID_PARAMS Invalid options parameters
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_tx_power_set(core_tx_role_t role, radio_tx_power_t tx_power);

/**
 * Gets the TX power level for the given role.
 *
 * @param[in] role           Advertiser role
 * @param[in,out] p_tx_power Returns the current TX power setting
 *
 * @retval NRF_SUCCESS The TX power was successfully returned.
 * @retval NRF_ERROR_NULL The @p p_tx_power pointer was NULL.
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_tx_power_get(core_tx_role_t role, radio_tx_power_t * p_tx_power);


/**
 * Sets the advertisement address for the given role.
 *
 * @param[in] role       Advertiser role
 * @param[in] p_adv_addr The new advertisement address
 *
 * @retval NRF_SUCCESS The advertisement address was successfully set.
 * @retval NRF_ERROR_NULL The @p p_adv_addr pointer was NULL.
 * @retval NRF_ERROR_INVALID_PARAMS Invalid options parameters
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_adv_addr_set(core_tx_role_t role, const ble_gap_addr_t * p_adv_addr);

/**
 * Gets the advertisement address for the given role.
 *
 * @param[in] role           Advertiser role
 * @param[in,out] p_adv_addr Returns the current advertisement address
 *
 * @retval NRF_SUCCESS The advertisement address was successfully returned.
 * @retval NRF_ERROR_NULL The @p p_adv_addr pointer was NULL.
 * @retval NRF_ERROR_NOT_FOUND Unknown role.
 */
uint32_t mesh_opt_core_adv_addr_get(core_tx_role_t role, ble_gap_addr_t * p_adv_addr);

/** @} end of MESH_OPT_CORE */

#endif /* MESH_OPT_CORE_H__ */
