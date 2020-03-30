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
#include "mesh_adv.h"
#include "ble_advdata.h"
#include "ble_gap.h"
#include "ble_types.h"
#include "app_error.h"
#include "timeslot.h"

/**
 * This is a sample implementation of the `mesh_adv.h` interface.
 * You may change the implementation as long as the interface remains
 * the same. Note that requirements in @tagMeshSp
 * chapter 7, restricts available bytes in the advertisement data. However,
 * the scan response data is freely available.
 *
 * The implementation adheres to both the new advertisement API in the
 * SoftDevice v6.0.0 as well as the older legacy APIs.
 */

#if NRF_SD_BLE_API_VERSION >= 6
#define MESH_ADV_ADV_TYPE BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED
#define MESH_ADV_DATA_SIZE_MAX BLE_GAP_ADV_SET_DATA_SIZE_MAX

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static ble_gap_adv_data_t m_gap_adv_data;
static ble_gap_adv_params_t m_adv_params =
{
    .properties = {
        .type = MESH_ADV_ADV_TYPE,
    },
    .interval = MESH_ADV_INTERVAL_DEFAULT,
    .duration = MESH_ADV_TIMEOUT_INFINITE
};

#else  /* NRF_SD_BLE_API_VERSION == 5 */

#define MESH_ADV_ADV_TYPE BLE_GAP_ADV_TYPE_ADV_IND
#define MESH_ADV_DATA_SIZE_MAX BLE_GAP_ADV_MAX_SIZE

/* Recreation of the ble_gap_adv_data_t type in the SoftDevice version 6 API. */
static struct
{
    ble_data_t adv_data;
    ble_data_t scan_rsp_data;
} m_gap_adv_data;

static ble_gap_adv_params_t m_adv_params =
{
    .type = MESH_ADV_ADV_TYPE,
    .interval = MESH_ADV_INTERVAL_DEFAULT,
    .timeout = MESH_ADV_TIMEOUT_INFINITE
};

#endif  /* NRF_SD_BLE_API_VERSION >= 6 */

/**
 * In the SoftDevice v6.0.0 API, the advertisement data buffers are owned by the application.
 * If we want to change the advertisement data while the advertiser is running, we need to double
 * buffer the data.
 */
static uint8_t m_advdata_raw[2][MESH_ADV_DATA_SIZE_MAX];
static uint8_t m_srdata_raw[2][MESH_ADV_DATA_SIZE_MAX];
static uint8_t m_adv_set_index;

void mesh_adv_data_set(uint16_t service_uuid, const uint8_t * p_service_data, uint8_t length)
{
    memset(&m_gap_adv_data, 0, sizeof(m_gap_adv_data));
    m_gap_adv_data.adv_data.p_data      = m_advdata_raw[m_adv_set_index];
    m_gap_adv_data.scan_rsp_data.p_data = m_srdata_raw[m_adv_set_index];
    m_gap_adv_data.adv_data.len         = sizeof(m_advdata_raw[0]);
    m_gap_adv_data.scan_rsp_data.len    = sizeof(m_srdata_raw[m_adv_set_index]);
    m_adv_set_index = (m_adv_set_index + 1) & 1;

    ble_advdata_t advdata;
    memset(&advdata, 0, sizeof(advdata));
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t uuids[] = {{service_uuid, BLE_UUID_TYPE_BLE}};
    advdata.uuids_complete.uuid_cnt = ARRAY_SIZE(uuids);
    advdata.uuids_complete.p_uuids = uuids;

    ble_advdata_service_data_t service_data;
    service_data.service_uuid = service_uuid;
    service_data.data.size = length;
    service_data.data.p_data = (uint8_t *) p_service_data;

    advdata.p_service_data_array = &service_data;
    advdata.service_data_count = 1;

    /* The application may freely set anything into the scan response data. */
    ble_advdata_t srdata;
    memset(&srdata, 0, sizeof(srdata));
    srdata.name_type = BLE_ADVDATA_FULL_NAME;

    APP_ERROR_CHECK(ble_advdata_encode(&advdata, m_gap_adv_data.adv_data.p_data, &m_gap_adv_data.adv_data.len));
    APP_ERROR_CHECK(ble_advdata_encode(&srdata, m_gap_adv_data.scan_rsp_data.p_data, &m_gap_adv_data.scan_rsp_data.len));

#if NRF_SD_BLE_API_VERSION >= 6

    uint32_t err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_gap_adv_data, &m_adv_params);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_gap_adv_data, NULL);
    }
#else

    uint32_t err_code = sd_ble_gap_adv_data_set(m_gap_adv_data.adv_data.p_data,
                                                m_gap_adv_data.adv_data.len,
                                                m_gap_adv_data.scan_rsp_data.p_data,
                                                m_gap_adv_data.scan_rsp_data.len);
#endif  /* NRF_SD_BLE_API_VERSION >= 6 */
    APP_ERROR_CHECK(err_code);
}

void mesh_adv_params_set(uint32_t timeout_ms, uint32_t interval_units)
{
#if NRF_SD_BLE_API_VERSION >= 6
    uint32_t scaled_timeout = MIN(UINT16_MAX, timeout_ms / 10);   /* 10 ms steps */
    if (timeout_ms > 0 && scaled_timeout == 0)
    {
        scaled_timeout = 1;
    }
    m_adv_params.duration = scaled_timeout;
#else
    uint32_t scaled_timeout = MIN(0x3FFF, timeout_ms / 1000);   /* 1 s steps, Max: 0x3FFF. */
    if (timeout_ms > 0 && scaled_timeout == 0)
    {
        scaled_timeout = 1;
    }
    m_adv_params.timeout = scaled_timeout;
#endif
    m_adv_params.interval = interval_units;
}

void mesh_adv_start(void)
{
#if NRF_SD_BLE_API_VERSION >= 6
    APP_ERROR_CHECK(sd_ble_gap_adv_start(m_adv_handle,  MESH_SOFTDEVICE_CONN_CFG_TAG));
#else
    APP_ERROR_CHECK(sd_ble_gap_adv_start(&m_adv_params,  MESH_SOFTDEVICE_CONN_CFG_TAG));
#endif
    /* We restart the mesh timeslot to yield time for the softdevice advertiser to start. */
    timeslot_restart(TIMESLOT_PRIORITY_LOW);
}

void mesh_adv_stop(void)
{
#if NRF_SD_BLE_API_VERSION >= 6
    APP_ERROR_CHECK(sd_ble_gap_adv_stop(m_adv_handle));
#else
    APP_ERROR_CHECK(sd_ble_gap_adv_stop());
#endif
}
