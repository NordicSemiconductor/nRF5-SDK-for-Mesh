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
#include "composition_data.h"

#include <stdint.h>
#include <string.h>

#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"

static uint16_t composition_data_vendor_models_write(uint16_t element_index, const access_model_handle_t * p_handles, uint16_t count, uint8_t * p_buffer)
{
    uint16_t bytes = 0;
    access_model_id_t model_id;
    for (uint32_t i = 0; i < count; ++i)
    {
        NRF_MESH_ASSERT(access_model_id_get(p_handles[i], &model_id) == NRF_SUCCESS);
        if (model_id.company_id != ACCESS_COMPANY_ID_NONE)
        {
            /* Little-endian */
            memcpy(&p_buffer[bytes], (void *) &model_id.company_id, sizeof(model_id.company_id));
            bytes += sizeof(model_id.company_id);
            memcpy(&p_buffer[bytes], (void *) &model_id.model_id, sizeof(model_id.model_id));
            bytes += sizeof(model_id.model_id);
        }
    }
    return bytes;
}

static uint16_t composition_data_sig_models_write(uint16_t element_index, const access_model_handle_t * p_handles, uint16_t count, uint8_t * p_buffer)
{
    uint16_t bytes = 0;
    access_model_id_t model_id;
    for (uint32_t i = 0; i < count; ++i)
    {
        NRF_MESH_ASSERT(access_model_id_get(p_handles[i], &model_id) == NRF_SUCCESS);
        if (model_id.company_id == ACCESS_COMPANY_ID_NONE)
        {
            /* Little-endian */
            memcpy(&p_buffer[bytes], (void *) &model_id.model_id, sizeof(model_id.model_id));
            bytes += sizeof(model_id.model_id);
        }
    }
    return bytes;
}

void config_composition_data_get(uint8_t * p_data, uint16_t * p_size)
{
    NRF_MESH_ASSERT(p_data != NULL && p_size != NULL);
    NRF_MESH_ASSERT(*p_size >= CONFIG_COMPOSITION_DATA_SIZE);

    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    uint16_t element_models_count = 0;
    config_composition_data_header_t device;

    device.company_id = DEVICE_COMPANY_ID;
    device.product_id = DEVICE_PRODUCT_ID;
    device.version_id = DEVICE_VERSION_ID;
    device.replay_cache_entries = REPLAY_CACHE_ENTRIES;
    device.features = 0;

#if MESH_FEATURE_GATT_PROXY_ENABLED
    device.features |= CONFIG_FEATURE_PROXY_BIT;
#endif

#if MESH_FEATURE_LPN_ENABLED
    device.features |= CONFIG_FEATURE_LOW_POWER_BIT;
#endif

#if MESH_FEATURE_RELAY_ENABLED
    device.features |= CONFIG_FEATURE_RELAY_BIT;
#endif

#if MESH_FEATURE_FRIEND_ENABLED
    device.features |= CONFIG_FEATURE_FRIEND_BIT;
#endif

    memcpy(&p_data[0], &device, sizeof(device));
    *p_size = sizeof(device);

    for (int i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        config_composition_element_header_t element;

        NRF_MESH_ERROR_CHECK(access_element_sig_model_count_get(i, &element.sig_model_count));
        NRF_MESH_ERROR_CHECK(access_element_vendor_model_count_get(i, &element.vendor_model_count));
        /* Can not use element directly because taking address of packed member of 'struct
           <anonymous>' may result in an unaligned pointer value. Compiler warning with GNU Tools
           ARM Embedded 9-2019-q4-major. */
        uint16_t location;
        NRF_MESH_ASSERT(access_element_location_get(i, &location) == NRF_SUCCESS);
        element.location = location;

        element_models_count = ACCESS_MODEL_COUNT;
        NRF_MESH_ASSERT(access_element_models_get(i, &model_handles[0], &element_models_count) == NRF_SUCCESS);

        memcpy(&p_data[*p_size], &element, sizeof(element));
        *p_size += sizeof(element);

        if (element.sig_model_count > 0)
        {
            *p_size += composition_data_sig_models_write(i, model_handles, element_models_count, &p_data[*p_size]);
        }
        if (element.vendor_model_count > 0)
        {
            *p_size += composition_data_vendor_models_write(i, model_handles, element_models_count, &p_data[*p_size]);
        }
    }
}
