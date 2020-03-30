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
#ifndef COMPOSITION_DATA_H
#define COMPOSITION_DATA_H

#include <stdint.h>

/**
 * @defgroup COMPOSITION_DATA Composition data module
 * @ingroup CONFIG_MODEL
 * Composition data part of the config model.
 *
 * The composition data is the presentation of the model and element composition of a mesh device.
 * @{
 */

/** Number of bytes used to represent a Bluetooth SIG model identifier in the composition data. */
#define CONFIG_SIG_MODEL_ID_SIZE (2)
/** Number of bytes used to represent a vendor specific model identifier in the composition data. */
#define CONFIG_VENDOR_MODEL_ID_SIZE (4)
/** Size of the largest model identifier. */
#define CONFIG_MODEL_ID_SIZE_MAX (CONFIG_VENDOR_MODEL_ID_SIZE)

/** Minimum size of a composition data status block. */
#define COMPOSITION_DATA_LENGTH_MIN (sizeof(config_msg_composition_data_status_t) + \
                                     sizeof(config_composition_data_header_t) + \
                                     sizeof(config_composition_element_header_t) + \
                                     CONFIG_SIG_MODEL_ID_SIZE)

/*lint -align_max(push) -align_max(1) */

/** Composition data header. */
typedef struct __attribute((packed))
{
    /** Company ID. */
    uint16_t company_id;
    /** Product ID */
    uint16_t product_id;
    /** Version ID. */
    uint16_t version_id;
    /** Number of entries in the replay protection list. */
    uint16_t replay_cache_entries;
    /** Features supported. @see config_feature_bit_t*/
    uint16_t features;
} config_composition_data_header_t;

/** Access element header fields in the composition data. */
typedef struct __attribute((packed))
{
    /** Location of the element. */
    uint16_t location;
    /** Number of Bluetooth SIG models. */
    uint8_t sig_model_count;
    /** Number of vendor specific models. */
    uint8_t vendor_model_count;
} config_composition_element_header_t;

/*lint -align_max(pop) */

/** The (maximum) size of the composition data block. */
#define CONFIG_COMPOSITION_DATA_SIZE                                    \
    (sizeof(config_composition_data_header_t) +                         \
     (ACCESS_ELEMENT_COUNT) * sizeof(config_composition_element_header_t) + \
     (ACCESS_MODEL_COUNT) * CONFIG_MODEL_ID_SIZE_MAX)                   \


/** Configuration feature bits. */
typedef enum
{
    CONFIG_FEATURE_RELAY_BIT      = (1 << 0), /**< This node supports the relay feature. */
    CONFIG_FEATURE_PROXY_BIT      = (1 << 1), /**< This node supports the proxy feature. */
    CONFIG_FEATURE_FRIEND_BIT     = (1 << 2), /**< This node supports the friend feature. */
    CONFIG_FEATURE_LOW_POWER_BIT  = (1 << 3)  /**< This node supports the low power feature. */
} config_feature_bit_t;

/**
 * Gets the composition data block.
 *
 * @param[in,out] p_data Pointer to block of memory to write the composition data.
 * @param[in,out] p_size Size of the data block. Actual size is written back to the variable.
 *
 * @retval NRF_SUCCESS              Successfully wrote the composition data block.
 * @retval NRF_ERROR_NULL           Function parameter was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH Data block size too small.
 */
void config_composition_data_get(uint8_t * p_data, uint16_t * p_size);

/** @} */
#endif /* CONFIG_SERVER_H__ */
