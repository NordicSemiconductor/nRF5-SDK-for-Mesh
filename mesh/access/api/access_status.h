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

#ifndef ACCESS_STATUS_H__
#define ACCESS_STATUS_H__

/**
 * @ingroup ACCESS_TYPES
 * @{
 */

/** Access layer status codes. */
typedef enum
{
    ACCESS_STATUS_SUCCESS                            = 0x00, /**< Command successfully processed. */
    ACCESS_STATUS_INVALID_ADDRESS                    = 0x01, /**< The provided value is not a valid address in this context. */
    ACCESS_STATUS_INVALID_MODEL                      = 0x02, /**< The provided Model index is not valid in this Element. */
    ACCESS_STATUS_INVALID_APPKEY                     = 0x03, /**< The provided AppKey index is not valid for this node. */
    ACCESS_STATUS_INVALID_NETKEY                     = 0x04, /**< The provided NetKey index is not valid for this node. */
    ACCESS_STATUS_INSUFFICIENT_RESOURCES             = 0x05, /**< The node cannot store provided information due to insufficient resources on node. */
    ACCESS_STATUS_KEY_INDEX_ALREADY_STORED           = 0x06, /**< The key with given index is already stored in the node with a different value. */
    ACCESS_STATUS_NOT_A_PUBLISH_MODEL                = 0x07, /**< The referenced Model is not a Publish Model. */
    ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL              = 0x08, /**< The referenced Model is not a Subscribe Model. */
    ACCESS_STATUS_STORAGE_FAILURE                    = 0x09, /**< The node was not able to store new value in persistent storage . */
    ACCESS_STATUS_FEATURE_NOT_SUPPORTED              = 0x0A, /**< The feature is not supported in this node. */
    ACCESS_STATUS_CANNOT_UPDATE                      = 0x0B, /**< The requested update operation cannot be performed due to general constraints. */
    ACCESS_STATUS_CANNOT_REMOVE                      = 0x0C, /**< The requested delete operation cannot be performed due to general constraints. */
    ACCESS_STATUS_CANNOT_BIND                        = 0x0D, /**< The requested bind operation cannot be performed due to general constraints. */
    ACCESS_STATUS_TEMPORARILY_UNABLE_TO_CHANGE_STATE = 0x0E, /**< The requested operation temporarily cannot be performed due to internal state of the node. */
    ACCESS_STATUS_CANNOT_SET                         = 0x0F, /**< The requested set operation cannot be performed due to general constraints. */
    ACCESS_STATUS_UNSPECIFIED_ERROR                  = 0x10, /**< An error occurred that does not correspond to any error conditions defined for a given state. */
    ACCESS_STATUS_INVALID_BINDING                    = 0x11, /**< The NetKeyIndex and AppKeyIndex combination is not valid. */
} access_status_t;

/** @} ACCESS_TYPES */
#endif
