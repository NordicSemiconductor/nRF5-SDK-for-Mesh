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

#ifndef ACCESS_PUBLISH_H__
#define ACCESS_PUBLISH_H__

#include <stdint.h>
#include "access.h"

/**
 * @defgroup ACCESS_PUBLICATION Access layer periodic publishing internal interface
 * @ingroup ACCESS
 * The publication module provides functionality for scheduling periodic publications for a model.
 * @{
 */


/**
 * Model publication state.
 * @todo Handle publication with friendship security material.
 */
typedef struct __access_model_publication_state_t
{
    /** Model handle of the model this publication state belongs to. */
    access_model_handle_t model_handle;
    /** Publish period. */
    access_publish_period_t period;
    /** Callback called for each publishing event. */
    access_publish_timeout_cb_t publish_timeout_cb;
    /** Target time in units of 100 ms for when the next publishing operation is triggered. */
    uint32_t target;
    /** Pointer to the next publication event. */
    struct __access_model_publication_state_t * p_next;
} access_model_publication_state_t;

/**
 * Initializes the access layer publication module.
 */
void access_publish_init(void);

/**
 * Stops and clears the access layer publication module.
 */
void access_publish_clear(void);

/**
 * Sets the publishing period for a model.
 * @param[in] p_pubstate  Model publication state.
 * @param[in] resolution  Resolution of the publication timer.
 * @param[in] step_number Number of steps at the specified resolution per publication event.
 */
void access_publish_period_set(access_model_publication_state_t * p_pubstate, access_publish_resolution_t resolution, uint8_t step_number);

/**
 * Retrieves the publishing period for a model.
 * @param[in]  p_pubstate    Model publication state.
 * @param[out] p_resolution  Pointer to a variable where the timer resolution is returned.
 * @param[out] p_step_number Pointer to a variable where the number of steps per publication event is returned.
 */
void access_publish_period_get(const access_model_publication_state_t * p_pubstate, access_publish_resolution_t * p_resolution, uint8_t * p_step_number);

/** @} */

#endif

