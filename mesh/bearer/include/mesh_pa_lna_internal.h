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
#ifndef MESH_PA_LNA_INTERNAL_H__
#define MESH_PA_LNA_INTERNAL_H__

#include <stdint.h>

/**
 * @defgroup MESH_PA_LNA_INTERNAL Internal API for PA/LNA.
 * Internal API for controlling PA/LNA from the radio context.
 * @{
 */

/**
 * Sets PA up to start on the given hardware event.
 *
 * @param[in] rampup_start_capture_time Action timer value at rampup start time.
 * @param[in] timer_index Action timer index to use for setup.
 */
void mesh_pa_setup_start(uint32_t rampup_start_capture_time, uint32_t timer_index);


/**
 * Sets LNA up to start on the given hardware event.
 *
 * @param[in] rampup_start_capture_time Action timer value at rampup start time.
 * @param[in] timer_index Action timer index to use for setup.
 */
void mesh_lna_setup_start(uint32_t rampup_start_capture_time, uint32_t timer_index);

/**
 * Disables the start action set up by @ref mesh_pa_setup_start or @ref mesh_lna_setup_start.
 */
void mesh_pa_lna_disable_start(void);

/**
 * Sets PA/LNA up to stop on the next radio disabled event.
 */
void mesh_pa_lna_setup_stop(void);

/**
 * Cleans up after the previous PA/LNA signal.
 */
void mesh_pa_lna_cleanup(void);

/** @} */


#endif /* MESH_PA_LNA_INTERNAL_H__ */
