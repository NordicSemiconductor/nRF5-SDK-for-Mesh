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

#ifndef BEARER_DEFINES_H__
#define BEARER_DEFINES_H__

/**
 * @defgroup MESH_DEFINES_BEARER Defines
 * @ingroup MESH_API_GROUP_BEARER
 * Common definitions for the bearer layer.
 * @{
 */

/** Spec-defined access address for non-connected state */
#define BEARER_ACCESS_ADDR_NONCONN 0x8E89BED6U

/** Lower boundary on advertisement interval. */
#define BEARER_ADV_INT_MIN_MS (20)

/** Upper boundary on advertisement interval. */
#define BEARER_ADV_INT_MAX_MS (10240)

/** Lower Boundary on scan window. */
#define BEARER_SCAN_WIN_MIN_MS (3)

/** Upper Boundary on scan interval. */
#define BEARER_SCAN_INT_MAX_MS (40960)

/** Largest high frequency clock drift allowed by the Bluetooth specification. */
#define BEARER_HFCLK_DRIFT_PPM_MAX  (50)

/** @} end of MESH_DEFINES_BEARER */

/** @} */
#endif  /* BEARER_DEFINES_H__ */
