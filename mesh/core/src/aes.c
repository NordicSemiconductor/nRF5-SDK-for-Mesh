/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <string.h>

#ifdef SOFTDEVICE_PRESENT
#define ECB_ENCRYPT_TIME_WORST_CASE_US 50
#include "nrf_soc.h"
#endif

#include "nrf_error.h"
#include "nrf.h"

#include "aes.h"
#include "timeslot.h"
#include "toolchain.h"

typedef struct
{
    uint8_t key[16];
    uint8_t clear_text[16];
    uint8_t cipher_text[16];
} aes_data_t;

#if !defined(SOFTDEVICE_PRESENT)
static void aes_encrypt_hw(aes_data_t * p_aes_data)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    NRF_ECB->ECBDATAPTR = (uint32_t) p_aes_data;

    NRF_ECB->EVENTS_ENDECB  = 0;
    NRF_ECB->TASKS_STARTECB = 1;

    while (NRF_ECB->EVENTS_ENDECB == 0)
    {
        ; /* Wait for hardware to complete */
    }

    NRF_ECB->EVENTS_ENDECB = 0;
    _ENABLE_IRQS(was_masked);
}
#endif

void aes_encrypt(const uint8_t * const key, const uint8_t * const clear_text, uint8_t * const cipher_text)
{
    aes_data_t aes_data;
    memcpy(aes_data.key, key, NRF_MESH_KEY_SIZE);
    memcpy(aes_data.clear_text, clear_text, NRF_MESH_KEY_SIZE);

#if (defined(NRF51) || defined(NRF52_SERIES)) && SOFTDEVICE_PRESENT
    (void) sd_ecb_block_encrypt((nrf_ecb_hal_data_t *) &aes_data);
#else
    aes_encrypt_hw(&aes_data);
#endif
    memcpy(cipher_text, aes_data.cipher_text, NRF_MESH_KEY_SIZE);
}
