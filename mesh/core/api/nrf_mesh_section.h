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
#ifndef NRF_MESH_SECTION_H__
#define NRF_MESH_SECTION_H__

#include "nrf_section.h"
#include "nrf_mesh_assert.h"

/**
 * @defgroup NRF_MESH_SECTION Mesh section variables
 * @ingroup NRF_MESH
 * Mesh wrapper for the nRF5 SDK section variables.
 *
 * Allows all mesh section variables to stay under a single linker script entry in both flash and ram.
 * @{
 */

#if defined(__GNUC__) && !defined(HOST)

#define NRF_MESH_SECTION_START(section_name) CONCAT_2(__nrf_mesh_start_, section_name)
#define NRF_MESH_SECTION_END(section_name)   CONCAT_2(__nrf_mesh_stop_, section_name)

#define NRF_MESH_SECTION_ITEM_REGISTER_FLASH(section_name, section_var) section_var __attribute__ ((section(".nrf_mesh_flash." STRINGIFY(CONCAT_2(section_name, __1))))) __attribute__((used))

#define NRF_MESH_SECTION_ITEM_REGISTER_RAM(section_name, section_var)   section_var __attribute__ ((section(".nrf_mesh_ram." STRINGIFY(CONCAT_2(section_name, __1))))) __attribute__((used))


#define NRF_MESH_SECTION_DEF_FLASH(section_name, data_type)   \
    volatile data_type NRF_MESH_SECTION_START(section_name)[0] __attribute__ ((section(".nrf_mesh_flash." STRINGIFY(CONCAT_2(section_name, __0))))) __attribute__((used)); \
    volatile data_type NRF_MESH_SECTION_END(section_name)[0]   __attribute__ ((section(".nrf_mesh_flash." STRINGIFY(CONCAT_2(section_name, __2))))) __attribute__((used))

#define NRF_MESH_SECTION_DEF_RAM(section_name, data_type)   \
    volatile data_type NRF_MESH_SECTION_START(section_name)[0] __attribute__ ((section(".nrf_mesh_ram." STRINGIFY(CONCAT_2(section_name, __0))))) __attribute__((used)); \
    volatile data_type NRF_MESH_SECTION_END(section_name)[0]   __attribute__ ((section(".nrf_mesh_ram." STRINGIFY(CONCAT_2(section_name, __2))))) __attribute__((used))


#define NRF_MESH_SECTION_LENGTH(section_name)           \
    ((size_t)&NRF_MESH_SECTION_END(section_name)[0] -       \
     (size_t)&NRF_MESH_SECTION_START(section_name)[0])

#define NRF_MESH_SECTION_ITEM_GET(section_name, data_type, i)                                      \
    (data_type *) (NRF_MESH_SECTION_START(section_name) + i)

#define NRF_MESH_SECTION_ITEM_COUNT(section_name, data_type) \
    NRF_MESH_SECTION_LENGTH(section_name) / sizeof(data_type)

#else

#define NRF_MESH_SECTION_START(section_name) NRF_SECTION_START_ADDR(section_name)
#define NRF_MESH_SECTION_END(section_name)   NRF_SECTION_END_ADDR(section_name)

#define NRF_MESH_SECTION_ITEM_REGISTER_FLASH(section_name, section_var) NRF_SECTION_ITEM_REGISTER(section_name, section_var)
#define NRF_MESH_SECTION_ITEM_REGISTER_RAM(section_name, section_var) NRF_SECTION_ITEM_REGISTER(section_name, section_var)

#define NRF_MESH_SECTION_DEF_FLASH(section_name, data_type) NRF_SECTION_DEF(section_name, data_type)
#define NRF_MESH_SECTION_DEF_RAM(section_name, data_type) NRF_SECTION_DEF(section_name, data_type)

#define NRF_MESH_SECTION_LENGTH(section_name) NRF_SECTION_LENGTH(section_name)

#define NRF_MESH_SECTION_ITEM_GET(section_name, data_type, i) NRF_SECTION_ITEM_GET(section_name, data_type, i)
#define NRF_MESH_SECTION_ITEM_COUNT(section_name, data_type) NRF_SECTION_ITEM_COUNT(section_name, data_type)

#endif

#define NRF_MESH_SECTION_FOR_EACH(section_name, data_type, variable)                               \
    for (data_type * variable = (data_type *) NRF_MESH_SECTION_START(section_name);                \
         (intptr_t) variable != (intptr_t) NRF_MESH_SECTION_END(section_name);                         \
         variable++)

/** @} */

#endif /* NRF_MESH_SECTION_H__ */
