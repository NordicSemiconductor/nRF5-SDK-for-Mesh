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

#ifndef MESH_MEM_H__
#define MESH_MEM_H__

#include <stddef.h>
#include "nrf_mesh_defines.h"

/**
 * @defgroup MESH_MEM Mesh memory manager
 * @ingroup MESH_API_GROUP_CORE
 *
 * This API provides an interface for a general purpose dynamic memory manager. The API resembles
 * the standard malloc()/calloc()/free() interface.
 *
 * The standard library dynamic memory allocation functions are not used directly to allow the user
 * to use a different memory management backend than the one that the standard library provides.
 *
 * To change the memory management backend, replace the `mesh_mem_<backend>.c` to the desired
 * implementation. The default backend is `mesh_mem_stdlib.c`.
 *
 * @{
 */

/**
 * The smallest size of dynamic memory that the mesh requires to operate.
 *
 * If the dynamic memory available is less than @ref NRF_MESH_UPPER_TRANSPORT_PDU_SIZE_MAX, the
 * device cannot receive full length upper transport PDUs and may not function properly.
 */
#define MESH_MEM_SIZE_MIN NRF_MESH_UPPER_TRANSPORT_PDU_SIZE_MAX

/**
 * Initialize the memory manager.
 *
 * @note This API is called by nrf_mesh_init().
 */
void mesh_mem_init(void);

/**
 * Allocate `size` bytes and return a pointer to the allocated memory.
 *
 * @note The memory is not initialized.
 *
 * @param[in] size Size in bytes of the memory to allocate.
 *
 * @returns A pointer to the allocated memory, or `NULL` if no memory was allocated.
 */
void * mesh_mem_alloc(size_t size);

/**
 * Free the allocated memory.
 *
 * @param[in] ptr Pointer to memory that shall be freed. The pointer must be previously allocated by
 *                a call to mesh_mem_alloc() or mesh_mem_calloc(). Otherwise, the behavior is
 *                undefined.
 */
void mesh_mem_free(void * ptr);

/**
 * Allocate memory for an array of `nmemb` elements of `size` bytes.
 *
 * @note The memory returned is set to zero.
 *
 * @param[in] nmemb Number of members in the array.
 * @param[in] size  Size in bytes of the individual members.
 *
 * @returns A pointer to the allocated memory, or `NULL` if no memory was allocated.
 */
void * mesh_mem_calloc(size_t nmemb, size_t size);

/** @} end of MESH_MEM */

#endif /* MESH_MEM_H__ */
