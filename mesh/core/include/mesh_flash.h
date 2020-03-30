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
#ifndef MESH_FLASH_H__
#define MESH_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include "timer.h"
#include "bl_if.h"

/**
 * @defgroup MESH_FLASH Mesh flash handler
 * @ingroup MESH_CORE
 * @{
 */

/** Index of mesh flash handler users, used to index their context internally
 * to the module */
typedef enum
{
    MESH_FLASH_USER_DFU,  /**< User reserved to DFU operation. */
    MESH_FLASH_USER_MESH, /**< User reserved to storing mesh state. */
    MESH_FLASH_USER_APP,  /**< User available for application specific purposes. */
#ifdef UNIT_TEST
    MESH_FLASH_USER_TEST, /**< User reserved for unit testing. */
#endif

    MESH_FLASH_USERS      /**< Number of flash users, does not represent a valid user index. */
} mesh_flash_user_t;

/**
 * @defgroup FLASH_OPERATION_PARAMS Flash operation parameter structures
 * @{
 */

/** Parameters for write flash operation */
typedef struct
{
    uint32_t * p_start_addr; /**< The start address to write to. */
    uint32_t length;         /**< Number of bytes to write. */
    const uint32_t * p_data; /**< The data to write. */
} flash_operation_params_write_t;

/** Parameters for erase flash operation */
typedef struct
{
    uint32_t * p_start_addr; /**< The start address to erase. */
    uint32_t length; /**< Number of bytes to erase. */
} flash_operation_params_erase_t;

/** @} */

/** Single flash operation. */
typedef struct
{
    flash_op_type_t type;     /**< Type of flash operation. */
    /** Parameters for the flash operation. */
    union
    {
        flash_operation_params_write_t write; /**< Parameters for write operations. */
        flash_operation_params_erase_t erase; /**< Parameters for erase operations. */
    } params;
} flash_operation_t;

/**
 * Flash operation callback type. Used to notify user of an ended flash
 * operation.
 *
 * @param[in] user User that completed the operation.
 * @param[in] p_op Completed operation, or a special operation with type @ref FLASH_OP_TYPE_ALL
 * and invalid @c params if all operations have been completed.
 * @param[in] token Token returned in the op_push function's @p p_token parameter.
 */
typedef void (*mesh_flash_op_cb_t)(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token);

/**
 * Initialize the mesh flash module.
 */
void mesh_flash_init(void);

/**
 * Set the end-callback of the given user.
 *
 * @note The callback is NULL by default, which means that the user won't be
 * notified when the operation ended.
 * @note Will assert if the user doesn't exist.
 */
void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb);

/**
 * Push a single flash operation to the flash queue.
 *
 * @param[in] user User pushing the operation.
 * @param[in] p_op The flash operation to push.
 * @param[out] p_token An optional token that will be passed back in the end callback, or NULL if
 * the token will not be used.
 *
 * @retval NRF_SUCCESS The flash operation was successfully enqueued.
 * @retval NRF_ERROR_NOT_FOUND Couldn't find the given user.
 * @retval NRF_ERROR_NULL The @c p_op parameter was NULL.
 * @retval NRF_ERROR_INVALID_ADDR The flash operation's addresses weren't properly aligned.
 * @retval NRF_ERROR_INVALID_LENGTH The flash operation's length parameter wasn't word-aligned.
 * @retval NRF_ERROR_INVALID_PARAM The flash operation type isn't valid.
 * @retval NRF_ERROR_NO_MEM There was no more space in the given user's flash
 * queue, and the operation will not be performed.
 */
uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token);

/**
 * Get the number of flash operation slots available.
 *
 * @param[in] user User to check for.
 *
 * @returns The number of slots available in the flash write queue.
 */
uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user);

/**
 * Check whether a flash operation is currently in progress.
 *
 * @returns Whether a flash operation is currently being executed.
 */
bool mesh_flash_in_progress(void);

/**
 * Suspend or unsuspend flash operations.
 *
 * @note Safe for multiple users.
 *
 * @param[in] suspend Whether to suspend the flash operations.
 */
void mesh_flash_set_suspended(bool suspend);

/** @} */

#endif /* MESH_FLASH_H__ */
