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

#ifndef SCENE_COMMON_H__
#define SCENE_COMMON_H__

#include <stdint.h>
#include "model_common.h"

/**
 * @defgroup SCENE_MODELS Scene models
 * @ingroup MESH_API_GROUP_TIME_SCENE_MODELS
 *
 * This model implements the message based interface required to set the Scene Setup server states.
 * The model sends its state information to the mid app to be stored in flash (to be read at boot
 * time) and use it appropriately. Scenes are the stored states of a device that can be recalled
 * using messges or at a given time.
 * @{
 */

/** Model Company ID */
#define SCENE_COMPANY_ID 0xFFFF

/**
 * Minimum allowed value of the Scene Register Number defined in @tagMeshMdlSp section 5.1.3.1.
 * */
#define SCENE_REGISTER_SCENE_NUMBER_MIN (1)

/**
 * Defines the default value for the Current Scene from @tagMeshMdlSp section 5.1.3.2 and Target
 * Scene from @tagMeshMdlSp section 5.1.3.3.
 */
#define SCENE_NUMBER_NO_SCENE (0x0000)

/* See "@tagMeshDevPr, section 4.1.3".  0 is a prohibited value for property ID, so it can represent
 * "no property id" without concern for a collision.
 */
#define SCENE_NO_PROPERTY_ID 0

/** Maximum value of number of scenes, as defined in @tagMeshMdlSp */
#define SCENE_SCENES_MAX (65535)

/** Scene Register size definitions
 *
 * Although the Scene Register array has a variable size, it is never bigger than 16 elements. So
 * we set 16 elements to be used as array size. See @tagMeshMdlSp section 5.1.3.1.
 *
 * @warning Changing this value to anything other than `16` makes the Scene Model non-compliant with
 * the @tagMeshMdlSp.
 */
#ifndef SCENE_REGISTER_ARRAY_SIZE
#define SCENE_REGISTER_ARRAY_SIZE (16)
#endif

/**
 * Status code values used by @ref scene_status_params_t and @ref scene_register_status_params_t.
 */
typedef enum
{
    /** The previous operation is valid */
    SCENE_STATUS_SUCCESS,
    /** The previous operation is failed because register was full */
    SCENE_STATUS_REGISTER_FULL,
    /** The previous operation is failed because scene was not found */
    SCENE_STATUS_NOT_FOUND
} scene_status_t;

/** Parameters for the Scene Store message. */
typedef struct
{
    uint16_t scene_number;              /**< Number of the scene to be stored */
} scene_store_params_t;


/** Parameters for the Scene Recall message. */
typedef struct
{
    uint16_t scene_number;              /**< Number of the scene to be stored */
    uint8_t tid;                        /**< Transaction ID */
} scene_recall_params_t;

/** Parameters for the Scene Status message. */
typedef struct
{
    uint8_t status_code;                /**< Status code for the last operation */
    uint16_t current_scene;             /**< Scene number of a current scene */
    uint16_t target_scene;              /**< Scene number of a target scene */
    uint32_t remaining_time_ms;         /**< Remaining transition time in milliseconds */
} scene_status_params_t;

/** Parameters for the Scene Register Status message. */
typedef struct
{
    uint8_t status_code;                        /**< Status code for the last operation */
    uint16_t current_scene;                     /**< Scene number of a current scene */
    uint16_t scenes[SCENE_REGISTER_ARRAY_SIZE]; /**< A list of scenes stored within an element */
} scene_register_status_params_t;

/** Parameters for the Scene Delete message. */
typedef struct
{
    uint16_t scene_number;              /**< Number of the scene to be deleted */
} scene_delete_params_t;

/**@} end of SCENE_MODELS */
#endif /* SCENE_COMMON_H__ */
