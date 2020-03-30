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

#ifndef MESH_FSM_H__
#define MESH_FSM_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_mesh_config_core.h"

/**
 * @defgroup FSM Finite State Machine API
 * @ingroup MESH_CORE
 * @{
 */

/**@brief   Fixed-sized type for FSM state ID.
 */
typedef uint8_t fsm_state_id_t;


/**@brief   Fixed-sized type for FSM event ID.
 */
typedef uint8_t fsm_event_id_t;


/**@brief   Fixed-sized type for FSM guard condition ID.
 */
typedef uint8_t fsm_guard_id_t;


/**@brief   Fixed-sized type for FSM action ID.
 */
typedef uint8_t fsm_action_id_t;


/**@brief   FSM transition description (item of FSM transition table).
 *
 * @details When event with given event_id occurs, the guard condition with guard_id
 *          is checked, and if it returns true, the action with action_id is performed,
 *          and state machine is switched to the state with new_state_id.
 */
typedef struct
{
    fsm_event_id_t	event_id;        /**< FSM Event ID */
    fsm_guard_id_t	guard_id;        /**< FSM Guard ID */
    fsm_action_id_t	action_id;       /**< FSM Action ID */
    fsm_state_id_t	new_state_id;    /**< New state ID */
} fsm_transition_t;


/**@brief   FSM transition declaration (item of FSM transition table).
 */
#define FSM_TRANSITION(event_id, guard_id, action_id, new_state_id)	\
                     {(event_id), (guard_id), (action_id), (new_state_id)}


/**@brief    FSM state declaration.
 *
 * @details  State is an "aggregator" item of the FSM transition table, aggregating
 *           the transitions, declared immediately after this state declaration.
 *           All transition declaration items, following the state declaration item,
 *           will be "aggregated" in this state, until the next state declaration item,
 *           or the "end of table" item.
 */
#define FSM_STATE(state_id)   {(state_id) | FSM_STATE_FLAG, 0, 0, 0}


/**@brief   Empty guard condition ID.
 *
 * @details Special value of the guard_id field. If it is used in transition declaration,
 *          guard check will be omitted.
 */
#define FSM_NO_GUARD        0xFF


/**@brief   Empty guard condition ID (useful synonym).
 *
 * @details Special value of the guard_id field. If it is used in transition declaration,
 *          guard check will be omitted.
 */
#define FSM_OTHERWISE       0xFF


/**@brief   Empty guard condition ID (useful synonym).
 *
 * @details Special value of the guard_id field. If it is used in transition declaration,
 *          guard check will be omitted.
 */
#define FSM_ALWAYS          0xFF


/**@brief   Empty action ID.
 *
 * @details Special value of the action_id field. If it is used in transition declaration,
 *          no action will be performed during the transition.
 */
#define FSM_NO_ACTION       0xFF


/**@brief   Same state ID.
 *
 * @details Special value of the next_state_id field. If it is used in transition
 *          declaration, the current state won't be changed.
 */
#define FSM_SAME_STATE      0xFF


/**@brief   Any state ID.
 *
 * @details Special value of the event_id field. If it is used in transition
 *          declaration table, then the transitions listed in this state will be applied
 *          in case they have not been listed in the transition table for the
 *          current FSM state or the transition does not have guard function.
 *          Only one FSM_STATE(FSM_ANY_STATE) can be present in transition table.
 */
#define FSM_ANY_STATE      0xFF


/**@brief   State declaration flag.
 *
 * @details Special flag of the event_id field. This flag is used to distinguish
 *          between state declaration and transition declaration.
 */
#define FSM_STATE_FLAG      0x80


/**@brief   Prototype of a user-defined FSM guard condition function.
 *
 * @details     User shall implement a single FSM guard condition function, which will
 *              take an ID of the needed guard check as a parameter.
 *
 * @param[in]   guard_id    Guard condition ID to be checked.
 * @param[in]   p_data      Additional FSM specific data.
 *
 * @retval  true    Transition is allowed, false otherwise.
 */
typedef bool (* fsm_guard_t)(fsm_guard_id_t guard_id, void * p_data);


/**@brief Prototype of a user-defined FSM action function.
 *
 * @details     User shall implement a single FSM action function, which will
 *              take an ID of the needed action as a parameter.
 *
 * @param[in]   action_id   Action ID to be performed.
 * @param[in]   p_data      Additional FSM specific data.
 */
typedef void (* fsm_action_t)(fsm_action_id_t action_id, void * p_data);


/**@brief   Constant FSM descriptor, which can reside in read-only memory.
 */
typedef struct
{
    /** Pointer to the transition table.
     */
    const fsm_transition_t * transition_table;

    /** Number of transitions in the transition table.
     */
    uint8_t                  transitions_count;

    /** Initial state ID.
     */
    fsm_state_id_t           initial_state;

    /** Pointer to the guard condition function
     */
    fsm_guard_t              guard;

    /** Pointer to the action function
     */
    fsm_action_t             action;

#if FSM_DEBUG
    /** Pointer to the string with fsm name.
     */
    const char *             fsm_name;

    /** Pointer to the array with action name strings.
     */
    const char **            action_lookup;

    /** Pointer to the array with guard name strings.
     */
    const char **            guard_lookup;

    /** Pointer to the array with event name strings.
     */
    const char **            event_lookup;

    /** Pointer to the array with state name strings.
     */
    const char **            state_lookup;
#endif
} fsm_const_descriptor_t;


/**@brief   FSM dynamic descriptor, holding the current state of the FSM.
*/
typedef struct
{
    /** Pointer to the constant FSM descriptor, which can reside in read-only memory.
     */
    const fsm_const_descriptor_t * fsm_const_desc;

    /** Index of the "any state transitions" block.
     */
    uint8_t                        any_state_transitions_index;

    /** Current state ID.
     */
    volatile fsm_state_id_t        current_state;

    /** Recursion protection
     */
    volatile uint8_t               recursion_protection;
} fsm_t;


/**@brief   Initializes specific FSM.
 *
 * @param[in]   p_fsm       Pointer to FSM descriptor to initialize.
 * @param[in]   p_fsm_const Pointer to constant FSM descriptor with transition table, etc.
 */
void fsm_init(fsm_t * p_fsm, const fsm_const_descriptor_t * p_fsm_const);


/**@brief   Posts event to FSM.
 *
 * @details This function causes FSM transition from the current state to the new state,
 *          according to the transition table of this FSM.
 *          The corresponding guard check and action is performed.
 *
 * @param[in]   p_fsm       Pointer to FSM descriptor.
 * @param[in]   event_id    Event ID to post.
 * @param[in]   p_data      Pointer to the FSM specific data.
 */
void fsm_event_post(fsm_t * p_fsm, fsm_event_id_t event_id, void * p_data);

/**
 * Returns true if the FSM is currently processing an event.
 *
 * @returns @c true if the FSM is currently processing an event.
 */
bool fsm_is_processing(const fsm_t * p_fsm);

/** @} */

#endif // MESH_FSM_H__
