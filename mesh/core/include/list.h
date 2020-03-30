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

#ifndef MESH_LIST_H__
#define MESH_LIST_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <nrf_error.h>
#include "nrf_mesh_assert.h"

/**
 * @defgroup LINKED_LIST Generic linked list implementation.
 * @ingroup MESH_CORE
 * @{
 */

/**
 * Declares a for-loop for iterating linked lists.
 *
 * @param[in] ITEM Item pointer variable name.
 * @param[in] LIST Pointer to the head of the linked list.
 *
 * Usage
 * @code{.c}
 * LIST_FOREACH(p_item, p_my_list_head)
 * {
 *     my_struct_t * p_struct_item = PARENT_BY_FIELD_GET(my_struct_t, list_node, p_item);
 *     ...
 * }
 * @endcode
 */
#define LIST_FOREACH(ITEM, LIST)                                        \
    for (const list_node_t * (ITEM) = (LIST); (ITEM) != NULL; (ITEM) = (ITEM)->p_next)

/**
 * Generic linked list node.
 */
typedef struct list_node
{
    /** Pointer to the next item in the list. */
    struct list_node * p_next;
} list_node_t;

/**
 * List node compare function callback.
 *
 * @param[in] p1 Pointer to first node.
 * @param[in] p2 Pointer to second node.
 *
 * @returns @c true or @c false, depending on context.
 */
typedef bool (*list_cmp_cb_t)(const list_node_t * p1, const list_node_t * p2);

/**
 * Insert a new item at a given point in a list.
 *
 * @param[in,out] p_node The node at which the new node is inserted after.
 * @param[in,out] p_new  The new node to be inserted.
 */
static inline void list_insert(list_node_t * p_node, list_node_t * p_new)
{
    NRF_MESH_ASSERT(p_node != NULL && p_new != NULL);

    if (p_node != p_new)
    {
        p_new->p_next  = p_node->p_next;
        p_node->p_next = p_new;
    }
}

/**
 * Add an item at the end of the linked list.
 *
 * @param[in,out] pp_head Pointer to the head pointer of the linked list.
 * @param[in,out] p_new   New item to be pushed to the list.
 */
void list_add(list_node_t ** pp_head, list_node_t * p_new);

/**
 * Remove an item from the list.
 *
 * @param[in,out] pp_head Pointer to the head pointer of the list.
 * @param[in,out] p_node  Node to remove from the list.
 *
 * @retval NRF_SUCCESS         Successfully removed item.
 * @retval NRF_ERROR_NOT_FOUND Item not found in the list.
 */
uint32_t list_remove(list_node_t ** pp_head, list_node_t * p_node);

/**
 * Get the size of a list.
 *
 * @param[in] p_head Pointer to the head of the list.
 *
 * @return size Size of the linked list. 0 if an invalid pointer is supplied.
 */
uint32_t list_size_get(list_node_t * p_head);

/**
 * Adds an item to the list based on sort criterion.
 *
 * @param[in,out] pp_head Linked list head double pointer.
 * @param[in,out] p_new   Node to be added to the list.
 * @param[in]     cmp_cb  Comparison callback function.
 */
void list_sorted_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb);

/**
 * Adds an element to the end of a list if comparison returns false for every existing element.
 *
 * @param[in,out] pp_head Pointer to the head of the list.
 * @param[in,out] p_new   Pointer to the new element to add.
 * @param[in]     cmp_cb  Comparison callback function pointer. Returns @c true if two elements are
 *                        considered equal.
 *
 * @retval NRF_SUCCESS         Successfully added element to list.
 * @retval NRF_ERROR_FORBIDDEN Duplicate element added to list.
 */
uint32_t list_compare_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb);

/**
 * Adds an element to the end of a list if comparison returns false for every existing element.
 *
 * @param[in,out] pp_head Pointer to the head of the list.
 * @param[in,out] p_new   Pointer to the new element to add.
 * @param[in]     cmp_cb  Comparison callback function pointer. Returns @c true if two elements are
 *                        considered equal.
 *
 * @retval NRF_SUCCESS         Successfully added element to list.
 * @retval NRF_ERROR_FORBIDDEN Duplicate element added to list.
 */

/** @} */
#endif
