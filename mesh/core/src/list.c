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

#include "list.h"

void list_add(list_node_t ** pp_head, list_node_t * p_new)
{
    NRF_MESH_ASSERT(pp_head != NULL && p_new != NULL);
    NRF_MESH_ASSERT(*pp_head != p_new);

    if (*pp_head == NULL)
    {
        *pp_head           = p_new;
        (*pp_head)->p_next = NULL;    /* sanitize */
    }
    else
    {
        list_node_t * p_node = *pp_head;
        while (p_node->p_next != NULL)
        {
            p_node = p_node->p_next;
            NRF_MESH_ASSERT(p_node != p_new);
        }

        list_insert(p_node, p_new);
    }
}

uint32_t list_remove(list_node_t ** pp_head, list_node_t * p_node)
{
    NRF_MESH_ASSERT(p_node != NULL && pp_head != NULL);

    if (*pp_head == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (p_node == *pp_head)
    {
        *pp_head       = p_node->p_next;
        p_node->p_next = NULL;

        return NRF_SUCCESS;
    }

    list_node_t * p_item = *pp_head;
    while (p_item->p_next != NULL && p_item->p_next != p_node)
    {
        p_item = p_item->p_next;
    }

    if (p_item == NULL || p_item->p_next != p_node)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    p_item->p_next = p_node->p_next;
    p_node->p_next = NULL;

    return NRF_SUCCESS;
}

uint32_t list_size_get(list_node_t * p_head)
{
    if (p_head == NULL)
    {
        return 0;
    }

    uint32_t i = 1;
    while (p_head->p_next != NULL)
    {
        p_head = p_head->p_next;
        ++i;
    }

    return i;
}

void list_sorted_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb)
{
    NRF_MESH_ASSERT(pp_head != NULL && p_new != NULL && cmp_cb != NULL);
    NRF_MESH_ASSERT(*pp_head != p_new);

    if (*pp_head == NULL)
    {
        *pp_head           = p_new;
        (*pp_head)->p_next = NULL;    /* sanitize */
    }
    else
    {
        if (cmp_cb(p_new, *pp_head))
        {
            p_new->p_next = *pp_head;
            *pp_head = p_new;
        }
        else
        {
            list_node_t * p_node = (*pp_head)->p_next;
            list_node_t * p_prev = *pp_head;
            while (p_node != NULL &&
                   cmp_cb(p_node, p_new))
            {
                p_prev = p_node;
                p_node = p_node->p_next;
                NRF_MESH_ASSERT(p_node != p_new);
            }

            list_insert(p_prev, p_new);
        }
    }
}

uint32_t list_compare_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb)
{
    NRF_MESH_ASSERT(pp_head != NULL && p_new != NULL && cmp_cb != NULL);
    NRF_MESH_ASSERT(*pp_head != p_new);

    if (*pp_head == NULL)
    {
        *pp_head           = p_new;
        (*pp_head)->p_next = NULL;    /* sanitize */
        return NRF_SUCCESS;
    }
    else
    {

        list_node_t * p_node = (*pp_head);
        list_node_t * p_prev = NULL;
        while (p_node != NULL &&
               !cmp_cb(p_node, p_new))
        {
            p_prev = p_node;
            p_node = p_node->p_next;
            NRF_MESH_ASSERT(p_node != p_new);
        }

        /* This is the last element in the list. */
        if (p_node == NULL)
        {
            list_insert(p_prev, p_new);
            return NRF_SUCCESS;
        }
        else
        {
            return NRF_ERROR_FORBIDDEN;
        }
    }

}
