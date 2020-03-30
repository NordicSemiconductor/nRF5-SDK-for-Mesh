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
#include <string.h>
#include <stdbool.h>

#include "nrf_mesh_assert.h"
#include "packet_mgr.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "utils.h"
#include "log.h"
#include "debug_pins.h"

#if PACKET_MGR_DEBUG_MODE
#define __LOG_PACMAN(...) __LOG(LOG_SRC_PACMAN, LOG_LEVEL_INFO, __VA_ARGS__)
/* Seal is used as a buffer between memory blocks for overflow detection. */
#define PACKET_MGR_MEM_SEAL 0x33333333
#else
#define __LOG_PACMAN(...)
#endif

/**
 * Structure of the header for each allocated packet buffer.
 * Total size of this structure is 8 bytes (on target, it may be whatever on your PC).
 */
typedef struct
{
#if PACKET_MGR_DEBUG_MODE
    uint32_t seal;
#endif
    uint16_t size;      /**< Size of the block in bytes */
    uint16_t ref_count; /**< Reference count */
    void *   p_next_free; /** < Next available block in memory */
#if PACKET_MGR_BLAME_MODE
    uint32_t last_decreffer;       /**< Address of last caller that modified
                                 * refcount on the packet. */
    uint32_t last_allocer;
#endif
} buffer_header_t;

/** Make sure that the max possible packet size (PACKET_MGR_MEMORY_POOL_SIZE - sizeof(buffer_header_t)) is not
 * larger than the maximum value representable by the buffer_header_t */
NRF_MESH_STATIC_ASSERT(PACKET_MGR_MEMORY_POOL_SIZE <= (UINT16_MAX + sizeof(buffer_header_t)));

/* PACKET_MGR_MEMORY_POOL_SIZE must be aligned to the PACKET_MGR_ALIGNMENT */
NRF_MESH_STATIC_ASSERT(PACKET_MGR_MEMORY_POOL_SIZE == ALIGN_VAL(PACKET_MGR_MEMORY_POOL_SIZE, PACKET_MGR_ALIGNMENT));

/********************
 * Static variables *
 ********************/

static uint8_t m_pool[PACKET_MGR_MEMORY_POOL_SIZE] __attribute((aligned(PACKET_MGR_ALIGNMENT)));
static void * mp_memory_block = m_pool;
static buffer_header_t * mp_free_head; /** < Head of the free list of blocks */

/********************
 * Static functions *
 ********************/

/**
 * Gets the buffer header from a packet buffer.
 * @param p_buffer Packet buffer.
 * @return Returns a pointer to the buffer header of the packet buffer.
 */
static inline buffer_header_t * buffer_header_get(packet_generic_t * p_buffer)
{
    return (buffer_header_t *) (((uint8_t *) p_buffer) - sizeof(buffer_header_t));
}

/**
 * Gets the start of the memory region of a buffer.
 * @param p_header Pointer to the buffer header.
 * @return Returns a pointer to the start of the memory for a buffer.
 */
static inline packet_generic_t * buffer_get_mem(buffer_header_t * p_header)
{
    return (packet_generic_t *) (((uint8_t *) p_header) + sizeof(buffer_header_t));
}

/**
 * Gets the next buffer header relative to the current header.
 * @param p_current Current buffer header.
 * @return Returns a pointer to the header of the next packet buffer.
 */
static inline buffer_header_t * buffer_header_get_next(buffer_header_t * p_current)
{
    return (buffer_header_t *) (((uint8_t *) p_current) + sizeof(buffer_header_t) + p_current->size);
}


#if PACKET_MGR_DEBUG_MODE
/**
 * Gets the amount of free memory available, without using the free list.
 * @note This is used for debugging
 * @return Returns the amount of unused memory in the packet manager.
 */
static uint32_t buffer_get_available_space()
{
    buffer_header_t * p_iter = mp_memory_block;
    uint32_t size = 0;
    do {
        if (p_iter->ref_count == 0)
        {
            size += p_iter->size + sizeof(buffer_header_t);
        }
        NRF_MESH_ASSERT(p_iter->seal == PACKET_MGR_MEM_SEAL);
        p_iter = buffer_header_get_next(p_iter);
    } while ((uint8_t *) p_iter < ((uint8_t *) mp_memory_block) + PACKET_MGR_MEMORY_POOL_SIZE);

   size -= sizeof(buffer_header_t);

   return size;
}
#endif

/**
 * Checks if a buffer header is the last buffer header.
 * @param p_header Pointer to the buffer header.
 * @return @c true if the buffer header represents the last buffer.
 */
static inline bool buffer_is_last(buffer_header_t * p_header)
{
    return (uint8_t *) buffer_get_mem(p_header) + p_header->size >= (uint8_t *) mp_memory_block + PACKET_MGR_MEMORY_POOL_SIZE;
}

/**
 * Check that the pointer is within the buffer regions.
 *
 * @param p_buffer Pointer to a memory location.
 * @return @c true if the memory location is within the packet pool.
 */
static inline bool buffer_pointer_is_valid(const packet_generic_t * p_buffer)
{
    return (((uint8_t *) p_buffer >=  (uint8_t *) mp_memory_block) ||
            ((uint8_t *) p_buffer <  ((uint8_t *) mp_memory_block + PACKET_MGR_MEMORY_POOL_SIZE)));

}


static buffer_header_t * buffer_find_preceding_free_block(const buffer_header_t * buf)
{
    buffer_header_t * p_cur = mp_free_head;
    /* The function is called with mp_free_head */
    if (p_cur == buf)
    {
        return NULL;
    }

    /* Find the buffer in the free list */
    while (p_cur != NULL && p_cur->p_next_free != buf)
    {
#if PACKET_MGR_DEBUG_MODE
        NRF_MESH_ASSERT(p_cur->seal == PACKET_MGR_MEM_SEAL);
#endif
        p_cur = p_cur->p_next_free;
    }

    return p_cur;
}

/**
 * Remove a given buffer from the free list.
 *
 * @note The removed buffer will no longer be in the free list and it is the caller's responsibility to
 *       make use of the buffer.
 *
 * @note This function expects that the provided buffer exists in the free list.
 */
static void buffer_remove_from_free_list(buffer_header_t * buf)
{
    buffer_header_t * p_cur = mp_free_head;
    /* If the buffer to be released is a head, then move the head and return */
    if (p_cur == buf)
    {
        mp_free_head = p_cur->p_next_free;
        return;
    }

    /* Find the buffer in the free list */
    p_cur = buffer_find_preceding_free_block(buf);
    NRF_MESH_ASSERT(p_cur != NULL);

    /* Remove the buffer from the free list */
    p_cur->p_next_free = buf->p_next_free;
}

#if PACKET_MGR_DEBUG_MODE
/**
 * Gets the offset of a buffer relative to the start of the memory pool.
 * @param p_header header of the buffer.
 * @return offset of the buffer relative to the start of the memory pool.
 */
static inline uint32_t buffer_offset_get(buffer_header_t * p_header)
{
    return (uint32_t) p_header;
}
#endif /* PACKET_MGR_DEBUG_MODE */

/**
 * Shrinks the memory allocated for a packet.
 *
 * This can be used to free up memory in the packet pool when the packet stored
 * a packet buffer is shorter than the allocated packet buffer. Note that the
 * new size must be smaller than the buffer's current size.
 *
 * @param[in,out] header Pointer to the buffer to shrink.
 * @param[in]     new_size New size of the buffer.
 *
 * @retval NRF_SUCCESS             The buffer was successfully shrinked.
 * @retval NRF_ERROR_INVALID_PARAM The new size is not sufficiently small enough to split the buffer
 *                                 into two new buffers.
 */
static uint32_t buffer_shrink(buffer_header_t * header, uint16_t new_size)
{
    buffer_header_t * next_header = NULL;
    uint32_t old_size = header->size;

    uint32_t was_masked;
    uint32_t retval = NRF_SUCCESS;

    /* Ensure the new size is a multiple of PACKET_MGR_ALIGNMENT: */
    new_size = ALIGN_VAL(new_size, PACKET_MGR_ALIGNMENT);

    if (old_size == new_size)
    {
        retval = NRF_SUCCESS;
    }
    else if (new_size > old_size)
    {
        retval = NRF_ERROR_INVALID_PARAM;
    }
    else if (old_size - new_size <= sizeof(buffer_header_t))
    {
        retval = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        _DISABLE_IRQS(was_masked);
        /* Resize this block and create a new one at the end: */
        header->size = new_size;
        next_header = buffer_header_get_next(header);
        next_header->size = old_size - new_size - sizeof(buffer_header_t);
        next_header->ref_count = 0;
    #if PACKET_MGR_DEBUG_MODE
        next_header->seal = PACKET_MGR_MEM_SEAL;
    #endif
        if (header->ref_count == 0)
        {
            /* Shrinking a block in the free list */
            next_header->p_next_free = header->p_next_free;
            header->p_next_free = next_header;
        }
        else
        {
            /* Shrinking an already allocated block */
            next_header->p_next_free = mp_free_head;
            mp_free_head = next_header;
        }
        _ENABLE_IRQS(was_masked);

        __LOG_PACMAN("Resized block at offset %d from %d to %d and made a new one of size %d\n",
                     buffer_offset_get(header), old_size, new_size, next_header->size);
    }
    return retval;
}

/*
* Populate a given header with default values, used for newly created memory blocks.
*/
static void buffer_populate_new_header(buffer_header_t * p_header, uint16_t block_size, buffer_header_t * p_next_free)
{
    p_header->size = block_size;
    p_header->ref_count = 0;
#if PACKET_MGR_DEBUG_MODE
    p_header->seal = PACKET_MGR_MEM_SEAL;
#endif
    p_header->p_next_free = p_next_free;
}

/*
* Coalesce a given block (current) with it's neighbouring block (of ascending memory location) if possible,
* and return the preceding free block (i.e. prev->p_next_free == current).
*
* @param[in,out] current The block to merge with it's nearest neighbour.
* @param[out] prev The free block that precedes the enlarged block.
* @returns True if the block was combined with it's neighbour.
*/
static inline bool buffer_merge_with_next_block(buffer_header_t * p_current, buffer_header_t ** pp_prev)
{
    buffer_header_t * next = buffer_header_get_next(p_current);
    if (!buffer_is_last(p_current) && next->ref_count == 0)
    {
    #if PACKET_MGR_DEBUG_MODE
        NRF_MESH_ASSERT(p_current->seal == PACKET_MGR_MEM_SEAL);
        NRF_MESH_ASSERT(next->seal == PACKET_MGR_MEM_SEAL);
    #endif
        buffer_remove_from_free_list(next);
        /* If what we removed from list was the head, p_current could be the new head */
        if (p_current == mp_free_head)
        {
            *pp_prev = NULL;
        }
        else if (*pp_prev == next)
        {
            /* Find the new prev! */
            *pp_prev = buffer_find_preceding_free_block(p_current);
            NRF_MESH_ASSERT(NULL != *pp_prev);
        }
        p_current->size += sizeof(buffer_header_t) + next->size;
        return true;
    }
    else
    {
        return false;
    }
}

/******************************
 * Public interface functions *
 ******************************/

void packet_mgr_init(const nrf_mesh_init_params_t * p_init_params)
{
    uint32_t unpartitioned_pool_size = PACKET_MGR_MEMORY_POOL_SIZE;
    memset(mp_memory_block, 0, PACKET_MGR_MEMORY_POOL_SIZE);
    mp_free_head = (buffer_header_t *) mp_memory_block;

    /* Break the pool into @ref PACKET_MGR_DEFAULT_PACKET_LEN chunks and create headers */
#if PACKET_MGR_DEBUG_MODE
    uint32_t buf_no=0;
#endif
    buffer_header_t * p_header = (buffer_header_t *) mp_memory_block ;
    while (unpartitioned_pool_size > 2*PACKET_MGR_DEFAULT_PACKET_LEN)
    {
        buffer_populate_new_header(p_header, PACKET_MGR_DEFAULT_PACKET_LEN, NULL);
        p_header->p_next_free = buffer_header_get_next(p_header);
        __LOG_PACMAN("Created header at offset %d with size %d\n", 0, p_header->size);
        unpartitioned_pool_size -= sizeof(buffer_header_t) + PACKET_MGR_DEFAULT_PACKET_LEN;
        p_header = buffer_header_get_next(p_header);
#if PACKET_MGR_DEBUG_MODE
        buf_no++;
#endif
    }

    /* Allocate the last block */
    uint32_t last_block_size = unpartitioned_pool_size - sizeof(buffer_header_t);
    NRF_MESH_ASSERT(last_block_size > 0);
    /* Set the last block's next_free pointer to NULL */
    buffer_populate_new_header(p_header, last_block_size, NULL);
#if PACKET_MGR_DEBUG_MODE
    buf_no++;
#endif

    __LOG_PACMAN("Created header at offset %d with size %d\n", 0, p_header->size);
    __LOG_PACMAN("Packet manager initialized, free space: %d\n", PACKET_MGR_MEMORY_POOL_SIZE-sizeof(buffer_header_t));
    __LOG_PACMAN("\tdefault buffer size: %d\n", PACKET_MGR_DEFAULT_PACKET_LEN);
    __LOG_PACMAN("\ttotal number of buffers: %d\n", buf_no);
    __LOG_PACMAN("\tmaximum buffer size: %d\n", PACKET_MGR_PACKET_MAXLEN);
}

uint32_t packet_mgr_alloc(packet_generic_t ** pp_buffer, uint16_t size)
{
    /* Ensure the size is a multiple of PACKET_MGR_ALIGNMENT: */
    size = ALIGN_VAL(size, PACKET_MGR_ALIGNMENT);
    if (size > PACKET_MGR_PACKET_MAXLEN || size == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }


    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    if (mp_free_head == NULL)
    {
        _ENABLE_IRQS(was_masked);
        return NRF_ERROR_NO_MEM;
    }


    buffer_header_t * p_current = mp_free_head;
    buffer_header_t * p_prev = NULL;
    /* Look for a suitable free block: */
    do
    {
        if (p_current->size >= size)
        {
            /* Break if this block is big enough */
            break;
        }
        /* Join the two buffers if possible*/
        if (!buffer_merge_with_next_block(p_current, &p_prev))
        {
            p_prev = p_current;
            p_current = p_current->p_next_free;
        }
    } while (p_current != NULL);

    /* If no free block was found, we are out of memory: */
    if (p_current == NULL)
    {
        _ENABLE_IRQS(was_masked);
        return NRF_ERROR_NO_MEM;
    }

    /* If the block is of unusual size and bigger than necessary, shrink it: */
    uint32_t no_blocks_in_current = p_current->size/PACKET_MGR_DEFAULT_PACKET_LEN;
    uint32_t no_blocks_required = (size-1)/PACKET_MGR_DEFAULT_PACKET_LEN + 1;

    if ( no_blocks_in_current > no_blocks_required )
    {
        /* Shrink will handle the free pointers in the two buffers created. */
        uint16_t shrink_size = no_blocks_required * PACKET_MGR_DEFAULT_PACKET_LEN;
        uint32_t status = buffer_shrink(p_current, shrink_size);
        if (status != NRF_SUCCESS)
        {
            _ENABLE_IRQS(was_masked);
            return status;
        }
    }

    p_current->ref_count = 1;
    if (p_prev != NULL)
    {
        p_prev->p_next_free = p_current->p_next_free;
    }
    else
    {
        mp_free_head = p_current->p_next_free;
    }

    _ENABLE_IRQS(was_masked);
    NRF_MESH_ASSERT(p_current != mp_free_head);

    *pp_buffer = buffer_get_mem(p_current);
    __LOG_PACMAN("Allocated block of size %d (actual %d) at offset %d\n",
        size, p_current->size, (uint8_t *) p_current - (uint8_t *) mp_memory_block);

#if PACKET_MGR_DEBUG_MODE
    NRF_MESH_ASSERT(p_current->seal == PACKET_MGR_MEM_SEAL);
#endif

#if PACKET_MGR_BLAME_MODE
    _GET_LR(p_current->last_allocer);
#endif

    return NRF_SUCCESS;
}

void packet_mgr_free(packet_generic_t * p_buffer)
{
    if (!buffer_pointer_is_valid(p_buffer))
    {
        NRF_MESH_ASSERT(false);
    }

    buffer_header_t * p_header = buffer_header_get(p_buffer);
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    /* Check if the padding bits have been messed with */
    NRF_MESH_ASSERT(p_header->ref_count == 1);
    p_header->ref_count = 0;

    memset(p_buffer, 0, p_header->size);
    /* We need to slot the released memory in to the free list*/
    p_header->p_next_free = mp_free_head;
    mp_free_head = p_header;

#if PACKET_MGR_DEBUG_MODE
    NRF_MESH_ASSERT(p_header->seal == PACKET_MGR_MEM_SEAL);
#endif

#if PACKET_MGR_BLAME_MODE
    _GET_LR(p_header->last_decreffer);
#endif
    _ENABLE_IRQS(was_masked);
}

uint32_t packet_mgr_get_free_space(void)
{
    /* Calculate the free space. */
    uint32_t available_memory = 0;
    buffer_header_t * p_current = (buffer_header_t *) mp_free_head;
    do
    {
        available_memory += sizeof(buffer_header_t) + p_current->size;
#if PACKET_MGR_DEBUG_MODE
    NRF_MESH_ASSERT(p_current->seal == PACKET_MGR_MEM_SEAL);
#endif
        p_current = p_current->p_next_free;

    } while (p_current != NULL);

    /* Even with best case scenario we need to reserve for one header*/
    available_memory -= sizeof(buffer_header_t);
#if PACKET_MGR_DEBUG_MODE
    uint32_t unused_memory = buffer_get_available_space();
    NRF_MESH_ASSERT(available_memory == unused_memory);
#endif
    return available_memory;
}

uint8_t packet_mgr_refcount_get(packet_generic_t * p_packet)
{
    buffer_header_t * p_header = buffer_header_get(p_packet);
    return p_header->ref_count;
}

uint16_t packet_mgr_size_get(packet_generic_t * p_packet)
{
    buffer_header_t * p_header = buffer_header_get(p_packet);
    return p_header->size;
}
