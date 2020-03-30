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

#include "mesh_flash.h"
#include "nrf.h"

#include "bearer_event.h"
#include "fifo.h"
#include "nrf_flash.h"
#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "toolchain.h"
#include "timeslot_timer.h"
#include "msqueue.h"
#include "bearer_handler.h"
#include "hal.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/** Number of flash operations that can be queued at once. */
#define FLASH_OP_QUEUE_LEN					(8)

/** Maximum overhead of processing the flash queue. */
#define FLASH_PROCESS_TIME_OVERHEAD		    (500)

/* A single page erase operation must fit inside a bearer action */
NRF_MESH_STATIC_ASSERT(FLASH_TIME_TO_ERASE_PAGE_US + FLASH_PROCESS_TIME_OVERHEAD <= BEARER_ACTION_DURATION_MAX_US);

/* The queue length will overflow if it's longer than 256 */
NRF_MESH_STATIC_ASSERT(FLASH_OP_QUEUE_LEN < 256);
/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    FLASH_OP_STAGE_FREE,
    FLASH_OP_STAGE_QUEUED,
    FLASH_OP_STAGE_PROCESSED,
    FLASH_OP_STAGES
} flash_op_stage_t;

typedef struct
{
    uint16_t event_token;
    uint16_t push_token;
    uint32_t processed_bytes; /**< How many bytes have been processed in the current event. */
    mesh_flash_op_cb_t cb;
    struct
    {
        msq_t queue;
        uint8_t stages[FLASH_OP_STAGES];
        flash_operation_t elems[FLASH_OP_QUEUE_LEN];
    } flash_op_queue;
    bearer_action_t action;
    bool active;
} flash_user_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static bool                m_suspended; /**< Suspend flag, preventing flash operations while set. */

static flash_user_t        m_users[MESH_FLASH_USERS];
static bearer_event_flag_t m_event_flag;
/** Constant "All operations" operation, used to signalize that all operations
 * have been completed for a user. */
static const flash_operation_t m_all_operations =
{
    .type = FLASH_OP_TYPE_ALL
}; /*lint !e785 Too few initializers for flash_operation_t. */
/*****************************************************************************
* Static functions
*****************************************************************************/
static void flash_op_start(ts_timestamp_t start_time, void * p_args);

static void write_as_much_as_possible(const flash_operation_t* p_write_op, ts_timestamp_t available_time, uint32_t* p_bytes_written)
{
    uint32_t offset = *p_bytes_written;
    NRF_MESH_ASSERT(p_write_op->type == FLASH_OP_TYPE_WRITE);
    uint32_t bytes_to_write = WORD_SIZE * (available_time / FLASH_TIME_TO_WRITE_ONE_WORD_US);
    if (bytes_to_write > p_write_op->params.write.length - offset)
    {
        bytes_to_write = p_write_op->params.write.length - offset;
    }
    NRF_MESH_ASSERT(bytes_to_write > 0);

    NRF_MESH_ASSERT(nrf_flash_write(&p_write_op->params.write.p_start_addr[offset / WORD_SIZE],
                    &p_write_op->params.write.p_data[offset / WORD_SIZE],
                    bytes_to_write) == NRF_SUCCESS);

    *p_bytes_written += bytes_to_write;
}

static void erase_as_much_as_possible(const flash_operation_t * p_erase_op, ts_timestamp_t available_time, uint32_t* p_bytes_erased)
{
    uint32_t offset = *p_bytes_erased;
    NRF_MESH_ASSERT(p_erase_op->type == FLASH_OP_TYPE_ERASE);
    uint32_t bytes_to_erase = PAGE_SIZE * (available_time / FLASH_TIME_TO_ERASE_PAGE_US);
    if (bytes_to_erase > p_erase_op->params.erase.length - offset)
    {
        bytes_to_erase = p_erase_op->params.erase.length - offset;
    }
    NRF_MESH_ASSERT(bytes_to_erase > 0);

    NRF_MESH_ASSERT(nrf_flash_erase(&p_erase_op->params.erase.p_start_addr[offset / WORD_SIZE],
                    bytes_to_erase) == NRF_SUCCESS);

    *p_bytes_erased += bytes_to_erase;
}

/** Call the callbacks of all users for all processed events. */
static bool send_end_events(void)
{
    for (mesh_flash_user_t i = (mesh_flash_user_t) 0; i < MESH_FLASH_USERS; i++)
    {
        uint32_t notified_events = 0;
        flash_operation_t * p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        while (p_op != NULL)
        {
            if (m_users[i].cb != NULL)
            {
                m_users[i].cb(i, p_op, m_users[i].event_token);
            }
            m_users[i].event_token++;
            msq_move(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
            notified_events++;
            p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        }
        /* When every item in the queue has been processed and notified, we
         * tell the user. */
        if (notified_events != 0 &&
            (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) == FLASH_OP_QUEUE_LEN) &&
            m_users[i].cb != NULL)
        {
            m_users[i].cb(i, &m_all_operations, 0);
        }
    }
    return true;
}

static inline void end_event_schedule(void)
{
    bearer_event_flag_set(m_event_flag);
}

static bool execute_next_operation_chunk(flash_user_t * p_user, flash_operation_t * p_op, ts_timestamp_t available_time)
{
    uint32_t operation_length = 0;

    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            write_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
            operation_length = p_op->params.write.length;
            break;

        case FLASH_OP_TYPE_ERASE:
            erase_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
            operation_length = p_op->params.erase.length;
            break;

        default:
            NRF_MESH_ASSERT(false);
    }

    return (operation_length == p_user->processed_bytes);
}

static ts_timestamp_t flash_op_duration(flash_operation_t * p_op, uint32_t processed_bytes)
{
    ts_timestamp_t duration = FLASH_PROCESS_TIME_OVERHEAD;

    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            duration += FLASH_TIME_TO_WRITE_ONE_WORD_US * ((p_op->params.write.length - processed_bytes) / WORD_SIZE);
            break;

        case FLASH_OP_TYPE_ERASE:
            duration += FLASH_TIME_TO_ERASE_PAGE_US * ((p_op->params.erase.length - processed_bytes) / PAGE_SIZE);
            break;

        default:
            NRF_MESH_ASSERT(false);
    }

    return (duration < BEARER_ACTION_DURATION_MAX_US) ? duration : BEARER_ACTION_DURATION_MAX_US;
}

static ts_timestamp_t flash_op_type_min_duration(flash_operation_t * p_op)
{
    ts_timestamp_t min_duration = 0;

    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            min_duration = FLASH_TIME_TO_WRITE_ONE_WORD_US;
            break;

        case FLASH_OP_TYPE_ERASE:
            min_duration = FLASH_TIME_TO_ERASE_PAGE_US;
            break;

        default:
            NRF_MESH_ASSERT(false);
        }

        return min_duration;
}

static flash_operation_t* flash_op_action_prepare(flash_user_t * p_user)
{
    flash_operation_t * p_op = msq_get(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
    if (p_op != NULL)
    {
        p_user->action.start_cb = flash_op_start;
        p_user->action.radio_irq_handler = NULL;
        p_user->action.duration_us = flash_op_duration(p_op, p_user->processed_bytes);
        p_user->action.p_args = p_user;
        p_user->active = true;
    }

    return p_op;
}

static void flash_op_schedule(flash_user_t * p_user)
{
    flash_operation_t * p_op = flash_op_action_prepare(p_user);
    if (p_op != NULL)
    {
        NRF_MESH_ASSERT(NRF_SUCCESS == bearer_handler_action_enqueue(&p_user->action));
    }
}

static void flash_op_start(ts_timestamp_t start_time, void * p_args)
{
    flash_user_t * p_user = (flash_user_t *)p_args;
    ts_timestamp_t available_time = p_user->action.duration_us - FLASH_PROCESS_TIME_OVERHEAD;
    ts_timestamp_t elapsed_time = 0;

    /* Terminate bearer action immediately if suspended.
       Will be rescheduled when suspension is removed. */
    if (m_suspended)
    {
        bearer_handler_action_end();
        p_user->active = false;
        return;
    }

    flash_operation_t * p_op = msq_get(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
    NRF_MESH_ASSERT(p_op != NULL);

    /* Normally a flash operation takes just a fraction of the theoretical max time.
       For flash operations that are (theoretically) bigger than the maximum duration of a single
       bearer action we therefore try to execute several chunks of this operation. */
    for (;;)
    {
        bool operation_done = execute_next_operation_chunk(p_user, p_op, available_time - elapsed_time);
        if (operation_done)
        {
            bearer_handler_action_end();
            msq_move(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
            p_user->processed_bytes = 0;
            p_user->active = false;
            end_event_schedule();
            flash_op_schedule(p_user);
            break;
        }

        elapsed_time = TIMER_DIFF(ts_timer_now(), start_time);
        if (available_time < elapsed_time + flash_op_type_min_duration(p_op))
        {
            /* Not enough time to complete operation */
            bearer_handler_action_end();
            flash_op_schedule(p_user);
            break;
        }
    }
}

static void init_flash_op_queue(flash_user_t * p_user)
{
    p_user->flash_op_queue.queue.elem_count = FLASH_OP_QUEUE_LEN;
    p_user->flash_op_queue.queue.elem_size = sizeof(flash_operation_t);
    p_user->flash_op_queue.queue.p_elem_array = p_user->flash_op_queue.elems;
    p_user->flash_op_queue.queue.stage_count = FLASH_OP_STAGES;
    p_user->flash_op_queue.queue.p_stages = p_user->flash_op_queue.stages;
    msq_init(&p_user->flash_op_queue.queue);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void mesh_flash_init(void)
{
    m_event_flag = bearer_event_flag_add(send_end_events);
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
}

void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb)
{
    NRF_MESH_ASSERT(user < MESH_FLASH_USERS);
    m_users[user].cb = cb;
}

uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token)
{
    NRF_MESH_ASSERT(user < MESH_FLASH_USERS);
    NRF_MESH_ASSERT(p_op != NULL);

    if (p_op->type == FLASH_OP_TYPE_WRITE)
    {
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_start_addr));
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_data));
        NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.length));
        NRF_MESH_ASSERT(p_op->params.write.length != 0);
    }
    else if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.p_start_addr));
        NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.length));
        NRF_MESH_ASSERT(p_op->params.erase.length != 0);
    }
    else
    {
        /* operation type must be WRITE or ERASE */
        NRF_MESH_ASSERT(false);
    }

    flash_operation_t * p_op_to_enqueue = NULL;

    uint32_t was_masked;
    uint32_t status;
    _DISABLE_IRQS(was_masked);
    flash_operation_t * p_free_op = msq_get(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
    if (p_free_op == NULL)
    {
        status = NRF_ERROR_NO_MEM;
    }
    else
    {
        msq_move(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        memcpy(p_free_op, p_op, sizeof(flash_operation_t));
        status = NRF_SUCCESS;
        if (p_token != NULL)
        {
            *p_token = m_users[user].push_token;
        }
        m_users[user].push_token++;

        if (!m_users[user].active && !m_suspended)
        {
            p_op_to_enqueue = flash_op_action_prepare(&m_users[user]);
        }
    }
    _ENABLE_IRQS(was_masked);

    /* bearer_handler_action_enqueue() call is moved outside of the critical
     * section because of calling SVC inside it */
    if (p_op_to_enqueue != NULL)
    {
        NRF_MESH_ASSERT(NRF_SUCCESS == bearer_handler_action_enqueue(&m_users[user].action));
    }

    return status;
}

uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user)
{
    if (user < MESH_FLASH_USERS)
    {
        uint32_t was_masked;
        _DISABLE_IRQS(was_masked);
        uint32_t available_slots = msq_available(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        _ENABLE_IRQS(was_masked);

        return available_slots;
    }
    else
    {
        return 0;
    }
}

bool mesh_flash_in_progress(void)
{
    uint32_t was_masked;
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        _DISABLE_IRQS(was_masked);
        bool operations_in_progress = (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) != FLASH_OP_QUEUE_LEN);
        _ENABLE_IRQS(was_masked);

        if (operations_in_progress)
        {
            return true;
        }
    }
    return false;
}

void mesh_flash_set_suspended(bool suspend)
{
    static uint32_t suspend_count = 0;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    NRF_MESH_ASSERT(suspend || suspend_count > 0);
    if (suspend)
    {
        suspend_count++;
    }
    else
    {
        suspend_count--;
        if (suspend_count == 0)
        {
            for (uint32_t user = 0; user < MESH_FLASH_USERS; user++)
            {
                if (!m_users[user].active)
                {
                    flash_op_schedule(&m_users[user]);
                }
            }
        }
    }
    m_suspended = (suspend_count > 0);
    _ENABLE_IRQS(was_masked);
}

#ifdef UNIT_TEST
/**
 * @internal
 * Test-utility function to reset the state of the module between tests. Not
 * exposed in the header, as it should never be called when running on target.
 */
void mesh_flash_reset(void)
{
    m_event_flag = 0;
    m_suspended = false;
    memset(m_users, 0, sizeof(m_users));
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
}
#endif
