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

#ifndef FLASH_MANAGER_DEFRAG_H__
#define FLASH_MANAGER_DEFRAG_H__

#include "flash_manager.h"

/**
 * @internal
 * @defgroup   FLASH_MANAGER_DEFRAG Flash Manager Defrag Submodule
 * Defrag procedure handler for the flash manager module.
 * @warning    The @ref flash_manager_defrag_init and @ref flash_manager_defrag should not interrupt
 *             each other since these are non-reentrant functions and they share a common state.
 * @{
 */

/**
 * Initialize flash manager defrag handler.
 *
 * @return     Whether recovery was started.
 */
bool flash_manager_defrag_init(void);

/**
 * Check whether the given manager is being defragged.
 *
 * @param[in]  p_manager  Manager to check for.
 *
 * @return     Whether the given manager is being defragged.
 */
bool flash_manager_defragging(const flash_manager_t * p_manager);

/**
 * Check whether a defrag procedure is currently running.
 *
 * @returns Whether a defrag procedure is currently running.
 */
bool flash_manager_defrag_is_running(void);

/**
 * Get a pointer to the page currently being defragged.
 *
 * @return     A pointer to the page being defragged, or NULL if no defrag procedure is currently in
 *             progress.
 */
const flash_manager_page_t * flash_manager_defrag_page_get(void);

/**
 * Defrag the given flash manager.
 *
 * @warning    The p_manager must be complete with valid entries and in state @ref FM_STATE_READY
 *
 * @param[in]  p_manager  The flash manager instance to defrag.
 */
void flash_manager_defrag(const flash_manager_t * p_manager);

/**
 * Get a pointer to the flash page being used as a recovery area.
 *
 * @return     Pointer to the start of the recovery area. Always page aligned.
 */
const void * flash_manager_defrag_recovery_page_get(void);

/**
 * Emergency freezing of the defragmentation process.
 *
 * @note  The functionality is used to stop ongoing defragmentation activity.
 *        It helps to free flash availability for the emergency cache within power down.
 */
void flash_manager_defrag_freeze(void);

/** @} */

#endif /* FLASH_MANAGER_DEFRAG_H__ */

