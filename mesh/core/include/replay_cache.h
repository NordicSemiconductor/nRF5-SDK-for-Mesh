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
#ifndef REPLAY_CACHE_H__
#define REPLAY_CACHE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup REPLAY_CACHE Replay protection cache
 * @ingroup MESH_CORE
 * Stores information so that an already processed message originating from one
 * source would not be processed more than once.
 * @{
 */

/**
 * Initialize replay protection cache.
 */
void replay_cache_init(void);

/**
 * Enable the replay protection cache module.
 */
void replay_cache_enable(void);

/**
 * Add an element to the replay protection cache.
 *
 * @param[in] src Source address of the element.
 * @param[in] seqno Message sequence number.
 * @param[in] iv_index IV index of the packet.
 *
 * @retval NRF_SUCCESS      Successfully added element.
 * @retval NRF_ERROR_NO_MEM No more memory available in the cache.
 */
uint32_t replay_cache_add(uint16_t src, uint32_t seqno, uint32_t iv_index);

/**
 * Add a sequence authentication (SeqAuth) value of the element to
 * the replay protection cache.
 *
 * If the entry for the element does not exist in the cache, the entry will be created.
 *
 * This function does not store the entire SeqAuth. Instead, it stores the IV Index,
 * sequence number, and SeqZero.
 *
 * @param[in] src Source address of the element.
 * @param[in] seqno Message sequence number.
 * @param[in] iv_index IV index of the packet.
 * @param[in] SeqZero SeqZero of the segmented message.
 *
 * @retval NRF_SUCCESS      Successfully added SeqAuth.
 * @retval NRF_ERROR_NO_MEM No more memory available in the cache.
 */
uint32_t replay_cache_seqauth_add(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);

/**
 * Check if an element is in the cache.
 *
 * @note If the IV index bit is different than the one present in the cache, it
 * will be assumed that the index and sequence number has been reset. This is
 * valid since the packet has already passed network decryption, that is since the IV
 * index has become valid.
 *
 * @param[in] src Source address of the element.
 * @param[in] seqno Message sequence number.
 * @param[in] iv_index IV index.
 *
 * @retval true  If the message exists in the cache.
 * @retval false Otherwise.
 */
bool replay_cache_has_elem(uint16_t src, uint32_t seqno, uint32_t iv_index);

/**
 * Check if a sequence authentication (SeqAuth) value of the element is in
 * the replay protection cache.
 *
 * This function restores the SeqAuth from IV Index, sequence number, and
 * SeqZero stored in the cache.
 *
 * @note If the IV index bit is different than the one present in the cache, it
 * will be assumed that the index and sequence number has been reset. This is
 * valid since the packet has already passed network decryption, that is since the IV
 * index has become valid.
 *
 * @param[in] src Source address of the element.
 * @param[in] seqno Message sequence number.
 * @param[in] iv_index IV index.
 * @param[in] seqzero SeqZero of the segmented message.
 *
 * @retval true  If the SeqAuth exists in the cache.
 * @retval false Otherwise.
 */
bool replay_cache_has_seqauth(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);

/**
 * Check if a sequence authentication (SeqAuth) value of the element is the last
 * one that is stored in the replay protection cache.
 *
 * This function restores the SeqAuth from IV Index, sequence number, and
 * SeqZero stored in the cache.
 *
 * @note If the IV index bit is different than the one present in the cache, it
 * will be assumed that the index and sequence number has been reset. This is
 * valid since the packet has already passed network decryption, that is since the IV
 * index has become valid.
 *
 * @param[in] src Source address of the element.
 * @param[in] seqno Message sequence number.
 * @param[in] iv_index IV index.
 * @param[in] seqzero SeqZero of the segmented message.
 *
 * @retval true  If the SeqAuth is the last one stored in the cache.
 * @retval false Otherwise.
 */
bool replay_cache_is_seqauth_last(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);

/**
 * Function to call in IV update.
 */
void replay_cache_on_iv_update(void);

/**
 * Clear the replay protection cache.
 */
void replay_cache_clear(void);

/** @} */
#endif  /* REPLAY_CACHE_H__ */
