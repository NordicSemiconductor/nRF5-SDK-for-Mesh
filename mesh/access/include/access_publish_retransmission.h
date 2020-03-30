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

#ifndef ACCESS_PUBLISH_RETRANSMISSION_H__
#define ACCESS_PUBLISH_RETRANSMISSION_H__

#include "access.h"

/**
 * @internal
 * @defgroup ACCESS_PUBLISH_RETRANSMISSION Access layer publication re-transmission
 * internal interface
 * @ingroup ACCESS
 * Provides API for the publication re-transmission.
 * @{
 */

/**
 * Initializes the publish retransmission module.
 */
void access_publish_retransmission_init(void);

/**
 * Adds the published message for retransmission.
 * *
 * If it is not enough memory to publish the message during the re-transmission,
 * the re-transmission attempt will be skipped.
 *
 * @param[in] model_handle              Access handle of the model that sent data.
 * @param[in] p_publication_retransmit  Retransmit parameters of the model.
 * @param[in] p_tx_message              Parameter structure of the access layer TX message.
 * @param[in] p_access_payload          Access payload to be re-transmitted
                                        containing the access message and the opcode.
 * @param[in] access_payload_len        Access payload length.
 *
 */
void access_publish_retransmission_message_add(access_model_handle_t model_handle,
                                               const access_publish_retransmit_t *p_publication_retransmit,
                                               const access_message_tx_t *p_tx_message,
                                               const uint8_t *p_access_payload,
                                               uint16_t access_payload_len);

/** @} */

#endif /* ACCESS_PUBLISH_RETRANSMISSION_H__ */
