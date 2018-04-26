/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef EXAMPLE_COMMON_H__
#define EXAMPLE_COMMON_H__

#include "nrf_mesh_defines.h"
#include "device_state_manager.h"
#include "health_client.h"


#define  TOTAL_NODE_COUNT (SERVER_NODE_COUNT + CLIENT_NODE_COUNT)

/**
 * @defgroup LIGHT_SWT_V2 Light, Switch, and a Provisioner example common settings
 * @{
 */

#define APPKEY_INDEX (0)
#define NETKEY_INDEX (0)

#define GROUP_ADDRESS_EVEN (0xC002)
#define GROUP_ADDRESS_ODD  (0xC003)

#define PROVISIONER_ADDRESS  (0x0001)
#define UNPROV_START_ADDRESS (0x0100)

#define UNPROV_MAX_SCANNED_ITEMS_LIST   10

/* Element indices for ONOFF clients instantiated on the Switch node */
#define ELEMENT_IDX_ONOFF_CLIENT1       (1)
#define ELEMENT_IDX_ONOFF_CLIENT2       (2)
#define ELEMENT_IDX_ONOFF_CLIENT3       (3)
#define ELEMENT_IDX_ONOFF_CLIENT4       (4)

#define PROVISIONER_RETRY_COUNT  (2)

/** @} end of LIGHT_SWT_V2 */

#endif /* EXAMPLE_COMMON_H__ */
