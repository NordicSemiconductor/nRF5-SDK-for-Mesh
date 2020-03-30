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

#ifndef NRF_MESH_CONFIG_APP_H__
#define NRF_MESH_CONFIG_APP_H__

/**
 * Unit test mock for nrf_mesh_config_app.h
 *
 * @warning THIS IS NOT INTENDED TO BE USED FOR AN ACTUAL APPLICATION. THAT WOULD BE JUST CRAZY!
 */

#ifndef DEVICE_COMPANY_ID
#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NONE)
#endif
#ifndef DEVICE_PRODUCT_ID
#define DEVICE_PRODUCT_ID (0x0000)
#endif
#ifndef DEVICE_VERSION_ID
#define DEVICE_VERSION_ID (0x0000)
#endif

#ifndef ACCESS_DEFAULT_TTL
#define ACCESS_DEFAULT_TTL 2
#endif
#ifndef ACCESS_COMPANY_ID
#define ACCESS_COMPANY_ID 0
#endif
#ifndef ACCESS_PRODUCT_ID
#define ACCESS_PRODUCT_ID 0
#endif
#ifndef ACCESS_VENDOR_ID
#define ACCESS_VENDOR_ID 0
#endif
#ifndef ACCESS_FEATURES
#define ACCESS_FEATURES 0
#endif
#ifndef ACCESS_MODEL_COUNT
#define ACCESS_MODEL_COUNT 1
#endif
#ifndef ACCESS_ELEMENT_COUNT
#define ACCESS_ELEMENT_COUNT 1
#endif
#ifndef ACCESS_SUBSCRIPTION_LIST_COUNT
#define ACCESS_SUBSCRIPTION_LIST_COUNT 1
#endif
#ifndef HEALTH_SERVER_ELEMENT_COUNT
#define HEALTH_SERVER_ELEMENT_COUNT 1
#endif
#ifndef DSM_SUBNET_MAX
#define DSM_SUBNET_MAX 8
#endif
#ifndef DSM_APP_MAX
#define DSM_APP_MAX 8
#endif
#ifndef DSM_DEVICE_MAX
#define DSM_DEVICE_MAX 4
#endif
#ifndef DSM_VIRTUAL_ADDR_MAX
#define DSM_VIRTUAL_ADDR_MAX 4
#endif
#ifndef DSM_NONVIRTUAL_ADDR_MAX
#define DSM_NONVIRTUAL_ADDR_MAX 8
#endif

#ifndef ACCESS_RELIABLE_TRANSFER_COUNT
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)
#endif

/** @} */
#endif /* NRF_MESH_CONFIG_APP_H__ */

