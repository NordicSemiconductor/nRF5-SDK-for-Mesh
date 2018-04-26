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

#include "enocean.h"
#include <string.h>

#include "list.h"
#include "utils.h"
#include "ble.h"
#include "aes.h"
#include "ccm_soft.h"
#include "ble_gap.h"
#include "log.h"
#include "app_error.h"

#define PTM_CRYPTO_DATA_SIZE       (PTM215B_COMM_PACKET_KEY_SIZE)
#define PTM_NONCE_SIZE             (13)
#define PTM_NONCE_SRC_ADDR_OFFSET  (0)
#define PTM_NONCE_SEQ_OFFSET       (6)
#define PTM_NONCE_PADDING_OFFSET   (10)
#define CRYPTO_B1_DATA_OFFSET      (2)

#define A0_FLAG     (0x01)
#define B0_FLAG     (0x49)
#define L_LEN       (CCM_LENGTH_FIELD_LENGTH)


typedef enum
{
    ENOCEAN_COMMISSIONING_PACKET,
    ENOCEAN_DATA_PACKET
} enocean_packet_type_t;

typedef struct
{
    uint32_t seq;
    const uint8_t *p_key;
} enocean_packet_commissioning_t;

typedef struct
{
    uint32_t seq;
    const uint8_t *p_raw_data;
    uint8_t  length;
    const uint8_t *p_mic;
} enocean_packet_data_t;

typedef struct
{
    enocean_packet_type_t type;
    const uint8_t *p_ble_gap_addr;
    union
    {
        enocean_packet_data_t  data_packet;
        enocean_packet_commissioning_t comm_packet;
    } data;
} enocean_packet_t;

/* Packet parsing structures */
typedef struct __attribute((packed))
{
    uint8_t ad_len;
    uint8_t ad_type;
    uint16_t manufacturer_id;
    uint32_t seq;
    uint8_t status;
    union __attribute((packed))
    {
        struct __attribute((packed))
        {
            uint8_t mic[PTM215B_DATA_PACKET_MIC_SIZE];
        } no_opt_mic;
        struct __attribute((packed))
        {
            uint8_t opt[1];
            uint8_t mic[PTM215B_DATA_PACKET_MIC_SIZE];
        } opt1_mic;
        struct __attribute((packed))
        {
            uint8_t opt[2];
            uint8_t mic[PTM215B_DATA_PACKET_MIC_SIZE];
        } opt2_mic;
        struct __attribute((packed))
        {
            uint8_t opt[4];
            uint8_t mic[PTM215B_DATA_PACKET_MIC_SIZE];
        } opt4_mic;
    } data_and_mic;
} enocean_data_packet_t;

typedef struct __attribute((packed))
{
    uint8_t ad0_len;
    uint8_t ad0_type;
    uint8_t short_name[PTM215B_PRODUCT_SHORT_NAME_SIZE];

    uint8_t ad1_len;
    uint8_t ad1_type;
    uint16_t manufacturer_id;
    uint32_t seq;

    uint8_t key[PTM_CRYPTO_DATA_SIZE];
} enocean_commissioning_data_old_t;

typedef struct __attribute((packed))
{
    uint8_t ad_len;
    uint8_t ad_type;
    uint16_t manufacturer_id;
    uint32_t seq;

    uint8_t key[PTM_CRYPTO_DATA_SIZE];
    uint8_t ble_addr[BLE_GAP_ADDR_LEN];
} enocean_commissioning_data_new_t;

typedef union __attribute((packed))
{
    enocean_commissioning_data_old_t old_format;
    enocean_commissioning_data_new_t new_format;
} enocean_commissioning_packet_t;

static list_node_t * mp_secmats_list_head;
static enocean_translator_cb_t m_app_cb;


/* Checks that the received sequence number is greater than last seen sequence number */
static bool data_replay_check(const enocean_packet_t * p_data_pkt,
                                      enocean_commissioning_secmat_t * p_secmat)
{
    if (p_data_pkt->data.data_packet.seq > *p_secmat->p_seq)
    {
       *p_secmat->p_seq = p_data_pkt->data.data_packet.seq;
        return true;
    }
    return false;
}

/* Authenticates the given data packet. Returns true if authentication is successful */
static bool data_authenticate(const enocean_packet_t * p_data_pkt,
                                      enocean_commissioning_secmat_t * p_secmat)
{
    uint8_t nonce[PTM_NONCE_SIZE] = {0};
    uint8_t buf1[PTM_CRYPTO_DATA_SIZE] = {0};
    uint8_t buf2[PTM_CRYPTO_DATA_SIZE] = {0};
    uint8_t buf3[PTM_CRYPTO_DATA_SIZE] = {0};

    /* Validate packet length and BLE GAP address */
    if ((p_data_pkt->data.data_packet.length > (PTM215B_DATA_PACKET_MAX_SIZE - PTM215B_DATA_PACKET_MIC_SIZE))
        || memcmp(p_data_pkt->p_ble_gap_addr, p_secmat->p_ble_gap_addr, BLE_GAP_ADDR_LEN) != 0)
    {
        return false;
    }

    /* Derive Nonce */
    memcpy(&nonce[PTM_NONCE_SRC_ADDR_OFFSET], (void *)p_data_pkt->p_ble_gap_addr, BLE_GAP_ADDR_LEN);
    memcpy(&nonce[PTM_NONCE_SEQ_OFFSET],
          (void *)&p_data_pkt->data.data_packet.p_raw_data[PTM215B_DATA_PACKET_SEQ_COUNTER_OFFSET],
          PTM215B_DATA_PACKET_SEQ_COUNTER_SIZE);

    /* Calculate B0 */
    buf1[0] = B0_FLAG;
    memcpy(&buf1[1], &nonce[0], PTM_NONCE_SIZE);

    /* Derive X_1 */
    aes_encrypt(p_secmat->p_key, buf1, buf2);

    /* Calculate B1 */
    memset(buf1, 0x00, PTM_CRYPTO_DATA_SIZE);
    buf1[1] = p_data_pkt->data.data_packet.length;
    memcpy(&buf1[CRYPTO_B1_DATA_OFFSET],
           p_data_pkt->data.data_packet.p_raw_data,
           p_data_pkt->data.data_packet.length);

    /* Derive X_1A */
    utils_xor(buf3, buf1, buf2, PTM_CRYPTO_DATA_SIZE);

    /* Derive X_2 */
    aes_encrypt(p_secmat->p_key, buf3, buf1);

    /* Calculate A0 */
    memset(buf3, 0x00, PTM_CRYPTO_DATA_SIZE);
    buf3[0] = A0_FLAG;
    memcpy(&buf3[1], &nonce[0], PTM_NONCE_SIZE);

    /* Calculate S_0 */
    aes_encrypt(p_secmat->p_key, buf3, buf2);

    /* Calculate T_0 */
    utils_xor(buf3, buf1, buf2, PTM_CRYPTO_DATA_SIZE);

    if (memcmp(buf3, p_data_pkt->data.data_packet.p_mic, PTM215B_DATA_PACKET_MIC_SIZE) == 0)
    {
        return true;
    }

    return false;
}

static bool packet_parse(const nrf_mesh_adv_packet_rx_data_t * p_rx_data, enocean_packet_t * p_pkt)
{
    enocean_data_packet_t * p_data = (enocean_data_packet_t *)p_rx_data->p_payload;

    /* commissioning and data telegram differ by the packet size */
    if (p_data->ad_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA &&
        p_data->ad_len <= PTM215B_DATA_AD_LEN_MAX_VALUE &&
        p_data->manufacturer_id == PTM215B_TELEGRAM_MANUFACTURER_ID)
    {
        p_pkt->type = ENOCEAN_DATA_PACKET;
        p_pkt->data.data_packet.seq = p_data->seq;
        p_pkt->p_ble_gap_addr = p_rx_data->p_metadata->params.scanner.adv_addr.addr;
        p_pkt->data.data_packet.p_raw_data = (uint8_t *)p_data;
        p_pkt->data.data_packet.length = p_rx_data->length - PTM215B_DATA_PACKET_MIC_SIZE;
        p_pkt->data.data_packet.p_mic  = &p_rx_data->p_payload[p_pkt->data.data_packet.length];

        return true;
    }

    enocean_commissioning_packet_t * p_comm = (enocean_commissioning_packet_t *)p_rx_data->p_payload;
    /* commissioning telegram of the newer EnOcean devices as per PCN-04-20161201 */
    if (p_comm->new_format.ad_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA &&
        p_comm->new_format.ad_len == PTM215B_COMM_AD_LEN_VALUE &&
        p_comm->new_format.manufacturer_id == PTM215B_TELEGRAM_MANUFACTURER_ID)
    {
        p_pkt->type = ENOCEAN_COMMISSIONING_PACKET;
        p_pkt->data.comm_packet.seq = p_comm->new_format.seq;
        p_pkt->data.comm_packet.p_key = p_comm->new_format.key;
        p_pkt->p_ble_gap_addr = &p_rx_data->p_payload[PTM215B_COMM_PACKET_BLE_ADDR_OFFSET];
        return true;
    }

    /* commissioning telegram of the older EnOcean devices */
    /* AD Type BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME is present in the commissioning packet for the old devices */
    if (p_comm->old_format.ad0_type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME &&
        p_comm->old_format.ad1_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA &&
        p_comm->old_format.manufacturer_id == PTM215B_TELEGRAM_MANUFACTURER_ID)
    {
        p_pkt->type = ENOCEAN_COMMISSIONING_PACKET;
        p_pkt->data.comm_packet.seq = p_comm->old_format.seq;
        p_pkt->data.comm_packet.p_key = p_comm->old_format.key;
        p_pkt->p_ble_gap_addr = p_rx_data->p_metadata->params.scanner.adv_addr.addr;
        return true;
    }
    return false;
}

/**************************************************************************************************/
/* Public API */

void enocean_packet_process(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    enocean_packet_t pkt;
    enocean_evt_t evt;

    if (packet_parse(p_rx_data, &pkt))
    {
        switch (pkt.type)
        {
            case ENOCEAN_COMMISSIONING_PACKET:

                LIST_FOREACH(p_iter, mp_secmats_list_head)
                {
                    enocean_commissioning_secmat_t * p_secmat = PARENT_BY_FIELD_GET(enocean_commissioning_secmat_t,
                                                                                   node,
                                                                                   p_iter);
                    if (memcmp(p_secmat->p_ble_gap_addr, pkt.p_ble_gap_addr, BLE_GAP_ADDR_LEN) == 0)
                    {
                        return;
                    }
                }
                evt.type = ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED;
                evt.p_ble_gap_addr = pkt.p_ble_gap_addr;
                evt.params.secmat.seq = pkt.data.comm_packet.seq;
                evt.params.secmat.p_key = pkt.data.comm_packet.p_key;
                m_app_cb(&evt);
                break;

            case ENOCEAN_DATA_PACKET:
                evt.type = ENOCEAN_EVT_DATA_RECEIVED;
                evt.p_ble_gap_addr = pkt.p_ble_gap_addr;

                LIST_FOREACH(p_iter, mp_secmats_list_head)
                {
                    enocean_commissioning_secmat_t * p_secmat = PARENT_BY_FIELD_GET(enocean_commissioning_secmat_t,
                                                                                   node,
                                                                                   p_iter);
                    if (data_authenticate(&pkt, p_secmat) &&
                        data_replay_check(&pkt, p_secmat))
                    {
                        uint8_t sw_data = pkt.data.data_packet.p_raw_data[PTM215B_DATA_PACKET_SWITCH_STATUS_OFFSET] & ~PTM215B_SWITCH_STATUS_RFU_MASK;
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switch data authenticated \n");
                        evt.params.data.status.a0 = (sw_data >> PTM215B_SWITCH_STATUS_A0_BIT) & 0x01;
                        evt.params.data.status.a1 = (sw_data >> PTM215B_SWITCH_STATUS_A1_BIT) & 0x01;
                        evt.params.data.status.b0 = (sw_data >> PTM215B_SWITCH_STATUS_B0_BIT) & 0x01;
                        evt.params.data.status.b1 = (sw_data >> PTM215B_SWITCH_STATUS_B1_BIT) & 0x01;
                        evt.params.data.status.action = ((sw_data >> PTM215B_SWITCH_STATUS_ACTION_BIT) & 0x01) ?
                                                        RELEASE_ACTION : PRESS_ACTION;
                        evt.params.data.p_optional_data = &pkt.data.data_packet.p_raw_data[PTM215B_DATA_PACKET_OPTIONAL_DATA_OFFSET];
                        evt.params.data.optional_data_length = pkt.data.data_packet.length
                                                            - PTM215B_DATA_PACKET_OPTIONAL_DATA_OFFSET;

                        if (evt.params.data.optional_data_length > PTM215B_DATA_PACKET_OPTIONAL_DATA_MAX_SIZE)
                        {
                            return;
                        }

                        m_app_cb(&evt);
                    }
                }
                break;

            default:
                NRF_MESH_ASSERT(false);
                break;
        }
    }
}

void enocean_secmat_add(enocean_commissioning_secmat_t * p_secmat)
{
    NRF_MESH_ASSERT(p_secmat != NULL);

    list_add(&mp_secmats_list_head, &p_secmat->node);
}

void enocean_translator_init(enocean_translator_cb_t app_callback)
{
    NRF_MESH_ASSERT(app_callback != NULL);

    m_app_cb = app_callback;
}

