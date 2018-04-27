/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#include "net_beacon.h"
#include "beacon.h"
#include "enc.h"
#include "timer_scheduler.h"
#include "timer.h"
#include "rand.h"
#include "nrf_mesh_assert.h"
#include "net_state.h"
#include "nrf_mesh_externs.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/**
 * Size (in octets) of the CMAC field in a secure network broadcast beacon
 * packet.
 */
#define NET_BEACON_CMAC_SIZE 8

/** Highest number of received beacons to keep track of. */
#define BEACON_RX_COUNT_MAX         (0xFF)

#define NETWORK_BKEY_SALT_INPUT ((const uint8_t *) "nkbk")
#define NETWORK_BKEY_SALT_INPUT_LENGTH 4
#define NETWORK_BKEY_INFO ((const uint8_t *) "id128\x01")
#define NETWORK_BKEY_INFO_LENGTH 6


/*****************************************************************************
* Local typedefs
*****************************************************************************/

/*lint -align_max(push) -align_max(1) */

/**
 * Secure network broadcast beacon flags.
 */
typedef struct __attribute((packed))
{
    uint8_t key_refresh        : 1; /**< Key refresh procedure initiated. */
    uint8_t iv_update          : 1; /**< IV update active. */
    uint8_t _rfu               : 6; /**< Reserved for future use. */
} net_beacon_sec_flags_t;

/**
 * Secure network broadcast beacon payload.
 */
typedef struct __attribute((packed))
{
    net_beacon_sec_flags_t flags;                           /**< Beacon flags. */
    uint8_t                network_id[NRF_MESH_NETID_SIZE]; /**< Network identifier. */
    uint32_t               iv_index;                        /**< Current IV index. */
} net_beacon_payload_t;

/**
 * Secure Network Broadcast Beacon.
 */
typedef struct __attribute((packed))
{
    net_beacon_payload_t payload;                    /**< Payload of the secure network beacon. */
    uint8_t              cmac[NET_BEACON_CMAC_SIZE]; /**< CMAC authentication value. */
} net_beacon_t;

/*lint -align_max(pop) */

/*****************************************************************************
* Static globals
*****************************************************************************/
static uint8_t       m_beacon_salt[NRF_MESH_KEY_SIZE]; /**< Beacon salt used for all beacon key derivations. */
static timer_event_t m_tx_event;                       /**< Timer event for beacon transmissions. */
static bool          m_enabled;                        /**< Secure network beacon enabled for this node. */

/*****************************************************************************
* Static functions
*****************************************************************************/

/**
 * Calculate the secure network broadcast beacon CMAC.
 *
 * @param[in] p_beacon Beacon packet to set the CMAC for.
 * @param[in] p_beacon_secmat Network the given beacon belongs to.
 * @param[out] p_cmac CMAC destination pointer.
 */
static void make_network_beacon_cmac(const net_beacon_t* p_beacon, const nrf_mesh_beacon_secmat_t* p_beacon_secmat, uint8_t* p_cmac)
{
    /* We only want a subset of the cmac in the beacon - push it to a temporary
     * buffer that can fit everything, then paste it in its right place. */
    uint8_t temp[NRF_MESH_KEY_SIZE];
    enc_aes_cmac(p_beacon_secmat->key, (const uint8_t *) &p_beacon->payload,
            sizeof(net_beacon_payload_t), temp);
    memcpy(p_cmac, temp, NET_BEACON_CMAC_SIZE);
}

/**
 * Validates a beacon packet's CMAC value.
 *
 * @param[in] p_beacon_secmat Network beacon security material.
 * @param[in] p_packet_in     Packet to be validated.
 *
 * @return true  If the CMAC is correct.
 * @return false Otherwise.
 */
static bool is_valid_beacon_pkt(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                                const net_beacon_t* p_packet_in)
{
    uint8_t cmac[NET_BEACON_CMAC_SIZE];
    make_network_beacon_cmac(p_packet_in, p_beacon_secmat, cmac);

    return memcmp(cmac, p_packet_in->cmac, NET_BEACON_CMAC_SIZE) == 0;
}

static inline void make_network_beacon_packet(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                                            bool key_refresh,
                                            bool iv_update,
                                            uint32_t iv_index,
                                            net_beacon_t * p_net_beacon)
{

    p_net_beacon->payload.flags.key_refresh = key_refresh;
    p_net_beacon->payload.flags.iv_update   = iv_update;
    p_net_beacon->payload.flags._rfu = 0;

    memcpy(p_net_beacon->payload.network_id, p_beacon_secmat->net_id, NRF_MESH_NETID_SIZE);
    p_net_beacon->payload.iv_index = LE2BE32(iv_index);

    make_network_beacon_cmac(p_net_beacon, p_beacon_secmat, p_net_beacon->cmac);
}

static inline uint32_t beacon_interval_in_seconds(uint32_t time_diff_seconds, uint32_t observed_beacons)
{
    if (time_diff_seconds == 0)
    {
        /* At worst, the beacon will come in <interval> seconds before the next
         * timeout. Use the worst case, if we don't have a diff. */
        time_diff_seconds = (m_tx_event.interval / 1000000);
    }
    /* According to Mesh Profile Specification v1.0 section 3.9.3.1:
     *
     * Beacon interval = Observation period * (Observed beacons + 1) / Expected number of beacons
     *
     * Typically:
     * - Base beacon interval is 10 seconds.
     * - Observation period should be 2 * base beacon interval.
     * - Expected number of beacons = 1 per 10 seconds.
     */
    uint32_t beacons_per_base_interval = (observed_beacons * NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS) / time_diff_seconds;
    return (NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS * (beacons_per_base_interval + 1));
}

static void network_beacon_tx_cb(uint32_t timestamp, void * p_context)
{
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    uint32_t iv_index = net_state_beacon_iv_index_get();
    bool iv_update_flag = (net_state_iv_update_get() == NET_STATE_IV_UPDATE_IN_PROGRESS);

    /* Cycle through all the beacon info structures, and schedule a beacon for
     * each, if they haven't been transmitted too often. */
    nrf_mesh_key_refresh_phase_t kr_phase;
    for (nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase);
            p_beacon_info != NULL;
            nrf_mesh_beacon_info_next_get(NULL, &p_beacon_info, &kr_phase))
    {
        uint32_t next_timeout = p_beacon_info->p_tx_info->tx_timestamp + SEC_TO_US(p_beacon_info->p_tx_info->tx_interval_seconds);
        if (timestamp == next_timeout || TIMER_OLDER_THAN(next_timeout, timestamp))
        {
            const nrf_mesh_beacon_secmat_t * p_secmat = kr_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ?
                &p_beacon_info->secmat_updated : &p_beacon_info->secmat;
            net_beacon_t beacon_payload;
            make_network_beacon_packet(p_secmat,
                    kr_phase == NRF_MESH_KEY_REFRESH_PHASE_2,
                    iv_update_flag,
                    iv_index,
                    &beacon_payload);
            uint32_t status = beacon_tx(BEACON_TYPE_SEC_NET_BCAST, &beacon_payload, sizeof(beacon_payload), 1);
            if (status == NRF_SUCCESS)
            {
                p_beacon_info->p_tx_info->tx_interval_seconds = beacon_interval_in_seconds(p_beacon_info->p_tx_info->tx_interval_seconds, p_beacon_info->p_tx_info->rx_count);
                p_beacon_info->p_tx_info->tx_timestamp = timestamp;
                p_beacon_info->p_tx_info->rx_count = 0;
            }
        }
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void net_beacon_init(void)
{
    /* Start the beacon in enabled state: */
    m_enabled = true;

    /* Sample twice per interval to reduce penalty from minor misses: */
    m_tx_event.interval = SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS) / 2;
    m_tx_event.cb = network_beacon_tx_cb;
    m_tx_event.p_context = NULL;

    /* Avoid beacon collisions between devices powering up at the same time: */
    uint32_t offset;
    rand_hw_rng_get((uint8_t*) &offset, sizeof(offset));
    m_tx_event.timestamp = timer_now() + (offset % m_tx_event.interval);

    timer_sch_schedule((timer_event_t *) &m_tx_event);

    enc_s1(NETWORK_BKEY_SALT_INPUT, NETWORK_BKEY_SALT_INPUT_LENGTH, m_beacon_salt);
}

void net_beacon_state_set(bool enabled)
{
    /* Enable the beacon from disabled state: */
    if (!m_enabled && enabled)
    {
        timer_sch_reschedule((timer_event_t *) &m_tx_event, timer_now() + m_tx_event.interval);
    }
    else if (m_enabled && !enabled)
    {
        timer_sch_abort((timer_event_t *) & m_tx_event);
    }

    m_enabled = enabled;
}

bool net_beacon_state_get(void)
{
    return m_enabled;
}

void net_beacon_packet_in(const uint8_t * p_beacon_data, uint8_t data_length, const nrf_mesh_rx_metadata_t * p_meta)
{
    NRF_MESH_ASSERT(p_beacon_data != NULL);
    net_beacon_t* p_beacon = (net_beacon_t*) p_beacon_data;

    nrf_mesh_key_refresh_phase_t subnet_kr_phase;
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    for (nrf_mesh_beacon_info_next_get(p_beacon->payload.network_id, &p_beacon_info, &subnet_kr_phase);
            p_beacon_info != NULL;
            nrf_mesh_beacon_info_next_get(p_beacon->payload.network_id, &p_beacon_info, &subnet_kr_phase))
    {
        NRF_MESH_ASSERT(p_beacon_info->p_tx_info != NULL);
        const uint8_t * p_beacon_net_id = NULL;

        bool valid_pkt = false;
        if (is_valid_beacon_pkt(&p_beacon_info->secmat, p_beacon))
        {
            p_beacon_net_id = p_beacon_info->secmat.net_id;
            valid_pkt = true;
        }
        else if (subnet_kr_phase != NRF_MESH_KEY_REFRESH_PHASE_0 && is_valid_beacon_pkt(&p_beacon_info->secmat_updated, p_beacon))
        {
            p_beacon_net_id = p_beacon_info->secmat_updated.net_id;
            valid_pkt = true;
        }

        if (valid_pkt)
        {
            if (p_beacon_info->p_tx_info->rx_count < BEACON_RX_COUNT_MAX)
            {
                p_beacon_info->p_tx_info->rx_count++;
            }
            if (p_beacon_info->iv_update_permitted)
            {
                net_state_beacon_received(BE2LE32(p_beacon->payload.iv_index),
                        p_beacon->payload.flags.iv_update,
                        p_beacon->payload.flags.key_refresh);
            }
            if (p_beacon_info->callback != NULL)
            {
                p_beacon_info->callback(p_beacon_info,
                        p_beacon_net_id,
                        p_beacon->payload.iv_index,
                        p_beacon->payload.flags.iv_update,
                        p_beacon->payload.flags.key_refresh);
            }
        }
    }
}

