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

#include "net_beacon.h"
#include "beacon.h"
#include "enc.h"
#include "timer_scheduler.h"
#include "timer.h"
#include "rand.h"
#include "nrf_mesh_assert.h"
#include "net_state.h"
#include "nrf_mesh_externs.h"
#include "nrf_mesh_events.h"
#include "event.h"
#include "nordic_common.h"
#include "mesh_opt_core.h"
#include "mesh_config_entry.h"
#include "internal_event.h"

/*****************************************************************************
* Local defines
*****************************************************************************/


/** Highest number of received beacons to keep track of. */
#define BEACON_RX_COUNT_MAX         (0xFFFF)
#define BEACON_TYPICAL_INTERVAL_S   (10)
#define BEACON_OBSERVATION_PERIOD_S (NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS * NRF_MESH_BEACON_OBSERVATION_PERIODS)
#define BEACON_RX_COUNT_EXPECTED    ROUNDED_DIV(BEACON_OBSERVATION_PERIOD_S, BEACON_TYPICAL_INTERVAL_S)

#define NETWORK_BKEY_SALT_INPUT ((const uint8_t *) "nkbk")
#define NETWORK_BKEY_SALT_INPUT_LENGTH 4
#define NETWORK_BKEY_INFO ((const uint8_t *) "id128\x01")
#define NETWORK_BKEY_INFO_LENGTH 6

#define BEACON_INTERVAL_UPPER_LIMIT_S  600
#define BEACON_INTERVAL_LOWER_LIMIT_S  10

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

NRF_MESH_STATIC_ASSERT(NET_BEACON_BUFFER_SIZE == sizeof(net_beacon_t) + BEACON_PACKET_OVERHEAD);

/*****************************************************************************
* Static globals
*****************************************************************************/
static uint8_t       m_beacon_salt[NRF_MESH_KEY_SIZE]; /**< Beacon salt used for all beacon key derivations. */
static timer_event_t m_tx_timer;                       /**< Timer event for beacon transmissions. */
static bool          m_enabled;                        /**< Secure network beacon enabled for this node. */
static advertiser_t  m_adv;
static uint8_t       m_adv_buf[ADVERTISER_PACKET_BUFFER_PACKET_MAXLEN] __attribute((aligned(WORD_SIZE)));
static const nrf_mesh_beacon_info_t * mp_beacon_info;

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
    if (memcmp(p_beacon_secmat->net_id, p_packet_in->payload.network_id, NRF_MESH_NETID_SIZE) == 0)
    {
        uint8_t cmac[NET_BEACON_CMAC_SIZE];
        make_network_beacon_cmac(p_packet_in, p_beacon_secmat, cmac);

        return memcmp(cmac, p_packet_in->cmac, NET_BEACON_CMAC_SIZE) == 0;
    }
    else
    {
        return false;
    }
}

static inline void make_network_beacon_packet(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                                            bool key_refresh,
                                            net_state_iv_update_t iv_update,
                                            uint32_t iv_index,
                                            net_beacon_t * p_net_beacon)
{
    p_net_beacon->payload.flags.key_refresh = key_refresh;
    p_net_beacon->payload.flags.iv_update   = (iv_update == NET_STATE_IV_UPDATE_IN_PROGRESS);
    p_net_beacon->payload.flags._rfu = 0;

    memcpy(p_net_beacon->payload.network_id, p_beacon_secmat->net_id, NRF_MESH_NETID_SIZE);
    p_net_beacon->payload.iv_index = LE2BE32(iv_index);

    make_network_beacon_cmac(p_net_beacon, p_beacon_secmat, p_net_beacon->cmac);
}

static void update_beacon_interval(const nrf_mesh_beacon_tx_info_t * p_tx_info)
{
#ifndef NET_BEACON_LOCK_INTERVAL
    /* According to @tagMeshSp section 3.9.3.1:
     *
     * Beacon interval = Observation period * (Observed beacons + 1) / Expected number of beacons
     * Observation is started since the start of the beacon logic. Interval is recalculated every broadcast interval.
     * Beacon counter is cleared every observation interval.
     */
    uint16_t interval = ROUNDED_DIV(BEACON_OBSERVATION_PERIOD_S * (mp_beacon_info->p_tx_info->rx_count + 1), BEACON_RX_COUNT_EXPECTED);
    interval = MIN(interval, BEACON_INTERVAL_UPPER_LIMIT_S);
    interval = MAX(interval, BEACON_INTERVAL_LOWER_LIMIT_S);
    mp_beacon_info->p_tx_info->interval = interval;

    if (++mp_beacon_info->p_tx_info->observation_count == NRF_MESH_BEACON_OBSERVATION_PERIODS)
    {
        mp_beacon_info->p_tx_info->observation_count = 0;

        mp_beacon_info->p_tx_info->rx_count = 0;
    }
#endif
}

/**
 * Transmit any pending beacons.
 *
 * Beacons are transmitted if we haven't observed more than @c BEACON_RX_COUNT_EXPECTED beacons
 * per @c BEACON_OBSERVATION_PERIOD_S time. To prevent a scenario where a
 * short burst of beacons stops the device from sending beacons for a long time, the beacon interval
 * is changed every observation period. Each beacon info structure holds its own set
 * of counters.
 *
 * @param[in] time_now Current time.
 */
static void beacon_tx(timestamp_t time_now)
{
    nrf_mesh_key_refresh_phase_t kr_phase;

    for (nrf_mesh_beacon_info_next_get(NULL, &mp_beacon_info, &kr_phase);
         mp_beacon_info != NULL;
         nrf_mesh_beacon_info_next_get(NULL, &mp_beacon_info, &kr_phase))
    {
        update_beacon_interval(mp_beacon_info->p_tx_info);
        if (TIMER_DIFF(time_now, mp_beacon_info->p_tx_info->tx_timestamp) >= SEC_TO_US(mp_beacon_info->p_tx_info->interval))
        {
            uint32_t iv_index                         = net_state_beacon_iv_index_get();
            net_state_iv_update_t iv_update           = net_state_iv_update_get();
            const nrf_mesh_beacon_secmat_t * p_secmat = (kr_phase == NRF_MESH_KEY_REFRESH_PHASE_2)
                                                            ? &mp_beacon_info->secmat_updated
                                                            : &mp_beacon_info->secmat;

            net_beacon_t net_beacon;
            make_network_beacon_packet(p_secmat,
                                    net_beacon_key_refresh_flag(kr_phase),
                                    iv_update,
                                    iv_index,
                                    &net_beacon);

            adv_packet_t * p_packet =
                beacon_create(&m_adv, BEACON_TYPE_SEC_NET_BCAST, &net_beacon, sizeof(net_beacon));

            if (p_packet)
            {
                p_packet->config.repeats = 1;
                advertiser_packet_send(&m_adv, p_packet);

                __LOG(LOG_SRC_NETWORK, LOG_LEVEL_DBG1, "SNB TX ID: %02x%02x%02x%02x%02x%02x%02x%02x ivu: %u kr: %u IV: %u\n",
                      net_beacon.payload.network_id[0],
                      net_beacon.payload.network_id[1],
                      net_beacon.payload.network_id[2],
                      net_beacon.payload.network_id[3],
                      net_beacon.payload.network_id[4],
                      net_beacon.payload.network_id[5],
                      net_beacon.payload.network_id[6],
                      net_beacon.payload.network_id[7],
                      net_beacon.payload.flags.iv_update,
                      net_beacon.payload.flags.key_refresh,
                      iv_index);
                /* Start a new beacon interval for this beacon. */
                mp_beacon_info->p_tx_info->tx_timestamp = time_now;

                __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_NET_BEACON_TX, 0, 0, &net_beacon.payload);
            }

            /* We've scheduled a beacon, and should wait for the TX complete before we iterate */
            return;
        }
    }
}

static void beacon_tx_timeout(timestamp_t timestamp, void * p_context)
{
    beacon_tx(timestamp);
}

static void tx_complete_cb(advertiser_t * p_adv, nrf_mesh_tx_token_t token, timestamp_t timestamp)
{
    beacon_tx(timer_now());
}

/*****************************************************************************
 * Mesh Config wrapper functions
 *****************************************************************************/
static uint32_t beacon_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_SEC_NWK_BCN_RECORD == entry_id.record);

    bool enabled = *(bool *)p_entry;

    /* Enable the beacon from disabled state: */
    if (!m_enabled && enabled)
    {
        advertiser_enable(&m_adv);
        timer_sch_reschedule((timer_event_t *) &m_tx_timer, timer_now() + m_tx_timer.interval);
    }
    else if (m_enabled && !enabled)
    {
        advertiser_disable(&m_adv);
        timer_sch_abort((timer_event_t *) & m_tx_timer);
    }

    m_enabled = enabled;

    return NRF_SUCCESS;
}

static void beacon_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_CORE_SEC_NWK_BCN_RECORD == entry_id.record);

    *(bool *)p_entry = m_enabled;
}

static void beacon_deleter(mesh_config_entry_id_t entry_id)
{
#if MESH_FEATURE_LPN_ENABLED && !MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
    m_enabled = false;
    advertiser_disable(&m_adv);
    timer_sch_abort((timer_event_t *) & m_tx_timer);
#else
    m_enabled = true;
    advertiser_enable(&m_adv);
    timer_sch_reschedule((timer_event_t *) &m_tx_timer, timer_now() + m_tx_timer.interval);
#endif
}

MESH_CONFIG_ENTRY(net_beacon_enable,
                  MESH_OPT_CORE_SEC_NWK_BCN_EID,
                  1,
                  sizeof(m_enabled),
                  beacon_setter,
                  beacon_getter,
                  beacon_deleter,
                  true);

/*****************************************************************************
* Interface functions
*****************************************************************************/
void net_beacon_init(void)
{
    /* Start the beacon in enabled state: */
    mp_beacon_info = NULL;

    m_tx_timer.interval  = SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS);
    m_tx_timer.cb        = beacon_tx_timeout;
    m_tx_timer.p_context = NULL;

    /* Avoid beacon collisions between devices powering up at the same time: */
    uint32_t offset;
    rand_hw_rng_get((uint8_t *) &offset, sizeof(offset));
    m_tx_timer.timestamp = timer_now() + (offset % m_tx_timer.interval);

    advertiser_instance_init(&m_adv, tx_complete_cb, m_adv_buf, sizeof(m_adv_buf));
    /* Limit the beacon sending to once per second. For devices with more than 10 subnets, the
     * per-subnet beacons will run less frequent than every 10 seconds. This is to avoid a scenario
     * where a single device diminishes the throughput of its neighbors. */
    advertiser_interval_set(&m_adv, SEC_TO_MS(1));

    enc_s1(NETWORK_BKEY_SALT_INPUT, NETWORK_BKEY_SALT_INPUT_LENGTH, m_beacon_salt);
}

void net_beacon_enable(void)
{
    if (!m_enabled)
    {
        advertiser_enable(&m_adv);
        timer_sch_schedule((timer_event_t *) &m_tx_timer);
        m_enabled = true;
    }
}

void net_beacon_state_set(bool enabled)
{
    mesh_config_entry_id_t id = MESH_OPT_CORE_SEC_NWK_BCN_EID;

    NRF_MESH_ASSERT(NRF_SUCCESS == mesh_config_entry_set(id, &enabled));
}

bool net_beacon_state_get(void)
{
    bool enabled;
    mesh_config_entry_id_t id = MESH_OPT_CORE_SEC_NWK_BCN_EID;

    NRF_MESH_ASSERT(NRF_SUCCESS == mesh_config_entry_get(id, &enabled));
    return enabled;
}

uint32_t net_beacon_build(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                          uint32_t iv_index,
                          net_state_iv_update_t iv_update,
                          bool key_refresh,
                          uint8_t * p_buffer)
{
    if (p_beacon_secmat == NULL || p_buffer == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_buffer[0] = BEACON_TYPE_SEC_NET_BCAST;

    make_network_beacon_packet(p_beacon_secmat,
                               key_refresh,
                               iv_update,
                               iv_index,
                               (net_beacon_t *) &p_buffer[BEACON_PACKET_OVERHEAD]);

    return NRF_SUCCESS;
}

void net_beacon_packet_in(const uint8_t * p_beacon_data, uint8_t data_length, const nrf_mesh_rx_metadata_t * p_meta)
{
    NRF_MESH_ASSERT(p_beacon_data != NULL);
    net_beacon_t* p_beacon = (net_beacon_t*) p_beacon_data;
    nrf_mesh_key_refresh_phase_t subnet_kr_phase;
    const nrf_mesh_beacon_info_t * p_beacon_info = NULL;
    const uint32_t iv_index = BE2LE32(p_beacon->payload.iv_index);

    __LOG(LOG_SRC_NETWORK, LOG_LEVEL_DBG1, "SNB RX ID: %02x%02x%02x%02x%02x%02x%02x%02x ivu: %u kr: %u IV: %u\n",
          p_beacon->payload.network_id[0],
          p_beacon->payload.network_id[1],
          p_beacon->payload.network_id[2],
          p_beacon->payload.network_id[3],
          p_beacon->payload.network_id[4],
          p_beacon->payload.network_id[5],
          p_beacon->payload.network_id[6],
          p_beacon->payload.network_id[7],
          p_beacon->payload.flags.iv_update,
          p_beacon->payload.flags.key_refresh,
          iv_index);


    for (nrf_mesh_beacon_info_next_get(p_beacon->payload.network_id, &p_beacon_info, &subnet_kr_phase);
            p_beacon_info != NULL;
            nrf_mesh_beacon_info_next_get(p_beacon->payload.network_id, &p_beacon_info, &subnet_kr_phase))
    {
        NRF_MESH_ASSERT(p_beacon_info->p_tx_info != NULL);
        const nrf_mesh_beacon_secmat_t * p_beacon_secmat = NULL;


        bool valid_pkt = false;
        if (is_valid_beacon_pkt(&p_beacon_info->secmat, p_beacon))
        {
            p_beacon_secmat = &p_beacon_info->secmat;
            valid_pkt = true;
        }
        else if (subnet_kr_phase != NRF_MESH_KEY_REFRESH_PHASE_0 && is_valid_beacon_pkt(&p_beacon_info->secmat_updated, p_beacon))
        {
            p_beacon_secmat = &p_beacon_info->secmat_updated;
            valid_pkt = true;
        }

        if (valid_pkt)
        {
            if (p_beacon_info->p_tx_info->rx_count < BEACON_RX_COUNT_MAX)
            {
                p_beacon_info->p_tx_info->rx_count++;
            }

            nrf_mesh_evt_t beacon_evt;
            beacon_evt.type = NRF_MESH_EVT_NET_BEACON_RECEIVED;
            beacon_evt.params.net_beacon.flags.iv_update   = (net_state_iv_update_t) p_beacon->payload.flags.iv_update;
            beacon_evt.params.net_beacon.flags.key_refresh = p_beacon->payload.flags.key_refresh;
            beacon_evt.params.net_beacon.iv_index          = iv_index;
            beacon_evt.params.net_beacon.p_beacon_info     = p_beacon_info;
            beacon_evt.params.net_beacon.p_beacon_secmat   = p_beacon_secmat;
            beacon_evt.params.net_beacon.p_rx_metadata     = p_meta;
            beacon_evt.params.net_beacon.p_auth_value      = p_beacon->cmac;
            event_handle(&beacon_evt);
        }
    }
}
