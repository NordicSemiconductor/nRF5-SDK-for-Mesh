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

#include "device_state_manager.h"

#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_mesh.h"
#include "nrf_mesh_defines.h"
#include "packet_mesh.h"
#include "nrf_mesh_keygen.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_assert.h"
#include "bitfield.h"
#include "utils.h"
#include "bearer_event.h"
#include "event.h"
#include "net_state.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_externs.h"
#include "mesh_opt_core.h"
#include "nrf_mesh_keygen.h"
#include "heartbeat.h"

#if MESH_FEATURE_LPN_ENABLED
#include "mesh_lpn.h"
#include "mesh_lpn_internal.h"
#endif

#if MESH_FEATURE_FRIEND_ENABLED
#include "mesh_opt_friend.h"
#endif

#include "mesh_opt_dsm.h"
#include "mesh_config_entry.h"

#if MESH_FEATURE_GATT_PROXY_ENABLED
#include "proxy.h"
#endif  /* MESH_FEATURE_GATT_PROXY_ENABLED */

/*lint -e415 -e416 Lint fails to understand the boundary checking used for handles in this module (MBTLE-1831). */

/*****************************************************************************
* Local defines
*****************************************************************************/

/** Key index of the primary subnet. */
#define PRIMARY_SUBNET_INDEX        (0)


/** In order for the two to share address space, the first devkey handle starts
 * after the last appkey handle */
#define DSM_DEVKEY_HANDLE_START     DSM_APP_MAX

/** In order for the two to share address space, the first virtual address
 * handle starts after the last nonvirtual handle */
#define DSM_VIRTUAL_HANDLE_START     DSM_NONVIRTUAL_ADDR_MAX

NRF_MESH_STATIC_ASSERT(DSM_APP_MAX >= 1 && DSM_APP_MAX <= DSM_APP_MAX_LIMIT);
NRF_MESH_STATIC_ASSERT(DSM_SUBNET_MAX >= 1 && DSM_SUBNET_MAX <= DSM_SUBNET_MAX_LIMIT);
NRF_MESH_STATIC_ASSERT(DSM_DEVICE_MAX >= 1);
NRF_MESH_STATIC_ASSERT(DSM_ADDR_MAX <= DSM_ADDR_MAX_LIMIT);

/**
 * Number of friendship credentials to be supported by the mesh stack.
 *
 * @note Shall be set to at least 1 if LPN feature is enabled
 */
#if MESH_FEATURE_LPN_ENABLED && MESH_FEATURE_FRIEND_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS (MESH_FRIEND_FRIENDSHIP_COUNT + 1)
#elif MESH_FEATURE_LPN_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS  1
#elif MESH_FEATURE_FRIEND_ENABLED
    #define MESH_FRIENDSHIP_CREDENTIALS (MESH_FRIEND_FRIENDSHIP_COUNT)
#endif

/* Compares 2 application key identifiers. */
#define IS_AIDS_EQUAL(aid1, aid2) (((aid1) & PACKET_MESH_TRS_ACCESS_AID_MASK) == ((aid2) & PACKET_MESH_TRS_ACCESS_AID_MASK))

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    mesh_key_index_t net_key_index;

    uint8_t root_key[NRF_MESH_KEY_SIZE];
    uint8_t root_key_updated[NRF_MESH_KEY_SIZE];

    nrf_mesh_network_secmat_t secmat;
    nrf_mesh_network_secmat_t secmat_updated;

    nrf_mesh_key_refresh_phase_t key_refresh_phase;

    struct
    {
        nrf_mesh_beacon_tx_info_t tx_info;
        nrf_mesh_beacon_info_t info;
    } beacon;
} subnet_t;

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
typedef struct
{
    dsm_handle_t subnet_handle;
    nrf_mesh_keygen_friendship_secmat_params_t secmat_params;

    nrf_mesh_network_secmat_t secmat;
    nrf_mesh_network_secmat_t secmat_updated;
} friendship_secmat_t;
#endif

typedef struct
{
    mesh_key_index_t app_key_index;
    dsm_handle_t subnet_handle; /**< Subnetwork this application key is bound to. */
    nrf_mesh_application_secmat_t secmat;

    bool key_updated;
    nrf_mesh_application_secmat_t secmat_updated;
} appkey_t;

/** Device key instance. */
typedef struct
{
    uint16_t key_owner; /**< Unicast address of the device that owns the devkey. */
    dsm_handle_t subnet_handle; /**< Subnetwork this device key is bound to. */
    nrf_mesh_application_secmat_t secmat; /**< Security material for packet encryption and decryption. */
} devkey_t;

typedef struct
{
    uint16_t address;
    uint8_t publish_count;
    uint8_t subscription_count;
} regular_address_t;

typedef struct
{
    uint16_t address;
    uint8_t publish_count;
    uint8_t subscription_count;
    /** 128-bit virtual label UUID. */
    uint8_t uuid[NRF_MESH_UUID_SIZE];
} virtual_address_t;

typedef enum
{
    DSM_ADDRESS_ROLE_SUBSCRIBE,
    DSM_ADDRESS_ROLE_PUBLISH
} dsm_address_role_t;

typedef struct
{
    uint8_t is_metadata_stored : 1;
    uint8_t is_load_failed : 1;
    uint8_t is_legacy_found : 1;
} local_dsm_status_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
/** Virtual addresses */
static virtual_address_t m_virtual_addresses[DSM_VIRTUAL_ADDR_MAX];
/** Non-virtual addresses */
static regular_address_t m_addresses[DSM_NONVIRTUAL_ADDR_MAX];
/** Local RX unicast address */
static dsm_local_unicast_address_t m_local_unicast_addr;
/** Security information for each subnet and its associated application */
static subnet_t m_subnets[DSM_SUBNET_MAX];

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
/** Security information for friendship */
static friendship_secmat_t m_friendships[MESH_FRIENDSHIP_CREDENTIALS];
#endif

/** Security information associated with each appkey */
static appkey_t m_appkeys[DSM_APP_MAX];
/** Security information associated with each devkey */
static devkey_t m_devkeys[DSM_DEVICE_MAX];

/** Flag indicating whether the device is part of the primary subnet */
static bool m_has_primary_subnet;
/** Mesh event handler */
static nrf_mesh_evt_handler_t m_mesh_evt_handler;

/* Bitfields for all entry types, indicating whether they're allocated or not. */
static uint32_t m_addr_unicast_allocated[BITFIELD_BLOCK_COUNT(1)];
static uint32_t m_addr_nonvirtual_allocated[BITFIELD_BLOCK_COUNT(DSM_NONVIRTUAL_ADDR_MAX)];
static uint32_t m_addr_virtual_allocated[BITFIELD_BLOCK_COUNT(DSM_VIRTUAL_ADDR_MAX)];
static uint32_t m_subnet_allocated[BITFIELD_BLOCK_COUNT(DSM_SUBNET_MAX)];
static uint32_t m_appkey_allocated[BITFIELD_BLOCK_COUNT(DSM_APP_MAX)];
static uint32_t m_devkey_allocated[BITFIELD_BLOCK_COUNT(DSM_DEVICE_MAX)];

/** Set of the global flags to keep track of the dsm changes.*/
static local_dsm_status_t m_status;

static void dsm_entry_store(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property);
static void dsm_entry_invalidate(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property);

/*****************************************************************************
* Static functions
*****************************************************************************/
/* Checks if a given address handle is a valid non-virtual address handle. */
static inline bool address_handle_nonvirtual_valid(dsm_handle_t address_handle)
{
    if (address_handle >= DSM_NONVIRTUAL_ADDR_MAX || !bitfield_get(m_addr_nonvirtual_allocated, address_handle))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/* Checks if a given address handle is a valid virtual address handle. */
static inline bool address_handle_virtual_valid(dsm_handle_t address_handle)
{
    address_handle -= DSM_VIRTUAL_HANDLE_START;
    if (address_handle >= DSM_VIRTUAL_ADDR_MAX || !bitfield_get(m_addr_virtual_allocated, address_handle))
    {
        return false;
    }
    else
    {
        return true;
    }
}

static inline bool address_handle_valid(dsm_handle_t address_handle)
{
    return (address_handle_nonvirtual_valid(address_handle) ||
            address_handle_virtual_valid(address_handle));
}


/** Gets whether the given address is one of our local unicast addresses. */
static inline bool is_own_unicast_addr(uint16_t addr)
{
    return (bitfield_get(m_addr_unicast_allocated, 0) &&
            addr >= m_local_unicast_addr.address_start &&
            addr < m_local_unicast_addr.address_start + m_local_unicast_addr.count);
}

/** Gets the unicast address if it's in the rx address list.
 *  Returns true if found, otherwise false.
 */
static bool rx_unicast_address_get(uint16_t address, nrf_mesh_address_t * p_address)
{
    if (is_own_unicast_addr(address))
    {
        p_address->value = address;
        p_address->type = NRF_MESH_ADDRESS_TYPE_UNICAST;
        p_address->p_virtual_uuid = NULL;
        return true;
    }
    return false;
}

/** Checks if an address exists in the rx address list.
 *  Returns a suitable location for a new address if it does not.
 */
static bool address_exists(uint16_t address, dsm_handle_t * p_handle, nrf_mesh_address_type_t * p_type)
{
    *p_handle = DSM_HANDLE_INVALID;

    *p_type = nrf_mesh_address_type_get(address);
    if (*p_type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
    {
        for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX; ++i)
        {
            if (*p_handle == DSM_HANDLE_INVALID && !bitfield_get(m_addr_virtual_allocated, i))
            {
                *p_handle = i;
            }
            else if (m_virtual_addresses[i].address == address)
            {
                *p_handle = i;
                return true;
            }
        }
    }
    else if (*p_type == NRF_MESH_ADDRESS_TYPE_GROUP || *p_type == NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
        {
            if (*p_handle == DSM_HANDLE_INVALID && !bitfield_get(m_addr_nonvirtual_allocated, i))
            {
                *p_handle = i;
            }
            else if (m_addresses[i].address == address)
            {
                *p_handle = i;
                return true;
            }
        }
    }

    return false;
}

/** Checks if the given nonvirtual address exists in the rx address list.
 *  Returns a suitable location for a new group rx address if it does not.
 */
static bool address_nonvirtual_subscription_exists(uint16_t address, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        if (*p_handle == DSM_HANDLE_INVALID && !bitfield_get(m_addr_nonvirtual_allocated, i))
        {
            *p_handle = i;
        }
        else if (m_addresses[i].address == address && m_addresses[i].subscription_count > 0)
        {
            *p_handle = i;
            return true;
        }
    }
    /* Finally, check whether heartbeat subscribes to this address. */
    const heartbeat_subscription_state_t * p_hb_sub = heartbeat_subscription_get();
    return (p_hb_sub->dst == address);
}

/** Gets the group address if it's in the address subscription list.
 *  Returns true if found, otherwise false.
 */
static bool rx_group_address_get(uint16_t address, nrf_mesh_address_t * p_address)
{
    p_address->value = address;
    p_address->type = NRF_MESH_ADDRESS_TYPE_GROUP;
    p_address->p_virtual_uuid = NULL;

    switch (address)
    {
        case NRF_MESH_ALL_PROXIES_ADDR:
#if MESH_FEATURE_GATT_PROXY_ENABLED
            return proxy_is_enabled();
#else
            return false;
#endif
        case NRF_MESH_ALL_FRIENDS_ADDR:
        {
#if MESH_FEATURE_FRIEND_ENABLED
            bool is_enabled;
            NRF_MESH_ERROR_CHECK(mesh_opt_friend_get(&is_enabled));
            return is_enabled;
#else
            return false;
#endif
        }
        case NRF_MESH_ALL_RELAYS_ADDR:
        {
#if MESH_FEATURE_RELAY_ENABLED
            mesh_opt_core_adv_t options;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &options));
            return options.enabled;
#else
            return false;
#endif
        }
        case NRF_MESH_ALL_NODES_ADDR:
            return true;
        default:
        {
            dsm_handle_t handle;
            return address_nonvirtual_subscription_exists(address, &handle);
        }
    }
}

/** Checks if the given 16-bit virtual address exists in the address list and provides the index.
 *  Provides a suitable location for a new virtual address via p_index if it does not.
 *  Since there might be multiple virtual addresses with the same address value,
 *  this function will start its search from the given handle value.
 *  Returns true if the address already exists.
 */
static bool virtual_address_index_get(uint16_t address, uint16_t * p_index)
{
    uint32_t i;
    if (*p_index >= DSM_VIRTUAL_ADDR_MAX)
    {
        i = 0;
    }
    else
    {
        i = *p_index + 1;
    }
    *p_index = DSM_HANDLE_INVALID;

    for (; i < DSM_VIRTUAL_ADDR_MAX; ++i)
    {
        if (!bitfield_get(m_addr_virtual_allocated, i))
        {
            *p_index = i;
        }
        else if (m_virtual_addresses[i].address == address)
        {
            *p_index = i;
            return true;
        }
    }
    return false;
}
/** Checks if the given virtual address uuid exists in the address list and provides the index to it.
 *  Provides a suitable location for a new virtual address via p_index if it does not.
 *  Returns true if the address already exists.
 */
static bool virtual_address_uuid_index_get(const uint8_t * p_uuid, uint16_t * p_index)
{
    *p_index = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX; ++i)
    {
        if (!bitfield_get(m_addr_virtual_allocated, i))
        {
            if (*p_index == DSM_HANDLE_INVALID)
            {
                *p_index = i;
            }
        }
        else if (memcmp(m_virtual_addresses[i].uuid, p_uuid, NRF_MESH_UUID_SIZE) == 0)
        {
            *p_index = i;
            return true;
        }
    }
    return false;
}

/** Gets the virtual address if it's in the rx address list.
 *  Returns true if found, otherwise false.
 */
static bool rx_virtual_address_get(uint16_t address, nrf_mesh_address_t * p_address)
{
    uint16_t virtual_addr_index;
    /* Set the virtual_addr_index to the given uuid if it exists.*/
    if (NULL == p_address->p_virtual_uuid || !virtual_address_uuid_index_get(p_address->p_virtual_uuid, &virtual_addr_index))
    {
        virtual_addr_index = DSM_HANDLE_INVALID;
    }

    if (virtual_address_index_get(address, &virtual_addr_index) && m_virtual_addresses[virtual_addr_index].subscription_count > 0)
    {
        p_address->value = address;
        p_address->type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
        p_address->p_virtual_uuid = m_virtual_addresses[virtual_addr_index].uuid;
        return true;
    }
    else
    {
        return false;
    }
}

/** Checks if the given address (must be group or unicast) exists in the address list.
 *  Provides the address location via p_handle if it does or a suitable location
 *  for a new address if it does not.
 *  Returns true if the address exists.
 */
static bool non_virtual_address_handle_get(uint16_t address, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    /* Check for duplicates, and make note of the first valid handle */
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        if (bitfield_get(m_addr_nonvirtual_allocated, i))
        {
            if (m_addresses[i].address == address)
            {
                *p_handle = i;
                return true;
            }
        }
        else if (*p_handle == DSM_HANDLE_INVALID)
        {
            *p_handle = i;
        }
    }
    return false;
}

/** Checks if the given network key index exists in the network key list.
 *  Provides a suitable location for a new network key via p_handle if it does not.
 *  Returns true if the network key exists, otherwise false.
 */
static inline bool net_key_handle_get(mesh_key_index_t net_key_index, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_SUBNET_MAX; i++)
    {
        if (bitfield_get(m_subnet_allocated, i))
        {
            if (m_subnets[i].net_key_index == net_key_index)
            {
                *p_handle = i;
                return true;
            }
        }
        else if (*p_handle == DSM_HANDLE_INVALID)
        {
            *p_handle = i;
        }
    }
    return false;
}

/** Finds the devkey handle of the given owner address, or an available
 * handle if it doesn't exist. Returns true if the devkey exists.
 */
static bool dev_key_handle_get(uint16_t owner_addr, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_DEVICE_MAX; i++)
    {
        if (bitfield_get(m_devkey_allocated, i))
        {
            if (m_devkeys[i].key_owner == owner_addr)
            {
                *p_handle = DSM_DEVKEY_HANDLE_START + i;
                return true;
            }
        }
        else if (*p_handle == DSM_HANDLE_INVALID)
        {
            *p_handle = DSM_DEVKEY_HANDLE_START + i;
        }
    }
    return false;
}

/** Checks if the given appkey index exists in the appkey list.
 *  Provides a suitable location for a new appkey via p_handle if it does not.
 *  Returns true if the appkey exists, otherwise false.
 */
static bool app_key_handle_get(mesh_key_index_t app_key_index, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_APP_MAX; i++)
    {
        if (bitfield_get(m_appkey_allocated, i))
        {
            if (m_appkeys[i].app_key_index == app_key_index)
            {
                *p_handle = i;
                return true;
            }
        }
        else if (*p_handle == DSM_HANDLE_INVALID)
        {
            *p_handle = i;
        }
    }
    return false;
}

/** Provides all the available appkey indices via p_key_list.
 *  Returns false if the given list is not large enough to store all the available appkey indices.
 */
static bool get_all_appkeys(dsm_handle_t subnet_handle, mesh_key_index_t * p_key_list, uint32_t * p_count)
{
    const uint32_t key_list_max_size = *p_count;
    *p_count = 0;

    for (uint32_t i = 0; i < DSM_APP_MAX; ++i)
    {
        if (bitfield_get(m_appkey_allocated, i) && m_appkeys[i].subnet_handle == subnet_handle)
        {
            if (*p_count == key_list_max_size)
            {
                /* We would be overstepping the buffer size if we add one more */
                return false;
            }
            p_key_list[*p_count] = m_appkeys[i].app_key_index;
            (*p_count)++;
        }
    }
    return true;
}

/** Provides all the available network key indices via p_key_list.
 *  Returns false if the given list is not large enough to store all the available network key indices.
 */
static bool get_all_subnets(mesh_key_index_t * p_key_list, uint32_t * p_count)
{
    const uint32_t key_list_max_size = *p_count;
    *p_count = 0;

    for (uint32_t i = 0; (i < DSM_SUBNET_MAX); ++i)
    {
        if (bitfield_get(m_subnet_allocated, i))
        {
            if (*p_count == key_list_max_size)
            {
                /* We would be overstepping the buffer size if we add one more */
                return false;
            }
            p_key_list[*p_count] = m_subnets[i].net_key_index;
            (*p_count)++;
        }
    }
    return true;
}

/** Returns the index to the m_subnets array for the given network secmat,
 *  Returns DSM_HANDLE_INVALID if not found.
 */
static dsm_handle_t get_subnet_handle(const nrf_mesh_network_secmat_t * p_secmat)
{
    NRF_MESH_ASSERT(NULL != p_secmat);

    dsm_handle_t retval = DSM_HANDLE_INVALID;
    for(uint16_t i = 0; i < DSM_SUBNET_MAX; i++)
    {
        if (m_subnets[i].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
        {
            if (p_secmat == &m_subnets[i].secmat || p_secmat == &m_subnets[i].secmat_updated)
            {
                retval = i;
                break;
            }
        }
        else if (p_secmat == &m_subnets[i].secmat)
        {
            retval = i;
            break;
        }
    }

    return retval;
}

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
/**
 * Returns an index of the m_friendships array for the given network secmat
 * Returns end of m_friendship array if secmat is not found
 */
static uint32_t friendship_index_by_secmat_get(const nrf_mesh_network_secmat_t * p_secmat)
{
    uint32_t j = 0;
    for (; j < ARRAY_SIZE(m_friendships); j++)
    {
        if (m_friendships[j].subnet_handle != DSM_HANDLE_INVALID)
        {
            if (p_secmat == &m_friendships[j].secmat ||
                (p_secmat == &m_friendships[j].secmat_updated &&
                m_subnets[m_friendships[j].subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0))
            {
                return j;
            }
        }
    }

    return j;
}
#endif  /* MESH_FEATURE_LPN_ENABLED */

/** Returns the index to the m_subnets array for the given beacon info,
 *  Returns DSM_HANDLE_INVALID if not found.
 */
static dsm_handle_t get_subnet_handle_by_beacon_info(const nrf_mesh_beacon_info_t * p_beacon_info)
{
    NRF_MESH_ASSERT(NULL != p_beacon_info);

    if (p_beacon_info >= &m_subnets[0].beacon.info &&
        p_beacon_info <= &m_subnets[DSM_SUBNET_MAX - 1].beacon.info)
    {
        /* The beacon info is offset by the same amount in each structure, so since
         * we're getting the delta between two substructures of the same structure
         * type, this will get the right index. */
        return (((uint32_t) p_beacon_info - (uint32_t) &m_subnets[0].beacon.info) / sizeof(subnet_t));
    }
    return DSM_HANDLE_INVALID;
}

/** Returns the index to the m_appkeys array for the given appkey secmat,
 *  Returns DSM_HANDLE_INVALID if not found.
 */
static dsm_handle_t get_app_handle(const nrf_mesh_application_secmat_t * p_secmat)
{
    NRF_MESH_ASSERT(NULL != p_secmat);

    for (size_t i = 0; i < DSM_APP_MAX; i++)
    {
        if (p_secmat == &m_appkeys[i].secmat ||
            p_secmat == &m_appkeys[i].secmat_updated)
        {
            return i;
        }
    }

    if (p_secmat >= &m_devkeys[0].secmat &&
        p_secmat <= &m_devkeys[DSM_DEVICE_MAX - 1].secmat)
    {
        /* The secmat is offset by the same amount in each structure, so since
         * we're getting the delta between two substructures of the same structure
         * type, this will get the right index. */
        return DSM_DEVKEY_HANDLE_START + (((uint32_t) p_secmat - (uint32_t) &m_devkeys[0].secmat) / sizeof(devkey_t));

    }
    return DSM_HANDLE_INVALID;
}

static void net_beacon_rx_handle(const nrf_mesh_beacon_info_t * p_beacon_info,
                                 const nrf_mesh_beacon_secmat_t * p_secmat,
                                 uint32_t iv_index,
                                 bool iv_update,
                                 bool key_refresh)
{
    dsm_handle_t subnet_handle = get_subnet_handle_by_beacon_info(p_beacon_info);
    if (subnet_handle == DSM_HANDLE_INVALID)
    {
        return;
    }

    if (p_secmat == &p_beacon_info->secmat_updated)
    {
        if (key_refresh)
        {
            if (m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
            {
                (void) dsm_subnet_update_swap_keys(subnet_handle);
            }
        }
        else
        {
            if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                (void) dsm_subnet_update_commit(subnet_handle);
            }
        }
    }
}

#if MESH_FEATURE_LPN_ENABLED
static void lpn_friend_update_rx_handle(const nrf_mesh_network_secmat_t * p_net_secmat,
                                        uint32_t iv_index,
                                        bool iv_update,
                                        bool key_refresh)
{
    dsm_handle_t subnet_handle = dsm_subnet_handle_get(p_net_secmat);
    NRF_MESH_ASSERT_DEBUG(subnet_handle != DSM_HANDLE_INVALID);

    if (key_refresh)
    {
        if (m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
        {
            (void) dsm_subnet_update_swap_keys(subnet_handle);
        }
    }
    else
    {
        if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
        {
            (void) dsm_subnet_update_commit(subnet_handle);
        }
    }
}
#endif

static bool get_net_secmat_by_nid(dsm_handle_t subnet_handle, uint8_t nid,
                                  const nrf_mesh_network_secmat_t * p_secmat_cmp,
                                  const nrf_mesh_network_secmat_t * p_secmat_secondary_cmp,
                                  const nrf_mesh_network_secmat_t ** pp_secmat_out,
                                  const nrf_mesh_network_secmat_t ** pp_secmat_secondary_out)
{
    if (!bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return false;
    }

    /* If the NIDs for the old and the new network are equal, return both: */
    if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
        && (p_secmat_cmp->nid == nid && p_secmat_secondary_cmp->nid == nid))
    {
        *pp_secmat_out = p_secmat_cmp;
        *pp_secmat_secondary_out = p_secmat_secondary_cmp;
        return true;
    }
    /* During key refresh, return the updated key if it matches the NID: */
    else if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
             && p_secmat_secondary_cmp->nid == nid)
    {
        *pp_secmat_out = p_secmat_secondary_cmp;
        *pp_secmat_secondary_out = NULL;
        return true;
    }
    else if (p_secmat_cmp->nid == nid)
    {
        *pp_secmat_out = p_secmat_cmp;
        *pp_secmat_secondary_out = NULL;
        return true;
    }

    return false;
}

static const nrf_mesh_application_secmat_t * get_devkey_secmat(uint16_t key_address)
{
    if (key_address == NRF_MESH_ADDR_UNASSIGNED)
    {
        return NULL;
    }
    for (uint32_t i = 0; i < DSM_DEVICE_MAX; ++i)
    {
        if (m_devkeys[i].key_owner == key_address)
        {
            return &m_devkeys[i].secmat;
        }
    }
    return NULL;
}

static void get_app_secmat(dsm_handle_t subnet_handle, uint8_t aid, const nrf_mesh_application_secmat_t ** pp_app_secmat,
                           const nrf_mesh_application_secmat_t ** pp_app_secmat_secondary)
{
    uint32_t i = 0;
    if (*pp_app_secmat != NULL)
    {
        /* Iterate over the proceeding elements */
        i = get_app_handle(*pp_app_secmat) + 1;
    }
    for (; i < DSM_APP_MAX; i++)
    {
        if (bitfield_get(m_appkey_allocated, i) && m_appkeys[i].subnet_handle == subnet_handle)
        {
            /* If the AIDs for the old and the new application keys are equal, return both: */
            if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
                && m_appkeys[i].key_updated
                && IS_AIDS_EQUAL(m_appkeys[i].secmat.aid, aid)
                && IS_AIDS_EQUAL(m_appkeys[i].secmat_updated.aid, aid))
            {
                *pp_app_secmat = &m_appkeys[i].secmat;
                *pp_app_secmat_secondary = &m_appkeys[i].secmat_updated;
                return;
            }
            /* During key refresh, return the updated key if it matches the AID: */
            else if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
                     && m_appkeys[i].key_updated
                     && IS_AIDS_EQUAL(m_appkeys[i].secmat_updated.aid, aid))
            {
                *pp_app_secmat = &m_appkeys[i].secmat_updated;
                *pp_app_secmat_secondary = NULL;
                return;
            }
            else if (IS_AIDS_EQUAL(m_appkeys[i].secmat.aid, aid))
            {
                *pp_app_secmat = &m_appkeys[i].secmat;
                *pp_app_secmat_secondary = NULL;
                return;
            }
        }
    }
    *pp_app_secmat = NULL;
    *pp_app_secmat_secondary = NULL;
}

static uint32_t app_tx_secmat_get(dsm_handle_t app_handle, dsm_handle_t *p_subnet_handle, const nrf_mesh_application_secmat_t ** p_app)
{
    if (app_handle >= DSM_DEVKEY_HANDLE_START + DSM_DEVICE_MAX)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!bitfield_get(app_handle < DSM_DEVKEY_HANDLE_START ? m_appkey_allocated : m_devkey_allocated,
                      app_handle < DSM_DEVKEY_HANDLE_START ? app_handle : app_handle - DSM_DEVKEY_HANDLE_START))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (DSM_HANDLE_INVALID == *p_subnet_handle)
    {
        *p_subnet_handle = app_handle < DSM_DEVKEY_HANDLE_START ? m_appkeys[app_handle].subnet_handle :
                           m_devkeys[app_handle - DSM_DEVKEY_HANDLE_START].subnet_handle;
    }

    if (*p_subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, *p_subnet_handle))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (app_handle < DSM_DEVKEY_HANDLE_START)
    {/* Application key */
        /* Use updated application security credentials (if available) during key refresh phase 2: */
        if (m_subnets[*p_subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 && m_appkeys[app_handle].key_updated)
        {
            *p_app = &m_appkeys[app_handle].secmat_updated;
        }
        else
        {
            *p_app = &m_appkeys[app_handle].secmat;
        }
    }
    else
    {/* Device key */
        *p_app = &m_devkeys[app_handle - DSM_DEVKEY_HANDLE_START].secmat;
    }

    return NRF_SUCCESS;
}

static void subnet_set(mesh_key_index_t net_key_index, const uint8_t * p_key, dsm_handle_t handle)
{
    m_subnets[handle].beacon.info.p_tx_info = &m_subnets[handle].beacon.tx_info;
    if (net_key_index == PRIMARY_SUBNET_INDEX)
    {
        m_has_primary_subnet = true;
    }
    NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_network_secmat(p_key, &m_subnets[handle].secmat));
    NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_beacon_secmat(p_key, &m_subnets[handle].beacon.info.secmat));
#if MESH_FEATURE_GATT_PROXY_ENABLED
    NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_identitykey(p_key, m_subnets[handle].beacon.info.secmat.identity_key));
#endif

    memcpy(m_subnets[handle].root_key, p_key, NRF_MESH_KEY_SIZE);
    memset(m_subnets[handle].root_key_updated, 0, NRF_MESH_KEY_SIZE);

    m_subnets[handle].net_key_index = net_key_index;
    m_subnets[handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    bitfield_set(m_subnet_allocated, handle);
}

static void appkey_set(mesh_key_index_t app_key_index, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t handle)
{
    memcpy(m_appkeys[handle].secmat.key, p_key, NRF_MESH_KEY_SIZE);
    memset(m_appkeys[handle].secmat_updated.key, 0, NRF_MESH_KEY_SIZE);
    NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_aid(p_key, &m_appkeys[handle].secmat.aid));
    m_appkeys[handle].secmat.is_device_key = false;
    m_appkeys[handle].key_updated = false;
    m_appkeys[handle].app_key_index = app_key_index;
    m_appkeys[handle].subnet_handle = subnet_handle;
    bitfield_set(m_appkey_allocated, handle);
}

static void devkey_set(uint16_t key_owner, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t handle)
{
    uint32_t index = handle - DSM_DEVKEY_HANDLE_START;
    memcpy(m_devkeys[index].secmat.key, p_key, NRF_MESH_KEY_SIZE);
    m_devkeys[index].secmat.aid = 0;
    m_devkeys[index].secmat.is_device_key = true;
    m_devkeys[index].subnet_handle = subnet_handle;
    m_devkeys[index].key_owner = key_owner;
    bitfield_set(m_devkey_allocated, index);
}

static void nonvirtual_address_set(uint16_t raw_address, dsm_handle_t handle)
{
    m_addresses[handle].address = raw_address;
    m_addresses[handle].subscription_count = 0;
    m_addresses[handle].publish_count = 0;
    bitfield_set(m_addr_nonvirtual_allocated, handle);
}

static void virtual_address_set(const uint8_t * p_label_uuid, dsm_handle_t handle)
{
    uint32_t index = handle - DSM_VIRTUAL_HANDLE_START;
    memcpy(m_virtual_addresses[index].uuid, p_label_uuid, NRF_MESH_UUID_SIZE);
    NRF_MESH_ASSERT(nrf_mesh_keygen_virtual_address(p_label_uuid, &m_virtual_addresses[index].address) == NRF_SUCCESS);
    bitfield_set(m_addr_virtual_allocated, index);
}

static uint32_t address_delete_if_unused(dsm_handle_t address_handle)
{
    if (address_handle_nonvirtual_valid(address_handle))
    {
        if (m_addresses[address_handle].publish_count == 0 && m_addresses[address_handle].subscription_count == 0)
        {
            m_addresses[address_handle].address = NRF_MESH_ADDR_UNASSIGNED;
            dsm_entry_invalidate(MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD, address_handle, m_addr_nonvirtual_allocated);
        }

        return NRF_SUCCESS;
    }
    else if (address_handle_virtual_valid(address_handle))
    {
        uint16_t addr_virtual_index = address_handle - DSM_VIRTUAL_HANDLE_START;

        if (m_virtual_addresses[addr_virtual_index].publish_count == 0 &&
            m_virtual_addresses[addr_virtual_index].subscription_count == 0)
        {
            m_virtual_addresses[addr_virtual_index].address = NRF_MESH_ADDR_UNASSIGNED;
            dsm_entry_invalidate(MESH_OPT_DSM_VIRTUAL_ADDR_RECORD, addr_virtual_index, m_addr_virtual_allocated);
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

static uint32_t add_address(uint16_t raw_address, dsm_handle_t * p_address_handle, dsm_address_role_t role)
{
    if (p_address_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    nrf_mesh_address_type_t type;
    dsm_handle_t handle;

    bool address_found = address_exists(raw_address, &handle, &type);
    uint32_t status = NRF_SUCCESS;
    /* Check if the address type is valid. */
    if (
        (role == DSM_ADDRESS_ROLE_SUBSCRIBE && type != NRF_MESH_ADDRESS_TYPE_GROUP)
        ||
        (role == DSM_ADDRESS_ROLE_PUBLISH
            && type != NRF_MESH_ADDRESS_TYPE_GROUP && type != NRF_MESH_ADDRESS_TYPE_UNICAST)
       )
    {
        status = NRF_ERROR_INVALID_ADDR;
    }
    else if (!address_found)
    {
        if (handle == DSM_HANDLE_INVALID)
        {
            status = NRF_ERROR_NO_MEM;
        }
        else
        {
            nonvirtual_address_set(raw_address, handle);
            dsm_entry_store(MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD, handle, m_addr_nonvirtual_allocated);
        }
    }

    if (status == NRF_SUCCESS)
    {
        *p_address_handle = handle;
        if (role == DSM_ADDRESS_ROLE_SUBSCRIBE)
        {
#if MESH_FEATURE_LPN_ENABLED
            if (m_addresses[*p_address_handle].subscription_count == 0 && mesh_lpn_is_in_friendship())
            {
                NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_add(m_addresses[*p_address_handle].address) == NRF_SUCCESS);
            }
#endif
            m_addresses[*p_address_handle].subscription_count++;
        }
        else
        {
            m_addresses[*p_address_handle].publish_count++;
        }
    }

    return status;
}

static uint32_t add_address_virtual(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle, dsm_address_role_t role)
{
    uint16_t dest = DSM_HANDLE_INVALID;
    if (p_label_uuid == NULL || p_address_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }
    bool address_found = virtual_address_uuid_index_get(p_label_uuid, &dest);
    dsm_handle_t handle = dest + DSM_VIRTUAL_HANDLE_START;

    if (!address_found)
    {
        if (dest == DSM_HANDLE_INVALID)
        {
            return NRF_ERROR_NO_MEM;
        }

        virtual_address_set(p_label_uuid, handle);
        dsm_entry_store(MESH_OPT_DSM_VIRTUAL_ADDR_RECORD, dest, m_addr_virtual_allocated);
    }
    *p_address_handle = handle;
    if (role == DSM_ADDRESS_ROLE_SUBSCRIBE)
    {
#if MESH_FEATURE_LPN_ENABLED
        if (m_virtual_addresses[dest].subscription_count == 0 && mesh_lpn_is_in_friendship())
        {
            NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_add(m_virtual_addresses[dest].address) == NRF_SUCCESS);
        }
#endif
        m_virtual_addresses[dest].subscription_count++;
    }
    else
    {
        m_virtual_addresses[dest].publish_count++;
    }

    return NRF_SUCCESS;
}

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
static void friendship_clear(friendship_secmat_t *p_friendship)
{
    p_friendship->subnet_handle = DSM_HANDLE_INVALID;
    memset(&p_friendship->secmat, 0x00, sizeof(nrf_mesh_network_secmat_t));
    memset(&p_friendship->secmat_updated, 0x00, sizeof(nrf_mesh_network_secmat_t));
}

#if MESH_FEATURE_LPN_ENABLED
static void friendship_established_lpn_handle(void)
{
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        if (bitfield_get(m_addr_nonvirtual_allocated, i) && m_addresses[i].subscription_count > 0)
        {
            NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_add(m_addresses[i].address) == NRF_SUCCESS);
        }
    }

    for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX; ++i)
    {
        if (bitfield_get(m_addr_virtual_allocated, i) && m_virtual_addresses[i].subscription_count > 0)
        {
            NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_add(m_virtual_addresses[i].address) == NRF_SUCCESS);
        }
    }

    const heartbeat_subscription_state_t * p_hb_sub = heartbeat_subscription_get();
    if (nrf_mesh_address_type_get(p_hb_sub->dst) == NRF_MESH_ADDRESS_TYPE_GROUP)
    {
        NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_add(p_hb_sub->dst) == NRF_SUCCESS);
    }
}
#endif

static void friendship_established_handle(const nrf_mesh_evt_friendship_established_t * p_established)
{
#if MESH_FEATURE_LPN_ENABLED
    if (p_established->role == NRF_MESH_FRIENDSHIP_ROLE_LPN)
    {
        friendship_established_lpn_handle();
    }
#endif
}

static void friendship_termination_handle(const nrf_mesh_evt_friendship_terminated_t * p_terminated)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        if ((m_friendships[i].secmat_params.friend_address == p_terminated->friend_src)
            && (m_friendships[i].secmat_params.lpn_address == p_terminated->lpn_src))
        {
            friendship_clear(&m_friendships[i]);
        }
    }
}

#if MESH_FEATURE_LPN_ENABLED
static void lpn_friend_request_timeout_handle(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        if (m_friendships[i].secmat_params.lpn_address == m_local_unicast_addr.address_start)
        {
            friendship_clear(&m_friendships[i]);
            break;
        }
    }
}
#endif /* MESH_FEATURE_LPN_ENABLED */
#endif /* (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED) */

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_NET_BEACON_RECEIVED:
            net_beacon_rx_handle(p_evt->params.net_beacon.p_beacon_info,
                                p_evt->params.net_beacon.p_beacon_secmat,
                                p_evt->params.net_beacon.iv_index,
                                p_evt->params.net_beacon.flags.iv_update,
                                p_evt->params.net_beacon.flags.key_refresh);
            break;
#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            friendship_established_handle(&p_evt->params.friendship_established);
            break;
        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            friendship_termination_handle(&p_evt->params.friendship_terminated);
            break;
#endif

#if MESH_FEATURE_LPN_ENABLED
        case NRF_MESH_EVT_LPN_FRIEND_UPDATE:
            lpn_friend_update_rx_handle(p_evt->params.friend_update.p_secmat_net,
                                        p_evt->params.friend_update.iv_index,
                                        p_evt->params.friend_update.iv_update_active,
                                        p_evt->params.friend_update.key_refresh_in_phase2);
            break;
        case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
            lpn_friend_request_timeout_handle();
            break;
        case NRF_MESH_EVT_HB_SUBSCRIPTION_CHANGE:
            /* We need to add and remove heartbeat subscription addresses to/from the LPN
             * subscription list to ensure that the friend delivers the messages to us */
            if (mesh_lpn_is_in_friendship())
            {
                if (p_evt->params.hb_subscription_change.p_old != NULL &&
                    (nrf_mesh_address_type_get(p_evt->params.hb_subscription_change.p_old->dst) ==
                     NRF_MESH_ADDRESS_TYPE_GROUP))
                {
                    /* Shouldn't remove the heartbeat subscription if there's a model that subscribes to it as well. */
                    dsm_handle_t dummy_handle;
                    if (non_virtual_address_handle_get(p_evt->params.hb_subscription_change.p_old->dst,
                                                       &dummy_handle) != NRF_SUCCESS)
                    {
                        NRF_MESH_ASSERT_DEBUG(
                            mesh_lpn_subman_remove(p_evt->params.hb_subscription_change.p_old->dst) ==
                            NRF_SUCCESS);
                    }
                }

                if (p_evt->params.hb_subscription_change.p_new != NULL &&
                    (nrf_mesh_address_type_get(p_evt->params.hb_subscription_change.p_new->dst) ==
                     NRF_MESH_ADDRESS_TYPE_GROUP))
                {
                    /* Adding duplicate addresses isn't a problem, no need to check if a model subscribes to it. */
                    NRF_MESH_ASSERT_DEBUG(
                        mesh_lpn_subman_add(p_evt->params.hb_subscription_change.p_new->dst) ==
                        NRF_SUCCESS);
                }
            }
            break;
#endif

        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            if (p_evt->params.config_load_failure.id.file == MESH_OPT_DSM_FILE_ID)
            {
                m_status.is_load_failed = 1;
            }
            break;

        default:
            break;
    }
}
/******************************* PERSISTENT STORAGE MANAGEMENT *****************************************/
void dsm_legacy_pretreatment_do(mesh_config_entry_id_t * p_id, uint32_t entry_len)
{
    if (p_id->file == MESH_OPT_DSM_FILE_ID &&
        IS_IN_RANGE(p_id->record, MESH_OPT_DSM_SUBNETS_RECORD, MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1) &&
        entry_len == ALIGN_VAL(sizeof(dsm_entry_subnet_t) - NRF_MESH_KEY_SIZE, WORD_SIZE))
    {
        p_id->record += MESH_OPT_DSM_LEGACY_SUBNETS_RECORD - MESH_OPT_DSM_SUBNETS_RECORD;
    }
    else if (p_id->file == MESH_OPT_DSM_FILE_ID &&
        IS_IN_RANGE(p_id->record, MESH_OPT_DSM_APPKEYS_RECORD, MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
    {
        if (entry_len == ALIGN_VAL(sizeof(dsm_legacy_entry_appkey_t), WORD_SIZE))
        {
            p_id->record += MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD - MESH_OPT_DSM_APPKEYS_RECORD;
        }
        else if (entry_len == ALIGN_VAL(sizeof(dsm_legacy_entry_appkey_t) - NRF_MESH_KEY_SIZE, WORD_SIZE))
        {
            p_id->record += MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD - MESH_OPT_DSM_APPKEYS_RECORD;
        }
    }
}

static uint32_t dsm_metadata_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_DSM_METADATA_RECORD == id.record);

    dsm_entry_metainfo_t * p_metadata = (dsm_entry_metainfo_t *)p_entry;

    if (p_metadata->max_subnets == DSM_SUBNET_MAX &&
        p_metadata->max_devkeys == DSM_DEVICE_MAX &&
        p_metadata->max_appkeys == DSM_APP_MAX &&
        p_metadata->max_addrs_virtual == DSM_VIRTUAL_ADDR_MAX &&
        p_metadata->max_addrs_nonvirtual == DSM_NONVIRTUAL_ADDR_MAX)
    {
        m_status.is_metadata_stored = 1;
    }
    else
    {
        return NRF_ERROR_INVALID_DATA;
    }

    return NRF_SUCCESS;
}

static void dsm_metadata_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_DSM_METADATA_RECORD == id.record);

    dsm_entry_metainfo_t * p_metadata = (dsm_entry_metainfo_t *)p_entry;
    p_metadata->max_subnets = DSM_SUBNET_MAX;
    p_metadata->max_devkeys = DSM_DEVICE_MAX;
    p_metadata->max_appkeys = DSM_APP_MAX;
    p_metadata->max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX;
    p_metadata->max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX;
}

static uint32_t dsm_unicast_addr_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_DSM_UNICAST_ADDR_RECORD == id.record);

    if (bitfield_get(m_addr_unicast_allocated, 0))
    {
        return NRF_SUCCESS;
    }

    dsm_entry_addr_unicast_t * p_src = (dsm_entry_addr_unicast_t *)p_entry;
    if (nrf_mesh_address_type_get(p_src->addr.address_start) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    memcpy(&m_local_unicast_addr, &p_src->addr, sizeof(m_local_unicast_addr));
    bitfield_set(m_addr_unicast_allocated, 0);

    return NRF_SUCCESS;
}

static void dsm_unicast_addr_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(MESH_OPT_DSM_UNICAST_ADDR_RECORD == id.record);
    NRF_MESH_ASSERT(bitfield_get(m_addr_unicast_allocated, 0));

    dsm_entry_addr_unicast_t * p_dst = (dsm_entry_addr_unicast_t *)p_entry;
    memcpy(&p_dst->addr, &m_local_unicast_addr, sizeof(dsm_local_unicast_address_t));
}

static uint32_t dsm_nonvirtual_addr_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD,
                     MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t idx = id.record - MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD;

    if (bitfield_get(m_addr_nonvirtual_allocated, idx))
    {
        return NRF_SUCCESS;
    }

    dsm_entry_addr_nonvirtual_t * p_src = (dsm_entry_addr_nonvirtual_t *)p_entry;
    nonvirtual_address_set(p_src->addr, idx);

    return NRF_SUCCESS;
}

static void dsm_nonvirtual_addr_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD,
                                      MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD + DSM_NONVIRTUAL_ADDR_MAX - 1));

    uint16_t idx = id.record - MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD;

    NRF_MESH_ASSERT(bitfield_get(m_addr_nonvirtual_allocated, idx));
    dsm_entry_addr_nonvirtual_t * p_dst = (dsm_entry_addr_nonvirtual_t *)p_entry;
    p_dst->addr = m_addresses[idx].address;
}

static uint32_t dsm_virtual_addr_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD,
                     MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_VIRTUAL_ADDR_MAX - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t idx = id.record - MESH_OPT_DSM_VIRTUAL_ADDR_RECORD;

    if (bitfield_get(m_addr_virtual_allocated, idx))
    {
        return NRF_SUCCESS;
    }

    dsm_entry_addr_virtual_t * p_src = (dsm_entry_addr_virtual_t *)p_entry;
    virtual_address_set(p_src->uuid, DSM_VIRTUAL_HANDLE_START + idx);

    return NRF_SUCCESS;
}

static void dsm_virtual_addr_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD,
                         MESH_OPT_DSM_VIRTUAL_ADDR_RECORD + DSM_VIRTUAL_ADDR_MAX - 1));

    uint16_t idx = id.record - MESH_OPT_DSM_VIRTUAL_ADDR_RECORD;

    NRF_MESH_ASSERT(bitfield_get(m_addr_virtual_allocated, idx));
    dsm_entry_addr_virtual_t * p_dst = (dsm_entry_addr_virtual_t *)p_entry;
    memcpy(p_dst->uuid, m_virtual_addresses[idx].uuid, NRF_MESH_UUID_SIZE);
}

static uint32_t dsm_subnet_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                     MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    dsm_entry_subnet_t * p_src = (dsm_entry_subnet_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_SUBNETS_RECORD;

    if (bitfield_get(m_subnet_allocated, idx))
    {
        return NRF_SUCCESS;
    }

    subnet_set(p_src->key_index, p_src->key, idx);
    m_subnets[idx].key_refresh_phase = p_src->key_refresh_phase;
    if (m_subnets[idx].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        memcpy(m_subnets[idx].root_key_updated, p_src->key_updated, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_network_secmat(p_src->key_updated, &m_subnets[idx].secmat_updated) == NRF_SUCCESS);
    }

    return NRF_SUCCESS;
}

static void dsm_subnet_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_DSM_SUBNETS_RECORD,
                         MESH_OPT_DSM_SUBNETS_RECORD + DSM_SUBNET_MAX - 1));

    dsm_entry_subnet_t * p_dst = (dsm_entry_subnet_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_SUBNETS_RECORD;
    memset(p_entry, 0, sizeof(dsm_entry_subnet_t));

    NRF_MESH_ASSERT(bitfield_get(m_subnet_allocated, idx));
    p_dst->key_index = m_subnets[idx].net_key_index;
    p_dst->key_refresh_phase = m_subnets[idx].key_refresh_phase;
    memcpy(p_dst->key, m_subnets[idx].root_key, NRF_MESH_KEY_SIZE);

    if (m_subnets[idx].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        memcpy(p_dst->key_updated, m_subnets[idx].root_key_updated, NRF_MESH_KEY_SIZE);
    }
}

static uint32_t dsm_appkey_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                     MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    dsm_entry_appkey_t * p_src = (dsm_entry_appkey_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_APPKEYS_RECORD;

    if (bitfield_get(m_appkey_allocated, idx))
    {
        return NRF_SUCCESS;
    }

    appkey_set(p_src->key_index, p_src->subnet_handle, p_src->key, idx);
    m_appkeys[idx].key_updated = p_src->is_key_updated;

    if (p_src->is_key_updated)
    {
        memcpy(m_appkeys[idx].secmat_updated.key, p_src->key_updated, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_aid(p_src->key_updated, &m_appkeys[idx].secmat_updated.aid) == NRF_SUCCESS);
        m_appkeys[idx].secmat_updated.is_device_key = m_appkeys[idx].secmat.is_device_key;
    }

    return NRF_SUCCESS;
}

static void dsm_appkey_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_DSM_APPKEYS_RECORD,
                         MESH_OPT_DSM_APPKEYS_RECORD + DSM_APP_MAX - 1));

    dsm_entry_appkey_t * p_dst = (dsm_entry_appkey_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_APPKEYS_RECORD;
    memset(p_entry, 0, sizeof(dsm_entry_appkey_t));

    NRF_MESH_ASSERT(bitfield_get(m_appkey_allocated, idx));
    memcpy(p_dst->key, m_appkeys[idx].secmat.key, NRF_MESH_KEY_SIZE);
    p_dst->key_index = m_appkeys[idx].app_key_index;
    p_dst->subnet_handle = m_appkeys[idx].subnet_handle;
    p_dst->is_key_updated = m_appkeys[idx].key_updated;

    if (p_dst->is_key_updated)
    {
        memcpy(p_dst->key_updated, m_appkeys[idx].secmat_updated.key, NRF_MESH_KEY_SIZE);
    }
}

static uint32_t dsm_devkey_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, MESH_OPT_DSM_DEVKEYS_RECORD,
                     MESH_OPT_DSM_DEVKEYS_RECORD + DSM_DEVICE_MAX - 1))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    dsm_entry_devkey_t * p_dst = (dsm_entry_devkey_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_DEVKEYS_RECORD;

    if (bitfield_get(m_devkey_allocated, idx))
    {
        return NRF_SUCCESS;
    }

    devkey_set(p_dst->key_owner, p_dst->subnet_handle, p_dst->key, DSM_DEVKEY_HANDLE_START + idx);

    return NRF_SUCCESS;
}

static void dsm_devkey_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, MESH_OPT_DSM_DEVKEYS_RECORD,
                         MESH_OPT_DSM_DEVKEYS_RECORD + DSM_DEVICE_MAX - 1));

    dsm_entry_devkey_t * p_dst = (dsm_entry_devkey_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_DEVKEYS_RECORD;
    NRF_MESH_ASSERT(bitfield_get(m_devkey_allocated, idx));
    memcpy(p_dst->key, m_devkeys[idx].secmat.key, NRF_MESH_KEY_SIZE);
    p_dst->key_owner = m_devkeys[idx].key_owner;
    p_dst->subnet_handle = m_devkeys[idx].subnet_handle;
}

static uint32_t dsm_legacy_subnet_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    dsm_entry_subnet_t * p_src = (dsm_entry_subnet_t *)p_entry;
    uint16_t idx = id.record - MESH_OPT_DSM_LEGACY_SUBNETS_RECORD;
    subnet_set(p_src->key_index, p_src->key, idx);
    m_subnets[idx].key_refresh_phase = p_src->key_refresh_phase;
    m_status.is_legacy_found = 1;

    return NRF_SUCCESS;
}

static uint32_t dsm_legacy_appkey_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    uint16_t idx;
    bool is_key_updated;

    if (IS_IN_RANGE(id.record, MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD,
                    MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD + DSM_APP_MAX - 1))
    {
        idx = id.record - MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD;
        is_key_updated = false;
    }
    else if (IS_IN_RANGE(id.record, MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD,
                         MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD + DSM_APP_MAX - 1))
    {
        idx = id.record - MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD;
        is_key_updated = true;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }

    m_status.is_legacy_found = 1;
    dsm_legacy_entry_appkey_t * p_src = (dsm_legacy_entry_appkey_t *)p_entry;
    appkey_set(p_src->key_index, p_src->subnet_handle, p_src->key, idx);
    m_appkeys[idx].key_updated = is_key_updated;

    if (is_key_updated)
    {
        memcpy(m_appkeys[idx].secmat_updated.key, p_src->key_updated, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_aid(p_src->key_updated, &m_appkeys[idx].secmat_updated.aid) == NRF_SUCCESS);
        m_appkeys[idx].secmat_updated.is_device_key = m_appkeys[idx].secmat.is_device_key;
    }

    return NRF_SUCCESS;
}

MESH_CONFIG_ENTRY(dsm_metadata,
                  MESH_OPT_DSM_METADATA_EID,
                  1,
                  sizeof(dsm_entry_metainfo_t),
                  dsm_metadata_setter,
                  dsm_metadata_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_unicast_addr,
                  MESH_OPT_DSM_UNICAST_ADDR_EID,
                  1,
                  sizeof(dsm_entry_addr_unicast_t),
                  dsm_unicast_addr_setter,
                  dsm_unicast_addr_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_nonvirtual_addr,
                  MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD_EID,
                  DSM_NONVIRTUAL_ADDR_MAX,
                  sizeof(dsm_entry_addr_nonvirtual_t),
                  dsm_nonvirtual_addr_setter,
                  dsm_nonvirtual_addr_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_virtual_addr,
                  MESH_OPT_DSM_VIRTUAL_ADDR_RECORD_EID,
                  DSM_VIRTUAL_ADDR_MAX,
                  sizeof(dsm_entry_addr_virtual_t),
                  dsm_virtual_addr_setter,
                  dsm_virtual_addr_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_subnet,
                  MESH_OPT_DSM_SUBNETS_RECORD_EID,
                  DSM_SUBNET_MAX,
                  sizeof(dsm_entry_subnet_t),
                  dsm_subnet_setter,
                  dsm_subnet_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_appkey,
                  MESH_OPT_DSM_APPKEYS_RECORD_EID,
                  DSM_APP_MAX,
                  sizeof(dsm_entry_appkey_t),
                  dsm_appkey_setter,
                  dsm_appkey_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_devkey,
                  MESH_OPT_DSM_DEVKEYS_RECORD_EID,
                  DSM_DEVICE_MAX,
                  sizeof(dsm_entry_devkey_t),
                  dsm_devkey_setter,
                  dsm_devkey_getter,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_legacy_subnet,
                  MESH_OPT_DSM_LEGACY_SUBNETS_RECORD_EID,
                  DSM_SUBNET_MAX,
                  ALIGN_VAL(sizeof(dsm_entry_subnet_t) - NRF_MESH_KEY_SIZE, WORD_SIZE),
                  dsm_legacy_subnet_setter,
                  NULL,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_reduced_legacy_appkey,
                  MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD_EID,
                  DSM_APP_MAX,
                  ALIGN_VAL(sizeof(dsm_legacy_entry_appkey_t) - NRF_MESH_KEY_SIZE, WORD_SIZE),
                  dsm_legacy_appkey_setter,
                  NULL,
                  NULL,
                  NULL);

MESH_CONFIG_ENTRY(dsm_full_legacy_appkey,
                  MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD_EID,
                  DSM_APP_MAX,
                  ALIGN_VAL(sizeof(dsm_legacy_entry_appkey_t), WORD_SIZE),
                  dsm_legacy_appkey_setter,
                  NULL,
                  NULL,
                  NULL);

MESH_CONFIG_FILE(m_dsm_file, MESH_OPT_DSM_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

static void dsm_entry_store(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property)
{
    mesh_config_entry_id_t id;
    /* Type of entry does not matter because real copying happens in API functions.
     * Setters will avoid one more extra copying and ignore dummy parameter. */
    dsm_entry_addr_nonvirtual_t dummy = {0};

    NRF_MESH_ASSERT(bitfield_get(p_property, handle));

    id.file = MESH_OPT_DSM_FILE_ID;
    id.record = record_id + handle;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &dummy));
}

static void dsm_entry_invalidate(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property)
{
    mesh_config_entry_id_t id;

    NRF_MESH_ASSERT(bitfield_get(p_property, handle));

    id.file = MESH_OPT_DSM_FILE_ID;
    id.record = record_id + handle;
    NRF_MESH_ERROR_CHECK(mesh_config_entry_delete(id));
    bitfield_clear(p_property, handle);
}

static void metadata_store(void)
{
    mesh_config_entry_id_t entry_id = MESH_OPT_DSM_METADATA_EID;
    dsm_entry_metainfo_t metadata =
    {
        .max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX,
        .max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX,
        .max_subnets = DSM_SUBNET_MAX,
        .max_appkeys = DSM_APP_MAX,
        .max_devkeys = DSM_DEVICE_MAX
    };

    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(entry_id, &metadata));
}

static void dsm_mesh_config_clear(void)
{
    m_status.is_metadata_stored = 0;
    m_status.is_load_failed = 0;

    mesh_config_file_clear(MESH_OPT_DSM_FILE_ID);
}

static void legacy_remove(void)
{
    if (!!m_status.is_legacy_found)
    {
        m_status.is_legacy_found = 0;
        mesh_config_entry_id_t id = MESH_OPT_DSM_SUBNETS_RECORD_EID;

        for (uint32_t i = 0; i < DSM_SUBNET_MAX; i++)
        {
            if (!bitfield_get(m_subnet_allocated, i))
            {
                continue;
            }

            id.record = MESH_OPT_DSM_SUBNETS_RECORD + i;
            (void)mesh_config_entry_delete(id);
            dsm_entry_store(MESH_OPT_DSM_SUBNETS_RECORD, i, m_subnet_allocated);
        }

        id = MESH_OPT_DSM_APPKEYS_RECORD_EID;

        for (uint32_t i = 0; i < DSM_APP_MAX; i++)
        {
            if (!bitfield_get(m_appkey_allocated, i))
            {
                continue;
            }

            id.record = MESH_OPT_DSM_APPKEYS_RECORD + i;
            (void)mesh_config_entry_delete(id);
            dsm_entry_store(MESH_OPT_DSM_APPKEYS_RECORD, i, m_appkey_allocated);
        }
    }
}

uint32_t dsm_load_config_apply(void)
{
    if (!!m_status.is_load_failed)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if (!m_status.is_metadata_stored)
    {
        metadata_store();
#if PERSISTENT_STORAGE
        return NRF_ERROR_NOT_FOUND;
#else
        return NRF_SUCCESS;
#endif
    }

    legacy_remove();

    return NRF_SUCCESS;
}

void dsm_clear(void)
{
    /* Clear the nonvirtual address storage references */
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        m_addresses[i].subscription_count = 0;
        m_addresses[i].publish_count = 0;
    }
    /* Clear the virtual address storage references */
    for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX; ++i)
    {
        m_virtual_addresses[i].subscription_count = 0;
        m_virtual_addresses[i].publish_count = 0;
    }

    bitfield_clear_all(m_addr_unicast_allocated, BITFIELD_BLOCK_COUNT(1));
    bitfield_clear_all(m_addr_nonvirtual_allocated, BITFIELD_BLOCK_COUNT(DSM_NONVIRTUAL_ADDR_MAX));
    bitfield_clear_all(m_addr_virtual_allocated, BITFIELD_BLOCK_COUNT(DSM_VIRTUAL_ADDR_MAX));
    bitfield_clear_all(m_subnet_allocated, BITFIELD_BLOCK_COUNT(DSM_SUBNET_MAX));
    bitfield_clear_all(m_appkey_allocated, BITFIELD_BLOCK_COUNT(DSM_APP_MAX));
    bitfield_clear_all(m_devkey_allocated, BITFIELD_BLOCK_COUNT(DSM_DEVICE_MAX));

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        friendship_clear(&m_friendships[i]);
    }
#endif

    m_local_unicast_addr.address_start = NRF_MESH_ADDR_UNASSIGNED;
    m_local_unicast_addr.count = 0;
    m_has_primary_subnet = false;

    dsm_mesh_config_clear();
}


/*****************************************************************************
* Interface functions
*****************************************************************************/

void dsm_init(void)
{
    m_mesh_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

    m_status.is_load_failed = 0;
    m_status.is_metadata_stored = 0;

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        friendship_clear(&m_friendships[i]);
    }
#endif
}

uint32_t dsm_local_unicast_addresses_set(const dsm_local_unicast_address_t * p_address)
{
    NRF_MESH_ASSERT(p_address != NULL);
    if (bitfield_get(m_addr_unicast_allocated, 0))
    {
        return NRF_ERROR_FORBIDDEN;
    }
    else if (p_address->address_start + p_address->count - 1 < p_address->address_start ||
             NRF_MESH_ADDRESS_TYPE_UNICAST != nrf_mesh_address_type_get(p_address->address_start) ||
             NRF_MESH_ADDRESS_TYPE_UNICAST != nrf_mesh_address_type_get(p_address->address_start + p_address->count - 1))
    {
        return NRF_ERROR_INVALID_DATA;
    }
    else
    {
        bitfield_set(m_addr_unicast_allocated, 0);
        memcpy(&m_local_unicast_addr, p_address, sizeof(dsm_local_unicast_address_t));
        dsm_entry_store(MESH_OPT_DSM_UNICAST_ADDR_RECORD, 0, m_addr_unicast_allocated);
    }

    return NRF_SUCCESS;
}

void dsm_local_unicast_addresses_get(dsm_local_unicast_address_t * p_address)
{
    NRF_MESH_ASSERT(p_address != NULL);
    memcpy(p_address, &m_local_unicast_addr, sizeof(dsm_local_unicast_address_t));
}

uint32_t dsm_address_publish_add(uint16_t raw_address, dsm_handle_t * p_address_handle)
{
    return add_address(raw_address, p_address_handle, DSM_ADDRESS_ROLE_PUBLISH);
}

uint32_t dsm_address_publish_add_handle(dsm_handle_t address_handle)
{
    if (address_handle_valid(address_handle))
    {
        if (address_handle >= DSM_NONVIRTUAL_ADDR_MAX)
        {
            m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].publish_count++;
        }
        else
        {
            m_addresses[address_handle].publish_count++;
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t dsm_address_publish_virtual_add(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle)
{
    return add_address_virtual(p_label_uuid, p_address_handle, DSM_ADDRESS_ROLE_PUBLISH);
}

uint32_t dsm_address_publish_remove(dsm_handle_t address_handle)
{
    if (address_handle_nonvirtual_valid(address_handle))
    {
        m_addresses[address_handle].publish_count--;
        return address_delete_if_unused(address_handle);
    }
    else if (address_handle_virtual_valid(address_handle))
    {
        m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].publish_count--;
        return address_delete_if_unused(address_handle);
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t dsm_address_subscription_add(uint16_t raw_address, dsm_handle_t * p_address_handle)
{
    return add_address(raw_address, p_address_handle, DSM_ADDRESS_ROLE_SUBSCRIBE);
}

uint32_t dsm_address_subscription_virtual_add(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle)
{
    return add_address_virtual(p_label_uuid, p_address_handle, DSM_ADDRESS_ROLE_SUBSCRIBE);
}

uint32_t dsm_address_subscription_add_handle(dsm_handle_t address_handle)
{
    if (address_handle_valid(address_handle))
    {
        if (address_handle >= DSM_NONVIRTUAL_ADDR_MAX)
        {
            m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].subscription_count++;
        }
        else
        {
            m_addresses[address_handle].subscription_count++;
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

bool dsm_address_subscription_get(dsm_handle_t address_handle)
{
    if (address_handle_valid(address_handle))
    {
        if (address_handle >= DSM_NONVIRTUAL_ADDR_MAX)
        {
            return m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].subscription_count > 0;
        }
        else
        {
            return m_addresses[address_handle].subscription_count > 0;
        }
    }
    else
    {
        return false;
    }
}

bool dsm_address_is_rx(const nrf_mesh_address_t * p_addr)
{
    return nrf_mesh_is_address_rx(p_addr);
}

uint32_t dsm_address_subscription_count_get(dsm_handle_t address_handle, uint16_t * p_count)
{
    if (p_count == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (address_handle_valid(address_handle))
    {
        nrf_mesh_address_t addr;
        uint32_t status = dsm_address_get(address_handle, &addr);
        if (status == NRF_SUCCESS)
        {
            if (addr.type == NRF_MESH_ADDRESS_TYPE_GROUP)
            {
                *p_count = m_addresses[address_handle].subscription_count;
            }
            else if (addr.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
            {
                *p_count = m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].subscription_count;
            }
            else
            {
                return NRF_ERROR_INVALID_ADDR;
            }
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t dsm_address_subscription_remove(dsm_handle_t address_handle)
{
    if (address_handle_valid(address_handle))
    {
        nrf_mesh_address_t addr;
        uint32_t status = dsm_address_get(address_handle, &addr);
        if (status == NRF_SUCCESS)
        {
            if (addr.type == NRF_MESH_ADDRESS_TYPE_GROUP)
            {
                if (m_addresses[address_handle].subscription_count == 0)
                {
                    return NRF_ERROR_NOT_FOUND;
                }
                else
                {
                    --m_addresses[address_handle].subscription_count;
#if MESH_FEATURE_LPN_ENABLED
                    if (m_addresses[address_handle].subscription_count == 0 && mesh_lpn_is_in_friendship())
                    {
                        NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_remove(m_addresses[address_handle].address)  == NRF_SUCCESS);
                    }
#endif
                    return address_delete_if_unused(address_handle);
                }
            }
            else if (addr.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
            {
                uint32_t virtual_index = address_handle - DSM_VIRTUAL_HANDLE_START;
                if (m_virtual_addresses[virtual_index].subscription_count == 0)
                {
                    return NRF_ERROR_NOT_FOUND;
                }
                else
                {
                    --m_virtual_addresses[virtual_index].subscription_count;
#if MESH_FEATURE_LPN_ENABLED
                    if (m_virtual_addresses[virtual_index].subscription_count == 0 && mesh_lpn_is_in_friendship())
                    {
                        NRF_MESH_ASSERT_DEBUG(mesh_lpn_subman_remove(m_virtual_addresses[virtual_index].address)  == NRF_SUCCESS);
                    }
#endif
                    return address_delete_if_unused(address_handle);
                }
            }
            else
            {
                return NRF_ERROR_FORBIDDEN;
            }
        }

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t dsm_address_get(dsm_handle_t address_handle, nrf_mesh_address_t * p_address)
{
    uint32_t status = NRF_ERROR_NOT_FOUND;

    if (p_address == NULL)
    {
        status = NRF_ERROR_NULL;
    }

    else if (address_handle_nonvirtual_valid(address_handle))
    {
        p_address->value = m_addresses[address_handle].address;
        p_address->type = nrf_mesh_address_type_get(m_addresses[address_handle].address);
        p_address->p_virtual_uuid = NULL;
        status = NRF_SUCCESS;
    }
    else if (address_handle_virtual_valid(address_handle))
    {
        p_address->value = m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].address;
        p_address->p_virtual_uuid = &m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].uuid[0];
        p_address->type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
        status = NRF_SUCCESS;
    }

    return status;
}

uint32_t dsm_address_get_all(dsm_handle_t * p_address_handle_list, uint32_t * p_count)
{
    if (p_address_handle_list == NULL ||
        p_count == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t count = 0;
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        if (bitfield_get(m_addr_nonvirtual_allocated, i))
        {
            if (count == *p_count)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
            p_address_handle_list[count++] = i;
        }
    }
    for (uint32_t i = 0; i < DSM_VIRTUAL_ADDR_MAX; ++i)
    {
        if (bitfield_get(m_addr_virtual_allocated, i))
        {
            if (count == *p_count)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }
            p_address_handle_list[count++] = DSM_VIRTUAL_HANDLE_START + i;
        }
    }
    *p_count = count;
    return NRF_SUCCESS;
}

uint32_t dsm_address_handle_get(const nrf_mesh_address_t * p_address, dsm_handle_t * p_address_handle)
{
    if (p_address == NULL || p_address_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t status = NRF_ERROR_NOT_FOUND;
    nrf_mesh_address_type_t type = nrf_mesh_address_type_get(p_address->value);
    switch (type)
    {
        case NRF_MESH_ADDRESS_TYPE_UNICAST:
        case NRF_MESH_ADDRESS_TYPE_GROUP:
            if (non_virtual_address_handle_get(p_address->value, p_address_handle))
            {
                status = NRF_SUCCESS;
            }
            break;
        case NRF_MESH_ADDRESS_TYPE_VIRTUAL:
        {
            dsm_handle_t virtual_addr_index = DSM_HANDLE_INVALID;
            if (NULL == p_address->p_virtual_uuid)
            {
                status = NRF_ERROR_NULL;
            }
            else if (virtual_address_uuid_index_get(p_address->p_virtual_uuid, &virtual_addr_index))
            {
                status = NRF_SUCCESS;
                *p_address_handle = virtual_addr_index + DSM_VIRTUAL_HANDLE_START;
            }
            break;
        }
        default:
            status = NRF_ERROR_INVALID_ADDR;
            break;
    }
    return status;
}

dsm_handle_t dsm_net_key_index_to_subnet_handle(mesh_key_index_t net_key_index)
{
    dsm_handle_t handle;
    if (net_key_handle_get(net_key_index, &handle))
    {
        return handle;
    }
    else
    {
        return DSM_HANDLE_INVALID;
    }
}

dsm_handle_t dsm_subnet_handle_get(const nrf_mesh_network_secmat_t * p_secmat)
{
    if (p_secmat == NULL)
    {
        return DSM_HANDLE_INVALID;
    }

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    uint32_t i = friendship_index_by_secmat_get(p_secmat);
    if (i < ARRAY_SIZE(m_friendships))
    {
        return m_friendships[i].subnet_handle;
    }
#endif

    return get_subnet_handle(p_secmat);
}

uint32_t dsm_subnet_handle_to_netkey_index(dsm_handle_t subnet_handle, mesh_key_index_t * p_netkey_index)
{
    if (p_netkey_index == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (subnet_handle < DSM_SUBNET_MAX && bitfield_get(m_subnet_allocated, subnet_handle))
    {
        *p_netkey_index = m_subnets[subnet_handle].net_key_index;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

dsm_handle_t dsm_appkey_index_to_appkey_handle(mesh_key_index_t appkey_index)
{
    dsm_handle_t handle;
    if (app_key_handle_get(appkey_index, &handle))
    {
        return handle;
    }
    else
    {
        return DSM_HANDLE_INVALID;
    }
}

dsm_handle_t dsm_appkey_handle_get(const nrf_mesh_application_secmat_t * p_secmat)
{
    if (p_secmat == NULL)
    {
        return DSM_HANDLE_INVALID;
    }
    return get_app_handle(p_secmat);
}

uint32_t dsm_appkey_handle_to_appkey_index(dsm_handle_t appkey_handle, mesh_key_index_t * p_index)
{
    if (p_index == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (appkey_handle < DSM_APP_MAX && bitfield_get(m_appkey_allocated, appkey_handle))
    {
        *p_index = m_appkeys[appkey_handle].app_key_index;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t dsm_appkey_handle_to_subnet_handle(dsm_handle_t appkey_handle, dsm_handle_t *p_netkey_handle)
{
    if (appkey_handle >= DSM_DEVKEY_HANDLE_START + DSM_DEVICE_MAX)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (NULL == p_netkey_handle)
    {
        return NRF_ERROR_NULL;
    }

    if (appkey_handle < DSM_DEVKEY_HANDLE_START)
    {
        /* Application key */
        if (!bitfield_get(m_appkey_allocated, appkey_handle))
        {
            return NRF_ERROR_NOT_FOUND;
        }
        NRF_MESH_ASSERT(m_appkeys[appkey_handle].subnet_handle < DSM_SUBNET_MAX);
        NRF_MESH_ASSERT(bitfield_get(m_subnet_allocated, m_appkeys[appkey_handle].subnet_handle));

        *p_netkey_handle = m_appkeys[appkey_handle].subnet_handle;
    }
    else
    {
        /* Device key */
        if (!bitfield_get(m_devkey_allocated, appkey_handle - DSM_DEVKEY_HANDLE_START))
        {
            return NRF_ERROR_NOT_FOUND;
        }
        const devkey_t * p_dev = &m_devkeys[appkey_handle - DSM_DEVKEY_HANDLE_START];
        NRF_MESH_ASSERT(p_dev->subnet_handle < DSM_SUBNET_MAX);
        NRF_MESH_ASSERT(bitfield_get(m_subnet_allocated, p_dev->subnet_handle));

        *p_netkey_handle = p_dev->subnet_handle;
    }

    return NRF_SUCCESS;
}

uint32_t dsm_subnet_add(mesh_key_index_t net_key_index, const uint8_t * p_key, dsm_handle_t * p_subnet_handle)
{
    if (p_key == NULL || p_subnet_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (net_key_index > DSM_KEY_INDEX_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (net_key_handle_get(net_key_index, p_subnet_handle))
    {
        if (0 == memcmp(m_subnets[*p_subnet_handle].root_key, p_key, NRF_MESH_KEY_SIZE))
        {
            return NRF_ERROR_INTERNAL;
        }
        return NRF_ERROR_FORBIDDEN;
    }
    else if (*p_subnet_handle == DSM_HANDLE_INVALID)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        subnet_set(net_key_index, p_key, *p_subnet_handle);
        dsm_entry_store(MESH_OPT_DSM_SUBNETS_RECORD, *p_subnet_handle, m_subnet_allocated);
        nrf_mesh_subnet_added(net_key_index, m_subnets[*p_subnet_handle].beacon.info.secmat.net_id);

        __LOG_XB(LOG_SRC_DSM, LOG_LEVEL_DBG3, "Netkey added", p_key, NRF_MESH_KEY_SIZE);
        return NRF_SUCCESS;
    }
}

uint32_t dsm_subnet_kr_phase_get(dsm_handle_t subnet_handle, nrf_mesh_key_refresh_phase_t * p_phase)
{
    if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *p_phase = m_subnets[subnet_handle].key_refresh_phase;
        return NRF_SUCCESS;
    }
}

uint32_t dsm_subnet_update(dsm_handle_t subnet_handle, const uint8_t * p_key)
{
    if (NULL == p_key)
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        if (!(m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_1 &&
            memcmp(m_subnets[subnet_handle].root_key_updated, p_key, NRF_MESH_KEY_SIZE) == 0))
        {
            /* Network keys can only be updated if no key refresh is in progress for the new key
             * or Phase 1 for the same key. */
            return NRF_ERROR_INVALID_STATE;
        }
    }
    else
    {
        memcpy(m_subnets[subnet_handle].root_key_updated, p_key, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_network_secmat(p_key, &m_subnets[subnet_handle].secmat_updated) == NRF_SUCCESS);
        NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_beacon_secmat(m_subnets[subnet_handle].root_key_updated, &m_subnets[subnet_handle].beacon.info.secmat_updated));

#if MESH_FEATURE_GATT_PROXY_ENABLED
        NRF_MESH_ASSERT(nrf_mesh_keygen_identitykey(
                            p_key, m_subnets[subnet_handle].beacon.info.secmat_updated.identity_key) ==
                        NRF_SUCCESS);
#endif

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
        for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
        {
            if (m_friendships[i].subnet_handle == subnet_handle)
            {
                uint32_t error = nrf_mesh_keygen_friendship_secmat(p_key,
                                                                   &m_friendships[i].secmat_params,
                                                                   &m_friendships[i].secmat_updated);
                NRF_MESH_ERROR_CHECK(error);
            }
        }
#endif

        m_subnets[subnet_handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_1;
        net_state_key_refresh_phase_changed(m_subnets[subnet_handle].net_key_index,
                                            m_subnets[subnet_handle].beacon.info.secmat_updated.net_id,
                                            NRF_MESH_KEY_REFRESH_PHASE_1);
        dsm_entry_store(MESH_OPT_DSM_SUBNETS_RECORD, subnet_handle, m_subnet_allocated);
    }
    return NRF_SUCCESS;
}

uint32_t dsm_subnet_update_swap_keys(dsm_handle_t subnet_handle)
{
    if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_1)
    {
        /* Network keys can only be swapped in key refresh phase 1, */
        /* which will move the procedure to key refresh phase 2.    */
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        m_subnets[subnet_handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_2;
        net_state_key_refresh_phase_changed(m_subnets[subnet_handle].net_key_index,
                                            m_subnets[subnet_handle].beacon.info.secmat_updated.net_id,
                                            NRF_MESH_KEY_REFRESH_PHASE_2);
        dsm_entry_store(MESH_OPT_DSM_SUBNETS_RECORD, subnet_handle, m_subnet_allocated);
    }

    return NRF_SUCCESS;
}

uint32_t dsm_subnet_update_commit(dsm_handle_t subnet_handle)
{
    if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_subnets[subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        memcpy(m_subnets[subnet_handle].root_key, m_subnets[subnet_handle].root_key_updated, NRF_MESH_KEY_SIZE);
        memcpy(&m_subnets[subnet_handle].secmat, &m_subnets[subnet_handle].secmat_updated, sizeof(m_subnets[subnet_handle].secmat));
        memcpy(&m_subnets[subnet_handle].beacon.info.secmat, &m_subnets[subnet_handle].beacon.info.secmat_updated,
                sizeof(m_subnets[subnet_handle].beacon.info.secmat));

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
        for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
        {
            if (m_friendships[i].subnet_handle == subnet_handle)
            {
                memcpy(&m_friendships[i].secmat, &m_friendships[i].secmat_updated, sizeof(m_friendships[i].secmat));
            }
        }
#endif

        m_subnets[subnet_handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
        net_state_key_refresh_phase_changed(m_subnets[subnet_handle].net_key_index,
                                            m_subnets[subnet_handle].beacon.info.secmat.net_id,
                                            NRF_MESH_KEY_REFRESH_PHASE_0);

        /* Commit changes to application keys: */
        for (uint32_t i = 0; i < DSM_APP_MAX; i++)
        {
            if (bitfield_get(m_appkey_allocated, i)
                    && m_appkeys[i].subnet_handle == subnet_handle
                    && m_appkeys[i].key_updated)
            {
                memcpy(&m_appkeys[i].secmat, &m_appkeys[i].secmat_updated, sizeof(nrf_mesh_application_secmat_t));
                m_appkeys[i].key_updated = false;

                dsm_entry_store(MESH_OPT_DSM_APPKEYS_RECORD, i, m_appkey_allocated);
            }
        }

        dsm_entry_store(MESH_OPT_DSM_SUBNETS_RECORD, subnet_handle, m_subnet_allocated);
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_STATE;
}

uint32_t dsm_subnet_delete(dsm_handle_t subnet_handle)
{
    if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (bitfield_popcount(m_subnet_allocated, DSM_SUBNET_MAX) == 1)
    {
        return NRF_ERROR_FORBIDDEN;
    }

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        if (m_friendships[i].subnet_handle == subnet_handle)
        {
            friendship_clear(&m_friendships[i]);
        }
    }
#endif

    /* Ensure that the subnetwork has no apps associated */
    for (uint32_t i = 0; i < DSM_APP_MAX; i++)
    {
        if (bitfield_get(m_appkey_allocated, i) &&
            m_appkeys[i].subnet_handle == subnet_handle)
        {
            NRF_MESH_ASSERT(dsm_appkey_delete(i) == NRF_SUCCESS);
        }
    }

    if (m_subnets[subnet_handle].net_key_index == PRIMARY_SUBNET_INDEX)
    {
        m_has_primary_subnet = false;
    }

    dsm_entry_invalidate(MESH_OPT_DSM_SUBNETS_RECORD, subnet_handle, m_subnet_allocated);
    return NRF_SUCCESS;
}

uint32_t dsm_subnet_get_all(mesh_key_index_t * p_key_list, uint32_t * p_count)
{
    if (NULL == p_key_list || NULL == p_count )
    {
        return NRF_ERROR_NULL;
    }
    else if (!get_all_subnets(p_key_list, p_count))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    else
    {
        return NRF_SUCCESS;
    }
}

uint32_t dsm_subnet_key_get(dsm_handle_t subnet_handle, uint8_t * p_key)
{
    if (NULL == p_key)
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ||
             m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_3)
    {
        memcpy(p_key, m_subnets[subnet_handle].root_key_updated, NRF_MESH_KEY_SIZE);
        return NRF_SUCCESS;
    }
    else
    {
        memcpy(p_key, m_subnets[subnet_handle].root_key, NRF_MESH_KEY_SIZE);
        return NRF_SUCCESS;
    }
}

uint32_t dsm_devkey_add(uint16_t raw_unicast_addr, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_devkey_handle)
{
    dsm_handle_t handle;
    if (NULL == p_key || NULL == p_devkey_handle)
    {
        return NRF_ERROR_NULL;
    }
    else if (nrf_mesh_address_type_get(raw_unicast_addr) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX ||
             !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (dev_key_handle_get(raw_unicast_addr, &handle))
    {
        return NRF_ERROR_FORBIDDEN;
    }
    else if (handle == DSM_HANDLE_INVALID)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        devkey_set(raw_unicast_addr, subnet_handle, p_key, handle);
        dsm_entry_store(MESH_OPT_DSM_DEVKEYS_RECORD, handle - DSM_DEVKEY_HANDLE_START, m_devkey_allocated);
        *p_devkey_handle = handle;
    }

    __LOG_XB(LOG_SRC_DSM, LOG_LEVEL_DBG3, "Devkey added", p_key, NRF_MESH_KEY_SIZE);
    return NRF_SUCCESS;
}

uint32_t dsm_devkey_delete(dsm_handle_t devkey_handle)
{
    uint16_t devkey_index = devkey_handle - DSM_DEVKEY_HANDLE_START;
    if (devkey_handle < DSM_DEVKEY_HANDLE_START ||
        devkey_handle >= DSM_DEVKEY_HANDLE_START + DSM_DEVICE_MAX ||
        !bitfield_get(m_devkey_allocated, devkey_index))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        m_devkeys[devkey_index].key_owner = NRF_MESH_ADDR_UNASSIGNED;
        dsm_entry_invalidate(MESH_OPT_DSM_DEVKEYS_RECORD, devkey_index, m_devkey_allocated);
        return NRF_SUCCESS;
    }
}

uint32_t dsm_devkey_handle_get(uint16_t unicast_address, dsm_handle_t * p_devkey_handle)
{
    if (p_devkey_handle == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (nrf_mesh_address_type_get(unicast_address) != NRF_MESH_ADDRESS_TYPE_UNICAST)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    else
    {
        dsm_handle_t retval_handle;
        bool key_found = dev_key_handle_get(unicast_address, &retval_handle);
        if (!key_found)
        {
            return NRF_ERROR_NOT_FOUND;
        }
        else
        {
            *p_devkey_handle = retval_handle;
            return NRF_SUCCESS;
        }
    }
}

uint32_t dsm_appkey_add(mesh_key_index_t app_key_index, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_app_handle)
{
    if (NULL == p_key || NULL == p_app_handle)
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (app_key_index > DSM_KEY_INDEX_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (app_key_handle_get(app_key_index, p_app_handle))
    {
        if (0 == memcmp(m_appkeys[*p_app_handle].secmat.key, p_key, NRF_MESH_KEY_SIZE))
        {
            return NRF_ERROR_INTERNAL;
        }
        return NRF_ERROR_FORBIDDEN;
    }
    else if (*p_app_handle == DSM_HANDLE_INVALID)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        appkey_set(app_key_index, subnet_handle, p_key, *p_app_handle);
        dsm_entry_store(MESH_OPT_DSM_APPKEYS_RECORD, *p_app_handle, m_appkey_allocated);
    }

    __LOG_XB(LOG_SRC_DSM, LOG_LEVEL_DBG3, "Appkey added", p_key, NRF_MESH_KEY_SIZE);
    return NRF_SUCCESS;
}

uint32_t dsm_appkey_update(dsm_handle_t app_handle, const uint8_t * p_key)
{
    if (NULL == p_key)
    {
        return NRF_ERROR_NULL;
    }
    else if (app_handle >= DSM_APP_MAX  || !bitfield_get(m_appkey_allocated, app_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (m_subnets[m_appkeys[app_handle].subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_1)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        if (m_appkeys[app_handle].key_updated)
        {
            return 0 == memcmp(m_appkeys[app_handle].secmat_updated.key, p_key, NRF_MESH_KEY_SIZE) ?
                   NRF_SUCCESS : NRF_ERROR_INVALID_STATE;
        }

        m_appkeys[app_handle].key_updated = true;
        memcpy(m_appkeys[app_handle].secmat_updated.key, p_key, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_aid(p_key, &m_appkeys[app_handle].secmat_updated.aid) == NRF_SUCCESS);
        m_appkeys[app_handle].secmat_updated.is_device_key = m_appkeys[app_handle].secmat.is_device_key;

        dsm_entry_store(MESH_OPT_DSM_APPKEYS_RECORD, app_handle, m_appkey_allocated);
    }
    return NRF_SUCCESS;
}

uint32_t dsm_appkey_delete(dsm_handle_t app_handle)
{
    if (app_handle >= DSM_APP_MAX || !bitfield_get(m_appkey_allocated, app_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        dsm_entry_invalidate(MESH_OPT_DSM_APPKEYS_RECORD, app_handle, m_appkey_allocated);
        return NRF_SUCCESS;
    }
}

uint32_t dsm_appkey_get_all(dsm_handle_t subnet_handle, mesh_key_index_t * p_key_list, uint32_t * p_count)
{
    if (NULL == p_key_list || NULL == p_count )
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else if (!get_all_appkeys(subnet_handle, p_key_list, p_count))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    else
    {
        return NRF_SUCCESS;
    }
}

uint32_t dsm_tx_secmat_get(dsm_handle_t subnet_handle, dsm_handle_t app_handle, nrf_mesh_secmat_t * p_secmat)
{
    if (NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t status = app_tx_secmat_get(app_handle, &subnet_handle, &p_secmat->p_app);
    if (NRF_SUCCESS != status)
    {
        return status;
    }

    /* Use updated network security credentials during key refresh phase 2: */
    p_secmat->p_net = m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ?
            &m_subnets[subnet_handle].secmat_updated : &m_subnets[subnet_handle].secmat;

    return NRF_SUCCESS;
}

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
uint32_t dsm_tx_friendship_secmat_get(dsm_handle_t subnet_handle, dsm_handle_t app_handle, nrf_mesh_secmat_t * p_secmat)
{
    if (NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t error = app_tx_secmat_get(app_handle, &subnet_handle, &p_secmat->p_app);
    if (NRF_SUCCESS != error)
    {
        return error;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(m_friendships); i++)
    {
        if (m_friendships[i].subnet_handle == subnet_handle)
        {
            /* Use updated network security credentials during key refresh phase 2: */
            p_secmat->p_net = m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ?
                        &m_friendships[i].secmat_updated : &m_friendships[i].secmat;

            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NOT_FOUND;
}
#endif

uint32_t dsm_beacon_info_get(dsm_handle_t subnet_handle, const nrf_mesh_beacon_info_t ** pp_beacon_info)
{
    if (pp_beacon_info == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle) )
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *pp_beacon_info = &m_subnets[subnet_handle].beacon.info;
        return NRF_SUCCESS;
    }
}

uint32_t dsm_net_secmat_from_keyindex_get(mesh_key_index_t net_key_index, const nrf_mesh_network_secmat_t ** pp_net)
{
    dsm_handle_t subnet_handle = dsm_net_key_index_to_subnet_handle(net_key_index);

    if (pp_net == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle) )
    {
        return NRF_ERROR_NOT_FOUND;
    }
    else
    {
        *pp_net = &m_subnets[subnet_handle].secmat;
        return NRF_SUCCESS;
    }
}

/************************************** Externed functions ****************************************/

/* returns null via pp_secmat if end of search */
void nrf_mesh_net_secmat_next_get(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat, const nrf_mesh_network_secmat_t ** pp_secmat_secondary)
{
    NRF_MESH_ASSERT(NULL != pp_secmat);
    NRF_MESH_ASSERT(NULL != pp_secmat_secondary);

    nid &= PACKET_MESH_NET_NID_MASK;

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    uint32_t j = 0;
    if (*pp_secmat != NULL)
    {
        /* Find pp_secmat in friendships and go to the next one */
        j = friendship_index_by_secmat_get(*pp_secmat) + 1;
    }

    for (; j < ARRAY_SIZE(m_friendships); j++)
    {
        if (m_friendships[j].subnet_handle != DSM_HANDLE_INVALID)
        {
            if (get_net_secmat_by_nid(m_friendships[j].subnet_handle, nid,
                                      &m_friendships[j].secmat, &m_friendships[j].secmat_updated,
                                      pp_secmat, pp_secmat_secondary))
            {
                return;
            }
        }
    }
#endif

#if MESH_FEATURE_LPN_ENABLED
    /* For lpn: if control reaches here and lpn is in friendship => no matching friendship secmats
     * found and hence return null to stop iterating in the network layer.
     */
    if (mesh_lpn_is_in_friendship())
    {
        *pp_secmat = NULL;
        *pp_secmat_secondary = NULL;
        return;
    }
#endif

    dsm_handle_t i = 0;
    if (*pp_secmat != NULL)
    {
        dsm_handle_t prev = get_subnet_handle(*pp_secmat);
        i = (prev == DSM_HANDLE_INVALID) ? 0 : prev + 1;
    }

    *pp_secmat = NULL;
    *pp_secmat_secondary = NULL;

    for (; i < DSM_SUBNET_MAX; i++)
    {
        if (get_net_secmat_by_nid(i, nid,
                                  &m_subnets[i].secmat, &m_subnets[i].secmat_updated,
                                  pp_secmat, pp_secmat_secondary))
        {
            break;
        }
    }
}

/* returns null via pp_app_secmat if end of search */
void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat, uint8_t aid,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat_secondary)
{
    NRF_MESH_ASSERT(NULL != pp_app_secmat);
    NRF_MESH_ASSERT(NULL != pp_app_secmat_secondary);
    NRF_MESH_ASSERT(NULL != p_network_secmat);

    dsm_handle_t subnet_handle = dsm_subnet_handle_get(p_network_secmat);
    if (subnet_handle == DSM_HANDLE_INVALID)
    {
        *pp_app_secmat = NULL;
        *pp_app_secmat_secondary = NULL;
    }
    else
    {
        get_app_secmat(subnet_handle, aid, pp_app_secmat, pp_app_secmat_secondary);
    }
}

void nrf_mesh_devkey_secmat_get(uint16_t owner_addr, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    NRF_MESH_ASSERT(NULL != pp_app_secmat);
    *pp_app_secmat = get_devkey_secmat(owner_addr);
}

void nrf_mesh_primary_net_secmat_get(uint16_t owner_addr, const nrf_mesh_network_secmat_t ** pp_secmat)
{
    dsm_handle_t devkey_handle;
    dsm_handle_t netkey_handle;
    nrf_mesh_secmat_t secmat;

    NRF_MESH_ASSERT(NULL != pp_secmat);
    *pp_secmat = NULL;

    if (NRF_SUCCESS != dsm_devkey_handle_get(owner_addr, &devkey_handle))
    {
        return;
    }

    if (NRF_SUCCESS != dsm_appkey_handle_to_subnet_handle(devkey_handle, &netkey_handle))
    {
        return;
    }

    if (NRF_SUCCESS != dsm_tx_secmat_get(netkey_handle, devkey_handle, &secmat))
    {
        return;
    }

    *pp_secmat = secmat.p_net;
}

void nrf_mesh_beacon_info_next_get(const uint8_t * p_network_id, const nrf_mesh_beacon_info_t ** pp_beacon_info,
        nrf_mesh_key_refresh_phase_t * p_kr_phase)
{
    NRF_MESH_ASSERT(NULL != pp_beacon_info);
    NRF_MESH_ASSERT(NULL != p_kr_phase);

    uint32_t i = 0;
    if (*pp_beacon_info != NULL)
    {
        /* Iterate over the proceeding elements */
        i = get_subnet_handle_by_beacon_info(*pp_beacon_info) + 1;
    }
    for (; i < DSM_SUBNET_MAX; i++)
    {
        if (bitfield_get(m_subnet_allocated, i))
        {
            bool valid_beacon_info = false;
            if (m_subnets[i].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                valid_beacon_info = p_network_id == NULL
                    || memcmp(p_network_id, m_subnets[i].beacon.info.secmat.net_id, NRF_MESH_NETID_SIZE) == 0;
            }
            else
            {
                valid_beacon_info = p_network_id == NULL
                    || memcmp(p_network_id, m_subnets[i].beacon.info.secmat.net_id, NRF_MESH_NETID_SIZE) == 0
                    || memcmp(p_network_id, m_subnets[i].beacon.info.secmat_updated.net_id, NRF_MESH_NETID_SIZE) == 0;
            }

            if (valid_beacon_info)
            {
                /* Update the iv_update_permitted flag, in case we've added the
                 * primary subnetwork since the last access to this struct. If we
                 * are part of the primary subnet, that's the only subnet that can
                 * initiate an IV update. */
                m_subnets[i].beacon.info.iv_update_permitted =
                    (!m_has_primary_subnet ||
                     m_subnets[i].net_key_index == PRIMARY_SUBNET_INDEX);

                *pp_beacon_info = &m_subnets[i].beacon.info;
                *p_kr_phase = m_subnets[i].key_refresh_phase;
                return;
            }
        }
    }
    *pp_beacon_info = NULL;
}

bool nrf_mesh_rx_address_get(uint16_t raw_address, nrf_mesh_address_t * p_address)
{
    nrf_mesh_address_type_t type = nrf_mesh_address_type_get(raw_address);
    bool rx_addr_exists = false;
    switch (type)
    {
        case NRF_MESH_ADDRESS_TYPE_UNICAST:
            rx_addr_exists = rx_unicast_address_get(raw_address, p_address);
            break;
        case NRF_MESH_ADDRESS_TYPE_GROUP:
            rx_addr_exists = rx_group_address_get(raw_address, p_address);
            break;
        case NRF_MESH_ADDRESS_TYPE_VIRTUAL:
            rx_addr_exists = rx_virtual_address_get(raw_address, p_address);
            break;
        default:
            break;
    }
    return rx_addr_exists;
}


void nrf_mesh_unicast_address_get(uint16_t * p_addr_start, uint16_t * p_addr_count)
{
    NRF_MESH_ASSERT(p_addr_start && p_addr_count);
    *p_addr_start = m_local_unicast_addr.address_start;
    *p_addr_count = m_local_unicast_addr.count;
}

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
uint32_t nrf_mesh_friendship_secmat_params_set(const nrf_mesh_network_secmat_t * p_net, const nrf_mesh_keygen_friendship_secmat_params_t *p_secmat_params)
{
    if (NULL == p_net || NULL == p_secmat_params)
    {
        return NRF_ERROR_NULL;
    }

    dsm_handle_t subnet_handle = get_subnet_handle(p_net);

    if (DSM_HANDLE_INVALID == subnet_handle)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint32_t fs_subnet;
    for (fs_subnet = 0; fs_subnet < ARRAY_SIZE(m_friendships); fs_subnet++)
    {
        if (m_friendships[fs_subnet].subnet_handle == DSM_HANDLE_INVALID)
        {
            break;
        }
    }

    if (ARRAY_SIZE(m_friendships) == fs_subnet)
    {
        return NRF_ERROR_NO_MEM;
    }

    m_friendships[fs_subnet].subnet_handle = subnet_handle;
    m_friendships[fs_subnet].secmat_params = *p_secmat_params;

    if (m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        uint32_t error = nrf_mesh_keygen_friendship_secmat(m_subnets[subnet_handle].root_key,
                                                           &m_friendships[fs_subnet].secmat_params,
                                                           &m_friendships[fs_subnet].secmat);
        NRF_MESH_ERROR_CHECK(error);
    }
    else
    {
        uint32_t error = nrf_mesh_keygen_friendship_secmat(m_subnets[subnet_handle].root_key_updated,
                                                           &m_friendships[fs_subnet].secmat_params,
                                                           &m_friendships[fs_subnet].secmat_updated);
        NRF_MESH_ERROR_CHECK(error);
    }

    return NRF_SUCCESS;
}

void nrf_mesh_friendship_secmat_get(uint16_t lpn_addr, const nrf_mesh_network_secmat_t ** pp_secmat)
{
    NRF_MESH_ASSERT(NULL != pp_secmat);
    *pp_secmat = NULL;

    for (uint32_t count = 0; count < ARRAY_SIZE(m_friendships); count++)
    {
        if (m_friendships[count].subnet_handle != DSM_HANDLE_INVALID &&
            m_friendships[count].secmat_params.lpn_address == lpn_addr)
        {
            /* Use updated network security credentials during key refresh phase 2: */
            *pp_secmat = m_subnets[m_friendships[count].subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ?
                    &m_friendships[count].secmat_updated : &m_friendships[count].secmat;
            break;
        }
    }
}

#endif

#if MESH_FEATURE_FRIEND_ENABLED
nrf_mesh_network_secmat_t * nrf_mesh_net_master_secmat_get(const nrf_mesh_network_secmat_t * p_secmat)
{
    if (p_secmat == NULL)
    {
        return NULL;
    }

    /* Check friendships */
    for (uint32_t j = 0; j < ARRAY_SIZE(m_friendships); j++)
    {
        if (m_friendships[j].subnet_handle != DSM_HANDLE_INVALID)
        {
            if (p_secmat == &m_friendships[j].secmat)
            {
                return &m_subnets[m_friendships[j].subnet_handle].secmat;
            }

            if (p_secmat == &m_friendships[j].secmat_updated &&
                m_subnets[m_friendships[j].subnet_handle].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
            {
                return &m_subnets[m_friendships[j].subnet_handle].secmat_updated;
            }
        }
    }

    return NULL;
}
#endif

nrf_mesh_key_refresh_phase_t nrf_mesh_key_refresh_phase_get(const nrf_mesh_network_secmat_t * p_secmat)
{
    NRF_MESH_ASSERT(p_secmat != NULL);

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
    /* Check friendships */
    for (uint32_t j = 0; j < ARRAY_SIZE(m_friendships); j++)
    {
        if (m_friendships[j].subnet_handle != DSM_HANDLE_INVALID)
        {
            if (p_secmat == &m_friendships[j].secmat ||
                p_secmat == &m_friendships[j].secmat_updated)
            {
                return m_subnets[m_friendships[j].subnet_handle].key_refresh_phase;
            }
        }
    }
#endif

    dsm_handle_t handle = get_subnet_handle(p_secmat);
    if (handle != DSM_HANDLE_INVALID)
    {
        return m_subnets[handle].key_refresh_phase;
    }

    /* Should not be reached. */
    NRF_MESH_ASSERT(false);
    return NRF_MESH_KEY_REFRESH_PHASE_0;
}

const nrf_mesh_network_secmat_t * nrf_mesh_net_secmat_from_index_get(uint16_t subnet_index)
{
    const nrf_mesh_network_secmat_t * p_secmat = NULL;
    uint32_t status = dsm_net_secmat_from_keyindex_get(subnet_index, &p_secmat);
    if (status == NRF_SUCCESS)
    {
        return p_secmat;
    }
    else
    {
        return NULL;
    }
}

bool nrf_mesh_is_address_rx(const nrf_mesh_address_t * p_addr)
{
    switch (p_addr->type)
    {
        case NRF_MESH_ADDRESS_TYPE_UNICAST:
        case NRF_MESH_ADDRESS_TYPE_GROUP:
        {
            nrf_mesh_address_t dummy;
            return nrf_mesh_rx_address_get(p_addr->value, &dummy);
        }
        case NRF_MESH_ADDRESS_TYPE_VIRTUAL:
        {
            dsm_handle_t virtual_addr_index;
            if (virtual_address_uuid_index_get(p_addr->p_virtual_uuid, &virtual_addr_index))
            {
                return (m_virtual_addresses[virtual_addr_index].subscription_count > 0);
            }
            return false;
        }
        default:
            return false;
    }
}

bool nrf_mesh_is_device_provisioned(void)
{
    dsm_local_unicast_address_t addr;
    dsm_local_unicast_addresses_get(&addr);
    return (addr.address_start != NRF_MESH_ADDR_UNASSIGNED);
}
