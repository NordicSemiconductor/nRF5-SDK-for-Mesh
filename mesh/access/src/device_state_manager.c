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

#if PERSISTENT_STORAGE
#include "flash_manager.h"
#include "device_state_manager_flash.h"
#endif

#if GATT_PROXY
#include "proxy.h"
#endif  /* GATT_PROXY */

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

#if PERSISTENT_STORAGE
/** Margin to leave on each flash page, to accommodate padding. We'll never pad more than what's
 * required to fit the largest entry. */
#define DSM_FLASH_PAGE_MARGIN (sizeof(fm_header_t) + sizeof(dsm_flash_entry_t))

/** We must be able to store at least all the entries that go into the RAM representation in the
 * flash. Calculate the minimum and static assert. */
#define DSM_FLASH_DATA_SIZE_MINIMUM                                                                                  \
     (ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_unicast_t)), WORD_SIZE) +                               \
      ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_nonvirtual_t)), WORD_SIZE)  * DSM_NONVIRTUAL_ADDR_MAX + \
      ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_virtual_t)), WORD_SIZE)     * DSM_VIRTUAL_ADDR_MAX +    \
      ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_subnet_t)), WORD_SIZE)           * DSM_SUBNET_MAX +          \
      ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_devkey_t)), WORD_SIZE)           * DSM_DEVICE_MAX +          \
      ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_appkey_t)), WORD_SIZE)           * DSM_APP_MAX)

#define DSM_FLASH_PAGE_COUNT_MINIMUM FLASH_MANAGER_PAGE_COUNT_MINIMUM(DSM_FLASH_DATA_SIZE_MINIMUM, DSM_FLASH_PAGE_MARGIN)

#ifdef DSM_FLASH_AREA_LOCATION
NRF_MESH_STATIC_ASSERT(IS_PAGE_ALIGNED(DSM_FLASH_AREA_LOCATION));
#endif

/* If this fails, increase the DSM_FLASH_PAGE_COUNT: */
NRF_MESH_STATIC_ASSERT((DSM_FLASH_PAGE_COUNT) >= DSM_FLASH_PAGE_COUNT_MINIMUM);
NRF_MESH_STATIC_ASSERT(DSM_APP_MAX < DSM_FLASH_HANDLE_FILTER_MASK);
NRF_MESH_STATIC_ASSERT(DSM_SUBNET_MAX < DSM_FLASH_HANDLE_FILTER_MASK);
NRF_MESH_STATIC_ASSERT(DSM_DEVICE_MAX < DSM_FLASH_HANDLE_FILTER_MASK);
NRF_MESH_STATIC_ASSERT(DSM_FLASH_HANDLE_FILTER_MASK <= UINT16_MAX);
NRF_MESH_STATIC_ASSERT(DSM_FLASH_HANDLE_FILTER_MASK <= DSM_HANDLE_INVALID);
NRF_MESH_STATIC_ASSERT(DSM_NONVIRTUAL_ADDR_MAX < DSM_FLASH_HANDLE_FILTER_MASK);
NRF_MESH_STATIC_ASSERT(DSM_VIRTUAL_HANDLE_START + DSM_VIRTUAL_ADDR_MAX < DSM_FLASH_HANDLE_FILTER_MASK);
#endif /* PERSISTENT_STORAGE */

NRF_MESH_STATIC_ASSERT(DSM_APP_MAX >= 1);
NRF_MESH_STATIC_ASSERT(DSM_SUBNET_MAX >= 1);
NRF_MESH_STATIC_ASSERT(DSM_DEVICE_MAX >= 1);

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

typedef enum
{
    DSM_ENTRY_TYPE_LOCAL_UNICAST,
    DSM_ENTRY_TYPE_ADDR_NONVIRTUAL,
    DSM_ENTRY_TYPE_ADDR_VIRTUAL,
    DSM_ENTRY_TYPE_SUBNET,
    DSM_ENTRY_TYPE_APPKEY,
    DSM_ENTRY_TYPE_DEVKEY,
    DSM_ENTRY_TYPES
} dsm_entry_type_t;

#if PERSISTENT_STORAGE

/* Callback function for converting a DSM entry into a flash entry. */
typedef void (*dsm_entry_to_flash_entry_t)(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);
/* Callback function for converting a flash entry into a DSM entry. */
typedef void (*flash_entry_to_dsm_entry_t)(uint32_t index, const dsm_flash_entry_t * p_src, uint16_t entry_len);

typedef struct
{
    uint32_t entry_count;
    uint32_t * p_allocated_bitfield;
    uint32_t * p_needs_flashing_bitfield;
    fm_handle_t flash_start_handle;
    uint32_t flash_entry_data_size;
    dsm_entry_to_flash_entry_t to_flash_entry;
    flash_entry_to_dsm_entry_t to_dsm_entry;
} flash_group_t;

typedef enum
{
    FLASH_STATE_READY,
    FLASH_STATE_WAITING_FOR_MEMORY,
    FLASH_STATE_REMOVING,
    FLASH_STATE_REMOVED,
    FLASH_STATE_REBUILDING
} flash_state_t;
#endif /* PERSISTENT_STORAGE */

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
/* Bitfields for all entry types, indicating whether or not they need their flash representation to
 * be updated. */
static uint32_t m_addr_unicast_needs_flashing[BITFIELD_BLOCK_COUNT(1)];
static uint32_t m_addr_nonvirtual_needs_flashing[BITFIELD_BLOCK_COUNT(DSM_NONVIRTUAL_ADDR_MAX)];
static uint32_t m_addr_virtual_needs_flashing[BITFIELD_BLOCK_COUNT(DSM_VIRTUAL_ADDR_MAX)];
static uint32_t m_subnet_needs_flashing[BITFIELD_BLOCK_COUNT(DSM_SUBNET_MAX)];
static uint32_t m_appkey_needs_flashing[BITFIELD_BLOCK_COUNT(DSM_APP_MAX)];
static uint32_t m_devkey_needs_flashing[BITFIELD_BLOCK_COUNT(DSM_DEVICE_MAX)];

/*****************************************************************************
* Static functions
*****************************************************************************/

static bool flash_save(dsm_entry_type_t type, uint32_t index);
static bool flash_invalidate(dsm_entry_type_t type, uint32_t index);

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

    return false;
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
#if GATT_PROXY
            return proxy_is_enabled();
#else
            return false;
#endif
        case NRF_MESH_ALL_FRIENDS_ADDR:
            return false;
        case NRF_MESH_ALL_RELAYS_ADDR:
        {
            mesh_opt_core_adv_t options;
            NRF_MESH_ERROR_CHECK(mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &options));
            return options.enabled;
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

    if (p_secmat >= &m_appkeys[0].secmat &&
        p_secmat <= &m_appkeys[DSM_APP_MAX - 1].secmat)
    {
        /* The secmat is offset by the same amount in each structure, so since
         * we're getting the delta between two substructures of the same structure
         * type, this will get the right index. */
        return (((uint32_t) p_secmat - (uint32_t) &m_appkeys[0].secmat) / sizeof(appkey_t));
    }
    else if (p_secmat >= &m_devkeys[0].secmat &&
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

static void get_app_secmat(dsm_handle_t subnet_handle, uint8_t aid, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    uint32_t i = 0;
    if (*pp_app_secmat != NULL)
    {
        /* Iterate over the proceeding elements */
        i = get_app_handle(*pp_app_secmat) + 1;
    }
    for (; i < DSM_APP_MAX; i++)
    {
        if (bitfield_get(m_appkey_allocated, i) &&
            m_appkeys[i].subnet_handle == subnet_handle &&
            ((m_appkeys[i].secmat.aid & PACKET_MESH_TRS_ACCESS_AID_MASK) == (aid & PACKET_MESH_TRS_ACCESS_AID_MASK)))
        {
            *pp_app_secmat = &m_appkeys[i].secmat;
            return;
        }
    }
    *pp_app_secmat = NULL;
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
#if GATT_PROXY
    NRF_MESH_ASSERT(NRF_SUCCESS == nrf_mesh_keygen_identitykey(p_key, m_subnets[handle].beacon.info.secmat.identity_key));
#endif

    memcpy(m_subnets[handle].root_key, p_key, NRF_MESH_KEY_SIZE);
    memset(m_subnets[handle].root_key_updated, 0, NRF_MESH_KEY_SIZE);

    m_subnets[handle].net_key_index = net_key_index;
    m_subnets[handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    bitfield_set(m_subnet_allocated, handle);
    bitfield_set(m_subnet_needs_flashing, handle);
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
    bitfield_set(m_appkey_needs_flashing, handle);
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
    bitfield_set(m_devkey_needs_flashing, index);
}

static void nonvirtual_address_set(uint16_t raw_address, dsm_handle_t handle)
{
    m_addresses[handle].address = raw_address;
    m_addresses[handle].subscription_count = 0;
    m_addresses[handle].publish_count = 0;
    bitfield_set(m_addr_nonvirtual_allocated, handle);
    bitfield_set(m_addr_nonvirtual_needs_flashing, handle);
}

static void virtual_address_set(const uint8_t * p_label_uuid, dsm_handle_t handle)
{
    uint32_t index = handle - DSM_VIRTUAL_HANDLE_START;
    memcpy(m_virtual_addresses[index].uuid, p_label_uuid, NRF_MESH_UUID_SIZE);
    NRF_MESH_ASSERT(nrf_mesh_keygen_virtual_address(p_label_uuid, &m_virtual_addresses[index].address) == NRF_SUCCESS);
    bitfield_set(m_addr_virtual_allocated, index);
    bitfield_set(m_addr_virtual_needs_flashing, index);
}

static uint32_t address_delete_if_unused(dsm_handle_t address_handle)
{
    if (address_handle_nonvirtual_valid(address_handle))
    {
        if (m_addresses[address_handle].publish_count == 0 && m_addresses[address_handle].subscription_count == 0)
        {
            bitfield_clear(m_addr_nonvirtual_allocated, address_handle);
            m_addresses[address_handle].address = NRF_MESH_ADDR_UNASSIGNED;
            (void) flash_invalidate(DSM_ENTRY_TYPE_ADDR_NONVIRTUAL, address_handle);
        }

        return NRF_SUCCESS;
    }
    else if (address_handle_virtual_valid(address_handle))
    {
        uint32_t addr_virtual_index = address_handle - DSM_VIRTUAL_HANDLE_START;
        if (m_virtual_addresses[addr_virtual_index].publish_count == 0 &&
            m_virtual_addresses[addr_virtual_index].subscription_count == 0)
        {
            bitfield_clear(m_addr_virtual_allocated, addr_virtual_index);
            m_virtual_addresses[addr_virtual_index].address = NRF_MESH_ADDR_UNASSIGNED;
            (void) flash_invalidate(DSM_ENTRY_TYPE_ADDR_VIRTUAL, addr_virtual_index);
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
            (void) flash_save(DSM_ENTRY_TYPE_ADDR_NONVIRTUAL, handle);
        }
    }

    if (status == NRF_SUCCESS)
    {
        *p_address_handle = handle;
        if (role == DSM_ADDRESS_ROLE_SUBSCRIBE)
        {
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
        (void) flash_save(DSM_ENTRY_TYPE_ADDR_VIRTUAL, dest);
    }
    *p_address_handle = handle;
    if (role == DSM_ADDRESS_ROLE_SUBSCRIBE)
    {
        m_virtual_addresses[dest].subscription_count++;
    }
    else
    {
        m_virtual_addresses[dest].publish_count++;
    }
    return NRF_SUCCESS;
}

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_NET_BEACON_RECEIVED)
    {
        net_beacon_rx_handle(p_evt->params.net_beacon.p_beacon_info,
                             p_evt->params.net_beacon.p_beacon_secmat,
                             p_evt->params.net_beacon.iv_index,
                             p_evt->params.net_beacon.flags.iv_update,
                             p_evt->params.net_beacon.flags.key_refresh);
    }
}
/******************************* FLASH STORAGE MANAGEMENT *****************************************/

#if PERSISTENT_STORAGE
/** Flash manager owning the flash storage area. */
static flash_manager_t m_flash_manager;
/** State of our flash system */
static bool m_flash_is_available;
/** Memory listener used to recover from no-mem returns on the flash manager. */
static fm_mem_listener_t m_flash_mem_listener_update_all;

/* Flash utility functions */
static void addr_unicast_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);

static void addr_nonvirtual_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);
static void addr_virtual_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);
static void subnet_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);
static void appkey_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);
static void devkey_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len);

static void addr_unicast_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);

static void addr_nonvirtual_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);
static void addr_virtual_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);
static void subnet_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);
static void appkey_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);
static void devkey_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_dst, uint16_t entry_len);
static void build_flash_area(void);


/* Each entry type is represented as a flash group. The flash group contains information about the
 * entry type on flash status and behavior, to enable generalizing of the flash access. */

/** Macro to improve readability of flash group definitions. */
#define FLASH_GROUP(NAME, COUNT, FLASH_START_HANDLE)                                \
    {                                                                               \
        .entry_count               = COUNT,                                         \
        .p_allocated_bitfield      = m_##NAME##_allocated,                          \
        .p_needs_flashing_bitfield = m_##NAME##_needs_flashing,                     \
        .flash_start_handle        = FLASH_START_HANDLE,                            \
        .flash_entry_data_size     = sizeof(dsm_flash_entry_##NAME##_t),            \
        .to_flash_entry            = NAME##_to_flash_entry,                         \
        .to_dsm_entry              = NAME##_to_dsm_entry                            \
    }

/** Flash groups representing each entry type's behavior and status */
static const flash_group_t m_flash_groups[] =
{
    FLASH_GROUP(addr_unicast,    1,                       DSM_FLASH_HANDLE_UNICAST),
    FLASH_GROUP(addr_nonvirtual, DSM_NONVIRTUAL_ADDR_MAX, DSM_FLASH_GROUP_ADDR_NONVIRTUAL),
    FLASH_GROUP(addr_virtual,    DSM_VIRTUAL_ADDR_MAX,    DSM_FLASH_GROUP_ADDR_VIRTUAL),
    FLASH_GROUP(subnet,          DSM_SUBNET_MAX,          DSM_FLASH_GROUP_SUBNETS),
    FLASH_GROUP(appkey,          DSM_APP_MAX,             DSM_FLASH_GROUP_APPKEYS),
    FLASH_GROUP(devkey,          DSM_DEVICE_MAX,          DSM_FLASH_GROUP_DEVKEYS)
};
#undef FLASH_GROUP

/* Make sure we have a group for each entry type. */
NRF_MESH_STATIC_ASSERT(sizeof(m_flash_groups) / sizeof(m_flash_groups[0]) == DSM_ENTRY_TYPES);


/*****************************************************************************
* Flash utility functions
*****************************************************************************/
static void addr_unicast_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    /* Ignore the index, as there can only be one. */
    const dsm_flash_entry_addr_unicast_t * p_unicast = &p_entry->addr_unicast;
    memcpy(&m_local_unicast_addr, &p_unicast->addr, sizeof(m_local_unicast_addr));
}

static void subnet_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    const dsm_flash_entry_subnet_t * p_key_data = &p_entry->subnet;
    subnet_set(p_key_data->key_index, p_key_data->key, index);

    m_subnets[index].key_refresh_phase = p_key_data->key_refresh_phase;
    if (m_subnets[index].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        NRF_MESH_ASSERT(entry_len == ALIGN_VAL(sizeof(dsm_flash_entry_subnet_t), WORD_SIZE));
        memcpy(m_subnets[index].root_key_updated, p_key_data->key_updated, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_network_secmat(p_key_data->key_updated, &m_subnets[index].secmat_updated) == NRF_SUCCESS);
    }
    else
    {
        NRF_MESH_ASSERT(entry_len == ALIGN_VAL(sizeof(dsm_flash_entry_subnet_t) - sizeof(p_key_data->key_updated), WORD_SIZE));
    }
}

static void appkey_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    const dsm_flash_entry_appkey_t * p_key_data = &p_entry->appkey;
    appkey_set(p_key_data->key_index, p_key_data->subnet_handle, p_key_data->key, index);

    /* The length of this entry is used to determine if the key is currently being updated: */
    if (entry_len == ALIGN_VAL(sizeof(dsm_flash_entry_appkey_t), WORD_SIZE))
    {
        m_appkeys[index].key_updated = true;
        memcpy(m_appkeys[index].secmat_updated.key, p_key_data->key_updated, NRF_MESH_KEY_SIZE);
        NRF_MESH_ASSERT(nrf_mesh_keygen_aid(p_key_data->key_updated, &m_appkeys[index].secmat_updated.aid) == NRF_SUCCESS);
        m_appkeys[index].secmat_updated.is_device_key = m_appkeys[index].secmat.is_device_key;
    }
    else
    {
        NRF_MESH_ASSERT(entry_len == ALIGN_VAL(sizeof(dsm_flash_entry_appkey_t) - sizeof(p_key_data->key_updated), WORD_SIZE));
    }
}

static void devkey_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    const dsm_flash_entry_devkey_t * p_key_data = &p_entry->devkey;
    devkey_set(p_key_data->key_owner,
               p_key_data->subnet_handle,
               p_key_data->key,
               DSM_DEVKEY_HANDLE_START + index);
}

static void addr_nonvirtual_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    const dsm_flash_entry_addr_nonvirtual_t * p_addr_data = &p_entry->addr_nonvirtual;
    nonvirtual_address_set(p_addr_data->addr, index);
}

static void addr_virtual_to_dsm_entry(uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    const dsm_flash_entry_addr_virtual_t * p_addr_data = &p_entry->addr_virtual;
    virtual_address_set(p_addr_data->uuid, DSM_VIRTUAL_HANDLE_START + index);
}


/******************************************************************************/

static void addr_unicast_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    /* Ignore the index, as there can only be one. */
    dsm_flash_entry_addr_unicast_t * p_entry = &p_dst->addr_unicast;
    memcpy(&p_entry->addr, &m_local_unicast_addr, sizeof(dsm_local_unicast_address_t));
}

static void subnet_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    dsm_flash_entry_subnet_t * p_entry = &p_dst->subnet;
    memset(p_entry, 0, sizeof(dsm_flash_entry_subnet_t));

    p_entry->key_index = m_subnets[index].net_key_index;
    p_entry->key_refresh_phase = m_subnets[index].key_refresh_phase;
    memcpy(p_entry->key, m_subnets[index].root_key, NRF_MESH_KEY_SIZE);

    if (m_subnets[index].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
    {
        memcpy(p_entry->key_updated, m_subnets[index].root_key_updated, NRF_MESH_KEY_SIZE);
    }
    else
    {
        *p_entry_len -= sizeof(p_entry->key_updated) / WORD_SIZE;
    }
}

static void appkey_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    dsm_flash_entry_appkey_t * p_entry = &p_dst->appkey;
    memset(p_entry, 0, sizeof(dsm_flash_entry_appkey_t));

    memcpy(p_entry->key, m_appkeys[index].secmat.key, NRF_MESH_KEY_SIZE);
    p_entry->key_index = m_appkeys[index].app_key_index;
    p_entry->subnet_handle = m_appkeys[index].subnet_handle;

    if (m_appkeys[index].key_updated)
    {
        memcpy(p_entry->key_updated, m_appkeys[index].secmat_updated.key, NRF_MESH_KEY_SIZE);
    }
    else
    {
        *p_entry_len -= sizeof(p_entry->key_updated) / WORD_SIZE;
    }
}

static void devkey_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    dsm_flash_entry_devkey_t * p_entry = &p_dst->devkey;
    memcpy(p_entry->key, m_devkeys[index].secmat.key, NRF_MESH_KEY_SIZE);
    p_entry->key_owner = m_devkeys[index].key_owner;
    p_entry->subnet_handle = m_devkeys[index].subnet_handle;
}

static void addr_nonvirtual_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    dsm_flash_entry_addr_nonvirtual_t * p_entry = &p_dst->addr_nonvirtual;
    p_entry->addr = m_addresses[index].address;
}

static void addr_virtual_to_flash_entry(uint32_t index, dsm_flash_entry_t * p_dst, uint16_t * p_entry_len)
{
    dsm_flash_entry_addr_virtual_t * p_entry = &p_dst->addr_virtual;
    memcpy(p_entry->uuid, m_virtual_addresses[index].uuid, NRF_MESH_UUID_SIZE);
}

/** Flash operation function to call when the memory returns. */
typedef void (*flash_op_func_t)(void);

static void flash_mem_listener_callback(void * p_args)
{
    NRF_MESH_ASSERT(p_args != NULL);
    flash_op_func_t func = (flash_op_func_t) p_args; /*lint !e611 Suspicious cast */
    func();
}

static inline dsm_entry_type_t flash_handle_to_entry_type(fm_handle_t flash_handle)
{
    fm_handle_t flash_group = DSM_FLASH_HANDLE_FILTER_MASK & flash_handle;
    switch (flash_group)
    {
        case 0:
            return (dsm_entry_type_t)((flash_handle & DSM_FLASH_HANDLE_TO_DSM_HANDLE_MASK) -
                                       DSM_FLASH_COLLECTION_HANDLE_FIRST);
        case DSM_FLASH_GROUP_SUBNETS:         return DSM_ENTRY_TYPE_SUBNET;
        case DSM_FLASH_GROUP_APPKEYS:         return DSM_ENTRY_TYPE_APPKEY;
        case DSM_FLASH_GROUP_DEVKEYS:         return DSM_ENTRY_TYPE_DEVKEY;
        case DSM_FLASH_GROUP_ADDR_NONVIRTUAL: return DSM_ENTRY_TYPE_ADDR_NONVIRTUAL;
        case DSM_FLASH_GROUP_ADDR_VIRTUAL:    return DSM_ENTRY_TYPE_ADDR_VIRTUAL;
    }
    /* Found entry that doesn't belong to a flash group */
    NRF_MESH_ASSERT(false);
    return (dsm_entry_type_t) 0;
}

static inline fm_entry_t * dsm_flash_entry_alloc(fm_handle_t flash_handle, uint32_t data_size)
{
    return flash_manager_entry_alloc(&m_flash_manager,
                                     flash_handle,
                                     data_size);
}

static bool flash_store_metainfo(void)
{
    fm_entry_t * p_entry =
        dsm_flash_entry_alloc(DSM_FLASH_HANDLE_METAINFO, sizeof(dsm_flash_entry_metainfo_t));
    bool succeeded = (p_entry != NULL);
    if (succeeded)
    {
        dsm_flash_entry_metainfo_t * p_metainfo = (dsm_flash_entry_metainfo_t *) p_entry->data;
        p_metainfo->max_addrs_nonvirtual = DSM_NONVIRTUAL_ADDR_MAX;
        p_metainfo->max_addrs_virtual = DSM_VIRTUAL_ADDR_MAX;
        p_metainfo->max_subnets = DSM_SUBNET_MAX;
        p_metainfo->max_appkeys = DSM_APP_MAX;
        p_metainfo->max_devkeys = DSM_DEVICE_MAX;
        flash_manager_entry_commit(p_entry);
    }
    return succeeded;
}

static void flash_load(dsm_entry_type_t type, uint32_t index, const dsm_flash_entry_t * p_entry, uint16_t entry_len)
{
    NRF_MESH_ASSERT(type < DSM_ENTRY_TYPES);
    const flash_group_t * p_group = &m_flash_groups[type];

    p_group->to_dsm_entry(index, p_entry, entry_len);
    bitfield_set(p_group->p_allocated_bitfield, index);
}

static bool flash_save(dsm_entry_type_t type, uint32_t index)
{
    bearer_event_critical_section_begin();
    NRF_MESH_ASSERT(type < DSM_ENTRY_TYPES);
    const flash_group_t * p_group = &m_flash_groups[type];

    bool success = false;
    if (m_flash_is_available)
    {
        fm_entry_t * p_entry = dsm_flash_entry_alloc(p_group->flash_start_handle + index,
                                                     p_group->flash_entry_data_size);
        if (p_entry != NULL)
        {
            p_group->to_flash_entry(index, (dsm_flash_entry_t *) p_entry->data, &p_entry->header.len_words);
            flash_manager_entry_commit(p_entry);
            success = true;
        }
    }

    if (!success)
    {
        /* Mark the entry for later flashing */
        bitfield_set(p_group->p_needs_flashing_bitfield, index);
        flash_manager_mem_listener_register(&m_flash_mem_listener_update_all);
    }
    bearer_event_critical_section_end();
    return success;
}

static bool flash_invalidate(dsm_entry_type_t type, uint32_t index)
{
    bearer_event_critical_section_begin();
    NRF_MESH_ASSERT(type < DSM_ENTRY_TYPES);
    const flash_group_t * p_group = &m_flash_groups[type];

    bool success = false;
    if (m_flash_is_available)
    {
        success =
            (NRF_SUCCESS ==
             flash_manager_entry_invalidate(&m_flash_manager, p_group->flash_start_handle + index));
    }

    if (!success)
    {
        /* Mark the entry for later flashing */
        bitfield_set(p_group->p_needs_flashing_bitfield, index);
        flash_manager_mem_listener_register(&m_flash_mem_listener_update_all);
    }
    bearer_event_critical_section_end();
    return success;
}

/** Run through all entries, and update the flash state for the ones that need it. */
static void flash_update_all(void)
{
    bool flash_is_available = true;
    for (dsm_entry_type_t type = (dsm_entry_type_t) 0;
         type < DSM_ENTRY_TYPES && flash_is_available;
         ++type)
    {
        const flash_group_t * p_group = &m_flash_groups[type];
        if (!bitfield_is_all_clear(p_group->p_needs_flashing_bitfield, p_group->entry_count))
        {
            for (uint32_t index = 0; index < p_group->entry_count; index++)
            {
                if (bitfield_get(p_group->p_needs_flashing_bitfield, index))
                {
                    bool success;
                    if (bitfield_get(p_group->p_allocated_bitfield, index))
                    {
                        success = flash_save(type, index);
                    }
                    else
                    {
                        success = flash_invalidate(type, index);
                    }

                    if (success)
                    {
                        bitfield_clear(p_group->p_needs_flashing_bitfield, index);
                    }
                    else
                    {
                        flash_is_available = false;
                        break;
                    }
                }
            }
        }
    }
}

/**
 * Erase all entries, and re-add up to date metainfo once removal is complete.
 */
static void reset_flash_area(void)
{
    m_flash_is_available = false;
    if (flash_manager_remove(&m_flash_manager) != NRF_SUCCESS)
    {
        /* Register the listener and wait for some memory to be freed up before we retry. */
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_listener_callback,
                                                 .p_args = reset_flash_area};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

static void flash_operation_complete(const fm_entry_t * p_entry, fm_result_t result)
{
    /* If we get an AREA_FULL then our calculations for flash space required are buggy. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_AREA_FULL);
    /* We do not invalidate in this module, so a NOT_FOUND should not be received. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_NOT_FOUND);
    if (result == FM_RESULT_ERROR_FLASH_MALFUNCTION)
    {
        /* Let the user know that the flash is dying. */
        nrf_mesh_evt_t evt =
        {
            .type = NRF_MESH_EVT_FLASH_FAILED,
            .params.flash_failed.user = NRF_MESH_FLASH_USER_ACCESS,
            .params.flash_failed.p_flash_entry = p_entry,
            .params.flash_failed.p_flash_page = NULL,
            .params.flash_failed.p_area = m_flash_manager.config.p_area,
            .params.flash_failed.page_count = m_flash_manager.config.page_count,
        };
        event_handle(&evt);
    }
}

static void flash_write_complete(const flash_manager_t * p_manager, const fm_entry_t * p_entry, fm_result_t result)
{
    flash_operation_complete(p_entry, result);
}

static void flash_invalidate_complete(const flash_manager_t * p_manager, fm_handle_t handle, fm_result_t result)
{
    flash_operation_complete(NULL, result);
}

static void flash_remove_complete(const flash_manager_t * p_manager)
{
    build_flash_area();
}

static void build_flash_area(void)
{
    bool success = false;
    flash_manager_config_t manager_config;
    manager_config.write_complete_cb      = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb     = flash_remove_complete;
    manager_config.min_available_space    = 0;
    manager_config.p_area = dsm_flash_area_get();
    manager_config.page_count = DSM_FLASH_PAGE_COUNT;

    /* Lock the bearer event handler to ensure that we don't enter and leave the BUILDING state
     * between adding and checking. */
    bearer_event_critical_section_begin();
    if (flash_manager_add(&m_flash_manager, &manager_config) == NRF_SUCCESS)
    {
        /* If we have to build the flash manager, it means that it's new, and there's no metainfo. */
        if (m_flash_manager.internal.state == FM_STATE_BUILDING)
        {
            success = flash_store_metainfo();
        }
        else
        {
            success = true;
        }
    }
    bearer_event_critical_section_end();

    if (success)
    {
        m_flash_is_available = true;
    }
    else
    {
        /* Register the listener and wait for some memory to be freed up before we retry. */
        static fm_mem_listener_t mem_listener = {.callback = flash_mem_listener_callback,
                                                 .p_args = build_flash_area};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

bool dsm_flash_config_load(void)
{
    flash_manager_wait();
    if (!m_flash_is_available)
    {
        return false;
    }
    const fm_entry_t * p_metainfo =
        flash_manager_entry_get(&m_flash_manager, DSM_FLASH_HANDLE_METAINFO);
    if (p_metainfo == NULL)
    {
        return false;
    }
    /* make sure that the stored flash data isn't too big for this firmware: */
    const dsm_flash_entry_metainfo_t * p_metainfo_data =
        (const dsm_flash_entry_metainfo_t *) p_metainfo->data;

    if (p_metainfo_data->max_addrs_nonvirtual != DSM_NONVIRTUAL_ADDR_MAX ||
        p_metainfo_data->max_addrs_virtual != DSM_VIRTUAL_ADDR_MAX ||
        p_metainfo_data->max_appkeys != DSM_APP_MAX ||
        p_metainfo_data->max_devkeys != DSM_DEVICE_MAX ||
        p_metainfo_data->max_subnets != DSM_SUBNET_MAX)
    {
        /* The area is built with different metadata, reset it */
        reset_flash_area();
        return false;
    }
    const fm_entry_t * p_entry = NULL;

    /* Run through the rest of the entries and load them based on type */
    do
    {
        p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, p_entry);
        if (p_entry != NULL && p_entry != p_metainfo)
        {
            dsm_entry_type_t type = flash_handle_to_entry_type(p_entry->header.handle);
            flash_load(type,
                       p_entry->header.handle - m_flash_groups[type].flash_start_handle,
                       (const dsm_flash_entry_t *) p_entry->data,
                       (p_entry->header.len_words - FLASH_MANAGER_ENTRY_LEN_OVERHEAD) * WORD_SIZE);
        }
    } while (p_entry != NULL);

    /* The storage was valid if there was a local unicast address present */
    return bitfield_get(m_addr_unicast_allocated, 0);
}

bool dsm_has_unflashed_data(void)
{
    for (uint32_t i = 0; i < DSM_ENTRY_TYPES; i++)
    {
        if (!bitfield_is_all_clear(m_flash_groups[i].p_needs_flashing_bitfield, m_flash_groups[i].entry_count))
        {
            return true;
        }
    }
    return false;
}

const void * dsm_flash_area_get(void)
{
#ifdef DSM_FLASH_AREA_LOCATION
    return (const void *) DSM_FLASH_AREA_LOCATION;
#else
    /* Default to putting the area directly before the network flash area */
    return (((const uint8_t *) net_state_flash_area_get()) - (DSM_FLASH_PAGE_COUNT * PAGE_SIZE));
#endif
}

#else
static bool flash_save(dsm_entry_type_t type, uint32_t index)
{
    return true;
}

static bool flash_invalidate(dsm_entry_type_t type, uint32_t index)
{
    return true;
}

bool dsm_flash_config_load(void)
{
    return false;
}
bool dsm_has_unflashed_data(void)
{
    return false;
}

const void * dsm_flash_area_get(void)
{
    return NULL;
}
#endif /* PERSISTENT_STORAGE*/


void dsm_clear(void)
{
#if PERSISTENT_STORAGE
    for (uint32_t i = 0; i < DSM_ENTRY_TYPES; ++i)
    {
        bitfield_clear_all(m_flash_groups[i].p_allocated_bitfield, m_flash_groups[i].entry_count);
        bitfield_clear_all(m_flash_groups[i].p_needs_flashing_bitfield, m_flash_groups[i].entry_count);
    }
#endif

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

    m_local_unicast_addr.address_start = NRF_MESH_ADDR_UNASSIGNED;
    m_local_unicast_addr.count = 0;
    m_has_primary_subnet = false;

#if PERSISTENT_STORAGE
    reset_flash_area();
#endif
}


/*****************************************************************************
* Interface functions
*****************************************************************************/

void dsm_init(void)
{
    m_mesh_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_mesh_evt_handler);

#if PERSISTENT_STORAGE
    m_flash_mem_listener_update_all.callback = flash_mem_listener_callback;
    m_flash_mem_listener_update_all.p_args = flash_update_all;

    m_flash_is_available = false;
    build_flash_area();
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
        memcpy(&m_local_unicast_addr, p_address, sizeof(dsm_local_unicast_address_t));
        bitfield_set(m_addr_unicast_allocated, 0);
        (void) flash_save(DSM_ENTRY_TYPE_LOCAL_UNICAST, 0);
        bitfield_set(m_addr_unicast_needs_flashing, 0);
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
                    return address_delete_if_unused(address_handle);
                }
            }
            else if (addr.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL)
            {
                if (m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].subscription_count == 0)
                {
                    return NRF_ERROR_NOT_FOUND;
                }
                else
                {
                    --m_virtual_addresses[address_handle - DSM_VIRTUAL_HANDLE_START].subscription_count;
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
        (void) flash_save(DSM_ENTRY_TYPE_SUBNET, *p_subnet_handle);
        nrf_mesh_subnet_added(net_key_index, m_subnets[*p_subnet_handle].beacon.info.secmat.net_id);
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

#if GATT_PROXY
        NRF_MESH_ASSERT(nrf_mesh_keygen_identitykey(
                            p_key, m_subnets[subnet_handle].beacon.info.secmat_updated.identity_key) ==
                        NRF_SUCCESS);
#endif

        m_subnets[subnet_handle].key_refresh_phase = NRF_MESH_KEY_REFRESH_PHASE_1;
        net_state_key_refresh_phase_changed(m_subnets[subnet_handle].net_key_index,
                                            m_subnets[subnet_handle].beacon.info.secmat_updated.net_id,
                                            NRF_MESH_KEY_REFRESH_PHASE_1);

        bitfield_set(m_subnet_needs_flashing, subnet_handle);
        (void) flash_save(DSM_ENTRY_TYPE_SUBNET, subnet_handle);
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

        bitfield_set(m_subnet_needs_flashing, subnet_handle);
        (void) flash_save(DSM_ENTRY_TYPE_SUBNET, subnet_handle);
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

                bitfield_set(m_appkey_needs_flashing, i);
                (void) flash_save(DSM_ENTRY_TYPE_APPKEY, i);
            }
        }

        bitfield_set(m_subnet_needs_flashing, subnet_handle);
        (void) flash_save(DSM_ENTRY_TYPE_SUBNET, subnet_handle);
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

    bitfield_clear(m_subnet_allocated, subnet_handle);
    (void) flash_invalidate(DSM_ENTRY_TYPE_SUBNET, subnet_handle);
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
        (void) flash_save(DSM_ENTRY_TYPE_DEVKEY, handle - DSM_DEVKEY_HANDLE_START);
        *p_devkey_handle = handle;
    }
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
        bitfield_clear(m_devkey_allocated, devkey_index);
        (void) flash_invalidate(DSM_ENTRY_TYPE_DEVKEY, devkey_index);
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
        (void) flash_save(DSM_ENTRY_TYPE_APPKEY, *p_app_handle);
    }
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

        bitfield_set(m_appkey_needs_flashing, app_handle);
        (void) flash_save(DSM_ENTRY_TYPE_APPKEY, app_handle);
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
        bitfield_clear(m_appkey_allocated, app_handle);
        (void) flash_invalidate(DSM_ENTRY_TYPE_APPKEY, app_handle);
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
    if (app_handle >= DSM_DEVKEY_HANDLE_START + DSM_DEVICE_MAX)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }

    if (!bitfield_get(app_handle < DSM_DEVKEY_HANDLE_START ? m_appkey_allocated : m_devkey_allocated,
                      app_handle < DSM_DEVKEY_HANDLE_START ? app_handle : app_handle - DSM_DEVKEY_HANDLE_START))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (DSM_HANDLE_INVALID == subnet_handle)
    {
        subnet_handle = app_handle < DSM_DEVKEY_HANDLE_START ? m_appkeys[app_handle].subnet_handle :
                m_devkeys[app_handle - DSM_DEVKEY_HANDLE_START].subnet_handle;
    }

    if (subnet_handle >= DSM_SUBNET_MAX || !bitfield_get(m_subnet_allocated, subnet_handle))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (app_handle < DSM_DEVKEY_HANDLE_START)
    {/* Application key */
        /* Use updated application security credentials (if available) during key refresh phase 2: */
        if (m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 && m_appkeys[app_handle].key_updated)
        {
            p_secmat->p_app = &m_appkeys[app_handle].secmat_updated;
        }
        else
        {
            p_secmat->p_app = &m_appkeys[app_handle].secmat;
        }
    }
    else
    {/* Device key */
        p_secmat->p_app = &m_devkeys[app_handle - DSM_DEVKEY_HANDLE_START].secmat;
    }

    /* Use updated network security credentials during key refresh phase 2: */
    p_secmat->p_net = m_subnets[subnet_handle].key_refresh_phase == NRF_MESH_KEY_REFRESH_PHASE_2 ?
            &m_subnets[subnet_handle].secmat_updated : &m_subnets[subnet_handle].secmat;

    return NRF_SUCCESS;
}

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

    uint32_t i = 0;
    if (*pp_secmat != NULL)
    {
        i = get_subnet_handle(*pp_secmat) + 1; /* we want to iterate over the proceeding elements */
    }

    *pp_secmat = NULL;
    *pp_secmat_secondary = NULL;
    nid &= PACKET_MESH_NET_NID_MASK;

    for (; i < DSM_SUBNET_MAX; i++)
    {
        if (bitfield_get(m_subnet_allocated, i))
        {
            /* If the NIDs for the old and the new network are equal, return both: */
            if (m_subnets[i].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
                    && (m_subnets[i].secmat.nid == nid && m_subnets[i].secmat_updated.nid == nid))
            {
                *pp_secmat = &m_subnets[i].secmat;
                *pp_secmat_secondary = &m_subnets[i].secmat_updated;
                break;
            }
            /* During key refresh, return the updated key if it matches the NID: */
            else if (m_subnets[i].key_refresh_phase != NRF_MESH_KEY_REFRESH_PHASE_0
                    && m_subnets[i].secmat_updated.nid == nid)
            {
                *pp_secmat = &m_subnets[i].secmat_updated;
                break;
            }
            else if (m_subnets[i].secmat.nid == nid)
            {
                *pp_secmat = &m_subnets[i].secmat;
                break;
            }
        }
    }
}

/* returns null via pp_app_secmat if end of search */
void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat, uint8_t aid, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    NRF_MESH_ASSERT(NULL != pp_app_secmat);
    NRF_MESH_ASSERT(NULL != p_network_secmat);

    dsm_handle_t subnet_handle = get_subnet_handle(p_network_secmat);
    if (subnet_handle == DSM_HANDLE_INVALID)
    {
        *pp_app_secmat = NULL;
    }
    else
    {
        get_app_secmat(subnet_handle,aid, pp_app_secmat);
    }
}

void nrf_mesh_devkey_secmat_get(uint16_t owner_addr, const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    NRF_MESH_ASSERT(NULL != pp_app_secmat);
    *pp_app_secmat = get_devkey_secmat(owner_addr);
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
