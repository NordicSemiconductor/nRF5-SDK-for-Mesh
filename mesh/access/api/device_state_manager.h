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

#ifndef DEVICE_STATE_MANAGER_H__
#define DEVICE_STATE_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>
#include "nrf_mesh.h"

#include "nrf_mesh_config_app.h"
#include "toolchain.h"

/**
 * @addtogroup DEVICE_STATE_MANAGER
 * @{
 */

/**
 * @defgroup DEVICE_STATE_MANAGER_DEFINES Defines
 * Device State Manager defines.
 * @{
 */

/** Maximum number of addresses in total. */
#define DSM_ADDR_MAX (DSM_VIRTUAL_ADDR_MAX + DSM_NONVIRTUAL_ADDR_MAX)
/** Invalid handle index */
#define DSM_HANDLE_INVALID  (0xFFFF)

/** Maximum key index allowed. */
#define DSM_KEY_INDEX_MAX   (NRF_MESH_GLOBAL_KEY_INDEX_MAX)

/** The highest number of appkeys allowed on the device. */
#define DSM_APP_MAX_LIMIT   (247)
/** The highest number of subnets allowed on the device. */
#define DSM_SUBNET_MAX_LIMIT (252)
/** The highest number of addresses (virtual and group) allowed on the device. */
#define DSM_ADDR_MAX_LIMIT  (185)

/** @} */

/**
 * @defgroup DEVICE_STATE_MANAGER_TYPES Types
 * Device State Manager types.
 * @{
 */

/** Key index type, used for network key index and application key index. */
typedef uint16_t mesh_key_index_t;
/** DSM handle type, used for the handles returned for the each set of data added. */
typedef uint16_t dsm_handle_t;

/**
 * Structure representing the unicast addresses assigned to this device.
 */
typedef struct
{
    uint16_t address_start; /**< First address in the range of unicast addresses. */
    uint16_t count;         /**< Number of addresses in the range of unicast addresses. */
} dsm_local_unicast_address_t;

/** @} */

/**
 * Initialize the device state manager.
 */
void dsm_init(void);

/**
 * Apply data loaded from the mesh configuration system into Device State Manager structures.
 *
 * @note Actual metadata is restored automatically if it was not found or
 * if read out data is not equal configuration parameters.
 *
 * @retval     NRF_ERROR_NOT_FOUND    Device State Manager metadata was not found.
 * @retval     NRF_ERROR_INVALID_DATA Data stored in the persistent memory was corrupted.
 * @retval     NRF_SUCCESS            Data was restored and applied successfully.
 */
uint32_t dsm_load_config_apply(void);

/**
 * Clear all stored state in the Device State Manager, including flash state.
 */
void dsm_clear(void);

/**
 * @defgroup DEVICE_STATE_MANAGER_ADDRESSES Address management
 * Functions for managing the set of addresses known to the device.
 * @{
 */

/**
 * Adds an address to the DSM to be used as a publish address.
 *
 * This address can be either a unicast or a group address.
 *
 * @param[in]  raw_address      The address to add to the device.
 * @param[out] p_address_handle Pointer to a variable where the handle assigned to this
 *                              address is stored.
 *
 * @retval NRF_SUCCESS             The address was successfully added.
 * @retval NRF_ERROR_INVALID_PARAM The address provided was not a unicast or group address.
 * @retval NRF_ERROR_NULL          The @c p_address_handle parameter was NULL.
 */
uint32_t dsm_address_publish_add(uint16_t raw_address, dsm_handle_t * p_address_handle);

/**
 * Adds a publication to an existing address handle.
 *
 * @param[in] address_handle handle of the address.
 *
 * @retval NRF_SUCCESS The publication was successfully added to the address.
 * @retval NRF_ERROR_NOT_FOUND The handle did not point to a previously added address.
 */
uint32_t dsm_address_publish_add_handle(dsm_handle_t address_handle);

/**
 * Adds a virtual address to the DSM to be used as a publish address.
 *
 * @param[in]  p_label_uuid     Pointer to the start of the virtual address label UUID.
 * @param[out] p_address_handle Pointer to a variable where the handle assigned to this
 *                              address is stored.
 *
 * @retval NRF_SUCCESS      The address was successfully added.
 * @retval NRF_ERROR_NULL   The @c p_address_handle parameter was NULL.
 * @retval NRF_ERROR_NO_MEM The address could not be added due to resource constraints.
 */
uint32_t dsm_address_publish_virtual_add(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle);

/**
 * Removes an address that has been used as a publish address.
 *
 * @param[in] address_handle Handle of the address to remove.
 *
 * @retval NRF_SUCCESS         The address was successfully removed.
 * @retval NRF_ERROR_NOT_FOUND The @c address_handle was not pointing to a valid address.
 */
uint32_t dsm_address_publish_remove(dsm_handle_t address_handle);

/**
 * Set the unicast addresses of the device.
 *
 * @note This function can only be called once for the lifetime of a node, unless
 * the node has been removed from the mesh and needs to be reprovisioned, in that
 * case @ref dsm_clear must be called first.
 *
 * @param[in] p_address The unicast addresses to assign to this device.
 *
 * @retval NRF_SUCCESS The addresses has been successfully assigned.
 * @retval NRF_ERROR_FORBIDDEN The unicast addresses have been set once before,
 * and the device state must be reset before they can be changed again.
 * @retval NRF_ERROR_INVALID_DATA The given address range is invalid or it
 * overlaps with non-unicast type addresses.
 */
uint32_t dsm_local_unicast_addresses_set(const dsm_local_unicast_address_t * p_address);

/**
 * Get the local unicast address range of the device.
 *
 * @param[out] p_address Returns the unicast addresses assigned to this device.
 */
void dsm_local_unicast_addresses_get(dsm_local_unicast_address_t * p_address);

/**
 * Adds the specified address to the global subscription list.
 *
 * This function can only be called for group addresses.
 *
 * @param[in]  raw_address      The raw 16-bit address to subscribe to.
 * @param[out] p_address_handle Pointer to a variable where the handle of the address is stored.
 *
 * @retval NRF_SUCCESS The given address has been successfully added to the
 * global subscription list.
 * @retval NRF_ERROR_INVALID_PARAM The provided address handle does not represent
 * a group address.
 * @retval NRF_ERROR_NO_MEM The rx list for group addresses is full, @c
 * DSM_SUBS_LIST_MAX.
 * @retval NRF_ERROR_NOT_FOUND The given address handle does not represent a
 * valid address.
 */
uint32_t dsm_address_subscription_add(uint16_t raw_address, dsm_handle_t * p_address_handle);

/**
 * Adds the specified virtual address to the global subscription list.
 *
 * @param[in]  p_label_uuid     Pointer to the start of the virtual address label UUID.
 * @param[out] p_address_handle Pointer to a variable where the handle assigned to this
 *                              address is stored.
 *
 * @retval NRF_SUCCESS      The specified address has been successfully added to the global
 *                          subscription list.
 * @retval NRF_ERROR_NO_MEM The list of RX addresses is full, @c DSM_SUBS_LIST_MAX.
 */
uint32_t dsm_address_subscription_virtual_add(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle);

/**
 * Adds a subscription to an existing address handle.
 *
 * @param[in] address_handle handle of the address.
 *
 * @retval NRF_SUCCESS The subscription was successfully added to the address.
 * @retval NRF_ERROR_NOT_FOUND The handle did not point to a previously added address.
 */
uint32_t dsm_address_subscription_add_handle(dsm_handle_t address_handle);


/**
 * Returns whether the given address_handle is in the global subscription list.
 *
 * @deprecated This function is deprecated, and the more general @ref nrf_mesh_is_address_rx
 * should be used instead.
 *
 * @param[in] address_handle The reserved handle of the address.
 *
 * @returns Whether the given address handle is present in the global subscription list.
 */
_DEPRECATED bool dsm_address_subscription_get(dsm_handle_t address_handle);

/**
 * Returns whether the device will process packets received on the given destination address.
 *
 * @deprecated This function is deprecated, and the more general @ref nrf_mesh_is_address_rx
 * should be used instead.
 *
 * @param[in] p_addr The raw address to check for.
 *
 * @returns Whether the device will process packets received on the given destination address.
 */
_DEPRECATED bool dsm_address_is_rx(const nrf_mesh_address_t * p_addr);

/**
 * Returns the number of subscriptions registered for an address in the global subscription list.
 *
 * @param[in]  address_handle The reserved handle of the address.
 * @param[out] p_count        A pointer to a variable where the subscription count is stored.
 *
 * @retval NRF_SUCCESS         The subscription count was successfully returned.
 * @retval NRF_ERROR_NOT_FOUND The address was not found in the address list.
 * @retval NRF_ERROR_NULL      The @c p_count parameter was @c NULL.
 */
uint32_t dsm_address_subscription_count_get(dsm_handle_t address_handle, uint16_t * p_count);

/**
 * Removes the given address_handle from the global subscription list.
 * This function can only be called for group and virtual addresses.
 * The address will only be removed from the rx list, but it will still be available in the
 * relevant address list. Therefore, after the removal a valid address can still be
 * retreived with the same handle.
 *
 * @param[in] address_handle The reserved handle of the address.
 *
 * @retval NRF_SUCCESS The given address has been successfully removed from the rx list.
 * @retval NRF_ERROR_FORBIDDEN Unicast addresses can't be removed from the rx list.
 * @retval NRF_ERROR_NOT_FOUND The given address handle does not exist in the subscription lists.
 */
uint32_t dsm_address_subscription_remove(dsm_handle_t address_handle);

/**
 * Retrieves the address for a given address handle and fills out the given @ref nrf_mesh_address_t
 * structure.
 *
 * @param[in] address_handle The reserved handle of the address.
 * @param[in, out] p_address The address retrieved via the given @c address_handle.
 *
 * @retval NRF_SUCCESS An address has been found and returned via the @c p_address.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_NOT_FOUND The given address handle does not exist in the subscription lists.
 */
uint32_t dsm_address_get(dsm_handle_t address_handle, nrf_mesh_address_t * p_address);

/**
 * Get a list of all address handles in the address pool.
 *
 * @param[in, out] p_address_handle_list Pointer to the array for storing all
 * the address handles.
 * @param[in, out] p_count The size of the @c p_address_handle_list array. Will
 * be changed to the number of address handles returned via the @c
 * p_address_handle_list.
 *
 * @retval NRF_SUCCESS The @c p_address_handle_list has been successfully
 * populated by all known address handles.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_LENGTH The @c p_address_handle_list is not large
 * enough to store all address handles, so only a partial list (the first @c
 * *p_count indices) is returned.
 */
uint32_t dsm_address_get_all(dsm_handle_t * p_address_handle_list, uint32_t * p_count);

/**
 * Retrieves the address handle for a given @ref nrf_mesh_address_t structure.
 *
 * @param[in] p_address The address.
 * @param[in, out] p_address_handle The address handle retrieved via the given @c p_address.
 *
 * @retval NRF_SUCCESS An address has been found and returned via the @c p_address.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_NOT_FOUND The given address handle does not exist in the subscription lists.
 */
uint32_t dsm_address_handle_get(const nrf_mesh_address_t * p_address, dsm_handle_t * p_address_handle);

/** @} end of DEVICE_STATE_MANAGER_ADDRESSES */

/**
 * @defgroup DEVICE_STATE_MANAGER_KEYS Key management
 * Functions for managing the set of encryption keys known to the device.
 * @{
 */

/**
 * @defgroup DEVICE_STATE_MANAGER_NET_KEYS Network key management
 * Network key management.
 * @{
 */

/**
 * Retrieves the subnetwork handle for a given network key index.
 *
 * @param[in] net_key_index The network key index used in looking up for the handle.
 *
 * @returns The handle for the subnetwork if found, otherwise @ref DSM_HANDLE_INVALID.
 */
dsm_handle_t dsm_net_key_index_to_subnet_handle(mesh_key_index_t net_key_index);

/**
 * Retrieves the subnetwork handle for a given network security material structure.
 *
 * @param[in] p_secmat The network security material used in looking up for the handle.
 *
 * @returns If found, the handle for the subnetwork stored otherwise @ref DSM_HANDLE_INVALID.
 */
dsm_handle_t dsm_subnet_handle_get(const nrf_mesh_network_secmat_t * p_secmat);

/**
 * Retrieves the key index for a subnetwork.
 *
 * @param[in]  subnet_handle  Handle for the subnetwork.
 * @param[out] p_netkey_index Index of the network key associated with the subnetwork.
 *
 * @retval NRF_SCUCESS         The network key index was successfully returned.
 * @retval NRF_ERROR_NULL      The @c p_netkey_index parameter was @c NULL.
 * @retval NRF_ERROR_NOT_FOUND The specified subnetwork was not found.
 */
uint32_t dsm_subnet_handle_to_netkey_index(dsm_handle_t subnet_handle, mesh_key_index_t * p_netkey_index);

/**
 * Adds a subnetwork and its associated network key index to the device state storage.
 *
 * @param[in] net_key_id The network key index of the subnetwork being added.
 * @param[in] p_key The network key, it must be @ref NRF_MESH_KEY_SIZE bytes long.
 * @param[in,out] p_subnet_handle The assigned handle to the stored subnetwork.
 *
 * @retval NRF_SUCCESS The subnetwork and its associated index have been added successfully.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_PARAM Invalid network key index is given, @see DSM_KEY_INDEX_MAX.
 * @retval NRF_ERROR_FORBIDDEN The given network key index has already been added before.
 * @retval NRF_ERROR_INTERNAL The given network key index has already been added before and keys are the same.
 * @retval NRF_ERROR_NO_MEM The subnetwork storage is out of space, @see DSM_SUBNET_MAX.
 */
uint32_t dsm_subnet_add(mesh_key_index_t net_key_id, const uint8_t * p_key, dsm_handle_t * p_subnet_handle);

/**
 * Retrieves the current key refresh phase for a subnetwork.
 *
 * @param[in]  subnet_handle The handle for the subnetwork to obtain the key refresh phase from.
 * @param[out] p_phase       Pointer to a variable where the key refresh phase is returned.
 *
 * @retval NRF_SUCCESS The current key refresh phase for the network was successfully returned.
 * @retval NRF_ERROR_NOT_FOUND The specified subnet handle is not valid.
 */
uint32_t dsm_subnet_kr_phase_get(dsm_handle_t subnet_handle, nrf_mesh_key_refresh_phase_t * p_phase);

/**
 * Updates an existing subnetwork key.
 *
 * This starts the key refresh procedure for this subnetwork, and causes the subnetwork
 * to enter phase 1 of the procedure. In phase 1, the old keys are still used to transmit
 * messages, but messages can be received using either the old or the new keys.
 *
 * @param[in] subnet_handle The handle for the existing subnetwork.
 * @param[in] p_key The new network key to use.
 *
 * @retval NRF_SUCCESS The network key for the given handle was updated successfully.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_STATE The subnet key could not be updated in the current key refresh phase.
 * @retval NRF_ERROR_NOT_FOUND The given subnet handle is not valid.
 */
uint32_t dsm_subnet_update(dsm_handle_t subnet_handle, const uint8_t * p_key);

/**
 * Starts using the new subnetwork key.
 *
 * This switches the key refresh procedure into phase 2. In phase 2, the key refresh
 * flag is set, and the new keys are used to transmit messages. Messages can still be
 * received using either the old or the new keys.
 *
 * @param[in] subnet_handle The handle for the subnetwork to swap the keys for.
 *
 * @retval NRF_SUCCESS The network key for the given handle was updated successfully.
 * @retval NRF_ERROR_INVALID_STATE The network keys could not be swapped in the current key refresh phase.
 * @retval NRF_ERROR_NOT_FOUND The specified subnet handle is not valid.
 */
uint32_t dsm_subnet_update_swap_keys(dsm_handle_t subnet_handle);

/**
 * Commits to using the new subnetwork key.
 *
 * This switches the key refresh procedure into phase 3. In phase 3, the key refresh
 * flag is cleared and only the new keys are used to send and receive messages. The old
 * keys are cleared from memory. After the new keys have been set up, phase 0 is automatically
 * entered.
 *
 * @param[in] subnet_handle The handle for the subnetwork to commit the new key to.
 *
 * @retval NRF_SUCCESS The network key was successfully committed.
 * @retval NRF_ERROR_INVALID_STATE The network key could not be committed in the current key refresh phase.
 * @retval NRF_ERROR_NOT_FOUND The specified subnet handle is not valid.
 */
uint32_t dsm_subnet_update_commit(dsm_handle_t subnet_handle);

/**
 * Removes an existing subnetwork from the device state storage.
 * All applications bound to the specified subnetwork will also be deleted.
 *
 * @param[in] subnet_handle The handle for the existing subnetwork.
 *
 * @retval NRF_SUCCESS The given subnetwork has been freed successfully.
 * @retval NRF_ERROR_FORBIDDEN The NetList shall contain a minimum of one NetKey.
 * @retval NRF_ERROR_NOT_FOUND The given subnetwork handle is not valid.
 */
uint32_t dsm_subnet_delete(dsm_handle_t subnet_handle);

/**
 * Retrieves all the network key indices of the stored subnetworks.
 *
 * @param[in, out] p_key_list Pointer to the array for storing all the network key indices.
 * @param[in, out] p_count The size of the @c p_key_list array. Will be changed to the number of
 * network key indices returned via the @c p_key_list.
 *
 * @retval NRF_SUCCESS The @c p_key_list has been successfully populated by all the network key indices.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_LENGTH The @c p_key_list is not large enough to store all the network key indices,
 * so only a partial list (the first @c *p_count indices) is returned.
 */
uint32_t dsm_subnet_get_all(mesh_key_index_t * p_key_list, uint32_t * p_count);

/**
 * Gets the (root) network key for a given subnet handle.
 *
 * @note In NRF_MESH_KEY_REFRESH_PHASE_2 and NRF_MESH_KEY_REFRESH_PHASE_3 this will return the updated key.
 *
 * @param[in]  subnet_handle Subnet handle.
 * @param[out] p_key    Pointer to NRF_MESH_KEY_SIZE array to store the key.
 *
 * @retval NRF_SUCCESS         Successfully copied the key.
 * @retval NRF_ERROR_NOT_FOUND Invalid subnet handle.
 */
uint32_t dsm_subnet_key_get(dsm_handle_t subnet_handle, uint8_t * p_key);

/** @} end of DEVICE_STATE_MANAGER_NET_KEYS */

/**
 * @defgroup DEVICE_STATE_MANAGER_DEV_KEYS Device key management
 * Device key management.
 * @{
 */

/**
 * Adds a device key.
 *
 * @note           "A device key is implicitly bound to all network keys." see @tagMeshSp,
 *                 section 3.8.6. An exception to this is the provisioner who stores all the device
 *                 keys of the other nodes, see section 5.
 *
 * @param[in]      raw_unicast_addr          Unicast address associated with this device key.
 * @param[in]      subnet_handle             DSM handle for the subnet this device key is being
 *                                           added, must be a valid network handle.
 * @param[in]      p_key                     The device key, it must be @ref NRF_MESH_KEY_SIZE bytes.
 * @param[in, out] p_devkey_handle           The handle for the device key.
 *
 * @retval         NRF_SUCCESS               The device key has been added successfully.
 * @retval         NRF_ERROR_NULL            Unexpected NULL pointer is given.
 * @retval         NRF_ERROR_FORBIDDEN       The given device key has already been added before.
 * @retval         NRF_ERROR_INVALID_PARAMS  The given address isn't a unicast address.
 * @retval         NRF_ERROR_NO_MEM          The device key storage is out of space,
 *                                           @see DSM_DEVICE_MAX.
 */
uint32_t dsm_devkey_add(uint16_t raw_unicast_addr, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_devkey_handle);

/**
 * Removes an existing device key from the device state storage.
 *
 * @param[in] dev_handle The handle for the existing device key.
 *
 * @retval NRF_SUCCESS The given device handle has been freed successfully.
 * @retval NRF_ERROR_FORBIDDEN The given device key handle is the local device
 * key, which cannot be deleted.
 * @retval NRF_ERROR_NOT_FOUND The given device handle is not valid.
 */
uint32_t dsm_devkey_delete(dsm_handle_t dev_handle);

/**
 * Obtains the handle for a device key.
 *
 * @param[in]  unicast_address Unicast address of the node to look up the device key for.
 * @param[out] p_devkey_handle Pointer to a variable where the handle of the retrieved device key is stored.
 *
 * @retval NRF_SUCCESS            The device key handle was successfully found.
 * @retval NRF_ERROR_NULL         An unexpected @c NULL pointer was passed for the @c p_devkey_handle parameter.
 * @retval NRF_ERROR_INVALID_ADDR The specified address was invalid.
 * @retval NRF_ERROR_NOT_FOUND    The device key for the specified address was not found.
 */
uint32_t dsm_devkey_handle_get(uint16_t unicast_address, dsm_handle_t * p_devkey_handle);

/** @} end of DEVICE_STATE_MANAGER_DEV_KEYS */

/**
 * @defgroup DEVICE_STATE_MANAGER_APP_KEYS Application key management
 * Application key management.
 * @{
 */

/**
 * Retrieves the application key handle for a given application key index.
 *
 * @param[in] appkey_index The application key index used in looking up for the handle.
 *
 * @returns If found, the handle for the application key stored otherwise @ref DSM_HANDLE_INVALID.
 */
dsm_handle_t dsm_appkey_index_to_appkey_handle(mesh_key_index_t appkey_index);

/**
 * Retrieves the application key handle for a given application security material.
 *
 * @param[in] p_secmat The application security material used in looking up for the handle.
 *
 * @returns If found, the handle for the application key stored otherwise @ref DSM_HANDLE_INVALID.
 */
dsm_handle_t dsm_appkey_handle_get(const nrf_mesh_application_secmat_t * p_secmat);

/**
 * Retrieves the application key index for a specified application key handle.
 *
 * @param[in]  appkey_handle        Application key handle.
 * @param[out] p_index              Pointer to a variable where the application key index can be
 *                                  stored.
 *
 * @retval     NRF_SUCCESS          The index of the specified application key was successfully
 *                                  retrieved.
 * @retval     NRF_ERROR_NULL       @c p_index was @c NULL.
 * @retval     NRF_ERROR_NOT_FOUND  The specified application handle is invalid.
 */
uint32_t dsm_appkey_handle_to_appkey_index(dsm_handle_t appkey_handle, mesh_key_index_t * p_index);

/**
 * Stores the network key handle to the p_netkey_handle pointer for a specified application key handle.
 *
 * @param[in]  appkey_handle        Application key handle.
 * @param[out] p_netkey_handle      Pointer to a variable where the associated netkey handle can be
 *                                  stored.
 *
 * @retval     NRF_SUCCESS          The index of the network key associated with the specified application
 *                                  key was successfully retrieved.
 * @retval     NRF_ERROR_NULL       @c p_netkey_handle was @c NULL.
 * @retval     NRF_ERROR_NOT_FOUND  The specified application handle is invalid.
 */
uint32_t dsm_appkey_handle_to_subnet_handle(dsm_handle_t appkey_handle, dsm_handle_t * p_netkey_handle);

/**
 * Adds an application key and its associated application key index to the device state storage.
 * The added application key will be bound with the given subnetwork.
 *
 * @note "An application key shall only be bound to a single network key." see @tagMeshSp,
 * section 3.8.6
 *
 * @param[in] app_key_id The application key index of the application key being added.
 * @param[in] subnet_handle The handle of the subnetwork the application key belongs to.
 * @param[in] p_key The application key, it must be @ref NRF_MESH_KEY_SIZE bytes long.
 * @param[in,out] p_app_handle The assigned handle to the stored application key.
 *
 * @retval NRF_SUCCESS The application key and its associated appplication key index have been added
 * and bound to the given subnetwork successfully.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_NOT_FOUND The given subnetwork handle is not valid.
 * @retval NRF_ERROR_INVALID_PARAM Invalid application key index is given, @see DSM_KEY_INDEX_MAX.
 * @retval NRF_ERROR_FORBIDDEN The given application key index has already been added before.
 * @retval NRF_ERROR_INTERNAL The given application key index has already been added before and keys are the same.
 * @retval NRF_ERROR_NO_MEM The application key storage is out of space, @see DSM_APP_MAX.
 */
uint32_t dsm_appkey_add(mesh_key_index_t app_key_id, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_app_handle);

/**
 * Updates an existing application key.
 *
 * Application keys can only be updated in key refresh phase 1,
 * after the key refresh procedure has been initiated but before the new keys are in use.
 *
 * Updating an application key is an optional part of the key refresh procedure. Any
 * keys not updated will work as before after the key refresh, but be bound to the updated
 * network key.
 *
 * @param[in] app_handle The handle for the existing application key.
 * @param[in] p_key The new application key to use.
 *
 * @retval NRF_SUCCESS The application key for the given handle was updated successfully.
 * @retval NRF_ERROR_NULL Unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_STATE The key could not be updated because the key's subnet was not in the correct key refresh phase.
 * @retval NRF_ERROR_NOT_FOUND The given application handle is not valid.
 */
uint32_t dsm_appkey_update(dsm_handle_t app_handle, const uint8_t * p_key);

/**
 * Removes an existing application key from the device state storage.
 *
 * @param[in] app_handle The handle for the existing application key.
 *
 * @retval NRF_SUCCESS The given application handle has been freed successfully.
 * @retval NRF_ERROR_NOT_FOUND The given application handle is not valid.
 */
uint32_t dsm_appkey_delete(dsm_handle_t app_handle);

/**
 * Retrieves all the application key indices of the stored application keys of a specific subnetwork.
 *
 * @param[in] subnet_handle The handle of the subnetwork the application key belongs to.
 * @param[in, out] p_key_list Pointer to the array for storing the application key indices.
 * @param[in, out] p_count The size of the @c p_key_list array. Will be changed to the number of
 * application key indices returned via the @c p_key_list.
 *
 * @retval NRF_SUCCESS The @c p_key_list has been successfully populated by all
 * the application key indices of the requested subnetwork.
 * @retval NRF_ERROR_NOT_FOUND The given subnetwork handle is not valid.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_LENGTH The @c p_key_list is not large enough to store all the application
 * key indices, so only a partial list (the first @c *p_count indices) is returned.
 */
uint32_t dsm_appkey_get_all(dsm_handle_t subnet_handle, mesh_key_index_t * p_key_list, uint32_t * p_count);

/** @} end of DEVICE_STATE_MANAGER_APP_KEYS */
/** @} end of DEVICE_STATE_MANAGER_KEYS */

/**
 * Retrieves the necessary application and master network security material for sending a mesh packet.
 *
 * It is possible to set the @c subnet_handle to @ref DSM_HANDLE_INVALID value. In this case, the DSM will
 * try to find the network key bound to the application key.
 *
 * @param[in] subnet_handle The subnet handle of the network key used in the transmission.
 * @param[in] app_handle The application handle of the application key used in the transmission.
 * @param[in, out] p_secmat Pointer to the structure for the application and master network security
 * material for a mesh packet.
 *
 * @retval NRF_SUCCESS The requested network and application security materials successfully
 * populated the @c p_secmat.
 * @retval NRF_ERROR_NOT_FOUND The given application handle is not valid.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_STATE There are no allocated subnets.
 */
uint32_t dsm_tx_secmat_get(dsm_handle_t subnet_handle, dsm_handle_t app_handle, nrf_mesh_secmat_t * p_secmat);

#if (MESH_FEATURE_LPN_ENABLED || MESH_FEATURE_FRIEND_ENABLED)
/**
 * Retrieves the necessary application and friendship network security material for sending a mesh packet.
 *
 * It is possible to set the @c subnet_handle to @ref DSM_HANDLE_INVALID value. In this case, the DSM will
 * try to find the network key bound to the application key.
 *
 * @param[in] subnet_handle Subnet handle of the network key used in the transmission.
 * @param[in] app_handle Application handle of the application key used in the transmission.
 * @param[in, out] p_secmat Pointer to the structure for the application and network security
 * material of the current friendship.
 *
 * @retval NRF_SUCCESS The requested network and application security materials successfully
 * populated the @c p_secmat.
 * @retval NRF_ERROR_NOT_FOUND The given application handle is not valid.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_INVALID_STATE There are no allocated subnets.
 */
uint32_t dsm_tx_friendship_secmat_get(dsm_handle_t subnet_handle, dsm_handle_t app_handle, nrf_mesh_secmat_t * p_secmat);
#endif

/**
 * Retrieves the necessary info for sending a mesh network beacon packet.
 *
 * @param[in] subnet_handle The handle of the subnetwork the beacon represents.
 * @param[in, out] pp_beacon_info Pointer to the beacon info structure pointer.
 *
 * @retval NRF_SUCCESS The @c pp_beacon_info has been successfully populated.
 * @retval NRF_ERROR_NOT_FOUND The given subnetwork handle is not valid.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t dsm_beacon_info_get(dsm_handle_t subnet_handle,
                             const nrf_mesh_beacon_info_t ** pp_beacon_info);

/**
 * Retrieves the identity key for advertising with node identity, see @tagMeshSp section 7.2.2.2.3.
 *
 * @param[in] subnet_handle The handle of the subnetwork the node identity key belongs to.
 * @param[in, out] pp_identity Pointer to the identity key list pointer.
 *
 * @retval NRF_SUCCESS The identity key for the given subnetwork is successfully returned via @c pp_identity.
 * @retval NRF_ERROR_NOT_FOUND The given subnetwork handle is not valid.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 * @retval NRF_ERROR_NOT_SUPPORTED This function needs the GATT proxy feautre enabled.
 */
uint32_t dsm_proxy_identity_get(dsm_handle_t subnet_handle, const uint8_t ** pp_identity);

/**
 * Retrives the master network security material from the given netkey index.
 *
 * @param[in]  net_key_index    Netkey index for which security material will be retrived.
 * @param[out] pp_net           Pointer to a variable where the security material pointer is stored.
 *
 * @retval     NRF_SUCCESS          Security material is retrived successfully.
 * @retval     NRF_ERROR_NULL       @c pp_net is a null pointer.
 * @retval     NRF_ERROR_NOT_FOUND  Security material is not found for the given netkey index.
 */
uint32_t dsm_net_secmat_from_keyindex_get(mesh_key_index_t net_key_index,
                                          const nrf_mesh_network_secmat_t ** pp_net);

/** @} end of DEVICE_STATE_MANAGER */

#endif /* DEVICE_STATE_MANAGER_H__ */

