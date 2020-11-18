# Mesh configuration

Mesh configuration is a Bluetooth mesh submodule designed to abstract and simplify persistent key-value storage.
It provides a high-level API with swappable storage backends that can be used to store Bluetooth mesh and
application state.

**Table of contents**
- [Overview](@ref mesh_config_overview)
    - [State owners](@ref mesh_config_overview_state_owners)
    - [Change listeners](@ref mesh_config_overview_change_listeners)
    - [Backends](@ref mesh_config_overview_backends)
- [Information flow](@ref mesh_config_information_flow)
    - [Loading from persistent storage](@ref mesh_config_information_flow_loading)
    - [Setting a value](@ref mesh_config_information_flow_setting_value)
    - [Getting a value](@ref mesh_config_information_flow_getting_value)
- [Usage in Bluetooth mesh](@ref mesh_config_usage_mesh)
- [Usage in the application](@ref mesh_config_usage_app)
    - [Storing state](@ref mesh_config_usage_app_storing)
    - [Creating entries](@ref mesh_config_usage_app_creating)


---


## Overview @anchor mesh_config_overview

The Bluetooth mesh configuration module organizes data in files, where each file has a set of records, or entries.
A Bluetooth mesh configuration entry is uniquely identified by a file/record pair, and it can represent any
immutable structure with a predefined size.

![Mesh configuration module overview](images/mesh_config_overview.svg)

### State owners @anchor mesh_config_overview_state_owners

An entry is owned by a *state owner* submodule outside the Bluetooth mesh configuration module.
The state owner holds the entry structure and is responsible for sanitizing and storing
the actual representation of the value, or "live value". It provides access to the live value
through a `setter`/`getter`/`deleter` interface.

There are several rules concerning this interface:
- The live value must never be altered outside the state owner's `setter` callback.
- The `getter` callback must always return the value set in the previous `setter` callback.
- The state owner is notified of deleted entries through the `deleter` callback. It cannot interfere
with the deletion, only observe it.

The state owner:
- doesn't have to keep the live value in RAM, but must be able to atomically produce the live value
on the request from the configuration module.
- can specify that the entry has a default value to use if the user hasn't explicitly set the entry.
If the entry doesn't have a default value, attempts to read the value will result in an error
until the value has been set or loaded from persistent storage.
- can choose to reject values it considers invalid by returning an error code from its
`setter` callback.

See the [Information flow](@ref mesh_config_information_flow) section below for overview
of the `setter` and `getter` sequences.

### Change listeners @anchor mesh_config_overview_change_listeners

In some cases, other modules need to passively listen to state changes to a specific entry.
These modules are called "change listeners".

For example, the internal Bluetooth mesh heartbeat module needs to know when the Proxy feature is enabled,
as this triggers the Heartbeat module to publish a message.

This pattern is a common source of unwanted coupling between modules. To reduce this coupling,
the Bluetooth mesh config module provides @ref MESH_CONFIG_LISTENER_MODULE for registering passive change
listeners without involving the state owner. Any module can register a listener for any entry.
It is then notified of any state changes after they are sanitized by the state owner.

@note To enable or disable any interrupts available for the application inside
`setter`/`getter`/`deleter` or listener callbacks, use only @link_SoftDevice_NVIC.

### Backends @anchor mesh_config_overview_backends

The configuration module is designed to work on top of any key-value storage backend.
In this initial version of the module, the only supported backend is @ref FLASH_MANAGER,
with support for other backends planned for future releases.

The config backend API is considered internal and should never be called directly.


---


## Information flow @anchor mesh_config_information_flow
See the sequence charts in this section for an overview of the information flow of the configuration
module.

@note
Use the mesh_config API for all user interaction with the configuration module. Manipulating
the live values directly causes the module to fall out of sync.

### Loading from persistent storage @anchor mesh_config_information_flow_loading
The following sequence chart illustrates the flow of loading values from persistent storage.

![Loading](images/mesh_config_load.svg)

### Setting a value @anchor mesh_config_information_flow_setting_value
The following sequence chart illustrates the flow of storing values.

The state owner's `setter` callback is called before the value is stored persistently.
The entry should only be considered safely stored after the config module emits an
@ref NRF_MESH_EVT_CONFIG_STABLE event. The user can also call @ref mesh_config_is_busy to determine
whether the live values are in sync with the persistent storage.

![Storing](images/mesh_config_save.svg)

### Getting a value @anchor mesh_config_information_flow_getting_value
The following sequence chart illustrates the flow of reading a value.

![Reading](images/mesh_config_get.svg)


---


## Usage in Bluetooth mesh @anchor mesh_config_usage_mesh

The configuration module is used internally in the Bluetooth mesh to store options (through the @ref MESH_OPT
API) and the runtime state. The Bluetooth mesh reserves the following file IDs:

| File ID            | Owner       | Purpose
|--------------------|-------------|---------
| `0x0000`           | `net_state` | Network runtime state, such as sequence number and IV index.
| `0x0001`           | `dsm`       | Device State manager state, such as keys and addresses.
| `0x0002`           | `access`    | Access state, such as model key bindings.
| `0x0003`           | `mesh_opt`  | Bluetooth mesh runtime options.
| `0x0004`- `0x000F` | -           | Reserved for future use.

This usage can be compatible with the stack persistence used in nRF5 SDK for Mesh v3.2.0 (or an earlier
version). If `ACCESS_FLASH_PAGE_COUNT` and `DSM_FLASH_PAGE_COUNT` are defined, the flash manager backend will
be built in a backward compatible mode, with the stack files located where they would be located using the older
version. This mode also takes into account `NET_FLASH_PAGE_COUNT`, as
well as `ACCESS_FLASH_AREA_LOCATION`, `DSM_FLASH_AREA_LOCATION`, and `NET_FLASH_AREA_LOCATION`, for locating
the pages in flash.

---


## Usage in the application @anchor mesh_config_usage_app

The application may use the Bluetooth mesh configuration module to store its own state and create entries.

### Storing state @anchor mesh_config_usage_app_storing
To store the state, declare a Bluetooth mesh config file with a unique file ID using the @ref MESH_CONFIG_FILE
macro:
```C
MESH_CONFIG_FILE(m_app_file, 0x0010, MESH_CONFIG_STRATEGY_CONTINUOUS);
```
The file will be automatically registered in the Bluetooth mesh configuration. The `CONTINUOUS` storage
strategy ensures that the file's entries are stored in persistent memory as soon as
they are set.

### Creating entries @anchor mesh_config_usage_app_creating
To create entries, invoke the @ref MESH_CONFIG_ENTRY macro with a setter and a getter function.
This example creates an entry with record number `0x0001` in the `m_app_file` created above.
The value is a uint32_t, configured to only accept values below 10000, with the default value 5000.

```C

#define APP_ENTRY_ID    MESH_CONFIG_ENTRY_ID(0x0010, 0x0001)

/* Live RAM representation of the value */
static uint32_t m_live_value = 5000;


static uint32_t app_entry_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint32_t * p_value = (const uint32_t *) p_entry;
    if (*p_value >= 10000)
    {
        /* Rejecting an invalid value. The value will not be stored to persistent storage. */
        return NRF_ERROR_INVALID_DATA;
    }

    m_live_value = *p_value;
    return NRF_SUCCESS;
}

static void app_entry_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint32_t * p_value = (uint32_t *) p_entry;
    *p_value = m_live_value;
}

MESH_CONFIG_ENTRY(m_app_entry,
                  APP_ENTRY_ID,
                  1, // The entry is singular
                  sizeof(uint32_t),
                  app_entry_setter,
                  app_entry_getter,
                  NULL, // No need for a delete callback
                  true); // There is a default value
```

The config entry is registered automatically, and the user can set and get the value
with the following functions:

```C
void user_function(void)
{
    uint32_t set_value = 19;
    mesh_config_entry_set(APP_ENTRY_ID, &set_value);

    uint32_t get_value;
    mesh_config_entry_get(APP_ENTRY_ID, &get_value);

    // get_value == set_value
}
```
