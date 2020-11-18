# Power-down feature

The power-down feature is Nordic Semiconductor's proprietary functionality of the Bluetooth mesh stack.
It prepares the stack to power off in situations of expected critical power shortage, for example
when the battery that powers the device is about to fully discharge.

**Table of contents**
- [Overview](@ref power_down_overview)
- [Power-down sequence](@ref power_down_collaboration_flow)
    - [Power consumption example](@ref power_down_consumption_example)
- [Power-down usage and dependencies](@ref power_down_usage_mesh)
    - [Power-down file strategy](@ref power_down_file_strategy)
    - [Emergency cache](@ref power_down_emergency_cache)
    - [Replay protection cache](@ref power_down_replay_protection_cache)
    - [Getting power-down time](@ref power_down_mesh_config_power_down_time_getting)


---

## Overview @anchor power_down_overview

The power-down mode is triggered with the @ref mesh_stack_power_down() function.
Calling this function stops all ongoing activity of the stack and makes it store
all data that has not been stored yet in the Bluetooth mesh configuration submodule.

When the power-down procedure is completed, the event @ref NRF_MESH_EVT_READY_TO_POWER_OFF indicates
the stack's readiness for power-off. In this state, the stack is turned off, but
the application timer and timeslots continue to work, and the application can continue to work as well.

The entry point for the power-down mode is located in high-level management API.
However, the power-down functionality collaborates with multiple components within the stack.

![Power-down functionality overview](images/power_down_overview.svg)

## Power-down sequence @anchor power_down_collaboration_flow

The following sequence chart provides an overview of the collaboration flow
between the power-down functionality and other components, after the triggering of the functionality.

![Power-down sequence flow](images/power_down_sequence.svg)

After you call the @ref mesh_stack_power_down() API, the following actions are executed by the API:

1. The stack switches off the scanner as the first step to minimize the power consumption
    and to prevent the reception of new data from the radio channel.
-# The stack stops the activities of the proxy server (if it is used).
    At this point, the advertising of beacons with Node ID and Network ID is stopped.
    The device also closes the active GATT connection (if any), and stops and disables the proxy functionality.
-# The stack stops the timer scheduler.
    Since the advertisement-based bearers depend on the timer scheduler for their operation,
    it is enough to stop the timer scheduler to stop transmitting any data and to stop
    any other periodic tasks in the stack.
-# Using [the mesh configuration module](@ref power_down_usage_mesh), the power-down functionality
   starts storing all data that has not been stored yet.
    @note No Bluetooth mesh config entries should be updated at this point.
        The Bluetooth mesh stack can accept them, which might lead to conflicts with the power-down functionality
        and cause unpredictable behavior.
-# The Bluetooth mesh configuration module completes storing of the data and sends
    the @ref NRF_MESH_EVT_CONFIG_STABLE event to the power-down functionality.
-# The power-down functionality disables an internal scheduler that is responsible for fitting all activities
    within the provided timeslots from the SoftDevice.
-# The low-level scheduler sends the @ref NRF_MESH_EVT_DISABLED event to the power-down functionality
    to notify about stopping of all activities. The power-down functionality sends
    the @ref NRF_MESH_EVT_READY_TO_POWER_OFF event to the application to notify about being ready for power-off.

### Power consumption example @anchor power_down_consumption_example

The following diagram shows an example of power consumption for a device that is preparing to power down.

![Power-down power consumption overview](images/power_down_without_gatt_proxy_power.png)

The diagram is very contextual and depends on many aspects, including the amount of data waiting
to be stored at the moment of power-down, nRF chip variant, and more.

In this case, the device is running the [light switch server example](@ref md_examples_light_switch_README)
on nRF52832 in a network of four devices (two light switch servers, one light switch client,
and one static provisioner), and its replay list contained only a couple of entries.

The width of the power consumption waveform (that is, the pattern within the red circle)
depends on the number and size of entries that need to be stored in the flash memory.

---
## Power-down usage and dependencies @anchor power_down_usage_mesh

The Bluetooth mesh stack never triggers the power-down on its own.
However, when power-down is triggered by the user application it has an impact on the Bluetooth mesh configuration module,
which is discussed in the following sections.

### Power-down files @anchor power_down_file_strategy

The configuration module allows you to create files with the storing strategy labeled
as @ref MESH_CONFIG_STRATEGY_ON_POWER_DOWN.

Data is stored in these files only after the power-down is triggered.
This helps to increase the lifetime of the flash memory, as the flash memory area that is allocated
for these files is kept empty.

When the application calls the @ref mesh_stack_power_down() function, the working mode
of the configuration module changes.
All data from these files is then stored during the power-down preparation.
When the Bluetooth mesh stack comes back from the power-off, it restores data from all files.
It also cleans up the flash area for files that are created with the power-down storing strategy
before the next power-down API call.

### Emergency cache @anchor power_down_emergency_cache

The Bluetooth mesh configuration system and flash manager are separate components that work in parallel in the stack.
When the power-down is about to happen, they might be busy with defragmenting flash pages in the background,
as part of the wear leveling algorithm.
To prevent unpredictable delays caused by the defragmentation process, the power-down functionality
uses the emergency cache, a special flash memory area that is prepared before and always ready for storing data.

The data from files labeled as @ref MESH_CONFIG_STRATEGY_CONTINUOUS is stored in the emergency cache file.
As part of this strategy, the power-down functionality freezes all defragmentation activities
and uses the emergency cache to save all data that has not been stored yet.
The reason for using the cache instead of general areas for the data is that the general areas
might not be ready because of the pending defragmentation.

The emergency cache has descriptors for fake entries. These descriptors are configured beforehand
and are used only for flash memory allocation.
Real entries from different files are wrapped up by the service information
and are stored in the emergency cache.

The cache file is created dynamically. The application must provide the number of allocated flash memory pages
for the emergency cache that is used dynamically during the power-down.
The parameter responsible for the emergency cache area configuration is @ref EMERGENCY_CACHE_RESERVED_PAGE_NUMBER.
The area is cleared before the device is ready for the next power-down API usage.

### Replay protection cache @anchor power_down_replay_protection_cache

The replay protection is a feature of the transport layer that allows for storing
the replay protection cache into persistent memory, also in a power-down scenario.

The replay protection cache is stored into the persistent memory to avoid replay protection attacks
after device power cycle. The interaction with the persistent memory (MCU flash) is implemented
through the @ref md_doc_user_guide_modules_mesh_config module.

The replay protection cache is declared as a separate file.
It is possible to choose one of the following storing strategy options for this file:

- @ref MESH_CONFIG_STRATEGY_ON_POWER_DOWN - This is the default strategy for this file.
It is stored when the power-down happens and the application calls the stack's power-down API.
This allows to keep data in the flash without exhausting the MCU's flash.
    @warning
    If the power-down API is not triggered by the application before the system shuts down
    (with appropriate power back-up to allow power-down functionality to finish writing the entries),
    the replay protection cache will be cleared on the next reboot and this will
    open a possibility of a replay attack on the device.
- @ref MESH_CONFIG_STRATEGY_CONTINUOUS - This strategy stores cache items as soon as their value changes.
This allows to keep the replay cache up-to-date without additional efforts, but
causes the flash to wear out fast if the node receives a lot of messages during its lifetime.
- @ref MESH_CONFIG_STRATEGY_NON_PERSISTENT - This strategy excludes storing replay protection cache
in the flash memory.
    @warning
    Using this strategy causes the replay protection cache to be cleared on the next reboot and
    this will open a possibility of a replay attack on the device.

In the nRF5 SDK for Mesh v4.2.0 and earlier, the behavior of the replay list
was by default equivalent to the strategy based on @ref MESH_CONFIG_STRATEGY_NON_PERSISTENT.
To save the replay cache, you had to manually implement the behavior.

### Getting power-down time @anchor power_down_mesh_config_power_down_time_getting

Use the @ref mesh_config_power_down_time_get() function to calculate the longest time required
for writing to flash with the strategy based on @ref MESH_CONFIG_STRATEGY_ON_POWER_DOWN.
The function returns the theoretical maximum value based on the maximum file sizes and
on the MCU flash writing time from the MCU datasheet.
