# Resource usage

To be functional, the mesh stack requires a minimum set of the hardware resources provided
by the Nordic SoCs. The stack is designed to be built together with the user application
and it resides in the application code space. Moreover, it relies on the SoftDevice being present
and thus requires the same hardware resources as the SoftDevice.

For information on SoftDevice hardware resource requirements,
see the relevant SoftDevice Specification.

**Table of contents**
- [SoftDevice Radio Timeslot API](@ref resource_usage_radio_timeslot)
- [Hardware peripherals](@ref resource_usage_hardware_peripherals)
- [RAM and flash usage](@ref resource_usage_ram_and_flash)
    - [Build type: `MinSizeRel` (`-Os`), Logging: On (default)](@ref resource_usage_ram_and_flash_nRF52832-1)
    - [Build type: `MinSizeRel` (`-Os`), Logging: None](@ref resource_usage_ram_and_flash_nRF52832-2)
- [Flash hardware lifetime](@ref resource_usage_flash_lifetime)
    - [Calculating flash lifetime](@ref resource_usage_flash_lifetime_calculating)
        - [Flash example values](@ref resource_usage_flash_lifetime_example_values)
    - [Flash configuration parameters](@ref resource_usage_flash_lifetime_configuration_values)
        - [Sequence number block size](@ref resource_usage_flash_lifetime_configuration_values_sequence)
        - [Flash area size](@ref resource_usage_flash_lifetime_configuration_values_flash_area)




---

## SoftDevice Radio Timeslot API @anchor resource_usage_radio_timeslot

The mesh stack operates concurrently with the SoftDevice through the SoftDevice Radio Timeslot API.
Because the mesh stack takes complete control over the Radio Timeslot API,
this API is unavailable to the application.

---

## Hardware peripherals @anchor resource_usage_hardware_peripherals

The following hardware peripherals are occupied by the mesh stack:
- **RTC1**
    - @link_app_timer_group uses RTC1 as hardware base.
- **QDEC**
    - Although the quadrature decoder hardware is not used by the mesh, the interrupt request line
    dedicated to the QDEC module is utilized for post processing within the mesh stack. Because all
    the software interrupts available to the application on the nRF51 are frequently used
    in the nRF5 SDK, the mesh stack uses the QDEC IRQ handler for processing, as this peripheral
    is not commonly used. This makes combining the mesh stack with SDK applications easier.
        @note If the QDEC peripheral and its interrupt request line is needed by the application,
        the mesh stack can be configured to use the SWI0 IRQ by defining @ref BEARER_EVENT_USE_SWI0
        during the build.
- **RADIO**
    - Shared with the SoftDevice, the RADIO peripheral is occupied by the mesh stack during
    the acquired Radio Timeslot sessions. The application must not modify the RADIO peripheral.
- **TIMER0**
    - Shared with the SoftDevice, the TIMER0 peripheral is occupied by the mesh stack during
    the acquired Radio Timeslot sessions. The application must not modify the TIMER0 peripheral.
- **TIMER2**
    - By default, the mesh stack uses TIMER2 to manage its radio timing for all low-level operations.
    Which timer to use can be controlled by changing @ref BEARER_ACTION_TIMER_INDEX
    in mesh/bearer/api/nrf_mesh_config_bearer.h.
- **ECB**
    - The mesh stack uses the SoftDevice interface for the ECB.
- **UART0**
    - If built with serial support, the mesh stack uses the UART0 peripheral to serialize its API.
    The mesh stack takes full control over the peripheral. The application must not modify it.
- **PPI**
    - The mesh uses PPI channels 8, 9, 10, and 11 for various timing-related tasks
    when controlling the radio.


---


## RAM and flash usage @anchor resource_usage_ram_and_flash
Depending on the application needs, the core mesh can be configured to achieve
either higher performance and functionality or a reduced footprint.

When it comes to memory, the mesh stack:
- shares its call stack with the application and the SoftDevice;
- requires a minimum call stack size of 2 kB;
- requires the presence of a heap (of minimum @ref MESH_MEM_SIZE_MIN bytes), unless it is configured
with a custom memory allocator to replace the need for `stdlib.h`'s `malloc()`.

See the @ref MESH_MEM interface for more details on how to replace the memory manager backend.

The following tables show the flash and RAM requirements for the [mesh examples](@ref md_examples_README).
The values are valid for all [fully compatible configurations](@ref compatibility_list)
based on the nRF52 Series Development Kits.

The examples are built with the [minimum recommended version of GNU Arm Embedded Toolchain](@ref toolchain_build_environment_cmake).


### Build type: `MinSizeRel` (`-Os`), Logging: On (default) @anchor resource_usage_ram_and_flash_nRF52832-1

| Flash usage (kB) | RAM usage (kB) | Example                                                                         |
|-----------------:|---------------:|:--------------------------------------------------------------------------------|
|           96.240 |         11.784 | [Beaconing](@ref md_examples_beaconing_README)                                  |
|           98.776 |         12.064 | [DFU without serial interface](@ref md_examples_dfu_README)                     |
|          109.048 |         15.184 | [DFU with serial interface](@ref md_examples_dfu_README)                        |
|          111.608 |         13.096 | [Dimming client](@ref md_examples_dimming_README)                               |
|          115.680 |         13.244 | [Dimming server](@ref md_examples_dimming_README)                               |
|          112.824 |         13.504 | [EnOcean switch translator client](@ref md_examples_enocean_switch_README)      |
|          114.648 |         13.120 | [Light CTL client](@ref md_examples_light_ctl_README)                           |
|          151.156 |         17.368 | [Light CTL+LC server](@ref md_examples_light_ctl_README)                        |
|          137.152 |         15.752 | [Light CTL server](@ref md_examples_light_ctl_README)                           |
|          139.300 |         16.164 | [Light LC server](@ref md_examples_light_lc_server_README)                      |
|          114.220 |         13.084 | [Light Lightness client](@ref md_examples_light_lightness_README)               |
|          125.328 |         14.556 | [Light Lightness server](@ref md_examples_light_lightness_README)               |
|          110.040 |         13.064 | [Light switch client](@ref md_examples_light_switch_client_README)              |
|          123.944 |         17.252 | [Light switch server](@ref md_examples_light_switch_server_README)              |
|          124.108 |         13.272 | [Low Power node](@ref md_examples_lpn_README)                                   |
|          103.848 |         11.184 | [PB-remote client](@ref md_examples_pb_remote_README)                           |
|           99.836 |         11.612 | [PB-remote server](@ref md_examples_pb_remote_README)                           |
|          110.048 |         12.528 | [Provisioner](@ref md_examples_provisioner_README)                              |
|          119.208 |         17.128 | [Sensor client](@ref md_examples_sensor_README)                                 |
|          117.600 |         13.320 | [Sensor server](@ref md_examples_sensor_README)                                 |
|           96.636 |         14.232 | [Serial](@ref md_examples_serial_README)                                        |

### Build type: `MinSizeRel` (`-Os`), Logging: None @anchor resource_usage_ram_and_flash_nRF52832-2

| Flash usage (kB) | RAM usage (kB) | Example                                                                         |
|-----------------:|---------------:|:--------------------------------------------------------------------------------|
|           82.924 |          9.520 | [Beaconing](@ref md_examples_beaconing_README)                                  |
|           83.428 |          9.800 | [DFU without serial interface](@ref md_examples_dfu_README)                     |
|           92.884 |         12.920 | [DFU with serial interface](@ref md_examples_dfu_README)                        |
|           93.640 |         13.080 | [Dimming client](@ref md_examples_dimming_README)                               |
|           98.288 |         13.228 | [Dimming server](@ref md_examples_dimming_README)                               |
|           94.680 |         13.488 | [EnOcean switch translator client](@ref md_examples_enocean_switch_README)      |
|           94.904 |         13.104 | [Light CTL client](@ref md_examples_light_ctl_README)                           |
|          124.612 |         17.352 | [Light CTL+LC server](@ref md_examples_light_ctl_README)                        |
|          114.976 |         15.736 | [Light CTL server](@ref md_examples_light_ctl_README)                           |
|          116.292 |         16.148 | [Light LC server](@ref md_examples_light_lc_server_README)                      |
|           94.732 |         13.068 | [Light Lightness client](@ref md_examples_light_lightness_README)               |
|          106.640 |         14.540 | [Light Lightness server](@ref md_examples_light_lightness_README)               |
|           93.336 |         13.048 | [Light switch client](@ref md_examples_light_switch_client_README)              |
|          103.720 |         17.236 | [Light switch server](@ref md_examples_light_switch_server_README)              |
|          108.404 |         13.256 | [Low Power node](@ref md_examples_lpn_README)                                   |
|           83.348 |         11.168 | [PB-remote client](@ref md_examples_pb_remote_README)                           |
|           83.584 |          9.348 | [PB-remote server](@ref md_examples_pb_remote_README)                           |
|           87.328 |         12.512 | [Provisioner](@ref md_examples_provisioner_README)                              |
|           95.256 |         17.112 | [Sensor client](@ref md_examples_sensor_README)                                 |
|           97.184 |         13.304 | [Sensor server](@ref md_examples_sensor_README)                                 |
|           83.640 |         11.968 | [Serial](@ref md_examples_serial_README)                                        |


---

## Flash hardware lifetime @anchor resource_usage_flash_lifetime
The flash hardware can withstand a limited number of write and erase cycles.
As the mesh stack uses the flash to store state across power failures, the device flash
will eventually start failing, resulting in unexpected behavior in the mesh stack.

To improve flash lifetime, flash manager does wear leveling by writing a new data to the flash page
by allocating a new entry and then invalidating the old one. Eventually, flash page fills up
and must be erased and re-written (see [flash manager documentation](@ref md_doc_user_guide_modules_flash_manager)).

The mesh stack uses flash to store the following states:
- Encryption keys
- Mesh addresses
- Access model composition
- Access model configuration
- Network message sequence number
- Network IV index state
- DFU metadata

Based on the assumption that the reconfiguration of keys, addresses, and access configuration
is rare, the most likely source of flash write exhaustion are the network states.
The network message sequence number is written to flash continuously, in user-configurable blocks.

### Calculating flash lifetime @anchor resource_usage_flash_lifetime_calculating
The following table lists parameters that must be defined to calculate the flash lifetime
of a device.

| Name              | Description and Configuration parameter | Default nRF51 Series | Default nRF52 Series | Unit |
|-------------------|-----------------------------------------|----------------------|----------------------|------|
| `MSG_PER_SEC`     | The number of messages created by the device every second (relayed messages not included). The message sequence number field is 24 bits. It cannot be depleted within one IV update period, which must be at least 192 hours. Because of this, a device cannot possibly send more than `2^24 / (192 * 60 * 60) = 24.3` messages per second on average without breaking the specification.<br><br> **Configuration parameter:** N/A | 24.3 | 24.3 | messages/s |
| `BLOCK_SIZE`      | The message sequence numbers are allocated in blocks. Every block represents a set number of messages.<br><br> **Configuration parameter:** @ref NETWORK_SEQNUM_FLASH_BLOCK_SIZE | 8192 | 8192 | messages |
| `ENTRY_SIZE`      | The size of a single allocated block entry in flash storage.<br><br> **Configuration parameter:** N/A | 8 | 8 | bytes |
| `AREA_SIZE`       | Size of the storage area. Must be in flash-page-size increments. Defaults to a single page.<br><br> **Configuration parameter:** N/A | 1024 | 4096 | bytes |
| `AREA_OVERHEAD`   | Static overhead in the storage area, per page.<br><br> **Configuration parameter:** N/A | 8 | 8 | bytes |
| `ERASE_CYCLES`    | The number of times the device can erase a flash page before it starts faulting.<br><br> **Configuration parameter:** N/A | 20000 | 10000 | cycles |


The formula for the network state flash exhaustion is as follows:

`FLASH LIFETIME [seconds] = ((AREA_SIZE - AREA_OVERHEAD) * ERASE_CYCLES) / (ENTRY_SIZE * MSG_PER_SEC / BLOCK_SIZE)`

#### Flash example values @anchor resource_usage_flash_lifetime_example_values

| SoC   | Settings  | Case       | Result      |
|-------|-----------|------------|-------------|
| nRF51 | Default   | Worst case | 26.97 years |
| nRF52 | Default   | Worst case | 54.58 years |

As any changes made to the default flash configuration may significantly reduce the product lifetime,
recalculate the network state flash exhaustion time if any of the parameters change.

### Flash configuration parameters @anchor resource_usage_flash_lifetime_configuration_values
While the default settings will be sufficient for most applications,
there are tradeoffs in the flash configuration that you might want to take advantage of.

#### Sequence number block size @anchor resource_usage_flash_lifetime_configuration_values_sequence
The sequence number block size affects the number of power resets that the device can do within
a 192-hour IV update period.

For security reasons, the device can never send a message with the same sequence number twice
within an IV update period. This means that the device must allocate a new block of sequence numbers
_before_ it sends its first packet after a power reset, to avoid a scenario where it reuses
the same sequence number on next powerup. As a consequence, every power reset requires a sequence
number block allocation, which can exhaust the sequence number space faster than accounted for
in the lifetime calculations.

With the default block size of 8192, the device may reset 2048 times in a 192-hour interval.
If you expect a higher rate of resets, consider a smaller block size. Keep in mind that this
will directly affect the flash lifetime, because more frequent writes are required during
the normal operation.

The block size can also be increased if the number of power resets is expected to be lower than 2048,
resulting in longer device lifetime.

#### Flash area size @anchor resource_usage_flash_lifetime_configuration_values_flash_area
The flash area size affects the number of erases required for the configuration
and network state areas.

This does not alter the device lifetime significantly, because the flash manager defragmentation
process requires a separate backup page that will be erased for every backed-up page.
Adding pages to the flash area will therefore result in fewer, but more expensive defragmentations,
with effectively no change to the number of erases required.
