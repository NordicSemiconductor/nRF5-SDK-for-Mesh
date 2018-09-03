# Resource usage

The mesh stack is designed to be built together with the user application. It resides in the application code space. It also relies on the SoftDevice being present and thus requires the same hardware resources as the SoftDevice. For information on SoftDevice hardware resource requirements, see the relevant SoftDevice Specification. To be functional, the mesh stack requires a minimum set of the hardware resources provided by the Nordic SoCs in addition to the SoftDevice's requirements.

## SoftDevice Radio Timeslot API

The mesh stack operates concurrently with the SoftDevice through the SoftDevice Radio Timeslot API. The mesh stack takes complete control over the Radio Timeslot API, and this is unavailable to the application.

## Hardware peripherals
The following hardware peripherals are occupied by the mesh stack:

### QDEC
Although the quadrature decoder hardware is not used by the mesh, the interrupt request line dedicated to the QDEC module is utilized for post processing within the mesh stack. Because all the software interrupts available to the application on the nRF51 are frequently used in the nRF5 SDK, the mesh stack uses the QDEC IRQ handler for processing, as this peripheral is not commonly used. This makes combining the mesh stack with SDK applications easier.

If the QDEC peripheral and its interrupt request line is needed by the application, the mesh stack can be configured to use the SWI0 IRQ by defining `BEARER_EVENT_USE_SWI0` during the build.

### RADIO
Shared with the SoftDevice, the RADIO peripheral is occupied by the mesh stack during the acquired Radio Timeslot sessions. The RADIO peripheral should not be modified by the application at any time.

### TIMER0
Shared with the SoftDevice, the TIMER0 peripheral is occupied by the mesh stack during the acquired Radio Timeslot sessions. The TIMER0 peripheral should not be modified by the application at any time.

### TIMER2
By default, the mesh stack uses TIMER2 to manage its radio timing for all low-level operations. Which timer to use can be controlled by changing @ref BEARER_ACTION_TIMER_INDEX in mesh/bearer/api/nrf_mesh_config_bearer.h.

### ECB
Shared with the SoftDevice, the ECB peripheral is occupied by the mesh stack during the acquired Radio Timeslot sessions on the nRF51. For the nRF52, the mesh stack uses the SoftDevice interface for the ECB.

### UART0
If built with serial support, the mesh stack uses the UART0 peripheral to serialize its API. The mesh stack takes full control over the peripheral, and it should not be modified by the application at any time.

### PPI
The mesh uses PPI channels 8, 9, 10 and 11 for various timing related tasks when controlling the radio.

## RAM and flash usage
The core mesh can be configured to achieve higher performance and functionality, or reduced footprint depending on application needs. The mesh stack shares its call stack with the application and the SoftDevice and requires a minimum call stack size of *2 KB*. The mesh stack also requires the presence of a heap (of minimum *380 bytes*), unless it is configured with a custom memory allocator to replace the need for malloc (see `transport_sar_mem_funcs_set` in the transport module).

### nRF52
The following tables show the flash and RAM requirements for the mesh examples on nRF52832. The
examples are build with the GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`) v6.3.1.

#### Build type: `RelWithDebInfo` (`-O3`), Logging: Full

| Flash usage (kB) | RAM usage (kB) | Example                                                         |
|-----------------:|---------------:|:----------------------------------------------------------------|
| 129.784          | 13.760         | Beaconing                                                       |
| 135.600          | 14.044         | DFU without serial interface                                    |
| 149.552          | 17.420         | DFU with serial interface                                       |
| 137.268          | 14.072         | Enocean switch                                                  |
| 128.552          | 12.956         | Light switch dimming client                                     |
| 135.228          | 13.092         | Light switch dimming server                                     |
| 127.480          | 13.140         | Light switch client                                             |
| 140.100          | 14.292         | Light switch provisioner                                        |
| 142.092          | 14.108         | Light switch client with GATT Proxy support                     |
| 145.916          | 14.332         | Light switch server with GATT Proxy support                     |
| 125.240          | 12.712         | Light switch server                                             |
| 130.464          | 13.152         | PB-Remote client                                                |
| 129.308          | 13.620         | PB-Remote server                                                |
| 123.176          | 17.412         | Serial                                                          |

#### Build type: `MinSizeRel` (`-Os`), Logging: None

| Flash usage (kB) | RAM usage (kB) | Example                                                         |
|-----------------:|---------------:|:----------------------------------------------------------------|
| 75.696           | 12.448         | Beaconing                                                       |
| 76.248           | 12.732         | DFU without serial interface                                    |
| 86.584           | 15.852         | DFU with serial interface                                       |
| 77.296           | 14.028         | Enocean switch                                                  |
| 75.476           | 12.912         | Light switch dimming client                                     |
| 81.016           | 13.056         | Light switch dimming server                                     |
| 75.140           | 13.096         | Light switch client                                             |
| 79.840           | 14.240         | Light switch provisioner                                        |
| 85.796           | 14.064         | Light switch client with GATT Proxy support                     |
| 86.532           | 14.288         | Light switch server with GATT Proxy support                     |
| 75.044           | 12.668         | Light switch server                                             |
| 76.076           | 13.104         | PB-Remote client                                                |
| 76.644           | 12.308         | PB-Remote server                                                |
| 73.440           | 15.844         | Serial                                                          |

## Flash lifetime
The flash hardware can withstand a limited number of write/erase cycles. As the mesh stack uses the flash to store state across power failures, the device flash will eventually start failing, resulting in unexpected behavior in the mesh stack. As explained in the [flash manager documentation](@ref md_doc_libraries_flash_manager), the flash manager will write new data to the area by allocating a new entry before invalidating the old one. Because of this, the area must be erased periodically.

The mesh stack uses flash to store the following states:
- Encryption keys
- Mesh addresses
- Access model composition
- Access model configuration
- Network message sequence number
- Network IV index state
- DFU metadata

Assuming that reconfiguration of keys, addresses, and access configuration is rare, the most likely source of flash write exhaustion is the network states. The network message sequence number is written to flash continuously, in user-configurable blocks.

To calculate the flash lifetime of a device, some parameters must be defined:

| Name              | Description                      | Configuration parameter | Default nRF51 | Default nRF52 | Unit |
|------             |---------------                   |-------------------------|------------   | --------------|------|
| `MSG_PER_SEC`     | The number of messages created by the device every second (relayed messages not included). The message sequence number field is 24 bits. It cannot be depleted within one IV update period, which must be at least 192 hours. Because of this, a device cannot possibly send more than `2^24 / (192 * 60 * 60) = 24.3` messages per second on average without breaking the specification. | N/A | 24.3 | 24.3 | messages/s |
| `BLOCK_SIZE`      | The message sequence numbers are allocated in blocks. Every block represents a set number of messages. | `NETWORK_SEQNUM_FLASH_BLOCK_SIZE` | 8192 | 8192 | messages |
| `ENTRY_SIZE`      | The size of a single allocated block entry in flash storage. | N/A | 8 | 8 | bytes |
| `AREA_SIZE`       | Size of the storage area. Must be in flash page sized increments. Defaults to a single page. | N/A | 1024 | 4096 | bytes |
| `AREA_OVERHEAD`   | Static overhead in the storage area, per page. | N/A | 8 | 8 | bytes |
| `ERASE_CYCLES`    | The number of times the device can erase a flash page before it starts faulting. | N/A | 20000 | 10000 | cycles |


The formula for network state flash exhaustion is as follows:

`FLASH LIFETIME [seconds] = ((AREA_SIZE - AREA_OVERHEAD) * ERASE_CYCLES) / (ENTRY_SIZE * MSG_PER_SEC / BLOCK_SIZE)`

### Examples

| Case   | Result   |
|--      | --       |
| Worst case nRF51, default settings | 26.97 years |
| Worst case nRF52, default settings | 54.58 years |

You should recalculate the flash lifetime for any changes to the default flash configuration, because it might cause significantly reduced product lifetime.

### Flash configuration parameters

While the default settings should be sufficient for most applications, there are tradeoffs in the flash configuration that you might want to tune.

The sequence number block size will affect the number of power resets that the device can do within a 192 hour IV update period. For security reasons, the device can never send a message with the same sequence number twice within an IV update period. This means that the device must allocate a new block of sequence numbers _before_ it sends its first packet after a power reset, to avoid a scenario where it reuses the same sequence number on next powerup. As a consequence, every power reset requires a sequence number block allocation, which can exhaust the sequence number space faster than accounted for in the lifetime calculations. With the default block size of 8192, the device may reset 2048 times in a 192 hour interval. If a higher rate of resets is expected, a smaller block size should be considered. Keep in mind that this will directly affect the flash lifetime, because more frequent writes are required during normal operation. The block size can also be increased if the number of power resets are expected to be lower than 2048, resulting in longer device lifetime.

The flash area size will affect the number of erases required for the configuration and network state areas. Be aware though that this does not alter the device lifetime significantly, because the flash manager defragmentation process requires a separate backup page that will be erased once for every backed up page. Adding pages to the flash area will therefore result in fewer, but more expensive defragmentations, with effectively no change to the number of erases required.
