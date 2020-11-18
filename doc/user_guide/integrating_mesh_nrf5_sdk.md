# Integrating Bluetooth mesh into nRF5 SDK examples

The nRF5 SDK for Mesh is compatible with Nordic's @link_nRF5SDK. This allows you to either
include resources from nRF5 SDK in an existing Bluetooth mesh project or include nRF5 SDK for Mesh
functionalities in an nRF5 SDK example.

See @ref md_doc_getting_started_how_to_build for information on how to download
and install the nRF5 SDK.
Check @ref md_examples_sdk_coexist_README to see how the nRF5 SDK features can be simultaneously
used with nRF5 SDK for Mesh.

@note
- The integration with nRF5 SDK is tested only with the nRF5 SDK v17.0.2.
- The nRF5 SDK v17.0.2 does not support devices of the nRF51 series.

**Table of contents**
- [Dynamic memory](@ref coexistence_memory)
- [Concurrent SoftDevice and Bluetooth mesh activity](@ref coexistence_softdevice_mesh_activity)
- [Including nRF5 SDK in an nRF5 SDK for Mesh example](@ref coexistence_nrf5_sdk_in_mesh)
- [Including nRF5 SDK for Mesh functionality in an nRF5 SDK example](@ref coexistence_mesh_in_nrf5_sdk)
    - [Optional changes](@ref coexistence_mesh_in_nrf5_sdk_optional)
    - [nRF5 SDK NVM storage modules](@ref coexistence_mesh_in_nrf5_sdk_nvm)



---

## Dynamic memory @anchor coexistence_memory
While using nRF5 SDK features along with nRF5 Mesh SDK features (as shown in coexistent examples)
you may encounter situation where the application considerations may require changes to how
dynamic memory is allocated and the amount of dynamic memory available for allocation.
The Bluetooth mesh stack uses the @ref MESH_MEM interface for dynamic memory allocation. The default backend,
`mesh_mem_stdlib.c`, uses the standard library `malloc()`, which requires a sufficiently large
heap size to be defined. This behavior can be changed by replacing the backend with another
memory manager.

If you are using Segger Embedded Studio for building the application set the *Heap Size* to *8192*
bytes in the *Project Options> Code> Runtime Memory Area* settings.

---

## Concurrent SoftDevice and Bluetooth mesh activity @anchor coexistence_softdevice_mesh_activity
The largest performance issues when running the SoftDevice and the Bluetooth mesh concurrently usually comes
from radio time contention. While the SoftDevice usually operates in short, scheduled bursts,
the Bluetooth mesh attempts to use the radio for as much time as possible. As long as the SoftDevice
has no radio activity, the Bluetooth mesh will be scanning and advertising continuously.
The SoftDevice activity will reduce the amount of time the Bluetooth mesh gets on-air, and to maintain
a consistently good Bluetooth mesh performance, the SoftDevice radio parameters must be set as conservatively
as possible, without breaking the user experience.

When advertising with the SoftDevice, try using the highest advertising interval
your usage scenario can tolerate. If possible, turn off the SoftDevice advertiser when it is
not needed, and activate it only when you expect to receive a connection request.
If you only need to send non-connectable, non-scannable advertisements (for example,
for third party beacon protocols), use the [Bluetooth mesh Advertiser API](@ref ADVERTISER),
as it is optimized for minimal context switching when used together with the Bluetooth mesh.

When the SoftDevice operates in a connection:
- Try negotiating for the largest possible connection interval your application can tolerate.
If high throughput over the connection is required, it is better to send more data
in each connection event than to reduce the connection interval, as the majority of the overhead
comes from context switching.
- If the Bluetooth mesh device is acting as a peripheral (slave) in its SoftDevice connection, you can also
increase the "slave latency", which should let the SoftDevice skip connection events
without increasing the latency of any outgoing data transfers over the link.
- Just like with the advertisements, it is recommended to only keep the connection alive
when it is needed. Idle SoftDevice connections impact Bluetooth mesh performance almost as much as connections
with a lot of traffic, particularly if the slave latency is low.

SoftDevice-based scanning has the biggest impact on the mesh network performance
of all the SoftDevice activity. The Bluetooth mesh stack is not able to receive packets while the SoftDevice
is scanning, so every SoftDevice scan window replaces Bluetooth mesh scanning. SoftDevice scanning
should only be used when trying to establish connections or when active scanning is required.
If general passive BLE scanning is required (for listening for beacons or other third party activity),
hook into the Bluetooth mesh scanner by setting an RX callback with the @ref nrf_mesh_rx_cb_set function.
If your application requires active scanning or needs to initiate a connection,
the scan parameters should be set as conservatively as possible. Long scan intervals
with short scan windows will give the Bluetooth mesh as much time as possible for its own radio activity.
Similarly, it might be beneficial to perform continuous scanning for a short period
of time when establishing connections, instead of performing long running, duty cycled scanning,
as the context switching would cause a lot of unnecessary overhead.
Finally, setting a timeout for SoftDevice connection initiation calls to avoid idle scanning
for long periods of time is highly recommended.

As the Bluetooth mesh does not actively block SoftDevice radio activity, reducing Bluetooth mesh
activity on the device that runs SoftDevice activity concurrently does not directly affect
the SoftDevice performance. However, other Bluetooth mesh devices nearby will interfere with SoftDevice
activity in the advertisement channels, which could make connection initiation take longer.

Generally, the Bluetooth mesh has to perform all its radio operations between the SoftDevice activity,
so if a device is sending a lot of Bluetooth mesh packets while performing SoftDevice radio operations,
it will spend most of its mesh network time on this, instead of receiving incoming data. To combat this,
reduce the Bluetooth mesh packet sending by scaling the number of outgoing packets
according to the radio time available. If possible, suspend Bluetooth mesh packet relaying during
time-consuming SoftDevice operations by calling the @ref mesh_opt_core_adv_set
on the @c CORE_TX_ROLE_RELAY role.


---


## Including nRF5 SDK in an nRF5 SDK for Mesh example @anchor coexistence_nrf5_sdk_in_mesh
Depending on your toolchain:
- When using Segger Embedded Studio, add code files and include paths to the corresponding
SES project file.
- When building the nRF5 SDK for Mesh stack using CMake, add code files and include paths
to the corresponding CMakeLists.txt file. The SDK_ROOT root symbol is used to refer
to the nRF5 SDK installation folder (see for example `CMakeLists.txt` in the Light Switch server
example).

The Bluetooth mesh example projects already include an `sdk_config.h` file in their `include/` directory.
These files are copies of the default SDK configuration files, and all changes required
by the Bluetooth mesh example are contained in the `include/app_config.h` file in the example's directory.

@note Some SDK features must be explicitly enabled in the SDK configuration file
before they can be used. See the SDK documentation page @link_SDK_configuration_header_file for details.



---


## Including nRF5 SDK for Mesh functionality in an nRF5 SDK example @anchor coexistence_mesh_in_nrf5_sdk

1. Include the following source files from nRF5 SDK for Mesh in the nRF5 SDK example's project file:
    - All C files in `mesh/core/src`
    - All C files in `mesh/bearer/src`
    - All C files in `mesh/prov/src` except nrf_mesh_prov_bearer_gatt.c
    - All C files in `mesh/access/src`
    - All C files in `mesh/dfu/src`
    - All C files in `mesh/stack/src`
    - `models/foundation/config/src/config_server.c`
    - `models/foundation/config/src/composition_data.c`
    - `models/foundation/config/src/packed_index_list.c`
    - `models/foundation/health/src/health_server.c`
    - Any other Bluetooth mesh models that are used in your application
    - `external/micro-ecc/uECC.c`
    - `examples/common/src/assertion_handler_weak.c`
    - `examples/common/src/mesh_provisionee.c`
@note
If various Bluetooth mesh features are not needed (for example, DFU), the corresponding files may simply be
omitted from the project file. However, add `examples/common/src/nrf_mesh_weak.c` in their place to
provide stubs for the missing API functions.
2. Add the following folders to the project include path of the nRF5 SDK example:
    - `mesh/core/api`
    - `mesh/core/include`
    - `mesh/bearer/api`
    - `mesh/bearer/include`
    - `mesh/prov/api`
    - `mesh/prov/include`
    - `mesh/access/api`
    - `mesh/access/include`
    - `mesh/dfu/api`
    - `mesh/dfu/include`
    - `mesh/stack/api`
    - `models/foundation/config/include`
    - `models/foundation/health/include`
    - Path to include folder of any other Bluetooth mesh models that are used in your application
    - `external/micro-ecc`
    - `examples/common/include`
    - Path to any other resources in the Bluetooth mesh examples that are used in your application
3. Add the following preprocessor symbols to the project file of the nRF5 SDK example:
    - `NRF52_SERIES`
    - `NRF_MESH_LOG_ENABLE=NRF_LOG_USES_RTT` (because logging in the Bluetooth mesh stack relies on RTT)
    - `CONFIG_APP_IN_CORE`


### Optional changes @anchor coexistence_mesh_in_nrf5_sdk_optional

Additionally, you might need to apply one or more of the following changes:
- Examples using the `simple_hal` module in the Bluetooth mesh stack may need to be updated to use the
Nordic nRF5 SDK `bsp` module if integrated with the nRF5 SDK. It is possible to use both, but in
this case `GPIOTE_IRQHandler` must be removed from one of them, and only one of the modules may
register callback functions.
- If the original Nordic nRF5 SDK example uses the SoftDevice, make sure that the Bluetooth mesh stack is
initialized and enabled *after* the SoftDevice is enabled. In that case, SoftDevice events must be
forwarded to the Bluetooth mesh stack. Add the following code to your application:
```
#include "nrf_sdh_soc.h"

#define MESH_SOC_OBSERVER_PRIO 0

static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);
```
- If you have multiple SOC observers, ensure that the SOC observer events are forwarded to the Bluetooth mesh
stack only from one of the observers at a time.
- Flash storage of network configuration is enabled by default in the Bluetooth mesh stack as well as in some of
the Nordic nRF5 SDK applications. The flash areas used for this purpose may overlap and cause
errors. To allow safe coexistence of the flash storage module @ref md_doc_user_guide_modules_flash_manager
in the Bluetooth mesh stack and the flash storage module `fstorage` in the Nordic nRF5 SDK, add the following
code block to `nrf_mesh_config_app.h`:
```
#include "fds.h"
#include "fds_internal_defs.h"

#define FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES FDS_PHY_PAGES
```
- If you are adding you own Bluetooth mesh functionality rather than working from an existing Bluetooth mesh
example, you also need to add the file `nrf_mesh_config_app.h`. Copy it from the `examples/templates`
folder in Bluetooth mesh stack repository into your project folder, and remove \c \#error message at the top
of the file. Make other appropriate changes to the file content,
like adjusting `ACCESS_ELEMENT_COUNT` and `ACCESS_MODEL_COUNT` to the required number of elements
and models.

### Flash placement project files @anchor coexistence_mesh_in_nrf5_sdk_flash_placement_xml

The Segger Embedded Studio projects all have a flash_placement.xml file next to them,
which acts as input to the linker. In the flash_placement.xml file, the nRF5 SDK
configures a set of `ProgramSection` listings, which are used to place certain variables.
In addition to all the `ProgramSection`s required by the nRF5 SDK components, the Bluetooth mesh
requires two additional sections, `nrf_mesh_flash` and `nrf_mesh_ram`.

The additional Bluetooth-mesh-related sections must be added to the flash_placement.xml file:

-# Add the following line to the memory segment marked `<MemorySegment name="FLASH" ...>`:
```
<ProgramSection alignment="4" keep="Yes" load="Yes" name=".nrf_mesh_flash"  inputsections="*(SORT(.nrf_mesh_flash.*))" address_symbol="__start_nrf_mesh_flash" end_symbol="__stop_nrf_mesh_flash"/>
```
-# Add the following line to the memory segment marked `<MemorySegment name="RAM" ...>`:
```
<ProgramSection alignment="4" keep="Yes" load="No" name=".nrf_mesh_ram"  inputsections="*(SORT(.nrf_mesh_ram.*))" address_symbol="__start_nrf_mesh_ram" end_symbol="__stop_nrf_mesh_ram"/>
```

@note
This change has already been made for the @ref md_examples_sdk_coexist_README. You can use the
flash_placement.xml files in these examples as a reference when editing flash placement files of
any of the existing nRF5 SDK examples.

### nRF5 SDK NVM storage modules @anchor coexistence_mesh_in_nrf5_sdk_nvm
Using nRF5 SDK modules such as `fstorage`, `pstorage`, or `ble_flash` for writing to flash may be
problematic due to long `timeslot` events occupied by the Bluetooth mesh stack. Use the
@ref md_doc_user_guide_modules_flash_manager module provided by the Bluetooth mesh stack instead.

Furthermore, when writing to flash, ensure not to write or erase areas utilized by the Bluetooth mesh stack
modules and the bootloader (if present). The flash area used by the Bluetooth mesh stack is returned by
`mesh_stack_persistence_flash_usage`. This will work even when the stack is built with the legacy
persistence mode enabled. Note, however, that if any of the flash areas have been specified manually
by `ACCESS_FLASH_AREA_LOCATION`, `NET_FLASH_AREA_LOCATION`, or `DSM_FLASH_AREA_LOCATION`, these
pages will not be included in the flash usage count.
