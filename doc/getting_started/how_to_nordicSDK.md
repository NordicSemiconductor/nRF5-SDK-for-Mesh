# Integrating Mesh into nRF5 SDK examples

You can either:
- [include resources from nRF5 SDK in an existing mesh project](@ref coexistence_nrf5_sdk_in_mesh),
- [include nRF5 SDK for Mesh functionalities in an nRF5 SDK example](@ref coexistence_mesh_in_nrf5_sdk).

See @ref md_doc_getting_started_how_to_build for information on how to download and install the nRF5 SDK.
See @ref md_examples_sdk_coexist_README examples to see how the nRF5 SDK features can be simultaneously used with nRF5 SDK for Mesh.

@note
* nRF5 SDK integration is only tested with nRF5 SDK version 15.2.
* nRF5 SDK version 15.2 does not support nRF51.

## Dynamic memory @anchor coexistence_memory
The mesh stack uses the @ref MESH_MEM interface for dynamic memory allocation. The default backend,
`mesh_mem_stdlib.c`, uses the standard library `malloc()`, which requires a sufficiently large
heap size to be defined. This behavior can be changed by replacing the backend with another
memory manager.

If you are using Segger Embedded Studio for building the application set the *Heap Size* to *8192* bytes in the
*Project Options> Code> Runtime Memory Area* settings.

## Concurrent SoftDevice and mesh activity @anchor coexistence_softdevice_mesh_activity
By design, the SoftDevice activity is prioritized over mesh activity. Therefore, you should keep the
connection and advertisement intervals used by the SoftDevice as large as possible when using
Bluetooth low energy connections. If scanning, keep the scan intervals as long as possible, and the
scan windows as short as possible. You should also reduce mesh activity while the SoftDevice is
doing fast advertising and continue normal activity after a connection is established.

---


## Including nRF5 SDK in an nRF5 SDK for Mesh example @anchor coexistence_nrf5_sdk_in_mesh

Depending on your toolchain:
- When using Segger Embedded Studio, add code files and include paths to the corresponding
SES project file.
- When building the nRF5 SDK for Mesh stack using CMake, add code files and include paths to the corresponding CMakeLists.txt file.
The SDK_ROOT root symbol is used to refer to the nRF5 SDK installation folder
(see e.g. `CMakeLists.txt` in the Light Switch Server example).


---


## Including nRF5 SDK for Mesh functionality in an nRF5 SDK example @anchor coexistence_mesh_in_nrf5_sdk

Include the following source files from nRF5 SDK for Mesh in the nRF5 SDK example's project file:
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
- Any other mesh models that are used in your application
- `external/micro-ecc/uECC.c`
- `examples/common/src/assertion_handler_weak.c`
- `examples/common/src/mesh_provisionee.c`

@note
If various mesh features are not needed (like e.g. DFU), the corresponding files may simply be
omitted from the project file. Then `examples/nrf_mesh_weak.c` must be added in their place to
provide stubs for the missing API functions.

Add the following folders to the nRF5 SDK example's project include path:
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
- Path to include folder of any other mesh models that are used in your application
- `external/micro-ecc`
- `examples/common/include`
- Path to any other resources in the mesh examples that are used in your application

Add the following preprocessor symbols to the nRF5 SDK example's project file:
 - `NRF52_SERIES`
 - `NRF_MESH_LOG_ENABLE=NRF_LOG_USES_RTT` (because the logging in the mesh stack relies on RTT)
 - `CONFIG_APP_IN_CORE`

@note
Examples using the `simple_hal` module in the mesh stack may need to be updated to use the
Nordic nRF5 SDK `bsp` module if integrated with the nRF5 SDK. It is possible to use both, but in
this case `GPIOTE_IRQHandler` must be removed from one of them, and only one of the modules may
register callback functions.

If the original Nordic nRF5 SDK example uses the SoftDevice, make sure that the mesh stack is
initialized and enabled *after* the SoftDevice is enabled. In that case, SoftDevice events must be
forwarded to the mesh stack. Add the following code to your application:
```
#include "nrf_sdh_soc.h"

#define MESH_SOC_OBSERVER_PRIO 0

static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);
```

Flash storage of network configuration is enabled by default in the mesh stack as well as in some of
the Nordic nRF5 SDK applications. The flash areas used for this purpose may overlap and cause
errors. To allow safe coexistence of the flash storage module @ref md_doc_libraries_flash_manager
in the mesh stack and the flash storage module `fstorage` in the Nordic nRF5 SDK, add the following
code block to `nrf_mesh_config_app.h`:
```
#include "fds.h"
#include "fds_internal_defs.h"

#define FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES FDS_PHY_PAGES
```

@note
If you are adding you own mesh functionality rather than working from an existing mesh
example, you also need to add the file nrf_mesh_config_app.h. Copy it from the examples/templates
folder in mesh stack repository into your project folder, and remove \c \#error message at the top
of the file. Make other appropriate changes to the file content,
like adjusting `ACCESS_ELEMENT_COUNT` and `ACCESS_MODEL_COUNT` to the required number of elements
and models.

### nRF5 SDK NVM storage modules @anchor coexistence_mesh_in_nrf5_sdk_nvm
Using nRF5 SDK modules such as `fstorage`, `pstorage`, or `ble_flash` for writing to flash may be
problematic due to long `timeslot` events occupied by the mesh stack. Use the
@ref md_doc_libraries_flash_manager module provided by the mesh stack instead.

Furthermore, when writing to flash, ensure to not write or erase areas utilized by the mesh stack
modules and the bootloader (if present). By default, the mesh modules utilize the last `x` number of
pages before the start of the bootloader, if present, or the last `x` number of pages of the
available flash on the Nordic SoC.
The value of `x` depends on the configuration of the mesh stack and can be calculated by:

    x = 2 + ACCESS_FLASH_PAGE_COUNT + DSM_FLASH_PAGE_COUNT

- `ACCESS_FLASH_PAGE_COUNT` shall be equal to or greater than

        (1 + ((DATA_SIZE) / (FLASH_MANAGER_DATA_PER_PAGE - LARGEST_ENTRY_SIZE)))

  where:
   - `DATA_SIZE` is

         (ALIGN_VAL((sizeof(fm_header_t) + sizeof(access_model_state_data_t)), WORD_SIZE) * ACCESS_MODEL_COUNT) +
         (ALIGN_VAL((sizeof(fm_header_t) + sizeof(access_flash_subscription_list_t)), WORD_SIZE) * ACCESS_SUBSCRIPTION_LIST_COUNT) +
         (ALIGN_VAL((sizeof(fm_header_t) + sizeof(uint16_t)), WORD_SIZE) * ACCESS_ELEMENT_COUNT) +

   - `ALIGN_VAL` returns the total field size aligned to WORD boundaries.

   - `FLASH_MANAGER_DATA_PER_PAGE` is

         (PAGE_SIZE - sizeof(flash_manager_metadata_t))

   - `LARGEST_ENTRY_SIZE` is `ACCESS_MODEL_STATE_FLASH_SIZE`

- `DSM_FLASH_PAGE_COUNT` shall be equal to or greater than

         (1 + ((DATA_SIZE) / (FLASH_MANAGER_DATA_PER_PAGE - LARGEST_ENTRY_SIZE)))

 where:
   - `DATA_SIZE` is

         (ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_unicast_t)), WORD_SIZE) +
         ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_nonvirtual_t)), WORD_SIZE)  * DSM_NONVIRTUAL_ADDR_MAX +
         ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_addr_virtual_t)), WORD_SIZE)     * DSM_VIRTUAL_ADDR_MAX +
         ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_subnet_t)), WORD_SIZE)           * DSM_SUBNET_MAX +
         ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_devkey_t)), WORD_SIZE)           * DSM_DEVICE_MAX +
         ALIGN_VAL((sizeof(fm_header_t) + sizeof(dsm_flash_entry_appkey_t)), WORD_SIZE)           * DSM_APP_MAX)


   - `FLASH_MANAGER_DATA_PER_PAGE` is

         (PAGE_SIZE - sizeof(flash_manager_metadata_t))

   - `LARGEST_ENTRY_SIZE` is

         (sizeof(fm_header_t) + sizeof(dsm_flash_entry_t))

### Estimated sizes @anchor coexistence_mesh_in_nrf5_sdk_sizes

The following are estimated sizes based on the
[Light switch server example](@ref md_examples_light_switch_README),
built using Keil v5 with optimization level `O3` for nRF52832.

**Definitions**
|Definition                      |Value|
|:-------------------------------|----:|
|`ACCESS_MODEL_COUNT`            | 3   |
|`ACCESS_SUBSCRIPTION_LIST_COUNT`| 1   |
|`ACCESS_ELEMENT_COUNT`          | 1   |
|`DSM_NONVIRTUAL_ADDR_MAX`       | 3   |
|`DSM_VIRTUAL_ADDR_MAX`          | 1   |
|`DSM_SUBNET_MAX`                | 1   |
|`DSM_DEVICE_MAX`                | 1   |
|`DSM_APP_MAX`                   | 1   |

**Base sizes**
| Structure/union                   | Size in bytes |
|:--------------------------------- |--------------:|
|`fm_header_t`                      |     4         |
|`access_model_state_data_t`        |    20         |
|`access_flash_subscription_list_t` |     4         |
|`dsm_local_unicast_address_t`      |     4         |
|`dsm_flash_entry_addr_nonvirtual_t`|     2         |
|`dsm_flash_entry_addr_virtual_t`   |    16         |
|`dsm_flash_entry_subnet_t`         |    36         |
|`dsm_flash_entry_devkey_t`         |    20         |
|`dsm_flash_entry_appkey_t`         |    36         |
|`flash_manager_metadata_t`         |     8         |
|`dsm_flash_entry_t`                |    36         |

**Results**
| Count name                | Value |
|:--------------------------|------:|
|`ACCESS_FLASH_PAGE_COUNT`  |     1 |
|`DSM_FLASH_PAGE_COUNT`     |     1 |
|Total page count           |     4 |


