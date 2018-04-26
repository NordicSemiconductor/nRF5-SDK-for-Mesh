# Coexistence with other Nordic SDKs

**NOTE:**
* nRF5 SDK coexistence is only tested with nRF5 SDK version 14.2.
* nRF5 SDK version 14.2 does not support nRF51.

## nRF5 SDK integration

If you plan to use an example from the nRF5 SDK in combination with the mesh stack, move the mesh stack repository into the Nordic nRF5 SDK examples folder and include the necessary source files in the Nordic nRF5 SDK project that you want to use.

To do so, make the following changes:
 - Apply the patches supplied in the mesh stack's external/nRF5_SDK_14.2.0_17b948a:
    - Start a Git Bash shell
    - Change the working directory to the top level folder inside your SDK installation
    - Run the command `patch -p3 < ../nRF5_SDK_14.2.0_17b948a.patch` (modify to point to your actual patch file location)
 - Remove the external/nRF5_SDK_14.2.0_17b948a folder from the mesh stack folder.
 - Add all `core`, `prov`, `access`, `dfu`, `nrf_mesh_weak.c`, `external/micro-ecc`, relevant `model` and `example` files, and (if used) `serial` files and their include paths to the Nordic nRF5 SDK project.
 - If the Nordic nRF5 SDK application already uses the SoftDevice, remove the following functions and variables from the `nrf_mesh_sdk` module (they will be replaced by corresponding functionality in the nRF5 SDK):
    - `app_error_handler`
    - `HardFault_Handler`
    - `SD_EVT_IRQHandler`
    - `softdevice_assert_handler`
    - `mesh_softdevice_enable`
    - `mesh_softdevice_setup` (including the call to it in `mesh_core_setup`)
    - `m_ble_evt_buffer`
 - Define `NRF_MESH_LOG_ENABLE` to be ` NRF_LOG_USES_RTT` in `nrf_mesh_config_core.h`, because the logging in the mesh stack relies on RTT.
 - Examples using the module `simple_hal` in the mesh stack may need to be updated to use the Nordic nRF5 SDK module `bsp`. It is possible to use both, but in this case `GPIOTE_IRQHandler` must be removed from one of the files and only one of the modules may register callback functions.
 - If the original Nordic nRF5 SDK example uses the SoftDevice, make sure that the mesh stack is initialized and enabled after the SoftDevice is enabled. In that case SoftDevice events must be forwarded to the mesh stack (see the *SoftDevice events* section for more information). Add the following defines and functions:
```
#include "nrf_sdh_soc.h"

#define MESH_SOC_OBSERVER_PRIO <set priority, see nRF5 SDK documentation for details>

static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}
```
 Then add the following code to the mesh stack initialization in the application's main.c:
```
    NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);
```
 - Flash storage of network configuration is enabled by default in the mesh stack as well as in some of the Nordic nRF5 SDK applications. The flash areas used for this purpose may overlap and cause errors. To allow safe coexistence of the flash storage module @ref md_doc_libraries_flash_manager in the mesh stack and the flash storage module `fstorage` in the Nordic nRF5 SDK, add the following code block to `nrf_mesh_config_core.h`:
```
#include "fds.h"
#include "fds_internal_defs.h"

static inline uint32_t fs_page_end_addr(void)
{
   uint32_t bootloader_addr = NRF_UICR->NRFFW[0];
   return (bootloader_addr != 0xFFFFFFFFU) ?
          bootloader_addr : (NRF_FICR->CODESIZE * NRF_FICR->CODEPAGESIZE);
}

#define FLASH_MANAGER_RECOVERY_PAGE ((uint32_t) fs_page_end_addr() - (FDS_PHY_PAGES + 1) * (FDS_PHY_PAGE_SIZE * 4))
```
 - Copy nrf_mesh_config_app.h from mesh/app folder in mesh stack repository into your project folder. Remove `#error` message in top of file. Make other appropriate changes to file content, like adjusting ACCESS_ELEMENT_COUNT and ACCESS_MODEL_COUNT to the required number of elements and models.
 - Add NRF52_SERIES to list of preprocessor symbols passed to the compiler.
 - Add SD_BLE_API_VERSION to list of preprocessor symbols passed to the compiler, and set it to the same value as NRF_SD_BLE_API_VERSION.

If there is no particular nRF5 SDK example that you want to start with, but instead you want to use some of the nRF5 SDK modules, it might be easier to move the respective nRF5 SDK modules into the mesh stack repository and create an example project using one of the methods provided by the mesh stack repository.

### nRF5 SDK NVM storage modules
Using nRF5 SDK modules such as `fstorage`, `pstorage`, or `ble_flash` for writing to flash may be problematic due to long `timeslot` events occupied by the mesh stack. Use the @ref md_doc_libraries_flash_manager module provided by the mesh stack instead.

Furthermore, when writing to flash, ensure to not write or erase areas utilized by the mesh stack modules and the bootloader (if present). By default, the mesh modules utilize the last `x` number of pages before the start of the bootloader, if present, or the last `x` number of pages of the available flash on the Nordic SoC. The value of `x` depends on the configuration of the mesh stack and can be calculated by:
```
x = 1 + ACCESS_FLASH_PAGE_COUNT + DSM_FLASH_PAGE_COUNT + NET_FLASH_PAGE_COUNT
```

## SoftDevice
### SoftDevice events
The mesh stack relies on receiving the events generated by the SoftDevice for proper functioning. The SoftDevice may generate events relevant only to the application, only to the mesh stack, or both for application and the mesh stack. Therefore, it is important that the SoftDevice events reported via `SD_EVT_IRQHandler` and extracted via `sd_evt_get` are processed by the mesh stack as well as the relevant application handlers.

### Concurrent SoftDevice and mesh activity
By design, the SoftDevice activity is prioritized over mesh activity. Therefore, you should keep the connection and advertisement intervals used by the SoftDevice as large as possible (i.e. infrequent) when using Bluetooth low energy connections. If scanning, keep the scan duty cycle as low as possible. You should also reduce mesh activity while the SoftDevice is doing fast advertising and continue normal activity after a connection is established.
