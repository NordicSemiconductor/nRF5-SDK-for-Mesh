# Migration guide

This migration guide is complementary to the @ref md_RELEASE_NOTES.
It describes practical actions you must take to update your code to a newer
version of the nRF5 SDK for Mesh.

**Table of contents**
- [Migration from v3.0.0 to v3.1.0](@ref md_doc_migration_3_0_0_to_3_1_0)
    - [Example UUIDs updated](@ref migration_310_uuid_update)
    - [PB-ADV bearer selection define change](@ref migration_310_pb-adv_change)
    - [Error checking on OOB input data](@ref migration_310_oob_error_checking)
    - [Generic Power OnOff API simplified](@ref migration_310_onoff_api)
- [Migration from v2.2.0 to v3.0.0](@ref md_doc_migration_2_2_0_to_3_0_0)
- [Migration from v2.0.1 to v2.2.0](@ref md_doc_migration_2_0_1_to_2_2_0)
- [Migration from v1.0.1 to v2.0.0](@ref md_doc_migration_1_0_1_to_2_0_0)


---

## Migration from v3.0.0 to v3.1.0 @anchor md_doc_migration_3_0_0_to_3_1_0

Read this migration guide together with the [nRF5 SDK for Mesh v3.1.0 release notes](@ref release_notes_310).

### Example UUIDs updated @anchor migration_310_uuid_update

- Removed `mesh_app_uuid_gen()` API to prevent use of non RFC4122 compliant UUIDs.
- Updated `nrf_mesh_configure_device_uuid_reset()` API to make generated UUID compliant with
UUID version 4 as per RFC4122.

#### Required actions
- If your systems rely on the old UUIDs, specify the old UUID in the @ref mesh_stack_init_params_t structure
during stack initialization before updating the firmware of such devices.
- Use @ref mesh_app_uuid_print() API to print the UUIDs in a standard format. When printed as array of bytes,
the UUID may not be understandable by systems that use the string representation of the UUID for parsing.

### PB-ADV bearer selection define change @anchor migration_310_pb-adv_change

The `MESH_PROVISIONEE_BEARER_ADV_ENABLED` define has been replaced by @ref MESH_FEATURE_PB_ADV_ENABLED.
Setting `MESH_FEATURE_PB_ADV_ENABLED=1` (which replaces `MESH_PROVISIONEE_BEARER_ADV_ENABLED=1`) will no longer cause compilation error.

#### Required actions

Make sure that your code reflects this change to avoid errors. 

### Error checking on OOB input data @anchor migration_310_oob_error_checking
  
The interface for providing provisioning authentication data @ref nrf_mesh_prov_auth_data_provide() will now do stricter error checking on OOB input data.
This ensures that the inputs are better tested and sanitized.

#### Required actions

Make sure that your usage of the API is still valid.

### Generic Power OnOff API simplified @anchor migration_310_onoff_api

The API of Generic Power OnOff client initialization has been simplified. The @ref generic_ponoff_client_init function doesn't call @ref generic_onoff_client_init anymore.

#### Required actions

Manually extend the Generic Power OnOff Client with the Generic OnOff model. To do this, call the @ref generic_onoff_client_init function
and share subscription list between Generic OnOff and Generic Power OnOff clients by calling the @ref access_model_subscription_lists_share function.


---


## Migration from v2.2.0 to v3.0.0 @anchor md_doc_migration_2_2_0_to_3_0_0

Read this migration guide together with the [nRF5 SDK for Mesh v3.0.0 release notes](@ref release_notes_300).

**Table of contents**
- [Added and removed files](@ref migration_300_added_removed)
- [IRQ priority level change](@ref migration_300_irq_priority)
- [Stack separation on high frequency and low frequency time domains](@ref migration_300_stack_separation)
- [Initialization and start phase changes](@ref migration_300_initialization)
- [Only one segmented message from an element to a destination address at a time](@ref migration_300_segmented_messages)
- [Device compile time configuration changes](@ref migration_300_compile_time)
- [New provisioning parameter](@ref migration_300_provisioning)


### Added and removed files @anchor migration_300_added_removed

The following files have been added in the v3.0.0 release:
- LPN feature:
	- `mesh/core/src/core_tx_lpn.c`
	- `mesh/core/src/lpn.c`
	- `mesh/core/src/mesh_lpn_subman.c`
- Core:
	- `mesh/core/src/mesh_mem_stdlib.c`
	- `mesh/core/src/timeslot_timer.c`
	- `external/app_timer/app_timer_mesh.c` ([details below](@ref migration_300_stack_separation))
- Common example files:
	- `examples/common/src/app_level.c`
	- `examples/common/src/ble_softdevice_support.c` ([details below](@ref migration_300_initialization))

The following files have been removed in the v3.0.0 release:
- `examples/common/src/mesh_softdevice_init.c` ([details below](@ref migration_300_initialization))

#### Required actions
Update your projects accordingly to include or remove these files.


### IRQ priority level change @anchor migration_300_irq_priority

The integration with the nRF5 SDK v15.2 caused the following changes:
- The function `softdevice_irq_priority_checker()` has been removed from `mesh_app_utils.h`.
- @ref NRF_MESH_IRQ_PRIORITY_LOWEST has been changed from 7 to 6, which corresponds
to @link_app_irq_priority_low.
- The `nrf_sdh.c` replacement has been removed.

#### Required actions
- Go back to using `nrf_sdh.c` in the nRF5 SDK, as the IRQ level fix was upstreamed.
- Make sure that mesh API is called at the same IRQ priority level as it is specified
in the configuration.


### Stack separation on high frequency and low frequency time domains @anchor migration_300_stack_separation

Stack has been divided into two time frequency domains:
- High-frequency domain, which is based on the timeslot structure of Soft Device.
Low layers of the stack (advertiser, scanner, bearer etc) are based in this domain to be able
to fit their activities within the provided timeslot.
- Low frequency domain, which is based on app_timer functionality.
app_timer is based on RTC with low frequency source clock (which works independently from MCU clock).
All components above bearer are based in this domain. The reason for this change is that low-power nodes need to
put the MCU to sleep for long periods of time between actions. The slower clock can then wake up the MCU right before it is needed.

Additionally, app_timer API can be used in an application as-is. However, the SDK app_timer abilities are not sufficient
for managing the stack functionality. Extended app_timer with modifications has been added in `external/app_timer`.
Modified version will be compiled and linked to applications (based on stack build system) instead of SDK app_timer.
SDK app_timer API header file should be used as well.

#### Required actions
The following define that turns on `app_timer` in the building process must be added to `app_config.h` to make these changes work:

		#define APP_TIMER_ENABLED 1




### Initialization and start phase changes @anchor migration_300_initialization

In the initialization phase of the examples, the `mesh_softdevice_init` module has been replaced with the @ref BLE_SOFTDEVICE_SUPPORT.

In the start phase of the examples, the `execution_start()` function has been removed.

#### Required actions
Use the function @ref ble_stack_init() to initialize the SoftDevice Handler and the BLE stack,
and to register the Mesh handler for SoC events.

Make sure the function @ref mesh_stack_start() is called at the end of the start phase
(see `start()` function in the Light Switch Server code as an example). After calling @ref mesh_stack_start(), all mesh API interaction
must happen at the IRQ priority level specified in the call to @ref mesh_stack_init(). Calling mesh functions at wrong IRQ priority levels
after enabling the stack can cause unpredictable behavior (see @ref md_doc_introduction_mesh_interrupt_priorities for details).

### Only one segmented message from an element to a destination address at a time @anchor migration_300_segmented_messages

The nRF Mesh SDK is now enforcing the Mesh Profile Specification v1.0 rule that disallows multiple
simulatneous segmented messages from a source address to the same destination.

This change means that models must wait until their previous segmented message is finished sending
before they can send another. If multiple models on the same element have the same publish address,
they have to coordinate their publishing as well.

If the application attempts to publish two simultaneous segmented messages with the same source
and destination, the second publish call gets rejected with error code `NRF_ERROR_INVALID_STATE`.

#### Required actions
Models that previously scheduled multiple segmented messages in the same context must
instead subscribe to @ref NRF_MESH_EVT_TX_COMPLETE events or set up timers for sending to avoid
the error code.

@note
This change only affects segmented messages, that is messages with @ref access_message_tx_t::force_segmented set to
`true` or messages that are longer than @ref NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX.
Unsegmented messages can be interleaved with both segmented and unsegmented messages of any source
and destination.


### Device compile time configuration changes @anchor migration_300_compile_time

Compile time configuration for supported features of the device changed,
which makes `DEVICE_FEATURES`, `MESH_FEATURE_GATT`, `GATT_PROXY` macros outdated.

#### Required actions
Make sure you apply the following changes to your code:
- Use @ref MESH_FEATURE_PB_GATT_ENABLED macro instead of `MESH_FEATURE_GATT`.
- Use @ref MESH_FEATURE_GATT_PROXY_ENABLED macro instead of `GATT_PROXY`.
- Use @ref MESH_FEATURE_RELAY_ENABLED, @ref MESH_FEATURE_LPN_ENABLED and
@ref MESH_FEATURE_GATT_PROXY_ENABLED macros to enable or disable required feature
instead of `DEVICE_FEATURES`.


### New provisioning parameter @anchor migration_300_provisioning

There is a new parameter `attention_duration_s` for the @ref nrf_mesh_prov_provision() function in `mesh/prov/api/nrf_mesh_prov.h`.
This parameter is required in the provisioning process.

#### Required actions
Update all uses of the @ref nrf_mesh_prov_provision() function with the additional `attention_duration_s` argument.


---

## Migration from v2.0.1 to v2.2.0 @anchor md_doc_migration_2_0_1_to_2_2_0

### New source files
The following source files have been added in this release:

#### Core
- mesh/core/src/mesh_config.c
- mesh/core/src/mesh_config_backend.c
- mesh/core/src/mesh_config_flashman_glue.c
- mesh/core/src/mesh_opt.c

#### Models
- models/model_spec/commn/src/model_common.c

### Removed source files
The following source files have been removed in this release:

#### Core
- mesh/core/src/ticker.c

### Models

- Updated model directory structure:
    - Foundation models have been moved to models/foundation:
        - Configuration model moved from `models/config` to `models/foundation/config`
        - Health model moved from `models/health` to `models/foundation/health`
    - Generic models are present in models/model_spec:
        - Common functionality used by the Mesh models: `models/model_spec/common`
        - Generic Default Transition Time model: `models/model_spec/generic_dtt`
        - Generic Level model: `models/model_spec/generic_level`
        - Generic OnOff model: `models/model_spec/generic_onoff`
    - Vendor-specific models have been moved to models/vendor:
        - Simple OnOff model moved from `models/simple_on_off` to `models/vendor/simple_on_off`
    - Experimental models have been moved to models/experimental:
        - Provisioning over Mesh model moved from `models/pb_remote` to `models/experimental/pb_remote`
- The `light_switch` examples and `enocean` example have been updated to use Generic OnOff models.

#### Backwards-compatible changes to model API

##### simple_on_off_client.h

- `::SIMPLE_ON_OFF_CLIENT_ACKED_TRANSACTION_TIMEOUT` macro definition added.

##### pb_remote_client.h

- `::PB_REMOTE_CLIENT_ACKED_TRANSACTION_TIMEOUT` macro definition added.

##### pb_remote_server.h

- `::PB_REMOTE_SERVER_ACKED_TRANSACTION_TIMEOUT` macro definition added.

##### health_client.h

- `::HEALTH_CLIENT_ACKED_TRANSACTION_TIMEOUT` macro definition added.

##### config_client.h

- `::CONFIG_CLIENT_ACKED_TRANSACTION_TIMEOUT` macro definition added.

### Use of the nRF5 SDK's section variables

As of v2.2.0, the nRF5 SDK for Mesh makes use of the nRF5 SDK's section variable module
@link_section_vars.

The required changes for supporting section variables are already in place in the example applications,
but any user applications carried over from the previous release might require modifications to work
correctly, depending on the toolchain used:

#### Segger Embedded Studio

The Segger Embedded Studio projects base their section placement on the bundled `flash_placement.xml` file.
The example projects in version 2.2.0 contain updated flash_placement files. If you did not do any modifications
to this file in your migrated project, you can safely replace the existing flash_placement file with
the one in CMake/SES/flash_placement.xml.

If you modified the file, perform the migration manually by adding the following ProgramSection to the `FLASH` MemorySegment:

```xml
<ProgramSection alignment="4" keep="Yes" load="Yes" name=".nrf_mesh_flash"  inputsections="*(SORT(.nrf_mesh_flash.*))" address_symbol="__start_nrf_mesh_flash" end_symbol="__stop_nrf_mesh_flash"/>
```

and the following ProgramSection to the `RAM` MemorySegment:

```xml
<ProgramSection alignment="4" keep="Yes" load="No" name=".nrf_mesh_ram"  inputsections="*(SORT(.nrf_mesh_ram.*))" address_symbol="__start_nrf_mesh_ram" end_symbol="__stop_nrf_mesh_ram"/>
```

Restart Segger Embedded Studio after saving your changes and clean the solution before rebuilding (Build->Clean Solution).


#### GCC

When building with GCC, the section variables must be registered in the application's linker script
so that they end up in the right memory area.

All section variables used by mesh go into two new memory sections (one in RAM and one in
flash). These memory sections must be added to the linker script.

The `nrf_mesh_ram`-section must be added to the sections marked with `INSERT AFTER .data`:

```
SECTIONS
{
  . = ALIGN(4);

  /* ...Any other sections */

  .nrf_mesh_ram :
  {
    PROVIDE(__start_nrf_mesh_ram = .);
    KEEP(*(SORT(.nrf_mesh_ram.*)))
    PROVIDE(__stop_nrf_mesh_ram = .);
  } > RAM
} INSERT AFTER .data
```

The `nrf_mesh_flash`-section must be added to the sections marked with `INSERT AFTER .text`:

```
SECTIONS
{
  . = ALIGN(4);

  /* ...Any other sections */

  .nrf_mesh_flash :
  {
    PROVIDE(__start_nrf_mesh_flash = .);
    KEEP(*(SORT(.nrf_mesh_flash.*)))
    PROVIDE(__stop_nrf_mesh_flash = .);
  } > FLASH
} INSERT AFTER .text
```

#### ARMCC

When building with ARMCC, no further action is required.


## Mesh runtime options

The mesh runtime options API (@ref NRF_MESH_OPT) has been deprecated in favor of @ref MESH_OPT.
The new API uses the new mesh_config module to store options persistently, so once set,
the options do not have to be reset for the lifetime of the device.

The following options have been migrated to the new API:

| Old option                                               | New option
|----------------------------------------------------------|------------------------------------------------
| `NRF_MESH_OPT_PROV_ECDH_OFFLOADING`                      | @ref mesh_opt_prov_ecdh_offloading_set
| `NRF_MESH_OPT_NET_RELAY_ENABLE`                          | @ref mesh_opt_core_adv_set
| `NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT`                | @ref mesh_opt_core_adv_set
| `NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS`          | @ref mesh_opt_core_adv_set
| `NRF_MESH_OPT_NET_RELAY_TX_POWER`                        | @ref mesh_opt_core_tx_power_set
| `NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT`                | @ref mesh_opt_core_adv_set
| `NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS`          | @ref mesh_opt_core_adv_set
| `NRF_MESH_OPT_NET_NETWORK_TX_POWER`                      | @ref mesh_opt_core_tx_power_set

Note that the transport-layer options are still only present on the old nrf_mesh_opt API. They
will be migrated in the next release.


---

## Migration from v1.0.1 to v2.0.0 @anchor md_doc_migration_1_0_1_to_2_0_0

This guide describes the migration process from nRF5 SDK for Mesh version 1.0.1 to version 2.0.0.

### Initialization

In v1.0.1, the `nrf_mesh_sdk` and `nrf_mesh_node_config` modules could both be used for initializing
a mesh device, and they had partly overlapping functionality. They have now been replaced by a
single set of non overlapping modules:

- `mesh_stack`: Functions for initializing and starting all mesh stack modules, including the
  foundation models.
- `mesh_softdevice_init`: Function for initializing the SoftDevice.
- `mesh_provisionee`: Simple provisionee support module containing the provisioning related parts of
  the old `nrf_mesh_node_config`, using PB-ADV and static OOB authentication. It is mainly intended
  to be used by the nRF5 SDK for Mesh examples, but it can also be used as a starting point for
  provisioning in a user application.

The initialization code of the examples has now been restructured into an initialization phase and a
start phase, following the same pattern as the nRF5 SDK examples. We strongly advise user
applications to do the same. We have also added an `execution_start()` function to avoid the risk of
race conditions during the start phase.

### Error handling

Following the same pattern as the nRF5 SDK, weak implementations of `app_error_fault_handler`
(in `examples/common/src/app_error_weak.c`) and `mesh_assertion_handler`
(in `examples/common/src/assertion_handler_weak.c`) have been provided. Instead of installing error
handlers as callbacks, these functions are now used directly by the nRF5 SDK for Mesh error
handling. They can be overloaded by the user application if needed.

### nRF5 SDK

Files from the nRF5 SDK are no longer included as part of the nRF5 SDK for Mesh release. Instead,
the user is supposed to download the nRF5 SDK separately and provide a link to the nRF5 SDK
folder when building the mesh code. See [Retrieve nRF SDK](@ref how_to_build_nrf_sdk) for more
details.

### Non backwards compatible API changes

These are API changes that will break an existing application if ignored.

#### access.h

- `access_message_tx_t` has two new fields: `force_segmented` and `transmic_size`.
- `access_message_rx_t` now reports richer low-level metadata for incoming packets, replacing
  `meta_data.timestamp` and `meta_data.rssi` with a pointer to a `nrf_mesh_rx_metadata_t` structure.

#### access_reliable.h

- `access_reliable_status_t`: New enum value `ACCESS_RELIABLE_TRANSFER_CANCELLED`.

#### device_state_manager.h

- `dsm_beacon_secmat_get()` renamed to `dsm_beacon_info_get()`, and type of last parameter has
  changed from `nrf_mesh_beacon_secmat_t` to `nrf_mesh_beacon_info_t`.

#### nrf_mesh.h

- `nrf_mesh_init_params_t` (now included in `mesh_stack_init_params_t`): `assertion_handler` field
  removed, `p_uuid` field added.

#### nrf_mesh_events.h

- `nrf_mesh_sar_session_cancel_reason_t` (included in `nrf_mesh_evt_t`): New enum value
  `NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION`.

#### nrf_mesh_prov.h

- `nrf_mesh_prov_listen()`: `bearer_type` parameter changed to `bearer_types`, which is a bitfield
  for the supported bearers. That means that if multiple bearers are supported, you can listen for
  incoming provisioning requests on all of them at the same time.

#### nrf_mesh_prov_events.h

- `nrf_mesh_prov_evt_complete_t`: Contains a pointer to a `nrf_mesh_prov_provisioning_data_t`
  structure now instead of a copy of the provisioning data.

#### config_client.h:

- `config_client_event_type_t`: New enum value `CONFIG_CLIENT_EVENT_TYPE_CANCELLED`.
- `config_client_composition_data_get()`: Parameter `page_number` added.

#### health_client.h:

- `health_client_evt_type_t`: New enum value `HEALTH_CLIENT_EVT_TYPE_CANCELLED`.

#### simple_on_off_client.h

- `simple_on_off_status_t`: New enum value `SIMPLE_ON_OFF_STATUS_CANCELLED`.

#### nrf_mesh_config_app.h

- File moved from the `mesh/app` folder to the `examples/templates` folder.

### Backwards compatible API changes

These changes are API additions that will not break an existing application if ignored.

#### device_state_manager.h

- `dsm_appkey_handle_to_subnet_handle()` function added.

#### nrf_mesh.h

- `nrf_mesh_subnet_added()` function added.

#### nrf_mesh_events.h

- `nrf_mesh_evt_t` / `nrf_mesh_evt_type_t`: New event types `NRF_MESH_EVT_NET_BEACON_RECEIVED` and
  `NRF_MESH_EVT_FLASH_STABLE`.
- `nrf_mesh_evt_iv_update_notification_t`: `p_network_id` field added.
- `nrf_mesh_evt_key_refresh_notification_t`: `p_network_id` field added.

#### config_client.h:

- `config_client_pending_msg_cancel()` function added.

#### health_client.h:

- `health_client_pending_msg_cancel()` function added.

#### simple_on_off_client.h

- `simple_on_off_client_t`: New field `timeout_cb` added.
- `simple_on_off_client_pending_msg_cancel()` function added.

#### simple_on_off_server.h

- `simple_on_off_server_status_publish()` function added.

#### access_config.h

- `access_model_publication_stop()` function added.
- `access_model_publication_by_appkey_stop()` function added.