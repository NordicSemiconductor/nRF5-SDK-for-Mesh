# Release Notes

This page contains all the notes for the major and minor production releases of the nRF5 SDK for Mesh.

**Table of contents**
- [BLE Mesh v5.0.0](@ref release_notes_500)
    - [New features and highlights](@ref release_notes_500_highlights)
    - [Changes](@ref release_notes_500_changes)
    - [Bugfixes](@ref release_notes_500_bugfixes)
    - [Limitations](@ref release_notes_500_limitations)
    - [Known issues](@ref release_notes_500_known_issues)
- [BLE Mesh v4.2.0](@ref release_notes_420)
- [BLE Mesh v4.1.0](@ref release_notes_410)
- [BLE Mesh v4.0.0](@ref release_notes_400)
- [BLE Mesh v3.2.0](@ref release_notes_320)
- [BLE Mesh v3.1.0](@ref release_notes_310)
- [BLE Mesh v3.0.0](@ref release_notes_300)
- [BLE Mesh v2.2.0](@ref release_notes_220)
- [BLE Mesh v2.1.1](@ref release_notes_211)
- [BLE Mesh v2.0.0](@ref release_notes_200)
- [BLE Mesh v1.0.1](@ref release_notes_101)
- [BLE Mesh v1.0.0](@ref release_notes_100)
- [BLE Mesh v0.10.1-Alpha](@ref release_notes_0101)
- [BLE Mesh v0.10.0-Alpha](@ref release_notes_0100)
- [BLE Mesh v0.9.2-Alpha](@ref release_notes_092)
- [BLE Mesh v0.9.1-Alpha](@ref release_notes_091)
- [BLE Mesh v0.9.0](@ref release_notes_090)
- [BLE Mesh v0.8.1](@ref release_notes_081)
- [BLE Mesh v0.8.0](@ref release_notes_080)
- [BLE Mesh v0.7.7](@ref release_notes_077)

Check [Migration guide](@ref md_MIGRATION_GUIDE) for mandatory changes to your projects caused by
the release of new features and updates.

## BLE Mesh v5.0.0 @anchor release_notes_500

@par Release Date: Week 47, 2020

This is a major release that introduces Scene Client and Scene Server models with the respective API and example.
It also brings support for the nRF52840 dongle and the nRF52820 device and fixes several bugs.
Because of the updates to nRF5 SDK v17.0.2 and the recompilation of the bootloader binaries, it is recommended to upgrade to this version.

### New features and highlights @anchor release_notes_500_highlights
- Implemented Scene Client and Scene Server models with the respective API and the scene client example.
- Implemented the power-down storage.
- Added support for the nRF52840 dongle.
- Added support for the nRF52820 device.
- Added support for the nRF5 SDK v17.0.2, which introduces the new SoftDevice v7.2.0.

### Changes @anchor release_notes_500_changes
- Removed support for S113 on the nRF52810 device, as the nRF5 SDK for Mesh does not fit on the flash with S113.
- Added support for S112 on the nRF52832 device.
- Added support for more than 32 instances for bearers.
- Expanded serial interface to provide API for setting the IV index and sequence number.

#### Changes to examples

- Added support for the Scene Server model in the following examples:
    - @ref md_examples_light_switch_server_README "Light switch server example"
    - @ref md_examples_dimming_README "Dimming server example"
    - @ref md_examples_light_lightness_README "Light lightness server example"
    - @ref md_examples_light_ctl_README "Light CTL server example"
    - @ref md_examples_light_lc_server_README "Light LC server example"
- Added support for scene client examples to the static provisioner.
- Added support for scene server configuration to the static provisioner for examples that support the Scene Server model.

### Bugfixes @anchor release_notes_500_bugfixes
- Fixed a bug where Power OnOff Setup Server would allow setting up publishing. (MBTLE-4176)
- Fixed a bug where the Low Power node would keep scanner running until it issues the first Friend Poll message. (MBTLE-4168)
- Fixed a bug where repeated Friend Request messages were sent without the required waiting period of 1.1 second. (MBTLE-4165)
- Fixed a bug where Light PI regulator was using incorrect scaling for the accuracy factor during the calculation of the final output. (MBTLE-4158)
- Fixed a bug where the default TTL setting value was not stored in the persistent storage. (MBTLE-4148)
- Fixed a bug where the presence of a new mesh configuration file in a firmware that was updated using DFU could cause assertion when `mesh_config_clear()` API is used during the initialization of the device. (MBTLE-4143).
- Fixed a bug where a non-zero value of the key refresh flag that was received during provisioning process would not be handled. (MBTLE-4115)
- Fixed a bug where the API for setting TX Power would only set the TX power for `core_tx_adv`, and not for LPN and Friend. (MBTLE-4086)
- Fixed a bug where the Generic Power OnOff Setup Server model would not share subscription list with other associated CTL server models. (MBTLE-4077)
- Fixed several other bugs.

#### Known issue fixes
- Fixed a previously known issue where the replay protection list would not be stored in flash. (MBTLE-1975)
- Fixed a previously known issue where the Mesh Configuration Power-down files would require significant time for storing if defragmentation was in progress. (MBTLE-3467)
- Fixed a previously known issue where the Proxy Server would not propagate a locally generated SNB to the Proxy Client when IV update state or key refresh state is changed. (MBTLE-3945)
- Fixed a previously known issue where Sensor Status Delta Trigger would not consider Status Min Interval. (MBTLE-3968)

### Limitations @anchor release_notes_500_limitations
- If a poll is planned during the delay of another poll, the attempt counter is reset to the default
  value. (MBTLE-3175)
- It is not possible to use SAR for both request and response by using loopback. Use separate
  elements for such client-server models instantiated on the same device. (MBTLE-3439)
- The PA/LNA module is supported on nRF52840 and nRF52833, but only verified on nRF52832.
  (MBTLE-3576)

### Known issues @anchor release_notes_500_known_issues
- If the Bluetooth mesh stack is configured with the IRQ priority @ref NRF_MESH_IRQ_PRIORITY_THREAD and runs
  in the main loop with app_scheduler, there might be delays of ~15 ms. (MBTLE-2624)
- During the provisioning of a device, the Device Name is changed to the default `nRF5x` after
  completion of the PB-GATT connection. The name is later changed back to the correct name when the
  initial configuration of the device is complete. (MBTLE-3151)
- The app_timer library fails to trigger the timer correctly if no active timer is present
  for the RTC_MAX/2 value. (A workaround has been implemented for this issue in `app_timer_workaround.c`.) (MBTLE-3347)
- After the firmware upgrade, if the composition data has been changed
  or corrupted persistent data has been found (or both), the device will boot up as unprovisioned device.
  If the device is provisioned at that point without resetting, it asserts.
  As a workaround, reset the device once more before the start of provisioning. (MBTLE-3840)
- The device may assert after the application is started after the DFU, when the application is flashed for the first time.
  This happens when the device has the bootloader and SoftDevice flashed before the booting, but does not have the application flashed at that stage.
  As a workaround, reset the device after the assert.
  (MBTLE-3652)
- An SDK coexistence example will assert at the `peer_manager_init()` function if it is flashed
  on top of a previously provisioned example. As a workaround, erase the device before flashing
  any of the @ref md_examples_sdk_coexist_README. (MBTLE-4182)


---

## BLE Mesh v4.2.0 @anchor release_notes_420

@par Release Date: Week 26, 2020

This is a minor release that introduces sensor client and server models with examples, as well as bugfixes.
Because of updates to DFU and the recompilation of bootloader binaries, it is recommended to upgrade to this version.

### New features and highlights @anchor release_notes_420_highlights
- Implemented Sensor Client and Server model with the respective [API](@ref SENSOR_MODEL) and [examples](@ref md_examples_sensor_README).
- Implemented bugfixes to DFU.

### Changes @anchor release_notes_420_changes
- Added support for friendship poll timeouts greater than 2100 seconds for both Friend and Low Power nodes. This fixes the previously known MBTLE-3563 issue.
- Added a new flashing target in order to enable sector-only erase.
- Added functionality to clean Bluetooth mesh config files. The files are now cleaned as flash pages and not anymore as separate entries.
- Updated Level, Lightness, and CTL behavioral modules. The state variables for ongoing transitions for these modules are not affected until the delay for newly received message expires.
- Modified the common part for models, so that the device will boot as unprovisioned device if the stack detects the wrong model metadata.
- Bootloader binaries were recompiled because of changes and fixes to DFU.
- Made several minor improvements to the documentation pages.
  For example, reorganized sections on the [Building the Bluetooth mesh stack and examples](@ref md_doc_getting_started_how_to_build) page.

#### Example updates
- Added support for sensor server and client examples to the static provisioner.
- Updated the [Light LC example](@ref md_examples_light_lc_server_README) to work with the new sensor example.
    - Removed a pre-compiled hex file for the sensor example used for evaluating the LC server examples. This is now replaced by the Sensor server example.

### Bugfixes @anchor release_notes_420_bugfixes
- Fixed an issue where the timer functionality missed the COMPARE event from RTC1 if CC and COUNTER were close enough. (MBTLE-3811)
- Fixed an issue where setting the maximum values for RelayRetransmitIntervalSteps field in the Config Relay Set message and NetworkTransmitIntervalSteps field in the Config Network Transmit Set message would cause an assertion. (MBTLE-3851)
- Fixed an issue where the provisioner example would assert when attempting to reset the device multiple times. (MBTLE-3855)
- Fixed an issue where the PyACI interface time-out would occur after installing nRF Command Line Tools v10.7.0. (MBTLE-3878)
- Fixed an issue where the Light PI regulator code would wrongly assume that the luxlevel_out value is in "lux" instead of "illuminance" characteristic format. (MBTLE-3879)
- Fixed an issue where the Light ON event would not be generated at the boot time. (MBTLE-3880)
- Fixed an issue with documentation for the RX callback section on the [Beaconing example page](@ref md_examples_beaconing_README) and for the @ref nrf_mesh_rx_cb_set() API. (MBTLE-3881)
- Fixed an issue where a few unit tests would fail when running on Linux. (MBTLE-3933)
- Fixed several other bugs.

#### DFU bugfixes
- Fixed an issue where a PC-connected device not undergoing DFU would not respond to requests for missing packets. (MBTLE-3686)
- Fixed an issue where the DFU functionality would infinitely send or retransmit the state packets within the network. (MBTLE-3687)
- Updated nRF Util for the proprietary mesh DFU in order to add dynamic delay and additional logging messages. This allows nRF Util to wait for sufficient amount of time to allow full erase of bank area for large DFU transfers. (MBTLE-3813, MBTLE-3823)
- Fixed several other bugs.

#### Known issue fixes
- Fixed a previously known issue where the DFU segment recovery may get deadlocked. (MBTLE-3423)

### Limitations @anchor release_notes_420_limitations
- The replay protection list is not stored in flash. (MBTLE-1975)
- If a poll is planned during the delay of another poll, the attempt counter is reset to the default
  value. (MBTLE-3175)
- It is not possible to use SAR for both request and response by using loopback. Use separate
  elements for such client-server models instantiated on the same device. (MBTLE-3439)
- The PA/LNA module is supported on nRF52840 and nRF52833, but only verified on nRF52832.
  (MBTLE-3576)

### Known issues @anchor release_notes_420_known_issues
- If the Bluetooth mesh stack is configured with the IRQ priority @ref NRF_MESH_IRQ_PRIORITY_THREAD and runs
  in the main loop with app_scheduler, there might be delays of ~15 ms. (MBTLE-2624)
- During the provisioning of a device, the Device Name is changed to the default `nRF5x` after
  completion of the PB-GATT connection. The name is later changed back to the correct name when the
  initial configuration of the device is complete. (MBTLE-3151)
- The app_timer library fails to trigger the timer correctly if no active timer is present
  for the RTC_MAX/2 value. (A workaround has been implemented for this issue in `app_timer_workaround.c`.) (MBTLE-3347)
- The Mesh Configuration Power Down files require significant time for storing if defragmentation is
  in progress. (MBTLE-3467)
- After the firmware upgrade, if the composition data has been changed
  or corrupted persistent data has been found (or both), the device will boot up as unprovisioned device.
  If the device is provisioned at that point without resetting, it asserts.
  As a workaround, reset the device once more before the start of provisioning. (MBTLE-3840)
- If Proxy Server changes the SNB beacon state by itself, this change is not propagated to the Proxy Client. (MBTLE-3945)
- Sensor Status Delta Trigger does not consider Status Min Interval. (MBTLE-3968)


---

## BLE Mesh v4.1.0 @anchor release_notes_410

@par Release Date: Week 14, 2020

This is a minor release that introduces support for new models from the Bluetooth mesh model specification
and adds the related examples. It also includes changes to the documentation structure
and fixes several bugs.

### New features and highlights @anchor release_notes_410_highlights
- Added support for new models:
    - Light Lightness server and client models.
    - Light CTL server and client models.
    - Light LC server and client models.
- Added new examples:
    - Light Lightness server and client examples.
    - Light CTL server and client examples.
    - Light CTL LC server example.
    - Light LC server example.

### Changes @anchor release_notes_410_changes
- Updated GNU ARM Embedded Toolchain required minimum version to 9-2019-q4-major.

#### Updated Provisioner
- Added improvements to the static provisioner to include lighting and sensor scenarios.
    - The static provisioner example has been updated to support new examples: Light lightness, Light CTL and Light LC server.
- Additionally, provisioner was moved out of the light switch example dependencies and got separate configuration files.
    - The node configuration module was made flexible to support complex configuration scenarios like the combination of CTL and LC servers.
    Servers from the supported examples now publish statuses on group addresses instead of unicast clients' addresses.
    This allows to combine clients from one example and servers from another example if they support the client-server part of the same model.
        - For example, you can combine the EnOcean switch translator (Generic OnOff client) with the light lightness server (Generic OnOff server), and so on.

#### Documentation
- Changed the structure of the conceptual documentation pages to make it similar to the nRF5 SDK documentation structure.
    - The Getting Started section now includes only the pages that are related to starting the work with the nRF5 SDK for Mesh, including the Migration Guide.
    - The new User Guide section now includes pages about Bluetooth mesh concepts and architecture, libraries, and other pages that are more advanced than the Getting Started pages.
- Added basic information about Bluetooth mesh packet data flow to the Bluetooth mesh stack architecture page.
- Updated the Supported features section on the main overview page and removed the experimental label from some of the pages.

### Bugfixes @anchor release_notes_410_bugfixes
- Fixed an issue where the internal state of the bootloader would not reset after DFU completes. (MBTLE-3223)
- Fixed an issue where PyACI would attempt to print byte arrays as ASCII strings. They are now printed as hexadecimal strings. (MBTLE-3577)
- Fixed an issue where the Generic Power OnOff Server would incorrectly respond to messages containing prohibited values. (MBTLE-3608)
- Fixed an issue related to the config_server event callback not being extended for missing events. (MBTLE-3639)
- Fixed an issue when provisionee would send a wrong reason in the Provisioning Failed command when receiving unexpected values from the provisioner. (MBTLE-3712)
- Fixed an issue where invalid DSM metadata would not be deleted from flash. (MBTLE-3714)
- Fixed an issue where LPN Poll Timeout Get would not get handled by config server if the friendship feature was disabled. (MBTLE-3720)
- Fixed an issue where MESH_CONFIG_STRATEGY_ON_POWER_DOWN would prevent NRF_MESH_EVT_CONFIG_STABLE from being emitted. (MBTLE-3723)
- Fixed an issue where the Access layer data in flash would not be validated. (MBTLE-3739)
- Fixed an issue where setting the publish period to certain values would cause a loss of resolution. (MBTLE-3742)
- Fixed the generated raw UUID to match the UUID version 4 format and updated @ref mesh_app_uuid_print() to
  print the formatted UUID in order to directly match the raw UUID bytes. (MBTLE-3743)
- Fixed an issue where sar_ctx_tx_complete() would not assign a timestamp value. (MBTLE-3774)
- Fixed an issue where the Friend feature support was not set in the composition data. (MBTLE-3820)
- Fixed an issue where the SRC field for the Segmented Acknowledgment message would not be set to the Friend's address when acknowledging a segmented message on behalf of LPN. (MBTLE-3831)
- Fixed outdated links in the SES.md file. (MBTLE-3843)
- Fixed an issue where accessing periodic publication timer would not be stopped after the access layer states are cleared. (MBTLE-3844)
- Fixed an issue where the PB Remote client example would assert in case of incorrect user input. (MBTLE-3848)

#### Known issue fixes
- Fixed a previously known issue where the `flash_manager` API might write to `p_manager->internal.state` without considering the existing state. (MBTLE-1972)
- Fixed a previously known issue where calling @ref access_clear() when a reliable transfer was ongoing would result in an assert. (MBTLE-3181)
- Fixed a previously known issue where the nRF5 SDK for Mesh bootloader would not turn off UART before switching to the application. (MBTLE-3215)
- Fixed a previously known issue where the PA/LNA module in the BLE SoftDevice stack would not be switched on after the Bluetooth mesh provisioning process. (MBTLE-3279)
- Fixed a previously known issue where the DFU example would back out of a DFU target state to service a relay request. (MBTLE-3376)
- Fixed a previously known issue with the Link Open behavior of the Bluetooth mesh stack provisioner. This was fixed by aligning with the specification v1.0.1. (MBTLE-3547)

### Limitations @anchor release_notes_410_limitations
- The replay protection list is not stored in flash. (MBTLE-1975)
- If a poll is planned during the delay of another poll, the attempt counter is reset to the default
  value. (MBTLE-3175)
- It is not possible to use SAR for both request and response by using loopback. Use separate
  elements for such client-server models instantiated on the same device. (MBTLE-3439)
- The PA/LNA module is supported on nRF52840 and nRF52833, but only verified on nRF52832.
  (MBTLE-3576)

### Known issues @anchor release_notes_410_known_issues
- If the mesh stack is configured with the IRQ priority @ref NRF_MESH_IRQ_PRIORITY_THREAD and runs
  in the main loop with app_scheduler, there might be delays of ~15 ms. (MBTLE-2624)
- During the provisioning of a device, the Device Name is changed to the default `nRF5x` after
  completion of the PB-GATT connection. The name is later changed back to the correct name when the
  initial configuration of the device is complete. (MBTLE-3151)
- The app_timer library fails to trigger the timer correctly if no active timer is present
  for the RTC_MAX/2 value. (A workaround has been implemented for this issue in `app_timer_workaround.c`.) (MBTLE-3347)
- The DFU segment recovery may get deadlocked. (MBTLE-3423)
- The Mesh Configuration Power Down files require significant time for storing if defragmentation is
  in progress. (MBTLE-3467)
- Friendship and LPN poll timeouts greater than 2147 seconds are not supported. (MBTLE-3563)
- After the firmware upgrade, if the composition data has been changed
  or corrupted persistent data has been found (or both), the device will boot up as unprovisioned device.
  If the device is provisioned at that point without resetting, it asserts.
  As a workaround, reset the device once more before the start of provisioning. (MBTLE-3840)


---


## BLE Mesh v4.0.0 @anchor release_notes_400

@par Release Date: Week 48, 2019

This is a major release that introduces support for Nordic Semiconductor's nRF52833 device
and adds support for the nRF5 SDK v16.0.0 and the new SoftDevice versions. It includes
the Friendship feature in production-ready state, introduces major changes to the persistent storage
and addresses a number of bugs.

### New features and highlights @anchor release_notes_400_highlights
- Added support for nRF52833.
- Added support for the nRF5 SDK v16.0.0, which introduces the new SoftDevice v7.0.1
and the official support for the SoftDevice S113 v7.0.1.

### Changes @anchor release_notes_400_changes
- Support for the Friendship feature is now production-ready.

#### Persistent storage
- Migrated Access, DSM, and Net State modules, and EnOcean and Light Switch Provisioner examples to use the Bluetooth mesh configuration module.
Each of these components has a separate file, and the files are handled within the Mesh configuration module.
See [Migration guide](@ref mesh_config_for_dsm_and_access) for details about changes required in your application.
    - The following APIs have been removed as part of this change:
        - `bool access_flash_config_load(void)`
        - `void access_flash_config_store(void)`
        - `const void * access_flash_area_get(void)`
        - `bool dsm_flash_config_load(void)`
        - `const void * dsm_flash_area_get(void)`
- Added the Registered Fault state of the Health Model to the persistent storage.

#### Documentation
- Updated the section about provisioning. It is now called @ref md_doc_user_guide_modules_provisioning_main and includes new information about the provisioning process and APIs.
- Added a new page about @ref md_doc_user_guide_examples_adding.
- Added [the list of provisioning bearers in the nRF5 SDK for Mesh examples](@ref example_provisioning_bearers).
- Reworked the sections about [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app).
- Introduced several minor documentation improvements.

#### Other changes
- Added independent, dynamically registered AD listeners for Beacons, Bluetooth mesh core, Provisioning, and DFU.
- Removed the experimental support for the SoftDevice S113 v7.0.0.
- Changed the behavior of how network cache entries are managed. Now, if relaying of a packet fails, the packet is no longer cached by the network layer.
- Increased the default value for @ref MESH_FRIEND_QUEUE_SIZE from 16 to 35.
This allows the Friend node to receive at least one full segmented message meant for the Low Power node.
- Changed the location of the @ref BEARER_EVENT_USE_SWI0 option. See [Migration guide](@ref BEARER_EVENT_USE_SWI0_location) for details about changes required in your application.
- Added heartbeat event handling to the serial interface.

### Bugfixes @anchor release_notes_400_bugfixes
- Fixed an issue where the Bluetooth mesh configuration finds corrupted persistent parameters from Access or DSM files during initialization, which causes the persistent areas to be cleaned and the device start as unprovisioned.
- Fixed an issue where resetting a node might cause the node to wait for IV Update timeout again. Now the time to the next IV Update is stored in the flash each 30 minutes.
- Fixed an issue where it was not possible to set the authentication size to 0x00 for the static OOB method.
- Fixed an issue with the action type of the Input and Output requests in @ref NRF_MESH_PROV_EVT_INPUT_REQUEST and @ref NRF_MESH_PROV_EVT_OUTPUT_REQUEST provisioning events.
    - When the device is now acting as a provisionee, the action field of @ref nrf_mesh_prov_evt_input_request_t contains value from @ref nrf_mesh_prov_input_action_t and the action field of @ref nrf_mesh_prov_evt_output_request_t contains value from @ref nrf_mesh_prov_output_action_t.
    - When the device is now acting as a provisioner, the action field of @ref nrf_mesh_prov_evt_input_request_t contains value from @ref nrf_mesh_prov_output_action_t and the action field of @ref nrf_mesh_prov_evt_output_request_t contains value from @ref nrf_mesh_prov_input_action_t.
- Fixed a log message issue in the access layer when a message was about to be sent. Now the correct buffer will be printed.
- Fixed a bug where the Friend node would establish a friendship with the Low Power node, but would not send the @ref NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED event. Now the Friend node will send the @ref NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED event when establishing a friendship.
- Fixed an issue with an unnecessary parameter check in @ref mesh_friend_friendships_get() that would cause this function to always return `NRF_ERROR_NULL`. The parameter check is now removed.
- Fixed a bug where the Friend node would send the @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED event, but would not fill the role field. Now the Friend node will initialize the role field in the @ref nrf_mesh_evt_friendship_terminated_t structure when sending the @ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED event.
- Fixed a bug where the Friend node was not able to send self-generated messages to the Low Power node.
- Fixed a receive delay timeout calculation issue for the Low Power Node which caused the Low Power node to start the Receive Window ~1.9 ms earlier. Now the Low Power node starts the Receive Window at the proper time.
- Fixed a bug where the Friend node might start receiving segmented messages even if they would not fit into the Friend Queue. Now the Friend node will reject segmented messages that cannot be stored in the Friend Queue due to the limit of the queue size.
- Fixed a bug where the Friend node was not able to decode the Receive Window Factor and RSSI Factor correctly, which would cause the Friend node to schedule the Friend Offer earlier. Now the Friend node will correctly decode the factors and schedule the Friend Offer at the right time.
- Fixed several documentation bugs, including broken links in @ref md_doc_user_guide_modules_dfu_configuring_performing.
- Fixed documentation of the @ref access_model_reliable_publish() function. The user must retain the data provided in @ref access_message_tx_t::p_buffer until the acknowledged transfer has ended (see @ref access_reliable_t::status_cb) or the message is cancelled by @ref access_model_reliable_cancel.
- Fixed a bug where the OOB field in PB-GATT advertisements had the wrong endianness.
- Fixed an issue where @ref pb_remote_server_disable() was always returning `NRF_ERROR_INVALID_STATE`. Now @ref pb_remote_server_disable() performs checks correctly and it is possible to disable the PB remote server.
- Fixed an issue where `BOOTLOADERADDR()` would check `UICR->NRFFW[0]`, which is no longer used in the BLE bootloader, and which would make the `flash_anager_defrag_init()` assert when using BLE bootloader. Now `BOOTLOADERADDR()` does not perform this check.
- Fixed an issue where the internal state of stored entries could brake if an entry was deleted or when it was changed but not stored yet.
- Fixed an issue where the PR remote client or server might lose a message, which might cause the remote provisioning to fail. Now the PB remote models are tolerant to the packets loss between the PB remote client and the PB remote server.
- Fixed an issue where the number of elements reported by the composition data would not match the amount reported in the provisioning capabilities message, leading to a mismatch in the number of elements.

### Limitations @anchor release_notes_400_limitations

- The replay protection list is not stored in flash. (MBTLE-1975)
- When using the GNU ARM Embedded Toolchain `8-2018-q4-major` release, building with CMake on Windows is not working correctly because of a bug in this release. (MBTLE-3105)
- If a poll is planned during the delay of another poll, the attempt counter is reset to the default value. (MBTLE-3175)
- It is not possible to use SAR for both request and response by using loopback. Use separate elements for such client-server models instantiated on the same device. (MBTLE-3439)
- When the provisioning is started by the Bluetooth mesh stack provisioner, the Link Open message will be sent for a maximum of 6 times with an interval of about 60 ms (instead of resending it continuously for up to 60 seconds). If a link could not be established within this time, the application should start the procedure again. (MBTLE-3547)
- The PA/LNA module is supported on nRF52840 and nRF52833, but only verified on nRF52832. (MBTLE-3576)

### Known issues @anchor release_notes_400_known_issues
- The `flash_manager` API might write to `p_manager->internal.state` without considering the existing state. (MBTLE-1972)
- If the Bluetooth mesh stack is configured with the IRQ priority @ref NRF_MESH_IRQ_PRIORITY_THREAD and runs in the main loop with app_scheduler, there might be delays of ~15 ms. (MBTLE-2624)
- During the provisioning of a device, the Device Name is changed to the default `nRF5x` after completion of the PB-GATT connection. The name is later changed back to the correct name when the initial configuration of the device is complete. (MBTLE-3151)
- Calling access_clear() when a reliable transfer is ongoing will result in an assert. (MBTLE-3181)
- The nRF5 SDK for Mesh bootloader does not turn off UART before switching to the application. (MBTLE-3215)
- The PA/LNA module in the BLE SoftDevice stack is not switched on after the Bluetooth mesh provisioning process. (MBTLE-3279)
- The app_timer library fails to trigger the timer correctly if no active timer is present for the RTC_MAX/2 value. (MBTLE-3347)
- The DFU example will back out of a DFU target state to service a relay request. (MBTLE-3376)
- The DFU segment recovery may get deadlocked. (MBTLE-3423)
- The Mesh Configuration Power Down files require significant time for storing if defragmentation is in progress. (MBTLE-3467)
- Friendship and LPN poll timeouts greater than 2147 seconds are not supported. (MBTLE-3563)


---

## BLE Mesh v3.2.0 @anchor release_notes_320

@par Release Date: Week 30, 2019

This is a minor feature release. It adds Friend feature support, the RSSI monitoring model,
support for nRF5 SDK v15.3.0, significant changes to the DFU documentation,
and fixes to several bugs.

### New features and highlights @anchor release_notes_320_highlights
- Added an experimental support for the Friend feature.
    - The Friend feature is enabled with the @ref MESH_FEATURE_FRIEND_ENABLED define.
    - It is enabled by default for the Light switch server example.
    - For the list of the most relevant changes introduced with this feature, see
    [Changes > Friendship feature](@ref release_notes_320_changes).
- Added the vendor-specific [RSSI monitoring model](@ref md_models_vendor_rssi_monitor_README).
- Added experimental support for the S113 SoftDevice.
- Added support for the nRF5 SDK v15.3.0 and the GNU ARM Embedded Toolchain v7.3.1.

### Changes @anchor release_notes_320_changes

#### Documentation
- Added a new @ref md_doc_user_guide_modules_dfu_protocol subsection in the @ref md_doc_user_guide_modules_modules_main section.
    - Moved the page about configuring DFU from Getting started to the new section.
    It is now called @ref md_doc_user_guide_modules_dfu_configuring_performing and has received several updates.
    - Added new pages about the DFU protocol, @ref md_tools_dfu_README, @ref md_doc_user_guide_modules_dfu_integrating_into_app,
    and @ref md_doc_user_guide_modules_dfu_packet_format.
    - Rearranged the existing information about DFU due to the addition of new pages.
    - Reviewed and updated the existing DFU documentation.
- Updated the documentation for @ref md_doc_user_guide_mesh_interrupt_priorities.
  It now mentions how to use the `app_scheduler` module from the nRF5 SDK.
- Reviewed SES.md file that is visible only in SEGGER Embedded Studio.
- Fixed a bug on @ref md_doc_user_guide_modules_models_creating, where `send_reply()` was mentioned twice
  instead of `reply_status()`.
- Removed information about timeslot and flashing limitations after introducing high priority
  timeslots. This affects, for example, @ref NRF_MESH_PROV_BEARER_GATT_UNPROV_BEACON_INTERVAL_MS.

#### Friendship feature
- Made changes to the transport layer to support the Friendship feature. The layer now has
a new RX address logic and additional SAR RX complete logic.
- Modified the network_packet_alloc API to allow packets in the friend queue to be sent without
  modification to the network metadata fields.
- Introduced a new @ref MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP define
  to the LPN functionality that allows a node with the LPN feature to participate in a mesh network
  as a regular node when the friendship is not established by keeping scanner
  and net beacon modules enabled.
- Implemented the core loopback mechanism and modified the upper transport layer as part of the core
  loopback mechanism, so that the Friend nodes can send messages to LPNs.
- Removed Friend and Low Power RX sources. Scanner source is now used for both.

#### Provisioning
- Made examples re-generate provisioning private and public keys for every run.
- Updated the static provisioner example to set the Network Transmit count to `2` for newly
  provisioned devices.
- Added support to only allow secure provisioning if required. This can be enabled by setting
@ref NRF_MESH_PROV_FORCE_SECURE_PROVISIONING compile time option to `1`.

#### Other changes
- Aligned the secure network beacon update logic with the latest minor version update
  of the specification.
- Fixed a bug that would cause asserts in the extended model initialization. Additionally,
  to maintain integrity of the device composition, new models cannot be initialized during runtime
  after the Bluetooth mesh stack has been initialized when persistent storage is used.
- Made changes at the scanner level to ignore the non-mesh packets and reduce memory footprint.
- Added APIs to the Config Client model for sending Network Transmit Set and Network Transmit Get
  messages.
- Started using nRF5 SDK as a backend for static asserts.
- Implemented a change to the bootloader. It now explicitly disables the radio before moving to the application.
- Improved the bank address selection of the Bluetooth mesh DFU example.
- Added the possibility to set CMock and Unity paths through environment variables on Windows and Linux.
- Added a log message to the @ref BLE_DFU_SUPPORT module if the @ref ble_dfu_support_init function
fails to initialize the Buttonless DFU Service due to the missing bootloader.


### Bugfixes @anchor release_notes_320_bugfixes
- Removed outdated information about setting `SDK_ROOT` macro from command line.
- Introduced a new @ref NRF_MESH_EVT_PROXY_STOPPED event to fix a bug that would cause the node reset to
  get stuck when using GATT.
- Fixed a Health Server issue that would cause the server to return from fast period back to regular publish
  period without taking any configuration server updates into account, or report a wrong publish
  period value while publishing with fast cadence.
- Fixed a bug that would prevent the Health Server model from supporting subscriptions. See
  [Migration guide](@ref health_server_subscription) for details.
- Fixed wrong opcodes used in the Generic Power OnOff model.
- Fixed a bug that would cause level behavior module to not accept all transition time values
  provided by the Level Server model for the move speed calculation, and fixed some behavioral
  corner cases.
- Fixed a bug that would cause the access layer to use a non-zero TTL value when replying to a
  message received with the TTL set to zero.
- Fixed an issue that would cause the provisioning process to take around 10 seconds more than in
  the nRF5 SDK for Mesh v2.2.0.
- Implemented a fix that makes the Generic Default Transition Time Server model validate set message
  parameters.
- Fixed a bug that would cause the @ref nrf_mesh_disable() function to take several seconds to disable
  the mesh.
- Fixed a bug that would prevent erasing of the application-specific flash data in the @ref md_examples_enocean_switch_README.
- Fixed a bug that would cause the @ref model_timer_elapsed_ticks_get() function to return a wrong value
  when called from inside of the callback.
- Fixed a bug that would cause the transport decrypt to hardfault if someone sent a packet to a
  virtual address with a device key.
- Fixed an issue where the public key was transmitted before rejecting an invalid public key from
  the peer. The invalid public key is now rejected immediately.
- Added a change to the transport layer that will now reject old, failed SAR sessions.
- Fixed a bug that would break receiving on `ALL_FRIEND_ADDRESS` when the Friend feature is active.
- Fixed incorrect DSM API usage in the provisioner example.
- Fixed a bug in [PB remote server and client examples](@ref md_examples_pb_remote_README)
  that would cause examples to stop working when they were power cycled after provisioning a device.
- Fixed a bug in the Config Server model that would cause the Config Server to send a wrong status code
  in response to the Config Model Publication Set message when the target model does not support publish
  mechanism.
- Fixed the pyACI company ID endianness in the opcode parsing.


---


## BLE Mesh v3.1.0 @anchor release_notes_310

@par Release Date: Week 2, 2019

This is a minor feature release. In addition to several minor and major bugfixes, it brings DFU support
for the LPN example, publish re-transmission support in the access layer, improvements in the
heartbeat module, and a major rework of example documentation.

As part of this release, several files have been modified. See
[Migration guide](@ref migration_300_added_removed) for details and update your projects accordingly.

### New features and highlights @anchor release_notes_310_highlights
- Added Device Firmware Upgrade (DFU) support for the LPN example. You can now upgrade the LPN device firmware using the solution from nRF5 SDK.
    - For more information, see @ref md_examples_lpn_README and a new standalone page:
      @ref md_examples_lpn_dfu_ble.
- Implemented publish re-transmission support in the access layer. You can now re-transmit published
  messages according to the Publish Retransmit Count and the Publish Retransmit Interval Steps states.

### Changes @anchor release_notes_310_changes

#### Identifier changes
- Updated example UUIDs. See the [Migration guide](@ref migration_310_uuid_update) page for details
  and required changes.
- Implemented URI hash usage. All examples now advertise the URI hash in the unprovisioned device
  beacon, and static provisioner example uses this URI hash during the provisioning process.

#### Heartbeat module improvements
- The subscription address no longer needs to be owned by any model.
- Moved parameter sanitation from configuration server to Heartbeat.
- Heartbeat now can trigger when friendship is established.
- Fixed upper boundary of publication count value.

#### OOB-related updates
- Added better checking of OOB authentication data in @ref nrf_mesh_prov_auth_data_provide().
  See [Migration guide](@ref migration_310_oob_error_checking) page for details and required changes.
- Added new API @ref nrf_mesh_prov_oob_number_provide() for convenience when using a numerical
  provisioning OOB method.

#### Documentation
- Reworked documentation pages for @ref md_examples_README.
    - All pages now follow the same layout, which makes them more readable. This new structure is
      similar to the nRF5 SDK examples.
    - Contents of the pages have been reviewed and updated.
    - PB remote client and server examples are now described on one page:
      @ref md_examples_pb_remote_README.
- Edited the @ref md_doc_user_guide_mesh_compatibility page structure for clarity.
- Expanded information about [interacting with examples using SEGGER RTT Viewer](@ref segger-rtt).

#### Other changes
- Implemented stricter rules for transaction number handling in PB-ADV to avoid unexpected behavior
  when paired with misbehaving nodes. See [Migration guide](@ref migration_310_pb-adv_change) page
  for details and required changes.
- Added support for clearing the state in Bluetooth mesh config.
- Added upper boundaries to the number of keys, models, and addresses to avoid unexpected behavior
  when configuration messages exceed maximum access layer message size.
- Simplified Generic Power OnOff client initialization API.
  See [Migration guide](@ref migration_310_onoff_api) for details and required changes.
- Made replay protection more flexible and cut its RAM requirements by half.


### Bugfixes @anchor release_notes_310_bugfixes

#### Low Power node bugfixes
- Fixed a bug that would cause the LPN to terminate friendship after getting 5 data packets in a row.
- Added missing PB-ADV bearer in the LPN example compilation unit.
  See [Migration guide](@ref migration_310_pb-adv_change) for details.

#### Other bugfixes
- Fixed issues related to beaconing interval calculation for secure network beacons.
- Fixed a bug that would cause the relay and originator packets to have the same repeat count by default.
  They now have different repeat counts once again.
- Fixed a bug with the serial interface. It now propagates the SERIAL_OPCODE_EVT_MESH_TX_COMPLETE
  events as advertised.
- Fixed a bug in the app_timer that caused the LEDs to stop blinking.
- Fixed an assert when setting the GATT proxy state in the configuration server with the same value
  twice.
- Fixed a bug with the Health Server attention state. It no longer disables publication if the fast
  intervals are shorter than 100 ms.
- Fixed the incorrect scaling of advertisement timeout interval parameter provided to the
  `mesh_adv_params_set()` API.
- Fixed the light switch provisioner example. Invalid packets no longer cause the light switch
  server configuration to fail.


### Known issues and limitations @anchor release_notes_310_known_issues
- Stack applies only 20 seconds delay if secure network beacons from the other devices are
  transmitted unevenly.
- Due to a bug in the GNU ARM Embedded Toolchain `8-2018-q4-major` release, building with CMake on
  Windows is not working correctly when you use this latest version. A warning note was added to
  @ref md_doc_getting_started_how_to_toolchain.


---


## BLE Mesh v3.0.0 @anchor release_notes_300

@par Release Date: Week 48, 2018

This is a major release that brings integration with the latest version of the nRF5 SDK and
experimental support for the Low Power feature and GenericOnOff models. It also introduces changes
to API, core, and examples, several important bugfixes, and the first step in a major
documentation re-work.

As part of this release, several files have been added and removed. See
[Migration guide](@ref migration_300_added_removed) for details and update your projects accordingly.

### New features and highlights @anchor release_notes_300_highlights
- Added integration with the nRF5 SDK v15.2. The nRF5 SDK for Mesh is now incompatible with all
  older releases of the nRF5 SDK. See the [Migration guide](@ref migration_300_irq_priority) page
  for required changes.
- Added experimental support for Low Power node feature. See @ref md_doc_user_guide_modules_lpn_concept
  and @ref md_doc_user_guide_modules_lpn_integrating for documentation.
	- Due to this change, the high frequency crystal is stopped when the Bluetooth mesh is inactive.
	- Implemented Low Power node example. See @ref md_examples_lpn_README for documentation.
- Implemented experimental support for Generic PowerOnOff models.

### Changes @anchor release_notes_300_changes

#### API changes
- Changed @ref NRF_MESH_IRQ_PRIORITY_LOWEST IRQ priority level in accordance to nRF5 SDK 15.2.
  See [Migration guide](@ref migration_300_irq_priority) for details.
- Added new argument `attention_duration_s` to @ref nrf_mesh_prov_provision() in
  `mesh/prov/api/nrf_mesh_prov.h`. This is required in the provisioning process.
  See [Migration guide](@ref migration_300_provisioning) for details.
- Added enforcement of a spec rule that disallows simultaneous segmented transmissions between a
  source and a destination address by returning `NRF_MESH_ERROR_INVALID_STATE` on publish calls.
  See [Migration guide](@ref migration_300_segmented_messages) for details.
- Added new event @ref NRF_MESH_EVT_DISABLED.
    - After calling nrf_mesh_disable(), the Bluetooth mesh stack cannot be considered disabled until the
      new @ref NRF_MESH_EVT_DISABLED is received.
- Added a function to get the Health Server structure from the Bluetooth mesh stack module for usage in the
  Health Server API.
- Unified compile time flags for enabling and disabling Bluetooth mesh features. See
  [Migration guide](@ref migration_300_compile_time) for details.

#### Core changes
- The Bluetooth mesh stack scheduler now runs on `app_timer`.
	- Some modifications to the SDK 15.2 version are required. More information can be found in the
      [Migration guide](@ref migration_300_stack_separation).
		- The modified version can be found in `external/app_timer/app_timer_mesh.c`.
		- The modified implementation is added by default in the example projects.
	- There are no changes to the API and existing code using the `app_timer` will work without
      modification.
- Flash Manager now supports reading entries while in the defragmentation state.
- Updated subscription_list_share() to ensure there is only one allocated list at a time.
	- When the extended model now shares the subscription list across all of its models, the access
      deallocates any excess allocated lists.
- Removed all uses of Variable Length Arrays (VLAs). If compiled with ARMCC, the program will use
  heap allocated memory for the array and will hardfault if the allocation fails.
- Unified all dynamic memory management in the Bluetooth mesh stack into one @ref MESH_MEM module.

#### Example changes
- GATT Provisioning and Proxy features are now enabled in the Light Switch Server, Light Switch Client, Dimming Server, Dimming Client, and EnOcean examples.
	- Due to this change, the Light Switch Proxy Server and the Light Switch Proxy Client examples were removed.
- Lowered the default Node Identity advertisement interval.
    - Lowering the interval allows the provisioner to re-connect to the device more quickly after
      the provisioning is complete. This in turn makes the provisioning and configuration process
      faster when using the nRF Mesh mobile app.
- The `mesh_softdevice_init` module is replaced with the @ref BLE_SOFTDEVICE_SUPPORT.
	- The function @ref ble_stack_init() is used to initialize the SoftDevice Handler and the BLE
      stack and to register Bluetooth mesh handler for SoC events.
	- The functions @ref gap_params_init() and @ref conn_params_init() are used to run GATT
      Provisioning and Proxy features.

#### Documentation changes
- Updated the documentation. This is the first step in a longer process of improving the
  quality of the documentation.
 	- Modified the structure of documentation on the @link_mesh_doclib and in the @link_meshsdk_github, including:
  		- Moved several pages between sections. For example,
          @ref md_doc_user_guide_mesh_interrupt_priorities is now featured in the Overview section.
		- Removed Scripts section on the @link_mesh_doclib that contained a duplicated page about
          Interactive PyACI. This page is now available at
          [Libraries > Serial interface](@ref md_scripts_README).
  		- Removed the standalone deprecated list page. Deprecated functions are still marked as
          `_DEPRECATED` in the code.
  		- Removed the standalone Simple OnOff model page. Its contents are now integrated in the
          model's header file and the [Creating new models](@ref md_doc_user_guide_modules_models_creating) page.
  		- Renamed Introduction to Overview. This section now includes conceptual, descriptive documentation.
  		- Updated @ref md_doc_getting_started_getting_started section. It now includes instructional
          documentation.
		- Grouped experimental examples in the Experimental examples subsection.
		- Grouped provisioning-related pages in the @ref md_doc_user_guide_modules_provisioning_main
          subsection.
 	- Created new pages by splitting content on already existing pages.
      These new pages will be expanded in the future.
	- Edited @ref md_doc_getting_started_how_to_toolchain page for clarity. It now better lists
      required tools for each operating system.
	- Edited @ref md_doc_getting_started_how_to_build page, so that it lists building instructions
      for both SEGGER Embedded Studio and CMake on one page. The instructions were updated.
 	- Created new @ref md_doc_getting_started_how_to_run_examples page. It includes expanded
      contents from the main @ref md_examples_README page.
	- Changed names of several pages for consistency. For example, "How to add PA/LNA support"
      is now "Enabling PA/LNA support".
	- Fixed typos and various language issues on several pages.
	- Added table of contents and horizontal section separators to several pages. This formatting
      will be extended to other pages in the future.
- Edited documentation of several APIs for typos, clarity, and consistency.

### Bugfixes @anchor release_notes_300_bugfixes

#### Core bugfixes
- Fixed a bug that could cause the transport layer to not re-schedule retries for segmented messages
  if the sequence number allocation failed.
- Fixed a bug that would cause Timer Events Scheduler to be unable to reschedule events more than 65536 times.

#### Flash Manager bugfixes
- Fixed Flash Manager init behavior.
	- Timeslots are now started before the calls to flash_manager_add to prevent an init-deadlock.
	- Fixed a bug that would report Flash Manager as stable when it wasn't. `packet_buffer` now has
    an additional `packet_buffer_is_empty()` API for checking whether the queue is completely empty.
    The @ref flash_manager_is_stable() function makes use of this API to make the
    @ref flash_manager_wait() correctly start blocking.
- Fixed a bug that would block flash operations that can't fit between two Softdevice events
  (for example, Proxy connection events).

#### Model bugfixes
- Fixed a bug concerning Company ID in Vendor Model Opcode (see @ref access_opcode_t). Company ID
  is now correctly packed in little-endian order.
    @note Due to this fix, vendor specific models are now incompatible with BLE Mesh v2.2.0 and older.
- Fixed a bug that would make it impossible to add models that only send messages.

#### PyACI bugfixes
- Fixed a bug in PyACI that would accept 0xFFFF as a valid subscription address for "Config model
  subscription add". Now, Invalid Address error is returned.
- Fixed a bug that would trigger PyACI assert when loading Retransmit values from the provisioning database.
- Fixed a bug in PyACI that caused asserts when unpacking AppKey/NetKey Lists in multiple places.
- Fixed a bug that would cause a PyACI assert when unpacking some status replies in the PyACI
  Configuration Client.

#### Assertion and crash fixes
- Fixed a bug that would cause the mesh network devices to hardfault because of misformed packets. Network
  packets are now subject to length checks.
- Fixed a bug that caused GATT to assert when in connection, if the network ran out of sequence numbers.
- Fixed some rare asserts in the transition from provisioning to configuration when adding a device
  with the nRF Mesh App.

### Known issues and limitations @anchor release_notes_300_known_issues
- If the Bluetooth mesh stack is configured with IRQ priority @ref NRF_MESH_IRQ_PRIORITY_THREAD and run in the
  main loop with app_scheduler, there might be delays of ~15 ms.
- Publish re-transmission settings are not supported.



---


## BLE Mesh v2.2.0 @anchor release_notes_220
- This is a minor production release.

### New features
  - Generic server/client model interfaces for OnOff, Default Transition Time, and Level models
  - Sample generic OnOff server behavior implementation
  - New Mesh Config module that provides high-level access to persistent storage. This module uses
    the existing Flash Manager and aims to enable multiple flash backends (including nRF5 SDK
    fstorage) in the future.
  - Moved Bluetooth mesh runtime configuration options to a new, type-safe mesh_opt_* API in their
  respective submodules. The options are stored in persistent memory through the new
  mesh_config module.
  - Added persistent storage to several internal states:
    - Heartbeat publication
    - Net beacon
    - GATT proxy

### Other
  - Updated model directory structure:
    - Foundation models have been moved to models/foundation
    - Generic models are present in models/model_spec
    - Vendor specific models have been moved to models/vendor
    - Experimental models have been moved to models/experimental
  - Updated examples to support Generic OnOff models
  - Simplified EnOcean, Light Switch Client, and SDK coexistence examples to use only two Generic
    OnOff client model instances
  - Marked the old `nrf_mesh_opt` API deprecated (it will be removed in the next major production
    release)
  - Updated the mesh to use the section variables module from the nRF5 SDK (see the migration guide
    for details)
  - Updated various parts of the documentation (added documentation for GATT Proxy example and for
    PA/LNA support)

### Bugfixes
  - EnOcean example was not supporting multiple enocean switches
  - Mesh GATT asserted if other services uses HVX (MBTLE-2623)
  - Serial interface driver does no longer block on packet allocation (MBTLE-1844)
  - Made access address definition explicitly unsigned (MBTLE-2453)
  - `bootloader_verify.py` did not recognize nRF52840 (MBTLE-2610)
  - Fixed parsing error in PyACI `heartbeat_subscription_get()` (MBTLE-2690)

### Known issues and limitations
  - Softdevice S140 v6.0.0 sets the event IRQ priority into the wrong value 6 (should be 7).
  That might cause an internal stack memory corruption.
  To avoid the issue the file from Mesh SDK `<Mesh SDK folder>/external/sdk_fix/nrf_sdh.c` shall
  be used.
  Otherwise the examples which use GATT will generate assertion
  `Mesh error 3 at <address> (examples/common/include/mesh_app_utils.h:100)`
  - If the Bluetooth mesh stack is configured with IRQ priority NRF_MESH_IRQ_PRIORITY_THREAD and run in
    the main loop with app_scheduler, there might be delays of ~15 ms.


---

## BLE Mesh v2.1.1 @anchor release_notes_211
- This is a minor production release.

### New features
- Add PA/LNA support for timeslot projects

### Bugfixes
- Access loopback needs context switch
- Config server: send_publication_status() always sends status code as ACCESS_STATUS_SUCCESS or
  it Asserts
- Sending a reliable message via internal loop causes to double sending with opposite results
- Don't allow provisioner to use OOB public key if we don't support it
- Fix static assertions for flash size
- Config server replies to feature set when it shouldn't
- light_switch_proxy_client SES project imports app_error.c twice, leads to compile error
- Light switch client requires all buttons to be configured
- Heartbeat Publication Set message duplicates count value
- Bluetooth mesh GATT asserts on MTU requests
- In-place modification of event list during event handling
- Device page generator outputs file for nrf52832 no matter platform chosen
- Bluetooth mesh timeslot extension is prohibiting softdevice advertising (GATT)
- Bluetooth mesh proxy sets advertising timing in wrong order
- Invalid handling of service changed attribute
- Application defined softdevice settings are lost during GATT dabase reset
- Core TX alloc rejected by GATT proxy bearer
- Provisioner stops provisioning new nodes prematurely
- No support for reserved groups (all-nodes, all-proxies, ...)
- Heartbeat does not include all active features in published message
- Segger Embedded Studio projects have invalid memory configurations
- Connecting and disconnecting from PB-GATT leaves provisioning bearers in undefined state
- Persistent storage is turned off for proxy client
- Bluetooth mesh GATT module does not propagate ADV timeout event
- hal_led_blink_ms call is blocking and used in IRQs
- `device_page_generator.py` key parsing errors
- `device_page_generator.py` wrong output filename


### Known issues and limitations

- Publish re-transmission settings are not supported
- Some Config server and Health Server model states are not persistent
- Setting device in attention state during provisioning is not supported
- Light switch provisioner example:
  During the configuration of a node, the static provisioner example may sometimes consider a status
  response of a previous configuration step as the status response of the current configuration step.
  This may cause a node configuration to remain incomplete, without the provisioner noticing. If
  this happens, provisioned client or server nodes will not respond to user inputs as expected.
  This occurs due to Bluetooth mesh message re-transmissions logic built into the stack causing responses to
  SET messages to arrive out of order. This scenario is most likely to manifest itself in situations
  when the Bluetooth mesh stack is not scanning for the majority of the time. For example, while running other
  BLE connections with a short connection interval.


### Verification Notices / Test Errata
- Test Configurations:
-- nrf52832   ||   s132_6.0.0   ||   pca10040
-- nrf52840   ||   s140_6.0.0   ||   pca10056


---

## BLE Mesh v2.0.1 @anchor release_notes_201
- This is a minor bugfix release

### Bugfixes
- Ignore Config Proxy Set and Config Friend Set messages with invalid parameters


---

## BLE Mesh v2.0.0 @anchor release_notes_200
- This is a major production release.

### New features
- GATT Proxy (experimental)
- PB-GATT bearer (server) (experimental)
- Generic GATT interface for PB-GATT and Proxy (experimental)
- Interactive Configuration client for the Interactive PyACI
- Concurrent provisioning link listening
- Third party BLE device integration example: Integrating EnOcean switch with Bluetooth mesh networks
- Separate Light switch provisioner example

### Other
- Integrates with nRF5 SDK 15 (NOTE: nRF5 SDK is now a separate download from nRF5 SDK for Mesh)
- Examples more aligned with the nRF5 SDK examples
- Separated out provisioner role from Light switch client example

### Bugfixes
- No SLIP bytes before responses when using SLIP mode for the serial interface
- Buffer handling does not reset head and tail when buffer is empty
- Various asserts in Light switch example
- Stack doesn't send Provisioning Failed PDU in certain cases
- Transport layer sends ACK for timed-out message
- No way to stop IV update procedure in test mode
- Heartbeat subscription does not check destination state
- Provisionee does not sanitize input in provisioning start message
- `access_reliable_cancel_all()` and `access_model_reliable_cancel()` does not actually cancel
  pending reliable transfers
- Access status codes does not align with Bluetooth Mesh Profile v1.0 values
- Config server doesn't reply to Composition Data Get message requesting page `0xFF`
- Config server reports error for duplicate NetKey Add command
- Config server `send_publication_status()` always sends status code as `ACCESS_STATUS_SUCCESS` or
  it asserts
- Config server resets the device before reset response goes out
- Device key is bound to only one network key (primary)
- Access layer does not allow user to stop periodic publication by setting Unassigned (0x0000)
  publish address (or any other event which disabled model publication)
- Vendor Model IDs reversed according to the @tagMeshSp
- Redundant advertiser in beacon module
- Wrong IRQ level used for button GPIO and UART

### Known issues and limitations
- Publish re-transmission settings are not supported
- Some Config server and Health Server model states are not persistent
- Setting device in attention state during provisioning is not supported
- Light switch provisioner example:
  During the configuration of a node, the static provisioner example may sometimes consider a status
  response of a previous configuration step as the status response of the current configuration step.
  This may cause a node configuration to remain incomplete, without the provisioner noticing. If
  this happens, provisioned client or server nodes will not respond to user inputs as expected.
  This occurs due to Bluetooth mesh message re-transmissions logic built into the stack causing responses to
  SET messages to arrive out of order. This scenario is most likely to manifest itself in situations
  when the Bluetooth mesh stack is not scanning for the majority of the time. For example, while running other
  BLE connections with a short connection interval.
- `device_page_generator.py`
  - The script will not parse the `public_key` property of `bootloader_config_default.json` when
    using Python 2. A workaround for the issue is to call `bytearray.fromhex()` on the
    `public_key` in `device_page_generator.py:99`.
  - The script will store the new device page to `bin/device_page_nrf52832_xxAA_s132_5.0.0.hex`
    regardless of SoftDevice and platform setting. A workaround for the issue is to specify the path
    manually with the `--output-file` option, i.e., `--output-file bin/device_page.hex`.


---

## BLE Mesh v1.0.1 @anchor release_notes_101
This is a hotfix release with documentation/bug fixes.

### New features
- None

### Bugfixes
- Provisionee support added to Beacon example
- Beacon Example: Fixed beacon enable serial command
- Provisioner:  Fixed OOB authentication procedure
- Device Reset: Fixed 1 dropped packet on startup

### Other / Documentation
- SDK coexistence guide updated for better explanation of coexistence with other Nordic SDKs for
  concurrent GATT/GAP (and other usage)
- DFU quick start guide fixed along with device_page_generator.py script
- SDK patch file updated
- Serial Example documentation updated
- Bluetooth mesh Assert cleanup

### Verification / Test Errata
- nRF51 platform testing has been put on hold


---

## BLE Mesh v1.0.0 @anchor release_notes_100

This is the first production release of Nordic's nRF5 SDK for Mesh. This release implements mandatory
features for the @tagMeshSp and also some proprietary features
(PB-remote and Nordic Advertiser Extensions) in experimental state.

### New features
- Key refresh has been implemented
- Heartbeat support has been added
- DFU bootloader source code added
- Support for nRF52840_xxAA
- Proprietary Nordic Advertiser Extensions ("InstaBurst!") feature for improved throughput
- AD-listener module for simplified subscription to advertisement data
- IRQ levels are now aligned with the nRF5 SDK
    - All API-functions are expected to be called from `NRF_MESH_IRQ_PRIORITY_LOWEST`
- Node de-provisioning supported through the Config Server Node Reset message

### Bugfixes
- Fixed beacon advertiser not enabled in example
- Fixed trailing garbage data in Composition Data
- Build fails if the path to the repo has spaces
- Fixed issue where the serial interface gets corrupted memory or hangs on an incoming serial
  packet that is too long
- Fixed issue where the Config Server used the application key index and not the DSM handle when
  deleting key
- Removed duplication of Company ID
- Fixed assertion on invalid length for unprovisioned beacons
- Removed warnings and errors from documentation build

### Other
- Provisioning API changed slightly to de-couple the upper and lower layers more cleanly

### Known limitations
- Optional features of the @tagMeshSp are not part of this release.

### Test Errata
- DFU : replacing Softdevice live while running app is not tested.


---

## BLE Mesh v0.10.1-Alpha @anchor release_notes_0101

This is a hotfix release with no new features.

### Bugfixes

- Segger Embedded Studio Projects have devkit BOARD defines now instead of dongles.
- Standard BLE Access Address now used for all binary artifacts (.hex/.lib)


---

## BLE Mesh v0.10.0-Alpha @anchor release_notes_0100

This is a minor feature release for the experimental nRF5 SDK for Mesh

### New features

- Health Model

- Reworked build system
    - Support for nRF52840 (but not part of the integration testing)
    - Support for new SoftDevices S132 versions 4.0.4 and 5.0.0 and S140 version 5.0.0-3.alpha
    - Aligned with the nRF5 SDK version 14. Note: A subset of the nRF5 SDK is included in the nRF5
      SDK for Mesh with some minor changes.
      See `external/nRF5_SDK_14.0.0_3bcc1f7/nRF5_SDK_14.0.0_3bcc1f7.patch` for the changes made.

- Refactored Bearer Layer and Core for increased bandwidth and robustness
    - Any number of concurrent advertisers
    - Custom radio implementations possible
    - Improved SAR reliability and throughput
    - TX Complete events fully supported
    - Significant reduction in packet processing time
    - More efficient radio code, major throughput improvements in noisy conditions

- Access Layer loopback, messages sent between models on the same device will now be short-circuited
  through the access layer
- RSSI and timestamp information is now available in the `access_message_rx_meta_t` and used in the
  Light switch example
- New filter engine for scanner
- Scanner filters: AD type, advertisement packet type, GAP address, RSSI
- Support for user-defined packet filters
- Each example project now has its own Segger Embedded Studio project file

### Bugfixes

- SAR receiving segments with TTL=0 should reply with SegAck TTL=0
- Bug in validation of command line options in device_page.py
- "family" undefined in reset_device() in bootloader_verify.py
- Access application key bitfield needs to hold device keys also
- DSM clears regular address when deleting virtual address
- Reliable parameter unset in access:packet_tx()
- Various bugfixes

### Document updates
- Revised documentation for better distinction between implementation-specific information and
  concepts defined by the Bluetooth mesh profile specification, updated references to the latter
- Updated DFU Quick Start guide
- Added more detailed installation instructions

### Other
- "Light control example" has been renamed "Light switch example" to resolve similarity with the
  Light Control model in the Bluetooth mesh model specification
- Step-by-step howto for setting up Keil projects for Mesh now available on
  DevZone: https://devzone.nordicsemi.com/blogs/1180/creating-a-keil-project-for-a-bluetooth-mesh-examp/

### Known limitations
- Heartbeat feature is not supported


---

## BLE Mesh v0.9.2-Alpha @anchor release_notes_092

This is a hotfix release, providing critical bug fixes and improvements.

### New features

- nrf_mesh_packet_send() now supports the reliable feature. I.e., it is possible to send single
  segments messages using the transport layer SAR.
- Interactive PyACI has support for an interactive provisioner and provisionee
- New serial interface event "Prov Failed"

### Bug fixes

- Provisionee not handling invalid provisioning data properly
- Problems using "Release" configuration in SES examples
- Incorrect usage of hal_led_blink_ms() in light control server
- Serial buffers must be word aligned
- Number of elements not handled in Serial interface's "Capabilities set" command
- S110 build failure
- Default build type is set in CMake
- Word alignment problems caused by high optimization levels when using Segger Embedded Studio
- PB-remote opcodes overlapping with Configuration model opcodes
- Advertisement bearer used timer_scheduler contexts dangerously, potentially corrupting its
  internal linked list
- PB-remote server would get confused about out-of-order ACKs from the client
- Documentation has been updated

### Document updates

- Rename "Bluetooth Mesh SDK" to "nRF5 SDK for Bluetooth mesh"
- How to create a model document updated to current API
- Minor fixes to cryptography section in the "Bluetooth mesh basic concepts" document
- Added installation instructions for CMake, Ninja and ARM toolchain
- Added information on how to select compile target

### Other

- Serial handlers split into separate source file
- Fix unit tests to work with the public CMock
- Fix packet formats and application nonce for compliance
    - Update unit tests with new sample data
- Several minor bugfixes
- Serial interface "Init Context" command removed. Provisioning context initialization is managed
  automatically.

### Known limitations of this release

- Heartbeat and Health model not implemented
- Source for bootloader must be downloaded from OpenMesh gitHub repo
- No power down state storage


---

## BLE Mesh v0.9.1-Alpha @anchor release_notes_091

This is an experimental release for exploration of the Bluetooth mesh stack on the nRF5 device family.
It is not intended for commercial use.

### Key Features

- Bluetooth mesh software core stack
    - Based on Bluetooth 4.0 PHY
    - Bearer, network, transport, and access layers
    - Foundation models
- Support for Node and Relay Node roles
    - Configurable scanning interval and duty cycle (from 3ms-10240ms)
    - Configurable advertisement interval (from 20ms-10240ms
- Example applications and proprietary models
- Broadcast flooding mesh
    - Theoretically up to 32,000 nodes
    - No routing tables
    - No single point of failure
    - Node-to-node and node-to-group communication
- Two-layer 128-bit AES-CCM network and node-to-node security
- Provisioning support
    - Standard "local" provisioning over advertisement bearer
    - Proprietary "remote" provisioning via relaying nodes (implemented as proprietary model)
- Persistent (flash) storage of configuration data
- Support for concurrent beaconing (separate API, Eddystone, iBeacon)
- Python shell-based test and demo framework for PC
- Support for over-the-air secure background DFU
    - Application and/or stacks
- Cross-platform toolchain support
    - Segger Embedded Studio
    - GCC and armcc
    - Windows, Linux and macOS

### Bugfixes in this release
- AD types updated to use Bluetooth SIG allocated numbers
- Proprietary On/Off model example renamed from "Generic On/Off" to "Simple On/Off"
- Minor DFU bugfixes and improvements

### Other

- Revised and reorganized documentation.

### Compatibility

- nRF51 and nRF52
- Recommended >= 200 kB Flash and >= 32 kB RAM
- nRF5 SDK (v10.0.0 for S110, v12.1.0 for S130 and S132)
- Segger Embedded Studio v3.22

### Known limitations of this release

- Packet format in Alpha is not entirely Bluetooth mesh compatible
- Heartbeat and Health model not implemented
- ASZMIC field is missing from application/device nonce for segmented access messages
- Source for bootloader must be downloaded from OpenMesh GitHub repo
- No power down state storage


---

## BLE Mesh v0.9.0 @anchor release_notes_090

### Bugfixes

- DSM for-loop overflow bug. Looped over number of application keys rather than network keys.
  Caused fault on reception of secure mesh network beacons.
- IRQ level for nRF52 set too low in light control example. Caused hardfault on button press.
- Debounce algorithm for button presses in light control example used `TIMER_OLDER_THAN()` when it
  should have used `TIMER_DIFF()`. Made buttons unresponsive after ~1h.
- Replay protection cache checking wrong address
- Network cache not caching messages if transport layer fails
- Serial provisioning interface not setting all parameters of the provisioning struct

### New features
- Configuration server
- Configuration client
- light control example uses configuration client+server to configure the newly provisioned devices.
    - Also supports group addressing
- New host scripts for Serial ACI. Auto-generated the same way as documentation is.
- Persistent storage manager
    - Integrates with access layer and the Device State Manager
- Persistent storage integration with examples
- Node configuration module for provisionee applications
- Use asynchronous event flags with bearer event module


### Other

- Updated documentation for PB-remote example
- Added README for light control example
- Added Howto for creating proprietary models

### Known limitations

- No generated documentation or host scripts for model specific commands in the Serial ACI.
  This affects mostly the PB-remote client.
- In bootloader mode all DFU transfers will be accepted regardless of the FWID.
- BLE Mesh non-compliance (will be fixed in upcoming releases):
    - "Generic OnOff" model example is not complete nor spec compliant
    - Opcodes do not follow specification
    - Bluetooth SIG assigned numbers for AD_Types, Service UUIDs and Characteristic UUIDs are not up-to-date
    - Health Model and Heartbeat features are not implemented
    - Packet format changes related to removal of MD bit are not implemented


---

## BLE Mesh v0.8.1 @anchor release_notes_081

### Release notes
- PB-remote server and client updated for new access layer
- Bugfixes for new access layer and device state manager
- Asynchronous event flags
  - Timer scheduler
  - Serial
- Serial integration
  - Device state manager
  - Access layer
- Transport layer device key decryption bug fix
- Adds support for PB-remote server disabling (issue #66)
- Adds a Simple OnOff demo model
- Network beacon initialization added (was missing in v 0.8.0)

### Known issues
- Per-beacon TX power settings are not yet functional for the serial interface
- The transport layer still defaults to +malloc()+ to allocate SAR buffers
- IV update will trigger without checking the value of the received iv index
- Interactive_pyaci is not up-to-date with the latest serial interface


---

## BLE Mesh v0.8.0 @anchor release_notes_080

This release features a preview of the refactored nRF Mesh API and new key modules.

### Release highlights

- Device State Manager
  - Manages all mesh-related keys and addresses for the user
  - Reduces key-storage memory footprint
- Access layer
  - Reworked API to fit more usage scenarios
  - Reduced memory footprint
  - Now takes ownership of the model database, in preparation for persistent storage support
- New application configuration module for the Device State Manager and access layer pool sizes
  - Application specified compile-time configuration of memory usage for the upper layers
- Reworked serial interface for use with the Device State Manager
  - Added optional SLIP encoding for serial UART packets
  - Support for setting custom beacon data over serial
  - Improved documentation generator, now including command responses
  - Implemented GAP address get command (issue #38)
  - Reduced memory footprint
- Improved IV update procedure
  - Improved quality of service for mesh messages
  - Fully spec compliant
- DFU procedure improvements
  - Improved reliability
  - Packet loss recovery procedure has lower impact on the rest of the transfer
  - Fix for GitHub issue #77: Timeout in DFU ready phase

### Known issues and limitations

- Serial opcodes are not backwards compatible
- PB-Remote is not integrated with the new access layer
  - It is not supported by the serial interface either
- Per-beacon TX power settings are not yet functional for the serial interface
- The transport layer still defaults to +malloc()+ to allocate SAR buffers


---

## BLE Mesh v0.7.7 @anchor release_notes_077

- Documentation improvements
- Renamed some modules to prevent using Bluetooth SIG's confidential identifiers
- Bug fixes
  - A corner-case bug where the provisioning complete ACK is lost but a successful close packet
    is sent is now fixed
  - Fixed incorrect endianness in address check (issue #74)

### Notes
The transport SAR uses +malloc()+ to allocate buffers for SAR transactions. This behaviour
can be overridden using +transport_sar_mem_funcs_set()+, otherwise +__HEAPSIZE+ needs to be
defined.

**WARNING:** SoftDevice needs to be Flashed without memory protection

