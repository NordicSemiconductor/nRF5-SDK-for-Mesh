# Examples
@anchor Examples

Mesh devices are broadly categorized into two roles: a provisioner role and a node role.
The nRF5 SDK for Mesh provides several example projects to demonstrate these roles, mesh models, and certain features that will
help you get started on new mesh-based projects.

**Table of contents**
- [Read before testing](@ref read-before-testing)
- [Available examples](@ref available-examples)
- [Common example modules](@ref common-modules)
- [Provisioning bearers in the nRF5 SDK for Mesh examples](@ref example_provisioning_bearers)
- [Evaluating examples with the nRF Mesh mobile app](@ref nrf-mesh-mobile-app)

@link_btsig_glossary.

---
## Read before testing @anchor read-before-testing

All examples are guaranteed to work with [fully compatible device configurations](@ref compatibility_list).
Some of the examples are [not compatible with nRF52810](@ref compatibility_nRF52810).

Before you start using the examples, see the following pages:
- [Installing the toolchain](@ref md_doc_getting_started_how_to_toolchain)
- [Building the mesh stack and examples](@ref md_doc_getting_started_how_to_build)
- [Running examples](@ref md_doc_getting_started_how_to_run_examples)

You can also quickly run an example without going through the complete toolchain installation.
See [Running a first example](@ref md_doc_getting_started_mesh_quick_start) for details.

---
## Available examples @anchor available-examples

The following examples are provided with this SDK:
* @subpage md_examples_light_switch_README is a mesh ecosystem example
  that contains some smaller examples: client, server, and proxy-server.
* @subpage md_examples_enocean_switch_README demonstrates how to implement a third party device in Mesh ecosystem,
  namely an EnOcean-to-Mesh translator. The EnOcean switches send the button status using BLE advertising packets.
  These packets can be captured and can be used to generate equivalent mesh messages for controlling other mesh nodes.
* @subpage md_examples_beaconing_README implements custom beacon advertising
  and shows how to send and receive custom packets using the nRF5 SDK for Mesh.
* @subpage md_examples_sdk_coexist_README demonstrate how the nRF5 SDK features can be simultaneously used with nRF5 SDK for Mesh.
* @subpage md_examples_dfu_README shows how to use the mesh DFU framework to update the firmware of a device over the mesh.
* @subpage md_examples_dimming_README demonstrate how to use [Generic Level model](@ref GENERIC_LEVEL_MODEL) APIs
  in an application to implement dimming light and corresponding dimmer switch.
* @subpage md_examples_light_lightness_README demonstrates how to use the [Light Lightness model](@ref LIGHT_LIGHTNESS_MODELS).
* @subpage md_examples_light_lc_server_README demonstrates how to use the [Light LC Setup Server model](@ref LIGHT_LC_MODELS).
* @subpage md_examples_light_ctl_README demonstrates how to use the [Light CTL models](@ref LIGHT_CTL_MODELS).
* @subpage md_examples_lpn_README demonstrates the Low Power node feature.
* @subpage md_examples_provisioner_README demonstrates the provisioning process and can be extended by other functionalities.
* @subpage md_examples_pb_remote_README demonstrates the use of remote provisioning to provision devices outside of the provisioner's radio range.
* @subpage md_examples_sensor_README demonstrates how to use the [Sensor model](@ref SENSOR_MODEL).
* @subpage md_examples_serial_README demonstrates how to  use the serial interface to create a mesh connectivity device.


---
## Common example modules @anchor common-modules

The examples implement common functionalities through several common modules, including among others:
- simple hardware abstraction layer,
- RTT input functionality that uses the nRF5 SDK @link_app_timer and enables the examples to poll [RTT](@ref segger-rtt) for input characters,
- Mesh stack and SoftDevice initialization helper modules,
- behaviors for several generic models.

For full overview of all common modules and detailed information, check the \ref MESH_API_GROUP_APP_SUPPORT API section.


---

## Provisioning bearers in the nRF5 SDK for Mesh examples @anchor example_provisioning_bearers

The nRF5 SDK for Mesh examples can be provisioned using both of the provided provisioning bearers, without the OOB authentication method or with the static OOB authentication method.
See the following table for an overview of which example works with [PB-ADV](@ref provisioning_main_pb-adv) or [PB-GATT](@ref provisioning_main_pb-gatt), or both.

| Example / Bearer                              | PB-ADV   | PB-GATT  |
|-----------------------------------------------|----------|----------|
| @ref md_examples_beaconing_README             |   X      |   -      |
| @ref md_examples_dfu_README                   |   X      |   -      |
| @ref md_examples_dimming_README               |   X      |   X      |
| @ref md_examples_enocean_switch_README        |   X      |   X      |
| @ref md_examples_light_switch_README          |   X      |   X      |
| @ref md_examples_light_lightness_README       |   X      |   X      |
| @ref md_examples_light_lc_server_README       |   X      |   X      |
| @ref md_examples_light_ctl_README             |   X      |   X      |
| @ref md_examples_lpn_README                   |   -      |   X      |
| @ref md_examples_provisioner_README           |   N/A    |   N/A    |
| @ref md_examples_sdk_coexist_README           |   X      |   -      |
| @ref md_examples_pb_remote_README*            |   X      |   -      |
| @ref md_examples_sensor_README                |   X      |   X      |
| @ref md_examples_serial_README*               |   X      |   -      |

All these examples use the 16-byte static OOB value. The static OOB value is stored in `STATIC_AUTH_DATA`, which is defined in `example_common.h`.

@note
The following examples marked with (*) in the table are exceptions:
- @ref md_examples_pb_remote_README -- uses PB-ADV with the static OOB authentication method. However, it uses a different 16-byte static OOB value than other examples. The reason is to prevent provisioning all devices around using the PB remote client.
- @ref md_examples_serial_README -- uses PB-ADV. However, it can be provisioned through the static provisioner if the same 16-byte static OOB value is used to initialize the provisioner role.

For more information about provisioning, see the following pages:
- @ref md_doc_user_guide_modules_provisioning_main
- [Static OOB authentication method API](@ref NRF_MESH_PROV_OOB_METHOD_STATIC)
- @ref provisioning-commands


---
## Evaluating examples using the nRF Mesh mobile application @anchor nrf-mesh-mobile-app

You can use @link_nrf_mesh_app (available for @link_nrf_mesh_app_ios and @link_nrf_mesh_app_android) with almost all of the Mesh examples
for provisioning and configuring the boards. The only example that does not support the mobile application is the @ref md_examples_provisioner_README "mesh provisioner".

This said, the nRF Mesh mobile application is _recommended_ for use with the following examples:
- @ref md_examples_light_switch_README
- @ref md_examples_enocean_switch_README
- @ref md_examples_dimming_README
- @ref md_examples_lpn_README
- @ref md_examples_light_lightness_README
- @ref md_examples_light_lc_server_README
- @ref md_examples_light_ctl_README
- @ref md_examples_sensor_README

Check the section about evaluating the example using the nRF Mesh mobile application on the documentation pages of these examples for example-specific information, like the name of the nodes in the application.

@note
The following procedures related to the nRF Mesh mobile application are referring to:
- The nRF Mesh application on Android. However, there are no major differences in the UI of the iOS version.
- The documentation page of the corresponding example. The model instances you need to bind can be different for your example -- check the documentation pages of recommended example. Moreover, for @ref md_examples_lpn_README, setting publication requires a [different procedure](@ref examples_lpn_running_provisioning).

### Provisioning with nRF Mesh @anchor nrf-mesh-mobile-app-provisioning

To provision Mesh examples with the nRF Mesh mobile app, complete the following steps:
1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    1. Erase the flash of your development boards and program the SoftDevice.
    2. Flash the client firmware on individual boards and the server firmware on other board or boards.
2. Open the nRF Mesh mobile app. The main application window appears.
3. Add a new node. The application starts looking for unprovisioned nodes and lists them on the screen.
    - See the example pages for information about the node naming conventions.
4. Provision each node by completing the following steps for each of them:
    1. Tap the node name to connect to it.
    2. Identify the node.
    3. Provision the node and select the desired OOB option. The application starts the provisioning process. When it is complete, you receive a notification.
5. When all nodes are provisioned, note the addresses of the server nodes. You need them when @ref nrf-mesh-mobile-app-publication "setting the publication address".

@note If the automatic configuration is broken, manually get the composition data, add application keys, and only then go to the following procedure.

### Binding nodes with nRF Mesh @anchor nrf-mesh-mobile-app-binding

To bind Mesh nodes with the nRF Mesh mobile app, complete the following steps:
1. On the server nodes, bind the server model instance specified in the corresponding example with the same application key:
    1. Tap the server node name. The node configuration menu opens.
    2. In the expanded Elements section, tap the model instance name.
    3. In the section with bound application keys, tap the button for binding the application key and then tap the application key field. The key is now bound.
2. Repeat step 1 for each server board node.
3. On the client node, bind the client model instance specified in the corresponding example with the same application key:
    1. On the list of provisioned nodes, tap the client node name. The configuration menu opens.
    2. In the expanded Elements section, tap the first client model instance name.
    3. In the section with bound application keys, tap the button for binding the application key and then tap the application key field. The key is now bound. The application goes back to the model instance menu.

### Setting publication and subscription with nRF Mesh @anchor nrf-mesh-mobile-app-publication

To set publication rules between nodes with the nRF Mesh mobile application, complete the following steps:
1. On the client node, in the publication section of the client model instance menu, tap **Set Publication**.
2. Tap the publication address field. A dropdown menu appears.
3. Choose one of the following address types to set the publication addresses of the client model:
    - Unicast Address -- You need to complete the following steps:
        1. Provide the address of any server node noted @ref nrf-mesh-mobile-app-provisioning "at the end of the provisioning procedure".
        2. Apply the changes.
    - Groups -- You need to complete the following steps:
        1. Select an existing group to subscribe or create a new one.
        2. Apply the changes for the client node.
        3. On the server nodes, set the Subscription Address of the server model instance menu to the selected group address.
        4. Apply the changes for the server nodes.


@note
If applicable for the chosen example, you can also configure the publish address of the second client model instance.
To do this, bind the model instance with the same application key and set the model instance's publish address to the Unicast Address
of any server node.

Any unhandled error is indicated by all LEDs on the board turning on in steady state.
You will need to restart the application by resetting the board.
To do this, switch the board on and off or use the following command:
```
nrfjprog -f nrf52 --reset
```
