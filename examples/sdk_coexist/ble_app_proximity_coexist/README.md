# SDK Proximity coexistence example

@tag52840and52833and52832
@tag52840dongle52810and52820nosupport

This example demonstrates how nRF5 SDK for Mesh and nRF5 SDK examples can be used together
at the same time. It is built around two examples and has two parts:
- The Bluetooth mesh part of this example implements the
[light switch client example](@ref md_examples_light_switch_client_README) from
the nRF5 SDK for Mesh.
- The BLE part of the example implements the @link_ble_app_proximity_example example from the nRF5 SDK,
except for the following changes:
    - The BSP events `SLEEP` and `DISCONNECT` are ignored.
    - The advertising interval is increased to allow more time for the Bluetooth mesh stack.
    - The example is not configured to enter the system-off mode, like in its nRF5 SDK version.

As the result of running this example, you'll be able to use a Bluetooth mesh network
in which this example can replace the light switch client example.

Before you start testing this coexistence example, see the following pages:
- @ref md_doc_user_guide_integrating_mesh_nrf5_sdk
- @ref md_examples_light_switch_README and @ref md_examples_light_switch_client_README


---

## Hardware requirements @anchor coexistence_proximity_example_hw_reqs

You need the same amount of development kits as for testing the [light switch client example](@ref md_examples_light_switch_client_README).

Additionally, you need one development kit for the [static provisioner example](@ref md_examples_provisioner_README).
For details, see [software requirements](@ref coexistence_proximity_example_sw_reqs).

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

@note
While this example is compatible with nRF52840 and nRF52833 development kits, the nRF5 SDK for Mesh only provides SES files for nRF52832.


---

## Software requirements @anchor coexistence_proximity_example_sw_reqs

This example does not support PB-GATT for provisioning.
For this purpose, use the static provisioner example: `<InstallFolder>/examples/provisioner`.
See the [provisioner example](@ref md_examples_provisioner_README) page for more information.

---

## Setup @anchor coexistence_proximity_example_setup

You can find the source code of the SDK Proximity coexistence example
in the following nRF5 SDK for Mesh folder: `<InstallFolder>/examples/sdk_coexist/ble_app_proximity_coexist`

The source code of the light switch client example is located
at the following path: `<InstallFolder>/examples/light_switch/client`

---

## Testing the example @anchor coexistence_proximity_example_testing

Complete the following steps to program the SDK Proximity coexistence example and the light switch example devices:

1. Copy the `ble_app_proximity_coexist` folder into the `examples/ble_peripheral` folder
at your nRF5 SDK installation path.
2. Open the Segger Embedded Studio project at `ble_app_proximity_coexist/pca10040/s132/ses/ble_app_proximity_pca10040_s132.emProject`.
3. Add MESH_ROOT to your Segger Embedded Studio global macro list:
    1. From the SES menu bar, click **Tools > Options...**.
    2. In the left column, click **Building**.
    3. In right column, double click **Global macros**.
    4. Add your Bluetooth mesh root directory in a new line: `MESH_ROOT=<path to your Bluetooth mesh SDK installation>`.
4. Ensure that the flash is completely erased before programming this example.
Otherwise Flash Data Storage module will not work correctly.
5. Program the light switch example devices as described in the @ref md_examples_light_switch_README,
including the servers and the static provisioner, with the following exceptions:
    - Replace the light switch client with `examples/ble_peripheral/ble_app_proximity_coexist/pca10040/s132/ses/Output/Release/Exe/ble_app_proximity_pca10040_s132.hex`.
    - Do not use the nRF Mesh mobile application for provisioning.
      Only use the static provisioner.


You can now run the two coexisting examples in parallel or in sequence:
    - Run the light switch client example as described in [light switch example documentation](@ref light_switch_example_testing_interacting).
    - Run the @link_ble_app_proximity_example example as described in the nRF5 SDK documentation.
