# Examples
@anchor Examples

Mesh devices are broadly categorized into two roles: a provisioner role and a node role.

The nRF5 SDK for Mesh provides several example projects to demonstrate these roles, mesh models, and certain features that will
help you get started on new mesh-based projects.

The following examples are provided with this SDK:

* @subpage md_examples_light_switch_README "Light switch" - This is a mesh ecosystem example
  containing four sub examples: provisioner, client, server, and proxy-server.

  These examples demonstrate how to implement the following parts in the mesh ecosystem:
  - Mesh light switch (`client`)
  - Mesh light bulb (`server`)
  - Mesh provisioner (`provisioner`)
  - Mesh light bulb with proxy server (`proxy_server`)

  The client and server examples show how to use a [Generic OnOff model](@ref GENERIC_ONOFF_MODEL) APIs in an application.
  The proxy_server example additionally shows how to implement mesh proxy service.

* @subpage md_examples_experimental_dimming_README "Experimental dimming" - These examples demonstrate
  how to use [Generic Level model](@ref GENERIC_LEVEL_MODEL) APIs in an application to implement dimming
  light and corresponding dimmer switch.

* @subpage md_examples_enocean_switch_README "Enocean switch: Using third party devices in Mesh ecosystem" -
  This example shows how to implement an Enocean-to-Mesh translator. The Enocean switches send the button status using BLE advertising packets. These packets can be captured and can be used to generate equivalent
  mesh messages for controlling other mesh nodes.

* @subpage md_examples_pb_remote_client_README "Remote provisioning client" - This example demonstrates
  the use of remote provisioning to provision devices outside of the provisioner's radio range.

* @subpage md_examples_pb_remote_server_README "Remote provisioning server" - This is an
  example of a device that provides an end-point for relayed provisioning packets when
  using the remote provisioning feature to provision devices.

* @subpage md_examples_beaconing_README        "Beaconing" - This example implements custom beacon advertising
  and shows how to send and receive custom packets using the nRF5 SDK for Mesh.

* @subpage md_examples_dfu_README              "DFU over mesh" - This example shows how to use the
  mesh DFU framework to update the firmware of a device over the mesh.

* @subpage md_examples_serial_README           "Serial" - This example shows how to
  use the serial interface to create a mesh connectivity device.

* @subpage md_examples_sdk_coexist_README      "SDK coexistence" - These examples show how the nRF5 SDK features can be simultaneously used with nRF5 SDK for Mesh.

Example models are present in the @subpage md_models_README folder, while common example utility
modules are present in @subpage md_examples_common_README.

A simple [hardware abstraction layer](@ref md_examples_common_README) is shared by all the examples.

## How to build examples

To build the examples, follow the instructions in [Building the Mesh Stack](@ref md_doc_getting_started_how_to_build).

## How to run examples @anchor examples_how_to_run_examples

**Note**: The following procedure is not applicable for the DFU example. See [DFU Quick start guide](@ref md_doc_getting_started_dfu_quick_start) for this example.

To program examples onto a Development Kit, first connect it to your computer with a USB cable. When your board is detected, you can program examples as described below.

### Using Segger Embedded Studio
If you are using Segger Embedded Studio to run the examples, first erase the chip using the `Target -> Erase all`
menu option. Then the examples can be flashed and run using the `Target -> Download` option, which will flash
both the necessary SoftDevice and the application binary.

### Using Ninja

If you have set up the mesh tool chain and are able to build the examples, using `ninja` for
flashing examples would be the most easiest way.

To do so, execute the `ninja flash_<example_target_name>` target from the `build` directory:

To find out the exact target name for your example, run:

    build$ ninja -t targets

This will list all available ninja targets.

For example, to flash the light-switch-server example using ninja, run:

    build$ ninja flash_light_switch_server_nrf52832_xxAA_s132_6.0.0

After you issue this command, ninja will check if the example binaries are stale and, if required,
it will rebuild them before flashing.

Then it will display a list of connected boards and ask you to choose. After you make a selection,
it will program the SoftDevice and example firmware on the board.

### Using nrfjprog
Downloading examples using `nrfjprog` command line tool is a three step process. You will need to program a SoftDevice and the example hex file to your board.

The SoftDevice binaries are located in the `bin/softdevice/` folder, and example binaries will be built in the corresponding example folders within the `build/` directory.

#### 1. Program the SoftDevice
Download the SoftDevice which you chose to build mesh stack with. If you do not know the SoftDevice version that was used to build the mesh stack, check the name of the example binary.

For example, if the example's binary name is `light_switch_client_nrf52832_xxAA_s132_5.0.0.hex`, the required SoftDevice binary is `s132_nrf52_5.0.0_softdevice.hex`. To program this example run:

    nrf5_sdk_for_mesh$ nrfjprog --program ./bin/softdevice/s132_5.0.0/s132_nrf52_5.0.0_softdevice.hex --chiperase -f NRF52

#### 2. Program example application
To program the example binary mentioned above, run:

    nrf5_sdk_for_mesh$ nrfjprog --program ./build/examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_5.0.0.hex --sectorerase -f NRF52

#### 3. Reset the device
To launch the example, either power cycle the device or initiate a soft-reset. Soft-reset is particularly useful if you want to setup the debugger or RTT viewer just before launching the example and prevent disconnection of the RTT link. You can initiate a soft-reset by using the following command:

    nrf5_sdk_for_mesh$ nrfjprog -r -f NRF52

For some examples, additional steps might be required. See the Readme file for
each example for more information before running it.

## Command line interaction with SEGGER RTT @anchor segger-rtt

The examples can communicate with a host computer through @link_rtt<!--https://www.segger.com/products/debug-probes/j-link/technology/real-time-transfer/about-real-time-transfer/-->.

Segger Embedded Studio has a built-in RTT Viewer available when debugging the target code (go to
`Build -> Build and Debug`). Once debugging starts, the RTT communication with the device will be
available in the `Debug Terminal` window.

When using command line tools, the standalone J-Link RTT Viewer tool (included in the J-Link
toolchain) can be used to communicate with the device. Refer to the Segger J-Link documentation for
details on how to set up an RTT session using the J-Link RTT Viewer.
