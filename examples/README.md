# Examples {#example-projects}

The nRF5 SDK for Mesh provides several example projects to demonstrate certain features and
to help you get started on new mesh-based projects.

The following examples are provided:

* @subpage md_examples_light_switch_README    "Light switch" - A reference example
  that shows how to control a group of lights from a light switch. The example provides both the
  client (light switch) and server (lights) implementations and shows how to
  use a [custom model](@ref md_models_simple_on_off_README) in an application.
* @subpage md_examples_pb_remote_client_README "Remote provisioning client" - An
  example of a provisioner that uses remote provisioning to provision devices outside
  of radio range.
* @subpage md_examples_pb_remote_server_README "Remote provisioning server" - An
  example of a device that provides an end-point for relayed provisioning packets when
  using the remote provisioning feature to provision devices.
* @subpage md_examples_beaconing_README        "Beaconing" - An examples that implements custom beacon advertising
  and shows how to send and receive custom packets using the nRF5 SDK for Mesh.
* @subpage md_examples_dfu_README              "DFU over mesh" - An example that shows
  how to use the mesh DFU framework to update the firmware of a device over the mesh.
* @subpage md_examples_serial_README           "Serial" - An example that shows how to
  use the serial interface to create a mesh connectivity device.

Example models are collected in the @subpage md_models_README folder, while common example utility
modules are collected in @subpage md_examples_common_README.

A simple [hardware abstraction layer](@ref md_examples_common_README) is shared by the examples.

## How to build examples

To build the examples, follow the instructions in [Building the Mesh Stack](@ref md_doc_getting_started_how_to_build).

## How to run examples @anchor examples_how_to_run_examples

To program examples onto a Development Kit, first connect it to your computer with a USB cable. When your board is detected, you can program examples as described below.

### Using Segger Embedded Studio
If you are using Segger Embedded Studio to run the examples, first erase the chip using the `Target -> Erase all`
menu option. Then the examples can be flashed and run using the `Target -> Download` option, which will flash
both the necessary SoftDevice and the application binary.

### Using command line tools
Downloading examples using `nrfjprog` command line tool is a three step process. You will need to program a SoftDevice and the example hex file to your board.

The SoftDevice binaries are located in the `/external/softdevice/` folder, and example binaries will be built in the corresponding example folders within the `/build/` directory.

#### 1. Program the SoftDevice
Download the SoftDevice which you chose to build mesh stack with. If you do not know the SoftDevice version that was used to build the mesh stack, check the name of the example binary.

For example, if the example's binary name is `light_switch_client_nrf52832_xxAA_s132_5.0.0.hex`, the required SoftDevice binary is `s132_nrf52_5.0.0_softdevice.hex`. To program this example run:

    mesh-mbtle$ nrfjprog --program ./external/softdevice/s132_5.0.0/s132_nrf52_5.0.0_softdevice.hex --chiperase -f NRF52

#### 2. Program example application
To program the example binary mentioned above, run:

    mesh-mbtle$ nrfjprog --program ./build/examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_5.0.0.hex --sectorerase -f NRF52

#### 3. Reset the device
To launch the example, either power cycle the device or initiate a soft-reset. Soft-reset is particularly useful if you want to setup the debugger or RTT viewer just before launching the example and prevent disconnection of the RTT link. You can initiate a soft-reset by using the following command:

    mesh-mbtle$ nrfjprog -r -f NRF52

For some examples, additional steps might be required. See the Readme file for
each example for more information before running it.


