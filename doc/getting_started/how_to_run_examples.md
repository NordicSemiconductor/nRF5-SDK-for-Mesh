# Running examples
@anchor examples_how_to_run_examples

This page describes how to run [examples included in the nRF5 SDK for Mesh package](@ref md_examples_README).

Just as with [installing the toolchain](@ref md_doc_getting_started_how_to_toolchain) and [building the Bluetooth mesh stack and examples](@ref md_doc_getting_started_how_to_build),
the procedure for running examples depends on the IDE.

Once you have an example running, you can [interact with it through command line with SEGGER RTT](@ref segger-rtt).

@note
- The following procedures are not applicable for the DFU example. See [Configuring DFU](@ref md_doc_user_guide_modules_dfu_configuring_performing) for details.
- For some examples, additional steps might be required. See the [documentation for each example](@ref md_examples_README) for more information.

**Table of contents**
- [Running examples using SEGGER Embedded Studio](@ref how_to_run_examples_ses)
- [Running examples in CMake-based build environment](@ref how_to_run_examples_cmake)
    - [Running examples using custom CMake target](@ref how_to_run_examples_cmake_custom)
    - [Running examples using nrfjprog](@ref how_to_run_examples_nrfjprog)
- [Interacting with examples using SEGGER RTT](@ref segger-rtt)


---


## Running examples using SEGGER Embedded Studio @anchor how_to_run_examples_ses

The following procedure only works if you have [built the example with SEGGER Embedded Studio](@ref how_to_build_segger_compiling_building).

To run the examples in a build environment based on SEGGER Embedded Studio (SES):
1. Connect the Development Kit with the USB cable to your computer.
2. In SES, connect to the development kit with **Target -> Connect J-Link**.
3. Erase the device from the SES options menu: **Target -> Erase all**.
4. Run the example with **Debug -> Go**. This downloads the matching SoftDevice
and the compiled example, and starts the debugger.
5. When the download is complete, select **Debug -> Go** again to start the code execution.

If the debugging does not start, reset the J-Link: **Target -> Reset J-Link**.

---


## Running examples in CMake-based build environment @anchor how_to_run_examples_cmake

After you [built the example with CMake](@ref how_to_build_cmake), you can run the example using either:
- [custom CMake target](@ref how_to_run_examples_cmake_custom)
- [nrfjprog](@ref how_to_run_examples_nrfjprog)

Both use files created by CMake to compile the project.

### Running examples using custom CMake target @anchor how_to_run_examples_cmake_custom

You can run custom CMake targets using either ninja or make.
Depending on you choice, you have to use different commands:

1. Connect a Development Kit to your computer with a USB cable.
2. Wait until the board is detected.
3. Flash the example by running one of the following commands from the `build` directory:
	- for ninja: `ninja flash_<target-name>`. Example:

			build $ ninja flash_light_switch_server_nrf52832_xxAA_s132_7.2.0

	- for make: `make flash_<target-name>`. Example:

			build $ make flash_light_switch_server_nrf52832_xxAA_s132_7.2.0

	@note
	Targets that flash examples start with "flash_".
	To list all the available targets, run:
		- when using `ninja` build tool: `ninja -t targets` or `ninja help`
		- when using `make`: `make help`

	After you issue the command to flash the example, the build system checks
    whether the example binaries are up-to-date.
	If required, it rebuilds them before flashing.
	It then displays a list of connected boards.
4. From the list of connected boards, choose the board to program the SoftDevice and example firmware onto.


### Running examples using nrfjprog @anchor how_to_run_examples_nrfjprog
Running examples using `nrfjprog` command line tool requires you to program a SoftDevice
and the example hex file to your board.

You need to know the path to SoftDevice binaries to run examples with `nrfjprog`.
The SoftDevice binaries are located in the `bin/softdevice/` folder. The example binaries are built
in the corresponding example folder, in the `build/` directory.

If you do not know the SoftDevice version that was used to build the Bluetooth mesh stack, check the name
of the example binary.
For example, if the example's binary name is `light_switch_client_nrf52832_xxAA_s132_7.2.0.hex`,
the required SoftDevice binary is `s132_nrf52_7.2.0_softdevice.hex`.

To run an example with `nrfjprog`:
1. Connect a Development Kit to your computer with a USB cable.
2. Wait until the board is detected.
3. Program the SoftDevice:
	1. Download the SoftDevice that you want to build the Bluetooth mesh stack with.
	2. Run the following command: `nrfjprog --program <path_to_the_example_binary_file> --chiperase`.
       Example:

			nrf5_sdk_for_mesh$ nrfjprog --program ./bin/softdevice/s132_nrf52_7.2.0_softdevice.hex --chiperase

4. Program the example application with the following command: `nrfjprog --program <path_to_the_example_binary_file> --sectorerase`. Example:

		nrf5_sdk_for_mesh$ nrfjprog --program ./build/examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_7.2.0.hex --sectorerase

5. Launch the example by using one of the following options:
	- power the device off and on;
	- initiate a soft-reset by using the following command: `nrfjprog -r`. Soft-reset is particularly
    useful if you want to use the debugger or RTT viewer (see the following section)
    and prevent disconnection of the RTT link.


---


## Interacting with examples using SEGGER RTT @anchor segger-rtt

The nRF5 SDK for Mesh examples can communicate with a host computer through @link_rtt,
and several examples require or allow you to connect RTT viewer to observe output generated
in the RTT log.

The RTT viewer is available as:
- built-in feature of SEGGER Embedded Studio (SES)
- standalone application: J-Link RTT Viewer

Only the standalone application allows you to issue commands through RTT, for example
when testing @ref md_examples_dimming_README.

### Displaying RTT output in SES @anchor segger-rtt_ses

SEGGER Embedded Studio (SES) has a built-in RTT Viewer available when debugging the target code.

To see the RTT output generated when using SES, [build](@ref how_to_build_segger_compiling_building)
and [run](@ref how_to_run_examples_ses) the example.
Once debugging starts, the RTT communication with the device will be available
in the Debug Terminal window.

### Displaying RTT output in standalone J-Link RTT Viewer @anchor segger-rtt_standalone

nRF5x Command Line Tools come with the standalone J-Link RTT Viewer tool that can be used when using
both CMake or SEGGER.

To see the RTT log of single development boards in this tool, make sure you connect the nRF5 boards
to the USB ports.
When the boards are connected, complete the following steps for each board:
1. Start J-Link RTT viewer. The Configuration window appears.
@note You can also press the **F2** button or select **File > Connect** to open the Configuration window.
2. In the Configuration window,
depending on the [development kit board chip number](@ref compatibility_list) you are using,
make sure that the appropriate SoC is selected in the Specify Target Device dropdown menu.
3. Click **OK**. The Emulator selection window appears.
4. Choose the desired board by selecting its USB Identification (SEGGER ID).

After flashing the example firmware and running the example, you will see output printed
in the RTT log while testing.

Refer to @link_segger_jlink for more details about how to set up an RTT session
using the J-Link RTT Viewer.
