# Running examples

This page describes how to run [examples included in the nRF5 SDK for Mesh package](@ref md_examples_README).

@anchor examples_how_to_run_examples
Just as with [installing the toolchain](@ref md_doc_getting_started_how_to_toolchain) and [building the mesh stack and examples](@ref md_doc_getting_started_how_to_build),
the procedure for running examples depends on the IDE:
- [Running examples using SEGGER Embedded Studio](@ref how_to_run_examples_ses)
- [Running examples in CMake-based build environment](@ref how_to_run_examples_cmake)

Once you have an example running, you can [interact with it through command line with SEGGER RTT](@ref segger-rtt).

@note
- The following procedures are not applicable for the DFU example. See [Configuring DFU](@ref md_doc_getting_started_dfu_quick_start) for details.
- For some examples, additional steps might be required. See the [documentation for each example](@ref md_examples_README) for more information.


---


## Running examples using SEGGER Embedded Studio @anchor how_to_run_examples_ses
To run the examples in a build environment based on SEGGER Embedded Studio (SES):
1. Connect a Development Kit to your computer with a USB cable.
2. Wait until the board is detected.
3. Erase the chip from the SES options menu: **Target -> Erase all**.
4. Flash and run the example from the SES options menu: **Target -> Download**. This flashes
both the necessary SoftDevice and the application binary.


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
	
			build $ ninja flash_light_switch_server_nrf52832_xxAA_s132_6.1.0
			
	- for make: `make flash_<target-name>`. Example:
			
			build $ make flash_light_switch_server_nrf52832_xxAA_s132_6.1.0
			
	@note
	Targets that flash examples start with "flash_".
	To list all the available targets, run:
		- when using `ninja` build tool: `ninja -t targets` or `ninja help`
		- when using `make`: `make help`
	
	After you issue the command to flash the example, the build system checks whether the example binaries are up-to-date. 
	If required, it rebuilds them before flashing.
	It then displays a list of connected boards.
4. From the list of connected boards, choose the board to program the SoftDevice and example firmware onto.


### Running examples using nrfjprog @anchor how_to_run_examples_nrfjprog
Running examples using `nrfjprog` command line tool requires you to program a SoftDevice and the example hex file to your board.

You need to know the path to SoftDevice binaries to run examples with `nrfjprog`.
The SoftDevice binaries are located in the `bin/softdevice/` folder. The example binaries are built in the corresponding example folder, in the `build/` directory.

If you do not know the SoftDevice version that was used to build the mesh stack, check the name of the example binary.
For example, if the example's binary name is `light_switch_client_nrf52832_xxAA_s132_6.1.0.hex`, the required SoftDevice binary is `s132_nrf52_6.1.0_softdevice.hex`.

To run an example with `nrfjprog`:
1. Connect a Development Kit to your computer with a USB cable.
2. Wait until the board is detected.
3. Program the SoftDevice:
	1. Download the SoftDevice that you want to build mesh stack with.
	2. Run the following command: `nrfjprog --program <path_to_the_example_binary_file> --chiperase`. Example:
	
			nrf5_sdk_for_mesh$ nrfjprog --program ./bin/softdevice/s132_6.1.0/s132_nrf52_6.1.0_softdevice.hex --chiperase
		
4. Program the example application with the following command: `nrfjprog --program <path_to_the_example_binary_file> --sectorerase`. Example: 

		nrf5_sdk_for_mesh$ nrfjprog --program ./build/examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_6.1.0.hex --sectorerase
		
5. Launch the example by using one of the following options:
	- power the device off and on;
	- initiate a soft-reset by using the following command: `nrfjprog -r`. Soft-reset is particularly useful if you want to use the debugger or RTT viewer (see the following section) and prevent disconnection of the RTT link.


---

		
## Command line interaction with examples @anchor segger-rtt

The examples can communicate with a host computer through @link_rtt<!--https://www.segger.com/products/debug-probes/j-link/technology/real-time-transfer/about-real-time-transfer/-->.

SEGGER Embedded Studio has a built-in RTT Viewer available when debugging the target code (go to
`Build -> Build and Debug`). Once debugging starts, the RTT communication with the device will be
available in the `Debug Terminal` window.

When using command line tools, you can use the standalone J-Link RTT Viewer tool (included in the J-Link
toolchain) to communicate with the device. Refer to @link_jlink_docs <!--https://www.segger.com/downloads/jlink\--> for
details on how to set up an RTT session using the J-Link RTT Viewer.
