# Building the mesh stack and examples

The mesh library and example applications can be built with either SEGGER Embedded Studio or CMake.
Before you start building, remember to set up the mesh development environment first. See @ref md_doc_getting_started_how_to_toolchain for details.

**Table of contents**
- [Building with SEGGER Embedded Studio](@ref how_to_build_segger)
	- [First time setup](@ref how_to_build_segger_setup)
	- [Compiling and building with SES](@ref how_to_build_segger_compiling_building)
- [Building with CMake](@ref how_to_build_cmake)
	- [Generating build files](@ref how_to_build_cmake_generating)
		- [Customization](@ref how_to_build_cmake_generating_customization)
	- [Building the stack and examples](@ref how_to_build_cmake_building)
	- [Generating SEGGER Embedded Studio project files](@ref how_to_build_cmake_generating_SES)
	- [Useful CMake command line options](@ref how_to_build_cmake_command_line)
	- [Creating a new build target](@ref how_to_build_cmake_creating_target)
	- [Building documentation](@ref how_to_build_cmake_docs)
	- [Building and running unit tests (host)](@ref how_to_build_cmake_unit_tests)


---


## Building with SEGGER Embedded Studio @anchor how_to_build_segger

@link_seggerstudio <!--SEGGER Embedded Studio: https://www.segger.com/products/development-tools/embedded-studio/?L=0-->(SES) provides a way of quickly getting the example code up and running with full debug capability.

### First time setup @anchor how_to_build_segger_setup

Before building the mesh examples with SEGGER Embedded Studio for the first time, you must complete a one-time setup of the `SDK_ROOT` macro
in SEGGER Embedded Studio. This macro is used to find the nRF5 SDK files.

You can either:
- Use the default settings of the `SDK_ROOT` macro. It defaults to an nRF5 SDK 15.2.0 instance unzipped right next to the mesh
folder.
- Set the `SDK_ROOT` macro to a custom nRF5 SDK instance.

To set the `SDK_ROOT` macro from the command line to wherever the nRF5 SDK is, type:
`set SDK_ROOT=<path_to_nRF5_SDK_instance>`.

To set the `SDK_ROOT` macro manually in SEGGER Embedded Studio:
1. Go to **Tools -> Options**.
2. Select **Building**.
3. Under **Build** in the configuration list, edit **Global macros** to
contain `SDK_ROOT=<the path to nRF5 SDK instance>`.
4. Save the configuration.

You can verify the path by opening one of the source files under the nRF5 SDK
file group. If the macro is set correctly, the file opens in the editor
window. If not, an error message is displayed with information that the file cannot
be found.

For more info on SEGGER Embedded Studio macros, see the @link_seggerstudio_macros <!--SES Project macros: https://studio.segger.com/ide_project_macros.htm -->page.


### Compiling and building with SES @anchor how_to_build_segger_compiling_building

To build with SEGGER Embedded Studio, open one of the project files located in the `examples/` folder,
for instance `examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_6_1_0.emProject`.

To compile an example:
1. Go to **Build -> Build light_switch_client_nrf52832_xxAA_s132_6.1.0**. 
2. Wait for the compilation to finish.
3. Erase the device using **Target -> Erase all**.
4. Run the example with **Debug -> Go**. This downloads the matching SoftDevice and the compiled example and starts the debugger.

When the download is complete select **Debug -> Go** again to start the code execution.


---


## Building with CMake @anchor how_to_build_cmake

@link_cmake <!--CMake: https://cmake.org/--> provides the possibility to build both for host (unit tests) and target.

CMake is "an extensible, open-source system that manages the build process in an operating system and
in a compiler-independent manner" (as stated on the @link_about_cmake<!-- https://cmake.org/overview/ -->).

In other words, CMake does not build from the source directly, but generates the native build tool
files (for example, a set of Makefiles or a `build.ninja` configuration). The choice of which build tool is to be targeted is
controlled with the `-G` argument, for example: `-G Ninja`, `-G "Unix Makefiles"` and many more. CMake can
generate IDE project files for IDEs such as Eclipse as well. However, this guide only targets Ninja and GNU Make.

@note
All examples built by the CMake-generated build system do not include the SoftDevice
as part of the generated HEX files. Therefore, the SoftDevice must already be present on the device before
flashing the HEX file for the example mesh application.

When building with CMake, remember the following tips:
- **Tip 1:** There is a `merge_<target>` for each of the example
targets that uses `mergehex` to generate a hexfile with the application and SoftDevice merged. Take special care
with the order of programming when the application has bootloader support.
Follow the @ref md_doc_getting_started_dfu_quick_start in these cases.
Example use:

     build $ ninja merge_light_switch_server_nrf52832_xxAA_s132_6.1.0

- **Tip 2:** There is a `flash_<target>` for each of the example targets that runs an interactive
programming tool. This requires the nRFx Command Line tools and Python 3 to be installed (see @ref md_doc_getting_started_how_to_toolchain).
Example use:

     build $ ninja flash_light_switch_server_nrf52832_xxAA_s132_6.1.0



### Generating build files @anchor how_to_build_cmake_generating

This section describes generating build files for the Ninja build tool.

@note
On Debian/Linux, you may drop the `-G Ninja` argument as the default generator is for Unix Makefiles
and use the `make` command instead of `ninja`.

Good practice is to create a build folder in the root directory for the mesh stack repository, where all
artifacts generated by the Ninja build system are stored:

    nrf5_sdk_for_mesh $ mkdir build
    nrf5_sdk_for_mesh $ cd build

Before you can build with Ninja, you must generate the correct build files with CMake.

Calling `cmake -G Ninja` with no parameters will default to the `nrf52832_xxAA` platform with the `s132_6.1.0` SoftDevice and `gccarmemb` toolchain:

    build$ cmake -G Ninja ..


@note
You can also use the `cmake-gui -GNinja ..` command to open the CMake graphical user interface
when configuring the SDK. Press **Configure** and then **Generate** to generate the
build files.

#### Customization @anchor how_to_build_cmake_generating_customization

You can specify the required `TOOLCHAIN` and `PLATFORM` name to CMake. The build system will ensure
a valid `BOARD` and `SOFTDEVICE` combination for each given platform.

    build$ cmake -G Ninja -DTOOLCHAIN=<toolchain> -DPLATFORM=<platform> ..

	
Possible options for the `toolchain` and `platform`:

- `toolchain`
   - `gccarmemb` for the GNU ARM Embedded toolchain
   - `armcc` for the Keil ARMCC toolchain
   - `clang` for the Clang compiler (with GNU ARM Embedded assembler and linker)
- `platform`
   - `nrf51422_xxAC` (deprecated and no longer officially supported)
   - `nrf52810_xxAA`
   - `nrf52832_xxAA`
   - `nrf52840_xxAA`

**Example:** To build mesh stack for nRF52 DK with GNU ARM Embedded toolchain, use:

	build$ cmake -G Ninja -DTOOLCHAIN=gccarmemb -DPLATFORM=nrf52832_xxAA ..

You can also customize `BOARD` and `SOFTDEVICE` options with `-D` command line switches:
- `BOARD`: valid board combination based on platform type. You can choose one of the values from `nrf5_sdk_for_mesh/CMake/board`.
- `SOFTDEVICE`: valid SoftDevice based on platform type. You can choose one of the values from `nrf5_sdk_for_mesh/CMake/softdevice`.


### Building the stack and examples @anchor how_to_build_cmake_building

After the Ninja build files are generated,
running `ninja` will build all the targets (examples and libraries).

If you have PC-Lint installed, the sources can be linted using the `ninja lint` command.

To see a list of available build targets, run the following command:

    build $ ninja help

**Example:** To build a specific target from this list with the current platform `nrf52832_xxAA` and the `s132_6.1.0` SoftDevice, run:

	ninja light_switch_server_nrf52832_xxAA_s132_6.1.0
	

CMake generates Ninja build files in the folder in which CMake is run,
so all targets must be built from that directory. In other words, in-directory building is not supported
and running `ninja` in one of the example folders results in an error message generated by the Ninja build system.

### Generating SEGGER Embedded Studio project files @anchor how_to_build_cmake_generating_SES

**Warning:** The generator will overwrite any existing projects. Back up existing projects before
running the generator.

It is possible to generate SEGGER Embedded Studio project files using the CMake build system.
With the option `GENERATE_SES_PROJECTS` enabled, CMake will generate a SES project
based on the current settings.

**Example:** To generate a project for `nrf52832_xxAA` using the S132 v6.1.0 SoftDevice, run CMake in your build directory:

    cmake -G Ninja -DGENERATE_SES_PROJECTS=ON -DPLATFORM=nrf52832_xxAA -DSOFTDEVICE=s132_6.1.0 ..

	
### Useful CMake command line options @anchor how_to_build_cmake_command_line

CMake allows you to generate project files in release or debug configurations. To do so,
use the `-DCMAKE_BUILD_TYPE` option:

    build $ cmake -DCMAKE_BUILD_TYPE=Release ..      # Generates build files in release mode
    build $ cmake -DCMAKE_BUILD_TYPE=Debug ..        # Generates build files in debug mode
    build $ cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..   # Generates build files optimized for size

The default build type is `Debug` if the CMake project is a Git repository (contains a `.git` directory).
Otherwise, it is set to `RelWithDebInfo`.

### Creating a new build target @anchor how_to_build_cmake_creating_target

To create a new build target:

1. Copy one of the example folders, for example `examples/beaconing` to `examples/my_app`.
2. Add the folder to the `examples/CMakeLists.txt` with the `add_subfolder("my_app")` command.
3. Modify the target name in the first line of `examples/my_app/CMakeLists.txt` to `set(target "my_app")`.
4. Build your new target with the following command:

        build $ ninja my_app


### Building documentation @anchor how_to_build_cmake_docs

@note
Building documentation requires additional command line tools.
See [the list on Installing the toolchain page](@ref toolchain_cmake_docs).

To build all documentation (API documentation and internal documentation), call the build system with the target `doc`.

    build $ ninja doc

The Doxygen documentation is generated in `<build folder>/doc/offline/html`.


### Building and running unit tests (host) @anchor how_to_build_cmake_unit_tests

@note
Building unit tests is optional and requires additional tools.
See [the table of required tools on Installing the toolchain page](@ref toolchain_cmake_optional).

To build units tests:

1. Enter the `nrf5_sdk_for_mesh` directory and make a new build directory, for example `build_host`:

		nrf5_sdk_for_mesh $ mkdir -p build_host && cd build_host
	
2. Set the option `BUILD_HOST` to `ON` and `CMAKE_BUILD_TYPE` to `Debug`:

		build_host $ cmake -G Ninja -DBUILD_HOST=ON -DCMAKE_BUILD_TYPE=Debug ..
	
	@note
	- CMake is set up to look for CMock in the directory above the `nrf5_sdk_for_mesh` folder. If it is _not_ located next to the mesh folder, you can specify its path by passing `-DCMOCK_ROOT=<dir/cmock>`.
	- If a different version of Unity is used than the one included as a submodule in CMock, you can specify its path by passing `-DUNITY_ROOT=<dir/unity>`.
	- All paths given to CMake must use forward slashes ('/') as directory separators.
	
3. Build all the unit tests with ninja:

		build_host $ ninja

	

	
**Running unit tests**<br>
To run the tests, you can either:
- run `ctest` (bundled with CMake):
	
		build_host $ ctest # Run all unit tests

- call `ninja test` in the build directory:

		build_host $ ninja test
    

	