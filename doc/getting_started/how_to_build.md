# Building the mesh stack and examples

The mesh library and example applications can be built with either SEGGER Embedded Studio or CMake.

Before you start building, remember to set up the mesh development environment first. See @ref md_doc_getting_started_how_to_toolchain for details.

@note The building process changes slightly if you are [adding custom examples](@ref examples_adding_building).

**Table of contents**
- [Building with SEGGER Embedded Studio](@ref how_to_build_segger)
	- [First time setup](@ref how_to_build_segger_setup)
	- [Building with SES](@ref how_to_build_segger_compiling_building)
- [Building with CMake](@ref how_to_build_cmake)
	- [Step 1: Generating build files](@ref how_to_build_cmake_generating)
		- [Customization](@ref how_to_build_cmake_generating_customization)
	- [Step 2: Building the stack and examples](@ref how_to_build_cmake_building)
	- [Step 3: Generating SEGGER Embedded Studio project files](@ref how_to_build_cmake_generating_SES)
	- [Additional CMake options](@ref how_to_build_cmake_optional)
        - [Useful CMake command line options](@ref how_to_build_cmake_command_line)
        - [Building documentation](@ref how_to_build_cmake_docs)
        - [Building and running unit tests (host)](@ref how_to_build_cmake_unit_tests)


---


## Building with SEGGER Embedded Studio @anchor how_to_build_segger

@link_seggerstudio (SES) provides a way of quickly getting the example code up and running with full debug capability.

### First time setup @anchor how_to_build_segger_setup

Before building the mesh examples with SEGGER Embedded Studio for the first time, you must complete a one-time setup of the `SDK_ROOT` macro
in SEGGER Embedded Studio. This macro is used to find the nRF5 SDK files.

You can either:
- Use the default settings of the `SDK_ROOT` macro. It defaults to an nRF5 SDK instance unzipped right next to the mesh
folder.
- Set the `SDK_ROOT` macro to a custom nRF5 SDK instance.

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

For more info on SEGGER Embedded Studio macros, see the @link_seggerstudio_macros page.


### Building with SES @anchor how_to_build_segger_compiling_building

By default, the nRF5 SDK for Mesh package includes the SES project files for all examples.
This allows you to quickly start building examples with SES.

However, if you make changes to any of the `CMakeLists.txt` files,
you must [generate the SES project files again using CMake](@ref how_to_build_cmake_generating_SES).

To build an example with SEGGER Embedded Studio:
1. Open the desired project file located in the `examples/` folder,
for instance `examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_6_1_1.emProject`.
2. Go to **Build -> Build < name of the emProject file>**, for instance **Build light_switch_client_nrf52832_xxAA_s132_7.0.1**.
3. Wait for the compilation to finish.

You can now [run the example using SEGGER Embedded Studio](@ref how_to_run_examples_ses).


---


## Building with CMake @anchor how_to_build_cmake

@link_cmake provides the possibility to build both for host (unit tests) and target.

CMake is "an extensible, open-source system that manages the build process in an operating system and
in a compiler-independent manner" (as stated on the @link_cmake website).

In other words, CMake does not build from the source directly, but generates the native build tool
files (for example, a set of Makefiles or a `build.ninja` configuration). The choice of which build tool is to be targeted is
controlled with the `-G` argument, for example: `-G Ninja`, `-G "Unix Makefiles"` and many more. CMake can
generate IDE project files for IDEs such as Eclipse as well. However, this guide only targets Ninja and GNU Make.

@par SoftDevice requirement
All examples built by the CMake-generated build system do not include the SoftDevice
as part of the generated HEX files. Therefore, the SoftDevice must already be present on the device before
flashing the HEX file for the example mesh application.

@par mergehex exception
There is a `merge_<target>` for each of the example targets that uses `mergehex` to generate a hexfile with the application and SoftDevice merged.
Take special care with the order of programming when the application has bootloader support.
Follow the procedure in @ref md_doc_libraries_dfu_dfu_quick_start in these cases.
    - Example use:

            build $ ninja merge_light_switch_server_nrf52832_xxAA_s132_7.0.1


@par Interactive programming tool requirement
There is a `flash_<target>` for each of the example targets that run an interactive programming tool.
This requires the nRFx Command Line tools and Python 3 to be installed (see @ref md_doc_getting_started_how_to_toolchain).
    - Example use:

            build $ ninja flash_light_switch_server_nrf52832_xxAA_s132_7.0.1

There is also a `sector_<target>`, which runs the interactive programming tool
in `--sectorerase` mode.
    - Flashes device with `<target>.hex`, not `<target>_merged.hex`
    - Does not touch UICR, SoftDevice, or flash pages
    - Useful for pushing fixes without losing device provisioning
    - Example use:

            build $ ninja sector_light_switch_server_nrf52832_xxAA_s132_7.0.1

The following three steps are mandatory when building with CMake:
- [Step 1: Generating build files](@ref how_to_build_cmake_generating)
- [Step 2: Building the stack and examples](@ref how_to_build_cmake_building)
- [Step 3: Generating SEGGER Embedded Studio project files](@ref how_to_build_cmake_generating_SES)

Using CMake for building also comes with useful [additional options](@ref how_to_build_cmake_optional).


### Step 1: Generating build files @anchor how_to_build_cmake_generating

This section describes generating build files for the Ninja build tool.

@note
On Debian/Linux, you may drop the `-G Ninja` argument as the default generator is for Unix Makefiles
and use the `make` command instead of `ninja`.

Good practice is to create a build folder in the root directory for the mesh stack repository, where all
artifacts generated by the Ninja build system are stored:

    nrf5_sdk_for_mesh $ mkdir build
    nrf5_sdk_for_mesh $ cd build

Before you can build with Ninja, you must generate the correct build files with CMake.

Calling `cmake -G Ninja` with no parameters will default to the `nrf52832_xxAA` platform with the `s132_7.0.1` SoftDevice and `gccarmemb` toolchain:

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
   - `nrf51422_xxAC` ([deprecated support](@ref compatibility_nRF51))
   - `nrf52810_xxAA`
   - `nrf52832_xxAA`
   - `nrf52833_xxAA`
   - `nrf52840_xxAA`

**Example:** To build mesh stack for nRF52 DK with GNU ARM Embedded toolchain, use:

	build$ cmake -G Ninja -DTOOLCHAIN=gccarmemb -DPLATFORM=nrf52832_xxAA ..

You can also customize `BOARD` and `SOFTDEVICE` options with `-D` command line switches:
- `BOARD`: valid board combination based on platform type. You can choose one of the values from `nrf5_sdk_for_mesh/CMake/board`.
- `SOFTDEVICE`: valid SoftDevice based on platform type. You can choose one of the values from `nrf5_sdk_for_mesh/CMake/softdevice`.


### Step 2: Building the stack and examples @anchor how_to_build_cmake_building

After the Ninja build files are generated,
running `ninja` will build all the targets (examples and libraries).

If you have PC-Lint installed, the sources can be linted using the `ninja lint` command.

To see a list of available build targets, run the following command:

    build $ ninja help

**Example:** To build a specific target from this list with the current platform `nrf52832_xxAA` and the `s132_7.0.1` SoftDevice, run:

	ninja light_switch_server_nrf52832_xxAA_s132_7.0.1


CMake generates Ninja build files in the folder in which CMake is run,
so all targets must be built from that directory. In other words, in-directory building is not supported
and running `ninja` in one of the example folders results in an error message generated by the Ninja build system.

### Step 3: Generating SEGGER Embedded Studio project files @anchor how_to_build_cmake_generating_SES

@warning
The generator will overwrite any existing projects. Back up existing projects before
running the generator.

It is possible to generate SEGGER Embedded Studio project files using the CMake build system.
With the option `GENERATE_SES_PROJECTS` enabled, CMake will generate a SES project
based on the current settings.

**Example:** To generate a project for `nrf52832_xxAA` using the S132 v7.0.1 SoftDevice, run CMake in your build directory:

    cmake -G Ninja -DGENERATE_SES_PROJECTS=ON -DPLATFORM=nrf52832_xxAA -DSOFTDEVICE=s132_7.0.1 ..


### Additional CMake options @anchor how_to_build_cmake_optional

The following procedures can be useful when working with CMake:
- [Useful CMake command line options](@ref how_to_build_cmake_command_line)
- [Building documentation](@ref how_to_build_cmake_docs)
- [Building and running unit tests (host)](@ref how_to_build_cmake_unit_tests)

#### Useful CMake command line options @anchor how_to_build_cmake_command_line

CMake allows you to generate project files in release or debug configurations. To do so,
use the `-DCMAKE_BUILD_TYPE` option:

    build $ cmake -DCMAKE_BUILD_TYPE=Release ..      # Generates build files in release mode
    build $ cmake -DCMAKE_BUILD_TYPE=Debug ..        # Generates build files in debug mode
    build $ cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..   # Generates build files optimized for size

The default build type is `Debug` if the CMake project is a Git repository (contains a `.git` directory).
Otherwise, it is set to `RelWithDebInfo`.

#### Building documentation @anchor how_to_build_cmake_docs

@note
Building documentation requires additional command line tools.
See [the list on Installing the toolchain page](@ref toolchain_cmake_docs).

To build all documentation (API documentation and internal documentation), call the build system with the target `doc`.

    build $ ninja doc

The Doxygen documentation is generated in `<build folder>/doc/offline/html`.


#### Building and running unit tests (host) @anchor how_to_build_cmake_unit_tests

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




To run the tests, you can either:
- run `ctest` (bundled with CMake):

		build_host $ ctest # Run all unit tests

- call `ninja test` in the build directory:

		build_host $ ninja test



