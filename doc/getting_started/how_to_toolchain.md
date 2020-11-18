# Installing the toolchain

To build the example applications, you need a toolchain based on either CMake or SEGGER Embedded Studio.
Install instructions are provided for Windows and Debian/Ubuntu. The steps should be similar for
other platforms.

**Table of contents**
- [Build environment based on SEGGER Embedded Studio](@ref toolchain_build_environment_ses)
- [Build environment based on CMake](@ref toolchain_build_environment_cmake)
    - [Installing CMake on Windows](@ref toolchain_cmake_windows)
    - [Installing CMake on Debian/Ubuntu](@ref toolchain_cmake_debian)
        - [Installing Python on Debian/Ubuntu](@ref toolchain_cmake_debian_python)
	- [Additional tools for building documentation](@ref toolchain_cmake_docs)
	- [Optional: Additional tools for building unit tests (host)](@ref toolchain_cmake_optional)
- [Downloading nRF5 SDK](@ref how_to_build_nrf_sdk)
	- [Downloading nRF5 SDK manually](@ref how_to_build_nrf_sdk_manual)
	- [Downloading nRF5 SDK using a custom CMake target](@ref how_to_build_nrf_sdk_custom)


**Important note about Python**<br>
Python is _not_ required to build the Bluetooth mesh stack and examples. The nRF5 SDK for Mesh uses both @link_python35_download and @link_python27_download.

Tasks that require Python 3:
- working with @ref md_scripts_interactive_pyaci_README tool
- generating SEGGER Embedded Studio projects
- building documentation

Tasks that require Python 2:
- starting DFU transfer with @link_nrfutil_github_name. This is a legacy serial tool written for py2.

Remember to @link_python_path_tutorial both versions of Python on Windows.


---


## Build environment based on SEGGER Embedded Studio @anchor toolchain_build_environment_ses

To use SEGGER Embedded Studio, download the installer from the @link_segger_website
and follow the installation instructions. You will find project files for each of the examples in
their respective folders.

Moreover, you must install the following tools.

 | Download link                | Recommended *minimum* version | Installation notes                               |
 |------------------------------|-------------------------------|--------------------------------------------------|
 | @link_segger_jlink           | 6.16a                         |                                                  |
 | @link_python27_download      | 2.7                           | Required for DFU.                                |
 | @link_python35_download      | 3.5.1                         | Must be 32-bit for `nrfjprog` DLL to work. Ensure that `pip` is installed and that Python 3 is added to `PATH`.<br>Also required for @ref md_scripts_interactive_pyaci_README.     |
 | @link_nrf5SDK_download       | 17.0.2                        | Required for [building with SEGGER Embedded Studio](@ref how_to_build_segger). See [Downloading nRF5 SDK](@ref how_to_build_nrf_sdk).	|


---


## Build environment based on CMake @anchor toolchain_build_environment_cmake

As an alternative to SEGGER Embedded Studio, @link_cmake
is a build management system used for managing an environment that is independent of the
compiler and build system used. Version 3.6 or above is required by the Bluetooth mesh stack.

Required tools depend on your operating system:
- [Installing CMake on Windows](@ref toolchain_cmake_windows)
- [Installing CMake on Debian/Ubuntu](@ref toolchain_cmake_debian)

Additionally, if you want to build:
- **documentation**: install [additional tools for building documentation](@ref toolchain_cmake_docs);
- **unit tests**: install [optional, additional tools for building unit tests](@ref toolchain_cmake_optional).


### Installing CMake on Windows @anchor toolchain_cmake_windows

The following tools are required if you want to work with the nRF5 SDK for Mesh using CMake on Windows.

 | Download link                | Recommended *minimum* version | Installation notes                               |
 |------------------------------|-------------------------------|--------------------------------------------------|
 | @link_nrf5xclt_download      | 9.5.0                         | Ensure that all command line tools are available in a folder referenced by the system path (for example, the `PATH` environment variable).   |
 | @link_segger_jlink           | 6.16a                         |                                                  |
 | @link_python27_download      | 2.7                           | Required for DFU.                                |
 | @link_python35_download      | 3.5.1                         | Must be 32-bit for `nrfjprog` DLL to work. Ensure that `pip` is installed and that Python 3 is added to `PATH`.      |
 | @link_cmake_download         | 3.9.0                         | Download the latest installer and follow the installation instructions. |
 | @link_ninja_download         | 1.7.2                         | Preferred build system on Windows. Download the binary and place it in a suitable folder. |
 | @link_armnone_extended       | 9-2019-q4-major (9.2.1)       | One of two alternative build systems available on Windows. Download the @link_armnone installer and follow the installation instructions.  |
 | @link_keil_extended          | 5                             | The other alternative build system available on Windows. Follow the instructions provided for @link_armcc. The armcc v5 toolchain is also provided by @link_keil and comes bundled with the @link_keiluvision.   |
 | @link_nrf5SDK_download       | 17.0.2                        | Required for [building with CMake](@ref how_to_build_cmake). See [Downloading nRF5 SDK with CMake](@ref how_to_build_nrf_sdk).	|

You can also install [optional, additional tools for building unit tests](@ref toolchain_cmake_optional).


---


### Installing CMake on Debian/Ubuntu @anchor toolchain_cmake_debian

For Debian/Ubuntu, most tools are available from the system package manager `apt`.

The following tools are required if you want to work with the nRF5 SDK for Mesh using CMake on Debian/Ubuntu.

 | Download link                | Recommended *minimum* version | Installation notes                               |
 |------------------------------|-------------------------------|--------------------------------------------------|
 | @link_nrf5xclt_download      | 9.5.0                         | Reload the udev rules after installing the nRF5x Command Line Tools with the following commands:<br><br>`sudo udevadm control --reload`<br>`sudo udevadm trigger --action=add`<br><br>Ensure that all command line tools are available in a folder referenced bythe system path (for example, the `PATH` environment variable). |
 | @link_segger_jlink           | 6.16a                         |                                                  |
 | @link_python27_download      | 2.7                           | Required for DFU. See the [Installing Python on Debian/Ubuntu](@ref toolchain_cmake_debian_python) section below.                                |
 | @link_python35_download      | 3.5.1                         | Ensure that `pip` is installed and that Python 3 is added to `PATH`. See the [Installing Python on Debian/Ubuntu](@ref toolchain_cmake_debian_python) section below.     |
 | @link_cmake_download         | 3.9.0                         | For Ubuntu versions older than `zesty`, a manual installation of CMake is required as the version available in the package manager is older than 3.6. Visit @link_cmake to download the latest release and follow the installation instructions.<br>- Install CMake with the following command: `sudo apt-get install cmake cmake-curses-gui`<br>- Ensure that your CMake version is at least 3.6 with the following command: `cmake --version` |
 | @link_armnone_extended       | 9-2019-q4-major (9.2.1)       | Starting with Ubuntu 20.04, install the toolchain in the following way (alongside GDB, the GNU Debugger for ARM):<br><br>`sudo apt-get install gcc-arm-none-eabi`  |
 | @link_make                   | -                             | Default build system on Debian/Ubuntu. Usually comes with the distribution. As an alternative, you can use Ninja.
 | @link_ninja_download         | 1.7.2                         | Alternative build system on Debian/Ubuntu. You can install it with the following command: `sudo apt-get install ninja-build`  |
 | @link_nrf5SDK_download       | 17.0.2                        | Required for [building with CMake](@ref how_to_build_cmake). See [Downloading nRF5 SDK with CMake](@ref how_to_build_nrf_sdk).	|

You can also install [optional, additional tools for building unit tests](@ref toolchain_cmake_optional).

#### Installing Python on Debian/Ubuntu @anchor toolchain_cmake_debian_python

The default Python version that comes with most Linux distributions is Python 2.7, but the
nRF5 SDK for Mesh requires Python 3.5. It is recommended to use `virtualenv` to manage Python versions.
It makes managing Python settings across different projects easy.

To install Python:

1. Run the following command `$ sudo apt-get install virtualenv`
2. Make a directory to keep your virtual environments in and create a new environment for Bluetooth mesh
development:
```
$ mkdir virtualenvs
$ virtualenv -p python3 virtualenvs/mesh
```
3. Activate the environment:
```
$ source virtualenvs/mesh/bin/activate
...
$ which python
/home/<user-name>/virtualenvs/mesh/bin/python
$ which pip
/home/<user-name>/virtualenvs/mesh/bin/pip
```

This will set the `python` and `pip` commands to point to
the version within the given environment. All packages installed through `pip` will be local
to the active environment.

You can deactivate the environment with the command `$ deactivate`.
The environment will only be set for the active shell session.

To make this virtual environment the default when starting a new shell, add the following to your `~/.bashrc` file:

```
source virtualenvs/mesh/bin/activate
```

---


#### Additional tools for building documentation @anchor toolchain_cmake_docs

The nRF5 SDK for Mesh documentation is written in Markdown and Doxygen.
If you want to build the documentation, make sure that the following tools are installed.
Add them to `PATH` after installation to make them available from the command line.

 | Download link    | Required version  | Notes                                                         |
 |------------------|-------------------|---------------------------------------------------------------|
 | @link_doxygen    | 1.8.13            | Required for rendering the conceptual and API documentation.  |
 | @link_graphviz   | 2.38.0            | Required for visualizing graphs.                              |
 | @link_mscgen     | 0.20              | Required for visualizing Message Sequence Charts.             |


---

#### Optional: Additional tools for building unit tests (host) @anchor toolchain_cmake_optional

The nRF5 SDK for Mesh contains a set of unit tests that verify module behavior. These unit tests run on the
host system (PC, not the nRF5 device), and are built with GCC.

The following tools are required for [building unit tests](@ref how_to_build_cmake_unit_tests).


 | Download link       													| Windows or Debian/Ubuntu				| Installation notes 	                            |
 |----------------------------------------------------------------------|---------------------------------------|---------------------------------------------------|
 | @link_git                                							| Both									| Required for the installation of CMock and Unity.<br>On Debian/Ubuntu, you can install it with: `$ sudo apt-get install git`	|
 | @link_cmock                                                      	| Both									| Used by the unit tests to generate mocks.<br>Make sure to clone the CMock repository using the commit hash `7cc41dd`.<br>Do it recursively in the same directory as the nRF5 SDK for Mesh:<br><br>`git clone https://github.com/ThrowTheSwitch/CMock.git --recursive`<br><br>The directory structure should look like this:<br><br>`.`<br>`+-- CMock/`<br>`+-- nrf5_sdk_for_mesh/`			|
 | @link_ruby                                       					| Both									| Required by CMock.<br>On Debian/Ubuntu, you can install it with the following command: `sudo apt-get install ruby`	|
 | @link_unity                                                          | Both									| Unit testing framework that is used for running the tests.<br>CMock bundles Unity as a submodule, but you can also use a different version.	|
 | GCC compiler															| Both									| **Windows**: Available through MinGW.<br> **Debian/Ubuntu**: Available in the distribution by default.	|
 | @link_mingw                                                          | Windows								| Required to use the standard GCC compiler on Windows.<br>Install the `mingw-base` and ensure that the 32-bit version is installed or that 32-bit libraries are available.	|
 | `libpthread`															| Windows								| Needed for the multithreaded test.<br>Install it using `mingw-get.exe`. From the command line, call the following command: `mingw-get install libpthread`	|
 | `gcc-multilib`														| Debian/Ubuntu							| **Optional.** Required to enable compilation for a 32-bit architecture on a 64-bit system (`-m32`).<br>Install it with the following command: `sudo apt-get install gcc-multilib`	|
 | `lcov`																| Debian/Ubuntu							| **Optional.** Required if you want to generate code coverage report.<br>Install it with the following command: `sudo apt-get install lcov`	|


---


## Downloading nRF5 SDK @anchor how_to_build_nrf_sdk
The nRF5 SDK for Mesh now *requires* the nRF5 SDK to compile. By default, the nRF5 SDK is expected
to be stored next to the nRF5 SDK for Mesh, in a directory structure that looks like this:

    .
    +-- nrf5_sdk_for_mesh/
    +-- nRF5_SDK_17.0.2_d674dde/


You can get the correct SDK either manually or using a custom CMake target.

### Downloading nRF5 SDK manually @anchor how_to_build_nrf_sdk_manual

Download the nRF5 SDK version 17.0.2 from the @link_nrf5SDK_download website.
Extract the package in the same folder as the nRF5 SDK for Mesh to match the folder structure above.


### Downloading nRF5 SDK using a custom CMake target @anchor how_to_build_nrf_sdk_custom

1. Generate CMake build files:

		nrf5_sdk_for_mesh $ mkdir build
		nrf5_sdk_for_mesh $ cd build
		build $ cmake -GNinja ..

  You will get a warning that the nRF5 SDK is not found.
2. Run the `nRF5_SDK` target:

		build $ ninja nRF5_SDK

  This command downloads and extracts the correct nRF5 SDK in the folder next to the nRF5 SDK for Mesh.
3. Re-run CMake and it will pick up the correct path:

		build $ cmake ..


