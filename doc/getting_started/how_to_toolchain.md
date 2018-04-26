# Installing the toolchain

To build the example applications, a toolchain based on either CMake or SEGGER Embedded Studio is required.
Install instructions are provided for Windows and Debian/Ubuntu. The steps should be similar for
other platforms.

## List of tools

> **Important:** Ensure that all command line tools are available in a folder referenced by
> the system path (e.g. the `PATH` environment variable).

The following table lists the required tools to work with the nRF5 SDK for Mesh:

 | Download link                                                                                                                                                                                              | Recommended *minimum* version | Installation notes                               |
 |------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------|--------------------------------------------------|
 | <a href="http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html" target="_blank">nRF5x Command Line Tools</a> | 9.5.0                         |                                                  |
 | <a href="https://www.segger.com/downloads/jlink/" target="_blank">SEGGER J-Link Software Pack</a>                                                                                                          | 6.16a                         |                                                  |
 | <a href="https://www.python.org/downloads/" target="_blank">Python 2.7</a>                                                                                                                                 | 2.7                           | Only needed for DFU. See note about Python below |
 | <a href="https://www.python.org/downloads/" target="_blank">Python 3</a>                                                                                                                                   | 3.5.1.32                      | Must be 32-bit for `nrfjprog` DLL to work        |

In addition to these tools, a build environment is needed.
The nRF5 SDK for Mesh supports both CMake and SEGGER Embedded Studio.
You only need to install one of them.

### SEGGER Embedded Studio

To use SEGGER Embedded Studio, download the installer from the
<a href="https://www.segger.com/downloads/embedded-studio/" target="_blank">SEGGER website</a>
and follow the installation instructions. You will find project files for each of the examples in
their respective folders.

### CMake
As an alternative to SEGGER Embedded Studio, @link_cmake <!--CMake: https://cmake.org/-->
is a build management system used for managing an environment that is independent of the
compiler and build system used. Version 3.6 or above is required by the mesh stack.

The following tools must be installed:
* @link_cmake <!--CMake: https://cmake.org/-->
* A _toolchain_. Supported toolchains are the @link_armnone <!--arm-none-eabi-gcc: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm-->
  toolchain and the @link_armcc <!--armcc v5: https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5-->
  toolchain. The armcc v5 toolchain is also provided by @link_keil <!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--Keil uVision IDE: http://www2.keil.com/mdk5/uvision/-->.
* A _build system_. CMake supports a range of build systems, e.g., @link_ninja and @link_make.

Additionally, if you want to build the documentation, make sure that the following tools are installed and available from the command line:
* @link_doxygen <!--Doxygen: https://doxygen.org-->
* @link_graphviz <!--Graphviz: http://graphviz.org-->
* @link_mscgen <!--Mscgen: http://www.mcternan.me.uk/mscgen-->

Instructions for how to install these tools are provided for the following platforms:

* @subpage md_doc_getting_started_how_to_windows_tools
* @subpage md_doc_getting_started_how_to_debian_tools

## Python

Python is _not_ required to build the mesh stack and examples, but it is required when working with
DFU, the @ref md_scripts_interactive_pyaci_README, generating SEGGER Embedded Studio projects and when
building documentation. The nRF5 SDK for Mesh uses
<a href="https://www.python.org/downloads/" target="_blank">Python 3</a>,
_however_, the @link_nrfutil_github_name<!--https://github.com/NordicSemiconductor/pc-nrfutil/tree/mesh_dfu-->
tool used for DFU transfers over serial requires
<a href="https://www.python.org/downloads/" target="_blank">Python 2.7</a>.
