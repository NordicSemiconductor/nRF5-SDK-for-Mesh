# Installing the mesh toolchain

To build the example applications, a toolchain based on either CMake or SEGGER Embedded Studio is required.
Install instructions are provided for Windows and Debian/Ubuntu. The steps should be similar for
other platforms.

## SEGGER Embedded Studio

Please see the @link_seggerstudio
<!--https://www.segger.com/products/development-tools/embedded-studio/?L=0--> website for download and
installation instructions.

## Python

Python is _not_ required to build the mesh stack and examples, but it is required when working with
DFU, the @ref md_scripts_interactive_pyaci_README, generating SEGGER Embedded Studio projects and when
building documentation. The nRF5 SDK for Mesh uses @link_python3<!--Python 3: https://www.python.org/downloads-->,
_however_, the @link_nrfutil_github_name<!--https://github.com/NordicSemiconductor/pc-nrfutil/tree/mesh_dfu-->
tool used for transferring firmware images over serial requires @link_python27<!-- Python 2.7: https://www.python.org/downloads/ -->.

## Development tools

For programming nRF5x devices, using `nrfjprog` is recommended. It is available in the
@link_nrf5x_cmd_line_tools_w32 <!-- nRF5x command line tools: http://www.nordicsemi.com/eng/nordic/Products/nRF51822/nRF5x-Command-Line-Tools-Win32/33444 -->
and @link_nrf5x_cmd_line_tools_linux<!--nRF5x command line tools: https://www.nordicsemi.com/eng/nordic/Products/nRF51822/nRF5x-Command-Line-Tools-Linux64/51386 -->.

## CMake based setup

@link_cmake <!--CMake: https://cmake.org/--> is a build management system used for managing an
environment that is independent of the compiler and build system used. Version 3.6 or above is
required by the mesh stack.

The following tools must be installed:
* @link_cmake <!--CMake: https://cmake.org/-->
* A _toolchain_. Supported toolchaings are the @link_armnone <!--arm-none-eabi-gcc: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm-->
  toolchain and the @link_armcc <!--armcc v5: https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5-->
  toolchain. The armcc v5 toolchain is also provided by @link_keil <!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--Keil uVision IDE: http://www2.keil.com/mdk5/uvision/-->.
* A _build system_. CMake supports a range of build systems, e.g., @link_ninja and @link_make.

Instructions for how to install these tools are provided for the following platforms:
* @subpage md_doc_getting_started_how_to_windows_tools
* @subpage md_doc_getting_started_how_to_debian_tools

If you want to build the documentation, make sure that the following tools are installed and available from the command line:
* @link_doxygen <!--Doxygen: https://doxygen.org-->
* @link_graphviz <!--Graphviz: http://graphviz.org-->
* @link_mscgen <!--Mscgen: http://www.mcternan.me.uk/mscgen-->
* @link_python3 <!--Python 3: https://www.python.org/downloads-->

See the tools' websites for installation instructions.
