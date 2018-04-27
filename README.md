# Introduction to nRF5 SDK for Mesh

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to utilize the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

See the [Getting started guides](@ref md_doc_getting_started_getting_started) for more information
on
@subpage md_doc_introduction_basic_concepts,
@subpage md_doc_introduction_basic_architecture,
[installing the mesh toolchain](@ref md_doc_getting_started_how_to_toolchain),
[building the mesh stack](@ref md_doc_getting_started_how_to_build), and more.

Make sure to check the [release notes](@ref md_RELEASE_NOTES) for the current release.

**Important:**
In an application using nRF5 SDK for Mesh, interrupt priorities need to be considered carefully,
see @ref md_doc_getting_started_mesh_interrupt_priorities.

## Compatibility
The nRF5 SDK for Mesh has no direct dependencies on
the @link_nRF5SDK<!--nRF5 SDK: https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK-->,
but see @ref md_doc_getting_started_how_to_nordicSDK for instructions on how to integrate the two SDKs.

The mesh stack is compatible with the following configurations:

| SoC       | Board(s)            | SoftDevice(s)               |
| --------- | ------------------- | --------------------------- |
| nRF52840  | PCA10056            | S140 v5.0.1-Alpha           |
| nRF52832  | PCA10040            | S132 v3.\* / v4.\* / v5.\*  |
| nRF51xxx  | PCA10028 / PCA10031 | S130 v2.0.1 / S110 v8.0.0   |

The mesh stack is also compatible with the nRF52810 platform, but not supported by the current build system.

## Resource usage
For information about resource usage, see the @subpage md_doc_introduction_mesh_hw_resources document.

## Documentation Structure
The nRF5 SDK for Mesh documentation is organized as follows:
  - @ref index : Information providing an overview of nRF5 SDK for Mesh
  - @ref md_RELEASE_NOTES
  - @ref md_COPYING : nRF5 SDK for Mesh Copyright Notice
  - @ref md_doc_getting_started_getting_started : Tutorials describing how to perform common tasks
  - @ref LIBRARIES : Libraries implementing common tasks in an application using nRF5 SDK for Mesh
  - @ref example-projects : Examples demonstrating various features

## Repository Structure
The nRF5 SDK for Mesh repository is organized as follows:
  - `bin` contains prebuilt binaries such as the bootloader (required for DFU).
  - `CMake` contains CMake setup files and utility functions
  - `doc` contains the main documentation and configuration files for Doxygen generation
  - `examples` contains example applications using the mesh stack and supporting modules such as `nrf_mesh_sdk.c`.
  - `external` contains external dependencies used by the mesh stack and examples (mainly uECC, Segger RTT and the SoftDevice).
  - `mesh` contains the source code and unit tests for the mesh stack.
  - `models` contains the source code for various models
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the mesh stack.
  - `tools` contains tools useful for development.

