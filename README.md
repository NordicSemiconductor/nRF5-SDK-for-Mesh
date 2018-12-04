# Welcome to nRF5 SDK for Mesh

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to use the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

This GitHub repository contains all the files from the official release zip package. It is provided for reference only, so that you can familiarize yourself with the repository structure and contents.

For detailed and structured documentation, see the <a href="https://www.nordicsemi.com/DocLib/Content/SDK_Doc/Mesh_SDK/v3-0-0/index" target="_blank">Mesh documentation website</a>.

## Repository structure

The nRF5 SDK for Mesh repository is organized as follows:
  - `bin` contains prebuilt binaries of all examples.
  - `CMake` contains CMake setup files and utility functions.
  - `doc` contains the main documentation and configuration files for Doxygen generation.
	- Main documentation refers to Getting started, Introduction, Libraries, and Migration guide sections available on the <a href="https://www.nordicsemi.com/DocLib/Content/SDK_Doc/Mesh_SDK/v3-0-0/index" target="_blank">Mesh documentation website</a>.
  - `examples` contains example applications using the mesh stack and supporting modules.
	- Each example contains its own readme file. This documentation is grouped in the **Examples** section on the Mesh documentation website.
  - `external` contains external dependencies used by the mesh stack and examples (App timer, uECC, Segger RTT, SDK fix, and the SoftDevice).
  - `mesh` contains the source code and unit tests for the mesh stack.
  - `models` contains the source code for various models.
	- You can find more information about the Simple OnOff model in `models/vendor/` and in **Getting started > Creating new models** page on the Mesh documentation website.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial interface provided by the mesh stack.
	- You can fing more information about the Interactive PyACI script in the **Libraries > Serial interface library** section on the Mesh documentation website.
  - `tools` contains tools useful for development.
  
  
## Reporting issues

We appreciate all bug reports and fixes. Please report all issues on
<a href="https://devzone.nordicsemi.com" target="_blank">DevZone</a> and someone from
technical support will ensure they are tracked internally. Currently, we do not have a system in place
for integrating fixes through the public GitHub mirror. For this reason, we cannot accept any pull requests.
