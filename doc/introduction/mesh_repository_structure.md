# Repository structure

The nRF5 SDK for Mesh repository is organized as follows:
  - `bin` contains prebuilt binaries of all examples.
  - `doc` contains the main documentation and configuration files for Doxygen generation.
  - `examples` contains example applications using the mesh stack and supporting modules.
  - `external` contains external dependencies used by the mesh stack and examples (mainly uECC, Segger RTT and the SoftDevice).
  - `mesh` contains the source code and unit tests for the mesh stack.
  - `models` contains the source code for various models.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the mesh stack.
  - `tools` contains tools useful for development.
  - `CMake` contains CMake setup files and utility functions.
  
## GitHub repository @anchor github-repo

The repository is also <a href="https://github.com/NordicSemiconductor/nRF5-SDK-for-Mesh/" target="_blank">available on GitHub</a>.

The GitHub repository contains all the files from the official release zip package. It is provided for reference only. Currently, we do not have a system in place for integrating fixes through the public GitHub mirror.
For this reason, we cannot accept any pull requests.

@note Markdown documentation files in GitHub are placed as close as possible to the code. This placement does not correspond to the structure of the @link_mesh_doclib. However, the contents of the files and the documentation website are the same.