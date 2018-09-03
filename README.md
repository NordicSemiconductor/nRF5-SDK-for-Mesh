# Introduction to nRF5 SDK for Mesh

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to utilize the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

Make sure to check the @subpage md_RELEASE_NOTES for the current release, and the
@subpage md_MIGRATION_GUIDE for migration between releases.

Refer to @subpage md_COPYING for the Copyright Notice.

> **Important:**
>
> The nRF5 SDK for Mesh now **requires** the
> <a href="http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/" target="_blank">nRF5 SDK 15.0.0</a>
> to compile. Please refer to @ref md_doc_getting_started_how_to_build for more information.
>
> In an application using nRF5 SDK for Mesh, interrupt priorities need to be considered carefully,
> see @ref md_doc_getting_started_mesh_interrupt_priorities.

## Compatibility @anchor readme-compatibility

The mesh stack is compatible with the following configurations:

| SoC                                            | Board(s)  | SoftDevice(s)       |
| ---------------------------------------------- | --------- | ------------------- |
| nRF51422_xxAC (deprecated)                     | PCA10028  | S130 v2.0.1         |
| nRF52810_xxAA<sup><a href="#fn:1">1</a></sup>  | PCA10040e | S112 v6.0.0         |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0  |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0         |

The mesh stack is also compatible with the nRF52810 platform, but not supported by this version of the nRF5 SDK for Mesh.
Support for the nRF51 Series platform is being dropped, but building the examples<sup><a href="#fn:2">2</a></sup>
is still supported using
the CMake build system.

## Resource usage
For information about resource usage, see the @subpage md_doc_introduction_mesh_hw_resources document.

## Documentation

Refer to the following guides for understanding basic concepts of Bluetooth mesh and architecture of
the Nordic nRF5 SDK for Mesh:
  - @subpage md_doc_getting_started_mesh_quick_start
  - @subpage md_doc_introduction_basic_concepts
  - @subpage md_doc_introduction_basic_architecture

### SDK
The nRF5 SDK for Mesh documentation is organized as follows:
  - @ref index : Information providing an overview of nRF5 SDK for Mesh
  - @ref md_doc_getting_started_getting_started : Tutorials describing how to perform common tasks
  such as installing toolchain, building the stack and examples, how to perform DFU, etc.
  - @ref md_doc_libraries_libraries : Libraries implementing common tasks in an application using nRF5 SDK for Mesh
  - @ref md_scripts_README : The Interactive Python Application Controller Interface documentation
  - @ref md_examples_README : nRF5 SDK for Mesh example applications
  - @link_APIref : The API documentation
  - @link_Structref : Documentation for the data structures and data fields used by the APIs

## Repository Structure
The nRF5 SDK for Mesh repository is organized as follows:
  - `bin` contains prebuilt binaries of all examples optimized for space (directory: `\ospace`) and for time (directory: `\otime`).
  - `CMake` contains CMake setup files and utility functions.
  - `doc` contains the main documentation and configuration files for Doxygen generation.
  - `examples` contains example applications using the mesh stack and supporting modules.
  - `external` contains external dependencies used by the mesh stack and examples (mainly uECC, Segger RTT and the SoftDevice).
  - `mesh` contains the source code and unit tests for the mesh stack.
  - `models` contains the source code for various models.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the mesh stack.
  - `tools` contains tools useful for development.


## Reporting issues

We appreciate all bug reports and fixes. We kindly ask you to report all issues through
<a href="https://devzone.nordicsemi.com" target="_blank">DevZone</a> and someone from
technical support will ensure it is tracked internally. Unfortunately we currently do
not have a good system in place for integrating fixes through the public GitHub mirror.
That is why we cannot accept any pull request at the moment.

---

<sup id="fn:1">1</sup> nRF52810 support is limited. It is very limited in both Flash and RAM, therefore it does
not currently support DFU and logging is disabled to save flash. Furthermore, it should be compiled with
optimization for size, i.e., `-DCMAKE_BUILD_TYPE=MinSizeRel` for CMake or the 'ReleaseWithDebugInformation'
setting in SEGGER Embededd Studio.

<sup id="fn:2">2</sup> The Light switch proxy example is for the nRF52 Series only.
