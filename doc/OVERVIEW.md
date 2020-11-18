# Overview

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth mesh. It allows
applications to use the features provided by the Bluetooth mesh when running on Nordic's
nRF5 Series chips.

The @link_MeshSpec is developed and published by the
@link_BluetoothSIG. It allows one-to-one, one-to-many, and many-to-many
communication. It uses BLE protocol to exchange messages between the nodes on the network.
The nodes can communicate with each other as long as they are in direct
radio range of each other or there are enough devices available that are
capable of listening and forwarding these messages.

The end-user applications (such as Luminaire control) are defined with the help
of client-server Bluetooth mesh models defined in the @link_ModelSpec.

@link_btsig_glossary.
You can also find detailed information about Bluetooth mesh in @link_btsig_intro and the @link_btsig_spec.

**Table of contents**
- [Release notes](@ref overview_release_notes)
- [Supported devices](@ref supported_devices)
- [Supported features](@ref support_list)
- [Where to get started](@ref getting_started)
- [Repository structure](@ref repo_structure)
    - [GitHub repository](@ref github_repo)
- [Off-line documentation](@ref offline_docs)
- [Reporting issues](@ref reporting_issues)


---
## Release notes @anchor overview_release_notes
Check the @subpage md_RELEASE_NOTES page for the list of all the changes made in each release.

---

## Supported devices @anchor supported_devices

The nRF5 SDK for Mesh fully supports the following Nordic devices:
- nRF52840 (limited support for dongle)
- nRF52833
- nRF52832
- nRF52820 (limited support)
- nRF52810 (limited support)

For details, see the @ref md_doc_user_guide_mesh_compatibility page.

---

## Supported features @anchor support_list

The nRF5 SDK for Mesh supports all the mandatory and most of the optional features of the
@tagMeshSp and the @tagMeshMdlSp.

Check the [online documentation](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.meshsdk.v5.0.0/index.html)
for updated information on qualification status after the official release.
The following information is valid as of week 46, 2020.

The following mandatory features are qualified:
- Provisioner role (only Advertising bearer)
- Node role (Advertising and GATT bearer)
- Relay feature
- Proxy feature
    - Proxy Server
- Low power feature
- Friend feature
- Foundation models
    - Config Server
    - Health Server and Client

The following optional Bluetooth mesh features are supported by the nRF5 SDK for Mesh, but not qualified:
- Foundation models
    - Config Client

Additionally, the nRF5 SDK for Mesh supports the following models from the @tagMeshMdlSp,
yet to be qualified:
- Generic Default Transition Time Server and Client
- Generic Level Server and Client
- Generic OnOff Server and Client
- Generic PowerOnOff Server and Client
- Light Lightness Setup Server (with associated models) and Light Lightness Client
- Light CTL Setup Server (with associated models) and Light CTL Client
- Light LC Setup Server (with associated models) and Light LC Client
- Sensor Setup Server (with associated models) and Sensor Client
- Scene Setup Server (with associated models) and Scene Client

For more information about the models in the nRF5 SDK for Mesh, see @ref
md_doc_user_guide_modules_models_main.

The nRF5 SDK for Mesh also includes the following features that are specific to Nordic:
- @ref md_doc_user_guide_modules_provisioning_pb_remote
- @ref md_doc_user_guide_modules_instaburst


---
## Where to get started @anchor getting_started

Regardless of your familiarity with Bluetooth mesh, you can start working with the nRF5 SDK for Mesh
with the following pages:
- For a quick demonstration of some of the basic concepts of the Bluetooth mesh
network using Nordic's nRF5 SDK for Mesh, see the [quick demonstration guide](@ref md_doc_getting_started_mesh_quick_start).
- To set up the SDK toolchain and get familiar with the building process, visit the @ref md_doc_getting_started_getting_started section.

If you are not familiar with Bluetooth mesh, see @ref md_doc_user_guide_mesh_basic_concepts and @ref md_doc_user_guide_mesh_basic_architecture.

After setting up the toolchain, continue with the following pages:
- @ref md_doc_user_guide_user_guide_main contains detailed information about compatibility,
  interrupt priority levels, resource usage, and modules and components of this SDK.
  This section includes the following information:
    - @ref md_doc_user_guide_modules_modules_main, with conceptual documentation for several modules.
    - Pages that describe how to perform configuration tasks, including pages within modules.
- @ref md_examples_README includes nRF5 SDK for Mesh example applications.
- @link_APIref contains the complete Bluetooth mesh stack API documentation.
- @link_Structref lists data structures and data fields used by the APIs.

@note
If you've been using one of the previous versions of the nRF SDK for Mesh,
check the @ref md_MIGRATION_GUIDE in the Getting Started section.


---
## Repository structure @anchor repo_structure

The nRF5 SDK for Mesh repository directories are organized as follows:
  - `bin` contains prebuilt binaries of all examples.
  - `doc` contains the main documentation and configuration files for Doxygen generation.
  - `examples` contains example applications using the Bluetooth mesh stack and supporting modules.
  - `external` contains external dependencies used by the Bluetooth mesh stack and examples (mainly uECC,
  Segger RTT and the SoftDevice).
  - `mesh` contains the source code and unit tests for the Bluetooth mesh stack.
  - `models` contains the source code for various models.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the Bluetooth mesh stack.
  - `tools` contains tools useful for development.
  - `CMake` contains CMake setup files and utility functions.

### GitHub repository @anchor github_repo

The @link_meshsdk_github contains all the files from the official release zip package.
It is provided for reference only.
Currently, there is no system in place for integrating fixes through the public GitHub mirror.
For this reason, pull requests on GitHub are not accepted.

---
## Off-line documentation @anchor offline_docs

The release zip package for the nRF5 SDK for Mesh does not include off-line documentation.

However, you can build the documentation locally for off-line usage after downloading the package.
For more details, see [Building documentation with Ninja](@ref how_to_build_building_docs).

---
## Reporting issues @anchor reporting_issues

All bug reports and fix suggestions are appreciated. Please report all issues on
@link_devzone and Nordic's technical support team will ensure they are tracked.