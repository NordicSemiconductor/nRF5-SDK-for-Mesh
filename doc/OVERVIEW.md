# Overview

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to use the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

The Bluetooth @link_MeshSpec is developed and published by the
@link_BluetoothSIG. It allows one-to-one, one-to-many, and many-to-many
communication. It uses BLE protocol to exchange messages between the nodes on the network.
The nodes can communicate with each other as long as they are in direct
radio range of each other or there are enough devices available that are
capable of listening and forwarding these messages.

The end-user applications (such as Luminaire control) are defined with the help
of client-server Mesh Models defined in the @link_ModelSpec.

@link_btsig_glossary.
You can also find detailed information about Bluetooth Mesh in @link_btsig_intro and the @link_btsig_spec.

## Release notes @anchor overview_release_notes
Check the @subpage md_RELEASE_NOTES page for the list of all the changes made in each release.

---

## Supported devices @anchor supported_devices

The nRF5 SDK for Mesh fully supports the following Nordic devices:
- nRF52840
- nRF52833
- nRF52832
- nRF52810 (limited support)

For details, see the @ref md_doc_user_guide_mesh_compatibility page.

---

## Supported features @anchor support_list

The nRF5 SDK for Mesh supports all the mandatory and most of the optional features of the
@tagMeshSp and the @tagMeshMdlSp.

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

The following optional mesh features are supported by the nRF5 SDK for Mesh, but not qualified:
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

For more information about the models in the nRF5 SDK for Mesh, see @ref
md_doc_user_guide_modules_models_main.

The nRF5 SDK for Mesh also includes the following features that are specific to Nordic:
- @ref md_doc_user_guide_modules_provisioning_pb_remote
- @ref md_doc_user_guide_modules_instaburst


---
## Where to get started @anchor getting_started

Regardless of your familiarity with Bluetooth Mesh, you can start working with the nRF5 SDK for Mesh
with the following pages:
- For a quick demonstration of some of the basic concepts of the Bluetooth Mesh
network using Nordic's nRF5 SDK for Mesh, see [Quick start guide: running a first example](@ref md_doc_getting_started_mesh_quick_start).
- To set up the SDK toolchain and get familiar with the building process, visit the @ref md_doc_getting_started_getting_started section.

If you are not familiar with Bluetooth Mesh, see @ref md_doc_user_guide_mesh_basic_concepts and @ref md_doc_user_guide_mesh_basic_architecture.

After setting up the toolchain, continue with the following pages:
- @ref md_doc_user_guide_user_guide_main contains detailed information about compatibility,
  interrupt priority levels, resource usage, and modules and components of this SDK.
  This section includes the following information:
    - @ref md_doc_user_guide_modules_modules_main, with conceptual documentation for several modules.
    - pages that describe how to perform configuration tasks, including pages within modules.
- @ref md_examples_README includes nRF5 SDK for Mesh example applications.
- @link_APIref contains the complete mesh stack API documentation.
- @link_Structref lists data structures and data fields used by the APIs.

@note
If you've been using one of the previous versions of the nRF SDK for Mesh,
check the @ref md_MIGRATION_GUIDE in the Getting Started section.


---
## Repository structure @anchor repo_structure

The nRF5 SDK for Mesh repository is organized as follows:
  - `bin` contains prebuilt binaries of all examples.
  - `doc` contains the main documentation and configuration files for Doxygen generation.
  - `examples` contains example applications using the mesh stack and supporting modules.
  - `external` contains external dependencies used by the mesh stack and examples (mainly uECC,
  Segger RTT and the SoftDevice).
  - `mesh` contains the source code and unit tests for the mesh stack.
  - `models` contains the source code for various models.
  - `scripts` contains useful scripts, such as a parser and communication script for the serial
  interface provided by the mesh stack.
  - `tools` contains tools useful for development.
  - `CMake` contains CMake setup files and utility functions.
  
### GitHub repository @anchor github_repo

The @link_meshsdk_github contains all the files from the official release zip package.
It is provided for reference only.
Currently, there is no system in place for integrating fixes through the public GitHub mirror.
For this reason, pull requests on GitHub are not accepted.

---
## Reporting issues @anchor reporting_issues

All bug reports and fix suggestions are appreciated. Please report all issues on 
@link_devzone and Nordic's technical support team will ensure they are tracked.