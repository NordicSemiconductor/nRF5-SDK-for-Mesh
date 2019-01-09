# Overview

The nRF5 SDK for Mesh is Nordic Semiconductor's implementation of the Bluetooth Mesh. It allows
applications to use the features provided by the Bluetooth Mesh when running on Nordic's
nRF5 Series chips.

The Bluetooth @link_MeshSpec is developed and published by the
@link_BluetoothSIG<!--http://www.bluetooth.org/-->. It allows one-to-one, one-to-many, and many-to-many
communication. It uses BLE protocol to exchange messages between the nodes on the network.
The nodes can communicate with each other as long as they are in direct
radio range of each other or there are enough devices available that are
capable of listening and forwarding these messages.

The end-user applications (such as Luminaire control) are defined with the help
of client-server Mesh Models defined in the @link_ModelSpec.

You can find detailed information about Bluetooth Mesh in @link_btsig_intro and the @link_btsig_spec.

## Supported features @anchor support_list

The nRF5 SDK for Mesh supports all the mandatory features of the Mesh Profile Specification.
These mandatory features are qualified.

The following optional mesh features are supported by the nRF5 SDK for Mesh,
but not qualified yet:
- Provisioning over GATT bearer
- GATT bearer
- Config client
- Mesh Proxy Service with Proxy Server
- Low Power feature
- Generic server and client models

For a quick demonstration of some of the basic concepts of the Bluetooth Mesh
network using Nordic's nRF5 SDK for Mesh, see [Quick start guide: running a first example](@ref md_doc_getting_started_mesh_quick_start).

@note
The nRF5 SDK for Mesh requires the
<a href="http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/" target="_blank">nRF5 SDK 15.2.0</a>
to compile. See @ref md_doc_getting_started_how_to_toolchain for more information.

## Introduction to nRF5 SDK for Mesh @anchor this_section

The overview section contains the following conceptual documentation:
  - @subpage md_RELEASE_NOTES list all the changes made in the latest release.
  - @subpage md_doc_introduction_basic_concepts describes the profile's relation to Bluetooth Low Energy, its network topology and relaying, as well as other key Bluetooth Mesh notions.
  - @subpage md_doc_introduction_basic_architecture offers specific information about Nordic's implementation of the Bluetooth Mesh.
  - @subpage md_doc_introduction_mesh_compatibility lists SoCs, boards, and Soft Devices that can work with the mesh stack.
  - @subpage md_doc_introduction_mesh_interrupt_priorities describes the two interrupt priorities available for the mesh stack. Consider these interrupt priorities carefully in an application that uses nRF SDK for Mesh.
  - @subpage md_doc_introduction_lpn_concept describes concepts related to the Friendship protocol, which enables power-constrained devices to be part of a Mesh network.
  - @subpage md_doc_introduction_mesh_hw_resources provides information about hardware resources required by the stack, including hardware peripherals, and RAM and flash usage.
  - @subpage md_doc_introduction_mesh_repository_structure lists the folder structure of the mesh stack repository.

## Working with nRF5 SDK for Mesh @anchor other_sections

Once familiar with the Bluetooth Mesh concepts, see the following sections of the nRF5 SDK for Mesh documentation:
  - @ref md_doc_getting_started_getting_started contains tutorials about how to perform common tasks
  such as installing toolchain, building the stack and examples, implementing DFU, and other.
  - @ref md_doc_libraries_libraries describes libraries that implement common tasks in an application that uses nRF5 SDK for Mesh.
  - @ref md_examples_README includes nRF5 SDK for Mesh example applications.
  - @link_APIref contains the complete mesh stack API documentation.
  - @link_Structref lists data structures and data fields used by the APIs.

@note
If you've been using one of the previous versions of the nRF SDK for Mesh, check the @ref md_MIGRATION_GUIDE in the Getting Started section.

## Reporting issues @anchor reporting_issues

We appreciate all bug reports and fixes. Please report all issues on 
<a href="https://devzone.nordicsemi.com" target="_blank">DevZone</a> and our
technical support team will ensure they are tracked.
