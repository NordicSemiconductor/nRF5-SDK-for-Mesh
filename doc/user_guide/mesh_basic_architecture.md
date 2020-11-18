# Bluetooth mesh stack architecture

The mesh stack consists of a number of subsystems that are interfaced through a set of API modules,
and are involved in the [Bluetooth mesh network data flow](@ref architecture_flow).
The API modules hide the complexity of their subsystems.
The functionality provided in the API is sufficient to make a functioning Bluetooth mesh device,
so that there is no need to bypass the API.

 ![Basic architecture of the Bluetooth mesh stack in the nRF5 SDK for Mesh](images/basic_architecture.svg)

The Bluetooth mesh stack's structure corresponds to the structure of the Bluetooth mesh specification
and follows the same naming conventions:
- [Models](@ref architecture_models): The Bluetooth mesh models present and implement device behavior.
- [Access](@ref architecture_access): The Bluetooth mesh access layer organizes models and communication.
- [DSM](@ref architecture_dsm): The Device State Manager stores addresses and encryption keys
for usage in the models.
- [Core](@ref architecture_mesh_core): The Core Bluetooth mesh layer takes care of encryption
and message relaying.
- [Provisioning](@ref architecture_provisioning): The Bluetooth mesh provisioning protocol is used
for adding devices to the network.
- [Bearer](@ref architecture_bearer): The Bearer layer takes care of low-level radio operation.
- [DFU](@ref architecture_dfu): The Device Firmware Upgrade module cooperates with a bootloader
to enable firmware upgrades through the Bluetooth mesh.
- [Bluetooth mesh Stack](@ref architecture_mesh_stack) (not pictured): Top level functionality for initializing
and starting the stack.
- [Serial](@ref architecture_serial) (not pictured): Application-level serialization of the Bluetooth mesh API
allows the mesh to be controlled by a separate host device.

@link_btsig_glossary. You can also read [Basic Bluetooth mesh concepts](@ref md_doc_user_guide_mesh_basic_concepts)
for a concise introduction to the Bluetooth mesh.

---

## Bluetooth mesh network data flow @anchor architecture_flow

The following figure demonstrates how the data packets flow between mesh network nodes and their layers within the Bluetooth mesh stack structure.

![Basic data flow within a mesh network in the nRF5 SDK for Mesh](images/mesh_data_packet_flow.svg)

For example, after a light switch is pressed on the source node, the following process takes place:
- Source
    -# The application calls the light switch model's publish function
    -# The model includes an on/off message in a publishing packet with an opcode and sends it to the access layer.
    -# The access layer fetches the necessary publish parameters, like destination address, encryption keys and time to live value (TTL value), and passes the packet to the transport layer, the highest of the core layers.
    -# The transport layer then encrypts the message with the selected application key, and splits the message into segments if necessary.
    Each segment is passed to the network layer, which attaches a network header with a sequence number and encrypts the packet with the network key before passing it to the bearer.
    -# The bearer includes the network message in an advertisement packet, and schedules a time slot for the packet to be broadcasted.
- Relays
    -# The broadcast is received by all mesh network nodes within range, and is passed from their bearer layers to their network layers.
    -# The network packet is decrypted, and if the receiving node is not its destination, the packet's TTL value is decreased by one, before being re-encrypted with the same network key and passed back to the bearer to be relayed.
- Destination
    -# Once the packet is relayed to the destination light bulb node, its network layer will decrypt the packet and pass it to the transport layer.
    -# Once all transport layer segments are received in this manner, the assembled message is decrypted with an application key, and passed on to the access layer.
    -# The access layer checks the opcode, application key and destination address, and passes the message to all eligible models.
    -# If one of these models is a light bulb model, the model parses the contents of the message, and notifies the application to turn on or off the light bulb.

The light bulb model may respond to acknowledge the transmission, following the same procedure back to the light switch node, which can notify the application that the on/off message was received.


---


## Models @anchor architecture_models

*API:* \ref MESH_API_GROUP_MODELS

The models define the behavior and communication formats of all data that is transmitted across the mesh network.
Equivalent to Bluetooth Low Energy's GATT services, the Bluetooth mesh models are independent,
immutable implementations of specific behaviors or services. All Bluetooth mesh communication happens through models,
and any application that exposes its behavior through the mesh must channel the communication through one or more models.

The Bluetooth mesh specification defines a set of immutable models for typical usage scenarios,
but vendors are also free to implement their own models.

You can read more about models, including how to implement your own models, in @ref md_doc_user_guide_modules_models_main.

---


## Access @anchor architecture_access

*API:* \ref MESH_API_GROUP_ACCESS

The access layer controls the device's model composition. It holds references to:
- models that are present on the device,
- messages these models accept,
- configuration of these models.

As the device receives Bluetooth mesh messages, the access layer finds which models the messages are for
and forwards them to the model implementations.

---


## Device State Manager @anchor architecture_dsm

*API:* \ref DEVICE_STATE_MANAGER

The Device State Manager stores the [encryption keys and addresses](@ref concepts_security_encryption)
used by the Bluetooth mesh stack. When models get assigned application keys and publish addresses
through configuration server, the Device State Manager stores the raw values and provides handles
to these values. The models can use the handles when referencing these values.

The Device State Manager stores its data in persistent storage, which it can recover on bootup.

---


## Mesh Core @anchor architecture_mesh_core

*API:* \ref MESH_API_GROUP_CORE

Consisting of a network and a transport layer, the Mesh Core module provides
the Bluetooth mesh-specific transport for the messages.

The transport layer provides in-network security by encrypting Bluetooth mesh packets with _application keys_
and splitting them into smaller segments that can go on air.
he transport layer re-assembles incoming packet segments and presents the full Bluetooth mesh message to the access layer.

The network layer encrypts each transport layer packet segment with a _network key_
and populates the source and destination address fields. When receiving a Bluetooth mesh packet,
the network layer decrypts the message, inspects the source and destination addresses,
and decides whether the packet is intended for this device and whether the network layer should relay it.

The Mesh Core provides protection against malicious behavior and attacks
against the Bluetooth mesh network through two-layer encryption, replay protection, and packet header obfuscation.

---


## Provisioning @anchor architecture_provisioning

*API:* \ref MESH_API_GROUP_PROV

Provisioning is the act of adding a device to a Bluetooth mesh network. The provisioning module takes care
of both sides of this process, by implementing a provisioner role (the network owner)
and a provisionee role (the device to add).

For detailed information about the provisioning process, see @ref md_doc_user_guide_modules_provisioning_implementing.

The mesh stack provides two ways to provision a device:
- Provisioning directly through the PB-ADV/PB-GATT provisioning bearer, which can only happen between
a provisioner and a provisionee that are within radio range of each other.
- Provisioning through remote provisioning, which implements two Bluetooth mesh models that together create
a tunnel through the mesh network, allowing the provisioner to add devices from a distance, with the help of a PB-ADV proxy device.

@note The remote provisioning is a Nordic proprietary feature that cannot be used with devices
from other vendors.

The [remote provisioning example](@ref md_examples_pb_remote_README) demonstrates remote provisioning.
The [light switch example](@ref md_examples_light_switch_README) shows the provisioner
and provisionee side of PB-ADV as a first step to establishing the network.

---


## Bearer @anchor architecture_bearer

*API:* \ref MESH_API_GROUP_BEARER

The Bearer is the low-level radio controller that provides an asynchronous interface
to the radio packet sending and receiving for the upper layers.
It enforces Bluetooth low energy compliance for packet formats and timing, and operates directly
on radio hardware through the @link_ic_SDtimeslotAPI.

The Bearer is an internal module that normally does not need to be accessed by the application.

---


## DFU @anchor architecture_dfu

*API:* \ref MESH_API_GROUP_DFU

The Device Firmware Upgrade module provides firmware update capabilities over the Bluetooth mesh by cooperating
with a bootloader.
It is capable of concurrent, authenticated firmware transfers to all devices in a network,
without halting the application.

@note
- The proprietary mesh DFU is a Nordic proprietary feature that cannot be used with devices from other vendors.
- DFU procedure is not compatible with the Bluetooth low energy secure DFU procedure used in the nRF5 SDK.

For more information about DFU, see the [DFU protocol](@ref md_doc_user_guide_modules_dfu_protocol)
section, including [information about how to run the proprietary mesh DFU](@ref md_doc_user_guide_modules_dfu_configuring_performing).

---


## Mesh Stack @anchor architecture_mesh_stack

*API:* \ref MESH_STACK

The Mesh Stack module is a thin wrapper around the top-level Bluetooth mesh modules that makes it easy
to get started using Bluetooth mesh.
It takes care of mesh initialization and enabling. It also contains functions for storing
and erasing provisioning and state related data.

---


## Serial @anchor architecture_serial

*API:* \ref MESH_API_GROUP_SERIAL

The serial module provides full serialization of the Bluetooth mesh API, allowing other devices to control
the nRF5 mesh device through a UART interface.
Intended for network gateways and similar complex applications, the serial interface provides
a way to access the mesh network through a Nordic device, without making it the unit's main controller.

The serial interface is based on the @link_nRF8001 ACI serial interface and optionally supports @link_SLIP-encoded operation.
The serial protocol can be run as a stand-alone application (see [serial example](@ref md_examples_serial_README))
or alongside a normal Bluetooth mesh application.

For an overview of the serial packet format, commands, and events, as well as the related Interactive PyACI script,
the @ref md_scripts_README section.