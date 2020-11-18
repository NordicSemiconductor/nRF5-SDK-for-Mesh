# Bluetooth mesh concepts

Bluetooth mesh is a profile specification developed and published by the @link_BluetoothSIG.
This document explains the basic concepts of the Bluetooth mesh and gives an overview of the operation
and capabilities of the profile, as well as explaining the life cycle of a mesh network device.
For more specific information about Nordic Semiconductor's implementation of the Bluetooth mesh,
see [Bluetooth mesh architecture documentation](@ref md_doc_user_guide_mesh_basic_architecture).

The Bluetooth mesh is based on the Bluetooth low energy part of the Bluetooth 4.0 specification
and shares the lowest layers with this protocol. On-air, the Bluetooth mesh physical representation
is compatible with existing Bluetooth low energy devices, as mesh network messages are contained
inside the payload of Bluetooth low energy _advertisement_ packets. However, Bluetooth mesh specifies
a completely new host layer, and although some concepts are shared, Bluetooth mesh
is incompatible with the Bluetooth low energy host layer.

![Relationship between Bluetooth mesh and Bluetooth low energy specifications](images/mesh_and_ble.svg)

Read more about basic Bluetooth mesh concepts in the following sections:
- [Application areas](@ref concepts_application_areas)
- [Network topology and relaying](@ref concepts_network_topo), including information about
[transport](@ref concepts_network_topo_transport), [relays](@ref concepts_network_topo_relays),
[power consumption](@ref concepts_network_topo_power), and [GATT proxy](@ref concepts_network_topo_gatt).
- [Addressing](@ref concepts_addressing)
- [Models and elements](@ref concepts_models_and_elements)
- [Device life cycle](@ref concepts_lifecycle), including information about
[provisioning](@ref concepts_lifecycle_provisioning) and [network configuration](@ref concepts_lifecycle_network_config).
- [Security](@ref concepts_security), including information about [authentication](@ref concepts_security_authentication),
[message encryption](@ref concepts_security_encryption), [privacy key](@ref concepts_security_privacy),
and [replay protection](@ref concepts_security_replay_protection).

@link_btsig_glossary.

---


## Application areas @anchor concepts_application_areas

Bluetooth mesh primarily targets simple control and monitoring applications, like light control
or sensor data gathering. The packet format is optimized for small control packets,
issuing single commands or reports, and is not intended for data streaming or other high-bandwidth
applications.

Using Bluetooth mesh causes higher power consumption than traditional Bluetooth low energy applications.
This is mainly due to the need for keeping the radio running constantly. Therefore, unlike Bluetooth
low energy advertisers, active mesh network devices cannot be run off coin-cell batteries
for extended periods of time.

Bluetooth mesh supports up to 32767 devices in a network, with a maximum network diameter of 126 hops.

---


## Network topology and relaying @anchor concepts_network_topo

Bluetooth mesh is a broadcast-based network protocol, where every device in the network sends
and receives all messages to and from all devices within radio range.

There is no concept of connections in a mesh network. Any device in the network may relay
messages from any other device, which makes it possible for a Bluetooth mesh device to send a message
to a device outside of radio range by having one or more other devices relay the message
towards the destination. This property also allows devices to move around and drop in
and out of the network at any time.

### Mesh transport @anchor concepts_network_topo_transport

Bluetooth mesh utilizes the Bluetooth low energy advertiser and scanner roles,
communicating through Bluetooth low energy advertisement packets. The advertisement packets
are picked up by nearby Bluetooth mesh devices and handled like other Bluetooth low energy
advertisement packets. The Bluetooth mesh packets are represented with a unique AD type
and added to the advertisement packet payload.

Bluetooth low energy devices send advertisement packets at regular _advertisement intervals_,
and mesh packets are no exception. However, unlike traditional advertisers, Bluetooth mesh devices
will change their advertisement payload on every transmission, broadcasting new Bluetooth mesh packets
as they are queued up in the stack.

If there is no traffic in the Bluetooth mesh network or the Bluetooth mesh stack, or if the application
does not generate any messages, the devices stay silent until there is something to transmit.

### Relays @anchor concepts_network_topo_relays

Bluetooth mesh expands the range of the network by relaying messages. Any Bluetooth mesh device
may be configured to act as a relay, and no dedicated relay devices are needed to build a network.

Every device acting as a relay will decrement the Time To Live (TTL)
value in received messages and forward them if the TTL equals two or higher. This undirected relaying
is referred to as _message flooding_ and ensures a high probability of message delivery,
without requiring any information on the network topology.

The Bluetooth mesh profile specification does not provide any routing mechanisms, and all messages are forwarded
by all relays until the TTL value reaches zero. To avoid messages being forwarded by the same relays
over and over, all Bluetooth mesh devices maintain a _message cache_. This cache is used for filtering out
packets that the device has already handled.

The flooding-based approach to message relaying can cause a lot of redundant traffic on air, which may impact the throughput and
reliability of the network. Therefore, it is highly recommended to limit the number of relays in a network to restrict this effect.

The number of relay-enabled devices in the network is a trade-off between message route-redundancy and reliability.
It should be tuned according to:
- network density,
- traffic volumes,
- network layout,
- requirements for reliability and responsiveness.

### Power consumption @anchor concepts_network_topo_power

To enable broadcast-based communication, the devices must continuously keep their radio in listening mode.
This causes significantly higher power consumption than in a typical Bluetooth low energy device.

To enable low power devices to take part in the mesh
network, Bluetooth mesh contains a _friendship_ feature.
This protocol lets low power devices establish a relationship with a regular Bluetooth mesh device,
which will then cache and forward messages to the low power device at regular intervals.
This saves the low power device from having to stay on to listen for incoming messages.

### GATT proxy @anchor concepts_network_topo_gatt

To enable support for legacy Bluetooth low energy devices that do not support receiving mesh network packets,
Bluetooth mesh defines a separate protocol for tunneling mesh messages over the Bluetooth low energy
GATT protocol. For this purpose, the Bluetooth mesh profile specification defines a GATT bearer
and the corresponding GATT Proxy Protocol. This protocol allows legacy Bluetooth low energy devices
to participate in the Bluetooth mesh network by establishing a GATT connection to a Bluetooth mesh device
that has the proxy feature enabled.

The legacy device gets assigned an address and the necessary keys to become a full-fledged member
of the network. The device receives the security credentials through the regular provisioning procedure
or through some out-of-band mechanism.

---


## Addressing @anchor concepts_addressing

The Bluetooth mesh addressing scheme is different from the Bluetooth low energy addressing scheme.
It features three types of addresses:
- *Unicast addresses*: unique for every device
- *Group addresses*: for forming a group of devices and addressing them all at once
- *Virtual addresses*: untracked UUID-based addresses with a large address space

When a device is added to a network, it is assigned a range of unicast addresses that represents it.
A device's unicast addresses cannot be changed and are always sequential. The unicast address space
supports having 32767 unicast addresses in a single Bluetooth mesh
network. Unicast addresses can be used by any application to directly send a message to a device.

Group addresses are allocated and assigned as part of the network configuration procedure.
A group address may represent any number of devices, and a device may be part of any number of groups.
There can at most be 16127 general purpose group addresses in a mesh network.

Virtual addresses can be considered a special form of group addresses, and can be used to represent
any number of devices. Each virtual address is a 128-bit UUID generated from a text label.
The virtual addresses do not have to be tracked by a network configuration device,
and in this way, users can generate virtual addresses before deployment or addresses can be generated
ad-hoc between devices in the network.

---


## Models and Elements @anchor concepts_models_and_elements

To standardize communication between devices from different vendors, the Bluetooth mesh profile specification
defines an access layer, which routes Bluetooth mesh messages between the various _models_ in a device.
A model represents a specific behavior or service and defines a set of states and messages that act
on these states. The Bluetooth mesh profile specification and the Bluetooth mesh model specification each define a set
of models to cover typical usage scenarios like device configuration, sensor readings,
and light control. In addition to these, vendors are free to define their own models
with accompanying messages and states.

The models in a device belong in _elements_. Every device has one or more elements,
 each acting as a virtual entity in the Bluetooth mesh with its own unique unicast address.
 Each incoming message is handled by a model instance in an element.
 To make it possible to uniquely resolve how messages are handled, only one model instance per element
 can implement a handler for a specific message opcode. If a device has multiple instances
 of the same model, each instance must be assigned to a separate element.
 Similarly, if two models implement handlers for the same message, these models must be in separate elements.

To represent complex behavior with minimal message and state duplication, models can be made up
of other models, potentially spanning multiple elements. These models are referred to as _extended models_.
Models that are purely self-contained are referred to as root models.

Models talk to each other through a publish-and-subscribe system. Every model may subscribe
to a set of group and virtual addresses, and the model will only handle messages that are published
to one of its subscription addresses or the containing element's unicast address.
Any model may maintain a publish address that it publishes messages to. This publish address can be of any type.

![Access layer structure](images/access.svg)

For more information about models in the nRF5 SDK for Mesh, see @ref md_doc_user_guide_modules_models_main.

---

## Device life cycle @anchor concepts_lifecycle

Every new device that is to be added to the Bluetooth mesh network must go through the following stages
to become a Bluetooth mesh network node:
- [Provisioning](@ref concepts_lifecycle_provisioning) -- after this stage, an unprovisioned device
becomes a network node. This stage includes the following steps:
    - Discovery
    - [Authentication](@ref concepts_security_authentication)
    - [Address](@ref concepts_addressing) assignment and network information exchange -- at the end
    of this step, the device becomes a node.
- [Configuration](@ref concepts_lifecycle_network_config) -- after this stage,
a node is able to perform its tasks that require exchanging Bluetooth mesh messages with neighboring nodes.
This stage includes the following steps:
    - Configuration of the node using the mandatory Config Server model
    - Addition of desired application keys and additional network keys
    - Optional configuration of the application-specific [models](@ref concepts_models_and_elements),
    for example for key bindings, publications or subscriptions (or both)

Both of these stages are typically carried out by one device that acts as provisioner and configurator.

![Bluetooth mesh node life cycle](images/mesh_device_lifecycle.svg)

A Bluetooth mesh node can revert to being an unprovisioned device by performing a Node Reset procedure,
which removes the node from the network.

The node can be also forcibly excluded from participating in the network with the key refresh procedure.
Once the key refresh procedure is completed for the rest of the nodes in a network,
the node's unicast address can be allocated to a new unprovisioned device.

### Provisioning @anchor concepts_lifecycle_provisioning

Before a device can participate in normal Bluetooth mesh operations, it must be provisioned.

The provisioning is done by a _Provisioner_, which is a trusted device with access to the full list
of devices in the network, and their configuration data. After the new device, called _Provisionee_,
has been provisioned, the provisioner uses the new device's device key to establish a secure channel
to configure it.

For more information about provisioning in the nRF5 SDK for Mesh, see @ref md_doc_user_guide_modules_provisioning_main.

### Configuration @anchor concepts_lifecycle_network_config

Bluetooth mesh leaves the network configuration to a central network configurator.
Devices are not expected to do any sort of service discovery on their own.

To control other devices, new devices must be configured by a provisioner, either through user interaction
or by loading a predetermined configuration from a database. Every device must implement a mandatory
Configuration Server model in their first element, which is used to configure the rest of its models.

As soon as provisioning is complete, the provisioner uses its instance of the Configuration Client model
to give the new device a set of application keys and addresses.
The device will use these keys and addresses for the duration of its lifetime on the network,
unless it gets reconfigured.


#### Configuration example scenario: A light bulb and a switch @anchor concepts_lifecycle_network_config_example

After a new light switch has been provisioned:
-# The Configuration Client model in the provisioner reads out a list of the new device's
models and elements and presents them to the user.
-# The user finds the light switch model in the device's model list and gives it the
"Light Control" application key.
-# The user sets the model's publish address to the "Kitchen Area" group address, to which all
the light bulbs in the kitchen subscribe.

The next time the new light switch is pressed, all light bulbs in the kitchen turn on.

---

## Security @anchor concepts_security

Bluetooth mesh employs several security measures to prevent third-party interference and monitoring:
- [Authentication](@ref concepts_security_authentication)
- [Message encryption](@ref concepts_security_encryption)
- [Privacy key](@ref concepts_security_privacy)
- [Replay protection](@ref concepts_security_replay_protection)

### Authentication @anchor concepts_security_authentication

Device authentication is part of the provisioning process and lets the user confirm that the device
being added to the network is indeed the device they think it is.

The Bluetooth mesh profile specification defines a range of out-of-band authentication methods, such as:
- blinking of lights,
- output and input of passphrases,
- static authentication against a pre-shared key.

To secure the provisioning procedure, elliptic curve Diffie-Helman (ECDH) public key cryptography is used.
After a device has been provisioned, it is part of the network and all its messages
are considered authenticated.

### Message encryption @anchor concepts_security_encryption

Bluetooth mesh features two levels of AES-CCM encryption with 128-bit keys for all messages going
across the network:
- **Network encryption**: The lowest layer that protects all messages in a Bluetooth mesh network from being
readable by devices that are not part of the network.
    - The encryption is done with a network encryption key, and any network may consist
    of up to 4096 different subnets, each with their own network key.
    - All devices sharing a network key are considered part of the network and may send
    and relay messages across it.
    - By using multiple network keys, a network administrator may effectively divide their network
    into multiple subnets, because a Bluetooth mesh relay only forwards messages that are encrypted
    with a known network key.
- **Transport encryption**: The second encryption layer that limits which devices can do what
_within a network_ by encrypting the application payload with an application or device key.
    - As an example, consider a mesh network deployed in a hotel, where it is desirable to limit
    some features to be controlled by the staff (like configuration of key cards or access to storage areas)
    and some features to be available to guests (like controlling room lighting or air conditioning).
    For this, we can have one application key for the guests and one for the staff,
    allowing the messages to be relayed across the same network, while preventing the guests
    and the staff from reading each other's messages.

While application keys are used to separate access rights to different applications in the network,
the device keys are used to manage devices in the network.

Every device has a unique device key, which is only known to the provisioner and the device itself.
The device key is used when configuring a device with new encryption keys (network or application keys)
or addresses, in addition to setting other device-specific parameters. It can also be used to evict
malicious devices from a network by transferring new keys to all the other devices in the network
(using their individual device keys when transferring the keys).
This process is called the _Key Refresh Procedure_.

Each encryption layer contains a message integrity check value that validates that the content
of the message was encrypted with the indicated encryption keys.

### Privacy key @anchor concepts_security_privacy

All Bluetooth mesh message payloads are fully encrypted. Message metadata like source address
and message sequence number is obfuscated with the privacy key, derived from the network key,
providing limited privacy even for public header fields.

### Replay protection @anchor concepts_security_replay_protection

To guard against malicious devices replaying previous messages, every device keeps a running sequence number,
which is used for outbound messages. Each Bluetooth mesh message is sent with a unique pair of sequence number
and source address. When receiving a message, the receiving device stores the sequence number
and makes sure that it is more recent than the last sequence number it received from the same source
address.