# Proprietary mesh DFU protocol
@anchor dfu-protocol

A Device Firmware Update (DFU) is the process of updating the firmware on a Bluetooth mesh device.

Nordic Semiconductor's mesh DFU protocol is adopted from the proprietary @link_openmesh_github
project, and operates on
the proprietary OpenMesh protocol. The OpenMesh protocol is an advertising based protocol like the
Bluetooth mesh, but it does not support addressing, acknowledged message passing or encryption.

This page and its subpages describe the proprietary mesh DFU protocol, including @subpage md_tools_dfu_README
and @subpage md_mesh_bootloader_README.
In this section, you can also learn more about @subpage md_doc_user_guide_modules_dfu_setup
using the [DFU example](@ref dfu_example) included in the nRF5 SDK for Mesh.

**Table of contents**
- [Characteristics](@ref dfu-protocol-characteristics)
    - [Transfer modes and types](@ref dfu-protocol-transfer-modes)
    - [Roles](@ref dfu-protocol-roles)
    - [Concurrent transfers](@ref dfu-protocol-receive-and-relay)
    - [Mixed-device networks](@ref dfu-protocol-network)
    - [Transfer rate](@ref dfu-protocol-transfer-rate)
    - [Background operation](@ref dfu-protocol-background)
    - [Transfer banking](@ref dfu-protocol-banking)
    - [Memory map](@ref dfu-protocol-memory-map)
    - [Security](@ref dfu-protocol-security)
- [DFU Firmware IDs](@ref dfu-protocol-fwid)
    - [Application firmware ID](@ref dfu-protocol-fwid-app)
    - [SoftDevice firmware ID](@ref dfu-protocol-fwid-sd)
    - [Bootloader firmware ID](@ref dfu-protocol-fwid-bl)
- [Device page](@ref dfu-protocol-device-page)
    - [Format](@ref dfu-protocol-device-page-format)
    - [Contents](@ref dfu-protocol-device-page-contents)
        - [Signature public key](@ref dfu-protocol-device-page-format-signature-key)
        - [Firmware ID](@ref dfu-protocol-device-page-format-fwid)
        - [Flags](@ref dfu-protocol-device-page-format-flags)
        - [SoftDevice area](@ref dfu-protocol-device-page-format-sd-area)
        - [Bootloader area](@ref dfu-protocol-device-page-format-bl-area)
        - [Application area](@ref dfu-protocol-device-page-format-app-area)
        - [SoftDevice signature](@ref dfu-protocol-device-page-format-sd-signature)
        - [Bootloader signature](@ref dfu-protocol-device-page-format-bl-signature)
        - [Application signature](@ref dfu-protocol-device-page-format-app-signature)
        - [SoftDevice bank](@ref dfu-protocol-device-page-format-sd-bank)
        - [Bootloader bank](@ref dfu-protocol-device-page-format-bl-bank)
        - [Application bank](@ref dfu-protocol-device-page-format-app-bank)


---

## Characteristics @anchor dfu-protocol-characteristics

The proprietary mesh DFU protocol is optimized for updating all devices in a network as efficiently as
possible. Although it shares some tooling and code modules with @link_bootloader_and_dfu_modules,
there are several differences both in protocol and operation to make updating of large amounts
of devices as painless as possible.


| Category                                                      | nRF5 SDK DFU                                                 | nRF5 SDK for Mesh DFU                                 |
|---------------------------------------------------------------|--------------------------------------------------------------|-------------------------------------------------------|
| [Transfer modes](@ref dfu-protocol-transfer-modes)            | Background and bootloader DFUs.                              | Background and bootloader DFUs.                       |
| [Transfer types](@ref dfu-protocol-transfer-modes)            | SoftDevice, bootloader, application, SoftDevice and bootloader.<br>SoftDevice and bootloader can be updated independently.  | SoftDevice, bootloader, application.<br>SoftDevice and bootloader can be updated independently.  |
| [Roles](@ref dfu-protocol-roles)                              | DFU target and DFU controller.                               | DFU source, DFU target, and DFU relay.                |
| [Concurrent transfers](@ref dfu-protocol-receive-and-relay)   | Not possible: each device updated individually.              | Implemented: entire network updated simultaneously.   |
| [Device network](@ref dfu-protocol-network)                   | Devices with single role or firmware.                        | Network of devices of different role and firmware.    |
| [Transfer rate](@ref dfu-protocol-transfer-rate)              | Fast transfer rate.                                          | Very slow transfer rate.                              |
| [Background operation] (@ref dfu-protocol-background)         | Background mode transfer in the application not available.   | Background mode transfer in the application implemented and active by default.    |
| [Transfer banking](@ref dfu-protocol-banking)                 | Banking in an unused area of flash.                          | Banking in an unused area of flash.                   |
| [Memory map](@ref dfu-protocol-memory-map)                    | Flash memory map with the MBR parameter storage between the bootloader and the bootloader settings.  | The same flash memory map as for the nRF5 SDK, but with the MBR parameter storage between the bootloader and the application area.    |
| [Security](@ref dfu-protocol-security)                        | No encryption on the DFU level. Optional security key signing.           | No encryption on the DFU level. Optional security key signing.         |


### Transfer modes and types @anchor dfu-protocol-transfer-modes

The proprietary mesh DFU supports two modes:
- _Background DFU_ that transfers the new firmware in the background in the application
while it is running, and reports to the application when the transfer is done.
The application can then flash the new firmware when ready.
    - To learn how to configure the DFU example application with the background DFU mode,
    see [Configuring DFU over Mesh](@ref md_doc_user_guide_modules_dfu_configuring_performing).
- _Bootloader DFU_, in which the application is not running and the [Mesh Bootloader](@ref md_mesh_bootloader_README)
takes care of the transfer.
    - This mode is primarily meant as a fallback mechanism, in case the application malfunctions.

As part of each transfer mode, the proprietary mesh DFU protocol is using three different transfer types (packages):
- SoftDevice
- bootloader
- application

Each of these three firmware pieces must be transferred separately, and each has its
own identifier. Each has also its own area in the [memory map](@ref dfu-protocol-memory-map).

The nRF5 SDK for Mesh SoftDevice and bootloader can be updated independently.
This saves time and memory if you want to make small changes in the bootloader.

### Roles @anchor dfu-protocol-roles

The following roles are supported by the proprietary mesh DFU:
    - The _source_ role is used when the device acts as an initiator of the DFU transfer.
    The source device controls an interval at which packets are sent.
    It also responds to [DFU data request](@ref dfu-packet-data-request) packets.
    The source role is controlled by the nRF Util tool.
    See @link_nrfutil_github and [Requirements](@ref dfu_configuration_requirements)
    for more information about nRF Util.
    - The _target_ role is used when the device needs to be upgraded. The proprietary mesh DFU module receives
    the new firmware and notifies the application when the transfer is done. In this role the proprietary mesh DFU
    module also retransmits the DFU data packets it receives.
    - The _relay_ role is used to retransmit the [DFU data](@ref dfu-packet-data) packets received
    from other devices. In this role, the device does not store the received packets in the flash.

See @ref md_doc_user_guide_modules_dfu_integrating_into_app for more information about
when the DFU module switches to the target and relay roles.

### Concurrent transfers @anchor dfu-protocol-receive-and-relay

Contrary to the nRF5 SDK DFU, the entire Bluetooth mesh network can be updated simultaneously
with concurrent transfers.

A Bluetooth mesh network can contain hundreds of devices, and updating all of them one by one can take
a lot of time. To get around this, the proprietary mesh DFU protocol lets mesh network devices relay the data
they are receiving to their neighbors. Both the passive mesh devices and the devices receiving
a transfer will relay all data packets, thus ensuring that the DFU transfer reaches all devices
in the network. This method is much faster than passing the entire DFU transfer to each device
individually.

A device that receives a DFU transfer will perform the following steps for each data message it receives:

1. Verification that the data packet has not already been received.
2. Storing of the payload of the data packet in the flash at the appropriate offset in the transfer data.
3. Marking of the data packet as received.
4. Retransmission of the data packet a predefined amount of times at an exponential interval.

Devices that are not directly interested in the contents of the transfer, will only perform steps 1,
3, and 4 to ensure that the target devices further out in the network still receive the packets.

To learn more about proprietary mesh DFU packets, see @subpage md_doc_user_guide_modules_dfu_packet_format.

### Mixed-device networks @anchor dfu-protocol-network

Typically, a Bluetooth mesh network contains devices with several different roles and firmware. When
performing a DFU transfer, it is important to be able to distinguish between these devices.
If an update for light switch devices is flashed to a light bulb, it is likely that the light bulb stops
working. This is different for one-to-one DFU transfers, like the ones performed by the nRF5 SDK DFU
protocol, where the sender is able to identify the target device and send it the correct firmware.

The proprietary mesh DFU protocol deals with mixed-device networks by allowing each device type to have its own
application ID in addition to an application version number. When a device is notified of an
upcoming DFU transfer, it can compare the application ID of the transfer with the application ID of
its own firmware. If the application IDs match, and the incoming transfer has a higher version, the
device generally accepts the transfer. If a device is notified of an upcoming DFU transfer with
a different application ID, it can choose to either act as a relay device for this transfer, or
to ignore it.

### Transfer rate @anchor dfu-protocol-transfer-rate

Compared to the nRF5 SDK DFU protocol, the proprietary mesh DFU transfer is quite slow.
For example, a 100 kB firmware image will take about an hour to transfer.

The proprietary mesh DFU protocol depends on redundancy to ensure reliable communication, and therefore takes
a lot longer to propagate the same amount of data than the nRF5 SDK DFU. The DFU data is sent
in 16-byte chunks at regular intervals, with a few redundant transmissions for every packet
to ensure that all devices receive it.

The packet interval is controlled by the transfer source device. By default, the Bluetooth mesh
DFU tools emit a new packet every 500 ms (making the transfer rate `16 B/500 ms = 32 B/s`), but this
number should be tuned according to the Bluetooth mesh network properties.

Some network characteristics to consider are:
- _Network density_: The amount of devices that are within radio range of each other impacts the
packet receive rate significantly. A higher number of devices within the radio distance of each other
causes more packet collisions, which decreases the total throughput.
- _Network span_: Each hop in the network poses a certain risk of packet loss, and creates a delay in
the traffic. The higher the number of hops in the network, the higher the risk of some devices
missing the transfer.
- _Network topology_: While a high node density could have a negative impact on the transfer success
rate, having too few paths to reach a target node can cause packet drops. The more a target node relies
on several relay devices to all succeed with their transmissions, the higher the likelihood of missing that
target node at some point during the transfer.
- _External noise_: When deployed in a noisy environment, the proprietary mesh DFU performs worse (like all wireless
technologies).

Because these characteristics are different for all deployments, it is not possible to make a general
rule that will work for all networks. To maximize the DFU performance:
- tune individual deployments;
- if possible, schedule the DFU transfers in an expected low-traffic period with the least amount of noise.

### Background operation @anchor dfu-protocol-background

Due to the relatively slow transfer rate of the proprietary mesh DFU protocol, transfers could end up taking
over an hour, which is an unacceptable downtime for a lot of applications. To deal with this, the
proprietary mesh DFU implements a background mode.

The background mode allows the application to continue the normal operation
while the DFU transfer progresses. The background mode is the default mode of operation, unless there is
no valid application on the device, in which case it falls back to the bootloader.

### Transfer banking @anchor dfu-protocol-banking

Devices receiving background transfers have to store the incoming transfer in a bank in an unused
area of flash, to avoid overwriting themselves while they're running. While in progress, the DFU
transfer data gets stored in the bank, and once finished, the application can tell the bootloader to
copy the bank to the application area, effectively finalizing the update. Although the bootloader
notifies the application about the completion of the transfer as soon as it can, the application is
free to copy the bank at any point, or even not at all. There can only be one bank per transfer type
at the time, and banks cannot overlap. Finishing a banked DFU transfer will remove any existing bank
of the same transfer type.

Note that the bank must be placed in a flash area that is both large enough to fit the entire
incoming application during the transfer, and that doesn't overlap with the new application or old
application. To ensure maximum space for the incoming transfer, both as a bank and as a finished
application, it's generally recommended to place the start of the bank right in the middle of the
application section of the device. If the transfer is not able to fit both as a bank or as an
application after the transfer is finalized, the device must fall back to the bootloader, and
perform the transfer in the bootloader mode.

### Memory map @anchor dfu-protocol-memory-map

The proprietary mesh DFU protocol uses the same flash memory map as the nRF5 SDK
@link_bootloader_and_dfu_modules, with one minor difference. Instead of placing the MBR parameter
storage between the bootloader and the bootloader settings (called [device page](@ref dfu-protocol-device-page)
in the proprietary mesh DFU), the MBR parameter storage in the proprietary mesh DFU protocol goes between
the bootloader and the application area.

![Flash memory map](images/bootloader_memory_nrf52.svg)

The memory map contains the following major firmware elements:
- SoftDevice
- Application
- Bootloader

These correspond to the [transfer types](@ref dfu-protocol-transfer-modes) mentioned earlier.
Each firmware element can be updated individually with a DFU transfer.

The application uses the bootloader to perform the [receive-and-relay algorithm steps](@ref dfu-protocol-receive-and-relay),
even when working in the background mode. When initializing the Bluetooth mesh framework, the DFU module initializes
a command handler module in the bootloader, which runs alongside the application.

To be able to run alongside the application, the bootloader reserves the last 768 bytes of RAM
on the device. This reserved RAM is accounted for in all Bluetooth mesh project files and linker scripts.
Failing to reserve these bytes causes unexpected behavior from the bootloader
when the application starts.

### Security @anchor dfu-protocol-security

Similarly to the nRF5 SDK, the proprietary mesh DFU does not encrypt the DFU transfer data.

In the Bluetooth mesh case, this is a limitation inherited from the OpenMesh protocol, and means that under
no circumstance should security-sensitive data (like keys) be sent as part of a DFU transfer.

The proprietary mesh DFU does however feature Elliptic Curve Digital Signatures (ECDSA) for authenticating the transfers.
Although signing is optional, it is highly recommended to sign all transfers.
Signing is performed with a private signing key when the DFU transfer is created,
and all Bluetooth mesh devices can get preprogrammed with a matching public signing key to
authenticate the firmware. If a Bluetooth mesh device has a public signing key, it will always require that
the signature passes before the transfer is finalized.

The signing algorithm is performed by creating a SHA256 hash of the transfer metadata and firmware.

See the following table for the breakdown of the hash.

| Field                    | Offset (bytes) | Size (bytes) | Value                                                      |
|--------------------------|----------------|--------------|------------------------------------------------------------|
| DFU type                 | 0              | 1            | _DFU type_ from the DFU state packet.                      |
| Start address            | 1              | 4            | _Start address_ from the DFU start packet.                 |
| Firmware length in bytes | 5              | 4            | `L`, `L` = 4 * _Firmware Length_ from the DFU start packet.|
| Padding                  | 9              | 1            | `0`                                                        |
| Firmware ID              | 10             | `F`          | _Firmware ID_ from the DFU state packet.                   |
| Firmware data            | 10 + `F`       | `L`          | The entire firmware image (not including signature).       |

@note The size of the firmware ID (`F`) depends on the type of transfer. See [DFU Firmware IDs](@ref dfu-protocol-fwid).

The signature is created by ECDSA with the NIST P-256 curve (secp256r1):

```
signature = ecc_sign(curve=P-256, private_key, hash)
```

The signature is verified on the target device with the matching public key:

```
authenticated = ecc_verify(curve=P-256, public_key, hash, signature)
```

@warning As the entire firmware data is required for the hash, the signature is not checked until
after the entire transfer is complete. This design creates a possibility for a denial of service
attack on the target devices, as an attacker might be able to initiate a false transfer. This false
transfer could be indistiguishable from a normal transfer until the final signature check, at which
point the target device may have spent significant resources receiving the transfer.
However, this attack vector does not allow the attacker to execute any code on the target device,
as the target device will delete all knowledge of the transfer when the signature check fails.


---

## DFU Firmware IDs @anchor dfu-protocol-fwid

All proprietary mesh DFU transfers are identified by a firmware ID. The structure of the firmware ID depends on
the [transfer type](@ref dfu-protocol-transfer-modes):
- [Application firmware ID](@ref dfu-protocol-fwid-app)
- [SoftDevice firmware ID](@ref dfu-protocol-fwid-sd)
- [Bootloader firmware ID](@ref dfu-protocol-fwid-bl)

To decide whether to accept an incoming transfer or not, all Bluetooth mesh devices carry their current firmware IDs
in their [device page](@ref dfu-protocol-device-page).

### Application firmware ID @anchor dfu-protocol-fwid-app

The application firmware ID identifies applications.

Every application has an ID and a version number. Generally, a device should accept DFU application transfers
with the same application ID as their own firmware, and higher version numbers. The version number is a 32-bit number,
and the versioning scheme is user-definable.

When in the bootloader mode, the prebuilt bootloader enforces a
strictly increasing version number. This prevents malicious devices from downgrading firmware to a
previous version, an attack that could be used to reintroduce old weaknesses in the firmware.

When in the background mode, this rule is not enforced, but strongly recommended, as there is no
other native way to prevent malicious downgrades.

To create a unique firmware ID for each application, the application ID includes a 32-bit
_Company ID_ field, which identifies the device vendor. The _Company ID_ field can either contain:
- a @link_companyID (`0xFFFF` or lower), or
- a randomly chosen identifier higher than `0xFFFF`.

The randomly chosen identifier is not guaranteed to be unique, but it allows vendors that do not
have an assigned Company ID to keep a high probability of uniqueness.

| Field               | Offset (bytes) | Size (bytes) |
|---------------------|----------------|--------------|
| Company ID          | 0              | 4            |
| Application ID      | 4              | 2            |
| Application version | 6              | 4            |

### SoftDevice firmware ID @anchor dfu-protocol-fwid-sd

Nordic Semiconductor assigns a unique 16-bit @link_softdevice_id for every SoftDevice release,
which can be read out from the firmware.

The SoftDevice firmware ID on the device is meant to match this number
(although this is not explicitly required for correct operation).
As the SoftDevice IDs represent the release IDs, and not the increasing version numbers,
the strategy for whether to accept an incoming SoftDevice DFU transfer to take or not is user-definable.

When in the bootloader mode, the bootloader starts requesting SoftDevice updates if it receives
a FWID beacon with an application ID that represents a higher version of its current application,
but a different SoftDevice firmware ID.

When in the background mode, the upgrade strategy is user-definable.

| Field         | Offset (bytes) | Size (bytes) |
|---------------|----------------|--------------|
| SoftDevice ID | 0              | 2            |

@note Due to limitations in the bootloader implementation, the SoftDevice DFU transfers can only be
accepted if the new SoftDevice is able to fit within the defined SoftDevice area (see [Device page](@ref dfu-protocol-device-page)).

### Bootloader firmware ID @anchor dfu-protocol-fwid-bl

The bootloader firmware ID consists of an 8-bit Bootloader ID field and an 8-bit Bootloader
version field. Just like applications, bootloaders can come in different configurations, and each
configuration can come in different versions.

When in the bootloader mode, the bootloader
accepts incoming bootloader DFU transfers when the _Bootloader ID_ field is identical to its
current bootloader ID and the _Bootloader version_ field is larger than the current bootloader
version.

When in the background mode, the upgrade strategy is user-definable.

| Field              | Offset (bytes) | Size (bytes) |
|--------------------|----------------|--------------|
| Bootloader ID      | 0              | 1            |
| Bootloader version | 1              | 1            |


---

## Device page @anchor dfu-protocol-device-page

All devices running the proprietary mesh DFU bootloader are required to keep a device page in flash. The device
page defines the device configuration and acts as operational parameters for the bootloader.

The device page must be generated on a host computer and flashed on each device before
deployment. The `device_page_generator.py` script for generating device pages can be found in `tools/dfu/`.
For details, see [Device Page Generator Tool](@ref dfu-device-page-generator).

### Format @anchor dfu-protocol-device-page-format

The device page is a single-page [flash manager area](@ref flash_manager_areas) in the last flash
page of the device.

### Contents @anchor dfu-protocol-device-page-contents

The device page contains all the information the bootloader needs to participate in the DFU transfers:
- **Flash areas:** Each transfer type (SoftDevice, bootloader, and application) has an assigned area
in flash. These areas are defined in the device page, and must be able to contain the largest
potential firmware piece of their type.
- **Firmware IDs:** Each transfer type (SoftDevice, bootloader, and application) has an assigned
firmware ID. These firmware IDs are used for deciding whether to accept the incoming DFU transfers or not,
and are updated once each DFU transfer is finalized.
- **Public signing keys:** Used for verifying the DFU transfer signature. See the [DFU protocol security](@ref dfu-protocol-security)
section for details.
- **Status flags:** Flags indicating the validity of each firmware piece (SoftDevice, bootloader,
and application).
- **Banked transfers:** Each banked transfer on the device has a dedicated structure
describing it. This entry type is generated by the bootloader after each transfer banked successfully.
There can only be one bank per transfer type.
- **Firmware signatures:** The signature of each firmware piece, if present.

See the following table for the list of possible entries, including required and optional ones.

| Entry                                                                       | ID       | Required           |
|-----------------------------------------------------------------------------|----------|--------------------|
| [Signature public key](@ref dfu-protocol-device-page-format-signature-key)  | `0x0001` | @tagGreenTick      |
| [Firmware ID](@ref dfu-protocol-device-page-format-fwid)                    | `0x0002` | @tagGreenTick      |
| [Flags](@ref dfu-protocol-device-page-format-flags)                         | `0x0004` | @tagRedCross       |
| [SoftDevice area](@ref dfu-protocol-device-page-format-sd-area)             | `0x0010` | @tagGreenTick      |
| [Bootloader area](@ref dfu-protocol-device-page-format-bl-area)             | `0x0011` | @tagGreenTick      |
| [Application area](@ref dfu-protocol-device-page-format-app-area)           | `0x0012` | @tagGreenTick      |
| [SoftDevice signature](@ref dfu-protocol-device-page-format-sd-signature)   | `0x001A` | @tagRedCross       |
| [Bootloader signature](@ref dfu-protocol-device-page-format-bl-signature)   | `0x001B` | @tagRedCross       |
| [Application signature](@ref dfu-protocol-device-page-format-app-signature) | `0x001C` | @tagRedCross       |
| [SoftDevice bank](@ref dfu-protocol-device-page-format-sd-bank)             | `0x0021` | @tagRedCross       |
| [Bootloader bank](@ref dfu-protocol-device-page-format-bl-bank)             | `0x0022` | @tagRedCross       |
| [Application bank](@ref dfu-protocol-device-page-format-app-bank)           | `0x0024` | @tagRedCross       |

#### Signature public key @anchor dfu-protocol-device-page-format-signature-key

Public key used for signature verification.

| Field      | Offset (bytes) | Size (bytes) |
|------------|----------------|--------------|
| Public key | 0              | 64           |

#### Firmware ID @anchor dfu-protocol-device-page-format-fwid

Current firmware ID. For details, see [DFU Firmware IDs](@ref dfu-protocol-fwid).

@note The firmware ID entry is a concatenation of the three different proprietary mesh firmware IDs.

| Field               | Offset (bytes) | Size (bytes) |
|---------------------|----------------|--------------|
| SoftDevice ID       | 0              | 2            |
| Bootloader ID       | 2              | 1            |
| Bootloader version  | 3              | 1            |
| Company ID          | 4              | 4            |
| Application ID      | 8              | 2            |
| Application version | 10             | 4            |


#### Flags @anchor dfu-protocol-device-page-format-flags

Current state of each firmware piece.

| Field               | Offset (bytes) | Size (bytes) |
|---------------------|----------------|--------------|
| SoftDevice intact   | 0              | 1            |
| Bootloader intact   | 1              | 1            |
| Application intact  | 2              | 1            |
| Device page invalid | 3              | 1            |

#### SoftDevice area @anchor dfu-protocol-device-page-format-sd-area

Flash area where the SoftDevice can reside.

| Field                           | Offset (bytes) | Size (bytes) |
|---------------------------------|----------------|--------------|
| SoftDevice area start address   | 0              | 4            |
| SoftDevice area length in bytes | 4              | 4            |

#### Bootloader area @anchor dfu-protocol-device-page-format-bl-area

Flash area where the bootloader resides.

| Field                           | Offset (bytes) | Size (bytes) |
|---------------------------------|----------------|--------------|
| Bootloader area start address   | 0              | 4            |
| Bootloader area length in bytes | 4              | 4            |

#### Application area @anchor dfu-protocol-device-page-format-app-area

Flash area where the application can reside.

| Field                            | Offset (bytes) | Size (bytes) |
|----------------------------------|----------------|--------------|
| Application area start address   | 0              | 4            |
| Application area length in bytes | 4              | 4            |

#### SoftDevice signature @anchor dfu-protocol-device-page-format-sd-signature

Signature of the current SoftDevice.

| Field                | Offset (bytes) | Size (bytes) |
|----------------------|----------------|--------------|
| SoftDevice signature | 0              | 64           |

#### Bootloader signature @anchor dfu-protocol-device-page-format-bl-signature

Signature of the current bootloader.

| Field                | Offset (bytes) | Size (bytes) |
|----------------------|----------------|--------------|
| Bootloader signature | 0              | 64           |

#### Application signature @anchor dfu-protocol-device-page-format-app-signature

Signature of the current application.

| Field                 | Offset (bytes) | Size (bytes) |
|-----------------------|----------------|--------------|
| Application signature | 0              | 64           |

#### SoftDevice bank @anchor dfu-protocol-device-page-format-sd-bank

Information about a banked SoftDevice transfer.

Padding is a fixed data field that has always a `0` value.

| Field                     | Offset (bytes) | Size (bytes) |
|---------------------------|----------------|--------------|
| Bank address              | 0              | 4            |
| Bank length in bytes      | 4              | 4            |
| Bank SoftDevice ID        | 8              | 2            |
| _Padding_                 | 10             | 8            |
| Bank is signed            | 18             | 1            |
| Bank state                | 19             | 1            |
| Bank signature (optional) | 20             | 64           |

#### Bootloader bank @anchor dfu-protocol-device-page-format-bl-bank

Information about a banked bootloader transfer.

Padding is a fixed data field that has always a `0` value.

| Field                     | Offset (bytes) | Size (bytes) |
|---------------------------|----------------|--------------|
| Bank address              | 0              | 4            |
| Bank length in bytes      | 4              | 4            |
| Bank Bootloader ID        | 8              | 1            |
| Bank Bootloader version   | 9              | 1            |
| _Padding_                 | 10             | 8            |
| Bank is signed            | 18             | 1            |
| Bank state                | 19             | 1            |
| Bank signature (optional) | 20             | 64           |

#### Application bank @anchor dfu-protocol-device-page-format-app-bank

Information about a banked application transfer.

| Field                     | Offset (bytes) | Size (bytes) |
|---------------------------|----------------|--------------|
| Bank address              | 0              | 4            |
| Bank length in bytes      | 4              | 4            |
| Bank Company ID           | 8              | 4            |
| Bank Application ID       | 12             | 2            |
| Bank Application version  | 14             | 4            |
| Bank is signed            | 18             | 1            |
| Bank state                | 19             | 1            |
| Bank signature (optional) | 20             | 64           |

