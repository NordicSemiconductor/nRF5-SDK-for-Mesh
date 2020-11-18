# DFU protocol packet formats
@anchor dfu-packet-format

The proprietary mesh DFU packets are packets of data transferred between devices during the proprietary mesh
DFU protocol process.

Their format corresponds to the proprietary @link_openmesh_github packets.
The proprietary mesh DFU packets do not share any packet fields with the Bluetooth mesh packets,
apart from their common underlying Bluetooth 4.0 protocol.

@note All fields use the little-endian order.

**Table of contents**
- [Bearers](@ref dfu-packet-bearers)
    - [OpenMesh bearer](@ref dfu-packet-bearers-om)
    - [Serial bearer](@ref dfu-packet-bearers-serial)
- [Transfer info and ID fields](@ref dfu-packet-state-transfer-subsection)
    - [Transfer info field](@ref dfu-packet-state-transfer-info)
    - [Transfer ID field](@ref dfu-packet-state-transfer-id)
- [DFU packets](@ref dfu-packet-packets)
    - [Firmware ID](@ref dfu-packet-fwid)
    - [DFU state](@ref dfu-packet-state)
        - [DFU state SoftDevice](@ref dfu-packet-state-sd)
        - [DFU state Bootloader](@ref dfu-packet-state-bl)
        - [DFU state Application](@ref dfu-packet-state-app)
    - [DFU start](@ref dfu-packet-start)
        - [DFU start Flags field](@ref dfu-packet-start-flags)
    - [DFU data](@ref dfu-packet-data)
    - [DFU data request](@ref dfu-packet-data-request)
    - [DFU data response](@ref dfu-packet-data-response)


---

## Bearers @anchor dfu-packet-bearers

The proprietary mesh DFU packets are sent:
- over the air as nRF OpenMesh packets,
- through the [Bluetooth mesh serial interface](@ref md_doc_user_guide_modules_serial).

### OpenMesh bearer @anchor dfu-packet-bearers-om

When going over the air between Bluetooth mesh devices, the DFU packets are sent as OpenMesh advertising
data in the Bluetooth 4.0 advertisements. The OpenMesh data is sent as "Service Data - 16-bit UUID"
Advertising Data Type, with the Nordic Semiconductor's Service UUID `0xFEE4`. Just like the
Bluetooth mesh packets, the OpenMesh data is placed in the Advertisement data of an advertisement packet.

See the following table for the advertising data breakdown.

| Field        | Offset (bytes) | Length (bytes) | Value                  |
|--------------|----------------|----------------|------------------------|
| AD length    | 0              | 1              | `L + 4`                |
| AD type      | 1              | 1              | `0x16`                 |
| Service UUID | 2              | 2              | `0xFEE4`               |
| DFU packet   | 4              | `L`            | Proprietary mesh DFU protocol data. |

### Serial bearer @anchor dfu-packet-bearers-serial

The proprietary mesh DFU packets have an assigned command opcode in the
[Bluetooth mesh serial interface](@ref md_doc_user_guide_modules_serial), which allows them to be encapsulated in the
payload of the [serial packets](@ref serial_interface_packet_format).

See the following table for the proprietary mesh DFU serial packet breakdown.

| Field      | Offset (bytes) | Length (bytes) | Value                  |
|------------|----------------|----------------|------------------------|
| Length     | 0              | 1              | `L + 1`                |
| Opcode     | 1              | 1              | `0x78`                 |
| DFU packet | 2              | `L`            | Proprietary mesh DFU protocol data. |


---

## Transfer info and ID fields @anchor dfu-packet-state-transfer-subsection

The DFU packets can contain additional information and ID fields that further specify
the transfer characteristics.

### Transfer info field @anchor dfu-packet-state-transfer-info

All DFU state packet variants contain a 1-byte _Transfer info_ field that contains the following fields:

| Field     | Offset (bits) | Length (bits) | Description                                                       |
|-----------|---------------|---------------|-------------------------------------------------------------------|
| Authority | 0             | 3             | Authority level of the transfer source.                           |
| Flood     | 3             | 1             | Whether the transfer source wants devices to flood this transfer. |
| RFU       | 4             | 4             | Reserved for future use.                                          |

The _Authority_ field is used to prioritize conflicting DFU transfer sources if there are
multiple. Target devices will prioiritize transfers from higher authority sources.
@note This is not a security mechanism, but rather a way to facilitate multiple sources of transfers.

The _Flood_ field allows the transfer source to prevent the transfer from being flooded through
the network if it knows that only the local device is to receive the transfer.
@note This field is not implemented in the bootloader, and is ignored as of the nRF5 SDK for Mesh v2.0.0.

The _RFU_ field must be set to `0` by the sender and ignored by the receivers.

### Transfer ID field @anchor dfu-packet-state-transfer-id

Each DFU transfer is identified by a 32-bit random number identifier. This identifier is present
in all DFU packets (except for the Firmware ID packet) and is used by the Bluetooth mesh devices to determine
which transfer the packets belong to.

Transfer sources must pick a new pseudorandom number for every transfer to minimize risk of conflicts
and invalid behavior.


---

## DFU packets @anchor dfu-packet-packets

The following proprietary mesh DFU packet types exist, with some types having slight variations depending
on which type of transfer is being performed:
- [Firmware ID](@ref dfu-packet-fwid)
- [DFU state](@ref dfu-packet-state)
- [DFU start](@ref dfu-packet-start)
- [DFU data](@ref dfu-packet-data)
- [DFU data request](@ref dfu-packet-data-request)
- [DFU data response](@ref dfu-packet-data-response)

The proprietary mesh DFU packets are inserted into the _DFU packet_ field of the bearer packets
without any form of encryption or obfuscation.

All proprietary mesh DFU packets start with a 16-bit _Packet type_ field.

### Firmware ID @anchor dfu-packet-fwid

The Firmware ID packets are broadcasted when the device is not participating in a DFU transfer.
Their role is to inform the neighboring devices of the firmware version of the sender.
This allows the devices to discover whether their firmware ID is outdated,
while providing diagnostic information to the user.

See [DFU Firmware IDs](@ref dfu-protocol-fwid) for more information about the fields
in the Firmware ID packets.

See the following table for proprietary mesh DFU Firmware ID packet breakdown.

| Field               | Offset (bytes) | Length (bytes) | Value                                     |
|---------------------|----------------|----------------|-------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFE`                                  |
| Softdevice ID       | 2              | 2              | Current Softdevice ID.                    |
| Bootloader ID       | 4              | 1              | Current bootloader ID.                    |
| Bootloader Version  | 5              | 1              | Current bootloader version.               |
| Company ID          | 6              | 4              | Company identifier of the device vendor.  |
| Application ID      | 10             | 2              | Current application ID.                   |
| Application version | 12             | 4              | Current application version.              |

### DFU state variants @anchor dfu-packet-state

The DFU state packets are used during the DFU transfer setup to propagate information about the
transfer. Devices can use this information to determine whether they want to participate in the
transfer.

The DFU state packets come in three different variants, one for each [transfer type](@ref dfu-protocol-transfer-modes):
- [DFU state Softdevice](@ref dfu-packet-state-sd)
- [DFU state Bootloader](@ref dfu-packet-state-bl)
- [DFU state Application](@ref dfu-packet-state-app)

All the variants contain the same information, but their firmware ID part only contains the
relevant information for the given DFU type, as well as the relevant [transfer information](@ref dfu-packet-state-transfer-info)
and [transfer ID](@ref dfu-packet-state-transfer-id).

#### DFU state Softdevice @anchor dfu-packet-state-sd

When the transfer contains a Softdevice, the _DFU type_ field is set to `0x01`, and the packet
contains only the Softdevice-related part of the firmware ID. See the following table for details.

| Field         | Offset (bytes) | Length (bytes) | Value                                                          |
|---------------|----------------|----------------|----------------------------------------------------------------|
| Packet type   | 0              | 2              | `0xFFFD`                                                       |
| DFU type      | 2              | 1              | `0x01`                                                         |
| Transfer info | 3              | 1              | Transfer information. See [Transfer info field](@ref dfu-packet-state-transfer-info). |
| Transfer ID   | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id).    |
| Softdevice ID | 8              | 2              | New Softdevice ID.                                             |

#### DFU state Bootloader @anchor dfu-packet-state-bl

When the transfer contains a Bootloader, the _DFU type_ field is set to `0x02`, and the packet
contains only the Bootloader-related part of the firmware ID. See the following table for details.

| Field              | Offset (bytes) | Length (bytes) | Value                                                          |
|--------------------|----------------|----------------|----------------------------------------------------------------|
| Packet type        | 0              | 2              | `0xFFFD`                                                       |
| DFU type           | 2              | 1              | `0x02`                                                         |
| Transfer info      | 3              | 1              | Transfer information. See [Transfer info field](@ref dfu-packet-state-transfer-info). |
| Transfer ID        | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id).    |
| Bootloader ID      | 8              | 1              | New bootloader ID.                                             |
| Bootloader version | 9              | 1              | New bootloader version.                                        |

#### DFU state Application @anchor dfu-packet-state-app

When the transfer contains an application, the _DFU type_ field is set to `0x04`, and the packet
contains only the application-related part of the firmware ID. See the following table for details.

| Field               | Offset (bytes) | Length (bytes) | Value                                                          |
|---------------------|----------------|----------------|----------------------------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFD`                                                       |
| DFU type            | 2              | 1              | `0x04`                                                         |
| Transfer info       | 3              | 1              | Transfer information. See [Transfer info field](@ref dfu-packet-state-transfer-info). |
| Transfer ID         | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id).    |
| Company ID          | 8              | 4              | Company identifier of the device vendor.                       |
| Application ID      | 12             | 2              | New application ID.                                            |
| Application version | 14             | 4              | New application version.                                       |


### DFU start @anchor dfu-packet-start

The DFU start packet marks the beginning of the transfer and provides additional information
about the transfer. The DFU start packet is the first data segment, and its _Segment index_ field
is always `0x0000`.

| Field               | Offset (bytes) | Length (bytes) | Value                                                         |
|---------------------|----------------|----------------|---------------------------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFC`                                                     |
| Segment index       | 2              | 2              | `0x0000`                                                     |
| Transfer ID         | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id).   |
| Start address       | 8              | 4              | Start address for the transfer, or `0xFFFFFFFF`.              |
| Firmware length     | 12             | 4              | Transfer length in words.                                     |
| Signature length    | 16             | 2              | Signature length in bytes or `0` if signature is not enabled. |
| Flags               | 18             | 1              | Flags. See [DFU start Flags field](@ref dfu-packet-start-flags).                       |

#### DFU start Flags field @anchor dfu-packet-start-flags

The DFU start packet contains a flag field at the end. This field lists some additional properties
of the transfer.
@note As of the nRF5 SDK for Mesh v2.0.0, the implementation of the bootloader only supports the specified values.

| Field          | Offset (bits) | Length (bits) | Value | Description                                                   |
|----------------|---------------|---------------|-------|---------------------------------------------------------------|
| RFU            | 0             | 1             | `0`   | Reserved for future use.                                      |
| Single bank    | 1             | 1             | `0`   | The target device should store this transfer without banking. |
| First transfer | 2             | 1             | `1`   | This transfer is the first in a set.                          |
| Last transfer  | 3             | 1             | `1`   | This transfer is the last in a set.                            |
| RFU            | 4             | 4             | `0`   | Reserved for future use.                                      |

### DFU data @anchor dfu-packet-data

The DFU data packet contains all firmware and signature data for the transfer. Each data packet
contains a single 16-byte data segment, except for the last segment of the firmware data, which may
be shorter.

The DFU data packets must be sent in strictly increasing, continuous segment indexes, starting
at segment index 1. Each segment must contain firmware data from byte
`(Segment index - 1) * 16` to `(Segment - 1 index) * 16 + 15` of the firmware. Once all firmware
data has been sent in segments `1` to `N`, any signature data must be transmitted in segment
`N + 1` to `M` until all signature data has been sent. If the firmware length is not aligned
to 16-byte boundaries, the segment `N` must be shorted down to only contain the rest of the firmware.
The signature data always starts in segment `N + 1`, even if there is space left in segment `N`.
If the signature length is `0`, the segment `N` is the last segment of the transfer.

| Field               | Offset (bytes) | Length (bytes) | Value                                                       |
|---------------------|----------------|----------------|-------------------------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFC`                                                    |
| Segment index       | 2              | 2              | The segment index of this data packet.                      |
| Transfer ID         | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id). |
| Data segment        | 8              | 1-16           | Segment data.                                               |

### DFU data request @anchor dfu-packet-data-request

The DFU data request packets are sent by target devices to request missing segments from the transfer
source device. Together with the DFU data response packets, they provide a recovery mechanism for
single packets that the device can use to combat packet loss.

When a target device discovers that it has gaps in the transfer data, it must start
broadcasting DFU data request packets at regular intervals for the oldest missing segment.
If a device receives a data request packet for a data segment it already knows, it responds with
a data response, regardless of its role in the transfer. When a device receives the missing data
it is requesting, either through a DFU data response or through a regular data packet,
it stops requesting that segment.

| Field               | Offset (bytes) | Length (bytes) | Value                                                       |
|---------------------|----------------|----------------|-------------------------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFB`                                                    |
| Segment index       | 2              | 2              | The segment index of the missing data packet.               |
| Transfer ID         | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id). |

### DFU data response @anchor dfu-packet-data-response

The DFU data response packet is identical to the DFU data packet, except for its packet type.
It will be sent as a respose to DFU data request packets if the device has the contents of the
requested segment stored locally, regardless of whether the device is acting as a target, source,
or relay for the transfer.

| Field               | Offset (bytes) | Length (bytes) | Value                                                       |
|---------------------|----------------|----------------|-------------------------------------------------------------|
| Packet type         | 0              | 2              | `0xFFFA`                                                    |
| Segment index       | 2              | 2              | The segment index of this data packet.                      |
| Transfer ID         | 4              | 4              | Transfer identifier. See [Transfer ID field](@ref dfu-packet-state-transfer-id). |
| Data segment        | 8              | 1-16           | Segment data.                                               |

