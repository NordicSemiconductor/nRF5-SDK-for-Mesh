
# Serial events

# Serial event overview {#serial-events}

Serial Events are messages sent from the nRF5 to the host controller. Messages
are sent in one of the following ways:
- asynchronously as a result of some interaction in the mesh;
- as a response to a command.

See the following table for an overview of the various events pushed by the nRF5
to the host. Each entry links to the details section, where the
parameters and reason for each event are described.


Event                                                                    | Opcode
-------------------------------------------------------------------------|-------
[Cmd Rsp](#cmd-rsp)                                                      | 0x84
[Device Started](#device-started)                                        | 0x81
[Device Echo Rsp](#device-echo-rsp)                                      | 0x82
[Device Internal Event](#device-internal-event)                          | 0x83
[Application](#application)                                              | 0x8a
[SAR Start](#sar-start)                                                  | 0x8b
[SAR Continue](#sar-continue)                                            | 0x8c
[DFU Req Relay](#dfu-req-relay)                                          | 0xa0
[DFU Req Source](#dfu-req-source)                                        | 0xa1
[DFU Start](#dfu-start)                                                  | 0xa2
[DFU End](#dfu-end)                                                      | 0xa3
[DFU Bank Available](#dfu-bank-available)                                | 0xa4
[DFU Firmware Outdated](#dfu-firmware-outdated)                          | 0xa5
[DFU Firmware Outdated No Auth](#dfu-firmware-outdated-no-auth)          | 0xa6
[Openmesh New](#openmesh-new)                                            | 0xb3
[Openmesh Update](#openmesh-update)                                      | 0xb4
[Openmesh Conflicting](#openmesh-conflicting)                            | 0xb5
[Openmesh TX](#openmesh-tx)                                              | 0xb6
[Prov Unprovisioned Received](#prov-unprovisioned-received)              | 0xc0
[Prov Link Established](#prov-link-established)                          | 0xc1
[Prov Link Closed](#prov-link-closed)                                    | 0xc2
[Prov Caps Received](#prov-caps-received)                                | 0xc3
[Prov Invite Received](#prov-invite-received)                            | 0xc4
[Prov Start Received](#prov-start-received)                              | 0xca
[Prov Complete](#prov-complete)                                          | 0xc5
[Prov Auth Request](#prov-auth-request)                                  | 0xc6
[Prov ECDH Request](#prov-ecdh-request)                                  | 0xc7
[Prov Output Request](#prov-output-request)                              | 0xc8
[Prov Failed](#prov-failed)                                              | 0xc9
[Mesh Message Received Unicast](#mesh-message-received-unicast)          | 0xd0
[Mesh Message Received Subscription](#mesh-message-received-subscription)| 0xd1
[Mesh TX Complete](#mesh-tx-complete)                                    | 0xd2
[Mesh IV Update Notification](#mesh-iv-update-notification)              | 0xd3
[Mesh Key Refresh Notification](#mesh-key-refresh-notification)          | 0xd4
[Mesh Heartbeat Received](#mesh-heartbeat-received)                      | 0xd8
[Mesh IV Entry Set Notification](#mesh-iv-entry-set-notification)        | 0xd9
[Mesh Seqnum Entry Set Notification](#mesh-seqnum-entry-set-notification)| 0xda
[Mesh SAR Failed](#mesh-sar-failed)                                      | 0xd7
[Model Specific](#model-specific)                                        | 0xf0

---

## Serial event details {#serial-event-details}

This subsection contains detailed description of each serial event, including opcodes, total length,
description, and parameters (if any exist).

### Cmd Rsp          {#cmd-rsp}

_Opcode:_ `0x84`

_Total length:_ 3..255 bytes

Command response. Each command (except the Echo command) immediately gets a command response. See the individual commands for their responses.

_Cmd Rsp Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Opcode                                  | 1     | 0      | Opcode of original command.
`uint8_t`         | Status                                  | 1     | 1      | Return status of the serial command.
`uint8_t[252]`    | Data                                    | 0..252 | 2      | Optional command response data.

---
### Device Started          {#device-started}

_Opcode:_ `0x81`

_Total length:_ 4 bytes

The device has started, and is ready for commands. No commands will be accepted before this event, and it is guaranteed to be the first event to cross the serial.

_Device Started Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Operating Mode                          | 1     | 0      | Operating mode of the device. see @ref serial_device_operating_mode_t for accepted values.
`uint8_t`         | Hw Error                                | 1     | 1      | Hardware error code, or 0 if no error occurred.
`uint8_t`         | Data Credit Available                   | 1     | 2      | The number of bytes available in each of the tx and rx buffers.

---
### Device Echo Rsp          {#device-echo-rsp}

_Opcode:_ `0x82`

_Total length:_ 1..255 bytes

Response to the Echo command. Contains the exact same data as received in the echo command.

_Device Echo Rsp Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t[254]`    | Data                                    | 0..254 | 0      | Data received in the echo command.

---
### Device Internal Event          {#device-internal-event}

_Opcode:_ `0x83`

_Total length:_ 35 bytes

Internal stack event occurred.

_Device Internal Event Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Event Type                              | 1     | 0      | Reported event. See @ref internal_event_type_t for accepted values.
`uint8_t`         | State                                   | 1     | 1      | State information about the event type reported.
`uint8_t`         | Packet Size                             | 1     | 2      | Size (in bytes) of the packet.
`uint8_t[31]`     | Packet                                  | 31    | 3      | Event data.

---
### Application          {#application}

_Opcode:_ `0x8a`

_Total length:_ 1..255 bytes

Application event, only sent by the device application.

_Application Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t[254]`    | Data                                    | 0..254 | 0      | Application data.

---
### SAR Start          {#sar-start}

_Opcode:_ `0x8b`

_Total length:_ 1 byte

Start of a Segmentation and Reassembly message from the device.

_SAR Start has no parameters._


---
### SAR Continue          {#sar-continue}

_Opcode:_ `0x8c`

_Total length:_ 1 byte

Continuation of a Segmentation and Reassembly message from the device.

_SAR Continue has no parameters._


---
### DFU Req Relay          {#dfu-req-relay}

_Opcode:_ `0xa0`

_Total length:_ 13 bytes

Received a request from another device to act as a relay in a DFU transaction.

_DFU Req Relay Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | DFU Type                                | 1     | 0      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | FWID                                    | 10    | 1      | Firmware ID of the requested transfer.
`uint8_t`         | Authority                               | 1     | 11     | Authority level of the transfer.

---
### DFU Req Source          {#dfu-req-source}

_Opcode:_ `0xa1`

_Total length:_ 2 bytes

Recevied a request from another device to act as a source in a DFU transaction.

_DFU Req Source Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | DFU Type                                | 1     | 0      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.

---
### DFU Start          {#dfu-start}

_Opcode:_ `0xa2`

_Total length:_ 13 bytes

The current DFU operation started its data transfer stage.

_DFU Start Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Role                                    | 1     | 0      | The device's role in the transfer. See @ref nrf_mesh_dfu_role_t.
`uint8_t`         | DFU Type                                | 1     | 1      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | FWID                                    | 10    | 2      | Firmware ID of the transfer.

---
### DFU End          {#dfu-end}

_Opcode:_ `0xa3`

_Total length:_ 14 bytes

The current DFU operation ended its data transfer stage.

_DFU End Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Role                                    | 1     | 0      | The device's role in the transfer. See @ref nrf_mesh_dfu_role_t.
`uint8_t`         | DFU Type                                | 1     | 1      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | FWID                                    | 10    | 2      | Firmware ID of the transfer.
`uint8_t`         | End Reason                              | 1     | 12     | Reason for ending the transfer. See @ref nrf_mesh_dfu_end_t.

---
### DFU Bank Available          {#dfu-bank-available}

_Opcode:_ `0xa4`

_Total length:_ 21 bytes

A DFU firmware bank is available for flashing.

_DFU Bank Available Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | DFU Type                                | 1     | 0      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | FWID                                    | 10    | 1      | Firmware ID of the transfer.
`uint32_t`        | Start Addr                              | 4     | 11     | Start address of the bank.
`uint32_t`        | Length                                  | 4     | 15     | Length of the banked firmware.
`uint8_t`         | Is Signed                               | 1     | 19     | Whether the bank is signed or not.

---
### DFU Firmware Outdated          {#dfu-firmware-outdated}

_Opcode:_ `0xa5`

_Total length:_ 22 bytes

The mesh has received a secure notification indicating that the framework is out of date. Call _DFU Request_ to initiate a request to receive the firmware upgrade.

_DFU Firmware Outdated Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | DFU Type                                | 1     | 0      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | Available FWID                          | 10    | 1      | Firmware ID of the newest firmware available.
`nrf_mesh_fwid_t` | Current FWID                            | 10    | 11     | Firmware ID of the current version of the outdated firmware.

---
### DFU Firmware Outdated No Auth          {#dfu-firmware-outdated-no-auth}

_Opcode:_ `0xa6`

_Total length:_ 22 bytes

The mesh has received an insecure notification indicating that the framework is out of date. Call _Direct Firmware Upgrade Request_ to initiate a request to receive the firmware upgrade.

_DFU Firmware Outdated No Auth Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | DFU Type                                | 1     | 0      | DFU type of the transfer. See @ref nrf_mesh_dfu_type_t.
`nrf_mesh_fwid_t` | Available FWID                          | 10    | 1      | Firmware ID of the newest firmware available.
`nrf_mesh_fwid_t` | Current FWID                            | 10    | 11     | Firmware ID of the current version of the outdated firmware.

---
### Openmesh New          {#openmesh-new}

_Opcode:_ `0xb3`

_Total length:_ 1 byte

Not implemented.

_Openmesh New has no parameters._


---
### Openmesh Update          {#openmesh-update}

_Opcode:_ `0xb4`

_Total length:_ 1 byte

Not implemented.

_Openmesh Update has no parameters._


---
### Openmesh Conflicting          {#openmesh-conflicting}

_Opcode:_ `0xb5`

_Total length:_ 1 byte

Not implemented.

_Openmesh Conflicting has no parameters._


---
### Openmesh TX          {#openmesh-tx}

_Opcode:_ `0xb6`

_Total length:_ 1 byte

Not implemented.

_Openmesh TX has no parameters._


---
### Prov Unprovisioned Received          {#prov-unprovisioned-received}

_Opcode:_ `0xc0`

_Total length:_ 26 bytes

The node received an unprovisioned beacon. Requires scanning to be enabled with the _Provisioning Scan Enable_ command.

_Prov Unprovisioned Received Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t[16]`     | UUID                                    | 16    | 0      | UUID in the unprovisioned beacon.
`int8_t`          | Rssi                                    | 1     | 16     | RSSI of the received unprovisioned beacon.
`uint8_t`         | Gatt Supported                          | 1     | 17     | Whether the unprovisioned device supports GATT provisioning.
`uint8_t`         | Adv Addr Type                           | 1     | 18     | The advertisement address type of the sender of the unprovisioned beacon.
`uint8_t[6]`      | Adv Addr                                | 6     | 19     | The advertisement address of the sender of the unprovisioned beacon.

---
### Prov Link Established          {#prov-link-established}

_Opcode:_ `0xc1`

_Total length:_ 2 bytes

The given provisioning link has been established.

_Prov Link Established Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the established link.

---
### Prov Link Closed          {#prov-link-closed}

_Opcode:_ `0xc2`

_Total length:_ 3 bytes

The given provisioning link has been closed. If received before a _Provisioning Complete_ event, the link was closed because of an error.

_Prov Link Closed Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the closed link.
`uint8_t`         | Close Reason                            | 1     | 1      | Reason for closing the link.

---
### Prov Caps Received          {#prov-caps-received}

_Opcode:_ `0xc3`

_Total length:_ 11 bytes

The device received provisioning capabilities on the provisioning link with the given context ID.

_Prov Caps Received Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the link the capabilities were received on.
`uint8_t`         | Num Elements                            | 1     | 1      | The number of elements on the unprovisoined device.
`uint8_t`         | Public Key Type                         | 1     | 2      | The public key type used for the provisioning session.
`uint8_t`         | Static OOB Types                        | 1     | 3      | The available static OOB authentication methods.
`uint8_t`         | Output OOB Size                         | 1     | 4      | Maximum size of the output OOB supported.
`uint16_t`        | Output OOB Actions                      | 2     | 5      | Available OOB output actions.
`uint8_t`         | Input OOB Size                          | 1     | 7      | Maximum size of the input OOB supported.
`uint16_t`        | Input OOB Actions                       | 2     | 8      | Available OOB input actions.

---
### Prov Invite Received          {#prov-invite-received}

_Opcode:_ `0xc4`

_Total length:_ 3 bytes

The device received provisioning invite, so it can start identifying itself using any means it can.

_Prov Invite Received Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the provisioning link.
`uint8_t`         | Attention Duration S                    | 1     | 1      | Time in seconds during which the device will identify itself using any means it can.

---
### Prov Start Received          {#prov-start-received}

_Opcode:_ `0xca`

_Total length:_ 2 bytes

The device received provisioning start meaning that the provisioning process has been started and will stop identifying itself.

_Prov Start Received Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the provisioning link.

---
### Prov Complete          {#prov-complete}

_Opcode:_ `0xc5`

_Total length:_ 44 bytes

The provisioning process was successfully completed.

_Prov Complete Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the completed provisioning link.
`uint32_t`        | Iv Index                                | 4     | 1      | IV index for the network.
`uint16_t`        | Net Key Index                           | 2     | 5      | Network key index.
`uint16_t`        | Address                                 | 2     | 7      | Unicast address for the device.
`uint8_t`         | Iv Update Flag                          | 1     | 9      | IV update in progress flag.
`uint8_t`         | Key Refresh Flag                        | 1     | 10     | Key refresh in progress flag.
`uint8_t[16]`     | Device Key                              | 16    | 11     | The device key of the provisioned device.
`uint8_t[16]`     | Net Key                                 | 16    | 27     | The network key of the provisioned device.

---
### Prov Auth Request          {#prov-auth-request}

_Opcode:_ `0xc6`

_Total length:_ 5 bytes

Static authentication data is required to continue. Use the _Provisioning AuthData_ command to respond to this event.

_Prov Auth Request Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the link the authorization request appeared on.
`uint8_t`         | Method                                  | 1     | 1      | Method of authentication requested.
`uint8_t`         | Action                                  | 1     | 2      | Authentication action.
`uint8_t`         | Size                                    | 1     | 3      | Authentication size.

---
### Prov ECDH Request          {#prov-ecdh-request}

_Opcode:_ `0xc7`

_Total length:_ 98 bytes

An ECDH shared secret must be calculated. Use the _Provisioning ECDH Secret_ command to respond to this event.

_Prov ECDH Request Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the link the ECDH request appeared on.
`uint8_t[64]`     | Peer Public                             | 64    | 1      | ECDH public key.
`uint8_t[32]`     | Node Private                            | 32    | 65     | ECDH private key.

---
### Prov Output Request          {#prov-output-request}

_Opcode:_ `0xc8`

_Total length:_ 3..19 bytes

The device is required to do an action the user can recognize and use for authentication.

_Prov Output Request Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the link the output request appeared on.
`uint8_t`         | Output Action                           | 1     | 1      | Output action requested.
`uint8_t[16]`     | Data                                    | 0..16 | 2      | Data for the output request.

---
### Prov Failed          {#prov-failed}

_Opcode:_ `0xc9`

_Total length:_ 3 bytes

The provisioning procedure failed.

_Prov Failed Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Context ID                              | 1     | 0      | Context ID of the link the error happened on.
`uint8_t`         | Error Code                              | 1     | 1      | Provisioning error code.

---
### Mesh Message Received Unicast          {#mesh-message-received-unicast}

_Opcode:_ `0xd0`

_Total length:_ 20..255 bytes

The mesh framework received a message matching a registered local unicast address, with the given parameters and data.

_Mesh Message Received Unicast Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint16_t`        | SRC                                     | 2     | 0      | Source address of the received packet.
`uint16_t`        | DST                                     | 2     | 2      | Destination unicast address or subscription handle.
`uint16_t`        | Appkey Handle                           | 2     | 4      | Handle of the application the message was received on.
`uint16_t`        | Subnet Handle                           | 2     | 6      | Handle of the subnetwork the message was received on.
`uint8_t`         | TTL                                     | 1     | 8      | Packet time to live value when first received.
`uint8_t`         | Adv Addr Type                           | 1     | 9      | Advertisement address type of the last hop sender.
`uint8_t[6]`      | Adv Addr                                | 6     | 10     | Advertisement address of the last hop sender.
`int8_t`          | Rssi                                    | 1     | 16     | RSSI value of the message when received.
`uint16_t`        | Actual Length                           | 2     | 17     | Length of the received message, may be larger than the data reported if @ref SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN is not big enough.
`uint8_t[235]`    | Data                                    | 0..235 | 19     | Data payload of the packet.

---
### Mesh Message Received Subscription          {#mesh-message-received-subscription}

_Opcode:_ `0xd1`

_Total length:_ 20..255 bytes

The mesh framework received a message matching one of the registered subscription addresses, with the given parameters and data.

_Mesh Message Received Subscription Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint16_t`        | SRC                                     | 2     | 0      | Source address of the received packet.
`uint16_t`        | DST                                     | 2     | 2      | Destination unicast address or subscription handle.
`uint16_t`        | Appkey Handle                           | 2     | 4      | Handle of the application the message was received on.
`uint16_t`        | Subnet Handle                           | 2     | 6      | Handle of the subnetwork the message was received on.
`uint8_t`         | TTL                                     | 1     | 8      | Packet time to live value when first received.
`uint8_t`         | Adv Addr Type                           | 1     | 9      | Advertisement address type of the last hop sender.
`uint8_t[6]`      | Adv Addr                                | 6     | 10     | Advertisement address of the last hop sender.
`int8_t`          | Rssi                                    | 1     | 16     | RSSI value of the message when received.
`uint16_t`        | Actual Length                           | 2     | 17     | Length of the received message, may be larger than the data reported if @ref SERIAL_EVT_MESH_MESSAGE_RECEIVED_DATA_MAXLEN is not big enough.
`uint8_t[235]`    | Data                                    | 0..235 | 19     | Data payload of the packet.

---
### Mesh TX Complete          {#mesh-tx-complete}

_Opcode:_ `0xd2`

_Total length:_ 5 bytes

A radio packet TX has completed.

_Mesh TX Complete Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`nrf_mesh_tx_token_t` | Token                                   | 4     | 0      | TX token for the completed packet.

---
### Mesh IV Update Notification          {#mesh-iv-update-notification}

_Opcode:_ `0xd3`

_Total length:_ 5 bytes

The IV update procedure has been triggered for the network with the given index.

_Mesh IV Update Notification Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint32_t`        | Iv Index                                | 4     | 0      | IV index updated to.

---
### Mesh Key Refresh Notification          {#mesh-key-refresh-notification}

_Opcode:_ `0xd4`

_Total length:_ 4 bytes

A network has entered a new phase in the key refresh procedure.

_Mesh Key Refresh Notification Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint16_t`        | Netkey Index                            | 2     | 0      | Network key index of the network key being updated.
`uint8_t`         | Phase                                   | 1     | 2      | Current key refresh phase for the network key being updated.

---
### Mesh Heartbeat Received          {#mesh-heartbeat-received}

_Opcode:_ `0xd8`

_Total length:_ 7 bytes

A heartbeat message has been received. To control the heartbeat subscriptions of the device through the local Configuration Client, add any mesh device key through _Device Key_ and bind the provided handle to the device Configuration Server through _Config Server Bind_. Use the provided device key handle to configure the device.

_Mesh Heartbeat Received Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint8_t`         | Init TTL                                | 1     | 0      | Initial TTL value used for sending this heartbeat message.
`uint8_t`         | Hops                                    | 1     | 1      | Number of hops equals: (Initial TTL - Received message TTL + 1).
`uint16_t`        | Features                                | 2     | 2      | State bitmap of the feature. See @ref MESH_DEFINES_HEARTBEAT to interpret bit fields.
`uint16_t`        | SRC                                     | 2     | 4      | Source address for the received heartbeat message.

---
### Mesh IV Entry Set Notification          {#mesh-iv-entry-set-notification}

_Opcode:_ `0xd9`

_Total length:_ 8 bytes

The mesh config entry for the IV index has been set.

_Mesh IV Entry Set Notification Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint32_t`        | Iv Index                                | 4     | 0      | The current IV index.
`uint8_t`         | Iv Update In Progress                   | 1     | 4      | Indicating the phase in the IV update process.
`uint16_t`        | Iv Update Timout Counter                | 2     | 5      | Counter for the IV update process.

---
### Mesh Seqnum Entry Set Notification          {#mesh-seqnum-entry-set-notification}

_Opcode:_ `0xda`

_Total length:_ 5 bytes

The mesh config entry for the next sequence number block has been set.

_Mesh Seqnum Entry Set Notification Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`uint32_t`        | Next Block                              | 4     | 0      | The next unallocated sequence number block.

---
### Mesh SAR Failed          {#mesh-sar-failed}

_Opcode:_ `0xd7`

_Total length:_ 1 byte

A Mesh transmission of a SAR packet failed.

_Mesh SAR Failed has no parameters._


---
### Model Specific          {#model-specific}

_Opcode:_ `0xf0`

_Total length:_ 6..255 bytes

An event generated by one of the initialized models. Model ID and event type is provided by each event. Further model specific information is provided as part of the data field.

_Model Specific Parameters:_

Type              | Name                                    | Size  | Offset | Description
------------------|-----------------------------------------|-------|--------|------------
`serial_evt_model_specific_header_t` | Model Evt Info                          | 5     | 0      | Contains the model id the event generates from and the model specific event type.
`uint8_t[249]`    | Data                                    | 0..249 | 5      | Additional data provided by the event

---
