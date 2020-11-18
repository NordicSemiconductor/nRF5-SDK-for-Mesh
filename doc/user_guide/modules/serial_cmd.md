
# Serial commands

@ingroup LIBRARIES
Serial commands are messages sent from the host controller to the nRF5. Most
serial commands result in a _CMD RSP_ event, indicating whether the command was
successful and returning any relevant data depending on the command type.

# Serial command groups {#serial-commands}

The serial commands are broken into the following groups:

- [Device](#device-commands): Hardware commands for device operation.

- [Application](#application-commands): Single opcode made available to the application.

- [Segmentation and Reassembly](#segmentation-and-reassembly-commands): Segmentation and reassembly of serial packets. Used to allow packets larger than the largest serial packet size.

- [Configuration](#configuration-commands): Configuration of various device parameters, like addresses and radio properties.

- [Provisioning](#provisioning-commands): Provisioning-specific commands and operations.

- [nRF Open Mesh](#nrf-open-mesh-commands): Set of commands used by the nRF Open Mesh.

- [Bluetooth Mesh](#bluetooth-mesh-commands): Bluetooth Mesh commands for controlling the behavior of a device on the mesh.

- [Direct Firmware Upgrade](#direct-firmware-upgrade-commands): Commands controlling the behavior of the Device Firmware Update part of the mesh stack.

- [Access Layer](#access-layer-commands): Commands to interface the access layer on mesh.

- [Model Specific](#model-specific-commands): Commands for initializing and commanding specific models.



See the tables in the following subsections for lists of serial commands available for each command
group in the nRF5 mesh serial interface, along with their opcodes. Each entry links
to the respective details section, where the parameters and effects of each
command are described.

---


## Device commands {#device-commands}

Command                                 | Opcode
----------------------------------------|-------
[Echo](#device-echo)                             | `0x02`
[Internal Events Report](#device-internal-events-report)           | `0x03`
[Serial Version Get](#device-serial-version-get)               | `0x09`
[FW Info Get](#device-fw-info-get)                      | `0x0a`
[Radio Reset](#device-radio-reset)                      | `0x0e`
[Beacon Start](#device-beacon-start)                     | `0x10`
[Beacon Stop](#device-beacon-stop)                      | `0x11`
[Beacon Params Get](#device-beacon-params-get)                | `0x13`
[Beacon Params Set](#device-beacon-params-set)                | `0x12`
[Housekeeping Data Get](#device-housekeeping-data-get)            | `0x14`
[Housekeeping Data Clear](#device-housekeeping-data-clear)          | `0x15`

---

## Application commands {#application-commands}

Command                                 | Opcode
----------------------------------------|-------
[Application](#application-application)                 | `0x20`

---

## Segmentation and Reassembly commands {#segmentation-and-reassembly-commands}

Command                                 | Opcode
----------------------------------------|-------
[Start](#segmentation-and-reassembly-start)       | `0x21`
[Continue](#segmentation-and-reassembly-continue)    | `0x22`

---

## Configuration commands {#configuration-commands}

Command                                 | Opcode
----------------------------------------|-------
[Adv Addr Set](#configuration-adv-addr-set)              | `0x40`
[Adv Addr Get](#configuration-adv-addr-get)              | `0x41`
[Channel Map Set](#configuration-channel-map-set)           | `0x42`
[Channel Map Get](#configuration-channel-map-get)           | `0x43`
[TX Power Set](#configuration-tx-power-set)              | `0x44`
[TX Power Get](#configuration-tx-power-get)              | `0x45`
[UUID Set](#configuration-uuid-set)                  | `0x53`
[UUID Get](#configuration-uuid-get)                  | `0x54`

---

## Provisioning commands {#provisioning-commands}

Command                                 | Opcode
----------------------------------------|-------
[Scan Start](#provisioning-scan-start)                 | `0x61`
[Scan Stop](#provisioning-scan-stop)                  | `0x62`
[Provision](#provisioning-provision)                  | `0x63`
[Listen](#provisioning-listen)                     | `0x64`
[OOB Use](#provisioning-oob-use)                    | `0x66`
[Auth Data](#provisioning-auth-data)                  | `0x67`
[ECDH Secret](#provisioning-ecdh-secret)                | `0x68`
[Keypair Set](#provisioning-keypair-set)                | `0x69`
[Capabilities Set](#provisioning-capabilities-set)           | `0x6a`

---

## nRF Open Mesh commands {#nrf-open-mesh-commands}

Command                                 | Opcode
----------------------------------------|-------
[Init](#nrf-open-mesh-init)                      | `0x70`
[Value Set](#nrf-open-mesh-value-set)                 | `0x71`
[Value Enable](#nrf-open-mesh-value-enable)              | `0x72`
[Value Disable](#nrf-open-mesh-value-disable)             | `0x73`
[Start](#nrf-open-mesh-start)                     | `0x74`
[Stop](#nrf-open-mesh-stop)                      | `0x75`
[Flag Set](#nrf-open-mesh-flag-set)                  | `0x76`
[Flag Get](#nrf-open-mesh-flag-get)                  | `0x77`
[DFU Data](#nrf-open-mesh-dfu-data)                  | `0x78`
[Value Get](#nrf-open-mesh-value-get)                 | `0x7a`
[Build Version Get](#nrf-open-mesh-build-version-get)         | `0x7b`
[Access Addr Get](#nrf-open-mesh-access-addr-get)           | `0x7c`
[Channel Get](#nrf-open-mesh-channel-get)               | `0x7d`
[Interval Min ms Get](#nrf-open-mesh-interval-min-ms-get)       | `0x7f`

---

## Bluetooth Mesh commands {#bluetooth-mesh-commands}

Command                                 | Opcode
----------------------------------------|-------
[Enable](#bluetooth-mesh-enable)                   | `0x90`
[Disable](#bluetooth-mesh-disable)                  | `0x91`
[Subnet Add](#bluetooth-mesh-subnet-add)               | `0x92`
[Subnet Update](#bluetooth-mesh-subnet-update)            | `0x93`
[Subnet Delete](#bluetooth-mesh-subnet-delete)            | `0x94`
[Subnet Get All](#bluetooth-mesh-subnet-get-all)           | `0x95`
[Subnet Count Max Get](#bluetooth-mesh-subnet-count-max-get)     | `0x96`
[Appkey Add](#bluetooth-mesh-appkey-add)               | `0x97`
[Appkey Update](#bluetooth-mesh-appkey-update)            | `0x98`
[Appkey Delete](#bluetooth-mesh-appkey-delete)            | `0x99`
[Appkey Get All](#bluetooth-mesh-appkey-get-all)           | `0x9a`
[Appkey Count Max Get](#bluetooth-mesh-appkey-count-max-get)     | `0x9b`
[Devkey Add](#bluetooth-mesh-devkey-add)               | `0x9c`
[Devkey Delete](#bluetooth-mesh-devkey-delete)            | `0x9d`
[Devkey Count Max Get](#bluetooth-mesh-devkey-count-max-get)     | `0x9e`
[Addr Local Unicast Set](#bluetooth-mesh-addr-local-unicast-set)   | `0x9f`
[Addr Local Unicast Get](#bluetooth-mesh-addr-local-unicast-get)   | `0xa0`
[Addr Get](#bluetooth-mesh-addr-get)                 | `0xa7`
[Addr Get All](#bluetooth-mesh-addr-get-all)             | `0xa8`
[Addr Nonvirtual Count Max Get](#bluetooth-mesh-addr-nonvirtual-count-max-get)| `0xa9`
[Addr Virtual Count Max Get](#bluetooth-mesh-addr-virtual-count-max-get)| `0xaa`
[Addr Subscription Add](#bluetooth-mesh-addr-subscription-add)    | `0xa1`
[Addr Subscription Add Virtual](#bluetooth-mesh-addr-subscription-add-virtual)| `0xa2`
[Addr Subscription Remove](#bluetooth-mesh-addr-subscription-remove) | `0xa3`
[Addr Publication Add](#bluetooth-mesh-addr-publication-add)     | `0xa4`
[Addr Publication Add Virtual](#bluetooth-mesh-addr-publication-add-virtual)| `0xa5`
[Addr Publication Remove](#bluetooth-mesh-addr-publication-remove)  | `0xa6`
[Packet Send](#bluetooth-mesh-packet-send)              | `0xab`
[State Clear](#bluetooth-mesh-state-clear)              | `0xac`
[Config Server Bind](#bluetooth-mesh-config-server-bind)       | `0xad`
[Net State Set](#bluetooth-mesh-net-state-set)            | `0xae`
[Net State Get](#bluetooth-mesh-net-state-get)            | `0xaf`

---

## Direct Firmware Upgrade commands {#direct-firmware-upgrade-commands}

Command                                 | Opcode
----------------------------------------|-------
[Jump To Bootloader](#direct-firmware-upgrade-jump-to-bootloader)| `0xd0`
[Request](#direct-firmware-upgrade-request)         | `0xd1`
[Relay](#direct-firmware-upgrade-relay)           | `0xd2`
[Abort](#direct-firmware-upgrade-abort)           | `0xd3`
[Bank Info Get](#direct-firmware-upgrade-bank-info-get)   | `0xd4`
[Bank Flash](#direct-firmware-upgrade-bank-flash)      | `0xd5`
[State Get](#direct-firmware-upgrade-state-get)       | `0xd6`

---

## Access Layer commands {#access-layer-commands}

Command                                 | Opcode
----------------------------------------|-------
[Model Pub Addr Set](#access-layer-model-pub-addr-set)         | `0xe0`
[Model Pub Addr Get](#access-layer-model-pub-addr-get)         | `0xe1`
[Model Pub Period Set](#access-layer-model-pub-period-set)       | `0xe2`
[Model Pub Period Get](#access-layer-model-pub-period-get)       | `0xe3`
[Model Subs Add](#access-layer-model-subs-add)             | `0xe4`
[Model Subs Remove](#access-layer-model-subs-remove)          | `0xe5`
[Model Subs Get](#access-layer-model-subs-get)             | `0xe6`
[Model App Bind](#access-layer-model-app-bind)             | `0xe7`
[Model App Unbind](#access-layer-model-app-unbind)           | `0xe8`
[Model App Get](#access-layer-model-app-get)              | `0xe9`
[Model Pub App Set](#access-layer-model-pub-app-set)          | `0xea`
[Model Pub App Get](#access-layer-model-pub-app-get)          | `0xeb`
[Model Pub TTL Set](#access-layer-model-pub-ttl-set)          | `0xec`
[Model Pub TTL Get](#access-layer-model-pub-ttl-get)          | `0xed`
[Elem Loc Set](#access-layer-elem-loc-set)               | `0xee`
[Elem Loc Get](#access-layer-elem-loc-get)               | `0xef`
[Elem Sig Model Count Get](#access-layer-elem-sig-model-count-get)   | `0xf0`
[Elem Vendor Model Count Get](#access-layer-elem-vendor-model-count-get)| `0xf1`
[Model ID Get](#access-layer-model-id-get)               | `0xf2`
[Handle Get](#access-layer-handle-get)                 | `0xf3`
[Elem Models Get](#access-layer-elem-models-get)            | `0xf4`

---

## Model Specific commands {#model-specific-commands}

Command                                 | Opcode
----------------------------------------|-------
[Models Get](#model-specific-models-get)               | `0xfc`
[Init](#model-specific-init)                     | `0xfd`
[Command](#model-specific-command)                  | `0xfe`

---

## Serial command details {#serial-command-details}

This subsection contains detailed description of each serial command, including opcodes, total length,
description, and parameters (if any are taken).

The commands are listed in order corresponding to groups. The group name precedes the name of the command
in each case.

---
### Device Echo {#device-echo}

_Opcode:_ `0x02`

_Total length:_ 1..255 bytes

A simple loopback test command. Used to verify that the serial transport layer is working as intended.

_Echo Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[254]` | Data                                    | 0..254 | 0      | Data to echo back.

#### Response

Potential status codes:

- `INVALID_LENGTH`

_Echo Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[254]` | Data                                    | 0..254 | 0      | Data received in the echo command.


---
### Device Internal Events Report {#device-internal-events-report}

_Opcode:_ `0x03`

_Total length:_ 1 byte

Start reporting internal events, if supported.

_Internal Events Report takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Device Serial Version Get {#device-serial-version-get}

_Opcode:_ `0x09`

_Total length:_ 1 byte

Request the implemented version of the serial interface.

_Serial Version Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Serial Version Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Serial Ver                              | 2    | 0      | Serial interface version.


---
### Device FW Info Get {#device-fw-info-get}

_Opcode:_ `0x0a`

_Total length:_ 1 byte

Request the firmware version info structure for the firmware.

_FW Info Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_FW Info Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`nrf_mesh_fwid_t` | FWID                                    | 10   | 0      | Firmware ID data.


---
### Device Radio Reset {#device-radio-reset}

_Opcode:_ `0x0e`

_Total length:_ 1 byte

Restart the device.

_Radio Reset takes no parameters._

#### Response

_This command does not yield any response._

---
### Device Beacon Start {#device-beacon-start}

_Opcode:_ `0x10`

_Total length:_ 2..33 bytes

Start an application controlled beacon with the given payload. Will hotswap the payload if the beacon is already running.

_Beacon Start Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to set the payload for.
`uint8_t[31]` | Data                                    | 0..31 | 1      | Beacon payload.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `ERROR_BUSY`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Device Beacon Stop {#device-beacon-stop}

_Opcode:_ `0x11`

_Total length:_ 2 bytes

Stop transmitting the current beacon.

_Beacon Stop Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to stop.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Device Beacon Params Get {#device-beacon-params-get}

_Opcode:_ `0x13`

_Total length:_ 2 bytes

Set parameters for application controlled beacon.

_Beacon Params Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to get the parameters of.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_Beacon Params Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to start.
`uint8_t`     | TX Power                                | 1    | 1      | TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.
`uint8_t`     | Channel Map                             | 1    | 2      | Channel map bitfield for beacon, starting at channel 37.
`uint32_t`    | Interval ms                             | 4    | 3      | TX interval in milliseconds.


---
### Device Beacon Params Set {#device-beacon-params-set}

_Opcode:_ `0x12`

_Total length:_ 8 bytes

Set parameters for application controlled beacon.

_Beacon Params Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Beacon Slot                             | 1    | 0      | Slot number of the beacon to start.
`uint8_t`     | TX Power                                | 1    | 1      | TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.
`uint8_t`     | Channel Map                             | 1    | 2      | Channel map bitfield for beacon, starting at channel 37.
`uint32_t`    | Interval ms                             | 4    | 3      | TX interval in milliseconds.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Device Housekeeping Data Get {#device-housekeeping-data-get}

_Opcode:_ `0x14`

_Total length:_ 1 byte

Get the current housekeeping data values.

_Housekeeping Data Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Housekeeping Data Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint32_t`    | Alloc Fail Count                        | 4    | 0      | Number of failed serial packet allocations.


---
### Device Housekeeping Data Clear {#device-housekeeping-data-clear}

_Opcode:_ `0x15`

_Total length:_ 1 byte

Clear the current housekeeping data values.

_Housekeeping Data Clear takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Application Application {#application-application}

_Opcode:_ `0x20`

_Total length:_ 1..255 bytes

Application-specific command. Has no functionality in the framework, but is forwarded to the application.

_Application Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[254]` | Data                                    | 0..254 | 0      | Application data.

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Segmentation and Reassembly Start {#segmentation-and-reassembly-start}

_Opcode:_ `0x21`

_Total length:_ 1 byte

Opening message of a Segmentation and Reassembly message.

_Start takes no parameters._

#### Response

Potential status codes:

- `TRANSACTION_CONTINUE`

- `TRANSACTION_COMPLETE`

- `ERROR_DATA_SIZE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Segmentation and Reassembly Continue {#segmentation-and-reassembly-continue}

_Opcode:_ `0x22`

_Total length:_ 1 byte

Continuation of a Segmentation and Reassembly message.

_Continue takes no parameters._

#### Response

Potential status codes:

- `TRANSACTION_CONTINUE`

- `TRANSACTION_COMPLETE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration Adv Addr Set {#configuration-adv-addr-set}

_Opcode:_ `0x40`

_Total length:_ 8 bytes

Set the device's BLE advertisement address used for all BLE advertisment messages sent by the device.

_Adv Addr Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Addr Type                               | 1    | 0      | BLE advertising address type.
`uint8_t[6]`  | Adv Addr                                | 6    | 1      | BLE advertising address.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration Adv Addr Get {#configuration-adv-addr-get}

_Opcode:_ `0x41`

_Total length:_ 1 byte

Get the device's BLE advertisement address.

_Adv Addr Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Adv Addr Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Addr Type                               | 1    | 0      | Advertisement address type.
`uint8_t[6]`  | Addr                                    | 6    | 1      | Advertisement address.


---
### Configuration Channel Map Set {#configuration-channel-map-set}

_Opcode:_ `0x42`

_Total length:_ 2 bytes

Set the channel map for advertisement packets. The device will send the advertisement packets on all enabled channels in increasing order. The channel map parameter is a bitmap, where the first bit represents channel 37, the second bit channel 38, and the third bit channel 39. The rest of the byte is ignored. Set to `0x07` to enable all channels, `0x01` to only enable channel 37, and so on.

_Channel Map Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Channel Map                             | 1    | 0      | Channel map bitfield for mesh to use, starting at channel 37.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration Channel Map Get {#configuration-channel-map-get}

_Opcode:_ `0x43`

_Total length:_ 1 byte

Get the channel map used for advertisement packets.

_Channel Map Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration TX Power Set {#configuration-tx-power-set}

_Opcode:_ `0x44`

_Total length:_ 2 bytes

Set the transmission power of the radio. Must be a valid enumeration in `serial_cmd_config_tx_power_value`.

_TX Power Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | TX Power                                | 1    | 0      | Transmit power of radio, see @ref serial_cmd_tx_power_value_t for accepted values.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration TX Power Get {#configuration-tx-power-get}

_Opcode:_ `0x45`

_Total length:_ 1 byte

Get the transmission power of the radio.

_TX Power Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_TX Power Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | TX Power                                | 1    | 0      | TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.


---
### Configuration UUID Set {#configuration-uuid-set}

_Opcode:_ `0x53`

_Total length:_ 17 bytes

Set the device UUID used for identifying the device during provisioning. If the UUID isn't set, the device will use a preprogrammed UUID.

_UUID Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Device UUID.

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Configuration UUID Get {#configuration-uuid-get}

_Opcode:_ `0x54`

_Total length:_ 1 byte

Get the device UUID used for identifying the device during provisioning.

_UUID Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_UUID Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | Device UUID                             | 16   | 0      | Device UUID.


---
### Provisioning Scan Start {#provisioning-scan-start}

_Opcode:_ `0x61`

_Total length:_ 1 byte

Start reporting of incoming unprovisioned beacons.

_Scan Start takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Provisioning Scan Stop {#provisioning-scan-stop}

_Opcode:_ `0x62`

_Total length:_ 1 byte

Stop reporting of incoming unprovisioned beacons.

_Scan Stop takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Provisioning Provision {#provisioning-provision}

_Opcode:_ `0x63`

_Total length:_ 45 bytes

Start provisioning of a device. When a provisioning link has been successfully established, a _Provisioning Link Established_ event is received. If an error occurs, a _Provisioning Link Closed_ event is received. After a link has been established, a _Provisioning Capabilities Received_ event will be emitted upon receiving the peer node's OOB capabilities. To continue the provisioning process, a _Provisioning OOB Use_ command must be sent to select which kind of OOB authentication to use.

_Provision Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | Context ID to use for this provisioning session.
`uint8_t[16]` | Target UUID                             | 16   | 1      | UUID of the device to provision.
`uint8_t[16]` | Network Key                             | 16   | 17     | Network key to give to the device.
`uint16_t`    | Network Key Index                       | 2    | 33     | Network key index.
`uint32_t`    | Iv Index                                | 4    | 35     | Initial IV index of the network.
`uint16_t`    | Address                                 | 2    | 39     | Unicast address to assign to the device.
`uint8_t`     | Iv Update Flag                          | 1    | 41     | IV update in progress flag.
`uint8_t`     | Key Refresh Flag                        | 1    | 42     | Key refresh in progress flag.
`uint8_t`     | Attention Duration S                    | 1    | 43     | Time in seconds during which the device will identify itself using any means it can.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `ERROR_INVALID_DATA`

- `INVALID_LENGTH`

_Provision Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


---
### Provisioning Listen {#provisioning-listen}

_Opcode:_ `0x64`

_Total length:_ 1 byte

As an uprovisioned device, listen for incoming provisioning requests.

_Listen takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Provisioning OOB Use {#provisioning-oob-use}

_Opcode:_ `0x66`

_Total length:_ 5 bytes

Used to respond to the _Provisioning Capabilities Received_ event. It is used to select which kind of OOB authentication method to use. The values can be found in nrf_mesh_prov.h.<br><br>If authentication is enabled, the application will receive a _Provisioning Auth Request_ event requesting authentication data.  A _Provisioning ECDH Request_ will be received when the provisioner needs to calculate the ECDH shared secret for the nodes.  The _Provisioning Complete_ event is received when the provisioning procedure has completed successfully. At this point, a provisioner must wait for the _Provisioning Link Closed_ event before re-using the provisioning context.

_OOB Use Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of context to set the oob method for.
`uint8_t`     | OOB Method                              | 1    | 1      | OOB method to use, see @ref nrf_mesh_prov_oob_method_t for accepted values.
`uint8_t`     | OOB Action                              | 1    | 2      | OOB action to use, see @ref nrf_mesh_prov_input_action_t or @ref nrf_mesh_prov_output_action_t for values.
`uint8_t`     | Size                                    | 1    | 3      | Size of the OOB data.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_OOB Use Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


---
### Provisioning Auth Data {#provisioning-auth-data}

_Opcode:_ `0x67`

_Total length:_ 2..18 bytes

Used to respond to a _Provisioning Auth Request_ event. It passes OOB authentication data back to the mesh stack.

_Auth Data Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of the context to set the authentication data for.
`uint8_t[16]` | Data                                    | 0..16 | 1      | Authentication data.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Auth Data Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


---
### Provisioning ECDH Secret {#provisioning-ecdh-secret}

_Opcode:_ `0x68`

_Total length:_ 34 bytes

Used to respond to a _Provisioning ECDH Request_ event. It passes the calculated ECDH shared secret back to the mesh stack.

_ECDH Secret Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context ID                              | 1    | 0      | ID of the context to set the shared secret for.
`uint8_t[32]` | Shared Secret                           | 32   | 1      | ECDH shared secret.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_ECDH Secret Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Context                                 | 1    | 0      | Provisioning context ID


---
### Provisioning Keypair Set {#provisioning-keypair-set}

_Opcode:_ `0x69`

_Total length:_ 97 bytes

Send a public/private keypair to the device. These keys are used for some of the encryption involved in provisioning.

_Keypair Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[32]` | Private Key                             | 32   | 0      | Private key.
`uint8_t[64]` | Public Key                              | 64   | 32     | Public key.

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Provisioning Capabilities Set {#provisioning-capabilities-set}

_Opcode:_ `0x6a`

_Total length:_ 10 bytes

Used to set the out-of-band authentication capabilities of a device. The values for the parameters can be found in the various defines in the nrf_mesh_prov.h header file.

_Capabilities Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Num Elements                            | 1    | 0      | The number of elements in the device
`uint8_t`     | Public Key Type                         | 1    | 1      | The type of public key used in the device.
`uint8_t`     | Static OOB Types                        | 1    | 2      | The types of static OOB authentication methods.
`uint8_t`     | Output OOB Size                         | 1    | 3      | Maximum size of the OOB authentication output.
`uint16_t`    | Output OOB Actions                      | 2    | 4      | Available output actions for OOB authentication.
`uint8_t`     | Input OOB Size                          | 1    | 6      | Maximum size of the OOB authentication input.
`uint16_t`    | Input OOB Actions                       | 2    | 7      | Available input actions for OOB authentication.

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Init {#nrf-open-mesh-init}

_Opcode:_ `0x70`

_Total length:_ 1 byte

Not implemented.

_Init takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Value Set {#nrf-open-mesh-value-set}

_Opcode:_ `0x71`

_Total length:_ 1 byte

Not implemented.

_Value Set takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Value Enable {#nrf-open-mesh-value-enable}

_Opcode:_ `0x72`

_Total length:_ 1 byte

Not implemented.

_Value Enable takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Value Disable {#nrf-open-mesh-value-disable}

_Opcode:_ `0x73`

_Total length:_ 1 byte

Not implemented.

_Value Disable takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Start {#nrf-open-mesh-start}

_Opcode:_ `0x74`

_Total length:_ 1 byte

Not implemented.

_Start takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Stop {#nrf-open-mesh-stop}

_Opcode:_ `0x75`

_Total length:_ 1 byte

Not implemented.

_Stop takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Flag Set {#nrf-open-mesh-flag-set}

_Opcode:_ `0x76`

_Total length:_ 1 byte

Not implemented.

_Flag Set takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Flag Get {#nrf-open-mesh-flag-get}

_Opcode:_ `0x77`

_Total length:_ 1 byte

Not implemented.

_Flag Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh DFU Data {#nrf-open-mesh-dfu-data}

_Opcode:_ `0x78`

_Total length:_ 32 bytes

Send DFU data to the device.

_DFU Data Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[31]` | DFU Packet                              | 31   | 0      | DFU packet data.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Value Get {#nrf-open-mesh-value-get}

_Opcode:_ `0x7a`

_Total length:_ 1 byte

Not implemented.

_Value Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Build Version Get {#nrf-open-mesh-build-version-get}

_Opcode:_ `0x7b`

_Total length:_ 1 byte

Not implemented.

_Build Version Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Access Addr Get {#nrf-open-mesh-access-addr-get}

_Opcode:_ `0x7c`

_Total length:_ 1 byte

Not implemented.

_Access Addr Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Channel Get {#nrf-open-mesh-channel-get}

_Opcode:_ `0x7d`

_Total length:_ 1 byte

Not implemented.

_Channel Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### nRF Open Mesh Interval Min ms Get {#nrf-open-mesh-interval-min-ms-get}

_Opcode:_ `0x7f`

_Total length:_ 1 byte

Not implemented.

_Interval Min ms Get takes no parameters._

#### Response

Potential status codes:

- `ERROR_CMD_UNKNOWN`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Enable {#bluetooth-mesh-enable}

_Opcode:_ `0x90`

_Total length:_ 1 byte

Enable mesh operation. Starts radio scanning and transmissions.

_Enable takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Disable {#bluetooth-mesh-disable}

_Opcode:_ `0x91`

_Total length:_ 1 byte

Disable mesh operation. Stops radio scanning and transmissions.

_Disable takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Subnet Add {#bluetooth-mesh-subnet-add}

_Opcode:_ `0x92`

_Total length:_ 19 bytes

Add a mesh subnetwork to the device.

_Subnet Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Net Key Index                           | 2    | 0      | Mesh-global key index.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to add.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Add Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


---
### Bluetooth Mesh Subnet Update {#bluetooth-mesh-subnet-update}

_Opcode:_ `0x93`

_Total length:_ 19 bytes

Update a mesh subnetwork's root key.

_Subnet Update Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to change.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to change to.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Update Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


---
### Bluetooth Mesh Subnet Delete {#bluetooth-mesh-subnet-delete}

_Opcode:_ `0x94`

_Total length:_ 3 bytes

Delete a subnetwork from the device.

_Subnet Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to delete.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Delete Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Subnetwork handle operated on.


---
### Bluetooth Mesh Subnet Get All {#bluetooth-mesh-subnet-get-all}

_Opcode:_ `0x95`

_Total length:_ 1 byte

Get all known subnetwork key indexes.

_Subnet Get All takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Subnet Get All Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t[126]` | Subnet Key Index                        | 252  | 0      | List of all subnetwork key indexes known by the device.


---
### Bluetooth Mesh Subnet Count Max Get {#bluetooth-mesh-subnet-count-max-get}

_Opcode:_ `0x96`

_Total length:_ 1 byte

Get the maximum number of subnetworks the device can fit.

_Subnet Count Max Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Subnet Count Max Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


---
### Bluetooth Mesh Appkey Add {#bluetooth-mesh-appkey-add}

_Opcode:_ `0x97`

_Total length:_ 21 bytes

Add a mesh application key to the device.

_Appkey Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | App Key Index                           | 2    | 0      | Mesh-global key index.
`uint16_t`    | Subnet Handle                           | 2    | 2      | Handle of the subnetwork to add the appkey to.
`uint8_t[16]` | Key                                     | 16   | 4      | Key to add.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Add Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


---
### Bluetooth Mesh Appkey Update {#bluetooth-mesh-appkey-update}

_Opcode:_ `0x98`

_Total length:_ 19 bytes

Update a mesh application key.

_Appkey Update Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Handle of the appkey to change.
`uint8_t[16]` | Key                                     | 16   | 2      | Key to change to.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Update Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


---
### Bluetooth Mesh Appkey Delete {#bluetooth-mesh-appkey-delete}

_Opcode:_ `0x99`

_Total length:_ 3 bytes

Delete a application key from the device.

_Appkey Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Handle of the appkey to delete.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Delete Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Application key handle operated on.


---
### Bluetooth Mesh Appkey Get All {#bluetooth-mesh-appkey-get-all}

_Opcode:_ `0x9a`

_Total length:_ 3 bytes

Get all the application key indices of the stored application keys associated with a specific subnetwork.

_Appkey Get All Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the subnet to get all appkeys of.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Appkey Get All Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Subnet Handle                           | 2    | 0      | Handle of the Subnetwork associated with the application keys.
`uint16_t[125]` | Appkey Key Index                        | 250  | 2      | List of all application key indexes known by the device.


---
### Bluetooth Mesh Appkey Count Max Get {#bluetooth-mesh-appkey-count-max-get}

_Opcode:_ `0x9b`

_Total length:_ 1 byte

Get the maximum number of application keys the device can fit.

_Appkey Count Max Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Appkey Count Max Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


---
### Bluetooth Mesh Devkey Add {#bluetooth-mesh-devkey-add}

_Opcode:_ `0x9c`

_Total length:_ 21 bytes

Add a mesh device key to the device.

_Devkey Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Owner Addr                              | 2    | 0      | Unicast address of the device that owns the given devkey.
`uint16_t`    | Subnet Handle                           | 2    | 2      | Handle of the subnetwork to bind the devkey to.
`uint8_t[16]` | Key                                     | 16   | 4      | Key to add.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Devkey Add Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Device key handle operated on.


---
### Bluetooth Mesh Devkey Delete {#bluetooth-mesh-devkey-delete}

_Opcode:_ `0x9d`

_Total length:_ 3 bytes

Delete a device key from the device.

_Devkey Delete Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Handle of the devkey to delete.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Devkey Delete Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Devkey Handle                           | 2    | 0      | Device key handle operated on.


---
### Bluetooth Mesh Devkey Count Max Get {#bluetooth-mesh-devkey-count-max-get}

_Opcode:_ `0x9e`

_Total length:_ 1 byte

Get the maximum number of device keys the device can fit.

_Devkey Count Max Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Devkey Count Max Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


---
### Bluetooth Mesh Addr Local Unicast Set {#bluetooth-mesh-addr-local-unicast-set}

_Opcode:_ `0x9f`

_Total length:_ 5 bytes

Set the start and count of the device's local unicast address.

_Addr Local Unicast Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Start Address                           | 2    | 0      | First address in the range of unicast addresses.
`uint16_t`    | Count                                   | 2    | 2      | Number of addresses in the range of unicast addresses.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Addr Local Unicast Get {#bluetooth-mesh-addr-local-unicast-get}

_Opcode:_ `0xa0`

_Total length:_ 1 byte

Get the start and count of the device's local unicast addresses.

_Addr Local Unicast Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Local Unicast Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Start                           | 2    | 0      | First address in the range of unicast addresses.
`uint16_t`    | Count                                   | 2    | 2      | Number of addresses in the range of unicast addresses.


---
### Bluetooth Mesh Addr Get {#bluetooth-mesh-addr-get}

_Opcode:_ `0xa7`

_Total length:_ 3 bytes

Get the raw representation of the address with the given handle. If the given address is a virtual address, the virtual UUID will be included in the response.

_Addr Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of address to get raw representation of.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle requested.
`uint8_t`     | Addr Type                               | 1    | 2      | Address type of the given address. See @ref nrf_mesh_address_type_t for accepted values.
`uint8_t`     | Subscribed                              | 1    | 3      | Flag indicating whether the given address is subscribed to or not.
`uint16_t`    | Raw Short Addr                          | 2    | 4      | Raw representation of the address.
`uint8_t[16]` | Virtual UUID                            | 16   | 6      | Optional virtual UUID of the given address.


---
### Bluetooth Mesh Addr Get All {#bluetooth-mesh-addr-get-all}

_Opcode:_ `0xa8`

_Total length:_ 1 byte

Get a list of all address handles in the address pool, not including local unicast addresses.

_Addr Get All takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Get All Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t[126]` | Address Handles                         | 252  | 0      | List of all address handles known by the device, not including local unicast addresses.


---
### Bluetooth Mesh Addr Nonvirtual Count Max Get {#bluetooth-mesh-addr-nonvirtual-count-max-get}

_Opcode:_ `0xa9`

_Total length:_ 1 byte

Get the maximum number of non-virtual addresses the device can fit.

_Addr Nonvirtual Count Max Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Nonvirtual Count Max Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


---
### Bluetooth Mesh Addr Virtual Count Max Get {#bluetooth-mesh-addr-virtual-count-max-get}

_Opcode:_ `0xaa`

_Total length:_ 1 byte

Get the maximum number of virtual addresses the device can fit.

_Addr Virtual Count Max Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_Addr Virtual Count Max Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | List Size                               | 2    | 0      | Size of the list requested by the command.


---
### Bluetooth Mesh Addr Subscription Add {#bluetooth-mesh-addr-subscription-add}

_Opcode:_ `0xa1`

_Total length:_ 3 bytes

Add the specified address to the set of active address subscriptions.

_Addr Subscription Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address                                 | 2    | 0      | Address to add as a subscription address.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Add Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Addr Subscription Add Virtual {#bluetooth-mesh-addr-subscription-add-virtual}

_Opcode:_ `0xa2`

_Total length:_ 17 bytes

Add the virtual address with the specified UUID to the set of active address subscriptions.

_Addr Subscription Add Virtual Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Virtual address UUID.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Add Virtual Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Addr Subscription Remove {#bluetooth-mesh-addr-subscription-remove}

_Opcode:_ `0xa3`

_Total length:_ 3 bytes

Remove the address with the given handle from the set of active address subscriptions.

_Addr Subscription Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of address to remove from address subscription list.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Subscription Remove Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Addr Publication Add {#bluetooth-mesh-addr-publication-add}

_Opcode:_ `0xa4`

_Total length:_ 3 bytes

Add the specified address to the set of active publish addresses.

_Addr Publication Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address                                 | 2    | 0      | Address to add as a publication address.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Add Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Addr Publication Add Virtual {#bluetooth-mesh-addr-publication-add-virtual}

_Opcode:_ `0xa5`

_Total length:_ 17 bytes

Add the virtual address with the specified UUID to the set of active publish addresses.

_Addr Publication Add Virtual Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t[16]` | UUID                                    | 16   | 0      | Virtual address UUID.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Add Virtual Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Addr Publication Remove {#bluetooth-mesh-addr-publication-remove}

_Opcode:_ `0xa6`

_Total length:_ 3 bytes

Remove the address with the specified handle from the set of active publish addresses.

_Addr Publication Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of the address to remove from the publication address list.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Addr Publication Remove Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Address handle operated on.


---
### Bluetooth Mesh Packet Send {#bluetooth-mesh-packet-send}

_Opcode:_ `0xab`

_Total length:_ 11..255 bytes

Send a mesh packet. The source address handle must represent a local unicast address.

_Packet Send Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Appkey Handle                           | 2    | 0      | Appkey or devkey handle to use for packet sending. Subnetwork will be picked automatically.
`uint16_t`    | SRC Addr                                | 2    | 2      | Raw unicast address to use as source address. Must be in the range of local unicast addresses.
`uint16_t`    | DST Addr Handle                         | 2    | 4      | Handle of destination address to use in packet.
`uint8_t`     | TTL                                     | 1    | 6      | Time To Live value to use in packet.
`uint8_t`     | Force Segmented                         | 1    | 7      | Whether or not to force use of segmented message type for the transmission.
`uint8_t`     | Transmic Size                           | 1    | 8      | Transport MIC size used enum. SMALL=0, LARGE=1, DEFAULT=2. LARGE may only be used with segmented packets.
`uint8_t`     | Friendship Credential Flag              | 1    | 9      | Control parameter for credentials used to publish messages from a model. 0 for master, 1 for friendship.
`uint8_t[244]` | Data                                    | 0..244 | 10     | Payload of the packet.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_DATA_SIZE`

- `ERROR_REJECTED`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_Packet Send Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`nrf_mesh_tx_token_t` | Token                                   | 4    | 0      | TX Token assigned to the packet. Can be used to resolve which packet a @ref SERIAL_OPCODE_EVT_MESH_TX_COMPLETE event refers to.


---
### Bluetooth Mesh State Clear {#bluetooth-mesh-state-clear}

_Opcode:_ `0xac`

_Total length:_ 1 byte

Reset the device and network state and erase the flash copies.

_State Clear takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Config Server Bind {#bluetooth-mesh-config-server-bind}

_Opcode:_ `0xad`

_Total length:_ 3 bytes

Binds the config server model instance to given device key handle.

_Config Server Bind Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Address Handle                          | 2    | 0      | Handle of the address to get the raw representation of.

#### Response

Potential status codes:

- `SUCCESS`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Net State Set {#bluetooth-mesh-net-state-set}

_Opcode:_ `0xae`

_Total length:_ 12 bytes

Sets the network state.

_Net State Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint32_t`    | Iv Index                                | 4    | 0      | The IV index to set.
`uint8_t`     | Iv Update In Progress                   | 1    | 4      | Value indicating the phase of the IV update process.
`uint16_t`    | Iv Update Timeout Counter               | 2    | 5      | Timeout counter for IV update process.
`uint32_t`    | Next Seqnum Block                       | 4    | 7      | The first sequence number block which is not yet allocated.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Bluetooth Mesh Net State Get {#bluetooth-mesh-net-state-get}

_Opcode:_ `0xaf`

_Total length:_ 1 byte

Gets the network state.

_Net State Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Net State Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint32_t`    | Iv Index                                | 4    | 0      | The current IV index.
`uint8_t`     | Iv Update In Progress                   | 1    | 4      | Value indicating the phase of the IV update process.
`uint16_t`    | Iv Update Timeout Counter               | 2    | 5      | Current value of timeout counter for IV update.
`uint32_t`    | Next Seqnum Block                       | 4    | 7      | The start of the next unused sequence number block.


---
### Direct Firmware Upgrade Jump To Bootloader {#direct-firmware-upgrade-jump-to-bootloader}

_Opcode:_ `0xd0`

_Total length:_ 1 byte

Immediately jump to bootloader mode. If successful, this call will not yield a command response. It will however yield a _Device Started_ event if the current bootloader supports serial communication.

_Jump To Bootloader takes no parameters._

#### Response

Potential status codes:

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Direct Firmware Upgrade Request {#direct-firmware-upgrade-request}

_Opcode:_ `0xd1`

_Total length:_ 16 bytes

Request a DFU transfer.

_Request Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to request.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID to request.
`uint32_t`    | Bank Addr                               | 4    | 11     | Address in which to bank firmware.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Direct Firmware Upgrade Relay {#direct-firmware-upgrade-relay}

_Opcode:_ `0xd2`

_Total length:_ 12 bytes

Relay a DFU transfer.

_Relay Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to relay.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID of firmware that should be relayed.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Direct Firmware Upgrade Abort {#direct-firmware-upgrade-abort}

_Opcode:_ `0xd3`

_Total length:_ 1 byte

Abort the ongoing DFU transfer. Fails if there is no ongoing transfer.

_Abort takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Direct Firmware Upgrade Bank Info Get {#direct-firmware-upgrade-bank-info-get}

_Opcode:_ `0xd4`

_Total length:_ 2 bytes

Get information about the firmware bank of the given type, if it exists.

_Bank Info Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to get bank info about.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_PARAMETER`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Bank Info Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU type of the bank.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 1      | Firmware ID of the bank.
`uint8_t`     | Is Signed                               | 1    | 11     | Flag indicating whether the bank is signed with an encryption key.
`uint32_t`    | Start Addr                              | 4    | 12     | Start address of the bank.
`uint32_t`    | Length                                  | 4    | 16     | Length of the firmware in the bank.


---
### Direct Firmware Upgrade Bank Flash {#direct-firmware-upgrade-bank-flash}

_Opcode:_ `0xd5`

_Total length:_ 2 bytes

Flash the bank with the given firmware type. If successful, this serial call does not produce a command response. Note that all volatile memory will be lost, as the device will restart.<br><br>If the new firmware supports serial communication, the device issues a device started event when it is ready to receive new commands.

_Bank Flash Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | DFU Type                                | 1    | 0      | DFU Firmware type to flash.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Direct Firmware Upgrade State Get {#direct-firmware-upgrade-state-get}

_Opcode:_ `0xd6`

_Total length:_ 1 byte

Get the current state of the DFU module. Only works if the DFU module has been initialized.

_State Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_State Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Role                                    | 1    | 0      | This device's intended role in the transfer, see @ref nrf_mesh_dfu_role_t for accepted values.
`uint8_t`     | Type                                    | 1    | 1      | The DFU type of the transfer, see @ref nrf_mesh_dfu_type_t for accepted values.
`nrf_mesh_fwid_t` | FWID                                    | 10   | 2      | The FWID of the new data in the transfer.
`uint8_t`     | State                                   | 1    | 12     | The current global state of the transfer, see @ref nrf_mesh_dfu_state_t for accepted values.
`uint8_t`     | Data Progress                           | 1    | 13     | The progress of the transfer in percent (0-100).


---
### Access Layer Model Pub Addr Set {#access-layer-model-pub-addr-set}

_Opcode:_ `0xe0`

_Total length:_ 5 bytes

Set the publish address for a model instance.

_Model Pub Addr Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Pub Addr Get {#access-layer-model-pub-addr-get}

_Opcode:_ `0xe1`

_Total length:_ 3 bytes

Get the publish address for a model instance.

_Model Pub Addr Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_Model Pub Addr Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`dsm_handle_t` | Addr Handle                             | 2    | 0      | Address handle for the publish address.


---
### Access Layer Model Pub Period Set {#access-layer-model-pub-period-set}

_Opcode:_ `0xe2`

_Total length:_ 5 bytes

Set the publish period for a model instance.

_Model Pub Period Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the model that the access module should operate on.
`uint8_t`     | Resolution                              | 1    | 2      | see @ref access_publish_resolution_t for accepted values.
`uint8_t`     | Step Number                             | 1    | 3      | Must not be larger than @ref ACCESS_PUBLISH_PERIOD_STEP_MAX.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Pub Period Get {#access-layer-model-pub-period-get}

_Opcode:_ `0xe3`

_Total length:_ 3 bytes

Get the publish period for a model instance.

_Model Pub Period Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_Model Pub Period Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Resolution                              | 1    | 0      | Resolution of each step.
`uint8_t`     | Step Number                             | 1    | 1      | Number of steps in each period.


---
### Access Layer Model Subs Add {#access-layer-model-subs-add}

_Opcode:_ `0xe4`

_Total length:_ 5 bytes

Add a subscription address to a model instance.

_Model Subs Add Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Subs Remove {#access-layer-model-subs-remove}

_Opcode:_ `0xe5`

_Total length:_ 5 bytes

Remove a subscription address from a model instance.

_Model Subs Remove Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Subs Get {#access-layer-model-subs-get}

_Opcode:_ `0xe6`

_Total length:_ 3 bytes

Get the list of subscription addresses from a model instance.

_Model Subs Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_Model Subs Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Count                                   | 2    | 0      | Number of available handles in @c address_handles
`dsm_handle_t[125]` | Address Handles                         | 250  | 2      | List of the address handles of all subscription addresses bound to the given model


---
### Access Layer Model App Bind {#access-layer-model-app-bind}

_Opcode:_ `0xe7`

_Total length:_ 5 bytes

Bind an application key to a model instance.

_Model App Bind Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model App Unbind {#access-layer-model-app-unbind}

_Opcode:_ `0xe8`

_Total length:_ 5 bytes

Unbind an application key from a model instance.

_Model App Unbind Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model App Get {#access-layer-model-app-get}

_Opcode:_ `0xe9`

_Total length:_ 3 bytes

Get all the application keys bound to a model instance.

_Model App Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_Model App Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Count                                   | 2    | 0      | Number of available handles in @c appkey_handles
`dsm_handle_t[125]` | Appkey Handles                          | 250  | 2      | List of the address handles of all subscription addresses bound to the given model


---
### Access Layer Model Pub App Set {#access-layer-model-pub-app-set}

_Opcode:_ `0xea`

_Total length:_ 5 bytes

Set the application key to be used when publishing for a model instance.

_Model Pub App Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle for the model being modified.
`dsm_handle_t` | Dsm Handle                              | 2    | 2      | Handle for a value (e.g. address) stored by the device state manager.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Pub App Get {#access-layer-model-pub-app-get}

_Opcode:_ `0xeb`

_Total length:_ 3 bytes

Get the application key used when publishing for a model instance.

_Model Pub App Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_Model Pub App Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`dsm_handle_t` | Appkey Handle                           | 2    | 0      | Handle of the application key used for publishing.


---
### Access Layer Model Pub TTL Set {#access-layer-model-pub-ttl-set}

_Opcode:_ `0xec`

_Total length:_ 4 bytes

Set the default TTL value used when publishing for a model instance.

_Model Pub TTL Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the model that the access module should operate on.
`uint8_t`     | TTL                                     | 1    | 2      | TTL for outgoing messages.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `ERROR_INVALID_PARAMETER`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Model Pub TTL Get {#access-layer-model-pub-ttl-get}

_Opcode:_ `0xed`

_Total length:_ 3 bytes

Get the default TTL value used when publishing for a model instance.

_Model Pub TTL Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_Model Pub TTL Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | TTL                                     | 1    | 0      | TTL for published messages.


---
### Access Layer Elem Loc Set {#access-layer-elem-loc-set}

_Opcode:_ `0xee`

_Total length:_ 5 bytes

Set the location descriptor for an element.

_Elem Loc Set Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element.
`uint16_t`    | Location                                | 2    | 2      | Location value for the element.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_The response has no parameters._

---
### Access Layer Elem Loc Get {#access-layer-elem-loc-get}

_Opcode:_ `0xef`

_Total length:_ 3 bytes

Get the location descriptor for an element.

_Elem Loc Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Elem Loc Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Location                                | 2    | 0      | Element location info.


---
### Access Layer Elem Sig Model Count Get {#access-layer-elem-sig-model-count-get}

_Opcode:_ `0xf0`

_Total length:_ 3 bytes

Get the number of Bluetooth SIG models for an element.

_Elem Sig Model Count Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Elem Sig Model Count Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Model Count                             | 1    | 0      | Number of existing models.


---
### Access Layer Elem Vendor Model Count Get {#access-layer-elem-vendor-model-count-get}

_Opcode:_ `0xf1`

_Total length:_ 3 bytes

Get the number of vendor specific models for an element.

_Elem Vendor Model Count Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Elem Vendor Model Count Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Model Count                             | 1    | 0      | Number of existing models.


---
### Access Layer Model ID Get {#access-layer-model-id-get}

_Opcode:_ `0xf2`

_Total length:_ 3 bytes

Get the model ID of a model instance.

_Model ID Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Handle                                  | 2    | 0      | Handle of the model that the access module should operate on.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `ERROR_INVALID_STATE`

- `INVALID_LENGTH`

_Model ID Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_id_t` | Model ID                                | 4    | 0      | Company and model IDs.


---
### Access Layer Handle Get {#access-layer-handle-get}

_Opcode:_ `0xf3`

_Total length:_ 7 bytes

Get the handle assigned to the model instance of a model based on the element index and model ID.

_Handle Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element which owns the model.
`access_model_id_t` | Model ID                                | 4    | 2      | Company and model IDs.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Handle Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the requested model.


---
### Access Layer Elem Models Get {#access-layer-elem-models-get}

_Opcode:_ `0xf4`

_Total length:_ 3 bytes

Get the array of handles corresponding to an element.

_Elem Models Get Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Element Index                           | 2    | 0      | Index of the addressed element.

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_Elem Models Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Count                                   | 2    | 0      | Number of available handles in @c model_handles
`access_model_handle_t[125]` | Model Handles                           | 250  | 2      | List of the address handles of all subscription addresses bound to the given model


---
### Model Specific Models Get {#model-specific-models-get}

_Opcode:_ `0xfc`

_Total length:_ 1 byte

Get a list of all the models available on the device.

_Models Get takes no parameters._

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_INVALID_LENGTH`

- `INVALID_LENGTH`

_Models Get Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint16_t`    | Count                                   | 2    | 0      | Number of available handles in @c model_ids
`access_model_id_t[62]` | Model IDs                               | 248  | 2      | List of the model ids of all the available models.


---
### Model Specific Init {#model-specific-init}

_Opcode:_ `0xfd`

_Total length:_ 7..255 bytes

Call the initializer of the addressed model to create a new instance.

_Init Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`serial_cmd_model_specific_init_header_t` | Model Init Info                         | 6    | 0      | Basic information that is always needed to initialize a model
`uint8_t[248]` | Data                                    | 0..248 | 6      | Additional data provided to the initializer

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_LENGTH`

_Init Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`access_model_handle_t` | Model Handle                            | 2    | 0      | Handle of the initialized model.


---
### Model Specific Command {#model-specific-command}

_Opcode:_ `0xfe`

_Total length:_ 3..255 bytes

Forward a model specific command to a model instance. See the serial handler for the specific model being commanded for more information.

_Command Parameters:_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`serial_cmd_model_specific_command_header_t` | Model Cmd Info                          | 2    | 0      | Contains the handle of the model being addressed.
`uint8_t[252]` | Data                                    | 0..252 | 2      | Additional data provided to the event

#### Response

Potential status codes:

- `SUCCESS`

- `ERROR_REJECTED`

- `INVALID_STATE`

- `INVALID_LENGTH`

_Command Response Parameters_

Type          | Name                                    | Size | Offset | Description
--------------|-----------------------------------------|------|--------|------------
`uint8_t`     | Data Len                                | 1    | 0      | Length of data array. Set to 0 to indicate no data to send
`uint8_t[251]` | Data                                    | 0..251 | 1      | Command response data specific to each model.


