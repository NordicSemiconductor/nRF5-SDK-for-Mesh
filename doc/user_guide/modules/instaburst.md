# Nordic Advertiser Extensions (Instaburst)

The Instaburst feature is a drop-in replacement for the standard BLE Advertiser
bearer for the Bluetooth mesh. When enabled, all communication in the core Bluetooth mesh will happen
through Instaburst instead of regular advertisers, yielding higher throughput, but breaking
compatibility with other Bluetooth mesh implementations.

In optimal conditions, Instaburst is able to transmit 498 bytes of raw advertising data every
advertising interval, providing a 160-kbps bearer to the upper layers for 20-ms advertisement
intervals. The Bluetooth mesh packets have significant overhead however, and under the same conditions
the maximum theoretical access layer payload throughput is around 60 kbps.

@note
Instaburst is a Nordic proprietary feature that does not adhere to the Bluetooth mesh
specification. It does not have the same requirements for test coverage, API stability
or specification compliance as the rest of the Nordic nRF5 SDK for Mesh.

**Table of contents**
- [Protocol](@ref instaburst_protocol)
- [Extended advertisements](@ref instaburst_extended_advertisements)
- [Usage in Bluetooth mesh](@ref instaburst_usage)
- [TX Buffer management](@ref instaburst_tx_buffer)
    - [Advertising events](@ref instaburst_tx_buffer_advertising_evt)
    - [Packets](@ref instaburst_tx_buffer_packets)
- [RX Buffer management](@ref instaburst_rx_buffer)


---


## Protocol @anchor instaburst_protocol

Instaburst uses a subset of the Bluetooth 5.0 Extended advertising events feature to increase the
throughput. It runs a dynamic buffer allocation algorithm that makes a per-packet decision on
whether to transmit it as a regular advertisement packet or as an extended advertising event.
This maximizes throughput while minimizing the radio time spent by each Bluetooth mesh device.


---


## Extended advertisements @anchor instaburst_extended_advertisements

All Instaburst usage of advertising extensions is done with a locked configuration that always runs:
- an uncoded 2-Mbit bearer;
- a fixed, minimal delay from the indication packet to the first auxiliary packet.

Every advertisement event contains a set of packets:
- Three extended advertisement packets (*ADV_EXT_IND* in the Bluetooth
specification).
- Single auxiliary advertisement packet that the three extended advertisement packets point to
(*AUX_ADV_IND* in the Bluetooth specification).
    - If a bigger payload is required, the single auxiliary packet can point to a second
    auxiliary packet (*AUX_CHAIN_IND* in the Bluetooth specification), which contains more data.

To remain scalable for larger Bluetooth mesh networks, the chain length is limited to this initial auxiliary
packet and a single chain packet.

![Extended advertising event, as implemented in Instaburst](images/adv_ext_evt.svg)

For more information about the extended advertisement protocol, see the Bluetooth Core 5.0 specification.


---


## Usage in Bluetooth mesh @anchor instaburst_usage

When Instaburst is enabled in the build, the Bluetooth mesh will:
- _For TX:_ instantiate an Instaburst TX instance instead of its usual advertiser instance
for the core message sending.
- _For RX:_ register a processing callback with the Instaburst RX module, and funnel the payload
into @ref AD_LISTENER for processing. The Instaburst packets come with their own set of
metadata, which is presented to the upper layers.

To start using Instaburst, enable the `EXPERIMENTAL_INSTABURST_ENABLED` option in the CMake
configuration. This also enables Instaburst for the Segger Embedded Studio project files,
which must be regenerated (see @ref md_doc_getting_started_how_to_build for details).


---


## TX Buffer management @anchor instaburst_tx_buffer

As the extended advertising packets have dynamically-sized headers and are allocated on a
per-advertising event basis, the buffer management in the Instaburst TX module is different
from the pattern of the advertiser module.

### Advertising events @anchor instaburst_tx_buffer_advertising_evt

The Instaburst packets are allocated on a per-advertising event basis, with:
- metadata for the event
- list of packets
- list of TX tokens

To match the advertiser API, each user buffer allocated in the Instaburst TX API must be
tied to a TX token (which will be passed back to the user once the packet has been sent).
TX tokens are used for internal book keeping, and shouldn't go on air. As there are multiple
user buffers in a single packet, the TX tokens must be allocated separately.
To avoid constraining the amount of separate user buffers that
go into a single extended advertisement event, the tokens are stored at the end of the
advertising event buffer, in reverse order.

The buffer is considered full when the list of tokens
has grown to meet the list of packets. To reduce the amount of memory that the token list takes
off the packet list, some extra padding space is allocated between the maximum length
of the packet list and the end of the buffer, where the token list starts.

![Extended Advertising packet TX buffer](images/adv_ext_evt_buf.svg)

### Packets @anchor instaburst_tx_buffer_packets

The sizes of dynamic headers of the extended advertising packets are determined by the packet's
role in the advertising event. At the time of transmission, the module decides which fields
in the common extended advertising header are to be included.

To avoid moving the payload according to the size of the header, the Instaburst packets
are allocated with the worst-case size header buffer, before the payload. Then, before transmitting
the packet, the module calculates the size of the header and builds it in such a way
that it ends exactly where the payload buffer begins, leaving out some padding ahead of it
in the header buffer.

![Extended Advertising TX packet](images/adv_ext_evt_packet.svg)


---

## RX Buffer management @anchor instaburst_rx_buffer

Instaburst RX buffers work the same way as the Advertiser RX buffers. They store each Extended
advertising packet in the buffer as a separate entity, bundled with its metadata.
