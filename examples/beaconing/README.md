# Beaconing example

This example shows how to do concurrent beaconing, allowing an application to
advertise beacons (such as iBeacon or Eddystone beacons) while at the same time
participating in the mesh network. The example illustrates usage of the Packet RX
callback functionality and application usage of advertisers.

## RX callback

The RX callback must be registered in the mesh framework by calling
`nrf_mesh_rx_cb_set()` (after `nrf_mesh_init()`). As input, the RX callback
function takes a pointer to a parameter struct that
contains all data available on the incoming packet.

The RX callback is invoked for all packets that are processed by the mesh after the mesh
itself has processed them. The mesh assumes that all incoming packets adhere to the
Bluetooth low energy advertisement packet format.

## Beacon transmission

To send beacons, the application uses the mesh-internal packet manager and
advertiser structure directly.

The application first initializes the advertiser (initialization is needed only once).
It then allocates and fills the fields of the packet. There is no need to set
the packet type or advertisement address, because this is
taken care of by the advertiser module. The application then schedules the packet
for transmission by putting it in the TX queue of the advertiser, with a parameter
indicating the number of repeats that the advertiser should do. In this example,
the repeat count is set to `BEARER_ADV_REPEAT_INFINITE`, causing the packet to
be retransmitted forever or until replaced by a different packet.

> **Important:** Using the packet manager and advertiser directly makes the
> application compete for the same resources as the core mesh framework. Incorrect or
> heavy usage will affect mesh performance or stability. Treat these modules
> with caution.

## Running the example

To build the example, follow the instructions in
[Building the Mesh Stack](@ref md_doc_getting_started_how_to_build). For commands required to program a device using `nrfjprog`,
see the [Running examples using nrfjprog](@ref how_to_run_examples_nrfjprog) section on the @ref md_doc_getting_started_how_to_run_examples page.

Once running, the example outputs all incoming packets over [RTT](@ref segger-rtt). Outgoing
beacons can be observed with @link_nRFConnectDesktop<!-- https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop -->
or @link_nRFConnectMobile<!-- https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-mobile -->.