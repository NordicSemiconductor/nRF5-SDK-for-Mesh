# Remote provisioning client example (experimental)

This example demonstrates how to use the remote provisioning client model to
provision remote devices. To use this example, flash the client application
to one device and the server application to two or more devices.

@note
This example is experimental, meaning it is ready for use, but not qualified.

For more information about remote provisioning, see the
[PB-remote API reference](@ref PB_REMOTE).

## Running the example

To build the example, follow the instructions in
[Building the Mesh Stack](@ref md_doc_getting_started_how_to_build). For commands required to program a device using `nrfjprog`,
see the [Running examples using nrfjprog](@ref how_to_run_examples_nrfjprog) section on the @ref md_doc_getting_started_how_to_run_examples page.

The client is controlled over [RTT](@ref segger-rtt) with a few simple commands:

- Send `1` to start normal PB-ADV on the first unprovisioned device.
- Send `2N`, where `N` is the handle of the newly provisioned device, to set the handle of the
  remote provisioning server.
- Send `3` to start remote scanning on the current server.
- Send `4` to cancel remote scanning on the current server.
- Send `5N`, where `N` is the device number, to start remote provisioning of the first unprovisioned
  device known to the current provisioning server.

> **Important:** These commands must happen in order.

