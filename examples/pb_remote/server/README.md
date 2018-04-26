# Remote provisioning server example (experimental)

This example demonstrates how the client can use a remote provisioning server as a relay for provisioning nodes that it cannot reach directly.

For more information about remote provisioning, see the
[PB-remote API reference](@ref PB_REMOTE).

The example application starts out as a normal provisionee. It
starts acting as a remote provisioning server only after it has been
provisioned into a network.

## Running the example

To build the example, follow the instructions in
[Building the Mesh Stack](@ref md_doc_getting_started_how_to_build). Refer to the *How to run examples*
section in [Examples README](@ref md_examples_README) for the commands required to program a
device using `nrfjprog`.

To use this example application with the client example application,
flash one device with the client application and two or
more devices with the server application.

See the [client documentation](@ref md_examples_pb_remote_client_README) for
instructions on how to run the example.
