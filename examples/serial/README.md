# Serial example

This example implements the serial interface for the mesh stack. It can be used
unaltered as connectivity firmware for mesh devices. You can also modify the example
to provide additional functionality.

The example shows how to initialize and use the serial interface.
To initialize the serial interface, only two API calls are required:
```C
    nrf_mesh_serial_init(NULL);
    nrf_mesh_serial_enable();
```

The @ref nrf_mesh_serial_init() API initializes the serial bearer interface, and the
@ref nrf_mesh_serial_enable() API sends the @ref SERIAL_OPCODE_EVT_DEVICE_STARTED event over the
serial interface to notify the external host that the device is ready to accept serial commands.

This example also demonstrates how to offload certain mesh related tasks (to the external host
using a serial interface) and improve the performance of the mesh stack.
The ECDH operations used in the provisioning process are a suitable candidate for such offloading.

The ECDH offloading allows the device to take advantage of the host's more powerful
processor to perform ECDH operations during provisioning. It is enabled by setting the corresponding
option value to 1, as shown below:
the [options API](@ref NRF_MESH_OPT):
```C
    nrf_mesh_opt_t value = {.len = 4, .opt.val = 1 };
    nrf_mesh_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &value);
```

When this offloading is enabled, the serial interface sends the @ref SERIAL_OPCODE_EVT_PROV_ECDH_REQUEST
event to the host processor with the public and private keys. The host processor then performs the ECDH
shared secret calculation and sends the calculated value back using the @ref SERIAL_OPCODE_CMD_PROV_ECDH_SECRET
serial command.

## Running the example

To build the example, follow the instructions in
[Building the Mesh Stack](@ref md_doc_getting_started_how_to_build). Refer to the *How to run examples*
section in [Examples README](@ref md_examples_README) for the commands required to program a
device using `nrfjprog`.

Go to the [Interactive PyACI documentation](@ref md_scripts_interactive_pyaci_README) to
get started with the serial interface.
