# Serial example

@tag52840and52833and52832
@tag52840dongle52810and52820nosupport

This example implements the serial interface for the Bluetooth mesh stack. It can be used
unaltered as connectivity firmware for Bluetooth mesh devices. You can also modify the example
to provide additional functionality.

**Table of contents**
- [Serial interface initialization](@ref serial_example_initialization)
- [Offloading feature](@ref serial_example_offloading)
- [Setup](@ref serial_example_setup)
- [Testing the example](@ref serial_example_testing)

## Serial interface initialization @anchor serial_example_initialization

The example shows how to initialize and use the serial interface.
To initialize the serial interface, only two API calls are required:
```C
    nrf_mesh_serial_init(NULL);
    nrf_mesh_serial_enable();
```

The @ref nrf_mesh_serial_init() API initializes the serial bearer interface, and the
@ref nrf_mesh_serial_enable() API sends the @ref SERIAL_OPCODE_EVT_DEVICE_STARTED event over the
serial interface to notify the external host that the device is ready to accept serial commands.

## Offloading feature @anchor serial_example_offloading

This example demonstrates how to offload certain Bluetooth-mesh-related tasks to the external host
using a serial interface, which improves the performance of the Bluetooth mesh stack.
The ECDH operations used in the provisioning process are a suitable candidate for such offloading.

The ECDH offloading allows the device to take advantage of the more powerful
processor of the host to perform ECDH operations during provisioning.

The offloading is enabled by calling @ref mesh_opt_prov_ecdh_offloading_set with `enabled=true`.

When this offloading is enabled, the serial interface sends
the @ref SERIAL_OPCODE_EVT_PROV_ECDH_REQUEST event to the host processor with the public
and private keys. The host processor then performs the ECDH shared secret calculation
and sends the calculated value back using the @ref SERIAL_OPCODE_CMD_PROV_ECDH_SECRET
serial command.

---

## Hardware requirements @anchor serial_example_hw

You need one of the compatible development kits for this example.
See @ref md_doc_user_guide_mesh_compatibility for more information.

---

## Setup @anchor serial_example_setup

You can find the source code of the serial example in the following folder: `<InstallFolder>/examples/serial`

---

## Testing the example @anchor serial_example_testing

To test the serial example:
1. Build the example by following the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).
2. Program the board by following the instructions in
@ref md_doc_getting_started_how_to_run_examples.
3. When the the serial example is running, start testing it using the Interactive PyACI.
Refer to the [Interactive PyACI documentation](@ref md_scripts_interactive_pyaci_README)
for details.
