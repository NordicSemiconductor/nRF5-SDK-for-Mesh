# Remote provisioning example

@tagSupportAllCompatibleBoards

This example demonstrates the PB remote models, which allows you to provision devices
that are outside of the provisioner's radio range. Normally, you have to be within radio range
of the device you want to provision, but PB remote channels your commands through the Bluetooth mesh.

This solution allows to save time and usability by allowing you to provision all the nodes
in the network after all of them have been installed, for example in an ecosystem with one
controller and a large set of preinstalled light fixtures spread on a considerable area.

@note
This example uses its own [static OOB value](@ref provisioning_provisionee_initialization).
For this reason, this example cannot be used with other examples.
For more information, see the [Examples main page](@ref example_provisioning_bearers).

The Remote provisioning example consists of two parts:
- Remote provisioning _client model_ that acts as provisioner and provisions remote devices.
- Remote provisioning _server_ that the client uses as relay for provisioning nodes that it
cannot reach directly; this is the end-point for relayed provisioning packets when
using the remote provisioning feature.

For more information about remote provisioning API, see the [PB-remote API reference](@ref PB_REMOTE).

**Table of contents**
- [Hardware requirements](@ref pb-remote_example_requirements_hw)
- [Software requirements](@ref pb-remote_example_requirements_sw)
- [Setup](@ref pb-remote_example_setup)
- [Testing the example](@ref pb-remote_example_testing)


---

## Hardware requirements @anchor pb-remote_example_requirements_hw

You need at least three compatible development kits for this example:
- One compatible development kit for the client.
- Two or more compatible development kits for the server.

The servers act as provisionees until they are included in the network, after which they will be able
to act as servers to help provision new devices.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

---

## Software requirements @anchor pb-remote_example_requirements_sw

For this example, you need to use both the remote provisioning client and the remote provisioning server.
Do not use them separately.

---

## Setup @anchor pb-remote_example_setup

You can find the source code of this example and its minor examples in the following folder:
`<InstallFolder>/examples/pb_remote`


---

## Testing the example @anchor pb-remote_example_testing

To test the remote provisioning example:
1. Build the examples by following the instructions in [Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).
2. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    1. Erase the flash of your development boards and program the SoftDevice.
    2. Flash the client firmware on one board, and the server firmware on other boards.
3. Start the [RTT](@ref segger-rtt) viewer to interact with the provisioner. The provisioner prints
details about the provisioning and the configuration process in the RTT log.
4. Provide the following commands to the client:
    1. Send `1` to start normal PB-ADV on the first unprovisioned device.
    2. Send `2N`, where `N` is the handle of the newly provisioned device, to set the handle
    of the remote provisioning server.
    3. Send `3` to start remote scanning on the current server.
    4. Send `4` to cancel remote scanning on the current server.
    5. Send `5N`, where `N` is the device number, to start remote provisioning
    of the first unprovisioned device known to the current provisioning server.

The provisionees are now provisioned remotely.