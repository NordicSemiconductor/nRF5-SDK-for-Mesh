# EnOcean switch translator client example

@tag52840and52833and52832
@tag52810and52820nosupport

This example demonstrates:
- how a PTM215B EnOcean switch can be integrated in the Bluetooth mesh ecosystem;
- how to capture the commissioning data of the EnOcean switch;
- how to translate the EnOcean switch messages into equivalent Bluetooth mesh messages.

The example uses two instances of the [Generic OnOff client model](@ref GENERIC_ONOFF_MODEL).
These on/off clients can be configured to control desired servers by the provisioner.

The following figure shows such a hybrid network that contains a device outside
of the Bluetooth mesh network, a translator client, and servers. All servers have
the relay functionality enabled, which allows the EnOcean switch to control the state
of any of the servers in the network.

![Integrating EnOcean switches in the Bluetooth mesh network](images/example_nw_config.png "Integrating EnOcean switches in the Bluetooth mesh network")

The translator client linked with the EnOcean switch has a provisionee role in the network.
The client receives messages from the PTM215B switch and converts them to
equivalent on/off client messages to control the state of LED 1 on servers.
The client instantiates two instances of the Generic OnOff Client model (A and B, each
with two buttons) and can either be provisioned and configured by
the [provisioner example](@ref md_examples_provisioner_README)
or by a GATT-based provisioner (@link_nrf_mesh_app).


**Table of contents**
- [Hardware requirements](@ref enocean_example_requirements_hw)
- [Software requirements](@ref enocean_example_requirements_sw)
- [Setup](@ref enocean_example_setup)
    - [LED and button assignments](@ref enocean_example_setup_assignments)
- [Testing the example](@ref enocean_example_setup_testing)
    - [Evaluating using the static provisioner](@ref enocean_example_testing_dk)
    - [Evaluating using the nRF Mesh mobile app](@ref enocean_example_testing_app)
    - [Capturing the commissioning data of the EnOcean switch](@ref enocean_example_testing_capturing)
    - [Interacting with the boards](@ref enocean_example_testing_interacting)


---


## Hardware requirements @anchor enocean_example_requirements_hw

You need at least two compatible development kits for this example:
- One compatible development kit for the EnOcean switch.
- One or more compatible development kits for the servers.

Additionally, you need one of the following for provisioning:
- One compatible development kit for the provisioner if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
- An iOS or Android smartphone if you decide to provision using the @link_nrf_mesh_app mobile application.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.


---


## Software requirements @anchor enocean_example_requirements_sw

To test this example, you need to use the server example from the [Light switch example](@ref md_examples_light_switch_README) folder,
regardless of the number of server boards you use: `<InstallFolder>/examples/light_switch/server`.
See [server details](@ref md_examples_light_switch_server_README) for more information about the API usage.

The example is configured to store security material for two EnOcean switches.
If you want this example to support more than two EnOcean switches in parallel, set the value
of `MAX_ENOCEAN_DEVICES_SUPPORTED` to the desired number of switches.

Depending on the provisioning method:
- If you decide to provision using a mobile application, you need @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) installed on the smartphone.
- If you decide to use the static provisioner example, you need the [provisioner example](@ref md_examples_provisioner_README).


---

## Setup @anchor enocean_example_setup

You can find the source code of the EnOcean example in the following folder:
`<InstallFolder>/examples/enocean_switch`


### LED and button assignments @anchor enocean_example_setup_assignments

- Server
    - During provisioning process:
        - LED 3 and 4 blinking: Device identification active.
		- LED 1 to 4: Blink four times to indicate provisioning process is completed.
    - After provisioning and configuration is over:
        - LED 1: Reflects the value of OnOff state on the server.
				- LED ON: Value of the OnOff state is 1 (`true`).
				- LED OFF: Value of the OnOff state is 0 (`false`).
- EnOcean client
    - During provisioning process:
        - LED 3 and 4 blinking: Device identification active.
		- LED 1 to 4: Blink four times to indicate provisioning process is completed.
    - Capturing commissioning telegrams:
        - LED 1 to 4: Blink four times to indicate a commissioning telegram is captured.
    - For node reset:
        - Button 4 (on the board): Reset the node by erasing Bluetooth mesh and application data.
        - LED 1: Blinks twice to indicate node reset is being executed.
- Provisioner
    - Button 1: Start provisioning.
    - LED 1: Reflects the state of the provisioning.
        - LED ON: Provisioning of the node is in progress.
        - LED OFF: No ongoing provisioning process.
    - LED 2: Reflects the state of the configuration.
        - LED ON: Configuration of the node is in progress.
        - LED OFF: No ongoing configuration process.


---

## Testing the example @anchor enocean_example_setup_testing

To test the EnOcean switch example, build the examples by following the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref enocean_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref enocean_example_testing_app)

Once the provisioning is complete, you can start [interacting with the boards](@ref enocean_example_testing_interacting).

### Evaluating using the static provisioner @anchor enocean_example_testing_dk

See [provisioner example testing section](@ref provisioner_example_evaluating) for detailed steps required
to provision and configure the boards using the static provisioner.

There are two additional steps specific to this example:
- Option to start [capturing the commissioning data](@ref enocean_example_testing_capturing)
- Connecting [RTT viewer](@ref segger-rtt) to view the RTT output generated by the translator client
and the provisioner.

### Evaluating using the nRF Mesh mobile app @anchor enocean_example_testing_app

See [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app) for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

When using the nRF Mesh app with this example, take the following information into account:
- After flashing the examples, you can decide whether to start [capturing the commissioning data](@ref enocean_example_testing_capturing).
This can also be done after the translator client provisioning.
- The following naming convention is used in the app:
    - The switch board is `nRF5x Mesh Enocean Translator`.
    - The server board is `nRF5x Mesh Light`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Light` server board: Generic On Off Server.
- For the `nRF5x Mesh Enocean Translator` switch board: Generic On Off Client.


### Capturing the commissioning data of the EnOcean switch @anchor enocean_example_testing_capturing

@note These steps can be done either before or after the translator client
has been provisioned.

To capture the commissioning data of the EnOcean switch, put the EnOcean switch in
the radio-based commissioning mode:
1. Make sure that the Disable Radio Commissioning flag in the Configuration register
of the NFC interface is set to `0b0` (default state).
2. Start by selecting one button contact of PTM 215B. Any button of
PTM 215B (A0, A1, B0, B1) can be used.
3. Execute the following sequence:
	1. Press and hold the selected button for more than 7 seconds before releasing it.
	2. Press the selected button quickly (hold for less than 2 seconds).
	3. Press and hold the selected button again for more than 7 seconds before releasing it.
    Upon detection of this sequence, PTM 215B enters the commissioning mode. See @link_enocean_ds
    for reference.

Once entered the radio commissioning mode, PTM215B transmits the commissioning telegrams. These
telegrams will be captured by the translator, and the security material contained within those
telegrams will be stored in the flash.

Once one of the commissioning telegrams is captured by the translator, 4 LEDs will blink 4 times.

@note
The example supports two EnOcean switches to be connected in parallel. You can repeat
the commissioning configuration steps to commission the second switch.
From the Bluetooth mesh network's perspective, both switches will be seen as one device,
and their messages will be forwarded through the same Generic OnOff clients.

### Interacting with the boards @anchor enocean_example_testing_interacting

Once the provisioning and the configuration of the EnOcean translator client node and at least one
of the server nodes are complete and the commissioning data is captured, you can press
rocker switches on the EnOcean switch to control various servers.
See [LED and button assignments](@ref enocean_example_setup_assignments) section.

If any of the devices is powered off and back on, it will remember its flash configuration
and rejoin the network. For more information about the flash manager, see @ref md_doc_user_guide_modules_flash_manager.

If you want to reset the application data without re-flashing the firmware, press Button 4.
LED 1 will blink twice to indicate that application-specific data and data related to Bluetooth mesh has been erased.
Press Reset button to reset the board and start the application.