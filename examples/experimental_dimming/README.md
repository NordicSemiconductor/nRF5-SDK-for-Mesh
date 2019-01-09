# Dimming examples (experimental)

@note The example is not supported by the nRF52810 Series.

This example demonstrates a mesh network scenario where a mesh dimmer controls and changes settings of a mesh dimmable light.
It consists of two minor examples:
- dimming server
- dimming client

Both these minor examples use the Generic Level Client/Server model.
Moreover, for provisioning purposes, the example requires either the provisioner example that is provided in the @ref md_examples_light_switch_README
or the nRF Mesh mobile app.

@note
This example is experimental, meaning it is ready for use, but not qualified.

**Table of contents**
- [Dimming client](@ref dimming_example_dimming_client)
- [Dimming server](@ref dimming_example_dimming_server)
- [Generic Level Client/Server model](@ref dimming_example_generic_level)
- [Hardware requirements](@ref dimming_example_requirements_hw)
- [Software requirements](@ref dimming_example_requirements_sw)
- [Setup](@ref dimming_example_setup)
    - [LED and button assignments](@ref dimming_example_setup_leds_buttons)
- [Testing the example](@ref dimming_example_testing)
    - [Evaluating using the static provisioner](@ref dimming_prov_prov_example)
    - [Evaluating using the nRF Mesh mobile app](@ref dimming_prov_nrf_mesh)
    - [Interacting with the boards](@ref dimming_example_testing_interacting)


### Dimming client @anchor dimming_example_dimming_client

The dimming client has a provisionee role in the network.
The client accepts RTT inputs from `0` to `6` to control the state of LED 1 on the servers.
It instantiates two instances of the Generic Level Client model.
It can either be provisioned and configured by the provisioner device or by
a GATT-based provisioner.
The provisioner configures this client model instances to communicate with the
servers.

### Dimming server @anchor dimming_example_dimming_server

The dimming server has a provisionee role in the network. It instantiates one instance
of the Generic Level server model to control the state of LED 1. It uses the APP_PWM library
of the nRF5 SDK to control the brightness of the LED. It can either be provisioned and configured
by the provisioner device or by a GATT-based provisioner. The provisioner configures this
server model instance to communicate with the client model on the client board and to publish
a message when the value of the Level state changes.

The examples are based on the Generic Level model, which works with signed 16-bit level values.
Therefore, the dimming server maps this range on the allowed range of PWM tick values.
As a consequence, sending a level model message that sets the target level to 0x0000 results in a 50%
duty cycle on the PWM output when the target level is reached.

### Generic Level Client/Server model @anchor dimming_example_generic_level

The Generic Level Client model is used for manipulating the
Level state associated with peer Generic Level Server model.
Note that when the server has a publish address set (as in this example),
the server publishes information about the state changes to its publish address once the final state
is reached.

More information about the Generic Level model can be found in the
[Generic Level model documentation](@ref GENERIC_LEVEL_MODEL)
and the [Generic Level server behaviour documentation](@ref APP_LEVEL).


---

## Hardware requirements @anchor dimming_example_requirements_hw

You need at least two supported boards for this example:
- One development board for the client.
- One or more development boards for the servers.

Additionally, you need one of the following:
- One nRF5 development board for the provisioner if you decide to use the static provisioner example
from the light switch example.
- @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) if you decide
to provision using the application.

See @ref md_doc_introduction_mesh_compatibility for information about the supported boards.

@note This example uses the PWM peripheral for dimming. Therefore, it cannot be run on nRF51 devices.


---

## Software requirements @anchor dimming_example_requirements_sw

If you decide to use the static provisioner example, you need 
the light switch provisioner example: `<InstallFolder>/examples/light_switch/provisioner`

See the [Light switch example](@ref md_examples_light_switch_README) page for more information
about the provisioner example. For details about the API usage,
check the [provisioner details](@ref md_examples_light_switch_provisioner_README)
page.

---

## Setup @anchor dimming_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/experimental_dimming`

### LED and button assignments @anchor dimming_example_setup_leds_buttons

- Provisioner:
  - Button 1: Start provisioning.
  - LED 1: Reflects the state of the provisioning.
		- LED ON: Provisioning of the node is in progress.
		- LED OFF: No ongoing provisioning process.
  - LED 2: Reflects the state of the configuration.
		- LED ON: Configuration of the node is in progress.
		- LED OFF: No ongoing configuration process.

- Dimming client:
  - When interacting with the boards, you can use one of the following options:
      - RTT input (recommended): Due to limited number of buttons on the DK board, use RTT input
      when evaluating this example.
        | RTT inputs    | DK Buttons    |   Effect                                                              |
        |---------------|---------------|-----------------------------------------------------------------------|
        | `0`           | Button 1      | The internal target level variable value is _decreased_ in large steps and Generic Level Set message is sent.             |
        | `1`           | Button 2      | The internal target level variable value is _increased_ in large steps and Generic Level Set message is sent.             |
        | `2`           | Button 3      | The internal target delta level variable value is _decreased_ in large steps and Generic Level Delta Set message is sent. |
        | `3`           | Button 4      | The internal target delta level variable value is _increased_ in large steps and Generic Level Delta Set message is sent. |
        | `4`           | -             | The internal target move level variable value is _decreased_ in large steps and Generic Level Move Set message is sent.   |
        | `5`           | -             | The internal target move level variable value is _increased_ in large steps and Generic Level Move Set message is sent.   |
        | `6`           | -             | The client switches between Odd or Even group nodes.                                                                      |

      - Buttons: If you decide use the buttons on the DK instead of the RTT input, you can only
      send Set and Delta Set messages. You cannot send Move message or switch between Odd or Even groups.

- Dimming server:
  - When interacting with the boards:
    - You cannot use buttons on the server boards since dimming server does not use the `simple_hal` module.
    - Use the following RTT input:
        | RTT inputs    | DK Buttons    |   Effect                                                              |
        |---------------|---------------|-----------------------------------------------------------------------|
        | `1`           | -             | The brightness of the LED 1 _increases_ in large steps.               |
        | `0`           | -             | The brightness of the LED 1 _decresses_ in large steps.               |


---

## Testing the example @anchor dimming_example_testing

To test the dimming example, build the examples by following the instructions in
[Building the mesh stack](@ref md_doc_getting_started_how_to_build).

@note
If you have more than 30 boards for the servers and decided to use the static provisioner example,
set `SERVER_NODE_COUNT` (in `light_switch_example_common.h`) to the number of boards available
and rebuild the provisioner example.

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref dimming_prov_prov_example)
- [Evaluating using the nRF Mesh mobile app](@ref dimming_prov_nrf_mesh)

### Evaluating using the static provisioner @anchor dimming_prov_prov_example

1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    1. Erase the flash of your development boards and program the SoftDevice.
    2. Flash the light switch provisioner firmware on one board, the client firmware on another board,
    and the server firmware on the remaining board or boards.
2. After the reset at the end of the flashing process, press Button 1 on the provisioner board
to start the provisioning process:
    - The provisioner first provisions and configures the client.
    - The provisioner configures the two client model instances on the client board to communicate
    with the Odd and Even server groups.
    - Finally, the provisioner moves on to provision and configure the servers, one by one.
@note You can use [RTT viewer](@ref segger-rtt) to view the RTT output generated by the provisioner.
The provisioner prints details about the provisioning and the configuration process in the RTT log.
3. Observe that the LED 1 on the provisioner board is turned ON when provisioner is scanning
and provisioning a device.
4. Observe that the LED 2 on the provisioner board is turned ON when configuration procedure
is in progress.
5. Wait until LED 1 on the provisioner board remains ON steadily for a few seconds,
which indicates that all available boards have been provisioned and configured.


If the provisioner encounters an error during the provisioning or configuration process for one
of the nodes, you can reset the provisioner to restart the process for that node.

### Evaluating using the nRF Mesh mobile app @anchor dimming_prov_nrf_mesh

1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    1. Erase the flash of your development boards and program the SoftDevice.
    2. Flash the client firmware on one board and the server firmware on the remaining board or boards.
2. Open the nRF Mesh mobile app.
3. Provision the nodes. The client board is `nRF5x Mesh Dimmer`,
the server board is `nRF5x Mesh Dimmable Light`.
5. Bind the Generic Level client and server model instances on the nodes with the same app key:
    1. Select the Network tab.
    2. On the server board tile, tap the **Configure** button to open Node Configuration.
    3. Expand the Elements section and tap **Generic Level Server**.
    4. In the Bound App Keys section, tap the **Bind Key** button and select the app key.
    5. On the client board tile, tap the **Configure** button to open Node Configuration.
    6. Expand the Elements section and tap **Generic Level Client**.
    7. In the Bound App Keys section, tap the **Bind Key** button and select the app key.
6. In the client Node Configuration, expand the Elements section. 
7. Select the first Generic OnOff Client model instance.
8. In the Publish section, set the Publish Address to one of the following addresses of the server nodes:
    - unicast address of any server node;
    - group addresses -- if you choose this option, remember to subscribe the server nodes to these
    group addresses.

@note You can also configure the publish address of the second Generic Level client model instance
to the unicast address of any other server node.

### Interacting with the boards @anchor dimming_example_testing_interacting

When provisioning and configuration of the client node and at least one of the server nodes are complete,
you can send command numbers using the RTT viewer and see the dimming action on the LED 1 on each
of the server boards.

To evaluate this interaction, connect the RTT viewer to the dimming client.

There are three message types available for this demonstration:
- Set
- Delta Set
- Move Set

**Dimming client interaction**<br>
The RTT input is used to emulate the button numbers `0` to `6` to send level messages. You can send
numbers from `0` to `6` via the RTT viewer and observe the changes in the brightness of the LED 1
on the corresponding server boards.

See the [LED and button assignments](@ref dimming_example_setup_leds_buttons) for the list
of available commands.

**Dimming-server interaction**<br>
See the [LED and button assignments](@ref dimming_example_setup_leds_buttons) for the list
of available commands.

If you send the commands to the server board, observe the corresponding status printed in the RTT log
of the client board.

If any of the devices are powered off and back on, they remember their configuration
in flash and rejoin the network. For more information about the flash manager, see @ref md_doc_libraries_flash_manager.