# Dimming examples

@tag52810nosupport

This example demonstrates the interaction between a mesh dimmable light and a dimmer. The mesh
dimmer controls the brightness of the mesh dimmable light.
The example consists of two minor examples:
- dimming server
- dimming client

Both these minor examples use the Generic Level Client/Server model.

For provisioning purposes, the example requires either the provisioner example that is provided in the @ref md_examples_provisioner_README
or the nRF Mesh mobile app.

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
The client accepts RTT inputs from `1` to `7` to control the state of LED 1 on the servers.
It instantiates two instances of the Generic Level Client model.
It can either be provisioned and configured by the provisioner device or by
a GATT-based provisioner.
The provisioner configures this client model instances to communicate with the
servers.

### Dimming server @anchor dimming_example_dimming_server

The dimming server has a provisionee role in the network. It instantiates one instance
of the Generic Level Server model to control the state of LED 1. It uses the @link_APP_PWM library
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

You need at least two supported development kits for this example:
- One development kit for the client.
- One or more development kits for the servers.

Additionally, you need one of the following:
- One nRF52 development kit for the provisioner if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
- @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) if you decide
to provision using the application.

See @ref md_doc_user_guide_mesh_compatibility for information about the supported development kits.

@note This example uses the PWM peripheral for dimming. Therefore, it cannot be run on nRF51 devices.


---

## Software requirements @anchor dimming_example_requirements_sw

If you decide to use the static provisioner example, you need
the provisioner example: `<InstallFolder>/examples/provisioner`

See the [provisioner details](@ref md_examples_provisioner_README) page for more information
about the provisioner example.

---

## Setup @anchor dimming_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/dimming`

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
        | `1`           | Button 1      | The internal target level variable value is _decreased_ in large steps and Generic Level Set message is sent.             |
        | `2`           | Button 2      | The internal target level variable value is _increased_ in large steps and Generic Level Set message is sent.             |
        | `3`           | Button 3      | The internal target delta level variable value is _decreased_ in large steps and Generic Level Delta Set message is sent. |
        | `4`           | Button 4      | The internal target delta level variable value is _increased_ in large steps and Generic Level Delta Set message is sent. |
        | `5`           | -             | The internal target move level variable value is _decreased_ in large steps and Generic Level Move Set message is sent.   |
        | `6`           | -             | The internal target move level variable value is _increased_ in large steps and Generic Level Move Set message is sent.   |
        | `7`           | -             | The client switches between Odd or Even group nodes.                                                                      |

      - Buttons: If you decide use the buttons on the DK instead of the RTT input, you can only
      send Set and Delta Set messages. You cannot send Move message or switch between Odd or Even groups.

- Dimming server:
  - When interacting with the boards:
    - You cannot use buttons on the server boards since dimming server does not use the `simple_hal` module.
    - Use the following RTT input:
        | RTT inputs    | DK Buttons    |   Effect                                                              |
        |---------------|---------------|-----------------------------------------------------------------------|
        | `1`           | -             | The brightness of the LED 1 _decresses_ in large steps.               |
        | `2`           | -             | The brightness of the LED 1 _increases_ in large steps.               |
        | `4`           | -             | Clear all the states to reset the node.                               |



---

## Testing the example @anchor dimming_example_testing

To test the dimming example, build the examples by following the instructions in
[Building the mesh stack](@ref md_doc_getting_started_how_to_build).

@note
If you have more than 40 boards for the server and decided to use the static provisioner example,
set `MAX_PROVISIONEE_NUMBER` (in `example_network_config.h`) to the number of boards available
and rebuild the provisioner example.

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref dimming_prov_prov_example)
- [Evaluating using the nRF Mesh mobile app](@ref dimming_prov_nrf_mesh)

### Evaluating using the static provisioner @anchor dimming_prov_prov_example

1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    1. Erase the flash of your development kits and program the SoftDevice.
    2. Flash the provisioner firmware on one board, the client firmware on another board,
    and the server firmware on the remaining board or boards.
2. After the reset at the end of the flashing process, press Button 1 on the provisioner board
to start the provisioning process:
    -# The provisioner provisions and configures the client.
    -# The provisioner configures the two client model instances on the client board to communicate
    with the Odd and Even server groups.
    -# The provisioner also provisions and configures the servers at random.
    It assigns them consecutive addresses starting with 0x301, and adds them to odd and even groups.
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

See @ref nrf-mesh-mobile-app "the information on the main Examples page" for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

When using the nRF Mesh app with this example, the following naming convention is used in the app:
- The server board is `nRF5x Mesh Dimmable Light`.
- The client board is `nRF5x Mesh Dimmer`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Dimmable Light` server board: Generic Level Server.
- For the `nRF5x Mesh Dimmer` client board: Generic Level Client.


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
The RTT input is used to emulate the button numbers `1` to `7` to send level messages. You can send
numbers from `1` to `7` via the RTT viewer and observe the changes in the brightness of the LED 1
on the corresponding server boards.

See the [LED and button assignments](@ref dimming_example_setup_leds_buttons) for the list
of available commands.

**Dimming-server interaction**<br>
See the [LED and button assignments](@ref dimming_example_setup_leds_buttons) for the list
of available commands.

If you send the commands to the server board, observe the corresponding status printed in the RTT log
of the client board.

If any of the devices are powered off and back on, they remember their configuration
in flash and rejoin the network. For more information about the flash manager, see @ref md_doc_user_guide_modules_flash_manager.