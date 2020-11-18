# Light CTL example

@tag52840and52833and52832
@tag52810and52820nosupport

This example demonstrates how you can use Bluetooth mesh messages and events
from the [Light CTL models](@ref LIGHT_CTL_MODELS) API to implement a tunable white light.

The example is composed of three minor examples that use the Light CTL Client and Light CTL Server models:
- Light CTL client model example
- Light CTL server model example
- Light CTL server with Light LC server models' example

For more information about the Light CTL Client and Server models, see the @link_ModelOverview from Bluetooth SIG.

For provisioning purposes, the example requires either [the provisioner example](@ref md_examples_provisioner_README)
or the @link_nrf_mesh_app.

All the minor examples have the provisionee role in the network.
They support provisioning over Advertising bearer (PB-ADV) and GATT bearer (PB-GATT)
and also support Bluetooth mesh Proxy Service (Server).
Read more about the Proxy feature in @ref md_doc_user_guide_modules_provisioning_gatt_proxy.

**Table of contents**
- [Light CTL client model example](@ref light_ctl_example_light_ctl_client)
- [Light CTL server model example](@ref light_ctl_example_light_ctl_server)
    - [Scene model](@ref light_ctl_example_light_ctl_server_scene_model)
- [Light CTL Server and Light LC Server models' example](@ref light_ctl_example_light_ctl_lc_server)
- [Light CTL models](@ref light_ctl_example_light_ctl_model)
- [Hardware requirements](@ref light_ctl_example_hw_requirements)
- [Software requirements](@ref light_ctl_example_sw_requirements)
- [Setup](@ref light_ctl_example_setup)
    - [LED and button assignments](@ref light_ctl_example_setup_leds_buttons)
    - [Scene model integration](@ref light_ctl_example_setup_scene_model)
- [Testing the example](@ref light_ctl_example_testing)
    - [Evaluating using the static provisioner](@ref light_ctl_example_testing_dk)
    - [Evaluating using the nRF Mesh mobile app](@ref light_ctl_example_testing_app)
    - [Interacting with the boards](@ref light_ctl_example_testing_interacting)
        - [Controlling CTL lightness, CTL temperature, and CTL delta UV values](@ref light_ctl_example_controlling_lightness_value)
        - [Changing behavior on power-up](@ref light_ctl_example_changing_behavior_on_powerup)
        - [Restricting the range of the CTL temperature value](@ref light_ctl_example_changing_temperature_range)
        - [Restricting the range of the CTL lightness value](@ref light_ctl_example_changing_lightness_range)
        - [Other factory default configuration](@ref light_ctl_example_other_factory_default_configuration)


---
## Light CTL client model example @anchor light_ctl_example_light_ctl_client

The light CTL client model example has a provisionee role in the network.
It implements two instances of the [Light CTL Client model](@ref LIGHT_CTL_CLIENT).
These instances are used to control the brightness of the LEDs on the servers,
the range of supported CTL temperature levels, and the CTL lightness, temperature, and delta UV
values after the servers' boot-up.

---
## Light CTL server model example @anchor light_ctl_example_light_ctl_server

The light CTL server model example has a provisionee role in the network.
It implements one instance of the [Light CTL Setup Server model](@ref LIGHT_CTL_SETUP_SERVER),
and [Light Lightness Setup Server model] (@ref LIGHT_LIGHTNESS_SETUP_SERVER). These instances
in-turn initialize other necessary models.

The Light Lightness Server model is used by Light CTL Server to control the lightness value.
The Light CTL Server model is used to receive the lightness, temperature and delta UV values.
The received values are used to demonstrate a tunable white light by means of the
two LEDs on the DK board. The LED 1 represents a warm light, while LED 2
represents a cool white light. These two LEDs are controlled with different PWM duty cycles
corresponding to the given color temperature. Also, the overall light output is
scaled by the given lightness level to achieve dimming. The photometrically accurate
implementation of the tunable white light is out of scope of this example.

The model instance uses the @link_APP_PWM library from the nRF5 SDK to control
the brightness of the LEDs. To demonstrate a tunable white light, the lightness and
the temperature values are converted to appropriate brightness level for each LED. These
are then converted to the generic level values to map them to the PWM tick values for each LED.

![Light CTL example structure](ctl_example_structure.svg)

### Scene model @anchor light_ctl_example_light_ctl_server_scene_model

The light CTL server model example also implements one instance of the [Scene Setup Server model](@ref SCENE_SETUP_SERVER).
The Scene Setup Server instance can be used together with the [Scene Client](@ref SCENE_CLIENT),
although both model instances are optional and [can be excluded](@ref light_ctl_example_setup_scene_model).
The Scene Server model uses the Default Transition Time Server instance instantiated
in the Light CTL Setup Server model instance.

For the values stored and recalled by the Scene model, see the @tagMeshMdlSp, Table 6.123.
For more information on how to use the [Scene models](@ref SCENE_MODELS),
see the [scene example](@ref md_examples_scene_README).

---
## Light CTL server and light LC server models' example @anchor light_ctl_example_light_ctl_lc_server

The light CTL server and light LC server models' example has a provisionee role in the network.
It implements one instance of the [Light CTL Setup Server model](@ref LIGHT_CTL_SETUP_SERVER),
a [Light LC Setup Server model](@ref LIGHT_LC_SETUP_SERVER), and other necessary models in such
a way that light LC server is able to control the lightness output. This shows how a tunable white
light can be implemented to have a feature of automated lighting control.

The Light CTL Server model instance is used to receive the CTL lightness, temperature, and
delta UV values and change the brightness and color temperature of the lights. The LC model
adds the capability for automated lightness control handled by the LC FSM and
PI regulator, based on predefined settings and sensor inputs. Refer to
[light LC server model example](@ref light_lc_server_example_testing_interacting) for information about
how a light LC server works.

The hardware interface of this example is similar to the light CTL server model example.

---
## Light CTL models @anchor light_ctl_example_light_ctl_model

The Light CTL Client model is used for manipulating the following states
associated with the peer Light CTL Server, Light CTL Setup Server, and Light CTL Temperature
Server models:
- Light CTL Lightness
- Light CTL Temperature
- Light CTL Delta UV
- Light CTL Lightness, Temperature, and Delta UV Default
- Light CTL Temperature Range

@note The Light CTL Lightness is bound with Light Lightness Actual state. The received Light
Lightness state value is represented in the form of Light CTL Lightness value as a result of
this binding.

More information about the Light CTL models can be found in the
[Light CTL models documentation](@ref LIGHT_CTL_MODELS).


---


## Hardware requirements @anchor light_ctl_example_hw_requirements

You need at least two compatible development kits for this example:

- One compatible development kit for the client.
- One or more compatible development kits for the servers.

  On the server boards, you can either run the light CTL server model example or
  the light CTL server with light LC server models' example, or both of them.

  If you choose the light CTL server with light LC server models' example, you also need at least one of the
  following:
  - One compatible development kit for emulating the occupancy sensor.
  - One compatible development kit for the [light switch client](@ref light_switch_demo_client).

  See [light LC server model example testing](@ref light_lc_server_example_testing) section
  for more information about how the light switch client and sensor example is used for testing
  the Light LC Server model.

Additionally, you need one of the following for provisioning:
- One compatible development kit for the provisioner if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
- An iOS or Android smartphone if you decide to provision using the @link_nrf_mesh_app mobile application.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

@note This example uses the PWM peripheral to control the brightness of the LEDs.
For this reason, it cannot be run on nRF51 devices,
even after solving the issues related to their [deprecated compatibility](@ref compatibility_nRF51).


---

## Software requirements @anchor light_ctl_example_sw_requirements

Depending on the provisioning method:
- If you decide to provision using a mobile application, you need @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) installed on the smartphone.
- If you decide to use the static provisioner example, you need the [provisioner example](@ref md_examples_provisioner_README).


---

## Setup @anchor light_ctl_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/light_ctl`


### LED and button assignments @anchor light_ctl_example_setup_leds_buttons

- Light CTL server model example:
    - LED 1: Represents a warm white LED and reflects the effective lightness and color temperature.
    - LED 2: Represents a cool white LED and reflects the effective lightness and color temperature.
    When interacting with the boards, you cannot use the buttons on the server boards, because the light lightness server model example
    does not use the `simple_hal` module. Instead of the buttons on the server boards, use the following RTT input:
      | RTT input     | DK Button     |   Effect                                                             |
      |---------------|---------------|----------------------------------------------------------------------|
      | `1`           | -             | The brightness of the LEDs is decreased in a large step.             |
      | `2`           | -             | The brightness of the LEDs is increased in a large step.             |
      | `3`           | -             | The delta UV value is increased in large steps. The value wraps around when maximum value is reached. |
      | `4`           | -             | All Bluetooth mesh data is erased and the device is reset.           |

- Light CTL and light LC server models' example:
    - LED 1: Represents a warm white LED and reflects the effective lightness and color temperature.
    - LED 2: Represents a cool white LED and reflects the effective lightness and color temperature.
    When interacting with the boards, you cannot use buttons on the server boards, because the light CTL and light LC server model example
    does not use the `simple_hal` module. Instead of the buttons on the server boards, use the following RTT input:
      | RTT input     | DK Button     |   Effect                                                                 |
      |---------------|---------------|--------------------------------------------------------------------------|
      | `1`           | -             | Toggles the values of the properties between 0 and the default values.   |
      | `2`           | -             | The color temperature of the LEDs is increased in large steps. The value wraps around when maximum value is reached. |
      | `4`           | -             | All Bluetooth mesh data is erased and the device is reset.               |

        - When sending the `1` RTT command, the following properties are toggled between 0 and the default values:
          - Light Control Time Fade
          - Light Control Time Fade On
          - Light Control Time Fade Standby Auto
          - Light Control Time Fade Standby Manual
          - Light Control Time Run On

          See Section 4.1.3 of the @tagMeshDevPr, @link_MeshProperties, and
          @link_MeshCharacteristics for more information about the properties.
    @note As Light LC Server is used for automated control of the lightness, buttons to change
    the LED brightness manually (that is, by changing the lightness) are not provided.
    Chaning the brightness manually will switch off the Light LC Server
    (by setting Light LC Mode state to `0`) as soon as the Light
    Lightness Status message is published on account of the local state change.

- Light CTL client model example:
  - When interacting with the boards, you can use one of the following options:
      - RTT input (recommended): Due to a limited number of buttons on the DK board,
      use the following RTT input when testing this example:
        | RTT input     | DK Button     |   Effect                                                                                                    |
        |---------------|---------------|-------------------------------------------------------------------------------------------------------------|
        | `1`           | Button 1      | The CTL lightness value is increased in large steps and the Light CTL Set Unacknowledged message is sent. |
        | `2`           | Button 2      | The CTL lightness value is decreased in large steps and the Light CTL Set Unacknowledged message is sent. |
        | `3`           | Button 3      | The CTL temperature value is increased_ in large steps and Light CTL Temperature Set Unacknowledged message is sent. |
        | `4`           | Button 4      | The CTL temperature value is decreased in large steps and Light CTL Temperature Set Unacknowledged message is sent. |
        | `5`           | -             | The CTL delta UV value is increased in large steps and Light CTL Temperature Set Unacknowledged message is sent.    |
        | `6`           | -             | The CTL delta UV value is decreased in large steps and Light CTL Temperature Set Unacknowledged message is sent.    |
        | `7`           | -             | The Light CTL Get message is sent to request the Light CTL state value.                                                    |
        | `8`           | -             | The Light CTL Temperature Get message is sent to request the Light CTL temperature value.                                  |
        | `9`           | -             | The Light CTL Temperature Range Get message is sent to request the Light CTL temperature range value.                      |
        | `a`           | -             | The Light Lightness Default Get message is sent to request the Light CTL default values.                                   |
        | `b`           | -             | The CTL lightness value is increased in large steps and the Light Lightness Default Set message is sent.   |
        | `c`           | -             | The CTL lightness value is decreased in large steps and the Light Lightness Default Set message is sent.   |
        | `d`           | -             | The CTL temperature value is increased in large steps and the Light Lightness Default Set message is sent. |
        | `e`           | -             | The CTL temperature value is decreased in large steps and the Light Lightness Default Set message is sent. |
        | `f`           | -             | The CTL delta UV value is increased in large steps and the Light Lightness Default Set message is sent.    |
        | `g`           | -             | The CTL delta UV value is decreased in large steps and the Light Lightness Default Set message is sent.    |
        | `h`           | -             | The internal minimum value of light CTL temperature range is increased and the Light CTL Temperature Range Set Unacknowledged message is sent. |
        | `i`           | -             | The internal minimum value of light CTL temperature range is decreased and the Light CTL Temperature Range Set Unacknowledged message is sent. |
        | `j`           | -             | The internal maximum value of light CTL temperature range is increased and the Light CTL Temperature Range Set Unacknowledged message is sent. |
        | `k`           | -             | The internal maximum value of light CTL temperature range is decreased and the Light CTL Temperature Range Set Unacknowledged message is sent. |
        | `l`           | -             | Switches the client instance to be used for sending messages.                                                              |
      - Buttons: If you decide to use the buttons on the DK instead of the RTT input, you can only
      change the Light CTL Lightness and Light CTL Temperature states.

### Scene model integration @anchor light_ctl_example_setup_scene_model

[Scene Setup Server model instance](@ref light_ctl_example_light_ctl_server_scene_model) is used by default by this example.
You can exclude it by setting @ref SCENE_SETUP_SERVER_INSTANCES_MAX to `0` (from the default value of `1`) in
`examples/light_ctl/ctl_server/include/nrf_mesh_config_app.h`.

If you decide to exclude the Scene Setup Server model instance, exclude it also from the
[provisioner example](@ref provisioner_example_no_scene_setup_server) if you want
to [evaluate using the static provisioner](@ref light_ctl_example_testing_dk).

---

## Testing the example @anchor light_ctl_example_testing

To test the light CTL example, build the examples by following the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).

@note
The @link_ModelSpec mentions that the default value of the mode of the light controller should be set
to `(0x0)`. This means that the light controller is turned off by default.
To enable the light controller, the Light LC Client model is used.
However, this SDK does not provide the light LC client example.
For this reason, in this example the light controller is switched on by default.
This has been done by changing the default value of the @ref LIGHT_LC_DEFAULT_MODE in
`nrf_mesh_config_app.h` to `(0x1)`.

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref light_ctl_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref light_ctl_example_testing_app)

### Evaluating using the static provisioner @anchor light_ctl_example_testing_dk

See [provisioner example testing section](@ref provisioner_example_evaluating) for detailed steps required
to provision and configure the boards using the static provisioner.

### Evaluating using the nRF Mesh mobile app @anchor light_ctl_example_testing_app

See [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app) for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

The following naming convention is used in the app:
- Each server board is either `nRF5x Mesh Light CTL Setup Server` or `nRF5x Mesh Light CTL+LC Setup Server`.
- The client board is `nRF5x Mesh Light CTL Client`.

The following model instances must be configured in the app for evaluating this example:
- For the `nRF5x Mesh CTL Client` boards: Light CTL Client.
- For the `nRF5x Mesh Light CTL Setup Server` boards:
    - Mandatory: Light CTL Setup Server, Light CTL Server
    - Optional (with [Scene model integration included](@ref light_ctl_example_setup_scene_model)):
    Scene Setup Server, Scene Server
- For the `nRF5x Mesh Light CTL+LC Setup Server` boards: Light CTL Setup Server,
  Light CTL Server, Light LC Server.
  Refer to [light LC example configuration using nRF Mesh mobile app section]
  (@ref light_lc_server_example_testing_app) for configuring `nRF5x Mesh Switch` and
  `nRF Mesh Sensor Server` boards to work with Light LC server instantiated on this example.


@note The light CTL client model example allows to control the Light CTL states.
For this purpose, it is enough to configure only the Light CTL Setup Server and Light CTL Server model
instances. If you want to see how the binding works between the Light CTL Lightness states and
the Generic states, configure the Generic models instantiated in the server examples
and use the appropriate clients to control the generic states.

Once the provisioning is complete, you can start [interacting with the boards]
(@ref light_ctl_example_testing_interacting).

@note
You can also configure the publish address of the second Light CTL Client model instance.
To do this, repeat [step 3 from binding nodes](@ref nrf-mesh-mobile-app-binding) and
[all steps from setting publication](@ref nrf-mesh-mobile-app-publication).


### Interacting with the boards @anchor light_ctl_example_testing_interacting

Once the provisioning and the configuration of the client node and of at least one of
the server nodes are complete, you can press buttons on the client or send command
numbers using the RTT Viewer to observe the changes in the brightness and emulated color
temperature of the LEDs on the corresponding server boards.

The following set of message types is available for this demonstration:
- Light CTL Set Unacknowledged
- Light CTL Get
- Light CTL Temperature Set Unacknowledged
- Light CTL Temperature Get
- Light CTL Delta UV Set Unacknowledged
- Light CTL Delta UV Get
- Light CTL Default Set Unacknowledged
- Light CTL Default Get
- Light CTL Temperature Range Set Unacknowledged
- Light CTL Temperature Range Get

See [LED and button assignments](@ref light_ctl_example_setup_leds_buttons)
section for the full list of available commands.

If any of the devices is powered off and then back on, it will remember its flash configuration
and rejoin the network. It will also restore values of the Light CTL states.
For more information about the flash manager, see @ref md_doc_user_guide_modules_flash_manager.

#### Controlling CTL lightness, CTL temperature, and CTL delta UV values @anchor light_ctl_example_controlling_lightness_value

You can control the Lightness and the Color Temperature states on the server
using the RTT commands `1` - `4` or the buttons 1 - 4 on the client board.
Use the RTT commands `7`,`8`,`9`, and `a` on the client board to retrieve the current
CTL state values from the servers.

@note If you are using the Light CTL server with LC server models' example, the light controller
is switched off automatically by the Light LC Server as soon any Bluetooth mesh message to change
the lightness value is received (for example, the Light CTL Set (that can change
the lightness) or any other message that can change the bound lightness state value).
Use a Light LC Client model to turn the light controller on again.

#### Changing behavior on power-up @anchor light_ctl_example_changing_behavior_on_powerup

You can change how the light CTL lightness value and the light CTL temperature value are restored
during a power-up sequence.

The Light CTL Lightness state is bound to Light Lightness Actual state to reflect each other's values
when these states are changed (see section 6.1.3.6.1 of @link_ModelSpec). Therefore, the power-up
behavior of the light CTL lightness, temperature, and delta UV values can be changed by controlling
the Generic OnPowerUp state instantiated by the Light Lightness Setup Server model.

See [Changing behavior on power-up] (@ref light_lightness_example_changing_behavior_on_powerup)
section in the light lightness example documentation to check how light CTL lightness value is
restored upon a power-up.

The following table demonstrates how the light CTL temperature and delta UV values are restored:
| Value (power-up) | Temperature value                                                 | Delta UV value |
|------------------|-------------------------------------------------------------------|----------------|
| 0 or 1           | The value of the Light CTL Temperature Default state is used.     | The value of the Light CTL Delta UV Default state is used.     |
| 2                | Last known value for the light CTL temperature before power-down. | Last known value for the light CTL delta UV before power-down. |

Use the Light CTL Client model board to change the Light CTL Temperature Default and the Light CTL
Delta UV Default state:
- Use the RTT commands `b` and `c` to change the Light Lightness Default state.
- Use the RTT commands `d` and `e` to change the Light CTL Temperature Default state.
- Use the RTT commands `f` and `g` to change the Light CTL Delta UV Default state.
- Use the RTT command `a` to retrieve the current default values of the servers that you use.

See [LED and button assignments](@ref light_ctl_example_setup_leds_buttons) for additional commands.

The factory default values for these states are controlled through the following defines:
- @ref LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP
- @ref LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_DEFAULT
- @ref LIGHT_CTL_DEFAULT_TEMPERATURE
- @ref LIGHT_CTL_DEFAULT_DELTA_UV_DEFAULT

If you want to edit the factory default values, do this in `nrf_mesh_config_app.h` of the
server example you are using.
Follow the instructions in [Testing the example](@ref light_ctl_example_testing)
to rebuild and reprovision the example.

#### Restricting the range of the CTL temperature value @anchor light_ctl_example_changing_temperature_range

You can restrict the range of the CTL temperature value by changing the Light CTL Temperature Range
state. The new value of the Light CTL Range state will be reflected in the Light CTL Lightness
state at the next lightness value change.

Use RTT commands `h`, `i`, `j`, and `k` on the client board to change the Light CTL Temperature
Range state, and the RTT command `6` to retrieve the current range.
See [LED and button assignments](@ref light_ctl_example_setup_leds_buttons) for additional commands.

The factory default values for the minimum and maximum possible range values are controlled through
@ref LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN and @ref LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX
values in the `nrf_mesh_config_app.h` file of the server examples.

#### Restricting the range of the CTL lightness value @anchor light_ctl_example_changing_lightness_range

As the Light CTL Lightness state is bound with the Light Lightness Actual state, you can restrict
the range of the light CTL lightness values by changing the Light Lightness Range state using
the Light Lightness Client. See [Restricting the range of the lightness value]
(@ref light_lightness_example_changing_lightness_range) section in the light lightness example
documentation.

#### Other factory default configuration @anchor light_ctl_example_other_factory_default_configuration

In addition to the parameters described in the previous sections, you can also set the factory default
transition time in milliseconds when changing the lightness levels.

The transition time used by the Light CTL Server model uses the Default Transition Time state
instance that belongs to the the Light Lightness Server model.
For this reason, to change the factory default transition time
for the server model example, redefine the @ref LIGHT_LIGHTNESS_DEFAULT_DTT value of the
Generic Default Transition Time state in the `nrf_mesh_config_app.h` file of the server examples.
