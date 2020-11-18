# Light lightness example

@tag52840and52833and52832
@tag52810and52820nosupport

This example demonstrates how you can use Bluetooth mesh messages and events
from the [Light Lightness model](@ref LIGHT_LIGHTNESS_MODELS) API
to control the brightness of the LED on your board.

The example is composed of two minor examples that use the Light Lightness Client and Setup Server model:
- Light lightness server model example
- Light lightness client model example

For more information about the Light Lightness Client and Server model, see also the Bluetooth SIG's @link_ModelOverview.

For provisioning purposes, the example requires either the provisioner example that is provided in the @ref md_examples_provisioner_README
or the nRF Mesh mobile app.

Both the light lightness server and light lightness client examples have the provisionee role
in the network.
They support provisioning over Advertising bearer (PB-ADV) and GATT bearer (PB-GATT)
and also support Bluetooth mesh Proxy Service (Server).
Read more about the Proxy feature in @ref md_doc_user_guide_modules_provisioning_gatt_proxy.

**Table of contents**
- [Light lightness client model example](@ref light_lightness_example_light_lightness_client)
- [Light lightness server model example](@ref light_lightness_example_light_lightness_server)
    - [Scene model](@ref light_lightness_example_light_lightness_server_scene_model)
- [Light Lightness Client and Setup Server model](@ref light_lightness_example_light_lightness_model)
- [Hardware requirements](@ref light_lightness_example_hw_requirements)
- [Software requirements](@ref light_lightness_example_sw_requirements)
- [Setup](@ref light_lightness_example_setup)
    - [LED and button assignments](@ref light_lightness_example_setup_leds_buttons)
    - [Scene model integration](@ref light_lightness_example_setup_scene_model)
- [Testing the example](@ref light_lightness_example_testing)
    - [Evaluating using the static provisioner](@ref light_lightness_example_testing_dk)
    - [Evaluating using the nRF Mesh mobile app](@ref light_lightness_example_testing_app)
    - [Interacting with the boards](@ref light_lightness_example_testing_interacting)
        - [Controlling the lightness value](@ref light_lightness_example_controlling_lightness_value)
        - [Changing behavior on power-up](@ref light_lightness_example_changing_behavior_on_powerup)
        - [Restricting the range of the lightness value](@ref light_lightness_example_changing_lightness_range)
        - [Other factory default configuration](@ref light_lightness_example_other_factory_default_configuration)


---

![Light lightness example structure](ll_example_structure.png)

## Light lightness client model example @anchor light_lightness_example_light_lightness_client

The light lightness client model example has a provisionee role in the network.
It implements two instances of the [Light Lightness Client model](@ref LIGHT_LIGHTNESS_CLIENT).
These instances are used to control the brightness of the LED 1 on the servers,
the range of supported lightness levels, and the default lightness value after the servers' boot-up.

---
## Light lightness server model example @anchor light_lightness_example_light_lightness_server

The light lightness server model example has a provisionee role in the network.
It implements one instance of the [Light Lightness Setup Server model](@ref LIGHT_LIGHTNESS_SETUP_SERVER).

This model instance is used to receive the lightness level and change
the brightness of the LED 1 on the server board, whenever the Light Lightness Actual
or Light Lightness Linear state is changed. A change in the Light Lightness Actual state is
reflected in the Light Lightness Linear state, and the other way around.

The model instance uses the @link_APP_PWM library of the nRF5 SDK to control
the brightness of the LED. To map the lightness level to the allowed range of
the PWM ticks, the value of the Light Lightness Actual state is converted to the value
of the Generic Level state.

### Scene model @anchor light_lightness_example_light_lightness_server_scene_model

The light lightness server model example also implements one instance of the [Scene Setup Server model](@ref SCENE_SETUP_SERVER).
The Scene Setup Server instance can be used together with the [Scene Client](@ref SCENE_CLIENT),
although both model instances are optional and [can be excluded](@ref light_lightness_example_setup_scene_model).
The Scene Server model uses the Default Transition Time Server instance instantiated
in the Light Lightness Setup Server model instance.

For the values stored and recalled by the Scene model, see the @tagMeshMdlSp, Table 6.118.
For more information on how to use the [Scene models](@ref SCENE_MODELS),
see the [scene example](@ref md_examples_scene_README).


---
## Light Lightness Client and Setup Server model @anchor light_lightness_example_light_lightness_model

The Light Lightness Client model is used for manipulating the following states
associated with the peer Light Lightness Setup Server model:
- Light Lightness Actual
- Light Lightness Linear
- Light Lightness Default
- Light Lightness Last
- Light Lightness Range

More information about the Light Lightness models can be found in the
[Light Lightness model documentation](@ref LIGHT_LIGHTNESS_MODELS).


---


## Hardware requirements @anchor light_lightness_example_hw_requirements

You need at least two compatible development kits for this example:

- One compatible development kit for the client.
- One or more compatible development kits for the servers.

Additionally, you need one of the following for provisioning:
- One compatible development kit for the provisioner if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
- An iOS or Android smartphone if you decide to provision using the @link_nrf_mesh_app mobile application.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

@note This example uses the PWM peripheral to control the brightness of the LED.
For this reason, it cannot be run on nRF51 devices,
even after solving the issues related to their [deprecated compatibility](@ref compatibility_nRF51).


---

## Software requirements @anchor light_lightness_example_sw_requirements

Depending on the provisioning method:
- If you decide to provision using a mobile application, you need @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) installed on the smartphone.
- If you decide to use the static provisioner example, you need the [provisioner example](@ref md_examples_provisioner_README).


---

## Setup @anchor light_lightness_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/light_lightness`


### LED and button assignments @anchor light_lightness_example_setup_leds_buttons

- Server:
  - LED 1: Reflects the value of the Light Lightness Actual state on the server.
  - When interacting with the boards:
    - You cannot use buttons on the server boards, because the light lightness setup server example
    does not use the `simple_hal` module.
    - Instead of the buttons on the server boards, use the following RTT input:
      | RTT input     | DK Button     |   Effect                                                                            |
      |---------------|---------------|-------------------------------------------------------------------------------------|
      | `1`           | -             | The lightness value for LED 1 (and its brightness) is _decreased_ in large step.    |
      | `2`           | -             | The lightness value for LED 1 (and its brightness) is _increased_ in large step.    |
      | `4`           | -             | All Bluetooth mesh data is erased and the device is reset.                          |

- Client:
    - When interacting with the boards, you can use one of the following options:
        - RTT input (recommended): Due to a limited number of buttons on the DK board,
        use the following RTT input when evaluating this example:
            | RTT input     | DK Button     | Effect                                                                                                                                  |
            |---------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------|
            | `1`           | Button 1      | The actual lightness value is _increased_ in large steps and the Light Lightness Set Unacknowledged message is sent.                      |
            | `2`           | Button 2      | The actual lightness value is _decreased_ in large steps and the Light Lightness Set Unacknowledged message is sent.                      |
            | `3`           | Button 3      | The actual lightness value in a linear scale is _increased_ in large steps and Light Lightness Linear Set Unacknowledged message is sent. |
            | `4`           | Button 4      | The actual lightness value in a linear scale is _decreased_ in large steps and Light Lightness Linear Set Unacknowledged message is sent. |
            | `5`           | -             | The Light Lightness Last Get message is sent to request the last lightness value.                                                         |
            | `6`           | -             | The Light Lightness Default Get message is sent to request the default lightness value.                                                   |
            | `7`           | -             | The Light Lightness Range Get message is sent to request the range of supported lightness levels.                                         |
            | `8`           | -             | The Light Lightness Get message is sent to request the actual lightness value.                                                            |
            | `9`           | -             | The Light Lightness Linear Get message is sent to request the actual lightness value in linear scale.                                     |
            | `a`           | -             | The default light lightness value is _increased_ and the Light Lightness Default Set Unacknowledged message is sent.                      |
            | `b`           | -             | The internal default light lightness value is _decreased_ and the Light Lightness Default Set Unacknowledged message is sent.             |
            | `c`           | -             | The internal minimum value of lightness levels range is _increased_ and the Light Lightness Range Set Unacknowledged message is sent.     |
            | `d`           | -             | The internal minimum value of lightness levels range is _decreased_ and the Light Lightness Range Set Unacknowledged message is sent.     |
            | `e`           | -             | The internal maximum value of lightness levels range is _increased_ and the Light Lightness Range Set Unacknowledged message is sent.     |
            | `f`           | -             | The internal maximum value of lightness levels range is _decreased_ and the Light Lightness Range Set Unacknowledged message is sent.     |
            | `g`           | -             | The internal actual lightness value in a linear scale is set to 0 and the Light Lightness Linear Set message is sent.                     |
            | `h`           | -             | Switches the client instance to be used for sending messages.                                                                             |
        - Buttons: If you decide to use the buttons on the DK instead of the RTT input, you can only
        change the Light Lightness Actual state by sending Light Lightness Set Unacknowledged messages.

### Scene model integration @anchor light_lightness_example_setup_scene_model

[Scene Setup Server model instance](@ref light_lightness_example_light_lightness_server_scene_model) is used by default by this example.
You can exclude it by setting @ref SCENE_SETUP_SERVER_INSTANCES_MAX to `0` (from the default value of `1`) in
`examples/light_lightness/server/include/nrf_mesh_config_app.h`.

If you decide to exclude the Scene Setup Server model instance,
exclude it also from the [Provisioner example](@ref provisioner_example_no_scene_setup_server)
if you want to [evaluate using the static provisioner](@ref light_lightness_example_testing_dk).

---

## Testing the example @anchor light_lightness_example_testing

To test the light lightness example, build the examples by following the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref light_lightness_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref light_lightness_example_testing_app)


### Evaluating using the static provisioner @anchor light_lightness_example_testing_dk

See [provisioner example testing section](@ref provisioner_example_evaluating) for detailed steps required
to provision and configure the boards using the static provisioner.

### Evaluating using the nRF Mesh mobile app @anchor light_lightness_example_testing_app

See [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app) for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

The following naming convention is used in the app:
- Each server board is `nRF5x Mesh Lightness Setup Server`.
- The client board is `nRF5x Mesh Lightness Client`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Lightness Setup Server` server boards:
    - Mandatory: Light Lightness Setup Server, Light Lightness Server
    - Optional (with [Scene model integration included](@ref light_lightness_example_setup_scene_model)):
    Scene Setup Server, Scene Server
- For the `nRF5x Mesh Lightness Client` client board: Light Lightness Client.

@note The light lightness client example allows to control the Light Lightness states.
For this purpose, it is enough to configure only the Light Lightness Setup Server and Light Lightness Server model
instances. If you want to see how the binding works between the Light Lightness states and
the Generic states, configure the generic models instantiated in the light lightness server model example
and use the appropriate clients to control the Generic states.

Once the provisioning is complete, you can start [interacting with the boards](@ref light_lightness_example_testing_interacting).

@note
You can also configure the publish address of the second Light Lightness Client model instance.
To do this, repeat [step 3 from binding nodes](@ref nrf-mesh-mobile-app-binding) and
[all steps from setting publication](@ref nrf-mesh-mobile-app-publication).


### Interacting with the boards @anchor light_lightness_example_testing_interacting

Once the provisioning and the configuration of the client node and of at least one of
the server nodes are complete, you can press buttons on the client or send command
numbers using the RTT Viewer to observe the changes in the brightness of the LED 1
on the corresponding server boards.

The following set of message types is available for this demonstration:
- Light Lightness Set Unacknowledged
- Light Lightness Get
- Light Lightness Linear Set
- Light Lightness Linear Set Unacknowledged
- Light Lightness Linear Get
- Light Lightness Default Set Unacknowledged
- Light Lightness Default Get
- Light Lightness Range Set Unacknowledged
- Light Lightness Range Get
- Light Lightness Last Get

See [LED and button assignments](@ref light_lightness_example_setup_leds_buttons)
section for the full list of available commands.

If any of the devices is powered off and then back on, it will remember its flash configuration
and rejoin the network. It will also restore values of the Light Lightness states.
For more information about the flash manager, see @ref md_doc_user_guide_modules_flash_manager.

#### Controlling the lightness value @anchor light_lightness_example_controlling_lightness_value

You can control the lightness value of the LED 1 using the RTT commands `1` - `4` or the buttons 1 - 4 on the board.
Use the RTT commands `8` and `9` to retrieve the current lightness value in the perceived (Actual) lightness or
the measured (Linear) lightness value accordingly.

To set the lightness value to `0`, use the RTT command `g`.

For more information about the difference between the Actual and the Linear lightness values,
see @link_ModelSpec appendix A.2.

#### Changing behavior on power-up @anchor light_lightness_example_changing_behavior_on_powerup

You can change how the lightness value will be restored during a power-up sequence.
This can be done by controlling the Generic OnPowerUp state instantiated by the Light Lightness Setup Server model.

The following table describes how the lightness value will be restored:
| On PowerUp value | Lightness value          |
|------------------|--------------------------|
| 0                | 0                        |
| 1                | The value of the Light Lightness Default state is used if it is not a zero. Otherwise, the Light Lightness Last state will be used. |
| 2                | Last known value for the Light Lightness Actual before power down.  |

Use the RTT commands `a` and `b` to change the Light Lightness Default state, and
the RTT commands `5` and `6` to retrieve the current last and default values.
See [LED and button assignments](@ref light_lightness_example_setup_leds_buttons) for additional commands.

The factory default values for these states are controlled through the following defines:
- @ref LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP
- @ref LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_DEFAULT
- @ref LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST
- @ref LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_ACTUAL

If you want to edit the factory default values, do this in `nrf_mesh_config_app.h` of the Light Lightness Setup Server example.
Follow instructions in [Testing the example](@ref light_lightness_example_testing)
to re-build and re-provision the example.

#### Restricting the range of the lightness value @anchor light_lightness_example_changing_lightness_range

You can restrict the range of the lightness value by changing the Light Lightness Range state.
The new value of the Light Lightness Range state will be reflected in the Light
Lightness Actual state at the next lightness value change.

Use RTT commands `c`, `d`, `e`, and `f` to change the Light Lightness Range state, and
the RTT command `7` to retrieve the current range.
See [LED and button assignments](@ref light_lightness_example_setup_leds_buttons) for additional commands.

The factory default values for the minimum and maximum possible range values are controlled through
@ref LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN and @ref LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX
values in the `nrf_mesh_config_app.h` file of the light lightness server model example.

#### Other factory default configuration @anchor light_lightness_example_other_factory_default_configuration

In addition to the parameters described in the previous sections, you can also set the factory default
transition time in milliseconds when changing the lightness levels. To do this, redefine the
@ref LIGHT_LIGHTNESS_DEFAULT_DTT value of the Generic Default Transition Time state
in the `nrf_mesh_config_app.h` file of the light lightness server model example.
