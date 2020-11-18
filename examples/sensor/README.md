# Sensor example

@tag52840and52833and52832
@tag52810and52820nosupport

This example demonstrates how to use the [Sensor model](@ref SENSOR_MODEL). It
implements an emulated Motion Sensed sensor and shows how such a sensor can
be used and controlled.

The example is composed of the following minor examples:
- Sensor server: A minimalistic server that implements a
[Sensor Server model](@ref SENSOR_MODEL), with the sensor value emulated and controllable
by buttons on the board or RTT input.
- Sensor client: A minimalistic client that implements two instances of a
[Sensor Client model](@ref SENSOR_MODEL). Buttons on the board and RTT input can be used to control
the cadence of the reports sent by the Sensor servers and to request status messages.

For more information about the Sensor Cadence state, see @link_ModelSpec, section 4.1.3. 

@note The Sensor server example can also be used together with the
[Light LC example](@ref md_examples_light_lc_server_README).

**Table of contents**
- [Hardware requirements](@ref fast_cadence_example_hw_requirements)
- [Software requirements](@ref fast_cadence_example_sw_requirements)
- [Setup](@ref fast_cadence_example_setup)
  - [Button assignments](@ref fast_cadence_example_setup_leds_buttons)
- [Testing the example](@ref fast_cadence_example_testing)
  - [Evaluating using the static provisioner](@ref fast_cadence_example_testing_dk)
  - [Evaluating using the nRF Mesh mobile app](@ref fast_cadence_example_testing_app)
  - [Interacting with the boards](@ref fast_cadence_example_testing_interaction)


---
## Hardware requirements @anchor fast_cadence_example_hw_requirements

You need at least two compatible development kits for this example:

- One compatible development kit for the Sensor client.
- One or more compatible development kits for the Sensor servers.

Additionally, you need one of the following for provisioning:
- One compatible development kit for the provisioner if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
- An iOS or Android smartphone if you decide to provision using the @link_nrf_mesh_app mobile application.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

---

## Software requirements @anchor fast_cadence_example_sw_requirements

Depending on the provisioning method:
- If you decide to provision using a mobile application, you need @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) installed on the smartphone.
- If you decide to use the static provisioner example, you need the [provisioner example](@ref md_examples_provisioner_README).


---

## Setup @anchor fast_cadence_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/sensor`

### Button assignments @anchor fast_cadence_example_setup_leds_buttons

- Sensor server:
  - When interacting with the boards, you can use one of the following options:
    - RTT input (recommended): Due to a limited number of buttons on the DK board,
    use the following RTT input when evaluating this example:
      | RTT input | DK Button | Effect                                                          |
      |-----------|-----------|-----------------------------------------------------------------|
      | `1`       | Button 1  | The mocked sensor value is _decreased_ by 1%.                   |
      | `2`       | Button 2  | The mocked sensor value is _decreased_ by 10%.                  |
      | `3`       | Button 3  | The mocked sensor value is _increased_ by 1%.                   |
      | `4`       | Button 4  | The mocked sensor value is _increased_ by 10%.                  |
      | `5`       | -         | All Bluetooth mesh data is erased and the device is reset.      |
    - Buttons: If you decide to use the buttons on the DK instead of the RTT input, you cannot
      erase all Bluetooth mesh data and reset the device.

- Sensor client:
  - When interacting with the boards, you can use one of the following options:
    - RTT input (recommended): Due to a limited number of buttons on the DK board,
    use the following RTT input when evaluating this example:
      | RTT input | DK Button | Effect                                                          |
      |-----------|-----------|-----------------------------------------------------------------|
      | `1`       | Button 1  | Send a descriptor get message for all properties.               |
      | `2`       | Button 2  | Send a descriptor get message for the Motion Sensed property.   |
      | `3`       | Button 3  | Send a cadence get message for the Motion Sensed property.      |
      | `4`       | Button 4  | Send an acknowledged cadence set message for the Motion Sensed property. |
      | `5`       | -         | Send an unacknowledged cadence set message for the Motion Sensed property. |
      | `6`       | -         | Send a settings get message for the Motion Sensed property.     |
      | `7`       | -         | Send a status get message for all properties.                   |
      | `8`       | -         | Send a status get message for the Motion Sensed property.       |
      | `9`       | -         | Switches the client instance to be used for sending messages.   |
    - Buttons: If you decide to use the buttons on the DK instead of the RTT input, you can only
      send descriptor get, cadence get, and acknowledged cadence set messages.


---

## Testing the example @anchor fast_cadence_example_testing

To test the sensor example, build the examples by following the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref fast_cadence_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref fast_cadence_example_testing_app)

### Evaluating using the static provisioner @anchor fast_cadence_example_testing_dk

See [provisioner example testing section](@ref provisioner_example_evaluating) for detailed steps required
to provision and configure the boards using the static provisioner.

@note
Using this provisioning method limits the amount of actions you can take when [interacting with the boards](@ref fast_cadence_example_testing_interaction).

### Evaluating using the nRF Mesh mobile app @anchor fast_cadence_example_testing_app

See [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app) for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

The following naming convention is used in the app:
- Each server board is `nRF5x Mesh Sensor Setup Server`.
- The client board is `nRF5x Mesh Sensor Client`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Sensor Setup Server` server boards: Sensor Setup Server, Sensor Server.
- For the `nRF5x Mesh Sensor Client` client board: Sensor Client.

When [setting publication with nRF Mesh mobile app](@ref nrf-mesh-mobile-app-publication), use the following procedure specific to this example:
1. On `nRF5x Mesh Sensor Setup Server`, in the publication section of the Sensor Serverc model instance menu, tap **Set Publication**.
2. Set the Publish Address to the address of the Sensor Client model on the `nRF5x Mesh Sensor Client` board.
3. Set the Publish Period to one of the following values:
  - Zero value for testing [standard publication behavior](@ref fast_cadence_example_testing_interaction_1).
  - Non-zero value for testing [periodic publication behavior](@ref fast_cadence_example_testing_interaction_2).

Once the provisioning with the mobile app is complete, you can start [interacting with the boards](@ref fast_cadence_example_testing_interaction).

@note
You can also configure the publish address of the second Sensor Client model instance.
To do this, repeat [step 3 from binding nodes](@ref nrf-mesh-mobile-app-binding) and
[all steps from setting publication](@ref nrf-mesh-mobile-app-publication).

### Interacting with the boards @anchor fast_cadence_example_testing_interaction

Once the provisioning and the configuration of the client node and of at least one of
the server nodes are complete, you can start interacting with the examples using buttons or RTT, or both.

The publication behavior of the Sensor server depends on the following parameters:
  - Model Publish Period
  - Sensor Cadence
  - Current sensor value (in this case, the Motion Sensed value)

#### Standard behavior when Publish Period is set to zero @anchor fast_cadence_example_testing_interaction_1

@note
If you use a mobile application to provision the example, 
set the Publish Period Interval of the Sensor Server model on the server to `Disabled`.

When the Publish Period of the Sensor Server model is set to zero, all button presses 
on the server will result in the sensor status message publications.
Use the RTT Viewer to observe the incoming sensor status messages on the client and 
to observe the output of the server.

#### Periodic publication behavior @anchor fast_cadence_example_testing_interaction_2

You can change the publication parameters to see how this will affect the publication behavior.

@note
The steps in this section require you to set the Publish Period of the Sensor Server model.
As of now, this is only possible if you choose to provision and configure the example using the nRF Mesh
mobile app.

@par Step 1: Set periodic publication to 10 seconds (default cadence)
Set a Publish Period to 10 seconds on the Sensor Server model on the server to make the server send the status messages every 10 seconds.
When the periodic publishing is configured, changing the sensor value does not result in the sensor status messages publications.
Instead, the server sends all sensor statuses on its periodic publication callback.
In this step, the cadence for the Motion Sensed property is using the default initial values.

@par Step 2: Set periodic publication to 60 seconds (custom cadence)
Set a Publish Period to 60 seconds on the Sensor Server model on the server.
To set the custom cadence values on the server, press Button 4 on the client.
This sets the following custom cadence for the sensor:
| Field                   | Value                    |
| ----------------------- | ------------------------ |
| `fast_period_exponent`  | `7` (representing `2^7`) |
| `trigger_type`          | `0`                      |
| `trigger_delta_down`    | `1`                      |
| `trigger_delta_up`      | `1`                      |
| `min_interval_exponent` | `1`                      |
| `fast_cadence_low`      | `0`                      |
| `fast_cadence_high`     | `49` (`31` in hex)       |
The `fast_cadence_high` value is higher than the `fast_cadence_low` value,
so Motion Sensed values in the closed interval of [`fast_cadence_low` (`0`), `fast_cadence_high`
(`49`)] will cause the server's sensor publication period to be the server's Publish Period divided
by `2^n`, where `n` is the `fast_period_exponent` (that is, `7`). This fast period is constrained to
a minimum interval by `min_interval`.

@par Step 3: Set a sensor value within fast cadence region (custom cadence)
Set the sensor value to `20`.
At the next periodic publication time, the server will find a Motion Sensed value of `20`.
As the sensor value is within the fast cadence region, the server will begin to publish 
on a period of `60 sec / 2^7 = 0.4687 sec`. The client receives information about two 
messages per second if the motion sensor value remains between `0` and `49`.

@par Step 4: Set a sensor value outside fast cadence region (custom cadence)
Set the sensor value to `50` or higher.
When this value is first published by the periodic publication
on the server, the periodic publication period will be set to 60 seconds (the configured Publish
Period for the server).
As long as the server's publications find Motion Sensed values of at least `50`, the client receives one
message per minute.

@note
The client's cadence message targets the sensor identified by @ref SENSOR_MOTION_SENSED_PROPERTY_ID.
When interacting with a multi-sensor server, the client can program each sensor to a unique cadence.
