# Light LC server example

@tag52810nosupport

This example demonstrates how the light controller uses the data coming from sensors and switches
to control the lightness level of lights. After configuring and running this example, triggering Light On,
Light Off, and occupancy events will change the brightness of the LED on your board.

This examples uses the [Light LC Setup Server model](@ref LIGHT_LC_SETUP_SERVER).
For more information about the Light LC Setup Server model, see also the Bluetooth SIG's @link_ModelOverview.

For provisioning purposes, the example requires either the provisioner example that is provided
in the @ref md_examples_provisioner_README or the nRF Mesh mobile app.

**Table of contents**
- [Light LC Setup Server model](@ref light_lc_server_example_light_lc_setup_server_model)
- [Hardware requirements](@ref light_lc_server_example_hw_requirements)
- [Software requirements](@ref light_lc_server_example_sw_requirements)
- [Setup](@ref light_lc_server_example_setup)
    - [LED and button assignments](@ref light_lc_server_example_setup_leds_buttons)
- [Testing the example](@ref light_lc_server_example_testing)
    - [Evaluating using the static provisioner](@ref light_lc_server_example_testing_dk)
    - [Evaluating using the nRF Mesh mobile app](@ref light_lc_server_example_testing_app)
    - [Interacting with the boards](@ref light_lc_server_example_testing_interacting)
      - [Lightness, illuminance, and transition times properties](@ref light_lc_server_example_testing_interacting_properties)
      - [Delaying occupancy report transition](@ref light_lc_server_example_testing_reporting_occupancy)


The example instantiates the following models:
- One [Light LC Setup Server model](@ref LIGHT_LC_SETUP_SERVER) with the Light LC Server model and
its own Generic OnOff Server model on the second element.
- One [Light Lightness Setup Server model](@ref LIGHT_LIGHTNESS_SETUP_SERVER) with all its extended
models on the first element.

The model instance uses the Light On and Light Off events and the data coming from occupancy and ambient sensors
to adjust the lightness output, which is used to control the brightness of the LED using the
@link_APP_PWM library of the nRF5 SDK.

![Light LC server example structure](lc_example_structure.svg)

This example has the provisionee role in the network.
It supports provisioning over Advertising bearer (PB-ADV) and GATT bearer (PB-GATT),
and the @ref md_doc_user_guide_modules_provisioning_gatt_proxy "Mesh Proxy Service (Server)".

## Light LC Setup Server model @anchor light_lc_server_example_light_lc_setup_server_model

The Light LC Server model collects the data from occupancy and ambient sensors. When the light controller
is enabled (see @ref LIGHT_LC_DEFAULT_MODE), this model controls the lightness output.
The Light Lightness Setup Server model is used to reflect the changes in the lightness output
on the lightness level of lights through the binding between the Light LC Linear Output and
Light Lightness Linear states of these two models.

The Light Lightness Setup Server model can be still controlled with the Light Lightness Client.
However, the light controller is switched off automatically by the Light LC Server as soon any mesh
message to change the lightness value is received (for example, the Light Lightness Set
or any other message that can change the bound lightness state value).
Use a Light LC Client model to turn the light controller on again.

@note This example does not provide the Light LC Client model example.

For more information about the Light Lightness Setup Server model, see [the light lightness example](@ref light_lightness_example_light_lightness_server).

The Light LC Setup Server supports BLE Mesh Occupancy and Ambient Lux Level sensors -- whose values
are received through the Sensor Status messages.
Sensors use the Sensor Server model to publish Sensor Status messages.

For demonstration purposes, this example includes a prebuilt HEX file with the occupancy sensor example.

The Light LC Setup Server model uses the Proportional Integral (PI) Feedback Regulator to control
the lightness output. The PI Feedback Regulator reads the ambient light sensors' data at regular intervals.

For more information about the Light LC models, see [Light LC models documentation](@ref LIGHT_LC_MODELS).

---

## Hardware requirements @anchor light_lc_server_example_hw_requirements

You need at least two supported development kits for this example:

- One nRF52 development kit for the [Light switch client](@ref light_switch_demo_client).
- One or more nRF52 development kits for the light LC servers.
- One or more nRF52 development kits running mesh firmware with the provided precompiled occupancy sensor example.

Additionally, you need one development kit for the provisioner
if you decide to use the [static provisioner example](@ref md_examples_provisioner_README).
For details, see [software requirements](@ref light_lightness_example_sw_requirements).

See @ref md_doc_user_guide_mesh_compatibility for the supported development kits.

@note This example uses the PWM peripheral to control the brightness of the LED.
For this reason, it cannot be run on the nRF51 devices.


---

## Software requirements @anchor light_lc_server_example_sw_requirements

To test this example, you need to use the following additional software:
    - The client example from the light switch example folder:
      `<InstallFolder>/examples/light_switch/client`
        - See the [Light switch example](@ref md_examples_light_switch_README) page for more information
          about the client example.
    - As the Sensor Server model is not implemented in this version of the nRF5 SDK for Mesh, use the
    precompiled version of the occupancy sensor example. The precompiled occupancy sensor example HEX
    files are available at `<InstallFolder>/examples/light_lc/server/bin`.
    - Depending on your choice of the provisioning method:
        - If you decide to use the static provisioner example, you need the provisioner example:
          `<InstallFolder>/examples/provisioner`
            - See the [Provisioner example](@ref md_examples_provisioner_README) page for more information
            about the provisioner example.
        - If you decide to provision using the mobile application, you need to download and install
        @link_nrf_mesh_app (available for @link_nrf_mesh_app_ios and @link_nrf_mesh_app_android).
    
    
---

## Setup @anchor light_lc_server_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/light_lc/server`

### LED and button assignments @anchor light_lc_server_example_setup_leds_buttons

The following LED and button assignments are defined for this example:
- Light LC Server:
    - LED 1: Reflects the value of the Light Lightness Actual state on the server.
    - When interacting with the boards:
        - You cannot use buttons on the server boards because the Light LC server example
        does not use the `simple_hal` module.
        - Instead of the buttons on the server boards, use the following RTT input:
        | RTT input     | DK Buttons    |   Effect                                                                 |
        |---------------|---------------|--------------------------------------------------------------------------|
        | `1`           | -             | Toggles the values of the properties between 0 and the default values.   |
        | `4`           | -             | All mesh data is erased and the device is reset.                         |
        - When sending the `1` RTT command, the following properties are toggled between 0 and the default values:
            - Light Control Time Fade
            - Light Control Time Fade On
            - Light Control Time Fade Standby Auto
            - Light Control Time Fade Standby Manual
            - Light Control Time Run On

            See Section 4.1.3 of the Mesh Device Properties v1.1, @link_MeshProperties, and @link_MeshCharacteristics
            for more information about the properties.
- Sensor Server:
    - LED 1: Reflects the value of the Presence Detect state of the emulated occupancy sensor.
    - When interacting with the boards, use the following buttons or RTT input:
    | RTT input   | DK buttons  |  Effect                                                      |
    |-------------|-------------|--------------------------------------------------------------|
    | `1`         | 1           | Toggles the value of the emulated Presence Detect state.     |
    | `4`         | 4           | All mesh data is erased and the device is reset.             |


---


## Testing the example @anchor light_lc_server_example_testing

To test the light LC server example, first build this example and the light switch client example by following the instructions in [Building the mesh stack](@ref md_doc_getting_started_how_to_build).

@par Using 40+ servers with static provisioner
If you have more than 40 boards for the servers and decided to use the static provisioner example:
1. Set `MAX_PROVISIONEE_NUMBER` (in `example_network_config.h`) to the number of boards available.
2. Rebuild the provisioner example.
3. Set `MAX_AVAILABLE_SERVER_NODE_NUMBER` in `nrf_mesh_config_app.h`
of the client example to the value set for `MAX_PROVISIONEE_NUMBER`.

@note
The @link_ModelSpec mentions that the default value of the mode of the light controller to be set
to `(0x0)`. This means that the light controller is turned off by default.
To enable the light controller, the Light LC Client model is used.
However, this example does not provide the Light LC Client example.
For this reason, in this example the light controller is switched on by default.
This has been done by changing the default value of the @ref LIGHT_LC_DEFAULT_MODE in
`nrf_mesh_config_app.h` to `(0x1)`.

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref light_lc_server_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref light_lc_server_example_testing_app)

### Evaluating using the static provisioner @anchor light_lc_server_example_testing_dk

Complete the following steps:
1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    -# Erase the flash of your development boards and program the SoftDevice.
    -# Flash the provisioner and the client firmware on individual boards and the server firmwares on other boards.
2. After the reset at the end of the flashing process, press Button 1 on the provisioner board
to start the provisioning process:
    -# The provisioner provisions and configures the client and assigns the address 0x100 to the client node.
    -# The two instances of the Light Switch client models are instantiated on separate secondary elements.
    For this reason, they get consecutive addresses starting with 0x101.
    -# The provisioner also provisions and configures the servers at random. It assigns the Light LC servers
    consecutive addresses starting with 0x501, and adds them to odd and even groups. It assigns
    the Sensor servers consecutive addresses starting with 0x801, and adds them to odd and even
    groups.
@note - The sequence of provisioned devices depends on the sequence of received unprovisioned beacons.
@note - You can use [RTT viewer](@ref segger-rtt) to view the RTT output generated by the provisioner.
The provisioner prints details about the provisioning and the configuration process in the RTT log.
3. Observe that the LED 1 on the provisioner board is turned ON when provisioner is scanning and provisioning a device.
4. Observe that the LED 2 on the provisioner board is turned ON when configuration procedure is in progress.
5. Wait until LED 1 on the provisioner board remains lit steadily for a few seconds, which indicates that
all available boards have been provisioned and configured.

If the provisioner encounters an error during the provisioning or configuration process for a certain node,
you can reset the provisioner to restart this process for that node.

### Evaluating using the nRF Mesh mobile app @anchor light_lc_server_example_testing_app

See @ref nrf-mesh-mobile-app "the information on the main Examples page" for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

The following naming convention is used in the app:
- Each server board is `nRF5x Mesh Light LC Setup Server`.
- The Light switch client board is `nRF5x Mesh Switch`.
- The Occupancy sensor board is `nRF5x Mesh Occupancy Sensor`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Light LC Setup Server` server boards: Light LC Setup Server,
  Light LC Server, Generic OnOff Server instantiated on the second element.
- For the `nRF5x Mesh Switch` client board: Generic OnOff Client.
- For the `nRF Mesh Sensor Server` server board(s): Sensor Server.

When [setting publication with nRF Mesh mobile app](@ref nrf-mesh-mobile-app-publication):
- For the Light switch client example, set the publication address of the second Generic OnOff Client
model instance to the second element address of the Light LC server example.
- For the Sensor Server example, set the publication address of the Sensor Server model instance
to the second element address of the Light LC server example.

Once the provisioning is complete, you can start [Interacting with the boards](@ref light_lc_server_example_testing_interacting).

### Interacting with the boards @anchor light_lc_server_example_testing_interacting

Once the Light LC server board is started after provisioning, the light controller will be turned on.
The example starts in the Standby state, and the LED 1 is driven to the lowest light level
(either determined by the @ref LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID propety or a higher level,
because of the light feedback from the ambient light sensor that attempts to keep the light
at a minimum illuminance determined by the @ref LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID property).

At this stage, you can interact with the Light LC server example by using the following options:
- Press Button 1 on the Light switch client board to send the `Light On` event.
    - Even though this example does not provide the Light LC Client example, you can still send Light On and Light Off events to
    the Light LC Setup Server example, because the Light LC Light OnOff state of the Light LC Server is bound
    with the Generic OnOff state of the Generic OnOff Server model extended by the Light LC Server model instance.
- Press Button 1 on the Sensor Server board to trigger the `Occupancy On` event.

Issuing the `Occupancy On` event or the `Light On` event triggers the following chain of events, as shown on
the Figure 6.4 of the @link_ModelSpec:
- The Light LC server's state machine transitions to the Fade On state.
  The LED 1 starts increasing in brightness to the Run level.
    - The time it takes to reach the Run state depends upon the Light On message that was sent.
    If the message was sent with a non-zero transition time, the specified transition time is used;
    otherwise, the time determined by @ref LIGHT_LC_SERVER_TIME_FADE_ON_PID property is used.
- After reaching the Run state, the light stays at the Run level for the time determined by the
  @ref LIGHT_LC_SERVER_TIME_RUN_ON_PID property.
- The light then transitions to the Fade state.
  The light starts decreasing to the Prolong level for the time
  determined by the @ref LIGHT_LC_SERVER_TIME_FADE_PID property.
- After reaching the Prolong state, the light stays at the Prolong level for the time determined
  by the @ref LIGHT_LC_SERVER_TIME_PROLONG_PID property.
- The light then transitions through the Fade Standby Auto state back to the Standby state.
  The amount of time this takes is determined by the @ref LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID property.

You can issue new events at any time.
This includes pressing Button 2 (or 4) on the Light switch client board to send a `Light Off` event.
In such case, the state machine will start transition depending on its current state machine state
and will set the brightness of the LED 1 according to the lightness level defined for the current state.

@note
For the testing purposes, the default time values for the various states are very short (between 2 and 10 seconds).
When the Light LC Setup Server is running in a production environment, these values can be set to minutes or hours.
For more the information about the format, units and allowed values of the properties representing these values,
see Section 4.1.3 of @link_MeshDeviceProperties, @link_MeshProperties and @link_MeshCharacteristics.

#### Lightness, illuminance, and transition time properties @anchor light_lc_server_example_testing_interacting_properties

The properties' values are controlled through the Light LC Client model.

The minimum lightness value at any state is determined by the following properties:
- @ref LIGHT_LC_SERVER_LIGHTNESS_STANDBY_PID
- @ref LIGHT_LC_SERVER_LIGHTNESS_ON_PID
- @ref LIGHT_LC_SERVER_LIGHTNESS_PROLONG_PID

The illuminance value at any state is determined by the following properties:
- @ref LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_STANDBY_PID
- @ref LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_ON_PID
- @ref LIGHT_LC_SERVER_AMBIENT_LUXLEVEL_PROLONG_PID

If the values for these properties are not being detected by the PI Feedback Regulator,
it will attempt to drive the LED 1 to a higher brightness.

If there were no reports from an ambient light sensor, the PI Feedback Regulator will not
attempt to drive the LED 1, and the LED 1 will be driven by the state machine to the lightness
values defined by the properties values specified for each state.

The factory default values of the lightness and illuminance properties are controlled through
the corresponding defines:
- @ref LIGHT_LC_DEFAULT_PR_LIGHTNESS_STANDBY
- @ref LIGHT_LC_DEFAULT_PR_LIGHTNESS_ON
- @ref LIGHT_LC_DEFAULT_PR_LIGHTNESS_PROLONG
- @ref LIGHT_LC_DEFAULT_PR_LUXLEVEL_STANDBY
- @ref LIGHT_LC_DEFAULT_PR_LUXLEVEL_ON
- @ref LIGHT_LC_DEFAULT_PR_LUXLEVEL_PROLONG

The transition time is determined by the following properties:
- @ref LIGHT_LC_SERVER_TIME_FADE_ON_PID
- @ref LIGHT_LC_SERVER_TIME_RUN_ON_PID
- @ref LIGHT_LC_SERVER_TIME_FADE_PID
- @ref LIGHT_LC_SERVER_TIME_PROLONG_PID
- @ref LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID

You can make the state machine to transition instantly between the Standby and Run states.
To do that, use RTT command `1` to toggle the properties of the state machine.
After setting the corresponding properties to `0` you should notice the immediate transition between
these states, when `Occupancy On` or `Light On` event is reported.

The factory default values of the time properties are controlled through the corresponding defines:
- @ref LIGHT_LC_DEFAULT_PR_TIME_RUN_ON_MS
- @ref LIGHT_LC_DEFAULT_PR_TIME_FADE_ON_MS
- @ref LIGHT_LC_DEFAULT_PR_TIME_PROLONG_MS
- @ref LIGHT_LC_DEFAULT_PR_TIME_FADE_MS
- @ref LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_AUTO_MS

If you want to edit the factory default values, do this in `nrf_mesh_config_app.h` of the Light LC server example.
Follow instructions in [Testing the example](@ref light_lc_server_example_testing)
to re-build and re-provision the example.

#### Delaying occupancy report transition @anchor light_lc_server_example_testing_reporting_occupancy

By default, when occupancy has been reported by the Sensor Server model through Sensor Status messages,
the transition will start immediately. You can postpone the transition by setting a delay time in
@ref LIGHT_LC_DEFAULT_PR_TIME_OCCUPANCY_DELAY_MS in `nrf_mesh_config_app.h` of the LC server example.

