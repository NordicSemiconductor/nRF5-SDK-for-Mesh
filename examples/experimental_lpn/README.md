# Low Power node example (experimental)

This example shows the implementation of a device supporting the Low Power node (LPN) feature. It
emulates an occupancy sensor device through button presses and a timer.

You can use this example as the starting point for adding the LPN feature to your device,
as it demonstrates all the required changes to the application. Read more about the Low Power Node feature
in @ref md_doc_introduction_lpn_concept.

When configured to interact with a device with a Generic OnOff Server model, the device running
this example turns on the LED on the light switch server device upon a button press, which emulates
triggering the occupancy sensor. It also sends an off message to the light switch server device after
five seconds to emulate inactivity.

This example uses [GATT for provisioning](@ref md_doc_getting_started_gatt_proxy), and instantiates
a [Generic OnOff Client model](@ref GENERIC_ONOFF_CLIENT) that can be used to control light switch
servers.

This example also supports the @link_buttonless_secure_dfu_service to perform Device Firmware Upgrade
over BLE. The DFU over BLE is disabled by default. See @link_bootloader_and_dfu_modules for more information.

**Table of contents**
- [Hardware requirements](@ref examples_lpn_requirements_hw)
- [Software requirements](@ref examples_lpn_requirements_sw)
- [Setup](@ref examples_lpn_setup)
        - [LED and button assignments](@ref examples_lpn_setup_buttons)
- [Testing the example](@ref examples_lpn_running)
        - [Building and flashing](@ref examples_lpn_initial_building)
        - [Provisioning and configuration](@ref examples_lpn_running_provisioning)
                - [Configuring light switch server device](@ref examples_lpn_running_provisioning_server)
                - [Configuring the LPN device](@ref examples_lpn_running_provisioning_lpn)
                - [Provisioning the Friend device](@ref examples_lpn_running_provisioning_friend)
        - [Establishing the friendship](@ref examples_lpn_running_friendship)
        - [Sending messages](@ref examples_lpn_running_sending)
        - [Updating the LPN node firmware through DFU over BLE](@ref examples_lpn_perform_dfu)
        - [Resetting the device](@ref examples_lpn_running_resetting)
        

---

## Hardware requirements @anchor examples_lpn_requirements_hw

Running this example requires three PCA10040 Development Kits:
- One development kit for the LPN device running the Low Power Node example.
- One development kit for thr Friend device running mesh firmware with the Friend feature enabled.
- One development kit for the light switch server device running the
[light switch server example](@ref light_switch_demo_server).


---

## Software requirements @anchor examples_lpn_requirements_sw

As the Friend feature is not supported by this version of the nRF5 SDK for Mesh, use a
precompiled version of the Friend node from the @link_zephyr to fulfill the
Friend device role. The precompiled Friend node hex files are available at
`<InstallFolder>/examples/experimental_lpn/bin`. See @subpage md_examples_experimental_lpn_bin_README
for additional information on how they were built.

You can find the light switch server example files in the following folder: `<InstallFolder>/examples/light_switch/server`

You also need to download and install
@link_nrf_mesh_app (available for @link_nrf_mesh_app_ios and @link_nrf_mesh_app_android)
for [Provisioning and configuration](@ref examples_lpn_running_provisioning).

---

## Setup @anchor examples_lpn_setup

You can find the source code and the project file of the example in the following folder:
`<InstallFolder>/examples/experimental_lpn`

### LED and button assignments @anchor examples_lpn_setup_buttons

These assignments refer to the LPN device only.

- LEDs
    - LED 1: Reacts to pressing button 1 (turns on) and button 2 (turns off). Can be off for other reasons,
    for example when joining the network.
    - LED 2: Friendship established.
    - LED 3 and 4 blinking: Device identification active.
    - All LEDs blinking four times: Provisioning complete.
    - All LEDs flashing instensively: An error occurred. See the RTT log for details.
- Buttons
    - Button 1: Send the on message.
    - Button 2: Send the off message.
    - Button 3: Establish or terminate friendship.
    - Button 4: Reset the device (erase all mesh data).


---

## Testing the example @anchor examples_lpn_running

To send messages between the LPN device and the light switch server device, complete the following steps:
- [Building and flashing](@ref examples_lpn_initial_building)
- [Provisioning and configuration](@ref examples_lpn_running_provisioning)
	- [Configuring light switch server device](@ref examples_lpn_running_provisioning_server)
	- [Configuring the LPN device](@ref examples_lpn_running_provisioning_lpn)
	- [Provisioning the Friend device](@ref examples_lpn_running_provisioning_friend)
- [Establishing the friendship](@ref examples_lpn_running_friendship)
- [Sending messages](@ref examples_lpn_running_sending)
- [Updating the Low Power node example through DFU over BLE](@ref examples_lpn_perform_dfu)
- [Resetting the device](@ref examples_lpn_running_resetting)


### Building and flashing @anchor examples_lpn_initial_building

To set up the example:
1. Decide about the Device Firmware Upgrade approach for the Low Power node:
    - Use the standard setting (Device Firmware Upgrade over BLE disabled by default).
        1. Build the Low Power node example. To build the example, follow the instructions in
        [Building the mesh stack](@ref md_doc_getting_started_how_to_build).
        2. Program the Low Power node example onto one development kit.
        See @ref md_doc_getting_started_how_to_run_examples for the instructions.
    - Use the node with DFU over BLE:
        - Follow the instructions on the @subpage md_examples_experimental_lpn_dfu_ble page and program
        the example onto one development kit.
4. Build the Light switch server example. To build the example, follow the instructions
in [Building the mesh stack](@ref md_doc_getting_started_how_to_build).
5. Program the Light Switch Server example onto another development kit.
See @ref md_doc_getting_started_how_to_run_examples for the instructions.
6. Choose the Friend node hex file appropriate for your hardware platform from
`examples/experimental_lpn/bin/`, and program it to the third development kit using nrfjprog:

```
nrfjprog --program <path-to-hex-file> --chiperase -r
```

@note The Zephyr Friend node hex file does not require a SoftDevice, so it starts running
immediately after nrfjprog finshes execution.

All three devices are now running Bluetooth Mesh enabled firmware.

### Provisioning and configuration @anchor examples_lpn_running_provisioning

Before a friendship can be established between the LPN device and the Friend device,
they both must be provisioned to the same mesh network. As the Low Power node example
only supports the PB-GATT bearer for provisioning, use @link_nrf_mesh_app
(for @link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) to provision and configure
all three devices.

#### Configuring light switch server device @anchor examples_lpn_running_provisioning_server

The light switch server device shows up in the nRF Mesh App as "nRF5x Mesh LPN Switch".

1. Provision the light switch server device with the nRF Mesh App.
2. Give the light switch server device `Application key 1` through its configuration menu,
if it doesn't already have it. This application key will represent the light switch application.
The key will be used by the Generic OnOff client on the LPN device and the Generic OnOff server
to communicate.
3. Open the Generic OnOff Server model's configuration menu, and bind `Application key 1`
to a Generic OnOff Server model that the light switch server device instantiates in its first element.
4. Disconnect from the light switch server device, and go back to the scanner screen.
5. Take note of the light switch server device's unicast address, as it is needed for the
configuration of the LPN device.

The light switch server is now a part of the mesh network, and is ready to receive messages from
the LPN device.

#### Configuring the LPN device @anchor examples_lpn_running_provisioning_lpn

The LPN device shows up in the nRF Mesh App as "nRF5x Mesh LPN".

1. Provision the LPN device with the nRF Mesh App.
2. Give the LPN device `Application key 1` through its configuration menu, if it doesn't already
have it. The key has the same role and usage as in the case of the light switch server device.
3. Open the Generic OnOff Client model's configuration menu, and bind `Application key 1`
to a Generic OnOff Client model that the LPN device instantiates in its first element.
4. Open the Generic OnOff Client model's publication settings.
5. Set the Publish Address to the unicast address of the light switch server that you took note of.
6. Disconnect from the LPN device, and go back to the scanner screen.

The LPN device is now part of the mesh network and can control the LEDs of the light switch server
device. It has not entered the low power mode yet, as it needs a Friend device to accept its Friend
requests.

#### Provisioning the Friend device @anchor examples_lpn_running_provisioning_friend

The Friend shows up in the nRF Mesh App as "Zephyr".
Provision the Friend device with the nRF Mesh App.
No additional configuration is needed.

### Establishing the friendship @anchor examples_lpn_running_friendship

After the initial configuration and provisioning are complete, the LPN device enters the idle state.

To start the friendship establishment process, press button 3 on the LPN device. The device starts
searching for an appropriate Friend in the mesh network.

The LPN example always accepts the first friendship it is offered with.
Establishing a friendship normally takes less than a second. If the LPN device cannot find a
Friend after 5-10 seconds, all LEDs will blink intensively. You have to press button 3 again
to retry.

Once the friendship is established, LED 2 turns on and stays lit throughout the friendship.

@note Pressing button 3 again while the LPN is in a friendship causes it to terminate the friendship
and go back to the normal power mode.


### Sending messages @anchor examples_lpn_running_sending

You can send on and off messages by pressing buttons 1 and 2, respectively.
These buttons also control the LED 1 of the light switch server through the Generic OnOff client model.

- Pressing button 1 turns on the LED 1 on the LPN device and sends message to the light switch server.
This message turns on the LED 1 on the light switch server.
- Pressing button 2 turns off the LED 1 on the LPN device and sends message to the light switch server.
This message turns off the LED 1 on the light switch server.

This behavior is identical to the one in the [light switch client](@ref light_switch_demo_client) example.

### Updating the LPN node firmware through DFU over BLE @anchor examples_lpn_perform_dfu

@note Skip this section if you decided to use the node without DFU over BLE.

When the Low Power node is running, you can update it using DFU over BLE:
1. Add any changes to the Low Power node example, for example change the log message
in the `initialize()`function to the following:
```
__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh LPN Demo (Updated) -----\n");
```
2. Rebuild the Low Power node example.
3. Create a new firmware package with the modified Low Power node example.
Refer to [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package) section
of the @ref md_examples_experimental_lpn_dfu_ble page to create a new firmware package.
4. Upload the new firmware package to the LPN device. Refer to [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu) section
of the @ref md_examples_experimental_lpn_dfu_ble page to upload the new firmware package.
5. Verify that the new firmware is uploaded be checking the RTT log.

### Resetting the device @anchor examples_lpn_running_resetting

Pressing button 4 resets and unprovisions the LPN device.
