# Low Power node example

@tag52840and52833and52832
@tag52810and52820nosupport

This example shows the implementation of a device supporting the Low Power node (LPN) feature. It
emulates an occupancy sensor device through button presses and a timer.

You can use this example as the starting point for adding the LPN feature to your device,
as it demonstrates all the required changes to the application. Read more about the Low Power Node feature
in @ref md_doc_user_guide_modules_lpn_concept.

When configured to interact with a device with a Generic OnOff Server model, the device running
this example turns on the LED on the light switch server device upon a button press, which emulates
triggering the occupancy sensor. It also sends an off message to the light switch server device after
five seconds to emulate inactivity.

This example uses [GATT provisioning](@ref md_doc_user_guide_modules_provisioning_gatt_proxy),
and instantiates a [Generic OnOff Client model](@ref GENERIC_ONOFF_CLIENT) that can be used
to control light switch servers.

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
        - [Establishing the friendship](@ref examples_lpn_running_friendship)
        - [Sending messages](@ref examples_lpn_running_sending)
        - [Updating the LPN node firmware through DFU over BLE](@ref examples_lpn_perform_dfu)
        - [Resetting the device](@ref examples_lpn_running_resetting)


---

## Hardware requirements @anchor examples_lpn_requirements_hw

You need three compatible development kits for this example:
- One compatible development kit for the LPN device running the Low Power Node example.
- Two compatible development kits for the light switch server device running the
[light switch server example](@ref light_switch_demo_server), which also includes a friend functionality.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

---

## Software requirements @anchor examples_lpn_requirements_sw

The Friend feature is supported by this version of the nRF5 SDK for Mesh and it is enabled
in the light switch server example.

You can find the light switch server example files in the following folder: `<InstallFolder>/examples/light_switch/server`

You also need to download and install
@link_nrf_mesh_app (available for @link_nrf_mesh_app_ios and @link_nrf_mesh_app_android)
for [Provisioning and configuration](@ref examples_lpn_running_provisioning).

---

## Setup @anchor examples_lpn_setup

You can find the source code and the project file of the example in the following folder:
`<InstallFolder>/examples/lpn`

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
    - Button 4: Reset the device (erase all Bluetooth mesh data).


---

## Testing the example @anchor examples_lpn_running

To send messages between the LPN device and the light switch server device, complete the following steps:
- [Building and flashing](@ref examples_lpn_initial_building)
- [Provisioning and configuration](@ref examples_lpn_running_provisioning)
- [Establishing the friendship](@ref examples_lpn_running_friendship)
- [Sending messages](@ref examples_lpn_running_sending)
- [Updating the Low Power node example through DFU over BLE](@ref examples_lpn_perform_dfu)
- [Resetting the device](@ref examples_lpn_running_resetting)


### Building and flashing @anchor examples_lpn_initial_building

To set up the example:
1. Decide about the Device Firmware Upgrade approach for the Low Power node:
    - Use the standard setting (Device Firmware Upgrade over BLE disabled by default).
        1. Build the Low Power node example. To build the example, follow the instructions in
        [Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).
        2. Program the Low Power node example onto one development kit.
        See @ref md_doc_getting_started_how_to_run_examples for the instructions.
    - Use the node with DFU over BLE:
        - Follow the instructions on the @subpage md_examples_lpn_dfu_ble page and program
        the example onto one development kit.
2. Build the Light switch server example. To build the example, follow the instructions
in [Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).
3. Program the Light Switch Server example onto two development kits.
See @ref md_doc_getting_started_how_to_run_examples for the instructions.

All three devices are now running Bluetooth-mesh-enabled firmware.

@note When building and running the firmware, you might encounter the following error:
```
app_error_weak.c, 119, Mesh error 4 at 0x00000000
```
This error means that the bootloader is not flashed. Go to
[Building and programming the bootloader](@ref examples_lpn_dfu_ble_program_bootloader)
to flash the bootloader.

### Provisioning and configuration @anchor examples_lpn_running_provisioning

Before a friendship can be established between the LPN device and the Friend device,
they both must be provisioned to the same Bluetooth mesh network.

As the Low Power node example only supports the PB-GATT bearer for provisioning, use @link_nrf_mesh_app
(for @link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) to provision and configure
all three devices. See @ref nrf-mesh-mobile-app "the information on the main Examples page" for detailed steps.

The following naming convention is used in the app:
- Each server board is `nRF5x Mesh Light`.
- The LPN device client board is `nRF5x Mesh LPN`.

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Light` server boards: Generic On Off Server.
- For the `nRF5x Mesh LPN` client board: Generic On Off Client.

When [setting publication with nRF Mesh mobile app](@ref nrf-mesh-mobile-app-publication), use the following procedure specific to the LPN example:
1. On `nRF5x Mesh LPN`, in the publication section of the Generic On Off Client model instance menu, tap **Set Publication**.
2. Tap the publication address field. A dropdown menu appears.
3. Set the publication to a group address:
    1. Select an existing group to subscribe or create a new one.
    2. Apply the changes for the client node.
4. On `nRF5x Mesh Light`, in the publication section of the Generic On Off Server model instance menu, tap **Set Publication**.
5. Tap the publication address field. A dropdown menu appears.
6. Set the publication to a unicast address of the `nRF5x Mesh LPN` node.
7. In the subscriptions section of the Generic On Off Server model instance menu, tap **Subscribe**.
8. Set the subscription address to the selected group address.

At the end of the configuration process:
- The light switch servers are part of the Bluetooth mesh network, and are ready to receive messages from the LPN device.
- The LPN device is part of the Bluetooth mesh network and can control the LEDs of the light switch server device.
It has not entered the low power mode yet, as the friendship is not established.
- After assigning the addresses of the server nodes to the client, you can see the messages received from the servers in the RTT log of the LPN device.


### Establishing the friendship @anchor examples_lpn_running_friendship

After the initial configuration and provisioning are complete, the LPN device enters the idle state.

To start the friendship establishment process, press button 3 on the LPN device. The device starts
searching for an appropriate Friend in the Bluetooth mesh network.

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

- Pressing button 1 turns on the LED 1 on the LPN device and sends message to the light switch servers.
This message turns on the LED 1 on both of the light switch servers.
- Pressing button 2 turns off the LED 1 on the LPN device and sends message to the light switch servers.
This message turns off the LED 1 on both of the light switch servers.

This behavior is identical to the one in the [light switch client](@ref light_switch_demo_client) example.

If you connect the RTT viewer to the LPN device, you will see the status messages being received from
the servers through the Friend node.


### Updating the LPN node firmware through DFU over BLE @anchor examples_lpn_perform_dfu

@note Skip this section if you decided to use the node without DFU over BLE.

When the Low Power node is running, you can update it using DFU over BLE:
1. Add any changes to the Low Power node example, for example change the log message
in the `initialize()` function to the following:
```
__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh LPN Demo (Updated) -----\n");
```
2. Rebuild the Low Power node example.
3. Create a new firmware package with the modified Low Power node example.
Refer to [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package) section
of the @ref md_examples_lpn_dfu_ble page to create a new firmware package.
4. Upload the new firmware package to the LPN device. Refer to [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu) section
of the @ref md_examples_lpn_dfu_ble page to upload the new firmware package.
5. Verify that the new firmware is uploaded be checking the RTT log.

### Resetting the device @anchor examples_lpn_running_resetting

Pressing button 4 resets and unprovisions the LPN device.
