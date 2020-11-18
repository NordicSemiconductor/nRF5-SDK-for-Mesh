# Provisioner example

@tagSupportAllCompatibleBoards

The provisoner example is the default Bluetooth mesh network configurator.
It works in a fixed, predefined way and can be used as the static provisioner with the following examples:
- [Dimming examples](@ref md_examples_dimming_README)
- [EnOcean switch translator client example](@ref md_examples_enocean_switch_README)
- [Light switch example](@ref md_examples_light_switch_README)
- [Light lightness example](@ref md_examples_light_lightness_README)
- [Light LC server example](@ref md_examples_light_lc_server_README)
- [Light CTL example](@ref md_examples_light_ctl_README)
- [Scene example](@ref md_examples_scene_README)
- [Sensor example](@ref md_examples_sensor_README)


**Table of contents**
- [Network configuration and address assignment](@ref provisioner_example_assignment)
    - [Node address assignments](@ref provisioner_example_assignment_node_address)
    - [Publish subscribe configuration](@ref provisioner_example_assignment_publish_subscribe)
    - [Provisioner state machine](@ref provisioner_example_assignment_state_machine)
- [Usage of Bluetooth mesh APIs](@ref provisioner_APIs)
- [Hardware requirements](@ref provisioner_example_hw_requirements)
- [Setup](@ref provisioner_example_setup)
    - [LED and button assignments](@ref provisioner_example_setup_leds_buttons)
- [Testing the example](@ref provisioner_example_testing)
    - [Specifying the maximum number of provisionee devices](@ref provisioner_example_testing_40boards)
    - [Provisioning examples without Scene Setup Server model](@ref provisioner_example_no_scene_setup_server)
    - [Evaluating examples using the static provisioner](@ref provisioner_example_evaluating)

Because of the asynchronous nature of the provisioning and configuration process, the provisioner is implemented as a multi-layered state machine.
The application provisions and configures devices when it receives unprovisioned beacons from them.

Every supported example has a unique Uniform Resource Identifier (URI) included in the beacons.
The provisioner example uses URI hash to understand the type of example being provisioned and
to select the suitable configuration to be applied to the node after provisioning.
For this reason, this provisioner can provision devices only with the known URI.
The list of URIs is predefined.

The static provisioner has its own limitations and is provided as a tool to evaluate SDK examples without the need to use a mobile application provisioner.
If you are using a mobile application for provisioning, you can see the models present on the device and configure the nodes accordingly.
Moreover, you can also use more sophisticated automated provisioners that can use the UUID, URI, and the device composition data to understand which devices are being provisioned and
configure them automatically.

---

## Network configuration and address assignment @anchor provisioner_example_assignment

The following diagram shows how the provisioner configures Bluetooth mesh examples.

![Typical configuration of Bluetooth mesh examples by the provisioner](images/provisioning_cfg.svg "Typical configuration of Bluetooth mesh examples by provisioner")

### Node address assignments @anchor provisioner_example_assignment_node_address

During the provisioning procedure, the client boards are provisioned at random and their elements are assigned addresses starting in the following manner:
| Example       | Client board               | Primary<br>element address  | First secondary<br>element address  | Second secondary<br>element address |
|---------------|----------------------------|-----------------------------|-------------------------------------|-------------------------------------|
| All examples  | Primary element (1st board)| 0x100                       | 0x101                               | 0x102                               |
| All examples  | Primary element (2nd board)| 0x104                       | 0x105                               | 0x106                               |
| All examples  | Primary element (3rd board)| 0x108                       | 0x109                               | 0x10A                               |
| All examples  | ...                        | ...                         | ...                                 | ...                                 |

On each client example, the two instances of the client models are instantiated on separate secondary elements to communicate
with the Odd and Even server groups. Each subsequent secondary element gets a consecutive address starting from the address of the primary element.
The provisioner configures the client model instantiated on an element with the Odd address to communicate with the Odd group,
and the client model instantiated on an element with the Even address to communicate with the Even group.

If you have multiple client boards in the network, their primary elements will receive a consecutive even address
after the address of the last element on a previously provisioned client node.

The following table shows the list of client examples.

| Example                                                     | Client example    |
|-------------------------------------------------------------|-------------------|
| @ref light_ctl_example_testing "Light CTL"                  | Light CTL         |
| @ref light_lc_server_example_testing_dk "Light LC"          | Light switch      |
| @ref light_lightness_example_testing_dk "Light lightness"   | Light Lightness   |
| @ref light_switch_example_testing_dk "Light switch"         | Light switch      |
| @ref dimming_prov_prov_example "Dimming"                    | Dimming           |
| @ref enocean_example_testing_dk "EnOcean translator client" | EnOcean (as a Light switch client) |
| @ref scene_example_testing_dk "Scene client"                | Scene             |
| @ref fast_cadence_example_testing_dk "Sensor client"        | Sensor            |

The provisioner also provisions and configures the servers at random.
It assigns them addresses and adds them to Odd and Even groups.

The starting addresses are different for each server example.
Subsequently provisioned server boards get the next consecutive Odd and Even addresses.

| Example                                                     | Server example                                        | Primary<br>element address | 1st secondary<br>element address | 2nd secondary<br>element address | Next board's<br>Primary element address | ... |
|-------------------------------------------------------------|-------------------------------------------------------|----------------------------|----------------------------------|----------------------------------|-----------------------------------------|-----|
| @ref light_ctl_example_testing "Light CTL"                  | Light CTL servers with Scene servers<br>Light CTL Server with Light LC servers   | 0x601<br>0x701             | 0x602<br>0x702                   | NA<br>0x703                      | 0x604<br>0x704                          | ... |
| @ref light_lc_server_example_testing_dk "Light LC"          | Light LC servers with Scene servers                   | 0x501                      | 0x502                            | NA                               | 0x504                                   | ... |
| @ref light_lightness_example_testing_dk "Light lightness"   | Lightness servers with Scene servers                  | 0x401                      | NA                               | NA                               | 0x402                                   | ... |
| @ref light_switch_example_testing_dk "Light switch"         | Light switch servers with Scene servers               | 0x201                      | NA                               | NA                               | 0x202                                   | ... |
| @ref dimming_prov_prov_example "Dimming"                    | Dimming servers with Scene servers                    | 0x301                      | NA                               | NA                               | 0x302                                   | ... |
| @ref enocean_example_testing_dk "EnOcean translator client" | Any server that extends the generic OnOff <br>server, for example Light switch server. | Depends on the server used.    |Depends on the server used. |Depends on the server used. |Depends on the server used. |... |
| @ref fast_cadence_example_testing_dk "Sensor server"        | Sensor server                                         | 0x801                      | NA                               | NA                               | 0x802                                   | ... |


### Publish subscribe configuration @anchor provisioner_example_assignment_publish_subscribe

After getting provisioned, the device is configured depending on whether it is running a client example or a server example:
- For client, two instances of the models are instantiated on different elements.
  The first model instance is configured to publish to the odd group address (0xc003), and the second one is configured to publish to the even group address (0xc002).
  Similarly, these model instances are subscribed to the addresses of 0xc005 and 0xc004 respectively.
- For server, the subscription and publication addresses are set depending on whether the unicast address of the node is even or odd.
  Models on the nodes with even address publish to 0xc004 and are subscribed to the address 0xc002.
  Models on the nodes with odd address publish to 0xc005 and are subscribed to the address 0xc003.
  (Light LC server is additionally subscribed to receive sensor server statuses on 0xc004 and 0xc005, respectively).

### Provisioner state machine @anchor provisioner_example_assignment_state_machine

The following diagram shows the typical state transitions of the provisioner while provisioning and configuring a light switch client.

![Light switch client state diagram](images/light_switch_client_state_diagram.svg "Light switch client state diagram")

For more information on how a provisioning process works, see the [provisioning guide](@ref md_doc_user_guide_modules_provisioning_main).

---

## Usage of Mesh APIs @anchor provisioner_APIs

The provisioner uses the following set of APIs:
- [Application support API modules](@ref MESH_API_GROUP_APP_SUPPORT)
- [Management API module](@ref MESH_STACK)
- [Core mesh stack API](@ref MESH_API_GROUP_CORE)
- [Provisioning API](@ref MESH_API_GROUP_PROV)
- [Configuration client API](@ref CONFIG_CLIENT)

The provisioner role is much more complex than the provisionee role,
both in terms of resource requirements and application complexity.
For these reasons, there is no simple API for the provisioning and configuration process.

However, for a specific use case, the API usage can be reduced into a set of
simple steps, as implemented in the provisioner example:

-#  Initialize:
    -#  Core Bluetooth mesh stack.
    -#  Device state manager.
    -#  Access layer.
    -#  Load provisioner persistent data.
-#  Listen for unprovisioned beacons.
-#  Provision the device.
-#  Configure the device.
-#  If more devices should join the network, go back to step 2.

@note
If the provisioner is not able to configure a device, you will receive a notification
and can press a button to provision the next device or to try configuration one more time.

In the example code, this behavior is split between the following modules:
- `examples/provisioner/src/main.c` -- deals with the initialization and setup of the Bluetooth mesh stack.
- `examples/provisioner/src/provisioner_helper.c` -- deals with the provisioning process.
- `examples/provisioner/src/node_setup.c` -- deals with the configuration process of the
node once the provisioning is completed.

The following figure shows the details of how provisioning and configuration are implemented with the provided APIs.
Note that the figure may simplify some API calls to provide a clearer picture.
See the relevant source files for details.

![Provisioning and configuring devices](images/provisioner_interface.svg "Provisioning and configuring devices")

---


## Hardware requirements @anchor provisioner_example_hw_requirements

You need one of the compatible development kits for the provisioner, in addition to the development kits required by the example you are using the provisioner with.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.


---

## Setup @anchor provisioner_example_setup

You can find the source code of the Bluetooth mesh provisioner in the following folder:
`<InstallFolder>/examples/provisioner`

### LED and button assignments @anchor provisioner_example_setup_leds_buttons

The buttons (1 to 4) are used to initiate certain actions, and the LEDs (1 to 4) are used to reflect
the status of actions as follows:
  - Button 1: Start provisioning.
  - Button 4: Clear all the states and reset the node.
  - LED 1: Reflects the state of the provisioning.
		- LED ON: Provisioning of the node is in progress.
		- LED OFF: No ongoing provisioning process.
  - LED 2: Reflects the state of the configuration.
		- LED ON: Configuration of the node is in progress.
		- LED OFF: No ongoing configuration process.


---

## Testing the example @anchor provisioner_example_testing

There is no standalone testing procedure for the provisioner example.
You need to use this example together with one of the examples that requires the static provisioner example.
These examples are listed at the beginning of this page.
Make sure to check their documentation pages, especially the section about testing the example.

### Specifying the maximum number of provisionee devices @anchor provisioner_example_testing_40boards

If you have more than 40 servers and clients in the example you want to use the provisioner with,
make sure to set `MAX_PROVISIONEE_NUMBER`. This define specifies the maximum number of provisionee
devices that the provisioner is able to introduce in the network.

Defining this value is important for setting key device array and TTL value, so that the network can work properly.
This is because the provisioner keeps the device key for every provisioned device.
Updating this define is also useful for calculating the replay cache size, which must be equal or higher to `MAX_PROVISIONEE_NUMBER`.
Otherwise, the transport layer will reject all incoming traffic from new devices, because it cannot guarantee protection from a replay attack.

Complete the following steps:
1. In `example_network_config.h`, set `MAX_PROVISIONEE_NUMBER` to the total number of servers and clients available, if it is higher than 40.
-# If the value of `MAX_PROVISIONEE_NUMBER` increases, also scale up the replay protection list size (@ref REPLAY_CACHE_ENTRIES).
-# Rebuild the provisioner example.
-# Set `MAX_AVAILABLE_SERVER_NODE_NUMBER` in `nrf_mesh_config_app.h` of the client example to the value set for `MAX_PROVISIONEE_NUMBER`.

### Provisioning examples without Scene Setup Server model @anchor provisioner_example_no_scene_setup_server

[Scene Setup Server model](@ref SCENE_SETUP_SERVER) does not have a corresponding server example of its own
and is instead showcased by several examples:
- [Dimming examples](@ref md_examples_dimming_README)
- [Light switch example](@ref md_examples_light_switch_README)
- [Light lightness example](@ref md_examples_light_lightness_README)
- [Light LC server example](@ref md_examples_light_lc_server_README)
- [Light CTL example](@ref md_examples_light_ctl_README)

You can exclude this model for these examples.
If you decide to do so, you will also need to exclude the Scene Setup Server model
from the static provisioner to be able to provision the server example. Complete the following steps:
1. Open the file `examples/provisioner/include/nrf_mesh_config_app.h`.
-# Locate the `SCENE_SETUP_SERVER_INSTANCES_MAX` define. By default, its value is set to `1`.
-# Set the value to `0` and recompile.

### Evaluating examples using the static provisioner @anchor provisioner_example_evaluating

After building the examples (the provisioner example and the examples you want to evaluate)
by following the instructions in [Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build),
complete the following steps:
1. Flash the examples by following the instructions in @ref md_doc_getting_started_how_to_run_examples,
including:
    -# Erase the flash of your development boards and program the SoftDevice.
    -# Flash the provisioner firmware on one board and the client or the server example firmware (or both) on other boards.
-# After all boards are flashed, press Button 1 on the provisioner board
to start the provisioning process. See the [Network configuration and address assignment](@ref provisioner_example_assignment) section for details.
    - The sequence of provisioned devices depends on the sequence of received unprovisioned beacons.
    - You can use [RTT viewer](@ref segger-rtt) to view the RTT output generated by the provisioner.
    The provisioner prints details about the provisioning and the configuration process in the RTT log.
-# Observe that the LED 1 on the provisioner board is turned ON when the provisioner is scanning and provisioning a device.
-# Observe that the LED 2 on the provisioner board is turned on when the configuration procedure is in progress.
-# Wait until LED 1 on the provisioner board remains lit steadily for a few seconds, which indicates that
all available boards have been provisioned and configured.

If the provisioner encounters an error during the provisioning or configuration process for a certain node,
you can reset the provisioner to restart this process for that node.
