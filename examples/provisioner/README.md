# Provisioner example
@anchor provisioner_example

The provisoner example is the default Mesh network configurator.
It works in a fixed, predefined way and can be used as the static provisioner with the following examples:
- [Dimming examples](@ref md_examples_dimming_README)
- [EnOcean switch translator client example](@ref md_examples_enocean_switch_README)
- [Light switch example](@ref md_examples_light_switch_README)
- [Light lightness example](@ref md_examples_light_lightness_README)
- [Light LC server example](@ref md_examples_light_lc_server_README)
- [Light CTL example](@ref md_examples_light_ctl_README)


**Table of contents**
- [Usage of Mesh APIs](@ref provisioner_APIs)
- [Hardware requirements](@ref provisioner_example_hw_requirements)
- [Setup](@ref provisioner_example_setup)
    - [LED and button assignments](@ref provisioner_example_setup_leds_buttons)
- [Testing the example](@ref provisioner_example_testing)

Because of the asynchronous nature of the provisioning and configuration process, the provisioner is implemented as a multi-layered state machine.
The application provisions and configures devices when it receives unprovisioned beacons from them.

Every supported example has a unique Uniform Resource Identifier (URI) included in the beacons.
The provisioner example uses URI hash to understand the type of example being provisioned and
select the suitable configuration to be applied to the node after provisioning.
For this reason, this provisioner can provision devices only with the known URI.
The list of URIs is predefined.

The static provisioner has its own limitations and is provided as a tool to evaluate SDK examples without the need to use a mobile application provisioner.
If you are using a mobile application for provisioning, you can see the models present on the device and configure the nodes accordingly.
Moreover, you can also use more sophisticated automated provisioners that can use the UUID, URI, and the device composition data to understand which devices are being provisioned and
configure them automatically.

The device being provisioned is configured depending on whether it has the client or the server part of the models:
- For client, two instances of the models are instantiated on different elements.
  The first model instance is subscribed to the odd group address (0xC005), and the second one is subscribed to the even group address (0xC004).
  Similarly, these model instances are assigned publish addresses of 0xC003 and 0xC002 respectively.
- For server, only the subscription and publication addresses are set, depending on whether the unicast address of the node is even or odd.
  Models on the nodes with even address publish to 0xC004 and are subscribed to the address 0xC002.
  Models on the nodes with odd address publish to 0xC005 and are subscribed to the address 0xC003.

The following diagram shows how the provisioner configures Mesh examples.

![Typical configuration of Mesh examples by the provisioner](images/provisioning_cfg.svg "Typical configuration of Mesh examples by Provisioner")

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
    -#  Core mesh stack.
    -#  Device state manager.
    -#  Access layer.
    -#  Load provisioner persistent data.
-#  Listen for unprovisioned beacons.
-#  Provision the device.
-#  Configure the device.
-#  If more devices should join the network, go back to step 2.

In the example code, this behavior is split between the following modules:
- `examples/provisioner/src/main.c` -- deals with the initialization and setup of the mesh stack.
- `examples/provisioner/src/provisioner_helper.c` -- deals with the provisioning process.
- `examples/provisioner/src/node_setup.c` -- deals with the configuration process of the
node once the provisioning is completed.

The following figure shows the details of how provisioning and configuration are implemented with the provided APIs.
Note that the figure may simplify some API calls to provide a clearer picture.
See the relevant source files for details.

![Provisioning and configuring devices](images/provisioner_interface.svg "Provisioning and configuring devices")

---


## Hardware requirements @anchor provisioner_example_hw_requirements

You need one of the supported boards for the provisioner, in addition to the boards required by the example you are using the provisioner with.

See @ref md_doc_user_guide_mesh_compatibility for the supported boards.


---

## Setup @anchor provisioner_example_setup

You can find the source code of the mesh provisioner in the following folder:
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
You need to use this example together with one of the examples that use the static provisioner example.

After building the example by following the instructions in [Building the mesh stack](@ref md_doc_getting_started_how_to_build),
see the following sections, depending on the example you are using:
- [Evaluating the light switch example using the static provisioner](@ref light_switch_example_testing_dk)
- [Evaluating the experimental dimming examples using the static provisioner](@ref dimming_prov_prov_example)
- [Evaluating the EnOcean switch example using the static provisioner](@ref enocean_example_testing_dk)
- [Evaluating the light lightness example using the static provisioner](@ref light_lightness_example_testing_dk)
- [Evaluating the light LC server example using the static provisioner](@ref light_lc_server_example_testing_dk)
- [Evaluating the Light CTL example using the static provisioner](@ref light_ctl_example_testing_dk)
