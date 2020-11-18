# Scene example

@tag52840and52833and52832
@tag52810and52820nosupport

This example demonstrates how you can use the [Scene model](@ref SCENE_MODELS)
to store and recall states of other models.

The example is composed of the Scene client minor example.
It does not provide a standalone Scene server example, but instead uses one of the compatible server
examples listed in [Server example (model)](@ref scene_example_scene_server) and
[Client example (model)](@ref scene_example_clients).

For provisioning purposes, the example requires either the provisioner example
that is provided in the @ref md_examples_provisioner_README or the nRF Mesh mobile app.

The Scene client example has a provisionee role in the network.
It supports provisioning over Advertising bearer (PB-ADV) and GATT bearer (PB-GATT)
and also support Bluetooth mesh Proxy Service (Server).
Read more about the Proxy feature in @ref md_doc_user_guide_modules_provisioning_gatt_proxy.

**Table of contents**
- [Scene client example](@ref scene_example_scene_client)
- [Server example (model)](@ref scene_example_scene_server)
- [Client example (model)](@ref scene_example_clients)
- [Scene Client and Server model](@ref scene_example_scene_model)
- [Hardware requirements](@ref scene_example_hw_requirements)
- [Software requirements](@ref scene_example_sw_requirements)
- [Setup](@ref scene_example_setup)
    - [LED and button assignments](@ref scene_example_setup_leds_buttons)
- [Testing the example](@ref scene_example_testing)
    - [Evaluating using the static provisioner](@ref scene_example_testing_dk)
    - [Evaluating using the nRF Mesh mobile app](@ref scene_example_testing_app)
    - [Interacting with the boards](@ref scene_example_testing_interacting)
        - [Storing scenes](@ref scene_example_testing_storing)
        - [Recalling scenes](@ref scene_example_testing_recalling)
        - [Getting the current scene and register](@ref scene_example_testing_getting)
        - [Deleting scenes](@ref scene_example_testing_deleting)


---
## Scene client example @anchor scene_example_scene_client

The Scene client example has a provisionee role in the network.
It implements two instances of the [Scene Client model](@ref SCENE_CLIENT).
These instances are used to store and recall the server states in a scene, and to delete
the stored scenes.

---
## Server example (model) @anchor scene_example_scene_server

There is no standalone Scene server example.
Instead, other example applications showcase the Scene Server models
and implement the Scene Server and Scene Setup Server models:
- [Light switch server example](@ref md_examples_light_switch_README)
- [Dimming server example](@ref dimming_example_dimming_server)
- [Light Lightness server example](@ref light_lightness_example_light_lightness_server)
- [Light CTL server example](@ref light_ctl_example_light_ctl_server)
- [Light LC server model example](@ref md_examples_light_lc_server_README)

This is because the Scene Server models refer to the scene register table that gathers scenes (with
the associated state values) from a number of elements within a node.

---
## Client example (model) @anchor scene_example_clients

If you plan to evaluate using static provisioner, you need one of the following
client examples that can control the state on the server example:
- [Light switch client example](@ref md_examples_light_switch_README)
- [Dimming client example](@ref dimming_example_dimming_client)
- [Light Lightness client example](@ref light_lightness_example_light_lightness_client)
- [Light CTL client example](@ref light_ctl_example_light_ctl_server)


@note There is no client example for Light LC Client model, therefore scene storage for Light LC
Server model cannot be evaluated using static provisioner.

If you plan to evaluate using @link_nrf_mesh_app, you do not need the client
example, as the mobile application allows you to control server states directly from the
application.

@note As of the nRF SDK for Mesh v5.0.0, the mobile app has a limited built-in client
support. Therefore, not all states of all server models can be controlled through this app.
The built-in client support is limited to Generic OnOff server and Generic Level servers.


---
## Scene Client and Server model @anchor scene_example_scene_model

The Scene Client model is used for manipulating the scene state in the Scene Server model.
For more information about the Scene models, see the [Scene model documentation](@ref SCENE_MODELS),
[Scene Server behavior documentation](@ref APP_SCENE), and Bluetooth SIG's @link_ModelOverview.

---
## Hardware requirements @anchor scene_example_hw_requirements

You need at least two compatible development kits for this example:

- One compatible development kit for the Scene client example.
- One or more compatible development kits for the server examples that showcase the Scene Server models.

Additionally, you need one of the following for provisioning:
- Two compatible development kits if you decide to use the [static provisioner example](@ref md_examples_provisioner_README):
  - One for the provisioner.
  - One for the suitable [client example](@ref scene_example_clients) to control server states.
- An iOS or Android smartphone if you decide to provision using the @link_nrf_mesh_app mobile application.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.


---

## Software requirements @anchor scene_example_sw_requirements

Depending on the provisioning method:
- If you decide to provision using a mobile application, you need @link_nrf_mesh_app (@link_nrf_mesh_app_ios or @link_nrf_mesh_app_android) installed on the smartphone.
- If you decide to use the static provisioner example, you need the [provisioner example](@ref md_examples_provisioner_README).


---

## Setup @anchor scene_example_setup

You can find the source code of this example in the following folder:
`<InstallFolder>/examples/scene`

See the Setup section in the documentation of the [server example you want to use](@ref scene_example_scene_server)
for the source code of the server example.

### LED and button assignments @anchor scene_example_setup_leds_buttons

- Scene Client:
    - When interacting with the boards, you can use one of the following options:
        - RTT input (recommended): Due to a limited number of buttons on the DK board,
        use the following RTT input when evaluating this example:
            | RTT input     | DK Button     | Effect                                                                                                                                  |
            |---------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------|
            | `1` | Button 1 | The Scene Store message is sent with the current scene number. |
            | `2` | Button 2 | The Scene Recall message is sent with the current scene number. |
            | `3` | Button 3 | The Scene Delete message is sent with the current scene number. |
            | `4` | Button 4 | Switches the client instance used for sending. |
            | `5` | -        | Decreases the current scene number by 1. |
            | `6` | -        | Increases the current scene number by 1. |
            | `7` | -        | Send a Scene Get message to get the currently active scenes. |
            | `8` | -        | Send a Scene Register Get message to get the contents of the scene registers. |
        - Buttons: If you decide to use the buttons on the DK instead of the RTT input, you can only
        send Scene Store, Scene Recall and Scene Delete messages for scene number 1.


---

## Testing the example @anchor scene_example_testing

To test the example, build both the Scene client example and the server example you want to use
by following the instructions in [Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).

After building is complete, use one of the following methods, depending on the preferred
provisioning approach:
- [Evaluating using the static provisioner](@ref scene_example_testing_dk)
- [Evaluating using the nRF Mesh mobile app](@ref scene_example_testing_app)


### Evaluating using the static provisioner @anchor scene_example_testing_dk

See [provisioner example testing section](@ref provisioner_example_evaluating) for detailed steps required
to provision and configure the boards using the static provisioner.

### Evaluating using the nRF Mesh mobile app @anchor scene_example_testing_app

See [Evaluating examples using the nRF Mesh mobile application](@ref nrf-mesh-mobile-app) for detailed steps required
to provision and configure the boards using the nRF Mesh mobile app.

The following naming convention is used in the app:
- The client board is `nRF5x Mesh Scene Client`.
- The server boards' naming convention is detailed in the corresponding documentation section
of the [server example you are using](@ref scene_example_scene_server).

The following model instances must be configured in the app for this example:
- For the `nRF5x Mesh Scene Client` client board: Scene Client.
- The server boards' model instances convention are detailed in the corresponding section
of the [server example you are using](@ref scene_example_scene_server).
These include the Scene models, which are marked as optional there.

@note To change the states on the server example you decide to use, you can also add and configure
a client example associated with the corresponding server example.

Once the provisioning is complete, you can start [interacting with the boards](@ref scene_example_testing_interacting).

@note
You can also configure the publish address of the second Scene client model instance.
To do this, repeat [step 3 from binding nodes](@ref nrf-mesh-mobile-app-binding) and
[all steps from setting publication](@ref nrf-mesh-mobile-app-publication).


### Interacting with the boards @anchor scene_example_testing_interacting

Once the provisioning and the configuration of the client node and of at least one of
the server nodes are complete, you can press buttons on the client or send command
numbers using the RTT Viewer to create and recall scenes and observe how the server states change.

The following set of message types is available for this demonstration:
- Scene Get
- Scene Register Get
- Scene Store
- Scene Recall
- Scene Delete

See [LED and button assignments](@ref scene_example_setup_leds_buttons)
section for the full list of available commands.

If any of the devices is powered off and then back on, it will remember its flash configuration
and rejoin the network. It will also restore values of the Scene states.
For more information about the flash manager, see @ref md_doc_user_guide_modules_flash_manager.

#### Storing scenes @anchor scene_example_testing_storing

Before storing each scene change the server examples states using client example (see
corresponding client example documentation [here](@ref scene_example_clients)) or
@ref nrf-mesh-mobile-app. Changing server states will allow you to visually distinguish various
scenes during evaluation.

To store the states of the servers in a scene, perform the following steps:
-# Set the states of each server to a desired value by following
the Interacting with the boards section in the documentation of the [server example you are using](@ref scene_example_scene_server).
To know which states are stored and recalled, see the Scene model section in the server example documentation.
-# Use RTT input `1` or Button 1 on the client node to store these states as a scene with scene number 1.
-# Alter the states on the server nodes.
-# Use RTT input `6` on the client node to change to scene number 2.
-# Use RTT input `1` or Button 1 on the client node to store these states as scene number 2.

#### Recalling scenes @anchor scene_example_testing_recalling

To recall any of the scenes stored in the previous step, use RTT input on the client node:
-# Use RTT input `5` and `6` to select the scene number to recall.
-# Use RTT input `2` or press Button 2 to recall the scene.
-# Observe the change in the states on the server nodes. To know which states are
stored and recalled, see the Scene model section in the documentation
of the [server example you are using](@ref scene_example_scene_server).

#### Getting the current scene and register @anchor scene_example_testing_getting

To get the currently active scene on the servers, use RTT input `7` on the
Scene client. Observe the RTT output to see the Scene status messages.

If you use RTT input `8` on the client node, a Scene Register get message will be sent to
the servers. The RTT output will list the scene register content of each server.

#### Deleting scenes @anchor scene_example_testing_deleting

To delete the stored scene, use RTT input `5` and `6` to select a scene number,
and then use RTT input `3` or Button 3 to delete the selected scene. Observe that
the deleted scenes can no longer be recalled. They also do not show up in the
scene register when you use RTT input `8`.
