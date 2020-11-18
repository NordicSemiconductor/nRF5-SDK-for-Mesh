# Light switch client details and Bluetooth mesh APIs
@anchor light_switch_demo_client

The light switch client implements a Generic OnOff client. Together with [light switch server](@ref md_examples_light_switch_server_README)
and [Bluetooth mesh provisioner](@ref md_examples_provisioner_README), it is part of the [light switch example](@ref md_examples_light_switch_README)
network demonstration, in which it has a provisionee role.

The light switch client has four buttons to control the state of LED 1 on servers. It instantiates two instances of Generic OnOff Client model.
It can either be provisioned and configured by the provisioner device or by a GATT-based provisioner. The provisioner configures this client model instances
to communicate with servers.

---

## Source code

You can find the source code of the light switch client in the following folder:
`<InstallFolder>/examples/light_switch/client`

To run the light switch client example, see @ref md_examples_light_switch_README.

---

## Use of Bluetooth mesh APIs

The client uses the following set of APIs:

- [Application support modules](@ref MESH_API_GROUP_APP_SUPPORT)
- [Management module](@ref MESH_STACK)
- [Core mesh stack](@ref MESH_API_GROUP_CORE)
- [Generic OnOff client](@ref GENERIC_ONOFF_MODEL)

The client application is implemented in a similar way as that of the server
(see `examples/light_switch/client/src/main.c`). Additionally, it has the following functionalities:
- Handle button presses and call [Generic OnOff client APIs](@ref SIMPLE_ON_OFF_CLIENT) to send
Bluetooth mesh messages to the desired nodes or a group of nodes.
- Handle model callbacks and print corresponding messages in RTT log.


The following figure shows the calling sequence of key Bluetooth mesh stack APIs used by the light switch client.

![Light switch client setup](images/light_switch_client_interface.svg "Light switch client setup")