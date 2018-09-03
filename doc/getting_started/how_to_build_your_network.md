# Exploring Mesh APIs using light switch example

The light switch example is meant to showcase the APIs for the provisioner and provisionee roles
and how a simple Bluetooth Mesh network may be set up and configured. The network consists of one static
provisioner (that configures a network in a fixed predefined way), one light switch
(that implements a Generic OnOff client), and one or more light bulbs (that implement Generic OnOff servers).


#### Provisionee: light switch server

The server uses the following set of APIs:

1. [Application support modules](@ref MESH_API_GROUP_APP_SUPPORT)
2. [Initialization module](@ref MESH_STACK)
3. [Core mesh stack](@ref MESH_API_GROUP_CORE)
4. [Generic OnOff server model](@ref GENERIC_ONOFF_MODEL)

The initialization module implements the behavior of a provisionee device. It handles the
interface with the provisioning stack, setting up the configuration server, and restoring the device
state from flash.

The main application (`examples/light_switch/server/src/main.c`) implements the following functionality:

1.  Setting basic configuration parameters, supported Out-Of-Band (OOB) methods, clock configuration, callbacks,
    etc.
2.  Adding models and their event callbacks.

When the `provisioning_complete_cb()` callback is called, the device is provisioned and ready to be
configured by the provisioner. The following figure illustrates the setup with the relevant API calls:

![Figure 2: Light switch server setup](img/light_switch_server_interface.svg "Figure 2: Light switch server setup")


#### Provisionee: Light switch client

The client uses the following set of APIs:

1.  [Application support modules](@ref MESH_API_GROUP_APP_SUPPORT)
2.  [Initialization module](@ref MESH_STACK)
3.  [Core mesh stack](@ref MESH_API_GROUP_CORE)
4.  [Generic OnOff client](@ref GENERIC_ONOFF_MODEL)

The client application is implemented in a similar way as that of the server
(see `examples/light_switch/client/src/main.c`). Additionally, it has the following functionality:

1.  Handle button presses and call [Generic OnOff client APIs](@ref SIMPLE_ON_OFF_CLIENT) to send
mesh messages to the desired nodes or a group of nodes.
2.  Handle model callbacks and print corresponding messages in RTT log.

![Figure 2: Light switch server setup](img/light_switch_client_interface.svg "Figure 3: Light switch server setup")

#### Provisioner: Setting up the network

The provisioner uses the following set of APIs:

1.  [Application support modules](@ref MESH_API_GROUP_APP_SUPPORT)
2.  [Initialization module](@ref MESH_STACK)
3.  [Core mesh stack](@ref MESH_API_GROUP_CORE)
4.  [Provisioning](@ref MESH_API_GROUP_PROV)
5.  [Configuration client](@ref CONFIG_CLIENT)

In general, the provisioner role is an order of magnitude more complex than the provisionee role, both in
resource requirements and application complexity. Therefore, there is no simple "press play and it
works"-API for the provisioner. However, for a specific use case, it can be reduced into a set of
simple steps, as implemented in the provisioner example:

1.  Initialize:
    1.  Core mesh stack
    2.  Device state manager
    3.  Access layer
    4.  (Optional) Load flash configuration.
2.  Listen for unprovisioned beacons.
3.  Provision device.
4.  Configure device.
5.  If more devices should join the network, go back to step 2.

In the example code, this behavior is split between the following modules:

- `examples/light_switch/provisioner/src/main.c`: Deals with the initialization and setup of the mesh stack.
- `examples/light_switch/provisioner/src/provisioner_helper.c`: Deals with the provisioning process.
- `examples/light_switch/provisioner/src/node_setup.c`: Deals with the configuration process of the
node once the provisioning is completed.

The following figure shows the details of how provisioning and configuration are implemented with the provided APIs. Note that the
figure may simplify some API calls to provide a clearer understanding. See the relevant source
files for details.

![Figure 3: Provisioning and configuring devices](img/light_switch_prov_interface.svg "Figure 3: Provisioning and configuring devices")


