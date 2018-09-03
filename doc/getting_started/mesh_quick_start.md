# Quick Start Guide for the nRF5 SDK for Mesh

This quick start guide gives a quick demonstration of the Bluetooth Mesh
network using Nordic's nRF5 SDK for Mesh and introduces you to some of the basic concepts.
You don't need to build any binaries for this demonstration as we will use pre-built binaries of the [light-switch examples](@ref md_examples_light_switch_README)
as a starting point for this guide.

The Bluetooth @link_MeshSpec is developed and published by the
@link_BluetoothSIG<!--http://www.bluetooth.org/-->. It allows one-to-one, one-to-many, and many-to-many
communication. It uses BLE protocol to exchange messages between the nodes on the network.
The nodes can communicate with each other as long as they are in direct
radio range of each other or there are enough devices available that are
capable of listening and forwarding these messages.

The end-user applications (such as Luminaire control) are defined with the help
of client-server Mesh Models defined in the @link_ModelSpec.

The [light-switch examples](@ref md_examples_light_switch_README) (`examples/light-switch`) demonstrate the major parts of
the mesh network ecosystem.

These examples contain three sub-examples:
- Light Switch Server: A minimalistic server implementing a vendor-specific
[Generic OnOff server model](@ref GENERIC_ONOFF_MODEL) that is used to
receive the state data and control the state of LED 1 on the board.
- Light Switch Client: A minimalistic client implementing four instances of a vendor-specific
[Generic OnOff client model](@ref GENERIC_ONOFF_MODEL).
When a user presses any of the buttons, an OnOff Set message is sent out to the
configured destination address.
- Mesh Provisioner: A simple static provisioner implementation. This provisioner provisions all
the nodes in one mesh network. Additionally, the provisioner also configures bindings
and publication and subscription settings of mesh model instances on these nodes
to enable them to talk to each other.

These three example applications will be referred to as the client, the server, and the provisioner
in the following text.

We will use an nRF52 DK for this demonstration (see [Figure 1](@ref qs-Figure1)). The buttons (Button 1..4) are used
to initiate certain actions, and the LEDs (LED 1..4) are used to reflect the status of actions as follows:

- Provisioner:
  - Button 1: Start provisioning.
  - LED 1: ON: Provisioning of the node is in progress.
  - LED 2: ON: Configuration of the node is in progress.

- Client:

  During provisioning process:
  - LED 1..4: Blink four times to indicate provisioning process is completed.

  After provisioning and configuration is over, buttons on the client are used to send OnOff Set
  messages to the servers:
    - Button 1: Send a message to the odd group (address: 0xC003) to turn on LED 1.
    - Button 2: Send a message to the odd group (address: 0xC003) to turn off LED 1.
    - Button 3: Send a message to the even group (address: 0xC002) to turn on LED 1.
    - Button 4: Send a message to the even group (address: 0xC002) to turn off LED 1.

- Server:

  During provisioning process:
  - LED 1..4: Blink four times to indicate provisioning process is completed.
  <br>
  After provisioning and configuration is over:
  - LED 1: Reflects the value of OnOff state on the server.
    <br>
    LED ON: Value of the OnOff state is zero.
    <br>
    LED OFF: Value of the OnOff state is one.
    <br>
    Note: LEDs are wired as active low on the nRF52 DK.


@anchor qs-Figure1
![Figure 1. nRF52 DK illustration](img/pca10040_front_v1.0.0.svg "nRF52 DK illustration")

The following figure gives the overall view of the mesh network that we will set up
in this demonstration. Numbers in parentheses indicate the addresses that are assigned
to these nodes by the provisioner.

![Figure 2. Mesh network demonstration](img/mesh-nw-demo_r02.svg "Mesh network demonstration")

## Hardware requirements
At least three nRF5 development boards are required for this demo. See [compatiblity](@ref readme-compatibility) section for the supported boards.

- One nRF5 development board for the client.
- One nRF5 development board for the provisioner.
- One or more nRF5 development boards for the servers (maximum up to 30 boards).

## Software requirements

1. nRF5 SDK for Mesh.
@link_MeshSdk_download <!-- <a href=\"http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK-for-Mesh#Downloads\">Download</a> --> and extract the SDK archive.

2. nrfjprog (recommended for flashing the devices). Download and install the
@link_nrf5x_cmd_line_tools_w32 <!-- nRF5x command line tools: http://www.nordicsemi.com/eng/nordic/Products/nRF51822/nRF5x-Command-Line-Tools-Win32/33444 -->
or the @link_nrf5x_cmd_line_tools_linux<!--nRF5x command line tools: https://www.nordicsemi.com/eng/nordic/Products/nRF51822/nRF5x-Command-Line-Tools-Linux64/51386 -->.

3. <a href="https://www.python.org/downloads/" target="_blank">Python 3</a> or <a href="https://www.python.org/downloads/" target="_blank">Python 2.7</a>.

> **Important:** For Debian/Ubuntu, you must reload the udev rules after installing the nRF5x Command Line Tools:
>
>     $ sudo udevadm control --reload
>     $ sudo udevadm trigger --action=add

## Flashing the example firmware @anchor qs-flashing-firmware

1. Connect the nRF5 boards to the USB ports and decide which one you want to use as a
client and which one as a provisioner by noting down their 9-digit Segger IDs.

2. Execute the python script `scripts/quick_start/quick_start_demo.py` by specifying the
   Segger IDs for the provisioner and client boards as follows:

       nrf5_sdk_for_mesh$ python scripts/quick_start/quick_start_demo.py

The script will ask you to choose the provisioner and client boards and it will flash
the required SoftDevice and example firmware on the boards. At the end the script executes a reset operation to
start the example applications.

Note:
  1. You can manually specify Segger IDs for the provisioner (`-p`) and client (`-c`) boards as follows:

         nrf5_sdk_for_mesh$ python scripts/quick_start/quick_start_demo.py -p 682438729 -c 682204868

  2. There is no command line argument for programming server boards. The script automatically
     loads the server firmware to all connected boards other than the specified ones.
  3. If you do not have a sufficient number of USB ports, you can program the boards one by one. In
    this case, switch off or disconnect the boards that you have finished programming to
    prevent them from being overwritten by the script.
  4. Use the command line switch `-v` if you want to increase the verbosity of the output.

## Running and observing the demonstration

After the reset, the provisioner waits for user input. Follow these steps to see the mesh network in action:

1. Press Button 1 on the provisioner board to start the provisioning process.

   The provisioner first provisions and configures the client and assigns the address 0x100 to the client
   node. The two instances of the OnOff client models are instantiated on separate secondary elements.
   Therefore, they get consecutive addresses starting with 0x101.
   After this, the provisioner picks up the available devices at random, assigns them consecutive addresses, and adds them to odd and even groups.

   Observe the LED status on the provisioner, client, and server boards.
2. Wait until LED 1 on the provisioner board remains ON steadily for a few seconds, indicating that
   all available boards have been provisioned and configured.

   Now you can press buttons on the client board to change the state of LED 1 on the server boards.
3. Press Button 1 on the client board to turn ON LED 1 on all servers with ODD addresses.
4. Press Button 2 on the client board to turn OFF LED 1 on all servers with ODD addresses.
5. Press Button 3 on the client board to turn ON LED 1 on all servers with EVEN addresses.
6. Press Button 4 on the client board to turn OFF LED 1 on all servers with EVEN addresses.

7. If you want to see the RTT logs printed during the provisioning and configuration process,
 connect J-Link RTT viewer to the provisioner or client board and repeat the above steps
 starting from [Flashing the example firmware](@ref qs-flashing-firmware).

8. In the client's RTT log status, observe messages sent by the servers in response to acknowledged
Set messages. The client example sends acknowledged Set messages only to odd servers, and hence only those
servers respond with status messages. Additionally, the provisioner enables publication for all servers,
so that they can publish messages to their corresponding group client.

9. Press Button 1 on the servers to locally change the state of LED 1 and observe that the client receives
the status message from the corresponding server containing the new state value.


## More information and further reading

See the documentation of the [light-switch example](@ref md_examples_light_switch_README) for more detailed information
about these examples.

You will need to install toolchain for building the mesh stack and examples.
See the @ref md_doc_getting_started_getting_started documentation for information on environment setup,
installing the mesh toolchain, building the mesh stack, and more.
