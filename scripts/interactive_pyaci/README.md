# Interactive PyACI

The Interactive Python Application Controller Interface (PyACI) (`interactive_pyaci.py`) can be used to
interactively control devices running the mesh stack and the serial interface. The script opens
up one or more COM ports and enables an interactive Python command line.

The interface consists of
- `aci/aci_cmd.py`: command class definitions (serialization)
- `aci/aci_evt.py`: event class definitions (de-serialization)
- `aci/aci_uart.py`: the UART serial driver
- `aci/aci_utils.py`: utility functions and class definitions
- `aci/aci_config.py`: utility class for parsing firmware configuration file (`nrf_mesh_app_config.h`)

**Important:** `aci/aci_cmd.py` and `aci/aci_evt.py` are auto-generated from the C header files of the serial
interface with the `tools/serial_doc` scripts. To re-generate the files, build the `serial_pyaci` target
(requires a CMake based setup).

## Prerequisites

The interactive console is written for Python 3.5. Install the requirements using `pip` like this:


    $ pip install -r requirements.txt


To follow this guide, you need at least two boards (`PCA10028`, `PCA10030`, or `PCA10040`) running
the [serial example](@ref md_examples_serial_README).

## Using the interface

To start the serial interface, simply call `python interactive_console.py -d <com port>`. The baud
rate is set to the default value used in the mesh serial stack. To disable logging to file, append
the `--no-logfile` argument.

### Hello World

After building and flashing the serial example, connect to the device with the interactive console:


    $ python interactive_pyaci.py -d /dev/ttyACM0 --no-logfile

        To control your device use d[x], type d[x]. and hit tab to see the available methods.
        x is for the device index; devices will be indexed based on the order of com ports
        given with the option -d(the first device, d[0], can also be accessed using device).

    Python 3.5.2 (default, Nov 17 2016, 17:05:23)
    Type 'copyright', 'credits' or 'license' for more information
    IPython 6.1.0 -- An enhanced Interactive Python. Type '?' for help.


Send an `echo` command to test that the device is connected:


    In [1]: send(cmd.Echo("hello world"))
    2017-08-02 10:06:29,338 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'hello world')}}


The `send` function is a helper function for sending commands to the first device. It accepts a
`CommandPacket` object as a parameter. All of these are imported from `aci/aci_cmd.py` by the
interactive script.

### Sending mesh packets

Let's set up a tiny mesh network. Connect two devices and start the serial interface:


    $ python interactive_pyaci.py -d /dev/ttyACM0 /dev/ttyACM1 --no-logfile


Next, add a network key, application key, and a local unicast address with the
`quick_setup()` method:


    In [1]: for dev in d: dev.quick_setup()
    2017-08-02 10:01:53,154 - INFO - ttyACM0: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,157 - INFO - ttyACM0: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,158 - INFO - ttyACM0: Success
    2017-08-02 10:01:53,161 - INFO - ttyACM1: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,162 - INFO - ttyACM1: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,166 - INFO - ttyACM1: Success


Add publish addresses:


    In [2]: d[0].send(cmd.AddrPublicationAdd(d[1].local_unicast_adress_start))
    2017-08-02 10:10:11,502 - INFO - ttyACM0: AddrPublicationAdd: {'address_handle': 15}
    In [3]: d[1].send(cmd.AddrPublicationAdd(d[0].local_unicast_adress_start))
    2017-08-02 10:10:13,132 - INFO - ttyACM1: AddrPublicationAdd: {'address_handle': 15}


Notice the handles; they are used to reference the addresses and keys at a later point.
Let's store them in some variables:


    In [4]: publish_handle = 15
    In [5]: appkey_handle = 0


Next, we test out sending a message between the devices:

    In [6]: d[0].send(cmd.PacketSend(appkey_handle, d[0].local_unicast_adress_start, publish_handle, 1, 0, "Hello World"))
    2017-08-02 10:15:18,073 - INFO - ttyACM0: Success
    2017-08-02 10:15:18,092 - INFO - ttyACM1: {event: MeshMessageReceivedUnicast, data: {'actual_length': 11, 'adv_addr_type': 1,            \
                                               'adv_addr': bytearray(b'\xe9\x04/\xcc\xcf\xf7'), 'src': 1, 'data': bytearray(b'Hello World'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 2, 'ttl': 1}}

    In [7]: d[1].send(cmd.PacketSend(appkey_handle, d[1].local_unicast_adress_start, publish_handle, 1, 0, "Hi there"))
    2017-08-02 10:15:42,806 - INFO - ttyACM1: Success
    2017-08-02 10:15:42,837 - INFO - ttyACM0: {event: MeshMessageReceivedUnicast, data: {'actual_length': 8, 'adv_addr_type': 1,           \
                                               'adv_addr': bytearray(b'\xd6\x89\x1c\x8d\r\xd7'), 'src': 2, 'data': bytearray(b'Hi there'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 1, 'ttl': 1}}


### Getting help

To read the documentation for one of the commands, enter a `?` before or after the command
object and press `<enter>`. E.g., for the `BeaconParamsSet` command, you would get the following:



    In [1]: ?cmd.BeaconParamsSet
    Init signature: cmd.BeaconParamsSet(beacon_slot, tx_power, channel_map, interval_ms)
    Docstring:
    Set parameters for application controlled beacon.

    Parameters
    ----------
        beacon_slot : uint8_t
            Slot number of the beacon to start.
        tx_power : uint8_t
            TX Power value, must be a value from @ref serial_cmd_tx_power_value_t.
        channel_map : uint8_t
            Channel map bitfield for beacon, starting at channel 37.
        interval_ms : uint32_t
            TX interval in milliseconds.
    File:           <nrf5_sdk_for_bluetooth_mesh>\scripts\interactive_pyaci\aci\aci_cmd.py
    Type:           type


The help prompt may be used for any python object or function.

The console also provides auto completion, i.e., typing `?cmd.BeaconParamsS` and pressing `<tab>`
will complete the command packet object.

For more details about the commands, see the [serial commands documentation](@ref md_doc_libraries_serial_cmd).

### Miscellaneous

As with the normal Python shell, you can do more complex scripting, e.g.:


    In [1]: import time
    In [2]: for i in range(0, 10): send(cmd.Echo("Hello: " + str(i))); time.sleep(1)
    2017-08-02 10:21:34,459 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 0')}}
    2017-08-02 10:21:35,378 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 1')}}
    2017-08-02 10:21:36,394 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 2')}}
    2017-08-02 10:21:37,400 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 3')}}
    2017-08-02 10:21:38,406 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 4')}}
    2017-08-02 10:21:39,440 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 5')}}
    2017-08-02 10:21:40,414 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 6')}}
    2017-08-02 10:21:41,420 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 7')}}
    2017-08-02 10:21:42,427 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 8')}}
    2017-08-02 10:21:43,442 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': bytearray(b'Hello: 9')}}


Here we use a simple `for`-loop to send 10 echo commands with a one second delay.

## Provisioning
To illustrate how to use a more complex application, we have implemented a sample provisioner and provisionee
class. In `provisioning.py`, you find three main classes:

* `ProvDevice`
* `Provisioner`
* `Provisionee`

The `ProvDevice` class is a base class that implements the common functionality between the provisioner and provisionee, which is
mainly authentication and encryption offloading.

Upon initialization, the `ProvDevice` registers the event handler of the provisioner/provisionee and adds an event
filter that suppresses the provisioning events from being printed by default. It also generates a new
public/private key pair and sets it for the device. If a provisioner is instantiated, the given network key and address
is added to the device. If a provisionee is instantiated, the number of elements given is configured in the
device capability structure.

### Example

Start the serial interface with at least two connected devices, all running the serial example.
Here, we use three devices:

    $ python interactive_pyaci.py -d COM96 COM97 COM98 --no-logfile

        To control your device use d[x], type d[x]. and hit tab to see the available methods.
        x is for the device index; devices will be indexed based on the order of com ports
        given with the option -d (the first device, d[0], can also be accessed using device).

    Python 3.5.2 (v3.5.2:4def2a2901a5, Jun 25 2016, 22:01:18) [MSC v.1900 32 bit (Intel)]
    Type 'copyright', 'credits' or 'license' for more information
    IPython 6.1.0 -- An enhanced Interactive Python. Type '?' for help.

    In [1]: p = Provisioner(d[0])
    2017-07-28 13:53:17,574 - INFO - COM96: Success
    2017-07-28 13:53:17,576 - INFO - COM96: Success
    2017-07-28 13:53:17,581 - INFO - COM96: SubnetAdd: {'subnet_handle': 0}

    In [2]: a = Provisionee(d[1]); b = Provisionee(d[2])
    2017-07-28 14:05:42,931 - INFO - COM97: Success
    2017-07-28 14:05:42,934 - INFO - COM98: Success
    2017-07-28 14:05:42,938 - INFO - COM97: Success
    2017-07-28 14:05:42,939 - INFO - COM98: Success
    2017-07-28 14:05:42,940 - INFO - COM97: Success
    2017-07-28 14:05:42,940 - INFO - COM98: Success

Now `d[0]` is instantiated as a provisioner. `d[1]` and `d[2]` are provisionees. Next, we start the
provisioner's scanning and the provisionees' listening:

    In [3]: p.scan_start()
    2017-07-28 14:11:18,040 - INFO - COM96: Success

    In [4]: a.listen(); b.listen()
    2017-07-28 14:11:29,529 - INFO - COM97: Success
    2017-07-28 14:11:29,532 - INFO - COM98: Success
    2017-07-28 14:11:29,828 - INFO - COM96: Received UUID 0AB8C5B084599859E9042FCCCF7729BD with RSSI: -31 dB
    2017-07-28 14:11:31,389 - INFO - COM96: Received UUID 667434C597F83695D358DB65D2CAE3EE with RSSI: -34 dB

Notice how `COM96`, our provisioner, reports the received UUIDs. Let us provision the first device:

    In [5]: p.provision()
    2017-07-28 14:13:17,287 - INFO - COM96: Success
    2017-07-28 14:13:17,293 - INFO - COM97: Link established
    2017-07-28 14:13:17,300 - INFO - COM96: Provision
    2017-07-28 14:13:17,318 - INFO - COM96: Link established
    2017-07-28 14:13:17,366 - INFO - COM96: Received capabilities
    2017-07-28 14:13:17,366 - INFO - COM96: Number of elements: 1
    2017-07-28 14:13:17,369 - INFO - COM96: OobUse
    2017-07-28 14:13:17,579 - INFO - COM96: ECDH request received
    2017-07-28 14:13:17,586 - INFO - COM97: ECDH request received
    2017-07-28 14:13:17,587 - INFO - COM96: EcdhSecret
    2017-07-28 14:13:17,592 - INFO - COM97: EcdhSecret
    2017-07-28 14:13:17,823 - INFO - COM96: Provisioning complete
    2017-07-28 14:13:17,824 - INFO - COM96:         Address(es): 0x10-0x10
    2017-07-28 14:13:17,824 - INFO - COM96:         Device key: 2D55D072A4EE5202018D4EC64F1475B8
    2017-07-28 14:13:17,825 - INFO - COM96:         Network key: 00000000000000000000000000000000
    2017-07-28 14:13:17,825 - INFO - COM96: Adding device key to subnet 0
    2017-07-28 14:13:17,825 - INFO - COM96: Adding publication address(es)
    2017-07-28 14:13:17,830 - INFO - COM96: DevkeyAdd: {'devkey_handle': 8}
    2017-07-28 14:13:17,831 - INFO - COM96: AddrPublicationAdd: {'address_handle': 15}
    2017-07-28 14:13:17,835 - INFO - COM97: Provisioning complete
    2017-07-28 14:13:17,835 - INFO - COM97:         Address(es): 0x10-0x10
    2017-07-28 14:13:17,836 - INFO - COM97:         Device key: 2D55D072A4EE5202018D4EC64F1475B8
    2017-07-28 14:13:17,836 - INFO - COM97:         Network key: 00000000000000000000000000000000
    2017-07-28 14:13:17,837 - INFO - COM97: Adding network key (subnet)
    2017-07-28 14:13:17,837 - INFO - COM97: Adding device key to subnet 0
    2017-07-28 14:13:17,838 - INFO - COM97: Setting the local unicast address range
    2017-07-28 14:13:17,842 - INFO - COM97: SubnetAdd: {'subnet_handle': 0}
    2017-07-28 14:13:17,843 - INFO - COM97: DevkeyAdd: {'devkey_handle': 8}
    2017-07-28 14:13:17,847 - INFO - COM97: Success
    2017-07-28 14:13:17,856 - INFO - COM97: Provisioning link closed
    2017-07-28 14:13:17,935 - INFO - COM96: Provisioning link closed

Running the cryptography on the host (ECDH offloading) speeds up the provisioning procedure. From the log, we
can see that the whole procedure is completed in approximately 0.6 seconds!

Provisioning the second device is as easy as the first:

    In [8]: p.provision()
    2017-07-28 14:17:01,923 - INFO - COM96: Success
    2017-07-28 14:17:01,930 - INFO - COM96: Provision
    2017-07-28 14:17:01,936 - INFO - COM98: Link established
    2017-07-28 14:17:01,992 - INFO - COM96: Link established
    2017-07-28 14:17:02,058 - INFO - COM96: Received capabilities
    2017-07-28 14:17:02,059 - INFO - COM96: Number of elements: 1
    2017-07-28 14:17:02,061 - INFO - COM96: OobUse
    2017-07-28 14:17:02,262 - INFO - COM96: ECDH request received
    2017-07-28 14:17:02,269 - INFO - COM96: EcdhSecret
    2017-07-28 14:17:02,274 - INFO - COM98: ECDH request received
    2017-07-28 14:17:02,280 - INFO - COM98: EcdhSecret
    2017-07-28 14:17:02,511 - INFO - COM96: Provisioning complete
    2017-07-28 14:17:02,512 - INFO - COM96:         Address(es): 0x11-0x11
    2017-07-28 14:17:02,513 - INFO - COM96:         Device key: 40FD35B4223291ECF757471D7C606259
    2017-07-28 14:17:02,513 - INFO - COM96:         Network key: 00000000000000000000000000000000
    2017-07-28 14:17:02,513 - INFO - COM96: Adding device key to subnet 0
    2017-07-28 14:17:02,514 - INFO - COM96: Adding publication address(es)
    2017-07-28 14:17:02,517 - INFO - COM96: DevkeyAdd: {'devkey_handle': 9}
    2017-07-28 14:17:02,519 - INFO - COM96: AddrPublicationAdd: {'address_handle': 14}
    2017-07-28 14:17:02,522 - INFO - COM98: Provisioning complete
    2017-07-28 14:17:02,523 - INFO - COM98:         Address(es): 0x11-0x11
    2017-07-28 14:17:02,524 - INFO - COM98:         Device key: 40FD35B4223291ECF757471D7C606259
    2017-07-28 14:17:02,525 - INFO - COM98:         Network key: 00000000000000000000000000000000
    2017-07-28 14:17:02,525 - INFO - COM98: Adding network key (subnet)
    2017-07-28 14:17:02,525 - INFO - COM98: Adding device key to subnet 0
    2017-07-28 14:17:02,526 - INFO - COM98: Setting the local unicast address range
    2017-07-28 14:17:02,530 - INFO - COM98: SubnetAdd: {'subnet_handle': 0}
    2017-07-28 14:17:02,532 - INFO - COM98: DevkeyAdd: {'devkey_handle': 8}
    2017-07-28 14:17:02,532 - INFO - COM98: Success
    2017-07-28 14:17:02,542 - INFO - COM98: Provisioning link closed
    2017-07-28 14:17:02,621 - INFO - COM96: Provisioning link closed

Notice how the address range is changed for the second device. The `Provisioner` class keeps track of
the address space and assigns addresses incrementally based on the number of elements for each unprovisioned node.

The new device key is added to both the provisioner and provisionee. The addresses are added as local unicast addresses
for the provisionee and publication addresses for the provisioner. The network key is added only to the provisionee.

Now that we have provisioned two devices into our network, let us send some messages. Notice the _handles_ that are received
when adding addresses and keys. They are references used by the @ref DEVICE_STATE_MANAGER and in the message sending
API. From the documentation of the `PacketSend` command, we see that we need the `appkey_handle` and the `dst_addr_handle`:

    In [6]: cmd.PacketSend?
    Init signature: cmd.PacketSend(appkey_handle, src_addr, dst_addr_handle, ttl, reliable, data)
    Docstring:
    Send a mesh packet.

    Parameters
    ----------
        appkey_handle : uint16_t
            Appkey or devkey handle to use for packet sending. Subnetwork will be picked
            automatically.
        src_addr : uint16_t
            Raw unicast address to use as source address. Must be in the range of local
            unicast addresses.
        dst_addr_handle : uint16_t
            Handle of destination address to use in packet.
        ttl : uint8_t
            Time To Live value to use in packet.
        reliable : uint8_t
            Whether or not to make the transmission reliable.
        data : uint8_t[89]
            Payload of the packet.
    File:           <nrf5_sdk_for_bluetooth_mesh>\scripts\interactive_pyaci\aci\aci_cmd.py
    Type:           type

In our case, the `appkey_handle` is really a device key and has the value `8`. The `dst_addr_handle` is `15`.
We send the packet with the following command:

    In [10]: d[0].send(PacketSend(8, 0x0001, 15, 1, 0, "Hello world"))
    2017-07-28 14:27:30,215 - INFO - COM96: Success

We receive the message more or less instantaneous as a `MeshMessagereceivedunicast` event on the other device:

    2017-07-28 14:27:30,247 - INFO - COM97: {event: MeshMessageReceivedUnicast, data: {'src': 1,             \
        'adv_addr': bytearray(b'\xf4\x92\x94)B\xe2'), 'adv_addr_type': 1, 'data': bytearray(b'Hello world'), \
        'appkey_handle': 8, 'ttl': 1, 'dst': 16, 'rssi': 36, 'subnet_handle': 0, 'actual_length': 11}}
