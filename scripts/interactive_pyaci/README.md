# Interactive PyACI script

The Interactive Python Application Controller Interface (PyACI) (`interactive_pyaci.py`) can be used to
interactively control devices running the mesh stack and the serial interface. The script opens
up one or more COM ports and enables an
<a href="http://ipython.readthedocs.io/en/stable/index.html" target="_blank">interactive Python command line</a>.

Follow the instructions in this README to get your environment up and running.
Afterwards, you can run through the provided examples and tutorials.

- The @subpage md_scripts_interactive_pyaci_doc_demo_loopback tutorial demonstrates simple
loopback communication and is a good place to start to ensure everything is working right.
- The @subpage md_scripts_interactive_pyaci_doc_demo_sending_packets shows you how
to make serial devices talk to each other over mesh.
- The @subpage md_scripts_interactive_pyaci_doc_demo_configuration
tutorial demonstrates how to use the serial interface to provision and configure a mesh network.

**Table of contents**
- [Prerequisites](@ref serial_prerequisites)
- [Using the interface](@ref serial_using_the_interface)
    - [Windows](@ref serial_using_the_interface_win)
    - [Ubuntu/Linux](@ref serial_using_the_interface_ubuntu)
    - [Other options](@ref serial_using_the_interface_other)
- [About the interface](@ref serial_about_interface)
    - [Commands and events](@ref serial_about_interface_commands)
    - [Getting help](@ref serial_about_interface_help)
- [Miscellaneous](@ref serial_miscellaneous)


---


## Prerequisites @anchor serial_prerequisites

The interactive console is written for <a href="https://www.python.org/downloads/" target="_blank">Python 3</a>.
To install the required packages, move to the `scripts/interactive_pyaci` directory
and install the requirements using `pip`:

    nrf5_sdk_for_mesh$ cd scripts/interactive_pyaci
    interactive_pyaci$ pip install -r requirements.txt


---


## Using the interface @anchor serial_using_the_interface

To start the serial interface, run the following command in the directory of the script:

    interactive_pyaci$ python interactive_pyaci.py -d <COM>

Where `<COM>` is the COM port of the device you're connecting to. You may specify to multiple COM ports separated
by a space. Note that COM port names are *case sensitive*.

### Windows @anchor serial_using_the_interface_win

On Windows, the COM port is on the form `COM12`. To identify the correct COM port, open up the "Device Manager"
(`Run -> devmgmt.msc`) and connect the device. Your device should appear under "Ports (COM & LPT)".


### Ubuntu/Linux @anchor serial_using_the_interface_ubuntu
On Ubuntu/Linux and similar systems, the COM port is usually on the form `/dev/tty*`, e.g., `/dev/ttyACM0`.
To identify the correct COM port, you can use `dmesg`. Connect the device and run:

    $ dmesg | tail

The output should look like this:

    [46406.952479] usb 1-2: new high-speed USB device number 8 using ehci-pci
    [46407.258960] usb 1-2: New USB device found, idVendor=1366, idProduct=0105
    [46407.258964] usb 1-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
    [46407.258966] usb 1-2: Product: J-Link
    [46407.258967] usb 1-2: Manufacturer: SEGGER
    [46407.258969] usb 1-2: SerialNumber: 000682576262
    [46407.273985] cdc_acm 1-2:1.0: ttyACM1: USB ACM device

As you can see from the output, the connected device `682576262` was assigned to `/dev/ttyACM1`. Alternatively,
you could also use this command:

    $ dmesg | grep -C 3 SEGGER

@note This command requires 'sudo' when accessing /dev/tty* devices.

### Other options @anchor serial_using_the_interface_other

Calling the script with the `-h` option will give you information about the other options.

    $ python interactive_pyaci.py -h
    usage: interactive_pyaci.py [-h] -d DEVICES [DEVICES ...] [-b BAUDRATE]
                                [--no-logfile] [-l LOG_LEVEL]

    nRF5 SDK for Mesh Interactive PyACI

    optional arguments:
      -h, --help            show this help message and exit
      -d DEVICES [DEVICES ...], --device DEVICES [DEVICES ...]
                            Device Communication port, e.g., COM216 or
                            /dev/ttyACM0. You may connect to multiple devices.
                            Separate devices by spaces, e.g., "-d COM123 COM234"
      -b BAUDRATE, --baudrate BAUDRATE
                            Baud rate. Default: 115200
      --no-logfile          Disables logging to file.
      -l LOG_LEVEL, --log-level LOG_LEVEL
                            Set default logging level: 1=Errors only, 2=Warnings,
                            3=Info, 4=Debug


---


## About the interface @anchor serial_about_interface

The interface consists of the following files:

    .
    ├── aci
    │   ├── aci_cmd.py                    # Auto generated command class definitions (serialization)
    │   ├── aci_config.py                 # Utility class for parsing firmware configuration file (`nrf_mesh_app_config.h`)
    │   ├── aci_evt.py                    # Auto generated event class definitions (de-serialization)
    │   ├── aci_uart.py                   # The UART serial driver
    │   └── aci_utils.py                  # Utility functions and class definitions
    │
    ├── database
    │   ├── example_database.json         # Example mesh database file
    │   └── example_database.json.backup  # Backup of the original database
    │
    ├── doc                               # Tutorials and documentation
    │   ├── demo_configuration.md
    │   ├── demo_loopback.md
    │   └── demo_sending_packets.md
    │
    ├── interactive_pyaci.py              # The interactive script itself
    │
    ├── mesh                              # Mesh helper modules
    │   ├── access.py                     # Stripped down access layer
    │   ├── database.py                   # Database storage module
    │   ├── provisioning.py               # Provisioning interface module
    │   └── types.py                      # Mesh type definitions
    │
    ├── models                            # Mesh models
    │   ├── config.py                     # Configuration client
    │   ├── common.py                     # Contains common defines, structures, and functions used by the Mesh Models
    │   ├── simple_on_off.py              # Simple On/Off client
    │   └── generic_on_off.py             # Generic On/Off client
    │
    ├── README.md                         # The README you're reading now
    └── requirements.tx                   # Python pip requirements file


@note
`aci/aci_cmd.py` and `aci/aci_evt.py` are auto-generated from the C header files of the serial
interface with the `tools/serial_doc` scripts. To re-generate the files, build the `serial_pyaci` target
(requires a CMake based setup).


### Commands and events @anchor serial_about_interface_commands

Commands available in the interface can be found in `aci/aci_cmd.py`. They are imported through the
`cmd` namespace, e.g., `cmd.Echo`. Similarly, available events are found in `aci/aci_evt.py` and
available in the `evt` namespace, e.g., `evt.MeshMessageReceivedSubscription`.

### Getting help @anchor serial_about_interface_help

To read the documentation for one of the commands, events, functions or anything else,
enter a `?` before or after the object and press `<enter>`. E.g., for the
`BeaconParamsSet` command, you would get the following:


    In [1]: cmd.BeaconParamsSet?
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
    File:           <nrf5_sdk_for_bluetooth_mesh>/scripts/interactive_pyaci/aci/aci_cmd.py
    Type:           type


The help prompt may be used for any python object or function.

The console also provides auto completion, i.e., typing `cmd.BeaconParamsS` and pressing `<tab>`
will complete the command packet object.

For more details about the commands, see the [serial commands documentation](@ref md_doc_libraries_serial_cmd).


---


## Miscellaneous @anchor serial_miscellaneous

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
