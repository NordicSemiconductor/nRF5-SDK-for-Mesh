# Interactive PyACI script

The Interactive Python Application Controller Interface (PyACI) (`interactive_pyaci.py`) can be used to
interactively control devices running the mesh stack and the serial interface. The script opens
up one or more COM ports and enables the @link_python_interactive_command_line.

**Table of contents**
- [Prerequisites](@ref serial_prerequisites)
- [Installing the script](@ref serial_installing_script)
- [Starting the interface](@ref serial_using_the_interface)
    - [Windows COM port](@ref serial_using_the_interface_win)
    - [Ubuntu/Linux COM port](@ref serial_using_the_interface_ubuntu)
- [Available options](@ref serial_using_the_interface_options)
- [Interface file structure](@ref serial_file_structure)
- [Commands and events](@ref serial_about_interface_commands)
- [Getting help](@ref serial_about_interface_help)
- [Complex scripting](@ref serial_complex_scripting)

Follow the instructions on this page to get your environment up and running.
Afterwards, you can go through the provided examples and tutorials:
- @subpage md_scripts_interactive_pyaci_doc_demo_loopback demonstrates simple
loopback communication. You can use it to ensure everything is working right.
- @subpage md_scripts_interactive_pyaci_doc_demo_sending_packets shows how
to make serial devices talk to each other over mesh.
- @subpage md_scripts_interactive_pyaci_doc_demo_configuration
demonstrates how to use the serial interface to provision and configure a mesh network.


---


## Prerequisites @anchor serial_prerequisites

The interactive console is written for @link_python35_download,
which is required for the [standard toolchain installation](@ref md_doc_getting_started_how_to_toolchain).

---

## Installing the script @anchor serial_installing_script

To install packages required by the Interactive PyACI, go to the `scripts/interactive_pyaci` directory
and run the installation using `pip`:

    nrf5_sdk_for_mesh$ cd scripts/interactive_pyaci
    interactive_pyaci$ pip install -r requirements.txt


---


## Starting the interface @anchor serial_using_the_interface

To start the serial interface, run the following command in the directory of the script:

    interactive_pyaci$ python interactive_pyaci.py -d <COM>

In this command, `<COM>` is the COM port of the device you are connecting to.
You may specify  multiple COM ports separated by a space. COM port names are *case sensitive*.

The COM port variable is different depending on your operating system.

### Windows COM port @anchor serial_using_the_interface_win

On Windows, the COM port is labelled `COMnn`, where `nn` is the number of the port, for example `COM12`.

To identify the correct COM port, open up the "Device Manager" (`Run -> devmgmt.msc`)
and connect the device.
Your device will appear under "Ports (COM & LPT)".

### Ubuntu/Linux COM port @anchor serial_using_the_interface_ubuntu
On Ubuntu/Linux and similar systems, the COM port is usually labelled `/dev/tty*`, 
for example `/dev/ttyACM0`.

To identify the correct COM port, you can use `dmesg`.
Connect the device and run:

    $ dmesg | tail

The output has the following format:

    [46406.952479] usb 1-2: new high-speed USB device number 8 using ehci-pci
    [46407.258960] usb 1-2: New USB device found, idVendor=1366, idProduct=0105
    [46407.258964] usb 1-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
    [46407.258966] usb 1-2: Product: J-Link
    [46407.258967] usb 1-2: Manufacturer: SEGGER
    [46407.258969] usb 1-2: SerialNumber: 000682576262
    [46407.273985] cdc_acm 1-2:1.0: ttyACM1: USB ACM device

As you can see, the connected device `682576262` was assigned to `/dev/ttyACM1`.

Alternatively, you could also use the following command:

    $ dmesg | grep -C 3 SEGGER

@note This command requires 'sudo' when accessing /dev/tty* devices.


---

## Available options @anchor serial_using_the_interface_options

Call the script with the `-h` option to get information about the available options:

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


## Interface file structure @anchor serial_file_structure

The interface consists of the following files:

    .
    ├── aci
    │   ├── aci_cmd.py                    # Auto-generated command class definitions (serialization).
    │   ├── aci_config.py                 # Utility class for parsing firmware configuration file (`nrf_mesh_app_config.h`).
    │   ├── aci_evt.py                    # Auto-generated event class definitions (de-serialization).
    │   ├── aci_uart.py                   # UART serial driver.
    │   └── aci_utils.py                  # Utility functions and class definitions.
    │
    ├── database
    │   ├── example_database.json         # Example mesh database file.
    │   └── example_database.json.backup  # Backup of the original database.
    │
    ├── doc                               # Tutorials and documentation.
    │   ├── demo_configuration.md
    │   ├── demo_loopback.md
    │   └── demo_sending_packets.md
    │
    ├── interactive_pyaci.py              # Interactive script itself.
    │
    ├── mesh                              # Mesh helper modules.
    │   ├── access.py                     # Stripped down access layer.
    │   ├── database.py                   # Database storage module.
    │   ├── provisioning.py               # Provisioning interface module.
    │   └── types.py                      # Mesh type definitions.
    │
    ├── models                            # Mesh models.
    │   ├── config.py                     # Configuration client.
    │   ├── common.py                     # Common defines, structures, and functions used by the Mesh Models.
    │   ├── simple_on_off.py              # Simple On/Off client.
    │   └── generic_on_off.py             # Generic On/Off client.
    │
    ├── README.md                         # Contents of the page you are reading now.
    └── requirements.tx                   # Python pip requirements file.


@note
The `aci/aci_cmd.py` and `aci/aci_evt.py` files are auto-generated from the C header files of the serial
interface with the `tools/serial_doc` scripts. To re-generate the files, build the `serial_pyaci` target
(requires a CMake-based setup).


---

## Commands and events @anchor serial_about_interface_commands

Commands available in the interface can be found in `aci/aci_cmd.py`. They are imported through the
`cmd` namespace, for example `cmd.Echo`. Similarly, available events are found in `aci/aci_evt.py` and
available in the `evt` namespace, for example `evt.MeshMessageReceivedSubscription`.

---


## Complex scripting @anchor serial_complex_scripting

As with the normal Python shell, you can do more complex scripting.

For example, the following simple `for`-loop that sends 10 echo commands with one second delay:


    In [1]: import time
    In [2]: for i in range(0, 10): send(cmd.Echo("Hello: " + str(i))); time.sleep(1)
    2020-03-24 12:41:18,657 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 0'}}
    2020-03-24 12:41:19,658 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 1'}}
    2020-03-24 12:41:20,659 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 2'}}
    2020-03-24 12:41:21,663 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 3'}}
    2020-03-24 12:41:22,663 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 4'}}
    2020-03-24 12:41:23,667 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 5'}}
    2020-03-24 12:41:24,671 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 6'}}
    2020-03-24 12:41:25,676 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 7'}}
    2020-03-24 12:41:26,681 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 8'}}
    2020-03-24 12:41:27,684 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'Hello: 9'}}


---

## Getting help @anchor serial_about_interface_help

To read the documentation for one of the commands, events, functions or other options,
enter a `?` before or after the object and press `<enter>`. For example, for the
`BeaconParamsSet` command, you would get the following output:


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

The console also provides autocompletion. For example, typing `cmd.BeaconParamsS` and pressing `<tab>`
will complete the command packet object.

For more details about the commands, see the [serial commands documentation](@ref md_doc_user_guide_modules_serial_cmd).

