# Sending Echo packets to check communication

This tutorial demonstrates how to use PyACI and send a simple Echo command to the connected device.
The response of the Echo command will contain the same data as the data that was sent
when issuing the command. You can use this command every time you connect to the target board
with PyACI to verify that the serial transport layer is working as intended.

**Table of contents**
- [Requirements](@ref pyaci_demo_loopback_requirements)
- [Configuring and testing connection](@ref pyaci_demo_loopback_configuration_testing)


---

## Requirements @anchor pyaci_demo_loopback_requirements

You need at least one compatible development kit for this demonstration,
[flashed](@ref examples_how_to_run_examples) with the following example:
- @ref md_examples_serial_README

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.


---

## Configuring and testing connection @anchor pyaci_demo_loopback_configuration_testing

After building and programming the serial example, complete the following steps:
1. Connect to the device with the interactive console:
```
    $ python interactive_pyaci.py -d /dev/ttyACM0 --no-logfile

        To control your device use d[x], type d[x]. and hit tab to see the available methods.
        x is for the device index; devices will be indexed based on the order of com ports
        given with the option -d(the first device, d[0], can also be accessed using device).

    Python 3.5.2 (default, Nov 17 2016, 17:05:23)
    Type 'copyright', 'credits' or 'license' for more information
    IPython 6.1.0 -- An enhanced Interactive Python. Type '?' for help.
```
2. Send a test `Echo` command to see that the device is connected:
    -# Display information about the command:
```
    In [1]: cmd.Echo?
    Init signature: cmd.Echo(data)
    Docstring:
    A simple loopback test command, to verify that the serial transport layer is working as
    intended.
    
    Parameters
    ----------
        data : uint8_t[254]
            Data to echo back.
    File:           nrf5_sdk_for_mesh/scripts/interactive_pyaci/aci/aci_cmd.py
    Type:           type
```
    -# Test the command:
```
    In [2]: send(cmd.Echo("hello world"))
    2020-03-24 12:45:18,063 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'hello world'}}
```
    The `send` function is a helper function for sending commands to the first connected device. It accepts a
    `CommandPacket` object as a parameter. All of these are imported from `aci/aci_cmd.py` by the
    interactive script.
3. Send an Echo command to the first device by addressing the device through the list of devices `d`,
in this case `d[0]`:
```
    In [3]: d[0].send(cmd.Echo("hello world"))
    2020-03-24 12:46:36,147 - INFO - ttyACM0: {event: DeviceEchoRsp, data: {'data': 'hello world'}}
```


@note
You might experience the following timeout error:
```
    2017-08-02 10:13:45,427 - INFO - ttyACM0: cmd Echo, timeout waiting for event
```
In this case, use one of the following options:
- Ensure that you are connected to the correct board and verify that the example is
running. You can also try resetting or reconnecting the board, or both.
- Using a serial terminal emulator, try to send an Echo command raw:
```
    06 02 48 65 6c 6c 6f
```
You should receive:
```
    06 82 48 65 6c 6c 6f
```