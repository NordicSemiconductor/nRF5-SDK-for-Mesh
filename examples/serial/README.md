# Serial example

This example implements the serial interface for the mesh stack. It can be used
unaltered as connectivity firmware for mesh devices. You can also modify the example
to provide additional functionality.

The example shows how to initialize and use the serial interface.
To initialize the serial interface, only two API calls are required:
```C
    nrf_mesh_serial_init(NULL);
    nrf_mesh_serial_enable();
```

The @ref nrf_mesh_serial_init() API initializes the serial bearer interface, and the
@ref nrf_mesh_serial_enable() API sends the @ref SERIAL_OPCODE_EVT_DEVICE_STARTED event over the
serial interface to notify the external host that the device is ready to accept serial commands.

This example also demonstrates how to offload certain mesh related tasks (to the external host
using a serial interface) and improve the performance of the mesh stack.
The ECDH operations used in the provisioning process are a suitable candidate for such offloading.

The ECDH offloading allows the device to take advantage of the host's more powerful
processor to perform ECDH operations during provisioning. It is enabled by setting the corresponding
option value to 1, as shown below:
the [options API](@ref NRF_MESH_OPT):
```C
    nrf_mesh_opt_t value = {.len = 4, .opt.val = 1 };
    nrf_mesh_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &value);
```

When this offloading is enabled, the serial interface sends the @ref SERIAL_OPCODE_EVT_PROV_ECDH_REQUEST
event to the host processor with the public and private keys. The host processor then performs the ECDH
shared secret calculation and sends the calculated value back using the @ref SERIAL_OPCODE_CMD_PROV_ECDH_SECRET
serial command.


## Hardware requirements
- At least one PCA10028 or PCA10040 development board, or PCA10031 dongle


## Running the example

Refer to [How to run examples](@ref examples_how_to_run_examples) in [Examples README.md](@ref example-projects) for the commands required to download an example firmware using `nrfjprog`.

1. Erase the device flash of your development board, and program the SoftDevice.
2. Flash the serial example firmware on your board.
3. Find the COM port number for the attached board.
4. Open a command prompt or a suitable shell and launch python interactive aci script. Use a baud rate
of 115200 and previously identified COM port number. Note that the COM port identifier is case
sensitive.

```
mesh-sdk$ python scripts\interactive_pyaci\interactive_pyaci.py -d COM18 -b 115200
```

The python script will show the following messages and provide you with a prompt for sending
serial commands.

```
mesh-sdk$ python scripts\interactive_pyaci\interactive_pyaci.py  -d COM18 -b 115200

    To control your device, use d[x], where x is the device index.
    Devices are indexed based on the order of the COM ports specified by the -d option.
    The first device, d[0], can also be accessed using device.

    Type d[x]. and hit tab to see the available methods.

Activating auto-logging. Current session state plus future input saved.
Filename       : interactive_session_18-16-10-9.log
Mode           : backup
Output logging : False
Raw input log  : False
Timestamping   : False
State          : active
Python 3.5.1 (v3.5.1:37a07cee5969, Dec  6 2015, 01:38:48) [MSC v.1900 32 bit (Intel)]
Type 'copyright', 'credits' or 'license' for more information
IPython 6.2.0 -- An enhanced Interactive Python. Type '?' for help.

In [1]:
```

To explore various serial interface commands see [Using the interface](@ref serial_using_the_interface), and to see demonstration of ECDH offloading see [Provisioning](@ref serial_provisioning) in the [Interactive PyACI documentation](@ref md_scripts_interactive_pyaci_README).

For more information about the serial interface commands, events, and status codes, see the [Serial interface](@ref md_doc_libraries_serial) documentation.



