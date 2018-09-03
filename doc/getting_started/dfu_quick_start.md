# DFU quick start guide

A Device Firmware Update (DFU) is the process of updating the firmware on a mesh device. The
following guide offers step-by-step instructions on how to prepare and program the DFU example
application, create a DFU file that contains example firmware, and transfer it. This guide should
make it easy to use the mesh DFU to transfer any firmware to any device on the mesh network.

To perform DFU transfers over mesh, use a customized version of the `nrfutil` tool. This tool sends
the DFU packets to one device over serial interface. This device then stores these
packets, and also forwards them to the other devices withing the radio range.

The required steps are listed below, followed by a more detailed explanation for each step.

Before you start, note the following information:
- This guide assumes that you have at least 2 development kits or devices, where one is to be
interfaced over the serial port, while the other receives the DFU from the first device over the
mesh. To specify which device to use in which context, add the `-s <serial-number>` option for each
call to the `nrfjprog` command, where `<serial-number>` is the Segger ID of your device. This will
make `nrfjprog` execute its operations on the specified device only.
- The @link_ic_nrfutil <!--nrfutil: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.tools/dita/tools/nrfutil/nrfutil_intro.html-->
tool that is required to transfer the firmware image is available at
@link_nrfutil_github<!--https://github.com/NordicSemiconductor/pc-nrfutil/tree/mesh_dfu-->. The
tool is open source.

  > **Important:** The master branch of the pc-nrfutil repository does not contain the additional
  > code that is needed to handle a mesh DFU. To use the tool with mesh DFU, use the `mesh_dfu`
  > branch as mentioned above. See the tool's documentation for more information about installation
  > and prerequisites.
- Make sure to use the correct precompiled bootloader for your chip variant (nRF51/nRF52, xxAA,
xxAB, xxAC). These variants have different flash and RAM sizes, as specified in the Product
Specification for @link_ic_nRF51PS <!--nRF51: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf51/dita/nrf51/pdflinks/51822_ps.html-->
and @link_ic_nRF52832PS<!--nRF52832: http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/chips/nrf52832_ps.html-->.
- This guide also assumes that you have built the mesh stack for your device by following the steps
outlined in [Building the mesh stack](@ref md_doc_getting_started_how_to_build).
- Ensure that `intelhex` package is installed for your python installation.


## Steps

1. Optional: Generate a signing key file with nrfutil.
2. Optional: Paste the public key from nrfutil into your device page.
3. Generate a DFU archive with nrfutil, giving arguments that match the device page.
4. Generate a HEX version of your device page with the tool in `tools/dfu`.
5. Erase all chip memory (including the UICR) on all devices.
6. Flash the SoftDevice on all devices.
7. Flash the serial bootloader on all devices.
8. Flash the first application on all devices.
9. Flash the device page on all devices.
10. Transfer the DFU archive over serial with nrfutil.

### 1. Optional: Generate a signing key file with nrfutil

DFU images can be signed to ensure they stem from a trusted source. If you want to use this
signature verification functionality, you need a signing key. The nrfutil tool can be used to
generate a signing key:

```
mesh-sdk$ nrfutil keys --gen-key private_key.txt
```

This will create a text file in your current directory named `private_key.txt`. This key must only
be shared with trusted sources, and if it is lost, you would also lose authorization to do DFU
updates to your devices in the future. The only way to recover from the loss of the private key
is to reflash the device manually.

### 2. Optional: Add the public key from nrfutil to your device page

Now that you have a private key, you can generate the public key for it:

```
mesh-sdk$ nrfutil keys --show-vk hex private_key.txt
```

This will output something like this:

```
Verification key Qx: ed09a58df6db5cd15b8637304f31d31f4042492ed7c7e4839fbe903f260a2ba1
Verification key Qy: a855e92b72885825481ad56282bcf549ad7455ec46f000f0f62d97eeec883ba6
```

These two HEX strings make up your public key, which you can safely share with everyone. Note that
the keys above are example keys and will only work with a specific private key.

All dfu-enabled mesh devices require a device page that contains information about the device and
the firmware that is installed on the device. The device page is generated using the
`device_page_generator.py` script file available in the `tools/dfu` folder. This script file uses
`bootloader_config_default.json` file to prepare the device page. The public key obtained above must
be inserted in this JSON object.

To add the public key information to the bootloader configurator JSON, create a new property named `"public_key"` and
assign the concatenated values of the `Qx` and the `Qy` strings to this key. For example:

```
{
    "bootloader_config": {
        "bootloader_id": 1,
        "bootloader_version": 1,
        "company_id": 89,
        "application_id": 1,
        "application_version": 1,
        "public_key": "ed09a58df6db5cd15b8637304f31d31f4042492ed7c7e4839fbe903f260a2ba1a855e92b72885825481ad56282bcf549ad7455ec46f000f0f62d97eeec883ba6"
    }
}
```

This allows your device to verify that the person that has initiated the DFU
transfer has the private key associated with this public key.

You may also want to change the company ID entry. In the example, the company ID is set to
`89`, which is the decimal version of Nordic Semiconductor's @link_companyID
<!--Bluetooth SIG assigned Company ID: https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers-->\.

If your company has an assigned ID, you can use that ID number. If you do not represent a company
with an assigned ID, use a random 32-bit number higher than 65535.
The company ID works as a namespace for application IDs in the mesh DFU.
This way, any company with an assigned company ID may safely use any application ID
for their products, without risking an application ID conflict.

### 3. Generate a DFU file with nrfutil

To do a DFU transfer, you must create a DFU archive. The DFU archive is a zip file that contains
the application binary along with some metadata.

A HEX file of an example application that causes an LED to blink on the boards is located in the
`bin/blinky` folder. Use the HEX file that corresponds to the chip and the SoftDevice you are using.
Note that this file is generated when you [build the mesh stack](@ref md_doc_getting_started_how_to_build).

Use the `nrfutil` tool to generate the DFU archive matching your SoftDevice requirement, for example:

- For nRF51:

  ```
  mesh-sdk$ nrfutil dfu genpkg --application bin/blinky/blinky_nrf51422_xxAC_s130_2.0.1.hex \
      --company-id 0x00000059 \
      --application-id 1 \
      --application-version 2 \
      --key-file private_key.txt \
      --sd-req 0x0087 \
      --mesh dfu_test.zip
  ```
- For nRF52:

  ```
  mesh-sdk$ nrfutil dfu genpkg --application bin/blinky/blinky_nrf52832_xxAA_s132_6.0.0.hex \
      --company-id 0x00000059 \
      --application-id 1 \
      --application-version 2 \
      --key-file private_key.txt \
      --sd-req 0x009D \
      --mesh dfu_test.zip
  ```

Note that the `--company-id` and the `--application-id` values must match the values used for generating a
device page as described in *Step 2*. The `--application-version` must be higher than the version
number that you used for the previous firmware image.

This command generates a DFU archive called `dfu_test.zip` in the current directory. You can call
`nrfutil dfu genpkg --help` to get a list of possible command line arguments and their meaning.
Note that some of the options do not apply to the mesh DFU, because the tool also supports the
regular Nordic Semiconductor DFU transfer.

The example commands use the Nordic Semiconductor company ID, so make sure you use your own
instead. Also note that the application version is set to 2. A device will only accept application
transfers that match its current company and application IDs and have a higher version number.

### 4. Generate a HEX version of your device page

In the same folder as the example device page file (`tools/dfu`), there is a Python script called
`device_page_generator.py` that should be used to generate device pages (works for both Python 2.7
and Python 3, requires the `intelhex` package from PyPi).
See the README file in the `tools/dfu` folder if you want more detailed information about usage of
this script.

Use the `-d` option to specify the device series, and the `-sd` option to specify the SoftDevice version.
All device pages contain a `SD_VERSION` entry, which must match the `--sd-req` value passed to `nrfutil` when
generating the DFU archive in step 3. Failing to match the SoftDevice version requirement
parameters will make the device reject the transfer, as its own firmware ID won't match the one
in the transfer.

For example, run the following command from inside the `tools/dfu` folder to generate a device
page hex file for an NRF52 Series device using s132 SoftDevice version 6.0.0:

For nRF52:

  ```
  dfu$ python device_page_generator.py -d nrf52832_xxAA -sd "s132_6.0.0"
  ```

This creates a device page .hex file in the `tools\dfu\bin` folder. This file
will be used in step 9.

### 5. Erase all chip memory (including UICR) on all devices

Use nrfjprog (available on @link_nordicsemi<!--http://www.nordicsemi.com/-->) to erase all previous
data on your device (including UICR):

```
mesh-sdk$ nrfjprog --eraseall
```

### 6. Flash the SoftDevice on all devices

> **Important:** Steps 6-9 must be executed in order.

SoftDevices for nRF51 and nRF52 are located in the `bin/softdevice` folder.

```
mesh-sdk$ nrfjprog --program bin/softdevice/<SoftDevice HEX file>
```

For example, to flash S132 SoftDevice v6.0.0, run the following command:

```
mesh-sdk$ nrfjprog --program bin/softdevice/s132_nrf52_6.0.0_softdevice.hex --chiperase
```

### 7. Flash the serial bootloader on all devices

Flash the precompiled bootloader with serial support to your device using nrfjprog.
You can find precompiled versions of the bootloader under `bin/`. The bootloader version must
match your chip version, see the following table:


| Chip version   | Bootloader                            |
|----------------|---------------------------------------|
| nRF51422_xxAC  | `mesh_bootloader_serial_<compiler>_nrf51422_xxAC.hex` |
| nRF52832_xxAA  | `mesh_bootloader_serial_<compiler>_nrf52832_xxAA.hex` |
| nRF52840_xxAA  | `mesh_bootloader_serial_<compiler>_nrf52840_xxAA.hex` |

```
mesh-sdk$ nrfjprog --program bin/bootloader/<compiler>/<bootloader serial HEX file>
```

For example, you can run the following command to flash a bootloader compiled with GCC ARM compiler on an nRF52832_xxAA device:

```
mesh-sdk$ nrfjprog --program bin/bootloader/gccarmemb/mesh_bootloader_serial_gccarmemb_nrf52832_xxAA.hex
```


### 8. Flash the first application on all devices

> **Note:** This step assumes that you have built the mesh examples with CMake as described in
[Building the mesh stack](@ref md_doc_getting_started_how_to_build). If you have built them with
Segger Embedded Studio, you can flash the DFU example directly from the IDE, as described in the
build-guide, and move on to step 9.

To be able to do Device Firmware Updates, you must flash an application that
supports DFU. The DFU example application can be found in `examples/dfu/`.

From your build folder, flash the DFU example application HEX file matching your chip version and
SoftDevice to your device, for example `build/examples/dfu/dfu_nrf52832_xxAA_s132_6.0.0.hex`
if your device is an `nRF52832_xxAA` with s132 SoftDevice v6.0.0.

Flash the file with the following command:

```
mesh-sdk$ nrfjprog --program build/examples/dfu/dfu_nrf52832_xxAA_s132_6.0.0.hex
```

### 9. Flash the device page on all devices

Flash the device page HEX file that you generated in step 4 to the devices:

```
mesh-sdk$ nrfjprog --program tools/dfu/bin/device_page_nrf52832_xxAA_s132_6.0.0.hex
```

Then reset the device to start the application:
```
nrfjprog --reset
```

After reset, observe that for every development kit that you programmed, all user LEDs are OFF.
At this point you have everything ready for performing the DFU over the mesh.

### 10. Transfer the DFU archive over serial with nrfutil

> **Important:** Close all running instances of nRFgo Studio before you continue. If running, nRFgo
Studio might cause problems with the reset procedure for the nRF51.

Now we come to the interesting part: Doing the DFU! First, figure out to which COM port your serial
device is connected:

- On Windows, serial ports are called COMxx, where xx is an integer. To figure out which COM port
is used for a device, open Windows Device Manager and look under "Ports (COM & LPT)" for the number
of the port.
- On Linux, serial ports for JLink devices are called ttyACMx, where x is an integer, and are
located in the `/dev` directory. Use the `dmesg` command after you have plugged in a device to see
which serial port file has been assigned to the device.

To start the DFU, run the following command:

```
nrfutil dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh
```

A progress bar should pop up, and the transfer should take a couple of minutes.

> **NOTE:** To get more verbose output, you can add `--verbose` before any other arguments as follows:
> `nrfutil --verbose dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh`.

When finished, the bootloader should switch to the application and one user LED should start blinking on each kit.
Note that you cannot do the DFU twice with the same DFU archive, because the application version in
the device page on your device is incremented to the latest version. Therefore, the bootloader will
reject any attempt to transfer the same firmware again.

To try another DFU, re-run steps 3 and 10 with an increased version number, for example
`--application-version 3`, and use the new zip file to do the DFU again.

## Troubleshooting: Verifying your bootloader with the bootloader_verify.py script

To verify that the bootloader is working correctly, run the bootloader verification script located
in `tools/dfu`. Note that it requires the @link_pyserial <!--pyserial package: https://pypi.python.org/pypi/pyserial-->
and that `nrfjprog` is present in your `PATH`.

```
python bootloader_verify.py <serial number> <COM port>
```

The output should look like this:

```
Reading UICR..                  OK.
Reading Device page..           OK.
Resetting device..              OK.
Checking serial connection..    OK.

Bootloader verification OK
```

Run `nrfjprog --reset` to reset the board back to a well-known state of operation after running the
bootloader verification script.

