# Configuring and performing DFU over Mesh

This guide offers step-by-step instructions on how to prepare and program the DFU example
application, create a DFU file that contains example firmware, and transfer it. Use this guide
to set up the proprietary mesh DFU to transfer any firmware to any device on the Bluetooth mesh network.
The firmware to be transfered can be a bootloader, a new SoftDevice, or a new application.

Before configuring DFU over Bluetooth mesh, make sure that you are familiar
with the [proprietary mesh DFU protocol](@ref md_doc_user_guide_modules_dfu_protocol)
documentation and that you have [integrated DFU process into your application](@ref md_doc_user_guide_modules_dfu_integrating_into_app).

The [DFU example](@ref dfu_example) demonstrates the [background mode DFU]
(@ref dfu-protocol-transfer-modes). For simplicity, various steps are provided for the
application firmware upgrade and are using the default nRF Util options.
Additional options and some debugging methods are documented in the last two sections of this guide.


**Table of contents**
- [Requirements](@ref dfu_configuration_requirements)
- [Optional preparation steps](@ref dfu_configuration_optional)
    - [Generating a signing key file with nRF Util](@ref dfu_configuration_signing_key)
    - [Adding the public key from nRF Util to device page](@ref dfu_configuration_public_key)
- [Preparing for DFU](@ref dfu_configuration)
    - [Step 1: Generate a DFU archive with nRF Util](@ref dfu_configuration_dfu_archive)
    - [Step 2: Generate a HEX version of device page](@ref dfu_configuration_device_page)
    - [Step 3: Erase chip memory and flash SoftDevice on all devices](@ref dfu_configuration_erase)
    - [Step 4: Flash the serial bootloader on all devices](@ref dfu_configuration_flash_bl)
    - [Step 5: Flash the first application on all devices](@ref dfu_configuration_flash_app)
    - [Step 6: Flash the device page on all devices and reset the device](@ref dfu_configuration_flash_device_page)
- [Transferring the DFU archive over serial with nRF Util](@ref dfu_configuration_transfer)
- [Additional nRF Util options for DFU procedure](@ref additional-dfu-options)
    - [Preparing DFU package for bootloader](@ref additional-dfu-options-bootloader)
    - [Preparing DFU package for SoftDevice](@ref additional-dfu-options-softdevice)
- [Debugging](@ref dfu-debugging)
    - [Interval selection for DFU transfer](@ref dfu-debugging-interval)
    - [Verbose nRF Util output during transfer](@ref dfu-debugging-verbose)
    - [Verifying bootloader](@ref dfu-debugging-bl-verify)



---

## Requirements @anchor dfu_configuration_requirements

To perform DFU transfers over Bluetooth mesh:
- Use a customized version of the nRF Util tool. This tool sends
the DFU packets to one device over serial interface. This device then stores these
packets, and also forwards them to the other devices withing the radio range.
    - The @link_ic_nrfutil is available at @link_nrfutil_github. The tool is open source.
    @note
    The master branch of the `pc-nrfutil` repository does not contain the additional
    code that is needed to handle the proprietary mesh DFU. To use the tool with the proprietary mesh DFU, use the `mesh_dfu`
    branch mentioned above. See the tool's documentation for more information about installation
    and prerequisites.
- Prepare at least 2 development kits or devices, where one is to be
interfaced over the serial port, while the other receives the DFU from the first device over
Bluetooth mesh.
    - To specify which device to use in which context, add the `-s <serial-number>` option for each
    call to the `nrfjprog` command, where `<serial-number>` is the SEGGER ID of your device. This will
    make `nrfjprog` execute its operations on the specified device only.
- Make sure to use the correct precompiled bootloader for your chip variant (nRF51 or nRF52, xxAA,
xxAB, xxAC). These variants have different flash and RAM sizes, as specified in the Product
Specification for @link_ic_nRF51PS and @link_ic_nRF52832PS.
- Build the Bluetooth mesh stack for your device by following the steps in
[Building the Bluetooth mesh stack and examples](@ref md_doc_getting_started_how_to_build).
- Ensure that `intelhex` package is installed for your Python installation.
- Use an application that supports DFU, such as the DFU example included in the nRF5 SDK for Mesh.
- Create a device page using the [Device Page Generator Tool](@ref dfu-device-page-generator).
- Ensure you have [selected a bank address](@ref integrating_dfu_update_request_fmw_bank_address) when
  you [integrated DFU process into your application](@ref md_doc_user_guide_modules_dfu_integrating_into_app).


---

## Generating and adding public key @anchor dfu_configuration_optional

Before you start properly preparing your devices for the proprietary mesh DFU, you can decide whether you want
to [generate a signing file with nRF Util](@ref dfu_configuration_signing_key)
and [add the public key from nRF Util to your device page](@ref dfu_configuration_public_key),
for an additional verification feature.

@note These are optional preparation steps. You can skip them if you do not need to add public key
and continue to [Preparing for DFU](@ref dfu_configuration).

### Generating a signing key file with nRF Util @anchor dfu_configuration_signing_key

DFU images can be signed to ensure they stem from a trusted source. If you want to use this
signature verification functionality, you need a signing key.

Use the nRF Util tool to generate a signing key. Run the following command:

```
mesh-sdk$ nrfutil keys --gen-key private_key.txt
```

This command creates a `private_key.txt` file in your current directory.

This key must only be shared with trusted sources. If you lose it, you also lose the authorization
to do DFU updates to your devices in the future. The only way to recover from the loss
of the private key is to reflash the device manually.

### Adding the public key from nRF Util to device page @anchor dfu_configuration_public_key

Now that you have a private key, you can generate the public key for it,
which you can safely share with everyone.
You must then add this public key to the bootloader configurator file, which is used to create
the [device page](@ref dfu-protocol-device-page). For details about the device page generator script,
see @ref md_tools_dfu_README.

Complete the following steps:
-# Run the following command:
```
mesh-sdk$ nrfutil keys --show-vk hex private_key.txt
```
    The result will be an output similar to the following example lines:
```
Verification key Qx: ed09a58df6db5cd15b8637304f31d31f4042492ed7c7e4839fbe903f260a2ba1
Verification key Qy: a855e92b72885825481ad56282bcf549ad7455ec46f000f0f62d97eeec883ba6
```
    These two HEX strings make up your public key.
@anchor creating-bootloader-config-json
-# In the `tools/dfu` folder, edit the `bootloader_config_default.json` file to prepare the device page:
        -# Create a new property named `"public_key"`.
        -# Assign the concatenated values of `Qx` and `Qy` strings to this key. For example:
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

Optionally, you can also change the company ID entry in the device page.
The company ID works as a namespace for application IDs in the proprietary mesh DFU.
In this way, any company with an assigned company ID can use any application ID
for its products, without risking an application ID conflict.

In the example, the company ID is set to `89`, which is the decimal version of Nordic Semiconductor's @link_companyID.
If your company has an assigned ID, you can use that ID number. If you do not represent a company
with an assigned ID, use a random 32-bit number higher than 65535.


---

## Preparing for DFU @anchor dfu_configuration

Complete the following steps to prepare your devices for the background mode of the proprietary mesh DFU:
-# [Generate a DFU archive with nRF Util](@ref dfu_configuration_dfu_archive)
-# [Generate a HEX version of device page with the tool in `tools/dfu`](@ref dfu_configuration_device_page)
-# [Erase chip memory and flash SoftDevice on all devices](@ref dfu_configuration_erase)
-# [Flash the serial bootloader on all devices](@ref dfu_configuration_flash_bl)
-# [Flash the first application on all devices](@ref dfu_configuration_flash_app)
-# [Flash the device page on all devices and reset the device](@ref dfu_configuration_flash_device_page)


### Step 1: Generate a DFU file with nRF Util @anchor dfu_configuration_dfu_archive

To do a DFU transfer, you must create a DFU archive with nRF Util, giving arguments that match the device page.
The DFU archive is a zip file that contains the application binary along with some metadata.

You need the HEX file of an example application that causes a LED to blink on the boards.
The file is generated when you [build the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build)
and is located in the `bin/blinky` folder. Use the HEX file that corresponds to the chip
and the SoftDevice you are using.

To generate the DFU archive, run the `nrfutil` command that corresponds to your SoftDevice.
For example:
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
- For nRF52 (nrf52832):
```
  mesh-sdk$ nrfutil dfu genpkg --application bin/blinky/blinky_nrf52832_xxAA_s132_7.2.0.hex \
      --company-id 0x00000059 \
      --application-id 1 \
      --application-version 2 \
      --key-file private_key.txt \
      --sd-req 0x00CB \
      --mesh dfu_test.zip
```

This command generates a DFU archive called `dfu_test.zip` in the current directory. You can call
`nrfutil dfu genpkg --help` to get a list of possible command line arguments and their meaning.

@note
- For ease of demonstration, the sample `bin/blinky` application does not include
the support for DFU over Bluetooth mesh. Once you upgrade your existing application with `blinky`,
the device cannot be upgraded further using DFU over Bluetooth mesh.
- If you completed [optional configuration steps](@ref dfu_configuration_optional), the `--company-id`
and the `--application-id` values must match the values used for generating the device page.
The `--application-version` must be higher than the version number that you used
for the previous firmware image.
- Some of the command line options do not apply to the proprietary mesh DFU, because the tool also supports the
regular Nordic Semiconductor DFU transfer.
- The example commands use the Nordic Semiconductor company ID, so make sure you use your own
instead.
- The application version is set to `2`. A device only accepts application
transfers that match its current company and application IDs and have a higher version number.
- For bootloader or SoftDevice firmware upgrade, DFU archive needs to be created differently. See
[Additional nRF Util options for DFU procedure](@ref additional-dfu-options).

### Step 2: Generate a HEX version of device page @anchor dfu_configuration_device_page

To generate a HEX version of your device page, use the `device_page_generator.py` script located
in the device page file folder: `tools/dfu`.
For details about the device page generator script, see @ref md_tools_dfu_README.

You have to specify the device series (`-d` option), and the SoftDevice version (`-sd` option).
All device pages contain a `SD_VERSION` entry, which must match the `--sd-req` value passed to `nrfutil` when
generating the DFU archive in the previous step. Failing to match the SoftDevice version requirement
parameters will make the device reject the transfer, as its own firmware ID will not match the one
in the transfer.

To generate a device page hex file for an nRF52 Series device (for example, nRF52832) using
s132 SoftDevice version 7.2.0, run the following command from inside the `tools/dfu` folder:
```
dfu$ python device_page_generator.py -d nrf52832_xxAA -sd "s132_7.2.0"
```

This creates a device page HEX file in the `tools\dfu\bin` folder. This file
will be used for [flashing the first application on all devices](@ref dfu_configuration_flash_app).

### Step 3: Erase chip memory and flash SoftDevice on all devices @anchor dfu_configuration_erase

@note
Steps 3 to 6 must be executed in the provided order.

Use nrfjprog (available on @link_nordicsemi) to erase all data on your device (including UICR)
and flash the SoftDevice.

SoftDevices for the nRF51 and nRF52 chips are located in the `bin/softdevice` folder.

Run the following nrfjprog command with the correct name of the HEX file:
```
mesh-sdk$ nrfjprog --program bin/softdevice/<SoftDevice HEX file> --chiperase
```

For example, to erase data and flash the S132 SoftDevice v7.2.0, run the following command:
```
mesh-sdk$ nrfjprog --program bin/softdevice/s132_nrf52_7.2.0_softdevice.hex --chiperase
```

### Step 4: Flash the serial bootloader on all devices @anchor dfu_configuration_flash_bl

For this step, use the precompiled bootloader with the serial support.
You can find the precompiled versions of the bootloader under `bin/`. The bootloader version must
match your chip version, as per table.


| Chip version   | Bootloader                            |
|----------------|---------------------------------------|
| nRF51422_xxAC  | `mesh_bootloader_serial_<compiler>_nrf51422_xxAC.hex` |
| nRF52832_xxAA  | `mesh_bootloader_serial_<compiler>_nrf52832_xxAA.hex` |
| nRF52833_xxAA  | `mesh_bootloader_serial_<compiler>_nrf52833_xxAA.hex` |
| nRF52840_xxAA  | `mesh_bootloader_serial_<compiler>_nrf52840_xxAA.hex` |

Flash the precompiled bootloader with the following nrfjprog command:
```
mesh-sdk$ nrfjprog --program bin/bootloader/<compiler>/<bootloader serial HEX file>
```

For example, you can run the following command to flash a bootloader compiled with GCC ARM compiler
on an nRF52832_xxAA device:
```
mesh-sdk$ nrfjprog --program bin/bootloader/gccarmemb/mesh_bootloader_serial_gccarmemb_nrf52832_xxAA.hex
```


### Step 5: Flash the first application on all devices @anchor dfu_configuration_flash_app

@note
This step assumes that you have built the Bluetooth mesh examples with CMake as described in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build). If you have built them with
SEGGER Embedded Studio, move to the next step.

To be able to do Device Firmware Updates, you must flash an application that
supports DFU. You can use the DFU example application in `examples/dfu/`.

You must flash the DFU example application from your build folder, using the HEX file that matches
your chip version and SoftDevice. For example, `build/examples/dfu/dfu_nrf52832_xxAA_s132_7.2.0.hex`
if your device is an `nRF52832_xxAA` with the s132 SoftDevice v7.2.0.

Run the following nrfjprog command:
```
mesh-sdk$ nrfjprog --program build/examples/dfu/dfu_nrf52832_xxAA_s132_7.2.0.hex
```

### Step 6: Flash the device page on all devices and reset the device @anchor dfu_configuration_flash_device_page

To flash the [device page HEX file](@ref dfu_configuration_device_page) to the devices,
run the following nrfjprog command:
```
mesh-sdk$ nrfjprog --program tools/dfu/bin/device_page_nrf52832_xxAA_s132_7.2.0.hex
```

Then, reset the device to start the application:
```
nrfjprog --reset
```

After the reset, observe that for every development kit that you programmed, all LEDs are OFF.
At this point, you have everything ready for performing the DFU over Bluetooth mesh.

---

## Transferring the DFU archive over serial with nRF Util @anchor dfu_configuration_transfer

Before you start transfering data:
- Close all running instances of nRFgo Studio before you continue. If running, nRFgo
Studio can cause problems with the reset procedure for the nRF51.
- Figure out to which COM port your serial device is connected:
    - _Windows_: Open Windows Device Manager and look under "Ports (COM & LPT)" for the number
    of the port. The serial ports are called COMxx, where xx is an integer.
    - _Linux_: Use the `dmesg` command after you have plugged in a device to see
    which serial port file has been assigned to the device. The serial ports for J-Link devices
    are called ttyACMx, where x is an integer, and are located in the `/dev` directory.

To start the DFU transfer, run the following command:
```
nrfutil dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh
```
A progress bar appears. The transfer takes a couple of minutes.

When finished, the bootloader switches to the application and one of the LEDs starts blinking
on each kit.
Note that you cannot do the DFU twice with the same DFU archive, because the application version in
the device page on your device is incremented to the latest version. Therefore, the bootloader will
reject any attempt to transfer the same firmware again.

If your newly transferred application also includes the DFU support, you can update it again in the
future when needed. To carry out a subsequent DFU transfer, run the preparation steps and
the transfer command again, but with an increased version number.
For example, `--application-version 3`. Also, use the new zip file.

---
## Additional nRF Util options for DFU procedure @anchor additional-dfu-options

So far, this guide described how to configure and carry out the application firmware upgrade.
The procedure for carrying out bootloader and SoftDevice firmware upgrade is quite similar,
except for the generation of a DFU archive.

### Preparing DFU package for bootloader @anchor additional-dfu-options-bootloader

If you want to upgrade the bootloader on the device, during [Step 1]
(@ref dfu_configuration_dfu_archive) you need to prepare the DFU archive using the new bootloader
hex file and version number. To do this, use the following command line options with the `nrfutil` commands:
  - `--bootloader` (instead of the `--application` option) - This option provides the bootloader hex file.
  - `--bootloader-id` - This option specifies the bootloader ID and version.
    - The `--bootloader-id` is a `uint16` number that has the bootloader ID as the least significant byte
    and the bootloader version as the most significant byte.
    - The bootloader ID must be same as the `bootloader_id` that is specified in
    the `bootloader_config_default.json` file that was used
    to [create the device page](@ref creating-bootloader-config-json) on the device undergoing upgrade.

For example, to create a DFU archive to upgrade the bootloader from version `1`
(see `bootloader_config_default.json` [snippet]
(@ref dfu_configuration_signing_key)) to version `2`, the command would be the following:
```
  nrfutil dfu genpkg --bootloader ./mesh/bootloader/mesh_bootloader_serial_gccarmemb_nrf52832_xxAA.hex \
        --company-id 0x00000059 \
        --bootloader-id 0x0201 \
        --key-file private_key.txt \
        --sd-req 0x00CB \
        --mesh dfu_test.zip
```

### Preparing DFU package for SoftDevice @anchor additional-dfu-options-softdevice

If you want to upgrade the SoftDevice on the device, during [Step 1]
(@ref dfu_configuration_dfu_archive) you need to prepare the DFU archive using the new SoftDevice
hex file. To do this, use the following command line options with the `nrfutil` commands:
  - `--softdevice` (instead of the `--application` option) - This option provides the SoftDevice hex file.

For example, to create DFU archive to upgrade SoftDevice to a new SoftDevice with the version
`0x0101`, the command would be the following:
```
  nrfutil dfu genpkg --softdevice ../bin/softdevice/s132_nrf52_7.2.0_softdevice.hex \
        --company-id 0x00000059 \
        --key-file private_key.txt \
        --sd-req 0x0101 \
        --mesh dfu_test.zip
```

---
## Debugging @anchor dfu-debugging

This section describes additional nRF Util options and scripts that can be used for debugging
the issues in the DFU procedure.

### Interval selection for DFU transfer @anchor dfu-debugging-interval

Each DFU packet is transmitted after a certain interval by the source device. The default value of
the interval is 500 ms (see [Transfer rate](@ref dfu-protocol-transfer-rate)).
You can change this interval by using `-i` option while starting the DFU.

For example, to start DFU with the interval of 100 ms, use the following command:
```
nrfutil --verbose dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh -i 100
```

The higher values of the interval are better for dense networks with a lot of noise,
even despite the drawback of increased overall transfer time.
This is because the longer interval gives more time for devices
to recover missing segments during the transfer.

@note
Do not reduce this interval below 60 ms to avoid termination of the DFU process
due to packet losses.

### Verbose nRF Util output during transfer @anchor dfu-debugging-verbose

To get a more verbose output during the DFU transfer using nRF Util, add `--verbose`
before the arguments:
```
nrfutil --verbose dfu serial -pkg dfu_test.zip -p <COM port> -b 115200 -fc --mesh.
```

### Verifying bootloader @anchor dfu-debugging-bl-verify

To verify that the bootloader is working correctly, run the [bootloader verification script](@ref dfu-bootloader-verify).
