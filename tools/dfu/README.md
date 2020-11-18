# DFU Utilities and Tools

The proprietary mesh DFU requires a number of scripts to work correctly. Several scripts are provided with the nRF5 SDK for Mesh.
You can find them at the following folder: `<InstallFolder>\tools\dfu`.

**Table of contents**
- [Device page generator (`device_page_generator.py`)](@ref dfu-device-page-generator)
    - [Requirements](@ref dfu-device-page-generator-requirements)
    - [Usage](@ref dfu-device-page-generator-usage)
    - [Input file](@ref dfu-device-page-generator-input)
- [Bootloader verification (`bootloader_verify.py`)](@ref dfu-bootloader-verify)
- [Device page reader (`read_devpage.py`)](@ref dfu-device-page-reader)


---

## Device Page Generator Tool (`device_page_generator.py`) @anchor dfu-device-page-generator

A device page is an in-flash storage area from which the bootloader can read device information
required for correct operation. For example, values like the application start address,
public key for verifying DFU transfers, and firmware IDs. This allows the bootloader to make
qualified decisions about whether it should accept incoming DFU transfers and where the data belongs.

This simple tool generates a device page for the combination of:
- platform, for example nRF52832_xxAA;
- SoftDevice, for example S132 v5.0.0;
- application, for example Blinky v1.0;
- bootloader, for example nRF5 SDK for Mesh bootloader v0.8.


### Requirements @anchor dfu-device-page-generator-requirements

The device page generator works in both Python 2.7, Python 3.2 and above. It depends on the
[Intel Hex package](https://pypi.python.org/pypi/IntelHex) by Alexander Belchenko,
available on `pypi`.

### Usage @anchor dfu-device-page-generator-usage

To generate a device page, run the following command:
```
python2 device_page_generator.py -c bootloader_config_default.json
```
Where `bootloader_config_default.json` can be replaced with the name of your [input file](@ref dfu-device-page-generator-input).
The device page file is created in the `bin/` directory.

Moreover, the Device Page Generator Tool comes with the following configuration options:
```
python2 device_page_generator.py -h
usage: device_page_generator.py [-h] [-d DEVICE] [-sd SOFTDEVICE]
                                [-c BOOTLOADER_CONFIG] [-o OUTPUT_FILE]
                                [--all]

Device Page Generator

optional arguments:
  -h, --help            show this help message and exit
  -d DEVICE, --device DEVICE
                        Select device
  -sd SOFTDEVICE, --softdevice SOFTDEVICE
                        Select SoftDevice
  -c BOOTLOADER_CONFIG, --bootloader-config BOOTLOADER_CONFIG
                        Bootloader configuration file
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        Output hex file
  --all                 Writes all known device page combinations to 'bin/'

```

### Input file @anchor dfu-device-page-generator-input

The input file is a JSON file that defines a specific set of bootloader and application configuration
parameters. An example file is provided in `booltoader_config_default.json`.
See the following table for the available parameters.

| Parameter             | Data type     | Required | Description
|-----------------------|---------------|----------|-------------|
| `company_id`          | `uint32_t`    | Yes      | Company ID associated with the application on this device.                     |
| `app_id`              | `uint16_t`    | Yes      | Application ID associated with the application on this device.                 |
| `app_version`         | `uint32_t`    | Yes      | Currently flashed version of the application.                                  |
| `bl_id`               | `uint8_t`     | Yes      | ID of the bootloader on this device.                                           |
| `bl_version`          | `uint8_t`     | Yes      | Version of the bootloader on this device.                                      |
| `public_key`          | `uint8_t[64]` | No       | Full version of the public signing key used to verify incoming DFU transfers.  |

Parameters for the selected platform and SoftDevice are calculated based on the information in
`softdevices.json` and `platforms.json` files in the following folder: `<InstallFolder>\tools\configuration`.


---

## Bootloader verification tool (`bootloader_verify.py`) @anchor dfu-bootloader-verify

The bootloader verification script is used to verify that the bootloader is flashed and initialized correctly.

The tool requires the @link_pyserial and adding `nrfjprog` to the `PATH` environment variable.

To run the script, use the following command with the serial number and the COM port of the serial device involved in the DFU transfer:
```
python bootloader_verify.py <serial number> <COM port>
```
For information about where to find the COM port number, see the [Configuring DFU over Bluetooth mesh](@ref dfu_configuration_transfer) page.

The script output looks like this:

```
Reading UICR..                  OK.
Reading Device page..           OK.
Resetting device..              OK.
Checking serial connection..    OK.

Bootloader verification OK
```

Run `nrfjprog --reset` to reset the board back to a well-known state of operation after running the
bootloader verification script.


---

## Device page reader (`read_devpage.py`) @anchor dfu-device-page-reader

The device page reader script allows you to read the device page from a device.
