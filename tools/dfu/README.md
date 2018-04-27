# DFU Utilities and Tools

To work with the mesh DFU, the following scripts are provided:

- Device page generator tool, `device_page_generator.py`
- Bootloader verification tool, `bootloader_verify.py`
- Device page reader tool, `read_devpage.py`

The following sections describe each of the tools in more detail.

## Device Page Generator Tool (`device_page_generator.py`)

This simple tool will generate a device page for the combination of a platform, SoftDevice, application and bootloader.
E.g., nRF52832_xxAA, S132 v5.0.0, Blinky v1.0 and mesh bootloader v0.8.

A device page is an in-flash storage area from which the bootloader can read device information
required for correct operation. Values like the application start address, public key
for verifying DFU transfers and firmware IDs, allows the bootloader to make qualified decisions
about whether it should accept incoming DFU transfers and where the data belongs.

### Requirements

The device page generator works in both Python 2.7 and 3.2 and above. It depends on the
[Intel Hex package](https://pypi.python.org/pypi/IntelHex) by Alexander Belchenko,
available on `pypi`.

### Usage

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

### Input file

The input file is JSON file that defines a specific set of bootloader and application configuration
parameters. An example file is provided in `booltoader_config_default.json`.
The available parameters are given in the table below.

| Parameter             | Data type     | Required | Description
|-----------------------|---------------|----------|-------------|
| `company_id`          | `uint32_t`    | Yes      | Company ID associated with the application on this device.
| `app_id`              | `uint16_t`    | Yes      | Application ID associated with the application on this device.
| `app_version`         | `uint32_t`    | Yes      | Currently flashed version of the application.
| `bl_id`               | `uint8_t`     | Yes      | ID of the bootloader on this device.
| `bl_version`          | `uint8_t`     | Yes      | Version of the bootloader on this device.
| `public_key`          | `uint8_t[64]` | No       | Full-version of the public signing key used to verify incoming DFU transfers.

Parameters for the selected platform and SoftDevice are calculated based on the information in
`../configuration/softdevices.json` and `../configuration/platforms.json`.

## Bootloader Verification Tool (`bootloader_verify.py`)

The bootloader verification script is used to verify that the bootloader is flashed and initialized correctly.

## Device Page Reader (`read_devpage.py`)

This script allows you to read the device page from a device.
