# Configuring DFU over BLE using the LPN example

Device Firmware Upgrade (DFU) over Bluetooth LE (BLE) is the process that provides ability to update the application,
SoftDevice, and bootloader through BLE.

While this example is closely related to the @ref md_examples_lpn_README, it is not related
to the nRF5 SDK for Mesh DFU functionality described in @ref md_doc_user_guide_modules_dfu_configuring_performing.
The proprietary mesh DFU is not normally suitable for the LPN device, because it requires an always-on radio, which
increases power consumption significantly. For this reason, the LPN example uses the DFU solution
of the nRF5 SDK, which is GATT-based. In particular, it uses the following features of the nRF5 SDK
DFU over BLE functionality:
- @link_buttonless_secure_dfu_service that enables entering DFU mode from the application.
- @link_secure_bootloader to perform the DFU.

The DFU over BLE does not support the side-by-side mode. Once the DFU process starts, the device reboots
to the bootloader and the application will not work until the bootloader finishes.

For more information about these features, see @link_bootloader_and_dfu_modules.

@warning
In the Low Power node example, the DFU over BLE support is disabled by default.
You need to enable it to continue with this example. To do this, set @ref BLE_DFU_SUPPORT_ENABLED
to 1 in `examples/lpn/include/nrf_mesh_config_app.h`.

**Table of contents**
- [Hardware requirements](@ref examples_lpn_dfu_ble_requirements_hw)
- [Software requirements](@ref examples_lpn_dfu_ble_requirements_sw)
- [Setup](@ref examples_lpn_dfu_ble_setup)
- [Configuring DFU over BLE](@ref examples_lpn_dfu_ble_configure)
    - [Creating signature for the Low Power node example](@ref examples_lpn_dfu_ble_create_signature)
    - [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package)
    - [Building and programming the bootloader](@ref examples_lpn_dfu_ble_program_bootloader)
    - [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu)


---

## Hardware requirements @anchor examples_lpn_dfu_ble_requirements_hw

Performing DFU over BLE using PC requires one additional [compatible development kit](@ref md_doc_user_guide_mesh_compatibility), besides the development kits required by @ref md_examples_lpn_README.

---

## Software requirements @anchor examples_lpn_dfu_ble_requirements_sw

Install the following additional tools:
- @link_ic_nrfutil for generating the application signature and the firmware package. See @link_nrfutil_installing
for details.
- Depending on whether you want to perform DFU using mobile or PC:
    - @link_nRFConnectMobile (@link_nrf_connect_mobile_ios or @link_nrf_connect_mobile_android) for performing DFU using a mobile phone.
    - @link_nRFConnectDesktop for performing DFU using a PC.


---

## Setup @anchor examples_lpn_dfu_ble_setup

The nRF5 SDK project files for the DFU over BLE can be found at: `<the path to nRF5 SDK instance>/examples/dfu`.

You can find the source code of the Secure Bootloader example at: `<path to nRF5 SDK instance>/examples/dfu/secure_bootloader`.

The LPN device uses [PCA10040 Development Kit](@ref examples_lpn_requirements_hw).
 You can find the Secure Bootloader project files for this board in the `.../pca10040_s132_ble` folder.


---


## Configuring DFU over BLE @anchor examples_lpn_dfu_ble_configure

The following steps show how to use the Buttonless DFU Service to upgrade the device
containing only the bootloader and the SoftDevice (and no user application) with the Low
Power example, and then how to further upgrade this example with the new version of its hex file:
1. [Creating signature for the Low Power node example](@ref examples_lpn_dfu_ble_create_signature)
2. [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package)
3. [Building and programming the bootloader](@ref examples_lpn_dfu_ble_program_bootloader)
4. [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu)

@note
If you get the device configuration broken after
performing the firmware update, increase the value of the @link_app_data_area_size parameter.
The example code keeps the Bluetooth mesh configuration unchanged when the device enters the DFU mode.
So if the device was provisioned before the firmware update, it stays provisioned
after the firmware update is completed. This is achieved by configuring the @link_app_data_area_size
parameter that allows to reserve a flash area used by the application. As a consequence, this flash area won't
be used by the bootloader when the DFU is performed.

### Creating signature for the Low Power node example @anchor examples_lpn_dfu_ble_create_signature

To create the keys required for the DFU process:
1. Create a private key:
```
nrfutil keys generate lpn_private_key.pem
```
2. Create a public key in code format and store it in a file named `dfu_public_key.c`:
```
nrfutil keys display --key pk --format code lpn_private_key.pem --out_file dfu_public_key.c
```
3. Replace `dfu_public_key.c` file in the `<the path to nRF5 SDK instance>/examples/dfu` folder with the new one.

@note See @link_working_with_keys and @link_bootloader_signature_verification for more information about signatures.

### Generating a firmware package with the Low Power node example @anchor examples_lpn_dfu_ble_generate_dfu_package

To generate a firmware package:
1. Make sure the DFU over BLE support is enabled (@ref BLE_DFU_SUPPORT_ENABLED is set to 1
in examples/lpn/include/nrf_mesh_config_app.h).
2. Build the Low Power node example. To build the example, follow the instructions in
[Building the Bluetooth mesh stack](@ref md_doc_getting_started_how_to_build).
3. Generate a firmware package with the Low Power node example by using the
Low Power node hex file and the private key generated when building the example:
```
nrfutil pkg generate --application <path-to-lpn-example-hex-file> --application-version <application-version> --hw-version 52 --sd-req 0x0101 --key-file lpn_private_key.pem lpn_dfu_package.zip
```
    In this command:
        - Replace `<path-to-lpn-example-hex-file>` with the path to the LPN example HEX file and the file name.
        - Replace `<application-version>` with any positive number.
            - After the first time upgrade, make sure that each next update has the application version number greater than the current.
        - `--sd-req` can be obtained with the following command:
```
nrfutil pkg generate --help
```

@note See @link_nrfutil_generating_dfu_packages for more information.

### Building and programming the bootloader @anchor examples_lpn_dfu_ble_program_bootloader

To perform DFU over BLE update for the Low Power node example, you must build and program the @link_secure_bootloader.

**Building**<br>
To build the bootloader:
1. Install @link_micro_ecc for nRF5 SDK:
    1. Complete steps 2 and 3 from @link_micro_ecc_install.
    2. Compile micro-ecc:
        - On Windows, run `build_all.bat`.
        - On Ubuntu/Linux and similar systems, run `build_all.sh`.
2. Compile the Secure Bootloader example by using one of the following options:
    - Compile using SEGGER Embedded Studio (SES): Use the SES project file located under
    `.../ses` and follow the [Building with SEGGER Embedded Studio](@ref how_to_build_segger)
    instruction.
    - Compile using `make`: Run `make` under `.../armgcc`.

**Programming**<br>
To program the bootloader:
1. You can use one of the following options:
    - Program using SES: Follow the [Running examples using SEGGER Embedded Studio](@ref how_to_run_examples_ses)
    instruction.
    - Program using `make`:

            $ make erase
            $ make flash_softdevice
            $ make flash

    @note See the @link_programming_bootloader page in the nRF5 SDK documentation for more information.
2. Observe LED 1 and LED 2 on the device. Both light up when the bootloader enters the DFU mode.

### Performing DFU over BLE @anchor examples_lpn_dfu_ble_perform_dfu

The Device Firmware Upgrade over BLE can be performed using either a mobile phone or PC.

**Mobile**<br>
To perform the DFU transfer over BLE using a mobile phone:
1. Copy the generated `lpn_dfu_package.zip` firmware package to your mobile phone.
2. Use @link_nRFConnectMobile to scan for a target device and connect to it:
    - If the DFU is performed for the first time, the device will show up as "DfuTarget".
    - If the Low Power node example application is already programmed, the device will show up as "nRF5x Mesh LPN Switch".
3. Press the DFU button to perform DFU.
4. Provide the zip archive of the application when prompted.

**PC**<br>
To perform the DFU transfer over BLE using PC:
1. Plug in the one additional PCA10040 Development Kit.
2. Add "Bluetooth Low Energy" app to @link_nRFConnectDesktop.
3. Select the development kit you plugged in.
4. Launch "Bluetooth Low Energy" app.
5. Scan for a target device and connect to it:
    - If the DFU is performed for the first time, the device will show up as "DfuTarget".
    - If the Low Power node example application is already programmed, the device will show up as "nRF5x Mesh LPN Switch".
6. Press the DFU button to perform DFU.
7. Provide the zip archive of the application when prompted.

Regardless of the chosen method, the device resets and runs the new application. It shows up as "nRF5x Mesh LPN Switch"
in both the mobile app and on the PC.

To use the DFU over BLE feature for the Low Power example after its configuration, you only need to use the following
procedures described above:
- [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package)
- [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu)