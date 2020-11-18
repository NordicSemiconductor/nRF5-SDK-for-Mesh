# Compatibility

This page contains information about the compatibility status of the nRF5 SDK for Mesh with Nordic's
development kits and SoftDevice versions.

If a Nordic kit is not mentioned on this page, it means that it is not supported.

For more information about Nordic kits, see @link_nrf_dk.

**Table of contents**
- [Full compatibility](@ref compatibility_list)
- [Limited or deprecated compatibility](@ref compatibility_limited)
    - [Limited compatibility with nRF52840 on PCA10059 dongle](@ref compatibility_nRF52840_dongle)
    - [Limited compatibility with nRF52810 and nRF52820](@ref compatibility_nRF52810_nRF52820)
    - [Deprecated compatibility with nRF51](@ref compatibility_nRF51)


---

## Full compatibility @anchor compatibility_list

The Bluetooth mesh stack is fully compatible with the configurations mentioned in the following table.

| Device                                         | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0/v6.1.0/v6.1.1/v7.0.1/v7.2.0<br>S113 v7.0.1/v7.2.0         |
| nRF52833_xxAA                                  | PCA10100  | S140 v7.0.1/v7.2.0<br>S113 v7.0.1/v7.2.0                              |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0/v6.1.0/v6.1.1/v7.0.1/v7.2.0<br>S113 v7.0.1/v7.2.0<br>S112 v7.2.0  |

All [examples](@ref md_examples_README) are compatible with these configurations.

---

## Limited or deprecated compatibility @anchor compatibility_limited

The Bluetooth mesh stack is not fully compatible with the configurations mentioned in the following table.

| Device                                         | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52840_xxAA ([limited compatibility](@ref compatibility_nRF52840_dongle))    | PCA10059  | S140 v6.0.0/v6.1.0/v6.1.1/v7.0.1/v7.2.0<br>S113 v7.0.1/v7.2.0  |
| nRF52820_xxAA ([limited compatibility](@ref compatibility_nRF52810_nRF52820))  | PCA10100e | S112 v7.2.0<br>S113 v7.2.0  |
| nRF52810_xxAA ([limited compatibility](@ref compatibility_nRF52810_nRF52820))  | PCA10040e | S112 v6.0.0/v6.1.0/v6.1.1/v7.0.1/v7.2.0  |
| nRF51422_xxAC ([deprecated compatibility](@ref compatibility_nRF51))           | PCA10028  | S130 v2.0.1                |

See the following sections for detailed information.

### Limited compatibility with nRF52840 on PCA10059 dongle @anchor compatibility_nRF52840_dongle

The Bluetooth mesh stack is fully compatible with the nRF52840 platform,
but only few examples have support for running on the nRF52840 dongle, and this support is limited.
Only [the server light switch example](@ref md_examples_light_switch_server_README) has been tested with
the dongle.

The nRF52840 dongle is shipped with a USB bootloader that can be used
to flash the desired application and SoftDevice using the Programmer application available
in @link_nRFConnectDesktop.

The following limitations apply to all supported examples:
- Only a single button is available on the dongle. Therefore, example functionalities are limited,
  because only the first button assignment will work. See example documentation pages for details.
- RTT interface is not available. Therefore, neither RTT input nor RTT logging is supported for any
  of the examples. For this reason, the examples that completely depend on RTT input
  for user interaction cannot be used on the dongle.
- The dongle has one green LED and three color LEDs. All four LEDs can be used in examples.
  However, the three color LEDs are combined into a single physical RGB LED.
  For this reason, various blinking patterns used for user signalling (such as device identification)
  in the example applications will look different on the dongle.
- As of v1.4.3, the dongle Programmer application does not support erasing
  of the empty flash areas. Therefore, once the device is provisioned, it cannot be unprovisioned
  by flashing the same example again. To unprovision the device, use one of the following methods:
    - Trigger a node reset using the nRF Mesh mobile application or any other configuration client.
    - Modify the example application to trigger the node reset after pressing **SW1** (**Button 1**)
      on the dongle.
    - Flash another example with different device composition and the Bluetooth mesh stack will automatically
      unprovision the device on the next reboot.

The following examples cannot be run on the nRF52840 dongle, and therefore they are not supported:
- @ref md_examples_dfu_README (because of the presence of the USB bootloader and due to the lack of built-in serial port emulator)
- @ref md_examples_serial_README (because of the lack of built-in Serial port emulator)
- @ref md_examples_sdk_coexist_README

Other examples can be potentially run on the nRF52840 dongle (only the server example),
but with limited functionality.

### Limited compatibility with nRF52810 and nRF52820 @anchor compatibility_nRF52810_nRF52820

The Bluetooth mesh stack is compatible with the nRF52810 and nRF52820 platforms, but these specific platforms are not supported
by the current version of the nRF5 SDK for Mesh.

These platforms are very limited in both flash and RAM, therefore they do not currently support DFU,
and logging is disabled to save flash. Furthermore, the nRF52810 and nRF52820 must be compiled with optimization
for size, that is with one of the following options based
on your [building approach choice](@ref md_doc_getting_started_how_to_build):
- for CMake: `-DCMAKE_BUILD_TYPE=MinSizeRel`
- for Segger Embedded Studio: `ReleaseWithDebugInformation` setting enabled

For these reasons, the following examples do not support nRF52810 and nRF52820, and can be run only on the fully
compatible Development Kits:
- @ref md_examples_light_switch_README
- @ref md_examples_enocean_switch_README
- @ref md_examples_dimming_README
- @ref md_examples_lpn_README
- @ref md_examples_light_lightness_README
- @ref md_examples_light_lc_server_README
- @ref md_examples_light_ctl_README
- @ref md_examples_dfu_README
- @ref md_examples_sensor_README
- @ref md_examples_serial_README


### Deprecated compatibility with nRF51 @anchor compatibility_nRF51

nRF51 is no longer officially supported. The examples are built for the nRF5 SDK v17.0.2
and the SoftDevices S113, S132 and S140, and are not compatible with the nRF51 devices.