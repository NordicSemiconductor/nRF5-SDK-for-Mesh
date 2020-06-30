# Compatibility

This page contains information about the compatibility status of the nRF5 SDK for Mesh with Nordic's
development kits and SoftDevice versions.

If a Nordic kit is not mentioned on this page, it means that it is not supported.

For more information about Nordic kits, see @link_nrf_dk.

**Table of contents**
- [Full compatibility](@ref compatibility_list)
- [Limited or deprecated compatibility](@ref compatibility_limited)
    - [Compatibility with nRF52810 limited](@ref compatibility_nRF52810)
    - [Compatibility with nRF51 deprecated](@ref compatibility_nRF51)


---

## Full compatibility @anchor compatibility_list

The mesh stack is fully compatible with the configurations mentioned in the following table.

| Device                                         | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1         |
| nRF52833_xxAA                                  | PCA10100  | S140 v7.0.1<br>S113 v7.0.1                              |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1  |

All [examples](@ref md_examples_README) are compatible with these configurations.

---

## Limited or deprecated compatibility @anchor compatibility_limited

The mesh stack is not fully compatible with the configurations mentioned in the following table.

| Device                                         | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52810_xxAA ([limited compatibility](@ref compatibility_nRF52810))                | PCA10040e | S112 v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1  |
| nRF51422_xxAC ([deprecated compatibility](@ref compatibility_nRF51))                | PCA10028  | S130 v2.0.1                |

See the following sections for detailed information.

### Compatibility with nRF52810 limited @anchor compatibility_nRF52810

The mesh stack is compatible with the nRF52810 platform, but this specific platform is not supported
by the current version of the nRF5 SDK for Mesh.

This platform is very limited in both Flash and RAM, therefore it does not currently support DFU,
and logging is disabled to save flash. Furthermore, the nRF52810 must be compiled with optimization
for size, that is with one of the following options based
on your [building approach choice](@ref md_doc_getting_started_how_to_build):
- for CMake: `-DCMAKE_BUILD_TYPE=MinSizeRel`
- for Segger Embedded Studio: `ReleaseWithDebugInformation` setting enabled

For these reasons, the following examples do not support nRF52810 and can be run only on the fully
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


### Compatibility with nRF51 deprecated @anchor compatibility_nRF51

nRF51 is no longer officially supported. The examples are built for the nRF5 SDK v16.0
and the SoftDevices S113, S132 and S140, and are not compatible with the nRF51 devices.