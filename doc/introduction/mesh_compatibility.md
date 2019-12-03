# Compatibility

This page contains information about the compatibility status of the nRF5 SDK for Mesh with Nordic's boards and SoftDevice versions.
For more information about Nordic boards, see @link_nrf_dk.

**Table of contents**
- [Full compatibility](@ref compatibility_list)
- [Limited or deprecated compatibility](@ref compatibility_limited)
    - [Support for nRF52810 limited](@ref compatibility_nRF52810)
    - [Support for nRF51 deprecated](@ref compatibility_nRF51)


---

## Full compatibility @anchor compatibility_list

The mesh stack is fully compatible with the configurations mentioned in the following table.

| SoC                                            | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1         |
| nRF52833_xxAA                                  | PCA10100  | S140 v7.0.1<br>S113 v7.0.1                              |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1  |

All [examples](@ref md_examples_README) support these configurations.

---

## Limited or deprecated compatibility @anchor compatibility_limited

The mesh stack does not fully support the configurations mentioned in the following table.

| SoC                                            | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52810_xxAA ([limited support](@ref compatibility_nRF52810))                | PCA10040e | S112 v6.0.0/v6.1.0/v6.1.1/v7.0.1<br>S113 v7.0.1  |
| nRF51422_xxAC ([deprecated support](@ref compatibility_nRF51))                | PCA10028  | S130 v2.0.1                |

See the following sections for detailed information.

### Support for nRF52810 limited @anchor compatibility_nRF52810

The mesh stack is compatible with the nRF52810 platform, but this specific platform is not supported
by the current version of the nRF5 SDK for Mesh.

This platform is very limited in both Flash and RAM, therefore it does not currently support DFU, and logging is disabled to save flash.
Furthermore, the nRF52810 must be compiled with optimization for size, that is with one of the following options based
on your [building approach choice](@ref md_doc_getting_started_how_to_build):
- for CMake: `-DCMAKE_BUILD_TYPE=MinSizeRel`
- for Segger Embedded Studio: `ReleaseWithDebugInformation` setting enabled

For these reasons, the following examples do not support nRF52810 and can be run only on the fully compatible Development Kits:
- @ref md_examples_light_switch_README
- @ref md_examples_enocean_switch_README
- @ref md_examples_experimental_dimming_README
- @ref md_examples_experimental_lpn_README
- @ref md_examples_dfu_README
- @ref md_examples_serial_README


### Support for nRF51 deprecated @anchor compatibility_nRF51

nRF51 is no longer officially supported. The examples are built for the nRF5 SDK v16.0 and the SoftDevices S113, S132 and S140, and do not support nRF51.