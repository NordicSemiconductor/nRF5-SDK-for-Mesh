# Compatibility

This page contains information about the compatibility status of the nRF5 SDK for Mesh with Nordic's boards and SoftDevice versions.

**Table of contents**
- [Compatibility list](@ref compatibility_list)
    - [Support for nRF52](@ref compatibility_nRF52)
        - [Experimental support for S113](@ref compatibility_s113)
        - [Support for nRF52810 limited](@ref compatibility_nRF52810)
    - [Support for nRF51 deprecated](@ref compatibility_nRF51)


---

## Compatibility list @anchor compatibility_list

The mesh stack is compatible with the configurations mentioned in the following table.

| SoC                                            | Boards    | SoftDevices                       |
| ---------------------------------------------- | --------- | --------------------------------- |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0/v6.1.0/v6.1.1<br>[S113 v7.0.0 (experimental)](@ref compatibility_s113)         |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0/v6.1.0/v6.1.1<br>[S113 v7.0.0 (experimental)](@ref compatibility_s113)  |
| nRF52810_xxAA ([limited support](@ref compatibility_nRF52810))                | PCA10040e | S112 v6.0.0/v6.1.0/v6.1.1<br>[S113 v7.0.0 (experimental)](@ref compatibility_s113)  |
| nRF51422_xxAC ([deprecated support](@ref compatibility_nRF51))                | PCA10028  | S130 v2.0.1                |

### Support for nRF52 @anchor compatibility_nRF52

The nRF52 Series is the primary supported platform for the nRF5 SDK for Mesh.

All the examples require the nRF52 Development Kit.
However, not all can run on nRF52810, because of the [limited support for this platform](@ref compatibility_nRF52810).

![nRF52 DK](img/pca10040_front_v1.0.0.svg "nRF52 DK")


#### Experimental support for S113 @anchor compatibility_s113

An experimental support for the S113 v7.0.0 SoftDevice has been added to the nRF5 SDK for Mesh.
As a consequence, the nRF5 SDK must be patched if you want to use S113.
The patch file is available at the following location:
```
external\softdevice\s113_7.0.0\s113_patch_for_sdk_15_3.diff
```

The @ref md_examples_experimental_lpn_README and @ref md_examples_sdk_coexist_README cannot be used
with the S113 SoftDevice due to a significant number of patches that are required to make other
BLE features in the nRF5 SDK work seamlessly with the S113 SoftDevice.


#### Support for nRF52810 limited @anchor compatibility_nRF52810

The mesh stack is compatible with the nRF52810 platform, but this specific platform is not supported
by the current version of the nRF5 SDK for Mesh.

This platform is very limited in both Flash and RAM, therefore it does
not currently support DFU and logging is disabled to save flash. Furthermore, the nRF52810 must be compiled with
optimization for size, that is with one of the following options based on your [building approach choice](@ref md_doc_getting_started_how_to_build):
- for CMake: `-DCMAKE_BUILD_TYPE=MinSizeRel`
- for Segger Embedded Studio: `ReleaseWithDebugInformation` setting enabled

For these reasons, the Light Switch, EnOcean, Dimming, DFU, and Serial examples do not support nRF52810 and can only be run
on the nRF52832 Series and the nRF52840 Series Development Kits.

### Support for nRF51 deprecated @anchor compatibility_nRF51

nRF51 is no longer officially supported. The examples are built for the nRF5 SDK v15.2 and the SoftDevices S132 and S140, and do not support nRF51.