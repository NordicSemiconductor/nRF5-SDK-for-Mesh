# Compatibility

The mesh stack is compatible with the following configurations:

| SoC                                            | Boards    | SoftDevices                |
| ---------------------------------------------- | --------- | -------------------------- |
| nRF51422_xxAC (deprecated)                     | PCA10028  | S130 v2.0.1                |
| nRF52810_xxAA                                  | PCA10040e | S112 v6.0.0/v6.1.0         |
| nRF52832_xxAA                                  | PCA10040  | S132 v5.0.0/v6.0.0/v6.1.0  |
| nRF52840_xxAA                                  | PCA10056  | S140 v6.0.0/v6.1.0         |

The mesh stack is compatible with the nRF52810 platform, but not supported by this version of the nRF5 SDK for Mesh.

Moreover, the nRF52810 support is limited. It is very limited in both Flash and RAM, therefore it does
not currently support DFU and logging is disabled to save flash. Furthermore, it must be compiled with
optimization for size, that is `-DCMAKE_BUILD_TYPE=MinSizeRel` for CMake or the 'ReleaseWithDebugInformation'
setting in SEGGER Embededd Studio.

Support for the nRF51 Series platform is being dropped, but building the examples
is still supported using the CMake build system.

@note The Light Switch, EnOcean, Dimming, DFU, and Serial examples are only for
the nRF52832 Series and the nRF52840 Series.

![nRF52 DK](img/pca10040_front_v1.0.0.svg "nRF52 DK")
