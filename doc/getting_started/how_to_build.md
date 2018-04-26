# Building the mesh stack and examples

The mesh library and example applications can be built using either @link_cmake <!--CMake: https://cmake.org/--> or
@link_seggerstudio<!--SEGGER Embedded Studio: https://www.segger.com/products/development-tools/embedded-studio/?L=0-->.

Using CMake provides the possibility to build both for host (unit tests) and target, while SEGGER
Embedded Studio provides a way of quickly getting the example code up and running with full debug
capability.

Before you continue, check @ref md_doc_getting_started_how_to_toolchain for instructions on setting up the
development environment for mesh.

## nRF5 SDK @anchor how_to_build_nrf_sdk
The nRF5 SDK for Mesh now *requires* the nRF5 SDK to compile. By default, the nRF5 SDK is expected
to be adjacent to the nRF5 SDK for Mesh. The directory structure should look like this:

    .
    +-- nrf5_sdk_for_mesh/
    +-- nRF5_SDK_15.0.0_a53641a/


You can get the correct SDK in two ways: manually or by using a custom CMake target.

### Manually
Download the nRF5 SDK version 15.0.0 from the web:
<a href="http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/" target="_blank">developer.nordicsemi.com</a>.
Extract it in the same folder as the nRF5 SDK for Mesh to match the folder structure above.

### Using CMake

Generate CMake build files as in [Building with CMake ](@ref md_doc_getting_started_how_to_build_cmake),
e.g.:

    nrf5_sdk_for_mesh $ mkdir build
    nrf5_sdk_for_mesh $ cd build
    build $ cmake -GNinja ..

You will get a warning that the nRF5 SDK isn't found. Run the `nRF5_SDK` target:

    build $ ninja nRF5_SDK

This will download and extract the correct nRF5 SDK in the folder adjecent to the nRF5 SDK for mesh.
After the download is complete, re-run CMake and it will pick up the correct paths:

    build $ cmake ..

## Building

Please follow the guide for your preferred toolchain. See @subpage md_doc_getting_started_how_to_build_cmake or
@subpage md_doc_getting_started_how_to_build_segger.