# DFU example

@note This example is not supported by the nRF52810 Series.

This Device Firmware Update (DFU) example illustrates how to create an application that can be updated
over the mesh. It shows the general setup and how to handle the relevant events.

## DFU modes

The mesh DFU supports two modes: side-by-side DFU and bootloader DFU. The side-by-side mode transfers the new firmware in the background while the application is running and reports to the application when the transfer is done. The application may then flash the new firmware when ready. In bootloader mode, the application is not running, and the bootloader takes care of the transfer. This mode is primarily meant as a fallback mechanism, in case the application malfunctions. This example application demonstrates the side-by-side mode DFU.


## Getting started with mesh DFU

This DFU application requires a bootloader and a valid device page to function correctly. Precompiled bootloaders are included in the `bin` directory at the project root directory, and the device page must be generated with the *device_page_generator.py* script located in *tools/dfu/device_page.py*.

See the [DFU quick start guide](@ref md_doc_getting_started_dfu_quick_start) for detailed instructions on how to perform a DFU operation using this example application.

