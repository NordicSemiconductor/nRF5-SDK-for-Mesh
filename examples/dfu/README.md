# DFU example

@note This example is not supported by the nRF52810 Series.

This Device Firmware Update (DFU) example illustrates how to create an application
that can be updated over the mesh using side-by-side mode DFU. In this mode, the new firmware
is transferred in the background while the application is running and the DFU reports
to the application when the transfer is done. The application can then flash the new firmware
when ready.

More information about the DFU can be found on the @ref md_doc_getting_started_dfu_quick_start page.

---

## Software requirements @anchor dfu_example_requirements_hw

This DFU application requires a bootloader and a valid device page to function correctly.

---

## Setup @anchor dfu_example_setup

You can find the source code of the DFU example in the following folder: `<InstallFolder>/examples/dfu`

Precompiled bootloaders are included in the `bin` directory at the project root directory
and the device page must be generated with the `device_page_generator.py` script located
in `tools/dfu/device_page.py`.

---

## Testing the example @anchor dfu_example_testing

See @ref md_doc_getting_started_dfu_quick_start for detailed instructions on how to perform
a DFU operation using this example application.

