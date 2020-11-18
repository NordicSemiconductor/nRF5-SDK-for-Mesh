# DFU example
@anchor dfu_example

@tag52840and52833and52832
@tag52840dongle52810and52820nosupport

This Device Firmware Update (DFU) example illustrates how to create an application that can be updated
over Bluetooth mesh using background mode DFU. In this mode, the new firmware is transferred in the background
while the application is running and the DFU reports to the application when the transfer is done.
The application can then flash the new firmware when ready.

More information about the DFU can be found in the @ref md_doc_user_guide_modules_dfu_protocol section.

---

## Hardware requirements @anchor dfu_example_requirements_hw

You need one compatible development kit for this DFU example.

See @ref md_doc_user_guide_mesh_compatibility for information about
the compatible development kits.

---

## Software requirements @anchor dfu_example_requirements_sw

This DFU application requires a bootloader and a valid device page to function correctly.

---

## Setup @anchor dfu_example_setup

You can find the source code of the DFU example in the following folder: `<InstallFolder>/examples/dfu`

Precompiled bootloaders are included in the `bin` directory at the project root directory
and the device page must be generated with the `device_page_generator.py` script located in `tools/dfu`.
See also @ref md_tools_dfu_README.

---

## Testing the example @anchor dfu_example_testing

See @ref md_doc_user_guide_modules_dfu_configuring_performing for detailed instructions on how to perform
a DFU operation using this example application.

