# Modules

This section includes conceptual documentation about different modules included in the nRF5 SDK for Mesh.
These easy-to-use modules were prepared with the goal of simplifying the development process of your Bluetooth mesh applications.
Some of the module pages may also contain step-by-step procedures that allow you to configure them
for use in your application.

The following documentation pages are available in this section:
- @subpage md_doc_user_guide_modules_dfu_protocol includes information about the DFU protocol
in the Bluetooth mesh stack, with details about its characteristics, proprietary mesh DFU Firmware IDs, and the device page.
- @subpage md_doc_user_guide_modules_flash_manager contains information about flash manager areas,
handles, defragmentation, power failure protection, and flash area locations.
- @subpage md_doc_user_guide_modules_lpn_concept describes concepts related to the Friendship protocol,
which enables power-constrained devices to be part of a Bluetooth mesh network.
- @subpage md_doc_user_guide_modules_mesh_config describes the Bluetooth mesh submodule
for simplifying persistent key-value storage, including details about information flow and the usage
in the stack and in the application.
- @subpage md_doc_user_guide_modules_instaburst contains information about a Nordic proprietary feature
that is a drop-in replacement for the standard BLE Advertiser bearer for the mesh.
- @subpage md_doc_user_guide_modules_models_main is a section that includes documentation
for Bluetooth mesh models available in the stack.
- @subpage md_doc_user_guide_modules_pa_lna_support describes a PA/LNA module that can be used
to increase the range of Bluetooth Low Energy communication.
- @subpage md_doc_user_guide_modules_power_down describes a proprietary functionality of the stack that can be used
to turn off the stack in situations of expected power shortage.
- @subpage md_doc_user_guide_modules_provisioning_main describes in detail the provisioning process
and bearers in the nRF5 SDK for Mesh.
- @subpage md_scripts_README is a section dedicated to the serial module,
which provides full serialization of the Bluetooth mesh API, allowing other devices
to control the nRF5 mesh network device through a UART interface.