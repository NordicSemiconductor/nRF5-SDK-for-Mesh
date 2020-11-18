# Bluetooth mesh stack user guide

This section contains a broad range of documentation that is related to the nRF5 SDK for Mesh stack,
from the description of basic concepts and architecture to detailed guides for working with the stack
modules.

@par Stack overview
The following sections provide an overview of the stack structure:
- @subpage md_doc_user_guide_mesh_basic_concepts describes the Bluetooth mesh profile's relation
to Bluetooth Low Energy, its network topology and relaying, as well as other key concepts.
- @subpage md_doc_user_guide_mesh_basic_architecture offers specific information
about Nordic's implementation of the Bluetooth mesh.

@par Stack structure
The following sections provide detailed information about the nRF5 SDK for Mesh structure
and its relation to hardware:
- @subpage md_doc_user_guide_mesh_compatibility lists SoCs, boards, and SoftDevices that can work
with the Bluetooth mesh stack.

- @subpage md_doc_user_guide_mesh_hw_resources provides information about hardware resources
required by the stack, including hardware peripherals, and RAM and flash usage.

Make sure to take a look at the @ref md_examples_light_switch_README. It shows how a simple
application can use the Bluetooth mesh stack and serves as an introduction to the Bluetooth mesh concepts
and SDK features and APIs.

@par Modules
The @subpage md_doc_user_guide_modules_modules_main section describes some of the modules available
in the nRF5 SDK for Mesh, and provides detailed information about how to set up and use some
of these modules.

@par Stack procedures
The following pages provide detailed information about some of the procedures you might want to use
when working with the nRF5 SDK for Mesh:
- @subpage md_doc_user_guide_mesh_interrupt_priorities describes the two interrupt priorities
available for the Bluetooth mesh stack, and how to configure them in an application that uses nRF5 SDK for Mesh.
- @subpage md_doc_user_guide_examples_adding describes how to create, configure,
and build and run custom examples.
- @subpage md_doc_user_guide_integrating_mesh_nrf5_sdk is a guide for either
including nRF5 SDK in a Bluetooth mesh example or including a Bluetooth mesh functionality in an nRF5 SDK example.