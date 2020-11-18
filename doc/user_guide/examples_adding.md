# Adding custom examples

You can create your own examples and add them to the compilation for both SEGGER Embedded Studio
and CMake.
Do this only when you are familiar with the [toolchain](@ref md_doc_getting_started_how_to_toolchain)
and how to [build the stack and the examples](@ref md_doc_getting_started_how_to_build).

The process of adding new examples is composed of the following stages:
- [Creating custom example folder and CMakeLists file](@ref examples_adding_files)
- [Defining application configuration](@ref examples_adding_configuration)
- [Building and running custom examples](@ref examples_adding_building)

@note
The custom example to be added is referred to as `my_app` on this page.
Replace this name with the name of your application each time it is mentioned.


---

## Creating custom example folder and CMakeLists file @anchor examples_adding_files

To add folders and the `CMakeLists.txt` file for your custom example:
-# In the `examples` folder of your nRF5 SDK for Mesh installation path, add a new `my_app` folder.
-# Copy the contents of one of the existing example folders to this new folder.
For example, copy the contents of `examples/beaconing`.
-# In the `examples/CMakeLists.txt` file, depending on your platform (see @ref md_doc_user_guide_mesh_compatibility):
    - If your example supports the nRF52810 or nRF52820 SoC, add the new folder as a new `add_subdirectory` command entry.
    For example:

            set(WEAK_SOURCE_FILES
                "${CMAKE_CURRENT_SOURCE_DIR}/common/src/nrf_mesh_weak.c"
                "${CMAKE_CURRENT_SOURCE_DIR}/common/src/app_error_weak.c"
                "${CMAKE_CURRENT_SOURCE_DIR}/common/src/assertion_handler_weak.c"
                CACHE INTERNAL "")
            add_subdirectory("common")
            add_subdirectory("beaconing")
            add_subdirectory("pb_remote")
            add_subdirectory("provisioner")
            add_subdirectory("my_app")

    - If your example does not support the nRF52810 or nRF52820 SoC, add the new folder
    as a new `add_subdirectory` command entry in the platform-related `if` section.
    For example:

            if ((NOT PLATFORM MATCHES "nrf52810") AND (NOT PLATFORM MATCHES "nrf52820"))
                add_subdirectory("my_app")
                add_subdirectory("sensor")
                add_subdirectory("scene")
                add_subdirectory("light_lightness")
                add_subdirectory("light_lc")
                add_subdirectory("light_ctl")
                add_subdirectory("enocean_switch")
                add_subdirectory("dimming")
                add_subdirectory("lpn")
                add_subdirectory("dfu")
                add_subdirectory("serial")
                add_subdirectory("light_switch")
            endif()

-# Modify the target name in the first line of the `examples/my_app/CMakeLists.txt` file
to `set(target "my_app")`.


---

## Defining application configuration @anchor examples_adding_configuration

When you created custom example folder and CMakeLists.txt file, you also copied
the following files used for configuring the application:
- `nrf_mesh_config_app.h`
- `app_config.h`

To define the example configuration:
-# In the `examples/my_app/CMakeLists.txt` file, add or remove the application support modules
based on your needs.
-# Edit the `include/app_config.h` file to match your application configuration
and override the configuration of the nRF5 SDK (see @link_SDK_configuration_header_file).
-# Edit `nrf_mesh_config_app.h` file to modify:
    - the application support module section (see @ref NRF_MESH_CONFIG_EXAMPLES and @ref MESH_API_GROUP_APP_CONFIG)
    - the Bluetooth mesh stack (see configuration pages for modules in @link_APIref, for example @ref NRF_MESH_CONFIG_DFU)

After defining the configuration, build the example with the following procedure.

---

## Building and running custom examples @anchor examples_adding_building

The building procedure for custom examples is similar
to the [standard building procedure](@ref md_doc_getting_started_how_to_build),
with the following differences:
- If you want to build examples with SEGGER Embedded Studio,
you must [generate new project files using CMake](@ref how_to_build_cmake_generating_SES)
after you have configured the application.
- If you want to build examples with CMake using ninja, build your example with the following commands:
    - The first time you build your custom example after adding it:

            build $ ninja

    - Each next time you build your custom example after building it for the first time:

            build $ ninja my_app

When [running the example in CMake-based environment](@ref md_doc_getting_started_how_to_run_examples),
flash your example with the following command:

    build $ ninja flash_my_app