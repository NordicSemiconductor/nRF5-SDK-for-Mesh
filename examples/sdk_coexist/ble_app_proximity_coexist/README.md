# SDK Proximity coexistence example

The mesh part of this example implements the Light Switch Client example in nRF5 SDK for Mesh.

The BLE part of the example is identical to the ble_app_proximity example in the nRF5 SDK for the following changes:
- The BSP events `SLEEP` and `DISCONNECT` are ignored.
- The advertising interval is increased to allow more time for the mesh stack.

The example is intended to replace the Light Switch Client when running the Light Switch example.

## Setting up the example

- Copy example into your nRF5 SDK installation's examples\ble_peripheral folder
- Open the example in Segger Embedded Studio by double clicking the `examples\ble_peripheral\ble_app_proximity_coexist\pca10040\s132\ses\ble_app_proximity_pca10040_s132.emProject`.
- In Segger Embedded Studio, set the MESH_ROOT global macro by selecting `Options...` on the `Tools` menu. In the `Environment Options` dialog, go to the `Building` group, and select the `Global macros` option. Click the ellipsis button on the right, and the macro `MESH_ROOT=<path to your mesh installation>`.

## Running the example

- Program the Light Switch example devices as described in the @ref doc_examples_light_switch_README_md, except for replacing the Light Switch Client with `examples\ble_peripheral\ble_app_proximity_coexist\pca10040\s132\ses\Output\Release\Exe\ble_app_proximity_pca10040_s132.hex`.
- Run the two coexisting examples in parallel or in sequence:
    - Run the Light Switch example as described in @ref doc_examples_light_switch_README_md.
    - Run the ble_app_proximity example as described in the nRF5 SDK documentation.