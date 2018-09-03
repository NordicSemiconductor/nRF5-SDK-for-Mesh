# SDK Proximity coexistence example

The mesh part of this example implements the Light Switch Client example in nRF5 SDK for Mesh.

The BLE part of the example is identical to the ble_app_proximity example in the nRF5 SDK for the following changes:
- The BSP events `SLEEP` and `DISCONNECT` are ignored.
- The advertising interval is increased to allow more time for the mesh stack.

The example is intended to replace the Light Switch Client when running the Light Switch example.

## Setting up the example

- Copy the `ble_app_proximity_coexist` folder into your nRF5 SDK installation's `examples\ble_peripheral` folder
- Open the Segger Embedded Studio project at `ble_app_proximity_coexist\pca10040\s132\ses\ble_app_proximity_pca10040_s132.emProject`.
- Add MESH_ROOT to your Segger Embedded Studio global macro list.
 --- From SES menu bar, click 'Tools' -> 'Options...'
 --- In left column, click 'Building'
 --- In right column, double click 'Global macros'
 --- Add your mesh root directory on a new line. `MESH_ROOT=<path to your mesh installation>`

## Running the example

- Program the Light Switch example devices as described in the @ref md_examples_light_switch_README, except for replacing the Light Switch Client with `examples\ble_peripheral\ble_app_proximity_coexist\pca10040\s132\ses\Output\Release\Exe\ble_app_proximity_pca10040_s132.hex`.
> Note: Ensure that the device flash is completely erased before programing this example. Otherwise Flash Data Storage module will not work correctly.
- Run the two coexisting examples in parallel or in sequence:
    - Run the Light Switch example as described in @ref md_examples_light_switch_README.
    - Run the ble_app_proximity example as described in the nRF5 SDK documentation.