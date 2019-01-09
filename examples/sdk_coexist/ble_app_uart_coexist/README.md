# SDK UART coexistence example

This example demonstrates how nRF5 SDK for Mesh and nRF5 SDK examples can be used together
at the same time. It is built around two examples and has two parts:
- The mesh part of this example implements
the [light switch client example](@ref md_examples_light_switch_client_README)
from the nRF5 SDK for Mesh.
- The BLE part of the example implements the @link_ble_app_uart_example example from the nRF5 SDK,
except for the following changes:
    - The BSP events `SLEEP`, `DISCONNECT` and `WHITELIST_OFF` are ignored. This makes
    the board buttons only control the mesh part of the application.
    - The advertising interval is increased to allow more time for the mesh stack.
    - The light switch servers can be controlled over BLE UART by sending
    the light switch client button number to be simulated.

As the result of running this example, you'll be able to use the mesh network in which this example
can replace the light switch client example. 

Before you start testing this coexistence example, see the following pages:
- @ref md_doc_getting_started_how_to_nordicSDK
- @ref md_examples_light_switch_README and @ref md_examples_light_switch_client_README


---

## Setup @anchor coexistence_uart_example_setup

You can find the source code of the SDK UART coexistence example
in the following nRF5 SDK for Mesh folder: `<InstallFolder>/examples/sdk_coexist/ble_app_uart_coexist`

The source code of the light switch client example is located
at the following path: `<InstallFolder>/examples/light_switch/client`

---

## Testing the example @anchor coexistence_uart_example_testing

1. Copy the `ble_app_uart_coexist` folder into the `examples/ble_peripheral` folder
at your nRF5 SDK installation path.
2. Open the Segger Embedded Studio project at `ble_app_uart_coexist/pca10040/s132/ses/ble_app_uart_pca10040_s132.emProject`.
3. Add MESH_ROOT to your Segger Embedded Studio global macro list:
    1. From the SES menu bar, click **Tools > Options...**.
    2. In the left column, click **Building**.
    3. In right column, double click **Global macros**.
    4. Add your mesh root directory in a new line: `MESH_ROOT=<path to your mesh installation>`.
4. Program the Light Switch example devices as described in the @ref md_examples_light_switch_README,
with the following exception:
    - Replace the light switch client with `examples/ble_peripheral/ble_app_uart_coexist/pca10040/s132/ses/Output/Release/Exe/ble_app_uart_pca10040_s132.hex`.

You can now run the two coexisting examples in parallel or in sequence:
    - Run the light switch example as described in @ref md_examples_light_switch_README.
    - Run the @link_ble_app_uart_example example as described in the nRF5 SDK documentation.
        - Write the hexadecimal ASCII codes equivalent to one of the digits (1, 2, 3, or 4)
        in the UART RX characteristic to simulate a button press.