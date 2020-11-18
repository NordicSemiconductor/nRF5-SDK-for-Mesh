# PA/LNA module

The nRF5 SDK for Mesh provides a PA/LNA module with APIs for interfacing the external Front End Modules
(FEMs) to increase the range of Bluetooth Low Energy communication.

The FEMs are controlled by the enable signals that turn on a power amplifier (PA) or
a low noise amplifier (LNA) (see the following figure). To ensure sufficient ramp-up time,
these signals must be activated some time before the start of the radio transmission or reception.

@anchor pa-lna-Figure1
![Interfacing PA/LNA with an nRF52 device](images/pa-lna-block-dia.svg "Interfacing PA/LNA with an nRF52 device")

The Bluetooth mesh PA/LNA module enables users to control such external components using GPIOs
that are synchronized to the radio operation. The PA/LNA module drives the chosen GPIO pins
according to the chosen polarity (Active High/Active Low).

See the following section for an example of how to use the PA/LNA module.
You can find more information about the available APIs
in the [PA/LNA API documentation](@ref MESH_PA_LNA).

The PA/LNA module works with all the [fully compatible configurations based on nRF52 devices](@ref compatibility_list).

---

## Adding PA/LNA support to the application @anchor pa-lna_adding_support

This section describes how to add PA/LNA support to the `light_switch/server` example.
For more information about this example, see @ref md_examples_light_switch_README
and @ref md_examples_light_switch_server_README.

To add the PA/LNA support, complete the steps in the following subsections:
- [Selecting GPIO pins and PPI channels](@ref pa-lna_adding_support_pins_channels)
- [Editing the main.c file](@ref pa-lna_adding_support_main)
- [Running the example and observing the results](@ref pa-lna_adding_support_running)

### Selecting GPIO pins and PPI channels @anchor pa-lna_adding_support_pins_channels

Complete the following steps:

-# Select the unused GPIO pins that can be used by the PA/LNA module.
    - For this example, use GPIO 25 for controlling the LNA and GPIO 24 for controlling the PA.
    - Also, assume that the control signals required by the external hardware module are Active High.
    The PA/LNA module uses the PPI and GPIOTE hardware modules to generate these signals.
    To read more about these modules in nRF52832, see the following documents: @link_52832_PPI and @link_52832_GPIOTE .
-# Select the unused PPI channels 0 and 1, and the GPIOTE channel 0.

### Editing the main.c file @anchor pa-lna_adding_support_main

Make the following changes in `light_switch/server/src/main.c`:

-# Include the required module header file: `mesh_pa_lna.h`
-# Create a static global variable of type @ref mesh_pa_lna_gpiote_params_t and initialize it
with the selected values:
```
        static mesh_pa_lna_gpiote_params_t m_pa_lna_params = {
                .lna_cfg = {
                    .enable = 1,
                    .active_high = 1,
                    .gpio_pin = 25
                },
                .pa_cfg = {
                    .enable = 1,
                    .active_high = 1,
                    .gpio_pin = 24
                },
                .ppi_ch_id_set = 0,
                .ppi_ch_id_clr = 1,
                .gpiote_ch_id = 0
            };
```
-# Enable the PA/LNA module by calling @ref mesh_pa_lna_gpiote_enable() after the call to
`mesh_init()`:

    mesh_pa_lna_gpiote_enable(&m_pa_lna_params);
-# Enable the PA/LNA module in the BLE stack by completing the following steps:
    -# Create a static global variable of type @link_ble_opt_t.
    You can use the same GPIO pins and the same PPI and GPIO channels as for
    Bluetooth mesh PA/LNA configuration:
```
        static ble_opt_t ble_pa_lna_opts = {
             .common_opt = {
                .pa_lna = {
                    .pa_cfg = {
                        .enable = 1,
                        .active_high = 1,
                        .gpio_pin = 24
                    },
                    .lna_cfg = {
                        .enable = 1,
                        .active_high = 1,
                        .gpio_pin = 25
                    },
                    .ppi_ch_id_set = 0,
                    .ppi_ch_id_clr = 1,
                    .gpiote_ch_id = 0
                }
            }
        };
```
    -# Call @link_sd_ble_opt_set function before the call to `mesh_init()`:
```
        err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &ble_pa_lna_opts);
        APP_ERROR_CHECK(err_code);
```
    -# If @ref MESH_PROVISIONEE is used, implement @ref mesh_provisionee_start_params_t.prov_sd_ble_opt_set_cb
    callback and call @link_sd_ble_opt_set in the callback:
```
        static void provisioning_sd_ble_opt_cb()
        {
            err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &ble_pa_lna_opts);
            APP_ERROR_CHECK(err_code);
        }

        ...

        static void start(void)
        {
            rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

            if (!m_device_provisioned)
            {
                static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
                mesh_provisionee_start_params_t prov_start_params =
                {
                    .p_static_data    = static_auth_data,
                    .prov_sd_ble_opt_set_cb = provisioning_sd_ble_opt_cb,
                    .prov_complete_cb = provisioning_complete_cb,
                    .prov_device_identification_start_cb = device_identification_start_cb,
                    .prov_device_identification_stop_cb = NULL,
                    .prov_abort_cb = provisioning_aborted_cb,
                    .p_device_uri = EX_URI_LS_SERVER
                };
                ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
            }

            ...
        }
```
    This is required to restore the PA/LNA configuration in the BLE stack after it is
    restarted during the provisioning when [PB GATT feature](@ref MESH_FEATURE_PB_GATT_ENABLED)
    is enabled.


### Running the example and observing the results @anchor pa-lna_adding_support_running

Complete the following steps:

-# Build the example by following the instructions in @ref md_doc_getting_started_how_to_build.
-# Run the example by following the instructions in @ref md_doc_getting_started_how_to_build
for [commands required to program a device using `nrfjprog`](@ref how_to_run_examples_nrfjprog).

If you connect a logic analyzer to the GPIO pins 25 and 24, you will see them toggling.

The unprovisioned device sends the unprovisioned node beacons every
two seconds (@ref NRF_MESH_PROV_BEARER_ADV_UNPROV_BEACON_INTERVAL_MS) and scans for
the incoming provisioning invite for the rest of the time.

![GPIO waveforms after enabling PA/LNA module](images/pa-lna-waveform.png "GPIO waveforms after enabling PA/LNA module")

You will see brief Active High pulses on the GPIO pin 24, which is used for the PA control.
Similarly, the GPIO pin 25 (used for the LNA control) is almost always ON,
except for the time when the radio switches to the next advertising channel for scanning
or for sending advertisements.
