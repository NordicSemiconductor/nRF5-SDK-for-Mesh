# How to add PA/LNA support

The nRF5 SDK for Mesh provides APIs for interfacing external Front End Modules (FEMs) to increase
the range of Bluetooth Low Energy communication.

Typically, these FEMs are controlled by the enable signals that turn on a power amplifier (PA) or
a low noise amplifier (LNA) (see [Interfacing PA/LNA with nRF52](@ref pa-lna-Figure1)). These signals must be activated some time
before the actual radio transmission or reception begins, to provide sufficient ramp-up time.

@anchor pa-lna-Figure1
![Interfacing PA/LNA with nRF52](img/pa-lna-block-dia.svg "Interfacing PA/LNA with nRF52")

The Mesh PA/LNA module enables users to control such external components using GPIOs that are synchronized
to the radio operation. The PA/LNA module drives the chosen GPIO pins according to the chosen polarity
(Active High/Active Low).


To show how to use the Mesh PA/LNA module, we start with the `light_switch/server` example and add PA/LNA support to it.

## Adding PA/LNA support to the application

First, select unused GPIO pins that can be used by the PA/LNA module. For this example, we use GPIO 25 for
controlling the LNA and GPIO 24 for controlling the PA. Let's assume that the control signals
required by the external hardware module are active high.

The Mesh PA/LNA module uses the @link_52832_PPI <!--PPI: http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fppi.html&anchor=concept_sxf_21l_1s -->
and @link_52832_GPIOTE <!-- GPIOTE: http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fgpiote.html&anchor=concept_knz_kww_lr -->
hardware modules to generate these signals.

For this example, we choose the unused PPI channels 0 and 1, and GPIOTE channel 0.

Now, make the following changes in `light_switch/server/src/main.c`:
1. Include the required module header file: `mesh_pa_lna.h`
2. Create a static global variable of type @ref mesh_pa_lna_gpiote_params_t and initialize it
with the selected values:

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

3. Enable the PA/LNA module by calling @ref mesh_pa_lna_gpiote_enable() after the call to
`mesh_init()`:

        mesh_pa_lna_gpiote_enable(&m_pa_lna_params);

4. To build the example, follow the instructions in
[Building the Mesh Stack](@ref md_doc_getting_started_how_to_build). See *How to run examples*
in the [examples documentation](@ref md_examples_README) for the commands required to program a
device using `nrfjprog`.

If you connect a logic analyzer to the GPIO pins 25 and 24, you should see them toggling.

Observe that the unprovisioned device sends the unprovisioned node beacons every
two seconds (@ref NRF_MESH_UNPROV_BEACON_INTERVAL_MS) and scans for the incoming provisioning
invite for the rest of the time.

![GPIO waveforms after enabling PA/LNA module](img/pa-lna-waveform.png "GPIO waveforms after enabling PA/LNA module")

You will see brief active high pulses on GPIO 24, which we have used for the PA control.
Similarly, GPIO 25, which we have used for the LNA control, is almost always ON
except for the time when the radio switches to the next advertising channel (for scanning
or for sending advertisements).

See the [PA/LNA API documentation](@ref MESH_PA_LNA) for more details.
