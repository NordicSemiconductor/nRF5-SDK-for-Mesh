# Building with SEGGER Embedded Studio

To build with SEGGER Embedded Studio, open one of the project files located in the `examples/` folder,
e.g.,  `examples/light_switch/client/light_switch_client_nrf52832_xxAA_s132_6_0_0.emProject`.

> **Important:** Before building the mesh examples with SEGGER Embedded Studio for the first time,
> it must be set up to find the nRF5 SDK.
>
> See @subpage md_doc_getting_started_SES for more information.

To compile the example, go to `Build -> Build light_switch_client_nrf52832_xxAA_s132_6.0.0`. After the
compilation is complete, first erase the device using `Target -> Erase all` and run the example with `Debug -> Go`.
This will download the matching SoftDevice and the compiled example and start the debugger.
When the download is complete select `Debug -> Go` again to start the code execution.
