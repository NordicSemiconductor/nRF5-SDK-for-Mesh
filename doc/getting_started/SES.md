# Segger Embedded Studio: SDK_ROOT first time setup

Segger Embedded Studio determines the location of the nRF5 SDK through macros.
Before building the Bluetooth mesh examples with SEGGER Embedded Studio, you must complete
a one-time setup of the `SDK_ROOT` macro in SEGGER Embedded Studio.

You can either:
- Use the default settings of the `SDK_ROOT` macro. It defaults to an nRF5 SDK 17.0.2
instance unzipped right next to the nRF5 SDK for Mesh folder.
- Set the `SDK_ROOT` macro to a custom nRF5 SDK instance.

To set the `SDK_ROOT` macro manually in SEGGER Embedded Studio:
1. Go to "Tools" -> "Options".
2. Select "Building".
3. Under "Build" in the configuration list, edit "Global macros" to
contain `SDK_ROOT=<the path to nRF5 SDK instance>`.
4. Save the configuration.

You can verify the path by opening one of the source files under the nRF5 SDK
file group. If the macro is set correctly, the file opens in the editor
window. If not, an error message is displayed with information that the file cannot
be found.

## Further reading

For more information, see the following pages:
- Segger Embedded Studio macros: https://studio.segger.com/ide_project_macros.htm
- Building examples with SES: https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.meshsdk.v4.1.0/md_doc_getting_started_how_to_build.html#how_to_build_segger
- Running examples with SES: https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.meshsdk.v4.1.0/md_doc_getting_started_how_to_run_examples.html#how_to_run_examples_ses
- nRF5 SDK for Mesh examples: https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.meshsdk.v4.1.0/md_examples_README.html
