# Bluetooth mesh models

In the Bluetooth mesh solution, models are used to define the functionality of nodes.
Each model represents a set of states and behaviors and defines messages that are
used to interact with the model states.

One example of a model is the @ref CONFIG_MODEL, which is a mandatory model in all
Bluetooth mesh devices. This model represents the configuration of a node (in the form of various
states) and provides messages to set or retrieve configuration parameters (behavior).

In addition to the foundation models defined in the @link_ModelSpec, the nRF5 SDK for Mesh
provides unique models for performing functionalities on the nodes in a Bluetooth mesh network.

You can also create your own models. For details,
see @subpage md_doc_user_guide_modules_models_creating.

@link_btsig_glossary.

**Table of contents**
- [Models in the nRF5 SDK for Mesh](@ref model_list_nrf5_sdk_mesh)
    - [Pointers as arguments to model callbacks](@ref model_callback_pointer_note)
    - [Segmentation forcing](@ref mesh_model_force_segmented)
    - [Large MIC size](@ref mesh_model_large_mic)


---

## Models in the nRF5 SDK for Mesh @anchor model_list_nrf5_sdk_mesh

See the following list for the complete overview of models implemented in the nRF5 SDK for Mesh.

@note Links under names indicate models with documentation pages.

- Generic models
    - Default Transition Time model
        - Source files: `<InstallFolder>/models/model_spec/generic_dtt`
        - @ref GENERIC_DTT_MODEL "API reference"
    - OnOff model
        - Source files: `<InstallFolder>/models/model_spec/generic_onoff`
        - @ref GENERIC_ONOFF_MODEL "API reference"
    - Level model
        - Source files: `<InstallFolder>/models/model_spec/generic_level`
        - @ref GENERIC_LEVEL_MODEL "API reference"
    - Power OnOff model
        - Source files: `<InstallFolder>/models/model_spec/generic_ponoff`
        - @ref GENERIC_PONOFF_MODEL "API reference"
- Foundation models
    - Health model
        - Source files: `<InstallFolder>/models/foundation/health`
        - @ref HEALTH_MODEL "API reference"
    - Configuration model
        - Source files: `<InstallFolder>/models/foundation/config`
        - @ref CONFIG_MODEL "API reference"
- Scenes models
    - Scenes model
        - Source files: `<InstallFolder>/models/model_spec/scene`
        - @ref SCENE_MODELS "API reference"
- Sensor models
    - Sensor model
        - Source files: `<InstallFolder>/models/model_spec/sensor`
        - @ref SENSOR_MODEL "API reference"
- Light models
    - Light Lightness models
        - Source files: `<InstallFolder>/models/model_spec/light_lightness`
        - @ref LIGHT_LIGHTNESS_MODELS "API reference"
    - Light CTL models
        - Source files: `<InstallFolder>/models/model_spec/light_ctl`
        - @ref LIGHT_CTL_MODELS "API reference"
    - Light LC models
        - Source files: `<InstallFolder>/models/model_spec/light_lc`
        - @ref LIGHT_LC_MODELS "API reference"
- Proprietary models
    - PB-remote model
        - Source files: `<InstallFolder>/models/proprietary/pb_remote`
        - @ref PB_REMOTE "API reference"
- Vendor models
    - @subpage md_models_vendor_simple_on_off_README
        - Source files: `<InstallFolder>/models/vendor/simple_on_off`
        - @ref SIMPLE_ON_OFF_MODEL "API reference"
    - @subpage md_models_vendor_rssi_monitor_README (@ref md_models_vendor_rssi_monitor_doc_rssi_model
    and @ref md_models_vendor_rssi_monitor_doc_rssi_util)
        - Source files: `<InstallFolder>/models/vendor/rssi_monitor`
        - @ref MESH_OPT_CORE "API reference"

For the complete overview of available Bluetooth mesh model APIs, see \ref MESH_API_GROUP_MODELS
in the API Reference section.

### Pointers as arguments to model callbacks @anchor model_callback_pointer_note

Models in the nRF5 SDK for Mesh communicate with the application through a number of
callback functions implemented by the application. Many of these, especially get callbacks,
supply a data pointer as one of the arguments from the model.

These pointers are to be regarded as local (stack) variables,
and will go out of scope once the callback chain returns.
For this reason, do not be store or dereference these pointers outside the scope of the callback implementation.
If the data is needed outside this scope, make sure that the application copies the data.

### Segmentation forcing @anchor mesh_model_force_segmented

The model implementation in the nRF5 SDK for Mesh provides the ability to apply segmentation behavior
for model messages even if these messages are fit in a single transport layer payload.
Use the `force_segmented` boolean for this purpose.

However, this approach has its advantages and disadvantages.
On the one hand, forcing segmentation improves the reliability of communication.
The Bluetooth mesh stack will apply the Segment Acknowledgement message to confirm every model message.

On the other hand, this approach will cause poor latency, because the confirmation stage requires time.
Moreover, a model might get the NRF_ERROR_NO_MEM status back from the stack for the next transaction.
This can happen if a model sends more messages that require SAR, but the application does not
take into account notifications about the completion of the previous SAR messages.

### Large MIC size @anchor mesh_model_large_mic

The model implementation in the nRF5 SDK for Mesh provides the ability
to use the 8-byte Message Integrity Check (MIC) for messages. Use `transmic_size` for this purpose.

When using this MIC, it is important to check that the model data still fits in the transport payload.
If a model message cannot be fit in a single transport payload, the segmentation mechanism will be applied automatically.
