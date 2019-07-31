# Mesh models

In the Bluetooth Mesh solution, models are used to define the functionality of nodes.
Each model represents a set of states and behaviors and defines messages that are
used to interact with the model states.

One example of a model is the @ref CONFIG_MODEL, which is a mandatory model in all
mesh devices. This model represents the configuration of a node (in the form of various
states) and provides messages to set or retrieve configuration parameters (behavior).

In addition to the foundation models defined in the @link_ModelSpec, the nRF5 SDK for Mesh
provides unique models for performing functionalities on the nodes in a mesh network.

You can also create your own models. For details,
see @subpage md_doc_libraries_how_to_models.

---

## Models in the nRF5 SDK for Mesh

See the following list for the complete overview of models implemented in the nRF5 SDK for Mesh.

@note Links under names indicate models with documentation pages.

- Generic models
    - Default Transition time model
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
- Vendor models
    - @subpage md_models_vendor_simple_on_off_README
        - Source files: `<InstallFolder>/models/vendor/simple_on_off`
        - @ref SIMPLE_ON_OFF_MODEL "API reference"
    - @subpage md_models_vendor_rssi_monitor_README (@ref md_models_vendor_rssi_monitor_doc_rssi_model
    and @ref md_models_vendor_rssi_monitor_doc_rssi_util)
        - Source files: `<InstallFolder>/models/vendor/rssi_monitor`
        - @ref MESH_OPT_CORE "API reference"
- Experimental models
    - PB-remote model
        - Source files: `<InstallFolder>/models/experimental/pb_remote`
        - @ref PB_REMOTE "API reference"
        
For the complete overview of available Mesh Model APIs, see \ref MESH_API_GROUP_MODELS
in the API Reference section.