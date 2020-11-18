# Creating new models

This guide presents the basics of how to create new models. You may implement your own
vendor-specific model that will enable your devices to provide custom states and behaviors
not covered by the already defined standard models.

For the list of available Bluetooth mesh model APIs,
see \ref MESH_API_GROUP_MODELS in the API Reference section.

**Table of contents**
- [Step 1: Define opcode handlers](@ref creating_models_implementing_define)
- [Step 2: Allocate and bind the model to an element](@ref creating_models_implementing_allocate)
- [Step 3: Set up publication and subscription](@ref creating_models_publication_subscription)
    - [Step 3.1: Set up publication](@ref creating_models_publication_subscription_1)
    - [Step 3.2: Set up subscription](@ref creating_models_publication_subscription_2)
- [Example](@ref creating_models_example)


---

## Step 1: Define opcode handlers @anchor creating_models_implementing_define

Define a table of handlers for incoming messages by creating an array
of `access_opcode_handler_t`. Each element in this array functions as a lookup table entry for
handling opcodes of incoming messages destined for the model.

---

## Step 2: Allocate and bind the model to an element @anchor creating_models_implementing_allocate

All models must be bound to an element. An element represents an addressable unit in a device,
such as a light bulb in a light fixture. For this reason, each element is assigned a separate
unicast address by the provisioner.

Use the `access_model_add()` API to allocate, initialize, and bind the model to the element
at the given element index.

The model instance is identified by a handle value assigned to the output parameter `p_model_handle`.
Use this handle when calling access layer API functions.

@note
A model can extend one or more other models. These parent model instances can be bound
to different elements, making the complete model span multiple elements.
These models are called extended models. Refer to the @tagMeshMdlSp of
the @link_btsig_spec for more information and examples.


---

## Step 3: Set up publication and subscription @anchor creating_models_publication_subscription

A mesh application is specified using a client-server architecture, where client and server
models use publish and subscribe mechanism to communicate with each other. For this reason,
you have to set up publication and subscription when using models in your application.

### Step 3.1: Set up publication @anchor creating_models_publication_subscription_1

The publication allows sending messages from models.

Each model has a publish address. The publication of messages can be periodic or one-shot,
and the published messages can be sent to a unicast, a group, or a virtual address.

The configuration of publication-related states is generally controlled by a provisioner
through the configuration model. The publication is useful for example for allowing sensor nodes
to periodically report data readings. A message can be published
by using the `access_model_publish()` API function, which will publish a message according
to the publication settings of the model (interval, destination).

The publication is also used by client models to send messages to server models.
However, in many cases the application wants to control the destination of messages
published from a client model instead of relying on an external provisioner (in many
cases, the application containing the client _is_ the provisioner). For this purpose,
the API function `access_model_publish_address_set()` is provided.

### Step 3.2: Set up subscription @anchor creating_models_publication_subscription_2

Subscriptions allow models to listen for incoming messages from specific addresses.
This feature can be used for example to listen to periodic messages published from sensor nodes.

To allow a model to subscribe to an address, you must first allocate a subscription
list with the `access_model_subscription_list_alloc()` API function.

@note
When using a client model, it is not required to subscribe to the address you
are sending messages to in order to receive replies to those messages. Subscriptions are
only used to receive unsolicited messages from nodes.


---

## Example @anchor creating_models_example

See @ref md_models_vendor_simple_on_off_README for the documentation of a basic Simple OnOff model
composed of a server model and a client model. The documentation includes detailed description
of how the model was implemented.
