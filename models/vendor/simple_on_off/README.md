# Simple OnOff model

The Bluetooth mesh model specification specifies a Generic OnOff Model to be used in real applications
with the Bluetooth mesh. This vendor-specific model is a simplified version of the Generic OnOff Model.
It is an introductory example for @ref md_doc_user_guide_modules_models_creating, but you can also
use it in your applications.

See the following sections for information about how to implement a vendor-specific
Simple OnOff model that turns something On or Off, for example a light bulb, a heater,
or a washing machine.

@note
For brevity, some important features such as error handling are not discussed on this page.
When writing your application, check the error codes returned by all API functions
to avoid bugs in your application.

**Table of contents**
- [Properties and features](@ref simple_onoff_model_overview)
    - [Supported opcodes](@ref simple_onoff_model_overview_opcodes)
    - [Identifiers](@ref simple_onoff_model_overview_identifiers)
- [Implementing the model](@ref simple_onoff_model_implementing)
    - [Implementing the server model](@ref simple_onoff_model_implementing_server)
    - [Implementing the client model](@ref simple_onoff_model_implementing_client)

You can check the complete model implementation and its layout
in the `models/vendor/simple_on_off` directory.

If you want to see how this model can be integrated into a complete application, take a look at
the @ref md_examples_light_switch_README and at the `examples/light_switch` directory.

---

## Properties and features @anchor simple_onoff_model_overview

A Bluetooth mesh application is specified using a client-server architecture, where client and server
models use publish and subscribe mechanism to communicate with each other.
For this reason, the intended functionality of this model will be realized using two parts:
- the server model, used for maintaining the OnOff state;
- a client model, used for manipulating the OnOff state on the server.

When the server model receives a GET or a (reliable) SET message from a client model, it sends the
current value of the OnOff state as response. This keeps the client up-to-date about the server
state.

For more details about setting up publication and subscription,
see [Creating new models](@ref creating_models_publication_subscription).

### Supported opcodes @anchor simple_onoff_model_overview_opcodes

The following table shows the opcodes that are supported by this model.

| Name               | Definition                               | Opcode       | Description                   | Parameter     | Parameter size |
| ------------------ | -----------------------------------------| ------------:| ----------------------------- | ------------- | --------------:|
| SET                | `::SIMPLE_ON_OFF_OPCODE_SET`             |         0xc1 | Sets the current on/off state | New state     |         1 byte |
| GET                | `::SIMPLE_ON_OFF_OPCODE_GET`             |         0xc2 | Gets the current on/off state | N/A           |   No parameter |
| SET UNRELIABLE     | `::SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE`  |         0xc3 | Sets the current on/off state | New state     |         1 byte |
| Status             | `::SIMPLE_ON_OFF_OPCODE_STATUS`          |         0xc4 | Contains the current state    | Current state |         1 byte |

The opcodes sent on-air are three bytes for the vendor-specific models. The
complete opcode is the combination of the vendor-specific opcode and the company identifier.
For more information, see the `access_opcode_t` documentation.

### Identifiers @anchor simple_onoff_model_overview_identifiers

For this model, the following identifiers are used.

| Description        | Value     |
| ------------------ | ---------:|
| Company identifier |    0x0059 |
| Server identifier  |    0x0000 |
| Client identifier  |    0x0001 |

The company identifier used in this table is Nordic Semiconductor's assigned Bluetooth
company ID. In a real application, use your own company's assigned ID.

---

## Implementing the model @anchor simple_onoff_model_implementing

As described earlier, a model comprises of two entities that together implement
the complete behavior:
- Server model, which typically has states and exposes messages to control
value of these states and trigger behaviors.
- Client model, which sends the messages to control and observe the states on the server model.

Implement both the [server](@ref simple_onoff_model_implementing_server) and the [client](@ref simple_onoff_model_implementing_client) models
for the Simple OnOff model to work.

### Implementing the server model @anchor simple_onoff_model_implementing_server

The behavior of the simple OnOff server is illustrated by the following message sequence chart.

![Simple OnOff behavior](img/simple_on_off_model.png)

When the OnOff server receives SET and GET messages:
- It calls a callback function provided by the application.
- It shares or requests the data through callback function parameters.

To implement the server model:
-# Define a model context structure that contains pointers to the callback functions.
This context structure gets passed to all message handlers.
    - The following code snippet shows the context structure required
    for the server model (`simple_on_off_server_t`) and associated callbacks:
```C
/** Forward declaration. */
typedef struct __simple_on_off_server simple_on_off_server_t;

/**
 * Get callback type.
 * @param[in] p_self Pointer to the Simple OnOff Server context structure.
 * @returns @c true if the state is On, @c false otherwise.
 */
typedef bool (*simple_on_off_get_cb_t)(const simple_on_off_server_t * p_self);

/**
 * Set callback type.
 * @param[in] p_self Pointer to the Simple OnOff Server context structure.
 * @param[in] on_off Desired state
 * @returns @c true if the set operation was successful, @c false otherwise.
 */
typedef bool (*simple_on_off_set_cb_t)(const simple_on_off_server_t * p_self, bool on_off);

/** Simple OnOff Server state structure. */
struct __simple_on_off_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;
    /** Get callback. */
    simple_on_off_get_cb_t get_cb;
    /** Set callback. */
    simple_on_off_set_cb_t set_cb;
};
```
-# Define the opcodes and create the necessary opcode handler functions to handle
the incoming messages for the server model.
    - You need three opcode handlers in the server to handle `::SIMPLE_ON_OFF_OPCODE_GET`,
    `::SIMPLE_ON_OFF_OPCODE_SET`, and `::SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE` messages. Each of
    these opcode handlers calls the corresponding user callback function from the context
    structure. This context structure gets passed to the opcode handlers via the `p_args` parameter.
    - All opcode handlers of all the models must use the same function prototype:
```C
typedef void (*access_opcode_handler_cb_t)(access_model_handle_t handle,
                                           const access_message_rx_t * p_message,
                                           void * p_args);
```
    - The following snippet shows the opcode handlers defined for the Simple OnOff server model:
```C
static void handle_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->set_cb != NULL);

    bool value = (((simple_on_off_msg_set_t*) p_message->p_data)->on_off) > 0;
    value = p_server->set_cb(p_server, value);
    reply_status(p_server, p_message, value);
    (void) simple_on_off_server_status_publish(p_server, value); /* We don't care about status */
}

static void handle_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->get_cb != NULL);
    reply_status(p_server, p_message, p_server->get_cb(p_server));
}

static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->set_cb != NULL);
    bool value = (((simple_on_off_msg_set_unreliable_t*) p_message->p_data)->on_off) > 0;
    value = p_server->set_cb(p_server, value);
    (void) simple_on_off_server_status_publish(p_server, value);
}
```
-# Implement the `reply_status()` function.
    - As defined in section 3.7.5.2 of the @tagMeshSp,
    each receiving element acknowledges the received acknowledged message by responding to that message.
    The response is typically a status message.
    The status message usually contains the current value of the state set by the SET message.
    For this reason, the model uses the `set_cb()` callback to fetch the current OnOff state value
    from the user application and sends this value with the `reply_status()` function.
    The server model also publishes its state in response to any received message, using the
    `simple_on_off_server_status_publish()` function, if its publish address is set by the provisioner.
    - The `reply_status()` function sends the value of the current state
    in an `::SIMPLE_ON_OFF_OPCODE_STATUS` message as a reply to the client
    using the `access_model_reply()` API. This API requires certain parameters
    to send the message correctly, which is why it has been wrapped in `reply_status()`.
    - The following snippet shows the implementation of the `reply_status()` function:
```C
static void reply_status(const simple_on_off_server_t * p_server,
                         const access_message_rx_t * p_message,
                         bool present_on_off)
{
    simple_on_off_msg_status_t status;
    status.present_on_off = present_on_off ? 1 : 0;
    access_message_tx_t reply;
    reply.opcode.opcode = SIMPLE_ON_OFF_OPCODE_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reply.p_buffer = (const uint8_t *) &status;
    reply.length = sizeof(status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}
```
-# Implement the `simple_on_off_server_status_publish()` function.
    - The `simple_on_off_server_status_publish()` function is very similar to the `reply_status()` function, except it uses
    the `access_model_publish()` API to publish the response message.
    If the publish address of the client model is not configured by the provisioner, the
    `access_model_publish()` will not publish the given message.
-# Ensure the specified opcode and company ID are linked to the corresponding handler function
in the specified opcode handler lookup table.
    - This lookup table is given as an input parameter when registering the
    model with the access layer. Each entry in the table is of the `access_opcode_handler_t` type
    and consists of the opcode, vendor ID, and an opcode handler function pointer.
    - For the server model the lookup table is defined as follows:
```C
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_SET,            SIMPLE_ON_OFF_COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_GET,            SIMPLE_ON_OFF_COMPANY_ID), handle_get_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE, SIMPLE_ON_OFF_COMPANY_ID), handle_set_unreliable_cb}
};
```
-# Put the model together in an initialization function.
    - The initialization function must allocate and add the model to the access layer:
```C
uint32_t simple_on_off_server_init(simple_on_off_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL ||
        p_server->get_cb == NULL ||
        p_server->set_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = SIMPLE_ON_OFF_SERVER_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_server->model_handle);
}
```

You now have the basic skeleton of a simple OnOff server model, which can be expanded
or tweaked to produce more complex server models. See `models/vendor/simple_on_off/` for the
complete code of this model.

### Implementing the client model @anchor simple_onoff_model_implementing_client

The client model is used to interact with the corresponding server model. It sends
SET and GET messages and processes incoming status replies. The client model sends messages using
a publish mechanism. It uses assigned publication address as the destination for outgoing messages.

Just as in the server implementation, the client needs a context structure to keep
information about callbacks and its model handle. In addition, a boolean variable
is used to keep track of whether a transaction is currently active and to
prevent running multiple simultaneous transactions.

@note
In a Bluetooth mesh network, messages may be delivered out of order, or may not be delivered at all.
For this reason, a client must perform only one transaction at a time with its corresponding server.

The client model uses a callback function to provide information about the state of the server to
the user application. If the server does not reply within a given time frame, it will notify the
user application with the error code `::SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY`.

The following code snippet shows the status codes (`::simple_on_off_status_t`), the context
structure (`simple_on_off_client_t`), and associated callbacks required for this model:

```C
/** Simple OnOff status codes. */
typedef enum
{
    /** Received status ON from the server. */
    SIMPLE_ON_OFF_STATUS_ON,
    /** Received status OFF from the server. */
    SIMPLE_ON_OFF_STATUS_OFF,
    /** The server did not reply to a Simple OnOff Set/Get. */
    SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY,
    /** Simple OnOff Set/Get was cancelled. */
    SIMPLE_ON_OFF_STATUS_CANCELLED
} simple_on_off_status_t;

/** Forward declaration. */
typedef struct __simple_on_off_client simple_on_off_client_t;

/**
 * Simple OnOff status callback type.
 *
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 * @param[in] status The received status of the remote server.
 * @param[in] src    Element address of the remote server.
 */
typedef void (*simple_on_off_status_cb_t)(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src);

/**
 * Simple OnOff timeout callback type.
 *
 * @param[in] handle Model handle
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 */
typedef void (*simple_on_off_timeout_cb_t)(access_model_handle_t handle, void * p_self);

/** Simple OnOff Client state structure. */
struct __simple_on_off_client
{
    /** Model handle assigned to the client. */
    access_model_handle_t model_handle;
    /** Status callback called after status received from server. */
    simple_on_off_status_cb_t status_cb;
    /** Periodic timer timeout callback used for periodic publication. */
    simple_on_off_timeout_cb_t timeout_cb;
    /** Internal client state. */
    struct
    {
        bool reliable_transfer_active; /**< Variable used to determine if a transfer is currently active. */
        simple_on_off_msg_set_t data;  /**< Variable reflecting the data stored in the server. */
    } state;
};
```

To implement the client model:
-# Define the message type the client model will send by choosing one of the following:
    - an unreliable (unacknowledged) message;
        - Use the `access_model_publish()` API to send an unreliable message.
    - a reliable (acknowledged) message.
        - Use the `access_model_reliable_publish()` API to send a reliable message. This API guarantees
        delivery of a message by retransmitting it until a reply is received from the destination node
        or the transaction times out.
            - When the transaction finishes with or without response, a status callback function
            is called to notify the user application.
        - The following snippet shows the `reliable_status_cb()` callback and the `send_reliable_message()`
        function for the client model:
```C
static void reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    simple_on_off_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    p_client->state.reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            p_client->status_cb(p_client, SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY, NRF_MESH_ADDR_UNASSIGNED);
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            p_client->status_cb(p_client, SIMPLE_ON_OFF_STATUS_CANCELLED, NRF_MESH_ADDR_UNASSIGNED);
            break;
        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}

static uint32_t send_reliable_message(const simple_on_off_client_t * p_client,
                                      simple_on_off_opcode_t opcode,
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_client->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = opcode;
    reliable.message.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = SIMPLE_ON_OFF_OPCODE_STATUS;
    reliable.reply_opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    reliable.timeout = SIMPLE_ON_OFF_CLIENT_ACKED_TRANSACTION_TIMEOUT;
    reliable.status_cb = reliable_status_cb;

    return access_model_reliable_publish(&reliable);
}
```
-# Create the API functions for the user application to send GET and SET messages.
The following snippet defines these functions:
```C
uint32_t simple_on_off_client_set(simple_on_off_client_t * p_client, bool on_off)
{
    if (p_client == NULL || p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_client->state.data.on_off = on_off ? 1 : 0;
    p_client->state.data.tid = m_tid++;

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_ON_OFF_OPCODE_SET,
                                            (const uint8_t *)&p_client->state.data,
                                            sizeof(simple_on_off_msg_set_t));
    if (status == NRF_SUCCESS)
    {
        p_client->state.reliable_transfer_active = true;
    }
    return status;
}

uint32_t simple_on_off_client_set_unreliable(simple_on_off_client_t * p_client, bool on_off, uint8_t repeats)
{
    simple_on_off_msg_set_unreliable_t set_unreliable;
    set_unreliable.on_off = on_off ? 1 : 0;
    set_unreliable.tid = m_tid++;

    access_message_tx_t message;
    message.opcode.opcode = SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE;
    message.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    message.p_buffer = (const uint8_t*) &set_unreliable;
    message.length = sizeof(set_unreliable);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    uint32_t status = NRF_SUCCESS;
    for (uint8_t i = 0; i < repeats; ++i)
    {
        message.access_token = nrf_mesh_unique_token_get();
        status = access_model_publish(p_client->model_handle, &message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t simple_on_off_client_get(simple_on_off_client_t * p_client)
{
    if (p_client == NULL || p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_ON_OFF_OPCODE_GET,
                                            NULL,
                                            0);
    if (status == NRF_SUCCESS)
    {
        p_client->state.reliable_transfer_active = true;
    }
    return status;
}
```
-# Add an opcode handler for the `::SIMPLE_ON_OFF_OPCODE_STATUS` opcode to process the reply message.
    - All incoming messages, even when they are a reply to a message that was sent from the node,
    need an opcode handler to be processed.
    - The following snippet shows the opcode handler implementation and defines the opcode handler
    lookup table for the client model:
```C
static void handle_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    simple_on_off_msg_status_t * p_status =
        (simple_on_off_msg_status_t *) p_message->p_data;
    simple_on_off_status_t on_off_status = (p_status->present_on_off ?
                                              SIMPLE_ON_OFF_STATUS_ON : SIMPLE_ON_OFF_STATUS_OFF);
    p_client->status_cb(p_client, on_off_status, p_message->meta_data.src.value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{SIMPLE_ON_OFF_OPCODE_STATUS, SIMPLE_ON_OFF_COMPANY_ID}, handle_status_cb}
};
```
-# Provide callback for supporting periodic publication.
    - To support publishing features, a model must provide `publish_timeout_cb`. This callback
    will be called by the publish mechanism if the provisioner configures periodic publishing.
    - The following snippet shows the implementation of the periodic publishing callback. In this
    implementation, the client model's periodic publish timeout callback calls the user-specified
    callback.
```C
static void handle_publish_timeout(access_model_handle_t handle, void * p_args)
{
    simple_on_off_client_t * p_client = p_args;

    if (p_client->timeout_cb != NULL)
    {
        p_client->timeout_cb(handle, p_args);
    }
}
```
-# Initialize the client model.
    - The initialization is the same as for the server model:
```C
uint32_t simple_on_off_client_init(simple_on_off_client_t * p_client, uint16_t element_index)
{
    if (p_client == NULL ||
        p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.model_id.model_id = SIMPLE_ON_OFF_CLIENT_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    init_params.element_index = element_index;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_client->model_handle);
}
```

The client model is now implemented. You can now use it to turn something On or Off by communicating
with the server node.
