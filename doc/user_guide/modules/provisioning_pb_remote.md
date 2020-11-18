# Remote provisioning (PB-remote)

Remote provisioning (PB-remote) allows a provisioner to provision devices located outside
of the provisioner's radio range. This is done by using Bluetooth mesh nodes to relay provisioning
messages to a node within the range of the remote device that is to be provisioned.

![Remote provisioning topology](images/remote_provisioning.png)

The PB-Remote functionality is provided by the PB-remote client and the PB-remote Server models.

For more information about the PB-remote APIs, see
the [Remote provisioning APIs](@ref provisioning_pb-remote_apis) section
and the [PB-remote API group](@ref PB_REMOTE).

**Table of contents**
- [Remote provisioning process](@ref provisioning_pb-remote_process)
- [Remote provisioning APIs](@ref provisioning_pb-remote_apis)
    - [Client APIs](@ref provisioning_pb-remote_apis_client)
    - [Server APIs](@ref provisioning_pb-remote_apis_server)


---


## Remote provisioning process @anchor provisioning_pb-remote_process

The following figure shows the complete remote provisioning process that starts once the PB-remote
client and server are initialized and provisioned with the normal provisioning procedure,
with both of them in the idle state.

The provisioning process is made of the following steps:
- To find out remote unprovisioned devices, the application can request that the PB-remote client
instruct the PB-remote server to start scanning for unprovisioned devices by
calling @ref pb_remote_client_remote_scan_start().
- The PB-remote server starts scanning for the unprovisioned devices and reports
back the observed UUIDs.
- The application can then select the desired UUID and instruct the PB-remote client to start the
remote provisioning process by calling @ref nrf_mesh_prov_provision() API, with `bearer_type`
set to @ref NRF_MESH_PROV_BEARER_MESH.
- After this call, the PB-remote client requests that the PB-remote server open the provisioning link
with the unprovisioned device.
- Once the link is opened, the PB-remote client executes the rest of the provisioning process
by transferring provisioning PDUs to the server, which are then sent by the server
to the unprovisioned device. The PB-remote server also receives the PDUs
from the unprovisioned device and forwards them to the PB-remote client.

The process ends with successful provisioning of an unprovisioned device, and with the return of the
client and server devices to the idle state.

@mscfile pb_remote_provisioning.msc  Figure 1. Remote provisioning overview.


---

## Remote provisioning APIs @anchor provisioning_pb-remote_apis

This section outlines the provisioning process phases you need to take into account to implement
the PB-remote client and the PB-remote server in your application.

You can also check the [PB-remote example](@ref md_examples_pb_remote_README)
for implementation details.

### Client APIs @anchor provisioning_pb-remote_apis_client

The PB-remote client model provides a remote provisioning bearer and a set of APIs for performing
scanning using the PB-remote server.

To use the PB-remote client in your application, implement the following steps:
- Initialize the PB-remote client by calling `pb_remote_client_init()`.
- Add the remote provisioning bearer to the Bluetooth mesh stack:
    - Call @ref nrf_mesh_prov_bearer_add() and provide a statically-allocated provisioning context
    structure and the remote provisioning bearer reference. The reference is obtained by calling
    @ref pb_remote_client_bearer_interface_get().
- Start the remote scanning by calling @ref pb_remote_client_remote_scan_start(). After the scanning
starts, the PB-remote server will report the observed UUIDs to the client and the client will generate
the @ref PB_REMOTE_EVENT_REMOTE_UUID event.
    - The unprovisioned device's UUID is provided as an event parameter. Your application can store
    this information to use at later point in time.
- Stop the remote scanning by calling @ref pb_remote_client_remote_scan_cancel().
- Start the remote provisioning by calling @ref nrf_mesh_prov_provision() and provide the
 provisioning context, the pointer to UUID, and the attention duration.


### Server APIs @anchor provisioning_pb-remote_apis_server

The PB-remote Server model provides a remote provisioning bearer and a set of APIs for performing
scanning using the PB-remote server.

To use the PB-remote server in your application, implement the following steps:
- Initialize the PB-remote server by calling @ref pb_remote_server_init().
- Enable the server by calling @ref pb_remote_server_enable().
    - The application can disable the PB-remote server when it is not needed
    by calling @ref pb_remote_server_disable().
- Add the provisioning bearer to the PB-remote server:
    - Call @ref pb_remote_server_prov_bearer_set() and provide the PB-ADV provisioning bearer
    reference. The local reference is obtained by calling @ref nrf_mesh_prov_bearer_adv_interface_get().

The provisioning and scanning process is controlled by the peer PB-remote client and your
application does not need to take any other action.