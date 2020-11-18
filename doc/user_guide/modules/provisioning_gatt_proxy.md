# GATT provisioning and Proxy

Provisioning over GATT and the related Proxy protocol are optional features that
allow an unprovisioned device to be provisioned and controlled by a device
that cannot support the advertising bearer.

**Table of contents**
- [PB-GATT provisioning sequence](@ref pb-gatt_sequence)
    - [Advertising and provisioning](@ref pb-gatt_sequence_advertising)
    - [Resetting GATT database](@ref pb-gatt_sequence_resetting)
- [PB-GATT provisioning support in the nRF5 SDK for Mesh](@ref pb-gatt_support)

Provisioning over GATT and the Proxy protocol are mainly used by devices such as mobile phones
or tablets to:
- provision a new Bluetooth mesh device;
- communicate with the rest of the Bluetooth mesh network through a proxy device.

They can also be used to realize low power devices by excluding the PB-ADV
support, as shown in @ref md_examples_lpn_README.

The GATT provisioning bearer (PB-GATT) provisions a device using Proxy PDUs, which are exchanged
between the two roles of the Proxy protocol:
- Proxy Server -- a node that supports at least two different Bluetooth mesh bearers
- Proxy Client -- a node that supports a Bluetooth mesh bearer using the Proxy protocol.

The PDU size is determined based on the ATT_MTU. A single Proxy PDU can contain
a full message or segments of the message.

For more information about provisioning over GATT, see @link_MeshSpec, Section 5.2.2, page 229.
For more information about the Proxy protocol and the PDUs, see @link_MeshSpec, Chapter 6, page 260.

@note
- A device does not need to support both the GATT provisioning bearer (PB-GATT) and the Proxy feature,
but in the @ref md_examples_light_switch_README applications, both are supported by default.
- The GATT provisioning is used in @ref md_examples_lpn_README.
- You can control the GATT provisioning bearer through an app, for example @link_nrf_mesh_app.


---

## PB-GATT provisioning sequence @anchor pb-gatt_sequence

The following diagram shows how a device is provisioned over GATT and how it then transitions
to being a GATT Proxy server.

![GATT provisioning and Proxy server behavior](images/gatt_proxy.svg)

### Advertising and provisioning @anchor pb-gatt_sequence_advertising

The device first advertises both non-connectable and connectable Unprovisioned Device beacons:
- The non-connectable beacon is used for PB-ADV.
- The connectable beacon is used for PB-GATT and has a slightly different format, as it also includes the Bluetooth mesh
Provisioning Service UUID. (For more information, see @link_MeshSpec, Section 7.1.2.2.1, page 271.)

PB-GATT uses the interface provided in `mesh_adv.h` for advertising. Immediately after
the provisioner has connected to the device, it stops advertising both beacons.
The provisioning process starts.

### Resetting GATT database @anchor pb-gatt_sequence_resetting

When the provisioning process is complete, the device must reset its GATT database to comply with
the Bluetooth mesh specification requirement of exposing only the Bluetooth mesh Proxy service.

The reset is done by the helper module `mesh_provisionee.c`. To reset the database, the SoftDevice
is disabled and re-enabled. This means that any application service or setting needs to be
re-initialized after provisioning. This is done in the `prov_complete_cb()` callback from
`mesh_provisionee.h`.

For example, in the @ref md_examples_light_switch_README "Light switch server example":

```c
static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
    // ...
}
```

---

## PB-GATT provisioning support in the nRF5 SDK for Mesh @anchor pb-gatt_support

You can use the following settings to control the support for PB-GATT and GATT-PROXY functionalities
in nRF5 SDK for Mesh:
- @ref MESH_FEATURE_PB_GATT_ENABLED -- Enables provisioning over GATT.
- @ref MESH_FEATURE_GATT_PROXY_ENABLED -- Enables the GATT Proxy feature.

@note
Only the Proxy Server feature is available. The Proxy Client feature is not implemented.

