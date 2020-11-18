# Provisioning

Provisioning is the process of providing new devices in the Bluetooth mesh network with the information
they need to join a network. To become a node and participate in the Bluetooth mesh communication,
each device must be provisioned.

From the provisioning perspective, a device can be of one of the following types:
- Provisioner - which assumes the Provisioner role.
- Provisionee - which assumes the Node role.

Only one provisioner is required within a network for provisioning purposes, though several
provisioners can be used.

As part of the provisioning process, the provisioner and the provisionee establish communication
on a provisioning bearer layer. The layer enables the transportation of Provisioning PDUs between
a provisioner and an unprovisioned device.

The nRF5 SDK for Mesh offers provisioning with one of the following two bearers:
- [PB-ADV](@ref provisioning_main_pb-adv)
- [PB-GATT](@ref provisioning_main_pb-gatt)

To perform provisioning of a device that is located outside of the provisioner's radio range,
you can use the @subpage md_doc_user_guide_modules_provisioning_pb_remote feature.

The APIs related to the provisioning process are independent of the underlaying bearers.
See @subpage md_doc_user_guide_modules_provisioning_implementing for information about how to use
provisionings APIs in the user application.

@par Provisioning bearers in the nRF5 SDK for Mesh examples
The nRF5 SDK for Mesh examples use by default one of the two provided bearers.
For more information, see the [Examples main page](@ref example_provisioning_bearers).

@link_btsig_glossary.

---

## Information sent during provisioning @anchor provisioning_information_sent

During the provisioning process, the new unprovisioned device receives the following elements:
- a unicast address of the primary element,
- a network key and the associated key index,
- IV index,
- IV Update Flag,
- Key Refresh Flag.

Additionally, a _device key_ is computed at the end of the provisioning process.
The device key is a special key only used for private communication
between the provisioner and the provisionee (for example, when the device is being configured
after provisioning). It is derived from a shared secret established between the devices using
an Elliptic Curve Diffie-Hellman (ECDH) protocol.

---

## PB-ADV bearer @anchor provisioning_main_pb-adv

This bearer is used to exchange the provisioning PDUs over the advertising channels. This
bearer can be natively supported by the Nordic BLE devices (acting as Provisioner or Provisionee),
and may be supported by the tablets or computers (acting as Provisioner).

It is recommended to always support this bearer, unless it not possible due to application-specific
constraints.

To enable this bearer, set @ref MESH_FEATURE_PB_ADV_ENABLED to `1`.

@note This bearer can be disabled if not required. Moreover, it is independent
of the advertising bearer (ADV) used for the Bluetooth mesh communication once the device
is provisioned.

This bearer is enabled by default for most of the examples in the nRF5 SDK for Mesh.
For more information, see the [Examples main page](@ref example_provisioning_bearers).

---

## PB-GATT bearer @anchor provisioning_main_pb-gatt

Many mobile computing devices and computers have limited support for sending Bluetooth mesh packets
on the advertisement channels in accordance with the @tagMeshSp. For these cases,
GATT-based bearers are defined.

The PB-GATT bearer is used to exchange the provisioning PDUs using Bluetooth mesh Provisioning Service.
This bearer can be enabled on the provisionee devices to provision them using mobile applications.

To enable this bearer, set the following defines to `1`:
- @ref MESH_FEATURE_PB_GATT_ENABLED
- @ref MESH_FEATURE_GATT_PROXY_ENABLED

@note
It is mandatory to support this bearer if the GATT bearer (Proxy Service) is supported by the device.
For more information, see @subpage md_doc_user_guide_modules_provisioning_gatt_proxy.

