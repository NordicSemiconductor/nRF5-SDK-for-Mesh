# Enabling provisioning

Provisioning is the process of providing new devices in the mesh network with the information they need to join
a network.

To participate in mesh communication, each device must be provisioned.
 Through the provisioning process, the new device receives:
- a unicast address of the primary element,
- a network key and the associated key index, 
- a device key,
- IV index,
- IV Update Flag,
- Key Refresh Flag.

The nRF5 SDK for Mesh offers two provisioning modes:
- Standard provisioning, described in @subpage md_doc_getting_started_provisioning
- Optional provisioning over GATT, described in @subpage md_doc_getting_started_gatt_proxy