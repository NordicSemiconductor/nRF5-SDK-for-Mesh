# Using serial interface for provisioning and configuration

This tutorial describes how to use the serial interface to provision and configure a mesh network.

In this tutorial, a mesh node running a serial example is used as a provisioner and a light switch
server example is used as an unprovisioned device. PyACI is used to communicate
with the serial example and carry out the provisioning and configuration of the unprovisioned device.
After that, a PyACI-based Generic OnOff client is used to communicate with the newly provisioned node
that is running the Generic OnOff server.

**Table of contents**
- [Requirements](@ref pyaci_demo_configuration_reqs)
- [Starting the shell and loading the database](@ref pyaci_demo_configuration_starting)
- [Provisioning the devices](@ref pyaci_demo_configuration_starting_provisioning)
- [Configuring the devices](@ref pyaci_demo_configuration_starting_configuration)
    - [Adding Configuration Client model](@ref pyaci_demo_configuration_starting_configuration_1)
    - [Requesting composition data](@ref pyaci_demo_configuration_starting_configuration_2)
    - [Adding application keys](@ref pyaci_demo_configuration_starting_configuration_3)
    - [Using Generic OnOff client in PyACI](@ref pyaci_demo_configuration_starting_configuration_4)
    - [Adding a Light Switch client (optional)](@ref pyaci_demo_configuration_starting_configuration_5)
        - [Publishing and subscribing](@ref pyaci_demo_configuration_starting_configuration_5_publishing)
        

---

## Requirements @anchor pyaci_demo_configuration_reqs

You need at least two compatible development kits for this demonstration,
with the following examples [flashed](@ref examples_how_to_run_examples) on separate development kits:
- @ref md_examples_serial_README
- @ref md_examples_light_switch_README
    - Light switch server is mandatory.
    - Light switch client is optional.

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.
    
---
    
## Starting the shell and loading the database @anchor pyaci_demo_configuration_starting

Complete the following steps:
-# To start the interactive shell, run the following command:
```
    $ python interactive_pyaci.py -d COM1 --no-logfile

        To control your device, use d[x], where x is the device index.
        Devices are indexed based on the order of the COM ports specified by the -d option.
        The first device, d[0], can also be accessed using device.

        Type d[x]. and hit tab to see the available methods.

    In  [1]:
```
-# After the shell has been started, load the mesh network database by running the following command:
```
    In [1]: db = MeshDB("database/example_database.json")
```

This loads the JSON-formatted mesh network data and transforms this data into useful Python objects.
This database is used to store information about newly provisioned devices.
The Python objects can be accessed in the following manner:
```
    In  [2]: db.provisioners
    Out [2]: [{'name': 'BT Mesh Provisioner', 'UUID': _UUID(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'), 'allocated_unicast_range': [{'low_address': 0010, 'high_address': 7fff}], 'allocated_group_range': [{'low_address': c000, 'high_address': feff}]}]
```

The `mesh/scripts/interactive_pyaci/database` folder contains both the database file and its backup:

- `example_database.json`
- `example_database.json.backup`

You can use the backup file if you want to clear a modified database.

---

## Provisioning the devices @anchor pyaci_demo_configuration_starting_provisioning

When the shell is running and the database is loaded, complete the following steps:
1. Create a Provisioner object:
```
    In  [3]: p = Provisioner(device, db)
    2020-03-24 14:44:59,068 - INFO - COM27: Success
    2020-03-24 14:44:59,068 - INFO - COM27: Success
    2020-03-24 14:44:59,072 - INFO - COM27: SubnetAdd: {'subnet_handle': 0}
    2020-03-24 14:44:59,075 - INFO - COM27: AppkeyAdd: {'appkey_handle': 0}
    2020-03-24 14:44:59,077 - INFO - COM27: AppkeyAdd: {'appkey_handle': 1}
```
This command sets the local unicast address of the device and adds the application and network keys
stored in the database.
2. Take a note of the key handles returned, as they will be used for sending
messages to provisioned nodes.
3. After the provisioner has been created, start scanning for unprovisioned devices:
```
    In  [4]: p.scan_start()
    2020-03-24 14:45:13,345 - INFO - COM27: Success
    2020-03-24 14:45:14,210 - INFO - COM27: Received UUID eb0e9fd2713b9044899186208206ffff with RSSI: -48 dB
    In  [5]: p.scan_stop()
    2020-03-24 14:45:18,893 - INFO - COM27: Success
```
As you can see, there is one device nearby that we can provision. This is the light switch server.
4. Provision the light switch server:
```
    In  [6]: p.provision(name="Light bulb")

    2020-03-24 14:45:46,446 - INFO - COM27: Provision: {'context': 0}
    2020-03-24 14:45:46,540 - INFO - COM27: Link established
    2020-03-24 14:45:46,584 - INFO - COM27: Received capabilities
    2020-03-24 14:45:46,584 - INFO - COM27: Number of elements: 1
    2020-03-24 14:45:46,586 - INFO - COM27: OobUse: {'context': 0}
    2020-03-24 14:45:48,886 - INFO - COM27: ECDH request received
    2020-03-24 14:45:48,894 - INFO - COM27: EcdhSecret: {'context': 0}
    2020-03-24 14:45:49,330 - INFO - COM27: Provisioning complete
    2020-03-24 14:45:49,330 - INFO - COM27:         Address(es): 0x10-0x10
    2020-03-24 14:45:49,330 - INFO - COM27:         Device key: 20b7c58e83423ebfd3963c8415877b38
    2020-03-24 14:45:49,330 - INFO - COM27:         Network key: 18eed9c2a56add85049ffc3c59ad0e12
    2020-03-24 14:45:49,330 - INFO - COM27: Adding device key to subnet 0
    2020-03-24 14:45:49,331 - INFO - COM27: Adding publication address of root element
    2020-03-24 14:45:49,335 - INFO - COM27: DevkeyAdd: {'devkey_handle': 8}
    2020-03-24 14:45:49,336 - INFO - COM27: AddrPublicationAdd: {'address_handle': 0}
    2020-03-24 14:45:49,504 - INFO - COM27: Provisioning link closed
```
5. Write down the values for `devkey_handle` and `address_handle`.

You are now ready to configure the devices.

---

## Configuring the devices @anchor pyaci_demo_configuration_starting_configuration

To configure the devices, complete the following steps:
- [Adding Configuration Client model](@ref pyaci_demo_configuration_starting_configuration_1)
- [Requesting composition data](@ref pyaci_demo_configuration_starting_configuration_2)
- [Adding application keys](@ref pyaci_demo_configuration_starting_configuration_3)
- [Using Generic OnOff client in PyACI](@ref pyaci_demo_configuration_starting_configuration_4)

Optionally, you can also [add a Light Switch client](@ref pyaci_demo_configuration_starting_configuration_5)
and set up [publishing and subscribing](@ref pyaci_demo_configuration_starting_configuration_5_publishing).

### Adding Configuration Client model @anchor pyaci_demo_configuration_starting_configuration_1

The initial configuration includes loading the database and provisioning the devices.
To be able to configure mesh network, this initial configuration needs to be completed
by adding the Configuration Client model.

Complete the following steps:

1. Create an instance of the _Configuration Client_ model class:
```
    In  [7]: cc = ConfigurationClient(db)
```
2. Add the instance to the device:
```
    In  [8]: device.model_add(cc)
``` 
3. Set its publication state to match the first device:
```    
    In  [9]: cc.publish_set(8, 0)
```
The `8` and `0` are the device key and publish address handle, respectively,
written down at the end of the [provisioning](@ref pyaci_demo_configuration_starting_provisioning) procedure.


### Requesting composition data @anchor pyaci_demo_configuration_starting_configuration_2

The composition data allows you to get more details about the device.

To request composition data, run the following command:
```
    In  [10]: cc.composition_data_get()
    2020-03-24 14:47:10,873 - INFO - COM27: PacketSend: {'token': 1}
    2020-03-24 14:47:10,904 - INFO - COM27: {event: MeshTxComplete, data: {'token': 1}}
    2020-03-24 14:47:10,986 - INFO - COM27.ConfigurationClient: Received composition data (page 0x00): {
      "cid": "0059",
      "pid": "0000",
      "vid": "0000",
      "crpl": 40,
      "features": {
        "relay": 0,
        "proxy": 0,
        "friend": 0,
        "low_power": 2
      },
      "elements": [
        {
          "index": 0,
          "location": "0000",
          "models": [
            {
              "modelId": "0000"
            },
            {
              "modelId": "0002"
            },
            {
              "modelId": "1000"
            }
          ]
        }
      ]
    }
```

In the composition data, you can see that the node has one element with three models:
- Configuration server (`0000`)
- Health server (`0002`)
- Generic OnOff server (`1000`)

The `example_database.json` file is updated automatically with the information from the node.

For more details about the composition data, see section 4.2.1 of the @link_MeshSpec (v1.0).

### Adding application keys @anchor pyaci_demo_configuration_starting_configuration_3

During provisioning, only network credentials, addresses, and a device key are assigned to the
device. To enable applications and models on nodes in the network to communicate, add an
application key to the device and bind it to a model, for the model to receive and send messages
with the given key.

Add AppKey `0` to the device and bind it to the Generic OnOff server by running the following commands:
```
    In  [11]: cc.appkey_add(0)
    2020-03-24 14:49:11,687 - INFO - COM27: PacketSend: {'token': 2}
    2020-03-24 14:49:11,762 - INFO - COM27: {event: MeshTxComplete, data: {'token': 2}}
    2020-03-24 14:49:11,788 - INFO - COM27.ConfigurationClient: Appkey status: AccessStatus.SUCCESS
    2020-03-24 14:49:11,789 - INFO - COM27.ConfigurationClient: Appkey add 0 succeded for subnet 0 at node 0010

    In  [12]: cc.model_app_bind(db.nodes[0].unicast_address, 0, mt.ModelId(0x1000))
    2020-03-24 14:50:45,806 - INFO - COM27: PacketSend: {'token': 3}
    2020-03-24 14:50:45,831 - INFO - COM27: {event: MeshTxComplete, data: {'token': 3}}
    2020-03-24 14:50:45,861 - INFO - COM27.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2020-03-24 14:50:45,862 - INFO - COM27.ConfigurationClient: Appkey bind 0 to model 1000 at 0010
```

In the `example_database.json` file, you can find the application key "lights" with key index `0`.
This key was added to the device by the provisioner
during [provisioning](@ref pyaci_demo_configuration_starting_provisioning), with the `appkey_handle`
0.

@note Key indexes are global to the network. Similarities to key _handles_ are coincidental.

There are some helper objects defined in `mesh/types.py`. These can be accessed through the `mt`
namespace, for example `mt.ModelId(model_id, company_id=None)`.

### Using Generic OnOff client in PyACI @anchor pyaci_demo_configuration_starting_configuration_4

After completing all the previous configuration steps, the Generic OnOff server is configured
with the right security credentials and you can start sending messages to change
the state of the LEDs on the server.

To control the server, you need a client. A version of the Generic OnOff client has been implemented
in `models/generic_on_off.py`. You can add and configure the model the same way 
as you did for the [Configuration Client](@ref pyaci_demo_configuration_starting_configuration_1):
```
    In  [13]: gc = GenericOnOffClient()
    In  [14]: device.model_add(gc)
    In  [15]: gc.publish_set(0, 0)
```

At this point, you can turn the LED on by running the following command:
```
    In  [16]: gc.set(True)
    2020-03-24 14:51:37,274 - INFO - COM27: PacketSend: {'token': 4}
    2020-03-24 14:51:37,303 - INFO - COM27: {event: MeshTxComplete, data: {'token': 4}}
    2020-03-24 14:51:37,329 - INFO - COM27.GenericOnOffClient: Present OnOff: on
```

### Adding a Light Switch client (optional) @anchor pyaci_demo_configuration_starting_configuration_5

Adding a Light Switch client is optional and [requires an additional board](@ref pyaci_demo_configuration_reqs).
You do not need the Light Switch client, but using it allows you to configure the client using PyACI
to communicate with Generic OnOff Server.

To add the client:
-# Ensure that its example is [flashed and running](@ref examples_how_to_run_examples).
-# Provision the device exactly the same as you did for the server:
```
    In  [17]: p.scan_start()
    2020-03-24 14:52:14,942 - INFO - COM27: Received UUID 0d09dd57f2a1b8408b9186208206ffff with RSSI: -45 dB
    In  [18]: p.scan_stop()
    In  [19]: p.provision(name="Light switch")
    ...
    2020-03-24 14:52:30,615 - INFO - COM27: Provisioning complete
    2020-03-24 14:52:30,616 - INFO - COM27:         Address(es): 0x11-0x13
    2020-03-24 14:52:30,616 - INFO - COM27:         Device key: 0bd8f0952c807a30a79eefa4ba0f632e
    2020-03-24 14:52:30,616 - INFO - COM27:         Network key: 18eed9c2a56add85049ffc3c59ad0e12
    ...
    2020-03-24 14:52:30,623 - INFO - COM27: DevkeyAdd: {'devkey_handle': 9}
    2020-03-24 14:52:30,623 - INFO - COM27: AddrPublicationAdd: {'address_handle': 1}
    ...
```
-# Notice the `devkey_handle` and `address_handle`.
-# Set the publish state of the Configuration Client using the both handle values accordingly:
```
    In  [20]: cc.publish_set(9, 1)
```
-# Fetch the Composition Data:
```
    In  [21]: cc.composition_data_get()
    ...
```
In the Composition Data, you will see that the device has 5 elements with 4 different Generic OnOff
client models (Model ID `1001`). For the access layer to identify which
model a particular message is addressed to, there can only be _one_ model _instance_ on any given element.
@note For more information about the access layer, see section 3.7 in the @link_MeshSpec (v1.0).
-# Add AppKey `0` and bind it to the Generic OnOff client model on Element 1:
```
    In  [22]: cc.appkey_add(0)
    ...
    In  [23]: cc.model_app_bind(db.nodes[1].unicast_address + 1, 0, mt.ModelId(0x1001))
    2020-03-24 14:53:02,864 - INFO - COM27: PacketSend: {'token': 7}
    2020-03-24 14:53:02,887 - INFO - COM27: {event: MeshTxComplete, data: {'token': 7}}
    2020-03-24 14:53:02,915 - INFO - COM27.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2020-03-24 14:53:02,917 - INFO - COM27.ConfigurationClient: Appkey bind 0 to model 1001 at 0012
```
-# Link the client to the server by setting its _publication
state_ that corresponds to the element address of the Generic OnOff server (using AppKey `0` and TTL `1`):
```
    In  [24]: cc.model_publication_set(db.nodes[1].unicast_address + 1, mt.ModelId(0x1001), mt.Publish(db.nodes[0].unicast_address, index=0, ttl=1))
    2020-03-24 14:53:12,863 - INFO - COM27: PacketSend: {'token': 8}
    2020-03-24 14:53:14,413 - INFO - COM27: {event: MeshTxComplete, data: {'token': 8}}
    2020-03-24 14:53:14,471 - INFO - COM27.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2020-03-24 14:53:14,474 - INFO - COM27.ConfigurationClient: Publication status for model 1001 at element 18 to {'address': 0010, 'index': 0, 'ttl': 1, 'period': 0, 'retransmit': {'count': 0, 'interval_steps': 0, 'interval': 50}, 'credentials': <FriendshipCredentials.DISABLED: 0>}
```
You can now press Button 1 on the development kit with the Light Switch client
example and see the LED 1 of the Light Switch server light up.

#### Publishing and subscribing @anchor pyaci_demo_configuration_starting_configuration_5_publishing

With both the client and the server in the network, you can let the client _subscribe_ to state changes
from the server:
1. Ensure that you are talking to the Light switch server and set its publication state,
so that it can publish state changes to a pre-defined group address:
```
    In  [25]: cc.publish_set(8, 0)
    In  [26]: cc.model_publication_set(db.nodes[0].unicast_address, mt.ModelId(0x1000), mt.Publish(db.groups[0].address, index=0, ttl=1))
    2020-03-24 14:53:41,227 - INFO - COM27: PacketSend: {'token': 9}
    2020-03-24 14:53:41,326 - INFO - COM27: {event: MeshTxComplete, data: {'token': 9}}
    2020-03-24 14:53:41,372 - INFO - COM27.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2020-03-24 14:53:41,375 - INFO - COM27.ConfigurationClient: Publication status for model 1000 at element 16 to {'address': c001, 'index': 0, 'ttl': 1, 'period': 0, 'retransmit': {'count': 0, 'interval_steps': 0, 'interval': 50}, 'credentials': <FriendshipCredentials.DISABLED: 0>}
```
2. Add the same group address to the client's _subscription list_:
```
    In  [27]: cc.publish_set(9, 1)
    In  [28]: cc.model_subscription_add(db.nodes[1].unicast_address + 1, db.groups[0].address, mt.ModelId(0x1001))
    2020-03-24 14:53:54,685 - INFO - COM27: PacketSend: {'token': 10}
    2020-03-24 14:53:54,711 - INFO - COM27: {event: MeshTxComplete, data: {'token': 10}}
    2020-03-24 14:53:54,736 - INFO - COM27.ConfigurationClient: Model subscription status: AccessStatus.SUCCESS
    2020-03-24 14:53:54,738 - INFO - COM27.ConfigurationClient: Added subscription 'c001' to model 1001 at element 0012
```

You should now be able to use the Generic OnOff client to change the state of the server and observe
that the client also changes its state accordingly:
```
    In  [16]: gc.set(True)
    2020-03-24 14:54:06,652 - INFO - COM27: PacketSend: {'token': 11}
    2020-03-24 14:54:06,673 - INFO - COM27: {event: MeshTxComplete, data: {'token': 11}}
    2020-03-24 14:54:06,698 - INFO - COM27.GenericOnOffClient: Present OnOff: on
```

