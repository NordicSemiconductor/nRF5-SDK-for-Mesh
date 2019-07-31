# Interactive mesh provisioning and configuration

This tutorial describes how to use the serial interface to provision and configure a mesh network.

In this tutorial, a mesh node running a serial example is used as a provisioner and a light switch
server example is used as an unprovisioned device. PyACI is used to communicate with serial example
and carry out the provisioning and configuration of the unprovisioned device. After that,
a PyACI-based Generic OnOff client is used to communicate with the newly provisioned node
running the Generic OnOff server.

**Table of contents**
- [Requirements](@ref pyaci_demo_configuration_reqs)
- [Starting the shell and loading the database](@ref pyaci_demo_configuration_starting)
    - [Loading the database](@ref pyaci_demo_configuration_starting_database)
- [Provisioning](@ref pyaci_demo_configuration_starting_provisioning)
- [Configuring the devices](@ref pyaci_demo_configuration_starting_configuration)
    - [Completing initial configuration](@ref pyaci_demo_configuration_starting_configuration_1)
    - [Requesting composition data](@ref pyaci_demo_configuration_starting_configuration_2)
    - [Adding application keys](@ref pyaci_demo_configuration_starting_configuration_3)
    - [Adding Generic OnOff client](@ref pyaci_demo_configuration_starting_configuration_4)
    - [Adding a Light Switch client (optional)](@ref pyaci_demo_configuration_starting_configuration_5)
        - [Publishing and subscribing](@ref pyaci_demo_configuration_starting_configuration_5_publishing)
        

---

## Requirements @anchor pyaci_demo_configuration_reqs

Ensure that you have the following examples flashed on separate boards:
- @ref md_examples_serial_README
- @ref md_examples_light_switch_README
    - Light switch server is mandatory.
    - Light switch client is optional.

    
---
    
## Starting the shell and loading the database @anchor pyaci_demo_configuration_starting

To start the interactive shell, run the following command:
```
    $ python interactive_pyaci.py -d COM1 --no-logfile

        To control your device, use d[x], where x is the device index.
        Devices are indexed based on the order of the COM ports specified by the -d option.
        The first device, d[0], can also be accessed using device.

        Type d[x]. and hit tab to see the available methods.

    In  [1]:
```

### Loading the database @anchor pyaci_demo_configuration_starting_database

After the shell has been started, load the mesh network database:
```
    In [1]: db = MeshDB("database/example_database.json")
```

This loads the JSON-formatted data into useful Python objects that can be accessed
in the following manner:
```
    In  [2]: db.provisioners
    Out [2]: [{'allocated_group_range': [{'high_address': feff, 'low_address': c000}],
               'UUID': bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
               'name': 'BT Mesh Provisioner', 'allocated_unicast_range': [{'high_address': 7fff,
               'low_address': 0010}]}]
```

In the `mesh/scripts/interactive_pyaci/database` folder, there are two files:

- `example_database.json`
- `example_database.json.backup`

The second file is a backup file, if you want to clear a modified database.

---

## Provisioning @anchor pyaci_demo_configuration_starting_provisioning

Once the shell is running and the database is loaded:
1. Create a `Provisioner` object:
```
    In  [3]: p = Provisioner(device, db)
    2018-03-09 14:43:16,039 - INFO - COM1: Success
    2018-03-09 14:43:16,044 - INFO - COM1: SubnetAdd: {subnet_handle: 0}
    2018-03-09 14:43:16,048 - INFO - COM1: AppkeyAdd: {appkey_handle: 0}
    2018-03-09 14:43:16,052 - INFO - COM1: AppkeyAdd: {appkey_handle: 1}
```
This command sets the local unicast address of the device and adds the application and network keys
stored in the database.
2. Take a note of the key handles returned, as they will be used for sending
messages to provisioned nodes.
3. After the provisioner has been created, start scanning for unprovisioned devices:
```
    In  [4]: p.scan_start()
    2018-03-09 14:43:21,032 - INFO - COM1: Success
    2018-03-09 14:43:21,859 - INFO - COM1: Received UUID 0059ffff000000008007abda46190f5e with RSSI: -15 dB
    In  [5]: p.scan_stop()
    2018-03-09 14:43:27,947 - INFO - COM1: Success
```
As you can see, there is one device nearby that we can provision. This is the light switch server.
4. Provision the light switch server:
```
    In  [6]: p.provision(name="Light bulb")

    2018-03-09 14:43:36,669 - INFO - COM1: Provision: {'context': 0}
    2018-03-09 14:43:36,697 - INFO - COM1: Link established
    2018-03-09 14:43:36,743 - INFO - COM1: Received capabilities
    2018-03-09 14:43:36,744 - INFO - COM1: Number of elements: 1
    2018-03-09 14:43:36,748 - INFO - COM1: OobUse: {'context': 0}
    2018-03-09 14:43:36,898 - INFO - COM1: ECDH request received
    2018-03-09 14:43:36,908 - INFO - COM1: EcdhSecret: {'context': 0}
    2018-03-09 14:43:39,254 - INFO - COM1: Provisioning complete
    2018-03-09 14:43:39,255 - INFO - COM1:         Address(es): 0x10-0x10
    2018-03-09 14:43:39,256 - INFO - COM1:         Device key: 8c92f1b167110539c8990c5bf6606fbb
    2018-03-09 14:43:39,257 - INFO - COM1:         Network key: 18eed9c2a56add85049ffc3c59ad0e12
    2018-03-09 14:43:39,257 - INFO - COM1: Adding device key to subnet 0
    2018-03-09 14:43:39,259 - INFO - COM1: Adding publication address of root element
    2018-03-09 14:43:39,265 - INFO - COM1: DevkeyAdd: {'devkey_handle': 8}
    2018-03-09 14:43:39,267 - INFO - COM1: AddrPublicationAdd: {'address_handle': 0}
    2018-03-09 14:43:39,370 - INFO - COM1: Provisioning link closed
```
5. Write down the `devkey_handle` and `address_handle`.

You are now ready to configure the devices.

---

## Configuring the devices @anchor pyaci_demo_configuration_starting_configuration

To configure the devices, complete the following steps:
- [Completing initial configuration](@ref pyaci_demo_configuration_starting_configuration_1)
- [Requesting composition data](@ref pyaci_demo_configuration_starting_configuration_2)
- [Adding application keys](@ref pyaci_demo_configuration_starting_configuration_3)
- [Adding Generic OnOff client](@ref pyaci_demo_configuration_starting_configuration_4)

Optionally, you can also [add a Light Switch client](@ref pyaci_demo_configuration_starting_configuration_5)
and set up [publishing and subscribing](@ref pyaci_demo_configuration_starting_configuration_5_publishing).

### Completing initial configuration @anchor pyaci_demo_configuration_starting_configuration_1

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
The `8` and `0` are the device key and publish address handle, respectively.


### Requesting composition data @anchor pyaci_demo_configuration_starting_configuration_2
To request composition data and get more details about the device, run the following command:
```
    In  [10]: cc.composition_data_get()
    2018-03-09 15:16:02,430 - INFO - COM1: Success
    2018-03-09 15:16:02,514 - INFO - COM1.ConfigurationClient: Received composition data (page 0x00):
    {
      "crpl": 32,
      "cid": "0059",
      "features": {
        "low_power": 2,
        "friend": 2,
        "relay": 0,
        "proxy": 2
      },
      "pid": "0000",
      "elements": [
        {
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
          ],
          "index": 0
        }
      ],
      "vid": "0000"
    }
```

In the composition data, you can see that the node has one element with three models:
- Configuration server (`0000`)
- Health server (`0002`)
- Generic OnOff server (`1000`)

The `example_database.json` file is updated automatically with the information from the node.

For more details about composition data, see section 4.2.1 of the @link_MeshSpec (v1.0).

### Adding application keys @anchor pyaci_demo_configuration_starting_configuration_3

During provisioning, only network credentials, addresses and a device key is assigned to the
device. To enable applications and models on nodes in the network to communicate, add an
application key to the device.

In the `example_database.json` file, an application key called "lights" with key index `0` is defined.
This key was added to the device by the provisioner earlier on with the `appkey_handle`
0.

@note Key indexes are global to the network. Similarities to key _handles_ are coincidental.

For a model to receive and send messages with a given key, it has to be _bound_ to that model.
Add AppKey `0` to the device and bind it to the Generic OnOff server:
```
    In  [11]: cc.appkey_add(0)
    2018-03-09 14:53:05,786 - INFO - COM1: Success
    2018-03-09 14:53:05,841 - INFO - COM1.ConfigurationClient: Appkey status: AccessStatus.SUCCESS
    2018-03-09 14:53:05,845 - INFO - COM1.ConfigurationClient: Appkey add 0 succeded for subnet 0 at node 0010

    In  [12]: cc.model_app_bind(db.nodes[0].unicast_address, 0, mt.ModelId(0x1000))
    2018-03-09 14:53:13,714 - INFO - COM1: Success
    2018-03-09 14:53:13,733 - INFO - COM1.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2018-03-09 14:53:13,751 - INFO - COM1.ConfigurationClient: Appkey bind 0 to model 1000 at 0010
```

There are some helper objects defined in `mesh/types.py`. These can be accessed through the `mt`
namespace, for example `mt.ModelId(model_id, company_id=None)`.

### Adding Generic OnOff client @anchor pyaci_demo_configuration_starting_configuration_4

The Generic OnOff server is now configured with the right security credentials and you can start
sending messages to change the state of the LEDs on the server.

To control the server, you need a client. A version of the Generic OnOff client has been implemented
in `models/generic_on_off.py`. You can add and configure the model the same way as you did for the
Configuration Client:
```
    In  [13]: gc = GenericOnOffClient()
    In  [14]: device.model_add(gc)
    In  [15]: gc.publish_set(0, 0)
```

Now, you can turn the LED on:
```
    In  [16]: gc.set(True)
    2018-03-09 15:01:49,230 - INFO - COM1.GenericOnOffClient: Present value is on
```

### Adding a Light Switch client (optional) @anchor pyaci_demo_configuration_starting_configuration_5

To add a Light Switch client, ensure that its example is flashed and running.

To add the client:
1. Provision the device exactly the same as you did for the server:
```
    In  [17]: p.scan_start()
    2018-03-12 14:02:31,674 - INFO - COM1: Received UUID 0059abcdefabcdefaccdefabcdefabcd with RSSI: -15 dB
    In  [18]: p.scan_stop()
    In  [19]: p.provision(name="Light switch")
    ...
    2018-03-12 14:02:42,885 - INFO - COM1: Provisioning complete
    2018-03-12 14:02:42,885 - INFO - COM1:         Address(es): 0x11-0x15
    2018-03-12 14:02:42,886 - INFO - COM1:         Device key: 567b64c82c3f3901a659a37960a89451
    2018-03-12 14:02:42,886 - INFO - COM1:         Network key: 18eed9c2a56add85049ffc3c59ad0e12
    ...
    2018-03-12 14:02:42,906 - INFO - COM1: DevkeyAdd: {'devkey_handle': 9}
    2018-03-12 14:02:42,913 - INFO - COM1: AddrPublicationAdd: {'address_handle': 1}
    ...
```
2. Notice the `devkey_handle` and `address_handle`.
3. Set the publish state of the Configuration Client accordingly:
```
    In  [20]: cc.publish_set(9, 1)
```
4. Fetch the Composition Data:
```
    In  [21]: cc.composition_data_get()
    ...
```
In the Composition Data, you will see that the device has 5 elements with 4 different Generic OnOff
client models (Model ID `1001`). For the access layer to identify which
model a particular message is addressed to, there can only be _one_ model _instance_ on any given element.
@note For more information about the access layer, see section 3.7 in the @link_MeshSpec (v1.0).
5. Add AppKey `0` and bind it to the Generic OnOff client model on Element 1:
```
    In  [22]: cc.appkey_add(0)
    ...
    In  [23]: cc.model_app_bind(db.nodes[1].unicast_address + 1, 0, mt.ModelId(0x1001))
    2018-03-12 14:13:14,618 - INFO - COM1.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2018-03-12 14:13:14,623 - INFO - COM1.ConfigurationClient: Appkey bind 0 to model 1001 at 0012
```
6. Link the client to the server by setting its _publication
state_ corresponding to element address of the Generic OnOff server, using AppKey `0` and TTL `1`:
```
    In  [24]: cc.model_publication_set(db.nodes[1].unicast_address + 1, mt.ModelId(0x1001), mt.Publish(db.nodes[0].unicast_address, index=0, ttl=1))
    2018-03-12 15:08:48,071 - INFO - COM1.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2018-03-12 15:08:48,078 - INFO - COM1.ConfigurationClient: Publication status for model 1001 at element 18 to {'period': 0, 'ttl': 1, 'address': 0010, 'retransmit': {'interval': 50, 'interval_steps': 0, 'count': 0}, 'index': 0, 'credentials': <FriendshipCredentials.DISABLED: 0>}
```
You can now press Button 1 on the development kit with the Light Switch client
example and see the LED 1 of the Light Switch server light up.

#### Publishing and subscribing @anchor pyaci_demo_configuration_starting_configuration_5_publishing

With both the client and the server in the network, you can let the client _subscribe_ to state changes
from the server:
1. Ensure that you are talking to the Light switch server and set its publication state
so that it can publish state changes to a pre-defined group address:
```
    In  [25]: cc.publish_set(8, 0)
    In  [26]: cc.model_publication_set(db.nodes[0].unicast_address, mt.ModelId(0x1000), mt.Publish(db.groups[0].address, index=0, ttl=1))
    2018-03-12 15:10:48,071 - INFO - COM1.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2018-03-12 15:10:48,078 - INFO - COM1.ConfigurationClient: Publication status for model 1000 at element 10 to {'period': 0, 'ttl': 1, 'address': 0010, 'retransmit': {'interval': 50, 'interval_steps': 0, 'count': 0}, 'index': 0, 'credentials': <FriendshipCredentials.DISABLED: 0>}
```
2. Add the same group address to the client's _subscription list_:
```
    In  [27]: cc.publish_set(9, 1)
    In  [28]: cc.model_subscription_add(db.nodes[1].unicast_address + 1, db.groups[0].address, mt.ModelId(0x1001))
    2018-03-12 15:11:49,993 - INFO - COM1.ConfigurationClient: Model subscription status: AccessStatus.SUCCESS
    2018-03-12 15:11:49,996 - INFO - COM1.ConfigurationClient: Added subscription 'c001' to model 1001 at element 18
```

You should now be able to use the Generic OnOff client to change the state of the server and observe
that the client also changes its state accordingly:
```
    In  [16]: gc.set(True)
    2018-03-09 15:12:49,230 - INFO - COM1.GenericOnOffClient: Present value is on
```

