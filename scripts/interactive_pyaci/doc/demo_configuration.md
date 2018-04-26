# Interactive mesh provisioning and configuration

## Requirements

Ensure that you have the following examples flashed on separate boards:

- Serial
- Light switch server
- (Optional) Light switch client

## Instructions

Start the interactive shell:

    $ python interactive_pyaci.py -d COM1 --no-logfile

        To control your device, use d[x], where x is the device index.
        Devices are indexed based on the order of the COM ports specified by the -d option.
        The first device, d[0], can also be accessed using device.

        Type d[x]. and hit tab to see the available methods.

    In  [1]:


### Loading the database

The first thing we do is to load the mesh network database:


    In [1]: db = MeshDB("database/example_database.json")


This will load the JSON-formatted data into useful Python objects that can be accessed like this:


    In  [2]: db.provisioners
    Out [2]: [{'allocated_group_range': [{'high_address': feff, 'low_address': c000}],
               'UUID': bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'),
               'name': 'BT Mesh Provisioner', 'allocated_unicast_range': [{'high_address': 7fff,
               'low_address': 0010}]}]


In the `mesh/scripts/interactive_pyaci/database` folder, there are two files:

- `example_database.json`
- `example_database.json.backup`

The second file is meant as a backup file if you want to clear a modified database.


## Provisioning

Next, we create a `Provisioner` object:


    In  [3]: p = Provisioner(device, db)
    2018-03-09 14:43:16,039 - INFO - COM1: Success
    2018-03-09 14:43:16,044 - INFO - COM1: SubnetAdd: {subnet_handle: 0}
    2018-03-09 14:43:16,048 - INFO - COM1: AppkeyAdd: {appkey_handle: 0}
    2018-03-09 14:43:16,052 - INFO - COM1: AppkeyAdd: {appkey_handle: 1}


This will set the local unicast address of the device and add the application and network keys
stored in the database. Take a note of the key handles returned, we will use them later when sending
messages to provisioned nodes.

After the provisioner has been created, we are ready to scan for unprovisioned devices.


    In  [4]: p.scan_start()
    2018-03-09 14:43:21,032 - INFO - COM1: Success
    2018-03-09 14:43:21,859 - INFO - COM1: Received UUID 0059ffff000000008007abda46190f5e with RSSI: -15 dB
    2018-03-09 14:43:27,947 - INFO - COM1: Success
    In  [5]: p.scan_stop()


As we can see, there is one device nearby that we can provision. This is the light switch server.


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


Again note the `devkey_handle` and `address_handle`.

## Configuration

We are now ready to configure the devices. First, let us create an instance of the
_Configuration Client_ model class, add
it to the device and set its publication state to match the first device.


    In  [7]: cc = ConfigurationClient(db)
    In  [8]: device.model_add(cc)
    In  [9]: cc.publish_set(8, 0)


The `8` and `0` is the device key and publish address handle, respectively.


### Composition data
Next, we will request the device's Composition Data<sup><a href="#fn:1">1</a></sup>. This will give us more
details about the device.


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
              "modelId": "00590000"
            }
          ],
          "index": 0
        }
      ],
      "vid": "0000"
    }


In the composition data, we can read that the node has one element with three models: Configuration
server (`0000`), Health server (`0002`) and the Simple OnOff server (`00590000`).

Notice that the `example_database.json` is updated automatically with the information from the node.


### Application keys

During provisioning, only network credentials, address(es) and a device key is assigned to the
device. To enable applications/models on nodes in the network to communicate, we will add an
application key to the device.

In the `example_database.json` we have already defined an application key called "lights" with key
index `0`. This key was added to the device by the provisioner earlier on with the `appkey_handle`
0<sup><a href="#fn:2">2</a></sup>.

For a model to receive and send messages with a given key, it has to be _bound_ to that model. We
will add AppKey `0` to the device and bind it to the Simple OnOff server.


    In  [11]: cc.appkey_add(0)
    2018-03-09 14:53:05,786 - INFO - COM1: Success
    2018-03-09 14:53:05,841 - INFO - COM1.ConfigurationClient: Appkey status: AccessStatus.SUCCESS
    2018-03-09 14:53:05,845 - INFO - COM1.ConfigurationClient: Appkey add 0 succeded for subnet 0 at node 0010

    In  [12]: cc.model_app_bind(db.nodes[0].unicast_address, 0, mt.ModelId(0, 0x59))
    2018-03-09 14:53:13,714 - INFO - COM1: Success
    2018-03-09 14:53:13,733 - INFO - COM1.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2018-03-09 14:53:13,751 - INFO - COM1.ConfigurationClient: Appkey bind 0 to model 00590000 at 0010


Notice that there are some helper objects defined in `mesh/types.py`, these can be through the `mt`
namespace, e.g., `mt.ModelId(model_id, company_id)`.

### Blinking LEDs

We have now configured our Simple OnOff server with the right security credentials, so let us blink
some LEDs!

To control the server, we need a client. A version of the Simple OnOff client has been implemented
in `models/simple_on_off.py`. We add and configure the model the same way as we did for the
Configuration Client:


    In  [13]: sc = SimpleOnOffClient()
    In  [14]: device.model_add(sc)
    In  [15]: sc.publish_set(0, 0)


Now, let us turn the LED on:


    In  [16]: sc.set(True)
    2018-03-09 15:01:49,230 - INFO - COM1.SimpleOnOffClient: Present value is on


### Adding a Light Switch client (optional)

Ensure that the Light Switch Client example is flashed and running. We will provision the device
exactly the same as we did for the server:


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


Again, notice the `devkey_handle` and `address_handle`. Set the publish state of the Configuration
Client accordingly:


    In  [20]: cc.publish_set(9, 1)


Next, we fetch the Composition Data:


    In  [21]: cc.composition_data_get()
    ...


In the Composition Data you will see that the device has 5 elements with 4 different Simple OnOff
client models (Model ID `00590001`). For the access layer<sup><a href="#fn:3">3</a></sup> to identify which
model a particular message is addressed to, can only be _one_ model _instance_ on any given element.

We add AppKey `0` and bind it to the Simple OnOff client model on Element 1.


    In  [22]: cc.appkey_add(0)
    ...
    In  [23]: cc.model_app_bind(db.nodes[1].unicast_address + 1, 0, mt.ModelId(1, 0x59))
    2018-03-12 14:13:14,618 - INFO - COM1.ConfigurationClient: Model app bind status: AccessStatus.SUCCESS
    2018-03-12 14:13:14,623 - INFO - COM1.ConfigurationClient: Appkey bind 0 to model 00590001 at 0012


Finally, we "link" the client to the server by setting its _publication
state_<sup><a href="#fn:4">4</a></sup> corresponding to element address of the Simple OnOff server, using
AppKey `0` and TTL `1`:


    In  [24]: cc.model_publication_set(db.nodes[1].unicast_address + 1, mt.ModelId(1, 0x59), mt.Publish(db.nodes[0].unicast_address, index=0, ttl=1))
    2018-03-12 15:08:48,071 - INFO - COM1.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2018-03-12 15:08:48,078 - INFO - COM1.ConfigurationClient: Publication status for model 00590001 at element 18 to {'period': 0, 'ttl': 1, 'address': 0010, 'retransmit': {'interval': 50, 'interval_steps': 0, 'count': 0}, 'index': 0, 'credentials': <FriendshipCredentials.DISABLED: 0>}


You should now be able to press "Button 1" on the development kit with the Light Switch client
example and see the "LED1" of the Light Switch server light up.

#### Publishing and subscribing

With both the client and server in the network, we can let the client _subscribe_ to state changes
from the server.

To do that, we ensure that we're talking to the Light switch server, and set its publication state
such that it can publish state changes to a pre-defined group address:


    In  [25]: cc.publish_set(8, 0)
    In  [26]: cc.model_publication_set(db.nodes[0].unicast_address, mt.ModelId(0, 0x59), mt.Publish(db.groups[0].address, index=0, ttl=1))
    2018-03-12 15:10:48,071 - INFO - COM1.ConfigurationClient: Model publication status: AccessStatus.SUCCESS
    2018-03-12 15:10:48,078 - INFO - COM1.ConfigurationClient: Publication status for model 00590000 at element 10 to {'period': 0, 'ttl': 1, 'address': 0010, 'retransmit': {'interval': 50, 'interval_steps': 0, 'count': 0}, 'index': 0, 'credentials': <FriendshipCredentials.DISABLED: 0>}


Next, we add the same group address to the client's _subscription list_:

    In  [27]: cc.publish_set(9, 1)
    In  [28]: cc.model_subscription_add(db.nodes[1].unicast_address + 1, db.groups[0].address, mt.ModelId(1, 0x59))
    2018-03-12 15:11:49,993 - INFO - COM1.ConfigurationClient: Model subscription status: AccessStatus.SUCCESS
    2018-03-12 15:11:49,996 - INFO - COM1.ConfigurationClient: Added subscription 'c001' to model 00590001 at element 18


You should now be able to use the Simple OnOff client to change the state of the server and observe
that the client also changes its state accordingly:


    In  [16]: sc.set(True)
    2018-03-09 15:12:49,230 - INFO - COM1.SimpleOnOffClient: Present value is on

---

<sup id="fn:1">1</sup> Section 4.2.1, <a href="https://www.bluetooth.com/specifications/mesh-specifications" target="_blank">Mesh Profile specification v1.0</a>.
<sup id="fn:2">2</sup> Key indexes are global to the network. Similarities to key _handles_ are coincidental.
<sup id="fn:3">3</sup> Section 3.7, <a href="https://www.bluetooth.com/specifications/mesh-specifications" target="_blank">Mesh Profile specification v1.0</a>.
<sup id="fn:4">4</sup> Section 4.2.2, <a href="https://www.bluetooth.com/specifications/mesh-specifications" target="_blank">Mesh Profile specification v1.0</a>.
