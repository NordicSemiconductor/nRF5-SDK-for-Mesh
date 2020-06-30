# Sending mesh packets between devices

This tutorial shows how you can use two devices that are running the serial example
to send packets between them over mesh.

**Table of contents**
- [Requirements](@ref pyaci_demo_sending_packets_requirements)
- [Configuration](@ref pyaci_demo_sending_packets_configuration)
- [Testing](@ref pyaci_demo_sending_packets_testing)


---

## Requirements @anchor pyaci_demo_sending_packets_requirements

You need at least two compatible development kits for this demonstration,
[flashed](@ref examples_how_to_run_examples) with the following example:
- @ref md_examples_serial_README

See @ref md_doc_user_guide_mesh_compatibility for information about the compatible development kits.

---

## Configuration @anchor pyaci_demo_sending_packets_configuration

After building and programming the serial example, complete the following configuration steps:
1. Set up a minimal mesh network by connecting two devices and starting the serial interface:
```
    $ python interactive_pyaci.py -d /dev/ttyACM0 /dev/ttyACM1 --no-logfile
```
2. Add a network key, application key, and a local unicast address with the
`quick_setup()` method:
```
    In [1]: for dev in d: dev.quick_setup()
    2020-03-24 13:05:17,250 - INFO - ttyACM0: SubnetAdd: {'subnet_handle': 0}
    2020-03-24 13:05:17,251 - INFO - ttyACM1: SubnetAdd: {'subnet_handle': 0}
    2020-03-24 13:05:17,253 - INFO - ttyACM0: AppkeyAdd: {'appkey_handle': 0}
    2020-03-24 13:05:17,255 - INFO - ttyACM1: AppkeyAdd: {'appkey_handle': 0}
    2020-03-24 13:05:17,256 - INFO - ttyACM1: Success
    2020-03-24 13:05:17,256 - INFO - ttyACM0: Success
```
3. Disable the event filter that hides the raw messages:
```
    In [2]: d[0].event_filter_disable()
    In [3]: d[1].event_filter_disable()
```
4. Add publish addresses:
```
    In [4]: d[0].send(cmd.AddrPublicationAdd(d[1].local_unicast_address_start))
    2020-03-24 13:05:25,471 - INFO - ttyACM0: AddrPublicationAdd: {'address_handle': 0}
    In [5]: d[1].send(cmd.AddrPublicationAdd(d[0].local_unicast_address_start))
    2020-03-24 13:05:30,266 - INFO - ttyACM1: AddrPublicationAdd: {'address_handle': 0}  
```
The handles are used to reference the addresses and keys at a later point.
5. Store the handles in variables:
```
    In [6]: publish_handle = 0
    In [7]: appkey_handle = 0
```
6. Define some variables for the following parameters of `PacketSend()` API:
```
    In [8]: ttl = 1
    In [9]: segmented = 0
    In [10]: mic_size = 0
    In [11]: friendship_credentials_flag = 0
```
It is possible to use these variables while sending packets between the devices.


### Testing @anchor pyaci_demo_sending_packets_testing

To test sending a message between the devices, run the following commands:
```
    In [12]: d[0].send(cmd.PacketSend(appkey_handle, d[0].local_unicast_address_start, publish_handle, ttl, segmented, mic_size, friendship_credentials_flag, "Hello World"))
    2020-03-24 13:07:15,410 - INFO - ttyACM0: PacketSend: {'token': 2}
    2020-03-24 13:07:15,436 - INFO - ttyACM0: {event: MeshTxComplete, data: {'token': 2}}
    2020-03-24 13:07:15,439 - INFO - ttyACM1: {event: MeshMessageReceivedUnicast, data: {'src': 1, 'dst': 2, 'appkey_handle': 0, 'subnet_handle': 0, 'ttl': 1, 'adv_addr_type': 0, 'adv_addr': 'ff3941348306', 'rssi': -24, 'actual_length': 11, 'data': 'Hello World'}}

    In [13]: d[1].send(cmd.PacketSend(appkey_handle, d[1].local_unicast_address_start, publish_handle, ttl, segmented, mic_size, friendship_credentials_flag, "Hi there"))
    2020-03-24 13:12:03,133 - INFO - ttyACM1: PacketSend: {'token': 2}
    2020-03-24 13:12:03,160 - INFO - ttyACM1: {event: MeshTxComplete, data: {'token': 2}}
    2020-03-24 13:12:03,162 - INFO - ttyACM0: {event: MeshMessageReceivedUnicast, data: {'src': 2, 'dst': 1, 'appkey_handle': 0, 'subnet_handle': 0, 'ttl': 1, 'adv_addr_type': 1, 'adv_addr': '46f62b8cdde1', 'rssi': -22, 'actual_length': 8, 'data': 'Hi there'}}
```
