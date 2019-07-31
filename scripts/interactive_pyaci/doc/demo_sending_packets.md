# Sending mesh packets

This tutorial shows how you can use two devices running the serial example
to demonstrate a communication between them over mesh.

**Table of contents**
- [Requirements](@ref pyaci_demo_sending_packets_requirements)
- [Configuration and testing](@ref pyaci_demo_sending_packets_configuration_testing)


---

## Requirements @anchor pyaci_demo_sending_packets_requirements

- Two boards running @ref md_examples_serial_README


---

## Configuration and testing @anchor pyaci_demo_sending_packets_configuration_testing

To configure sending mesh packets:
1. Set up a tiny mesh network by connecting two devices and starting the serial interface:
```
    $ python interactive_pyaci.py -d /dev/ttyACM0 /dev/ttyACM1 --no-logfile
```
2. Add a network key, application key, and a local unicast address with the
`quick_setup()` method:
```
    In [1]: for dev in d: dev.quick_setup()
    2017-08-02 10:01:53,154 - INFO - ttyACM0: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,157 - INFO - ttyACM0: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,158 - INFO - ttyACM0: Success
    2017-08-02 10:01:53,161 - INFO - ttyACM1: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,162 - INFO - ttyACM1: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,166 - INFO - ttyACM1: Success
```
3. Disable the `access.py` module that hides the raw message events using an event filter
(and is enabled by default on the device): 
```
    In [2]: d[0].event_filter_disable()
    In [3]: d[1].event_filter_disable()
```
4. Add publish addresses:
```
    In [4]: d[0].send(cmd.AddrPublicationAdd(d[1].local_unicast_address_start))
    2017-08-02 10:10:11,502 - INFO - ttyACM0: AddrPublicationAdd: {'address_handle': 0}
    In [5]: d[1].send(cmd.AddrPublicationAdd(d[0].local_unicast_address_start))
    2017-08-02 10:10:13,132 - INFO - ttyACM1: AddrPublicationAdd: {'address_handle': 0}
```
The handles are used to reference the addresses and keys at a later point.
5. Store the handles in variables:
```
    In [6]: publish_handle = 0
    In [7]: appkey_handle = 0
```
6. Define some variables for the following parameters of `PacketSend()` API: `ttl`, `force_segmented`, `transmic_size`, and `friendship_credentials_flag`.
```
    In [8]: ttl = 1
    In [9]: segmented = 0
    In [10]: mic_size = 0
    In [11]: friendship_credentials_flag = 0
```
It is possible to use these variables while sending packets between the devices.


To test sending a message between the devices:
```
    In [12]: d[0].send(cmd.PacketSend(appkey_handle, d[0].local_unicast_address_start, publish_handle, ttl, segmented, mic_size, friendship_credentials_flag, "Hello World"))
    2017-08-02 10:15:18,073 - INFO - ttyACM0: Success
    2017-08-02 10:15:18,092 - INFO - ttyACM1: {event: MeshMessageReceivedUnicast, data: {'actual_length': 11, 'adv_addr_type': 1,            \
                                               'adv_addr': bytearray(b'\xe9\x04/\xcc\xcf\xf7'), 'src': 1, 'data': bytearray(b'Hello World'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 2, 'ttl': 1}}

    In [13]: d[1].send(cmd.PacketSend(appkey_handle, d[1].local_unicast_address_start, publish_handle, ttl, segmented, mic_size, friendship_credentials_flag, "Hi there"))
    2017-08-02 10:15:42,806 - INFO - ttyACM1: Success
    2017-08-02 10:15:42,837 - INFO - ttyACM0: {event: MeshMessageReceivedUnicast, data: {'actual_length': 8, 'adv_addr_type': 1,           \
                                               'adv_addr': bytearray(b'\xd6\x89\x1c\x8d\r\xd7'), 'src': 2, 'data': bytearray(b'Hi there'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 1, 'ttl': 1}}
```
