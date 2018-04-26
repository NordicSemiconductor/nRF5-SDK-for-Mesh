# Sending mesh packets

## Requirements

- Two boards running the serial example

## Instructions

Let's set up a tiny mesh network. Connect two devices and start the serial interface:


    $ python interactive_pyaci.py -d /dev/ttyACM0 /dev/ttyACM1 --no-logfile


Next, add a network key, application key, and a local unicast address with the
`quick_setup()` method:


    In [1]: for dev in d: dev.quick_setup()
    2017-08-02 10:01:53,154 - INFO - ttyACM0: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,157 - INFO - ttyACM0: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,158 - INFO - ttyACM0: Success
    2017-08-02 10:01:53,161 - INFO - ttyACM1: SubnetAdd: {'subnet_handle': 0}
    2017-08-02 10:01:53,162 - INFO - ttyACM1: AppkeyAdd: {'appkey_handle': 0}
    2017-08-02 10:01:53,166 - INFO - ttyACM1: Success

The device is by default set up to use the `access.py` module which hides
raw message events using an event filter. In this tutorial we'll disable that
filter:

    In [2]: d[0].event_filter_disable()
    In [3]: d[1].event_filter_disable()

Add publish addresses:


    In [4]: d[0].send(cmd.AddrPublicationAdd(d[1].local_unicast_adress_start))
    2017-08-02 10:10:11,502 - INFO - ttyACM0: AddrPublicationAdd: {'address_handle': 0}
    In [5]: d[1].send(cmd.AddrPublicationAdd(d[0].local_unicast_adress_start))
    2017-08-02 10:10:13,132 - INFO - ttyACM1: AddrPublicationAdd: {'address_handle': 0}


Notice the handles; they are used to reference the addresses and keys at a later point.
Let's store them in some variables:


    In [6]: publish_handle = 0
    In [7]: appkey_handle = 0


Next, we test out sending a message between the devices:

    In [8]: d[0].send(cmd.PacketSend(appkey_handle, d[0].local_unicast_adress_start, publish_handle, 1, 0, "Hello World"))
    2017-08-02 10:15:18,073 - INFO - ttyACM0: Success
    2017-08-02 10:15:18,092 - INFO - ttyACM1: {event: MeshMessageReceivedUnicast, data: {'actual_length': 11, 'adv_addr_type': 1,            \
                                               'adv_addr': bytearray(b'\xe9\x04/\xcc\xcf\xf7'), 'src': 1, 'data': bytearray(b'Hello World'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 2, 'ttl': 1}}

    In [9]: d[1].send(cmd.PacketSend(appkey_handle, d[1].local_unicast_adress_start, publish_handle, 1, 0, "Hi there"))
    2017-08-02 10:15:42,806 - INFO - ttyACM1: Success
    2017-08-02 10:15:42,837 - INFO - ttyACM0: {event: MeshMessageReceivedUnicast, data: {'actual_length': 8, 'adv_addr_type': 1,           \
                                               'adv_addr': bytearray(b'\xd6\x89\x1c\x8d\r\xd7'), 'src': 2, 'data': bytearray(b'Hi there'), \
                                               'rssi': 14, 'subnet_handle': 0, 'appkey_handle': 0, 'dst': 1, 'ttl': 1}}
