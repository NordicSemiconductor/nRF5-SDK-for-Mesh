# Serial interface library

The serial interface to the Bluetooth mesh stack provides a way for external devices
to interact with a device running the Bluetooth mesh stack via a serial port. The interface
is based on the nRF8001 interface and runs a simple length-opcode-data format
for both commands and events.

---

## Commands @anchor serial_interface_details

At bootup, the nRF5 sends a "Device started" event, after which it is
ready to receive commands from the host.

Each command from the host to the nRF5 gets a command response event in
return, except for the echo command, which gets an echo response.

A detailed overview over all serial commands and events can be found in
@subpage md_doc_user_guide_modules_serial_cmd and @subpage md_doc_user_guide_modules_serial_evt.

The serial interface uses a set of status codes to communicate the result of a command.
A list of potential status codes can be found in @subpage md_doc_user_guide_modules_serial_status.

---

## Serial packet format and transfer parameters @anchor serial_interface_packet_format

All serial commands, responses, and events are encapsulated in serial
packets that use the format detailed in the following table. All multi-byte values
are sent in little-endian format.

Field         | Size (bytes) | Description
--------------|--------------|-------------
Length        |          1   | Length of the serial command, response, or event.<br>Does not include the length field itself.
Opcode        |          1   | Opcode of the serial command, response, or event.
Payload       |          0-n | Parameters or data of the command.

The standard serial packet transfer parameters are as follows:
- 8 data bits
- 1 stop bit
- No parity
- Baud rate: 115200
- Hardware flow control: enabled


---

## Evaluation @anchor serial_interface_evaluation

To communicate with a device running the serial interface, you can use an interactive Python script.
See @ref md_scripts_interactive_pyaci_README to get started.
