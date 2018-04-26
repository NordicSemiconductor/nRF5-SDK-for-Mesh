# Serial interface

The serial interface to the mesh stack provides a way for external devices to
interact with a device running the mesh stack via a serial port. The interface
is based on the nRF8001 interface and runs a simple length-opcode-data format
for both commands and events.

Each command from the host to the nRF5 will get a command response event in
return, except for the echo command, which gets an echo response.

At bootup, the nRF5 sends a "Device started" event, after which it is
ready to receive commands from the host.

The serial frame format is 8n1, 115200 baud with hardware flow control enabled.

## Serial Packet Format

All serial packets, commands, responses, and events are encapsulated in serial
packets following the format detailed in the table below. All multi-byte values
are sent in little endian format.

Packet format:

Field         | Size (bytes) | Description
--------------|--------------|-------------
Length        |          1   | Length of the serial command, response, or event. Does not include the length field itself.
Opcode        |          1   | Opcode of the serial command, response, or event.
Payload       |          0-n | Parameters or data of the command.

## Details

A detailed overview over all serial commands and events can be found in
@subpage md_doc_libraries_serial_cmd and @subpage md_doc_libraries_serial_evt.

The Mesh serial interface uses a set of status codes to communicate the result of a command. A list of potential status codes can
be found here: @subpage md_doc_libraries_serial_status.

## Evaluation

There is an interactive Python script for communicating with a device running the serial interface.
Read the @subpage md_scripts_interactive_pyaci_README README to get started.
