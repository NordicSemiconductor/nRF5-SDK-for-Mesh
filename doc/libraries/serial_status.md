# Serial status codes

Most serial commands sent to an nRF5 device will yield a [command response](@ref cmd-rsp)
from the device. This command response always contains a status field that indicates
whether the command succeeded or what the reason for failing was.

The following table lists status codes yielded by the mesh serial interface.

| Code   | Name                      | Description
|--------|---------------------------|-------------
| `0x00` | `SUCCESS`                 | The command completed successfully.
| `0x80` | `ERROR_UNKNOWN`           | An unknown error occured.
| `0x81` | `ERROR_INTERNAL`          | An internal error occured. This indicates that there could be something wrong with the serial command handler implementation.
| `0x82` | `ERROR_CMD_UNKNOWN`       | The command was not recognized by the mesh device.
| `0x83` | `ERROR_INVALID_STATE`     | The command was received at a time when the state of the device prevents it from processing it.
| `0x84` | `ERROR_INVALID_LENGTH`    | The length of the received command was incorrect.
| `0x85` | `ERROR_INVALID_PARAMETER` | One or more of the command parameters were incorrect.
| `0x86` | `ERROR_BUSY`              | The mesh device was busy processing a previous command, or a required resource was temporarily unavailable.
| `0x87` | `ERROR_INVALID_DATA`      | Invalid data was sent as part of the command parameters.
| `0x8e` | `ERROR_REJECTED`          | The command was rejected, either because of insufficient resources or because the requested resource was temporarily unavailable.
| `0x93` | `ERROR_TIMEOUT`           | The command processing was aborted because of a timeout.
| `0x98` | `ERROR_INVALID_KEY_DATA`  | The key data sent as part of the command parameters was invalid.

