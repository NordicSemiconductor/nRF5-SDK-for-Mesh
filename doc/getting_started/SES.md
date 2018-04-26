# nRF5 SDK for Mesh Segger Embedded Studio first time setup

Segger Embedded Studio determines the location of the nRF5 SDK through macros.
Before building the example, you must first configure the `SDK_ROOT` macro
in Segger Embedded Studio. This is a one time global configuration that will
still be valid the next time you open Segger Embedded Studio. The `SDK_ROOT`
variable defaults to an nRF5 SDK 15.0.0 instance unzipped right next to the mesh
folder if not set.

The `SDK_ROOT` macro can be set by navigating to Tools -> Options, then
"Building". Under "Build" in the configuration list, edit "Global macros" to
contain `SDK_ROOT=<the path to your nRF5 SDK 15 instance>`. Save the
configuration.

The path can be verified by opening one of the source files under the "nRF5 SDK"
file group. If the macro was set correctly, the file should open in the editor
window. If not, it will show an error message telling you that the file couldn't
be found.

For more info on Segger Embedded Studio macros, see
https://studio.segger.com/ide_project_macros.htm
