# Common example modules

Modules that are common to the mesh examples.


## Simple hardware abstraction layer

The simple hardware abstraction layer is a module that provides simple drivers for
the buttons and LEDs on development kits. It is used in the various mesh example
projects.


## RTT input

The RTT input module enables the examples to poll [the RTT](@ref segger-rtt) for input characters.
It uses the nRF5 SDK @link_app_timer to avoid busy waiting.