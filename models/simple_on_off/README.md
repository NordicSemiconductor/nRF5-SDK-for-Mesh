# Simple OnOff model

This implementation of a simple OnOff model can be used to switch
things on or off by manipulating a single on/off state. The intention of this
model is to have a simple example model that can be used as a baseline for
constructing your own model.

Note that the simple OnOff model should not be confused with the
Generic OnOff Model as specified in the Mesh Model Specification v1.0. The Generic
OnOff Model provides additional features such as control over when
and for how long the transition between the on/off state should be
performed.

> **Important:** When the server has a publish address set (as in the light switch example),
> the server will publish its state to its publish address every time its state changes.

## Messages and behavior

@copydoc SIMPLE_ON_OFF_MODEL

See [the message documentation](@ref SIMPLE_ON_OFF_COMMON) for details about the message format.

The behavior of the simple OnOff server is very simple and illustrated by the following message chart.
![Simple OnOff behavior](img/simple_on_off_model.png "Simple OnOff behavior")

## More information
For more information about creating models, see
[Creating new models](@ref md_doc_getting_started_how_to_models).
