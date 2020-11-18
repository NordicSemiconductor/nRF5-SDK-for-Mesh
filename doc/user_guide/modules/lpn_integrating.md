# Integrating Low Power node feature

The nRF5 SDK for Mesh provides the [Low Power node feature](@ref md_doc_user_guide_modules_lpn_concept)
that you can use in your application.

To support the LPN feature, your application must handle the friendship establishment procedure
using the available @ref MESH_LPN API. Once the friendship is established, the communication between
the LPN and the Friend is based on a set of timing parameters exchanged during
the friendship establishment procedure.

**Table of contents**
- [Handling friendship establishment procedure](@ref lpn_integration_friendship_procedure)
- [Friendship timing parameters and request-response mechanism](@ref LPN_integration_friendship_parameters)
	- [ReceiveDelay and ReceiveWindow](@ref LPN_integration_friendship_parameters_1)
	- [PollTimeout](@ref LPN_integration_friendship_parameters_2)
	- [Additional timing control](@ref LPN_integration_friendship_parameters_pollinterval)
- [Practical recommendations for low power applications](@ref LPN_integration_practical_recommendations)

For an example of the Low Power node feature, see @ref md_examples_lpn_README.


---


## Handling friendship establishment procedure @anchor lpn_integration_friendship_procedure

Make sure you implement the following points in your code:
- Specify the required key timing parameters. Choosing the right values is particularly important.
It will influence the overall power consumption of the LPN for the duration of friendship.
The choice also depends on the use case and the desired responsiveness of the LPN node
to the incoming Bluetooth mesh messages. See the following table and the detailed description
in the following sections.

| Timing parameter 														| Short description 																	|	Chosen by		| Timing value				| Reference 																|
|-----------------------------------------------------------------------|---------------------------------------------------------------------------------------|-------------------|---------------------------|---------------------------------------------------------------------------|
| [ReceiveDelay](@ref mesh_lpn_friend_request_t::receive_delay_ms)		| Delay interval from the moment the Friend node received a suitable request message.	| LPN node			| 10-255 ms, steps of 1 ms	| @ref MESH_LPN_RECEIVE_DELAY_MIN_MS, @ref MESH_LPN_RECEIVE_DELAY_MAX_MS 	|
| [ReceiveWindow](@ref nrf_mesh_evt_lpn_friend_offer_t::offer)			| Interval within which the Friend node must send a response.							| Friend node		| 1-255 ms, steps of 1 ms	| @ref MESH_LPN_FRIEND_REQUEST_RETRY_COUNT, @ref MESH_LPN_POLL_RETRY_COUNT	|
| [PollTimeout](@ref mesh_lpn_friend_request_t::poll_timeout_ms)		| Interval within which the LPN node must send a request message.						| LPN node			| max. 95.9 hours			| @ref MESH_LPN_POLL_TIMEOUT_MIN_MS, @ref MESH_LPN_POLL_TIMEOUT_MAX_MS   	|

- If needed, set [the additional PollInterval timing parameter](@ref LPN_integration_friendship_parameters_pollinterval).
- Start the friendship establishment with @ref mesh_lpn_friend_request()
using the specified timing parameters.
If the potential Friend nodes within the radio range can accept the request,
they respond with a Friend Offer message (one per potential Friend node).
- Once the application firmware receives a callback from the Bluetooth mesh stack with an offer event
(@ref NRF_MESH_EVT_LPN_FRIEND_OFFER) with acceptable offer details provided as event parameters,
make sure it calls @ref mesh_lpn_friend_accept() to accept the offer. The application must call
this function before the procedure times out
(that is, the @ref NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT event is generated).
	- Typically, the first offer received will be the most suitable offer for the LPN. However,
	the application can wait for multiple offers before it selects the desired Friend. The criteria
	for selecting the Friend Offer depend on the parameters received in the offer.
- After the user application accepts the offer, the Bluetooth mesh stack sends a Friend Poll message
to the chosen Friend node.
	- If the Friend node responds with a Friend Update message, the friendship is considered established.
	- If the Friend node fails to send a response within the required timeframe or the LPN does not
	receive a response due to RF interference, the LPN will attempt to resend the first poll
	multiple times (@ref MESH_LPN_FRIEND_REQUEST_RETRY_COUNT)
	before the procedure times out.

@msc
hscale = "1.3";
APP,MESH_LPN,FRIEND_NODE;
|||;
APP rbox FRIEND_NODE  		[label="Friendship establishment procedure"];
|||;
APP rbox APP  		[label="Timing parameters defined within request"];
APP->MESH_LPN     		[label="mesh_lpn_friend_request()", URL="@ref mesh_lpn_friend_request()"];
MESH_LPN->FRIEND_NODE   [label="Request"];
FRIEND_NODE->MESH_LPN   [label="Friend Offer"];
MESH_LPN->APP      		[label="NRF_MESH_EVT_LPN_FRIEND_OFFER"];
APP->MESH_LPN     		[label="mesh_lpn_friend_accept()", URL="@ref mesh_lpn_friend_accept()"];
MESH_LPN->FRIEND_NODE   [label="Poll"];
FRIEND_NODE->MESH_LPN   [label="Friend Update"];
MESH_LPN->APP   [label="NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED"];
@endmsc

Once the friendship is established, the communication between the LPN and the Friend node happens
through a request-response mechanism.

If the Bluetooth mesh stack is unable to receive a response to the previously sent request, it will resend
the previous request. If there is no response after multiple attempts (@ref MESH_LPN_POLL_RETRY_COUNT),
the stack will terminate friendship and generate a termination event (@ref NRF_MESH_EVT_FRIENDSHIP_TERMINATED).
The application can start the friendship establishment procedure to re-establish friendship.
If required, the application can also terminate the friendship by calling @ref mesh_lpn_friendship_terminate().


---


## Friendship timing parameters and request-response mechanism @anchor LPN_integration_friendship_parameters

There are three important timing parameters that govern the friendship functionality:
- [ReceiveDelay and ReceiveWindow](@ref LPN_integration_friendship_parameters_1)
- [PollTimeout](@ref LPN_integration_friendship_parameters_2)

The nRF5 SDK for Mesh also provides an [additional timing control](@ref LPN_integration_friendship_parameters_pollinterval).

### ReceiveDelay and ReceiveWindow @anchor LPN_integration_friendship_parameters_1

![Friendship timings: ReceiveDelay and ReceiveWindow](images/polling_timings_params_v2.svg "Friendship timings: ReceiveDelay and ReceiveWindow")


When the Friend node receives any suitable request message from the LPN
(Friend Poll or Friend Subscription List Add/Remove), it waits for the ReceiveDelay interval
to send the response.
From the end of ReceiveDelay interval, the Friend must send a response within an interval
called ReceiveWindow. The LPN keeps its scanner turned off until the start of the ReceiveWindow,
then it turns the scanner on, and switches it off again as soon as a response is received
from the Friend node or when the ReceiveWindow ends.

If the Friend node fails to send a response within ReceiveWindow or the LPN does not
receive a response due to RF interference, the LPN will attempt to resend the previous request
multiple times (@ref MESH_LPN_FRIEND_REQUEST_RETRY_COUNT, @ref MESH_LPN_POLL_RETRY_COUNT)
before terminating the friendship.

### PollTimeout @anchor LPN_integration_friendship_parameters_2

Once the friendship is established, the LPN must send at least one request message within
an interval called PollTimeout. The Friend node counts the PollTimeout interval from the last
received request message from the LPN. If the LPN does not send a new request within the PollTimeout,
the Friend node considers the friendship terminated and it clears the Friend Queue.
If this happens, the LPN has to re-establish the friendship.

![Friendship timings: PollTimeout](images/polling_timings_params_poll_timeout.svg "Friendship timings: PollTimeout")

When the LPN determines that it is time to fetch the messages from the Friend node, it starts sending
consecutive request messages until the Friend Queue is empty. We call this sequence a poll cycle
(see the following image).

### Additional timing control @anchor LPN_integration_friendship_parameters_pollinterval

The nRF5 SDK for Mesh provides additional flexibility to the applications by enabling them to
control bursts of poll cycles within the long PollTimeout interval. This is done with the
optional [PollInterval](@ref mesh_lpn_poll_interval_set) parameter.

![Poll cycles and PollInterval](images/polling_timings_poll_interval.svg "Poll cycles and PollInterval")

The PollInterval parameter controls the interval between two successive poll cycles within the
PollTimeout interval. It affects the responsiveness of the LPN in relation to the rest of the network.
The application can change this parameter at runtime by calling @ref mesh_lpn_poll_interval_set().


---


## Practical recommendations for low power applications @anchor LPN_integration_practical_recommendations

When using the LPN feature and developing end applications around this feature, consider the
following recommendations:

* Provisioning and configuration procedures are power-consuming operations.
Disable PB-ADV and use only PB-GATT for provisioning a Low Power node.
After provisioning and configuration are complete, GATT proxy can be disabled to save additional power.
* Node configuration can also be done after establishing a friendship.
In this case, use shorter PollInterval settings on the node
and longer acknowledged message transaction timeouts and retry intervals on the peer configuration client.
* Use an event-driven approach for implementing application processing to
  minimize CPU usage.
* Choose the Friend Offer with the shortest receive window.