# Interrupt priority levels

This section gives an overview of interrupt priority levels used by the mesh stack, and
the interrupt priority level restrictions that are imposed on the application when using the stack.

The mesh stack runs in two interrupt priorities:
- The time-critical parts of the stack run in IRQ priority 0 (radio interrupt priority).
- The less time-critical parts should run in the same IRQ priority as the user application's
low priority code. This can be any priority lower than the SoftDevice API call priority,
but it should be the same as the SoftDevice event IRQ handler.
When using the nRF5 SDK, this will normally be `::NRF_MESH_IRQ_PRIORITY_LOWEST`
(APP_IRQ_PRIORITY_LOWEST).
However, if you use the nRF5 SDK's app_scheduler module to run the application from the main loop,
the low priority parts of the mesh stack should also run from the main loop.

## Configuration

The mesh network must know in which IRQ priority the user application is running.
nrf_mesh_init_params_t (used by nrf_mesh_init) now has an irq_priority field.
Normally it should be set to `::NRF_MESH_IRQ_PRIORITY_LOWEST`, or `::NRF_MESH_IRQ_PRIORITY_THREAD` if
running in the main loop. The APP_IRQ_PRIORITY_* defines in NRF5 SDK can also be used.

## Limitations

Except for calling initialization related functions before entering the main loop,
no mesh API functions must be called from an IRQ priority other than the one specified in
the configuration. Breaking this rule may cause unpredicable behavior.

## Required changes to applications made using nRF5 SDK for Mesh v0.10.0-Alpha or older

Applications made using nRF5 SDK for Mesh v0.10.0-Alpha or older would normally run low priority
event handling in several IRQ priorities.
Mesh events were processed in the main loop by simply calling nrf_mesh_process():
```
while (true)
{
    (void)nrf_mesh_process();
}
```

SoftDevice events were processed in the SoftDevice event IRQ handler.
For most SoftDevice versions the IRQ handler would by default be running in
APP_IRQ_PRIORITY_LOWEST. Bearer events were handled in APP_IRQ_PRIORITY_LOW.

In nRF5 SDK for Mesh v1.0.0 or later, all event processing shall run in the same IRQ priority.
The user can choose to do this in a low priority interrupt, or in the main loop.

### Run application and mesh event handling in low priority interrupt

When running in low priority interrupt, the application should no longer call nrf_mesh_process(),
so the main loop should be changed to e.g.:
```
while (true)
{
    (void)sd_app_evt_wait();
}
```

### Run application and mesh event handling in main loop using app_scheduler

When running from the main loop, nrf_mesh_process() still needs to be called,
but as soon as there are no more events to be processed, the CPU can be put to sleep:
```
while (true)
{
    app_sched_execute();
    bool done = nrf_mesh_process();
    if (done)
    {
        sd_app_evt_wait();
    }
}
```
