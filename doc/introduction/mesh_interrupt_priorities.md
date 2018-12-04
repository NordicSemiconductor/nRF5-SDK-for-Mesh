# Interrupt priority levels

This section gives an overview of interrupt priority levels used by the mesh stack, and
the interrupt priority level restrictions that are imposed on the application when using the stack.

The mesh stack runs in two interrupt priorities:
- The time-critical parts of the stack run in IRQ priority 0 (radio interrupt priority).
- The less time-critical parts should run in the same IRQ priority as the user application's
low priority code. This can be any priority lower than the SoftDevice API call priority,
but it should be the same as the SoftDevice event IRQ handler.
When using the nRF5 SDK, this will normally be @ref NRF_MESH_IRQ_PRIORITY_LOWEST
(@link_app_irq_priority_lowest).
However, if you use the nRF5 SDK's app_scheduler module to run the application from the main loop,
the low priority parts of the mesh stack should also run from the main loop.

## Configuration

The mesh network must know in which IRQ priority the user application is running.
nrf_mesh_init_params_t (used by @ref nrf_mesh_init) now has an irq_priority field.
Normally it should be set to @ref NRF_MESH_IRQ_PRIORITY_LOWEST, or @ref NRF_MESH_IRQ_PRIORITY_THREAD if
running in the main loop. The APP_IRQ_PRIORITY_* defines in NRF5 SDK can also be used.

## Limitations

Except for calling initialization related functions before calling @ref mesh_stack_start(),
no mesh API functions shall be called from an IRQ priority other than the one specified in
the configuration. Breaking this rule may cause unpredicable behavior.

The mesh stack uses the @link_SoftDevice_NVIC for accessing NVIC. In particular,
the functions @link_sd_nvic_criticial_region_enter and @link_sd_nvic_criticial_region_exit
are used to prevent the mesh event handler from being started in some cases. The application
must not use these functions in any callback functions provided by the application to the mesh stack.

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
@link_app_irq_priority_lowest. Bearer events were handled in @link_app_irq_priority_low.

In nRF5 SDK for Mesh v1.0.0 or later, all event processing shall run in the same IRQ priority.
The user can choose to do this in a low priority interrupt, or in the main loop.

### Run application and mesh event handling in low priority interrupt

When running in low priority interrupt, the application should no longer call @ref nrf_mesh_process(),
so the main loop should be changed to e.g.:
```
while (true)
{
    (void)sd_app_evt_wait();
}
```

### Run application and mesh event handling in main loop using app_scheduler

When running from the main loop, @ref nrf_mesh_process() still needs to be called,
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
