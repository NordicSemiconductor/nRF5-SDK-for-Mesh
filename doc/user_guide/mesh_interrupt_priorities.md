# Setting interrupt priority levels

The Bluetooth mesh stack has different interrupt priority levels defined by its architecture. It includes
various interrupt priority level restrictions that are imposed on the application when using
the stack to avoid race condition scenarios and ensure that all operations execute
in non-overlapping manner.

**Table of contents**
- [Interrupt priority level list and configuration](@ref irq_levels_list_configuration)
- [Interrupt priority level restrictions](@ref irq_levels_restrictions)
- [Changes in application IRQ priority from nRF5 SDK for Mesh v1.0.0](@ref irq_levels_changes)
    - [Run application and Bluetooth mesh event handling in low priority interrupt](@ref irq_levels_changes_low_priority_interrupt)
    - [Run application and Bluetooth mesh event handling in main loop](@ref irq_levels_changes_main_loop)


---

## Interrupt priority level list and configuration @anchor irq_levels_list_configuration

The Bluetooth mesh stack runs in the following interrupt priorities:
- _IRQ priority 0 (radio interrupt priority)_ -- for the time-critical parts of the stack.
- _Same IRQ priority as the user application's low priority code_ -- for the less time-critical
parts of the stack.
    - This can be any priority lower than the SoftDevice API call priority, but it must be the same
    as the SoftDevice event IRQ handler. When using the nRF5 SDK, this will normally be
    @ref NRF_MESH_IRQ_PRIORITY_LOWEST (@link_app_irq_priority_lowest). However, if you use the nRF5
    SDK's app_scheduler module to run the application from the main loop, the low priority parts
    of the Bluetooth mesh stack should also run from the main loop.

The Bluetooth mesh network must know in which IRQ priority the user application is running.
To know this, nrf_mesh_init_params_t (used by @ref nrf_mesh_init) now has an `irq_priority` field.
Normally, it must be set to @ref NRF_MESH_IRQ_PRIORITY_LOWEST, or @ref NRF_MESH_IRQ_PRIORITY_THREAD
if running in the main loop. You can also use the APP_IRQ_PRIORITY_* defines in the nRF5 SDK.

---

## Interrupt priority level restrictions @anchor irq_levels_restrictions

You must ensure that no Bluetooth mesh API functions are called from an IRQ priority other than
the one specified in the configuration. The only exception is calling the initialization-related
functions before calling @ref mesh_stack_start(). Breaking this rule can cause unpredictable behavior.

The Bluetooth mesh stack uses the @link_SoftDevice_NVIC for accessing NVIC. In particular,
the functions @link_sd_nvic_criticial_region_enter and @link_sd_nvic_criticial_region_exit
are used to prevent the Bluetooth mesh event handler from being started in some cases. The application
must not use these functions in any callback functions provided by the application to the Bluetooth mesh stack.

---

## Changes in application IRQ priority from nRF5 SDK for Mesh v1.0.0 @anchor irq_levels_changes

Starting with the nRF5 SDK for Mesh v1.0.0, all event processing must run in the same IRQ priority.

Depending on your application, you can choose to:
- [Run application and Bluetooth mesh event handling in low priority interrupt](@ref irq_levels_changes_low_priority_interrupt)
- [Run application and Bluetooth mesh event handling in main loop](@ref irq_levels_changes_main_loop)

Before the release of the nRF5 SDK for Mesh v1.0.0, applications made using nRF5 SDK for Mesh
v0.10.0-Alpha or an older version of the Mesh SDK would normally run low priority event handling
in several IRQ priorities:
- Bluetooth mesh events were processed in the main loop by simply calling nrf_mesh_process():
```
    while (true)
    {
        (void)nrf_mesh_process();
    }
```
- SoftDevice events were processed in the SoftDevice event IRQ handler.
For most SoftDevice versions, the IRQ handler would be running in
@link_app_irq_priority_lowest by default. Bearer events were handled in @link_app_irq_priority_low.


### Running application and Bluetooth mesh event handling in low priority interrupt @anchor irq_levels_changes_low_priority_interrupt

When running in a low priority interrupt, the application can no longer call @ref nrf_mesh_process().
You must change the main loop. For example:
```
while (true)
{
    (void)sd_app_evt_wait();
}
```

### Running application and Bluetooth mesh event handling in main loop @anchor irq_levels_changes_main_loop

You can use the application scheduler from the @link_nRF5SDK to run the application and the Bluetooth mesh
event handling.

To add app_scheduler to the application and initialize it:
-# In the example directory, edit `CMakeList.txt`.
-# Add the following module to the executable target sources:
```
"${SDK_ROOT}/components/libraries/scheduler/app_scheduler.c"
```
-# Add the target include directory that corresponds to the executable target source added:
```
    "${SDK_ROOT}/components/libraries/scheduler"
```
    @note If you are using SEGGER Embedded Studio, use one of the following options:
    - [Regenerate the SEGGER project](@ref how_to_build_cmake_generating_SES).
    - Manually add the source file and the include directory to your existing project.
-# In the example `include` directory, add the following defines in `app_config.h`:
```
#define APP_SCHEDULER_ENABLED 1
#define NRF_SDH_DISPATCH_MODEL 1
#define APP_TIMER_CONFIG_USE_SCHEDULER 1
```
This enables the application scheduler and changes the event dispatch model for the SoftDevice handler
and the application timer.
-# In the application `main()`, initialize app_scheduler:
```
    APP_SCHED_INIT(APP_SCHED_EVENT_SIZE, APP_SCHED_QUEUE_SIZE);
```

@note
- `APP_SCHED_EVENT_SIZE` must be greater than or equal to `APP_TIMER_SCHED_EVENT_DATA_SIZE`, because
    the app timer also needs to use the scheduler.
- `APP_SCHED_QUEUE_SIZE` must be sufficiently large. This is required to be able to hold many events
    (user events, SoftDevice events, multiple events from various user or stack timers, or both)
    that could be generated between two successive calls to the `app_sched_execute()`
    from the main loop. The value of `32` could be reasonable for typical use cases,
    when there is a minimal or no additional user code within the main loop.


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

If you need to use `simple_hal.c` module to handle button press actions,
use the scheduler to execute the `button_event_handler()` function from the thread priority.
To do this, use an intermediate function as a button event handler, and post the event
to the secondary function using the scheduler. For example:
```
static void button_event_handler_app_sched(void * p_event_data, uint16_t event_size)
{
    uint8_t * button_number = (uint8_t *) p_event_data;
    NRF_MESH_ASSERT(event_size == 1);
    button_event_handler(*button_number);
}

static void button_event_handler_irq(uint32_t button_number)
{
    static uint8_t bt_num;

    bt_num = button_number;
    app_sched_event_put((void *)&bt_num, sizeof(bt_num), button_event_handler_app_sched);
}

...

static void initialize(void)
{
...
#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler_irq));
#endif
...
}
```
