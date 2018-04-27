# Serial example

This example implements the serial interface for the mesh stack. It can be used
unaltered as connectivity firmware for mesh devices. You can also modify the example
to provide additional functionality.

The example shows how to initialize the serial interface and enable ECDH offloading.
To initialize the serial interface, only two API calls are required:
```C
    nrf_mesh_serial_init(NULL);
    nrf_mesh_serial_enable();
```

ECDH offloading allows the device to take advantage of the host's more powerful
processor to perform ECDH operations during provisioning. It is enabled by using
the [options API](@ref NRF_MESH_OPT):
```C
    nrf_mesh_opt_t value = {.len = 4, .opt.val = 1 };
    nrf_mesh_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &value);
```

More information about ECDH offloading can be found in the
[provisioning documentation](@ref md_doc_getting_started_provisioning).

For more information about the mesh serial interface, see the
[serial documentation](@ref md_doc_libraries_serial).


