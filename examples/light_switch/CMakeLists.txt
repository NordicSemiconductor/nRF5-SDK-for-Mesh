add_subdirectory("client")
add_subdirectory("server")
add_subdirectory("provisioner")

# Proxy only supported on nRF52 series devices due to nRF5 SDK dependency:
if (${${PLATFORM}_FAMILY} STREQUAL "NRF52" AND NOT PLATFORM MATCHES "nrf52810")
    add_subdirectory("proxy_server")
    add_subdirectory("proxy_client")
endif()
