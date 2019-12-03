/**
 * nRF5 SDK for Mesh Application linker script.
 *
 * This file is intended for nrf52810_xxAA with the s112_6.1.1 SoftDevice.
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    /* MBR_FLASH (rx)        : ORIGIN = 0x00000000, LENGTH = 0x00001000 */
    /* SOFTDEVICE_FLASH (rx) : ORIGIN = 0x00001000, LENGTH = 0x00018000 */

    FLASH (rx)               : ORIGIN = 0x00019000, LENGTH = 0x00017000
    RAM (rwx)                : ORIGIN = 0x20001680, LENGTH = 0x00004980
}

/* These sections are common to all the linker files and specific to the nRF5 SDK use of section variables. */
SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM
   .log_dynamic_data :
  {
    PROVIDE(__start_log_dynamic_data = .);
    KEEP(*(SORT(.log_dynamic_data*)))
    PROVIDE(__stop_log_dynamic_data = .);
  } > RAM
  .log_filter_data :
  {
    PROVIDE(__start_log_filter_data = .);
    KEEP(*(SORT(.log_filter_data*)))
    PROVIDE(__stop_log_filter_data = .);
  } > RAM
  .cli_sorted_cmd_ptrs :
  {
    PROVIDE(__start_cli_sorted_cmd_ptrs = .);
    KEEP(*(.cli_sorted_cmd_ptrs))
    PROVIDE(__stop_cli_sorted_cmd_ptrs = .);
  } > RAM
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
  .nrf_mesh_ram :
  {
    PROVIDE(__start_nrf_mesh_ram = .);
    KEEP(*(SORT(.nrf_mesh_ram.*)))
    PROVIDE(__stop_nrf_mesh_ram = .);
  } > RAM

} INSERT AFTER .data;

SECTIONS
{
  .sdh_ant_observers :
  {
    PROVIDE(__start_sdh_ant_observers = .);
    KEEP(*(SORT(.sdh_ant_observers*)))
    PROVIDE(__stop_sdh_ant_observers = .);
  } > FLASH
  .sdh_ble_observers :
  {
    PROVIDE(__start_sdh_ble_observers = .);
    KEEP(*(SORT(.sdh_ble_observers*)))
    PROVIDE(__stop_sdh_ble_observers = .);
  } > FLASH
  .sdh_soc_observers :
  {
    PROVIDE(__start_sdh_soc_observers = .);
    KEEP(*(SORT(.sdh_soc_observers*)))
    PROVIDE(__stop_sdh_soc_observers = .);
  } > FLASH
  .sdh_state_observers :
  {
    PROVIDE(__start_sdh_state_observers = .);
    KEEP(*(SORT(.sdh_state_observers*)))
    PROVIDE(__stop_sdh_state_observers = .);
  } > FLASH
  .sdh_stack_observers :
  {
    PROVIDE(__start_sdh_stack_observers = .);
    KEEP(*(SORT(.sdh_stack_observers*)))
    PROVIDE(__stop_sdh_stack_observers = .);
  } > FLASH
  .sdh_req_observers :
  {
    PROVIDE(__start_sdh_req_observers = .);
    KEEP(*(SORT(.sdh_req_observers*)))
    PROVIDE(__stop_sdh_req_observers = .);
  } > FLASH
  .log_const_data :
  {
    PROVIDE(__start_log_const_data = .);
    KEEP(*(SORT(.log_const_data*)))
    PROVIDE(__stop_log_const_data = .);
  } > FLASH
    .nrf_balloc :
  {
    PROVIDE(__start_nrf_balloc = .);
    KEEP(*(.nrf_balloc))
    PROVIDE(__stop_nrf_balloc = .);
  } > FLASH
    .nrf_queue :
  {
    PROVIDE(__start_nrf_queue = .);
    KEEP(*(.nrf_queue))
    PROVIDE(__stop_nrf_queue = .);
  } > FLASH
    .cli_command :
  {
    PROVIDE(__start_cli_command = .);
    KEEP(*(.cli_command))
    PROVIDE(__stop_cli_command = .);
  } > FLASH
  .crypto_data :
  {
    PROVIDE(__start_crypto_data = .);
    KEEP(*(SORT(.crypto_data*)))
    PROVIDE(__stop_crypto_data = .);
  } > FLASH
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(SORT(.pwr_mgmt_data*)))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > FLASH
  .log_backends :
  {
    PROVIDE(__start_log_backends = .);
    KEEP(*(SORT(.log_backends*)))
    PROVIDE(__stop_log_backends = .);
  } > FLASH
  .nrf_mesh_flash :
  {
    PROVIDE(__start_nrf_mesh_flash = .);
    KEEP(*(SORT(.nrf_mesh_flash.*)))
    PROVIDE(__stop_nrf_mesh_flash = .);
  } > FLASH
} INSERT AFTER .text


INCLUDE "nrf_common.ld"