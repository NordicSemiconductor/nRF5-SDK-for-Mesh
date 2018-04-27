/* nRF5 SDK for Mesh Bootloader linker script -- Generated 2017-12-08 13:44:31 by tools/linker/linker_file_generator.py */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
MEMORY
{
    FLASH (rx) :           ORIGIN = 0x3a000, LENGTH = 0x5800
    RAM (rw) :             ORIGIN = 0x20000004, LENGTH = 0x7cfc
    CORE_BL_RAM (rw) :     ORIGIN = 0x20007d00, LENGTH = 0x300
    UICR_BOOTADDR (r) :    ORIGIN = 0x10001014, LENGTH = 0x04
}

SECTIONS
{
    .core_data :
    {
        __core_ram_start__ = .;
        . = ALIGN(4);
        *sha256.c.obj (.data*)
        *bootloader_app_bridge.c.obj (.data*)
        *bootloader_info.c.obj (.data*)
        *uECC.c.obj (.data*)
        *dfu_mesh.c.obj (.data*)
        *dfu_transfer_mesh.c.obj (.data*)
        *dfu_bank.c.obj (.data*)
        *dfu_util.c.obj (.data*)
        . = ALIGN(4);
        __core_ram_end__ = .;
    } > CORE_BL_RAM
    .core_bss :
    {
        __core_ram_start__ = .;
        . = ALIGN(4);
        *sha256.c.obj (.bss*)
        *bootloader_app_bridge.c.obj (.bss*)
        *bootloader_info.c.obj (.bss*)
        *uECC.c.obj (.bss*)
        *dfu_mesh.c.obj (.bss*)
        *dfu_transfer_mesh.c.obj (.bss*)
        *dfu_bank.c.obj (.bss*)
        *dfu_util.c.obj (.bss*)
        . = ALIGN(4);
        __core_ram_end__ = .;
    } > CORE_BL_RAM

    /* Ensures the Bootloader start address in flash is written to UICR when flashing the image. */
    .uicrBootStartAddress :
    {
      KEEP(*(.uicrBootStartAddress))
    } > UICR_BOOTADDR

}

INCLUDE "nrf5x_common.ld"