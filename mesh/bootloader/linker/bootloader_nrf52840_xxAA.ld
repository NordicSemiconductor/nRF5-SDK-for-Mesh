/* nRF5 SDK for Mesh Bootloader linker script. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
MEMORY
{
    FLASH (rx) :           ORIGIN = 0x000f8000, LENGTH = 0x00006000
    RAM (rw) :             ORIGIN = 0x20000004, LENGTH = 0x0003fcfc
    CORE_BL_RAM (rw) :     ORIGIN = 0x2003fd00, LENGTH = 0x00000300
    UICR_BOOTADDR (r) :    ORIGIN = 0x10001014, LENGTH = 0x04
    UICR_MBRPARAMADDR (r): ORIGIN = 0x10001018, LENGTH = 0x04
}

SECTIONS
{
    .core_data (NOLOAD) :
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
    .core_bss (NOLOAD) :
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

    /* The MBR used for the supported nRF52 SoftDevices expects a pointer to an empty
     * flash page written in the UICR.NRFFW[1] register.
     */
    .uicrBootMBRParamAddress :
    {
      KEEP(*(.uicrBootMBRParamAddress))
    } > UICR_MBRPARAMADDR
}

INCLUDE "nrf_common.ld"