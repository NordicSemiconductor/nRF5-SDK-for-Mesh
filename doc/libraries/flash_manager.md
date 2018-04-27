# Flash manager 

The flash manager (@ref FLASH_MANAGER "API documentation") takes care of persistent storage for the mesh data. It stores data as handle/value pairs in individually managed multi-page flash sections. 

The flash entries are stored as handle/value pairs in the following format:

Field  | Size        | Offset  | Description
-------|-------------|---------|------------
Length | 16 bits     | 0 bytes | Length of the entry in words, including the header fields
Handle | 16 bits     | 2 bytes | Identifier for the entry
Data   | 0-128 bytes | 4 bytes | Data array for the entry

Handles can be written, read, modified, replaced, and deleted. To group entries, it is also possible to search for entries with entry filters.

## Areas

You can use any number of individual flash manager areas. Each area is made up of between one and 255 flash pages and has a separate handle-address space from other managers. 

Each flash page in a managed area is made up of a metadata header and a data section. The metadata header is a total of 8 bytes and holds the following values:
 
Field                            | Size    | Offset  | Value    | Description
---------------------------------|---------|---------|----------|------------------------------------------------
Metadata length                  | 8 bits  | 0 bytes | `8`      | Length of the metadata in bytes
Entry header length              | 8 bits  | 1 bytes | `4`      | Length of each entry header in bytes
Entry header handle field length | 8 bits  | 2 bytes | `16`     | Length of the entry header handle field in bits
Entry header length field length | 8 bits  | 3 bytes | `16`     | Length of the entry header length field in bits
Area page index                  | 8 bits  | 4 bytes | ?        | This page's index in its area
Area page count                  | 8 bits  | 5 bytes | ?        | The number of pages in this area
Padding                          | 16 bits | 6 bytes | `0xFFFF` | Padding

The data section of the flash page follows right after the metadata and continues to the end of the page.

## Handles

Each handle is unique to every flash manager. Writing an entry with the same handle to the same manager twice will invalidate the first entry. Handles can be any value in the range `0x0001` to `0x7EFF`. Handle `0x0000` is considered invalid, and handles `0x7F00` and up are considered special handles used for internal management.

### Searching with filters

Handle filters allow searching for entries based on matching criteria on their handles. In this way, you can group handles together in ranges without having to search for every handle in the range individually. 

Filters are patterns and masks applied to the handles. The mask tells the manager on which bits in the handle to look for pattern matching. In code, the flash manager will yield any entry whose handle passes the following test:

```C
((handle & filter.mask) == (filter.pattern & filter.mask))
```

The search function takes a `p_start` argument, which acts as an iterator token, from where the search is initiated (non-inclusive). Passing `NULL` starts the search from the beginning of the managed area. To keep searching through the area, keep passing the pointer yielded from the previous call to the search function.

> **Important:** Entries are returned in order of flashing, not in sequential handle order. 


## Defragmentation

Due to limitations in the flash hardware, replacing handles in the flash manager means writing a completely new entry at the end of the existing entries and then invalidating the old entry. This will eventually cause the flash area to fill up with invalid entries, which prevents adding or replacing more entries. When this happens, the invalid entries must be removed and the page rebuilt with only the valid entries that are present. This process is called _defragmentation_. It is executed by the flash manager defrag module. The process is triggered automatically when the manager runs out of space. For more details on the defrag procedure, see the `flash_manager_defrag.c` file.

The defrag procedure requires a dedicated flash page. This flash page is used to take a copy of all valid handles before they are all erased in the original. In this way, entries can be recovered if a power failure should occur during the procedure. 

> **Important:** Defragmentation moves entries around in the managed flash area. Therefore, you should never keep raw pointers to entries across contexts, because they may be invalidated with every written entry.

## Power failure protection

The flash manager can handle circuit power loss at any time without loss of data. This is ensured by a strict policy of always keeping at least one copy of every entry present in flash at all times. When replacing entries, the flash manager first writes the new version in the entry. Then, only when the new version is fully written, the old handle is invalidated. If the device loses power in the brief period when there are two entries with the same handle present in the manager, the first handle is invalidated upon bootup.

To ensure the validity of new entries, a seal is placed at the end of the entries. If power failure should occur in the middle of adding a new entry, the flash manager looks for a seal at the end of the entries upon reboot. If it fails to locate such a seal, it assumes that the last entry is broken and invalidate it before adding the missing seal at the end. The manager will never yield any unsealed entries in any get functions, but considers the last handle invalid until it is followed by a seal.

## Flash area locations

The flash areas may be located anywhere in the device flash with a couple of exceptions:
  - Address `0x0000000` is considered invalid.
  - The address of the defrag recovery page is reserved.

By default, the defrag recovery page is the last page before the bootloader starts, or the last page in device flash if the bootloader is not present (determined by the `UICR->BOOTLOADERADDR` register). This location can be overriden by defining `FLASH_MANAGER_RECOVERY_PAGE` in the compiler defines. Note that the recovery page should be the same page in every iteration of the firmware, to avoid loss of backed up recovery data.

There is no guaranteed overlap-check in the flash manager module, so you must ensure that two flash manager areas do not overlap.
