# CMake on Windows
## List of tools

> **Important:** Ensure that all command line tools are available in a folder referenced by
> the system path (e.g. the `PATH` environment variable).


 | Download link                                                                                                                 | Recommended *minimum* version |
 |-------------------------------------------------------------------------------------------------------------------------------|-------------------------------|
 | <a href="https://cmake.org/download/" target="_blank">CMake</a>                                                               | 3.9.0                         |
 | <a href="https://github.com/ninja-build/ninja/releases" target="_blank">Ninja</a>                                             | 1.7.2                         |
 | <a href="https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads" target="_blank">GNU ARM Embedded Toolchain</a> | 6-2017-q2-update (6.3.1)      |
 | <a href="http://www2.keil.com/mdk5/compiler/5/" target="_blank">ARM Compiler Version 5</a>                                    | 5                             |

### CMake

Download the latest installer from <a href="https://cmake.org/download/" target="_blank">CMake.org/download</a>
and follow the installation instructions.

### Toolchain

For Windows, both the armcc v5 and GCC ARM Embedded toolchains are supported.

* For ARMCC/Keil, follow the instructions provided for @link_armcc<!-- https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5 -->.
  The armcc v5 toolchain is also provided by @link_keil<!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--http://www2.keil.com/mdk5/uvision/-->.
* For the GCC ARM Embedded toolchain, download the @link_armnone<!-- https://developer.arm.com/open-source/gnu-toolchain/gnu-rm -->
  installer and follow the installation instructions.

### Python

Visit the <a href="https://www.python.org/downloads/" target="_blank">Python 3</a> website to download the
latest Python 3 installer. Ensure that `pip` is installed and that Python 3 is added to `PATH`.

### Build system
The Ninja build system is preferred on Windows. Download the binary from the
<a href="https://github.com/ninja-build/ninja/releases" target="_blank">ninja-build release page</a>
and place it in a suitable folder. Append the path to the folder to your `PATH` environment variable if
it is not there already.

## Additional tools for building unit tests (host) [Optional]
The unit test (host) build uses the standard GCC compiler, available on Windows through @link_mingw<!-- https://sourceforge.net/projects/mingw/files/-->.
Install the `mingw-base` and ensure that the 32-bit version is installed or that 32-bit libraries are
available. Furthermore, `libpthread` is needed for the multithreaded test.
This can be installed using `mingw-get.exe`. From the command line, simply call:

    $ mingw-get install libpthread

Ruby is used by the mocking framework @link_cmock<!-- https://github.com/ThrowTheSwitch/CMock -->.
Install it from the @link_ruby<!-- https://www.ruby-lang.org/ -->
website.

