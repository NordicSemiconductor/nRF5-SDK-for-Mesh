# Windows

> **Important:** Ensure that all installed tools are available in a folder referenced by the
> system path (e.g. the `PATH` environment variable).

## CMake

Download the latest installer from the @link_cmake_download<!-- https://cmake.org/download/ -->
and follow the installation instructions.

## Toolchain

For Windows, both the armcc v5 and GCC ARM Embedded toolchains are supported.

* For ARMCC/Keil, follow the instructions provided for @link_armcc<!-- https://developer.arm.com/products/software-development-tools/compilers/arm-compiler/downloads/version-5 -->.
  The armcc v5 toolchain is also provided by @link_keil<!--Keil: http://www2.keil.com/mdk5/compiler/5/-->
  and comes bundled with the @link_keiluvision<!--http://www2.keil.com/mdk5/uvision/-->.
* For the GCC ARM Embedded toolchain, download the @link_armnone<!-- https://developer.arm.com/open-source/gnu-toolchain/gnu-rm -->
  installer and follow the installation instructions.

### Python

Visit the @link_python3 <!--Python 3: https://www.python.org/downloads --> website to download the
latest Python 3 installer. Ensure that `pip` is installed and that Python 3 is added to `PATH`.

## Build system
The Ninja build system is preferred on Windows. Download the binary from the @link_ninja_download<!-- https://ninja-build.org/ -->
and place it in a suitable folder. Append the path to the folder to your `PATH` environment variable if
it is not there already.

## Building unit tests (host)
The unit test (host) build uses the standard GCC compiler, available on Windows through @link_mingw<!-- https://sourceforge.net/projects/mingw/files/-->.
Install the `mingw-base` and ensure that the 32-bit version is installed or that 32-bit libraries are
available. Furthermore, `libpthread` is needed for the multithreaded test.
This can be installed using `mingw-get.exe`. From the command line, simply call:

    $ mingw-get install libpthread

Ruby is used by the mocking framework @link_cmock<!-- https://github.com/ThrowTheSwitch/CMock -->.
Install it from the @link_ruby<!-- https://www.ruby-lang.org/ -->
website.


| Tool              | Recommended Minimum Version | Install Notes                                         | Download Link                                                                                |
|-------------------|-----------------------------|-------------------------------------------------------|----------------------------------------------------------------------------------------------|
| git for windows   | 2.13                        | make sure to select "Use git in windows command line" | https://git-scm.com/download/win                                                             |
| CMake             | 3.9.0                       | (add to Path)                                         | https://cmake.org/download/                                                                  |
| Nrfjprog          | 9.5.0                       | (add to Path)                                         | https://www.nordicsemi.com/eng/nordic/Products/nRF52840/nRF5x-Command-Line-Tools-Win32/58850 |
| J-Link            | 6.16a                       | (add to Path)                                         | included in nrfjprog toolset                                                                 |
| ninja             | 1.7.2                       | (add to Path)                                         | https://github.com/ninja-build/ninja/releases/download/v1.8.2/ninja-win.zip                  |
| python3           | 3.5.1.32                    | must be 32bit for pynrfjprog dll to work!             | https://www.python.org/ftp/python/3.5.1/python-3.5.1.exe                                     |
| python2           | 2.7                         |                                                       | https://www.python.org/ftp/python/2.7/python-2.7.msi                                         |
| arm-none-eabi-gcc | 6.3.1                       | (add to Path)                                         | https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads                         |


