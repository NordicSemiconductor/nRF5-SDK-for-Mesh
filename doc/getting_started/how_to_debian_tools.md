# CMake on Debian/Ubuntu

For Debian/Ubuntu, most tools are available from the system package manager `apt-get`.

## CMake

> **Important:** For Ubuntu versions older than `zesty`, a manual installation of CMake is required as the version
> available in the package manager is older than 3.6. Visit @link_cmake <!--CMake: https://cmake.org/-->
> to download the latest release and follow the installation instructions.

To install CMake, simply call:

    $ sudo apt-get install cmake cmake-curses-gui


Ensure that your CMake version is at least 3.6:

    $ cmake --version
    cmake version 3.9.2

    CMake suite maintained and supported by Kitware (kitware.com/cmake).


## Toolchain

For Debian/Ubuntu, the GNU ARM Embedded toolchain is supported. The version usually found in
the Debian package manager is quite old (4.9.3). Therefore, install the toolchain in the following
way:

    $ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
    $ sudo apt-get update
    $ sudo apt-get install gcc-arm-embedded

This will also install GDB (the GNU Debugger) for ARM.

### Python

The default Python version that comes with most Linux distributions is Python 2.7, but the
nRF5 SDK for Mesh requires Python 3.5. We highly recommend using `virtualenv` to manage Python versions.
It makes managing Python settings across different projects easy. Install it with:

    $ sudo apt-get install virtualenv

Next, make a directory to keep your virtual environments in and create a new environment for mesh
development:

    $ mkdir virtualenvs
    $ virtualenv -p python3 virtualenvs/mesh

The environment must be _activated_. This will set the `python` and `pip` commands to point to
the version within the given environment. All packages installed through `pip` will be local
to the active environment. This is how the environment is activated:

    $ source virtualenvs/mesh/bin/activate
    ...
    $ which python
    /home/<user-name>/virtualenvs/mesh/bin/python
    $ which pip
    /home/<user-name>/virtualenvs/mesh/bin/pip

Deactivate the environment with:

    $ deactivate

The environment will only be set for the active shell session. To make this virtual environment the
default when starting a new shell, add the following to your `~/.bashrc` file:

    source virtualenvs/mesh/bin/activate


## Build system

@link_make is the default build system on Debian/Ubuntu and usually comes with the distribution.
However, if @link_ninja is preferred, it can be installed with:

    $ sudo apt-get install ninja-build

## Additional tools for building unit tests (host) [Optional]

The unit test (host) build uses the standard GCC compiler, which should be available in the
Debian/Ubuntu distribution by default. To enable compilation for a 32-bit architecture on a 64-bit
system (`-m32`), `multilib` is required. Install it with:

    $ sudo apt-get install gcc-multilib

Ruby is used by the mocking framework @link_cmock. Install it with:

    $ sudo apt-get install ruby

If the GCC compiler is used, a code coverage report can be generated. This requires `lcov`. Install it with:

    $ sudo apt-get install lcov
