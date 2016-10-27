# Installation on Windows

## Supported compilers

At this moment, compilation has been tested on Windows 7&10, and is supported
when using Visual Studio 2013. Patches for other versions are welcome.

## Installation

Using a combination of pre-compiled binaries and self-compiled dependencies in a
local workspace. To make things easier, use a MinGW shell for your editing work,
and only use the Windows `cmd` for configuring and building.  You might also
need to [disable the Windows firewall](http://windows.microsoft.com/en-us/windows/turn-windows-firewall-on-off#turn-windows-firewall-on-off=windows-7).

1. Install cmake, make sure to select the "Add CMake to system path for all users" option in the install dialog box.

    [CMake](https://cmake.org/download)

1. Make a directory to work in, e.g.:

        mkdir manifold-ws
        cd manifold-ws

1. In a Windows Command Prompt, load your compiler setup, e.g.:

        "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" amd64

1. Clone manifold:

        hg clone https://bitbucket.org/osrf/manifold
        cd manifold
        mkdir build

1. In the Windows Command Prompt, configure and build:

	    cd manifold\build
        ..\configure
        nmake install


    You should now have an installation of manifold in manifold-ws/manifold/build/install.

## Run tests

1. In the Windows Command Prompt, run the test suite:

    cd manifold\build
    ctest
