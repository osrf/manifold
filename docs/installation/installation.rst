============
Installation
============

Instructions to install Manifold on all the platforms supported: major Linux distributions, Mac OS X and Windows.

Ubuntu Linux
============

Setup your computer to accept software from *packages.osrfoundation.org*:

.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable
    `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

Setup keys:

.. code-block:: bash

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

Install Manifold:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install libmanifold0-dev

Mac OS X
========

Manifold and several of its dependencies can be compiled on OS X with
`Homebrew <http://brew.sh/>`_ using the
`osrf/simulation tap <https://github.com/osrf/homebrew-simulation>`_. Manifold is straightforward to install on Mac OS X 10.9 (Mavericks) or higher.
Installation on older versions requires changing the default standard library
and rebuilding dependencies due to the use of c++11. For purposes of this
documentation, I will assume OS X 10.9 or greater is in use. Here are the
instructions:

Install homebrew, which should also prompt you to install the XCode command-line tools:

.. code-block:: bash

    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

Run the following commands:

.. code-block:: bash

    brew tap osrf/simulation
    brew install manifold0

Windows
=======

At this moment, compilation has been tested on Windows 7 and 8.1 and is
supported when using
`Visual Studio 2013 <https://www.visualstudio.com/downloads/>`_. Patches for
other versions are welcome.

This installation procedure uses pre-compiled binaries in a local workspace. To make things easier, use a MinGW shell for your editing work (such as the
`Git Bash Shell <https://msysgit.github.io/>`_ with
`Mercurial <http://tortoisehg.bitbucket.org/download/index.html>`_), and only
use the Windows cmd for configuring and building. You might also need to
`disable the Windows firewall <http://windows.microsoft.com/en-us/windows/turn-windows-firewall-on-off#turn-windows-firewall-on-off=windows-7>`_.

Make a directory to work in, e.g.:

.. code-block:: bash

    mkdir manifold-ws
    cd manifold-ws

Clone Manifold:

.. code-block:: bash

        hg clone https://bitbucket.org/osrf/manifold
        cd manifold

In a Windows Command Prompt, load your compiler setup, e.g.:

.. code-block:: bash

        "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" amd64

Configure and build:


.. code-block:: bash

        mkdir build
        cd build
        ..\configure
        nmake
        nmake install

You should now have an installation of ign-Manifold in ``manifold-ws/manifold/build/install``.


Install from sources (Ubuntu Linux)
=======

For compiling the latest version of Manifold you will need an Ubuntu
distribution equal to 14.04 (Trusty) or newer.

Make sure you have removed the Ubuntu pre-compiled binaries before installing
from source:

.. code-block:: bash

        sudo apt-get remove libmanifold0-dev

Setup your computer to accept software from *packages.osrfoundation.org*:

.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable
    `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

Setup keys:

.. code-block:: bash

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

Install prerequisites. A clean Ubuntu system will need:

.. code-block:: bash

        sudo apt-get install cmake pkg-config python ruby-ronn libignition-math2-dev

Clone the repository into a directory and go into it:

.. code-block:: bash

        hg clone https://bitbucket.org/osrf/manifold /tmp/manifold
        cd /tmp/manifold

Create a build directory and go there:

.. code-block:: bash

        mkdir build
        cd build

Configure Manifold (choose either method a or b below):

  A. Release mode: This will generate optimized code, but will not have debug   symbols. Use this mode if you don't need to use GDB.

  .. code-block:: bash

          cmake ../

  Note: You can use a custom install path to make it easier to switch between source   and debian installs:

  .. code-block:: bash

          cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

  B. Debug mode: This will generate code with debug symbols. Manifold
  will run slower, but you'll be able to use GDB.

  .. code-block:: bash

          cmake -DCMAKE_BUILD_TYPE=Debug ../

The output from ``cmake ../`` may generate a number of errors and warnings about
missing packages. You must install the missing packages that have errors and
re-run ``cmake ../``. Make sure all the build errors are resolved before
continuing (they should be there from the earlier step in which you installed
prerequisites).

Make note of your install path, which is output from cmake and should look something like:

.. code-block:: bash

        -- Install path: /home/$USER/local

Build Manifold:

.. code-block:: bash

        make -j4

Install Manifold:

.. code-block:: bash

        sudo make install

If you decide to install Manifold in a local directory you'll need to modify your
``LD_LIBRARY_PATH``:

.. code-block:: bash

        echo "export LD_LIBRARY_PATH=<install_path>/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

Uninstalling Source-based Install
---------------------------------

If you need to uninstall Manifold or switch back to a debian-based
install when you currently have installed the library from source, navigate to
your source code directory's build folders and run ``make uninstall``:

.. code-block:: bash

        cd /tmp/manifold/build
        sudo make uninstall
