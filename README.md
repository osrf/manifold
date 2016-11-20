# Manifold

** Classes and functions for parsing road networks **

Manifold is a portable C++ library for parsing road network files and streams in
different formats (e.g.: RNDF, OpenDrive).

  [http://bitbucket.org/osrf/manifold](http://bitbucket.org/osrf/manifold)

## Continuous integration

Please refer to the [Bitbucket Pipelines](https://bitbucket.org/osrf/manifold/addon/pipelines/home#!/).

[![Documentation Status](https://readthedocs.org/projects/manifold/badge/?version=latest)](https://readthedocs.org/projects/manifold/?badge=latest)


## Documentation

Check [here](http://manifold.readthedocs.io/en/latest/).

## Dependencies

The following dependencies are required to compile manifold from source:

 - cmake
 - pkg-config
 - ruby-ronn
 - mercurial
 - libignition-math2-dev
 - C++ compiler with c++11 support (eg. GCC>=4.8).

1. Setup your computer to accept software from packages.osrfoundation.org.

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
    ```

1. Setup keys.

    ```
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

1. Install libignition-math2-dev.

    ```
    sudo apt-get update
    sudo apt-get install libignition-math2-dev
    ```

1. Install the build dependencies:

    ```
    sudo apt-get install build-essential cmake pkg-config ruby-ronn mercurial
    ```

## Installation

Standard installation can be performed in UNIX systems using the following
steps:

 - mkdir build/
 - cd build/
 - cmake .. -DCMAKE_INSTALL_PREFIX=/usr
 - sudo make install

## Uninstallation

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall