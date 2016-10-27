# Manifold

** Classes and functions for parsing road networks **

Manifold is a portable C++ library for parsing road network files and streams in
different formats (e.g.: RNDF, OpenDrive).

  [http://bitbucket.org/osrf/manifold](http://bitbucket.org/osrf/manifold)

## Continuous integration

Please refer to the [Bitbucket Pipelines](https://bitbucket.org/osrf/manifold/addon/pipelines/home#!/).

[![Documentation Status](https://readthedocs.org/projects/manifold/badge/?version=latest)](https://readthedocs.org/projects/manifold/?badge=latest)


## Dependencies

The following dependencies are required to compile ignition-transport from
source:

 - cmake
 - pkg-config
 - ruby-ronn
 - mercurial
 - C++ compiler with c++11 support (eg. GCC>=4.8).

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