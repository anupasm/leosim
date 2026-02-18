.. include:: replace.txt
.. highlight:: cpp

LeoSim Module Documentation
----------------------------

.. heading hierarchy:
   ------------- Chapter
   ************* Section (#.#)
   ============= Subsection (#.#.#)
   ############# Paragraph (no number)

This is the documentation for the LeoSim module for ns-3.

Model Description
*****************

The LeoSim module provides functionality for simulating Low Earth Orbit (LEO) satellite networks.

Design
======

The module is designed to be extensible and modular, allowing for easy integration
with other ns-3 modules.

References
==========

Add references here.

Usage
*****

Building the module
===================

To build the LeoSim module::

    $ ./ns3 configure --enable-examples --enable-tests
    $ ./ns3 build

Examples
========

The module includes example programs demonstrating basic functionality::

    $ ./ns3 run leosim-example

Helpers
=======

The LeoSimHelper class provides convenience methods for setting up simulations.

Tests
*****

The module includes unit tests that can be run with::

    $ ./test.py -s leosim
