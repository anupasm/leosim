Device Installation Module
==========================

Overview
--------

The device installation module is responsible for attaching LeoSim components to ns-3 nodes and devices. It provides the helper logic that installs satellites, user terminals, and supporting network stacks onto the simulation topology.

Key responsibilities
-------------------

- install LeoSim devices on ns-3 nodes,
- configure the relevant helper objects,
- wire simulation components together during setup.

Primary implementation
----------------------

- ``helper/leosim-helper.h``
- ``helper/leosim-helper.cc``

Functional definition
---------------------

LeoSimDeviceInstaller wires LeoSim components into ns-3 nodes and devices. Its job is to install the correct mobility, channel, beam, and routing objects so that the scenario can run as a complete network model.

Typical usage
-------------

This module is used during scenario setup, usually as one of the first steps in a script that builds a LeoSim topology. It is the bridge between the high-level helper API and the underlying model objects.
