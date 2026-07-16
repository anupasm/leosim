Multi-Beam Module
=================

Overview
--------

The multi-beam module captures the coordination of several simultaneously active beams over a satellite coverage area. It is responsible for representing the set of beam resources available to users and to the network control logic.

Key responsibilities
-------------------

- organize beam sets per satellite,
- expose candidate beam resources to the beam manager,
- support multi-beam access and coordination.

Primary implementation
----------------------

- ``model/leosim-multi-beam.h``
- ``model/leosim-multi-beam.cc``

Functional definition
---------------------

LeoSimMultiBeamModel represents the set of beams available to a satellite or a network segment. It organizes those beams into a coherent resource pool that the beam manager can evaluate for access and handover.

Typical usage
-------------

The multi-beam model is used in scenarios where a satellite serves multiple cells or beam groups at the same time. It is normally instantiated alongside beam management and channel setup so that multi-beam selection can occur in a consistent way.
