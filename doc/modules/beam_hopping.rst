Beam Hopping Module
====================

Overview
--------

The beam hopping module models time-varying beam activity. It supports scenarios where beams are dynamically switched on or off to balance load or follow scheduling constraints.

Key responsibilities
-------------------

- manage beam activity over time,
- support beam hopping schedules,
- determine whether a beam is currently active for a given time slot.

Primary implementation
----------------------

- ``model/leosim-beam-hopping-manager.h``
- ``model/leosim-beam-hopping-manager.cc``

Functional definition
---------------------

LeoSimBeamHoppingManager manages time-varying beam activity. It exposes which beams are available at a given instant and is used when beam resources are scheduled rather than continuously active.

Typical usage
-------------

The manager is typically used in scenarios with dynamic resource allocation, where a beam becomes active only during selected time slots. It feeds the beam manager with a current availability view that affects access and handover decisions.
