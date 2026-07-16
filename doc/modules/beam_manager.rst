Beam Manager Module
===================

Overview
--------

The beam manager is the access-control and handover core of LeoSim. It tracks the current serving beam for each user equipment node and decides when to switch to another beam or satellite.

Key responsibilities
-------------------

- maintain serving beam associations,
- evaluate beam quality and coverage,
- trigger handover logic for BHO and CHO modes,
- rank candidate beams using multi-criteria methods,
- emit beam and handover traces for analysis.

Primary implementation
----------------------

- ``model/leosim-beam-manager.h``
- ``model/leosim-beam-manager.cc``

Relevant concepts
-----------------

The beam manager interacts with the channel model, SINR engine, weather model, and routing calculator. It is one of the most central modules in the LeoSim architecture.

Functional definition
---------------------

LeoSimBeamManager is the control plane for beam association and handover. It maintains the current serving beam for each node, evaluates the available beams, and decides when a switching event should occur.

Typical usage
-------------

The manager is installed with LeoSimBeamManagerHelper and then used by simulation scenarios that require adaptive beam selection, beam handover, or multi-beam access control. It is normally coupled with the mobility and channel modules so that serving-beam decisions follow node movement and link quality.
