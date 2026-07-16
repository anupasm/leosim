SINR Module
===========

Overview
--------

The SINR module evaluates the signal quality of a candidate serving beam. It computes signal power, interference from co-channel beams, thermal noise, and the resulting SINR for a ground terminal.

Key responsibilities
-------------------

- compute received signal power,
- model intra-satellite and inter-satellite interference,
- estimate thermal noise,
- return SINR values used by beam ranking and handover decisions.

Primary implementation
----------------------

- ``model/leosim-sinr-engine.h``
- ``model/leosim-sinr-engine.cc``

Why it matters
--------------

The SINR engine is central to determining which beam is best for a UE at a given time.

Functional definition
---------------------

LeoSimSinrEngine evaluates the quality of a candidate link by combining received signal power, interference, and noise into a scalar SINR value. The result is used to rank beams and to decide whether a handover should be triggered.

Typical usage
-------------

The engine is invoked by the beam manager while comparing candidate beams for a UE. It is typically configured through the LeoSim channel and beam helpers so that beam selection uses the same propagation assumptions as the rest of the simulation.
