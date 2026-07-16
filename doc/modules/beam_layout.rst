Beam Layout Module
==================

Overview
--------

The beam layout module defines the geometry and placement of satellite beams. It represents the physical layout used by the beam manager when evaluating which beams cover a ground terminal.

Key responsibilities
-------------------

- define beam coverage structure,
- describe beam centers and directional properties,
- provide the layout used by beam selection and handover decisions.

Primary implementation
----------------------

- ``model/leosim-beam-layout-engine.h``
- ``model/leosim-beam-layout-engine.cc``

Functional definition
---------------------

LeoSimBeamLayoutEngine defines the geometric structure of the beam set. Its role is to tell the rest of the system where beams are centered and how coverage is arranged so that beam selection can be evaluated deterministically.

Typical usage
-------------

The layout engine is used by the beam manager and the multi-beam model when mapping a ground terminal to the set of beams that cover it. It is a supporting utility rather than a stand-alone simulation agent.
